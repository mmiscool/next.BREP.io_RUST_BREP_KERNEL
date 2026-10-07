//! Display-only sectioning. Clip triangles and topology and cap with an
//! even-odd scan of the cut segments, preserving holes and concave regions.
use crate::scene::{DisplayMesh, EdgeDisplay, FaceDisplay, FaceKind, SolidDisplay};

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn mix(a: [f64; 3], b: [f64; 3], t: f64) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t)
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: [f64; 3]) -> [f64; 3] {
    let len = dot(a, a).sqrt().max(1e-20);
    a.map(|v| v / len)
}
fn triangle(mesh: &mut DisplayMesh, points: [[f64; 3]; 3], normals: [[f64; 3]; 3], face: u32) {
    if dot(
        cross(sub(points[1], points[0]), sub(points[2], points[0])),
        cross(sub(points[1], points[0]), sub(points[2], points[0])),
    ) < 1e-24
    {
        return;
    }
    let first = mesh.positions.len() as u32;
    mesh.positions.extend(points.map(|p| p.map(|v| v as f32)));
    mesh.normals
        .extend(normals.map(|n| unit(n).map(|v| v as f32)));
    mesh.indices.extend([first, first + 1, first + 2]);
    mesh.face_ids.push(face);
}

pub(super) fn clip_display(display: &mut SolidDisplay, point: [f64; 3], normal: [f64; 3]) {
    let distance = |p| dot(sub(p, point), normal);
    let mut mesh = DisplayMesh::default();
    let mut cuts = Vec::<[[f64; 3]; 2]>::new();
    for (index, ids) in display.mesh.indices.chunks_exact(3).enumerate() {
        let vertices: Vec<([f64; 3], [f64; 3])> = ids
            .iter()
            .map(|&i| {
                (
                    display.mesh.positions[i as usize].map(|v| v as f64),
                    display
                        .mesh
                        .normals
                        .get(i as usize)
                        .copied()
                        .unwrap_or([0., 0., 1.])
                        .map(|v| v as f64),
                )
            })
            .collect();
        let mut polygon = Vec::new();
        let mut cut = Vec::new();
        for i in 0..3 {
            let (a, na) = vertices[i];
            let (b, nb) = vertices[(i + 1) % 3];
            let da = distance(a);
            let db = distance(b);
            if da >= 0. {
                polygon.push((a, na));
            }
            if (da >= 0.) != (db >= 0.) {
                let t = da / (da - db);
                let p = mix(a, b, t);
                polygon.push((p, mix(na, nb, t)));
                cut.push(p);
            }
        }
        if cut.len() == 2 {
            cuts.push([cut[0], cut[1]]);
        }
        for i in 1..polygon.len().saturating_sub(1) {
            triangle(
                &mut mesh,
                [polygon[0].0, polygon[i].0, polygon[i + 1].0],
                [polygon[0].1, polygon[i].1, polygon[i + 1].1],
                display.mesh.face_ids.get(index).copied().unwrap_or(0),
            );
        }
    }
    // Keep face ranges coherent after clipping; mesh triangles remain in source order.
    for face in &mut display.faces {
        face.tri_start = 0;
        face.tri_count = 0;
    }
    for (triangle, &face_id) in mesh.face_ids.iter().enumerate() {
        if let Some(face) = display.faces.get_mut(face_id as usize) {
            if face.tri_count == 0 {
                face.tri_start = triangle as u32;
            }
            face.tri_count += 1;
        }
    }
    let cap_id = display.faces.len() as u32;
    let cap_start = mesh.face_ids.len() as u32;
    let right = unit(cross(
        normal,
        if normal[2].abs() < 0.9 {
            [0., 0., 1.]
        } else {
            [0., 1., 0.]
        },
    ));
    let up = cross(normal, right);
    let segments: Vec<[[f64; 2]; 2]> = cuts
        .iter()
        .map(|segment| segment.map(|p| [dot(sub(p, point), right), dot(sub(p, point), up)]))
        .collect();
    let mut levels: Vec<f64> = segments.iter().flat_map(|s| [s[0][1], s[1][1]]).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup_by(|a, b| (*a - *b).abs() < 1e-8);
    let lift = |p: [f64; 2]| std::array::from_fn(|i| point[i] + right[i] * p[0] + up[i] * p[1]);
    for band in levels.windows(2) {
        let mid = (band[0] + band[1]) * 0.5;
        let x_at = |s: &[[f64; 2]; 2], y: f64| {
            s[0][0] + (s[1][0] - s[0][0]) * (y - s[0][1]) / (s[1][1] - s[0][1])
        };
        let mut active: Vec<_> = segments
            .iter()
            .filter(|s| mid > s[0][1].min(s[1][1]) && mid < s[0][1].max(s[1][1]))
            .collect();
        active.sort_by(|a, b| x_at(a, mid).total_cmp(&x_at(b, mid)));
        for pair in active.chunks_exact(2) {
            let a = lift([x_at(pair[0], band[0]), band[0]]);
            let b = lift([x_at(pair[1], band[0]), band[0]]);
            let c = lift([x_at(pair[1], band[1]), band[1]]);
            let d = lift([x_at(pair[0], band[1]), band[1]]);
            let n = normal.map(|v| -v);
            triangle(&mut mesh, [a, c, b], [n; 3], cap_id);
            triangle(&mut mesh, [a, d, c], [n; 3], cap_id);
        }
    }
    let cap_count = mesh.face_ids.len() as u32 - cap_start;
    if cap_count > 0 {
        display.faces.push(FaceDisplay {
            name: String::new(),
            topo_id: 0,
            tri_start: cap_start,
            tri_count: cap_count,
            kind: FaceKind::Unknown,
            color_override: None,
        });
    }
    let mut edges = Vec::new();
    for edge in &display.edges {
        let mut run = Vec::new();
        for pair in edge.polyline.windows(2) {
            let a = pair[0].map(|v| v as f64);
            let b = pair[1].map(|v| v as f64);
            let da = distance(a);
            let db = distance(b);
            if da >= 0. && run.is_empty() {
                run.push(pair[0]);
            }
            if (da >= 0.) != (db >= 0.) {
                let p = mix(a, b, da / (da - db)).map(|v| v as f32);
                run.push(p);
                if da >= 0. {
                    let mut clipped = edge.clone();
                    clipped.polyline = std::mem::take(&mut run);
                    if clipped.polyline.len() >= 2 {
                        edges.push(clipped);
                    }
                }
            }
            if db >= 0. {
                run.push(pair[1]);
            }
        }
        if run.len() >= 2 {
            let mut clipped = edge.clone();
            clipped.polyline = run;
            edges.push(clipped);
        }
    }
    edges.extend(cuts.iter().map(|s| EdgeDisplay {
        name: String::new(),
        topo_id: 0,
        polyline: s.map(|p| p.map(|v| v as f32)).to_vec(),
        aux: true,
        centerline: false,
    }));
    display.edges = edges;
    display.vertices.retain(|v| distance(v.position) >= 0.);
    display.mesh = mesh;
    let mut bbox = crate::camera::Aabb::empty();
    for p in &display.mesh.positions {
        bbox.expand(p.map(|v| v as f64));
    }
    display.bbox = bbox;
    display.revision = crate::scene::next_revision();
}

