//! Display-only sectioning. Clip triangles and topology and cap with an
//! even-odd scan of the cut segments, preserving holes and concave regions.
//!
//! A section is one or more planes, each `(point, normal)` keeping the side
//! its normal points to. Material is removed only where EVERY plane removes
//! it: one plane is a plain section, two a half section (a quarter cut
//! away), three a corner cut away. The removed region is the intersection of
//! the planes' removed half-spaces, so it is convex: a segment loses one
//! interval, a triangle one convex piece.
use crate::scene::{DisplayMesh, EdgeDisplay, FaceDisplay, FaceKind, SolidDisplay};

/// The cut faces' colour (sRGB): a warm red that no model face defaults to,
/// so the cut reads as the cut, as the hatching does on a sheet.
pub const SECTION_CAP_RGB: [f32; 3] = [0.86, 0.42, 0.36];

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

/// Where a whole solid lies against the cutting plane, from its box.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Side {
    /// Entirely on the kept side (or on the plane): drawn as it is.
    Kept,
    /// Entirely on the removed side: nothing of it is drawn.
    Removed,
    /// The plane crosses it.
    Cut,
}

/// One cutting plane: `(point, normal)`, kept where `(p − point)·normal ≥ 0`.
pub(super) type Plane = ([f64; 3], [f64; 3]);

fn signed(plane: &Plane, p: [f64; 3]) -> f64 {
    dot(sub(p, plane.0), plane.1)
}

/// Which [`Side`] `display` is on. A box the plane misses decides it without
/// touching a triangle: on a large assembly most solids are not cut at all.
pub(super) fn side_of(display: &SolidDisplay, point: [f64; 3], normal: [f64; 3]) -> Side {
    side_of_all(display, &[(point, normal)])
}

/// [`side_of`] for a section of several planes: kept whole when one plane
/// keeps the whole box, removed when every plane removes it.
pub(super) fn side_of_all(display: &SolidDisplay, planes: &[Plane]) -> Side {
    let bbox = &display.bbox;
    if bbox.is_empty() || planes.is_empty() {
        return Side::Kept;
    }
    let mut removed_by_all = true;
    for plane in planes {
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for corner in 0..8 {
            let p = [
                if corner & 1 == 0 { bbox.min[0] } else { bbox.max[0] },
                if corner & 2 == 0 { bbox.min[1] } else { bbox.max[1] },
                if corner & 4 == 0 { bbox.min[2] } else { bbox.max[2] },
            ];
            let d = signed(plane, p);
            low = low.min(d);
            high = high.max(d);
        }
        if low >= 0. {
            return Side::Kept;
        }
        removed_by_all &= high < 0.;
    }
    if removed_by_all {
        Side::Removed
    } else {
        Side::Cut
    }
}

/// `polygon` (positions with normals) where `f ≥ 0`, Sutherland–Hodgman.
fn clip_polygon(polygon: &[([f64; 3], [f64; 3])], f: &dyn Fn([f64; 3]) -> f64) -> Vec<([f64; 3], [f64; 3])> {
    let mut out = Vec::new();
    for i in 0..polygon.len() {
        let (a, na) = polygon[i];
        let (b, nb) = polygon[(i + 1) % polygon.len()];
        let (da, db) = (f(a), f(b));
        if da >= 0. {
            out.push((a, na));
        }
        if (da >= 0.) != (db >= 0.) {
            let t = da / (da - db);
            out.push((mix(a, b, t), mix(na, nb, t)));
        }
    }
    out
}

/// The parameter interval of `a → b` that every plane removes (`d < 0` for
/// all), or `None`. Convex, so one interval.
fn removed_interval(planes: &[Plane], a: [f64; 3], b: [f64; 3]) -> Option<(f64, f64)> {
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    for plane in planes {
        let (da, db) = (signed(plane, a), signed(plane, b));
        // d(t) = da + (db − da) t < 0.
        if (db - da).abs() < 1e-300 {
            if da >= 0. {
                return None;
            }
            continue;
        }
        let t = da / (da - db);
        if db > da {
            hi = hi.min(t);
        } else {
            lo = lo.max(t);
        }
        if lo >= hi {
            return None;
        }
    }
    Some((lo, hi))
}

pub(super) fn clip_display(display: &mut SolidDisplay, point: [f64; 3], normal: [f64; 3]) {
    clip_display_all(display, &[(point, normal)]);
}

/// Clip `display` to a section of several planes (see the module docs) and
/// cap each plane where the material behind it is removed.
pub(super) fn clip_display_all(display: &mut SolidDisplay, planes: &[Plane]) {
    if planes.is_empty() {
        return;
    }
    let kept = |p: [f64; 3]| planes.iter().any(|plane| signed(plane, p) >= 0.);
    let mut mesh = DisplayMesh::default();
    // Per plane, the solid's section by it: the caps' boundary.
    let mut cuts: Vec<Vec<[[f64; 3]; 2]>> = vec![Vec::new(); planes.len()];
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
        for (k, plane) in planes.iter().enumerate() {
            let mut cut = Vec::new();
            for i in 0..3 {
                let (a, b) = (vertices[i].0, vertices[(i + 1) % 3].0);
                let (da, db) = (signed(plane, a), signed(plane, b));
                if (da >= 0.) != (db >= 0.) {
                    cut.push(mix(a, b, da / (da - db)));
                }
            }
            if cut.len() == 2 {
                cuts[k].push([cut[0], cut[1]]);
            }
        }
        // The kept part, as disjoint convex pieces: kept by plane k and
        // removed by every plane before it.
        for k in 0..planes.len() {
            let mut polygon = clip_polygon(&vertices, &|p| signed(&planes[k], p));
            for earlier in &planes[..k] {
                if polygon.len() < 3 {
                    break;
                }
                polygon = clip_polygon(&polygon, &|p| -signed(earlier, p));
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
    // Each plane's cut face is its section of the solid where every OTHER
    // plane removes the material too (elsewhere the plane runs through kept
    // material and shows nothing).
    let mut cut_edges: Vec<[[f64; 3]; 2]> = Vec::new();
    for (k, &(point, normal)) in planes.iter().enumerate() {
        let others: Vec<Plane> = planes.iter().enumerate().filter(|(j, _)| *j != k).map(|(_, plane)| *plane).collect();
        let cuts = &cuts[k];
        let mut cap = DisplayMesh::default();
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
                triangle(&mut cap, [a, c, b], [n; 3], cap_id);
                triangle(&mut cap, [a, d, c], [n; 3], cap_id);
            }
        }
        for ids in cap.indices.chunks_exact(3) {
            let mut polygon: Vec<([f64; 3], [f64; 3])> =
                ids.iter().map(|&i| (cap.positions[i as usize].map(|v| v as f64), normal.map(|v| -v))).collect();
            for other in &others {
                if polygon.len() < 3 {
                    break;
                }
                polygon = clip_polygon(&polygon, &|p| -signed(other, p));
            }
            for i in 1..polygon.len().saturating_sub(1) {
                triangle(&mut mesh, [polygon[0].0, polygon[i].0, polygon[i + 1].0], [polygon[0].1; 3], cap_id);
            }
        }
        for segment in cuts {
            match removed_interval(&others, segment[0], segment[1]) {
                _ if others.is_empty() => cut_edges.push(*segment),
                Some((lo, hi)) => cut_edges.push([mix(segment[0], segment[1], lo), mix(segment[0], segment[1], hi)]),
                None => {}
            }
        }
    }
    let cap_count = mesh.face_ids.len() as u32 - cap_start;
    if cap_count > 0 {
        display.faces.push(FaceDisplay {
            name: String::new(),
            topo_id: 0,
            tri_start: cap_start,
            tri_count: cap_count,
            kind: FaceKind::SectionCap,
            color_override: Some(SECTION_CAP_RGB),
        });
    }
    let mut edges = Vec::new();
    for edge in &display.edges {
        // Each piece loses at most one interval (the removed region is convex).
        let mut run: Vec<[f32; 3]> = Vec::new();
        let flush = |run: &mut Vec<[f32; 3]>, edges: &mut Vec<EdgeDisplay>| {
            if run.len() >= 2 {
                let mut clipped = edge.clone();
                clipped.polyline = std::mem::take(run);
                edges.push(clipped);
            }
            run.clear();
        };
        for pair in edge.polyline.windows(2) {
            let a = pair[0].map(|v| v as f64);
            let b = pair[1].map(|v| v as f64);
            match removed_interval(planes, a, b) {
                None => {
                    if run.is_empty() {
                        run.push(pair[0]);
                    }
                    run.push(pair[1]);
                }
                Some((lo, hi)) => {
                    if lo > 0. {
                        if run.is_empty() {
                            run.push(pair[0]);
                        }
                        run.push(mix(a, b, lo).map(|v| v as f32));
                    }
                    flush(&mut run, &mut edges);
                    if hi < 1. {
                        run.push(mix(a, b, hi).map(|v| v as f32));
                        run.push(pair[1]);
                    }
                }
            }
        }
        flush(&mut run, &mut edges);
    }
    edges.extend(cut_edges.iter().map(|s| EdgeDisplay {
        name: String::new(),
        topo_id: 0,
        polyline: s.map(|p| p.map(|v| v as f32)).to_vec(),
        aux: true,
        centerline: false,
    }));
    display.edges = edges;
    display.vertices.retain(|v| kept(v.position));
    display.mesh = mesh;
    let mut bbox = crate::camera::Aabb::empty();
    for p in &display.mesh.positions {
        bbox.expand(p.map(|v| v as f64));
    }
    display.bbox = bbox;
    display.revision = crate::scene::next_revision();
    display.shape_revision = 0;
    display.pose = None;
}

