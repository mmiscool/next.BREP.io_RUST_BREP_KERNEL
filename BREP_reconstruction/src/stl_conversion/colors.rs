//! Source-triangle COLOURS onto the reconstructed bodies and faces.
//!
//! A mesh file can colour its triangles (a 3MF's base materials; see
//! `brep_kernel::ThreeMfReadResult::triangle_colors`). Reconstruction replaces
//! the triangles with exact faces, so the colour has to be carried across by
//! GEOMETRY: each source triangle's centroid is matched to the nearest triangle
//! of the kernel's own tessellation of the finished solids, which knows which
//! face (and which solid) it belongs to. The matches are then counted — one
//! vote per source triangle, `None` ("no colour") a candidate like any other —
//! and the plurality decides: the body takes the colour most of its triangles
//! carry, and a face whose own plurality differs from its body's is styled on
//! its own. The result goes out as the STEP writer's presentation styles
//! (`brep_kernel::StepColors`), the chain every importer of this crate's STEP
//! already reads back into `color` metadata.
//!
//! The nearest-triangle search is a uniform grid over the tessellation, so it
//! is linear in the triangle counts rather than their product. It is built
//! only when at least one source triangle has a colour; a plain import pays
//! nothing. A match counts only within [`MATCH_FRACTION`] of the source
//! extent: a source triangle farther than that from every finished face — a
//! body the lanes refused, left out of the import — is counted as unmatched
//! and votes nowhere, rather than colouring whatever neighbour is nearest. The
//! same bound stops the ring search, so such a triangle costs a few cells, not
//! the whole grid.
//!
//! The style writer keys bodies and faces by NAME, and the map is one map for
//! the whole file, so every face is (re)named `<body>:Face_<index>` — unique
//! across bodies, which `Face_<index>` alone is not — and bodies are named
//! `<part> 1`, `<part> 2`, … when there are several. None of these names
//! reaches the file: the STEP writer puts no name in `ADVANCED_FACE`, and the
//! Import 3D Model feature names bodies and faces positionally, not from the
//! file, so what the import shows is unchanged.

use std::collections::HashMap;

use super::{Mesh, StlConversionError};
use brep_kernel::{tessellate_brep_watertight, BrepSolid, StepColors};

/// What the pass decided — the names the STEP writer keys the styles on, the
/// styles, and the counts the report states.
pub(super) struct ColorAssignment {
    /// One name per solid, in order: the `MANIFOLD_SOLID_BREP` name.
    pub body_names: Vec<String>,
    pub colors: StepColors,
    /// Bodies that took a colour.
    pub bodies: usize,
    /// Faces styled in a colour other than their body's.
    pub faces: usize,
    /// Source triangles that matched no tessellated triangle.
    pub unmatched: usize,
}

/// Rings of grid cells searched outward before a source triangle is given up
/// on, whatever the cell size.
const MAX_RINGS: i32 = 16;
/// How far from a finished face a source triangle may sit and still vote, as
/// a fraction of the source mesh's extent. Both meshes describe the same
/// surface to within the tessellation chord (a thousandth of the extent), so
/// ten times that is generous for a true match and still a tenth of the gap
/// between two parts on a build plate.
const MATCH_FRACTION: f64 = 0.01;

/// Decide the colours of `solids` from the colours of `mesh`'s triangles.
/// `triangle_colors` is parallel to `mesh.triangles`. Faces with no name are
/// named in place, so the caller must export the SAME solids afterwards.
pub(super) fn assign(
    mesh: &Mesh,
    triangle_colors: &[Option<[u8; 3]>],
    solids: &mut [BrepSolid],
    part_name: &str,
) -> Result<ColorAssignment, StlConversionError> {
    if triangle_colors.len() != mesh.triangles.len() {
        return Err(StlConversionError(format!(
            "triangle_colors has {} entries for {} source triangles",
            triangle_colors.len(),
            mesh.triangles.len()
        )));
    }
    let body_names: Vec<String> = if solids.len() == 1 {
        vec![part_name.to_string()]
    } else {
        (1..=solids.len()).map(|index| format!("{part_name} {index}")).collect()
    };
    // Name every face uniquely across the file — the key the style writer
    // needs; the name itself goes no further (see the module doc).
    let mut face_names: Vec<Vec<String>> = Vec::with_capacity(solids.len());
    for (solid, body_name) in solids.iter_mut().zip(&body_names) {
        let mut names = Vec::new();
        for face in solid.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
            let name = format!("{body_name}:Face_{}", names.len());
            face.name = Some(name.clone());
            names.push(name);
        }
        face_names.push(names);
    }

    // The search structure: every tessellated triangle of every solid, tagged
    // with its solid and face.
    let (low, high) = bounds(mesh);
    let diagonal = distance(low, high).max(1e-9);
    let chord = (diagonal * 1e-3).max(1e-6);
    let mut grid = Grid::new(low, high);
    for (solid_index, solid) in solids.iter().enumerate() {
        // A solid the tessellator cannot mesh simply gets no colour; the
        // import itself is not at stake here.
        let Ok(tessellated) = tessellate_brep_watertight(solid, chord) else {
            continue;
        };
        for (triangle, corners) in tessellated.indices.chunks_exact(3).enumerate() {
            let face = tessellated.face_ids.get(triangle).copied().unwrap_or(0);
            let point = |corner: u32| {
                let at = corner as usize * 3;
                [tessellated.positions[at], tessellated.positions[at + 1], tessellated.positions[at + 2]]
            };
            grid.insert([point(corners[0]), point(corners[1]), point(corners[2])], solid_index, face);
        }
    }
    grid.finish();

    // One vote per source triangle, on its body and on its face.
    let mut body_votes: Vec<HashMap<Option<[u8; 3]>, usize>> = vec![HashMap::new(); solids.len()];
    let mut face_votes: HashMap<(usize, u32), HashMap<Option<[u8; 3]>, usize>> = HashMap::new();
    let mut unmatched = 0usize;
    for (corners, color) in mesh.triangles.iter().zip(triangle_colors) {
        let a = mesh.vertices[corners[0] as usize];
        let b = mesh.vertices[corners[1] as usize];
        let c = mesh.vertices[corners[2] as usize];
        let centroid = [
            (a.x + b.x + c.x) / 3.0,
            (a.y + b.y + c.y) / 3.0,
            (a.z + b.z + c.z) / 3.0,
        ];
        match grid.nearest(centroid) {
            Some((solid, face)) => {
                *body_votes[solid].entry(*color).or_insert(0) += 1;
                *face_votes.entry((solid, face)).or_default().entry(*color).or_insert(0) += 1;
            }
            None => unmatched += 1,
        }
    }

    let mut colors = StepColors::new();
    let mut bodies = 0usize;
    let mut faces = 0usize;
    for (solid_index, votes) in body_votes.iter().enumerate() {
        let body = plurality(votes);
        if let Some(rgb) = body {
            colors.set(&body_names[solid_index], rgb);
            bodies += 1;
        }
        for (face_index, name) in face_names[solid_index].iter().enumerate() {
            let Some(votes) = face_votes.get(&(solid_index, face_index as u32)) else {
                continue;
            };
            let face = plurality(votes);
            if let Some(rgb) = face {
                if face != body {
                    colors.set(name, rgb);
                    faces += 1;
                }
            }
        }
    }
    Ok(ColorAssignment { body_names, colors, bodies, faces, unmatched })
}

/// The candidate with the most votes; ties go to the colour (then the darker
/// of two colours, by the triple's order) so the answer is deterministic.
fn plurality(votes: &HashMap<Option<[u8; 3]>, usize>) -> Option<[u8; 3]> {
    let mut best: Option<(usize, Option<[u8; 3]>)> = None;
    for (candidate, count) in votes {
        let better = match best {
            None => true,
            Some((best_count, best_candidate)) => {
                *count > best_count
                    || (*count == best_count
                        && match (candidate, best_candidate) {
                            (Some(_), None) => true,
                            (None, _) => false,
                            (Some(a), Some(b)) => *a < b,
                        })
            }
        };
        if better {
            best = Some((*count, *candidate));
        }
    }
    best.and_then(|(_, candidate)| candidate)
}

fn bounds(mesh: &Mesh) -> ([f64; 3], [f64; 3]) {
    let mut low = [f64::INFINITY; 3];
    let mut high = [f64::NEG_INFINITY; 3];
    for vertex in &mesh.vertices {
        for (axis, value) in [vertex.x, vertex.y, vertex.z].into_iter().enumerate() {
            low[axis] = low[axis].min(value);
            high[axis] = high[axis].max(value);
        }
    }
    if low[0] > high[0] {
        return ([0.0; 3], [0.0; 3]);
    }
    (low, high)
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    dot(sub(a, b), sub(a, b)).sqrt()
}

/// Squared distance from `p` to triangle `abc` (Ericson, *Real-Time Collision
/// Detection* §5.1.5: the closest point by Voronoi region).
fn point_triangle_distance_squared(p: [f64; 3], [a, b, c]: [[f64; 3]; 3]) -> f64 {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ab, ap);
    let d2 = dot(ac, ap);
    let closest = if d1 <= 0.0 && d2 <= 0.0 {
        a
    } else {
        let bp = sub(p, b);
        let d3 = dot(ab, bp);
        let d4 = dot(ac, bp);
        if d3 >= 0.0 && d4 <= d3 {
            b
        } else {
            let vc = d1 * d4 - d3 * d2;
            if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
                let v = if (d1 - d3).abs() > 0.0 { d1 / (d1 - d3) } else { 0.0 };
                [a[0] + v * ab[0], a[1] + v * ab[1], a[2] + v * ab[2]]
            } else {
                let cp = sub(p, c);
                let d5 = dot(ab, cp);
                let d6 = dot(ac, cp);
                if d6 >= 0.0 && d5 <= d6 {
                    c
                } else {
                    let vb = d5 * d2 - d1 * d6;
                    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
                        let w = if (d2 - d6).abs() > 0.0 { d2 / (d2 - d6) } else { 0.0 };
                        [a[0] + w * ac[0], a[1] + w * ac[1], a[2] + w * ac[2]]
                    } else {
                        let va = d3 * d6 - d5 * d4;
                        if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
                            let denominator = (d4 - d3) + (d5 - d6);
                            let w = if denominator > 0.0 { (d4 - d3) / denominator } else { 0.0 };
                            [b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]), b[2] + w * (c[2] - b[2])]
                        } else {
                            let denominator = va + vb + vc;
                            if denominator.abs() > 0.0 {
                                let v = vb / denominator;
                                let w = vc / denominator;
                                [a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w]
                            } else {
                                a
                            }
                        }
                    }
                }
            }
        }
    };
    let d = sub(p, closest);
    dot(d, d)
}

/// A uniform grid of tessellated triangles, each tagged `(solid, face)`.
struct Grid {
    origin: [f64; 3],
    cell: f64,
    triangles: Vec<([[f64; 3]; 3], usize, u32)>,
    /// Filled by `finish`, once the triangle count is known.
    cells: HashMap<(i32, i32, i32), Vec<u32>>,
    extent: f64,
    /// The farthest a query may be from its match (`MATCH_FRACTION` of the
    /// extent), squared.
    reach_squared: f64,
}

impl Grid {
    fn new(low: [f64; 3], high: [f64; 3]) -> Self {
        let extent = distance(low, high);
        let reach = (extent * MATCH_FRACTION).max(1e-9);
        Self {
            origin: low,
            cell: 1.0,
            triangles: Vec::new(),
            cells: HashMap::new(),
            extent,
            reach_squared: reach * reach,
        }
    }

    fn insert(&mut self, corners: [[f64; 3]; 3], solid: usize, face: u32) {
        self.triangles.push((corners, solid, face));
    }

    /// Choose the cell size from the triangle count (about one triangle per
    /// cell on a surface) and bucket every triangle into the cells its box
    /// touches.
    fn finish(&mut self) {
        let count = self.triangles.len().max(1) as f64;
        // Surface triangles fill O(n^(1/2)) cells per side on a surface, but
        // the volume has n^(1/3) per side; the geometric mean keeps buckets
        // small without exploding the cell count.
        let per_side = count.powf(0.4).max(1.0);
        // No cell smaller than half the longest tessellated edge, so a large
        // flat face's two triangles are not bucketed into thousands of cells.
        let longest_edge = self
            .triangles
            .iter()
            .map(|(c, _, _)| distance(c[0], c[1]).max(distance(c[1], c[2])).max(distance(c[2], c[0])))
            .fold(0.0, f64::max);
        self.cell = (self.extent / per_side).max(longest_edge * 0.5).max(self.extent * 1e-6).max(1e-9);
        for (index, (corners, _, _)) in self.triangles.iter().enumerate() {
            let (low, high) = self.cell_span(*corners);
            for x in low[0]..=high[0] {
                for y in low[1]..=high[1] {
                    for z in low[2]..=high[2] {
                        self.cells.entry((x, y, z)).or_default().push(index as u32);
                    }
                }
            }
        }
    }

    fn coordinate(&self, point: [f64; 3]) -> (i32, i32, i32) {
        let c = |axis: usize| ((point[axis] - self.origin[axis]) / self.cell).floor().clamp(-1e6, 1e6) as i32;
        (c(0), c(1), c(2))
    }

    fn cell_span(&self, corners: [[f64; 3]; 3]) -> ([i32; 3], [i32; 3]) {
        let mut low = [i32::MAX; 3];
        let mut high = [i32::MIN; 3];
        for corner in corners {
            let (x, y, z) = self.coordinate(corner);
            for (axis, value) in [x, y, z].into_iter().enumerate() {
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
            }
        }
        (low, high)
    }

    /// The `(solid, face)` of the tessellated triangle nearest `point`, or
    /// `None` when none lies within reach (`MATCH_FRACTION` of the extent,
    /// and never more than `MAX_RINGS` cells).
    fn nearest(&self, point: [f64; 3]) -> Option<(usize, u32)> {
        if self.triangles.is_empty() {
            return None;
        }
        let (cx, cy, cz) = self.coordinate(point);
        let mut best: Option<(f64, usize, u32)> = None;
        for ring in 0..=MAX_RINGS {
            // Everything in this ring and beyond is at least `ring - 1` cells
            // away; past the reach there is nothing left to accept.
            if (ring - 1).max(0) as f64 * self.cell > self.reach_squared.sqrt() {
                break;
            }
            for x in cx - ring..=cx + ring {
                for y in cy - ring..=cy + ring {
                    for z in cz - ring..=cz + ring {
                        // Only the shell of this ring; the inside was searched.
                        if (x - cx).abs() != ring && (y - cy).abs() != ring && (z - cz).abs() != ring {
                            continue;
                        }
                        let Some(bucket) = self.cells.get(&(x, y, z)) else {
                            continue;
                        };
                        for index in bucket {
                            let (corners, solid, face) = &self.triangles[*index as usize];
                            let d = point_triangle_distance_squared(point, *corners);
                            if best.is_none_or(|(best_d, _, _)| d < best_d) {
                                best = Some((d, *solid, *face));
                            }
                        }
                    }
                }
            }
            // Everything not yet visited lies at least `ring` cells away.
            if let Some((d, solid, face)) = best {
                let reach = ring as f64 * self.cell;
                if d <= reach * reach {
                    return Some((solid, face));
                }
            }
        }
        best.filter(|(d, _, _)| *d <= self.reach_squared).map(|(_, solid, face)| (solid, face))
    }
}
