//! Stock-versus-part comparison: the deviation map.
//!
//! For every column of the simulated stock the stock's top point is compared
//! with the part mesh (machine coordinates): the deviation is the 3D distance
//! to the nearest part triangle, signed by where the point lies — outside the
//! part (above or beside it) is EXCESS stock (positive), inside the part is a
//! GOUGE (negative), so a finishing pass with stock-to-leave `s` reads `+s`
//! everywhere it reached, on sloped faces too (normal distance, not vertical
//! gap). An empty column where the part has material is a gouge measured from
//! the stock floor. Samples are aggregated per part face (by the nearest
//! triangle's face) and returned as a per-column grid for the viewport's
//! colour map.

use crate::index::{ProjectionMode, TriangleIndex};
use crate::mesh::MeshSnapshot;
use crate::model::{Bounds3, Point3};
use crate::stock_sim::DexelGrid;
use serde::{Deserialize, Serialize};

/// Deviation parameters.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct DeviationOptions {
    /// |deviation| at or below this counts as within tolerance.
    pub tolerance: f64,
    /// Largest distance searched for the nearest part triangle; columns farther
    /// from the part than this report `None` (stock far from the part).
    pub max_distance: f64,
}

impl Default for DeviationOptions {
    fn default() -> Self {
        Self { tolerance: 0.05, max_distance: 1_000.0 }
    }
}

/// Classification of one deviation value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DeviationClass {
    Within,
    Excess,
    Gouge,
}

/// Where a column's stock stands against the part, vertically.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ColumnPlacement {
    /// The part is in this column and the stock reaches its lowest surface
    /// there or higher: excess or gouge against the part as usual.
    AbovePart,
    /// No part in this column (beside it, or in a through hole) and the stock
    /// top within the part's height range: measured to the nearest face.
    WithinExtent,
    /// The stock top is under the part's lowest surface (the part's own in
    /// this column, else the whole part's): stock below the part, e.g. the
    /// block under a flange exit. Counted with its volume, never attributed to
    /// a face and never read as excess on the nearest wall.
    BelowPart,
}

/// Aggregate of the samples whose nearest triangle belongs to one part face.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceDeviation {
    /// Index into the part snapshot's `faces`.
    pub face: u32,
    pub solid: String,
    pub face_name: String,
    pub samples: u64,
    pub min: f64,
    pub max: f64,
    pub mean: f64,
    pub gouged: u64,
    pub excess: u64,
}

/// The deviation of the stock from the part, column by column.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DeviationMap {
    pub origin: Point3,
    pub cell_x: f64,
    pub cell_y: f64,
    pub nx: u32,
    pub ny: u32,
    /// Row-major per column: signed deviation at the stock top, `None` where
    /// the column has no stock and no part, or lies farther than `max_distance`.
    pub values: Vec<Option<f64>>,
    /// Stock-top sample height per column (the stock floor for empty columns).
    pub heights: Vec<Option<f64>>,
    /// Nearest part face per column.
    pub faces_of: Vec<Option<u32>>,
    pub faces: Vec<FaceDeviation>,
    pub tolerance: f64,
    pub min: f64,
    pub max: f64,
    pub within: u64,
    pub excess: u64,
    pub gouged: u64,
    /// Row-major per column: where its stock stands against the part
    /// (`None`: no stock and no part).
    #[serde(default)]
    pub placement: Vec<Option<ColumnPlacement>>,
    /// Columns of stock below the part, and their material volume. They carry
    /// no `values` entry and no face.
    #[serde(default)]
    pub below_part: u64,
    #[serde(default)]
    pub below_part_volume: f64,
}

impl DeviationMap {
    pub fn classify(&self, deviation: f64) -> DeviationClass {
        if deviation.abs() <= self.tolerance {
            DeviationClass::Within
        } else if deviation > 0.0 {
            DeviationClass::Excess
        } else {
            DeviationClass::Gouge
        }
    }

    pub fn value(&self, ix: u32, iy: u32) -> Option<f64> {
        self.values[(iy * self.nx + ix) as usize]
    }

    /// Column containing `(x, y)`.
    pub fn value_at(&self, x: f64, y: f64) -> Option<f64> {
        let ix = ((x - self.origin.x) / self.cell_x).floor();
        let iy = ((y - self.origin.y) / self.cell_y).floor();
        if ix < 0.0 || iy < 0.0 || ix >= self.nx as f64 || iy >= self.ny as f64 {
            return None;
        }
        self.value(ix as u32, iy as u32)
    }

    pub fn face(&self, name: &str) -> Option<&FaceDeviation> {
        self.faces.iter().find(|f| f.face_name == name)
    }
}

/// The point of triangle `abc` closest to `p` (Ericson, Real-Time Collision Detection).
pub fn closest_point_on_triangle(p: Point3, a: Point3, b: Point3, c: Point3) -> Point3 {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let ap = p.sub(a);
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }
    let bp = p.sub(b);
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = if (d1 - d3).abs() > 0.0 { d1 / (d1 - d3) } else { 0.0 };
        return a.add(ab.scale(v));
    }
    let cp = p.sub(c);
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = if (d2 - d6).abs() > 0.0 { d2 / (d2 - d6) } else { 0.0 };
        return a.add(ac.scale(w));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let denom = (d4 - d3) + (d5 - d6);
        let w = if denom > 0.0 { (d4 - d3) / denom } else { 0.0 };
        return b.add(c.sub(b).scale(w));
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    a.add(ab.scale(v)).add(ac.scale(w))
}

/// Squared distance from `p` to triangle `abc`.
pub fn point_triangle_distance_squared(p: Point3, a: Point3, b: Point3, c: Point3) -> f64 {
    p.sub(closest_point_on_triangle(p, a, b, c)).length_squared()
}

/// The part's lowest crossing just past its point `q` nearest to the stock
/// point `p` (stepped a hair from `p` towards and beyond `q`, into the part):
/// the bottom of the part beside a column that holds no part itself.
pub(crate) fn part_bottom_beside(part: &MeshSnapshot, index: &dyn TriangleIndex, p: Point3, q: Point3, step: f64, scratch: &mut Vec<u32>) -> Option<f64> {
    let (dx, dy) = (q.x - p.x, q.y - p.y);
    let len = (dx * dx + dy * dy).sqrt();
    let (x, y) = if len > 0.0 { (q.x + dx / len * step, q.y + dy / len * step) } else { (q.x, q.y) };
    part_bottom_under(part, index, x, y, scratch)
}

/// Nearest part triangle to `p` within `max_distance`: `(triangle id, distance)`.
/// Searches the index in boxes that double until a hit is found and confirmed
/// (every triangle within the found distance has been examined).
pub fn nearest_triangle(part: &MeshSnapshot, index: &dyn TriangleIndex, p: Point3, start_radius: f64, max_distance: f64) -> Option<(u32, f64)> {
    let mut radius = start_radius.max(f64::MIN_POSITIVE);
    let mut candidates = Vec::new();
    loop {
        let q = Bounds3::from_point(p).expanded(radius);
        candidates.clear();
        index.query_aabb(&q, ProjectionMode::Xyz, &mut candidates);
        let mut best: Option<(u32, f64)> = None;
        for &id in &candidates {
            let t = part.triangle(id);
            let d2 = point_triangle_distance_squared(p, t.a, t.b, t.c);
            if best.map_or(true, |(_, d)| d2 < d * d) {
                best = Some((id, d2.sqrt()));
            }
        }
        // A hit inside the box radius is final: anything closer would be in the box too.
        if let Some((id, d)) = best {
            if d <= radius {
                return Some((id, d));
            }
        }
        if radius >= max_distance {
            return best.filter(|(_, d)| *d <= max_distance);
        }
        radius = (radius * 2.0).min(max_distance);
    }
}

/// Whether `p` is inside the closed part mesh: parity of the part's vertical
/// crossings above `p` under its column (brute force over the column's candidates).
pub(crate) fn inside_part(part: &MeshSnapshot, index: &dyn TriangleIndex, p: Point3, scratch: &mut Vec<u32>) -> bool {
    scratch.clear();
    let q = Bounds3::new(Point3::new(p.x, p.y, f64::NEG_INFINITY), Point3::new(p.x, p.y, f64::INFINITY));
    index.query_aabb(&q, ProjectionMode::Xy, scratch);
    let mut hits: Vec<f64> = scratch
        .iter()
        .filter_map(|&id| {
            let t = part.triangle(id);
            crate::stock_sim::vertical_ray_hit(t.a, t.b, t.c, p.x, p.y)
        })
        .collect();
    hits.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    hits.dedup();
    hits.iter().filter(|&&z| z > p.z).count() % 2 == 1
}

/// Lowest part crossing of the vertical line through `(x, y)`, if the part is there.
pub(crate) fn part_bottom_under(part: &MeshSnapshot, index: &dyn TriangleIndex, x: f64, y: f64, scratch: &mut Vec<u32>) -> Option<f64> {
    scratch.clear();
    let q = Bounds3::new(Point3::new(x, y, f64::NEG_INFINITY), Point3::new(x, y, f64::INFINITY));
    index.query_aabb(&q, ProjectionMode::Xy, scratch);
    scratch
        .iter()
        .filter_map(|&id| {
            let t = part.triangle(id);
            crate::stock_sim::vertical_ray_hit(t.a, t.b, t.c, x, y)
        })
        .fold(None, |m: Option<f64>, z| Some(m.map_or(z, |m| m.min(z))))
}

/// Highest part surface under column `(x, y)` and its face, if the part is there.
fn part_top_under(part: &MeshSnapshot, index: &dyn TriangleIndex, x: f64, y: f64, scratch: &mut Vec<u32>) -> Option<(f64, u32)> {
    scratch.clear();
    let q = Bounds3::new(Point3::new(x, y, f64::NEG_INFINITY), Point3::new(x, y, f64::INFINITY));
    index.query_aabb(&q, ProjectionMode::Xy, scratch);
    let mut best: Option<(f64, u32)> = None;
    for &id in scratch.iter() {
        let t = part.triangle(id);
        if let Some(z) = crate::stock_sim::vertical_ray_hit(t.a, t.b, t.c, x, y) {
            if best.map_or(true, |(bz, _)| z > bz) {
                best = Some((z, t.face));
            }
        }
    }
    best
}

/// Compare the stock with the part. `part` is the target bodies' snapshot in
/// machine coordinates (protected bodies excluded); `index` is built on it.
/// One sampled column whose stock stands more than the tolerance off the part.
#[derive(Clone, Debug, PartialEq)]
pub struct ExcessSample {
    /// The stock surface sampled (the column's top), in the map's frame.
    pub point: Point3,
    /// Its distance to the part, > the map's tolerance.
    pub excess: f64,
    /// Index into the part snapshot's `faces`: the nearest face.
    pub face: u32,
    /// The part point nearest to `point`.
    pub nearest: Point3,
}

/// The columns of `map` that read excess, with the stock point, the excess and
/// the nearest part face and point, in column order. `part`, `index` and
/// `options` must be the ones `map` was computed with. Lets a caller place the
/// material left (against trimmed or unreachable toolpath regions, say) where a
/// per-face summary only counts it.
pub fn excess_samples(map: &DeviationMap, part: &MeshSnapshot, index: &dyn TriangleIndex, options: &DeviationOptions) -> Vec<ExcessSample> {
    let start_radius = map.cell_x.max(map.cell_y);
    let mut out = Vec::new();
    for iy in 0..map.ny {
        for ix in 0..map.nx {
            let k = (iy * map.nx + ix) as usize;
            let (Some(excess), Some(z), Some(face)) = (map.values[k], map.heights[k], map.faces_of[k]) else { continue };
            if excess <= map.tolerance {
                continue;
            }
            let point = Point3::new(map.origin.x + (ix as f64 + 0.5) * map.cell_x, map.origin.y + (iy as f64 + 0.5) * map.cell_y, z);
            // An excess reading is the distance to this nearest triangle.
            let Some((tri, _)) = nearest_triangle(part, index, point, start_radius, options.max_distance) else { continue };
            let t = part.triangle(tri);
            out.push(ExcessSample { point, excess, face, nearest: closest_point_on_triangle(point, t.a, t.b, t.c) });
        }
    }
    out
}

pub fn deviation_map(grid: &DexelGrid, part: &MeshSnapshot, index: &dyn TriangleIndex, options: &DeviationOptions) -> DeviationMap {
    let n = grid.columns.len();
    let mut values = vec![None; n];
    let mut heights = vec![None; n];
    let mut faces_of = vec![None; n];
    let mut per_face: Vec<Option<FaceDeviation>> = vec![None; part.faces.len()];
    let mut placement = vec![None; n];
    let (mut below_part, mut below_part_volume) = (0u64, 0.0f64);
    // The part's lowest point: stock entirely under it is below the part.
    let part_bottom = part.bounds.min.z;
    let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
    let (mut within, mut excess, mut gouged) = (0u64, 0u64, 0u64);
    let start_radius = grid.cell_x.max(grid.cell_y);
    let mut scratch = Vec::new();
    for iy in 0..grid.ny {
        for ix in 0..grid.nx {
            let k = (iy * grid.nx + ix) as usize;
            let (x, y) = grid.center(ix, iy);
            let column = grid.column(ix, iy);
            // Sample point: the stock top, or the stock floor where everything is gone.
            let (z, empty) = match column.top() {
                Some(top) => (top, false),
                None => (grid.z_range[0], true),
            };
            let p = Point3::new(x, y, z);
            let (deviation, face) = if empty {
                // Everything is gone: a gouge of the part's full height here, if the part is here.
                match part_top_under(part, index, x, y, &mut scratch) {
                    Some((top, face)) if top > z => {
                        placement[k] = Some(ColumnPlacement::AbovePart);
                        (-(top - z), face)
                    }
                    _ => continue,
                }
            } else {
                // Stock under the part's lowest surface is below the part: counted
                // with its volume, never measured to the nearest face.
                // Where the part is in this column, only stock strictly under its
                // lowest surface there is below it (stock ending AT that surface
                // means the part above was cut away: a gouge). Where it is not
                // (beside the part, a through hole), stock reaching no higher than
                // the part's lowest point is below it: the block under a flange
                // exit, flush with it.
                let local_bottom = part_bottom_under(part, index, x, y, &mut scratch);
                let below = match local_bottom {
                    Some(b) => z < b - options.tolerance,
                    None => {
                        // The part beside this column: its bottom next to the nearest
                        // part point (a bolt hole's flange, not the whole part's
                        // lowest point, which may lie far lower elsewhere).
                        let beside = nearest_triangle(part, index, p, start_radius, options.max_distance).and_then(|(tri, _)| {
                            let t = part.triangle(tri);
                            let q = closest_point_on_triangle(p, t.a, t.b, t.c);
                            part_bottom_beside(part, index, p, q, 1e-3 * grid.cell_x.min(grid.cell_y), &mut scratch)
                        });
                        z <= beside.unwrap_or(part_bottom) + options.tolerance
                    }
                };
                if below {
                    placement[k] = Some(ColumnPlacement::BelowPart);
                    below_part += 1;
                    below_part_volume += column.height() * grid.cell_x * grid.cell_y;
                    continue;
                }
                placement[k] = Some(if local_bottom.is_some() { ColumnPlacement::AbovePart } else { ColumnPlacement::WithinExtent });
                // The part is in this column but the stock ends at (or under) its
                // lowest surface: the part's whole height here is gone, a gouge of
                // that height, as for an emptied column. (The top sample sits on the
                // part's bottom face and would read 0.)
                if let Some(b) = local_bottom {
                    if z <= b + options.tolerance {
                        if let Some((top, face)) = part_top_under(part, index, x, y, &mut scratch) {
                            if top > z + options.tolerance {
                                let deviation = -(top - z);
                                values[k] = Some(deviation);
                                heights[k] = Some(z);
                                faces_of[k] = Some(face);
                                min = min.min(deviation);
                                max = max.max(deviation);
                                gouged += 1;
                                let entry = per_face[face as usize].get_or_insert_with(|| FaceDeviation {
                                    face,
                                    solid: part.faces[face as usize].solid.clone(),
                                    face_name: part.faces[face as usize].face.clone(),
                                    samples: 0,
                                    min: f64::INFINITY,
                                    max: f64::NEG_INFINITY,
                                    mean: 0.0,
                                    gouged: 0,
                                    excess: 0,
                                });
                                entry.samples += 1;
                                entry.min = entry.min.min(deviation);
                                entry.max = entry.max.max(deviation);
                                entry.mean += deviation;
                                entry.gouged += 1;
                                continue;
                            }
                        }
                    }
                }
                let inside = inside_part(part, index, p, &mut scratch);
                let Some((tri, distance)) = nearest_triangle(part, index, p, start_radius, options.max_distance) else {
                    continue;
                };
                (if inside { -distance } else { distance }, part.triangle(tri).face)
            };
            values[k] = Some(deviation);
            heights[k] = Some(z);
            faces_of[k] = Some(face);
            min = min.min(deviation);
            max = max.max(deviation);
            let class = if deviation.abs() <= options.tolerance {
                within += 1;
                DeviationClass::Within
            } else if deviation > 0.0 {
                excess += 1;
                DeviationClass::Excess
            } else {
                gouged += 1;
                DeviationClass::Gouge
            };
            let entry = per_face[face as usize].get_or_insert_with(|| FaceDeviation {
                face,
                solid: part.faces[face as usize].solid.clone(),
                face_name: part.faces[face as usize].face.clone(),
                samples: 0,
                min: f64::INFINITY,
                max: f64::NEG_INFINITY,
                mean: 0.0,
                gouged: 0,
                excess: 0,
            });
            entry.samples += 1;
            entry.min = entry.min.min(deviation);
            entry.max = entry.max.max(deviation);
            entry.mean += deviation;
            match class {
                DeviationClass::Gouge => entry.gouged += 1,
                DeviationClass::Excess => entry.excess += 1,
                DeviationClass::Within => {}
            }
        }
    }
    let faces: Vec<FaceDeviation> = per_face
        .into_iter()
        .flatten()
        .map(|mut f| {
            f.mean /= f.samples as f64;
            f
        })
        .collect();
    if min > max {
        min = 0.0;
        max = 0.0;
    }
    DeviationMap {
        origin: grid.origin,
        cell_x: grid.cell_x,
        cell_y: grid.cell_y,
        nx: grid.nx,
        ny: grid.ny,
        values,
        heights,
        faces_of,
        faces,
        tolerance: options.tolerance,
        min,
        max,
        within,
        excess,
        gouged,
        placement,
        below_part,
        below_part_volume,
    }
}

