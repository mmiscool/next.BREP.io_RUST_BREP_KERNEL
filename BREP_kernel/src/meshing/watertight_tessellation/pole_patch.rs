//! Watertight tessellation of an UNTRIMMED pole-to-pole patch on a general
//! surface: one face whose single loop is the whole parameter rectangle —
//! a degenerate (pole) edge along `v = v0`, the seam up `u = u1`, the other
//! pole edge along `v = v1` and the seam back down `u = u0` — on a surface
//! closed in `u` and open in `v`. That is the sphere's topology on a surface
//! that is NOT a sphere: the NURBS Solid's control-cage body
//! (`geometry/control_cage.rs`), or a revolved spline profile that touches
//! the axis at both ends.
//!
//! The generic path meshes such a face as a trim region in its own domain:
//! it seeds a station grid into the ear-clipped boundary with a linear
//! host-triangle walk, then flips and refines, and on a 10 mm cage body that
//! is two seconds at the display chord (18 000 triangles, thirty million
//! host visits) — for a face that has no trim at all. The cage editor re-runs
//! the history on every drag frame, so that path makes the body un-draggable.
//!
//! An untrimmed rectangle needs no triangulation: this lane lays a STRUCTURED
//! grid over the domain — columns at the surface's `u` knots and rows at the
//! seam edge's own sample parameters, both bisected until every cell's chord
//! sag is under the tolerance — and emits the quads as triangle pairs, with
//! the pole rows collapsed into one vertex each and the `u = u1` column
//! reading the `u = u0` column's vertices. Watertightness holds by
//! construction: the seam is one shared column, each pole is one shared
//! vertex, and the seam column sits exactly on the shared edge samples the
//! seam polyline is drawn from.
//!
//! The lane DECLINES — `Ok(false)`, leaving the face to the existing paths
//! bit-for-bit — for every face that is not exactly this shape: an analytic
//! carrier (the sphere atlas, the cone fan and the periodic-wall paths own
//! those), a trimmed face, more than one loop, a loop whose coedges are not
//! the four above, or seam samples that do not lie on the surface.

use super::*;

/// The most columns or rows the grid may reach; beyond this the chord is
/// simply not met (a pathological cage cannot explode the mesh).
const MAX_GRID_LINES: usize = 512;
/// Refinement passes: each halves the offending intervals.
const MAX_GRID_PASSES: usize = 10;
/// Agreement needed between a shared edge sample and the surface's own
/// evaluation, as a fraction of the patch's size.
const SAMPLE_AGREEMENT: f64 = 1e-7;

fn decline(face_id: u32, why: &str) -> Result<bool, String> {
    if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
        eprintln!("[pole-patch] face {face_id}: DECLINED — {why}");
    }
    Ok(false)
}

/// The loop's role for each coedge, in loop order: which seam direction and
/// which pole an untrimmed pole-to-pole rectangle's four coedges carry.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    BottomPole,
    SeamUp,
    TopPole,
    SeamDown,
}

/// The endpoints of a coedge's pcurve as `(u, v)` pairs.
fn pcurve_ends(coedge: &CoedgeRecord) -> Result<([f64; 2], [f64; 2]), String> {
    let [t0, t1] = coedge.pcurve.domain()?;
    let a = coedge.pcurve.evaluate(t0)?;
    let b = coedge.pcurve.evaluate(t1)?;
    Ok(([a.x, a.y], [b.x, b.y]))
}

/// Classify one coedge of the loop against the domain, or `None` when it is
/// not one of the four sides.
fn classify(coedge: &CoedgeRecord, domains: [f64; 4], degenerate: bool) -> Result<Option<Side>, String> {
    let [u0, u1, v0, v1] = domains;
    let tol = 1e-9 * ((u1 - u0).abs() + (v1 - v0).abs());
    let (a, b) = pcurve_ends(coedge)?;
    let near = |x: f64, y: f64| (x - y).abs() <= tol;
    let side = if degenerate {
        if near(a[1], v0) && near(b[1], v0) {
            Side::BottomPole
        } else if near(a[1], v1) && near(b[1], v1) {
            Side::TopPole
        } else {
            return Ok(None);
        }
    } else if near(a[0], u1) && near(b[0], u1) && near(a[1], v0) && near(b[1], v1) {
        Side::SeamUp
    } else if near(a[0], u0) && near(b[0], u0) && near(a[1], v1) && near(b[1], v0) {
        Side::SeamDown
    } else {
        return Ok(None);
    };
    Ok(Some(side))
}

/// Mesh `face` through the structured grid when it is an untrimmed
/// pole-to-pole patch (see the module docs); `Ok(false)` leaves it alone.
pub(super) fn tessellate_pole_patch_watertight(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<bool, String> {
    if std::env::var("BREP_NO_POLE_PATCH").is_ok() {
        return Ok(false);
    }
    if face.surface.analytic().is_some() {
        return Ok(false);
    }
    if face.loops.len() != 1 || face.loops[0].coedges.len() != 4 {
        return Ok(false);
    }
    let (closed_u, closed_v) = face.surface.closed_directions()?;
    if !closed_u || closed_v {
        return Ok(false);
    }
    let ku = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let domains = [u0, u1, v0, v1];

    // 1. The four sides, each once, and the seam used by BOTH seam coedges.
    let coedges = &face.loops[0].coedges;
    let mut sides = Vec::with_capacity(4);
    for coedge in coedges {
        let edge_samples = samples
            .get(&coedge.edge_id)
            .ok_or("pole patch: missing edge samples")?;
        let degenerate = edge_samples
            .positions
            .windows(2)
            .all(|pair| pair[0].sub(pair[1]).length() == 0.0);
        match classify(coedge, domains, degenerate)? {
            Some(side) => sides.push((side, coedge)),
            None => return decline(face_id, "a coedge is not a pole or seam side"),
        }
    }
    let find = |wanted: Side| sides.iter().find(|(side, _)| *side == wanted).map(|(_, c)| *c);
    let (Some(bottom), Some(up), Some(top), Some(down)) =
        (find(Side::BottomPole), find(Side::SeamUp), find(Side::TopPole), find(Side::SeamDown))
    else {
        return decline(face_id, "the loop does not carry each side once");
    };
    if up.edge_id != down.edge_id || up.forward == down.forward {
        return decline(face_id, "the two seam coedges do not share one edge in opposite senses");
    }
    if bottom.edge_id == top.edge_id {
        return decline(face_id, "both poles name the same edge");
    }

    // 2. The rows: the seam edge's own samples, which must lie ON the surface
    //    at `u0` — then the seam column reproduces the drawn seam exactly.
    let seam = &samples[&up.edge_id];
    let mut extent = 0.0f64;
    for row in &face.surface.control_points {
        for cp in row {
            let w = if cp.w != 0.0 { cp.w } else { 1.0 };
            extent = extent.max((cp.x / w).abs()).max((cp.y / w).abs()).max((cp.z / w).abs());
        }
    }
    let agreement = SAMPLE_AGREEMENT * extent.max(1e-9);
    let seam_v: Vec<f64> = if up.forward {
        seam.fractions.iter().map(|f| v0 + f * (v1 - v0)).collect()
    } else {
        seam.fractions.iter().rev().map(|f| v1 - f * (v1 - v0)).collect()
    };
    let seam_positions: Vec<Vec3> = if up.forward {
        seam.positions.clone()
    } else {
        seam.positions.iter().rev().copied().collect()
    };
    if seam_v.len() < 2 || seam_v.len() != seam_positions.len() {
        return decline(face_id, "the seam carries no samples");
    }
    for (v, position) in seam_v.iter().zip(&seam_positions) {
        if face.surface.evaluate(u0, *v)?.sub(*position).length() > agreement {
            return decline(face_id, "a seam sample is off the surface at u0");
        }
    }
    let pole_bottom = samples[&bottom.edge_id].positions[0];
    let pole_top = samples[&top.edge_id].positions[0];
    if face.surface.evaluate(u0, v0)?.sub(pole_bottom).length() > agreement
        || face.surface.evaluate(u0, v1)?.sub(pole_top).length() > agreement
    {
        return decline(face_id, "a pole vertex is off the surface");
    }

    // 3. The grid: columns at the u knots, rows at the seam samples, both
    //    bisected until every cell's chord sag is within tolerance.
    let mut cols: Vec<f64> = vec![u0];
    for knot in &face.surface.knots_u {
        if *knot > u0 && *knot < u1 && cols.last().is_some_and(|last| knot - last > 1e-12 * (u1 - u0)) {
            cols.push(*knot);
        }
    }
    cols.push(u1);
    let mut rows: Vec<f64> = seam_v.clone();
    rows.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (v1 - v0));
    // The pole rows are the first and last; interior rows carry the sag.
    for _pass in 0..MAX_GRID_PASSES {
        let mut split_cols = vec![false; cols.len() - 1];
        let mut split_rows = vec![false; rows.len() - 1];
        let mut corners: Vec<Vec<Vec3>> = Vec::with_capacity(cols.len());
        for u in &cols {
            let mut column = Vec::with_capacity(rows.len());
            for v in &rows {
                column.push(face.surface.evaluate(*u, *v)?);
            }
            corners.push(column);
        }
        for ci in 0..cols.len() - 1 {
            let um = 0.5 * (cols[ci] + cols[ci + 1]);
            for ri in 0..rows.len() - 1 {
                let vm = 0.5 * (rows[ri] + rows[ri + 1]);
                let a = corners[ci][ri];
                let b = corners[ci + 1][ri];
                let c = corners[ci + 1][ri + 1];
                let d = corners[ci][ri + 1];
                // Along u, on the row with the longer chord (a pole row has none).
                if !split_cols[ci] {
                    let (row_v, pa, pb) = if b.sub(a).length() >= c.sub(d).length() {
                        (rows[ri], a, b)
                    } else {
                        (rows[ri + 1], d, c)
                    };
                    let mid = face.surface.evaluate(um, row_v)?;
                    if deflection_to_chord(mid, pa, pb) > chord_tolerance {
                        split_cols[ci] = true;
                    }
                }
                // Along v, on the column with the longer chord.
                if !split_rows[ri] {
                    let (col_u, pa, pd) = if d.sub(a).length() >= c.sub(b).length() {
                        (cols[ci], a, d)
                    } else {
                        (cols[ci + 1], b, c)
                    };
                    let mid = face.surface.evaluate(col_u, vm)?;
                    if deflection_to_chord(mid, pa, pd) > chord_tolerance {
                        split_rows[ri] = true;
                    }
                }
                // Across the cell: the centre against the two diagonals.
                if !split_cols[ci] || !split_rows[ri] {
                    let centre = face.surface.evaluate(um, vm)?;
                    let sag = deflection_to_chord(centre, a, c).min(deflection_to_chord(centre, b, d));
                    if sag > chord_tolerance {
                        if cols[ci + 1] - cols[ci] >= rows[ri + 1] - rows[ri] {
                            split_cols[ci] = true;
                        } else {
                            split_rows[ri] = true;
                        }
                    }
                }
            }
        }
        let any_col = split_cols.iter().any(|s| *s);
        let any_row = split_rows.iter().any(|s| *s);
        if !any_col && !any_row {
            break;
        }
        if any_col && cols.len() < MAX_GRID_LINES {
            let mut refined = Vec::with_capacity(cols.len() * 2);
            for (ci, u) in cols.iter().enumerate() {
                refined.push(*u);
                if ci + 1 < cols.len() && split_cols[ci] {
                    refined.push(0.5 * (cols[ci] + cols[ci + 1]));
                }
            }
            cols = refined;
        }
        if any_row && rows.len() < MAX_GRID_LINES {
            let mut refined = Vec::with_capacity(rows.len() * 2);
            for (ri, v) in rows.iter().enumerate() {
                refined.push(*v);
                if ri + 1 < rows.len() && split_rows[ri] {
                    refined.push(0.5 * (rows[ri] + rows[ri + 1]));
                }
            }
            rows = refined;
        }
        if cols.len() >= MAX_GRID_LINES && rows.len() >= MAX_GRID_LINES {
            break;
        }
    }

    // 4. Emit: one vertex per (interior row, column except the seam copy),
    //    one per pole, the seam column shared.
    let nc = cols.len() - 1; // distinct columns (the last col is the u0 column again)
    let nr = rows.len();
    let base = (mesh.positions.len() / 3) as u32;
    let mut push = |position: Vec3, normal: Vec3| -> u32 {
        mesh.positions.extend([position.x, position.y, position.z]);
        mesh.normals.extend([normal.x, normal.y, normal.z]);
        (mesh.positions.len() / 3 - 1) as u32
    };
    let pole_normal = |v: f64| -> Result<Vec3, String> {
        // The pole's own derivative is zero; average the normals just inside
        // it around the ring so one shared vertex carries one direction.
        let inset = if v == v0 { v0 + 1e-4 * (v1 - v0) } else { v1 - 1e-4 * (v1 - v0) };
        let mut sum = Vec3::default();
        for u in &cols[..nc] {
            sum = sum.add(face_normal_at(face, [*u, inset], domains)?);
        }
        match sum.normalized() {
            Ok(n) => Ok(n),
            Err(_) => face_normal_at(face, [u0, inset], domains),
        }
    };
    let bottom_index = push(pole_bottom, pole_normal(v0)?);
    let top_index = push(pole_top, pole_normal(v1)?);
    // interior[ri - 1][ci] for ri in 1..nr-1, ci in 0..nc
    let mut interior: Vec<Vec<u32>> = Vec::with_capacity(nr.saturating_sub(2));
    for ri in 1..nr.saturating_sub(1) {
        let mut row = Vec::with_capacity(nc);
        for (ci, u) in cols.iter().take(nc).enumerate() {
            let position = if ci == 0 {
                // The seam column: the shared edge sample where one exists,
                // the surface otherwise (a bisected row).
                seam_v
                    .iter()
                    .position(|sv| (*sv - rows[ri]).abs() <= 1e-12 * (v1 - v0))
                    .map(|i| seam_positions[i])
                    .unwrap_or(face.surface.evaluate(*u, rows[ri])?)
            } else {
                face.surface.evaluate(*u, rows[ri])?
            };
            let normal = face_normal_at(face, [*u, rows[ri]], domains)?;
            row.push(push(position, normal));
        }
        interior.push(row);
    }
    let vertex = |ri: usize, ci: usize| -> u32 {
        if ri == 0 {
            bottom_index
        } else if ri == nr - 1 {
            top_index
        } else {
            interior[ri - 1][ci % nc]
        }
    };
    let position_of = |index: u32| -> Vec3 {
        let i = (index - base) as usize * 3;
        Vec3::new(mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2])
    };
    let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(nc * nr * 2);
    for ri in 0..nr - 1 {
        for ci in 0..nc {
            let a = vertex(ri, ci);
            let b = vertex(ri, ci + 1);
            let c = vertex(ri + 1, ci + 1);
            let d = vertex(ri + 1, ci);
            // uv-CCW: (u, v) → (u+, v) → (u+, v+) and (u, v) → (u+, v+) → (u, v+).
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    for [a, b, c] in triangles {
        // A pole fan's collapsed side (two coincident vertices) is no triangle.
        if a == b || b == c || c == a {
            continue;
        }
        let (pa, pb, pc) = (position_of(a), position_of(b), position_of(c));
        if pa.sub(pb).length() == 0.0 || pb.sub(pc).length() == 0.0 || pc.sub(pa).length() == 0.0 {
            continue;
        }
        if face.same_sense {
            mesh.indices.extend([a, b, c]);
        } else {
            mesh.indices.extend([a, c, b]);
        }
        mesh.face_ids.push(face_id);
    }
    if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
        eprintln!(
            "[pole-patch] face {face_id}: {} columns × {} rows, {} triangles",
            nc,
            nr,
            mesh.face_ids.iter().filter(|id| **id == face_id).count()
        );
    }
    Ok(true)
}

