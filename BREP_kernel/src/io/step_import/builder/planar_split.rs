//! Phase 1.75: exactify "planar" faces whose boundary does not lie on their
//! plane, BEFORE any edge curve is bent onto a surface and before pcurves are
//! derived.
//!
//! A mesh-derived STEP body — the STL conversion's region fit, a vendor
//! polyhedral export — can carry a PLANE whose boundary vertices sit off it by
//! far more than the kernel's assembly weld radius: 3.7e-3 on the 2026-09-12
//! "boolean fails after long time" report against a 1e-5 weld. Nothing exact
//! downstream can consume that face. `reconcile_edges_onto_surfaces` bends
//! the straight edge onto the FIRST face that claims it, which only moves the
//! disagreement to the neighbour; the pcurve band is model-relative and wide
//! by design, so the import validates; and then the boolean imprint clips a
//! section curve against this face's pcurve boundary on ITS plane while the
//! neighbour clips against ITS plane, and the two endpoints of what must be
//! one vertex land a few 1e-3 apart. The assembler bridges them with slivers
//! and the shell fails its genus gate — after paying twenty perturbation rungs.
//!
//! The only EXACT repair is to replace the face by faces whose planes DO
//! contain their boundary: the straight-edged polygon is ear-clipped in the
//! plane's own frame and every triangle gets a plane through its three
//! vertices. Original edges keep their 3D lines, so the neighbouring faces
//! share them unchanged; the diagonals are the same implicit straight edges a
//! FACETED_BREP uses (`faceted_edge`); and `finish_face` then derives every
//! pcurve as the exact projection of a line onto its own plane. Faces whose
//! boundary already lies within the band, faces with holes, and faces with a
//! curved boundary edge are left untouched.
//!
//! The band is the boolean's own assembly weld at its default model
//! tolerance: below it two points are one vertex, above it the assembler keeps
//! them apart, so an edge farther off its face than this can never assemble.

use super::*;
use crate::AnalyticSurface;

impl<'a> SolidBuilder<'a> {
    /// Replace every pending planar face whose straight boundary sits more than
    /// the assembly weld off its plane by exactly planar triangles. Returns the
    /// number of faces split.
    pub(super) fn split_off_plane_planar_faces(
        &mut self,
        pending: &mut Vec<PendingFace>,
    ) -> Result<usize, String> {
        let band = crate::tolerance::assembler_weld(crate::BooleanOptions::default().tolerance);
        let debug = std::env::var("BREP_DEBUG_PLANAR_SPLIT").is_ok();
        let mut split = 0usize;
        let mut faces = Vec::with_capacity(pending.len());
        for face in pending.drain(..) {
            match self.plan_planar_split(&face, band)? {
                Some(plan) => {
                    if debug {
                        eprintln!(
                            "PLANAR SPLIT surface #{}: {} vertices {:.3e} off the plane -> {} triangles",
                            face.surface_ref,
                            plan.vertex_ids.len(),
                            plan.off_plane,
                            plan.triangles.len()
                        );
                    }
                    let pieces = self.build_planar_triangles(&face, &plan)?;
                    faces.extend(pieces);
                    split += 1;
                }
                None => faces.push(face),
            }
        }
        *pending = faces;
        if debug {
            eprintln!("PLANAR SPLIT replaced {split} face(s) (band {band:.1e})");
        }
        Ok(split)
    }

    fn plan_planar_split(&self, face: &PendingFace, band: f64) -> Result<Option<SplitPlan>, String> {
        let Some(AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        }) = face.surface.analytic()
        else {
            return Ok(None);
        };
        // Holes need a bridged triangulation; nothing that reaches here has one.
        let [(specs, true)] = face.bounds.as_slice() else {
            return Ok(None);
        };
        if specs.len() < 3 {
            return Ok(None);
        }
        let Ok(normal) = u_dir.cross(*v_dir).normalized() else {
            return Ok(None);
        };
        let Ok(e1) = u_dir.normalized() else {
            return Ok(None);
        };
        let e2 = normal.cross(e1);

        let mut vertex_ids = Vec::with_capacity(specs.len());
        let mut off_plane = 0.0f64;
        for (index, &(edge_id, forward)) in specs.iter().enumerate() {
            let edge = self.edge_record(edge_id);
            if edge.degenerate {
                return Ok(None);
            }
            let (from, to) = if forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            };
            // The loop must chain vertex to vertex; anything else is for validate.
            let (next_id, next_forward) = specs[(index + 1) % specs.len()];
            let next = self.edge_record(next_id);
            let next_from = if next_forward { next.start_vertex_id } else { next.end_vertex_id };
            if next_from != to {
                return Ok(None);
            }
            let start = self.vertex_point(from);
            let end = self.vertex_point(to);
            // Only a straight edge keeps a triangle through its ends exactly planar.
            if !edge_is_straight(edge, start, end, band)? {
                return Ok(None);
            }
            vertex_ids.push(from);
            off_plane = off_plane.max(start.sub(*origin).dot(normal).abs());
        }
        if off_plane <= band {
            return Ok(None);
        }
        // A loop that visits a vertex twice is pinched; ear clipping assumes a
        // simple polygon.
        let mut sorted = vertex_ids.clone();
        sorted.sort_unstable();
        if sorted.windows(2).any(|pair| pair[0] == pair[1]) {
            return Ok(None);
        }
        let polygon: Vec<[f64; 2]> = vertex_ids
            .iter()
            .map(|&id| {
                let delta = self.vertex_point(id).sub(*origin);
                [delta.dot(e1), delta.dot(e2)]
            })
            .collect();
        let Some(triangles) = ear_clip(&polygon) else {
            return Ok(None);
        };
        Ok(Some(SplitPlan {
            vertex_ids,
            triangles,
            off_plane,
        }))
    }

    fn build_planar_triangles(
        &mut self,
        face: &PendingFace,
        plan: &SplitPlan,
    ) -> Result<Vec<PendingFace>, String> {
        let specs = &face.bounds[0].0;
        let count = plan.vertex_ids.len();
        let outward = {
            let Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) = face.surface.analytic() else {
                return Err("step_import: planar split planned on a non-plane".into());
            };
            let normal = u_dir.cross(*v_dir).normalized()?;
            if face.same_sense {
                normal
            } else {
                normal.scale(-1.0)
            }
        };
        let mut pieces = Vec::with_capacity(plan.triangles.len());
        for triangle in &plan.triangles {
            let ids = [
                plan.vertex_ids[triangle[0]],
                plan.vertex_ids[triangle[1]],
                plan.vertex_ids[triangle[2]],
            ];
            let a = self.vertex_point(ids[0]);
            let b = self.vertex_point(ids[1]);
            let c = self.vertex_point(ids[2]);
            // Plane through the three vertices, u along ab, v toward c; the
            // domain spans the triangle's uv bounding box because evaluation
            // clamps to it (an obtuse apex sits at negative u otherwise).
            let ab = b.sub(a);
            let u_length = ab.length();
            if !(u_length > 0.0) {
                return Err("step_import: planar split produced a degenerate triangle".into());
            }
            let u_direction = ab.scale(1.0 / u_length);
            let ac = c.sub(a);
            let c_u = ac.dot(u_direction);
            let v = ac.sub(u_direction.scale(c_u));
            let v_length = v.length();
            if !(v_length > 0.0) {
                return Err("step_import: planar split produced a degenerate triangle".into());
            }
            let v_direction = v.scale(1.0 / v_length);
            let u_low = c_u.min(0.0);
            let u_high = c_u.max(u_length);
            let surface = make_plane(
                a.add(u_direction.scale(u_low)),
                u_direction,
                v_direction,
                u_high - u_low,
                v_length,
            )?;
            // The triangle is counter-clockwise in its own frame by
            // construction (c has positive v), so its interior is on the
            // loop's left; the material side is then whichever sense
            // reproduces the source face's outward normal.
            let same_sense = u_direction.cross(v_direction).dot(outward) > 0.0;
            let mut triangle_specs = Vec::with_capacity(3);
            for side in 0..3 {
                let from_index = triangle[side];
                let to_index = triangle[(side + 1) % 3];
                let spec = if (from_index + 1) % count == to_index {
                    // A boundary side in loop order: the source coedge verbatim.
                    specs[from_index]
                } else {
                    // A diagonal: the implicit straight edge between the two
                    // vertices, shared with the triangle across it.
                    let diagonal = self.faceted_edge(ids[side], ids[(side + 1) % 3])?;
                    if let Some(capture) = self.readings.as_mut() {
                        // The file states no such edge: it is ours.
                        capture.written.remove(&diagonal.0);
                    }
                    if self.readings.as_ref().is_some_and(|c| !c.stages.contains_key(&diagonal.0)) {
                        self.capture_stage(diagonal.0, || "planar-split diagonal".into());
                    }
                    diagonal
                };
                triangle_specs.push(spec);
            }
            pieces.push(PendingFace {
                face_ref: face.face_ref,
                surface,
                // No STEP surface entity describes this plane, so no supplied
                // pcurve can name it.
                surface_ref: usize::MAX,
                same_sense,
                bounds: vec![(triangle_specs, true)],
            });
        }
        Ok(pieces)
    }
}

/// The boundary polygon of a face that needs splitting: vertex ids in loop
/// order and the ear-clipped triangles as loop-index triples in loop
/// orientation.
struct SplitPlan {
    vertex_ids: Vec<u64>,
    triangles: Vec<[usize; 3]>,
    off_plane: f64,
}

/// A straight edge stays within `band` of the chord between its ends.
fn edge_is_straight(edge: &EdgeRecord, start: Vec3, end: Vec3, band: f64) -> Result<bool, String> {
    let chord = end.sub(start);
    let length = chord.length();
    if length <= band {
        return Ok(false);
    }
    let direction = chord.scale(1.0 / length);
    for fraction in [0.25, 0.5, 0.75] {
        let point = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)?;
        let delta = point.sub(start);
        let along = delta.dot(direction);
        let off = delta.sub(direction.scale(along)).length();
        if off > band || along < -band || along > length + band {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Ear-clip a simple polygon given as 2D points in loop order. The triangles
/// come back as index triples in the polygon's own orientation (clockwise or
/// counter-clockwise, whichever the input is), so a caller emitting boundary
/// sides in triple order reproduces the loop's traversal direction. Collinear
/// vertices are never ear tips (a zero-area triangle is not a face), and a
/// polygon with no ear at all — self-intersecting, or fully collinear — yields
/// `None`.
pub(in crate::step_import) fn ear_clip(polygon: &[[f64; 2]]) -> Option<Vec<[usize; 3]>> {
    let count = polygon.len();
    if count < 3 {
        return None;
    }
    let signed_area: f64 = (0..count)
        .map(|index| {
            let a = polygon[index];
            let b = polygon[(index + 1) % count];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    if signed_area == 0.0 {
        return None;
    }
    let orientation = signed_area.signum();
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> f64 {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    // Scale-relative floor for "zero area": below this a triangle is a line.
    let extent = polygon
        .iter()
        .fold(0.0f64, |max, point| max.max(point[0].abs()).max(point[1].abs()))
        .max(1e-300);
    let area_floor = extent * extent * 1e-14;
    let inside_or_on = |p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]| -> bool {
        let s0 = cross(a, b, p) * orientation;
        let s1 = cross(b, c, p) * orientation;
        let s2 = cross(c, a, p) * orientation;
        s0 >= -area_floor && s1 >= -area_floor && s2 >= -area_floor
    };
    let mut indices: Vec<usize> = (0..count).collect();
    let mut triangles = Vec::with_capacity(count - 2);
    while indices.len() > 3 {
        let remaining = indices.len();
        let mut clipped = false;
        for offset in 0..remaining {
            let previous = indices[(offset + remaining - 1) % remaining];
            let current = indices[offset];
            let next = indices[(offset + 1) % remaining];
            let (a, b, c) = (polygon[previous], polygon[current], polygon[next]);
            // Convex (in the polygon's orientation) with real area.
            if cross(a, b, c) * orientation <= area_floor {
                continue;
            }
            let blocked = indices.iter().any(|&other| {
                other != previous
                    && other != current
                    && other != next
                    && inside_or_on(polygon[other], a, b, c)
            });
            if blocked {
                continue;
            }
            triangles.push([previous, current, next]);
            indices.remove(offset);
            clipped = true;
            break;
        }
        if !clipped {
            return None;
        }
    }
    let (a, b, c) = (polygon[indices[0]], polygon[indices[1]], polygon[indices[2]]);
    if cross(a, b, c) * orientation <= area_floor {
        return None;
    }
    triangles.push([indices[0], indices[1], indices[2]]);
    Some(triangles)
}
