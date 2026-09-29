use super::*;

impl<'a> SolidBuilder<'a> {
    /// Pre-pass over all faces: re-seat OCC full-cylinder/cone rim circles onto
    /// the surface seam meridian.
    ///
    /// OCC represents a full periodic wall as TWO rim loops, each a single full
    /// CIRCLE edge with no seam ruling. When the two rim circles' seam vertices
    /// sit at DIFFERENT azimuths (e.g. 0° vs 180°), one circle crosses the
    /// surface u-seam and its loop cannot close in parameter space — the aligned
    /// `stitch_seam_circle_loops` cannot merge them, so the body fails to import.
    ///
    /// The fix (standard periodic-surface loop handling): split each off-seam
    /// rim circle at the seam meridian — equivalently, re-seat its start/end
    /// vertex onto the u=0 iso-meridian and rebuild the edge as the surface's
    /// v-iso circle through it. Because a full circle meets the seam half-plane
    /// exactly once, this "split" is a single edge whose coincident endpoints
    /// now lie on the seam. With both rims on the seam, the pair reduces to the
    /// already-working aligned case and `stitch_seam_circle_loops` merges it.
    ///
    /// Run as a global pre-pass (before any pcurve is derived) because a rim
    /// circle is shared with its planar cap, and caps can precede walls in the
    /// shell — re-seating in place mid-wall would leave the cap's pcurve stale.
    pub(super) fn relocate_periodic_rim_seams(&mut self, faces: &[PendingFace]) -> Result<(), String> {
        for face in faces {
            self.relocate_face_rim_seams(face)?;
        }
        Ok(())
    }

    /// Pre-pass over all faces: split any OPEN edge that CROSSES a periodic
    /// surface's seam (full rim circles are re-seated by the pass above).
    ///
    /// A point-by-point projected pcurve for a seam-crossing arc jumps the
    /// branch cut mid-span, so the fitted trim folds onto the wrong side of
    /// the domain and the loop cannot close (OCC writes such arcs freely —
    /// STEP does not require boundary edges to respect a carrier's seam).
    /// Splitting the EDGE at the seam meridian gives each piece a clean
    /// in-domain pcurve; the loop then closes across the seam through the
    /// existing periodic connection.
    ///
    /// The split is GLOBAL — the edge store and every face's bound specs —
    /// because the edge is shared with an adjacent face, which must reference
    /// the same two pieces or the shell's edge-use counts break.
    pub(super) fn split_seam_crossing_edges(&mut self, faces: &mut [PendingFace]) -> Result<(), String> {
        loop {
            let mut split = None;
            'search: for face in faces.iter() {
                let surface = &face.surface;
                let (closed_u, closed_v) = surface.closed_directions()?;
                if !closed_u && !closed_v {
                    continue;
                }
                for (specs, _) in &face.bounds {
                    for (edge_id, _) in specs {
                        let edge = self.edge_record(*edge_id);
                        // Anchored rim circles never cross (the relocation
                        // pre-pass seats them ON the seam), but a closed edge
                        // shared by TWO periodic walls whose seam meridians
                        // differ cannot be anchored for both — the un-anchored
                        // wall sees a mid-span crossing and the edge must
                        // split exactly like an open arc (problemInbox
                        // 2026-08-05T01-04: fillet-torus rim 180° out of
                        // phase with its cylinder neighbour folded its pcurve).
                        if edge.degenerate {
                            continue;
                        }
                        if closed_u {
                            if let Some(t) = seam_crossing_parameter(surface, edge, true)? {
                                split = Some((*edge_id, t));
                                break 'search;
                            }
                        }
                        if closed_v {
                            if let Some(t) = seam_crossing_parameter(surface, edge, false)? {
                                split = Some((*edge_id, t));
                                break 'search;
                            }
                        }
                    }
                }
            }
            let Some((edge_id, t)) = split else {
                break;
            };
            self.split_edge_at(edge_id, t, faces)?;
        }
        Ok(())
    }

    /// Pre-pass over all faces: pull edges whose 3D curves have drifted OFF a
    /// carrier surface back onto it.
    ///
    /// OCC SURFACE_CURVEs declare `.PCURVE_S1.` as the MASTER representation:
    /// the written 3D curve is an approximation within OCC's edge tolerance
    /// (near 1e-2 on real exports), while the true edge is the pcurve mapped
    /// through the surface. Our model keeps exact 3D edges, so re-derive the
    /// 3D curve by projecting it onto the surface it should ride. Endpoints
    /// stay EXACTLY at the existing vertices (vendor vertices are accurate;
    /// the drift is mid-span) so shared topology and loop chaining are
    /// untouched. Runs before any pcurve is fitted, so every face fits
    /// against the corrected curve.
    ///
    /// **"Off the carrier" is read off the trim the face will be given, not
    /// off the closest-point projector.** A projector answer is SOME point of
    /// the surface, so its distance is an upper bound: a pass is conclusive and
    /// a fail is not. On a general patch nearly closed in one direction it
    /// returns the far end's local minimum — on `abc_00000026`'s thread flanks
    /// at alternate stations of an edge lying on the flank to 1e-13, and along
    /// the whole of another, reading 2.2e-2 .. 4.2e-2 for an edge that is on
    /// its carrier. This pass used to believe that reading and interpolate a
    /// replacement through those feet, which moved 89 edges by up to 4.4e-2
    /// off the carriers the file had put them on and produced every one of
    /// that file's 73 loop self-crossings and 16 face-pair crossings.
    /// `build_loop` already decides the branch correctly — jointly around the
    /// loop where the carrier aliases (`joint_branch_pcurves`), per coedge
    /// with the fitter's jump repair elsewhere — so a screened edge is measured
    /// against exactly that trim's image, and a reconciled edge rides that
    /// image: a continuous branch, not a sequence of independent projector
    /// answers.
    pub(super) fn reconcile_edges_onto_surfaces(&mut self, faces: &[PendingFace]) -> Result<(), String> {
        let scale = 1.0 + solid_like_scale(&self.vertices);
        // Reconcile anything that would trip the validator's pcurve band —
        // the band has an ABSOLUTE floor (4e-3), so a scale-proportional
        // screen misses exactly the vendor near-misses on large (BIM-sized)
        // parts.
        let band =
            0.5 * crate::KernelTolerances::for_scale(scale, 1e-7).pcurve_consistency;
        // In-band curves keep their exact representation; wildly off curves (a
        // mismatched carrier) are left alone too — forcing those onto the
        // surface would corrupt geometry that a later stage may still refuse
        // honestly. The upper guard is scale-proportional: vendor 3D
        // approximations reach ~1e-3 of model size (PCURVE_S1 master
        // semantics), anything beyond that is a wrong carrier, not drift.
        let upper_rel = std::env::var("BREP_RECONCILE_MAX")
            .ok()
            .and_then(|s| s.parse::<f64>().ok())
            .unwrap_or(1e-3);
        const SAMPLES: usize = 32;
        let mut replaced = 0usize;
        for face in faces {
            let surface = &face.surface;
            for (specs, _) in &face.bounds {
                // Cheap screen first: most edges ride their surfaces exactly,
                // and the full sweep on a high-degree carrier is what dominates
                // import time. The projector's distance never under-reads, so
                // an edge that passes here is on the carrier.
                let mut screened = Vec::new();
                for (index, (edge_id, _)) in specs.iter().enumerate() {
                    let edge = self.edge_record(*edge_id);
                    if edge.degenerate {
                        continue;
                    }
                    let mut screen_worst = 0.0f64;
                    for station in 0..=4 {
                        let t = edge.t0 + (edge.t1 - edge.t0) * (0.1 + 0.2 * station as f64);
                        let point = edge.curve.evaluate(t)?;
                        let projection = crate::project_point_to_surface(surface, point)?;
                        let on_surface = surface.evaluate(projection.u, projection.v)?;
                        screen_worst = screen_worst.max(on_surface.sub(point).length());
                    }
                    if screen_worst > band {
                        screened.push(index);
                    }
                }
                if screened.is_empty() {
                    continue;
                }
                let trims = self.derived_loop_trims(surface, specs, &screened)?;
                for index in screened {
                    let (edge_id, forward) = specs[index];
                    let Some(trim) = trims[index].as_ref() else {
                        continue;
                    };
                    let edge = self.edge_record(edge_id).clone();
                    let [p0, p1] = trim.domain()?;
                    let mut worst = 0.0f64;
                    let mut projected = Vec::with_capacity(SAMPLES + 1);
                    let mut parameters = Vec::with_capacity(SAMPLES + 1);
                    for station in 0..=SAMPLES {
                        let along = station as f64 / SAMPLES as f64;
                        let t = edge.t0 + (edge.t1 - edge.t0) * along;
                        let point = edge.curve.evaluate(t)?;
                        // A trim runs in its COEDGE's direction.
                        let fraction = if forward { along } else { 1.0 - along };
                        let uv = trim.evaluate(p0 + (p1 - p0) * fraction)?;
                        let on_surface = surface.evaluate(uv.x, uv.y)?;
                        worst = worst.max(on_surface.sub(point).length());
                        projected.push(on_surface);
                        parameters.push(t);
                    }
                    if worst <= band || worst > upper_rel * scale {
                        continue;
                    }
                    // The replacement rides the surface at every interior
                    // sample; only the endpoints are forced back onto the
                    // existing vertices (vendor vertices are true corner
                    // intersections, so that correction is tiny and confined
                    // to the first/last spans). A partial blend is NOT an
                    // option: any residual off-surface stretch gives the
                    // pcurve fitter an unrepresentable target and its
                    // refinement degenerates into max-insertion every round.
                    let mut reconciled = projected;
                    reconciled[0] = edge.curve.evaluate(edge.t0)?;
                    reconciled[SAMPLES] = edge.curve.evaluate(edge.t1)?;
                    let replacement = crate::interpolate_curve(&reconciled, 3, &parameters)?;
                    if let Some(record) =
                        self.edges.iter_mut().find(|record| record.id == edge_id)
                    {
                        record.curve = replacement;
                        [record.t0, record.t1] = [parameters[0], parameters[SAMPLES]];
                        replaced += 1;
                    }
                    self.capture_stage(edge_id, || {
                        format!(
                            "reconciled onto face #{} (surface #{}): its trim reads {worst:.3e} off it",
                            face.face_ref, face.surface_ref
                        )
                    });
                }
            }
        }
        if std::env::var("BREP_DEBUG_STEP_LOOP").is_ok() {
            eprintln!("RECONCILE replaced {replaced} edge curves (band {band:.6})");
        }
        Ok(())
    }

    /// The trims `build_loop` derives for one loop, for the coedges in
    /// `wanted`: the joint branch assignment where the loop takes it, the
    /// per-coedge fit otherwise. Neither the supplied-pcurve lane nor seam
    /// pinning is repeated — the first is off by default and the second moves
    /// a trim along its own seam, not off it.
    ///
    /// The per-coedge fit is also the lane that lands on the wrong end of a
    /// nearly closed patch (edge 438 of `abc_00000026` reads 2.163e-2 through
    /// it). Where `joint_branch_pcurves` declines a loop on such a carrier,
    /// this pass inherits that answer and can move an edge that is on its
    /// carrier; `abc_00000026` escapes only because all 48 of its aliased loops
    /// are adopted.
    fn derived_loop_trims(
        &self,
        surface: &NurbsSurface,
        specs: &[(u64, bool)],
        wanted: &[usize],
    ) -> Result<Vec<Option<NurbsCurve>>, String> {
        let pcurve_tol = loop_pcurve_tolerance(surface)?;
        if let Some(joint) = self.joint_branch_pcurves(surface, specs, pcurve_tol)? {
            return Ok(joint.into_iter().map(Some).collect());
        }
        let mut trims = vec![None; specs.len()];
        for &index in wanted {
            let (edge_id, forward) = specs[index];
            let edge = self.edge_record(edge_id);
            trims[index] = Some(build_pcurve_on_surface_range(
                surface,
                &edge.curve,
                edge.t0,
                edge.t1,
                forward,
                pcurve_tol,
            )?);
        }
        Ok(trims)
    }

    /// Split one kernel edge at parameter `t` into two pieces sharing a new
    /// vertex, replacing every face's bound references so traversal order is
    /// preserved on both sides.
    fn split_edge_at(
        &mut self,
        edge_id: u64,
        t: f64,
        faces: &mut [PendingFace],
    ) -> Result<(), String> {
        let index = self
            .edges
            .iter()
            .position(|edge| edge.id == edge_id)
            .ok_or_else(|| format!("step_import: split target edge {edge_id} vanished"))?;
        let edge = self.edges[index].clone();
        let mid_point = edge.curve.evaluate(t)?;
        let mid_vertex = self.fresh();
        self.push_vertex(VertexRecord {
            id: mid_vertex,
            point: mid_point,
        });
        let (left, right) = edge.curve.split(t)?;
        let [left_t0, left_t1] = left.domain()?;
        let [right_t0, right_t1] = right.domain()?;
        let left_id = self.fresh();
        let right_id = self.fresh();
        self.edges[index] = EdgeRecord {
            id: left_id,
            curve: left,
            t0: left_t0,
            t1: left_t1,
            start_vertex_id: edge.start_vertex_id,
            end_vertex_id: mid_vertex,
            degenerate: false,
            name: edge.name.clone(),
        };
        self.edges.insert(
            index + 1,
            EdgeRecord {
                id: right_id,
                curve: right,
                t0: right_t0,
                t1: right_t1,
                start_vertex_id: mid_vertex,
                end_vertex_id: edge.end_vertex_id,
                degenerate: false,
                name: edge.name,
            },
        );
        // The in-place reassign above changed the id at `index` and this insert
        // shifted every following edge's position, so rebuild the id -> index
        // map. Splits are bounded by seam-crossing edges (rare), not O(E).
        self.rebuild_edge_index();
        self.capture_split(edge_id, t, left_id, right_id);
        for face in faces.iter_mut() {
            for (specs, _) in &mut face.bounds {
                let mut rebuilt = Vec::with_capacity(specs.len() + 1);
                for (id, forward) in specs.iter() {
                    if *id == edge_id {
                        if *forward {
                            rebuilt.push((left_id, true));
                            rebuilt.push((right_id, true));
                        } else {
                            rebuilt.push((right_id, false));
                            rebuilt.push((left_id, false));
                        }
                    } else {
                        rebuilt.push((*id, *forward));
                    }
                }
                *specs = rebuilt;
            }
        }
        // Drop the stale STEP-ref cache entry so nothing resolves the old id.
        self.edge_of_ref.retain(|_, id| *id != edge_id);
        Ok(())
    }

    /// Re-seat the two rim circles of one candidate wall face (see
    /// [`relocate_periodic_rim_seams`]). A no-op unless the face is a genuine
    /// full-periodic cylindrical/conical wall (closed surface, exactly two
    /// single full-circle rim loops) with at least one rim off the seam.
    fn relocate_face_rim_seams(&mut self, face: &PendingFace) -> Result<(), String> {
        let surface = &face.surface;
        let (closed_u, closed_v) = surface.closed_directions()?;
        // OCC full walls are u-periodic (angle = u, generatrix = v); the rim
        // circles are v-iso curves. A v-only-periodic wall does not occur in the
        // supported inputs, so leave it to the existing paths.
        if !closed_u || closed_v {
            return Ok(());
        }
        let [u0, u1] = surface.domain_u()?;

        // Rim loops: a single coedge whose edge is a non-degenerate CLOSED curve
        // (a cross-section circle that wraps the periodic u direction).
        let rims: Vec<u64> = face
            .bounds
            .iter()
            .filter_map(|(specs, _)| {
                if specs.len() != 1 {
                    return None;
                }
                let edge = self.edge_record(specs[0].0);
                (!edge.degenerate && edge.start_vertex_id == edge.end_vertex_id)
                    .then_some(specs[0].0)
            })
            .collect();
        let has_vertex_loop = face
            .bounds
            .iter()
            .any(|(specs, _)| specs.len() == 1 && self.edge_record(specs[0].0).degenerate);
        let pointed_cone = rims.len() == 1 && has_vertex_loop;
        if rims.len() != 2 && !pointed_cone {
            // Handle ordinary two-rim walls and pointed cones represented by
            // one circular rim plus a VERTEX_LOOP at the apex. Other layouts
            // fall through to the general periodic paths.
            if std::env::var("BREP_DEBUG_RIMS").is_ok() {
                eprintln!("RIMS bail: rims={} pointed={}", rims.len(), pointed_cone);
                for (specs, _) in &face.bounds {
                    if specs.len() != 1 {
                        continue;
                    }
                    let edge = self.edge_record(specs[0].0);
                    let gap = self
                        .vertex_point(edge.start_vertex_id)
                        .sub(self.vertex_point(edge.end_vertex_id))
                        .length();
                    eprintln!(
                        "   single-coedge loop edge {}: start={} end={} gap={gap:.3e} degen={}",
                        edge.id, edge.start_vertex_id, edge.end_vertex_id, edge.degenerate
                    );
                }
            }
            return Ok(());
        }

        // Project each rim's seam vertex to find its (u, v): u tells us whether
        // the rim already sits on the seam (u≈u0 or u≈u1, the same meridian on a
        // periodic surface); v is the rim's axial station for the iso circle.
        let on_seam_tol = 0.05 * (u1 - u0).abs().max(1e-9);
        let [v_lo, v_hi] = surface.domain_v()?;
        // (edge_id, Some(v_rim) = true iso rim to rebuild | None = general
        // closed girdling curve to rotate onto the seam, geometry preserved)
        let mut plan: Vec<(u64, Option<f64>)> = Vec::new();
        let mut any_off_seam = false;
        for &edge_id in &rims {
            let vertex_id = self.edge_record(edge_id).start_vertex_id;
            let point = self.vertex_point(vertex_id);
            let projection = crate::project_point_to_surface(surface, point)?;
            let on_seam = (projection.u - u0).abs() <= on_seam_tol
                || (projection.u - u1).abs() <= on_seam_tol;
            if !on_seam {
                any_off_seam = true;
                // Guard: only re-seat a rim vertex that belongs solely to this
                // one circle. If another edge shares it, moving it would break
                // that edge — leave the whole pair for the clear error path.
                if self.vertex_incident_edge_count(vertex_id) != 1 {
                    if std::env::var("BREP_DEBUG_RIMS").is_ok() {
                        eprintln!(
                            "RIMS bail: shared vertex {} ({} incident edges)",
                            vertex_id,
                            self.vertex_incident_edge_count(vertex_id)
                        );
                    }
                    return Ok(());
                }
                // A rim may only be REBUILT as the wall's v-iso circle when it
                // actually IS one — constant v along its whole span. A closed
                // girdling curve at VARYING v (e.g. the saddle where a drilled
                // hole meets a curved face, exported as a closed B-spline) has
                // its geometry preserved: it is rotated so its seam vertex
                // lands on the wall's seam meridian instead (problemInbox
                // 2026-08-05T01-04: the reseat flattened such saddles into
                // circles, leaving edges up to 0.09 model-units off-surface).
                let mut v_min = f64::INFINITY;
                let mut v_max = f64::NEG_INFINITY;
                let edge = self.edge_record(edge_id).clone();
                for index in 0..=16 {
                    let t = edge.t0 + (edge.t1 - edge.t0) * index as f64 / 16.0;
                    let sample = crate::project_point_to_surface(surface, edge.curve.evaluate(t)?)?;
                    v_min = v_min.min(sample.v);
                    v_max = v_max.max(sample.v);
                }
                let iso = (v_max - v_min) <= 0.02 * (v_hi - v_lo).abs().max(1e-9);
                plan.push((edge_id, iso.then_some(projection.v)));
            }
        }
        if !any_off_seam {
            // Both rims already on the seam: the aligned stitch handles it.
            return Ok(());
        }

        for (edge_id, v_rim) in plan {
            match v_rim {
                Some(v_rim) => self.reseat_rim_circle_to_seam(surface, edge_id, v_rim, u0, u1)?,
                None => {
                    self.rotate_closed_edge_to_seam(surface, edge_id)?;
                }
            }
        }
        Ok(())
    }

    /// Rotate a CLOSED girdling edge's parameterization so its seam vertex
    /// lands on the wall's seam meridian — geometry untouched (split at the
    /// seam crossing, swap the halves, C0-join at the old vertex azimuth).
    /// Returns false when the edge never crosses the seam (nothing to do).
    fn rotate_closed_edge_to_seam(
        &mut self,
        surface: &NurbsSurface,
        edge_id: u64,
    ) -> Result<bool, String> {
        let edge = self.edge_record(edge_id).clone();
        let Some(t_cross) = seam_crossing_parameter(surface, &edge, true)? else {
            return Ok(false);
        };
        let margin = 1e-6 * (edge.t1 - edge.t0).abs();
        if t_cross <= edge.t0 + margin || t_cross >= edge.t1 - margin {
            return Ok(false); // already anchored at the seam
        }
        let (first, second) = edge.curve.split(t_cross)?;
        let rotated = concatenate_curves_c0(&second, &first)?;
        let [t0, t1] = rotated.domain()?;
        let seam_point = rotated.evaluate(t0)?;
        let vertex_id = edge.start_vertex_id;
        if let Some(vertex) = self
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == vertex_id)
        {
            vertex.point = seam_point;
        }
        if let Some(record) = self.edges.iter_mut().find(|record| record.id == edge_id) {
            record.curve = rotated;
            record.t0 = t0;
            record.t1 = t1;
        }
        self.capture_stage(edge_id, || "rotated so its vertex sits on the seam".into());
        Ok(true)
    }

    /// Rebuild one rim circle edge as the surface's v-iso circle at `v_rim`
    /// (which starts/ends on the u-seam meridian), preserving the original
    /// edge's rotational sense, and move its shared start/end vertex onto that
    /// seam point. Leaves the edge a single closed circle — its coincident
    /// endpoints just now lie on the seam instead of at an interior azimuth.
    fn reseat_rim_circle_to_seam(
        &mut self,
        surface: &NurbsSurface,
        edge_id: u64,
        v_rim: f64,
        u0: f64,
        u1: f64,
    ) -> Result<(), String> {
        let iso = surface.iso_curve_v(v_rim)?;
        // Match the rebuilt circle's traversal direction to the original edge so
        // the two coedges sharing it (wall + cap) keep their STEP senses and
        // both faces retain their correct loop winding.
        let (edge_curve, edge_t0, edge_t1) = {
            let edge = self.edge_record(edge_id);
            (edge.curve.clone(), edge.t0, edge.t1)
        };
        let forward_in_u =
            original_u_direction(surface, &edge_curve, edge_t0, edge_t1, u0, u1)? >= 0.0;
        let new_curve = if forward_in_u { iso } else { iso.reversed()? };
        let [t0, t1] = new_curve.domain()?;
        let seam_point = new_curve.evaluate(t0)?;

        let vertex_id = self.edge_record(edge_id).start_vertex_id;
        if let Some(vertex) = self
            .vertices
            .iter_mut()
            .find(|vertex| vertex.id == vertex_id)
        {
            vertex.point = seam_point;
        }
        if let Some(edge) = self.edges.iter_mut().find(|edge| edge.id == edge_id) {
            edge.curve = new_curve;
            edge.t0 = t0;
            edge.t1 = t1;
        }
        self.capture_stage(edge_id, || format!("rim re-seated as the iso circle v = {v_rim:.9}"));
        Ok(())
    }

    /// Number of distinct edges that reference `vertex_id` as a start or end.
    fn vertex_incident_edge_count(&self, vertex_id: u64) -> usize {
        self.edges
            .iter()
            .filter(|edge| edge.start_vertex_id == vertex_id || edge.end_vertex_id == vertex_id)
            .count()
    }
}
