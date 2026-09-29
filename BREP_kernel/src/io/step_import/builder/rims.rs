use super::*;
use super::solids::RULING_INTERIOR_STATIONS;

impl<'a> SolidBuilder<'a> {
    /// Recognise a loop that WRAPS the surface's periodic direction at a
    /// constant iso value — a rim of a full periodic wall. OCC may emit such a
    /// rim as ONE closed circle, or SPLIT it into several arcs where the wall
    /// meets neighbouring features (ABC ad34a3f family: cylinder/torus rims cut
    /// at u=0 and u=0.5). Returns the loop's coedges rotated to START at a
    /// vertex sitting on the seam meridian, plus that vertex, so the caller can
    /// join two rims with a single seam ruling. Returns None for any loop that
    /// does not cleanly wrap the periodic direction with a seam-meridian vertex.
    fn wrapping_rim(
        &self,
        surface: &NurbsSurface,
        specs: &[(u64, bool)],
        closed_u: bool,
        closed_v: bool,
    ) -> Result<Option<(Vec<(u64, bool)>, u64)>, String> {
        if specs.is_empty() {
            return Ok(None);
        }
        // A single edge can only bound a rim if it is a real CLOSED wire.
        // Reject an open or degenerate single edge outright. A CLOSED single
        // edge is the classic rim circle — but only when it actually WRAPS the
        // periodic direction. The historical fast path accepted ANY single
        // closed edge verbatim; a full periodic wall can also carry a single
        // closed edge that is an INTERIOR hole (a drilled notch on a cylinder
        // wall: closed, but net winding ~0, not ±period). Such a phantom rim
        // made `stitch_seam_circle_loops` see three rims on a two-rim wall, bail
        // out, and leave the cylinder unseamed — its parameter domain never
        // closed, so the tessellator filled only thin bands at the two real
        // rims and dropped the whole cylinder body (ABC 00000290: a 0.22-long
        // rod rendered as two disconnected end caps; ~73% of the surface area
        // and 60% of the volume missing from the mesh). So a single closed edge
        // now runs the same winding test as a multi-arc rim below; when it
        // passes it is returned with its own vertex (byte-identical to the old
        // fast path), and only a non-wrapping notch is newly rejected.
        let single_rim_vertex = if specs.len() == 1 {
            let edge = self.edge_record(specs[0].0);
            if edge.degenerate || edge.start_vertex_id != edge.end_vertex_id {
                return Ok(None);
            }
            Some(edge.start_vertex_id)
        } else {
            None
        };
        // Every edge real, and the coedges must chain (in the given order) into
        // a closed wire.
        let mut directed_start: Vec<u64> = Vec::with_capacity(specs.len());
        let mut directed_end: Vec<u64> = Vec::with_capacity(specs.len());
        for (edge_id, forward) in specs {
            let edge = self.edge_record(*edge_id);
            if edge.degenerate {
                return Ok(None);
            }
            let (s, e) = if *forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            };
            directed_start.push(s);
            directed_end.push(e);
        }
        for i in 0..specs.len() {
            if directed_end[i] != directed_start[(i + 1) % specs.len()] {
                return Ok(None);
            }
        }
        // Sample the loop densely (directed) and project to surface parameters.
        let [u0, u1] = surface.domain_u()?;
        let [v0, v1] = surface.domain_v()?;
        let u_span = u1 - u0;
        let v_span = v1 - v0;
        let mut us: Vec<f64> = Vec::new();
        let mut vs: Vec<f64> = Vec::new();
        let mut vertex_uv: Vec<(usize, f64, f64)> = Vec::new();
        for (index, (edge_id, forward)) in specs.iter().enumerate() {
            let edge = self.edge_record(*edge_id);
            for step in 0..4 {
                let frac = step as f64 / 4.0;
                let directed = if *forward { frac } else { 1.0 - frac };
                let t = edge.t0 + (edge.t1 - edge.t0) * directed;
                let point = edge.curve.evaluate(t)?;
                let pr = crate::project_point_to_surface(surface, point)?;
                us.push(pr.u);
                vs.push(pr.v);
                if step == 0 {
                    vertex_uv.push((index, pr.u, pr.v));
                }
            }
        }
        let wrapped = |d: f64, span: f64| -> f64 {
            if span <= 0.0 {
                d
            } else {
                d - (d / span).round() * span
            }
        };
        let net_winding = |params: &[f64], span: f64| -> f64 {
            let mut total = 0.0;
            for i in 0..params.len() {
                total += wrapped(params[(i + 1) % params.len()] - params[i], span);
            }
            total
        };
        let spread = |params: &[f64], span: f64| -> f64 {
            let base = params[0];
            let (mut lo, mut hi) = (0.0f64, 0.0f64);
            for &p in params {
                let d = wrapped(p - base, span);
                lo = lo.min(d);
                hi = hi.max(d);
            }
            hi - lo
        };
        // Pick the coedge whose START vertex lies closest to the seam meridian
        // (u=u0≡u1 for a u-wrap, v=v0≡v1 for a v-wrap); the rim can only be
        // seam-ruled there. Require it within 5% of the period of the seam.
        let seam_start = |on_u: bool| -> Option<(usize, u64)> {
            let (span, lo, hi) = if on_u { (u_span, u0, u1) } else { (v_span, v0, v1) };
            let tol = 0.05 * span.abs().max(1e-12);
            vertex_uv
                .iter()
                .map(|&(index, u, v)| {
                    let value = if on_u { u } else { v };
                    let dist = (value - lo).abs().min((value - hi).abs());
                    (index, dist)
                })
                .filter(|&(_, dist)| dist <= tol)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(index, _)| (index, directed_start[index]))
        };

        // u-wrap: constant v, one full turn in u.
        if closed_u
            && u_span > 0.0
            && (net_winding(&us, u_span).abs() - u_span).abs() <= 0.15 * u_span
            && spread(&vs, if closed_v { v_span } else { f64::INFINITY })
                <= 0.05 * v_span.abs().max(1.0)
        {
            // A single closed rim circle keeps its own vertex (the historical
            // fast path never required seam proximity); only multi-arc rims
            // rotate to start on the seam meridian.
            if let Some(vertex) = single_rim_vertex {
                return Ok(Some((specs.to_vec(), vertex)));
            }
            if let Some((idx, seam_vertex)) = seam_start(true) {
                let mut rotated = specs.to_vec();
                rotated.rotate_left(idx);
                return Ok(Some((rotated, seam_vertex)));
            }
        }
        // v-wrap: constant u, one full turn in v.
        if closed_v
            && v_span > 0.0
            && (net_winding(&vs, v_span).abs() - v_span).abs() <= 0.15 * v_span
            && spread(&us, if closed_u { u_span } else { f64::INFINITY })
                <= 0.05 * u_span.abs().max(1.0)
        {
            if let Some(vertex) = single_rim_vertex {
                return Ok(Some((specs.to_vec(), vertex)));
            }
            if let Some((idx, seam_vertex)) = seam_start(false) {
                let mut rotated = specs.to_vec();
                rotated.rotate_left(idx);
                return Ok(Some((rotated, seam_vertex)));
            }
        }
        Ok(None)
    }

    /// Merge the two full-circle rim loops of an OCC-style full cylinder/cone
    /// wall (each a single closed edge that wraps the surface's periodic
    /// direction) into one loop joined by a synthesized seam ruling — the loop
    /// structure the kernel builders use — so the face closes in parameter space.
    pub(super) fn stitch_seam_circle_loops(
        &mut self,
        surface: &NurbsSurface,
        bounds: &mut Vec<(Vec<(u64, bool)>, bool)>,
    ) -> Result<(), String> {
        let (closed_u, closed_v) = surface.closed_directions()?;
        if !(closed_u || closed_v) {
            return Ok(());
        }
        // BI-PERIODIC carriers (tori) are left UNSTITCHED: their two rims bound
        // two valid regions (the between-rims band and its cross-seam
        // complement), a choice the watertight tessellator and the mass
        // integrator each resolve from the rim senses (see
        // `watertight_tessellation::close_periodic_trim_dir` and
        // `mass_properties::biperiodic_band_range`). Forging a seam here would
        // pin one region unconditionally — wrong for every fillet torus whose
        // material lies across the seam. Only SINGLY-periodic walls
        // (cylinder/cone/sphere), where just the between-rims band exists, are
        // seam-ruled below.
        if closed_u && closed_v {
            return Ok(());
        }
        // Rim loops: any loop that WRAPS the periodic direction at a constant
        // iso value — whether OCC emitted it as one closed circle or split it
        // into several arcs at the seam and interior junctions. Each entry is
        // (bound index, coedges rotated to start on the seam meridian, that
        // seam vertex).
        let mut rim_infos: Vec<(usize, Vec<(u64, bool)>, u64)> = Vec::new();
        for (index, (specs, _)) in bounds.iter().enumerate() {
            if let Some((rotated, seam_vertex)) =
                self.wrapping_rim(surface, specs, closed_u, closed_v)?
            {
                rim_infos.push((index, rotated, seam_vertex));
            }
        }
        // Pointed cones (a countersink drilled to a point, a VERTEX_LOOP
        // apex, or a vendor zero-length apex edge): ONE rim plus ONE
        // degenerate apex loop merges exactly like the two-rim wall — the
        // seam ruling runs down the generatrix to the apex. Left unmerged,
        // the periodic band has no seam sides and its triangulation sprays
        // across the domain (boxy_with_diamsize countersinks).
        let apexes: Vec<usize> = bounds
            .iter()
            .enumerate()
            .filter(|(_, (specs, _))| specs.len() == 1 && self.edge_record(specs[0].0).degenerate)
            .map(|(index, _)| index)
            .collect();
        // (index1, specs1, v1) and (index2, specs2, v2): the two boundaries to
        // merge and the vertices where the seam ruling attaches.
        let (index1, specs1, v1, index2, specs2, v2) = if rim_infos.len() == 2 {
            let (i1, s1, a1) = rim_infos[0].clone();
            let (i2, s2, a2) = rim_infos[1].clone();
            (i1, s1, a1, i2, s2, a2)
        } else if rim_infos.len() == 1 && apexes.len() == 1 {
            let (i1, s1, a1) = rim_infos[0].clone();
            let apex_index = apexes[0];
            let apex_edge = bounds[apex_index].0[0];
            let apex_vertex = self.edge_record(apex_edge.0).start_vertex_id;
            (i1, s1, a1, apex_index, vec![apex_edge], apex_vertex)
        } else if apexes.len() == 2 {
            // Both v-ends of a closed surface-of-revolution collapse to POLES: a
            // profile that runs pole-to-pole (a lens / sphere-like blob), often
            // punched by interior holes, arrives as two VERTEX_LOOP apexes. The
            // holes read as extra "rim" loops here, so this is the apexes==2 case
            // whatever `rim_infos` counts. Left unmerged, the domain has no outer
            // boundary at all, so the winding integrator (and tessellator) take
            // the interior hole loops themselves as the material region — the
            // COMPLEMENT of the real face (ABC 00000013 face #179: the lens
            // surface, whose 3 drilled holes were filled and the lens body
            // dropped, inflating the solid volume ~17%). Stitch the two poles
            // with the seam meridian into ONE domain-spanning outer loop (the
            // make_sphere_brep pattern) so the material region becomes the full
            // closed surface minus its holes; the hole/rim loops are preserved.
            let a1 = apexes[0];
            let a2 = apexes[1];
            let e1 = bounds[a1].0[0];
            let e2 = bounds[a2].0[0];
            let vtx1 = self.edge_record(e1.0).start_vertex_id;
            let vtx2 = self.edge_record(e2.0).start_vertex_id;
            (a1, vec![e1], vtx1, a2, vec![e2], vtx2)
        } else {
            // Only the two-rim wall/band and the pointed cone are stitched
            // here; other periodic configurations fall through (and error
            // clearly if unclosable).
            if std::env::var("BREP_DEBUG_RIMS").is_ok() && !bounds.is_empty() {
                let shape: Vec<usize> = bounds.iter().map(|(s, _)| s.len()).collect();
                eprintln!("STITCH bail: rims={} apexes={} bounds={shape:?}", rim_infos.len(), apexes.len());
                for (specs, _) in bounds.iter() {
                    for (edge_id, forward) in specs {
                        let edge = self.edge_record(*edge_id);
                        eprintln!(
                            "    edge {} fwd={} sv={} ev={} degen={} closed={}",
                            edge_id, forward, edge.start_vertex_id, edge.end_vertex_id,
                            edge.degenerate, edge.start_vertex_id == edge.end_vertex_id
                        );
                    }
                }
            }
            return Ok(());
        };

        let p1 = self.vertex_point(v1);
        let p2 = self.vertex_point(v2);
        // The seam ruling must be a valid axis-parallel edge lying ON the
        // surface: its midpoint must be on the surface (the two rim vertices
        // share an azimuth). Otherwise this is not a simple two-rim wall — leave
        // it for the general seam handling rather than forge a diagonal chord.
        // Guard tolerance: a WRONG pairing forges a diagonal chord across the
        // face (~face-sized error); a right pairing sits within projection
        // noise of the generatrix. Vendor B-spline cones carry ~1e-4 noise
        // (boxy_with_diamsize countersinks missed the old 1e-6-scale gate by
        // 1.3x), so gate at the validator's pcurve band instead — still
        // orders of magnitude below any genuine diagonal.
        //
        // The absolute band alone is not enough on SUB-UNIT models: there the
        // pcurve band (0.5·pcurve_consistency ≈ 2e-3) dwarfs the whole part,
        // so a genuine diagonal chord slips through (onshape-unclamped-periodic
        // edges 165/80: midpoint 1.86e-4 off-surface across an 8.98e-4 ruling —
        // 21% of the ruling — was accepted, forging a false seam that shifted
        // the solid volume 4.32e-5 → 4.61e-5). A correct axis-parallel ruling's
        // straight midpoint deviates only by curvature/vendor noise, a tiny
        // FRACTION of the ruling length (boxy cones: ~2e-5 relative). So ALSO
        // require the deviation to stay under 2% of the ruling — orders of
        // magnitude below any diagonal, orders above any legitimate cone.
        let seam_len = p1.sub(p2).length();
        let on_surface_tol = (0.5
            * crate::KernelTolerances::for_scale(surface_scale(surface)?, 1e-7).pcurve_consistency)
            .min(0.02 * seam_len);
        // Check the whole ruling, not just its midpoint.  A curved periodic
        // B-spline profile can cross its endpoint chord exactly at mid-span
        // while bowing well off it in both adjacent spans; the old midpoint
        // test then accepted a straight synthetic seam that no pcurve on the
        // carrier could represent.  Interior eighth stations distinguish it
        // while leaving every genuinely straight cylinder/cone ruling alone.
        let straight_on_surface = ruling_is_on_surface_at_interior_stations(|fraction| {
            let point = p1.add(p2.sub(p1).scale(fraction));
            crate::project_point_to_surface(surface, point)
                .map(|projection| projection.distance <= on_surface_tol)
                .unwrap_or(false)
        });
        // The seam ruling connecting the two rim seam-vertices must LIE ON the
        // carrier. For a GENERAL NURBS carrier (including one recognized as an
        // arbitrary `AnalyticSurface::Revolution`), prefer its endpoint-validated
        // u-iso meridian before considering a chord: a high-frequency profile
        // can cross a fixed station grid at every sample while bowing between
        // them (ABC 00000565), whereas the iso curve rides the carrier exactly.
        // Recognized analytic carriers retain the straight cylinder/cone fast
        // path. A CURVED analytic generatrix fails the station test and falls
        // back to the same u-iso meridian. Two
        // curved cases arise on singly-periodic walls (bi-periodic tori are
        // skipped upstream): a SURFACE_OF_REVOLUTION with a spline/arc profile
        // (revolution_seam_ruling) and a spherical/analytic band whose rims sit
        // at different minor angles (curved_seam_ruling). Both helpers return
        // None when inapplicable, so try them in turn. Left unstitched, such a
        // wall's rim/apex loops never enclose the parameter domain and the face
        // integrates to zero area/volume (ABC 00000039 bands; the case-36 vase).
        let exact_general_ruling = if prefer_exact_general_ruling(surface.analytic()) {
            self.revolution_seam_ruling(surface, p1, p2)?
        } else {
            None
        };
        let seam_curve = if let Some(curve) = exact_general_ruling {
            curve
        } else if straight_on_surface {
            make_line(p1, p2)?
        } else if let Some(curve) = self.revolution_seam_ruling(surface, p1, p2)? {
            curve
        } else if let Some(arc) = curved_seam_ruling(surface, p1, p2)? {
            arc
        } else {
            if std::env::var("BREP_DEBUG_RIMS").is_ok() {
                let d = RULING_INTERIOR_STATIONS
                    .into_iter()
                    .filter_map(|fraction| {
                        let point = p1.add(p2.sub(p1).scale(fraction));
                        crate::project_point_to_surface(surface, point)
                            .ok()
                            .map(|projection| projection.distance)
                    })
                    .fold(0.0f64, f64::max);
                eprintln!(
                    "STITCH bail: ruling off-surface dist={d:.4e} p1=({:.3},{:.3},{:.3}) p2=({:.3},{:.3},{:.3})",
                    p1.x, p1.y, p1.z, p2.x, p2.y, p2.z
                );
            }
            return Ok(());
        };
        // Pin the ruling's endpoints EXACTLY onto the two rim/apex vertices.
        // An iso-meridian (revolution/curved branch) is trimmed to the vertices'
        // PROJECTED v-stations, so near a pole — where the rim seam-vertex sits a
        // few thousandths off the exact seam parameter — its endpoints can land a
        // fit tolerance (~1e-4) from the authoritative vertices, tripping the
        // topology validator's edge-start/end-vs-vertex check. The vertices are
        // the shared, authoritative corners; snap the curve to them (a clamped
        // NURBS interpolates its first/last control points, so this only nudges
        // the endpoint spans, staying well inside the pcurve band). Snap ONLY an
        // endpoint that genuinely missed: the band sits well under the validator
        // floor yet orders above the sub-micron noise of a ruling that already
        // meets its vertices, so a clean stitch is left byte-identical.
        let snap_tol = 1e-6 * (1.0 + surface_scale(surface)?);
        let seam_curve = snap_curve_endpoints(&seam_curve, p1, p2, snap_tol)?;
        let [t0, t1] = seam_curve.domain()?;
        let seam_id = self.fresh();
        self.push_edge(EdgeRecord {
            id: seam_id,
            curve: seam_curve,
            t0,
            t1,
            start_vertex_id: v1,
            end_vertex_id: v2,
            degenerate: false,
            name: None,
        });

        // Merged loop: rim1 → seam(v1→v2) → rim2 → seam(v2→v1). Each rim's
        // coedges are kept in order and orientation (their manifold pairing
        // with the neighbouring faces), rotated to start on the seam vertex;
        // the seam edge is used twice with opposite senses inside this loop.
        let mut merged: Vec<(u64, bool)> = specs1;
        merged.push((seam_id, true));
        merged.extend(specs2);
        merged.push((seam_id, false));
        let mut rebuilt: Vec<(Vec<(u64, bool)>, bool)> = Vec::new();
        rebuilt.push((merged, true));
        for (index, bound) in bounds.drain(..).enumerate() {
            if index != index1 && index != index2 {
                rebuilt.push(bound);
            }
        }
        *bounds = rebuilt;
        Ok(())
    }

    /// The generatrix ruling of a periodic surface-of-revolution wall from rim
    /// vertex `p1` to `p2`, as the surface's u = u0 seam meridian trimmed to the
    /// two vertices' axial (v) stations. Unlike a straight chord this rides the
    /// CURVED profile exactly, so its pcurve is a clean u-constant seam and the
    /// stitched loop encloses the parameter domain. Returns None (caller falls
    /// through) when the meridian does not actually connect the two vertices —
    /// they are not both on the seam, so this is not a simple revolution wall.
    fn revolution_seam_ruling(
        &self,
        surface: &NurbsSurface,
        p1: Vec3,
        p2: Vec3,
    ) -> Result<Option<NurbsCurve>, String> {
        // Never propagate a geometry error from here: an unusable meridian just
        // means "not a stitchable revolution wall", so fall through (Ok(None))
        // to the general seam handling exactly as the straight-chord bail did.
        //
        // Only a SINGLY-periodic wall (revolution of an OPEN profile: u = angle
        // closed, v = generatrix open) gets a meridian seam. A BI-periodic torus
        // fillet band (both directions closed) has two valid regions between its
        // rims and is unwrapped by the periodic-aware tessellator, NOT closed
        // with a seam edge — forging one there merges the loops and picks the
        // wrong sweep. Leave those exactly as the straight-chord path did (bail).
        let (closed_u, closed_v) = surface.closed_directions()?;
        if closed_u && closed_v {
            return Ok(None);
        }
        let [u0, _] = surface.domain_u()?;
        let (Ok(proj1), Ok(proj2)) = (
            crate::project_point_to_surface(surface, p1),
            crate::project_point_to_surface(surface, p2),
        ) else {
            return Ok(None);
        };
        let iso = match surface.iso_curve_u(u0) {
            Ok(curve) => curve,
            Err(_) => return Ok(None),
        };
        let [iso0, iso1] = iso.domain()?;
        let span = (iso1 - iso0).abs();
        if span <= 1e-12 {
            return Ok(None);
        }
        let v_lo = proj1.v.min(proj2.v).clamp(iso0, iso1);
        let v_hi = proj1.v.max(proj2.v).clamp(iso0, iso1);
        // Both rim vertices at the same axial station (or a collapsed
        // projection) is not a wall this stitches — leave it alone.
        if v_hi - v_lo <= 1e-6 * span {
            return Ok(None);
        }
        // Trim the seam meridian to the two rim stations. Split only at a
        // strictly interior parameter; a split at the domain edge would error,
        // so treat any failure as "not stitchable" and fall through.
        let span_tol = (1e-9 * span).max(crate::curve::KNOT_IDENTITY_TOL);
        let mut ruling = iso;
        if v_lo > iso0 + span_tol {
            ruling = match ruling.split(v_lo) {
                Ok((_, right)) => right,
                Err(_) => return Ok(None),
            };
        }
        let cur1 = ruling.domain()?[1];
        if v_hi < cur1 - span_tol {
            ruling = match ruling.split(v_hi) {
                Ok((left, _)) => left,
                Err(_) => return Ok(None),
            };
        }
        // Orient the meridian so evaluate(t0) == p1, evaluate(t1) == p2 (the
        // merged loop expects seam(v1 -> v2)).
        let d0 = ruling.domain()?[0];
        let start = ruling.evaluate(d0)?;
        let oriented = if start.sub(p1).length() <= start.sub(p2).length() {
            ruling
        } else {
            match ruling.reversed() {
                Ok(curve) => curve,
                Err(_) => return Ok(None),
            }
        };
        let [e0, e1] = oriented.domain()?;
        // The meridian's endpoints must land on the two rim vertices. A vertex
        // NOT on the seam meridian (a diagonal / wrong pairing, e.g. an OCC
        // off-seam rim that was never re-seated) leaves an endpoint gap that
        // scales with the ruling; a genuine wall matches to projection noise.
        // Gate at 2% of the ruling — orders below any diagonal, above any noise
        // — mirroring the straight-chord midpoint guard this replaces.
        let seam_len = p1.sub(p2).length();
        let match_tol = (0.02 * seam_len).max(1e-9);
        if oriented.evaluate(e0)?.sub(p1).length() > match_tol
            || oriented.evaluate(e1)?.sub(p2).length() > match_tol
        {
            return Ok(None);
        }
        Ok(Some(oriented))
    }

    /// JOINT loop-level branch assignment for a self-overlapping NON-PERIODIC
    /// carrier (an AP214 thread band with ZERO pcurves: the trim must be
    /// synthesised by inverting the 3D edges, and a helical band wraps ~8 turns
    /// with adjacent wraps microns apart but a whole parameter turn away).
    ///
    /// The per-coedge fit inverts each edge sample by GLOBAL closest-point
    /// projection, which snaps to whichever fold is momentarily closest — so a
    /// single edge's samples ALIAS across wraps (dozens of branch jumps) and,
    /// worse, the loop VERTICES land on different folds than their edge
    /// interiors. The loop's UV polygon then self-crosses and the tessellator's
    /// hole-bridge faithfully fills ~1000 mm² membranes over the thread.
    /// Distance CANNOT arbitrate the fold — the loop-consistent branch is often
    /// FARTHER than an aliased one (the `project_point_to_surface` poisoned
    /// oracle) — so branch identity must come from CONTINUITY + LOOP CLOSURE.
    ///
    /// This derives one consistent branch for the WHOLE loop: pick the coedge
    /// whose own seeded chain is cleanest (its start vertex sits NEAR-EXACT on
    /// the surface — the structural guarantee that it rides the true fold, since
    /// no edge lies within fit tolerance of a wrong wrap), then propagate the
    /// footpoint seed around the loop, each coedge seeded from the previous
    /// coedge's END and the shared vertex COPIED so the ring closes head-to-tail
    /// by construction. Adopted ONLY when every coedge chains continuously
    /// on-surface AND the loop closes (a purely STRUCTURAL gate; residual is a
    /// validity BOUND, never the branch arbiter). Any failure returns `None` and
    /// the caller keeps the existing per-coedge fit byte-for-byte.
    ///
    /// Hatch: `BREP_LOOP_BRANCH_JOINT=0` restores the old per-coedge path.
    pub(super) fn joint_branch_pcurves(
        &self,
        surface: &NurbsSurface,
        specs: &[(u64, bool)],
        pcurve_tol: f64,
    ) -> Result<Option<Vec<NurbsCurve>>, String> {
        if std::env::var("BREP_LOOP_BRANCH_JOINT").as_deref() == Ok("0") {
            return Ok(None);
        }
        // Structural class. The wrap-aliasing pathology exists only on a general
        // (non-analytic, non-affine) carrier that is OPEN in BOTH directions yet
        // self-overlaps in 3D. Periodic carriers are excluded: their legitimate
        // seam wrap looks like a branch jump and is owned by the seam machinery.
        let (closed_u, closed_v) = surface.closed_directions()?;
        let dbg = std::env::var("BREP_DEBUG_PCURVE").is_ok();
        if closed_u || closed_v || surface.analytic().is_some() || surface.is_affine()? {
            return Ok(None);
        }
        let m = specs.len();
        // Two coedges is a lens — two trims between one pair of vertices — and
        // the joint assignment is exactly as well posed there as on a longer
        // ring: an anchor, one propagation step, and the same closure test. The
        // guard used to read `m < 3` with no stated reason; on `abc_00000026`
        // three 2-coedge lens faces self-cross twice each and were never
        // offered to this pass at all.
        if m < 2 {
            return Ok(None);
        }
        let [u0, u1] = surface.domain_u()?;
        let [v0, v1] = surface.domain_v()?;
        let u_span = (u1 - u0).abs().max(1e-12);
        let v_span = (v1 - v0).abs().max(1e-12);
        let norm_step = |a: (f64, f64), b: (f64, f64)| -> f64 {
            (((a.0 - b.0) / u_span).powi(2) + ((a.1 - b.1) / v_span).powi(2)).sqrt()
        };
        // Local carrier extent (translation-invariant), for the physical bounds.
        let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for i in 0..=4 {
            for j in 0..=8 {
                let u = u0 + (u1 - u0) * i as f64 / 4.0;
                let v = v0 + (v1 - v0) * j as f64 / 8.0;
                let p = surface.evaluate(u, v)?;
                lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            }
        }
        let extent = hi.sub(lo).length().max(1e-9);

        // Sample each coedge in coedge sense; a degenerate (collapsed pole) edge
        // has no branch to track — that loop is not this class, keep the old path.
        const N: usize = 64;
        let mut samples: Vec<Vec<Vec3>> = Vec::with_capacity(m);
        for (edge_id, forward) in specs {
            let edge = self.edge_record(*edge_id);
            if edge.degenerate {
                return Ok(None);
            }
            let mut pts = Vec::with_capacity(N + 1);
            for k in 0..=N {
                let f = k as f64 / N as f64;
                let edge_fraction = if *forward { f } else { 1.0 - f };
                pts.push(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * edge_fraction)?);
            }
            samples.push(pts);
        }

        // Aliasing signature: the SINGLE-SEED inversion of the samples (Newton
        // from the seed grid's nearest sample) branch-JUMPS where the carrier
        // comes back within one seed-grid spacing of itself — the seed grid's
        // nearest sample alternates between the two ends. On a well-behaved
        // open carrier (no self-overlap) it is continuous and this is FALSE, so
        // the face keeps its existing (byte-identical) fit.
        //
        // It is that inversion BY NAME, not the nearest-point projector. Until
        // 2026-09-14 they were the same function; the projector now returns
        // the least minimum over the domain, which does not jump on these
        // loops, and reading it here sent `abc_00000026`'s flank loops to the
        // per-coedge fit: congruent lens faces spread 3.7e-7 in area instead
        // of 3e-12, and two runout loops closed through a degenerate edge.
        //
        // The same pass reads `global_residual`, the worst distance that
        // inversion reports over the loop's stations, for the trace only. It is
        // NOT a floor: its distance is an upper bound, and on exactly the
        // carriers this pass exists for it is the far end's local minimum —
        // 1.2e-1 on `abc_00000026`, whose edges lie within 7.6e-4 of those
        // carriers.
        const JUMP: f64 = 0.2;
        let mut aliased = false;
        let mut global_residual = 0.0f64;
        for pts in &samples {
            let mut prev: Option<(f64, f64)> = None;
            for p in pts {
                let g = crate::projection::project_point_to_surface_from_nearest_seed(surface, *p)?;
                global_residual = global_residual.max(g.distance);
                if let Some(pv) = prev {
                    if norm_step((g.u, g.v), pv) > JUMP {
                        aliased = true;
                    }
                }
                prev = Some((g.u, g.v));
            }
        }
        if dbg {
            eprintln!(
                "joint scan: m={m} extent={extent:.3} aliased={aliased} global_residual={global_residual:.3e}"
            );
        }
        if !aliased {
            return Ok(None);
        }

        // Seeded chain of one coedge from a start seed → (footpoints, worst
        // residual, worst normalized internal step).
        let chain = |ci: usize, seed: (f64, f64)| -> Result<(Vec<(f64, f64)>, f64, f64), String> {
            let pts = &samples[ci];
            let mut cur = seed;
            let mut out = Vec::with_capacity(pts.len());
            let mut worst_res = 0.0f64;
            let mut worst_step = 0.0f64;
            let mut prev: Option<(f64, f64)> = None;
            for p in pts {
                let pr = crate::project_point_to_surface_seeded(surface, *p, cur.0, cur.1)?;
                cur = (pr.u, pr.v);
                worst_res = worst_res.max(pr.distance);
                if let Some(pv) = prev {
                    worst_step = worst_step.max(norm_step(cur, pv));
                }
                prev = Some(cur);
                out.push(cur);
            }
            Ok((out, worst_res, worst_step))
        };

        // Anchor = the coedge whose own self-seeded chain (from its global start
        // image) is cleanest. The gate rejects a WRONG-WRAP anchor, which sits
        // at the wrap gap — orders above the fit band.
        //
        // Measured from zero. From 2026-09-13 to 2026-09-14 it was measured
        // from `global_residual` instead, on the reading that `abc_00000026`'s
        // edges sit 3.052e-2 off their carriers and no branch could beat that.
        // They do not: the edges this gate saw had been moved there by the
        // importer's reconcile pass, which trusted the same projector answer,
        // and the "floor" it was measured from read 1.2e-1 — above the 2.5e-2
        // wrap gap the gate exists to reject, so on that file it rejected
        // nothing. With the file's own edges the 48 loops it adopts read anchor
        // residuals of 1.02e-4 or less against a bound of at least 1.06e-3, and
        // dropping the floor moves no row of the 1069-solid fixture corpus.
        let anchor_bound = (2e-4 * extent).max(20.0 * pcurve_tol);
        let mut anchor: Option<(usize, (f64, f64))> = None;
        let mut best_res = f64::INFINITY;
        for ci in 0..m {
            let g = crate::project_point_to_surface(surface, samples[ci][0])?;
            let (_c, res, step) = chain(ci, (g.u, g.v))?;
            if step < 0.15 && res < best_res {
                best_res = res;
                anchor = Some((ci, (g.u, g.v)));
            }
        }
        let (anchor_ci, anchor_seed) = match anchor {
            Some(a) if best_res <= anchor_bound => a,
            _ => {
                if dbg {
                    eprintln!(
                        "joint-branch: REFUSED anchor m={m} best_res={best_res:.3e} bound={anchor_bound:.3e} global_residual={global_residual:.3e} found={}",
                        anchor.is_some()
                    );
                }
                return Ok(None);
            }
        };

        // Propagate the seed around the whole loop from the anchor: each coedge
        // seeded from the PREVIOUS coedge's end, and the shared loop vertex COPIED
        // (not re-projected) so consecutive coedges are identical there and the
        // ring closes head-to-tail.
        let mut chains: Vec<Vec<(f64, f64)>> = vec![Vec::new(); m];
        let mut cur = anchor_seed;
        let mut worst_res_all = 0.0f64;
        let mut worst_step_all = 0.0f64;
        for step in 0..m {
            let ci = (anchor_ci + step) % m;
            let (mut c, res, st) = chain(ci, cur)?;
            if step > 0 {
                let prev_ci = (anchor_ci + step - 1) % m;
                if let Some(&pe) = chains[prev_ci].last() {
                    c[0] = pe;
                }
            }
            worst_res_all = worst_res_all.max(res);
            worst_step_all = worst_step_all.max(st);
            cur = *c.last().unwrap();
            chains[ci] = c;
        }
        let closure = norm_step(cur, anchor_seed);

        // STRUCTURAL adoption gate: every coedge continuous (no branch jump left),
        // the loop closes, nothing flew off the surface. Any failure → keep the
        // existing per-coedge fit (this face is not cleanly recoverable here).
        let continuous = worst_step_all < 0.15;
        let closes = closure <= 1e-3;
        let on_surface = worst_res_all <= 1e-2 * extent;
        if !(continuous && closes && on_surface) {
            if dbg {
                eprintln!(
                    "joint-branch: REFUSED adopt m={m} anchor=ce{anchor_ci} step={worst_step_all:.3e} closure={closure:.3e} res={worst_res_all:.3e} bound={:.3e} (continuous={continuous} closes={closes} on_surface={on_surface})",
                    1e-2 * extent
                );
            }
            return Ok(None);
        }

        // Force exact ring closure (last coedge's end IS the anchor's start —
        // the same 3D vertex; make them identical in UV so no bridge is minted).
        let last_ci = (anchor_ci + m - 1) % m;
        if let Some(&le) = chains[last_ci].last() {
            chains[anchor_ci][0] = le;
        }

        // Build each coedge's pcurve by interpolating its chained footpoints —
        // already on-surface, a single fold, in coedge-sense 0..1.
        let parameters: Vec<f64> = (0..=N).map(|k| k as f64 / N as f64).collect();
        let mut pcurves = Vec::with_capacity(m);
        for c in &chains {
            let points: Vec<Vec3> = c.iter().map(|&(u, v)| Vec3::new(u, v, 0.0)).collect();
            pcurves.push(crate::interpolate_curve(&points, 3, &parameters)?);
        }
        if std::env::var("BREP_DEBUG_PCURVE").is_ok() {
            eprintln!(
                "joint-branch: adopted loop m={m} anchor=ce{anchor_ci} anchorRes={best_res:.2e} worstRes={worst_res_all:.2e} closure={closure:.2e}"
            );
        }
        Ok(Some(pcurves))
    }
}
