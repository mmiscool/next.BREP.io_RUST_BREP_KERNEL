use super::*;

impl BrepSolid {
    pub fn validate(&self) -> Vec<ValidationIssue> {
        self.validate_with_tolerances(&KernelTolerances::for_solid(self, 1e-7))
    }

    pub fn validate_with_tolerances(&self, tolerances: &KernelTolerances) -> Vec<ValidationIssue> {
        self.validate_detailed(tolerances).issues
    }

    pub fn validate_detailed(&self, tolerances: &KernelTolerances) -> ValidationReport {
        let mut issues = Vec::new();
        let mut wire_warnings = Vec::new();
        let mut max_pcurve_error = 0.0f64;
        // Model bounding-box diagonal: the LOCAL extent the edge/pcurve
        // coincidence band size-couples to (see `pcurve_acceptance`). Raw, not
        // `solid_scale` — that floors at 1.0, which would hand a genuinely tiny
        // part a coincidence band 30x its own size; the tight absolute
        // `pcurve_consistency` floor already guards the small end.
        let mut bbox_lo = crate::Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut bbox_hi = crate::Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for vertex in &self.vertices {
            bbox_lo.x = bbox_lo.x.min(vertex.point.x);
            bbox_lo.y = bbox_lo.y.min(vertex.point.y);
            bbox_lo.z = bbox_lo.z.min(vertex.point.z);
            bbox_hi.x = bbox_hi.x.max(vertex.point.x);
            bbox_hi.y = bbox_hi.y.max(vertex.point.y);
            bbox_hi.z = bbox_hi.z.max(vertex.point.z);
        }
        let model_diagonal = if self.vertices.is_empty() {
            0.0
        } else {
            bbox_hi.sub(bbox_lo).length()
        };
        let pcurve_limit = tolerances.pcurve_acceptance(model_diagonal);
        // Vendor STEP files commit a vertex point and its incident edges' 3D-curve
        // endpoints as INDEPENDENT approximations that disagree by a small, roughly
        // ABSOLUTE amount (ABC 00000084/109/218 fail ONLY here: every gap clusters
        // tightly at ~1.0-2.2e-5 regardless of the sub-unit part size; ABC 00010746
        // has the same repeated pattern at ~3.2-3.9e-5 on 24 matching endpoints —
        // a fixed export-precision offset, not a size-relative one). Like OCC/Parasolid
        // absorbing the gap in a widened per-vertex tolerance, accept it: floor the
        // identity band at a vendor-precision absolute and size-couple for large
        // parts via `heal_band`. The old fixed `max(model*100, 1e-5)` band (model
        // is pinned at 1e-7, so a flat 1e-5) rejected these shared vertices; a
        // genuinely disconnected edge sits orders of magnitude above this floor, so
        // real breakage is still refused. Clean parts already match far tighter, so
        // this only admits the vendor near-miss.
        // k = 2e-5: at metre numerics the .max(4e-5) ABSOLUTE floor was the
        // effective acceptance (~1.4e-4 of a 0.28-extent part) and silently
        // absorbed vendor edge-vs-vertex gaps; in mm the floor is inert and
        // k=1e-5 narrowly rejected an authored 1.25e-5-of-diagonal gap
        // (ABC 8575). 2e-5 keeps the check meaningful while accepting vendor
        // imprecision the old floor accepted. Validation acceptance only —
        // no merge radius derives from this.
        // Floor = vendor EXPORT PRECISION in mm. The old 4e-5 floor encoded
        // metre-era numerics (~4e-5 of a metre); the same physical export
        // imprecision lands x1000 bigger now that imports convert to mm
        // (ABC 8575: a 3.5e-6 m authored vertex/curve offset = 3.5e-3 mm).
        // 5e-3 mm (= 5 um) accepts vendor precision for metre- and
        // mm-authored files alike. Validation acceptance only — no merge
        // radius derives from this.
        let vertex_match = tolerances
            .heal_band(model_diagonal, 2e-5)
            .max(crate::VERTEX_MATCH_FLOOR);
        let vertices: HashMap<u64, &VertexRecord> = self
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex))
            .collect();
        let edges: HashMap<u64, &EdgeRecord> =
            self.edges.iter().map(|edge| (edge.id, edge)).collect();
        if vertices.len() != self.vertices.len() {
            issues.push(ValidationIssue::error("duplicate vertex id"));
        }
        if edges.len() != self.edges.len() {
            issues.push(ValidationIssue::error("duplicate edge id"));
        }

        let mut edge_uses: HashMap<u64, Vec<bool>> = HashMap::default();
        let mut face_ids = HashSet::default();
        for shell in &self.shells {
            for face in &shell.faces {
                if !face_ids.insert(face.id) {
                    issues.push(ValidationIssue::error(format!(
                        "duplicate face id {}",
                        face.id
                    )));
                }
                let surface = match NurbsSurface::new(
                    face.surface.degree_u,
                    face.surface.degree_v,
                    face.surface.knots_u.clone(),
                    face.surface.knots_v.clone(),
                    face.surface.control_points.clone(),
                ) {
                    Ok(surface) => surface,
                    Err(error) => {
                        issues.push(ValidationIssue::error(format!(
                            "face {} has invalid surface: {}",
                            face.id, error
                        )));
                        continue;
                    }
                };
                for (loop_index, loop_record) in face.loops.iter().enumerate() {
                    if loop_record.coedges.is_empty() {
                        issues.push(ValidationIssue::error(format!(
                            "loop {} of face {} is empty",
                            loop_record.id, face.id
                        )));
                        continue;
                    }
                    for coedge in &loop_record.coedges {
                        edge_uses
                            .entry(coedge.edge_id)
                            .or_default()
                            .push(coedge.forward);
                    }
                    for index in 0..loop_record.coedges.len() {
                        let current = &loop_record.coedges[index];
                        let next = &loop_record.coedges[(index + 1) % loop_record.coedges.len()];
                        let Some(current_edge) = edges.get(&current.edge_id) else {
                            issues.push(ValidationIssue::error(format!(
                                "coedge {} references missing edge {}",
                                current.id, current.edge_id
                            )));
                            continue;
                        };
                        let Some(next_edge) = edges.get(&next.edge_id) else {
                            issues.push(ValidationIssue::error(format!(
                                "coedge {} references missing edge {}",
                                next.id, next.edge_id
                            )));
                            continue;
                        };
                        let current_end = if current.forward {
                            current_edge.end_vertex_id
                        } else {
                            current_edge.start_vertex_id
                        };
                        let next_start = if next.forward {
                            next_edge.start_vertex_id
                        } else {
                            next_edge.end_vertex_id
                        };
                        if current_end != next_start {
                            issues.push(ValidationIssue::error(format!(
                                "loop {} of face {} is open between coedges {} and {}",
                                loop_record.id, face.id, current.id, next.id
                            )));
                        }

                        let pcurve = match NurbsCurve::new(
                            current.pcurve.degree,
                            current.pcurve.knots.clone(),
                            current.pcurve.control_points.clone(),
                        ) {
                            Ok(curve) => curve,
                            Err(error) => {
                                issues.push(ValidationIssue::error(format!(
                                    "coedge {} has invalid pcurve: {}",
                                    current.id, error
                                )));
                                continue;
                            }
                        };
                        let curve = match NurbsCurve::new(
                            current_edge.curve.degree,
                            current_edge.curve.knots.clone(),
                            current_edge.curve.control_points.clone(),
                        ) {
                            Ok(curve) => curve,
                            Err(error) => {
                                issues.push(ValidationIssue::error(format!(
                                    "edge {} has invalid curve: {}",
                                    current_edge.id, error
                                )));
                                continue;
                            }
                        };
                        match coedge_error_floored(
                            &surface,
                            &pcurve,
                            &curve,
                            current_edge,
                            current.forward,
                            pcurve_limit,
                        ) {
                            Ok(error) => {
                                max_pcurve_error = max_pcurve_error.max(error);
                                if error > pcurve_limit {
                                    issues.push(ValidationIssue::error(format!(
                                        "coedge {} of face {} pcurve is inconsistent with edge {} \
                                         (max deviation {:.6}, limit {:.6})",
                                        current.id, face.id, current_edge.id, error, pcurve_limit,
                                    )));
                                }
                            }
                            Err(error) => issues.push(ValidationIssue::error(format!(
                                "coedge {} of face {} cannot be evaluated: {}",
                                current.id, face.id, error
                            ))),
                        }
                    }

                    if let Some(warning) = validate_uv_wire(
                        &surface,
                        loop_record,
                        &edges,
                        tolerances,
                        loop_index == 0,
                        face.same_sense,
                    ) {
                        wire_warnings.push(ValidationIssue::warning(format!(
                            "face {} loop {}: {}",
                            face.id, loop_record.id, warning
                        )));
                    }
                }
            }
        }

        for edge in &self.edges {
            if !(edge.t0.is_finite() && edge.t1.is_finite() && edge.t0 < edge.t1) {
                issues.push(ValidationIssue::error(format!(
                    "edge {} has invalid parameter range",
                    edge.id
                )));
            }
            let Some(start) = vertices.get(&edge.start_vertex_id) else {
                issues.push(ValidationIssue::error(format!(
                    "edge {} references missing start vertex {}",
                    edge.id, edge.start_vertex_id
                )));
                continue;
            };
            let Some(end) = vertices.get(&edge.end_vertex_id) else {
                issues.push(ValidationIssue::error(format!(
                    "edge {} references missing end vertex {}",
                    edge.id, edge.end_vertex_id
                )));
                continue;
            };
            if let Ok(curve) = NurbsCurve::new(
                edge.curve.degree,
                edge.curve.knots.clone(),
                edge.curve.control_points.clone(),
            ) {
                if let Ok(point) = curve.evaluate(edge.t0) {
                    let gap = point.sub(start.point).length();
                    if gap > vertex_match {
                        issues.push(ValidationIssue::error_kind(IssueKind::CurveVertexGap { at_start: true }, format!(
                            "edge {} curve start does not match vertex {} \
                             (gap={gap:.9}, curve={point:?}, vertex={:?})",
                            edge.id, start.id, start.point
                        )));
                    }
                }
                if let Ok(point) = curve.evaluate(edge.t1) {
                    let gap = point.sub(end.point).length();
                    if gap > vertex_match {
                        issues.push(ValidationIssue::error_kind(IssueKind::CurveVertexGap { at_start: false }, format!(
                            "edge {} curve end does not match vertex {} \
                             (gap={gap:.9}, curve={point:?}, vertex={:?})",
                            edge.id, end.id, end.point
                        )));
                    }
                }
            }
            let uses = edge_uses.get(&edge.id).map(Vec::as_slice).unwrap_or(&[]);
            if edge.degenerate {
                // A synthesized VERTEX_LOOP/pole boundary is single-use, but
                // vendor STEP may share one explicit collapsed EDGE_CURVE
                // between the two faces meeting at that pole. The latter is a
                // valid manifold incidence exactly when the coedge senses are
                // opposite, just like an ordinary shared edge.
                let valid = uses.len() == 1 || (uses.len() == 2 && uses[0] != uses[1]);
                if !valid {
                    issues.push(ValidationIssue::error(format!(
                        "degenerate edge {} has invalid incidence {:?} \
                         (expected one use or an opposite-sense pair)",
                        edge.id, uses,
                    )));
                }
            } else if uses.len() != 2 {
                let kind = if uses.len() < 2 { IssueKind::OpenEdge } else { IssueKind::OverUsedEdge };
                issues.push(ValidationIssue::error_kind(kind, format!(
                    "edge {} used {} times (expected 2)",
                    edge.id,
                    uses.len()
                )));
            } else if uses[0] == uses[1] {
                issues.push(ValidationIssue::error(format!(
                    "edge {} has coedges with the same sense",
                    edge.id
                )));
            }
        }

        for edge_id in edge_uses.keys() {
            if !edges.contains_key(edge_id) {
                issues.push(ValidationIssue::error(format!(
                    "topology references missing edge {}",
                    edge_id
                )));
            }
        }

        // A pole edge is a collapsed parameter-space boundary. Its endpoint
        // is not an independent 0-cell unless a non-degenerate edge also
        // uses it, so use the reduced complex for Euler accounting.
        let non_degenerate_vertex_ids = self
            .edges
            .iter()
            .filter(|edge| !edge.degenerate)
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .collect::<HashSet<_>>();
        let vertex_count = vertices
            .keys()
            .filter(|id| non_degenerate_vertex_ids.contains(id))
            .count() as i64;
        let edge_count = self.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
        let face_count = face_ids.len() as i64;
        let hole_count = self.bounding_hole_count();
        let shell_count = self.shells.len() as i64;
        let actual = vertex_count - edge_count + face_count - hole_count;
        let expected = 2 * (shell_count - self.genus);
        // What this comparison actually tests, and it is not the genus.
        //
        // On a freshly derived solid the producer set `genus = S - chi/2` in
        // INTEGER division, so `expected = 2*(S - genus) = 2*trunc(chi/2)` and
        // `actual == expected` holds EXACTLY when the characteristic is even.
        // Measured over 1042 imported bodies, the rows that mismatch and the
        // rows with an odd characteristic are the same rows, both ways. After a
        // mutation that adjusts `genus` by hand -- `closed_heal`'s `genus -= 1`,
        // `delete_faces`' `genus += shift / 2` -- it becomes a drift check on
        // that adjustment. It is never an independent measurement of the genus:
        // `genus = S - chi/2` IS the definition, so any recomputation from the
        // same complex re-runs this arithmetic.
        //
        // The PRECONDITION, which this check did not used to state.
        //
        // `chi = 2(S - G)` is a fact about CLOSED orientable surfaces. On an
        // open shell -- one still being assembled, or a sheet -- the
        // characteristic is unconstrained (a disk is 1) and comparing it to
        // anything is meaningless. Until 2026-09-11 the comparison ran on open
        // shells too and was merely hidden, on some of them, by a skip for
        // degenerate edges: three offset-shell cases turn out to reach it with
        // an open shell and an EVEN characteristic, so the skip was covering
        // for a missing precondition rather than for a pole-collapse problem.
        //
        // The degenerate-edge skip that used to stand here is GONE, and what it
        // was hiding was not what its comment claimed.
        //
        // It read "the reduced V-E+F formula does not model parameter-space
        // pole collapses reliably". Removing it reddened three offset-shell
        // tests with an EVEN characteristic and a recorded genus that
        // contradicted it -- so not a parity problem, and reproducible with the
        // pre-2026-09-11 hole rule, so not a consequence of that work either.
        // The cause was `finalize.rs` forcing `genus = 0` whenever a body's only
        // single-use edges are degenerate, which is true of any body carrying an
        // ordinary pole edge. A shelled solid with a spherical cavity wall was
        // therefore recorded genus 0 while its own count said 3, and an
        // independent watertight MESH count of the same shell confirmed the
        // three handles (chi = -4 at three chord tolerances, closed, no boundary
        // or non-manifold edges). The handles are real -- shelling on an ANNULAR
        // opening joins outer and inner skins along two rim curves -- so only the
        // recorded number was wrong. With `finalize` recording what it counts,
        // this skip has nothing left to hide and the check runs on every solid.
        //
        // `closed` is the precondition the formula genuinely needs: chi =
        // 2(S - G) is a fact about CLOSED orientable surfaces, and an open
        // shell's characteristic is unconstrained (a disk is 1). It is stated so
        // this check and `soundness::solid_euler` agree about the domain they
        // apply to -- the disagreement between them, one having the precondition
        // and the other not, is what made a green case gate look like coverage
        // it was not.
        let closed = self
            .edges
            .iter()
            .filter(|edge| !edge.degenerate)
            .all(|edge| edge_uses.get(&edge.id).map(Vec::len) == Some(2));
        if closed && actual != expected {
            // OCC-style models carry COINCIDENT PARALLEL edges: two
            // ref-distinct edges riding one geometric locus between the same
            // vertices (a wall split exactly where an adjoining ring meets
            // it). The complex is manifold — the four incident faces pair
            // off along the shared locus — but the formula counts the locus
            // twice. Re-check with each coincident pair credited once; the
            // scan runs only on this failure path, so ordinary solids pay
            // nothing for it.
            let pairs = self.coincident_parallel_edge_pairs() as i64;
            if actual + pairs != expected {
                issues.push(ValidationIssue::error_kind(IssueKind::GenusMismatch, format!(
                    "Euler formula: V-E+F-H = {}, expected {} ({} coincident parallel edge pair(s) credited)",
                    actual + pairs,
                    expected,
                    pairs
                )));
            }
        }
        ValidationReport {
            issues,
            wire_warnings,
            max_pcurve_error,
        }
    }

    /// `H` in the reduced-complex Euler count `V - E + F - H`: per face, the
    /// loops that actually bound area, less its outer one.
    ///
    /// A boundary collapsed to a single point — STEP's `VERTEX_LOOP`, which
    /// this kernel models as a single-use degenerate edge carrying a pcurve —
    /// encloses no area, so it is not a hole. The reduced complex already
    /// leaves that edge out of `E` and its vertex out of `V`; leaving its loop
    /// out of `H` is the same rule applied to the same collapsed cell.
    ///
    /// Counting it as a hole subtracts a spurious 1 per puncture. An ODD number
    /// of punctures then makes the characteristic odd, which no closed
    /// orientable surface has; an even number keeps it even and quietly inflates
    /// the derived genus by half of them.
    pub(crate) fn bounding_hole_count(&self) -> i64 {
        let degenerate: HashSet<u64> = self
            .edges
            .iter()
            .filter(|edge| edge.degenerate)
            .map(|edge| edge.id)
            .collect();
        self.shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .map(|face| {
                face.loops
                    .iter()
                    .filter(|record| {
                        !record.coedges.is_empty()
                            && !record
                                .coedges
                                .iter()
                                .all(|coedge| degenerate.contains(&coedge.edge_id))
                    })
                    .count()
                    .saturating_sub(1) as i64
            })
            .sum()
    }

    /// Count disjoint pairs of non-degenerate edges that ride ONE geometric
    /// locus between the same endpoint vertices (OCC wall-split rims). Each
    /// such pair contributes a single locus to the cell complex, so Euler
    /// accounting credits it once.
    pub(crate) fn coincident_parallel_edge_pairs(&self) -> usize {
        let candidates: Vec<&EdgeRecord> =
            self.edges.iter().filter(|edge| !edge.degenerate).collect();
        let mut consumed = vec![false; candidates.len()];
        let mut pairs = 0usize;
        for first_index in 0..candidates.len() {
            if consumed[first_index] {
                continue;
            }
            let first = candidates[first_index];
            for second_index in first_index + 1..candidates.len() {
                if consumed[second_index] {
                    continue;
                }
                let second = candidates[second_index];
                let endpoints_match = (first.start_vertex_id == second.start_vertex_id
                    && first.end_vertex_id == second.end_vertex_id)
                    || (first.start_vertex_id == second.end_vertex_id
                        && first.end_vertex_id == second.start_vertex_id);
                if !endpoints_match {
                    continue;
                }
                // Locus agreement at interior samples, direction-agnostic.
                let coincident = (1..4).all(|sample| {
                    let fraction = sample as f64 / 4.0;
                    let Ok(on_first) = first
                        .curve
                        .evaluate(first.t0 + (first.t1 - first.t0) * fraction)
                    else {
                        return false;
                    };
                    let forward = second.t0 + (second.t1 - second.t0) * fraction;
                    let backward = second.t1 - (second.t1 - second.t0) * fraction;
                    let tolerance = 1e-6 * (1.0 + on_first.length());
                    let matches_forward = second
                        .curve
                        .evaluate(forward)
                        .map(|point| point.sub(on_first).length() <= tolerance)
                        .unwrap_or(false);
                    let matches_backward = second
                        .curve
                        .evaluate(backward)
                        .map(|point| point.sub(on_first).length() <= tolerance)
                        .unwrap_or(false);
                    matches_forward || matches_backward
                });
                if coincident {
                    consumed[first_index] = true;
                    consumed[second_index] = true;
                    pairs += 1;
                    break;
                }
            }
        }
        pairs
    }
}

fn coedge_sample(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    fraction: f64,
) -> Result<(f64, Vec3, Vec3), String> {
    let [q0, q1] = pcurve.domain()?;
    let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction)?;
    // A pcurve that straddles a periodic seam carries parameters just past the
    // domain; the surface WRAPS there (evaluate_extended), so a coedge riding
    // the seam still reproduces its edge exactly instead of reading as a gross
    // deviation against the clamped boundary.
    let on_surface = surface.evaluate_extended(uv.x, uv.y)?;
    let t = if forward {
        edge.t0 + (edge.t1 - edge.t0) * fraction
    } else {
        edge.t1 - (edge.t1 - edge.t0) * fraction
    };
    let on_curve = curve.evaluate(t)?;
    Ok((on_surface.sub(on_curve).length(), on_surface, on_curve))
}

/// Start with 32 intervals, then subdivide where either represented curve
/// bends appreciably or the deviation function is non-linear.  This catches
/// interior NURBS drift that four fixed samples cannot see without imposing
/// the cost of a uniformly extreme sampling count on every coedge.
pub(crate) fn adaptive_coedge_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    tolerance: f64,
) -> Result<f64, String> {
    fn interval(
        surface: &NurbsSurface,
        pcurve: &NurbsCurve,
        curve: &NurbsCurve,
        edge: &EdgeRecord,
        forward: bool,
        a: f64,
        b: f64,
        sample_a: (f64, Vec3, Vec3),
        sample_b: (f64, Vec3, Vec3),
        tolerance: f64,
        depth: usize,
    ) -> Result<f64, String> {
        let mid = (a + b) * 0.5;
        let sample_mid = coedge_sample(surface, pcurve, curve, edge, forward, mid)?;
        let error_nonlinearity = (sample_mid.0 - (sample_a.0 + sample_b.0) * 0.5).abs();
        let surface_bend = sample_mid
            .1
            .sub(sample_a.1.add(sample_b.1).scale(0.5))
            .length();
        let curve_bend = sample_mid
            .2
            .sub(sample_a.2.add(sample_b.2).scale(0.5))
            .length();
        let local_max = sample_a.0.max(sample_mid.0).max(sample_b.0);
        if depth >= 3
            || (error_nonlinearity <= tolerance * 0.05
                && surface_bend.max(curve_bend) <= tolerance * 0.25)
        {
            return Ok(local_max);
        }
        Ok(interval(
            surface,
            pcurve,
            curve,
            edge,
            forward,
            a,
            mid,
            sample_a,
            sample_mid,
            tolerance,
            depth + 1,
        )?
        .max(interval(
            surface,
            pcurve,
            curve,
            edge,
            forward,
            mid,
            b,
            sample_mid,
            sample_b,
            tolerance,
            depth + 1,
        )?))
    }

    let mut maximum = 0.0f64;
    let mut previous = coedge_sample(surface, pcurve, curve, edge, forward, 0.0)?;
    maximum = maximum.max(previous.0);
    for index in 1..=32 {
        let a = (index - 1) as f64 / 32.0;
        let b = index as f64 / 32.0;
        let next = coedge_sample(surface, pcurve, curve, edge, forward, b)?;
        maximum = maximum.max(interval(
            surface, pcurve, curve, edge, forward, a, b, previous, next, tolerance, 0,
        )?);
        previous = next;
    }
    // This pass refines only while the band it is given says to, so on a part
    // whose band is slack it stops at its 33 uniform stations and a pcurve
    // exact at its own nodes and wrong between them reads as exact (measured
    // 2026-09-18 over 54556 imported pcurves: more than 2x low on 2204). The
    // band-free floor is [`coedge_error_floored`]; `validate` reads through
    // it, the boolean's repair decisions do not yet.
    Ok(maximum)
}

/// [`adaptive_coedge_error`] with the band-free floor: the midpoint of every
/// knot span of the pcurve and of the edge curve, read whatever the band. This
/// is what `validate` and the offset family's measured tolerance read. The
/// boolean's repair decisions (`csg/boolean/assemble/finalize.rs`,
/// `refusal_welds.rs`) and the healing coalesce keep the band-steered reading
/// above: read through the floor, the zero-clearance ball-and-stick joint
/// gains an edge and a vertex at its crotch ((5, 9, 6) against (5, 8, 5),
/// `tests/suites/tangent_node_assembly.rs`, 2026-09-26) — a consumer that
/// depends on its instrument's coarseness, named for its own slice.
pub(crate) fn coedge_error_floored(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    tolerance: f64,
) -> Result<f64, String> {
    let steered = adaptive_coedge_error(surface, pcurve, curve, edge, forward, tolerance)?;
    Ok(steered.max(knot_span_midpoint_error(surface, pcurve, curve, edge, forward)?))
}

/// `BREP_DEBUG_COEDGE_FLOOR`: trace every knot-span midpoint the floor reads
/// over 1e-3, once per process (the floor runs per span of every coedge of
/// every `validate()`, so the environment is not read per sample).
fn coedge_floor_tracing() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_DEBUG_COEDGE_FLOOR").is_some())
}

/// `‖S(q(s)) − c(t(s))‖` at the midpoint of every knot span of `pcurve` and of
/// every knot span of `curve` inside the edge's `[t0, t1]`, both mapped to the
/// coedge fraction `adaptive_coedge_error` samples on — no tolerance steers
/// where it looks. The pcurve half is the one no other pass takes: a fitted
/// pcurve interpolates at its nodes and misses between them, and its nodes
/// are its own knots, not the edge curve's.
fn knot_span_midpoint_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
) -> Result<f64, String> {
    let mut worst = 0.0_f64;
    let [q0, q1] = pcurve.domain()?;
    if q1 > q0 {
        let mut previous = q0;
        for &knot in pcurve.knots.iter().chain(std::iter::once(&q1)) {
            if knot > previous && knot <= q1 {
                let fraction = ((previous + knot) * 0.5 - q0) / (q1 - q0);
                let sample = coedge_sample(surface, pcurve, curve, edge, forward, fraction)?;
                if coedge_floor_tracing() && sample.0 > 1e-3 {
                    let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction)?;
                    let ([u0, u1], [v0, v1]) = (surface.domain_u()?, surface.domain_v()?);
                    eprintln!(
                        "COEDGE-FLOOR edge {} pcurve-span midpoint fraction {fraction:.6}: miss {:.4e} at uv ({:.6}, {:.6}) domain u[{u0},{u1}] v[{v0},{v1}] closed {:?} analytic {:?} degree {} knots {} cps {}",
                        edge.id, sample.0, uv.x, uv.y, surface.closed_directions()?, surface.analytic().map(|_| "yes"), pcurve.degree, pcurve.knots.len(), pcurve.control_points.len()
                    );
                }
                worst = worst.max(sample.0);
                previous = knot;
            }
        }
    }
    let span = edge.t1 - edge.t0;
    if span.is_finite() && span != 0.0 {
        let low = edge.t0.min(edge.t1);
        let high = edge.t0.max(edge.t1);
        let mut previous = low;
        for &knot in curve.knots.iter().chain(std::iter::once(&high)) {
            if knot > previous && knot <= high {
                let midpoint = (previous + knot) * 0.5;
                let fraction = if forward {
                    (midpoint - edge.t0) / span
                } else {
                    (edge.t1 - midpoint) / span
                };
                let sample = coedge_sample(surface, pcurve, curve, edge, forward, fraction)?;
                if coedge_floor_tracing() && sample.0 > 1e-3 {
                    let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction)?;
                    eprintln!(
                        "COEDGE-FLOOR edge {} edge-span midpoint fraction {fraction:.6}: miss {:.4e} at uv ({:.6}, {:.6})",
                        edge.id, sample.0, uv.x, uv.y
                    );
                }
                worst = worst.max(sample.0);
                previous = knot;
            }
        }
    }
    Ok(worst)
}

fn segments_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2, tolerance: f64) -> bool {
    let cross = |first: Vec2, second: Vec2| first.x * second.y - first.y * second.x;
    let ab = b.sub(a);
    let cd = d.sub(c);
    let denominator = cross(ab, cd);
    if denominator.abs() <= tolerance {
        return false;
    }
    let ac = c.sub(a);
    let t = cross(ac, cd) / denominator;
    let u = cross(ac, ab) / denominator;
    t > tolerance && t < 1.0 - tolerance && u > tolerance && u < 1.0 - tolerance
}

fn validate_uv_wire(
    surface: &NurbsSurface,
    loop_record: &LoopRecord,
    edges: &HashMap<u64, &EdgeRecord>,
    tolerances: &KernelTolerances,
    outer: bool,
    same_sense: bool,
) -> Option<String> {
    let u_domain = crate::KnotVector::new(surface.knots_u.clone(), surface.degree_u)
        .ok()?
        .domain();
    let v_domain = crate::KnotVector::new(surface.knots_v.clone(), surface.degree_v)
        .ok()?
        .domain();
    let u_span = u_domain[1] - u_domain[0];
    let v_span = v_domain[1] - v_domain[0];
    let mut points = Vec::<Vec2>::new();
    for coedge in &loop_record.coedges {
        let edge = edges.get(&coedge.edge_id)?;
        if edge.degenerate {
            continue;
        }
        let [q0, q1] = coedge.pcurve.domain().ok()?;
        for index in 0..=8 {
            if !points.is_empty() && index == 0 {
                continue;
            }
            let fraction = index as f64 / 8.0;
            let parameter = q0 + (q1 - q0) * fraction;
            let value = coedge.pcurve.evaluate(parameter).ok()?;
            let mut point = Vec2 {
                x: value.x,
                y: value.y,
            };
            if let Some(previous) = points.last() {
                while point.x - previous.x > u_span * 0.5 {
                    point.x -= u_span;
                }
                while point.x - previous.x < -u_span * 0.5 {
                    point.x += u_span;
                }
                while point.y - previous.y > v_span * 0.5 {
                    point.y -= v_span;
                }
                while point.y - previous.y < -v_span * 0.5 {
                    point.y += v_span;
                }
            }
            points.push(point);
        }
    }
    if points.len() < 4 {
        return None;
    }
    let first = points[0];
    let last = *points.last()?;
    let uv_tolerance = tolerances.model.max(1e-8);
    if last.sub(first).length() > uv_tolerance * 100.0 {
        // Parameter seams may differ by a complete period.  Compare modulo
        // both domains before reporting a genuine open wire.
        let du = (last.x - first.x) / u_span;
        let dv = (last.y - first.y) / v_span;
        if (du - du.round()).abs() * u_span > uv_tolerance * 100.0
            || (dv - dv.round()).abs() * v_span > uv_tolerance * 100.0
        {
            return Some(format!(
                "wire is open in parameter space (gap {:.3e})",
                last.sub(first).length()
            ));
        }
    }
    for first_index in 0..points.len() - 1 {
        for second_index in first_index + 2..points.len() - 1 {
            if first_index == 0 && second_index + 1 == points.len() - 1 {
                continue;
            }
            if segments_cross(
                points[first_index],
                points[first_index + 1],
                points[second_index],
                points[second_index + 1],
                1e-10,
            ) {
                return Some(format!(
                    "wire self-intersects near sampled segments {first_index} and {second_index}"
                ));
            }
        }
    }
    let area = points
        .windows(2)
        .map(|pair| pair[0].x * pair[1].y - pair[1].x * pair[0].y)
        .sum::<f64>()
        * 0.5;
    if area.abs() > uv_tolerance * uv_tolerance {
        let expected_positive = if outer { same_sense } else { !same_sense };
        if (area > 0.0) != expected_positive {
            return Some(format!(
                "{} wire winding disagrees with face sense (signed UV area {:.6})",
                if outer { "outer" } else { "inner" },
                area
            ));
        }
    }
    None
}
