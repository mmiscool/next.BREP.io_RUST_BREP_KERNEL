use super::*;

impl BrepSolid {
    pub fn validate(&self) -> Vec<ValidationIssue> {
        self.validate_with_tolerances(&KernelTolerances::for_solid(self, 1e-7))
    }

    /// [`validate`](Self::validate) with its CLEAN verdicts memoised by the
    /// body's content (lane R1 row 3, lane B's B3). A rim built by the
    /// closed-rim lane is validated by the surgery's repaired-branch check, by
    /// the group lane and by the pipeline's exit acceptance; `validate` is a
    /// pure function of the body, so a body whose serialised record read clean
    /// once reads clean again. The key is two independent 64-bit hashes of the
    /// serialisation, so ANY mutation (a moved control point, a re-trimmed
    /// coedge, a renamed face) is validated afresh — the key is the content,
    /// never an address. A body with issues is validated every time. The memo
    /// is thread-local, bounded, and emptied by [`BrepSolid::clear_validate_memo`] at
    /// each fillet operation's entry. On by default (promoted 2026-10-18 on
    /// the 98 + 144 census; `BREP_VALIDATE_MEMO=0` validates every time); counted as
    /// `count.validate.memo_hit` / `count.validate.memo_miss`.
    pub(crate) fn validate_memoised(&self) -> Vec<ValidationIssue> {
        if !validate_memo_on() {
            return self.validate();
        }
        let key_timing = crate::blend::BlendTimer::start("timing.validate.memo_key_ms");
        let Some(key) = self.validate_memo_key() else {
            return self.validate();
        };
        key_timing.stop();
        if VALIDATED_CLEAN.with(|clean| clean.borrow().contains(&key)) {
            crate::blend::record_count("count.validate.memo_hit", 1);
            return Vec::new();
        }
        crate::blend::record_count("count.validate.memo_miss", 1);
        let issues = self.validate();
        if issues.is_empty() {
            VALIDATED_CLEAN.with(|clean| {
                let mut clean = clean.borrow_mut();
                if clean.len() >= VALIDATED_CLEAN_CAP {
                    clean.clear();
                }
                clean.insert(key);
            });
        }
        issues
    }

    pub fn validate_with_tolerances(&self, tolerances: &KernelTolerances) -> Vec<ValidationIssue> {
        self.validate_detailed(tolerances).issues
    }

    pub fn validate_detailed(&self, tolerances: &KernelTolerances) -> ValidationReport {
        self.validate_detailed_scoped(tolerances, None)
    }

    /// [`validate`](Self::validate) reading the per-coedge GEOMETRY (carrier,
    /// pcurve against edge, uv wire) of the faces in `geometry_faces` only.
    /// Every structural check — ids, loop closure, edge use counts, curve
    /// endpoints against vertices, the Euler count — still reads the whole
    /// solid. A boolean uses this for faces it carried through unchanged
    /// from an operand (`BooleanOptions::untouched_passthrough`): their
    /// geometry is the operand's, byte for byte, and re-reading it is the
    /// cost the record named.
    pub fn validate_scoped(&self, geometry_faces: &HashSet<u64>) -> Vec<ValidationIssue> {
        self.validate_detailed_scoped(&KernelTolerances::for_solid(self, 1e-7), Some(geometry_faces))
            .issues
    }

    /// [`validate_detailed`](Self::validate_detailed) with the geometry checks
    /// restricted as [`validate_scoped`](Self::validate_scoped) describes;
    /// `None` reads every face.
    pub fn validate_detailed_scoped(
        &self,
        tolerances: &KernelTolerances,
        geometry_faces: Option<&HashSet<u64>>,
    ) -> ValidationReport {
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
                let read_geometry =
                    geometry_faces.is_none_or(|scope| scope.contains(&face.id));
                let surface = if read_geometry {
                    match NurbsSurface::new(
                        face.surface.degree_u,
                        face.surface.degree_v,
                        face.surface.knots_u.clone(),
                        face.surface.knots_v.clone(),
                        face.surface.control_points.clone(),
                    ) {
                        Ok(surface) => Some(surface),
                        Err(error) => {
                            issues.push(ValidationIssue::error(format!(
                                "face {} has invalid surface: {}",
                                face.id, error
                            )));
                            continue;
                        }
                    }
                } else {
                    None
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
                        let Some(surface) = &surface else {
                            continue;
                        };

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
                        match memoized_coedge_error(
                            surface,
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

                    let Some(surface) = &surface else {
                        continue;
                    };
                    if let Some(warning) = validate_uv_wire(
                        surface,
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
            // Non-manifold plan 1.2: only a DECLARED Sheet admits a lamina
            // edge. Unreferenced edges and three-or-more uses still fail;
            // shared edges keep the existing opposite-sense rule below.
            } else if self.kind == BodyKind::Sheet && uses.len() == 1 {
            } else if self.kind == BodyKind::Cellular {
                // Region closure and radial senses are checked per declared
                // cell below. A shared wall can give three global face uses.
                if uses.len() < 2 {
                    issues.push(ValidationIssue::error_kind(IssueKind::OpenEdge,
                        format!("Cellular edge {} used {} times (expected at least 2)", edge.id, uses.len())));
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
        // Non-manifold plan 1.2: a Sheet does not declare a solid genus,
        // including when its particular set of faces happens to be closed.
        if !matches!(self.kind, BodyKind::Sheet | BodyKind::Cellular) && closed && actual != expected {
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
        if self.kind == BodyKind::Cellular {
            self.validate_cells(&edges, &mut issues);
        }
        ValidationReport {
            issues,
            wire_warnings,
            max_pcurve_error,
        }
    }

    // Plan 2.1: this is reached only for a DECLARED Cellular body. All
    // geometry/identity checks above still run; no Solid/Sheet rule changes.
    fn validate_cells(&self, edges: &HashMap<u64, &EdgeRecord>, issues: &mut Vec<ValidationIssue>) {
        if self.cells.is_empty() {
            issues.push(ValidationIssue::error("Cellular body has no declared cells"));
            return;
        }
        let faces: HashMap<_, _> = self.shells.iter().flat_map(|s| &s.faces).map(|f| (f.id, f)).collect();
        let mut cell_ids = HashSet::default();
        let mut owners: HashMap<u64, Vec<bool>> = HashMap::default();
        // A wall contributes one directed use for EACH incident region.
        let mut fans: HashMap<u64, Vec<(&FaceRecord, bool, &CoedgeRecord)>> = HashMap::default();
        for cell in &self.cells {
            if !cell_ids.insert(cell.id) {
                issues.push(ValidationIssue::error(format!("duplicate cell id {}", cell.id)));
            }
            if cell.faces.is_empty() {
                issues.push(ValidationIssue::error(format!("cell {} has no faces", cell.id)));
            }
            let mut seen = HashSet::default();
            let mut uses: HashMap<u64, Vec<bool>> = HashMap::default();
            let mut holes = 0i64;
            for &(id, forward) in &cell.faces {
                if !seen.insert(id) {
                    issues.push(ValidationIssue::error(format!("cell {} repeats face {id}", cell.id)));
                    continue;
                }
                let Some(&face) = faces.get(&id) else {
                    issues.push(ValidationIssue::error(format!("cell {} references missing face {id}", cell.id)));
                    continue;
                };
                owners.entry(id).or_default().push(forward);
                holes += face.loops.iter().filter(|record| !record.coedges.is_empty()
                    && record.coedges.iter().any(|c| edges.get(&c.edge_id).is_some_and(|e| !e.degenerate)))
                    .count().saturating_sub(1) as i64;
                for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
                    uses.entry(coedge.edge_id).or_default().push(coedge.forward == forward);
                    fans.entry(coedge.edge_id).or_default().push((face, forward, coedge));
                }
            }
            let mut vertices = HashSet::default();
            let mut edge_count = 0i64;
            for (id, senses) in &uses {
                let Some(&edge) = edges.get(id) else { continue; };
                let opposite = senses.len() == 2 && senses[0] != senses[1];
                if !(opposite || (edge.degenerate && senses.len() == 1)) {
                    issues.push(ValidationIssue::error(format!(
                        "cell {} edge {id} is not closed with opposite senses: {senses:?}", cell.id)));
                }
                if !edge.degenerate {
                    edge_count += 1;
                    vertices.extend([edge.start_vertex_id, edge.end_vertex_id]);
                }
            }
            let chi = vertices.len() as i64 - edge_count + seen.len() as i64 - holes;
            if chi % 2 != 0 {
                issues.push(ValidationIssue::error_kind(IssueKind::GenusMismatch,
                    format!("cell {} has odd Euler characteristic {chi}", cell.id)));
            }
        }
        for id in faces.keys() {
            let senses = owners.get(id).map(Vec::as_slice).unwrap_or(&[]);
            if senses.is_empty() || senses.len() > 2 || (senses.len() == 2 && senses[0] == senses[1]) {
                issues.push(ValidationIssue::error(format!(
                    "face {id} has invalid cell ownership senses {senses:?}")));
            }
        }
        for (id, uses) in fans {
            let Some(&edge) = edges.get(&id) else { continue; };
            if edge.degenerate || uses.len() <= 2 { continue; }
            for fraction in [0.25, 0.5, 0.75] {
                let reading = (|| -> Result<Vec<(f64, bool)>, String> {
                    let t = edge.t0 + (edge.t1-edge.t0)*fraction;
                    let derivatives = edge.curve.derivatives(t, 1)?;
                    let tangent = derivatives[1].scale(if edge.t1 >= edge.t0 { 1.0 } else { -1.0 }).normalized()?;
                    let axis = tangent.perpendicular()?;
                    let other = tangent.cross(axis);
                    let mut radial = Vec::new();
                    for &(face, forward, coedge) in &uses {
                        let uv = crate::project_point_to_surface(&face.surface, derivatives[0])?;
                        let normal = face.surface.normal(uv.u, uv.v)?
                            .scale(if face.same_sense { 1.0 } else { -1.0 });
                        // Order face HALF-planes about the edge: coplanar
                        // faces on opposite sides have equal normals but
                        // opposite inward rays. A shared wall contributes
                        // the same ray twice, with opposite cell senses.
                        let ray = normal.cross(tangent.scale(if coedge.forward { 1.0 } else { -1.0 }));
                        let (x,y) = (ray.dot(axis), ray.dot(other));
                        if !x.is_finite() || !y.is_finite() || x*x+y*y < 1e-20 {
                            return Err("undefined radial face tangent".into());
                        }
                        radial.push((y.atan2(x), coedge.forward == forward));
                    }
                    // At a shared-wall ray, close the preceding region
                    // before opening the following one (true then false).
                    radial.sort_by(|a,b| a.0.total_cmp(&b.0).then_with(|| b.1.cmp(&a.1)));
                    Ok(radial)
                })();
                match reading {
                    Ok(radial) if radial.iter().enumerate().all(|(i, a)| a.1 != radial[(i+1)%radial.len()].1) => {},
                    Ok(_) => { issues.push(ValidationIssue::error(format!(
                        "Cellular edge {id} radial cell senses do not alternate at {fraction}"))); break; },
                    Err(error) => { issues.push(ValidationIssue::error(format!(
                        "Cellular edge {id} radial cell fan cannot be read: {error}"))); break; },
                }
            }
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
thread_local! {
    static COEDGE_MEMO: std::cell::RefCell<Option<HashMap<Vec<u64>, Result<f64, String>>>> =
        const { std::cell::RefCell::new(None) };
    /// Reads served from the memo and reads computed, in the open scope.
    static COEDGE_MEMO_COUNTS: std::cell::Cell<(u64, u64)> = const { std::cell::Cell::new((0, 0)) };
}

/// `BREP_VALIDATE_COEDGE_MEMO` (lane X): while a [`CoedgeMemoScope`] is open,
/// `validate` reads each coedge's geometry check once per distinct input.
/// [`coedge_error_floored`] is a pure function of the carrier, the pcurve, the
/// edge curve, the edge's `[t0, t1]`, the sense and the band (validate builds
/// all three NURBS fresh, so no recognition cache reaches it), and the key is
/// every one of those inputs bit for bit, compared in full on a hit: a repeat
/// returns exactly what re-reading would. A boolean validates its result three
/// times (assembly, face merge, the closing check) and most coedges arrive at
/// the later two unchanged. On by default; `0`/`off` disables it.
pub(crate) fn coedge_memo_enabled() -> bool {
    let enabled = !matches!(std::env::var("BREP_VALIDATE_COEDGE_MEMO").as_deref(), Ok("0") | Ok("off"));
    enabled
}


/// The memo's lifetime: one operation. Nested scopes share the outer one.
pub(crate) struct CoedgeMemoScope {
    opened: bool,
}

impl CoedgeMemoScope {
    pub(crate) fn open(enabled: bool) -> Self {
        let opened = enabled
            && COEDGE_MEMO.with(|memo| {
                let mut memo = memo.borrow_mut();
                if memo.is_some() {
                    return false;
                }
                *memo = Some(HashMap::default());
                COEDGE_MEMO_COUNTS.with(|counts| counts.set((0, 0)));
                true
            });
        CoedgeMemoScope { opened }
    }
}

impl Drop for CoedgeMemoScope {
    fn drop(&mut self) {
        if self.opened {
            COEDGE_MEMO.with(|memo| *memo.borrow_mut() = None);
            let (hits, misses) = COEDGE_MEMO_COUNTS.with(|counts| counts.get());
            if std::env::var_os("BREP_PROFILE").is_some() {
                eprintln!("validate.coedge_memo hits={hits} misses={misses}");
            }
        }
    }
}

fn coedge_memo_key(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    tolerance: f64,
) -> Vec<u64> {
    let mut key = Vec::with_capacity(
        16 + surface.knots_u.len() + surface.knots_v.len() + 4 * surface.control_points.iter().map(Vec::len).sum::<usize>()
            + pcurve.knots.len() + 4 * pcurve.control_points.len() + curve.knots.len() + 4 * curve.control_points.len(),
    );
    let push_point = |key: &mut Vec<u64>, point: &crate::Vec4| key.extend([point.x.to_bits(), point.y.to_bits(), point.z.to_bits(), point.w.to_bits()]);
    key.extend([edge.t0.to_bits(), edge.t1.to_bits(), forward as u64, tolerance.to_bits()]);
    key.extend([surface.degree_u as u64, surface.degree_v as u64, surface.knots_u.len() as u64, surface.knots_v.len() as u64, surface.control_points.len() as u64]);
    key.extend(surface.knots_u.iter().chain(&surface.knots_v).map(|knot| knot.to_bits()));
    for row in &surface.control_points {
        key.push(row.len() as u64);
        for point in row {
            push_point(&mut key, point);
        }
    }
    for nurbs in [pcurve, curve] {
        key.extend([nurbs.degree as u64, nurbs.knots.len() as u64, nurbs.control_points.len() as u64]);
        key.extend(nurbs.knots.iter().map(|knot| knot.to_bits()));
        for point in &nurbs.control_points {
            push_point(&mut key, point);
        }
    }
    key
}

/// `BREP_VALIDATE_EXACT_LINES` (lane X): `validate` reads a line coedge on a
/// plane exactly instead of sampling it. On by default; `0`/`off` disables it.
/// Only `validate`'s own per-coedge read takes it: every other caller of
/// [`coedge_error_floored`] / [`adaptive_coedge_error`] keeps the sampler.
pub(crate) fn exact_lines_enabled() -> bool {
    let enabled = !matches!(std::env::var("BREP_VALIDATE_EXACT_LINES").as_deref(), Ok("0") | Ok("off"));
    enabled
}


/// The coedge error of a line edge on a plane, exactly, when the coedge's 3-D
/// curve and its pcurve image are one line BY CONSTRUCTION; `None` sends the
/// coedge to the sampler. The admitted class: an edge curve of degree 1 with
/// two control points of equal weight (linear in `t`); a pcurve of degree 1
/// with two control points of equal weight (linear in its parameter) whose
/// ends lie in the carrier's domain; a carrier of degree 1×1 on a 2×2 net of
/// equal weights whose parallelogram defect is exactly zero (an affine map,
/// no bilinear term). The sampler compares `S(q(q0 + f·Δq))` with
/// `c(t0 + f·Δt)` at fractions `f` of `[0, 1]`; for this class both are affine
/// in `f`, so their difference is too and its largest norm on `[0, 1]` is at
/// an end: the answer is the larger endpoint gap, read by the sampler's own
/// `coedge_sample`. That is the true error; the sampler's reading adds only
/// interior rounding (a box face's ~4e-15 becomes the endpoints' gap, 0 there).
pub(crate) fn exact_line_on_plane_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
) -> Option<f64> {
    let linear = |nurbs: &NurbsCurve| {
        nurbs.degree == 1
            && nurbs.control_points.len() == 2
            && nurbs.knots.len() == 4
            && nurbs.control_points[0].w == nurbs.control_points[1].w
            && nurbs.control_points[0].w > 0.0
    };
    if !linear(pcurve) || !linear(curve) {
        return None;
    }
    if surface.degree_u != 1
        || surface.degree_v != 1
        || surface.knots_u.len() != 4
        || surface.knots_v.len() != 4
        || surface.control_points.len() != 2
        || surface.control_points.iter().any(|row| row.len() != 2)
    {
        return None;
    }
    let weight = surface.control_points[0][0].w;
    if !(weight > 0.0) || surface.control_points.iter().flatten().any(|control| control.w != weight) {
        return None;
    }
    let corner = |i: usize, j: usize| surface.control_points[i][j].point().ok();
    let (p00, p01, p10, p11) = (corner(0, 0)?, corner(0, 1)?, corner(1, 0)?, corner(1, 1)?);
    let defect = p11.sub(p10).sub(p01.sub(p00));
    if defect.x != 0.0 || defect.y != 0.0 || defect.z != 0.0 {
        return None;
    }
    let ([u0, u1], [v0, v1]) = (surface.domain_u().ok()?, surface.domain_v().ok()?);
    let [q0, q1] = pcurve.domain().ok()?;
    for q in [q0, q1] {
        let uv = pcurve.evaluate(q).ok()?;
        if !(uv.x >= u0 && uv.x <= u1 && uv.y >= v0 && uv.y <= v1) {
            return None;
        }
    }
    let [c0, c1] = curve.domain().ok()?;
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    if !(low >= c0 && high <= c1) {
        return None;
    }
    let start = coedge_sample(surface, pcurve, curve, edge, forward, 0.0).ok()?.0;
    let end = coedge_sample(surface, pcurve, curve, edge, forward, 1.0).ok()?.0;
    Some(start.max(end))
}

/// Lane R1 (granted 2026-10-18): the closed-form coedge reading of an
/// iso-parameter line. On by default (promoted 2026-10-18 on the 98 + 144
/// census); `BREP_VALIDATE_EXACT_ISO=0` (or `off`) sends it to the sampler.
pub(crate) fn exact_iso_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| !matches!(std::env::var("BREP_VALIDATE_EXACT_ISO").as_deref(), Ok("0") | Ok("off")))
}


/// The coedge error of an ISO-PARAMETER line, bounded in closed form; `None`
/// sends the coedge to the sampler. The admitted class: a pcurve of degree 1
/// with two control points of equal weight (linear in its parameter) along
/// one iso line of the carrier (`v = c` or `u = c`, the two ends' coordinate
/// equal bit for bit) inside the carrier's domain; an edge curve equal to the
/// carrier's own iso curve there (`iso_curve_v(c)` / `iso_curve_u(c)`
/// rebuilt: degree, knots and weights bit for bit, control points within
/// `1e-12` of the net's size) whose parameter range IS the pcurve's range of
/// the free coordinate, in the coedge's sense. Then the sampler's two points
/// at every fraction are the carrier and the edge curve at one parameter:
/// with the same basis and weights their difference is a convex combination
/// of the control points' differences, so the error is at most the largest
/// of those, and at least the ends' gaps — the answer is the larger of the
/// two (`coedge_sample` reads the ends). That bounds the true error; the
/// sampler's own reading adds only the interior rounding of evaluating one
/// point two ways (2e-15 on a rim fillet's rail), as for
/// [`exact_line_on_plane_error`].
pub(crate) fn exact_iso_line_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
) -> Option<f64> {
    if pcurve.degree != 1
        || pcurve.control_points.len() != 2
        || pcurve.knots.len() != 4
        || pcurve.control_points[0].w != pcurve.control_points[1].w
        || !(pcurve.control_points[0].w > 0.0)
    {
        return None;
    }
    let [q0, q1] = pcurve.domain().ok()?;
    let (a, b) = (pcurve.evaluate(q0).ok()?, pcurve.evaluate(q1).ok()?);
    let ([u0, u1], [v0, v1]) = (surface.domain_u().ok()?, surface.domain_v().ok()?);
    for uv in [a, b] {
        if !(uv.x >= u0 && uv.x <= u1 && uv.y >= v0 && uv.y <= v1) {
            return None;
        }
    }
    // Which coordinate is fixed, and the free one's range along the pcurve.
    let (iso, from, to) = if a.y.to_bits() == b.y.to_bits() && a.x != b.x {
        (surface.iso_curve_v(a.y).ok()?, a.x, b.x)
    } else if a.x.to_bits() == b.x.to_bits() && a.y != b.y {
        (surface.iso_curve_u(a.x).ok()?, a.y, b.y)
    } else {
        return None;
    };
    // The edge's parameter at fraction f must be the free coordinate there.
    let (start, end) = if forward { (edge.t0, edge.t1) } else { (edge.t1, edge.t0) };
    if start.to_bits() != from.to_bits() || end.to_bits() != to.to_bits() {
        return None;
    }
    if iso.degree != curve.degree
        || iso.knots.len() != curve.knots.len()
        || iso.knots.iter().zip(&curve.knots).any(|(x, y)| x.to_bits() != y.to_bits())
        || iso.control_points.len() != curve.control_points.len()
    {
        return None;
    }
    let size = iso
        .control_points
        .iter()
        .filter_map(|c| c.point().ok())
        .fold(0f64, |m, q| m.max(q.x.abs()).max(q.y.abs()).max(q.z.abs()));
    let mut deviation = 0f64;
    for (p, q) in iso.control_points.iter().zip(&curve.control_points) {
        if p.w.to_bits() != q.w.to_bits() || !(p.w > 0.0) {
            return None;
        }
        deviation = deviation.max(p.point().ok()?.sub(q.point().ok()?).length());
    }
    if !(deviation <= 1e-12 * size.max(1.0)) {
        return None;
    }
    let start_gap = coedge_sample(surface, pcurve, curve, edge, forward, 0.0).ok()?.0;
    let end_gap = coedge_sample(surface, pcurve, curve, edge, forward, 1.0).ok()?.0;
    Some(start_gap.max(end_gap).max(deviation))
}

/// [`coedge_error_floored`] through the operation's memo when one is open.
fn memoized_coedge_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    tolerance: f64,
) -> Result<f64, String> {
    if exact_lines_enabled() {
        if let Some(error) = exact_line_on_plane_error(surface, pcurve, curve, edge, forward) {
            return Ok(error);
        }
    }
    if exact_iso_enabled() {
        if let Some(error) = exact_iso_line_error(surface, pcurve, curve, edge, forward) {
            crate::blend::record_count("count.validate.exact_iso", 1);
            return Ok(error);
        }
    }
    if COEDGE_MEMO.with(|memo| memo.borrow().is_none()) {
        return coedge_error_floored(surface, pcurve, curve, edge, forward, tolerance);
    }
    let key = coedge_memo_key(surface, pcurve, curve, edge, forward, tolerance);
    if let Some(hit) = COEDGE_MEMO.with(|memo| memo.borrow().as_ref().and_then(|map| map.get(&key).cloned())) {
        COEDGE_MEMO_COUNTS.with(|counts| counts.set((counts.get().0 + 1, counts.get().1)));
        return hit;
    }
    COEDGE_MEMO_COUNTS.with(|counts| counts.set((counts.get().0, counts.get().1 + 1)));
    let read = coedge_error_floored(surface, pcurve, curve, edge, forward, tolerance);
    COEDGE_MEMO.with(|memo| {
        if let Some(map) = memo.borrow_mut().as_mut() {
            map.insert(key, read.clone());
        }
    });
    read
}

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

/// Bound on the clean-verdict memo; the set is emptied when it would pass it.
const VALIDATED_CLEAN_CAP: usize = 4096;

thread_local! {
    static VALIDATED_CLEAN: std::cell::RefCell<rustc_hash::FxHashSet<(u64, u64)>> =
        std::cell::RefCell::new(rustc_hash::FxHashSet::default());
}

impl BrepSolid {
    /// Empty the clean-verdict memo of [`BrepSolid::validate_memoised`]:
    /// called at a fillet operation's entry, so each operation pays for the
    /// first read of every body it validates.
    pub(crate) fn clear_validate_memo() {
        VALIDATED_CLEAN.with(|clean| clean.borrow_mut().clear());
    }

    /// The body's memo key: the 128-bit two-lane content hash
    /// (`mass_properties::TwoLane`, bit-exact on every float) over EVERY
    /// field the body serialises — ids, names, kind, genus, vertices, edges,
    /// shells, faces, loops, coedges, cells — so any mutation is a new key.
    /// (A JSON serialisation of the body read the same content at ~10× the
    /// cost: float formatting.)
    pub(crate) fn validate_memo_key(&self) -> Option<(u64, u64)> {
        use crate::mass_properties::TwoLane;
        fn curve(lane: &mut TwoLane, curve: &crate::NurbsCurve) {
            lane.usize(curve.degree);
            lane.floats(&curve.knots);
            lane.usize(curve.control_points.len());
            for point in &curve.control_points {
                lane.floats(&[point.x, point.y, point.z, point.w]);
            }
        }
        fn name(lane: &mut TwoLane, name: &Option<String>) {
            lane.bool(name.is_some());
            if let Some(name) = name {
                lane.usize(name.len());
                for byte in name.bytes() {
                    lane.u64(u64::from(byte));
                }
            }
        }
        let mut lane = TwoLane::new();
        lane.u64(self.id);
        lane.u64(self.genus as u64);
        lane.usize(self.kind as usize);
        lane.usize(self.vertices.len());
        for vertex in &self.vertices {
            lane.u64(vertex.id);
            lane.floats(&[vertex.point.x, vertex.point.y, vertex.point.z]);
        }
        lane.usize(self.edges.len());
        for edge in &self.edges {
            lane.u64(edge.id);
            curve(&mut lane, &edge.curve);
            lane.floats(&[edge.t0, edge.t1]);
            lane.u64(edge.start_vertex_id);
            lane.u64(edge.end_vertex_id);
            lane.bool(edge.degenerate);
            name(&mut lane, &edge.name);
        }
        lane.usize(self.shells.len());
        for shell in &self.shells {
            lane.u64(shell.id);
            lane.usize(shell.faces.len());
            for face in &shell.faces {
                lane.u64(face.id);
                crate::mass_properties::hash_surface(&mut lane, &face.surface);
                lane.bool(face.same_sense);
                name(&mut lane, &face.name);
                lane.usize(face.loops.len());
                for loop_record in &face.loops {
                    lane.u64(loop_record.id);
                    lane.usize(loop_record.coedges.len());
                    for coedge in &loop_record.coedges {
                        lane.u64(coedge.id);
                        lane.u64(coedge.edge_id);
                        lane.bool(coedge.forward);
                        curve(&mut lane, &coedge.pcurve);
                    }
                }
            }
        }
        lane.usize(self.cells.len());
        for cell in &self.cells {
            lane.u64(cell.id);
            lane.bool(cell.void);
            lane.usize(cell.faces.len());
            for (face, sense) in &cell.faces {
                lane.u64(*face);
                lane.bool(*sense);
            }
        }
        let [a, b] = lane.finish();
        Some((a, b))
    }

}

fn validate_memo_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| !matches!(std::env::var("BREP_VALIDATE_MEMO").as_deref(), Ok("0") | Ok("off")))
}


