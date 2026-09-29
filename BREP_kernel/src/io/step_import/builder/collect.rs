use super::*;

impl<'a> SolidBuilder<'a> {
    pub(super) fn fresh(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Append a vertex and record its id -> index. Vertices are only ever
    /// appended, so the index stays valid for the record's lifetime.
    pub(super) fn push_vertex(&mut self, vertex: VertexRecord) {
        self.vertex_index.insert(vertex.id, self.vertices.len());
        self.vertices.push(vertex);
    }

    /// Append an edge and record its id -> index.
    pub(super) fn push_edge(&mut self, edge: EdgeRecord) {
        self.edge_index.insert(edge.id, self.edges.len());
        self.edges.push(edge);
    }

    /// Rebuild the edge id -> index map from scratch. Called after the one place
    /// that shifts edge positions (`split_edge_at`'s in-place reassign + insert).
    pub(super) fn rebuild_edge_index(&mut self) {
        self.edge_index.clear();
        for (index, edge) in self.edges.iter().enumerate() {
            self.edge_index.insert(edge.id, index);
        }
    }

    fn vertex(&mut self, vertex_ref: usize) -> Result<u64, String> {
        if let Some(id) = self.vertex_of_ref.get(&vertex_ref) {
            return Ok(*id);
        }
        let entity = self.resolver.get(vertex_ref)?;
        let point_ref = entity
            .find("VERTEX_POINT")
            .and_then(|args| args.get(1))
            .ok_or("step_import: VERTEX_POINT missing geometry")?
            .as_ref_id()?;
        let point = self.resolver.point(point_ref)?;
        // OCC writes ref-distinct duplicate VERTEX_POINTs at the same
        // location (across sub-shells and reused curves). Weld them: loops
        // chain by kernel vertex identity, so coordinate-equal duplicates
        // tear otherwise-closed loops apart at validation.
        let weld_tolerance = if std::env::var("BREP_STEP_NO_WELD").is_ok() {
            -1.0
        } else {
            IMPORT_IDENTITY_TOLERANCE
        };
        if let Some(existing) = self
            .vertices
            .iter()
            .find(|vertex| vertex.point.sub(point).length() <= weld_tolerance)
        {
            let id = existing.id;
            self.vertex_of_ref.insert(vertex_ref, id);
            return Ok(id);
        }
        let id = self.fresh();
        self.push_vertex(VertexRecord { id, point });
        self.vertex_of_ref.insert(vertex_ref, id);
        Ok(id)
    }

    fn edge(&mut self, edge_ref: usize) -> Result<u64, String> {
        if let Some(id) = self.edge_of_ref.get(&edge_ref) {
            return Ok(*id);
        }
        let entity = self.resolver.get(edge_ref)?;
        let args = entity
            .find("EDGE_CURVE")
            .ok_or_else(|| format!("step_import: #{edge_ref} is not an EDGE_CURVE"))?;
        let start_ref = args[1].as_ref_id()?;
        let end_ref = args[2].as_ref_id()?;
        let curve_ref = args[3].as_ref_id()?;
        let same_sense = args.get(4).map(|value| !value.enum_is("F")).unwrap_or(true);

        let start_vertex_id = self.vertex(start_ref)?;
        let end_vertex_id = self.vertex(end_ref)?;
        // Curve trimming follows the raw STEP endpoint geometry. Vertex welding
        // is a separate topology repair and must not erase a real microscopic
        // arc between ref-distinct, coordinate-close VERTEX_POINTs.
        let p_start = self.resolver.step_vertex_point(start_ref)?;
        let p_end = self.resolver.step_vertex_point(end_ref)?;
        let curve = self.resolver.resolve_edge_curve(
            curve_ref,
            p_start,
            p_end,
            same_sense,
            start_ref == end_ref,
        )?;
        let written = curve;
        let curve = heal_imported_edge_endpoints(&written, p_start, p_end)?;
        let [t0, t1] = curve.domain()?;
        // A zero-length edge on one vertex is a POLE loop (e.g. a countersink
        // cone apex written as a real EDGE_CURVE instead of a VERTEX_LOOP —
        // boxy_with_diamsize). Every pole/rim path keys on `degenerate`;
        // leaving it false strands the wall with an unstitched point "rim"
        // and its triangulation sprays across the periodic domain.
        let degenerate = start_vertex_id == end_vertex_id && {
            let welded_point = self.vertex_point(start_vertex_id);
            let tolerance = IMPORT_IDENTITY_TOLERANCE;
            let mut collapsed = true;
            for index in 1..8 {
                let t = t0 + (t1 - t0) * index as f64 / 8.0;
                if curve.evaluate(t)?.sub(welded_point).length() > tolerance {
                    collapsed = false;
                    break;
                }
            }
            collapsed
        };
        let id = self.fresh();
        self.push_edge(EdgeRecord {
            id,
            curve,
            t0,
            t1,
            start_vertex_id,
            end_vertex_id,
            degenerate,
            name: None,
        });
        self.edge_of_ref.insert(edge_ref, id);
        self.curve_ref_of_edge.insert(id, curve_ref);
        if self.readings.is_some() {
            self.capture_written(id, Some(curve_ref), &written)?;
            let [w0, w1] = written.domain()?;
            let edge = self.edge_record(id);
            let moved = written.evaluate(w0)?.sub(edge.curve.evaluate(t0)?).length()
                + written.evaluate(w1)?.sub(edge.curve.evaluate(t1)?).length();
            if moved > 0.0 {
                self.capture_stage(id, || {
                    format!("endpoints healed onto the vertices ({moved:.3e})")
                });
            }
        }
        Ok(id)
    }

    pub(super) fn edge_record(&self, edge_id: u64) -> &EdgeRecord {
        let index = *self.edge_index.get(&edge_id).expect("edge id must exist");
        &self.edges[index]
    }

    pub(super) fn vertex_point(&self, vertex_id: u64) -> Vec3 {
        let index = *self
            .vertex_index
            .get(&vertex_id)
            .expect("vertex id must exist");
        self.vertices[index].point
    }

    /// A FACETED_BREP vertex reference (an entry of a POLY_LOOP): either a
    /// VERTEX_POINT or, per ISO 10303-42, a bare CARTESIAN_POINT. Welded by the
    /// entity id so every polygon sharing that point reuses one `VertexRecord`,
    /// which is what makes the faceted shell watertight.
    fn faceted_vertex(&mut self, point_ref: usize) -> Result<u64, String> {
        if let Some(id) = self.vertex_of_ref.get(&point_ref) {
            return Ok(*id);
        }
        let entity = self.resolver.get(point_ref)?;
        let point = if entity.has("VERTEX_POINT") {
            let geom = entity
                .find("VERTEX_POINT")
                .and_then(|args| args.get(1))
                .ok_or("step_import: VERTEX_POINT missing geometry")?
                .as_ref_id()?;
            self.resolver.point(geom)?
        } else if entity.has("CARTESIAN_POINT") {
            self.resolver.point(point_ref)?
        } else {
            return Err(format!(
                "step_import: POLY_LOOP entry #{point_ref} is not a VERTEX_POINT/CARTESIAN_POINT"
            ));
        };
        let id = self.fresh();
        self.push_vertex(VertexRecord { id, point });
        self.vertex_of_ref.insert(point_ref, id);
        Ok(id)
    }

    /// Get (or synthesize) the implicit straight edge between two welded polygon
    /// vertices, returning `(edge_id, forward)` where `forward` is true iff the
    /// stored edge runs `a -> b`. The first face to touch the pair defines the
    /// edge's direction; the neighbouring face, walking it the other way, reuses
    /// the same edge with `forward = false` — giving each edge exactly two
    /// opposite-sense coedges (the manifold shell the validator requires).
    pub(super) fn faceted_edge(&mut self, a: u64, b: u64) -> Result<(u64, bool), String> {
        let key = (a.min(b), a.max(b));
        if let Some(&edge_id) = self.edge_of_vertex_pair.get(&key) {
            let forward = self.edge_record(edge_id).start_vertex_id == a;
            return Ok((edge_id, forward));
        }
        let p_start = self.vertex_point(a);
        let p_end = self.vertex_point(b);
        let curve = make_line(p_start, p_end)?;
        let [t0, t1] = curve.domain()?;
        let id = self.fresh();
        self.push_edge(EdgeRecord {
            id,
            curve,
            t0,
            t1,
            start_vertex_id: a,
            end_vertex_id: b,
            degenerate: false,
            name: None,
        });
        self.edge_of_vertex_pair.insert(key, id);
        let line = self.edge_record(id).curve.clone();
        self.capture_written(id, None, &line)?;
        Ok((id, true))
    }

    /// Resolve a POLY_LOOP (an ordered CARTESIAN_POINT/VERTEX_POINT polygon whose
    /// edges are the implicit straight lines between consecutive points, closing
    /// pn -> p1) into ordered `(edge_id, forward)` coedge specs over welded,
    /// shared edges.
    fn faceted_loop_specs(&mut self, loop_ref: usize) -> Result<Vec<(u64, bool)>, String> {
        let entity = self.resolver.get(loop_ref)?;
        let point_refs = entity
            .find("POLY_LOOP")
            .and_then(|args| args.get(1))
            .ok_or("step_import: POLY_LOOP missing point list")?
            .as_list()?
            .to_vec();
        let mut vertices: Vec<u64> = Vec::with_capacity(point_refs.len());
        for value in &point_refs {
            let vertex_id = self.faceted_vertex(value.as_ref_id()?)?;
            // Drop consecutive duplicate points defensively (a repeated vertex
            // would otherwise make a zero-length edge).
            if vertices.last() != Some(&vertex_id) {
                vertices.push(vertex_id);
            }
        }
        // Close the ring: also drop a trailing point equal to the first.
        if vertices.len() > 1 && vertices.first() == vertices.last() {
            vertices.pop();
        }
        if vertices.len() < 3 {
            return Err(format!(
                "step_import: degenerate POLY_LOOP #{loop_ref} ({} distinct points)",
                vertices.len()
            ));
        }
        let count = vertices.len();
        let mut specs = Vec::with_capacity(count);
        for index in 0..count {
            let a = vertices[index];
            let b = vertices[(index + 1) % count];
            specs.push(self.faceted_edge(a, b)?);
        }
        Ok(specs)
    }

    /// Resolve an EDGE_LOOP or POLY_LOOP into ordered `(edge_id, forward)` coedge
    /// specs. POLY_LOOP (FACETED_BREP) polygons are handled by the faceted path
    /// (implicit straight edges, welded across faces); EDGE_LOOP uses the
    /// ORIENTED_EDGE/EDGE_CURVE topology.
    fn loop_specs(&mut self, loop_ref: usize) -> Result<Vec<(u64, bool)>, String> {
        let entity = self.resolver.get(loop_ref)?;
        if entity.has("POLY_LOOP") {
            return self.faceted_loop_specs(loop_ref);
        }
        // A VERTEX_LOOP is the STEP representation of a boundary collapsed to
        // one point (most commonly the apex of a cone).  The kernel models the
        // same topology as a single-use degenerate edge so it can retain a
        // pcurve and participate in the face's loop like synthesized pole
        // boundaries do.
        if let Some(args) = entity.find("VERTEX_LOOP") {
            let vertex_ref = args
                .get(1)
                .ok_or("step_import: VERTEX_LOOP missing loop vertex")?
                .as_ref_id()?;
            let vertex_id = self.vertex(vertex_ref)?;
            let point = self.vertex_point(vertex_id);
            let curve = make_line(point, point)?;
            let [t0, t1] = curve.domain()?;
            let edge_id = self.fresh();
            self.push_edge(EdgeRecord {
                id: edge_id,
                curve,
                t0,
                t1,
                start_vertex_id: vertex_id,
                end_vertex_id: vertex_id,
                degenerate: true,
                name: None,
            });
            return Ok(vec![(edge_id, true)]);
        }
        let oriented = entity
            .find("EDGE_LOOP")
            .and_then(|args| args.get(1))
            .ok_or_else(|| {
                format!("step_import: loop #{loop_ref} is not an EDGE_LOOP/POLY_LOOP/VERTEX_LOOP")
            })?
            .as_list()?;
        let mut specs = Vec::with_capacity(oriented.len());
        for value in oriented {
            let oe = self.resolver.get(value.as_ref_id()?)?;
            let args = oe
                .find("ORIENTED_EDGE")
                .ok_or("step_import: expected ORIENTED_EDGE")?;
            let edge_ref = args[3].as_ref_id()?;
            let forward = !args[4].enum_is("F");
            let edge_id = self.edge(edge_ref)?;
            specs.push((edge_id, forward));
        }
        Ok(specs)
    }

    /// Phase 1 of face construction: resolve the ADVANCED_FACE's carrier
    /// surface and its ordered coedge bounds, but defer pcurve/loop assembly.
    /// Deferring lets a global pre-pass (`relocate_periodic_rim_seams`) first
    /// re-seat OCC full-cylinder rim circles onto the surface seam meridian —
    /// a mutation of shared edges/vertices that must happen before any face
    /// derives a pcurve from them (caps can precede their walls in the shell).
    pub(super) fn collect_face(&mut self, face_ref: usize) -> Result<PendingFace, String> {
        let entity = self.resolver.get(face_ref)?;
        // ADVANCED_FACE is a subtype of FACE_SURFACE with the identical instance
        // layout — (name, bounds, face_geometry, same_sense) — so both resolve
        // through the same coedge/surface path. SHELL_BASED_SURFACE_MODEL bodies
        // routinely carry plain FACE_SURFACE faces; any face whose bounds or
        // geometry aren't the shapes below still fails downstream (and is caught).
        let args = entity
            .find("ADVANCED_FACE")
            .or_else(|| entity.find("FACE_SURFACE"))
            .ok_or_else(|| {
                format!("step_import: #{face_ref} is not an ADVANCED_FACE/FACE_SURFACE")
            })?;
        let bound_refs = args[1].as_list()?.to_vec();
        let surface_ref = args[2].as_ref_id()?;
        let face_same_sense = !args[3].enum_is("F");

        // Build the edge topology first so analytic surfaces (whose STEP form
        // is unbounded) can be sized from the face's actual edge geometry.
        let mut bounds: Vec<(Vec<(u64, bool)>, bool)> = Vec::new();
        let mut samples: Vec<Vec3> = Vec::new();
        for bound_value in &bound_refs {
            let bound_ref = bound_value.as_ref_id()?;
            let bound = self.resolver.get(bound_ref)?;
            let (bargs, outer) = if let Some(a) = bound.find("FACE_OUTER_BOUND") {
                (a, true)
            } else if let Some(a) = bound.find("FACE_BOUND") {
                (a, false)
            } else {
                return Err(format!(
                    "step_import: face bound #{bound_ref} is not FACE_(OUTER_)BOUND"
                ));
            };
            let loop_ref = bargs[1].as_ref_id()?;
            let orientation = !bargs[2].enum_is("F");
            let mut specs = self.loop_specs(loop_ref)?;
            if !orientation {
                specs.reverse();
                for spec in &mut specs {
                    spec.1 = !spec.1;
                }
            }
            for (edge_id, _) in &specs {
                let edge = self.edge_record(*edge_id);
                // Dense sampling: a rational conic's parameter is non-uniform
                // and a vendor spline can DIP sharply between stations (a
                // pocket-bottom arc's low point sags ~quadratically between
                // samples), so sparse sampling undersizes an analytic
                // surface's covering domain and projections clamp at the
                // domain edge. 128 spans keep the worst sag well inside the
                // axial-extent margin.
                for index in 0..=128 {
                    let t = edge.t0 + (edge.t1 - edge.t0) * index as f64 / 128.0;
                    if let Ok(point) = edge.curve.evaluate(t) {
                        samples.push(point);
                    }
                }
            }
            bounds.push((specs, outer));
        }
        bounds.sort_by_key(|(_, outer)| !*outer); // outer bound(s) first

        // A pole-to-pole great circle divides a sphere into two exactly equal
        // angular gaps. Geometry-only reseaming cannot distinguish them: the
        // historical largest-gap tie picked whichever half happened to sort
        // first, which can put the carrier seam INSIDE the authored material
        // and make every downstream region consumer select the complementary
        // hemisphere. In this exact, standards-unambiguous case derive the
        // excluded half from the oriented STEP boundary. Material is left of
        // the coedge traversal on the oriented face, so `tangent × normal`
        // points into the excluded hemisphere; seating the periodic seam there
        // makes the authored half one ordinary in-domain trim for tessellation,
        // mass properties, and point classification alike.
        let sphere_seam = self.equal_gap_sphere_seam(surface_ref, &bounds, face_same_sense)?;
        let surface = self
            .resolver
            .surface_for_face(surface_ref, &samples, sphere_seam)?;
        // Route a vendor v-periodic POLE-CAP surface of revolution (angle on v,
        // meridian on u, capped by a degenerate VERTEX_LOOP apex) through the
        // kernel's universal u-periodic convention so the rim/apex loops stitch
        // and the face integrates its polar cap (not the base-side complement).
        // Gated on the apex loop: a v-periodic BAND between two rims (no pole)
        // is already handled by the general seam paths, and re-seating its
        // shared rims here would perturb neighbour pcurves (ABC 00000010).
        let has_apex_loop = bounds
            .iter()
            .any(|(specs, _)| specs.len() == 1 && self.edge_record(specs[0].0).degenerate);
        let surface = if has_apex_loop {
            u_periodic_normalized(surface)?
        } else {
            surface
        };

        Ok(PendingFace {
            face_ref,
            surface,
            surface_ref,
            same_sense: face_same_sense,
            bounds,
        })
    }

    /// Preferred periodic seam for the sole ambiguous spherical trim: one
    /// oriented, closed great circle whose plane contains both carrier poles.
    /// All gates are geometric/topological; a partial small circle, multi-loop
    /// sphere, non-outer bound, off-sphere curve, or non-pole-to-pole circle
    /// retains the historical geometry-only reseaming unchanged.
    fn equal_gap_sphere_seam(
        &self,
        surface_ref: usize,
        bounds: &[(Vec<(u64, bool)>, bool)],
        face_same_sense: bool,
    ) -> Result<Option<Vec3>, String> {
        let entity = self.resolver.get(surface_ref)?;
        let Some(args) = entity.find("SPHERICAL_SURFACE") else {
            return Ok(None);
        };
        let frame = self.resolver.placement(args[1].as_ref_id()?)?;
        // mm, like the sample geometry this seam derivation is compared with
        // (a raw native-unit radius here silently flipped the hemisphere pick
        // for metre-authored files once coordinates were converted).
        let radius = self.resolver.length(args[2].as_real()?.abs());
        if radius <= 1e-12 || bounds.len() != 1 || !bounds[0].1 {
            return Ok(None);
        }
        if bounds[0].0.len() == 1 {
            let (edge_id, forward) = bounds[0].0[0];
            let edge = self.edge_record(edge_id);
            if !edge.degenerate && edge.start_vertex_id == edge.end_vertex_id {
                if let Some(seam) =
                    equal_gap_great_circle_seam(&frame, radius, edge, forward, face_same_sense)?
                {
                    return Ok(Some(seam));
                }
            }
        }
        let edges = bounds[0]
            .0
            .iter()
            .map(|(edge_id, forward)| (self.edge_record(*edge_id), *forward))
            .collect::<Vec<_>>();
        interior_pole_loop_seam(&frame, radius, &edges, face_same_sense)
    }

    /// Phase 2 of face construction: stitch OCC full-cylinder rim loops and
    /// derive each loop's pcurves. Runs after `relocate_periodic_rim_seams`,
    /// so any misaligned rim circles have already been re-seated onto the seam
    /// and the (existing) aligned stitch path applies uniformly.
    pub(super) fn finish_face(&mut self, pending: PendingFace) -> Result<FaceRecord, String> {
        let PendingFace {
            face_ref,
            surface,
            surface_ref,
            same_sense,
            mut bounds,
        } = pending;

        // OCC/FreeCAD represent a full cylinder wall as TWO separate full-circle
        // loops (top + bottom) with no seam ruling; the kernel needs ONE loop
        // with explicit seam edges. Stitch them before building pcurves.
        self.stitch_seam_circle_loops(&surface, &mut bounds)?;

        let mut loops = Vec::with_capacity(bounds.len());
        for (specs, _) in bounds {
            let loop_record = self.build_loop(&surface, face_ref, surface_ref, &specs)?;
            loops.push(loop_record);
        }

        let id = self.fresh();
        self.capture_face(id, face_ref, surface_ref);
        let mut face = FaceRecord {
            id,
            surface,
            same_sense,
            loops,
            name: None,
        };
        if let Some(outer) = face.loops.first() {
            let mut uses: HashMap<u64, usize> = HashMap::default();
            for coedge in &outer.coedges {
                *uses.entry(coedge.edge_id).or_default() += 1;
            }
            // Two point bounds carry no UV winding. Their synthetic meridian
            // pair must orient the full-domain outer boundary with the face;
            // otherwise real hole loops can add to it instead of subtracting.
            // This loop has no ordinary shared rim whose sense we could alter:
            // every non-pole edge is the doubled, face-local seam.
            let poles = outer
                .coedges
                .iter()
                .filter(|c| self.edge_record(c.edge_id).degenerate)
                .count();
            if poles >= 2
                && outer.coedges.iter().all(|c| {
                    self.edge_record(c.edge_id).degenerate || uses[&c.edge_id] == 2
                })
            {
                let mut outer_face = face.clone();
                outer_face.loops.truncate(1);
                let area = crate::parameter_space_area(&outer_face)?;
                if area != 0.0 && (area > 0.0) != same_sense {
                    let outer = &mut face.loops[0];
                    outer.coedges.reverse();
                    for coedge in &mut outer.coedges {
                        if !self.edge_record(coedge.edge_id).degenerate {
                            coedge.forward = !coedge.forward;
                        }
                        coedge.pcurve = coedge.pcurve.reversed()?;
                    }
                }
            }
        }
        Ok(face)
    }
}

