use super::*;

/// Triangulate ONE planar region — an outer boundary minus its holes — supplied
/// as `(uv, world)` boundary vertices already projected onto the region's plane.
/// Reuses the watertight hole-bridging + ear clipping (the SAME planar path a
/// real face's trim loops take), so a synthesized sketch SHEET meshes exactly
/// like a kernel face. Winding is normalized here (outer CCW, holes CW in uv) so
/// the caller need not pre-orient. Returns world-space triangles (three corners
/// each); an under-3-vertex boundary or a failed bridge yields no triangles.
pub fn triangulate_planar_region(
    outer: &[([f64; 2], Vec3)],
    holes: &[Vec<([f64; 2], Vec3)>],
) -> Vec<[Vec3; 3]> {
    if outer.len() < 3 {
        return Vec::new();
    }
    let to_chain = |pts: &[([f64; 2], Vec3)]| -> Vec<FaceVertex> {
        pts.iter()
            .map(|(uv, position)| FaceVertex {
                uv: *uv,
                position: *position,
            })
            .collect()
    };
    // ear_clip consumes ears of POSITIVE signed area — i.e. a CCW outer ring.
    let mut outer_chain = to_chain(outer);
    if signed_area(&outer_chain) < 0.0 {
        outer_chain.reverse();
    }
    // Holes wind OPPOSITE the outer for the classic two-way bridge to stay simple.
    let hole_chains: Vec<Vec<FaceVertex>> = holes
        .iter()
        .filter(|hole| hole.len() >= 3)
        .map(|hole| {
            let mut chain = to_chain(hole);
            if signed_area(&chain) > 0.0 {
                chain.reverse();
            }
            chain
        })
        .collect();
    let bridged = bridge_holes(outer_chain, hole_chains);
    if bridged.len() < 3 {
        return Vec::new();
    }
    let ring: Vec<usize> = (0..bridged.len()).collect();
    let mut indices = ring.clone();
    let guard = RegionGuard {
        vertices: &bridged,
        ring: &ring,
        max_split_span: None,
    };
    ear_clip(&bridged, &mut indices, &guard)
        .into_iter()
        .map(|[a, b, c]| [bridged[a].position, bridged[b].position, bridged[c].position])
        .collect()
}

/// Parameter-space bounding box `[u0, u1, v0, v1]` of a boundary polygon. A
/// periodic region lifted past the domain end is still a rectangle in its OWN
/// uv box, and that box — not the surface domain — is what interior seeding
/// must span (the domain would clip away everything past the seam).
fn region_uv_box(vertices: &[FaceVertex]) -> [f64; 4] {
    let mut bounds = [
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ];
    for vertex in vertices {
        bounds[0] = bounds[0].min(vertex.uv[0]);
        bounds[1] = bounds[1].max(vertex.uv[0]);
        bounds[2] = bounds[2].min(vertex.uv[1]);
        bounds[3] = bounds[3].max(vertex.uv[1]);
    }
    bounds
}

pub(super) fn tessellate_face_watertight(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord_tolerance: f64,
    face_id: u32,
    mesh: &mut Mesh,
) -> Result<(), String> {
    // A SPHERE is meshed in the pole-free cube atlas instead of its polar domain
    // (`sphere_atlas.rs`): the two degenerate poles and the periodic seam are
    // artefacts of that domain, not of the geometry, and every helper below
    // exists to work around them. The lane declines — leaving this face to the
    // paths below, bit-for-bit — for any surface that is not a sphere and for any
    // sphere whose charts it could not triangulate cleanly.
    if tessellate_spherical_face_watertight(face, samples, chord_tolerance, face_id, mesh)? {
        return Ok(());
    }
    let ku = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let (closed_u, closed_v) = face.surface.closed_directions()?;
    // A torus fillet band whose loop rides one seam while wrapping the other
    // hops between opposite domain corners; sampled raw, its (u,v) boundary
    // self-crosses and ear-clipping sprays triangles across the whole domain.
    // Unwrap the implicit seam onto the covering plane (per-coedge offsets), so
    // each loop is one continuous polygon; the generic outer/hole + ear-clip
    // path then meshes it, refined through the domain-wrapping evaluator with
    // normals reduced back into the domain. Gated to doubly-periodic surfaces
    // with a genuine seam hop whose unwrapped outer loop is simple; everything
    // else keeps its existing chains bit-for-bit.
    let mut biperiodic_unwrapped = false;
    let mut chains = Vec::with_capacity(face.loops.len());
    // Unwrap an implicit periodic seam onto the covering plane whenever the
    // surface is periodic in EITHER direction (a torus in both). A trimmed
    // periodic face whose loops HOP the seam (a cylinder wall pierced by holes,
    // whose outer wire runs top-rim → seam → bottom-rim → seam and whose hole
    // loops straddle u=0) otherwise falls through every clean-rim helper to the
    // generic bridge/ear-clip path, which sprays domain-spanning triangles across
    // the seam — the double-cover that renders inside-out. Unwrapping folds those
    // hops out so each loop is one continuous covering-plane polygon the shared
    // bridge+ear-clip+domain-wrapping-refine path meshes cleanly, with the seam
    // an ordinary interior edge (one shared vertex column, no double cover).
    //
    // A CLEAN full rim nets a whole wrap, for which `loop_seam_offsets` returns
    // all-zero (topology.rs) — so a seamless cylinder/torus produces `any_offset
    // == false` and keeps its existing `close_periodic_*` path bit-for-bit. Only
    // faces with a genuine seam HOP take the unwrapped path.
    if closed_u || closed_v {
        let mut unwrapped = Vec::with_capacity(face.loops.len());
        let mut any_offset = false;
        for loop_record in &face.loops {
            let offsets = crate::topology::loop_seam_offsets(
                &loop_record.coedges,
                closed_u,
                closed_v,
                u1 - u0,
                v1 - v0,
            )?;
            if offsets.iter().any(|o| o[0] != 0.0 || o[1] != 0.0) {
                any_offset = true;
            }
            unwrapped.push(loop_chain_offset(&loop_record.coedges, samples, &offsets)?);
        }
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            let outer_simple = unwrapped
                .iter()
                .max_by(|a, b| signed_area(a).abs().total_cmp(&signed_area(b).abs()))
                .map(|chain| polygon_is_simple(chain))
                .unwrap_or(false);
            eprintln!(
                "[DISPATCH] face {face_id}: closed=(u:{closed_u},v:{closed_v}) loops={} any_offset={any_offset} outer_simple={outer_simple}",
                face.loops.len()
            );
            for (i, chain) in unwrapped.iter().enumerate() {
                let mut maxdu = 0.0f64;
                let mut maxdv = 0.0f64;
                for w in chain.windows(2) {
                    maxdu = maxdu.max((w[1].uv[0] - w[0].uv[0]).abs());
                    maxdv = maxdv.max((w[1].uv[1] - w[0].uv[1]).abs());
                }
                eprintln!(
                    "    loop {i}: n={} |area|={:.3e} simple={} maxdu={maxdu:.4} maxdv={maxdv:.4} periodU={:.4} periodV={:.4}",
                    chain.len(),
                    signed_area(chain).abs(),
                    polygon_is_simple(chain),
                    u1 - u0,
                    v1 - v0,
                );
            }
        }
        if any_offset {
            let outer = unwrapped
                .iter()
                .max_by(|a, b| signed_area(a).abs().total_cmp(&signed_area(b).abs()));
            if outer.map(|chain| polygon_is_simple(chain)).unwrap_or(false) {
                chains = unwrapped;
                biperiodic_unwrapped = true;
            }
        }
    }
    if chains.is_empty() {
        for loop_record in &face.loops {
            chains.push(loop_chain(&loop_record.coedges, samples)?);
        }
    }
    if chains.is_empty() {
        return Ok(());
    }
    // A doubly-periodic carrier has no geometric pole, so a loop whose whole
    // 3D trace collapses to one point is a zero-area puncture, not a material
    // boundary.  Some STEP writers nevertheless attach a VERTEX_LOOP at a
    // torus contact/seam point (ABC 00000039 faces 63/107).  Keeping that loop
    // turns an otherwise clean two-rim torus band into a three-loop face: every
    // periodic band helper rejects it and generic hole bridging sprays a
    // multi-cover across the parameter domain.  Mass-properties applies the
    // same rule in `biperiodic_band_range`; mirror it here so the exact and
    // tessellated regions agree.  Restrict this to surfaces closed in BOTH
    // directions—on a singly-periodic cone/sphere, a collapsed apex loop is a
    // real topological boundary needed by the pole-cap path.
    let full_biperiodic = crate::topology::doubly_periodic_has_only_collapsed_loops(face)?;
    if full_biperiodic {
        return tessellate_full_biperiodic_face(
            face,
            [u0, u1, v0, v1],
            chord_tolerance,
            face_id,
            mesh,
        );
    }
    if closed_u && closed_v {
        chains.retain(|chain| chain_extent(chain) > 0.0);
        if chains.is_empty() {
            return Ok(());
        }
    }
    // A periodic (closed) wall expressed without a seam ruling — a full rim
    // that spans the whole period as an open (u,v) line plus, for a cone, a
    // degenerate apex — cannot be closed by the outer/hole builder (its loops
    // never close in parameter space). Rebuild it as the seam-cut rectangle so
    // the shared ear-clip/refine path meshes it like a seamed wall.
    // A single trim loop that CROSSES the parametric seam (a window straddling
    // a cylinder's u=0 line) is triangulated boundary-only in an UNWRAPPED
    // domain — its seam is interior to the face, so seeding/refinement (which
    // evaluate strictly in [u0,u1]) are skipped and the boundary sampling
    // carries the wall's single-direction curvature.
    // `skip_interior`: a SHORT seam straddle is triangulated boundary-only in
    // the unwrapped domain (developable wall; no interior stations needed).
    // `periodic_interior`: a TALL helical thread ribbon is unwrapped across all
    // its periods and refined with the domain-wrapping evaluator so its
    // cross-period chords are subdivided down to the surface.
    let mut skip_interior = false;
    let mut periodic_interior = false;
    // A `periodic_interior` region that is still ONE parameter period wide — a
    // cross-seam torus band whose far rim was lifted a whole cross period — as
    // opposed to a MULTI-period helical cover. Such a region is an ordinary
    // rectangle in its own uv box, so it can (and must) be grid-seeded there:
    // without a seed, `refine_interior` has to build the whole interior by
    // bisection from the ear-clip boundary fan, which triples the triangle
    // count on every one of its 12 passes (ABC 00000039 faces 63/107 reached
    // 17392 / 14656 triangles for a band that seeds at ~1300).
    let mut single_period_cover = false;
    let mut planar_trim: Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)> = None;
    // PROTOTYPE (BREP_CDT): the outer + hole loops of a generic (non-seam-special)
    // face, captured before bridging so a constrained-Delaunay path can consume
    // them directly. `None` for the periodic-handler branches (phase 2).
    let mut defer_cdt = false;
    let mut cdt_input: Option<(Vec<FaceVertex>, Vec<Vec<FaceVertex>>)> = None;
    // A singly-periodic full-wrap wall pierced by a hole that STRADDLES the seam
    // (ABC 00000333 interlocking rings): rebuild it as a seam-cut rectangle whose
    // duplicated p0/p1 seam columns absorb the straddle hole as concave notches,
    // so the region is a simple PSLG the CDT owns instead of the domain-spanning
    // membrane bridge_holes sprays. Uses RAW in-domain chains (the seam stays put
    // at p0/p1). Returns None unless the wall matches this exact shape.
    // A singly-periodic full-wrap wall pierced by a loop that STRADDLES the seam
    // AND is too large for a clean straddle notch (a lens/window centred on the
    // seam, whose deep boundary bites the CDT/refine double-cover into a folded
    // membrane — the user's gold-spike bore). Re-cut the covering at a hole-free
    // meridian so every loop becomes a fully-interior hole in a plain rectangle.
    // Uses RAW in-domain chains (it rotates the parameter itself). Preferred over
    // the straddle-notch path; declines (None) unless the wall matches this shape
    // and a clear meridian exists, so a shallow round-bore straddle keeps its
    // existing notch path bit-for-bit. BREP_NO_RECUT restores that path.
    let recut = if closed_u != closed_v && std::env::var("BREP_NO_RECUT").is_err() {
        let raw = face
            .loops
            .iter()
            .map(|loop_record| loop_chain(&loop_record.coedges, samples))
            .collect::<Result<Vec<_>, _>>()?;
        close_periodic_recut_wall(
            face,
            &raw,
            [u0, u1, v0, v1],
            closed_u,
            closed_v,
            chord_tolerance,
        )?
    } else {
        None
    };
    let seam_straddle = if recut.is_none()
        && closed_u != closed_v
        && std::env::var("BREP_NO_STRADDLE").is_err()
    {
        let raw = face
            .loops
            .iter()
            .map(|loop_record| loop_chain(&loop_record.coedges, samples))
            .collect::<Result<Vec<_>, _>>()?;
        seam_straddle_wall_polygons(
            face,
            &raw,
            [u0, u1, v0, v1],
            closed_u,
            closed_v,
            chord_tolerance,
        )?
    } else {
        None
    };
    // A DOUBLY-periodic (torus) two-rim band whose material is the cross-seam
    // complement and whose bore straddles the periodic seam (ABC 00002621/00002624
    // signet-ring torus band): rebuild it as the seam-cut rectangle with the bore
    // woven into the seam columns, so the region is a simple PSLG the CDT owns
    // instead of the self-touching cover the biperiodic-unwrap path folds inside
    // out. Uses RAW in-domain chains (its net-winding/straddle classification and
    // seam split assume in-domain seam-hopping coordinates). Returns None (defer)
    // unless the wall matches this exact shape.
    let biperiodic_straddle = if closed_u && closed_v && std::env::var("BREP_NO_BISTRADDLE").is_err()
    {
        let raw = face
            .loops
            .iter()
            .map(|loop_record| loop_chain(&loop_record.coedges, samples))
            .collect::<Result<Vec<_>, _>>()?;
        close_biperiodic_straddle_band(
            face,
            &raw,
            [u0, u1, v0, v1],
            closed_u,
            closed_v,
            chord_tolerance,
        )?
    } else {
        None
    };
    let mut vertices = if let Some((outer, holes)) = recut {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!(
                "[DISPATCH] face {face_id}: -> close_periodic_recut_wall (outer={}, holes={})",
                outer.len(),
                holes.len()
            );
        }
        // The re-cut rectangle carries rim + hole parameters past the domain end
        // (the covering is rotated onto [seam_p, seam_p+period]), so refine through
        // the domain-wrapping evaluator; the simple multi-hole PSLG is offered to
        // the CDT (the clean multi-loop path that replaces the folding notch bite).
        periodic_interior = true;
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    } else if let Some((outer, holes)) = seam_straddle {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!(
                "[DISPATCH] face {face_id}: -> seam_straddle_wall_polygons (outer={}, holes={})",
                outer.len(),
                holes.len()
            );
        }
        // Curved wall: the seam-cut rectangle needs interior stations, and the
        // notch/seam vertices run to the domain border, so refine through the
        // domain-wrapping evaluator (like the biperiodic-unwrapped branch).
        periodic_interior = true;
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    } else if let Some((outer, holes)) = biperiodic_straddle {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!(
                "[DISPATCH] face {face_id}: -> close_biperiodic_straddle_band (outer={}, holes={})",
                outer.len(),
                holes.len()
            );
        }
        // The seam-cut rectangle carries lifted cross parameters past the domain
        // end and its bore notch runs to the seam columns, so refine through the
        // domain-wrapping evaluator; the simple PSLG is offered to the CDT.
        periodic_interior = true;
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    } else if biperiodic_unwrapped {
        // The implicit torus seam is already unwrapped into one continuous
        // covering-plane polygon per loop: bridge holes and ear-clip it like an
        // ordinary trimmed face, then refine through the domain-wrapping
        // evaluator (the tube's cross curvature needs interior stations, and the
        // unwrapped parameters run past the domain).
        periodic_interior = true;
        // Drop DEGENERATE loops (a pole: every vertex collapses onto one 3D
        // point). They are not real holes — bridging a pole into the polygon
        // teleports an edge to a single point and sprays the ear-clip. The
        // surface caps the pole naturally through the outer boundary + refine.
        chains.retain(|chain| chain_extent(chain) >= chord_tolerance);
        if chains.is_empty() {
            return Ok(());
        }
        let outer_index = (0..chains.len())
            .max_by(|&a, &b| {
                signed_area(&chains[a])
                    .abs()
                    .total_cmp(&signed_area(&chains[b]).abs())
            })
            .unwrap_or(0);
        let mut outer = chains.swap_remove(outer_index);
        if signed_area(&outer) < 0.0 {
            outer.reverse();
        }
        // A ring pierced ON its seam yields a "hole" whose two sides unwrap onto
        // the same meridian (a zero-width slit), and a tangency contact yields a
        // VERTEX_LOOP point; neither is a material hole and both make the seam a
        // self-touch the CDT declines. Drop them so the seam-cut rectangle (outer)
        // is a clean simple PSLG the CDT owns.
        let domain_area = ((u1 - u0) * (v1 - v0)).abs();
        let holes = drop_zero_area_holes(chains, domain_area)
            .into_iter()
            .map(|mut chain| {
                align_periodic_hole_to_outer(
                    &outer,
                    &mut chain,
                    closed_u,
                    closed_v,
                    u1 - u0,
                    v1 - v0,
                );
                if signed_area(&chain) > 0.0 {
                    chain.reverse();
                }
                chain
            })
            .collect::<Vec<_>>();
        // Offer the UNWRAPPED covering-plane loops to the CDT. The seam is now
        // INTERIOR to the region (the loops no longer hop it), so the CDT
        // constrains the real boundary instead of `bridge_holes` teleporting a
        // chord across the implicit seam — the double-cover that renders these
        // biperiodic faces inside-out (ABC 00002621/00002624 torus tubes,
        // 00006582 knurl wall, 00000333 multi-hole ring). `periodic_interior` is
        // already set above, so CDT-path refinement uses the domain-wrapping
        // evaluator. Orientation (outer CCW / holes CW) is established just above.
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    } else if let Some((polygon, wraps_cross_seam)) = close_periodic_trim(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> close_periodic_trim (wraps_cross_seam={wraps_cross_seam})");
        }
        // A band whose material side crosses the CROSS seam (a fillet torus)
        // carries cross parameters past the domain end: refine it through the
        // domain-wrapping evaluator and skip the grid seed, whose extent assumes
        // an in-domain rectangle.
        if wraps_cross_seam {
            periodic_interior = true;
            single_period_cover = true;
        }
        polygon
    } else if let Some(polygon) = close_biperiodic_seam_band(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> close_biperiodic_seam_band");
        }
        // A doubly-periodic (torus) band bounded by a constant-level seam rim and
        // a single-valued wavy cut: seam-cut in domain (the rim shifted to the
        // opposite extreme keeps it in domain), then the shared seed+refine path
        // fills the curved tube like close_periodic_trim's between-rims band.
        polygon
    } else if let Some(polygon) = close_periodic_band(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> close_periodic_band");
        }
        // A full BAND around the wrap bounded by a constant rim and a
        // single-valued varying-level profile loop (thread runout/transition):
        // seam-cut in domain, then the shared seed+refine path (like
        // close_periodic_trim).
        polygon
    } else if let Some(polygon) =
        close_periodic_spiral_annulus(face, &chains, [u0, u1, v0, v1], closed_u, closed_v)?
    {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> close_periodic_spiral_annulus");
        }
        // A rim plus a MULTI-period helical ribbon: the annulus is cut into a
        // disc between the two loops and lifted into the periodic cover, so its
        // parameters run many periods past the seam — refine through the
        // domain-wrapping evaluator, with no grid seed (single-period extent).
        periodic_interior = true;
        polygon
    } else if let Some((polygon, needs_periodic)) = unwrap_seam_crossing_trim(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> unwrap_seam_crossing_trim (needs_periodic={needs_periodic})");
        }
        if needs_periodic {
            periodic_interior = true;
        } else {
            skip_interior = true;
        }
        polygon
    } else if let Some((polygon, _wraps_cross_seam)) = close_periodic_wavy_band(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> close_periodic_wavy_band");
        }
        // A full-wrap castellated band (spoked / gear web): meshed like
        // close_periodic_trim's in-domain band — seed + refine over the seam-cut
        // rectangle, so the window notches stay outside the boundary (open).
        polygon
    } else if let Some((outer, holes)) = close_periodic_wrap_wall(
        face,
        &chains,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
        chord_tolerance,
    )? {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!(
                "[DISPATCH] face {face_id}: -> close_periodic_wrap_wall (outer={}, holes={})",
                outer.len(),
                holes.len()
            );
        }
        // A singly-periodic full-wrap wall whose outer WRAPS the seam at a varying
        // cross level (a weaving cap rim) bounded by a second full-wrap rim or a
        // pole: the seam-cut rectangle carries lifted parameters to the domain
        // border and the weaving rim needs interior stations, so refine through the
        // domain-wrapping evaluator; the simple PSLG is offered to the CDT (instead
        // of the self-touching wrap ear-clip would spray into a fold).
        periodic_interior = true;
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    } else {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!("[DISPATCH] face {face_id}: -> GENERIC bridge_holes (closed=(u:{closed_u},v:{closed_v}) loops={})", chains.len());
        }
        // Largest |area| loop is the outer boundary; orient CCW, holes CW.
        let outer_index = (0..chains.len())
            .max_by(|&a, &b| {
                signed_area(&chains[a])
                    .abs()
                    .total_cmp(&signed_area(&chains[b]).abs())
            })
            .unwrap_or(0);
        let mut outer = chains.swap_remove(outer_index);
        if signed_area(&outer) < 0.0 {
            outer.reverse();
        }
        // On a closed (periodic) wall, a VERTEX_LOOP contact point or a zero-width
        // seam slit is a degenerate "hole": bridging it teleports a chord across
        // the domain (the double-cover) and constraining it is a PSLG the CDT
        // declines. Drop them here too (the clean-full-wrap ring: outer rectangle
        // whose only extra loops are seam-tangency points, ABC 00000333 solids).
        if closed_u || closed_v {
            let domain_area = ((u1 - u0) * (v1 - v0)).abs();
            chains = drop_zero_area_holes(chains, domain_area);
        }
        let holes = chains
            .into_iter()
            .map(|mut chain| {
                if signed_area(&chain) > 0.0 {
                    chain.reverse();
                }
                chain
            })
            .collect::<Vec<_>>();
        if !holes.is_empty()
            && matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Plane { .. })
            )
        {
            planar_trim = Some((outer.clone(), holes.clone()));
        }
        // Offer the region to the metric-CDT only where `bridge_holes` would
        // actually misbehave: a face WITH holes (bridge chords can cross the trim)
        // or a NON-SIMPLE outer (ear-clip sprays domain-spanning triangles — the
        // fold/fill class, e.g. the embossed-pocket faces). A single SIMPLE outer
        // loop with no holes — a clean planar patch, or a full-wrap periodic
        // surface whose only boundary is the seam rectangle (untrimmed torus /
        // sphere) — normally ear-clips faithfully and symmetrically. Preserve
        // that distribution unless the ear clip produces an inverted triangle;
        // a nearly collinear rim can stall it and leave curved slivers that fold
        // during refinement. Keep the original loops for a validated CDT retry.
        // The CDT wants CCW outer / CW holes, as established just above.
        defer_cdt = holes.is_empty() && polygon_is_simple(&outer);
        cdt_input = Some((outer.clone(), holes.clone()));
        bridge_holes(outer, holes)
    };
    if vertices.len() < 3 {
        return Ok(());
    }
    // Intrinsic uv scale, computed HERE (before triangulation) so the CDT path
    // can metric-scale its Delaunay too: maximum first-derivative magnitude per
    // direction, so flips, margins, and CDT quality act in (approximately)
    // unrolled 3D proportions.
    let mut scale = [1e-12f64, 1e-12f64];
    for i in 0..=3 {
        for j in 0..=3 {
            let derivatives = face.surface.derivatives(
                u0 + (u1 - u0) * i as f64 / 3.0,
                v0 + (v1 - v0) * j as f64 / 3.0,
                1,
            )?;
            scale[0] = scale[0].max(derivatives[1][0].length());
            scale[1] = scale[1].max(derivatives[0][1].length());
        }
    }
    // Triangulate the trim region with a metric-aware constrained Delaunay instead
    // of bridge_holes + ear_clip whenever an offered `cdt_input` (the generic
    // branch and the biperiodic-unwrapped branch) forms a clean simple PSLG.
    // `surface_cdt` DECLINES (returns None) unless it can reproduce the authored
    // loops exactly — same boundary vertices, manifold, constraint set intact — so
    // every face it can't safely own falls back to the ear-clip path bit-for-bit.
    // It is now default-on (was gated behind the BREP_CDT prototype flag): the
    // constraint-preserving CDT is strictly safer than bridge_holes' spanning
    // bridges (which cause the domain-spraying double-cover on periodic faces).
    // BREP_NO_CDT is an escape hatch that restores the pure ear-clip path.
    let cdt = if defer_cdt || std::env::var("BREP_NO_CDT").is_ok() {
        None
    } else {
        cdt_input.as_ref().and_then(|(o, h)| surface_cdt(o, h, scale))
    };
    let (mut vertices, mut triangles, boundary_pairs, used_cdt) = if let Some(m) = cdt {
        if std::env::var("BREP_DEBUG_TESS_DISPATCH").is_ok() {
            eprintln!(
                "[DISPATCH] face {face_id}: -> surface_cdt ({} tris, {} boundary verts)",
                m.triangles.len(),
                m.vertices.len()
            );
        }
        (m.vertices, m.triangles, m.boundary_pairs, true)
    } else {
        // Ear-clip path on the bridged polygon (the historical route). Boundary
        // (and bridge) segments are the consecutive index pairs of the bridged
        // polygon; refinement must never split these.
        let mut boundary_pairs = HashSet::new();
        for index in 0..vertices.len() {
            let next = (index + 1) % vertices.len();
            boundary_pairs.insert((index.min(next), index.max(next)));
        }
        if std::env::var("BREP_DEBUG_TESS_CHAIN").is_ok_and(|value| value == face.id.to_string()) {
            for (index, vertex) in vertices.iter().enumerate() {
                eprintln!(
                    "chain[{index}] uv=({:.9},{:.9}) pos=({:.6},{:.6},{:.6})",
                    vertex.uv[0], vertex.uv[1], vertex.position.x, vertex.position.y, vertex.position.z
                );
            }
        }
        let mut indices: Vec<usize> = (0..vertices.len()).collect();
        let original_ring: Vec<usize> = indices.clone();
        let max_split_span = if periodic_interior && closed_u != closed_v {
            let axis = usize::from(closed_v);
            let period = if closed_u { u1 - u0 } else { v1 - v0 };
            let (minimum, maximum) = vertices.iter().fold(
                (f64::INFINITY, f64::NEG_INFINITY),
                |(minimum, maximum), vertex| {
                    (minimum.min(vertex.uv[axis]), maximum.max(vertex.uv[axis]))
                },
            );
            if std::env::var("BREP_DEBUG_TESS").is_ok() && maximum - minimum > 1.5 * period {
                eprintln!(
                    "face {}: periodic cover span {:.3} turns",
                    face.id,
                    (maximum - minimum) / period
                );
            }
            // Ordinary screw threads still use the historical balanced splitter.
            // The pathological family spans 24.5–25 turns; constrain only covers
            // beyond 24.25 periods, leaving the nearby 23.875-turn screw corpus
            // bit-for-bit unchanged while preventing the extreme cross-turn cut.
            (maximum - minimum > 24.25 * period).then_some((axis, 1.05 * period))
        } else {
            None
        };
        let region_guard = RegionGuard {
            vertices: &vertices,
            ring: &original_ring,
            max_split_span,
        };
        let triangles = ear_clip(&vertices, &mut indices, &region_guard);
        if std::env::var("BREP_DEBUG_TESS").is_ok() {
            eprintln!(
                "face {}: boundary={} ear_clip={} triangles",
                face.id,
                vertices.len(),
                triangles.len()
            );
        }
        // A stalled ear clip may emit inverted slivers along a nearly
        // collinear rim. Refining those folds a curved surface back over
        // itself. Retry with the constraint-preserving triangulator before
        // seeding or splitting any interior edge.
        let recovered = if defer_cdt
            && std::env::var("BREP_NO_CDT").is_err()
            && triangles.iter().any(|&[a, b, c]| {
                triangle_area(vertices[a].uv, vertices[b].uv, vertices[c].uv) < 0.0
            })
        {
            cdt_input.as_ref().and_then(|(o, h)| surface_cdt(o, h, scale))
        } else {
            None
        };
        if let Some(m) = recovered {
            (m.vertices, m.triangles, m.boundary_pairs, true)
        } else {
            (vertices, triangles, boundary_pairs, false)
        }
    };
    // A SHORT seam straddle is boundary-only (its interior stations would
    // evaluate out of domain). A TALL helical ribbon is refined with the
    // domain-wrapping evaluator: no grid seed (the grid extent assumes a single
    // period), just adaptive splitting of the cross-period chords that bulge off
    // the developable-but-u-curved wall. Every other face seeds and refines.
    //
    // INTERIOR accuracy is bounded by BOUNDARY accuracy. When `MAX_EDGE_SPANS`
    // cut one of this face's own edges short, that edge's chain misses the chord
    // tolerance by a known amount; splitting the interior below that figure
    // cannot make the face any truer to the surface — the rim it is stitched to
    // is already further off — while the cost of chasing it is a full 12-pass
    // cascade. Floor the interior target at the worst sag the boundary actually
    // achieved. Inert (bit-identical) on every face whose edges converged, which
    // is all but the pathological ones: `worst_sag` is 0.0 unless the cap bit.
    let mut interior_tolerance = chord_tolerance;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            if let Some(edge_samples) = samples.get(&coedge.edge_id) {
                interior_tolerance = interior_tolerance.max(edge_samples.worst_sag);
            }
        }
    }
    if used_cdt {
        // The CDT gave a Delaunay boundary triangulation. Seed its interior on
        // the same curvature grid the ear-clip path uses: reaching that density
        // by edge splitting alone costs a full refinement cascade (the
        // onshape-unclamped-periodic wall, 1679 boundary vertices, climbed to
        // 44k triangles over 12 passes to reach a density its seed grid
        // expresses directly). Containment comes from the host-triangle search
        // rather than ring parity, which is what `BoundaryForm::Constrained`
        // selects — the CDT's several loops have no single ring to test against.
        if !skip_interior {
            // Every CDT-fed periodic branch (re-cut wall, seam straddle,
            // biperiodic straddle, unwrapped torus, wrap wall) hands over ONE
            // seam-cut/unwrapped covering rectangle whose parameters merely sit
            // past the domain end, so — exactly as the ear-clip periodic band
            // does — the grid spans that region's own uv box and evaluates
            // through the domain-wrapping evaluator.
            let seed_box = if periodic_interior {
                region_uv_box(&vertices)
            } else {
                [u0, u1, v0, v1]
            };
            seed_interior_grid(
                face,
                &mut vertices,
                &mut triangles,
                &boundary_pairs,
                interior_tolerance,
                seed_box,
                scale,
                periodic_interior,
                BoundaryForm::Constrained,
            )?;
        }
        refine_interior(
            face,
            &mut vertices,
            &mut triangles,
            &boundary_pairs,
            interior_tolerance,
            scale,
            periodic_interior,
        )?;
    } else if periodic_interior {
        // A single-period periodic cover (the cross-seam torus band) is seeded
        // over its OWN uv box through the domain-wrapping evaluator. A
        // MULTI-period helical ribbon still skips the seed — the grid extent
        // assumes a single period there.
        if single_period_cover {
            let region = region_uv_box(&vertices);
            seed_interior_grid(
                face,
                &mut vertices,
                &mut triangles,
                &boundary_pairs,
                interior_tolerance,
                region,
                scale,
                true,
                BoundaryForm::SingleRing,
            )?;
        }
        refine_interior(
            face,
            &mut vertices,
            &mut triangles,
            &boundary_pairs,
            interior_tolerance,
            scale,
            true,
        )?;
    } else if !skip_interior {
        seed_interior_grid(
            face,
            &mut vertices,
            &mut triangles,
            &boundary_pairs,
            interior_tolerance,
            [u0, u1, v0, v1],
            scale,
            false,
            BoundaryForm::SingleRing,
        )?;
        refine_interior(
            face,
            &mut vertices,
            &mut triangles,
            &boundary_pairs,
            interior_tolerance,
            scale,
            false,
        )?;
    }
    if let Some((outer, holes)) = planar_trim {
        if !certified_planar_trim(&vertices, &triangles, &outer, &holes) {
            if let Some(direct) = direct_planar_cdt(outer, holes) {
                if std::env::var("BREP_DEBUG_PLANAR_CDT").is_ok() {
                    eprintln!("face {}: certified planar CDT fallback", face.id);
                }
                vertices = direct.vertices;
                triangles = direct.triangles;
            } else if std::env::var("BREP_DEBUG_PLANAR_CDT").is_ok() {
                eprintln!(
                    "face {}: legacy planar trim failed certification and CDT declined it",
                    face.id
                );
            }
        }
    }

    let base = (mesh.positions.len() / 3) as u32;
    for vertex in &vertices {
        // Unwrapped vertices carry a periodic parameter shifted past the seam
        // (a short straddle, many periods on a helical ribbon, or a cross-seam
        // band on a torus); reduce EVERY closed direction into its domain so the
        // normal evaluates in the real domain. Singly-periodic faces are
        // unaffected — only their one closed direction can be out of domain.
        let normal_uv = if skip_interior || periodic_interior {
            let mut uv = vertex.uv;
            if closed_u {
                uv[0] = u0 + (uv[0] - u0).rem_euclid(u1 - u0);
            }
            if closed_v {
                uv[1] = v0 + (uv[1] - v0).rem_euclid(v1 - v0);
            }
            uv
        } else {
            vertex.uv
        };
        let normal = face_normal_at(face, normal_uv, [u0, u1, v0, v1])?;
        mesh.positions
            .extend([vertex.position.x, vertex.position.y, vertex.position.z]);
        mesh.normals.extend([normal.x, normal.y, normal.z]);
    }
    for triangle in &triangles {
        let [a, b, c] = *triangle;
        // Drop only triangles with two POINT-COINCIDENT vertices (pole fans,
        // degenerate-edge slivers): their remaining two edges weld to the
        // same key, so parity is preserved and no crack can open. Collinear
        // slivers with three distinct points must be KEPT — removing one
        // would orphan its neighbors' edges.
        let coincident = vertices[a].position.sub(vertices[b].position).length() == 0.0
            || vertices[b].position.sub(vertices[c].position).length() == 0.0
            || vertices[c].position.sub(vertices[a].position).length() == 0.0;
        if coincident {
            continue;
        }
        // Emit with the `same_sense` uv-CCW convention. This winds each face
        // CONSISTENTLY with its own parametric orientation (it does NOT trust the
        // per-vertex analytic normal, which can FLIP within a folded/trimmed
        // face and would then wind half the face inside-out). A single global
        // coherence pass over the whole mesh — `orient_mesh_coherently`, run once
        // at the end of `tessellate_brep_watertight` — makes every shared edge
        // coherent and orients each component outward by its signed volume,
        // superseding the old per-triangle geometric check.
        if face.same_sense {
            mesh.indices
                .extend([base + a as u32, base + b as u32, base + c as u32]);
        } else {
            mesh.indices
                .extend([base + a as u32, base + c as u32, base + b as u32]);
        }
        mesh.face_ids.push(face_id);
    }
    Ok(())
}
