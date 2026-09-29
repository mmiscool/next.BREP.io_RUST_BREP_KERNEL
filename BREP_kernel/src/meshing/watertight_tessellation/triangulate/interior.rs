use super::*;
use super::earclip::{lawson_flips, lawson_flips_guarded};

/// How the boundary triangulation `seed_interior_grid` densifies was built —
/// it decides how a grid station's containment is tested. See the parameter
/// docs on [`seed_interior_grid`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub(in crate::watertight_tessellation) enum BoundaryForm {
    /// One closed ring occupying vertices `0..boundary_count` (bridged ear-clip).
    SingleRing,
    /// Several authored loops held as CDT constraints.
    Constrained,
}

/// Seed the face interior with a uv grid whose spacing follows directional
/// curvature (chord sag) and the boundary sampling density, clipped to the
/// trimmed polygon with a safety margin. Grid points are inserted by
/// splitting their containing triangle, so the boundary triangulation and
/// its index pairs stay untouched; Lawson flips afterwards turn the result
/// into a near-Delaunay structured mesh instead of boundary fans.
///
/// `domains` is the parameter box the grid spans — normally the surface domain,
/// but a PERIODIC region lifted past the domain end (a cross-seam torus band)
/// passes its OWN uv box; `periodic` then selects the domain-wrapping evaluator
/// (Golovanov §3.15) so stations outside `[u0,u1]×[v0,v1]` still land on the
/// real surface. Every in-domain caller passes `false` and keeps the plain
/// evaluator (bit-identical path).
pub(in crate::watertight_tessellation) fn seed_interior_grid(
    face: &FaceRecord,
    vertices: &mut Vec<FaceVertex>,
    triangles: &mut Vec<[usize; 3]>,
    boundary_pairs: &HashSet<(usize, usize)>,
    chord_tolerance: f64,
    domains: [f64; 4],
    scale: [f64; 2],
    periodic: bool,
    // The boundary triangulation this grid is dropped into. `SingleRing` is the
    // bridged ear-clip polygon, whose vertices 0..n are one closed ring, so a
    // crossing-parity test decides containment. `Constrained` is a CDT of the
    // authored outer + hole loops: those vertices form SEVERAL rings, parity
    // over their concatenation is meaningless, and containment instead comes
    // from the host-triangle search below (the CDT covers exactly the trimmed
    // region, so "lands in a triangle" IS "inside the trim").
    boundary: BoundaryForm,
) -> Result<(), String> {
    let _timer = stage_timer(Stage::Seed);
    let [u0, u1, v0, v1] = domains;
    let [su, sv] = scale;
    let boundary_count = vertices.len();
    // Directional curvature magnitudes over a coarse uv sweep. `suv` (the twist)
    // joins them because the chords this grid actually produces are the CELL
    // DIAGONALS, not the axis steps — see the diagonal correction below.
    let mut suu: f64 = 0.0;
    let mut svv: f64 = 0.0;
    let mut suv: f64 = 0.0;
    for i in 0..=4 {
        for j in 0..=4 {
            let u = u0 + (u1 - u0) * i as f64 / 4.0;
            let v = v0 + (v1 - v0) * j as f64 / 4.0;
            let derivatives = if periodic {
                face.surface.derivatives_extended(u, v, 2)?
            } else {
                face.surface.derivatives(u, v, 2)?
            };
            suu = suu.max(derivatives[2][0].length());
            svv = svv.max(derivatives[0][2].length());
            suv = suv.max(derivatives[1][1].length());
        }
    }
    // OpenCascade-style density: interior stations exist only where the
    // surface actually curves (chord sag per direction); flat directions
    // keep tall anisotropic strips and planar faces get no interior points
    // at all. The boundary sampling already carries the stations along a
    // singly-curved face's curved edges, so seeding is needed only when
    // BOTH directions curve.
    let step = |second_derivative: f64| -> f64 {
        if second_derivative > 1e-12 {
            (8.0 * chord_tolerance / second_derivative).sqrt()
        } else {
            f64::INFINITY
        }
    };
    let mut du = step(suu);
    let mut dv = step(svv);
    // DIAGONAL correction. `step` solves `S·d²/8 = tol` per AXIS, but the grid
    // is triangulated: the chord that actually spans a cell is its DIAGONAL,
    // whose second directional derivative is the full quadratic form
    // `Suu·du² + 2·Suv·du·dv + Svv·dv²`. With both directions curving that is
    // ≥ 2× the axis target even at zero twist, so EVERY diagonal starts one
    // split over tolerance and `refine_interior` — which can only halve, i.e.
    // quarter the sag — quadruples the whole interior to fix a 2× miss. Scaling
    // both steps by the single factor that makes the diagonal meet tolerance
    // costs ~2x stations but removes that cascade, so the converged mesh is both
    // coarser and (because the seed is a structured grid rather than a bisected
    // fan) better shaped. Scaling BOTH steps by one factor is the optimum, not a
    // convenience: maximising `du·dv` (i.e. minimising stations) under the
    // quadratic constraint puts `dv/du` at `sqrt(Suu/Svv)`, which is exactly the
    // ratio the per-axis steps already have.
    {
        // A step wider than the region cannot be taken, so measure the diagonal
        // of the widest cell that can actually occur — for a FLAT direction that
        // is the whole extent. A flat direction is NOT excused from this test:
        // zero second derivative along an axis says the axis is a straight
        // ruling, not that the cell diagonal is straight. On a TWISTED ruled
        // strip (`Suv != 0`, `Svv == 0`) the diagonal curves even though both the
        // rulings and one boundary are straight, and bailing out there left the
        // whole interior to bisection. A genuine extrusion keeps `Suv == 0`, so
        // its diagonal already meets tolerance, nothing shrinks, and it still
        // seeds nothing — unchanged.
        let cell_u = du.min(u1 - u0);
        let cell_v = dv.min(v1 - v0);
        let diagonal =
            suu * cell_u * cell_u + 2.0 * suv * cell_u * cell_v + svv * cell_v * cell_v;
        if diagonal > 8.0 * chord_tolerance {
            let shrink = (8.0 * chord_tolerance / diagonal).sqrt();
            du = cell_u * shrink;
            dv = cell_v * shrink;
        }
    }
    let clamp = |d: f64, extent: f64| -> f64 {
        if d.is_finite() {
            d.min(extent).max(extent / 256.0)
        } else {
            f64::INFINITY
        }
    };
    let du = clamp(du, u1 - u0);
    let dv = clamp(dv, v1 - v0);
    let nu = if du.is_finite() {
        ((u1 - u0) / du).ceil() as usize
    } else {
        1
    };
    let nv = if dv.is_finite() {
        ((v1 - v0) / dv).ceil() as usize
    } else {
        1
    };
    if nu <= 1 || nv <= 1 {
        return Ok(());
    }
    // Station budget. Skipping the seed entirely when the chord-driven grid
    // exceeded it (the old behavior) was catastrophic at fine tolerances: the
    // refinement cascade then converges the coarse boundary fan at 3×
    // triangles per full pass, overshooting the target density ~10× and
    // spending minutes (a unit sphere at chord 3e-4 produced 513k triangles
    // over 12 passes — the "Ultra render detail hangs" report; at
    // 5e-4, just under the cap, the same sphere was 31k triangles in 3 s).
    // CLAMP the grid to the budget instead: seed at the densest allowed
    // spacing and let refinement close the (now small) remaining gap.
    const STATION_BUDGET: usize = 8_192;
    // The budget caps the STATIONS this face receives, but the grid is laid over
    // the whole `domains` box while only the TRIMMED part of it can host one. On
    // a small trim of a large surface (a window in a full torus / cylinder
    // domain) most grid points are discarded, so budgeting the raw nu×nv spent
    // the allowance on empty parameter space and throttled the real density
    // several-fold — the seed then missed the tolerance by that factor and
    // `refine_interior` made it up by bisection, at 4x the stations per level.
    // Scale the allowance by the share of the box the region actually covers,
    // measured exactly as the uv area of the boundary triangulation (which
    // covers the trim, ear-clip and CDT alike). A region filling its box is
    // unchanged.
    let box_area = ((u1 - u0) * (v1 - v0)).abs();
    let region_area: f64 = triangles
        .iter()
        .map(|&[a, b, c]| triangle_area(vertices[a].uv, vertices[b].uv, vertices[c].uv).abs())
        .sum();
    let share = if box_area > 0.0 {
        (region_area / box_area).clamp(1.0 / 256.0, 1.0)
    } else {
        1.0
    };
    let budget = (STATION_BUDGET as f64 / share) as usize;
    let stations = nu.saturating_mul(nv);
    let (nu, nv) = if stations > budget {
        let shrink = (stations as f64 / budget as f64).sqrt();
        (
            ((nu as f64 / shrink).floor() as usize).max(2),
            ((nv as f64 / shrink).floor() as usize).max(2),
        )
    } else {
        (nu, nv)
    };
    // Effective spacing after the clamp (feeds the boundary-crowding margin).
    let du = du.max((u1 - u0) / nu as f64);
    let dv = dv.max((v1 - v0) / nv as f64);
    // Protected segments to keep clear of. For the bridged ear-clip polygon
    // these are exactly its consecutive ring pairs, so the distance test is the
    // same one the ring form always ran; for a CDT they are the authored loop
    // edges. Sorted so the scan order is deterministic.
    let single_ring = matches!(boundary, BoundaryForm::SingleRing);
    let segments: Vec<([f64; 2], [f64; 2])> = if single_ring {
        (0..boundary_count)
            .map(|index| {
                (
                    vertices[index].uv,
                    vertices[(index + 1) % boundary_count].uv,
                )
            })
            .collect()
    } else {
        let mut keys: Vec<(usize, usize)> = boundary_pairs.iter().copied().collect();
        keys.sort_unstable();
        keys.into_iter()
            .map(|(first, second)| (vertices[first].uv, vertices[second].uv))
            .collect()
    };
    let polygon: Vec<[f64; 2]> = (0..boundary_count)
        .map(|index| vertices[index].uv)
        .collect();
    let margin = 0.35 * (du * su).min(dv * sv).max(1e-12);
    // Both rejection tests below are decided by geometry local to the station,
    // so they are bucket lookups rather than scans of the whole boundary — the
    // candidates each station receives are a conservative superset of the ones
    // a scan would have tested, and each receives the identical test, so the
    // answer is the same boolean. See `station_index.rs`.
    let index = BoundaryIndex::new(
        &segments,
        if single_ring { &polygon } else { &[] },
        scale,
        margin,
    );
    let inside_with_margin = |uv: [f64; 2]| -> bool {
        if index.near_segment(uv) {
            return false;
        }
        if !single_ring {
            // Containment is decided by the host-triangle search.
            return true;
        }
        index.ring_encloses(uv)
    };
    tess_profile(|p| p.stations += (nu.saturating_sub(1) * nv.saturating_sub(1)) as u64);
    for iu in 1..nu {
        for iv in 1..nv {
            let uv = [
                u0 + (u1 - u0) * iu as f64 / nu as f64,
                v0 + (v1 - v0) * iv as f64 / nv as f64,
            ];
            if !inside_with_margin(uv) {
                continue;
            }
            // Containing triangle by strict barycentric sign test. Reverse
            // order: row-major grid points land in freshly split triangles,
            // so the newest entries are the likeliest hosts.
            let mut host = None;
            tess_profile(|p| p.stations_placed += 1);
            let host_timer = stage_timer(Stage::Host);
            let mut visits = 0u64;
            for (triangle_index, triangle) in triangles.iter().enumerate().rev() {
                visits += 1;
                let a = vertices[triangle[0]].uv;
                let b = vertices[triangle[1]].uv;
                let c = vertices[triangle[2]].uv;
                let total = triangle_area(a, b, c);
                if total <= 1e-30 {
                    continue;
                }
                let w0 = triangle_area(uv, b, c) / total;
                let w1 = triangle_area(uv, c, a) / total;
                let w2 = triangle_area(uv, a, b) / total;
                if w0 > 1e-6 && w1 > 1e-6 && w2 > 1e-6 {
                    host = Some(triangle_index);
                    break;
                }
            }
            tess_profile(|p| p.host_visits += visits);
            drop(host_timer);
            let Some(host) = host else {
                continue;
            };
            let position = if periodic {
                face.surface.evaluate_extended(uv[0], uv[1])?
            } else {
                face.surface.evaluate(uv[0], uv[1])?
            };
            let index = vertices.len();
            vertices.push(FaceVertex { uv, position });
            let [a, b, c] = triangles[host];
            triangles[host] = [a, b, index];
            triangles.push([b, c, index]);
            triangles.push([c, a, index]);
        }
    }
    lawson_flips(vertices, triangles, boundary_pairs, scale);
    Ok(())
}

pub(in crate::watertight_tessellation) fn refine_interior(
    face: &FaceRecord,
    vertices: &mut Vec<FaceVertex>,
    triangles: &mut Vec<[usize; 3]>,
    boundary_pairs: &HashSet<(usize, usize)>,
    chord_tolerance: f64,
    scale: [f64; 2],
    // An UNWRAPPED periodic ribbon (helical thread) carries interior vertices
    // whose periodic parameter runs many periods outside [u0,u1]; evaluate the
    // midpoints with the domain-wrapping extended evaluator (Golovanov §3.15
    // cyclic wrap of the closed direction) so positions land on the real
    // surface. Every non-periodic face passes `false` (bit-identical path).
    periodic: bool,
) -> Result<(), String> {
    let _timer = stage_timer(Stage::Refine);
    // The raw ear-clip output is a boundary fan; flips must reshape it
    // BEFORE the chord criterion runs, or fan diagonals trigger an
    // avalanche of unnecessary splits. UNGUARDED: nothing in an ear-clip fan is
    // within the chord tolerance yet, so the accuracy guard has nothing to
    // protect here — and this is the largest candidate set the flipper ever
    // sees, so paying two surface evaluations per candidate is pure cost.
    lawson_flips(vertices, triangles, boundary_pairs, scale);
    for _ in 0..MAX_REFINE_PASSES {
        tess_profile(|p| p.refine_passes += 1);
        // Corner list sorted into edge groups rather than a map of `Vec`s: the
        // groups come out in ascending edge order, which is what the explicit
        // `candidates.sort_unstable()` below produced, so the midpoint indices
        // — and therefore `lawson_flips`' cocircular tie-break — are unchanged.
        let mut corners: Vec<(usize, usize, usize)> = Vec::with_capacity(triangles.len() * 3);
        for (triangle_index, triangle) in triangles.iter().enumerate() {
            for corner in 0..3 {
                let a = triangle[corner];
                let b = triangle[(corner + 1) % 3];
                corners.push((a.min(b), a.max(b), triangle_index));
            }
        }
        corners.sort_unstable();
        let mut splits: HashMap<(usize, usize), usize> = HashMap::new();
        // Iterate the candidate edges in SORTED order, not `HashMap` order. The
        // midpoint vertices are pushed as they are accepted, so the iteration
        // order decides their INDICES — and `lawson_flips` breaks cocircular
        // ties by sorted index, so a shuffled numbering yields a different (also
        // Delaunay) triangulation. `RandomState` reseeds every map it builds, so
        // two runs over the same face would otherwise number the midpoints
        // differently and mesh a dense, highly cocircular grid (a torus) two
        // different ways — the snapshot round-trip fixtures caught exactly that.
        let mut group = 0usize;
        while group < corners.len() {
            let (a, b, _) = corners[group];
            let mut end = group + 1;
            while end < corners.len() && corners[end].0 == a && corners[end].1 == b {
                end += 1;
            }
            let users = end - group;
            group = end;
            if boundary_pairs.contains(&(a, b)) {
                continue;
            }
            if users != 2 {
                continue;
            }
            let uv_mid = [
                (vertices[a].uv[0] + vertices[b].uv[0]) * 0.5,
                (vertices[a].uv[1] + vertices[b].uv[1]) * 0.5,
            ];
            let surface_mid = if periodic {
                face.surface.evaluate_extended(uv_mid[0], uv_mid[1])?
            } else {
                face.surface.evaluate(uv_mid[0], uv_mid[1])?
            };
            let chord_sag =
                deflection_to_chord(surface_mid, vertices[a].position, vertices[b].position);
            if chord_sag <= chord_tolerance {
                // Chord sag is within tolerance; the curvature-adaptive
                // criterion can still force a split where the surface normal
                // swings too far across this interior edge (OCC angular
                // deflection). Interior edges only — the boundary guard above
                // means shared/seam edges are never touched, so watertightness
                // is preserved. The sag floor bounds how far below the chord
                // target angular refinement may cascade.
                let angular_gated = chord_sag < ANGULAR_SAG_FLOOR * chord_tolerance
                    || interior_edge_within_angle(face, vertices[a].uv, vertices[b].uv, periodic);
                if angular_gated {
                    continue;
                }
            }
            let index = vertices.len();
            vertices.push(FaceVertex {
                uv: uv_mid,
                position: surface_mid,
            });
            splits.insert((a, b), index);
            tess_profile(|p| p.splits += 1);
        }
        if std::env::var("BREP_DEBUG_TESS").is_ok() {
            // Histogram the split midpoints by v-band + the worst sag, so a
            // runaway concentrated at the poles (or anywhere) is visible
            // without printing per-split lines.
            let mut bands = [0usize; 5];
            let mut worst_sag = 0.0f64;
            for (&(a, b), &mid) in &splits {
                let v = vertices[mid].uv[1];
                let band = ((v * 5.0) as usize).min(4);
                bands[band] += 1;
                let surface_mid = vertices[mid].position;
                let sag = deflection_to_chord(
                    surface_mid,
                    vertices[a].position,
                    vertices[b].position,
                );
                worst_sag = worst_sag.max(sag);
            }
            eprintln!(
                "  refine pass: {} triangles, {} splits v-bands={:?} worst_sag={:.3e}",
                triangles.len(),
                splits.len(),
                bands,
                worst_sag
            );
        }
        if splits.is_empty() {
            lawson_flips_guarded(
                vertices,
                triangles,
                boundary_pairs,
                scale,
                Some((face, chord_tolerance, periodic)),
            );
            return Ok(());
        }
        let mut next_triangles = Vec::with_capacity(triangles.len() + splits.len() * 2);
        for triangle in triangles.iter() {
            // Split each triangle at every split midpoint on its edges
            // (1–3 splits per triangle), keeping the mesh conforming.  The
            // fan MUST start at a midpoint: fanning from an original corner
            // adjacent to a split edge emits a zero-UV-area sliver whose
            // off-chord 3D midpoint folds the mesh.
            let mut polygon: Vec<usize> = Vec::with_capacity(6);
            let mut first_midpoint: Option<usize> = None;
            for corner in 0..3 {
                let a = triangle[corner];
                let b = triangle[(corner + 1) % 3];
                polygon.push(a);
                if let Some(&middle) = splits.get(&(a.min(b), a.max(b))) {
                    if first_midpoint.is_none() {
                        first_midpoint = Some(polygon.len());
                    }
                    polygon.push(middle);
                }
            }
            let start = first_midpoint.unwrap_or(0);
            for offset in 1..polygon.len() - 1 {
                next_triangles.push([
                    polygon[start],
                    polygon[(start + offset) % polygon.len()],
                    polygon[(start + offset + 1) % polygon.len()],
                ]);
            }
        }
        *triangles = next_triangles;
        // Accuracy-guarded quality pass: reshaping must not hand back a
        // diagonal that sags past the chord tolerance (see
        // `lawson_flips_guarded`), or this loop spends its passes re-splitting
        // what the previous round's flips lengthened.
        lawson_flips_guarded(
            vertices,
            triangles,
            boundary_pairs,
            scale,
            Some((face, chord_tolerance, periodic)),
        );
    }
    lawson_flips_guarded(
        vertices,
        triangles,
        boundary_pairs,
        scale,
        Some((face, chord_tolerance, periodic)),
    );
    Ok(())
}

/// True when the unit surface normal swings by at most [`ANGULAR_TOL`] between
/// the two endpoints of an INTERIOR edge — the curvature-adaptive refinement
/// gate. Returns `true` (do not force an angular split) when either normal is
/// indeterminate (e.g. a degenerate Su×Sv at a pole), leaving such edges to the
/// chord criterion alone.
///
/// Alloc-free via `deriv1`/`deriv1_extended` → Su×Sv, mirroring the position
/// path (the extended evaluator for an unwrapped periodic ribbon whose interior
/// uvs run outside the domain). Symmetric in the two endpoints and a pure
/// function of the surface, so both adjacent faces — and serial vs. parallel
/// face passes — decide identically.
fn interior_edge_within_angle(
    face: &FaceRecord,
    uv_a: [f64; 2],
    uv_b: [f64; 2],
    periodic: bool,
) -> bool {
    let normal = |uv: [f64; 2]| -> Option<Vec3> {
        let (_, su, sv) = if periodic {
            face.surface.deriv1_extended(uv[0], uv[1]).ok()?
        } else {
            face.surface.deriv1(uv[0], uv[1]).ok()?
        };
        su.cross(sv).normalized().ok()
    };
    let (Some(na), Some(nb)) = (normal(uv_a), normal(uv_b)) else {
        return true;
    };
    // Both unit ⇒ na·nb = cos(swing). Within tolerance ⇔ cos(swing) ≥ cos(TOL).
    na.dot(nb) >= ANGULAR_TOL.cos()
}

pub(in crate::watertight_tessellation) fn face_normal_at(face: &FaceRecord, uv: [f64; 2], domains: [f64; 4]) -> Result<Vec3, String> {
    let (_, su, sv) = face.surface.deriv1(uv[0], uv[1])?;
    let mut normal = su.cross(sv);
    if normal.length() <= 1e-12 {
        let [u0, u1, v0, v1] = domains;
        let epsilon = 1e-4;
        let nudged_u = uv[0].clamp(u0 + epsilon * (u1 - u0), u1 - epsilon * (u1 - u0));
        let nudged_v = uv[1].clamp(v0 + epsilon * (v1 - v0), v1 - epsilon * (v1 - v0));
        let (_, nsu, nsv) = face.surface.deriv1(nudged_u, nudged_v)?;
        normal = nsu.cross(nsv);
        if normal.length() <= 1e-12 {
            normal = Vec3::new(0.0, 0.0, 1.0);
        }
    }
    let mut normal = normal.normalized()?;
    if !face.same_sense {
        normal = normal.scale(-1.0);
    }
    Ok(normal)
}

