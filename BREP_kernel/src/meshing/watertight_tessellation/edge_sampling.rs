use super::*;

pub(super) const MAX_EDGE_SPANS: usize = 512;
pub(super) const MAX_REFINE_PASSES: usize = 12;

/// Curvature-adaptive (angular) interior refinement threshold, in radians —
/// the surface-normal-deviation analog of OpenCASCADE's 0.5-rad angular
/// deflection in `BRepMesh_IncrementalMesh(shape, linDefl, false, 0.5, false)`.
///
/// The chord criterion alone refines by 3D sag, so at a fine (small-relative)
/// chord tolerance a curved face is already dense while at a coarse one it
/// facets. This adds a FLOOR on interior curved-face density: an interior
/// edge is also split when the surface normal swings more than `ANGULAR_TOL`
/// across it, regardless of how little the chord sags. Applied to face
/// INTERIOR edges only (shared boundary/seam edges are never split), so both
/// faces of every shared edge keep identical samples — the watertight
/// guarantee is untouched. Purely a function of the face surface and the two
/// endpoint uvs (symmetric in the endpoints), so it is deterministic and
/// independent of edge iteration order — the serial==parallel face-stride
/// fingerprint holds. `MAX_REFINE_PASSES` bounds the total work (each pass
/// halves an edge's normal swing, so it converges in a few passes).
pub(super) const ANGULAR_TOL: f64 = 0.35;

/// Lower bound, as a fraction of the chord tolerance, on the 3D chord sag an
/// interior edge must still have for the angular criterion to keep splitting
/// it. A split quarters an edge's sag but only halves its normal swing, so on a
/// small, high-curvature feature the angular test alone would cascade several
/// levels deep. Once an edge sags less than this fraction of the chord target
/// it is already far flatter than the display needs, so angular refinement
/// stops there — bounding it to roughly one subdivision beyond the chord-driven
/// mesh (the primitives whose seed grid sits at ≈ chord sag keep their
/// curvature win; a pathological face cannot explode).
pub(super) const ANGULAR_SAG_FLOOR: f64 = 0.25;

/// Curvature-adaptive (angular) EDGE-sampling threshold, in radians — the edge
/// analog of OpenCASCADE's scale-independent angular deflection.
///
/// The chord criterion in [`edge_fractions`] samples an edge purely by 3D chord
/// SAG, and the caller's chord tolerance is relative to the whole SOLID's scale
/// (`solid_scale × 1.5e-3`). For a small-radius circle on a large part that
/// chord is coarse relative to the radius, so only ~6–8 segments survive and the
/// circle renders as a polygon (the octagonal shaft / hexagonal head of a long
/// bolt). This adds an ANGULAR floor: an edge interval is also split when the
/// edge's own 3D-CURVE unit TANGENT rotates more than `EDGE_ANGULAR_TOL` across
/// it, so a circle gets ~`2π / EDGE_ANGULAR_TOL` segments at ANY radius, exactly
/// as OCC does.
///
/// WATERTIGHT-SAFE: the criterion is a pure function of the SHARED edge's 3D
/// curve (its tangent/curvature), evaluated at parameter values that depend only
/// on the interval fractions. It is symmetric in the endpoints and carries no
/// per-face dependence, so both adjacent faces still receive byte-identical
/// samples for the shared edge — unlike a surface-normal criterion, which
/// differs per face (that is why the interior fix above never touches edges).
/// Deterministic and independent of edge iteration order, so the serial==
/// parallel face-stride fingerprint holds. Bounded by `MAX_EDGE_SPANS`: the
/// total tangent rotation of an edge is finite (2π for a full circle → ~32
/// bisected segments), so a tiny high-curvature edge cannot explode.
pub(super) const EDGE_ANGULAR_TOL: f64 = 0.35;

/// Perpendicular distance from `point` to the segment [a, b] — the true
/// chordal deflection. Distance-to-midpoint would add the tangential offset
/// of non-uniform (rational) parameterizations and over-refine several-fold.
pub(super) fn deflection_to_chord(point: Vec3, a: Vec3, b: Vec3) -> f64 {
    let ab = b.sub(a);
    let length_squared = ab.dot(ab);
    if length_squared <= 1e-30 {
        return point.sub(a).length();
    }
    let t = (point.sub(a).dot(ab) / length_squared).clamp(0.0, 1.0);
    point.sub(a.add(ab.scale(t))).length()
}

/// Adaptive fractions in [0, 1] over the edge's [t0, t1] range such that the
/// 3D chord between consecutive samples sags less than `chord_tolerance`.
///
/// The second return value is TRUE when [`MAX_EDGE_SPANS`] stopped the
/// subdivision before the chord criterion was satisfied — i.e. the returned
/// chain does NOT meet the tolerance. Callers use it to floor the interior
/// refinement of the adjacent faces: refining a face interior below the
/// accuracy its own boundary can express buys nothing and costs everything
/// (a 215-long ruled strip whose rails want ~7000 spans but are capped at 512
/// drove `refine_interior` to 500k triangles chasing a tolerance the rails
/// missed by ~180x).
pub(super) fn edge_fractions(
    edge: &EdgeRecord,
    chord_tolerance: f64,
) -> Result<(Vec<f64>, bool), String> {
    if edge.degenerate {
        return Ok((vec![0.0, 1.0], false));
    }
    let evaluate = |fraction: f64| -> Result<Vec3, String> {
        edge.curve
            .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
    };
    // Unit tangent of the edge's 3D curve at a fraction, or `None` where the
    // first derivative degenerates (a pole/cusp). A degenerate tangent is
    // treated as "within angle" below so a zero-length tangent never
    // force-splits — there the chord criterion governs.
    let angular_cos = EDGE_ANGULAR_TOL.cos();
    let tangent = |fraction: f64| -> Result<Option<Vec3>, String> {
        let parameter = edge.t0 + (edge.t1 - edge.t0) * fraction;
        let (_, derivative) = edge.curve.deriv1(parameter)?;
        Ok(derivative.normalized().ok())
    };
    let mut fractions = vec![0.0, 1.0];
    // Seed every distinct interior knot of the represented span before the
    // bisection: midpoint chord deflection alone TERMINATES on S-shaped
    // multi-span fitted curves whose midpoint swings back near the chord
    // while interior spans wander — the sampled chain then short-circuits
    // whole lobes of the boundary and the face triangulates a region far
    // larger than its trim (the merged revolve-wall faces of the
    // 2026-07-30 user model meshed at 3x their trimmed area).  Within one
    // polynomial span the midpoint criterion is sound; across spans it is
    // not (Golovanov §4.12: steps must budget the whole span, not probe
    // one midpoint).
    let span = edge.t1 - edge.t0;
    if span.abs() > 1e-12 && edge.curve.knots.len() > 2 * edge.curve.degree {
        let interior =
            &edge.curve.knots[edge.curve.degree..edge.curve.knots.len() - edge.curve.degree];
        for &knot in interior {
            let fraction = (knot - edge.t0) / span;
            if fraction > 1e-9 && fraction < 1.0 - 1e-9 {
                fractions.push(fraction);
            }
        }
        fractions.sort_by(f64::total_cmp);
        fractions.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
        if fractions.len() > MAX_EDGE_SPANS {
            // Keep endpoints, thin the interior uniformly.
            let step = fractions.len() as f64 / MAX_EDGE_SPANS as f64;
            let thinned: Vec<f64> = (0..MAX_EDGE_SPANS)
                .map(|index| fractions[(index as f64 * step) as usize])
                .chain(std::iter::once(1.0))
                .collect();
            fractions = thinned;
        }
    }
    loop {
        let mut refined = Vec::with_capacity(fractions.len() * 2);
        let mut split_any = false;
        for pair in fractions.windows(2) {
            refined.push(pair[0]);
            if fractions.len() <= MAX_EDGE_SPANS {
                let middle = (pair[0] + pair[1]) * 0.5;
                let curve_mid = evaluate(middle)?;
                let chord_sags =
                    deflection_to_chord(curve_mid, evaluate(pair[0])?, evaluate(pair[1])?)
                        > chord_tolerance;
                // Angular (curvature) criterion: split when the edge's 3D-curve
                // unit tangent rotates more than `EDGE_ANGULAR_TOL` across the
                // interval. `t_a · t_b >= cos(EDGE_ANGULAR_TOL)` means the swing
                // is within tolerance; below it we bisect. Scale-independent, so
                // a small-radius circle on a large part still gets ~2π/tol
                // segments instead of a coarse polygon. A degenerate tangent at
                // either end (pole/cusp) leaves the chord criterion in charge.
                let tangent_swings = match (tangent(pair[0])?, tangent(pair[1])?) {
                    (Some(ta), Some(tb)) => ta.dot(tb) < angular_cos,
                    _ => false,
                };
                if chord_sags || tangent_swings {
                    refined.push(middle);
                    split_any = true;
                }
            }
        }
        refined.push(1.0);
        fractions = refined;
        if !split_any || fractions.len() > MAX_EDGE_SPANS {
            break;
        }
    }
    // Both ways the budget can bite — the knot-seed thinning and the bisection
    // break — leave the chain longer than the cap, so this one test covers them.
    // A chain that merely crossed the cap on its LAST needed pass is flagged too,
    // harmlessly: the caller measures the sag it actually achieved, which is then
    // within tolerance and floors nothing.
    let capped = fractions.len() > MAX_EDGE_SPANS;
    Ok((fractions, capped))
}

pub(super) struct EdgeSamples {
    pub(super) fractions: Vec<f64>,
    /// 3D positions; endpoints are the exact shared vertex-record points so
    /// corners coincide across every incident edge.
    pub(super) positions: Vec<Vec3>,
    /// Worst chord sag this chain actually achieves, but ONLY when
    /// [`MAX_EDGE_SPANS`] cut the subdivision short; `0.0` whenever the sampler
    /// converged (the overwhelmingly common case, where the chain is within
    /// tolerance by construction and the floor must stay inert).
    pub(super) worst_sag: f64,
}

/// Where an edge curve crosses a chart boundary of `atlas`, as fractions of the
/// edge's own `[t0, t1]` range.
///
/// A chart boundary is one of the six planes `|x_i| = |x_j|` through the sphere
/// centre, so the crossing is a root of a scalar function of ONE variable along
/// the edge — a bracketed bisection on the curve, never a surface solve, so
/// nothing here can converge onto extrapolated geometry outside the patch.
/// Brackets come from the chord-adaptive samples the edge already has, and only
/// a root where the tied pair of coordinates is the LARGEST pair is on a cube
/// edge; anywhere else the plane runs through a chart interior and means
/// nothing.
fn chart_crossing_fractions(
    edge: &EdgeRecord,
    atlas: &crate::sphere_chart::SphereAtlas,
    fractions: &[f64],
) -> Result<Vec<f64>, String> {
    if edge.degenerate || fractions.len() < 2 {
        return Ok(Vec::new());
    }
    let at = |fraction: f64| -> Result<[f64; 3], String> {
        let point = edge
            .curve
            .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)?;
        Ok(atlas.axis_coordinates(point))
    };
    let mut coordinates = Vec::with_capacity(fractions.len());
    for &fraction in fractions {
        coordinates.push(at(fraction)?);
    }
    // A curve that LIES IN one of the six planes evaluates that plane's function
    // as rounding noise, whose sign flips at random along the edge. Every one of
    // those flips brackets a "root", and each root inserts a sample a few
    // ulps from its neighbours — a cluster of near-duplicate boundary points that
    // no consumer can weld. A crossing has to be a real transversal one, so a
    // bracket counts only when the function is meaningfully non-zero at an end.
    let scale = coordinates
        .iter()
        .flat_map(|x| x.iter().map(|value| value.abs()))
        .fold(0.0, f64::max);
    let significant = 1e-9 * scale;
    let mut crossings = Vec::new();
    for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
        for combination in [1.0f64, -1.0] {
            let value = |x: &[f64; 3]| x[i] - combination * x[j];
            for window in 0..fractions.len() - 1 {
                let (mut lo, mut hi) = (fractions[window], fractions[window + 1]);
                let (mut low_value, high_value) =
                    (value(&coordinates[window]), value(&coordinates[window + 1]));
                if (low_value > 0.0) == (high_value > 0.0) || low_value == high_value {
                    continue;
                }
                if low_value.abs().max(high_value.abs()) <= significant {
                    continue;
                }
                // 60 bisections exhaust the f64 mantissa over any bracket.
                for _ in 0..60 {
                    let middle = 0.5 * (lo + hi);
                    if middle <= lo || middle >= hi {
                        break;
                    }
                    let middle_value = value(&at(middle)?);
                    if (middle_value > 0.0) == (low_value > 0.0) {
                        lo = middle;
                        low_value = middle_value;
                    } else {
                        hi = middle;
                    }
                }
                let root = 0.5 * (lo + hi);
                let x = at(root)?;
                let tied = x[i].abs().max(x[j].abs());
                let third = x[3 - i - j].abs();
                let scale = x[0].abs().max(x[1].abs()).max(x[2].abs());
                if scale > 0.0 && tied > 0.0 && tied >= third - 1e-9 * scale {
                    crossings.push(root);
                }
            }
        }
    }
    Ok(crossings)
}

pub(super) fn sample_all_edges(
    solid: &BrepSolid,
    chord_tolerance: f64,
) -> Result<HashMap<u64, EdgeSamples>, String> {
    // id -> point index built once; the linear `vertices.iter().find(id==)`
    // below was called ~2x per edge, i.e. O(E*V) on every display refresh.
    // Lookup-only, so it yields the exact same point the scan would.
    let vertex_points: FxHashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let vertex_point = |id: u64| -> Result<Vec3, String> {
        vertex_points
            .get(&id)
            .copied()
            .ok_or_else(|| format!("watertight tessellation: missing vertex {id}"))
    };
    // Edges that bound a SPHERICAL face need one extra guarantee: wherever the
    // edge crosses a boundary of that sphere's internal cube atlas, BOTH adjacent
    // faces must carry the crossing as a sample. The sphere's chart tessellation
    // has to split its trim there, and a split the neighbour does not share is a
    // T-junction — a crack. Inserting it HERE, in the one place every face reads
    // its edge samples from, keeps the watertight invariant intact: the criterion
    // is a pure function of the shared edge curve and the adjacent spheres, so
    // both faces still receive byte-identical samples. Edges bounding no sphere
    // are untouched.
    let mut chart_atlases: FxHashMap<u64, Vec<crate::sphere_chart::SphereAtlas>> =
        FxHashMap::default();
    for face in solid.shells.iter().flat_map(|shell| shell.faces.iter()) {
        let Some(atlas) = crate::sphere_chart::SphereAtlas::of_surface(&face.surface) else {
            continue;
        };
        for coedge in face.loops.iter().flat_map(|record| record.coedges.iter()) {
            chart_atlases
                .entry(coedge.edge_id)
                .or_default()
                .push(atlas);
        }
    }
    let mut samples = HashMap::new();
    for edge in &solid.edges {
        let (mut fractions, capped) = edge_fractions(edge, chord_tolerance)?;
        if let Some(atlases) = chart_atlases.get(&edge.id) {
            let mut extra = Vec::new();
            for atlas in atlases {
                extra.extend(chart_crossing_fractions(edge, atlas, &fractions)?);
            }
            if !extra.is_empty() {
                fractions.extend(extra);
                fractions.sort_by(f64::total_cmp);
                // Two of the six planes meet at every cube CORNER, so a curve
                // through one produces two roots a few ulps apart. Merge them:
                // they are one point, and keeping both would seed a sliver.
                fractions.dedup_by(|a, b| (*a - *b).abs() <= 1e-9);
                fractions.retain(|fraction| *fraction >= 0.0 && *fraction <= 1.0);
            }
        }
        let fractions = fractions;
        let start = vertex_point(edge.start_vertex_id)?;
        let end = vertex_point(edge.end_vertex_id)?;
        let mut positions = Vec::with_capacity(fractions.len());
        for (index, fraction) in fractions.iter().enumerate() {
            if index == 0 {
                positions.push(start);
            } else if index == fractions.len() - 1 {
                positions.push(end);
            } else if edge.degenerate {
                positions.push(start);
            } else {
                positions.push(
                    edge.curve
                        .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)?,
                );
            }
        }
        // Only a capped chain can miss the tolerance, so only a capped chain is
        // measured — one extra curve evaluation per span, on the rare edge whose
        // span budget ran out.
        let mut worst_sag = 0.0f64;
        if capped && !edge.degenerate {
            for index in 0..fractions.len() - 1 {
                let middle = (fractions[index] + fractions[index + 1]) * 0.5;
                let curve_mid = edge
                    .curve
                    .evaluate(edge.t0 + (edge.t1 - edge.t0) * middle)?;
                worst_sag = worst_sag.max(deflection_to_chord(
                    curve_mid,
                    positions[index],
                    positions[index + 1],
                ));
            }
        }
        samples.insert(
            edge.id,
            EdgeSamples {
                fractions,
                positions,
                worst_sag,
            },
        );
    }
    Ok(samples)
}

#[derive(Clone, Copy)]
pub(super) struct FaceVertex {
    pub(super) uv: [f64; 2],
    pub(super) position: Vec3,
}
