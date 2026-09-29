use super::*;
use super::earclip::crossing_parity_inside;

/// Boundary chain of one loop: shared samples in traversal order.
pub(in crate::watertight_tessellation) fn loop_chain(
    coedges: &[CoedgeRecord],
    samples: &HashMap<u64, EdgeSamples>,
) -> Result<Vec<FaceVertex>, String> {
    loop_chain_offset(coedges, samples, &[])
}

/// As [`loop_chain`], but shift each coedge's uv samples by `offsets[i]` (empty
/// = no shift). The offsets unwrap an implicit periodic seam onto the covering
/// plane so the boundary is one continuous polygon instead of a self-crossing
/// seam-hopping one.
pub(in crate::watertight_tessellation) fn loop_chain_offset(
    coedges: &[CoedgeRecord],
    samples: &HashMap<u64, EdgeSamples>,
    offsets: &[[f64; 2]],
) -> Result<Vec<FaceVertex>, String> {
    let mut chain = Vec::new();
    for (coedge_index, coedge) in coedges.iter().enumerate() {
        let offset = offsets.get(coedge_index).copied().unwrap_or([0.0, 0.0]);
        let edge_samples = samples
            .get(&coedge.edge_id)
            .ok_or("watertight tessellation: missing edge samples")?;
        let [p0, p1] = coedge.pcurve.domain()?;
        let count = edge_samples.fractions.len();
        // Skip the final sample of each coedge — it coincides with the next
        // coedge's first sample (shared topological vertex).
        for index in 0..count - 1 {
            let (edge_index, traversal_fraction) = if coedge.forward {
                (index, edge_samples.fractions[index])
            } else {
                (
                    count - 1 - index,
                    1.0 - edge_samples.fractions[count - 1 - index],
                )
            };
            let uv = coedge
                .pcurve
                .evaluate(p0 + (p1 - p0) * traversal_fraction)?;
            chain.push(FaceVertex {
                uv: [uv.x + offset[0], uv.y + offset[1]],
                position: edge_samples.positions[edge_index],
            });
        }
    }
    Ok(chain)
}

pub(in crate::watertight_tessellation) fn signed_area(chain: &[FaceVertex]) -> f64 {
    let mut area = 0.0;
    for index in 0..chain.len() {
        let a = chain[index].uv;
        let b = chain[(index + 1) % chain.len()].uv;
        area += a[0] * b[1] - b[0] * a[1];
    }
    area * 0.5
}

/// Diagonal of the 3D bounding box of a chain's vertex positions. ~0 for a pole
/// loop (every vertex collapses onto one point).
pub(in crate::watertight_tessellation) fn chain_extent(chain: &[FaceVertex]) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in chain {
        let p = [v.position.x, v.position.y, v.position.z];
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let d = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

/// Drop hole loops that enclose no parameter-space area — a VERTEX_LOOP point
/// (torus/ring contact tangency), or a zero-WIDTH seam slit whose two sides
/// collapse onto the same meridian once the periodic seam is unwrapped (a ring
/// pierced on its seam). They are not material holes: bridging one teleports a
/// chord onto the seam (the domain-spanning double-cover) and constraining one
/// is a degenerate PSLG the CDT rejects, so a whole full-wrap wall then falls
/// back to the spraying ear-clip. The threshold is gauged against the parameter
/// domain, so a genuinely thin (but positive-area) hole always survives.
pub(in crate::watertight_tessellation) fn drop_zero_area_holes(holes: Vec<Vec<FaceVertex>>, domain_area: f64) -> Vec<Vec<FaceVertex>> {
    let eps = 1e-9 * domain_area.abs().max(1e-30);
    holes
        .into_iter()
        .filter(|hole| signed_area(hole).abs() > eps)
        .collect()
}

/// Move one independently-unwrapped hole into the same periodic covering copy
/// as its outer loop. `loop_seam_offsets` makes each loop continuous, but its
/// integer-period origin is arbitrary: an outer loop near `u = 1` and a hole
/// near the equivalent `u = 0` copy are both individually valid, yet bridging
/// them draws a domain-wide diagonal and fills the wrong sheet.
///
/// This is deliberately conservative. An already-contained hole is untouched;
/// a loop spanning most of a closed direction is not a hole we can safely
/// rebase; and a candidate is accepted only when the whole sampled hole lies
/// inside/on the outer polygon without crossing it. Ambiguous multiple copies
/// keep the old data.
pub(in crate::watertight_tessellation) fn align_periodic_hole_to_outer(
    outer: &[FaceVertex],
    hole: &mut [FaceVertex],
    closed_u: bool,
    closed_v: bool,
    u_period: f64,
    v_period: f64,
) -> bool {
    if outer.len() < 3 || hole.is_empty() || !(closed_u || closed_v) {
        return false;
    }
    let representative = |chain: &[FaceVertex]| -> [f64; 2] {
        let sum = chain.iter().fold([0.0, 0.0], |sum, vertex| {
            [sum[0] + vertex.uv[0], sum[1] + vertex.uv[1]]
        });
        [sum[0] / chain.len() as f64, sum[1] / chain.len() as f64]
    };
    let bounds = |chain: &[FaceVertex], axis: usize| -> (f64, f64) {
        chain
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), vertex| {
                (lo.min(vertex.uv[axis]), hi.max(vertex.uv[axis]))
            })
    };
    let (hole_u0, hole_u1) = bounds(hole, 0);
    let (hole_v0, hole_v1) = bounds(hole, 1);
    let (outer_u0, outer_u1) = bounds(outer, 0);
    let (outer_v0, outer_v1) = bounds(outer, 1);
    if (closed_u && (!(u_period > 0.0) || hole_u1 - hole_u0 >= 0.75 * u_period))
        || (closed_v && (!(v_period > 0.0) || hole_v1 - hole_v0 >= 0.75 * v_period))
        || (closed_u && outer_u1 - outer_u0 > 1.5 * u_period)
        || (closed_v && outer_v1 - outer_v0 > 1.5 * v_period)
    {
        return false;
    }

    let scale = (outer_u1 - outer_u0)
        .abs()
        .max((outer_v1 - outer_v0).abs())
        .max(u_period.abs())
        .max(v_period.abs())
        .max(1.0);
    let coordinate_epsilon = 1e-9 * scale;
    let area_epsilon = 1e-12 * scale * scale;
    let point_inside_or_on = |point: [f64; 2]| {
        crossing_parity_inside(outer.len(), |index| outer[index].uv, point)
            || (0..outer.len()).any(|index| {
                let a = outer[index].uv;
                let b = outer[(index + 1) % outer.len()].uv;
                triangle_area(a, b, point).abs() <= area_epsilon
                    && point[0] >= a[0].min(b[0]) - coordinate_epsilon
                    && point[0] <= a[0].max(b[0]) + coordinate_epsilon
                    && point[1] >= a[1].min(b[1]) - coordinate_epsilon
                    && point[1] <= a[1].max(b[1]) + coordinate_epsilon
            })
    };
    let contained_after = |shift: [f64; 2]| {
        if hole
            .iter()
            .any(|vertex| !point_inside_or_on([vertex.uv[0] + shift[0], vertex.uv[1] + shift[1]]))
        {
            return false;
        }
        for hole_index in 0..hole.len() {
            let mut a = hole[hole_index].uv;
            let mut b = hole[(hole_index + 1) % hole.len()].uv;
            a[0] += shift[0];
            a[1] += shift[1];
            b[0] += shift[0];
            b[1] += shift[1];
            for outer_index in 0..outer.len() {
                if segments_properly_cross(
                    a,
                    b,
                    outer[outer_index].uv,
                    outer[(outer_index + 1) % outer.len()].uv,
                    area_epsilon,
                ) {
                    return false;
                }
            }
        }
        true
    };
    // Preserve an already-valid covering copy exactly.
    if contained_after([0.0, 0.0]) {
        return false;
    }

    let hole_center = representative(hole);
    let outer_center = representative(outer);
    let base_u = if closed_u {
        ((outer_center[0] - hole_center[0]) / u_period).round() as i32
    } else {
        0
    };
    let base_v = if closed_v {
        ((outer_center[1] - hole_center[1]) / v_period).round() as i32
    } else {
        0
    };
    let u_candidates: Vec<i32> = if closed_u {
        ((base_u - 1)..=(base_u + 1)).collect()
    } else {
        vec![0]
    };
    let v_candidates: Vec<i32> = if closed_v {
        ((base_v - 1)..=(base_v + 1)).collect()
    } else {
        vec![0]
    };
    let mut candidates: Vec<(i32, i32, [f64; 2])> = Vec::new();
    for &ku in &u_candidates {
        for &kv in &v_candidates {
            let shift = [ku as f64 * u_period, kv as f64 * v_period];
            if (ku == 0 && kv == 0) || !contained_after(shift) {
                continue;
            }
            candidates.push((ku, kv, shift));
        }
    }
    // More than one contained copy means the outer itself does not identify a
    // unique sheet. Refuse instead of making a periodic-region guess.
    let [(_, _, shift)] = candidates.as_slice() else {
        return false;
    };
    for vertex in hole {
        vertex.uv[0] += shift[0];
        vertex.uv[1] += shift[1];
    }
    true
}

