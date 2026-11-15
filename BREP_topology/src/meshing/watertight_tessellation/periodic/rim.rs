use super::*;

/// Rotate at the largest seam jump, then orient toward increasing periodic
/// coordinate. Sorting would interleave locally reversing boundary samples.
pub(super) fn rotate_rim_to_seam(chain: &[FaceVertex], p_is_u: bool) -> Vec<FaceVertex> {
    let p = usize::from(!p_is_u);
    let n = chain.len();
    let (mut seam, mut worst) = (0, -1.0);
    for index in 0..n {
        let distance = (chain[(index + 1) % n].uv[p] - chain[index].uv[p]).abs();
        if distance > worst {
            worst = distance;
            seam = (index + 1) % n;
        }
    }
    let mut points: Vec<_> = (0..n).map(|offset| chain[(seam + offset) % n]).collect();
    if points[0].uv[p] > points.last().unwrap().uv[p] {
        points.reverse();
    }
    points
}

/// Fill missing seam corners with the opposite endpoint's 3D position so
/// neighboring faces weld at the two parameter images of the same seam.
pub(super) fn complete_rim_seam(points: &mut Vec<FaceVertex>, range: [f64; 2], p_is_u: bool) {
    let p = usize::from(!p_is_u);
    let [p0, p1] = range;
    let epsilon = 1e-6 * (p1 - p0);
    let front = points[0];
    let back = *points.last().unwrap();
    if back.uv[p] < p1 - epsilon {
        let mut uv = back.uv;
        uv[p] = p1;
        points.push(FaceVertex {
            uv,
            position: front.position,
        });
    }
    if front.uv[p] > p0 + epsilon {
        let mut uv = front.uv;
        uv[p] = p0;
        points.insert(
            0,
            FaceVertex {
                uv,
                position: back.position,
            },
        );
    }
}

/// Split a seam-straddling hole into right/left arcs oriented toward increasing
/// cross coordinate. Require exactly one contiguous run on each side.
pub(super) fn split_seam_arcs(
    chain: &[FaceVertex],
    range: [f64; 2],
    p_is_u: bool,
) -> Option<(Vec<FaceVertex>, Vec<FaceVertex>)> {
    let [p0, p1] = range;
    let mid = 0.5 * (p0 + p1);
    let p = usize::from(!p_is_u);
    let q = 1 - p;
    let mk = |p, q| if p_is_u { [p, q] } else { [q, p] };
    let n = chain.len();
    if n < 4 {
        return None;
    }
    let side = |i: usize| chain[i].uv[p] > mid;
    let start = (0..n).find(|&i| side(i) != side((i + n - 1) % n))?;
    let mut runs: Vec<Vec<FaceVertex>> = Vec::new();
    let mut current: Vec<FaceVertex> = Vec::new();
    let mut current_side = side(start);
    for k in 0..n {
        let i = (start + k) % n;
        if side(i) != current_side {
            runs.push(std::mem::take(&mut current));
            current_side = side(i);
        }
        current.push(chain[i]);
    }
    if !current.is_empty() {
        runs.push(current);
    }
    if runs.len() != 2 {
        return None;
    }
    let second = runs.pop().unwrap();
    let first = runs.pop().unwrap();
    let (mut right, mut left) = if first[0].uv[p] > mid {
        (first, second)
    } else {
        (second, first)
    };
    // loop_chain omits the far crossing shared with the next coedge. Restore
    // it using this seam's UV and the other arc's exact 3D start position.
    let right_crossing = right[0];
    let left_crossing = left[0];
    right.push(FaceVertex {
        uv: mk(p1, left_crossing.uv[q]),
        position: left_crossing.position,
    });
    left.push(FaceVertex {
        uv: mk(p0, right_crossing.uv[q]),
        position: right_crossing.position,
    });
    if right[0].uv[q] > right.last().unwrap().uv[q] {
        right.reverse();
    }
    if left[0].uv[q] > left.last().unwrap().uv[q] {
        left.reverse();
    }
    Some((right, left))
}

