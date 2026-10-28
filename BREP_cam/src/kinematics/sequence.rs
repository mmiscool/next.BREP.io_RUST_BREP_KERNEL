//! Sequence solving: one pose per CL point with joint limits, unwinding of
//! continuous axes and the shortest rotary travel between neighbours
//! (section 3).

use super::{Chain, Joint, KinematicsError, KinematicsOptions, Point3, Pose};

/// One CL point's chosen pose.
#[derive(Clone, Debug, PartialEq)]
pub struct SolvedPoint {
    pub pose: Pose,
    /// The branch differs from the previous point's: the rotaries cannot
    /// interpolate continuously while cutting; the post repositions.
    pub branch_change: bool,
    /// Largest rotary move from the previous pose, degrees (0 for the first).
    pub travel_deg: f64,
}

/// Every representative `angle + 360k` of `angle_deg` the joint accepts.
/// A limited joint lists each turn inside `[lo, hi]`; a continuous joint
/// lists the two turns around `prev` (or the angle itself).
pub fn candidates(joint: &Joint, angle_deg: f64, prev: Option<f64>) -> Vec<f64> {
    match joint.limits_deg {
        Some([lo, hi]) => {
            let k_lo = ((lo - angle_deg) / 360.0).ceil() as i64;
            let k_hi = ((hi - angle_deg) / 360.0).floor() as i64;
            (k_lo..=k_hi).map(|k| angle_deg + 360.0 * k as f64).collect()
        }
        None => match prev {
            // The two turns bracketing `prev`; `pick` keeps the nearer one
            // and settles a tie on the smaller value.
            Some(p) => {
                let k = ((p - angle_deg) / 360.0).floor();
                vec![angle_deg + 360.0 * k, angle_deg + 360.0 * (k + 1.0)]
            }
            None => vec![angle_deg],
        },
    }
}

/// Nearest candidate to `prev` (smallest magnitude when there is none); ties
/// go to the smaller value.
fn pick(cands: &[f64], prev: Option<f64>) -> Option<f64> {
    let key = |c: &f64| match prev {
        Some(p) => ((c - p).abs(), *c),
        None => (c.abs(), *c),
    };
    cands
        .iter()
        .copied()
        .min_by(|a, b| key(a).0.total_cmp(&key(b).0).then(a.total_cmp(b)))
}

/// The chosen pose of one CL point given the previous rotary angles: the
/// (branch, unwinding) with the least travel (max over joints, then sum, then
/// the lower branch), every angle inside its joint's limits. `index` only
/// labels a `LimitViolation`.
pub fn choose_pose(
    chain: &Chain,
    tip: Point3,
    axis: Option<Point3>,
    prev: Option<&[f64]>,
    index: usize,
    options: &KinematicsOptions,
) -> Result<(Pose, f64), KinematicsError> {
    let poses = chain.inverse(tip, axis, prev, options)?;
    let mut best: Option<((f64, f64, u8), Pose)> = None;
    let mut blocked: Option<String> = None;
    'branch: for pose in poses {
        let mut angles = Vec::with_capacity(chain.joints.len());
        let mut max_travel = 0.0f64;
        let mut sum_travel = 0.0f64;
        for (j, joint) in chain.joints.iter().enumerate() {
            let p = prev.map(|v| v[j]);
            let cands = candidates(joint, pose.angles_deg[j], p);
            let Some(chosen) = pick(&cands, p) else {
                blocked.get_or_insert_with(|| joint.name.clone());
                continue 'branch;
            };
            let travel = match p {
                Some(p) => (chosen - p).abs(),
                None => chosen.abs(),
            };
            max_travel = max_travel.max(travel);
            sum_travel += travel;
            angles.push(chosen);
        }
        let cost = (max_travel, sum_travel, pose.branch);
        let better = match &best {
            None => true,
            Some((c, _)) => cost.0.total_cmp(&c.0).then(cost.1.total_cmp(&c.1)).then(cost.2.cmp(&c.2)).is_lt(),
        };
        if better {
            let linear = chain.linear_for(tip, &angles, options);
            best = Some((cost, Pose { linear, angles_deg: angles, branch: pose.branch, singular: pose.singular }));
        }
    }
    match best {
        Some((cost, pose)) => Ok((pose, if prev.is_some() { cost.0 } else { 0.0 })),
        None => Err(KinematicsError::LimitViolation { index, joint: blocked.unwrap_or_default() }),
    }
}

/// Solve a sequence of CL points. `start` is the rotary position before the
/// first point (continuity and held singular angles); `None` starts from
/// zero without charging travel to the first point.
pub fn solve_sequence(
    chain: &Chain,
    points: &[(Point3, Option<Point3>)],
    start: Option<&[f64]>,
    options: &KinematicsOptions,
) -> Result<Vec<SolvedPoint>, KinematicsError> {
    options.validate()?;
    let mut prev: Option<Vec<f64>> = start.map(|s| s.to_vec());
    if let Some(p) = &prev {
        if p.len() != chain.joints.len() {
            return Err(KinematicsError::InvalidInput("start angles do not match the chain".into()));
        }
    }
    let mut prev_branch: Option<u8> = None;
    let mut out = Vec::with_capacity(points.len());
    for (index, (tip, axis)) in points.iter().enumerate() {
        let (pose, travel_deg) = choose_pose(chain, *tip, *axis, prev.as_deref(), index, options)?;
        let branch_change = prev_branch.map_or(false, |b| b != pose.branch);
        prev = Some(pose.angles_deg.clone());
        prev_branch = Some(pose.branch);
        out.push(SolvedPoint { pose, branch_change, travel_deg });
    }
    Ok(out)
}
