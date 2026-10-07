use crate::{KernelRefusal, KernelStage};
use super::*;

/// In-segment stations per chain segment, for a wall that does not fold.
pub(in crate::blend) const CHAIN_PER_SEGMENT: usize = 24;

/// The TOP of the ladder a folding chain is re-marched up (24, 48, 96, ...).
///
/// A carve traces the crease on the surface the fit produced, so it is sound at
/// any budget, but the wall it carves is only as close to the rolling ball as
/// the fit. So the carve does not pick a budget: it climbs, and stops at the
/// first rung whose fitted rails pass the rolling ball's own contacts, re-solved
/// halfway between stations, to within `intersection_fit`
/// (`chain/closed.rs`). The 2026-09-02 collar stops at the rung the record
/// states. This is only the ceiling, and it is a RESOURCE bound, not an
/// accuracy one: a 768-station segment is a 1540-row control net, and a wall
/// that still misses there is refused by name rather than shipped.
pub(super) const CHAIN_CARVE_MAX_PER_SEGMENT: usize = 768;

/// What a march does when the wall it would build folds through itself.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(in crate::blend) enum FoldPolicy {
    /// Refuse by name, terminally (`blend/fold.rs`).
    Refuse,
    /// Build it anyway, because the caller is about to CARVE the fold out
    /// (`blend/carve.rs`) and has already re-marched at the budget that needs.
    Carve,
}

const CHAIN_OVERSHOOT: usize = 3;

/// One marched chain station, tagged with its segment and its position
/// (which may overshoot the segment for pcurve fitting).
#[derive(Clone)]
pub(in crate::blend) struct ChainSample {
    pub(in crate::blend) segment: usize,
    /// Station position within the segment: 0..CHAIN_PER_SEGMENT are
    /// in-segment; negatives and > CHAIN_PER_SEGMENT are overshoot.
    pub(in crate::blend) position: isize,
    /// Where between `position` and the next grid station a station inserted
    /// by local refinement sits, in (0, 1); 0 for every marched grid station.
    pub(in crate::blend) offset: f64,
    /// The edge parameter the station's section plane was taken at.
    pub(in crate::blend) t: f64,
    pub(in crate::blend) station: Station,
    /// Global chord parameter (filled after the march).
    pub(in crate::blend) parameter: f64,
    /// The rolling ball's EXACT contacts halfway to the next position — solved,
    /// not interpolated — so a fitted row can be measured where an interpolant
    /// is worst. Only a march for a carve pays for them.
    pub(in crate::blend) midpoint: Option<[Vec3; 2]>,
}

/// A chain station whose CONTINUATION solve did not converge: the tangency
/// Newton, seeded from its solved neighbour (after the branch guard's halving
/// retries), found no root at this budget.  A finer rung is a nearer seed, so
/// the closed ladder may climb on it; the refusal itself is kept verbatim and
/// is what a caller receives at the ladder's top or outside the ladder.
pub(in crate::blend) struct ChainStationFailure {
    pub(in crate::blend) segment: usize,
    pub(in crate::blend) position: isize,
    pub(in crate::blend) t: f64,
    pub(in crate::blend) from_position: isize,
    pub(in crate::blend) from_t: f64,
    pub(in crate::blend) seed_uv: [f64; 4],
    pub(in crate::blend) error: KernelRefusal,
}

/// How a chain march stops: a typed station-continuation failure, or any
/// other refusal, which passes through untouched.
pub(in crate::blend) enum ChainMarchError {
    Station(ChainStationFailure),
    /// Local refinement only: an inserted station, continued from its left
    /// neighbour, is not on the branch of the RETAINED right station (the
    /// existing `branch_hop` reading, carried as `error`). The rung cannot be
    /// refined coherently; the caller's ladder decides, as for a coarse rung.
    Incoherent(ChainStationFailure),
    Other(KernelRefusal),
}

impl ChainMarchError {
    /// The refusal exactly as the march produced it.
    pub(in crate::blend) fn into_refusal(self) -> KernelRefusal {
        match self {
            ChainMarchError::Station(failure) | ChainMarchError::Incoherent(failure) => failure.error,
            ChainMarchError::Other(error) => error,
        }
    }
}

impl From<KernelRefusal> for ChainMarchError {
    fn from(error: KernelRefusal) -> Self {
        ChainMarchError::Other(error)
    }
}

/// The tangency Newton's own non-convergence, by its typed slug
/// (`stations::solve_station_centered`); nothing else is a station failure.
fn is_tangency_newton(error: &KernelRefusal) -> bool {
    matches!(&error.class, crate::RefusalClass::NonConvergence { what } if what == "tangency_newton")
}

pub(in crate::blend) fn march_chain(
    segments: &[ChainSegment<'_>],
    radius: f64,
    open: bool,
    per_segment: usize,
    fold: FoldPolicy,
) -> Result<Vec<ChainSample>, KernelRefusal> {
    march_chain_typed(segments, radius, open, per_segment, fold).map_err(ChainMarchError::into_refusal)
}

/// [`march_chain`] with a station-continuation non-convergence kept typed, so
/// the closed ladder can tell it from every other refusal.
pub(in crate::blend) fn march_chain_typed(
    segments: &[ChainSegment<'_>],
    radius: f64,
    open: bool,
    per_segment: usize,
    fold: FoldPolicy,
) -> Result<Vec<ChainSample>, ChainMarchError> {
    let mut samples = Vec::new();
    for (index, segment) in segments.iter().enumerate() {
        // Signs from this segment's own cross-section seed.
        let (rho1, rho2) = signed_radii(
            segment.edge,
            segment.first.face,
            segment.first.coedge,
            segment.second.face,
            segment.second.coedge,
            radius,
        )?;
        let rho = [rho1, rho2];
        let surface1 = &segment.first.face.surface;
        let surface2 = &segment.second.face.surface;
        let edge = segment.edge;
        let span = edge.t1 - edge.t0;
        let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius)?;
        crate::report_scale_migration("blend_march_chain", scale, || {
            edge.curve
                .evaluate(edge.t0)
                .unwrap_or_default()
                .length()
                .max(1.0)
        });
        let chain_t = |position: isize| -> f64 {
            let fraction = position as f64 / per_segment as f64;
            if segment.forward {
                edge.t0 + span * fraction
            } else {
                edge.t1 - span * fraction
            }
        };
        let mid_position = per_segment as isize / 2;
        let seed_mid = {
            let t = chain_t(mid_position);
            let uv1 = edge_uv_on_face(segment.first.coedge, edge, t)?;
            let uv2 = edge_uv_on_face(segment.second.coedge, edge, t)?;
            [uv1[0], uv1[1], uv2[0], uv2[1]]
        };
        // The section plane at `t`, advancing past a stationary open end by the
        // march's own chord (`march_section`).
        let station_step = span.abs() / per_segment as f64;
        let section_at = |t: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
            march_section(edge, t, station_step, "blend chain march")
        };
        let solve_centered = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3), KernelRefusal> {
            let (section_point, tangent) = section_at(t)?;
            let tangent = if segment.forward {
                tangent
            } else {
                tangent.scale(-1.0)
            };
            solve_station_centered(
                surface1,
                surface2,
                rho,
                seed,
                section_point,
                tangent,
                scale,
            )
        };
        let solve_at = |t: f64, seed: [f64; 4]| -> Result<[f64; 4], KernelRefusal> {
            solve_centered(t, seed).map(|(uv, _)| uv)
        };
        let low = -(CHAIN_OVERSHOOT as isize);
        let high = (per_segment + CHAIN_OVERSHOOT) as isize;
        let count = (high - low + 1) as usize;
        let mut solutions = vec![[0.0f64; 4]; count];
        let mut centers = vec![Vec3::default(); count];
        let index_of = |position: isize| (position - low) as usize;
        // Every station but the middle one is the CONTINUATION of the
        // station it is seeded from: an undamped Newton can converge on the
        // far crossing of the section plane, and the guard retries such a
        // hop from a nearer seed (`stations::continue_station`).  The open
        // chain a box notch leaves of a 20° cylinder crossing's loop — the
        // crotch arc and the sliver past the thin cylinder's seam — hopped
        // here at 24 stations a segment (2026-09-26): its first station
        // past the crotch moved 3.06 and 4.10 periods, the centre 6.4.
        let hop = |left: &Continued, right: &Continued| {
            branch_hop([surface1, surface2], edge, &|_| rho, left, right)
        };
        let continue_from = |solutions: &mut [[f64; 4]],
                                 centers: &mut [Vec3],
                                 position: isize,
                                 from: isize|
         -> Result<(), ChainMarchError> {
            let left = Continued {
                t: chain_t(from),
                uv: solutions[index_of(from)],
                center: Some(centers[index_of(from)]),
            };
            let node = continue_station(&solve_centered, &hop, &left, chain_t(position), 0)
                .map_err(|error| {
                    if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1") {
                        eprintln!(
                            "blend chain: segment {index} position {position} (t {:.9}) from position {from}: {error}",
                            chain_t(position)
                        );
                    }
                    if is_tangency_newton(&error) {
                        ChainMarchError::Station(ChainStationFailure {
                            segment: index,
                            position,
                            t: chain_t(position),
                            from_position: from,
                            from_t: left.t,
                            seed_uv: left.uv,
                            error,
                        })
                    } else {
                        ChainMarchError::Other(error)
                    }
                })?;
            solutions[index_of(position)] = node.uv;
            centers[index_of(position)] = node.center.expect("a solved station has a centre");
            Ok(())
        };
        // The segment's OWN stations first, out from the middle, and the
        // overshoot past each end only after the fold check below has read
        // them: the overshoot evaluates both carriers past the segment, where
        // a Newton may not converge at all, and a march that stops there
        // would hide a fold its own stations already prove. Each station is
        // seeded by its neighbour either way, so the solutions are the same.
        let segment_end = per_segment as isize;
        {
            let (uv, center) = solve_centered(chain_t(mid_position), seed_mid).map_err(|error| {
                if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1") {
                    eprintln!("blend chain: segment {index} middle station: {error}");
                }
                error
            })?;
            solutions[index_of(mid_position)] = uv;
            centers[index_of(mid_position)] = center;
        }
        for position in (0..mid_position).rev() {
            continue_from(&mut solutions, &mut centers, position, position + 1)?;
        }
        for position in mid_position + 1..=segment_end {
            continue_from(&mut solutions, &mut centers, position, position - 1)?;
        }
        // Does the wall this segment would carry FOLD? Measured on the ball
        // centre curve the stations already sit on, by re-solving the tangency
        // system either side of each — the station spacing itself is far too
        // coarse to read a bend the size of the radius (`blend/fold.rs`).
        {
            let probe = |t: f64,
                         seed: [f64; 4]|
             -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
                let uv = solve_at(t, seed)?;
                let (section_point, tangent) = section_at(t)?;
                let tangent = if segment.forward {
                    tangent
                } else {
                    tangent.scale(-1.0)
                };
                let (_, p1, p2, center) =
                    tangency_residual(surface1, surface2, rho, uv, section_point, tangent)?;
                Ok((uv, p1, p2, center))
            };
            let probed: Vec<(f64, [f64; 4])> = (0..=per_segment as isize)
                .map(|position| (chain_t(position), solutions[index_of(position)]))
                .collect();
            let radius_at = |_: f64| radius.abs();
            if fold == FoldPolicy::Refuse {
                check_wall_fold(
                    &radius_at,
                    span,
                    &probed,
                    &probe,
                    segment.edge,
                    [segment.first.face, segment.second.face],
                    scale,
                )?;
            }
        }
        for position in (low..0).rev() {
            continue_from(&mut solutions, &mut centers, position, position + 1)?;
        }
        for position in segment_end + 1..=high {
            continue_from(&mut solutions, &mut centers, position, position - 1)?;
        }
        // Halfway contacts, for the closed chain's accuracy ladder — every
        // closed chain, not only one being carved: a collar that does not fold
        // is built from the same interpolated rails and can miss its carriers
        // just as far (9.4e-4 off its torus on the 2026-09-02 two-torus collar,
        // built at the first rung with nothing measuring it).
        let mut midpoints: Vec<Option<[Vec3; 2]>> = vec![None; count];
        if !open {
            for position in 0..per_segment as isize {
                let fraction = (position as f64 + 0.5) / per_segment as f64;
                let t = if segment.forward {
                    edge.t0 + span * fraction
                } else {
                    edge.t1 - span * fraction
                };
                // The halfway contact is the CONTINUATION of its left station
                // under the same branch guard every station takes: its first
                // attempt is the plain solve from the left seed it always was,
                // so a contact that converges on the left station's branch is
                // unchanged to the bit. Where that plain solve does not converge
                // (the 20° seam-split crotch, t 0.208 -> 0.25, needs two seed
                // halvings) the failure is a typed station failure, so the
                // closed ladder can climb on it like any station's.
                let left = Continued {
                    t: chain_t(position),
                    uv: solutions[index_of(position)],
                    center: Some(centers[index_of(position)]),
                };
                let uv = continue_station(&solve_centered, &hop, &left, t, 0)
                    .map_err(|error| {
                        if is_tangency_newton(&error) {
                            ChainMarchError::Station(ChainStationFailure {
                                segment: index,
                                position,
                                t,
                                from_position: position,
                                from_t: left.t,
                                seed_uv: left.uv,
                                error,
                            })
                        } else {
                            ChainMarchError::Other(error)
                        }
                    })?
                    .uv;
                let (section_point, tangent) = section_at(t)?;
                let tangent = if segment.forward {
                    tangent
                } else {
                    tangent.scale(-1.0)
                };
                let (_, p1, p2, _) =
                    tangency_residual(surface1, surface2, rho, uv, section_point, tangent)?;
                midpoints[index_of(position)] = Some([p1, p2]);
            }
        }
        for position in low..=high {
            let t = chain_t(position);
            let uv = solutions[index_of(position)];
            let (section_point, tangent) = section_at(t)?;
            let tangent = if segment.forward {
                tangent
            } else {
                tangent.scale(-1.0)
            };
            let (_, p1, p2, center) =
                tangency_residual(surface1, surface2, rho, uv, section_point, tangent)?;
            let n1 = raw_normal(surface1, uv[0], uv[1])?;
            let n2 = raw_normal(surface2, uv[2], uv[3])?;
            let cos_alpha = rho1.signum() * rho2.signum() * n1.dot(n2);
            let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
            if weight <= 1e-6 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a chain station").into());
            }
            let apex = apex_point(p1, n1, p2, n2, center)?;
            samples.push(ChainSample {
                segment: index,
                position,
                offset: 0.0,
                t,
                station: Station {
                    uv1: [uv[0], uv[1]],
                    uv2: [uv[2], uv[3]],
                    p1,
                    p2,
                    center,
                    weight,
                    apex,
                },
                parameter: 0.0,
                midpoint: midpoints[index_of(position)],
            });
        }
    }
    assign_chain_parameters(&mut samples, segments.len(), per_segment, open);
    Ok(samples)
}

/// LOCAL REFINEMENT of a closed chain's march (`chain/closed.rs`): one station
/// at the halfway edge parameter of every failing in-segment interval, each
/// named by its left sample. The station is the CONTINUATION of that left
/// station under the march's own branch guard (`stations::continue_station`),
/// its halfway contacts and the left station's (now over a half interval) are
/// re-solved as the march solves them, and the new stations are probed for a
/// fold exactly as the march probes its own. Grid stations — the junctions
/// with them — are never moved or re-solved. Chord parameters are NOT
/// re-assigned here (`assign_chain_parameters`).
pub(in crate::blend) fn insert_chain_stations(
    segments: &[ChainSegment<'_>],
    radius: f64,
    per_segment: usize,
    samples: &mut Vec<ChainSample>,
    failing: &[usize],
) -> Result<(), ChainMarchError> {
    let mut touched: Vec<usize> = failing.iter().map(|&index| samples[index].segment).collect();
    touched.sort_unstable();
    touched.dedup();
    let lefts: Vec<(usize, isize, f64)> = failing
        .iter()
        .map(|&index| (samples[index].segment, samples[index].position, samples[index].offset))
        .collect();
    for index in touched {
        let segment = &segments[index];
        let (rho1, rho2) = signed_radii(
            segment.edge,
            segment.first.face,
            segment.first.coedge,
            segment.second.face,
            segment.second.coedge,
            radius,
        )?;
        let rho = [rho1, rho2];
        let surface1 = &segment.first.face.surface;
        let surface2 = &segment.second.face.surface;
        let edge = segment.edge;
        let span = edge.t1 - edge.t0;
        let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius)?;
        let station_step = span.abs() / per_segment as f64;
        let section_at = |t: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
            let (section_point, tangent) = march_section(edge, t, station_step, "blend chain march")?;
            Ok((section_point, if segment.forward { tangent } else { tangent.scale(-1.0) }))
        };
        let solve_centered = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3), KernelRefusal> {
            let (section_point, tangent) = section_at(t)?;
            solve_station_centered(surface1, surface2, rho, seed, section_point, tangent, scale)
        };
        let hop = |left: &Continued, right: &Continued| {
            branch_hop([surface1, surface2], edge, &|_| rho, left, right)
        };
        let contacts_at = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
            let (uv, _) = solve_centered(t, seed)?;
            let (section_point, tangent) = section_at(t)?;
            let (_, p1, p2, center) = tangency_residual(surface1, surface2, rho, uv, section_point, tangent)?;
            Ok((uv, p1, p2, center))
        };
        // A halfway contact continued from `from` under the branch guard, as
        // the march solves its own; a non-converging Newton is a typed station
        // failure at the halfway parameter.
        let halfway = |from: &Continued, position: isize, t: f64| -> Result<(Vec3, Vec3), ChainMarchError> {
            let node = continue_station(&solve_centered, &hop, from, t, 0).map_err(|error| {
                if is_tangency_newton(&error) {
                    ChainMarchError::Station(ChainStationFailure {
                        segment: index,
                        position,
                        t,
                        from_position: position,
                        from_t: from.t,
                        seed_uv: from.uv,
                        error,
                    })
                } else {
                    ChainMarchError::Other(error)
                }
            })?;
            let (section_point, tangent) = section_at(t)?;
            let (_, p1, p2, _) = tangency_residual(surface1, surface2, rho, node.uv, section_point, tangent)?;
            Ok((p1, p2))
        };
        for &(_, position, offset) in lefts.iter().filter(|left| left.0 == index) {
            let left_index = samples
                .iter()
                .position(|sample| sample.segment == index && sample.position == position && sample.offset == offset)
                .expect("refined interval's left station");
            // The next station along the segment: a grid station, the
            // segment's own end station (position `per_segment`), or one
            // inserted earlier in this round.
            let right_index = (0..samples.len())
                .filter(|&other| {
                    samples[other].segment == index
                        && samples[other].position <= per_segment as isize
                        && chain_order(&samples[other], &samples[left_index]) == std::cmp::Ordering::Greater
                })
                .min_by(|&a, &b| chain_order(&samples[a], &samples[b]))
                .expect("refined interval's right station");
            let left = &samples[left_index];
            let right = &samples[right_index];
            let key = 0.5 * ((left.position as f64 + left.offset) + (right.position as f64 + right.offset));
            let new_position = key.floor() as isize;
            let new_offset = key - new_position as f64;
            let (t_left, t_right) = (left.t, right.t);
            let left_uv = [left.station.uv1[0], left.station.uv1[1], left.station.uv2[0], left.station.uv2[1]];
            let retained = Continued {
                t: t_right,
                uv: [right.station.uv1[0], right.station.uv1[1], right.station.uv2[0], right.station.uv2[1]],
                center: Some(right.station.center),
            };
            let from = Continued { t: t_left, uv: left_uv, center: Some(left.station.center) };
            let t = 0.5 * (t_left + t_right);
            let node = continue_station(&solve_centered, &hop, &from, t, 0).map_err(|error| {
                if is_tangency_newton(&error) {
                    ChainMarchError::Station(ChainStationFailure {
                        segment: index,
                        position: new_position,
                        t,
                        from_position: left.position,
                        from_t: t_left,
                        seed_uv: left_uv,
                        error,
                    })
                } else {
                    ChainMarchError::Other(error)
                }
            })?;
            // The inserted station must also continue INTO its retained right
            // neighbour (a grid station, or the junction's own end station):
            // a cached right that a refined left no longer reaches is the
            // defect `MarchFrame::march_interval` re-seeds; here the right is
            // fixed, so the incoherence is reported, never waived.
            if let Some(reason) = hop(&node, &retained) {
                return Err(ChainMarchError::Incoherent(ChainStationFailure {
                    segment: index,
                    position: new_position,
                    t,
                    from_position: new_position,
                    from_t: t,
                    seed_uv: node.uv,
                    error: reason,
                }));
            }
            let (section_point, tangent) = section_at(t)?;
            let (_, p1, p2, center) = tangency_residual(surface1, surface2, rho, node.uv, section_point, tangent)?;
            let n1 = raw_normal(surface1, node.uv[0], node.uv[1])?;
            let n2 = raw_normal(surface2, node.uv[2], node.uv[3])?;
            let cos_alpha = rho1.signum() * rho2.signum() * n1.dot(n2);
            let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
            if weight <= 1e-6 {
                return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a chain station").into());
            }
            let apex = apex_point(p1, n1, p2, n2, center)?;
            let (left_p1, left_p2) = halfway(&from, new_position, 0.5 * (t_left + t))?;
            let (mid_p1, mid_p2) = halfway(&node, new_position, 0.5 * (t + t_right))?;
            samples[left_index].midpoint = Some([left_p1, left_p2]);
            samples.push(ChainSample {
                segment: index,
                position: new_position,
                offset: new_offset,
                t,
                station: Station {
                    uv1: [node.uv[0], node.uv[1]],
                    uv2: [node.uv[2], node.uv[3]],
                    p1,
                    p2,
                    center,
                    weight,
                    apex,
                },
                parameter: 0.0,
                midpoint: Some([mid_p1, mid_p2]),
            });
        }
        // The refined segment is probed for a fold exactly as the march probes
        // its own stations: all of them in order along the segment, through its
        // end station, so the end differences and the peak refinement read
        // neighbours as the march's do — now with the inserted stations among them.
        let mut ordered: Vec<&ChainSample> = samples
            .iter()
            .filter(|sample| sample.segment == index && (0..=per_segment as isize).contains(&sample.position))
            .filter(|sample| sample.position < per_segment as isize || sample.offset == 0.0)
            .collect();
        ordered.sort_by(|a, b| chain_order(a, b));
        let probed: Vec<(f64, [f64; 4])> = ordered
            .iter()
            .map(|sample| (sample.t, [sample.station.uv1[0], sample.station.uv1[1], sample.station.uv2[0], sample.station.uv2[1]]))
            .collect();
        let probe = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
            contacts_at(t, seed)
        };
        let radius_at = |_: f64| radius.abs();
        check_wall_fold(
            &radius_at,
            span,
            &probed,
            &probe,
            segment.edge,
            [segment.first.face, segment.second.face],
            scale,
        )?;
    }
    Ok(())
}

/// Exact contacts at interior fractions of every in-segment interval — the
/// closed chain's local-path rail verdict and the `BREP_DEBUG_CHAIN_DENSE_RAILS`
/// diagnostic: each solved as the
/// continuation of the interval's left station under the branch guard, as the
/// march solves its own. A contact that does not solve is counted and its
/// interval named (`unsolved_at`), never read as a distance.
pub(in crate::blend) struct DenseContacts {
    /// (left sample, next sample, fraction of the interval, contact 1, contact 2).
    pub(in crate::blend) solved: Vec<(usize, usize, f64, Vec3, Vec3)>,
    /// The solved ball's centre for each entry of `solved`, in the same order
    /// (`None` when the continuation returned no centre).
    pub(in crate::blend) centers: Vec<Option<Vec3>>,
    pub(in crate::blend) unsolved: usize,
    /// The left sample of every interval with a contact that did not solve.
    pub(in crate::blend) unsolved_at: Vec<usize>,
}

pub(in crate::blend) fn chain_dense_contacts(
    segments: &[ChainSegment<'_>],
    radius: f64,
    per_segment: usize,
    samples: &[ChainSample],
    subdivisions: usize,
) -> Result<DenseContacts, KernelRefusal> {
    let mut out = DenseContacts { solved: Vec::new(), centers: Vec::new(), unsolved: 0, unsolved_at: Vec::new() };
    for (index, segment) in segments.iter().enumerate() {
        let (rho1, rho2) = signed_radii(
            segment.edge,
            segment.first.face,
            segment.first.coedge,
            segment.second.face,
            segment.second.coedge,
            radius,
        )?;
        let rho = [rho1, rho2];
        let surface1 = &segment.first.face.surface;
        let surface2 = &segment.second.face.surface;
        let edge = segment.edge;
        let span = edge.t1 - edge.t0;
        let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius)?;
        let station_step = span.abs() / per_segment as f64;
        let section_at = |t: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
            let (section_point, tangent) = march_section(edge, t, station_step, "blend chain march")?;
            Ok((section_point, if segment.forward { tangent } else { tangent.scale(-1.0) }))
        };
        let solve_centered = |t: f64, seed: [f64; 4]| -> Result<([f64; 4], Vec3), KernelRefusal> {
            let (section_point, tangent) = section_at(t)?;
            solve_station_centered(surface1, surface2, rho, seed, section_point, tangent, scale)
        };
        let hop = |left: &Continued, right: &Continued| {
            branch_hop([surface1, surface2], edge, &|_| rho, left, right)
        };
        let mut order: Vec<usize> = (0..samples.len())
            .filter(|&other| samples[other].segment == index && (0..=per_segment as isize).contains(&samples[other].position))
            .filter(|&other| samples[other].position < per_segment as isize || samples[other].offset == 0.0)
            .collect();
        order.sort_by(|&a, &b| chain_order(&samples[a], &samples[b]));
        for pair in order.windows(2) {
            let (left, next) = (&samples[pair[0]], &samples[pair[1]]);
            let from = Continued {
                t: left.t,
                uv: [left.station.uv1[0], left.station.uv1[1], left.station.uv2[0], left.station.uv2[1]],
                center: Some(left.station.center),
            };
            for step in 1..subdivisions {
                let fraction = step as f64 / subdivisions as f64;
                let t = left.t + fraction * (next.t - left.t);
                let Ok(node) = continue_station(&solve_centered, &hop, &from, t, 0) else {
                    out.unsolved += 1;
                    out.unsolved_at.push(pair[0]);
                    continue;
                };
                let (section_point, tangent) = section_at(t)?;
                let (_, p1, p2, _) = tangency_residual(surface1, surface2, rho, node.uv, section_point, tangent)?;
                out.solved.push((pair[0], pair[1], fraction, p1, p2));
                out.centers.push(node.center);
            }
        }
    }
    Ok(out)
}

/// The order stations take along a segment: grid position, then a refined
/// station's offset inside its interval.
fn chain_order(a: &ChainSample, b: &ChainSample) -> std::cmp::Ordering {
    a.segment
        .cmp(&b.segment)
        .then(a.position.cmp(&b.position))
        .then(a.offset.total_cmp(&b.offset))
}

/// Global chord parameters for every sample of a marched chain (the march's
/// own assignment, also re-run after local refinement inserts stations).
/// With no inserted station the chords are summed in the same order as the
/// march always summed them.
pub(in crate::blend) fn assign_chain_parameters(
    samples: &mut [ChainSample],
    segment_count: usize,
    per_segment: usize,
    open: bool,
) {
    // Global chord parameters over the IN-SEGMENT stations (position
    // 0..CHAIN_PER_SEGMENT-1 per segment, with any inserted station in its
    // interval, in chain order), then assign overshoot samples by
    // extrapolating with their own chords.
    let mut order: Vec<usize> = (0..samples.len())
        .filter(|&index| (0..per_segment as isize).contains(&samples[index].position))
        .collect();
    order.sort_by(|&a, &b| chain_order(&samples[a], &samples[b]));
    let mut accumulated = 0.0;
    let mut previous: Option<Vec3> = None;
    for &sample_index in &order {
        let midpoint = samples[sample_index]
            .station
            .p1
            .add(samples[sample_index].station.p2)
            .scale(0.5);
        if let Some(previous_point) = previous {
            accumulated += midpoint.sub(previous_point).length();
        }
        samples[sample_index].parameter = accumulated;
        previous = Some(midpoint);
    }
    // Wrap chord back to the chain start (CLOSED chains only; an OPEN chain
    // parameterises its in-segment stations onto [0, 1] with no wrap term so
    // the free ends sit at the parameter extremes, clamped like fit_open_rows).
    if !open {
        let first_midpoint = {
            let sample = samples
                .iter()
                .find(|sample| sample.segment == 0 && sample.position == 0 && sample.offset == 0.0)
                .expect("chain start sample");
            sample.station.p1.add(sample.station.p2).scale(0.5)
        };
        accumulated += first_midpoint.sub(previous.unwrap()).length();
    }
    let total = accumulated.max(1e-12);
    for sample in samples.iter_mut() {
        sample.parameter /= total;
    }
    // Overshoot samples: REAL chord distances from the boundary
    // in-segment stations (linear extrapolation misparameterises them
    // when the neighbouring segment's station spacing differs, and the
    // support pieces' crossing region lies exactly there).
    for segment in 0..segment_count {
        let sample_at = |samples: &[ChainSample], position: isize| -> usize {
            samples
                .iter()
                .position(|sample| sample.segment == segment && sample.position == position && sample.offset == 0.0)
                .expect("chain sample present")
        };
        let midpoint = |samples: &[ChainSample], index: usize| -> Vec3 {
            samples[index]
                .station
                .p1
                .add(samples[index].station.p2)
                .scale(0.5)
        };
        // Below position 0.
        let anchor = sample_at(samples, 0);
        let mut accumulated = samples[anchor].parameter;
        let mut previous_point = midpoint(samples, anchor);
        for position in (-(CHAIN_OVERSHOOT as isize)..0).rev() {
            let index = sample_at(samples, position);
            let point = midpoint(samples, index);
            accumulated -= point.sub(previous_point).length() / total;
            samples[index].parameter = accumulated;
            previous_point = point;
        }
        // Above the last in-segment station: the grid's last position, or a
        // station refinement inserted after it.
        let anchor = *order
            .iter()
            .filter(|&&index| samples[index].segment == segment)
            .last()
            .expect("chain sample present");
        let mut accumulated = samples[anchor].parameter;
        let mut previous_point = midpoint(samples, anchor);
        for position in per_segment as isize..=(per_segment + CHAIN_OVERSHOOT) as isize
        {
            let index = sample_at(samples, position);
            let point = midpoint(samples, index);
            accumulated += point.sub(previous_point).length() / total;
            samples[index].parameter = accumulated;
            previous_point = point;
        }
    }
    station_trace(samples, per_segment);
    // Re-origin the global parameter at segment 0's MIDDLE station so the
    // blend seam falls far from every junction (a seam ON a junction
    // collides with that junction's spoke crossing).  Kept UNWRAPPED so
    // every segment's sample window stays contiguous (segment 0 spans
    // negative params); the global closed fit wraps with rem_euclid.  An
    // OPEN chain has no seam, so its parameter stays anchored at the start
    // free end (segment 0's position 0 == 0).
    if !open {
        let p_mid = samples
            .iter()
            .find(|sample| sample.segment == 0 && sample.position == per_segment as isize / 2 && sample.offset == 0.0)
            .map(|sample| sample.parameter)
            .expect("chain mid sample");
        for sample in samples.iter_mut() {
            sample.parameter -= p_mid;
        }
    }
}

/// `BREP_BLEND_STATION_TRACE=1` prints every marched chain station once the
/// global chord parameters are assigned: the two contacts, the ball centre,
/// both parameter feet on the mates and the parameter the row fit will use.
///
/// The fitted rows are what a built wall carries, and reading them back off
/// the face cannot tell a station apart from the interpolant through it — a
/// branch hop between two stations hides inside one cubic span. This prints
/// the samples themselves, so the fit and the march can be told apart.
fn station_trace(samples: &[ChainSample], per_segment: usize) {
    if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() != Some("1") {
        return;
    }
    eprintln!(
        "blend chain stations: {} sample(s) (in-segment 0..{})",
        samples.len(),
        per_segment - 1
    );
    let mut ordered: Vec<&ChainSample> = samples.iter().collect();
    ordered.sort_by(|a, b| chain_order(a, b));
    let mut previous: Option<Vec3> = None;
    for sample in ordered {
        let station = &sample.station;
        let step = previous
            .map(|point| station.center.sub(point).length())
            .unwrap_or(f64::NAN);
        eprintln!(
            "  seg {} pos {:>3} t {:.9} p1 ({:.9}, {:.9}, {:.9}) p2 ({:.9}, {:.9}, {:.9}) \
             c ({:.9}, {:.9}, {:.9}) uv1 ({:.9}, {:.9}) uv2 ({:.9}, {:.9}) w {:.9} dc {:.9}",
            sample.segment,
            sample.position,
            sample.parameter,
            station.p1.x,
            station.p1.y,
            station.p1.z,
            station.p2.x,
            station.p2.y,
            station.p2.z,
            station.center.x,
            station.center.y,
            station.center.z,
            station.uv1[0],
            station.uv1[1],
            station.uv2[0],
            station.uv2[1],
            station.weight,
            step,
        );
        previous = Some(station.center);
    }
}
