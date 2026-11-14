//! A loft between two sections along guide curves through their corners.
//!
//! Each wall is the patch its four edges bound: a side of the first section,
//! the two guides from its ends, and the matching side of the last section,
//! filled by the bilinearly blended Coons patch ([`crate::coons_bilinear`]).
//! Where the two sides are straight, that patch is the surface ruled between
//! the two guides at equal parameters. A side of no length (the last section
//! closing on a point) makes its wall three-sided, closed on a pole. The walls
//! and the two flat ends are faces of their own, joined into the solid they
//! bound ([`crate::solid_from_sheets`]).
//!
//! [`loft_profile_brep_on_guide_chains`] takes any number of sections and
//! guides that are chains of curves: the sections stand anywhere along the
//! guides, and a wall is cut along its guides' joints, so a corner in a guide
//! is an edge of the solid. (With rational sides or guides the Coons patch is
//! the projective one, blended in homogeneous space; it still holds its four
//! edges exactly.)
use crate::{coons_bilinear, face_from_surface, face_from_surface_trimmed, make_plane, solid_from_sheets, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};

/// The prefix of every refusal of this operation.
pub const LOFT_ON_GUIDES_REFUSAL: &str = "loftOnGuides";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{LOFT_ON_GUIDES_REFUSAL}: {message}"))
}

fn ends(curve: &NurbsCurve) -> Result<[Vec3; 2], KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "loftOnGuides.domain")?;
    Ok([curve.evaluate(t0).or_refuse(KernelStage::Collect, "loftOnGuides.end")?, curve.evaluate(t1).or_refuse(KernelStage::Collect, "loftOnGuides.end")?])
}

fn poles(curves: &[NurbsCurve]) -> Vec<Vec3> {
    curves.iter().flat_map(|curve| &curve.control_points).map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)).collect()
}

/// The flat face inside the closed loop `section`, which must lie in a plane.
fn flat_end(section: &[NurbsCurve], size: f64, which: &str) -> Result<BrepSolid, KernelRefusal> {
    let points = poles(section);
    let origin = points[0];
    let far = |off: Option<Vec3>| {
        points.iter().map(|point| point.sub(origin)).map(|arm| off.map_or(arm, |axis| arm.sub(axis.scale(arm.dot(axis))))).fold(Vec3::new(0.0, 0.0, 0.0), |best, arm| if arm.length() > best.length() { arm } else { best })
    };
    let flat = || input("section_plane", format!("the {which} section does not lie in a plane"));
    let e1 = far(None).normalized().map_err(|_| flat())?;
    let across = far(Some(e1));
    if across.length() <= 1e-9 * size {
        return Err(flat());
    }
    let e2 = across.normalized().map_err(|_| flat())?;
    let normal = e1.cross(e2);
    if points.iter().any(|point| point.sub(origin).dot(normal).abs() > 1e-9 * size) {
        return Err(flat());
    }
    let reach = |axis: Vec3| points.iter().map(|point| point.sub(origin).dot(axis)).fold((0.0f64, 0.0f64), |(low, high), at| (low.min(at), high.max(at)));
    let ((low1, high1), (low2, high2)) = (reach(e1), reach(e2));
    let corner = origin.add(e1.scale(low1 - 0.5 * size)).add(e2.scale(low2 - 0.5 * size));
    let plane = make_plane(corner, e1, e2, high1 - low1 + size, high2 - low2 + size).or_refuse(KernelStage::Collect, "loftOnGuides.plane")?;
    face_from_surface_trimmed(&plane, &[section.to_vec()])
}

/// The wall standing on the straight side `side` between two guides that meet
/// at one point: the surface ruled between the guides at equal parameters,
/// closed on a pole where they meet. (The Coons patch of a straight side, two
/// guides and a point is that ruled surface.)
fn pointed_wall(side: &NurbsCurve, rail: &NurbsCurve, next: &NurbsCurve, k: usize, tolerance: f64) -> Result<crate::NurbsSurface, KernelRefusal> {
    let [from, to] = ends(side)?;
    let straight = poles(std::slice::from_ref(side)).iter().all(|point| point.sub(from).cross(to.sub(from)).length() <= tolerance * to.sub(from).length());
    if !straight {
        return Err(KernelRefusal::unsupported(KernelStage::Fragment, "pointed_wall_side", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} closes on a point from a side that is not straight")));
    }
    if rail.degree != next.degree || rail.knots != next.knots {
        return Err(KernelRefusal::unsupported(KernelStage::Fragment, "pointed_wall_guides", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} closes on a point between guides of different degree or knots")));
    }
    let net = vec![rail.control_points.clone(), next.control_points.clone()];
    crate::NurbsSurface::new(1, rail.degree, vec![0.0, 0.0, 1.0, 1.0], rail.knots.clone(), net).or_refuse(KernelStage::Fragment, "loftOnGuides.pointed_wall")
}

/// The solid lofted from the closed section `first` to the closed section
/// `last` along `guides`.
///
/// The two sections have the same number of curves, each a closed loop run the
/// same way round, curve `k` of the first matched with curve `k` of the last;
/// each lies in a plane. `guides[k]` runs from the start of `first[k]` to the
/// start of `last[k]` (given in either direction). A curve of `last` may have
/// no length: where every one of them is the same point the loft closes on
/// that point and has no last end.
///
/// Refuses sections of different counts, guides that do not join the matched
/// corners, sections off a plane, and walls the Coons patch refuses.
pub fn loft_profile_brep_on_guides(first: &[NurbsCurve], last: &[NurbsCurve], guides: &[NurbsCurve]) -> Result<BrepSolid, KernelRefusal> {
    let count = first.len();
    if count < 2 || last.len() != count || guides.len() != count {
        return Err(input("counts", format!("the sections and the guides must be equally many, at least 2 (got {count}, {} and {})", last.len(), guides.len())));
    }
    let all = poles(&[first, last, guides].concat());
    let size = all.iter().map(|point| point.sub(all[0]).length()).fold(0.0, f64::max);
    let tolerance = 1e-9 * size;
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;
    // each guide turned to run from the first section to the last
    let mut rails = Vec::with_capacity(count);
    for k in 0..count {
        let ([foot, _], [head, _]) = (ends(&first[k])?, ends(&last[k])?);
        let [from, to] = ends(&guides[k])?;
        let rail = if near(from, foot) && near(to, head) {
            guides[k].clone()
        } else if near(to, foot) && near(from, head) {
            guides[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?
        } else {
            return Err(input("guide_ends", format!("guide {k} does not join the start of curve {k} of the first section to that of the last")));
        };
        rails.push(rail);
    }
    let mut sheets = Vec::with_capacity(count + 2);
    for k in 0..count {
        let next = (k + 1) % count;
        let far_side = last[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?;
        let back = rails[k].reversed().or_refuse(KernelStage::Collect, "loftOnGuides.reverse")?;
        let [head, head_next] = ends(&last[k])?;
        let patch = if near(head, head_next) && poles(&last[k..=k]).iter().all(|point| near(*point, head)) {
            pointed_wall(&first[k], &rails[k], &rails[next], k, tolerance)?
        } else {
            coons_bilinear([first[k].clone(), rails[next].clone(), far_side, back])
                .map_err(|error| KernelRefusal::unsupported(KernelStage::Fragment, "wall", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} has no Coons patch: {error:?}")))?
        };
        sheets.push(face_from_surface(&patch)?);
    }
    sheets.push(flat_end(first, size, "first")?);
    let apex = ends(&last[0])?[0];
    let pointed = poles(last).iter().all(|point| near(*point, apex));
    if !pointed {
        sheets.push(flat_end(last, size, "last")?);
    }
    // sewing reads each edge's place by projection, whose own error on a cubic is near 1e-8
    solid_from_sheets(&sheets, (1e-8 * size).max(1e-7))
}

/// Where a point sits on a chain: the piece it is on and how far along the
/// chain that is, counting each piece as one.
#[derive(Clone, Copy)]
struct Station {
    along: f64,
}

/// The place on `chain` of `corner`, which must lie on it.
fn station(chain: &[NurbsCurve], corner: Vec3, tolerance: f64, guide: usize, section: usize) -> Result<Station, KernelRefusal> {
    let mut best: Option<(f64, f64)> = None;
    for (index, piece) in chain.iter().enumerate() {
        let [t0, t1] = piece.domain().or_refuse(KernelStage::Collect, "loftOnGuides.domain")?;
        let foot = crate::project_point_to_curve(piece, corner).or_refuse(KernelStage::Classify, "loftOnGuides.project")?;
        // an end of a piece is a joint: count it as the whole number it is
        let [from, to] = ends(piece)?;
        let along = if corner.sub(from).length() <= tolerance {
            index as f64
        } else if corner.sub(to).length() <= tolerance {
            (index + 1) as f64
        } else {
            index as f64 + (foot.u - t0) / (t1 - t0)
        };
        if best.map_or(true, |(distance, _)| foot.distance < distance) {
            best = Some((foot.distance, along));
        }
    }
    match best {
        Some((distance, along)) if distance <= tolerance => Ok(Station { along }),
        Some((distance, _)) => Err(input("corner_off_guide", format!("the start of curve {guide} of section {section} is {distance:.3e} off guide {guide}"))),
        None => Err(input("guide_empty", format!("guide {guide} has no curves"))),
    }
}

/// The pieces of `chain` between the places `from` and `to` along it.
fn between(chain: &[NurbsCurve], from: f64, to: f64) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    let stage = KernelStage::Collect;
    let mut pieces = Vec::new();
    for (index, piece) in chain.iter().enumerate() {
        let (low, high) = (from.max(index as f64), to.min((index + 1) as f64));
        if high - low <= 1e-12 {
            continue;
        }
        let [t0, t1] = piece.domain().or_refuse(stage, "loftOnGuides.domain")?;
        let at = |along: f64| t0 + (t1 - t0) * (along - index as f64);
        let mut part = piece.clone();
        if high < (index + 1) as f64 {
            part = part.split(at(high)).or_refuse(stage, "loftOnGuides.cut")?.0;
        }
        if low > index as f64 {
            part = part.split(at(low)).or_refuse(stage, "loftOnGuides.cut")?.1;
        }
        pieces.push(part);
    }
    Ok(pieces)
}

/// The solid lofted through the closed sections `sections` along `guides`,
/// each guide a chain of curves.
///
/// Every section has the same number of curves, run the same way round, curve
/// `k` of one matched with curve `k` of the next. `guides[k]` is a chain of
/// curves that follow one another head to tail (the chain may be given from
/// either end) and passes through the start of curve `k` of every section, in
/// the sections' order. The sections may stand anywhere along the guides, at a
/// joint of a chain or between two; what a guide has before the first section
/// or after the last is not used. The first and last sections lie in planes
/// and close the solid; the last may be curves of no length at one point, on
/// which the loft then closes.
///
/// Between two sections each wall is the Coons patch of a side of each and the
/// two guides between them, each guide taken as one curve with its pieces on
/// equal stretches of the parameter. Where the guides have joints between two
/// sections the wall is cut along them into one face for each piece, so a
/// corner in a guide is an edge of the solid and no face has a crease; the two
/// guides of a wall must then have equally many pieces between those sections.
///
/// Refuses, by name: counts that do not agree (`counts`), a section's corner
/// off its guide (`corner_off_guide`), sections out of order along a guide
/// (`stations_order`), two neighbouring guides with different numbers of
/// pieces between two sections (`guide_pieces`), a chain whose curves do not
/// meet (`guide_chain`), an end section off a plane (`section_plane`), a wall
/// closing on a point from a side that is not straight (`pointed_wall_side`),
/// and a wall the Coons patch refuses (`wall`).
pub fn loft_profile_brep_on_guide_chains(sections: &[Vec<NurbsCurve>], guides: &[Vec<NurbsCurve>]) -> Result<BrepSolid, KernelRefusal> {
    Ok(chains_with_walls(sections, guides)?.0)
}

/// Which wall a side face of a loft along guide chains is: the faces of the
/// solid come one for each of these, in this order, and then the flat ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GuideLoftWall {
    /// The curve of the first section the wall stands over.
    pub side: usize,
    /// The wall lies between sections `span` and `span + 1`.
    pub span: usize,
    /// The guide through the start of that curve, as the guides were given.
    pub guide: usize,
    /// The curve of that guide the wall runs along, as the chain was given.
    pub piece: usize,
}

/// [`loft_profile_brep_on_guide_chains`] for sections and guides that are not
/// yet matched: the guides in any order, and every section after the first
/// starting at any of its corners and run either way round. The first section
/// is taken as given; each guide is given to the corner of it that it passes
/// through, and every later section is turned (and reversed if need be) until
/// its corners lie on the guides of the first section's corners. A last section
/// at one point is left as it is.
///
/// Returns the solid and, for its side faces in order, which wall each is.
/// Refuses as the matched loft does, and by name a corner of the first section
/// that no guide, or more than one, passes through (`guide_for_corner`) and a
/// later section none of whose turns puts its corners on the guides
/// (`section_off_guides`).
pub fn loft_profile_brep_on_loose_guide_chains(sections: &[Vec<NurbsCurve>], guides: &[Vec<NurbsCurve>]) -> Result<(BrepSolid, Vec<GuideLoftWall>), KernelRefusal> {
    let stage = KernelStage::Collect;
    let count = guides.len();
    if sections.len() < 2 || count < 2 || sections.iter().any(|section| section.len() != count) || guides.iter().any(Vec::is_empty) {
        return Err(input("counts", format!("at least two sections are needed, each of as many curves as there are guides, at least 2, and no guide empty (got {} sections of {:?} curves and {count} guides)", sections.len(), sections.iter().map(Vec::len).collect::<Vec<_>>())));
    }
    let every: Vec<NurbsCurve> = sections.iter().flatten().chain(guides.iter().flatten()).cloned().collect();
    let all = poles(&every);
    let size = all.iter().map(|point| point.sub(all[0]).length()).fold(0.0, f64::max);
    let tolerance = 1e-9 * size;
    let on = |guide: usize, point: Vec3| -> Result<bool, KernelRefusal> {
        for piece in &guides[guide] {
            if crate::project_point_to_curve(piece, point).or_refuse(KernelStage::Classify, "loftOnGuides.project")?.distance <= tolerance {
                return Ok(true);
            }
        }
        Ok(false)
    };
    // the guide of each corner of the first section
    let mut order = Vec::with_capacity(count);
    for (k, curve) in sections[0].iter().enumerate() {
        let corner = ends(curve)?[0];
        let mut through = Vec::new();
        for guide in 0..count {
            if on(guide, corner)? {
                through.push(guide);
            }
        }
        match through[..] {
            [guide] if !order.contains(&guide) => order.push(guide),
            _ => return Err(input("guide_for_corner", format!("the start of curve {k} of the first section needs one guide of its own through it; the guides through it are {through:?}"))),
        }
    }
    let mut matched = vec![sections[0].clone()];
    for (index, section) in sections.iter().enumerate().skip(1) {
        let points = poles(section);
        if points.iter().all(|point| point.sub(points[0]).length() <= tolerance) {
            matched.push(section.clone());
            continue;
        }
        let backwards: Vec<NurbsCurve> = section.iter().rev().map(|curve| curve.reversed().or_refuse(stage, "loftOnGuides.reverse")).collect::<Result<_, _>>()?;
        let mut fit = None;
        'turns: for way in [section, &backwards] {
            for turn in 0..count {
                let mut fits = true;
                for k in 0..count {
                    if !on(order[k], ends(&way[(turn + k) % count])?[0])? {
                        fits = false;
                        break;
                    }
                }
                if fits {
                    fit = Some((0..count).map(|k| way[(turn + k) % count].clone()).collect::<Vec<_>>());
                    break 'turns;
                }
            }
        }
        matched.push(fit.ok_or_else(|| input("section_off_guides", format!("section {index} cannot be turned so that each of its corners lies on the guide through the matching corner of the first section")))?);
    }
    let ordered: Vec<Vec<NurbsCurve>> = order.iter().map(|guide| guides[*guide].clone()).collect();
    let (solid, walls) = chains_with_walls(&matched, &ordered)?;
    Ok((solid, walls.into_iter().map(|wall| GuideLoftWall { guide: order[wall.guide], ..wall }).collect()))
}

fn chains_with_walls(sections: &[Vec<NurbsCurve>], guides: &[Vec<NurbsCurve>]) -> Result<(BrepSolid, Vec<GuideLoftWall>), KernelRefusal> {
    let stage = KernelStage::Collect;
    let count = guides.len();
    if sections.len() < 2 || count < 2 || sections.iter().any(|section| section.len() != count) {
        return Err(input("counts", format!("at least two sections are needed, each of as many curves as there are guides, at least 2 (got {} sections of {:?} curves and {count} guides)", sections.len(), sections.iter().map(Vec::len).collect::<Vec<_>>())));
    }
    let every: Vec<NurbsCurve> = sections.iter().flatten().chain(guides.iter().flatten()).cloned().collect();
    let all = poles(&every);
    let size = all.iter().map(|point| point.sub(all[0]).length()).fold(0.0, f64::max);
    let tolerance = 1e-9 * size;
    let near = |a: Vec3, b: Vec3| a.sub(b).length() <= tolerance;

    // each chain run from the first section toward the last, and where each section meets it
    let mut chains: Vec<Vec<NurbsCurve>> = Vec::with_capacity(count);
    let mut turned = vec![false; count];
    let mut stations: Vec<Vec<f64>> = Vec::with_capacity(count);
    for (k, guide) in guides.iter().enumerate() {
        for index in 1..guide.len() {
            if !near(ends(&guide[index - 1])?[1], ends(&guide[index])?[0]) {
                return Err(input("guide_chain", format!("curve {index} of guide {k} does not start where curve {} ends", index - 1)));
            }
        }
        let place = |chain: &[NurbsCurve]| -> Result<Vec<f64>, KernelRefusal> {
            sections.iter().enumerate().map(|(i, section)| Ok(station(chain, ends(&section[k])?[0], tolerance, k, i)?.along)).collect()
        };
        let mut chain = guide.clone();
        let mut along = place(&chain)?;
        if along[0] > along[along.len() - 1] {
            chain = chain.iter().rev().map(|piece| piece.reversed().or_refuse(stage, "loftOnGuides.reverse")).collect::<Result<_, _>>()?;
            along = place(&chain)?;
            turned[k] = true;
        }
        if along.windows(2).any(|pair| pair[1] - pair[0] <= 1e-9) {
            return Err(input("stations_order", format!("the sections do not follow one another along guide {k} (they meet it at {along:?} of its curves)")));
        }
        chains.push(chain);
        stations.push(along);
    }

    let mut sheets = Vec::new();
    let mut walls = Vec::new();
    for span in 0..sections.len() - 1 {
        let (foot, head) = (&sections[span], &sections[span + 1]);
        let rails: Vec<Vec<NurbsCurve>> = (0..count).map(|k| between(&chains[k], stations[k][span], stations[k][span + 1])).collect::<Result<_, _>>()?;
        for k in 0..count {
            let next = (k + 1) % count;
            let tiers = rails[k].len();
            if rails[next].len() != tiers {
                return Err(input("guide_pieces", format!("between sections {span} and {} guide {k} has {tiers} piece(s) and guide {next} has {}; the two guides of a wall need equally many", span + 1, rails[next].len())));
            }
            // each guide as one curve, piece j on the stretch from j to j + 1
            let unit = vec![1.0; tiers];
            let join = |pieces: &[NurbsCurve]| crate::curve_chain::join_curve_chain_on_spans(pieces, &unit).or_refuse(stage, "loftOnGuides.join");
            let (rail, far_rail) = (join(&rails[k])?, join(&rails[next])?);
            let [head_from, head_to] = ends(&head[k])?;
            let pointed = near(head_from, head_to) && poles(&head[k..=k]).iter().all(|point| near(*point, head_from));
            let patch = if pointed {
                let family = crate::coons::compat::make_family_compatible(&[rail, far_rail], 1).or_refuse(KernelStage::Fragment, "loftOnGuides.compatible")?.0;
                pointed_wall(&foot[k], &family[0], &family[1], k, tolerance)?
            } else {
                let far_side = head[k].reversed().or_refuse(stage, "loftOnGuides.reverse")?;
                let back = rail.reversed().or_refuse(stage, "loftOnGuides.reverse")?;
                coons_bilinear([foot[k].clone(), far_rail, far_side, back])
                    .map_err(|error| KernelRefusal::unsupported(KernelStage::Fragment, "wall", format!("{LOFT_ON_GUIDES_REFUSAL}: wall {k} between sections {span} and {} has no Coons patch: {error:?}", span + 1)))?
            };
            // the guide's curves this wall runs along: the ones `between` kept a stretch of
            let (from, to) = (stations[k][span], stations[k][span + 1]);
            let pieces = (0..chains[k].len()).filter(|index| to.min((index + 1) as f64) - from.max(*index as f64) > 1e-12);
            for (tier, piece) in split_at_joints(&patch, ends(&foot[k])?, ends(&rails[k][tiers - 1])?[1], tiers, tolerance)?.iter().zip(pieces) {
                sheets.push(face_from_surface(tier)?);
                walls.push(GuideLoftWall { side: k, span, guide: k, piece: if turned[k] { chains[k].len() - 1 - piece } else { piece } });
            }
        }
    }
    sheets.push(flat_end(&sections[0], size, "first")?);
    let last = &sections[sections.len() - 1];
    let apex = ends(&last[0])?[0];
    if !poles(last).iter().all(|point| near(*point, apex)) {
        sheets.push(flat_end(last, size, "last")?);
    }
    // sewing reads each edge's place by projection, whose own error on a cubic is near 1e-8
    Ok((solid_from_sheets(&sheets, (1e-8 * size).max(1e-7))?, walls))
}

/// A wall's patch cut along its guides' joints into `tiers` faces' surfaces.
/// `side` is the foot side's two ends and `up` the far end of the guide from
/// the side's start: they tell which of the patch's two parameters runs along
/// the guides. The joints are the patch's own knots at the whole `tiers`-ths of
/// that parameter.
fn split_at_joints(patch: &crate::NurbsSurface, side: [Vec3; 2], up: Vec3, tiers: usize, tolerance: f64) -> Result<Vec<crate::NurbsSurface>, KernelRefusal> {
    if tiers == 1 {
        return Ok(vec![patch.clone()]);
    }
    let stage = KernelStage::Fragment;
    let [u0, u1] = patch.domain_u().or_refuse(stage, "loftOnGuides.domain")?;
    let [v0, v1] = patch.domain_v().or_refuse(stage, "loftOnGuides.domain")?;
    // the corner opposite the side's start along each parameter
    let corners = [(u0, v0), (u1, v0), (u0, v1), (u1, v1)];
    let mut origin = None;
    for (u, v) in corners {
        if patch.evaluate(u, v).or_refuse(stage, "loftOnGuides.corner")?.sub(side[0]).length() <= tolerance.max(1e-12) * 10.0 {
            origin = Some((u, v));
        }
    }
    let (ou, ov) = origin.ok_or_else(|| KernelRefusal::internal(stage, "wall_corner", format!("{LOFT_ON_GUIDES_REFUSAL}: the wall's patch does not start at its side's corner")))?;
    let (other_u, other_v) = (if ou == u0 { u1 } else { u0 }, if ov == v0 { v1 } else { v0 });
    let along_v = patch.evaluate(ou, other_v).or_refuse(stage, "loftOnGuides.corner")?.sub(up).length() <= patch.evaluate(other_u, ov).or_refuse(stage, "loftOnGuides.corner")?.sub(up).length();
    let ([low, high], knots, degree) = if along_v { ([v0, v1], &patch.knots_v, patch.degree_v) } else { ([u0, u1], &patch.knots_u, patch.degree_u) };
    let mut rest = patch.clone();
    let mut parts = Vec::with_capacity(tiers);
    for joint in 1..tiers {
        let wanted = low + (high - low) * joint as f64 / tiers as f64;
        let knot = knots.iter().copied().min_by(|a, b| (a - wanted).abs().partial_cmp(&(b - wanted).abs()).expect("finite knots")).expect("a knot");
        let multiplicity = knots.iter().filter(|other| (**other - knot).abs() <= 1e-12).count();
        if (knot - wanted).abs() > 1e-9 * (high - low) || multiplicity < degree {
            return Err(KernelRefusal::internal(stage, "wall_joint", format!("{LOFT_ON_GUIDES_REFUSAL}: the wall's patch has no knot of full multiplicity at joint {joint} of {tiers}")));
        }
        let (before, after) = crate::split_surface(&rest, along_v, knot).or_refuse(stage, "loftOnGuides.split")?;
        parts.push(before);
        rest = after;
    }
    parts.push(rest);
    Ok(parts)
}

