//! Dilation and erosion of planar regions by a disc, and the region a tool
//! sweeps along a chain, built from unions of single-segment capsules only.
//! The planar engine's level-set offset (`offset_region`) loses loops on
//! arc-rich boundaries, while its unions of simple loops hold up after the
//! boolean side-probe fix. Every boolean here is wrapped with an area
//! sanity check; a result outside the bounds set theory allows is an error
//! (and an operand dump when `BREP_CAM_ADAPTIVE_DUMP` is set), never a
//! silently wrong region.

use crate::model::Point2;
use crate::planar::{Loop, Region, Segment, Vec2};
use crate::{CamError, Tolerance};
use std::f64::consts::PI;

const TAU: f64 = std::f64::consts::TAU;

fn invalid(message: impl std::fmt::Display) -> CamError {
    CamError::InvalidInput(format!("adaptive: {message}"))
}

/// A full circle as four arcs whose vertices sit at an odd angle (0.618 rad
/// and every quarter turn from it), so they never coincide with the
/// axis-aligned corners of capsule bodies: the planar engine's arrangement
/// does not survive a vertex of one operand lying exactly on a vertex of the
/// other.
pub fn circle_loop(center: Point2, radius: f64, ccw: bool) -> Loop {
    const PHASE: f64 = 0.618_033_988_749_895;
    let at = |k: usize| center.add(Point2::new(radius * (PHASE + k as f64 * PI / 2.0).cos(), radius * (PHASE + k as f64 * PI / 2.0).sin()));
    let mut segs: Vec<Segment> = (0..4).map(|k| Segment::arc(at(k), at(k + 1), center, true)).collect();
    if !ccw {
        segs.reverse();
        for s in &mut segs {
            *s = s.reversed();
        }
    }
    Loop::new(segs)
}

/// A disc as a region.
pub fn disc(center: Point2, radius: f64, tol: &Tolerance) -> Result<Region, CamError> {
    Ok(Region::new(vec![circle_loop(center, radius, true)], tol)?)
}

/// Every arc of `region` replaced by chords within `chord`: the planar
/// engine's arrangement is robust on polygons (after the side-probe fix) but
/// not on the tangent and co-circular arcs iterated booleans on capsules
/// produce. All region operands of this module are polygons; the exact arcs
/// live only in the pass chains and the cut points. The chord deviation is
/// the module's stated approximation.
pub fn polygonal(region: &Region, chord: f64, tol: &Tolerance) -> Result<Region, CamError> {
    if region.loops.iter().all(|l| l.segments.iter().all(|s| s.arc.is_none())) {
        return Ok(region.clone());
    }
    let mut loops = Vec::with_capacity(region.loops.len());
    for l in &region.loops {
        let mut pts = l.discretize(chord.max(4.0 * tol.point_eps));
        pts.dedup_by(|a, b| a.distance(*b) <= tol.point_eps);
        if pts.len() >= 2 && pts[0].distance(pts[pts.len() - 1]) <= tol.point_eps {
            pts.pop();
        }
        if pts.len() < 3 {
            continue;
        }
        loops.push(Loop::polygon(&pts));
    }
    Ok(Region::new(loops, tol)?)
}

/// Merge consecutive collinear lines and consecutive co-circular arcs of a
/// chain, so no two neighbouring pieces share a supporting curve (their
/// capsules would then overlap partially along it, a case the planar
/// engine's coincidence handling does not survive).
pub fn merge_pieces(segments: &[Segment], tol: &Tolerance) -> Vec<Segment> {
    let mut out: Vec<Segment> = Vec::with_capacity(segments.len());
    for s in segments {
        if s.length() <= tol.point_eps {
            continue;
        }
        if let Some(last) = out.last_mut() {
            if last.end.distance(s.start) <= tol.point_eps {
                match (last.arc, s.arc) {
                    (None, None) => {
                        let u = last.end.sub(last.start);
                        let v = s.end.sub(s.start);
                        if u.cross(v).abs() <= 1e-9 * u.length() * v.length() && u.dot(v) > 0.0 {
                            last.end = s.end;
                            continue;
                        }
                    }
                    (Some(a), Some(b)) => {
                        if a.ccw == b.ccw && a.center.distance(b.center) <= tol.point_eps && (last.radius() - s.radius()).abs() <= tol.point_eps {
                            // Merged sweep must stay below a full turn.
                            let total = last.angles().1.abs() + s.angles().1.abs();
                            if total < TAU - 1e-9 {
                                last.end = s.end;
                                continue;
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
        out.push(*s);
    }
    out
}

/// The exact capsule of one segment as a single loop: a stadium for a line;
/// for an arc of radius above `r` the annular capsule (outer arc, end cap,
/// inner arc back, start cap); for a tighter arc the outer arc with the two
/// end caps meeting at the inner intersection `X` of the end discs (both of
/// radius `r`, which overlap since the chord is shorter than `2 r`). Built
/// without any boolean, so the only coincidences between neighbouring
/// capsules are complete end caps on one circle.
pub fn capsule_loop(seg: &Segment, r: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let Some(a) = seg.arc else {
        return Ok(Region::new(vec![Loop::stadium(seg.start, seg.end, r)], tol)?);
    };
    let rho = seg.radius();
    let (a0, sweep) = seg.angles();
    let a1 = a0 + sweep;
    let at = |rad: f64, ang: f64| a.center.add(Point2::new(rad * ang.cos(), rad * ang.sin()));
    let outer = rho + r;
    let mut segs: Vec<Segment> = Vec::new();
    segs.extend(Segment::arc(at(outer, a0), at(outer, a1), a.center, a.ccw).canonical(tol));
    // An inner arc thinner than the sample epsilon is degenerate: tight form.
    if rho > r + 4.0 * tol.sample_eps {
        let inner = rho - r;
        segs.extend(Segment::arc(at(outer, a1), at(inner, a1), seg.end, a.ccw).canonical(tol));
        segs.extend(Segment::arc(at(inner, a1), at(inner, a0), a.center, !a.ccw).canonical(tol));
        segs.extend(Segment::arc(at(inner, a0), at(outer, a0), seg.start, a.ccw).canonical(tol));
    } else {
        // Inner intersection of the end discs: the one farthest from the arc.
        let chord_mid = seg.start.add(seg.end).scale(0.5);
        let half = 0.5 * seg.start.distance(seg.end);
        let hh = (r * r - half * half).max(0.0).sqrt();
        let u = seg.end.sub(seg.start).normalized().unwrap_or(Point2::new(1.0, 0.0));
        let arc_mid = seg.midpoint();
        let away = chord_mid.sub(arc_mid).normalized().unwrap_or(u.perp().scale(-1.0));
        let c1 = chord_mid.add(u.perp().scale(hh));
        let c2 = chord_mid.sub(u.perp().scale(hh));
        let x = if c1.sub(arc_mid).dot(away) >= c2.sub(arc_mid).dot(away) { c1 } else { c2 };
        segs.extend(Segment::arc(at(outer, a1), x, seg.end, a.ccw).canonical(tol));
        segs.extend(Segment::arc(x, at(outer, a0), seg.start, a.ccw).canonical(tol));
    }
    let mut l = Loop::new(segs);
    if !l.is_ccw() {
        l = l.reversed();
    }
    Ok(Region::new(vec![l], tol)?)
}

/// A full circle of tool-centre radius `rho` swept by a tool of radius `r`:
/// the disc of `rho + r`, or the annulus when `rho > r`.
fn ring_region(center: Point2, rho: f64, r: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let mut loops = vec![circle_loop(center, rho + r, true)];
    if rho > r + 4.0 * tol.sample_eps {
        loops.push(circle_loop(center, rho - r, false));
    }
    Ok(Region::new(loops, tol)?)
}

/// Dilate `base` by the capsules of `segments` (the material a tool of
/// radius `r` sweeps along them, or a region's boundary pushed out by `r`):
/// unions of bodies and full vertex discs only, with every union's area
/// checked. Full circles of consecutive co-circular arcs become one disc or
/// annulus.
fn union_capsules(base: Region, segments: &[Segment], r: f64, min_len: f64, _chord: f64, what: &str, tol: &Tolerance) -> Result<Region, CamError> {
    // Pieces shorter than `min_len` lie under their neighbours' end discs up
    // to a sliver far below the tolerance; their own bodies would be slivers
    // the arrangement cannot stitch.
    let kept: Vec<Segment> = segments.iter().copied().filter(|s| s.length() > min_len).collect();
    // A capsule loop is simple only for an arc of at most half a turn: past
    // it the end caps face each other across the gap and, once they meet
    // there, the loop crosses itself (a helix entry's 277-degree arc beside a
    // wall). Longer arcs are halved; the halves stay consecutive and
    // co-circular, so a full circle is still found below.
    let mut merged: Vec<Segment> = Vec::new();
    for s in merge_pieces(&kept, tol) {
        let mut stack = vec![s];
        while let Some(piece) = stack.pop() {
            if piece.arc.is_some() && piece.angles().1.abs() > PI {
                let mid = piece.midpoint();
                stack.push(piece.sub_segment(mid, piece.end));
                stack.push(piece.sub_segment(piece.start, mid));
            } else {
                merged.push(piece);
            }
        }
    }
    let mut out = base;
    let join = |out: &mut Region, one: Region| -> Result<(), CamError> {
        if one.area() <= tol.area_eps {
            return Ok(());
        }
        if out.is_empty() {
            *out = one;
            return Ok(());
        }
        let joined = checked(what, out, &one, || out.union(&one, tol))?;
        check_union(what, out, &one, &joined, tol)?;
        *out = joined;
        Ok(())
    };
    let mut i = 0;
    while i < merged.len() {
        let s = &merged[i];
        // A full circle: this arc plus the next ones around the same centre.
        if let Some(a) = s.arc {
            let mut sweep = s.angles().1.abs();
            let mut j = i + 1;
            while j < merged.len() {
                let t = &merged[j];
                match t.arc {
                    Some(b) if b.ccw == a.ccw && b.center.distance(a.center) <= tol.point_eps && (t.radius() - s.radius()).abs() <= tol.point_eps && merged[j - 1].end.distance(t.start) <= tol.point_eps => {
                        sweep += t.angles().1.abs();
                        j += 1;
                    }
                    _ => break,
                }
            }
            if (sweep - TAU).abs() <= 1e-9 && merged[j - 1].end.distance(s.start) <= tol.point_eps {
                join(&mut out, ring_region(a.center, s.radius(), r, tol)?)?;
                i = j;
                continue;
            }
        }
        join(&mut out, capsule_loop(s, r, tol)?)?;
        i += 1;
    }
    Ok(out)
}

/// The region a tool of radius `r` sweeps along `segments`.
pub fn swept_region(segments: &[Segment], r: f64, chord_tol: f64, tol: &Tolerance) -> Result<Region, CamError> {
    union_capsules(Region::empty(), segments, r, 0.25 * chord_tol, 0.25 * chord_tol, "swept union", tol)
}

/// A planar boolean with context: on failure the operands are dumped when
/// `BREP_CAM_ADAPTIVE_DUMP` is set (diagnostics) and the error names the step.
pub fn checked(what: &str, a: &Region, b: &Region, f: impl FnOnce() -> Result<Region, crate::planar::PlanarError>) -> Result<Region, CamError> {
    match f() {
        Ok(r) => Ok(r),
        Err(e) => Err(refused(what, a, b, &e.to_string())),
    }
}

/// A boolean whose result failed an area sanity check: dump the operands when
/// asked and refuse the level.
pub fn refused(what: &str, a: &Region, b: &Region, message: &str) -> CamError {
    if let Ok(path) = std::env::var("BREP_CAM_ADAPTIVE_DUMP") {
        let dump = serde_json::json!({ "what": what, "a": a, "b": b, "error": message });
        let _ = std::fs::write(format!("{path}.boolean.json"), serde_json::to_string(&dump).unwrap_or_default());
    }
    invalid(format!("{what}: {message}"))
}

/// A union's area lies between the larger operand and the sum of both.
pub fn check_union(what: &str, a: &Region, b: &Region, joined: &Region, tol: &Tolerance) -> Result<(), CamError> {
    let (aa, ab, aj) = (a.area(), b.area(), joined.area());
    let eps = tol.area_eps + 1e-9 * (aa + ab);
    if aj < aa.max(ab) - eps || aj > aa + ab + eps {
        return Err(refused(what, a, b, &format!("union area {aj:.6} outside [{:.6}, {:.6}]", aa.max(ab), aa + ab)));
    }
    Ok(())
}

/// A difference's area lies between `a - b` and `a`.
pub fn check_difference(what: &str, a: &Region, b: &Region, cut: &Region, tol: &Tolerance) -> Result<(), CamError> {
    let (aa, ab, ac) = (a.area(), b.area(), cut.area());
    let eps = tol.area_eps + 1e-9 * (aa + ab);
    if ac > aa + eps || ac < aa - ab - eps {
        return Err(refused(what, a, b, &format!("difference area {ac:.6} outside [{:.6}, {aa:.6}]", (aa - ab).max(0.0))));
    }
    Ok(())
}

/// `region ⊕ disc(d)`: the region plus the capsules of its boundary segments
/// (the Minkowski sum with a disc, built from unions only, the planar
/// engine's level-set offset loses loops on arc-rich boundaries).
pub fn grow(region: &Region, d: f64, chord_tol: f64, tol: &Tolerance) -> Result<Region, CamError> {
    if d <= 4.0 * tol.point_eps {
        return Ok(region.clone());
    }
    let segs: Vec<Segment> = region.loops.iter().flat_map(|l| l.segments.iter().copied()).collect();
    union_capsules(region.clone(), &segs, d, 0.25 * chord_tol, 0.25 * chord_tol, "dilation union", tol)
}

/// `region ⊖ disc(d)`: the region minus the capsules of its boundary.
pub fn shrink(region: &Region, d: f64, chord_tol: f64, tol: &Tolerance) -> Result<Region, CamError> {
    if d <= 4.0 * tol.point_eps {
        return Ok(region.clone());
    }
    let segs: Vec<Segment> = region.loops.iter().flat_map(|l| l.segments.iter().copied()).collect();
    let capsules = union_capsules(Region::empty(), &segs, d, 0.25 * chord_tol, 0.25 * chord_tol, "erosion capsules", tol)?;
    if capsules.is_empty() {
        return Ok(region.clone());
    }
    let cut = checked("erosion difference", region, &capsules, || region.difference(&capsules, tol))?;
    check_difference("erosion difference", region, &capsules, &cut, tol)?;
    Ok(cut)
}
