//! A chain of curves as one curve, and a surface cut along a parameter line.
//!
//! [`join_curve_chain`] makes one B-spline curve of curves that follow one
//! another head to tail. Nothing is fitted: every piece is raised to the
//! chain's highest degree (exactly), its knots are moved onto its own stretch
//! of the joined parameter, and its homogeneous poles are scaled so the weights
//! agree where it meets the piece before (which moves no point of it). The
//! joints become knots of full multiplicity, so the joined curve is as smooth
//! there as the chain is, and no smoother.
//!
//! [`split_surface`] cuts a surface along a parameter line into the two
//! surfaces on either side, by knot insertion on every row of its poles.
use crate::coons::compat::elevate_degree;
use crate::{curve_arc_length, NurbsCurve, NurbsSurface, Vec3, Vec4};

fn ends(curve: &NurbsCurve) -> Result<[Vec3; 2], String> {
    let [t0, t1] = curve.domain()?;
    Ok([curve.evaluate(t0)?, curve.evaluate(t1)?])
}

/// The chain `pieces` as one curve, piece `k` on a stretch of the parameter
/// `spans[k]` long, the first starting at 0.
pub(crate) fn join_curve_chain_on_spans(pieces: &[NurbsCurve], spans: &[f64]) -> Result<NurbsCurve, String> {
    if pieces.is_empty() || spans.len() != pieces.len() {
        return Err("joinCurveChain: a chain needs at least one curve, and a span for each".into());
    }
    if spans.iter().any(|span| !(span.is_finite() && *span > 0.0)) {
        return Err("joinCurveChain: every piece's parameter span must be positive".into());
    }
    let degree = pieces.iter().map(|piece| piece.degree).max().expect("a piece");
    let mut size = 0.0f64;
    let origin = ends(&pieces[0])?[0];
    for piece in pieces {
        for pole in &piece.control_points {
            size = size.max(Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w).sub(origin).length());
        }
    }
    let mut knots = vec![0.0; degree + 1];
    let mut poles: Vec<Vec4> = Vec::new();
    let mut offset = 0.0;
    for (index, piece) in pieces.iter().enumerate() {
        let raised = if piece.degree == degree { piece.clone() } else { elevate_degree(piece, degree)? };
        let [t0, t1] = raised.domain()?;
        if index > 0 {
            let gap = ends(&pieces[index - 1])?[1].sub(ends(piece)?[0]).length();
            if gap > 1e-9 * size {
                return Err(format!("joinCurveChain: curve {index} does not start where curve {} ends (gap {gap:.3e})", index - 1));
            }
        }
        // the piece's interior knots on its stretch, then the joint (or the end) at full multiplicity
        let scale = spans[index] / (t1 - t0);
        let interior = &raised.knots[degree + 1..raised.knots.len() - degree - 1];
        knots.extend(interior.iter().map(|knot| offset + (knot - t0) * scale));
        offset += spans[index];
        knots.extend(std::iter::repeat(offset).take(degree));
        // the weights made to agree at the joint: a common factor moves no point of the piece
        let factor = match poles.last() {
            Some(last) => last.w / raised.control_points[0].w,
            None => 1.0,
        };
        let scaled = raised.control_points.iter().map(|pole| Vec4 { x: pole.x * factor, y: pole.y * factor, z: pole.z * factor, w: pole.w * factor });
        poles.extend(scaled.skip(usize::from(index > 0)));
    }
    knots.push(offset);
    NurbsCurve::new(degree, knots, poles)
}

/// The chain `pieces`, curves that follow one another head to tail, as one
/// curve: exactly the same points, each piece on a stretch of the parameter as
/// long as the piece itself, so the joined curve runs at an even pace from
/// piece to piece. Its joints are knots of full multiplicity (the curve is as
/// smooth there as the chain is).
///
/// Refuses an empty chain and pieces that do not meet end to start.
pub fn join_curve_chain(pieces: &[NurbsCurve]) -> Result<NurbsCurve, String> {
    let mut spans = Vec::with_capacity(pieces.len());
    for piece in pieces {
        let [t0, t1] = piece.domain()?;
        spans.push(curve_arc_length(piece, t0, t1)?);
    }
    join_curve_chain_on_spans(pieces, &spans)
}

/// `surface` cut along the parameter line `at` (a `v` line when `along_v`,
/// else a `u` line) into the part before it and the part after it. Exact: knot
/// insertion on each row of poles across the cut.
pub fn split_surface(surface: &NurbsSurface, along_v: bool, at: f64) -> Result<(NurbsSurface, NurbsSurface), String> {
    let net = &surface.control_points;
    let (rows, columns) = (net.len(), net.first().map_or(0, Vec::len));
    let (degree, knots, lines) = if along_v { (surface.degree_v, &surface.knots_v, rows) } else { (surface.degree_u, &surface.knots_u, columns) };
    let (mut before, mut after) = (Vec::with_capacity(lines), Vec::with_capacity(lines));
    for line in 0..lines {
        let poles: Vec<Vec4> = if along_v { net[line].clone() } else { (0..rows).map(|row| net[row][line]).collect() };
        let (first, second) = NurbsCurve::new(degree, knots.clone(), poles)?.split(at)?;
        before.push(first);
        after.push(second);
    }
    let part = |curves: &[NurbsCurve]| -> Result<NurbsSurface, String> {
        let count = curves[0].control_points.len();
        if along_v {
            let net = curves.iter().map(|curve| curve.control_points.clone()).collect();
            NurbsSurface::new(surface.degree_u, degree, surface.knots_u.clone(), curves[0].knots.clone(), net)
        } else {
            let net = (0..count).map(|row| curves.iter().map(|curve| curve.control_points[row]).collect()).collect();
            NurbsSurface::new(degree, surface.degree_v, curves[0].knots.clone(), surface.knots_v.clone(), net)
        }
    };
    Ok((part(&before)?, part(&after)?))
}

