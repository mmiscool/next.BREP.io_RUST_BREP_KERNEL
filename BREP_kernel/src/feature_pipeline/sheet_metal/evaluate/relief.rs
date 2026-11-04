//! Bend-zone reliefs on the folded wedge.
//!
//! A relief is an exact loop in the band's developed coordinates (`u` across
//! from the parent fold line, `s` along the hinge). Its folded image is the
//! developed prism `loop × [−t/2 − m, t/2 + m]` under the fold map
//! `F(u, s, d) = ((R_mid + d) cos(u/R_n), s, −(R_mid + d) sin(u/R_n))` of the
//! canonical wedge (`R_n` the neutral radius, so `u = allowance` lands on the
//! bend angle): slanted relief edges become helicoidal walls, refit to
//! tolerance by [`super::warp::warp_solid`]. The cutter overshoots the wedge
//! in radius and angle, and only the wedge is cut, so neither overshoot
//! reaches the flats; they take the same loop as exact holes.
use super::*;

/// `curve` cut where its x (developed `u`) crosses 0 or `allowance`.
fn split_at_fold_lines(curve: &crate::NurbsCurve, allowance: f64) -> Result<Vec<crate::NurbsCurve>, String> {
    let [t0, t1] = curve.domain()?;
    const SAMPLES: usize = 64;
    let mut cuts = Vec::new();
    for line in [0.0, allowance] {
        let f = |t: f64| curve.evaluate(t).map(|p| p.x - line);
        let mut a = t0;
        let mut fa = f(a)?;
        for i in 1..=SAMPLES {
            let b = t0 + (t1 - t0) * i as f64 / SAMPLES as f64;
            let fb = f(b)?;
            if fa * fb < 0.0 {
                // Bisect the bracketed crossing to machine precision.
                let (mut lo, mut hi, mut flo) = (a, b, fa);
                for _ in 0..80 {
                    let mid = 0.5 * (lo + hi);
                    let fm = f(mid)?;
                    if (fm < 0.0) == (flo < 0.0) {
                        lo = mid;
                        flo = fm;
                    } else {
                        hi = mid;
                    }
                }
                let t = 0.5 * (lo + hi);
                if t > t0 + 1e-9 * (t1 - t0) && t < t1 - 1e-9 * (t1 - t0) {
                    cuts.push(t);
                }
            }
            a = b;
            fa = fb;
        }
    }
    cuts.sort_by(f64::total_cmp);
    let mut out = Vec::with_capacity(cuts.len() + 1);
    let mut rest = curve.clone();
    for t in cuts {
        let (head, tail) = rest.split(t)?;
        out.push(head);
        rest = tail;
    }
    out.push(rest);
    Ok(out)
}

pub(super) fn cut_bend_reliefs(
    wedge: BrepSolid,
    bend: &Bend,
    thickness: f64,
    edge_len: f64,
    mirrored_axis: bool,
) -> Result<BrepSolid, String> {
    if bend.reliefs.is_empty() {
        return Ok(wedge);
    }
    let r_mid = bend.mid_radius(thickness);
    let r_n = bend.neutral_radius(thickness);
    let half_t = thickness * 0.5;
    let margin = (0.25 * thickness).min(0.5 * (r_mid - half_t).max(1e-9));
    let tol = 1e-6 * (r_mid + thickness).max(1.0);
    let fold = move |p: Vec3| {
        let (r, phi) = (r_mid + p.z, p.x / r_n);
        Vec3::new(r * phi.cos(), p.y, -r * phi.sin())
    };
    let options = BooleanOptions {
        merge_coplanar_faces: true,
        ..BooleanOptions::default()
    };
    let mut current = wedge;
    for (index, relief) in bend.reliefs.iter().enumerate() {
        // Developed loop at the cutter's floor; a negative fold anchors the
        // canonical wedge at the hinge END (s → L − s), a reflection, so the
        // loop is reversed to keep its winding.
        let floor = -half_t - margin;
        // Split the loop where it crosses the two fold lines (u = 0, u = the
        // allowance): the warp fits each piece alone and interpolates its ends
        // exactly, so the notch wall meets the flats' exact hole walls along
        // the very same radial lines and the union sees coincident edges.
        let allowance = bend.allowance(thickness);
        let mut pieces = Vec::with_capacity(relief.outer.len() + 4);
        for curve in &relief.outer {
            pieces.extend(split_at_fold_lines(curve, allowance)?);
        }
        let mut profile = Vec::with_capacity(pieces.len());
        for curve in &pieces {
            let points = curve
                .control_points
                .iter()
                .map(|p| {
                    let q = p.point()?;
                    let s = if mirrored_axis { edge_len - q.y } else { q.y };
                    Ok(crate::Vec4::from_point(Vec3::new(q.x, s, floor), p.w))
                })
                .collect::<Result<Vec<_>, String>>()?;
            let moved = crate::NurbsCurve::new(curve.degree, curve.knots.clone(), points)?;
            profile.push(if mirrored_axis { moved.reversed()? } else { moved });
        }
        if mirrored_axis {
            profile.reverse();
        }
        if profile.len() == 1 {
            let single = profile.pop().expect("one curve");
            let domain = single.domain()?;
            let (first, second) = single.split((domain[0] + domain[1]) * 0.5)?;
            profile.push(first);
            profile.push(second);
        }
        let mut prism = extrude_profile_brep(&profile, Vec3::new(0.0, 0.0, 1.0), thickness + 2.0 * margin)?;
        let sides = profile.len();
        for (face_index, face) in prism.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()).enumerate() {
            face.name = Some(if face_index < sides {
                format!("{}:CUTOUT:{index}:{face_index}", bend.id)
            } else {
                format!("{}:CUTOUT:{index}", bend.id)
            });
        }
        let cutter = super::warp::warp_solid(&prism, &fold, tol)?;
        let body = crate::register_solid_value(current);
        let tool = crate::register_solid_value(cutter);
        let cut = crate::with_two_registered_solids(body, tool, |body, tool| {
            boolean_operation(body, tool, BooleanOperation::Subtract, &options)
        });
        crate::free_registered_solid(body);
        crate::free_registered_solid(tool);
        current = cut.map_err(|error| format!("sheet-metal bend relief on `{}`: {error}", bend.id))?;
    }
    Ok(current)
}
