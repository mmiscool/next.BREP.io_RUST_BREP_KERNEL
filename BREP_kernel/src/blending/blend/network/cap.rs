//! Planar termination of a stripe on curved carriers.
use super::*;
use crate::{fit, NurbsSurface, Vec4};

pub(super) struct Section {
    pub cr: f64,
    pub cs: f64,
    pub first: Vec3,
    pub second: Vec3,
    pub curve: NurbsCurve,
    pub pcurve: NurbsCurve,
    pub legs: [(NurbsCurve, NurbsCurve); 2],
}

fn refusal() -> KernelRefusal {
    KernelRefusal::non_convergence(
        KernelStage::Intersect,
        "cap_plane_section",
        "blend network: could not trace the cap plane on its carriers",
    )
}

fn crossing(
    curve: &NurbsCurve,
    origin: Vec3,
    normal: Vec3,
    seed: f64,
) -> Result<f64, KernelRefusal> {
    let [lo, hi] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let mut t = seed;
    for _ in 0..40 {
        let d = curve
            .derivatives(t, 1)
            .or_refuse(KernelStage::Refine, "derivatives")?;
        let distance = d[0].sub(origin).dot(normal);
        if distance.abs() < 1e-11 {
            return Ok(t);
        }
        let slope = d[1].dot(normal);
        if slope.abs() < 1e-14 {
            break;
        }
        t -= distance / slope;
        if !(t > lo && t < hi) {
            break;
        }
    }
    Err(refusal())
}

pub(super) fn plane_section(
    stripe: &Stripe,
    origin: Vec3,
    normal: Vec3,
    station: f64,
    at_start: bool,
) -> Result<Section, KernelRefusal> {
    let cr = crossing(&stripe.rows.cr, origin, normal, station)?;
    let cs = crossing(&stripe.rows.cs, origin, normal, station)?;
    let first = stripe
        .rows
        .cr
        .evaluate(cr)
        .or_refuse(KernelStage::Refine, "evaluate")?;
    let second = stripe
        .rows
        .cs
        .evaluate(cs)
        .or_refuse(KernelStage::Refine, "evaluate")?;
    let u = second
        .sub(first)
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let v = normal
        .cross(u)
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let reach = first.sub(origin).length().max(second.sub(origin).length()) * 4.0;
    let mut plane = stripe.first.face.clone();
    plane.surface = crate::make_plane(
        origin.sub(u.scale(reach)).sub(v.scale(reach)),
        u,
        v,
        2.0 * reach,
        2.0 * reach,
    )
    .or_refuse(KernelStage::Refine, "make_plane")?;
    let (curve, pcurve, _) = transverse_curve(&stripe.rows, &plane, cr, cs)?;
    let leg = |mate: &BlendMate, row: &NurbsCurve, t, point| {
        let start = row.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let edge_t = if at_start {
            stripe.edge.t0
        } else {
            stripe.edge.t1
        };
        let end = edge_uv_on_face(mate.coedge, stripe.edge, edge_t)?;
        plane_leg(
            &mate.face.surface,
            origin,
            normal,
            point,
            [start.x, start.y],
            end,
        )
    };
    let legs = [
        leg(&stripe.first, &stripe.rows.cr_pcurve, cr, first)?,
        leg(&stripe.second, &stripe.rows.cs_pcurve, cs, second)?,
    ];
    Ok(Section {
        cr,
        cs,
        first,
        second,
        curve,
        pcurve,
        legs,
    })
}

/// Trace the plane/carrier intersection from the rail to the sharp vertex.
/// The second equation advances along the leg's chord; the UV interpolation
/// is only a seed, never a substitute for the intersection.
fn plane_leg(
    surface: &NurbsSurface,
    origin: Vec3,
    normal: Vec3,
    rim: Vec3,
    start: [f64; 2],
    end: [f64; 2],
) -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
    let chord = origin.sub(rim);
    let length = chord.length();
    let direction = chord
        .normalized()
        .or_refuse(KernelStage::Refine, "normalized")?;
    let mut points = Vec::new();
    let mut uvs = Vec::new();
    let mut ts = Vec::new();
    for i in 0..=32 {
        let t = i as f64 / 32.0;
        let mut uv = [
            start[0] * (1.0 - t) + end[0] * t,
            start[1] * (1.0 - t) + end[1] * t,
        ];
        let mut converged = false;
        for _ in 0..40 {
            let d = surface
                .derivatives_extended(uv[0], uv[1], 1)
                .or_refuse(KernelStage::Refine, "derivatives_extended")?;
            let p = d[0][0];
            let f = [
                p.sub(origin).dot(normal),
                p.sub(rim).dot(direction) - t * length,
            ];
            if f[0].abs().max(f[1].abs()) < 1e-11 {
                converged = true;
                break;
            }
            let a = d[1][0].dot(normal);
            let b = d[0][1].dot(normal);
            let c = d[1][0].dot(direction);
            let d = d[0][1].dot(direction);
            let det = a * d - b * c;
            if det.abs() < 1e-16 {
                break;
            }
            uv[0] -= (d * f[0] - b * f[1]) / det;
            uv[1] -= (a * f[1] - c * f[0]) / det;
        }
        if !converged {
            return Err(refusal());
        }
        let point = surface
            .evaluate_extended(uv[0], uv[1])
            .or_refuse(KernelStage::Refine, "evaluate_extended")?;
        // Preserve the rail's shared endpoint; its carrier discrepancy is
        // checked with the complete sewn body's pcurve consistency policy.
        points.push(Vec4::from_point(
            if i == 0 {
                rim
            } else if i == 32 {
                origin
            } else {
                point
            },
            1.0,
        ));
        uvs.push(Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0));
        ts.push(t);
    }
    Ok((
        fit::interpolate_homogeneous(&points, 3, &ts)
            .or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
        fit::interpolate_homogeneous(&uvs, 3, &ts)
            .or_refuse(KernelStage::Refine, "interpolate_homogeneous")?,
    ))
}
