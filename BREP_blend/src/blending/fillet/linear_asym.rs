//! A two-distance CHAMFER whose distances run LINEARLY along a straight edge
//! between two planes: `chamfer_edges_asymmetric_linear`.
//!
//! At each point of the edge the section square to it is the line from the
//! point `d1` into face 1 to the point `d2` into face 2 (the constant
//! two-distance chamfer's section, `chamfer_cross_section_offsets`). With
//! both distances linear along the edge, the rails on the two faces are
//! straight, and the chamfer is the ruled surface of the section lines
//! between the two end sections: a bilinear patch, a hyperbolic paraboloid
//! where the rails are skew and a plane where they are not. The cutter is the
//! loft between the two end triangles, ruled exactly, and the boolean takes
//! it from the body as the constant chamfer's prism is taken.
//!
//! Only where the cutter is flush: each end of the edge on a plane square to
//! it that holds that end's section. Anything else is refused by name.
use super::analyze::{analyze_edge, EdgePath};
use super::chamfer_extent::check_chamfer_extent;
use super::edges::resolve_edge_by_point;
use super::tool::{apply_tool, scaffold_tag, settle_cutter_scaffold};
use crate::analytic_surface::AnalyticSurface;
use crate::{make_line, BrepSolid, KernelRefusal, KernelStage, OrRefuse, Vec3};

const ENTRY: &str = "chamfer_edges_asymmetric_linear";

fn unsupported(what: &'static str, message: impl Into<String>) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{ENTRY}: {}", message.into()))
}

/// Two-distance chamfers of the straight edges through `edge_points`, one
/// after another, each with the distances `start` (`[d1, d2]`, `d1` on the
/// edge's first face as [`crate::fillet::chamfer_edges_asymmetric`] reads it)
/// at the end of the edge nearer `start_at` and `end` at its other end, both
/// linear between. Each edge lies between two planes and ends on planes
/// square to it.
pub fn chamfer_edges_asymmetric_linear(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    start_at: Vec3,
    start: [f64; 2],
    end: [f64; 2],
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let _operation = crate::blend::BlendOperation::enter();
    if edge_points.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "asymmetric_selection", format!("{ENTRY}: no edges selected")));
    }
    if start.iter().chain(&end).any(|d| !(d.is_finite() && *d > 0.0)) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "setback_distances",
            format!("{ENTRY}: every setback distance must be positive and finite, not {start:?} to {end:?}"),
        ));
    }
    let mut result = solid.clone();
    for point in edge_points {
        let edge_id = resolve_edge_by_point(&result, *point)?;
        result = chamfer_one(&result, edge_id, start_at, start, end, name)?;
    }
    if let Some(first) = result.validate_memoised().into_iter().find(|issue| issue.severity == "error") {
        return Err(KernelRefusal::new(
            crate::RefusalClass::InvalidResultTopology { issues: 1 },
            KernelStage::Validate,
            format!("{ENTRY}: the chamfer built a solid that fails validation (first: {}); refused rather than returned", first.message),
        ));
    }
    Ok(result)
}

fn chamfer_one(solid: &BrepSolid, edge_id: u64, start_at: Vec3, start: [f64; 2], end: [f64; 2], name: Option<&str>) -> Result<BrepSolid, KernelRefusal> {
    let smallest = start.iter().chain(&end).copied().fold(f64::INFINITY, f64::min);
    let cross = analyze_edge(solid, edge_id, smallest)?;
    let EdgePath::Straight { direction, length } = cross.path else {
        return Err(unsupported("linear_asymmetric_edge", "the edge is not a straight edge between two planes"));
    };
    let reach = [start[0].max(end[0]), start[1].max(end[1])];
    check_chamfer_extent(solid, edge_id, reach, ENTRY)?;
    // The law's start at the edge's end nearer `start_at`.
    let far = cross.start.add(direction.scale(length));
    let (first, last) = if far.sub(start_at).length() < cross.start.sub(start_at).length() { (end, start) } else { (start, end) };
    let tolerance = crate::KernelTolerances::for_solid(solid, 1e-7).intersection_fit;
    let section = |origin: Vec3, d: [f64; 2]| [origin, origin.add(cross.into_first.scale(d[0])), origin.add(cross.into_second.scale(d[1]))];
    let sections = [section(cross.start, first), section(far, last)];
    // Flush ends: each end section on a plane face of the body, square to
    // the edge.
    for corners in &sections {
        let flush = solid.shells.iter().flat_map(|shell| &shell.faces).any(|face| match face.surface.analytic() {
            Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => match u_dir.cross(*v_dir).normalized() {
                Ok(normal) => normal.cross(direction).length() <= 1e-9 && corners.iter().all(|c| normal.dot(c.sub(*origin)).abs() <= tolerance),
                Err(_) => false,
            },
            _ => false,
        });
        if !flush {
            return Err(unsupported("linear_asymmetric_end", "an end of the edge is not on a plane square to it"));
        }
    }
    let line = |from: Vec3, to: Vec3| make_line(from, to).or_refuse(KernelStage::Fragment, "linear_asymmetric_line");
    let profile = |[o, a, b]: [Vec3; 3]| -> Result<Vec<crate::NurbsCurve>, KernelRefusal> { Ok(vec![line(o, a)?, line(a, b)?, line(b, o)?]) };
    let mut tool = crate::loft_profile_brep(&[profile(sections[0])?, profile(sections[1])?])?;
    // The chamfer is the cutter's face ruled between the two chords; every
    // other face is its scaffold.
    let chords = [sections[0][1], sections[0][2], sections[1][1], sections[1][2]];
    let ruled_on_chords = |face: &crate::FaceRecord| {
        let poles: Vec<Vec3> = face.surface.control_points.iter().flatten().map(|p| Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w)).collect();
        let near = |q: Vec3| poles.iter().any(|p| p.sub(q).length() <= tolerance);
        chords.iter().all(|&q| near(q)) && poles.iter().all(|p| chords.iter().any(|q| p.sub(*q).length() <= tolerance))
    };
    let mut named = 0;
    for face in tool.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        if ruled_on_chords(face) {
            face.name = name.map(str::to_string);
            named += 1;
        } else {
            face.name = Some(scaffold_tag(name));
        }
    }
    if named != 1 {
        return Err(KernelRefusal::internal(KernelStage::Fragment, "linear_asymmetric_cutter", format!("{ENTRY}: the cutter's chamfer face is not one face ({named})")));
    }
    apply_tool(solid, &tool, cross.convex).map(settle_cutter_scaffold)
}
