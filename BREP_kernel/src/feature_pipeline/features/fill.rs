//! FILL — Fill Surface: a sheet through picked curves.
//!
//! Two ways to fill, chosen by what is picked:
//!
//! - **Boundary fill**: `curves` only. The picks must close into ONE loop. The
//!   loop is cut into sides at its corners (where one curve leaves at an angle
//!   to the last), and curves meeting smoothly join into one side. When the
//!   corners give fewer than three sides (a loop of smooth arcs), each picked
//!   curve is a side. Four sides fill with the bilinear Coons patch
//!   ([`crate::coons_bilinear_reported`]): the patch's edges are the input
//!   curves exactly, and its interior is the Coons blend of them. Three, five
//!   or six sides fill with one patch per corner ([`crate::n_sided_fill`]),
//!   faces `${featureId}:FILL:P{k}` under the container `${featureId}:FILL`.
//! - **Tangent continuity** (`continuity: TANGENT`, a boundary fill only):
//!   every boundary pick is an open edge of a face, and the N-sided fill is
//!   given that face's tangent plane along the side; the worst normal angle
//!   it reaches, to the faces and across its own patches, is the note. It is
//!   a measured match, not a guarantee.
//! - **Curve network**: `curves` and `crossCurves`, two families of curves that
//!   cross one another, at least two of each. The sheet is the Gordon surface
//!   ([`crate::gordon_surface_reported`]): it runs through every curve of both
//!   families. A family whose curves close round (a set of circular sections)
//!   gives a closed sheet.
//!
//! Each family member (and each side) is one chain of picks meeting end to end:
//! two picked edges that meet in line are one curve of the network.
//!
//! Picks (contract rule 1): a pick is a sketch or 3D sketch (its whole chain,
//! or every curve of it when it publishes no single chain), one of its curves
//! (`{sketch}:G{gid}`), a spline, or a resident edge. A pick that resolves to
//! nothing is an `unresolved` entry; a fill with nothing resolved is a soft
//! no-op. A construction that refuses is an error carrying the constructor's
//! refusal and its measurement verbatim.
//!
//! Naming (contract rule 2): the sheet is `${featureId}`, its one face
//! `${featureId}:FILL`, its edges the derived `${featureId}:FILL|BOUNDARY[n]`.
//! Every edge built on a picked curve is also reachable by that curve:
//! `${featureId}:${pick}` (the picks of a side made of several curves joined
//! with `+`), so a later feature that picks an edge of the fill does not care
//! in which order the sheet's edges come.
//!
//! What was built is reported as a feature note: the Coons boundary residual,
//! or the Gordon crossing gap and interpolation bound. The result is a sheet
//! (open faces); Thicken turns it into a solid.

use crate::feature_pipeline::features::common::{self, PathSegment};
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult};
use crate::{NurbsCurve, NurbsSurface, Vec3};

/// The turn between one curve's end and the next curve's start (radians)
/// above which the joint is a corner of the boundary. Curves a sketch joins
/// tangentially meet to round-off; anything that turns by more than this was
/// drawn as a corner.
pub const CORNER_ANGLE: f64 = 1e-3;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    let curve_names = common::unique_reference_names(ctx.param("curves"));
    let cross_names = common::unique_reference_names(ctx.param("crossCurves"));
    let curves = gather(ctx, &curve_names, &mut result)?;
    let cross = gather(ctx, &cross_names, &mut result)?;
    let requested: Vec<String> = curve_names.iter().chain(&cross_names).cloned().collect();
    if curves.is_empty() && cross.is_empty() {
        return Ok(result); // Soft no-op — nothing picked, or nothing resolved.
    }
    result.note_partial_resolution(&requested);

    let tangent = match ctx.string("continuity").unwrap_or_default().to_ascii_uppercase().as_str() {
        "" | "POSITION" => false,
        "TANGENT" => true,
        other => return Err(format!("Fill: unknown continuity '{other}' (POSITION, TANGENT)").into()),
    };
    let built = if cross.is_empty() {
        boundary_fill(ctx, curves, tangent)?
    } else {
        if tangent {
            return Err("Fill: a curve network fills through its curves only; tangent continuity is for a boundary fill (leave Cross curves empty)".into());
        }
        network_fill(curves, cross)?
    };
    let face_name = format!("{}:FILL", ctx.id);
    let mut face_groups = Vec::new();
    let mut sheet = if built.surfaces.len() == 1 {
        check_regular(&built.surfaces[0])?;
        let mut sheet = crate::face_from_surface(&built.surfaces[0])?;
        for shell in &mut sheet.shells {
            for face in &mut shell.faces {
                face.name = Some(face_name.clone());
            }
        }
        sheet
    } else {
        // one face per patch, `{id}:FILL:P{k}` for the patch at the start of
        // side k, and `{id}:FILL` the container of them all
        let mut pieces = Vec::with_capacity(built.surfaces.len());
        let mut members = Vec::with_capacity(built.surfaces.len());
        for (index, surface) in built.surfaces.iter().enumerate() {
            check_regular(surface).map_err(|error| format!("{error} (patch {index})"))?;
            let mut piece = crate::face_from_surface(surface)?;
            let name = format!("{face_name}:P{index}");
            for shell in &mut piece.shells {
                for face in &mut shell.faces {
                    face.name = Some(name.clone());
                }
            }
            members.push(name);
            pieces.push(piece);
        }
        face_groups.push((face_name.clone(), members));
        // the patches share their spokes and corners exactly (each is built
        // from the same curves), so they sew at a tight band, never the
        // corner band a side's ends were admitted within
        // (relative to the fill's own size, wherever it stands)
        let points: Vec<Vec3> = pieces.iter().flat_map(|piece| piece.vertices.iter().map(|vertex| vertex.point)).collect();
        let (low, high) = points.iter().fold((points[0], points[0]), |(low, high), p| {
            (Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z)), Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z)))
        });
        crate::sheet_from_sheets(&pieces, 1e-9 * high.sub(low).length())?
    };
    crate::feature_pipeline::features::common::stamp_derived_edge_names(&mut sheet);
    let edge_groups = edge_groups_by_source(&sheet, &built.sides, &ctx.id, built.band)?;
    result.notes.push(built.note);

    let face_names = common::collect_face_names(&sheet);
    let edge_names = common::collect_edge_names(&sheet);
    let handle = crate::register_solid_value(sheet);
    result.added.push(AddedSolid {
        handle,
        name: ctx.id.clone(),
        face_names,
        edge_names,
        edge_groups,
        face_groups,
        ..AddedSolid::default()
    });
    Ok(result)
}

/// The curves `names` pick, each a named segment; misses go to `unresolved`.
fn gather(ctx: &FeatureContext, names: &[String], result: &mut FeatureResult) -> Result<Vec<PathSegment>, String> {
    let mut segments: Vec<PathSegment> = Vec::new();
    for name in names {
        let picked = pick_segments(ctx, name)?;
        if picked.is_empty() {
            result.unresolved.push(name.clone());
        }
        for segment in picked {
            if !segments.iter().any(|seen| seen.name == segment.name) {
                segments.push(segment);
            }
        }
    }
    Ok(segments)
}

/// What one pick names, as segments: a sketch or 3D sketch is every one of
/// its curves (`{sketch}:G{gid}`, in id order), whether or not they form one
/// run; any other published path (one curve of a sketch, a spline) is its own
/// curves under their published names; a resident edge is its curve under its
/// own name. Picking one curve is never widened to its whole sketch. Empty when
/// the pick names nothing.
fn pick_segments(ctx: &FeatureContext, name: &str) -> Result<Vec<PathSegment>, String> {
    // A sketch's own curves first: its single published chain is only its
    // longest run, so a sketch of several separate curves would lose the rest.
    let base = crate::feature_pipeline::canonical_sketch_name(name);
    let prefix = format!("{base}:G");
    let mut members: Vec<(u64, &String, &Vec<NurbsCurve>)> = ctx
        .scene
        .paths
        .iter()
        .filter_map(|(key, curves)| {
            let gid = key.strip_prefix(&prefix)?.parse::<u64>().ok()?;
            Some((gid, key, curves))
        })
        .collect();
    if !members.is_empty() {
        members.sort_by_key(|(gid, _, _)| *gid);
        let mut segments = Vec::new();
        for (_, key, curves) in members {
            for (index, curve) in curves.iter().enumerate() {
                let segment_name = if curves.len() == 1 { key.clone() } else { format!("{key}[{index}]") };
                segments.push(PathSegment { name: segment_name, curve: curve.clone(), axis: None });
            }
        }
        return Ok(segments);
    }
    if let Some(chain) = ctx.scene.resolve_path(name) {
        let published = ctx.scene.resolve_path_segment_names(name);
        let single = chain.len() == 1;
        return Ok(chain
            .iter()
            .enumerate()
            .map(|(index, curve)| {
                let segment_name = published
                    .and_then(|names| names.get(index).cloned().flatten())
                    .filter(|name| !name.trim().is_empty())
                    .unwrap_or_else(|| if single { name.to_string() } else { format!("{name}[{index}]") });
                PathSegment { name: segment_name, curve: curve.clone(), axis: None }
            })
            .collect());
    }
    if let Some(edge) = ctx.scene.edges.get(name) {
        return Ok(vec![PathSegment { name: name.to_string(), curve: common::edge_curve(*edge)?, axis: None }]);
    }
    // an edge alias of several edges (a fill of one patch per corner names
    // both halves of a side by the curve they were built on): each member
    if let Some(members) = ctx.scene.edge_groups.get(name) {
        let mut segments = Vec::with_capacity(members.len());
        for member in members {
            if let Some(edge) = ctx.scene.resolve_edge(member) {
                segments.push(PathSegment { name: member.clone(), curve: common::edge_curve(edge)?, axis: None });
            }
        }
        if !segments.is_empty() {
            return Ok(segments);
        }
    }
    if let Some(edge) = ctx.scene.resolve_edge(name) {
        return Ok(vec![PathSegment { name: name.to_string(), curve: common::edge_curve(edge)?, axis: None }]);
    }
    Ok(Vec::new())
}

/// The surface refused where it folds or collapses, by the exact certificate
/// of [`crate::certify_regular`]: the normal (`∂S/∂u × ∂S/∂v`) is held in
/// Bernstein form and certified to vanish nowhere and never turn back (a
/// self-crossing boundary gives a bow tie; a network that folds back gives a
/// pleat), however narrow the fold. The four corners of the domain may
/// vanish: where two sides meet tangentially (a disc bounded by four arcs) the
/// corner is a parameter singularity of a regular sheet.
fn check_regular(surface: &NurbsSurface) -> Result<(), String> {
    match crate::certify_regular(surface) {
        Ok(_) => Ok(()),
        Err(crate::Irregularity::Collapses { u, v }) => Err(format!(
            "Fill: the surface collapses at ({u:.4}, {v:.4}) of its parameters (its normal vanishes): the curves fold the sheet onto itself; check that the boundary does not cross itself"
        )),
        Err(crate::Irregularity::FoldsOver { u, v }) => Err(format!(
            "Fill: the surface folds over near ({u:.4}, {v:.4}) of its parameters (its normal turns back): the curves fold the sheet onto itself; check that the boundary does not cross itself"
        )),
        Err(crate::Irregularity::Uncertified { u, v, reason }) => Err(format!(
            "Fill: the surface could not be certified free of folds near ({u:.4}, {v:.4}) of its parameters ({reason}); the curves may fold the sheet onto itself"
        )),
    }
}

/// One input curve of the fill, built from one chain of picks, and the picks'
/// names (`+`-joined).
struct Side {
    source: String,
    curve: NurbsCurve,
    segments: Vec<PathSegment>,
}

/// What a fill built: its patches (one, or one per corner of an N-sided
/// boundary), the curves its edges may lie on (for the edge aliases), its
/// note and the band its corners were met within.
struct Built {
    surfaces: Vec<NurbsSurface>,
    sides: Vec<Side>,
    note: String,
    band: f64,
}

fn ends(curve: &NurbsCurve) -> Result<(Vec3, Vec3), String> {
    let [t0, t1] = curve.domain()?;
    Ok((curve.evaluate(t0)?, curve.evaluate(t1)?))
}

/// One chain's segments as one curve: a single segment as it is (no
/// re-parameterisation), several joined end to end.
fn side_of(segments: &[PathSegment]) -> Result<Side, String> {
    let source = segments.iter().map(|segment| segment.name.as_str()).collect::<Vec<_>>().join("+");
    let curve = if segments.len() == 1 {
        segments[0].curve.clone()
    } else {
        let pieces: Vec<NurbsCurve> = segments.iter().map(|segment| segment.curve.clone()).collect();
        crate::join_curve_chain(&pieces).map_err(|error| format!("Fill: the curves {source} do not join into one curve: {error}"))?
    };
    Ok(Side { source, curve, segments: segments.to_vec() })
}

/// The angle (radians, `atan2` form) between `a`'s direction at its end and
/// `b`'s at its start.
fn turn(a: &NurbsCurve, b: &NurbsCurve) -> Result<f64, String> {
    let (_, out) = a.deriv1(a.domain()?[1])?;
    let (_, into) = b.deriv1(b.domain()?[0])?;
    let (x, y) = (out.cross(into).length(), out.dot(into));
    if !(x.is_finite() && y.is_finite()) || (x == 0.0 && y == 0.0) {
        return Err("Fill: a boundary curve has no direction at a joint".into());
    }
    Ok(x.atan2(y))
}

/// Boundary fill: the picks must close into one loop; its sides fill a Coons
/// patch when there are four.
fn boundary_fill(ctx: &FeatureContext, segments: Vec<PathSegment>, tangent: bool) -> Result<Built, String> {
    let mut chains = common::group_path_segments(segments)?;
    if chains.len() != 1 {
        let runs: Vec<String> = chains
            .iter()
            .map(|chain| chain.iter().map(|segment| segment.name.as_str()).collect::<Vec<_>>().join("+"))
            .collect();
        return Err(format!(
            "Fill: the boundary curves form {} separate runs ({}), not one closed loop; pick a loop, or put the second family of a curve network in Cross curves",
            chains.len(),
            runs.join("; ")
        ));
    }
    let chain = chains.remove(0);
    let (head, _) = ends(&chain[0].curve)?;
    let (_, tail) = ends(&chain[chain.len() - 1].curve)?;
    let scale = chain
        .iter()
        .map(|segment| ends(&segment.curve).map(|(a, b)| a.sub(head).length().max(b.sub(head).length())))
        .collect::<Result<Vec<f64>, String>>()?
        .into_iter()
        .fold(1.0_f64, f64::max);
    let gap = tail.sub(head).length();
    if gap > 1e-5 * scale {
        return Err(format!(
            "Fill: the boundary curves do not close into a loop (the run ends {gap:.3e} from where it starts); pick curves that close, or put the second family of a curve network in Cross curves"
        ));
    }
    // Cut the loop at its corners into runs of smoothly joined curves.
    let count = chain.len();
    let mut corner = Vec::with_capacity(count);
    for index in 0..count {
        corner.push(turn(&chain[index].curve, &chain[(index + 1) % count].curve)? > CORNER_ANGLE);
    }
    let corners = corner.iter().filter(|is| **is).count();
    let sides: Vec<Side> = if corners < 3 {
        chain.iter().map(|segment| side_of(std::slice::from_ref(segment))).collect::<Result<_, _>>()?
    } else {
        // Start right after a corner, so no side wraps past the loop's start.
        let start = (corner.iter().position(|is| *is).expect("corners >= 3") + 1) % count;
        let mut sides = Vec::new();
        let mut run: Vec<PathSegment> = Vec::new();
        for step in 0..count {
            let index = (start + step) % count;
            run.push(chain[index].clone());
            if corner[index] {
                sides.push(side_of(&run)?);
                run.clear();
            }
        }
        sides
    };
    // A re-entrant corner cannot be a patch corner (the patch folds there):
    // such a loop goes to the N-sided fill, which makes it a spoke point.
    let curves: Vec<NurbsCurve> = sides.iter().map(|side| side.curve.clone()).collect();
    let reentrant = crate::n_sided_reentrant_corners(&curves).map_err(|refusal| format!("Fill: {refusal}"))?;
    if sides.len() == 4 && !tangent && reentrant.is_empty() {
        let boundaries = [sides[0].curve.clone(), sides[1].curve.clone(), sides[2].curve.clone(), sides[3].curve.clone()];
        let (surface, report) = crate::coons_bilinear_reported(boundaries).map_err(|refusal| format!("Fill: {refusal}"))?;
        let note = format!(
            "Coons patch of 4 sides: boundary residual {:.3e} (corner band {:.3e}), degree {}x{}",
            report.boundary_residual, report.corner_band, report.degree_u, report.degree_v
        );
        return Ok(Built { surfaces: vec![surface], sides, note, band: report.corner_band });
    }
    if !(3..=6).contains(&(sides.len() - reentrant.len().min(sides.len()))) {
        return Err(format!(
            "Fill: the boundary has {} sides ({}); a boundary fill takes 3 to 6 sides (a side each side of a re-entrant corner counts once). Split or join the boundary, or fill a curve network (Curves and Cross curves)",
            sides.len(),
            sides.iter().map(|side| side.source.as_str()).collect::<Vec<_>>().join(", ")
        ));
    }
    let fields = if tangent {
        Some(sides.iter().map(|side| tangent_field(ctx, side)).collect::<Result<Vec<_>, _>>()?)
    } else {
        None
    };
    let (patches, report) = crate::n_sided_fill(&curves, fields.as_deref()).map_err(|refusal| format!("Fill: {refusal}"))?;
    let mut note = format!(
        "N-sided fill of {} sides ({} patches): across the spokes the patches' normals meet within {:.3e} rad",
        report.sides,
        patches.len(),
        report.spoke_normal_angle
    );
    if let Some(angle) = report.tangent_normal_angle {
        note.push_str(&format!(", the neighbouring faces' tangent planes within {angle:.3e} rad"));
    }
    if report.boundary_deviation > 0.0 {
        note.push_str(&format!(", rational sides held within {:.3e} by cubics", report.boundary_deviation));
    }
    if !report.reentrant_corners.is_empty() {
        let names: Vec<String> = report.reentrant_corners.iter().map(|corner| corner.to_string()).collect();
        note.push_str(&format!(
            "; re-entrant corner{} {} filled as spoke point{}, centred in the loop's kernel",
            if names.len() == 1 { "" } else { "s" },
            names.join(", "),
            if names.len() == 1 { "" } else { "s" }
        ));
    }
    // the patches' edges are the sides' halves, and the sides at a
    // re-entrant corner whole
    let count = sides.len();
    let mut halves = Vec::with_capacity(2 * count);
    for (index, side) in sides.iter().enumerate() {
        let at_reentrant = |corner: usize| report.reentrant_corners.contains(&(corner % count));
        if at_reentrant(index) || at_reentrant(index + 1) {
            halves.push(Side { source: side.source.clone(), curve: side.curve.clone(), segments: Vec::new() });
            continue;
        }
        // where the fill cut the side: its midpoint, or a knot of its own
        // right beside it (`NSidedReport::split_parameters`)
        let [t0, t1] = side.curve.domain()?;
        let cut = report.split_parameters.get(index).copied().flatten().unwrap_or(0.5 * (t0 + t1));
        let (first, second) = side.curve.split(cut)?;
        for curve in [first, second] {
            halves.push(Side { source: side.source.clone(), curve, segments: Vec::new() });
        }
    }
    let band = report.corner_band.max(report.boundary_deviation);
    Ok(Built { surfaces: patches, sides: halves, note, band })
}

/// The direction across `side` in the tangent plane of the face beside it:
/// at 65 points along the side, the normal of the face its picked edge
/// bounds (the point projected onto that face's surface) crossed with the
/// side's tangent, interpolated over the side's own parameter. Each pick of
/// the side must be an edge that bounds one face (a sheet's open edge).
fn tangent_field(ctx: &FeatureContext, side: &Side) -> Result<NurbsCurve, String> {
    let mut supports: Vec<(NurbsCurve, NurbsSurface)> = Vec::with_capacity(side.segments.len());
    for segment in &side.segments {
        let edge = ctx.scene.resolve_edge(&segment.name).ok_or_else(|| {
            format!("Fill: tangent continuity needs edges of a face; '{}' is a curve with no face beside it", segment.name)
        })?;
        let surface = crate::with_registered_solid_str(edge.handle, |solid| {
            let faces: Vec<&crate::topology::FaceRecord> = solid
                .shells
                .iter()
                .flat_map(|shell| &shell.faces)
                .filter(|face| face.loops.iter().flat_map(|record| &record.coedges).any(|coedge| coedge.edge_id == edge.edge_id))
                .collect();
            match faces.as_slice() {
                [face] => Ok(face.surface.clone()),
                [] => Err(format!("Fill: the edge '{}' bounds no face to be tangent to", segment.name)),
                _ => Err(format!(
                    "Fill: the edge '{}' bounds {} faces; tangent continuity needs an open edge (one face beside it)",
                    segment.name,
                    faces.len()
                )),
            }
        })?;
        supports.push((segment.curve.clone(), surface));
    }
    const SAMPLES: usize = 65;
    let [t0, t1] = side.curve.domain()?;
    let mut parameters = Vec::with_capacity(SAMPLES);
    let mut values = Vec::with_capacity(SAMPLES);
    for index in 0..SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / (SAMPLES - 1) as f64;
        let (point, along) = side.curve.deriv1(t)?;
        // the face of the picked edge this point lies on
        let mut best: Option<(f64, &NurbsSurface)> = None;
        for (curve, surface) in &supports {
            let distance = crate::project_point_to_curve(curve, point)?.distance;
            if best.is_none_or(|(least, _)| distance < least) {
                best = Some((distance, surface));
            }
        }
        let surface = best.expect("a side has a segment").1;
        let foot = crate::project_point_to_surface(surface, point)?;
        let (_, su, sv) = surface.deriv1(foot.u, foot.v)?;
        parameters.push(t);
        values.push(su.cross(sv).cross(along));
    }
    crate::interpolate_curve(&values, 3, &parameters)
}

/// Curve network: each family's picks grouped into curves, then the Gordon
/// surface through both families.
fn network_fill(curves: Vec<PathSegment>, cross: Vec<PathSegment>) -> Result<Built, String> {
    let family = |segments: Vec<PathSegment>, what: &str| -> Result<Vec<Side>, String> {
        common::group_path_segments(segments)
            .map_err(|error| format!("Fill: {what}: {error}"))?
            .iter()
            .map(|chain| side_of(chain))
            .collect()
    };
    let u_sides = family(curves, "Curves")?;
    let v_sides = family(cross, "Cross curves")?;
    let u: Vec<NurbsCurve> = u_sides.iter().map(|side| side.curve.clone()).collect();
    let v: Vec<NurbsCurve> = v_sides.iter().map(|side| side.curve.clone()).collect();
    let (surface, report) = crate::gordon_surface_reported(&u, &v, None).map_err(|refusal| {
        format!("Fill: curve network ({} curves by {} cross curves): {refusal}", u.len(), v.len())
    })?;
    let closed = match (report.periodic_u, report.periodic_v) {
        (false, false) => "",
        (true, false) => ", closed in u",
        (false, true) => ", closed in v",
        (true, true) => ", closed in u and v",
    };
    let note = format!(
        "Gordon surface through {} curves by {} cross curves{closed}: crossing gap {:.3e} (band {:.3e}), interpolation bound {:.3e}",
        u.len(),
        v.len(),
        report.crossing_gap,
        report.crossing_band,
        report.interpolation_bound
    );
    let mut sides = u_sides;
    sides.extend(v_sides);
    Ok(Built { surfaces: vec![surface], sides, note, band: report.crossing_band })
}

/// `${id}:${source}` for each sheet edge that lies on an input curve: the edge's
/// two ends and midpoint each within `band` of the curve's own (an edge of the
/// sheet is an input curve end for end, but for the corners the constructor
/// moved to meet, which it admits within its corner or crossing band; the
/// nearest curve wins). Edges on no input curve (a periodic sheet's seam) get
/// no alias.
fn edge_groups_by_source(sheet: &crate::BrepSolid, sides: &[Side], id: &str, band: f64) -> Result<Vec<(String, Vec<String>)>, String> {
    let mut groups: Vec<(String, Vec<String>)> = Vec::new();
    let mut uses = std::collections::HashMap::<u64, usize>::new();
    for coedge in sheet.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops).flat_map(|record| &record.coedges) {
        *uses.entry(coedge.edge_id).or_default() += 1;
    }
    for edge in &sheet.edges {
        let Some(edge_name) = &edge.name else { continue };
        // only the sheet's open edges lie on the picked curves (a joint
        // between two patches is a spoke, a periodic sheet's seam is neither)
        if edge.degenerate || uses.get(&edge.id).copied().unwrap_or(0) != 1 {
            continue;
        }
        let at = |t: f64| edge.curve.evaluate(t);
        let points = [at(edge.t0)?, at(0.5 * (edge.t0 + edge.t1))?, at(edge.t1)?];
        let size = points[0].sub(points[2]).length().max(points[0].sub(points[1]).length());
        let band = band.max(1e-7 * size.max(1.0));
        let mut best: Option<(f64, usize)> = None;
        for (index, side) in sides.iter().enumerate() {
            let [s0, s1] = side.curve.domain()?;
            let theirs = [side.curve.evaluate(s0)?, side.curve.evaluate(0.5 * (s0 + s1))?, side.curve.evaluate(s1)?];
            // the same curve either way round
            let forward = (0..3).map(|k| points[k].sub(theirs[k]).length()).fold(0.0_f64, f64::max);
            let backward = (0..3).map(|k| points[k].sub(theirs[2 - k]).length()).fold(0.0_f64, f64::max);
            let miss = forward.min(backward);
            if miss <= band && best.is_none_or(|(least, _)| miss < least) {
                best = Some((miss, index));
            }
        }
        if let Some((_, index)) = best {
            let alias = format!("{id}:{}", sides[index].source);
            match groups.iter_mut().find(|(name, _)| *name == alias) {
                Some((_, members)) => members.push(edge_name.clone()),
                None => groups.push((alias, vec![edge_name.clone()])),
            }
        }
    }
    Ok(groups)
}

/// Context-bar applicability: picked edges (or a sketch) can be filled.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "FILL",
    "shortName": "FILL",
    "longName": "Fill Surface",
    "ribbonPath": "Home/Modeling/Fill Surface",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the fill surface feature"
        },
        "curves": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE",
                "SKETCH"
            ],
            "multiple": true,
            "default_value": [],
            "label": "Curves",
            "hint": "Pick the boundary: curves or edges that close into one loop of 3 to 6 sides (a sketch's outline counts). Four sides fill with one Coons patch; 3, 5 or 6 sides (or tangent continuity) with one patch per corner. With Cross curves picked too, these are the first family of a curve network, at least two curves, each crossing every cross curve"
        },
        "crossCurves": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE",
                "SKETCH"
            ],
            "multiple": true,
            "default_value": [],
            "label": "Cross curves",
            "hint": "Optional: the second family of a curve network, at least two curves, each crossing every curve of the first. The surface then runs through every curve of both families (a Gordon surface). Leave empty to fill a boundary loop"
        },
        "continuity": {
            "type": "options",
            "options": [
                "POSITION",
                "TANGENT"
            ],
            "default_value": "POSITION",
            "label": "Continuity",
            "hint": "POSITION: the fill meets the boundary curves. TANGENT: it also meets the faces beside the boundary tangentially; every boundary pick must then be an open edge of a face (an edge with one face beside it). The worst normal angle it reaches is reported in the feature's note"
        }
    }
})
}

