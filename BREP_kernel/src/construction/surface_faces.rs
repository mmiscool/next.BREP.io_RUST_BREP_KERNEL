//! Faces on given surfaces, and the solid a set of such faces bounds.
//!
//! [`face_from_surface`] makes a sheet of one face on a surface over its whole
//! parameter domain. The face's four sides are the surface's boundary
//! iso-curves, read for what they are:
//!
//! - a side that is one point (every pole of the row coincides) is a POLE, a
//!   degenerate edge at a vertex;
//! - two opposite sides that coincide are a SEAM, one edge the face uses once
//!   each way (a cylinder's side has one seam, a ball's one seam and two
//!   poles, a ring's two seams);
//! - any other side is an open edge of the sheet.
//!
//! [`face_from_surface_trimmed`] makes a sheet of one face on a surface
//! bounded by closed loops of curves lying on it (a disc on a plane inside a
//! circle, a wall on a plane under a spline).
//!
//! [`solid_from_sheets`] joins sheets along their coinciding edges (the
//! kernel's sewing, [`crate::sew_solid`]) and returns the solid they bound,
//! outward. Sheets that leave an edge open are refused by name; nothing is
//! closed by guessing.
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{
    build_pcurve_on_surface, make_line, sew_solid, BodyKind, BrepSolid, KernelRefusal, KernelStage, NurbsCurve,
    NurbsSurface, OrRefuse, Vec3,
};

/// The prefix of every refusal of this module.
pub const SURFACE_FACE_REFUSAL: &str = "surfaceFace";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{SURFACE_FACE_REFUSAL}: {message}"))
}

fn refuse(stage: KernelStage, what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(stage, what, format!("{SURFACE_FACE_REFUSAL}: {message}"))
}

/// The records of a sheet under construction, ids counted from 1.
struct Records {
    vertices: Vec<VertexRecord>,
    edges: Vec<EdgeRecord>,
    next: u64,
    tolerance: f64,
}

impl Records {
    fn new(tolerance: f64) -> Self {
        Self { vertices: Vec::new(), edges: Vec::new(), next: 0, tolerance }
    }

    fn id(&mut self) -> u64 {
        self.next += 1;
        self.next
    }

    /// The vertex at `point`: an earlier one within the tolerance, or a new one.
    fn vertex(&mut self, point: Vec3) -> u64 {
        if let Some(found) = self.vertices.iter().find(|vertex| vertex.point.sub(point).length() <= self.tolerance) {
            return found.id;
        }
        let id = self.id();
        self.vertices.push(VertexRecord { id, point });
        id
    }

    fn edge(&mut self, curve: NurbsCurve, start: u64, end: u64, degenerate: bool) -> Result<u64, KernelRefusal> {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "surfaceFace.edge_domain")?;
        let id = self.id();
        self.edges.push(EdgeRecord { id, curve, t0, t1, start_vertex_id: start, end_vertex_id: end, degenerate, name: None });
        Ok(id)
    }

    fn sheet(mut self, surface: &NurbsSurface, loops: Vec<Vec<(u64, bool, NurbsCurve)>>) -> BrepSolid {
        let mut records = Vec::with_capacity(loops.len());
        for coedges in loops {
            let coedges = coedges
                .into_iter()
                .map(|(edge_id, forward, pcurve)| CoedgeRecord { id: self.id(), edge_id, forward, pcurve })
                .collect();
            records.push(LoopRecord { id: self.id(), coedges });
        }
        let face = FaceRecord { id: self.id(), surface: surface.clone(), same_sense: true, loops: records, name: None };
        let shell = ShellRecord { id: self.id(), faces: vec![face] };
        BrepSolid {
            mass_properties_cache: Default::default(),
            id: self.id(),
            vertices: self.vertices,
            edges: self.edges,
            shells: vec![shell],
            genus: 0,
            kind: BodyKind::Sheet,
            cells: Default::default(),
        }
    }
}

/// A length the surface's extent sets: the diagonal of its poles' box.
fn extent(surface: &NurbsSurface) -> f64 {
    let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for pole in surface.control_points.iter().flatten() {
        let point = [pole.x / pole.w, pole.y / pole.w, pole.z / pole.w];
        for axis in 0..3 {
            low[axis] = low[axis].min(point[axis]);
            high[axis] = high[axis].max(point[axis]);
        }
    }
    Vec3::new(high[0] - low[0], high[1] - low[1], high[2] - low[2]).length()
}

/// The point every pole of `curve` sits at, when they all do within `tolerance`.
fn collapsed(curve: &NurbsCurve, tolerance: f64) -> Option<Vec3> {
    let point = |index: usize| {
        let pole = curve.control_points[index];
        Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)
    };
    let first = point(0);
    (1..curve.control_points.len()).all(|index| point(index).sub(first).length() <= tolerance).then_some(first)
}

/// Whether `a` and `b`, two opposite sides of one patch, are the same curve
/// point for point (they share knots and degree, so sample at equal parameters).
fn coincide(a: &NurbsCurve, b: &NurbsCurve, tolerance: f64) -> Result<bool, KernelRefusal> {
    const SAMPLES: usize = 33;
    let [t0, t1] = a.domain().or_refuse(KernelStage::Collect, "surfaceFace.side_domain")?;
    for index in 0..=SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / SAMPLES as f64;
        let gap = a.evaluate(t).or_refuse(KernelStage::Collect, "surfaceFace.side")?
            .sub(b.evaluate(t).or_refuse(KernelStage::Collect, "surfaceFace.side")?);
        if gap.length() > tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

fn uv_line(from: [f64; 2], to: [f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
    make_line(Vec3::new(from[0], from[1], 0.0), Vec3::new(to[0], to[1], 0.0)).or_refuse(KernelStage::Collect, "surfaceFace.pcurve")
}

fn accept(sheet: BrepSolid, what: &'static str) -> Result<BrepSolid, KernelRefusal> {
    let issues = sheet.validate();
    if let Some(issue) = issues.first() {
        return Err(refuse(KernelStage::Validate, what, format!("the face is not valid: {issue:?}")));
    }
    Ok(sheet)
}

/// A sheet of one face on `surface` over its whole domain, its normal the
/// surface's own (`∂S/∂u × ∂S/∂v`).
///
/// Each side of the domain is read from the surface: a side collapsed to a
/// point is a pole (a degenerate edge), two opposite sides that coincide are
/// one seam edge, and every other side is an open edge. So a clamped patch
/// gives a face with four open edges, a tube's side a face with a seam and two
/// open rims, and a ball or a ring a closed face, which [`solid_from_sheets`]
/// takes alone.
///
/// Refuses a surface that is a point or a curve (two adjacent sides
/// collapsed, or a side collapsed whose neighbours coincide all along).
pub fn face_from_surface(surface: &NurbsSurface) -> Result<BrepSolid, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Collect, "surfaceFace.domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Collect, "surfaceFace.domain")?;
    let size = extent(surface);
    if !(size.is_finite() && size > 0.0) {
        return Err(input("surface_extent", "the surface has no extent".into()));
    }
    let tolerance = 1e-9 * size;
    let side = |along_u: bool, at: f64| -> Result<NurbsCurve, KernelRefusal> {
        if along_u { surface.iso_curve_v(at) } else { surface.iso_curve_u(at) }.or_refuse(KernelStage::Collect, "surfaceFace.side")
    };
    // the four sides, each along its own rising parameter
    let (bottom, top) = (side(true, v0)?, side(true, v1)?);
    let (left, right) = (side(false, u0)?, side(false, u1)?);
    let poles = [collapsed(&bottom, tolerance), collapsed(&right, tolerance), collapsed(&top, tolerance), collapsed(&left, tolerance)];
    if (0..4).any(|index| poles[index].is_some() && poles[(index + 1) % 4].is_some()) {
        return Err(input("surface_collapsed", "two adjacent sides of the surface are single points".into()));
    }
    let seam_u = poles[1].is_none() && poles[3].is_none() && coincide(&left, &right, tolerance)?;
    let seam_v = poles[0].is_none() && poles[2].is_none() && coincide(&bottom, &top, tolerance)?;

    let mut records = Records::new(tolerance);
    let corner = |u: f64, v: f64| surface.evaluate(u, v).or_refuse(KernelStage::Collect, "surfaceFace.corner");
    let c00 = records.vertex(corner(u0, v0)?);
    let c10 = records.vertex(corner(u1, v0)?);
    let c11 = records.vertex(corner(u1, v1)?);
    let c01 = records.vertex(corner(u0, v1)?);
    let edge = |records: &mut Records, curve: NurbsCurve, pole: Option<Vec3>, start: u64, end: u64| -> Result<u64, KernelRefusal> {
        match pole {
            Some(point) => {
                let line = make_line(point, point).or_refuse(KernelStage::Collect, "surfaceFace.pole")?;
                records.edge(line, start, start, true)
            }
            None => records.edge(curve, start, end, false),
        }
    };
    let bottom_edge = edge(&mut records, bottom, poles[0], c00, c10)?;
    let left_edge = edge(&mut records, left, poles[3], c00, c01)?;
    let top_edge = if seam_v { bottom_edge } else { edge(&mut records, top, poles[2], c01, c11)? };
    let right_edge = if seam_u { left_edge } else { edge(&mut records, right, poles[1], c10, c11)? };
    // counter-clockwise in (u, v), so the face lies on the left of each coedge
    // as seen along the surface's normal
    let coedges = vec![
        (bottom_edge, true, uv_line([u0, v0], [u1, v0])?),
        (right_edge, true, uv_line([u1, v0], [u1, v1])?),
        (top_edge, false, uv_line([u1, v1], [u0, v1])?),
        (left_edge, false, uv_line([u0, v1], [u0, v0])?),
    ];
    accept(records.sheet(surface, vec![coedges]), "face")
}

/// The area a closed chain of pcurves encloses in the parameter plane, signed
/// (positive counter-clockwise), from a sampled polygon.
fn signed_area(pcurves: &[NurbsCurve]) -> Result<f64, KernelRefusal> {
    const SAMPLES: usize = 48;
    let mut points = Vec::with_capacity(pcurves.len() * SAMPLES);
    for pcurve in pcurves {
        let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Collect, "surfaceFace.pcurve_domain")?;
        for index in 0..SAMPLES {
            let at = pcurve.evaluate(t0 + (t1 - t0) * index as f64 / SAMPLES as f64).or_refuse(KernelStage::Collect, "surfaceFace.pcurve")?;
            points.push([at.x, at.y]);
        }
    }
    let mut twice = 0.0;
    for index in 0..points.len() {
        let (a, b) = (points[index], points[(index + 1) % points.len()]);
        twice += a[0] * b[1] - b[0] * a[1];
    }
    Ok(0.5 * twice)
}

/// A sheet of one face on `surface` bounded by `loops`: the first its outer
/// boundary, the rest holes. Each loop is a closed chain of curves lying on
/// the surface, head to tail in either sense (a single closed curve is a loop
/// of one); each curve becomes an edge and each joint a vertex. The face's
/// normal is the surface's own.
///
/// The curves' images in the surface's parameters are the kernel's
/// ([`crate::build_pcurve_on_surface`]: exact on a plane, fitted elsewhere).
/// The loops are turned the way a face needs them, the outer one
/// counter-clockwise about the normal and the holes clockwise.
///
/// Refuses an empty loop, a chain that does not close, and a curve that does
/// not lie on the surface.
pub fn face_from_surface_trimmed(surface: &NurbsSurface, loops: &[Vec<NurbsCurve>]) -> Result<BrepSolid, KernelRefusal> {
    if loops.is_empty() || loops.iter().any(Vec::is_empty) {
        return Err(input("trim_no_loop", "at least one loop, each of at least one curve, is required".into()));
    }
    let size = extent(surface);
    if !(size.is_finite() && size > 0.0) {
        return Err(input("surface_extent", "the surface has no extent".into()));
    }
    let tolerance = 1e-9 * size;
    let ends = |curve: &NurbsCurve| -> Result<[Vec3; 2], KernelRefusal> {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Collect, "surfaceFace.curve_domain")?;
        Ok([curve.evaluate(t0).or_refuse(KernelStage::Collect, "surfaceFace.curve")?, curve.evaluate(t1).or_refuse(KernelStage::Collect, "surfaceFace.curve")?])
    };
    let mut records = Records::new(tolerance);
    let mut face_loops = Vec::with_capacity(loops.len());
    for (loop_index, curves) in loops.iter().enumerate() {
        let mut coedges = Vec::with_capacity(curves.len());
        for (index, curve) in curves.iter().enumerate() {
            let [start, end] = ends(curve)?;
            let [next, _] = ends(&curves[(index + 1) % curves.len()])?;
            if end.sub(next).length() > tolerance {
                return Err(input("trim_open_loop", format!("loop {loop_index} does not close after curve {index} (gap {:.3e})", end.sub(next).length())));
            }
            let (from, to) = (records.vertex(start), records.vertex(end));
            let pcurve = build_pcurve_on_surface(surface, curve).map_err(|error| {
                refuse(KernelStage::Fragment, "trim_off_surface", format!("curve {index} of loop {loop_index} has no image on the surface: {error}"))
            })?;
            coedges.push((records.edge(curve.clone(), from, to, false)?, true, pcurve));
        }
        let pcurves: Vec<NurbsCurve> = coedges.iter().map(|(_, _, pcurve)| pcurve.clone()).collect();
        let area = signed_area(&pcurves)?;
        if area == 0.0 {
            return Err(input("trim_no_area", format!("loop {loop_index} encloses no area on the surface")));
        }
        // the outer loop runs counter-clockwise, a hole clockwise
        if (area > 0.0) != (loop_index == 0) {
            let mut turned = Vec::with_capacity(coedges.len());
            for (edge, _, pcurve) in coedges.into_iter().rev() {
                turned.push((edge, false, pcurve.reversed().or_refuse(KernelStage::Collect, "surfaceFace.pcurve_reverse")?));
            }
            coedges = turned;
        }
        face_loops.push(coedges);
    }
    accept(records.sheet(surface, face_loops), "trimmed_face")
}

/// The solid `sheets` bound: their faces joined along the edges that coincide
/// within `tolerance` (the kernel's sewing, [`crate::sew_solid`], which pairs
/// edges by where they lie and needs no agreement between the sheets'
/// normals), then turned outward.
///
/// One closed face is a solid by itself (a ball's or a ring's face from
/// [`face_from_surface`]). Refuses sheets that leave an edge open after
/// sewing, saying how many, and a joined body the kernel's validation rejects.
pub fn solid_from_sheets(sheets: &[BrepSolid], tolerance: f64) -> Result<BrepSolid, KernelRefusal> {
    if sheets.is_empty() || sheets.iter().any(|sheet| sheet.shells.iter().all(|shell| shell.faces.is_empty())) {
        return Err(input("sheets_empty", "at least one sheet, each of at least one face, is required".into()));
    }
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return Err(input("sheets_tolerance", "the sewing tolerance must be positive".into()));
    }
    // one body of all the faces, every id made new
    let mut next = 0u64;
    let mut id = || {
        next += 1;
        next
    };
    let (mut vertices, mut edges, mut shells) = (Vec::new(), Vec::new(), Vec::new());
    for sheet in sheets {
        let vertex_ids: std::collections::HashMap<u64, u64> = sheet.vertices.iter().map(|vertex| (vertex.id, id())).collect();
        let edge_ids: std::collections::HashMap<u64, u64> = sheet.edges.iter().map(|edge| (edge.id, id())).collect();
        let renamed = |ids: &std::collections::HashMap<u64, u64>, old: u64| {
            ids.get(&old).copied().ok_or_else(|| input("sheets_reference", format!("a sheet refers to the missing id {old}")))
        };
        for vertex in &sheet.vertices {
            vertices.push(VertexRecord { id: vertex_ids[&vertex.id], point: vertex.point });
        }
        for edge in &sheet.edges {
            let mut record = edge.clone();
            record.id = edge_ids[&edge.id];
            record.start_vertex_id = renamed(&vertex_ids, edge.start_vertex_id)?;
            record.end_vertex_id = renamed(&vertex_ids, edge.end_vertex_id)?;
            edges.push(record);
        }
        for shell in &sheet.shells {
            let mut faces = Vec::with_capacity(shell.faces.len());
            for face in &shell.faces {
                let mut record = face.clone();
                record.id = id();
                for loop_record in &mut record.loops {
                    loop_record.id = id();
                    for coedge in &mut loop_record.coedges {
                        coedge.id = id();
                        coedge.edge_id = renamed(&edge_ids, coedge.edge_id)?;
                    }
                }
                faces.push(record);
            }
            shells.push(ShellRecord { id: id(), faces });
        }
    }
    let joined = BrepSolid {
        mass_properties_cache: Default::default(),
        id: id(),
        vertices,
        edges,
        shells,
        genus: 0,
        kind: BodyKind::Sheet,
        cells: Default::default(),
    };
    let (mut solid, report) = sew_solid(&joined, tolerance)?;
    // a pole's degenerate edge bounds its one face by nature; every other edge must bound two
    let mut uses = std::collections::HashMap::<u64, usize>::new();
    for coedge in solid.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| &face.loops).flat_map(|record| &record.coedges) {
        *uses.entry(coedge.edge_id).or_default() += 1;
    }
    let open = solid.edges.iter().filter(|edge| !edge.degenerate && uses.get(&edge.id).copied().unwrap_or(0) < 2).count();
    if open > 0 {
        return Err(refuse(
            KernelStage::Sew,
            "sheets_open",
            format!("the sheets do not close: {open} edge(s) bound one face only after sewing ({} sewn)", report.edges_sewn),
        ));
    }
    solid.kind = BodyKind::Solid;
    // V − E + F − H = 2·(shells) − 2g, poles' degenerate edges not counted
    let live = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let faces = solid.shells.iter().map(|shell| shell.faces.len()).sum::<usize>() as i64;
    let rings = solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.loops.len().saturating_sub(1)).sum::<usize>() as i64;
    let twice = 2 * solid.shells.len() as i64 - (solid.vertices.len() as i64 - live + faces - rings);
    if twice < 0 || twice % 2 != 0 {
        return Err(refuse(KernelStage::Validate, "sheets_euler", format!("the joined faces' counts describe no closed surface (2g = {twice})")));
    }
    solid.genus = twice / 2;
    let volume = crate::solid_signed_volume(&solid).or_refuse(KernelStage::Validate, "surfaceFace.volume")?;
    if volume < 0.0 {
        crate::offset_shell::flip_all_faces(&mut solid)?;
    }
    let issues = solid.validate();
    if let Some(issue) = issues.first() {
        return Err(refuse(KernelStage::Validate, "sheets_invalid", format!("the joined faces are not a valid solid: {issue:?}")));
    }
    Ok(solid)
}

