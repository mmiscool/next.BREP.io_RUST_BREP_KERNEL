//! A corner where three filleted edges meet, set back: each fillet stops on
//! its section `setback` from the corner (measured along its edge) instead of
//! meeting the corner ball, and the corner is closed by one smooth patch over
//! the three sections and, on each face, the curve between the two fillets'
//! contact lines there.
//!
//! Built as surgery on the rolling-ball result (`fillet_edges`, whose corner is
//! a ball patch the three fillets end on): the fillets' contact lines (rails)
//! are cut back to their sections, each section is the fillet's own section
//! there (an iso-curve of its carrier), each face's curve is the quadratic
//! tangent to both of its rails (its middle control where their lines cross),
//! and the six sides are closed by the N-sided fill, tangent to the fillets and
//! the faces along them. Anything the surgery does not recognise is refused.
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, VertexRecord};
use crate::{KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Refine, what, format!("fillet_edges_setback: {message}"))
}

fn ends(edge: &EdgeRecord) -> Result<(Vec3, Vec3), KernelRefusal> {
    Ok((edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?, edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?))
}

fn is_line(edge: &EdgeRecord) -> bool {
    edge.curve.degree == 1 && edge.curve.control_points.len() == 2
}

fn is_plane(face: &FaceRecord) -> bool {
    face.surface.degree_u == 1 && face.surface.degree_v == 1 && face.surface.control_points.len() == 2 && face.surface.control_points[0].len() == 2
}

fn uv_of(face: &FaceRecord, point: Vec3) -> Result<[f64; 2], KernelRefusal> {
    let q = crate::project_point_to_surface(&face.surface, point).or_refuse(KernelStage::Refine, "project")?;
    if q.distance > 1e-7 * (1.0 + point.length()) {
        return Err(refuse("setback_off_face", format!("a point of the corner stands {:.2e} off its face", q.distance)));
    }
    Ok([q.u, q.v])
}

fn line2(a: [f64; 2], b: [f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
    crate::make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0)).or_refuse(KernelStage::Refine, "make_line")
}

/// Set back the corner at `corner` (the solid's vertex the three filleted
/// edges met at) of `body`, the rolling-ball fillet of those edges at `radius`.
pub(crate) fn set_back_corner(body: &BrepSolid, corner: Vec3, radius: f64, setback: f64, name: Option<&str>) -> Result<BrepSolid, KernelRefusal> {
    if !(setback > radius) {
        return Err(KernelRefusal::input(KernelStage::Collect, "setback_size", format!("fillet_edges_setback: the setback {setback} must exceed the radius {radius}")));
    }
    if body.shells.len() != 1 {
        return Err(refuse("setback_shells", "the body has more than one shell".into()));
    }
    let faces = &body.shells[0].faces;
    let edge_by = |id: u64| body.edges.iter().find(|e| e.id == id);
    let reach = 2.0 * radius;
    // A convex corner only: the rolling ball took the corner away, so the old
    // corner is outside the body (at a concave one it is filled in).
    let corner_class = crate::classify_point(corner, body, 1e-9 * (1.0 + corner.length())).or_refuse(KernelStage::Refine, "classify_point")?.class;
    if corner_class != crate::PointClass::Out {
        return Err(refuse("setback_concave", "the corner is not convex: only a convex corner is set back".into()));
    }
    // The corner ball: the face curved both ways (the fillets are curved one
    // way) whose every vertex lies within 2r of the corner.
    let vertex_points = |face: &FaceRecord| -> Vec<Vec3> {
        face.loops.iter().flat_map(|l| l.coedges.iter()).filter_map(|c| edge_by(c.edge_id)).flat_map(|e| [e.curve.evaluate(e.t0).ok(), e.curve.evaluate(e.t1).ok()]).flatten().collect()
    };
    let balls: Vec<&FaceRecord> = faces.iter().filter(|f| f.surface.degree_u >= 2 && f.surface.degree_v >= 2 && vertex_points(f).iter().all(|p| p.sub(corner).length() <= reach)).collect();
    if balls.len() != 1 {
        return Err(refuse("setback_corner", format!("the corner has {} ball patches, not one", balls.len())));
    }
    let ball = balls[0];
    if ball.loops.len() != 1 {
        return Err(refuse("setback_corner", "the corner ball has holes".into()));
    }
    // Its arcs, each shared with one fillet.
    let users = |edge_id: u64| -> Vec<(usize, usize, usize)> {
        let mut out = Vec::new();
        for (fi, f) in faces.iter().enumerate() {
            for (li, l) in f.loops.iter().enumerate() {
                for (ci, c) in l.coedges.iter().enumerate() {
                    if c.edge_id == edge_id {
                        out.push((fi, li, ci));
                    }
                }
            }
        }
        out
    };
    let mut arcs = Vec::new();
    let mut dropped_edges = Vec::new();
    for c in &ball.loops[0].coedges {
        let e = edge_by(c.edge_id).ok_or_else(|| refuse("setback_corner", "a ball edge is missing".into()))?;
        let (a, b) = ends(e)?;
        dropped_edges.push(e.id);
        if a.sub(b).length() <= 1e-9 * (1.0 + radius) && e.curve.evaluate(0.5 * (e.t0 + e.t1)).is_ok_and(|m| m.sub(a).length() <= 1e-9 * (1.0 + radius)) {
            continue; // a pole
        }
        let others: Vec<_> = users(e.id).into_iter().filter(|u| faces[u.0].id != ball.id).collect();
        if others.len() != 1 {
            return Err(refuse("setback_corner", "a ball arc is not shared with one fillet".into()));
        }
        arcs.push((e.id, others[0]));
    }
    if arcs.len() != 3 {
        return Err(refuse("setback_corner", format!("the corner ball meets {} fillets, not three", arcs.len())));
    }

    let mut result = body.clone();
    let mut next = result.vertices.iter().map(|v| v.id).chain(result.edges.iter().map(|x| x.id)).chain(result.shells.iter().flat_map(|s| s.faces.iter()).flat_map(|f| std::iter::once(f.id).chain(f.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id)))))).max().unwrap_or(0) + 1;
    let mut id = || {
        next += 1;
        next
    };
    // Per fillet: its face, its section's ends (on its two rails), the section edge.
    struct Fillet {
        face: usize,
        cut: [Vec3; 2], // the section's ends on its two rails, in the fillet's loop order
        section: u64,
    }
    let mut fillets: Vec<Fillet> = Vec::new();
    // Rails cut back: rail id -> (the end at the ball replaced: old vertex, new vertex, new point)
    let mut cut_rails: Vec<(u64, u64, u64, Vec3)> = Vec::new();
    for &(arc_id, (fi, li, ci)) in &arcs {
        let ring = &faces[fi].loops[li];
        let n = ring.coedges.len();
        let (before, after) = (&ring.coedges[(ci + n - 1) % n], &ring.coedges[(ci + 1) % n]);
        let (rb, ra) = (edge_by(before.edge_id).ok_or_else(|| refuse("setback_corner", "a rail is missing".into()))?, edge_by(after.edge_id).ok_or_else(|| refuse("setback_corner", "a rail is missing".into()))?);
        if !is_line(rb) || !is_line(ra) {
            return Err(refuse("setback_rails", "a fillet's contact lines are not straight".into()));
        }
        let arc = edge_by(arc_id).expect("arc");
        let arc_coedge = &ring.coedges[ci];
        let (arc_from, arc_to) = if arc_coedge.forward { (arc.start_vertex_id, arc.end_vertex_id) } else { (arc.end_vertex_id, arc.start_vertex_id) };
        let mut cut = [Vec3::new(0.0, 0.0, 0.0); 2];
        let mut cut_vertex = [0u64; 2];
        for (slot, (rail, at_ball)) in [(rb, arc_from), (ra, arc_to)].into_iter().enumerate() {
            let (p, q) = ends(rail)?;
            let (far, near) = if rail.start_vertex_id == at_ball { (q, p) } else { (p, q) };
            let toward = near.sub(far).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            // The station `setback` back from the corner along the edge.
            let lambda = (corner.sub(far).dot(toward) - setback) / 1.0;
            let x = far.add(toward.scale(lambda));
            if !(lambda > 1e-9 && lambda < near.sub(far).dot(toward) - 1e-9) {
                return Err(refuse("setback_reach", format!("the setback {setback} runs past a fillet's contact line")));
            }
            let reused = cut_rails.iter().find(|r| r.0 == rail.id).map(|r| r.2);
            let v = match reused {
                Some(v) => v,
                None => {
                    let v = id();
                    result.vertices.push(VertexRecord { id: v, point: x });
                    cut_rails.push((rail.id, at_ball, v, x));
                    v
                }
            };
            cut[slot] = x;
            cut_vertex[slot] = v;
        }
        // The section: the carrier's iso-curve through both cut points.
        let face = &faces[fi];
        let (p0, p1) = (uv_of(face, cut[0])?, uv_of(face, cut[1])?);
        let (curve, along_u) = if (p0[1] - p1[1]).abs() <= 1e-9 * (1.0 + p0[1].abs()) {
            (face.surface.iso_curve_v(p0[1]).or_refuse(KernelStage::Refine, "iso_curve_v")?, true)
        } else if (p0[0] - p1[0]).abs() <= 1e-9 * (1.0 + p0[0].abs()) {
            (face.surface.iso_curve_u(p0[0]).or_refuse(KernelStage::Refine, "iso_curve_u")?, false)
        } else {
            return Err(refuse("setback_section", "a fillet's section is not one of its carrier's iso-curves".into()));
        };
        let (s0, s1) = if along_u { (p0[0], p1[0]) } else { (p0[1], p1[1]) };
        let (lo, hi) = (s0.min(s1), s0.max(s1));
        let mut piece = curve;
        let [d0, d1] = piece.domain().or_refuse(KernelStage::Refine, "domain")?;
        if hi < d1 - 1e-12 {
            piece = piece.split(hi).or_refuse(KernelStage::Refine, "split")?.0;
        }
        if lo > d0 + 1e-12 {
            piece = piece.split(lo).or_refuse(KernelStage::Refine, "split")?.1;
        }
        // Stored from cut[0] to cut[1] (the fillet's loop walks it that way, forward).
        let piece = if s0 > s1 { piece.reversed().or_refuse(KernelStage::Refine, "reversed")? } else { piece };
        let [e0, e1] = piece.domain().or_refuse(KernelStage::Refine, "domain")?;
        if piece.evaluate(e0).or_refuse(KernelStage::Refine, "evaluate")?.sub(cut[0]).length() > 1e-7 * (1.0 + radius) || piece.evaluate(e1).or_refuse(KernelStage::Refine, "evaluate")?.sub(cut[1]).length() > 1e-7 * (1.0 + radius) {
            return Err(refuse("setback_section", "a fillet's section does not run between its rails' cuts".into()));
        }
        let section = id();
        result.edges.push(EdgeRecord { id: section, curve: piece, t0: e0, t1: e1, start_vertex_id: cut_vertex[0], end_vertex_id: cut_vertex[1], degenerate: false, name: None });
        fillets.push(Fillet { face: fi, cut, section });
    }
    if cut_rails.len() != 6 {
        return Err(refuse("setback_rails", format!("the three fillets have {} contact lines at the corner, not six", cut_rails.len())));
    }
    // The rails' new ends.
    for &(rail, old, v, x) in &cut_rails {
        let edge = result.edges.iter_mut().find(|e| e.id == rail).expect("rail");
        let (p, q) = (edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Refine, "evaluate")?, edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Refine, "evaluate")?);
        let (from, to) = if edge.start_vertex_id == old {
            edge.start_vertex_id = v;
            (x, q)
        } else {
            edge.end_vertex_id = v;
            (p, x)
        };
        edge.curve = crate::make_line(from, to).or_refuse(KernelStage::Refine, "make_line")?;
        edge.t0 = 0.0;
        edge.t1 = 1.0;
    }
    // Each planar face at the corner: its two rails meet at the ball's contact;
    // the curve between their cuts, tangent to both (middle control where their lines cross).
    let mut face_curves: Vec<(usize, u64)> = Vec::new(); // (face index, edge)
    let rail_ids: Vec<u64> = cut_rails.iter().map(|r| r.0).collect();
    for (fi, face) in faces.iter().enumerate() {
        if !is_plane(face) {
            continue;
        }
        let ring = &face.loops[0];
        let n = ring.coedges.len();
        for ci in 0..n {
            let (c, d) = (&ring.coedges[ci], &ring.coedges[(ci + 1) % n]);
            if !(rail_ids.contains(&c.edge_id) && rail_ids.contains(&d.edge_id)) {
                continue;
            }
            let (rc, rd) = (cut_rails.iter().find(|r| r.0 == c.edge_id).expect("rail"), cut_rails.iter().find(|r| r.0 == d.edge_id).expect("rail"));
            let (from, to) = (rc.3, rd.3);
            // The rails' lines cross at the old contact.
            let contact = body.vertices.iter().find(|v| v.id == rc.1).map(|v| v.point).ok_or_else(|| refuse("setback_corner", "a contact vertex is missing".into()))?;
            let quad = NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![crate::Vec4::from_point(from, 1.0), crate::Vec4::from_point(contact, 1.0), crate::Vec4::from_point(to, 1.0)]).or_refuse(KernelStage::Refine, "face_curve")?;
            let e = id();
            result.edges.push(EdgeRecord { id: e, curve: quad, t0: 0.0, t1: 1.0, start_vertex_id: rc.2, end_vertex_id: rd.2, degenerate: false, name: None });
            face_curves.push((fi, e));
        }
    }
    if face_curves.len() != 3 {
        return Err(refuse("setback_faces", format!("{} faces at the corner carry two fillets' contact lines, not three", face_curves.len())));
    }
    // Re-read the loops: rails' pcurves, the sections in the fillets, the curves in the faces.
    let snapshot = result.clone();
    let edge_now = |id: u64| snapshot.edges.iter().find(|e| e.id == id).cloned();
    for fi in 0..result.shells[0].faces.len() {
        let face = snapshot.shells[0].faces[fi].clone();
        if face.id == ball.id {
            continue;
        }
        for li in 0..face.loops.len() {
            let ring = &face.loops[li];
            let n = ring.coedges.len();
            let mut coedges = Vec::with_capacity(n + 1);
            for ci in 0..n {
                let c = &ring.coedges[ci];
                if arcs.iter().any(|a| a.0 == c.edge_id) {
                    let fillet = fillets.iter().find(|f| f.face == fi).expect("fillet");
                    let section = edge_now(fillet.section).expect("section");
                    let (p0, p1) = (uv_of(&face, fillet.cut[0])?, uv_of(&face, fillet.cut[1])?);
                    let _ = section;
                    coedges.push(CoedgeRecord { id: id(), edge_id: fillet.section, forward: true, pcurve: line2(p0, p1)? });
                    continue;
                }
                if rail_ids.contains(&c.edge_id) {
                    let e = edge_now(c.edge_id).expect("rail");
                    let (p, q) = ends(&e)?;
                    let (a, b) = if c.forward { (p, q) } else { (q, p) };
                    coedges.push(CoedgeRecord { id: c.id, edge_id: c.edge_id, forward: c.forward, pcurve: line2(uv_of(&face, a)?, uv_of(&face, b)?)? });
                    if let Some(&(_, curve)) = face_curves.iter().find(|fc| fc.0 == fi) {
                        let d = &ring.coedges[(ci + 1) % n];
                        if rail_ids.contains(&d.edge_id) {
                            let e = edge_now(curve).expect("curve");
                            let mut controls = Vec::new();
                            for k in &e.curve.control_points {
                                let uv = uv_of(&face, k.point().or_refuse(KernelStage::Refine, "point")?)?;
                                controls.push(crate::Vec4 { x: uv[0] * k.w, y: uv[1] * k.w, z: 0.0, w: k.w });
                            }
                            let pcurve = NurbsCurve::new(2, e.curve.knots.clone(), controls).or_refuse(KernelStage::Refine, "pcurve")?;
                            coedges.push(CoedgeRecord { id: id(), edge_id: curve, forward: true, pcurve });
                        }
                    }
                    continue;
                }
                coedges.push(c.clone());
            }
            result.shells[0].faces[fi].loops[li].coedges = coedges;
        }
    }
    // Drop the ball, its arcs and pole, and the vertices nothing uses now.
    result.shells[0].faces.retain(|f| f.id != ball.id);
    result.edges.retain(|e| !dropped_edges.contains(&e.id));
    let used: Vec<u64> = result.edges.iter().flat_map(|e| [e.start_vertex_id, e.end_vertex_id]).collect();
    result.vertices.retain(|v| used.contains(&v.id));
    // The six sides, each with the face it meets and its pcurve there.
    // (by face id: removing the ball moved the indices)
    let mut sides: Vec<(u64, u64)> = fillets.iter().map(|f| (f.section, faces[f.face].id)).collect();
    sides.extend(face_curves.iter().map(|&(fi, e)| (e, faces[fi].id)));
    let mut ribs = Vec::with_capacity(6);
    for &(edge_id, face_id) in &sides {
        let face = result.shells[0].faces.iter().find(|f| f.id == face_id).ok_or_else(|| refuse("setback_corner", "a side's face is missing".into()))?;
        let coedge = face.loops.iter().flat_map(|l| l.coedges.iter()).find(|c| c.edge_id == edge_id).ok_or_else(|| refuse("setback_corner", "a side is not in its face's loop".into()))?;
        let edge = result.edges.iter().find(|e| e.id == edge_id).expect("side").clone();
        ribs.push(super::corner::RibbonRib { edge, surface: &face.surface, pcurve: coedge.pcurve.clone() });
    }
    // The patch faces the old corner (outside the result whatever the
    // setback; the ball's centre need not be inside once the corner is set back far).
    let star = super::corner::build_n_sided_star_with(&ribs, corner, false, name, &mut id, true)?;
    drop(ribs);
    result.vertices.extend(star.vertices);
    result.edges.extend(star.edges);
    for (rib_edge, first_half, second_half, (first_pcurve, second_pcurve)) in star.splits {
        let mut replaced = false;
        for face in result.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
            for lp in &mut face.loops {
                if let Some(at) = lp.coedges.iter().position(|c| c.edge_id == rib_edge) {
                    let second = CoedgeRecord { id: id(), edge_id: second_half, forward: true, pcurve: second_pcurve.clone() };
                    lp.coedges[at] = CoedgeRecord { id: id(), edge_id: first_half, forward: true, pcurve: first_pcurve.clone() };
                    lp.coedges.insert(at + 1, second);
                    replaced = true;
                }
            }
        }
        if !replaced {
            return Err(refuse("setback_corner", "no face holds a side the fill split".into()));
        }
        result.edges.retain(|edge| edge.id != rib_edge);
    }
    result.shells[0].faces.extend(star.faces);
    result.mass_properties_cache = Default::default();
    let problems: Vec<String> = result.validate().into_iter().filter(|i| i.severity == "error").map(|i| i.message).take(3).collect();
    if !problems.is_empty() {
        return Err(refuse("setback_result", format!("the set-back corner does not validate: {}", problems.join("; "))));
    }
    Ok(result)
}
