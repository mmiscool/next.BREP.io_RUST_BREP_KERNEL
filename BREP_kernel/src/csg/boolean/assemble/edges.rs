//! Bounded reconstruction from stored carrier geometry and planar trim images.
//! Exact analytic, isoparametric and rational ruled-plane sections supplement
//! planar images when the inherited image itself misses the paired carrier.
//! Candidates must reach settled vertices, stay on both carriers, and pass
//! movement and nonincident face clearance checks. No station fit is used.

use crate::{BrepSolid, KernelRefusal, KernelStage, KernelTolerances, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};
use rustc_hash::FxHashMap as HashMap;

/// An edge this pass replaced, with the curve it replaced.
pub(in crate::boolean) struct ReplacedEdge {
    pub edge_id: u64,
    pub previous: NurbsCurve,
    pub t0: f64,
    pub t1: f64,
    /// Worst sampled distance to either carrier, before and after.
    pub residual_before: f64,
    pub residual_after: f64,
}

/// Put back every edge in `replaced`.
pub(in crate::boolean) fn restore_edges(solid: &mut BrepSolid, replaced: Vec<ReplacedEdge>) {
    for item in replaced {
        if let Some(edge) = solid.edges.iter_mut().find(|edge| edge.id == item.edge_id) {
            edge.curve = item.previous;
            edge.t0 = item.t0;
            edge.t1 = item.t1;
        }
    }
}

const STATIONS: usize = 33;

/// Worst distance from `curve` over `[t0, t1]` to the carriers in `surfaces`.
fn worst_residual(curve: &NurbsCurve, t0: f64, t1: f64, surfaces: &[&NurbsSurface]) -> Result<f64, KernelRefusal> {
    let mut worst = 0.0f64;
    for t in carrier_stations(curve, t0, t1) {
        let point = curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?;
        for surface in surfaces {
            let projection = crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Sew, "project_point_to_surface")?;
            worst = worst.max(projection.distance);
        }
    }
    Ok(worst)
}

/// Uniform stations alone alias inherited polygon vertices. Include each
/// represented knot span's quarters in both entry and candidate checks.
pub(super) fn carrier_stations(curve: &NurbsCurve, t0: f64, t1: f64) -> Vec<f64> {
    let mut breaks = vec![t0, t1];
    breaks.extend(curve.knots.iter().copied().filter(|&t| t > t0 && t < t1));
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let mut stations: Vec<_> = (0..STATIONS)
        .map(|i| t0 + (t1 - t0) * i as f64 / (STATIONS - 1) as f64).collect();
    for span in breaks.windows(2) {
        for i in 0..=4 { stations.push(span[0] + (span[1] - span[0]) * i as f64 / 4.0); }
    }
    stations.sort_by(f64::total_cmp);
    stations.dedup();
    stations
}

/// Restrict the nearest-point witness to the represented edge interval.
/// A parent curve can contain the complementary branch of a section; using
/// that parent as the displacement witness would make a branch swap read zero.
pub(super) fn represented_curve(edge: &crate::topology::EdgeRecord) -> Option<NurbsCurve> {
    super::image::restricted_curve(&edge.curve,edge.t0,edge.t1)
}

/// Exact carrier constructions, without station interpolation. A constant
/// trim coordinate supports a represented isocurve; recognized analytic
/// surface pairs supply their existing exact sections. Unsupported pairs stay
/// unchanged. Restriction must reach both settled vertices: never bend these
/// curves by pinning their controls.
fn exact_carrier_candidates(
    surfaces: [&NurbsSurface; 2],
    uses: &[(usize, NurbsCurve, bool)],
    faces: &[&crate::FaceRecord],
    start: Vec3,
    end: Vec3,
    bar: f64,
) -> Vec<NurbsCurve> {
    let mut full = crate::intersect_analytic_pair(surfaces[0], surfaces[1], bar).unwrap_or_default();
    if let Ok(Some(curve))=crate::imprint::planar_carrier_section(surfaces[0],surfaces[1],bar) {full.push(curve);}
    for (plane,ruled) in [(surfaces[0],surfaces[1]),(surfaces[1],surfaces[0])] {
        if let Some(pair)=crate::imprint::plane_section_graph(plane,ruled).filter(|pair|plane_supports_curve(plane,&pair.curve,bar*0.1)) {
            // The algebraic construction also supplies the carrier chart.
            // Check its endpoint images before handing the 3D branch to the
            // existing restriction, displacement and clearance guards.
            if let Ok([a,b])=pair.pcurve.domain() {
                if [a,b].into_iter().all(|t| pair.pcurve.evaluate(t).ok().and_then(|uv|ruled.evaluate(uv.x,uv.y).ok())
                    .zip(pair.curve.evaluate(t).ok()).is_some_and(|(p,q)|p.sub(q).length()<=bar*0.1)) {full.push(pair.curve);}
            }
        }
        if let (Ok(a),Ok(b))=(crate::project_point_to_surface(ruled,start),crate::project_point_to_surface(ruled,end)) {
            if let Some(pair)=crate::imprint::plane_section_graph_range(plane,ruled,[a.u.min(b.u),a.u.max(b.u)]).filter(|pair|plane_supports_curve(plane,&pair.curve,bar*0.1)) {full.push(pair.curve);}
        }
    }
    for (face_index, trim, _) in uses {
        let surface = &faces[*face_index].surface;
        if trim.control_points.is_empty() || trim.control_points.iter().any(|p| !(p.w > 0.0)) { continue; }
        let first = trim.control_points[0];
        let u = first.x / first.w;
        let v = first.y / first.w;
        // Exact constant coordinates only. A UV-near trim is not proof of
        // physical boundary support on a large or reparameterized carrier.
        if trim.control_points.iter().all(|p| p.x / p.w == u) {
            if let Ok(curve) = surface.iso_curve_u(u) { full.push(curve); }
        }
        if trim.control_points.iter().all(|p| p.y / p.w == v) {
            if let Ok(curve) = surface.iso_curve_v(v) { full.push(curve); }
        }
    }
    full.into_iter().filter_map(|curve| {
        let a = crate::project_point_to_curve(&curve, start).ok()?;
        let b = crate::project_point_to_curve(&curve, end).ok()?;
        if a.distance > bar || b.distance > bar { return None; }
        if start.sub(end).length() <= bar {
            let [d0,d1] = curve.domain().ok()?;
            if curve.evaluate(d0).ok()?.sub(start).length() <= bar && curve.evaluate(d1).ok()?.sub(end).length() <= bar {
                return Some(curve);
            }
            return None;
        }
        if (a.u - b.u).abs() < 1e-12 { return None; }
        let [d0, d1] = curve.domain().ok()?;
        let lo = a.u.min(b.u);
        let hi = a.u.max(b.u);
        let mut part = curve;
        if hi < d1 { part = part.split(hi).ok()?.0; }
        if lo > d0 { part = part.split(lo).ok()?.1; }
        if a.u > b.u { part = part.reversed().ok()?; }
        Some(part)
    }).collect()
}

/// The exact rational image through a supported bilinear patch. Approximate
/// affine recognition is only a scope gate: it never removes a warp term.
pub(super) fn affine_image(surface: &NurbsSurface, pcurve: &NurbsCurve) -> Result<Option<NurbsCurve>, KernelRefusal> {
    if !surface.is_affine().or_refuse(KernelStage::Sew,"is_affine")? {return Ok(None);}
    Ok(super::image::bilinear_image(surface,pcurve))
}

/// A linear chart is a proposal, not proof of an approximately affine
/// surface. Compose it through the complete bilinear net and bound its
/// same-parameter difference from the proposed section over every span.
pub(super) fn plane_supports_curve(surface:&NurbsSurface,curve:&NurbsCurve,bar:f64)->bool {
    if !surface.is_affine().unwrap_or(false) {return false;}
    let Ok([a,b])=curve.domain() else {return false;};
    let Ok(fit)=crate::fit_pcurve_on_surface_range(surface,curve,a,b,true,bar) else {return false;};
    // Range fitting uses a normalized trim parameter. Restore the known
    // affine correspondence before a same-parameter whole-image comparison.
    let Ok([x,y])=fit.curve.domain() else {return false;};
    let mut knots=fit.curve.knots.clone();
    for t in &mut knots { *t=if *t==x {a} else if *t==y {b} else {a+(b-a)*(*t-x)/(y-x)}; }
    let Ok(pc)=NurbsCurve::new(fit.curve.degree,knots,fit.curve.control_points) else {return false;};
    let Some(image)=super::image::bilinear_image(surface,&pc) else {return false;};
    super::image::difference_bound(curve,&image).is_some_and(|d|d<=bar)
}

/// Move a curve's two end control points onto `start` and `end`.
fn pin_ends(curve: &NurbsCurve, start: Vec3, end: Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let mut controls = curve.control_points.clone();
    let last = controls.len() - 1;
    controls[0] = Vec4::from_point(start, controls[0].w);
    controls[last] = Vec4::from_point(end, controls[last].w);
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")
}

/// A one-span rational quadratic — the form every circular, elliptic and
/// other conic arc trim takes — read as the CONIC it lies on, so an arc can
/// be re-cut to new ends without leaving its circle. Moving one end control
/// point of such an arc (`pin_ends`) bends it (3.07e-5 off the offset ball
/// for `BadBoolean`'s arc 40, record §9); re-deriving the Bézier form on the
/// same conic from the new ends does not.
pub(super) struct Conic {
    h: [Vec4; 3],
    k0: f64,
    k1: f64,
}

impl Conic {
    pub(super) fn of(curve: &NurbsCurve) -> Option<Conic> {
        if curve.degree != 2 || curve.control_points.len() != 3 || curve.knots.len() != 6 {
            return None;
        }
        let k = &curve.knots;
        if (k[0] - k[2]).abs() > 0.0 || (k[3] - k[5]).abs() > 0.0 || !(k[3] > k[0]) {
            return None;
        }
        Some(Conic { h: [curve.control_points[0], curve.control_points[1], curve.control_points[2]], k0: k[0], k1: k[3] })
    }

    /// Homogeneous point and its derivative at Bézier parameter `u`, for ANY
    /// real `u` (the conic continues past the arc's ends).
    fn homogeneous(&self, u: f64) -> (Vec4, Vec4) {
        let b = [(1.0 - u) * (1.0 - u), 2.0 * u * (1.0 - u), u * u];
        let db = [-2.0 * (1.0 - u), 2.0 - 4.0 * u, 2.0 * u];
        let mut point = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
        let mut derivative = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
        for index in 0..3 {
            let h = self.h[index];
            point.x += b[index] * h.x;
            point.y += b[index] * h.y;
            point.z += b[index] * h.z;
            point.w += b[index] * h.w;
            derivative.x += db[index] * h.x;
            derivative.y += db[index] * h.y;
            derivative.z += db[index] * h.z;
            derivative.w += db[index] * h.w;
        }
        (point, derivative)
    }

    fn point(&self, u: f64) -> Option<Vec3> {
        let (p, _) = self.homogeneous(u);
        (p.w.abs() > 1e-300).then(|| Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w))
    }

    fn tangent(&self, u: f64) -> Option<Vec3> {
        let (p, d) = self.homogeneous(u);
        if p.w.abs() <= 1e-300 {
            return None;
        }
        let w2 = p.w * p.w;
        Some(Vec3::new((d.x * p.w - p.x * d.w) / w2, (d.y * p.w - p.y * d.w) / w2, (d.z * p.w - p.z * d.w) / w2))
    }

    /// The parameter of the conic point nearest `target`, from `seed`.
    pub(super) fn nearest(&self, target: Vec3, seed: f64) -> Option<f64> {
        let mut u = seed;
        for _ in 0..50 {
            let point = self.point(u)?;
            let tangent = self.tangent(u)?;
            let speed2 = tangent.dot(tangent);
            if !(speed2 > 1e-300) {
                return None;
            }
            let step = point.sub(target).dot(tangent) / speed2;
            u -= step;
            if !u.is_finite() || u.abs() > 1e3 {
                return None;
            }
            if step.abs() < 1e-16 {
                break;
            }
        }
        Some(u)
    }

    /// The same conic as a one-span rational quadratic from `ua` to `ub`,
    /// with the old parameter values as its knots.
    pub(super) fn arc(&self, ua: f64, ub: f64) -> Option<NurbsCurve> {
        if !(ub > ua) {
            return None;
        }
        let a = self.point(ua)?;
        let b = self.point(ub)?;
        let ta = self.tangent(ua)?;
        let tb = self.tangent(ub)?;
        // Middle control point: where the end tangents meet (least squares
        // in 3D; the conic is planar so the lines meet to rounding).
        let d = a.sub(b);
        let aa = ta.dot(ta);
        let bb = tb.dot(tb);
        let ab = ta.dot(tb);
        let det = aa * bb - ab * ab;
        if !(det.abs() > 1e-14 * aa * bb) {
            return None;
        }
        let da = d.dot(ta);
        let db = d.dot(tb);
        // a + s ta = b + r tb  ->  [aa -ab; ab -bb] [s r]^T = [-da; -db]
        let s = (-da * bb + ab * db) / det;
        let m = a.add(ta.scale(s));
        // Weight from a third conic point: its barycentric position in the
        // triangle A M B fixes both the Bézier parameter and the weight.
        let q = self.point(0.5 * (ua + ub))?;
        let e1 = m.sub(a);
        let e2 = b.sub(a);
        let n = e1.cross(e2);
        let n2 = n.dot(n);
        if !(n2 > 1e-300) {
            return None;
        }
        let qa = q.sub(a);
        let beta = qa.cross(e2).dot(n) / n2; // coordinate on M
        let gamma = e1.cross(qa).dot(n) / n2; // coordinate on B
        let alpha = 1.0 - beta - gamma;
        if !(alpha > 0.0 && beta > 0.0 && gamma > 0.0) {
            return None;
        }
        let ratio = (gamma / alpha).sqrt();
        let u = ratio / (1.0 + ratio);
        let w = beta * (1.0 - u) / (2.0 * u * alpha);
        if !(w.is_finite() && w > 0.0) {
            return None;
        }
        let ka = self.k0 + ua * (self.k1 - self.k0);
        let kb = self.k0 + ub * (self.k1 - self.k0);
        NurbsCurve::new(2, vec![ka, ka, ka, kb, kb, kb], vec![Vec4::from_point(a, 1.0), Vec4::from_point(m, w), Vec4::from_point(b, 1.0)]).ok()
    }
}

/// The candidate for one planar use: the trim's image re-cut to the edge's
/// vertices — on its own conic for a rational arc, by restriction for any
/// other curve whose image reaches both vertices — or, failing both, the
/// image with its ends pinned (which a later bound then judges).
fn candidate_from_image(image: &NurbsCurve, start: Vec3, end: Vec3, bar: f64) -> Result<(NurbsCurve, &'static str), KernelRefusal> {
    if let Some(conic) = Conic::of(image) {
        let (Some(ua), Some(ub)) = (conic.nearest(start, 0.0), conic.nearest(end, 1.0)) else {
            return Ok((pin_ends(image, start, end)?, "pinned"));
        };
        let on_conic = conic.point(ua).map_or(f64::INFINITY, |p| p.sub(start).length()).max(conic.point(ub).map_or(f64::INFINITY, |p| p.sub(end).length()));
        if on_conic <= bar {
            if let Some(arc) = conic.arc(ua, ub) {
                return Ok((pin_ends(&arc, start, end)?, "conic"));
            }
        }
        return Ok((pin_ends(image, start, end)?, "pinned"));
    }
    let [d0, d1] = image.domain().or_refuse(KernelStage::Sew, "domain")?;
    let epsilon = ((d1 - d0).abs().max(1.0) * 1e-10).max(2e-9);
    let pa = crate::project_point_to_curve(image, start).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
    let pb = crate::project_point_to_curve(image, end).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
    if pa.distance <= bar && pb.distance <= bar && pb.u > pa.u + 2.0 * epsilon {
        let mut restricted = image.clone();
        if pb.u < d1 - epsilon {
            restricted = restricted.split(pb.u).or_refuse(KernelStage::Sew, "split")?.0;
        }
        if pa.u > d0 + epsilon {
            restricted = restricted.split(pa.u).or_refuse(KernelStage::Sew, "split")?.1;
        }
        return Ok((pin_ends(&restricted, start, end)?, "restricted"));
    }
    Ok((pin_ends(image, start, end)?, "pinned"))
}

/// Reconstruct every two-use edge of `solid` that stands off one of its
/// carriers by more than the joint bar, from a planar use's trim. Returns the
/// edges replaced, with their previous curves.
pub(in crate::boolean) fn reconstruct_edges_from_carriers(solid: &mut BrepSolid, tolerance: f64) -> Result<Vec<ReplacedEdge>, KernelRefusal> {
    let policy = KernelTolerances::for_solid(solid, 1e-7);
    let bar = policy.model.min(tolerance.max(1e-12));
    let carrier_band = policy.pcurve_consistency;
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok() || std::env::var("BREP_DEBUG_JOINTS").is_ok();
    if std::env::var("BREP_RECONSTRUCT_EDGES").as_deref() == Ok("0") {
        // Diagnostic A/B hatch, not a policy (the record's before column).
        return Ok(Vec::new());
    }

    let faces: Vec<&crate::FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    // edge id -> (face index, pcurve, forward) for every use.
    let mut uses: HashMap<u64, Vec<(usize, NurbsCurve, bool)>> = HashMap::default();
    for (face_index, face) in faces.iter().enumerate() {
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                uses.entry(coedge.edge_id).or_default().push((face_index, coedge.pcurve.clone(), coedge.forward));
            }
        }
    }
    let points: HashMap<u64, Vec3> = solid.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();

    let mut candidates = Vec::<(usize, NurbsCurve, f64, f64)>::new();
    let mut declined = Vec::<String>::new();
    let mut off_edges = 0usize;
    for (edge_index, edge) in solid.edges.iter().enumerate() {
        if edge.degenerate || !(edge.t1 > edge.t0) {
            continue;
        }
        let Some(edge_uses) = uses.get(&edge.id) else { continue };
        if edge_uses.len() != 2 || edge_uses[0].0 == edge_uses[1].0 {
            continue;
        }
        let surfaces = [&faces[edge_uses[0].0].surface, &faces[edge_uses[1].0].surface];
        let before = worst_residual(&edge.curve, edge.t0, edge.t1, &surfaces)?;
        if before <= bar {
            continue;
        }
        off_edges += 1;
        if before > carrier_band {
            declined.push(format!("edge {} stands {before:.3e} off a carrier, over the {carrier_band:.3e} contract", edge.id));
            continue;
        }
        let (Some(&start), Some(&end)) = (points.get(&edge.start_vertex_id), points.get(&edge.end_vertex_id)) else { continue };
        let mut best: Option<(NurbsCurve, f64, String)> = None;
        let mut planar_uses = 0usize;
        for (face_index, pcurve, forward) in edge_uses {
            let Some(image) = affine_image(&faces[*face_index].surface, pcurve)? else { continue };
            planar_uses += 1;
            let image = if *forward { image } else { image.reversed().or_refuse(KernelStage::Sew, "reversed")? };
            let [i0, i1] = image.domain().or_refuse(KernelStage::Sew, "domain")?;
            let image_start = image.evaluate(i0).or_refuse(KernelStage::Sew, "evaluate")?;
            let image_end = image.evaluate(i1).or_refuse(KernelStage::Sew, "evaluate")?;
            let end_miss = image_start.sub(start).length().max(image_end.sub(end).length());
            if end_miss > carrier_band {
                declined.push(format!("edge {}: face {}'s trim image ends {end_miss:.3e} from the edge's vertices", edge.id, faces[*face_index].id));
                continue;
            }
            let (pinned, how) = candidate_from_image(&image, start, end, bar)?;
            let [i0, i1] = pinned.domain().or_refuse(KernelStage::Sew, "domain")?;
            let after = worst_residual(&pinned, i0, i1, &surfaces)?;
            // Displacement from the old edge: the distance from each sample
            // of the candidate to the old curve. Geometric, not at equal
            // parameter fractions — a re-cut conic carries the trim's
            // parameterisation, not the old edge's, and equal fractions read
            // that as a 0.29 move on an arc that lies on both carriers to 5e-12.
            let mut moved = 0.0f64;
            for step in 0..STATIONS {
                let fraction = step as f64 / (STATIONS - 1) as f64;
                let new_point = pinned.evaluate(i0 + (i1 - i0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
                let nearest = crate::project_point_to_curve(&edge.curve, new_point).or_refuse(KernelStage::Sew, "project_point_to_curve")?;
                moved = moved.max(nearest.distance);
            }
            let note = format!("face {} trim image ({how}): on carriers to {after:.3e}, ends from {end_miss:.3e}, moved {moved:.3e}", faces[*face_index].id);
            if after > bar || moved > carrier_band {
                declined.push(format!("edge {} ({before:.3e} off): {note}", edge.id));
                continue;
            }
            if best.as_ref().map_or(true, |(_, best_after, _)| after < *best_after) {
                best = Some((pinned, after, note));
            }
        }
        if best.is_none() {
            let Some(previous)=represented_curve(edge) else {continue;};
            for mut candidate in exact_carrier_candidates(surfaces, edge_uses, &faces, start, end, bar) {
                if start.sub(end).length() <= bar {
                    if let (Ok(a), Ok(b)) = (candidate.domain().and_then(|d| candidate.derivatives(d[0],1)), edge.curve.derivatives(edge.t0,1)) {
                        if a[1].dot(b[1]) < 0.0 {
                            let Ok(reverse) = candidate.reversed() else { continue };
                            candidate = reverse;
                        }
                    }
                }
                let Ok([c0, c1]) = candidate.domain() else { continue };
                let Ok(after) = worst_residual(&candidate, c0, c1, &surfaces) else { continue };
                if after > bar { continue; }
                // Bidirectional geometric movement avoids accepting a remote
                // analytic branch or the long complementary arc. The entry
                // residual supplies the repair budget, capped by the existing
                // representation contract; it never enlarges acceptance.
                let limit = (8.0 * before).min(carrier_band);
                let mut moved = 0.0f64;
                for (from, a, b, to) in [(&candidate, c0, c1, &previous), (&previous, edge.t0, edge.t1, &candidate)] {
                    for t in carrier_stations(from, a, b) {
                        let Ok(point) = from.evaluate(t) else { moved = f64::INFINITY; break };
                        let Ok(projection) = crate::project_point_to_curve(to, point) else { moved = f64::INFINITY; break };
                        moved = moved.max(projection.distance);
                    }
                }
                if moved <= limit && best.as_ref().map_or(true, |(_, r, _)| after < *r)
                    && super::clearance::motion_clear(edge, &candidate, &faces, &solid.edges) {
                    best = Some((candidate, after, format!("exact carrier section, moved {moved:.3e}, limit {limit:.3e}")));
                }
            }
        }
        if best.as_ref().is_some_and(|(candidate,_,_)|!super::clearance::motion_clear(edge,candidate,&faces,&solid.edges)) {
            declined.push(format!("edge {}: complete old-to-new image motion cannot exclude a nonincident face",edge.id));
            best=None;
        }
        match best {
            Some((curve, after, note)) => {
                if debug {
                    eprintln!("edges: reconstructed edge {} ({} controls, {before:.3e} off) from {note}", edge.id, edge.curve.control_points.len());
                }
                candidates.push((edge_index, curve, before, after));
            }
            None if planar_uses == 0 => declined.push(format!("edge {} ({before:.3e} off): both carriers curved, no planar trim to take", edge.id)),
            None => {}
        }
    }

    let mut replaced = Vec::new();
    for (edge_index, curve, residual_before, residual_after) in candidates {
        let edge = &mut solid.edges[edge_index];
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let previous = std::mem::replace(&mut edge.curve, curve);
        let (previous_t0, previous_t1) = (edge.t0, edge.t1);
        edge.t0 = t0;
        edge.t1 = t1;
        replaced.push(ReplacedEdge { edge_id: edge.id, previous, t0: previous_t0, t1: previous_t1, residual_before, residual_after });
    }
    if debug {
        eprintln!("edges: {off_edges} edges over the {bar:.1e} bar; {} reconstructed, {} declined", replaced.len(), declined.len());
        for line in &declined {
            eprintln!("edges:   declined {line}");
        }
    }
    Ok(replaced)
}

