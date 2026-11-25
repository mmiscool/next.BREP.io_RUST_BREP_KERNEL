//! AP242 tessellated solids (`TESSELLATED_SOLID` of `TRIANGULATED_FACE`s over
//! `COORDINATES_LIST`s), read as polyhedra: the triangles that lie in one plane
//! and meet across edges are one planar face, bounded by the edges of the
//! triangles that no other triangle of that face shares; the faces are sewn
//! along their shared edges into a closed solid.

use rustc_hash::FxHashMap as HashMap;

use super::parse::{Entity, Value};
use crate::{BrepSolid, NurbsCurve, NurbsSurface, Vec3, Vec4};

/// The solids of every `TESSELLATED_SOLID` in `entities`, each at the length
/// unit of the representation that lists it (`scale` mm per file unit where
/// none says). `Ok(vec![])` when the file has none.
///
/// Refuses a file that places its parts (mapped items, transformed
/// representation relationships or assembly usages): this reader builds each
/// solid where its points stand, and a part placed elsewhere would come in
/// at the wrong place.
pub(super) fn tessellated_solids(entities: &HashMap<usize, Entity>, scale: f64) -> Result<Vec<BrepSolid>, String> {
    let mut ids: Vec<usize> = entities.iter().filter(|(_, e)| e.has("TESSELLATED_SOLID")).map(|(&id, _)| id).collect();
    if ids.is_empty() {
        return Ok(Vec::new());
    }
    ids.sort_unstable();
    if places_parts(entities) {
        return Err("step_import: tessellated solid: the file places its parts (an assembly or mapped items), which this reader does not follow".into());
    }
    // Each solid's unit: the context of the representation listing it.
    let mut units: HashMap<usize, f64> = HashMap::default();
    for entity in entities.values() {
        let Some(args) = entity.records.iter().find(|(name, _)| name.ends_with("REPRESENTATION")).map(|(_, args)| args) else { continue };
        let (Some(Value::List(items)), Some(Value::Ref(context))) = (args.get(1), args.get(2)) else { continue };
        let Some(unit) = super::parse::context_length_scale_mm(entities, *context) else { continue };
        for item in items {
            if let Ok(id) = item.as_ref_id() {
                units.insert(id, unit);
            }
        }
    }
    ids.into_iter().map(|id| solid_of(entities, id, units.get(&id).copied().unwrap_or(scale))).collect()
}

/// Whether the file places its parts: an assembly usage, a transformed
/// representation relationship, or a mapped item.
///
/// `BREP_STEP_TESSELLATED_PRESENTATION_MAPS` (default on, `=0` restores; `check` reports a
/// file the switch would read and refuses it as before): a mapped item that
/// only draughting models use (a PMI saved view's placement of the part's
/// shape, NIST ftc_08's `MBD_A`..`MBD_D`) draws the part for that view and
/// places nothing in the shape the reader builds, so it does not count. A
/// mapped item anything else holds, or that nothing holds, still counts.
fn places_parts(entities: &HashMap<usize, Entity>) -> bool {
    if entities.values().any(|e| e.has("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION") || e.has("NEXT_ASSEMBLY_USAGE_OCCURRENCE")) {
        return true;
    }
    let mapped: rustc_hash::FxHashSet<usize> = entities.iter().filter(|(_, e)| e.has("MAPPED_ITEM")).map(|(&id, _)| id).collect();
    if mapped.is_empty() {
        return false;
    }
    let check = crate::env_read!("BREP_STEP_TESSELLATED_PRESENTATION_MAPS").as_deref() == Some("check");
    if !check && !crate::switch!("BREP_STEP_TESSELLATED_PRESENTATION_MAPS", true) {
        return true;
    }
    fn references(value: &Value, out: &mut Vec<usize>) {
        match value {
            Value::Ref(id) => out.push(*id),
            Value::List(values) | Value::Typed(_, values) => values.iter().for_each(|value| references(value, out)),
            _ => {}
        }
    }
    // A mapped item no draughting model holds (or that nothing holds) may
    // place a part: only one that draughting models alone use is set aside.
    let mut drawn: rustc_hash::FxHashSet<usize> = rustc_hash::FxHashSet::default();
    let mut placing = false;
    let mut held = Vec::new();
    for entity in entities.values() {
        held.clear();
        entity.records.iter().flat_map(|(_, args)| args).for_each(|value| references(value, &mut held));
        held.retain(|id| mapped.contains(id));
        if held.is_empty() {
            continue;
        }
        if entity.has("DRAUGHTING_MODEL") {
            drawn.extend(held.iter().copied());
        } else {
            placing = true;
        }
    }
    let placing = placing || drawn.len() < mapped.len();
    if check {
        if !placing {
            eprintln!("STEP_TESSELLATED_PRESENTATION_MAPS: {} mapped item(s), each only in draughting models; the switch reads the tessellated solid", mapped.len());
        }
        return true;
    }
    placing
}

/// `BREP_STEP_COMPLEX_TRIANGULATED_FACE` (default on, `=0` restores): a tessellated solid's
/// `COMPLEX_TRIANGULATED_FACE`s (triangle strips and fans, NIST ftc_08) are
/// read as their triangles; off, such a face is refused as before.
fn complex_triangulated_face_enabled() -> bool {
    crate::switch!("BREP_STEP_COMPLEX_TRIANGULATED_FACE", true)
}

fn get(entities: &HashMap<usize, Entity>, id: usize) -> Result<&Entity, String> {
    entities.get(&id).ok_or_else(|| format!("step_import: tessellated solid: entity #{id} is missing"))
}

/// One `TESSELLATED_SOLID`'s polyhedron.
fn solid_of(entities: &HashMap<usize, Entity>, id: usize, scale: f64) -> Result<BrepSolid, String> {
    let args = get(entities, id)?.find("TESSELLATED_SOLID").ok_or("step_import: tessellated solid: no record")?;
    let items = args.get(1).ok_or("step_import: tessellated solid: no items")?.as_list()?;
    // Points are welded where they are the same point, whichever list holds
    // them, so faces on lists of their own still share their corners.
    let mut points: Vec<Vec3> = Vec::new();
    let mut welded: HashMap<[u64; 3], usize> = HashMap::default();
    let mut triangles: Vec<[usize; 3]> = Vec::new();
    for item in items {
        let face_id = item.as_ref_id()?;
        let face = get(entities, face_id)?;
        let (face, complex) = match (face.find("TRIANGULATED_FACE"), face.find("COMPLEX_TRIANGULATED_FACE")) {
            (Some(face), _) => (face, false),
            (None, Some(face)) if complex_triangulated_face_enabled() => (face, true),
            _ => return Err(format!("step_import: tessellated solid: item #{face_id} is not a TRIANGULATED_FACE")),
        };
        // (name, coordinates, pnmax, normals, geometric_link, pnindex, triangles)
        let list_id = face.get(1).ok_or("step_import: triangulated face: no coordinates")?.as_ref_id()?;
        let list = get(entities, list_id)?.find("COORDINATES_LIST").ok_or("step_import: triangulated face: coordinates are not a COORDINATES_LIST")?;
        let coords = list.get(2).ok_or("step_import: coordinates list: no points")?.as_list()?;
        let pnindex: Vec<usize> = match face.get(5) {
            Some(Value::List(items)) => items.iter().map(index).collect::<Result<_, _>>()?,
            _ => Vec::new(),
        };
        let mut point = |k: usize| -> Result<usize, String> {
            // 1-based, through `pnindex` when it is given.
            let k = if pnindex.is_empty() { k } else { *pnindex.get(k.wrapping_sub(1)).ok_or("step_import: triangulated face: index past pnindex")? };
            let c = coords.get(k.wrapping_sub(1)).ok_or("step_import: triangulated face: index past its coordinates")?.as_list()?;
            if c.len() != 3 {
                return Err("step_import: coordinates list: a point is not three numbers".into());
            }
            let p = Vec3::new(c[0].as_real()? * scale, c[1].as_real()? * scale, c[2].as_real()? * scale);
            // +0.0 makes -0.0 the same key as 0.0.
            let key = [(p.x + 0.0).to_bits(), (p.y + 0.0).to_bits(), (p.z + 0.0).to_bits()];
            if let Some(&at) = welded.get(&key) {
                return Ok(at);
            }
            points.push(p);
            welded.insert(key, points.len() - 1);
            Ok(points.len() - 1)
        };
        if complex {
            // (name, coordinates, pnmax, normals, geometric_link, pnindex,
            // triangle_strips, triangle_fans): a strip's triangle k is its
            // points k, k+1, k+2, the odd ones turned (k+1, k, k+2) so every
            // triangle turns the same way; a fan's triangles share its first
            // point. A strip's triangle that repeats a corner (a strip joined
            // through a degenerate triangle) bounds nothing and is left out.
            let lists = |at: usize| -> Result<Vec<Vec<usize>>, String> {
                match face.get(at) {
                    Some(Value::List(items)) => items
                        .iter()
                        .map(|item| item.as_list()?.iter().map(index).collect::<Result<Vec<_>, _>>())
                        .collect(),
                    _ => Ok(Vec::new()),
                }
            };
            for strip in lists(6)? {
                for k in 0..strip.len().saturating_sub(2) {
                    let corners = [point(strip[k])?, point(strip[k + 1])?, point(strip[k + 2])?];
                    let corners = if k % 2 == 0 { corners } else { [corners[1], corners[0], corners[2]] };
                    if corners[0] != corners[1] && corners[1] != corners[2] && corners[0] != corners[2] {
                        triangles.push(corners);
                    }
                }
            }
            for fan in lists(7)? {
                for k in 1..fan.len().saturating_sub(1) {
                    let corners = [point(fan[0])?, point(fan[k])?, point(fan[k + 1])?];
                    if corners[0] != corners[1] && corners[1] != corners[2] && corners[0] != corners[2] {
                        triangles.push(corners);
                    }
                }
            }
            continue;
        }
        for tri in face.get(6).ok_or("step_import: triangulated face: no triangles")?.as_list()? {
            let t = tri.as_list()?;
            if t.len() != 3 {
                return Err("step_import: triangulated face: a triangle is not three indices".into());
            }
            let corners = [point(index(&t[0])?)?, point(index(&t[1])?)?, point(index(&t[2])?)?];
            triangles.push(corners);
        }
    }
    polyhedron(&points, &triangles)
}

fn index(value: &Value) -> Result<usize, String> {
    let real = value.as_real()?;
    if real.fract() != 0.0 || real < 1.0 {
        return Err(format!("step_import: triangulated face: {real} is not an index"));
    }
    Ok(real as usize)
}

/// The closed polyhedron the triangles bound, each turning counter-clockwise
/// seen from outside.
fn polyhedron(points: &[Vec3], triangles: &[[usize; 3]]) -> Result<BrepSolid, String> {
    let fail = |message: &str| format!("step_import: tessellated solid: {message}");
    if triangles.len() < 4 {
        return Err(fail("fewer than four triangles bound no solid"));
    }
    let (mut low, mut high) = (points[0], points[0]);
    for p in points {
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    let size = high.sub(low).length();
    let normal = |t: &[usize; 3]| points[t[1]].sub(points[t[0]]).cross(points[t[2]].sub(points[t[0]]));
    let unit: Vec<Vec3> = triangles
        .iter()
        .map(|t| {
            let n = normal(t);
            let l = n.length();
            if l > 0.0 { Ok(n.scale(1.0 / l)) } else { Err(fail("a triangle has no area")) }
        })
        .collect::<Result<_, _>>()?;
    // Each directed edge to its triangle; a manifold closed mesh has every
    // edge once each way.
    let mut owner: HashMap<(usize, usize), usize> = HashMap::default();
    for (k, t) in triangles.iter().enumerate() {
        for e in 0..3 {
            if owner.insert((t[e], t[(e + 1) % 3]), k).is_some() {
                return Err(fail("an edge is used twice the same way"));
            }
        }
    }
    if let Some((a, b)) = crossing(points, triangles, 1e-9 * size.max(1.0)) {
        return Err(fail(&format!("triangles {a} and {b} cross one another")));
    }
    if let Some((a, b)) = overlapping_parts(points, triangles, &owner, 1e-9 * size.max(1.0)) {
        return Err(fail(&format!("its closed parts {a} and {b} touch or overlap")));
    }
    // Triangles meeting across an edge in one plane are one face.
    let flat = 1e-9 * size.max(1.0);
    let coplanar = |a: usize, b: usize| {
        unit[a].dot(unit[b]) > 1.0 - 1e-12 && triangles[b].iter().all(|&v| points[v].sub(points[triangles[a][0]]).dot(unit[a]).abs() <= flat)
    };
    let mut group = vec![usize::MAX; triangles.len()];
    let mut groups = 0;
    for start in 0..triangles.len() {
        if group[start] != usize::MAX {
            continue;
        }
        group[start] = groups;
        let mut stack = vec![start];
        while let Some(k) = stack.pop() {
            let t = triangles[k];
            for e in 0..3 {
                let Some(&other) = owner.get(&(t[(e + 1) % 3], t[e])) else {
                    return Err(fail("the triangles do not close: an edge has no triangle on its other side"));
                };
                if group[other] == usize::MAX && coplanar(k, other) {
                    group[other] = groups;
                    stack.push(other);
                }
            }
        }
        groups += 1;
    }
    let line = |a: usize, b: usize| crate::make_line(points[a], points[b]).map_err(|e| fail(&e));
    let mut sheets = Vec::with_capacity(groups);
    for g in 0..groups {
        let members: Vec<usize> = (0..triangles.len()).filter(|&k| group[k] == g).collect();
        // The face's boundary: its triangles' edges whose other side is another face's.
        let mut next: HashMap<usize, usize> = HashMap::default();
        for &k in &members {
            let t = triangles[k];
            for e in 0..3 {
                let (a, b) = (t[e], t[(e + 1) % 3]);
                if group[owner[&(b, a)]] != g && next.insert(a, b).is_some() {
                    return Err(fail("a face's boundary passes a vertex twice"));
                }
            }
        }
        let mut loops: Vec<Vec<usize>> = Vec::new();
        let mut left = next.clone();
        while let Some(&start) = left.keys().min() {
            let mut chain = vec![start];
            let mut at = left.remove(&start).ok_or_else(|| fail("a face's boundary does not close"))?;
            while at != start {
                chain.push(at);
                at = left.remove(&at).ok_or_else(|| fail("a face's boundary does not close"))?;
            }
            loops.push(chain);
        }
        let n = unit[members[0]];
        let origin = points[triangles[members[0]][0]];
        let e1 = points[triangles[members[0]][1]].sub(origin);
        let e1 = e1.scale(1.0 / e1.length());
        let e2 = n.cross(e1);
        // The outer loop first: the one enclosing the most area about the normal.
        let area = |chain: &[usize]| {
            let mut twice = 0.0;
            for i in 0..chain.len() {
                let (a, b) = (points[chain[i]].sub(origin), points[chain[(i + 1) % chain.len()]].sub(origin));
                twice += a.dot(e1) * b.dot(e2) - b.dot(e1) * a.dot(e2);
            }
            0.5 * twice
        };
        loops.sort_by(|a, b| area(b).abs().total_cmp(&area(a).abs()));
        // The plane, normal outward, over the face's extent and a margin.
        let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for &k in &members {
            for &v in &triangles[k] {
                let d = points[v].sub(origin);
                (u0, u1, v0, v1) = (u0.min(d.dot(e1)), u1.max(d.dot(e1)), v0.min(d.dot(e2)), v1.max(d.dot(e2)));
            }
        }
        let margin = 0.1 * (u1 - u0).max(v1 - v0);
        let (u0, u1, v0, v1) = (u0 - margin, u1 + margin, v0 - margin, v1 + margin);
        let corner = |u: f64, v: f64| Vec4::from_point(origin.add(e1.scale(u)).add(e2.scale(v)), 1.0);
        let plane = NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], vec![vec![corner(u0, v0), corner(u0, v1)], vec![corner(u1, v0), corner(u1, v1)]])
            .map_err(|e| fail(&e))?;
        let curves: Vec<Vec<NurbsCurve>> = loops
            .iter()
            .map(|chain| (0..chain.len()).map(|i| line(chain[i], chain[(i + 1) % chain.len()])).collect::<Result<Vec<_>, _>>())
            .collect::<Result<_, _>>()?;
        sheets.push(crate::face_from_surface_trimmed(&plane, &curves).map_err(|e| fail(&e.to_string()))?);
    }
    crate::solid_from_sheets(&sheets, 1e-9 * size.max(1.0)).map_err(|e| fail(&e.to_string()))
}

/// Two of the mesh's closed parts (triangles joined across edges) of which
/// one has a corner on or inside the other: they touch or overlap, and are
/// not one solid's boundary. A corner is on a part within `flat` of one of
/// its triangles and inside it where the part's winding number about it
/// passes one half.
fn overlapping_parts(points: &[Vec3], triangles: &[[usize; 3]], owner: &HashMap<(usize, usize), usize>, flat: f64) -> Option<(usize, usize)> {
    let mut part = vec![usize::MAX; triangles.len()];
    let mut parts = 0;
    for start in 0..triangles.len() {
        if part[start] != usize::MAX {
            continue;
        }
        part[start] = parts;
        let mut stack = vec![start];
        while let Some(k) = stack.pop() {
            let t = triangles[k];
            for e in 0..3 {
                if let Some(&other) = owner.get(&(t[(e + 1) % 3], t[e])) {
                    if part[other] == usize::MAX {
                        part[other] = parts;
                        stack.push(other);
                    }
                }
            }
        }
        parts += 1;
    }
    if parts < 2 {
        return None;
    }
    let corners = |p: usize| -> Vec<usize> {
        let mut out: Vec<usize> = (0..triangles.len()).filter(|&k| part[k] == p).flat_map(|k| triangles[k]).collect();
        out.sort_unstable();
        out.dedup();
        out
    };
    for a in 0..parts {
        let own: Vec<usize> = (0..triangles.len()).filter(|&k| part[k] == a).collect();
        for b in (0..parts).filter(|&b| b != a) {
            for v in corners(b) {
                let x = points[v];
                let mut winding = 0.0;
                for &k in &own {
                    let [p, q, r] = triangles[k].map(|i| points[i]);
                    if distance_to_triangle(x, p, q, r) <= flat {
                        return Some((a.min(b), a.max(b)));
                    }
                    // Van Oosterom and Strackee: the solid angle the triangle subtends.
                    let (u, v, w) = (p.sub(x), q.sub(x), r.sub(x));
                    let (lu, lv, lw) = (u.length(), v.length(), w.length());
                    let det = u.dot(v.cross(w));
                    let div = lu * lv * lw + u.dot(v) * lw + v.dot(w) * lu + w.dot(u) * lv;
                    winding += 2.0 * det.atan2(div);
                }
                if (winding / (4.0 * std::f64::consts::PI)).abs() > 0.5 {
                    return Some((a.min(b), a.max(b)));
                }
            }
        }
    }
    None
}

/// The distance from `x` to the triangle `p q r`.
fn distance_to_triangle(x: Vec3, p: Vec3, q: Vec3, r: Vec3) -> f64 {
    let n = q.sub(p).cross(r.sub(p));
    let area2 = n.dot(n);
    if area2 > 0.0 {
        // Inside the prism over the triangle: the distance to its plane.
        let inside = [(p, q), (q, r), (r, p)].iter().all(|&(a, b)| b.sub(a).cross(x.sub(a)).dot(n) >= 0.0);
        if inside {
            return x.sub(p).dot(n).abs() / area2.sqrt();
        }
    }
    let to_segment = |a: Vec3, b: Vec3| {
        let d = b.sub(a);
        let t = if d.dot(d) > 0.0 { (x.sub(a).dot(d) / d.dot(d)).clamp(0.0, 1.0) } else { 0.0 };
        x.sub(a.add(d.scale(t))).length()
    };
    to_segment(p, q).min(to_segment(q, r)).min(to_segment(r, p))
}

/// Two triangles sharing no corner, one of whose edges passes through the
/// other's interior (`flat` clear of its plane on both sides and inside it):
/// the mesh crosses itself. Pairs are swept along x by their boxes.
fn crossing(points: &[Vec3], triangles: &[[usize; 3]], flat: f64) -> Option<(usize, usize)> {
    let boxes: Vec<[Vec3; 2]> = triangles
        .iter()
        .map(|t| {
            let (a, b, c) = (points[t[0]], points[t[1]], points[t[2]]);
            [Vec3::new(a.x.min(b.x).min(c.x), a.y.min(b.y).min(c.y), a.z.min(b.z).min(c.z)), Vec3::new(a.x.max(b.x).max(c.x), a.y.max(b.y).max(c.y), a.z.max(b.z).max(c.z))]
        })
        .collect();
    let mut order: Vec<usize> = (0..triangles.len()).collect();
    order.sort_by(|&a, &b| boxes[a][0].x.total_cmp(&boxes[b][0].x));
    let pierces = |edge: [Vec3; 2], t: &[usize; 3]| {
        let (a, b, c) = (points[t[0]], points[t[1]], points[t[2]]);
        let n = b.sub(a).cross(c.sub(a));
        let length = n.length();
        if length == 0.0 {
            return false;
        }
        let n = n.scale(1.0 / length);
        let (dp, dq) = (edge[0].sub(a).dot(n), edge[1].sub(a).dot(n));
        if !((dp > flat && dq < -flat) || (dp < -flat && dq > flat)) {
            return false;
        }
        let x = edge[0].add(edge[1].sub(edge[0]).scale(dp / (dp - dq)));
        // Strictly inside: on the inner side of each edge by more than `flat`.
        [(a, b), (b, c), (c, a)].iter().all(|&(p, q)| q.sub(p).cross(x.sub(p)).dot(n) > flat * q.sub(p).length())
    };
    for (i, &k) in order.iter().enumerate() {
        for &m in &order[i + 1..] {
            if boxes[m][0].x > boxes[k][1].x {
                break;
            }
            let apart = boxes[m][0].y > boxes[k][1].y || boxes[k][0].y > boxes[m][1].y || boxes[m][0].z > boxes[k][1].z || boxes[k][0].z > boxes[m][1].z;
            if apart || triangles[k].iter().any(|v| triangles[m].contains(v)) {
                continue;
            }
            let edges = |t: &[usize; 3]| [[points[t[0]], points[t[1]]], [points[t[1]], points[t[2]]], [points[t[2]], points[t[0]]]];
            if edges(&triangles[k]).iter().any(|&e| pierces(e, &triangles[m])) || edges(&triangles[m]).iter().any(|&e| pierces(e, &triangles[k])) {
                return Some((k, m));
            }
        }
    }
    None
}

