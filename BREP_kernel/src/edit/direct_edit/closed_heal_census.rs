use super::*;

use std::cell::RefCell;
use std::io::Write as _;

// The rail-parameterisation census of the CLOSED heals and of push face's
// pinned-seam crossing (`BREP_OPEN_HEAL_CENSUS=<dir>`, the open heal's
// instrument, its laws and its 5e-13 corner moves).
//
// Four entries are armed:
//
// * `heal_closed_transition` — the closed strip, cap arm included;
// * `heal_closed_band` — the multi-face closed band;
// * `cap_face_patch` with bridges — the bridged rejoin;
// * `offset_ruled_face` — a push whose rebuild crosses a plane.
//
// Each re-runs its operation on copies of the body whose boundary edges are
// re-parameterised exactly, whose corner vertices are moved 5e-13, and (for
// the two closed rim lanes, whose heal never reads a vertex) whose rim curves
// are translated 5e-13, and appends one JSON line per entry saying how each
// copy answers against the original. While the census runs a copy it also
// LISTENS: the heals write what decided them (`census_note`) — the lane, every
// candidate branch's rating, each extension request, each crossing's end
// heights — and those notes ride on the line beside the verdict.

thread_local! {
    static NOTES: RefCell<Option<serde_json::Map<String, serde_json::Value>>> =
        const { RefCell::new(None) };
}

/// Record what decided the operation being run, when a census is listening.
pub(super) fn census_note(key: &str, value: impl FnOnce() -> serde_json::Value) {
    NOTES.with(|notes| {
        if let Some(map) = notes.borrow_mut().as_mut() {
            map.insert(key.to_string(), value());
        }
    });
}

/// Append to a list note, when a census is listening.
pub(super) fn census_push(key: &str, value: impl FnOnce() -> serde_json::Value) {
    NOTES.with(|notes| {
        if let Some(map) = notes.borrow_mut().as_mut() {
            let entry = map
                .entry(key.to_string())
                .or_insert_with(|| serde_json::Value::Array(Vec::new()));
            if let serde_json::Value::Array(list) = entry {
                list.push(value());
            }
        }
    });
}

fn listening<T>(body: impl FnOnce() -> T) -> (T, serde_json::Value) {
    let outer = NOTES.with(|notes| notes.borrow_mut().replace(serde_json::Map::new()));
    let result = with_census_disarmed(body);
    let notes = NOTES.with(|notes| std::mem::replace(&mut *notes.borrow_mut(), outer));
    (result, serde_json::Value::Object(notes.unwrap_or_default()))
}

struct Run {
    outcome: Outcome,
    notes: serde_json::Value,
    healed: Option<BrepSolid>,
    input_volume: f64,
}

fn run_listening(
    solid: &BrepSolid,
    operation: &dyn Fn(&BrepSolid) -> Result<BrepSolid, String>,
) -> Run {
    let input_top = max_topology_id(solid);
    let input_volume = crate::solid_mass_properties(solid).map(|mass| mass.volume).unwrap_or(f64::NAN);
    let (verdict, notes) = listening(|| operation(solid));
    let healed = verdict.as_ref().ok().cloned();
    let verdict = verdict.map(|healed| {
        let volume = crate::solid_mass_properties(&healed)
            .map(|mass| mass.volume)
            .unwrap_or(f64::NAN);
        let corners: Vec<Vec3> = healed
            .vertices
            .iter()
            .filter(|vertex| vertex.id > input_top)
            .map(|vertex| vertex.point)
            .collect();
        let sound = with_census_disarmed(|| crate::accept_sound(healed.clone(), "closed_heal_census").is_ok());
        (bits_hash(&healed), volume, corners, sound)
    });
    Run { outcome: Outcome { verdict }, notes, healed, input_volume }
}

/// For a copy that `compare` reads as MOVED: how far the copy's healed body is
/// from the original's, by its vertices and by five samples of every edge
/// against the original's edges, with the topology counts. The mass
/// integrator reads a Möbius-substituted pcurve as a rational integrand its
/// Gauss rule is not exact for, so a volume can move where no point did; the
/// input copy's own volume, reported beside it, says when that is the case.
fn shape_deviation(original: &BrepSolid, variant: &BrepSolid) -> serde_json::Value {
    let counts = |body: &BrepSolid| {
        (
            body.shells.iter().map(|shell| shell.faces.len()).sum::<usize>(),
            body.edges.len(),
            body.vertices.len(),
        )
    };
    let mut vertex = 0.0f64;
    for point in &variant.vertices {
        let nearest = original
            .vertices
            .iter()
            .map(|other| other.point.sub(point.point).length())
            .fold(f64::INFINITY, f64::min);
        vertex = vertex.max(nearest);
    }
    let mut edge = 0.0f64;
    for record in &variant.edges {
        for sample in 0..5 {
            let Ok(point) = record.curve.evaluate(record.t0 + (record.t1 - record.t0) * sample as f64 / 4.0) else {
                continue;
            };
            let nearest = original
                .edges
                .iter()
                .filter_map(|other| crate::project_point_to_curve(&other.curve, point).ok())
                .map(|projection| projection.distance)
                .fold(f64::INFINITY, f64::min);
            edge = edge.max(nearest);
        }
    }
    serde_json::json!({
        "same_counts": counts(original) == counts(variant),
        "vertex_deviation": vertex,
        "edge_deviation": edge,
    })
}

/// A copy of `solid` with every control point of `edges` translated by `delta`.
fn shift_edges(solid: &BrepSolid, edges: &[u64], delta: Vec3) -> BrepSolid {
    let mut copy = solid.clone();
    for edge in &mut copy.edges {
        if !edges.contains(&edge.id) {
            continue;
        }
        for control in &mut edge.curve.control_points {
            control.x += delta.x * control.w;
            control.y += delta.y * control.w;
            control.z += delta.z * control.w;
        }
    }
    copy
}

fn nudge_listed_vertices(solid: &BrepSolid, vertices: &HashSet<u64>, delta: Vec3) -> BrepSolid {
    let mut copy = solid.clone();
    for vertex in &mut copy.vertices {
        if vertices.contains(&vertex.id) {
            vertex.point = vertex.point.add(delta);
        }
    }
    copy
}

fn axis_moves(step: f64) -> [(&'static str, Vec3); 6] {
    [
        ("+x", Vec3::new(step, 0.0, 0.0)),
        ("-x", Vec3::new(-step, 0.0, 0.0)),
        ("+y", Vec3::new(0.0, step, 0.0)),
        ("-y", Vec3::new(0.0, -step, 0.0)),
        ("+z", Vec3::new(0.0, 0.0, step)),
        ("-z", Vec3::new(0.0, 0.0, -step)),
    ]
}

/// Every variant of one census entry: the exact laws over `edges`, the corner
/// moves over `vertices`, and the curve translations over `shifted`.
fn census_line(
    lane: &str,
    solid: &BrepSolid,
    edges: &[u64],
    vertices: &HashSet<u64>,
    shifted: &[u64],
    operation: &dyn Fn(&BrepSolid) -> Result<BrepSolid, String>,
    context: serde_json::Value,
) -> serde_json::Value {
    let scale = solid_model_scale(solid);
    let first = run_listening(solid, operation);
    let variant_row = |law: String, run: Run, extra: serde_json::Value| {
        let verdict = compare(&first.outcome, &run.outcome, scale);
        let mut row = serde_json::json!({
            "law": law,
            "against_original": verdict,
            "outcome": describe(&run.outcome),
            "input_volume_shift": run.input_volume - first.input_volume,
            "notes": run.notes,
        });
        if verdict == "moved" {
            if let (Some(a), Some(b)) = (&first.healed, &run.healed) {
                row["shape"] = shape_deviation(a, b);
            }
        }
        if let (serde_json::Value::Object(map), serde_json::Value::Object(more)) = (&mut row, extra) {
            map.extend(more);
        }
        row
    };
    let mut variants = Vec::new();
    for law in [
        Law::Reverse,
        Law::Mobius(4.0),
        Law::Mobius(0.25),
        Law::ReverseMobius(4.0),
        Law::Affine(-2.0, 5.0),
    ] {
        match reparameterise_edges(solid, edges, law) {
            Ok((copy, residual)) => {
                let validates = copy.validate().is_empty();
                let run = run_listening(&copy, operation);
                variants.push(variant_row(
                    law.label(),
                    run,
                    serde_json::json!({ "reparam_residual": residual, "input_validates": validates }),
                ));
            }
            Err(error) => variants.push(serde_json::json!({ "law": law.label(), "skipped": error })),
        }
    }
    let step = 5e-13 * scale.max(1.0);
    for (axis, delta) in axis_moves(step) {
        let copy = nudge_listed_vertices(solid, vertices, delta);
        let run = run_listening(&copy, operation);
        variants.push(variant_row(format!("nudge{axis}"), run, serde_json::json!({})));
    }
    if !shifted.is_empty() {
        for (axis, delta) in axis_moves(step) {
            let copy = shift_edges(solid, shifted, delta);
            let run = run_listening(&copy, operation);
            variants.push(variant_row(format!("shift{axis}"), run, serde_json::json!({})));
        }
    }
    serde_json::json!({
        "lane": lane,
        "context": {
            "thread": std::thread::current().name().unwrap_or("").to_string(),
            "args": std::env::args().collect::<Vec<_>>(),
        },
        "input_validates": solid.validate().is_empty(),
        "detail": context,
        "rails": edges.iter().map(|edge_id| rail_row(solid, *edge_id)).collect::<Vec<_>>(),
        "original": describe(&first.outcome),
        "original_notes": first.notes,
        "variants": variants,
    })
}

fn append(directory: &str, line: &serde_json::Value) {
    let path =
        std::path::Path::new(directory).join(format!("census-{}.jsonl", std::process::id()));
    // One write per line, so lines from parallel tests do not interleave.
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(&path) {
        let _ = file.write_all(format!("{line}\n").as_bytes());
    }
}

fn surface_of(solid: &BrepSolid, face_id: u64) -> Option<NurbsSurface> {
    find_face(solid, face_id).map(|(shell, face)| solid.shells[shell].faces[face].surface.clone())
}

/// Distance from `point` to a neighbour's carrier: a plane's own plane (the
/// heal widens it), anything else by projection.
fn carrier_distance(surface: &NurbsSurface, point: Vec3, scale: f64) -> f64 {
    if let Ok(plane) = plane_of_surface(surface, (scale * 1e-6).max(1e-7), "census") {
        return point.sub(plane.origin).dot(plane.normal).abs();
    }
    crate::project_point_to_surface(surface, point)
        .map(|projection| projection.distance)
        .unwrap_or(f64::INFINITY)
}

fn carrier_kind(surface: &NurbsSurface, scale: f64) -> &'static str {
    if plane_of_surface(surface, (scale * 1e-6).max(1e-7), "census").is_ok() {
        "plane"
    } else if surface.analytic().is_some() {
        "curved"
    } else {
        "free-form"
    }
}

/// How many of `points` pass the marcher's seed gate onto BOTH carriers.
fn seed_gate_row(solid: &BrepSolid, points: &[Vec3], neighbours: [u64; 2], gate: f64) -> serde_json::Value {
    let scale = solid_model_scale(solid);
    let (Some(first), Some(second)) = (surface_of(solid, neighbours[0]), surface_of(solid, neighbours[1]))
    else {
        return serde_json::json!({ "missing": true });
    };
    let mut passing = 0usize;
    let mut nearest = f64::INFINITY;
    for &point in points {
        let a = carrier_distance(&first, point, scale);
        let b = carrier_distance(&second, point, scale);
        if a <= gate && b <= gate {
            passing += 1;
        }
        nearest = nearest.min(a.max(b));
    }
    serde_json::json!({
        "samples": points.len(),
        "passing_gate": passing,
        "gate": gate,
        "nearest_to_both": nearest,
    })
}

/// The closed strip (`heal_closed_transition`).
pub(super) fn record_closed_heal_census(
    directory: &str,
    solid: &BrepSolid,
    shell_index: usize,
    face_index: usize,
    boundary: &[(u64, bool)],
) {
    let face_id = solid.shells[shell_index].faces[face_index].id;
    let scale = solid_model_scale(solid);
    let mut edges: Vec<u64> = Vec::new();
    for (edge_id, _) in boundary {
        if !edges.contains(edge_id) {
            edges.push(*edge_id);
        }
    }
    let rims: Vec<u64> = edges
        .iter()
        .copied()
        .filter(|edge_id| boundary.iter().filter(|(id, _)| id == edge_id).count() == 1)
        .collect();
    let vertices: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edges.contains(&edge.id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let neighbours: Vec<u64> = rims
        .iter()
        .filter_map(|rim| other_face_of_edge(solid, *rim, face_id).ok())
        .collect();
    // The heal's own stations: eight per rim at equal arc length.
    let mut samples = Vec::new();
    for rim in &rims {
        if let Some(edge) = solid.edges.iter().find(|edge| edge.id == *rim) {
            if let Ok(mut stations) = arc_length_stations(edge, 9, true) {
                stations.pop();
                samples.extend(stations);
            }
        }
    }
    let march_tolerance = (scale * 1e-7).max(1e-9);
    let detail = serde_json::json!({
        "strip": face_id,
        "rims": rims,
        "neighbours": neighbours,
        "kinds": neighbours
            .iter()
            .map(|id| surface_of(solid, *id).map(|surface| carrier_kind(&surface, scale)).unwrap_or("missing"))
            .collect::<Vec<_>>(),
        "seed_gate": if neighbours.len() == 2 {
            seed_gate_row(solid, &samples, [neighbours[0], neighbours[1]], march_tolerance * 100.0)
        } else {
            serde_json::Value::Null
        },
    });
    let operation = |body: &BrepSolid| -> Result<BrepSolid, String> {
        let boundary = strip_boundary(body, face_id).ok_or("strip vanished")?;
        Ok(heal_closed_transition(body, shell_index, face_index, &boundary)?)
    };
    let line = census_line("closed", solid, &edges, &vertices, &rims, &operation, detail);
    append(directory, &line);
}

/// The closed band (`heal_closed_band`).
pub(super) fn record_closed_band_census(directory: &str, solid: &BrepSolid, band: &ClosedBand) {
    let mut faces: Vec<u64> = band.faces.iter().copied().collect();
    faces.sort_unstable();
    let mut rim_edges: Vec<u64> = Vec::new();
    for rim in &band.rims {
        if let Some((shell, position)) = find_face(solid, rim.neighbour) {
            let loop_record = &solid.shells[shell].faces[position].loops[rim.loop_index];
            for index in &rim.run {
                rim_edges.push(loop_record.coedges[*index].edge_id);
            }
        }
    }
    let mut edges: Vec<u64> = band.edges.iter().copied().collect();
    edges.sort_unstable();
    let vertices: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| band.edges.contains(&edge.id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let detail = serde_json::json!({
        "faces": faces,
        "neighbours": [band.rims[0].neighbour, band.rims[1].neighbour],
        "rim_edges": rim_edges,
    });
    let operation = |body: &BrepSolid| -> Result<BrepSolid, String> {
        let band = classify_closed_band(body, &faces).ok_or("the copy is no longer a closed band")?;
        Ok(heal_closed_band(body, &band, "delete_faces_and_heal")?)
    };
    let line = census_line("band", solid, &edges, &vertices, &rim_edges, &operation, detail);
    append(directory, &line);
}

/// The bridged rejoin: `cap_face_patch` on a patch that lays bridges. The copy
/// is re-classified from the same face ids, so the rejoin's own reading of the
/// re-parameterised circuit is what is measured.
pub(super) fn record_rejoin_census(
    directory: &str,
    solid: &BrepSolid,
    face_ids: &[u64],
    rejoined_faces: &[u64],
    operation: &dyn Fn(&BrepSolid) -> Result<BrepSolid, String>,
) {
    let mut edges: Vec<u64> = Vec::new();
    for face_id in face_ids.iter().chain(rejoined_faces) {
        if let Some((shell, position)) = find_face(solid, *face_id) {
            for coedge in solid.shells[shell].faces[position]
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
            {
                if !edges.contains(&coedge.edge_id) {
                    edges.push(coedge.edge_id);
                }
            }
        }
    }
    let vertices: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edges.contains(&edge.id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let detail = serde_json::json!({ "faces": face_ids, "rejoined": rejoined_faces });
    let line = census_line("rejoin", solid, &edges, &vertices, &[], operation, detail);
    append(directory, &line);
}

/// A push face (`offset_ruled_face`): the pushed face's boundary edges
/// re-parameterised and its corners moved.
pub(super) fn record_push_census(directory: &str, solid: &BrepSolid, face_id: u64, distance: f64) {
    let Some((shell, position)) = find_face(solid, face_id) else {
        return;
    };
    let mut edges: Vec<u64> = Vec::new();
    for coedge in solid.shells[shell].faces[position]
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
    {
        if !edges.contains(&coedge.edge_id) {
            edges.push(coedge.edge_id);
        }
    }
    let vertices: HashSet<u64> = solid
        .edges
        .iter()
        .filter(|edge| edges.contains(&edge.id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let detail = serde_json::json!({ "face": face_id, "distance": distance });
    let operation =
        |body: &BrepSolid| offset_ruled_face(body, face_id, distance).map_err(String::from);
    let line = census_line("push", solid, &edges, &vertices, &[], &operation, detail);
    append(directory, &line);
}

/// The signed heights of a curve's two ends over `plane`, and which sampled
/// interval the strict crossing's root fell in — what a crossing note carries.
pub(super) fn crossing_note(
    curve: &NurbsCurve,
    plane: &Plane,
    strict: Option<(f64, Vec3)>,
    lenient: Option<(f64, Vec3)>,
    closed: bool,
) -> serde_json::Value {
    let [t0, t1] = curve.domain().unwrap_or([f64::NAN, f64::NAN]);
    let height = |t: f64| {
        curve
            .evaluate(t)
            .map(|point| point.sub(plane.origin).dot(plane.normal))
            .unwrap_or(f64::NAN)
    };
    let interval = |t: f64| ((t - t0) / (t1 - t0) * 64.0).floor();
    serde_json::json!({
        "closed": closed,
        "h_start": height(t0),
        "h_end": height(t1),
        "strict": strict.map(|(t, p)| serde_json::json!({ "t_fraction": (t - t0) / (t1 - t0), "interval": interval(t), "point": [p.x, p.y, p.z] })),
        "lenient": lenient.map(|(t, p)| serde_json::json!({ "t_fraction": (t - t0) / (t1 - t0), "point": [p.x, p.y, p.z] })),
        "gap": match (strict, lenient) {
            (Some((_, a)), Some((_, b))) => a.sub(b).length(),
            _ => f64::NAN,
        },
    })
}
