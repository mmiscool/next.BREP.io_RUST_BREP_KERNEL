use super::*;

use std::cell::Cell;
use std::hash::{Hash, Hasher};
use std::io::Write as _;

// The rail-parameterisation census of the open mixed heal
// (`BREP_OPEN_HEAL_CENSUS=<dir>`).
//
// The heal reads the strip's four boundary edges as curves, and a curve is a
// geometry AND a parameterisation. This instrument asks whether the heal's
// answer depends on the second: at every entry to
// `heal_open_transition_mixed` it re-runs the heal on copies of the body whose
// four strip edges are re-parameterised EXACTLY — reversed, re-weighted by a
// Möbius substitution, or mapped to another domain — and appends one JSON line
// per heal to `<dir>/census-<pid>.jsonl` saying whether each copy answers with
// the same bits, the same geometry, other geometry, or the other verdict.
//
// A Möbius substitution `t = u / (λ + (1 − λ) u)` (normalised to the edge's
// own `[t0, t1]`) is a rational-linear change of parameter, so it keeps a
// NURBS a NURBS of the same degree: the knots map through its inverse and each
// homogeneous control point is scaled by the product of `λ + (1 − λ) u` over
// the new knots of its blossom. No point moves; only the speed law does, and
// a uniform-in-parameter sample lands somewhere else along the same curve.
// The coedge pcurves are substituted with the same law in their own direction
// (λ forward, 1/λ reverse), so `validate` reads the copy exactly as it reads
// the original.

thread_local! {
    static INSIDE: Cell<bool> = const { Cell::new(false) };
}

/// Run `body` with the census disarmed, so a heal the census re-runs does not
/// record itself.
pub(super) fn with_census_disarmed<T>(body: impl FnOnce() -> T) -> T {
    let outer = INSIDE.with(|inside| inside.replace(true));
    let result = body();
    INSIDE.with(|inside| inside.set(outer));
    result
}

pub(super) fn census_directory() -> Option<String> {
    let directory = std::env::var("BREP_OPEN_HEAL_CENSUS").ok()?;
    (!directory.is_empty() && !INSIDE.with(Cell::get)).then_some(directory)
}

/// A normalised Möbius map fixing 0 and 1: `τ(u) = u / (λ + (1 − λ) u)`.
fn mobius_inverse(tau: f64, lambda: f64) -> f64 {
    lambda * tau / (1.0 - tau + lambda * tau)
}

/// Substitute `t = lo + (hi − lo)·τ((s − lo)/(hi − lo))` into `curve`. `None`
/// when a knot outside `[lo, hi]` would cross the map's pole.
fn mobius_curve(curve: &NurbsCurve, lo: f64, hi: f64, lambda: f64) -> Option<NurbsCurve> {
    let span = hi - lo;
    if !(span > 0.0) {
        return None;
    }
    let mut knots = Vec::with_capacity(curve.knots.len());
    let mut normalised = Vec::with_capacity(curve.knots.len());
    for &knot in &curve.knots {
        let tau = (knot - lo) / span;
        if 1.0 - tau + lambda * tau <= 0.0 {
            return None;
        }
        let u = mobius_inverse(tau, lambda);
        normalised.push(u);
        knots.push(lo + span * u);
    }
    let degree = curve.degree;
    let mut control_points = Vec::with_capacity(curve.control_points.len());
    for (index, control) in curve.control_points.iter().enumerate() {
        let mut factor = 1.0;
        for k in 1..=degree {
            let denominator = lambda + (1.0 - lambda) * normalised[index + k];
            if denominator <= 0.0 {
                return None;
            }
            factor *= denominator;
        }
        control_points.push(crate::Vec4 {
            x: control.x * factor,
            y: control.y * factor,
            z: control.z * factor,
            w: control.w * factor,
        });
    }
    NurbsCurve::new(degree, knots, control_points).ok()
}

/// An exact change of an edge's parameterisation that keeps every point of it.
#[derive(Clone, Copy)]
pub(super) enum Law {
    Reverse,
    Mobius(f64),
    ReverseMobius(f64),
    Affine(f64, f64),
}

impl Law {
    pub(super) fn label(self) -> String {
        match self {
            Law::Reverse => "reverse".into(),
            Law::Mobius(lambda) => format!("mobius{lambda}"),
            Law::ReverseMobius(lambda) => format!("reverse+mobius{lambda}"),
            Law::Affine(low, high) => format!("affine[{low},{high}]"),
        }
    }
}

fn reverse_edge(solid: &mut BrepSolid, edge_id: u64) -> Result<(), String> {
    let edge = solid
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .ok_or("missing edge")?;
    let start = edge.curve.knots[0];
    let end = edge.curve.knots[edge.curve.knots.len() - 1];
    edge.curve = edge.curve.reversed()?;
    let (t0, t1) = (start + end - edge.t1, start + end - edge.t0);
    edge.t0 = t0;
    edge.t1 = t1;
    std::mem::swap(&mut edge.start_vertex_id, &mut edge.end_vertex_id);
    // The coedge keeps its traversal, so its pcurve is unchanged.
    for coedge in solid
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .flat_map(|face| &mut face.loops)
        .flat_map(|loop_record| &mut loop_record.coedges)
    {
        if coedge.edge_id == edge_id {
            coedge.forward = !coedge.forward;
        }
    }
    Ok(())
}

fn mobius_edge(solid: &mut BrepSolid, edge_id: u64, lambda: f64) -> Result<f64, String> {
    let edge = solid
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .ok_or("missing edge")?;
    let (lo, hi) = (edge.t0, edge.t1);
    let old = edge.curve.clone();
    edge.curve = mobius_curve(&old, lo, hi, lambda).ok_or("the substitution's pole crosses a knot")?;
    let [d0, d1] = old.domain()?;
    let map_in = |t: f64| lo + (hi - lo) * mobius_inverse((t - lo) / (hi - lo), lambda);
    edge.t0 = map_in(lo);
    edge.t1 = map_in(hi);
    // Pointwise residual of the substitution, over the edge's own range.
    let mut residual = 0.0f64;
    for sample in 0..=32 {
        let u = sample as f64 / 32.0;
        let tau = u / (lambda + (1.0 - lambda) * u);
        let before = old.evaluate((lo + (hi - lo) * tau).clamp(d0, d1))?;
        let after = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * u)?;
        residual = residual.max(before.sub(after).length());
    }
    for coedge in solid
        .shells
        .iter_mut()
        .flat_map(|shell| &mut shell.faces)
        .flat_map(|face| &mut face.loops)
        .flat_map(|loop_record| &mut loop_record.coedges)
    {
        if coedge.edge_id != edge_id {
            continue;
        }
        let [q0, q1] = coedge.pcurve.domain()?;
        let law = if coedge.forward { lambda } else { 1.0 / lambda };
        coedge.pcurve = mobius_curve(&coedge.pcurve, q0, q1, law).ok_or("pcurve substitution failed")?;
    }
    Ok(residual)
}

fn affine_edge(solid: &mut BrepSolid, edge_id: u64, low: f64, high: f64) -> Result<(), String> {
    let edge = solid
        .edges
        .iter_mut()
        .find(|edge| edge.id == edge_id)
        .ok_or("missing edge")?;
    let [d0, d1] = edge.curve.domain()?;
    let map = |t: f64| low + (high - low) * (t - d0) / (d1 - d0);
    let knots = edge.curve.knots.iter().map(|&knot| map(knot)).collect();
    edge.curve = NurbsCurve::new(edge.curve.degree, knots, edge.curve.control_points.clone())?;
    edge.t0 = map(edge.t0);
    edge.t1 = map(edge.t1);
    Ok(())
}

/// A copy of `solid` with `edges` re-parameterised by `law`, their coedges and
/// pcurves carried along, and the worst pointwise residual of the substitution.
pub(super) fn reparameterise_edges(solid: &BrepSolid, edges: &[u64], law: Law) -> Result<(BrepSolid, f64), String> {
    let mut copy = solid.clone();
    let mut residual = 0.0f64;
    for &edge_id in edges {
        match law {
            Law::Reverse => reverse_edge(&mut copy, edge_id)?,
            Law::Mobius(lambda) => residual = residual.max(mobius_edge(&mut copy, edge_id, lambda)?),
            Law::ReverseMobius(lambda) => {
                reverse_edge(&mut copy, edge_id)?;
                residual = residual.max(mobius_edge(&mut copy, edge_id, lambda)?);
            }
            Law::Affine(low, high) => affine_edge(&mut copy, edge_id, low, high)?,
        }
    }
    Ok((copy, residual))
}

/// A copy of `solid` with the strip's corner vertices moved by `delta` — far
/// below any tolerance, so the same body — which is how the census tells a row
/// that sits on a round-off knife edge from one that does not.
pub(super) fn nudge_vertices(solid: &BrepSolid, boundary: &[(u64, bool)], delta: Vec3) -> BrepSolid {
    let corners: HashSet<u64> = boundary
        .iter()
        .filter_map(|(edge_id, _)| solid.edges.iter().find(|edge| edge.id == *edge_id))
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    let mut copy = solid.clone();
    for vertex in &mut copy.vertices {
        if corners.contains(&vertex.id) {
            vertex.point = vertex.point.add(delta);
        }
    }
    copy
}

pub(super) fn strip_boundary(solid: &BrepSolid, face_id: u64) -> Option<Vec<(u64, bool)>> {
    let (shell, face) = find_face(solid, face_id)?;
    Some(
        solid.shells[shell].faces[face].loops[0]
            .coedges
            .iter()
            .map(|coedge| (coedge.edge_id, coedge.forward))
            .collect(),
    )
}

pub(super) fn bits_hash(solid: &BrepSolid) -> u64 {
    let mut hasher = rustc_hash::FxHasher::default();
    match crate::encode_solid(solid) {
        Ok((data, names)) => {
            for value in data {
                value.to_bits().hash(&mut hasher);
            }
            format!("{names:?}").hash(&mut hasher);
        }
        Err(error) => error.hash(&mut hasher),
    }
    hasher.finish()
}

pub(super) struct Outcome {
    pub(super) verdict: Result<(u64, f64, Vec<Vec3>, bool), String>,
}

fn run(solid: &BrepSolid, shell_index: usize, face_index: usize, face_id: u64, neighbour_ids: &[u64; 4]) -> Outcome {
    let Some(boundary) = strip_boundary(solid, face_id) else {
        return Outcome { verdict: Err("strip vanished".into()) };
    };
    let input_top = max_topology_id(solid);
    let verdict = heal_open_transition_mixed(solid, shell_index, face_index, &boundary, neighbour_ids)
        .map_err(String::from)
        .map(|healed| {
        let volume = crate::solid_mass_properties(&healed).map(|mass| mass.volume).unwrap_or(f64::NAN);
        let corners: Vec<Vec3> = healed
            .vertices
            .iter()
            .filter(|vertex| vertex.id > input_top)
            .map(|vertex| vertex.point)
            .collect();
        let sound = crate::accept_sound(healed.clone(), "open_heal_census").is_ok();
        (bits_hash(&healed), volume, corners, sound)
    });
    Outcome { verdict }
}

pub(super) fn rail_row(solid: &BrepSolid, edge_id: u64) -> serde_json::Value {
    let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id) else {
        return serde_json::json!({ "edge": edge_id, "missing": true });
    };
    let curve = &edge.curve;
    let rational = curve
        .control_points
        .iter()
        .any(|control| (control.w - curve.control_points[0].w).abs() > 1e-12);
    let spans = curve.knots.windows(2).filter(|pair| pair[1] > pair[0]).count();
    let mut slowest = f64::INFINITY;
    let mut fastest = 0.0f64;
    for sample in 0..=32 {
        let t = edge.t0 + (edge.t1 - edge.t0) * sample as f64 / 32.0;
        if let Ok((_, derivative)) = curve.deriv1(t) {
            slowest = slowest.min(derivative.length());
            fastest = fastest.max(derivative.length());
        }
    }
    let [d0, d1] = curve.domain().unwrap_or([f64::NAN, f64::NAN]);
    serde_json::json!({
        "edge": edge_id,
        "degree": curve.degree,
        "spans": spans,
        "rational": rational,
        "speed_ratio": fastest / slowest,
        "length": crate::edge_arc_length(edge).unwrap_or(f64::NAN),
        "full_domain": (edge.t0 - d0).abs() <= 1e-12 * (d1 - d0).abs() && (edge.t1 - d1).abs() <= 1e-12 * (d1 - d0).abs(),
    })
}

/// Distance from each uniform strip sample (the heal's own nine per edge) to
/// each neighbour's carrier, and how many samples pass the marcher's seed gate
/// (`tolerance × 100`) onto BOTH primaries of each opposite pairing.
fn seed_gate_rows(solid: &BrepSolid, boundary: &[(u64, bool)], neighbour_ids: &[u64; 4]) -> serde_json::Value {
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-7).max(1e-9);
    let gate = tolerance * 100.0;
    let mut points = Vec::new();
    for (edge_id, _) in boundary {
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == *edge_id) else {
            continue;
        };
        for sample in 0..=8 {
            if let Ok(point) = edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * sample as f64 / 8.0) {
                points.push(point);
            }
        }
    }
    let surfaces: Vec<Option<NurbsSurface>> = neighbour_ids
        .iter()
        .map(|id| find_face(solid, *id).map(|(shell, face)| solid.shells[shell].faces[face].surface.clone()))
        .collect();
    let mut rows = Vec::new();
    for start in 0..2usize {
        let (Some(first), Some(second)) = (&surfaces[start], &surfaces[start + 2]) else {
            continue;
        };
        let mut passing = 0usize;
        let mut nearest = f64::INFINITY;
        for &point in &points {
            let a = crate::project_point_to_surface(first, point).map(|p| p.distance).unwrap_or(f64::INFINITY);
            let b = crate::project_point_to_surface(second, point).map(|p| p.distance).unwrap_or(f64::INFINITY);
            if a <= gate && b <= gate {
                passing += 1;
            }
            nearest = nearest.min(a.max(b));
        }
        rows.push(serde_json::json!({
            "primaries": [neighbour_ids[start], neighbour_ids[start + 2]],
            "samples": points.len(),
            "passing_gate": passing,
            "gate": gate,
            "nearest_to_both": nearest,
        }));
    }
    serde_json::Value::Array(rows)
}

pub(super) fn compare(original: &Outcome, variant: &Outcome, scale: f64) -> &'static str {
    match (&original.verdict, &variant.verdict) {
        (Ok(a), Ok(b)) if a.0 == b.0 => "bits",
        (Ok(a), Ok(b)) => {
            let corners_agree = a.2.len() == b.2.len()
                && a.2.iter().all(|p| b.2.iter().any(|q| p.sub(*q).length() <= 1e-9 * scale.max(1.0)));
            if corners_agree && (a.1 - b.1).abs() <= 1e-9 * a.1.abs().max(1.0) {
                "geometry"
            } else {
                "moved"
            }
        }
        (Err(a), Err(b)) if a == b => "same-refusal",
        (Err(_), Err(_)) => "other-refusal",
        (Ok(_), Err(_)) => "build->refuse",
        (Err(_), Ok(_)) => "refuse->build",
    }
}

pub(super) fn describe(outcome: &Outcome) -> serde_json::Value {
    match &outcome.verdict {
        Ok((hash, volume, corners, sound)) => serde_json::json!({
            "ok": true,
            "bits": format!("{hash:016x}"),
            "volume": volume,
            "corners": corners.iter().map(|p| [p.x, p.y, p.z]).collect::<Vec<_>>(),
            "sound": sound,
        }),
        Err(message) => serde_json::json!({
            "ok": false,
            "error": message.chars().take(400).collect::<String>(),
        }),
    }
}

pub(super) fn record_open_heal_census(
    directory: &str,
    solid: &BrepSolid,
    shell_index: usize,
    face_index: usize,
    boundary: &[(u64, bool)],
    neighbour_ids: &[u64; 4],
) {
    INSIDE.with(|inside| inside.set(true));
    let face_id = solid.shells[shell_index].faces[face_index].id;
    let scale = solid_model_scale(solid);
    let edges: Vec<u64> = boundary.iter().map(|(edge_id, _)| *edge_id).collect();
    let kinds: Vec<&str> = neighbour_ids
        .iter()
        .map(|id| match find_face(solid, *id) {
            Some((shell, face)) => {
                let surface = &solid.shells[shell].faces[face].surface;
                if plane_of_surface(surface, (scale * 1e-6).max(1e-7), "census").is_ok() {
                    "plane"
                } else if surface.analytic().is_some() {
                    "curved"
                } else {
                    "free-form"
                }
            }
            None => "missing",
        })
        .collect();
    let original = run(solid, shell_index, face_index, face_id, neighbour_ids);
    let mut variants = Vec::new();
    for law in [
        Law::Reverse,
        Law::Mobius(4.0),
        Law::Mobius(0.25),
        Law::ReverseMobius(4.0),
        Law::Affine(-2.0, 5.0),
    ] {
        match reparameterise_edges(solid, &edges, law) {
            Ok((copy, residual)) => {
                let outcome = run(&copy, shell_index, face_index, face_id, neighbour_ids);
                variants.push(serde_json::json!({
                    "law": law.label(),
                    "reparam_residual": residual,
                    "input_validates": copy.validate().is_empty(),
                    "against_original": compare(&original, &outcome, scale),
                    "outcome": describe(&outcome),
                }));
            }
            Err(error) => variants.push(serde_json::json!({ "law": law.label(), "skipped": error })),
        }
    }
    let step = 5e-13 * scale.max(1.0);
    for (label, delta) in [
        ("nudge+x", Vec3::new(step, 0.0, 0.0)),
        ("nudge-x", Vec3::new(-step, 0.0, 0.0)),
        ("nudge+y", Vec3::new(0.0, step, 0.0)),
        ("nudge-y", Vec3::new(0.0, -step, 0.0)),
        ("nudge+z", Vec3::new(0.0, 0.0, step)),
        ("nudge-z", Vec3::new(0.0, 0.0, -step)),
    ] {
        let copy = nudge_vertices(solid, boundary, delta);
        let outcome = run(&copy, shell_index, face_index, face_id, neighbour_ids);
        variants.push(serde_json::json!({
            "law": label,
            "against_original": compare(&original, &outcome, scale),
            "outcome": describe(&outcome),
        }));
    }
    let line = serde_json::json!({
        "lane": "mixed",
        "context": {
            "thread": std::thread::current().name().unwrap_or("").to_string(),
            "args": std::env::args().collect::<Vec<_>>(),
        },
        "strip": face_id,
        "neighbours": neighbour_ids,
        "kinds": kinds,
        "input_validates": solid.validate().is_empty(),
        "rails": edges.iter().map(|edge_id| rail_row(solid, *edge_id)).collect::<Vec<_>>(),
        "seed_gate": seed_gate_rows(solid, boundary, neighbour_ids),
        "original": describe(&original),
        "variants": variants,
    });
    let path = std::path::Path::new(directory).join(format!("census-{}.jsonl", std::process::id()));
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(&path) {
        let _ = writeln!(file, "{line}");
    }
    INSIDE.with(|inside| inside.set(false));
}

/// The ALL-PLANAR open heal reads the strip's corner vertices and its
/// neighbours' planes and never a boundary curve; the census still measures it
/// rather than assume it, on the same exact re-parameterisations.
pub(super) fn record_planar_open_heal(directory: &str, solid: &BrepSolid, face_id: u64) {
    INSIDE.with(|inside| inside.set(true));
    let scale = solid_model_scale(solid);
    let summarise = |body: &BrepSolid| Outcome {
        verdict: delete_face_and_heal_impl(body, face_id).map_err(String::from).map(|healed| {
            let volume = crate::solid_mass_properties(&healed).map(|mass| mass.volume).unwrap_or(f64::NAN);
            (bits_hash(&healed), volume, Vec::new(), true)
        }),
    };
    let original = summarise(solid);
    let edges: Vec<u64> = strip_boundary(solid, face_id)
        .unwrap_or_default()
        .iter()
        .map(|(edge_id, _)| *edge_id)
        .collect();
    let mut variants = Vec::new();
    for law in [Law::Reverse, Law::Mobius(4.0), Law::ReverseMobius(4.0)] {
        match reparameterise_edges(solid, &edges, law) {
            Ok((copy, _)) => {
                let outcome = summarise(&copy);
                variants.push(serde_json::json!({
                    "law": law.label(),
                    "against_original": compare(&original, &outcome, scale),
                }));
            }
            Err(error) => variants.push(serde_json::json!({ "law": law.label(), "skipped": error })),
        }
    }
    let line = serde_json::json!({
        "lane": "planar",
        "context": {
            "thread": std::thread::current().name().unwrap_or("").to_string(),
            "args": std::env::args().collect::<Vec<_>>(),
        },
        "strip": face_id,
        "original": describe(&original),
        "variants": variants,
    });
    let path = std::path::Path::new(directory).join(format!("census-{}.jsonl", std::process::id()));
    if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(&path) {
        let _ = writeln!(file, "{line}");
    }
    INSIDE.with(|inside| inside.set(false));
}
