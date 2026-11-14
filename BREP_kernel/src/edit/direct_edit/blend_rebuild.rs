//! A face edit beside a VARIABLE-RADIUS fillet: set the fillet aside, run the
//! edit, and re-make the fillet on the edge the edit leaves, under the same
//! radius law.
//!
//! # Why
//!
//! A variable-radius fillet band is a free-form carrier. `move_faces` and
//! `rotate_faces` re-intersect a moved plane with planes, cylinders and cones
//! only, so a push or a turn of ANY face that borders such a band — the face
//! it is tangent to, or the face it ends on — refuses (`carrier_kind`). The
//! re-cut road cannot stand in either: its tools are bounded by half-spaces
//! and a moved face's trim must be a polygon, and the band's rail is a curve
//! wherever the law is not linear.
//!
//! None of those roads has to learn the band. A fillet is a DERIVED face: it
//! is fully stated by the two faces it rounds between and its radius along
//! their edge. So the edit is performed on the body WITHOUT it:
//!
//! 1. **Read** the band (`read_band_law`): its two planar flanks, the line
//!    they meet in, and the ball radius along that line, measured off both
//!    rails — a rail point at distance `d` from the line is touched by a ball
//!    of radius `d·tan(φ/2)`, `φ` the angle between the flanks on the ball's
//!    side. The reading is checked three ways before it is believed: both
//!    rails must read one radius at every station, the band's own surface must
//!    lie on the ball of that radius at that station, and the radius must vary
//!    (a constant band is the existing roads' to handle). Anything else is not
//!    a band this road knows and the edit's own refusal stands, word for word.
//! 2. **Strip** it with the delete-face heal, which extends the flanks to the
//!    sharp edge the fillet rounded.
//! 3. **Edit** the sharp body through the caller's own road (the re-solve, its
//!    acceptance, the re-cut), unchanged.
//! 4. **Re-make** the fillet on the one edge the two flanks now share, with
//!    the law's stops laid from the same END of the edge they were read from.
//!    The law is carried as `(fraction of the edge, radius)` stops, so an edit
//!    that changes the edge's length stretches the law with it and both end
//!    radii are kept. The new band is then read back and must show the law's
//!    end radii at the ends they belong to; a band that cannot be re-made is
//!    left off and named in the notes, as a constant-radius one already is.
//!
//! This road runs only where the caller's own road has refused, so an edit
//! that builds today builds the same body.

use super::*;
use crate::KernelRefusal;

const OP: &str = "rebuild_displaced_blends";

/// Uniform samples taken along each rail edge (its knots are added to them).
const RAIL_SAMPLES: usize = 32;

/// A face as an edit can find it again: by id on a road that keeps ids, by
/// name on one that re-stamps them.
#[derive(Clone, Debug)]
struct FaceKey {
    id: u64,
    name: Option<String>,
}

impl FaceKey {
    fn of(face: &FaceRecord) -> Self {
        Self { id: face.id, name: face.name.clone() }
    }

    fn label(&self) -> String {
        match &self.name {
            Some(name) => format!("`{name}`"),
            None => format!("face {}", self.id),
        }
    }
}

/// A variable-radius fillet band, read off a body so it can be re-made.
#[derive(Clone, Debug)]
pub(super) struct BandLaw {
    band: u64,
    name: Option<String>,
    /// The two faces the band is tangent to.
    flanks: [FaceKey; 2],
    /// The face the band ends on at the law's start and at its end, where
    /// that is a single plane.
    caps: [Option<FaceKey>; 2],
    /// The ends of the sharp edge the band rounds, law start first.
    ends: [Vec3; 2],
    /// `(fraction along the sharp edge, radius)`, from 0 to 1, linear between.
    stops: Vec<(f64, f64)>,
}

impl BandLaw {
    fn label(&self) -> String {
        match &self.name {
            Some(name) => format!("`{name}`"),
            None => format!("face {}", self.band),
        }
    }

    pub(super) fn radius_at(&self, fraction: f64) -> f64 {
        interpolate(&self.stops, fraction)
    }

}

/// Piecewise-linear reading of sorted `(abscissa, value)` samples, held flat
/// past both ends.
pub(super) fn interpolate(samples: &[(f64, f64)], at: f64) -> f64 {
    if at <= samples[0].0 {
        return samples[0].1;
    }
    for pair in samples.windows(2) {
        if at <= pair[1].0 {
            let width = pair[1].0 - pair[0].0;
            if width <= 0.0 {
                return pair[1].1;
            }
            return pair[0].1 + (pair[1].1 - pair[0].1) * (at - pair[0].0) / width;
        }
    }
    samples[samples.len() - 1].1
}

fn outward_plane(face: &FaceRecord, tolerance: f64) -> Option<(Plane, Vec3)> {
    let plane = plane_of_surface(&face.surface, tolerance, OP).ok()?;
    let outward = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
    Some((plane, outward))
}

/// Read `band_id` as a rolling-ball fillet of varying radius between two
/// planar faces. `Err` says why it is not one.
pub(super) fn read_band_law(solid: &BrepSolid, band_id: u64) -> Result<BandLaw, String> {
    let scale = solid_model_scale(solid);
    let tolerance = (scale * 1e-6).max(1e-9);
    let faces: HashMap<u64, &FaceRecord> =
        solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| (face.id, face)).collect();
    let band = *faces.get(&band_id).ok_or_else(|| format!("no face {band_id}"))?;
    if plane_of_surface(&band.surface, tolerance * 10.0, OP).is_ok() {
        return Err("it is planar".into());
    }
    if band.loops.len() != 1 {
        return Err(format!("it has {} loops", band.loops.len()));
    }
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in faces.values() {
        for coedge in face.loops.iter().flat_map(|wire| &wire.coedges) {
            owners.entry(coedge.edge_id).or_default().push(face.id);
        }
    }

    // Each boundary edge is a RAIL (shared with a plane the band is tangent
    // to along it) or an END (anything else).
    let mut rails: Vec<(u64, Vec<&EdgeRecord>)> = Vec::new();
    let mut end_faces: Vec<u64> = Vec::new();
    for coedge in &band.loops[0].coedges {
        let edge = *edges.get(&coedge.edge_id).ok_or_else(|| format!("edge {} is missing", coedge.edge_id))?;
        if edge.degenerate {
            return Err("it runs out to a point".into());
        }
        let other = owners
            .get(&edge.id)
            .into_iter()
            .flatten()
            .copied()
            .find(|id| *id != band_id)
            .ok_or_else(|| "it closes on itself".to_string())?;
        let neighbour = faces[&other];
        let tangent = outward_plane(neighbour, tolerance * 10.0).is_some_and(|(plane, _)| {
            let Ok([q0, q1]) = coedge.pcurve.domain() else { return false };
            [0.25, 0.5, 0.75].iter().all(|fraction| {
                coedge
                    .pcurve
                    .evaluate(q0 + (q1 - q0) * fraction)
                    .and_then(|uv| band.surface.normal(uv.x, uv.y))
                    .is_ok_and(|normal| normal.cross(plane.normal).length() <= 1e-4)
            })
        });
        if tangent {
            match rails.iter_mut().find(|(id, _)| *id == other) {
                Some((_, list)) => list.push(edge),
                None => rails.push((other, vec![edge])),
            }
        } else if !end_faces.contains(&other) {
            end_faces.push(other);
        }
    }
    if rails.len() != 2 {
        return Err(format!("it is tangent to {} planar faces, not two", rails.len()));
    }
    rails.sort_by_key(|(id, _)| *id);
    let (first, second) = (faces[&rails[0].0], faces[&rails[1].0]);
    let (plane_a, normal_a) = outward_plane(first, tolerance * 10.0).ok_or("a flank is not planar")?;
    let (plane_b, normal_b) = outward_plane(second, tolerance * 10.0).ok_or("a flank is not planar")?;
    let line = intersect_planes(&plane_a, &plane_b).ok_or("its two flanks are parallel")?;
    // The wedge the ball sits in, between the flanks: for a convex edge the
    // material's, for a concave one the void's; the same angle either way.
    let phi = std::f64::consts::PI - normal_a.dot(normal_b).clamp(-1.0, 1.0).acos();
    if !(1e-3..=std::f64::consts::PI - 1e-3).contains(&phi) {
        return Err("its two flanks are all but parallel".into());
    }
    let tan_half = (0.5 * phi).tan();
    let station = |point: Vec3| point.sub(line.point).dot(line.dir);
    let reach = |point: Vec3| {
        let offset = point.sub(line.point);
        offset.sub(line.dir.scale(offset.dot(line.dir))).length()
    };

    // The radius along the line, off each rail.
    let read_rail = |list: &[&EdgeRecord]| -> Result<Vec<(f64, f64)>, String> {
        let mut samples = Vec::new();
        for edge in list {
            let mut parameters: Vec<f64> = (0..=RAIL_SAMPLES)
                .map(|index| edge.t0 + (edge.t1 - edge.t0) * index as f64 / RAIL_SAMPLES as f64)
                .collect();
            let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
            parameters.extend(edge.curve.knots.iter().copied().filter(|knot| *knot > low && *knot < high));
            for parameter in parameters {
                let point = edge.curve.evaluate(parameter)?;
                samples.push((station(point), reach(point) * tan_half));
            }
        }
        samples.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(samples)
    };
    let rail_a = read_rail(&rails[0].1)?;
    let rail_b = read_rail(&rails[1].1)?;
    let largest = rail_a.iter().chain(&rail_b).map(|(_, radius)| *radius).fold(0.0, f64::max);
    let smallest = rail_a.iter().chain(&rail_b).map(|(_, radius)| *radius).fold(f64::INFINITY, f64::min);
    if !(largest.is_finite() && largest > tolerance) {
        return Err("its rails lie on the edge it would round".into());
    }
    let band_tolerance = 1e-5 * largest.max(scale * 1e-3);

    // 1. Both rails read one radius wherever both are there.
    let (a_low, a_high) = (rail_a[0].0, rail_a[rail_a.len() - 1].0);
    for &(at, radius) in &rail_b {
        if at < a_low || at > a_high {
            continue;
        }
        let other = interpolate(&rail_a, at);
        if (other - radius).abs() > band_tolerance {
            return Err(format!(
                "its two rails read different radii ({other:.6} and {radius:.6}), so it is not a \
                 rolling-ball fillet"
            ));
        }
    }

    // The sharp edge's extent: where the line meets the end faces, or, for an
    // end that is not one plane, where the rails stop.
    let mut caps: Vec<(f64, FaceKey)> = Vec::new();
    for id in &end_faces {
        let face = faces[id];
        if let Some(hit) = outward_plane(face, tolerance * 10.0).and_then(|(plane, _)| intersect_line_plane(&line, &plane)) {
            caps.push((station(hit), FaceKey::of(face)));
        }
    }
    caps.sort_by(|a, b| a.0.total_cmp(&b.0));
    let all_low = a_low.min(rail_b[0].0);
    let all_high = a_high.max(rail_b[rail_b.len() - 1].0);
    let (low, high, cap_keys) = match caps.as_slice() {
        [(low, first), (high, second)] if end_faces.len() == 2 && high - low > tolerance => {
            (*low, *high, [Some(first.clone()), Some(second.clone())])
        }
        _ => (all_low, all_high, [None, None]),
    };
    if high - low <= tolerance {
        return Err("it has no length along its edge".into());
    }

    // 2. The band's own surface lies on the ball of that radius. The ball's
    // centre is off the line along the flanks' bisector, on the side the rails
    // are on.
    let merged = {
        let mut merged: Vec<(f64, f64)> = rail_a.iter().chain(&rail_b).copied().collect();
        merged.sort_by(|a, b| a.0.total_cmp(&b.0));
        merged
    };
    let bisector = normal_a.add(normal_b).normalized().map_err(|_| "its flanks face each other".to_string())?;
    let [u0, u1] = band.surface.domain_u()?;
    let [v0, v1] = band.surface.domain_v()?;
    let mut on_ball = [0usize; 2];
    let mut read = 0usize;
    for i in 1..6 {
        for j in 1..6 {
            let point = band.surface.evaluate(u0 + (u1 - u0) * i as f64 / 6.0, v0 + (v1 - v0) * j as f64 / 6.0)?;
            let at = station(point);
            if at < low || at > high {
                continue;
            }
            read += 1;
            let radius = interpolate(&merged, at);
            let foot = line.point.add(line.dir.scale(at));
            for (slot, side) in [-1.0, 1.0].into_iter().enumerate() {
                let centre = foot.add(bisector.scale(side * radius / (0.5 * phi).sin()));
                if (point.sub(centre).length() - radius).abs() <= band_tolerance {
                    on_ball[slot] += 1;
                }
            }
        }
    }
    if read == 0 || on_ball.iter().all(|count| *count != read) {
        return Err("its surface is not the envelope of a ball rolling along its edge".into());
    }

    // 3. The radius varies: a constant band is not this road's.
    if largest - smallest <= 1e-6 * largest {
        return Err("its radius is constant".into());
    }

    // The law as (fraction, radius) stops over the sharp edge.
    let span = high - low;
    let mut stops: Vec<(f64, f64)> = Vec::new();
    for &(at, radius) in &merged {
        let fraction = (at - low) / span;
        if !(-1e-9..=1.0 + 1e-9).contains(&fraction) {
            continue;
        }
        let fraction = fraction.clamp(0.0, 1.0);
        match stops.last_mut() {
            Some(last) if fraction - last.0 <= 1e-9 => last.1 = 0.5 * (last.1 + radius),
            _ => stops.push((fraction, radius)),
        }
    }
    if stops.len() < 2 {
        return Err("its rails do not span its edge".into());
    }
    if stops[0].0 > 0.0 {
        stops.insert(0, (0.0, interpolate(&merged, low)));
    }
    if stops[stops.len() - 1].0 < 1.0 {
        stops.push((1.0, interpolate(&merged, high)));
    }
    let stops = simplify(&stops, 5e-4 * largest, band_tolerance);
    if stops.iter().any(|(_, radius)| !(radius.is_finite() && *radius > 0.0)) {
        return Err("its radius runs out to zero".into());
    }

    Ok(BandLaw {
        band: band_id,
        name: band.name.clone(),
        flanks: [FaceKey::of(first), FaceKey::of(second)],
        caps: cap_keys,
        ends: [line.point.add(line.dir.scale(low)), line.point.add(line.dir.scale(high))],
        stops,
    })
}

/// The law's stops, out of dense rail readings.
///
/// The fillet lane's law is linear between stops, and its band is a fitted
/// surface: its rail reads the law to about `fit` along a straight run and
/// rounds each corner of the law by more (1.5e-4 of the radius, measured on a
/// law that turns from rising to falling). Reading that rounding back as law
/// would re-make the fillet from a rounded law, rounded again. So the
/// readings are cut only where they leave a straight line by more than
/// `tolerance`, a line is fitted through each run between cuts, and each stop
/// is put where two neighbouring lines MEET — which is where the law's own
/// stop was, whether or not a reading fell on it.
fn simplify(stops: &[(f64, f64)], tolerance: f64, fit: f64) -> Vec<(f64, f64)> {
    let mut keep = vec![false; stops.len()];
    keep[0] = true;
    keep[stops.len() - 1] = true;
    let mut spans = vec![(0usize, stops.len() - 1)];
    while let Some((from, to)) = spans.pop() {
        let (a, b) = (stops[from], stops[to]);
        let mut worst: Option<(usize, f64)> = None;
        for index in from + 1..to {
            let (at, radius) = stops[index];
            let chord = a.1 + (b.1 - a.1) * (at - a.0) / (b.0 - a.0).max(1e-300);
            let off = (radius - chord).abs();
            if off > tolerance && worst.is_none_or(|(_, best)| off > best) {
                worst = Some((index, off));
            }
        }
        if let Some((index, _)) = worst {
            keep[index] = true;
            spans.push((from, index));
            spans.push((index, to));
        }
    }
    let cuts: Vec<usize> = (0..stops.len()).filter(|index| keep[*index]).collect();
    // One line `radius = base + slope·fraction` per run: least squares through
    // the run's readings, refitted twice without the ones that sit off it by
    // more than `fit` (the readings on the rounded corner beside a cut), or
    // the chord where the run is too short to fit.
    let least_squares = |readings: &[(f64, f64)]| -> Option<(f64, f64)> {
        let count = readings.len() as f64;
        let mean_at = readings.iter().map(|stop| stop.0).sum::<f64>() / count;
        let mean_radius = readings.iter().map(|stop| stop.1).sum::<f64>() / count;
        let spread = readings.iter().map(|stop| (stop.0 - mean_at).powi(2)).sum::<f64>();
        if readings.len() < 4 || spread <= 1e-18 {
            return None;
        }
        let slope =
            readings.iter().map(|stop| (stop.0 - mean_at) * (stop.1 - mean_radius)).sum::<f64>() / spread;
        Some((mean_radius - slope * mean_at, slope))
    };
    let lines: Vec<Option<(f64, f64)>> = cuts
        .windows(2)
        .map(|run| {
            let mut readings: Vec<(f64, f64)> = stops[run[0]..=run[1]].to_vec();
            let mut line = least_squares(&readings)?;
            for _ in 0..2 {
                let on_line: Vec<(f64, f64)> = readings
                    .iter()
                    .copied()
                    .filter(|stop| (line.0 + line.1 * stop.0 - stop.1).abs() <= fit)
                    .collect();
                match least_squares(&on_line) {
                    Some(refitted) => {
                        line = refitted;
                        readings = on_line;
                    }
                    None => break,
                }
            }
            Some(line)
        })
        .collect();
    // A run too short to fit is a rounded corner cut twice, not a piece of
    // the law: the runs either side of it meet across it.
    let fitted: Vec<(usize, (f64, f64))> =
        lines.iter().enumerate().filter_map(|(index, line)| line.map(|line| (index, line))).collect();
    if fitted.is_empty() {
        return cuts.iter().map(|cut| stops[*cut]).collect();
    }
    let mut law = Vec::with_capacity(fitted.len() + 1);
    law.push((0.0, fitted[0].1 .0));
    for pair in fitted.windows(2) {
        let ((first, before), (second, after)) = (pair[0], pair[1]);
        let (low, high) = (stops[cuts[first]].0, stops[cuts[second + 1]].0);
        let meet = (after.0 - before.0) / (before.1 - after.1);
        if meet.is_finite() && meet > low && meet < high {
            law.push((meet, before.0 + before.1 * meet));
        } else {
            law.push(stops[cuts[first + 1]]);
            if second > first + 1 {
                law.push(stops[cuts[second]]);
            }
        }
    }
    let last = fitted[fitted.len() - 1].1;
    law.push((1.0, last.0 + last.1));
    law
}

/// A radius as a note shows it: without the last digits of the reading.
fn shown(radius: f64) -> f64 {
    (radius * 1e6).round() / 1e6
}

/// The faces of `body` that are `key`'s face: the one with its id where the
/// road kept ids, otherwise every face under its name.
fn find_faces<'a>(body: &'a BrepSolid, key: &FaceKey, ids_kept: bool) -> Vec<&'a FaceRecord> {
    let all = || body.shells.iter().flat_map(|shell| &shell.faces);
    if ids_kept {
        if let Some(face) = all().find(|face| face.id == key.id && face.name == key.name) {
            return vec![face];
        }
    }
    match &key.name {
        Some(name) => all().filter(|face| face.name.as_deref() == Some(name)).collect(),
        None => Vec::new(),
    }
}

fn edges_of(faces: &[&FaceRecord]) -> HashSet<u64> {
    faces
        .iter()
        .flat_map(|face| face.loops.iter().flat_map(|wire| &wire.coedges).map(|coedge| coedge.edge_id))
        .collect()
}

/// Round the edge `law`'s two flanks share in `body` under `law`.
///
/// `ids_kept` says the road that made `body` kept face ids, so a face may be
/// found by id; a re-cut re-stamps names onto new ids and its faces are found
/// by name alone.
pub(super) fn remake_band(
    body: &BrepSolid,
    law: &BandLaw,
    ids_kept: bool,
    entry: &str,
) -> Result<BrepSolid, String> {
    let scale = solid_model_scale(body);
    let first = find_faces(body, &law.flanks[0], ids_kept);
    let second = find_faces(body, &law.flanks[1], ids_kept);
    for (found, key) in [(&first, &law.flanks[0]), (&second, &law.flanks[1])] {
        if found.is_empty() {
            return Err(match key.name {
                Some(_) => format!("{} is no longer on the body", key.label()),
                None => format!("{} has no name to find it by", key.label()),
            });
        }
    }
    let (first_edges, second_edges) = (edges_of(&first), edges_of(&second));
    let mut shared = first_edges.intersection(&second_edges).copied();
    let edge_id = match (shared.next(), shared.next()) {
        (Some(edge), None) => edge,
        (None, _) => {
            return Err(format!(
                "{} and {} no longer meet, so there is no edge left to round",
                law.flanks[0].label(),
                law.flanks[1].label()
            ))
        }
        _ => {
            return Err(format!(
                "{} and {} now meet in more than one edge",
                law.flanks[0].label(),
                law.flanks[1].label()
            ))
        }
    };
    let edge = body.edges.iter().find(|edge| edge.id == edge_id).ok_or("the shared edge is missing")?;
    let start = edge.curve.evaluate(edge.t0)?;
    let end = edge.curve.evaluate(edge.t1)?;
    let middle = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1))?;

    // Which end of the new edge the law starts at: the end on the face the
    // band started on, where both end faces can be told apart on the new
    // body; otherwise the end nearer to where the law started.
    let on_cap = |key: &Option<FaceKey>, vertex: u64| -> Option<bool> {
        let faces = find_faces(body, key.as_ref()?, ids_kept);
        if faces.is_empty() {
            return None;
        }
        let edges = edges_of(&faces);
        Some(body.edges.iter().any(|other| {
            edges.contains(&other.id) && (other.start_vertex_id == vertex || other.end_vertex_id == vertex)
        }))
    };
    let by_caps = match (
        on_cap(&law.caps[0], edge.start_vertex_id),
        on_cap(&law.caps[0], edge.end_vertex_id),
        on_cap(&law.caps[1], edge.start_vertex_id),
        on_cap(&law.caps[1], edge.end_vertex_id),
    ) {
        (Some(true), Some(false), Some(false), Some(true)) => Some(false),
        (Some(false), Some(true), Some(true), Some(false)) => Some(true),
        _ => None,
    };
    let reversed = by_caps.unwrap_or_else(|| {
        let as_laid = start.sub(law.ends[0]).length() + end.sub(law.ends[1]).length();
        let turned = start.sub(law.ends[1]).length() + end.sub(law.ends[0]).length();
        turned < as_laid
    });
    let stops: Vec<(f64, f64)> = if reversed {
        law.stops.iter().rev().map(|(fraction, radius)| (1.0 - fraction, *radius)).collect()
    } else {
        law.stops.clone()
    };

    let before: HashSet<u64> = body.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
    let mut rebuilt = crate::fillet_edges_variable(body, &[middle], None, &stops, false, law.name.as_deref())
        .map_err(|error| error.to_string())?;
    let tolerance = (scale * 1e-6).max(1e-9);
    let mut new_bands: Vec<u64> = Vec::new();
    for face in rebuilt.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        if !before.contains(&face.id) && plane_of_surface(&face.surface, tolerance * 10.0, OP).is_err() {
            face.name = law.name.clone();
            new_bands.push(face.id);
        }
    }
    let issues = rebuilt.validate();
    if !issues.is_empty() {
        return Err(format!("the new fillet is invalid: {issues:?}"));
    }
    let rebuilt = crate::accept_sound(rebuilt, entry).map_err(|error| error.to_string())?;

    // Read the new band back: its end radii must be the law's, at the ends
    // they belong to. A band this reader cannot read is not judged by it.
    if let [band] = new_bands.as_slice() {
        if let Ok(read) = read_band_law(&rebuilt, *band) {
            let turned = read.ends[0].sub(if reversed { end } else { start }).length()
                > read.ends[1].sub(if reversed { end } else { start }).length();
            let largest = law.stops.iter().map(|(_, radius)| *radius).fold(0.0, f64::max);
            for fraction in [0.0, 0.5, 1.0] {
                let got = read.radius_at(if turned { 1.0 - fraction } else { fraction });
                let want = law.radius_at(fraction);
                if (got - want).abs() > 1e-4 * largest {
                    return Err(format!(
                        "the new fillet reads radius {got:.6} at {fraction} of its edge where the \
                         law gives {want:.6}"
                    ));
                }
            }
        }
    }
    Ok(rebuilt)
}

/// Run a face edit past the variable-radius fillets beside the selection.
///
/// `edit` is the caller's whole road for this motion — re-solve, acceptance,
/// re-cut — taking the body to edit and returning the edited body with its
/// notes; `ids_kept` is told from the faces it returns. It has already refused
/// `solid` itself when this is called.
///
/// * `None`: no variable-radius fillet borders the selection, so this road has
///   nothing to offer and the caller's refusal stands as it is.
/// * `Some(Err(_))`: there is one, and the edit still could not be made
///   without it; the refusal says which step declined.
/// * `Some(Ok((body, notes)))`: the edited body, each fillet re-made under its
///   radius law, or named in `notes` with the reason where it could not be.
pub fn rebuild_displaced_blends<E>(
    solid: &BrepSolid,
    moved: &[u64],
    entry: &str,
    edit: E,
) -> Option<Result<(BrepSolid, Vec<String>), KernelRefusal>>
where
    E: FnOnce(&BrepSolid) -> Result<(BrepSolid, Vec<String>), KernelRefusal>,
{
    let moved_set: HashSet<u64> = moved.iter().copied().collect();
    let mut moved_edges: HashSet<u64> = HashSet::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if moved_set.contains(&face.id) {
            moved_edges.extend(face.loops.iter().flat_map(|wire| &wire.coedges).map(|coedge| coedge.edge_id));
        }
    }
    let mut laws: Vec<BandLaw> = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| {
            !moved_set.contains(&face.id)
                && face
                    .loops
                    .iter()
                    .flat_map(|wire| &wire.coedges)
                    .any(|coedge| moved_edges.contains(&coedge.edge_id))
        })
        .filter_map(|face| read_band_law(solid, face.id).ok())
        .collect();
    if laws.is_empty() {
        return None;
    }
    laws.sort_by_key(|law| law.band);

    Some((|| {
        let mut sharp = solid.clone();
        for law in &laws {
            sharp = super::delete_face::delete_face_and_heal_impl(&sharp, law.band).map_err(|error| {
                error.with_message(|message| {
                    format!(
                        "{entry}: variable-radius fillet {} could not be set aside for the edit — {message}",
                        law.label()
                    )
                })
            })?;
        }
        let sharp = crate::accept_sound(sharp, entry).map_err(|error| {
            error.with_message(|message| {
                format!("{entry}: the body without its variable-radius fillets is not sound — {message}")
            })
        })?;
        let sharp_ids: HashSet<u64> =
            sharp.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
        let (mut body, mut notes) = edit(&sharp)?;
        // A road that re-solves keeps every face under its id; a re-cut mints
        // its faces anew, and then only names say which is which.
        let ids_kept = body
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .filter(|face| sharp_ids.contains(&face.id))
            .count()
            == sharp_ids.len();
        for law in &laws {
            let (from, to) = (shown(law.stops[0].1), shown(law.stops[law.stops.len() - 1].1));
            match remake_band(&body, law, ids_kept, entry) {
                Ok(rebuilt) => {
                    body = rebuilt;
                    notes.push(format!(
                        "{entry}: variable-radius fillet {} was set aside for the edit and re-made on \
                         the edge it leaves, under the same radius law ({from} to {to} along the edge)",
                        law.label()
                    ));
                }
                Err(reason) => notes.push(format!(
                    "{entry}: variable-radius fillet {} (radius {from} to {to}) was lost: it was set \
                     aside for the edit and could not be re-made ({reason})",
                    law.label()
                )),
            }
        }
        Ok((body, notes))
    })())
}

/// [`rebuild_displaced_blends`]'s refusal, as context on the refusal the
/// caller's own road gave.
pub fn with_blend_rebuild_decline(refusal: KernelRefusal, decline: &KernelRefusal) -> KernelRefusal {
    refusal.with_message(|message| {
        format!(
            "{message}. The edit was also tried with the variable-radius fillet beside the selection \
             set aside, to be re-made afterwards, and that declined: {decline}"
        )
    })
}
