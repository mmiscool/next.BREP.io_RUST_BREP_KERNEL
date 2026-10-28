//! CAM simulation player rendering (lane C17, row 2): the viewport overlay
//! groups of the simulated stock (with an optional deviation colour map and
//! its legend), the tool with shank and holder at a scrub pose along any
//! tool axis, and collision / gouge markers. Pure functions over the
//! `brep_cam` payloads; the panel in BREP_app pushes the groups through
//! `WidgetRegistry::set_overlay_json`.

use brep_cam::cutter::{ToolPart, ToolProfile};
use brep_cam::model::{Point3, RigidTransform};
use brep_cam::stock_sim::mesh::StockMesh;
use brep_cam::stock_sim::remaining::RemainingStockMesh;
use brep_cam::verify::{DeviationClass, DeviationMap};
use serde_json::{json, Value};

/// Overlay group names.
pub const STOCK_GROUP: &str = "cam-sim-stock";
pub const TOOL_GROUP: &str = "cam-sim-tool";
pub const MARKS_GROUP: &str = "cam-sim-marks";

/// Stock surface colour (no deviation map).
pub const STOCK_COLOR: [f32; 3] = [0.72, 0.74, 0.70];
/// Deviation colours: within tolerance, excess stock, gouge.
pub const WITHIN_COLOR: [f32; 3] = [0.35, 0.75, 0.40];
pub const EXCESS_COLOR: [f32; 3] = [0.25, 0.45, 0.95];
pub const GOUGE_COLOR: [f32; 3] = [0.95, 0.25, 0.20];
/// Marker colours.
pub const COLLISION_COLOR: [f32; 3] = [1.0, 0.3, 0.1];
pub const GOUGE_MARK_COLOR: [f32; 3] = [1.0, 0.1, 0.5];

/// Colour of a deviation value: blended toward the class colour by how far
/// it sits beyond the tolerance (saturated at `scale` beyond it).
pub fn deviation_color(value: f64, map: &DeviationMap, scale: f64) -> [f32; 3] {
    match map.classify(value) {
        DeviationClass::Within => WITHIN_COLOR,
        DeviationClass::Excess => blend(WITHIN_COLOR, EXCESS_COLOR, ((value - map.tolerance) / scale.max(1e-9)).clamp(0.0, 1.0) as f32),
        DeviationClass::Gouge => blend(WITHIN_COLOR, GOUGE_COLOR, ((-value - map.tolerance) / scale.max(1e-9)).clamp(0.0, 1.0) as f32),
    }
}

fn blend(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    if t >= 1.0 {
        return b;
    }
    let t = 0.35 + 0.65 * t;
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
}

/// The legend of a deviation map: stops from the largest gouge to the
/// largest excess with their colours, and the tolerance.
pub fn deviation_legend(map: &DeviationMap, scale: f64) -> Value {
    let (mut lo, mut hi) = (0.0f64, 0.0f64);
    for v in map.values.iter().flatten() {
        lo = lo.min(*v);
        hi = hi.max(*v);
    }
    let stops: Vec<Value> = [lo, -map.tolerance, 0.0, map.tolerance, hi]
        .iter()
        .map(|v| {
            let c = deviation_color(*v, map, scale);
            json!({ "value": v, "color": c })
        })
        .collect();
    json!({ "min": lo, "max": hi, "tolerance": map.tolerance, "scale": scale, "stops": stops })
}

/// The stock surface as a triangle overlay group in MODEL coordinates
/// (`to_model` is the setup's machine-to-model transform). With a deviation
/// map each vertex takes the colour of the column under it; without one the
/// stock colour. An empty mesh removes the group.
pub fn stock_group(mesh: &StockMesh, to_model: &RigidTransform, deviation: Option<(&DeviationMap, f64)>) -> Value {
    if mesh.triangle_count() == 0 {
        return json!({ "name": STOCK_GROUP });
    }
    let n = mesh.triangle_count();
    let mut positions: Vec<f64> = Vec::with_capacity(n * 9);
    let mut colors: Vec<f32> = Vec::with_capacity(n * 9);
    let mut normals: Vec<f64> = Vec::with_capacity(n * 9);
    for i in 0..n {
        let tri = mesh.triangle(i);
        let nrm = tri[1].sub(tri[0]).cross(tri[2].sub(tri[0])).normalized().unwrap_or(Point3::Z);
        let nrm = to_model.apply_vector(nrm);
        for p in tri {
            let c = match deviation {
                Some((map, scale)) => match map.value_at(p.x, p.y) {
                    Some(v) => deviation_color(v, map, scale),
                    None => STOCK_COLOR,
                },
                None => STOCK_COLOR,
            };
            let q = to_model.apply_point(p);
            positions.extend_from_slice(&[q.x, q.y, q.z]);
            colors.extend_from_slice(&c);
            normals.extend_from_slice(&[nrm.x, nrm.y, nrm.z]);
        }
    }
    json!({ "name": STOCK_GROUP, "renderOrder": 6, "tris": { "positions": positions, "colors": colors, "normals": normals } })
}

/// Remaining-stock colours beyond the deviation ones: excess far above the
/// tolerance, and stock below the part (never attributed to a face).
pub const EXCESS_HIGH_COLOR: [f32; 3] = [0.95, 0.80, 0.20];
pub const BELOW_PART_COLOR: [f32; 3] = [0.62, 0.52, 0.80];

/// Remaining-stock bands, by signed remaining material `v` (mm) and the
/// operation tolerance `tol`: gouge red below -tol, within green in ±tol,
/// excess blue up to 10·tol, yellow above (and where the part is out of reach).
pub fn remaining_color(v: f64, tol: f64) -> [f32; 3] {
    if v < -tol {
        GOUGE_COLOR
    } else if v <= tol {
        WITHIN_COLOR
    } else if v <= 10.0 * tol {
        EXCESS_COLOR
    } else {
        EXCESS_HIGH_COLOR
    }
}

/// The remaining-stock legend: the four bands with their edges in mm and
/// colours, the tolerance, the vertex counts per class and per band, and the
/// largest finite excess and the deepest gouge.
pub fn remaining_legend(r: &RemainingStockMesh) -> Value {
    let tol = r.tolerance;
    let (within, excess, gouge) = r.counts();
    let high = (0..r.excess.len()).filter(|&i| !r.is_below_part(i) && r.excess[i] > 10.0 * tol).count();
    json!({
        "tolerance": tol,
        "bands": [
            { "band": "gouge", "to": -tol, "color": GOUGE_COLOR },
            { "band": "within", "from": -tol, "to": tol, "color": WITHIN_COLOR },
            { "band": "excess", "from": tol, "to": 10.0 * tol, "color": EXCESS_COLOR },
            { "band": "excess-high", "from": 10.0 * tol, "color": EXCESS_HIGH_COLOR },
            { "band": "below-part", "color": BELOW_PART_COLOR },
        ],
        "counts": { "within": within, "excess": excess, "gouge": gouge, "excessHigh": high, "belowPart": r.below_part_count() },
        "maxExcess": r.max.max(0.0),
        "minValue": r.min,
        "vertices": r.excess.len(),
    })
}

/// The stock surface coloured by remaining material, vertex by vertex (see
/// [`remaining_color`]), in MODEL coordinates. An empty mesh removes the group.
pub fn remaining_group(r: &RemainingStockMesh, to_model: &RigidTransform) -> Value {
    let mesh = &r.mesh;
    if mesh.triangle_count() == 0 {
        return json!({ "name": STOCK_GROUP });
    }
    let n = mesh.triangle_count();
    let mut positions: Vec<f64> = Vec::with_capacity(n * 9);
    let mut colors: Vec<f32> = Vec::with_capacity(n * 9);
    let mut normals: Vec<f64> = Vec::with_capacity(n * 9);
    for t in mesh.indices.chunks_exact(3) {
        let tri = [mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]];
        let nrm = tri[1].sub(tri[0]).cross(tri[2].sub(tri[0])).normalized().unwrap_or(Point3::Z);
        let nrm = to_model.apply_vector(nrm);
        for (k, p) in tri.into_iter().enumerate() {
            let q = to_model.apply_point(p);
            positions.extend_from_slice(&[q.x, q.y, q.z]);
            let i = t[k] as usize;
            colors.extend_from_slice(&if r.is_below_part(i) { BELOW_PART_COLOR } else { remaining_color(r.excess[i], r.tolerance) });
            normals.extend_from_slice(&[nrm.x, nrm.y, nrm.z]);
        }
    }
    json!({ "name": STOCK_GROUP, "renderOrder": 6, "tris": { "positions": positions, "colors": colors, "normals": normals } })
}

/// The tool's wireframe (cutting part, shank, holder in their colours) with
/// its tip at `tip` along the unit `axis`, in machine coordinates, mapped
/// to the model by `to_model`.
pub fn tool_group(profile: &ToolProfile, tip: Point3, axis: Point3, to_model: &RigidTransform) -> Value {
    const SIDES: usize = 24;
    const CUTTING: [f32; 3] = [0.95, 0.90, 0.35];
    const SHANK: [f32; 3] = [0.75, 0.75, 0.78];
    const HOLDER: [f32; 3] = [0.55, 0.58, 0.62];
    let a = axis.normalized().unwrap_or(Point3::Z);
    let e1 = a.cross(if a.z.abs() < 0.9 { Point3::Z } else { Point3::X }).normalized().unwrap_or(Point3::X);
    let e2 = a.cross(e1);
    let at = |radius: f64, height: f64, angle: f64| -> Point3 {
        to_model.apply_point(tip.add(a.scale(height)).add(e1.scale(radius * angle.cos())).add(e2.scale(radius * angle.sin())))
    };
    let colour = |part: ToolPart| match part {
        ToolPart::Holder => HOLDER,
        ToolPart::Shank => SHANK,
        ToolPart::Cutting => CUTTING,
    };
    let mut positions: Vec<f64> = Vec::new();
    let mut colors: Vec<f32> = Vec::new();
    let mut line = |p: Point3, q: Point3, c: [f32; 3]| {
        positions.extend_from_slice(&[p.x, p.y, p.z, q.x, q.y, q.z]);
        colors.extend_from_slice(&c);
        colors.extend_from_slice(&c);
    };
    for point in &profile.points {
        if point.radius <= 0.0 {
            continue;
        }
        let c = colour(point.part);
        for i in 0..SIDES {
            let a0 = i as f64 / SIDES as f64 * std::f64::consts::TAU;
            let a1 = (i + 1) as f64 / SIDES as f64 * std::f64::consts::TAU;
            line(at(point.radius, point.height, a0), at(point.radius, point.height, a1), c);
        }
    }
    for w in profile.points.windows(2) {
        let c = colour(w[1].part);
        for k in 0..4 {
            let angle = k as f64 * std::f64::consts::FRAC_PI_2;
            line(at(w[0].radius, w[0].height, angle), at(w[1].radius, w[1].height, angle), c);
        }
    }
    // The axis itself, from the tip past the top.
    let top = profile.top_height();
    line(to_model.apply_point(tip), to_model.apply_point(tip.add(a.scale(top * 1.2 + 1.0))), [1.0, 0.95, 0.2]);
    json!({ "name": TOOL_GROUP, "renderOrder": 8, "lines": { "positions": positions, "colors": colors } })
}

/// A collision or gouge marker.
#[derive(Clone, Debug, PartialEq)]
pub struct Marker {
    /// Machine-coordinate point.
    pub at: Point3,
    pub gouge: bool,
}

/// The markers as a point group; none removes the group.
pub fn marks_group(marks: &[Marker], to_model: &RigidTransform) -> Value {
    if marks.is_empty() {
        return json!({ "name": MARKS_GROUP });
    }
    let mut positions: Vec<f64> = Vec::with_capacity(marks.len() * 3);
    let mut colors: Vec<f32> = Vec::with_capacity(marks.len() * 3);
    for m in marks {
        let p = to_model.apply_point(m.at);
        positions.extend_from_slice(&[p.x, p.y, p.z]);
        colors.extend_from_slice(&if m.gouge { GOUGE_MARK_COLOR } else { COLLISION_COLOR });
    }
    json!({ "name": MARKS_GROUP, "renderOrder": 9, "points": { "positions": positions, "colors": colors, "size": 12.0 } })
}

/// ASCII STL of the stock mesh in MODEL coordinates.
pub fn stock_stl(mesh: &StockMesh, to_model: &RigidTransform, name: &str) -> String {
    let mut out = String::with_capacity(mesh.triangle_count() * 160 + 64);
    out.push_str(&format!("solid {name}\n"));
    for i in 0..mesh.triangle_count() {
        let tri = mesh.triangle(i);
        let nrm = to_model.apply_vector(tri[1].sub(tri[0]).cross(tri[2].sub(tri[0])).normalized().unwrap_or(Point3::Z));
        out.push_str(&format!("  facet normal {} {} {}\n    outer loop\n", nrm.x, nrm.y, nrm.z));
        for p in tri {
            let q = to_model.apply_point(p);
            out.push_str(&format!("      vertex {} {} {}\n", q.x, q.y, q.z));
        }
        out.push_str("    endloop\n  endfacet\n");
    }
    out.push_str(&format!("endsolid {name}\n"));
    out
}

/// The simulation's door to the background runner (lane C17b): the app's
/// panel holds the transport and a view of the stock; the runner holds the
/// stock and advances it in bounded chunks.
impl crate::engine_state::EngineState {
    /// Submit one simulation command; `false` when this runner cannot
    /// simulate (the panel then simulates inline).
    pub fn cam_sim_submit(&mut self, job: crate::runner::CamSimJob) -> bool {
        self.runner.submit_cam_sim(job)
    }

    /// The next landed simulation reply, oldest first.
    pub fn cam_sim_poll(&mut self) -> Option<crate::runner::CamSimReply> {
        self.runner.poll_cam_sim()
    }

    /// The newest progress of the chunk in flight.
    pub fn cam_sim_poll_progress(&mut self) -> Option<crate::runner::CamSimProgress> {
        self.runner.poll_cam_sim_progress()
    }

    /// Stop the chunk in flight between two motions where the runner can
    /// (the native thread); the panel drops its reply either way.
    pub fn cam_sim_cancel(&mut self) {
        self.runner.cancel_cam_sim();
    }
}

