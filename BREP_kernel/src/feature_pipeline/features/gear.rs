//! Gear features (`GEAR.*`): one feature per gear type, each building one
//! ordinary closed solid that later features cut, join and fillet.
//!
//! The gear is built in its own frame (axis +z, front face on z = 0, tooth 0
//! centred on +x) and placed by the one `placement` reference: a plane, datum
//! or planar face puts the front face on it with the axis along its normal; a
//! circular edge puts the axis on the circle's axis through its centre; a point
//! moves the origin only. With nothing picked the axis is +Y through the
//! origin, like Primitive Cylinder. `phase` turns the teeth about the axis.
//!
//! Face names are `{id}_{role}{tooth}` (see [`crate::gears::SectionCurve`]),
//! plus `{id}_F` (front) and `{id}_B` (back); a name survives an edit for every
//! tooth that still exists.
use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureResult, Frame};
use crate::gears::{spur_solid, GearData, SpurGear};
use crate::{transform_brep, AffineTransform, BrepSolid, Vec3};

/// Whether `type_code` is a gear feature (`GEAR.<kind>`).
pub fn is_gear_type(type_code: &str) -> bool {
    type_code.starts_with("GEAR.")
}

/// Context-bar applicability: gears are made from the ribbon, never offered from a selection.
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

const INCH: f64 = 25.4;

/// A numeric field, or its schema default when the document leaves it out
/// (a history written by a script names only the fields it changes).
fn number(ctx: &FeatureContext, key: &str) -> Result<f64, String> {
    match ctx.param(key) {
        None | Some(serde_json::Value::Null) => {
            // The feature's own schema default (a rack and a gear differ, e.g. in teeth).
            let schema = match ctx.feature_type.as_str() {
                "GEAR.RACK" => rack::schema(),
                "GEAR.RING" => ring::schema(),
                "GEAR.SPROCKET" => sprocket::schema(),
                "GEAR.HELICAL" => helical::schema(),
                "GEAR.HERRINGBONE" => helical::herringbone_schema(),
                "GEAR.SPLINE_STRAIGHT" => straight_spline::schema(),
                "GEAR.SPLINE_INV" => involute_spline::schema(),
                "GEAR.TIMING_PULLEY" => timing_pulley::schema(),
                "GEAR.RATCHET" => ratchet::schema(),
                "GEAR.GENEVA" => geneva::schema(),
                "GEAR.WORM" => worm::schema(),
                "GEAR.CYCLO_DISC" => cycloidal::disc_schema(),
                "GEAR.CYCLO_RING" => cycloidal::ring_schema(),
                "GEAR.CYCLOIDAL" => cycloidal_gear::schema(),
                "GEAR.BEVEL" => bevel::schema(),
                "GEAR.ELLIPTICAL" => elliptical::schema(),
                "GEAR.WORM_WHEEL" => worm_wheel::schema(),
                _ => spur::schema(),
            };
            let default = schema["inputParamsSchema"][key]["default_value"].as_f64();
            default.ok_or_else(|| format!("missing required param `{key}`"))
        }
        Some(_) => ctx.number(key),
    }
}

/// The module from the pitch system fields (module in model units; a
/// diametral pitch is teeth per inch of pitch diameter, read in a millimetre
/// document).
fn module_of(ctx: &FeatureContext) -> Result<f64, String> {
    let system = ctx.string("pitchSystem").unwrap_or_else(|| "MODULE".into());
    let module = match system.as_str() {
        "MODULE" => number(ctx, "module")?,
        "DIAMETRAL_PITCH" => {
            let dp = number(ctx, "diametralPitch")?;
            if !(dp > 0.0) {
                return Err(format!("The diametral pitch must be greater than zero (it is {dp})."));
            }
            INCH / dp
        }
        "CIRCULAR_PITCH" => number(ctx, "circularPitch")? / std::f64::consts::PI,
        other => return Err(format!("unknown pitch system `{other}`")),
    };
    if !(module > 0.0) || !module.is_finite() {
        return Err(format!("The module must be greater than zero (it is {module})."));
    }
    Ok(module)
}

fn teeth_of(ctx: &FeatureContext) -> Result<usize, String> {
    let teeth = number(ctx, "teeth")?;
    if teeth.fract() != 0.0 || teeth < 3.0 || teeth > 2000.0 {
        return Err(format!("The tooth count must be a whole number from 3 to 2000 (it is {teeth})."));
    }
    Ok(teeth as usize)
}

/// The shared involute tooth fields of a spur-type gear.
fn spur_params(ctx: &FeatureContext) -> Result<SpurGear, String> {
    Ok(SpurGear {
        module: module_of(ctx)?,
        teeth: teeth_of(ctx)?,
        pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
        profile_shift: number(ctx, "profileShift")?,
        addendum: number(ctx, "addendum")?,
        dedendum: number(ctx, "dedendum")?,
        root_fillet: number(ctx, "rootFillet")?,
        backlash: number(ctx, "backlash")?,
        face_width: number(ctx, "faceWidth")?,
    })
}

/// The placement frame of the gear's own (x, y, z = axis) frame in the world.
fn placement_frame(ctx: &FeatureContext) -> Result<Frame, String> {
    let names = common::reference_names(ctx.param("placement"));
    let Some(name) = names.first() else {
        // Axis +Y through the origin, tooth 0 on +X: local y runs along -Z.
        return Ok(Frame {
            origin: Vec3::new(0.0, 0.0, 0.0),
            x_axis: Vec3::new(1.0, 0.0, 0.0),
            y_axis: Vec3::new(0.0, 0.0, -1.0),
            z_axis: Vec3::new(0.0, 1.0, 0.0),
        });
    };
    if let Some(plane) = common::resolve_plane_reference(ctx, name) {
        if let Ok(frame) = plane.frame() {
            return Ok(frame);
        }
    }
    if let Some(edge) = ctx.scene.resolve_edge(name) {
        let curve = common::edge_curve(edge)?;
        let [t0, t1] = curve.domain()?;
        // The tolerance follows the edge's own size, not its distance from the origin.
        let samples: Vec<Vec3> = (0..=8).map(|i| curve.evaluate(t0 + (t1 - t0) * i as f64 / 8.0)).collect::<Result<_, _>>()?;
        let size = common::bounds_of(&samples).map(|(lo, hi)| hi.sub(lo).length()).unwrap_or(0.0);
        if let Some(circle) = crate::recognize_circle(&curve, t0, t1, 1e-7 * size.max(1e-9)) {
            return Frame::from_origin_normal(circle.centre, circle.axis);
        }
        return Err(format!("The placement edge `{name}` is not a circle or arc; pick a circular edge, a plane or a planar face."));
    }
    if let Some(point) = ctx.scene.resolve_point(name) {
        let mut frame = placement_frame_default();
        frame.origin = point;
        return Ok(frame);
    }
    Err(format!("The placement reference `{name}` was not found."))
}

fn placement_frame_default() -> Frame {
    Frame {
        origin: Vec3::new(0.0, 0.0, 0.0),
        x_axis: Vec3::new(1.0, 0.0, 0.0),
        y_axis: Vec3::new(0.0, 0.0, -1.0),
        z_axis: Vec3::new(0.0, 1.0, 0.0),
    }
}

/// Place a gear built in its own frame: turn it by `phase` (degrees) about its
/// axis, then map it onto `frame`.
fn place(solid: &BrepSolid, frame: &Frame, phase_degrees: f64) -> Result<BrepSolid, String> {
    let (s, c) = phase_degrees.to_radians().sin_cos();
    // Columns: the images of the local x and y after the phase turn, and the axis.
    let x = frame.x_axis.scale(c).add(frame.y_axis.scale(s));
    let y = frame.x_axis.scale(-s).add(frame.y_axis.scale(c));
    let z = frame.z_axis;
    let o = frame.origin;
    let affine = AffineTransform::new([
        x.x, y.x, z.x, o.x, //
        x.y, y.y, z.y, o.y, //
        x.z, y.z, z.z, o.z, //
        0.0, 0.0, 0.0, 1.0,
    ])?;
    transform_brep(solid, affine, false)
}

/// Prefix the role names a builder already put on its faces with `{id}_`.
fn prefix_names(solid: &mut BrepSolid, id: &str) -> Result<(), String> {
    for (index, face) in solid.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()).enumerate() {
        // A face a boolean split off without a name gets a stable positional one.
        let role = face.name.take().unwrap_or_else(|| format!("FACE{index}"));
        face.name = Some(format!("{id}_{role}"));
    }
    Ok(())
}

/// Stamp `{id}_{suffix}` on the faces, in the builder's face order.
fn stamp_names(solid: &mut BrepSolid, id: &str, suffixes: &[String]) -> Result<(), String> {
    let faces = &mut solid.shells.get_mut(0).ok_or("gear builder produced no shell")?.faces;
    if faces.len() != suffixes.len() {
        return Err(format!("gear builder produced {} faces for {} names", faces.len(), suffixes.len()));
    }
    for (face, suffix) in faces.iter_mut().zip(suffixes) {
        face.name = Some(format!("{id}_{suffix}"));
    }
    Ok(())
}

/// The gear's derived data as the feature records it (`persistentData.gearData`).
fn gear_data_json(data: &GearData) -> serde_json::Value {
    let mut out = serde_json::Map::new();
    for (key, value) in [
        ("pitchDiameter", data.pitch_diameter),
        ("baseDiameter", data.base_diameter),
        ("tipDiameter", data.tip_diameter),
        ("rootDiameter", data.root_diameter),
        ("formDiameter", data.form_diameter),
        ("toothThickness", data.tooth_thickness),
        ("tipThickness", data.tip_thickness),
        ("minShiftWithoutUndercut", data.min_shift_without_undercut),
    ] {
        // A rack has no diameters: only what the gear type defines is recorded.
        if value.is_finite() && value != 0.0 {
            out.insert(key.into(), serde_json::json!(value));
        }
    }
    out.insert("undercut".into(), serde_json::json!(data.undercut));
    serde_json::Value::Object(out)
}

fn finish(ctx: &FeatureContext, solid: BrepSolid, suffixes: &[String], data: &GearData) -> Result<FeatureResult, String> {
    let phase = number(ctx, "phase")?;
    finish_placed(ctx, solid, suffixes, data, phase, 0.0)
}

/// Name the faces, slide the solid by `shift` along its own x, turn it by
/// `phase` about its axis and place it, then register it (with the boolean).
fn finish_placed(
    ctx: &FeatureContext,
    mut solid: BrepSolid,
    suffixes: &[String],
    data: &GearData,
    phase: f64,
    shift: f64,
) -> Result<FeatureResult, String> {
    if suffixes.is_empty() {
        prefix_names(&mut solid, &ctx.id)?;
    } else {
        stamp_names(&mut solid, &ctx.id, suffixes)?;
    }
    let mut frame = placement_frame(ctx)?;
    frame.origin = frame.origin.add(frame.x_axis.scale(shift));
    let placed = place(&solid, &frame, phase)?;
    let mut result = common::finalize_solid(ctx, placed, &ctx.id);
    if result.error.is_none() {
        let mut persistent = ctx.persistent.clone();
        if !persistent.is_object() {
            persistent = serde_json::json!({});
        }
        persistent["gearData"] = gear_data_json(data);
        result.persistent_data = Some(persistent);
    }
    Ok(result)
}

/// `GEAR.SPUR` — external spur gear.
pub mod spur {
    use super::*;

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let gear = spur_params(ctx)?;
        let (solid, suffixes, data) = spur_solid(&gear).map_err(|refusal| refusal.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        super::pitch_hidden(values)
    }

    pub fn schema() -> serde_json::Value {
        let mut fields = super::involute_fields();
        fields.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), fields.remove("id").unwrap());
        ordered.insert("placement".into(), super::placement_field());
        for (key, value) in fields {
            ordered.insert(key, value);
        }
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.SPUR",
            "shortName": "GEAR.SPUR",
            "longName": "Spur gear",
            "ribbonPath": "Gears/Cylindrical/Spur gear",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.RACK` — straight-toothed rack.
pub mod rack {
    use super::*;
    use crate::gears::{rack_solid, Rack};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(1.0..=10000.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 1 to 10000 (it is {teeth})."));
        }
        let rack = Rack {
            module: module_of(ctx)?,
            teeth: teeth as usize,
            pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            root_fillet: number(ctx, "rootFillet")?,
            backlash: number(ctx, "backlash")?,
            face_width: number(ctx, "faceWidth")?,
            height: number(ctx, "height")?,
            helix_angle: match ctx.string("toothLine").as_deref().unwrap_or("STRAIGHT") {
                "STRAIGHT" => 0.0,
                "HELICAL" => {
                    let sign = match ctx.string("hand").as_deref().unwrap_or("RIGHT") {
                        "RIGHT" => 1.0,
                        "LEFT" => -1.0,
                        other => return Err(format!("unknown hand `{other}`")),
                    };
                    sign * number(ctx, "helixAngle")?.to_radians()
                }
                other => return Err(format!("unknown tooth line `{other}`")),
            },
        };
        let (solid, suffixes, data) = rack_solid(&rack).map_err(|refusal| refusal.to_string())?;
        finish_placed(ctx, solid, &suffixes, &data, 0.0, number(ctx, "shift")?)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        let mut hidden = super::pitch_hidden(values);
        if values.get("toothLine").and_then(serde_json::Value::as_str).unwrap_or("STRAIGHT") == "STRAIGHT" {
            hidden.extend(["helixAngle", "hand"]);
        }
        hidden
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), serde_json::json!({
            "type": "reference_selection",
            "label": "Placement",
            "selectionFilter": ["PLANE", "FACE", "DATUM", "EDGE", "VERTEX"],
            "multiple": false,
            "default_value": null,
            "hint": "Where the rack goes: its front face on a plane or planar face, the pitch line along the placement's X direction from its origin; a point moves the origin. Empty: the rack runs along +X from the origin, extruded along +Y, teeth towards −Z."
        }));
        let involute = super::involute_fields();
        for (key, value) in involute {
            if key == "profileShift" {
                continue;
            }
            let value = if key == "teeth" {
                serde_json::json!({"type": "number", "label": "Teeth", "default_value": 10, "minimum": 1,
                    "hint": "Number of teeth; the rack is teeth × circular pitch long and each end stops in the middle of a tooth space."})
            } else {
                value
            };
            ordered.insert(key, value);
        }
        for (key, value) in super::rack_fields() {
            ordered.insert(key, value);
        }
        ordered.insert("toothLine".into(), serde_json::json!({"type": "options", "label": "Tooth line", "options": ["STRAIGHT", "HELICAL"], "default_value": "STRAIGHT",
            "hint": "STRAIGHT teeth across the rack, or HELICAL (slanted) teeth to mesh with a helical gear; the module and pressure angle are then the normal ones."}));
        ordered.insert("helixAngle".into(), serde_json::json!({"type": "number", "label": "Helix angle", "default_value": 15,
            "hint": "Slant of helical teeth, in degrees (up to 45°): equal to the mating helical gear's."}));
        ordered.insert("hand".into(), serde_json::json!({"type": "options", "label": "Hand", "options": ["RIGHT", "LEFT"], "default_value": "RIGHT",
            "hint": "Hand of helical teeth: a rack meshes with a gear of the same hand."}));
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.RACK",
            "shortName": "GEAR.RACK",
            "longName": "Gear rack",
            "ribbonPath": "Gears/Cylindrical/Gear rack",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.RING` — internal (ring) spur gear.
pub mod ring {
    use super::*;
    use crate::gears::{ring_solid, RingGear};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let gear = RingGear {
            module: module_of(ctx)?,
            teeth: teeth_of(ctx)?,
            pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
            profile_shift: number(ctx, "profileShift")?,
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            root_fillet: number(ctx, "rootFillet")?,
            backlash: number(ctx, "backlash")?,
            face_width: number(ctx, "faceWidth")?,
            rim: number(ctx, "rim")?,
        };
        let hand = match ctx.string("hand").as_deref().unwrap_or("RIGHT") {
            "RIGHT" => crate::gears::Hand::Right,
            "LEFT" => crate::gears::Hand::Left,
            other => return Err(format!("unknown hand `{other}`")),
        };
        let (solid, data) = match ctx.string("toothLine").as_deref().unwrap_or("STRAIGHT") {
            "STRAIGHT" => ring_solid(&gear),
            "HELICAL" => crate::gears::ring_solid_helical(&gear, number(ctx, "helixAngle")?.to_radians(), hand, false),
            "DOUBLE_HELICAL" => crate::gears::ring_solid_helical(&gear, number(ctx, "helixAngle")?.to_radians(), hand, true),
            other => return Err(format!("unknown tooth line `{other}`")),
        }
        .map_err(|refusal| refusal.to_string())?;
        let phase = number(ctx, "phase")?;
        finish_placed(ctx, solid, &[], &data, phase, 0.0)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        let mut hidden = super::pitch_hidden(values);
        if values.get("toothLine").and_then(serde_json::Value::as_str).unwrap_or("STRAIGHT") == "STRAIGHT" {
            hidden.extend(["helixAngle", "hand"]);
        }
        hidden
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        for (key, value) in super::involute_fields() {
            let value = match key.as_str() {
                "teeth" => serde_json::json!({"type": "number", "label": "Teeth", "default_value": 60, "minimum": 6,
                    "hint": "Number of teeth (6 or more); its mating pinion needs fewer."}),
                "profileShift" => serde_json::json!({"type": "number", "label": "Profile shift", "default_value": 0,
                    "hint": "Profile shift coefficient x: the cutter moved outward by x × module (tip and root circles grow by x × module, the tooth spaces widen)."}),
                "rootFillet" => serde_json::json!({"type": "number", "label": "Root fillet", "default_value": 0.38, "minimum": 0,
                    "hint": "Radius of the root fillet × module: an arc tangent to the flank and the root circle."}),
                "backlash" => serde_json::json!({"type": "number", "label": "Backlash", "default_value": 0, "minimum": 0,
                    "hint": "Circumferential backlash added to each tooth space on the pitch circle, in model units."}),
                "addendum" => serde_json::json!({"type": "number", "label": "Addendum", "default_value": 1, "exclusiveMinimum": 0,
                    "hint": "Addendum coefficient: the inner (tip) radius = pitch radius − (addendum − x) × module."}),
                "dedendum" => serde_json::json!({"type": "number", "label": "Dedendum", "default_value": 1.25, "exclusiveMinimum": 0,
                    "hint": "Dedendum coefficient: the root radius = pitch radius + (dedendum + x) × module."}),
                _ => value,
            };
            ordered.insert(key, value);
        }
        ordered.insert("toothLine".into(), serde_json::json!({"type": "options", "label": "Tooth line", "options": ["STRAIGHT", "HELICAL", "DOUBLE_HELICAL"], "default_value": "STRAIGHT",
            "hint": "STRAIGHT (spur) teeth, HELICAL, or DOUBLE_HELICAL (herringbone); for the helical lines the module, pressure angle and coefficients are the normal ones."}));
        ordered.insert("helixAngle".into(), serde_json::json!({"type": "number", "label": "Helix angle", "default_value": 15,
            "hint": "Helix angle on the pitch cylinder, in degrees (0.5° to 45°): equal to its pinion's."}));
        ordered.insert("hand".into(), serde_json::json!({"type": "options", "label": "Hand", "options": ["RIGHT", "LEFT"], "default_value": "RIGHT",
            "hint": "Hand of the helix: an internal gear meshes with an external one of the SAME hand."}));
        ordered.insert("rim".into(), serde_json::json!({"type": "number", "label": "Rim", "default_value": 6, "exclusiveMinimum": 0,
            "hint": "Wall thickness outside the root circle, in model units: the outside diameter is the root diameter + 2 × rim."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.RING",
            "shortName": "GEAR.RING",
            "longName": "Ring gear",
            "ribbonPath": "Gears/Cylindrical/Ring gear",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.SPROCKET` — roller-chain sprocket.
pub mod sprocket {
    use super::*;
    use crate::gears::{sprocket_solid, Sprocket, SprocketForm};

    /// Chain sizes: (option, pitch, roller diameter, inner width), mm. ANSI
    /// sizes are the ISO 606 A series (25 and 35 are bush chains: their
    /// "roller" is the bush).
    pub const CHAINS: [(&str, f64, f64, f64); 19] = [
        ("ANSI 25", 6.35, 3.30, 3.18),
        ("ANSI 35", 9.525, 5.08, 4.77),
        ("ANSI 40", 12.70, 7.92, 7.85),
        ("ANSI 41", 12.70, 7.77, 6.38),
        ("ANSI 50", 15.875, 10.16, 9.40),
        ("ANSI 60", 19.05, 11.91, 12.57),
        ("ANSI 80", 25.40, 15.88, 15.75),
        ("ANSI 100", 31.75, 19.05, 18.90),
        ("ANSI 120", 38.10, 22.23, 25.22),
        ("ANSI 140", 44.45, 25.40, 25.22),
        ("ANSI 160", 50.80, 28.58, 31.55),
        ("ISO 05B", 8.00, 5.00, 3.00),
        ("ISO 06B", 9.525, 6.35, 5.72),
        ("ISO 08B", 12.70, 8.51, 7.75),
        ("ISO 10B", 15.875, 10.16, 9.65),
        ("ISO 12B", 19.05, 12.07, 11.68),
        ("ISO 16B", 25.40, 15.88, 17.02),
        ("ISO 20B", 31.75, 19.05, 19.56),
        ("ISO 24B", 38.10, 25.40, 25.40),
    ];

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let chain = ctx.string("chain").unwrap_or_else(|| "ANSI 40".into());
        let (_, pitch, roller, inner) = CHAINS
            .iter()
            .copied()
            .find(|(name, ..)| *name == chain)
            .ok_or_else(|| format!("unknown chain size `{chain}`"))?;
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(6.0..=300.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 6 to 300 (it is {teeth})."));
        }
        let form = match ctx.string("form").as_deref().unwrap_or("MEAN") {
            "MINIMUM" => SprocketForm::Minimum,
            "MAXIMUM" => SprocketForm::Maximum,
            "MEAN" => SprocketForm::Mean,
            other => return Err(format!("unknown tooth form `{other}`")),
        };
        let typed = number(ctx, "toothWidth")?;
        // ISO 606 simplex tooth width: 0.93 b1 up to 12.7 mm pitch, 0.95 b1 above.
        let width = if typed > 0.0 { typed } else if pitch <= 12.7 { 0.93 * inner } else { 0.95 * inner };
        let sprocket = Sprocket { pitch, roller_diameter: roller, teeth: teeth as usize, form, width };
        let (solid, suffixes, data) = sprocket_solid(&sprocket).map_err(|refusal| refusal.to_string())?;
        let phase = number(ctx, "phase")?;
        finish_placed(ctx, solid, &suffixes, &data, phase, 0.0)
    }

    pub fn schema() -> serde_json::Value {
        let chains: Vec<&str> = CHAINS.iter().map(|(name, ..)| *name).collect();
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("chain".into(), serde_json::json!({"type": "options", "label": "Chain", "options": chains, "default_value": "ANSI 40",
            "hint": "The roller chain: ANSI sizes (the ISO 606 A series) or the ISO 606 B series. It sets the pitch and the roller diameter."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 17, "minimum": 6,
            "hint": "Number of teeth (6 to 300)."}));
        ordered.insert("form".into(), serde_json::json!({"type": "options", "label": "Tooth form", "options": ["MEAN", "MINIMUM", "MAXIMUM"], "default_value": "MEAN",
            "hint": "ISO 606 tooth form: MINIMUM (smallest seating radius, largest flank radius and seating angle), MAXIMUM (the opposite), or MEAN (halfway)."}));
        ordered.insert("toothWidth".into(), serde_json::json!({"type": "number", "label": "Tooth width", "default_value": 0, "minimum": 0,
            "hint": "Width of the teeth across the sprocket, in mm; 0 takes the ISO 606 simplex width for the chain (0.93 or 0.95 × the chain's inner width)."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.SPROCKET",
            "shortName": "GEAR.SPROCKET",
            "longName": "Chain sprocket",
            "ribbonPath": "Gears/Chain and belt/Chain sprocket",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.HELICAL` and `GEAR.HERRINGBONE` — helical and double helical gears
/// (normal-section values: module, pressure angle and coefficients).
pub mod helical {
    use super::*;
    use crate::gears::{helical_solid, Hand, HelicalGear};

    fn build(ctx: &FeatureContext, herringbone: bool) -> Result<FeatureResult, String> {
        let hand = match ctx.string("hand").as_deref().unwrap_or("RIGHT") {
            "RIGHT" => Hand::Right,
            "LEFT" => Hand::Left,
            other => return Err(format!("unknown hand `{other}`")),
        };
        let gear = HelicalGear {
            spur: spur_params(ctx)?,
            helix_angle: number(ctx, "helixAngle")?.to_radians(),
            hand,
            herringbone,
        };
        let (solid, suffixes, data) = helical_solid(&gear).map_err(|refusal| refusal.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx, false).unwrap_or_else(|error| ctx.fail(error))
    }

    pub fn execute_herringbone(ctx: &FeatureContext) -> FeatureResult {
        build(ctx, true).unwrap_or_else(|error| ctx.fail(error))
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        super::pitch_hidden(values)
    }

    fn schema_of(code: &str, long: &str, helix_default: f64) -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        for (key, value) in super::involute_fields() {
            let value = match key.as_str() {
                "module" => serde_json::json!({"type": "number", "label": "Normal module", "default_value": 2, "exclusiveMinimum": 0,
                    "hint": "Module in the normal section (the cutter's module); the transverse module is this / cos(helix angle)."}),
                "pressureAngle" => serde_json::json!({"type": "number", "label": "Normal pressure angle", "default_value": 20,
                    "hint": "Pressure angle in the normal section, in degrees (10° to 35°)."}),
                _ => value,
            };
            ordered.insert(key.clone(), value);
            if key == "teeth" {
                ordered.insert("helixAngle".into(), serde_json::json!({"type": "number", "label": "Helix angle", "default_value": helix_default,
                    "hint": "Helix angle on the pitch cylinder, in degrees (0.5° to 45°). Mating external helical gears have equal angles and opposite hands."}));
                ordered.insert("hand".into(), serde_json::json!({"type": "options", "label": "Hand", "options": ["RIGHT", "LEFT"], "default_value": "RIGHT",
                    "hint": "Hand of the helix (of the first half, for a herringbone)."}));
            }
        }
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": code,
            "shortName": code,
            "longName": long,
            "ribbonPath": format!("Gears/Cylindrical/{long}"),
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }

    pub fn schema() -> serde_json::Value {
        schema_of("GEAR.HELICAL", "Helical gear", 15.0)
    }

    pub fn herringbone_schema() -> serde_json::Value {
        schema_of("GEAR.HERRINGBONE", "Herringbone gear", 30.0)
    }
}

/// `GEAR.SPLINE_STRAIGHT` — straight-sided spline shaft or hub (ISO 14).
pub mod straight_spline {
    use super::*;
    use crate::gears::{straight_spline_hub, straight_spline_shaft, StraightSpline};

    /// ISO 14 sizes: (option, keys, minor d, major D, key width B), mm.
    pub const SIZES: [(&str, usize, f64, f64, f64); 35] = [
        ("Light 6x23x26", 6, 23.0, 26.0, 6.0),
        ("Light 6x26x30", 6, 26.0, 30.0, 6.0),
        ("Light 6x28x32", 6, 28.0, 32.0, 7.0),
        ("Light 8x32x36", 8, 32.0, 36.0, 6.0),
        ("Light 8x36x40", 8, 36.0, 40.0, 7.0),
        ("Light 8x42x46", 8, 42.0, 46.0, 8.0),
        ("Light 8x46x50", 8, 46.0, 50.0, 9.0),
        ("Light 8x52x58", 8, 52.0, 58.0, 10.0),
        ("Light 8x56x62", 8, 56.0, 62.0, 10.0),
        ("Light 8x62x68", 8, 62.0, 68.0, 12.0),
        ("Light 10x72x78", 10, 72.0, 78.0, 12.0),
        ("Light 10x82x88", 10, 82.0, 88.0, 12.0),
        ("Light 10x92x98", 10, 92.0, 98.0, 14.0),
        ("Light 10x102x108", 10, 102.0, 108.0, 16.0),
        ("Light 10x112x120", 10, 112.0, 120.0, 18.0),
        ("Medium 6x11x14", 6, 11.0, 14.0, 3.0),
        ("Medium 6x13x16", 6, 13.0, 16.0, 3.5),
        ("Medium 6x16x20", 6, 16.0, 20.0, 4.0),
        ("Medium 6x18x22", 6, 18.0, 22.0, 5.0),
        ("Medium 6x21x25", 6, 21.0, 25.0, 5.0),
        ("Medium 6x23x28", 6, 23.0, 28.0, 6.0),
        ("Medium 6x26x32", 6, 26.0, 32.0, 6.0),
        ("Medium 6x28x34", 6, 28.0, 34.0, 7.0),
        ("Medium 8x32x38", 8, 32.0, 38.0, 6.0),
        ("Medium 8x36x42", 8, 36.0, 42.0, 7.0),
        ("Medium 8x42x48", 8, 42.0, 48.0, 8.0),
        ("Medium 8x46x54", 8, 46.0, 54.0, 9.0),
        ("Medium 8x52x60", 8, 52.0, 60.0, 10.0),
        ("Medium 8x56x65", 8, 56.0, 65.0, 10.0),
        ("Medium 8x62x72", 8, 62.0, 72.0, 12.0),
        ("Medium 10x72x82", 10, 72.0, 82.0, 12.0),
        ("Medium 10x82x92", 10, 82.0, 92.0, 12.0),
        ("Medium 10x92x102", 10, 92.0, 102.0, 14.0),
        ("Medium 10x102x112", 10, 102.0, 112.0, 16.0),
        ("Medium 10x112x125", 10, 112.0, 125.0, 18.0),
    ];

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let size = ctx.string("size").unwrap_or_else(|| "Light 6x23x26".into());
        let (teeth, minor, major, key_width) = if size == "CUSTOM" {
            let keys = number(ctx, "keys")?;
            if keys.fract() != 0.0 || keys < 2.0 {
                return Err(format!("The number of keys must be a whole number of 2 or more (it is {keys})."));
            }
            (keys as usize, number(ctx, "minorDiameter")?, number(ctx, "majorDiameter")?, number(ctx, "keyWidth")?)
        } else {
            let (_, n, d, big_d, b) = SIZES.iter().copied().find(|(name, ..)| *name == size).ok_or_else(|| format!("unknown spline size `{size}`"))?;
            (n, d, big_d, b)
        };
        let spline = StraightSpline { teeth, minor, major, key_width, length: number(ctx, "length")? };
        let phase = number(ctx, "phase")?;
        match ctx.string("role").as_deref().unwrap_or("SHAFT") {
            "SHAFT" => {
                let (solid, suffixes, data) = straight_spline_shaft(&spline).map_err(|r| r.to_string())?;
                finish_placed(ctx, solid, &suffixes, &data, phase, 0.0)
            }
            "HUB" => {
                let (solid, data) = straight_spline_hub(&spline, number(ctx, "hubDiameter")?).map_err(|r| r.to_string())?;
                finish_placed(ctx, solid, &[], &data, phase, 0.0)
            }
            other => Err(format!("unknown spline role `{other}`")),
        }
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        let mut hidden = Vec::new();
        if values.get("size").and_then(serde_json::Value::as_str).unwrap_or("Light 6x23x26") != "CUSTOM" {
            hidden.extend(["keys", "minorDiameter", "majorDiameter", "keyWidth"]);
        }
        if values.get("role").and_then(serde_json::Value::as_str).unwrap_or("SHAFT") != "HUB" {
            hidden.push("hubDiameter");
        }
        hidden
    }

    pub fn schema() -> serde_json::Value {
        let mut sizes: Vec<&str> = SIZES.iter().map(|(name, ..)| *name).collect();
        sizes.push("CUSTOM");
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("role".into(), serde_json::json!({"type": "options", "label": "Role", "options": ["SHAFT", "HUB"], "default_value": "SHAFT",
            "hint": "SHAFT: the splined shaft (keys on a core). HUB: a ring whose bore carries the matching key slots."}));
        ordered.insert("size".into(), serde_json::json!({"type": "options", "label": "Size", "options": sizes, "default_value": "Light 6x23x26",
            "hint": "ISO 14 size, keys × minor diameter × major diameter in mm (light or medium series), or CUSTOM."}));
        ordered.insert("keys".into(), serde_json::json!({"type": "number", "label": "Keys", "default_value": 6, "minimum": 2, "hint": "Number of keys (CUSTOM)."}));
        ordered.insert("minorDiameter".into(), serde_json::json!({"type": "number", "label": "Minor diameter", "default_value": 23, "exclusiveMinimum": 0, "hint": "Root (minor) diameter, in model units (CUSTOM)."}));
        ordered.insert("majorDiameter".into(), serde_json::json!({"type": "number", "label": "Major diameter", "default_value": 26, "exclusiveMinimum": 0, "hint": "Outside (major) diameter, in model units (CUSTOM)."}));
        ordered.insert("keyWidth".into(), serde_json::json!({"type": "number", "label": "Key width", "default_value": 6, "exclusiveMinimum": 0, "hint": "Width of each key, in model units (CUSTOM)."}));
        ordered.insert("length".into(), serde_json::json!({"type": "number", "label": "Length", "default_value": 30, "exclusiveMinimum": 0, "hint": "Length along the axis, in model units."}));
        ordered.insert("hubDiameter".into(), serde_json::json!({"type": "number", "label": "Hub diameter", "default_value": 40, "exclusiveMinimum": 0, "hint": "Outside diameter of the hub, in model units; larger than the major diameter."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.SPLINE_STRAIGHT",
            "shortName": "GEAR.SPLINE_STRAIGHT",
            "longName": "Straight-sided spline",
            "ribbonPath": "Gears/Splines/Straight-sided spline",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.SPLINE_INV` — involute spline shaft or hub to DIN 5480 (30°, the
/// basic rack haP 0.45 m, hfP 0.55 m, root radius 0.16 m; the profile shift
/// follows from the reference diameter: x m = (dB − m z − 1.1 m) / 2).
pub mod involute_spline {
    use super::*;
    use crate::gears::{ring_solid, spur_solid, RingGear, SpurGear};

    pub const MODULES: [f64; 16] = [0.5, 0.6, 0.75, 0.8, 1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0];

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let module = number(ctx, "module")?;
        if !MODULES.iter().any(|m| (m - module).abs() < 1e-12) {
            return Err(format!("DIN 5480 modules are 0.5, 0.6, 0.75, 0.8, 1, 1.25, 1.5, 1.75, 2, 2.5, 3, 4, 5, 6, 8 and 10 (not {module})."));
        }
        let teeth = teeth_of(ctx)?;
        let reference = number(ctx, "referenceDiameter")?;
        let shift = (reference - module * teeth as f64 - 1.1 * module) / (2.0 * module);
        if !(-0.05 - 1e-9..=0.45 + 1e-9).contains(&shift) {
            return Err(format!(
                "A DIN 5480 spline of {teeth} teeth at module {module} needs a reference diameter from {:.4} to {:.4} (profile shift −0.05 to 0.45 × module); {reference} gives {shift:.4}.",
                module * (teeth as f64 + 1.1 - 0.1),
                module * (teeth as f64 + 1.1 + 0.9)
            ));
        }
        let length = number(ctx, "length")?;
        let phase = number(ctx, "phase")?;
        match ctx.string("role").as_deref().unwrap_or("SHAFT") {
            "SHAFT" => {
                let gear = SpurGear {
                    module,
                    teeth,
                    pressure_angle: 30f64.to_radians(),
                    profile_shift: shift,
                    addendum: 0.45,
                    dedendum: 0.55,
                    root_fillet: 0.16,
                    backlash: 0.0,
                    face_width: length,
                };
                let (solid, suffixes, data) = spur_solid(&gear).map_err(|r| r.to_string())?;
                finish_placed(ctx, solid, &suffixes, &data, phase, 0.0)
            }
            "HUB" => {
                let outside = number(ctx, "hubDiameter")?;
                let root = reference; // df2 = dB
                if !(outside > root) {
                    return Err(format!("The hub's outside diameter ({outside}) must be larger than its root diameter ({root:.4})."));
                }
                let gear = RingGear {
                    module,
                    teeth,
                    pressure_angle: 30f64.to_radians(),
                    profile_shift: shift,
                    addendum: 0.45,
                    dedendum: 0.55,
                    root_fillet: 0.16,
                    backlash: 0.0,
                    face_width: length,
                    rim: (outside - root) / 2.0,
                };
                let (solid, data) = ring_solid(&gear).map_err(|r| r.to_string())?;
                finish_placed(ctx, solid, &[], &data, phase, 0.0)
            }
            other => Err(format!("unknown spline role `{other}`")),
        }
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        if values.get("role").and_then(serde_json::Value::as_str).unwrap_or("SHAFT") != "HUB" { vec!["hubDiameter"] } else { Vec::new() }
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("role".into(), serde_json::json!({"type": "options", "label": "Role", "options": ["SHAFT", "HUB"], "default_value": "SHAFT",
            "hint": "SHAFT: the external spline. HUB: a ring whose bore is the internal spline."}));
        ordered.insert("referenceDiameter".into(), serde_json::json!({"type": "number", "label": "Reference diameter", "default_value": 25, "exclusiveMinimum": 0,
            "hint": "DIN 5480 reference diameter dB in mm (the first number of W 25 × 1.25 × 18)."}));
        ordered.insert("module".into(), serde_json::json!({"type": "number", "label": "Module", "default_value": 1.25, "exclusiveMinimum": 0,
            "hint": "DIN 5480 module: 0.5, 0.6, 0.75, 0.8, 1, 1.25, 1.5, 1.75, 2, 2.5, 3, 4, 5, 6, 8 or 10."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 18, "minimum": 6,
            "hint": "Number of teeth (the last number of W 25 × 1.25 × 18)."}));
        ordered.insert("length".into(), serde_json::json!({"type": "number", "label": "Length", "default_value": 30, "exclusiveMinimum": 0, "hint": "Length along the axis, in model units."}));
        ordered.insert("hubDiameter".into(), serde_json::json!({"type": "number", "label": "Hub diameter", "default_value": 40, "exclusiveMinimum": 0,
            "hint": "Outside diameter of the hub, in model units; larger than the reference diameter."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.SPLINE_INV",
            "shortName": "GEAR.SPLINE_INV",
            "longName": "Involute spline",
            "ribbonPath": "Gears/Splines/Involute spline",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.TIMING_PULLEY` — timing-belt pulley with straight-sided grooves.
pub mod timing_pulley {
    use super::*;
    use crate::gears::{timing_pulley_solid, TimingPulley};

    /// Profiles: (option, pitch p, 2a, bw, hg, side angle each way (deg), rb, rt), mm.
    /// MXL..XXH: ISO 5294 Table 2; T5, T10: manufacturers' pulley drawings to DIN 7721.
    pub const PROFILES: [(&str, f64, f64, f64, f64, f64, f64, f64); 9] = [
        ("MXL", 2.032, 0.508, 0.84, 0.69, 20.0, 0.25, 0.13),
        ("XXL", 3.175, 0.508, 0.96, 0.84, 25.0, 0.35, 0.30),
        ("XL", 5.08, 0.508, 1.32, 1.65, 25.0, 0.41, 0.64),
        ("L", 9.525, 0.762, 3.05, 2.67, 20.0, 1.19, 1.17),
        ("H", 12.7, 1.372, 4.19, 3.05, 20.0, 1.60, 1.60),
        ("XH", 22.225, 2.794, 7.9, 7.14, 20.0, 1.98, 2.39),
        ("XXH", 31.75, 3.048, 12.17, 10.31, 20.0, 3.96, 3.18),
        ("T5", 5.0, 0.84, 1.50, 1.8, 25.0, 0.4, 0.6),
        ("T10", 10.0, 1.86, 3.4, 3.2, 25.0, 0.6, 0.8),
    ];

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let profile = ctx.string("profile").unwrap_or_else(|| "XL".into());
        let (_, pitch, pld, bw, hg, side, rb, rt) =
            PROFILES.iter().copied().find(|(name, ..)| *name == profile).ok_or_else(|| format!("unknown belt profile `{profile}`"))?;
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(6.0..=400.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 6 to 400 (it is {teeth})."));
        }
        let width = number(ctx, "width")?;
        let pulley = TimingPulley {
            pitch,
            teeth: teeth as usize,
            pld,
            bottom_width: bw,
            depth: hg,
            side_angle: side.to_radians(),
            bottom_radius: rb,
            top_radius: rt,
            width,
        };
        let (mut solid, suffixes, data) = timing_pulley_solid(&pulley).map_err(|r| r.to_string())?;
        let flanges = ctx.string("flanges").unwrap_or_else(|| "NONE".into());
        let mut suffixes = suffixes;
        if flanges != "NONE" {
            // Name the pulley's faces now; the union keeps them, and the flanges get theirs.
            for (face, suffix) in solid.shells[0].faces.iter_mut().zip(&suffixes) {
                face.name = Some(suffix.clone());
            }
            let height = number(ctx, "flangeHeight")?;
            let thickness = number(ctx, "flangeThickness")?;
            if !(height > 0.0 && thickness > 0.0) {
                return Err("The flange height and thickness must be greater than zero.".into());
            }
            let radius = data.tip_diameter / 2.0 + height;
            let disc = |z0: f64, name: &str| -> Result<crate::BrepSolid, String> {
                let mut d = crate::make_cylinder_brep(Vec3::new(0.0, 0.0, z0), Vec3::new(0.0, 0.0, 1.0), radius, thickness)?;
                for (face, suffix) in d.shells[0].faces.iter_mut().zip(["OD", "F", "B"]) {
                    face.name = Some(format!("{name}{suffix}"));
                }
                Ok(d)
            };
            let mut parts = vec![solid];
            if flanges == "ONE" || flanges == "BOTH" {
                parts.push(disc(-thickness, "FL1_")?);
            }
            if flanges == "BOTH" {
                parts.push(disc(width, "FL2_")?);
            }
            if !matches!(flanges.as_str(), "ONE" | "BOTH") {
                return Err(format!("unknown flange option `{flanges}`"));
            }
            solid = crate::boolean_operation_nary(&parts, crate::BooleanOperation::Union).map_err(|r| r.to_string())?;
            suffixes = Vec::new();
        }
        let phase = number(ctx, "phase")?;
        finish_placed(ctx, solid, &suffixes, &data, phase, 0.0)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        if values.get("flanges").and_then(serde_json::Value::as_str).unwrap_or("NONE") == "NONE" {
            vec!["flangeHeight", "flangeThickness"]
        } else {
            Vec::new()
        }
    }

    pub fn schema() -> serde_json::Value {
        let profiles: Vec<&str> = PROFILES.iter().map(|(name, ..)| *name).collect();
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("profile".into(), serde_json::json!({"type": "options", "label": "Belt profile", "options": profiles, "default_value": "XL",
            "hint": "Belt tooth profile: MXL, XXL, XL, L, H, XH, XXH (ISO 5294, inch pitches) or T5, T10 (metric, DIN 7721)."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 20, "minimum": 6, "hint": "Number of grooves (6 to 400)."}));
        ordered.insert("width".into(), serde_json::json!({"type": "number", "label": "Width", "default_value": 10, "exclusiveMinimum": 0,
            "hint": "Toothed width in model units: a little more than the belt width."}));
        ordered.insert("flanges".into(), serde_json::json!({"type": "options", "label": "Flanges", "options": ["NONE", "ONE", "BOTH"], "default_value": "NONE",
            "hint": "Flanges that keep the belt on: NONE, ONE (at the front) or BOTH."}));
        ordered.insert("flangeHeight".into(), serde_json::json!({"type": "number", "label": "Flange height", "default_value": 1.5, "exclusiveMinimum": 0,
            "hint": "How far a flange stands above the outside diameter, in model units."}));
        ordered.insert("flangeThickness".into(), serde_json::json!({"type": "number", "label": "Flange thickness", "default_value": 1, "exclusiveMinimum": 0,
            "hint": "Thickness of each flange, in model units (added outside the toothed width)."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.TIMING_PULLEY",
            "shortName": "GEAR.TIMING_PULLEY",
            "longName": "Timing pulley",
            "ribbonPath": "Gears/Chain and belt/Timing pulley",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.RATCHET` — ratchet wheel.
pub mod ratchet {
    use super::*;
    use crate::gears::{ratchet_solid, Ratchet};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(3.0..=360.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 3 to 360 (it is {teeth})."));
        }
        let ratchet = Ratchet {
            teeth: teeth as usize,
            outside: number(ctx, "outsideDiameter")?,
            depth: number(ctx, "toothDepth")?,
            lean: number(ctx, "faceAngle")?.to_radians(),
            symmetric: ctx.string("form").as_deref() == Some("SYMMETRIC"),
            clockwise_locks: ctx.string("direction").as_deref() == Some("CW"),
            width: number(ctx, "width")?,
        };
        let (solid, suffixes, data) = ratchet_solid(&ratchet).map_err(|r| r.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        if values.get("form").and_then(serde_json::Value::as_str) == Some("SYMMETRIC") { vec!["faceAngle", "direction"] } else { Vec::new() }
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("form".into(), serde_json::json!({"type": "options", "label": "Tooth form", "options": ["SAW", "SYMMETRIC"], "default_value": "SAW",
            "hint": "SAW: a locking face and a sloped back (one-way). SYMMETRIC: both faces sloped equally (for a reversible pawl)."}));
        ordered.insert("direction".into(), serde_json::json!({"type": "options", "label": "Locks against", "options": ["CCW", "CW"], "default_value": "CCW",
            "hint": "The turning the teeth stop, seen from the front: counter-clockwise (CCW) or clockwise (CW)."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 24, "minimum": 3, "hint": "Number of teeth (3 to 360)."}));
        ordered.insert("outsideDiameter".into(), serde_json::json!({"type": "number", "label": "Outside diameter", "default_value": 60, "exclusiveMinimum": 0, "hint": "Diameter over the tooth tips, in model units."}));
        ordered.insert("toothDepth".into(), serde_json::json!({"type": "number", "label": "Tooth depth", "default_value": 4, "exclusiveMinimum": 0, "hint": "From the tips down to the roots, in model units."}));
        ordered.insert("faceAngle".into(), serde_json::json!({"type": "number", "label": "Face lean", "default_value": 5, "minimum": 0,
            "hint": "How far the locking face leans back from radial, in degrees (0 to 30): a little lean draws the pawl in under load."}));
        ordered.insert("width".into(), serde_json::json!({"type": "number", "label": "Width", "default_value": 8, "exclusiveMinimum": 0, "hint": "Width along the axis, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.RATCHET",
            "shortName": "GEAR.RATCHET",
            "longName": "Ratchet wheel",
            "ribbonPath": "Gears/Intermittent/Ratchet wheel",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.GENEVA` — external Geneva wheel.
pub mod geneva {
    use super::*;
    use crate::gears::{geneva_sizes, geneva_solid, GenevaWheel};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let slots = number(ctx, "slots")?;
        if slots.fract() != 0.0 || !(3.0..=12.0).contains(&slots) {
            return Err(format!("A Geneva wheel has a whole number of slots from 3 to 12 (it has {slots})."));
        }
        let wheel = GenevaWheel {
            slots: slots as usize,
            centre_distance: number(ctx, "centreDistance")?,
            pin_diameter: number(ctx, "pinDiameter")?,
            clearance: number(ctx, "clearance")?,
            lock_radius: number(ctx, "lockRadius")?,
            width: number(ctx, "width")?,
        };
        let (solid, suffixes, mut data) = geneva_solid(&wheel).map_err(|r| r.to_string())?;
        let (crank, _, _, lock) = geneva_sizes(&wheel);
        // The driver's sizes, for its own construction: crank radius and locking disc.
        data.pitch_diameter = 2.0 * crank;
        data.root_diameter = 2.0 * lock;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("slots".into(), serde_json::json!({"type": "number", "label": "Slots", "default_value": 6, "minimum": 3, "hint": "Number of slots (3 to 12): the wheel turns 1/slots of a turn per driver turn."}));
        ordered.insert("centreDistance".into(), serde_json::json!({"type": "number", "label": "Centre distance", "default_value": 50, "exclusiveMinimum": 0,
            "hint": "From the wheel's axis to the driver's axis, in model units. The wheel's radius is centre distance × cos(180°/slots); the driver's crank radius is × sin(180°/slots)."}));
        ordered.insert("pinDiameter".into(), serde_json::json!({"type": "number", "label": "Pin diameter", "default_value": 5, "exclusiveMinimum": 0, "hint": "Diameter of the driver's pin, in model units."}));
        ordered.insert("clearance".into(), serde_json::json!({"type": "number", "label": "Clearance", "default_value": 0.2, "minimum": 0,
            "hint": "Running clearance, in model units: the slots are the pin diameter plus this wide, and the locking arcs stand half of it off the driver's locking disc."}));
        ordered.insert("lockRadius".into(), serde_json::json!({"type": "number", "label": "Locking disc radius", "default_value": 0, "minimum": 0,
            "hint": "Radius of the driver's locking disc, in model units; 0 takes the crank radius less the pin diameter."}));
        ordered.insert("width".into(), serde_json::json!({"type": "number", "label": "Width", "default_value": 6, "exclusiveMinimum": 0, "hint": "Thickness along the axis, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.GENEVA",
            "shortName": "GEAR.GENEVA",
            "longName": "Geneva wheel",
            "ribbonPath": "Gears/Intermittent/Geneva wheel",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.WORM` — cylindrical worm, Archimedean (ZA) thread.
pub mod worm {
    use super::*;
    use crate::gears::{worm_solid, Hand, Worm};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let starts = number(ctx, "starts")?;
        if starts.fract() != 0.0 || !(1.0..=8.0).contains(&starts) {
            return Err(format!("A worm has a whole number of starts from 1 to 8 (it has {starts})."));
        }
        let module = number(ctx, "module")?;
        let quotient = number(ctx, "diameterQuotient")?;
        if !(quotient > 0.0) {
            return Err(format!("The diameter quotient must be greater than zero (it is {quotient})."));
        }
        let hand = match ctx.string("hand").as_deref().unwrap_or("RIGHT") {
            "RIGHT" => Hand::Right,
            "LEFT" => Hand::Left,
            other => return Err(format!("unknown hand `{other}`")),
        };
        let worm = Worm {
            axial_module: module,
            starts: starts as usize,
            reference_diameter: quotient * module,
            pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            length: number(ctx, "length")?,
            hand,
        };
        let (solid, suffixes, data) = worm_solid(&worm).map_err(|r| r.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("module".into(), serde_json::json!({"type": "number", "label": "Axial module", "default_value": 2, "exclusiveMinimum": 0,
            "hint": "Axial module m: the axial pitch is pi × m; its worm wheel has the same transverse module."}));
        ordered.insert("starts".into(), serde_json::json!({"type": "number", "label": "Starts", "default_value": 1, "minimum": 1,
            "hint": "Number of thread starts (1 to 8): the lead is starts × axial pitch."}));
        ordered.insert("diameterQuotient".into(), serde_json::json!({"type": "number", "label": "Diameter quotient", "default_value": 10, "exclusiveMinimum": 0,
            "hint": "q: the reference diameter is q × module (DIN 3975 lists 6 to 20)."}));
        ordered.insert("pressureAngle".into(), serde_json::json!({"type": "number", "label": "Axial pressure angle", "default_value": 20,
            "hint": "Flank angle in the axial section, in degrees (10° to 35°)."}));
        ordered.insert("addendum".into(), serde_json::json!({"type": "number", "label": "Addendum", "default_value": 1, "exclusiveMinimum": 0, "hint": "× module: the tip diameter is the reference diameter + 2 × addendum × module."}));
        ordered.insert("dedendum".into(), serde_json::json!({"type": "number", "label": "Dedendum", "default_value": 1.2, "exclusiveMinimum": 0, "hint": "× module: the root diameter is the reference diameter − 2 × dedendum × module."}));
        ordered.insert("length".into(), serde_json::json!({"type": "number", "label": "Length", "default_value": 40, "exclusiveMinimum": 0, "hint": "Threaded length along the axis, in model units."}));
        ordered.insert("hand".into(), serde_json::json!({"type": "options", "label": "Hand", "options": ["RIGHT", "LEFT"], "default_value": "RIGHT", "hint": "Hand of the thread."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.WORM",
            "shortName": "GEAR.WORM",
            "longName": "Worm",
            "ribbonPath": "Gears/Worm/Worm",
            "commandSize": "Large",
            "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.CYCLO_DISC` and `GEAR.CYCLO_RING` — cycloidal drive disc and pin ring.
pub mod cycloidal {
    use super::*;
    use crate::gears::{cycloidal_disc_solid, cycloidal_pin_ring, CycloidalDisc};

    fn pins_of(ctx: &FeatureContext) -> Result<usize, String> {
        let pins = number(ctx, "pins")?;
        if pins.fract() != 0.0 || !(4.0..=200.0).contains(&pins) {
            return Err(format!("A cycloidal drive has a whole number of pins from 4 to 200 (it has {pins})."));
        }
        Ok(pins as usize)
    }

    pub fn execute_disc(ctx: &FeatureContext) -> FeatureResult {
        build_disc(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build_disc(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let disc = CycloidalDisc {
            pins: pins_of(ctx)?,
            pin_circle_diameter: number(ctx, "pinCircleDiameter")?,
            pin_diameter: number(ctx, "pinDiameter")?,
            eccentricity: number(ctx, "eccentricity")?,
            clearance: number(ctx, "clearance")?,
            width: number(ctx, "width")?,
        };
        let (solid, suffixes, data) = cycloidal_disc_solid(&disc).map_err(|r| r.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn execute_ring(ctx: &FeatureContext) -> FeatureResult {
        build_ring(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build_ring(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let (solid, data) = cycloidal_pin_ring(
            pins_of(ctx)?,
            number(ctx, "pinCircleDiameter")?,
            number(ctx, "pinDiameter")?,
            number(ctx, "outsideDiameter")?,
            number(ctx, "width")?,
        )
        .map_err(|r| r.to_string())?;
        let phase = number(ctx, "phase")?;
        finish_placed(ctx, solid, &[], &data, phase, 0.0)
    }

    fn common_fields(ordered: &mut serde_json::Map<String, serde_json::Value>) {
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("pins".into(), serde_json::json!({"type": "number", "label": "Pins", "default_value": 12, "minimum": 4,
            "hint": "Number of housing pins N (4 to 200); the disc has N − 1 lobes and the reduction is N − 1 : 1."}));
        ordered.insert("pinCircleDiameter".into(), serde_json::json!({"type": "number", "label": "Pin circle diameter", "default_value": 60, "exclusiveMinimum": 0,
            "hint": "Diameter of the circle through the pin centres, in model units."}));
        ordered.insert("pinDiameter".into(), serde_json::json!({"type": "number", "label": "Pin diameter", "default_value": 5, "exclusiveMinimum": 0, "hint": "Diameter of the pins (or rollers), in model units."}));
    }

    pub fn disc_schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        common_fields(&mut ordered);
        ordered.insert("eccentricity".into(), serde_json::json!({"type": "number", "label": "Eccentricity", "default_value": 1.2, "exclusiveMinimum": 0,
            "hint": "Offset of the disc's centre from the input axis, in model units; under pin circle radius / pins."}));
        ordered.insert("clearance".into(), serde_json::json!({"type": "number", "label": "Clearance", "default_value": 0, "minimum": 0,
            "hint": "Diametral running clearance: the profile stands half of it further off the pins."}));
        ordered.insert("width".into(), serde_json::json!({"type": "number", "label": "Width", "default_value": 8, "exclusiveMinimum": 0, "hint": "Disc thickness, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.CYCLO_DISC", "shortName": "GEAR.CYCLO_DISC", "longName": "Cycloidal disc",
            "ribbonPath": "Gears/Cycloidal/Cycloidal disc", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }

    pub fn ring_schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        common_fields(&mut ordered);
        ordered.insert("outsideDiameter".into(), serde_json::json!({"type": "number", "label": "Outside diameter", "default_value": 80, "exclusiveMinimum": 0,
            "hint": "Outside diameter of the ring, in model units; larger than the pin circle plus a pin."}));
        ordered.insert("width".into(), serde_json::json!({"type": "number", "label": "Width", "default_value": 8, "exclusiveMinimum": 0, "hint": "Ring thickness, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.CYCLO_RING", "shortName": "GEAR.CYCLO_RING", "longName": "Cycloidal pin ring",
            "ribbonPath": "Gears/Cycloidal/Cycloidal pin ring", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.CYCLOIDAL` — cycloidal (clock-style) gear.
pub mod cycloidal_gear {
    use super::*;
    use crate::gears::{cycloidal_gear_solid, CycloidalGear};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let module = number(ctx, "module")?;
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(6.0..=1000.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 6 to 1000 (it is {teeth})."));
        }
        let mate = number(ctx, "mateTeeth")?;
        if !(mate >= 4.0) {
            return Err(format!("The mating gear needs at least 4 teeth (it has {mate})."));
        }
        let gear = CycloidalGear {
            module,
            teeth: teeth as usize,
            // The mate's dedenda are radial when the rolling circle is half its pitch radius.
            generating_radius: module * mate / 4.0,
            tooth_fraction: number(ctx, "toothThickness")?,
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            width: number(ctx, "faceWidth")?,
        };
        let (solid, suffixes, data) = cycloidal_gear_solid(&gear).map_err(|r| r.to_string())?;
        finish(ctx, solid, &suffixes, &data)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("module".into(), serde_json::json!({"type": "number", "label": "Module", "default_value": 1, "exclusiveMinimum": 0, "hint": "Pitch diameter / teeth, in model units."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 60, "minimum": 6, "hint": "Number of teeth (6 to 1000)."}));
        ordered.insert("mateTeeth".into(), serde_json::json!({"type": "number", "label": "Mating teeth", "default_value": 8, "minimum": 4,
            "hint": "Teeth of the gear this one drives or is driven by: the addendum is rolled by a circle of half that gear's pitch radius."}));
        ordered.insert("toothThickness".into(), serde_json::json!({"type": "number", "label": "Tooth thickness", "default_value": 0.5,
            "hint": "Tooth thickness on the pitch circle as a fraction of the circular pitch (0.5 for wheels; pinions are often thinner)."}));
        ordered.insert("addendum".into(), serde_json::json!({"type": "number", "label": "Addendum", "default_value": 1.4, "exclusiveMinimum": 0,
            "hint": "Addendum limit × module; the tooth is pointed (ogival) where its flanks meet first."}));
        ordered.insert("dedendum".into(), serde_json::json!({"type": "number", "label": "Dedendum", "default_value": 1.6, "exclusiveMinimum": 0, "hint": "Dedendum × module."}));
        ordered.insert("faceWidth".into(), serde_json::json!({"type": "number", "label": "Face width", "default_value": 3, "exclusiveMinimum": 0, "hint": "Thickness along the axis, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.CYCLOIDAL", "shortName": "GEAR.CYCLOIDAL", "longName": "Cycloidal gear",
            "ribbonPath": "Gears/Cycloidal/Cycloidal gear", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.BEVEL` — straight bevel gear (spherical involute teeth, 90° shafts).
pub mod bevel {
    use super::*;
    use crate::gears::{bevel_solid, BevelGear};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let mate = number(ctx, "mateTeeth")?;
        if mate.fract() != 0.0 || !(6.0..=2000.0).contains(&mate) {
            return Err(format!("The mating gear's tooth count must be a whole number from 6 to 2000 (it is {mate})."));
        }
        let gear = BevelGear {
            module: module_of(ctx)?,
            teeth: teeth_of(ctx)?,
            mate_teeth: mate as usize,
            pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            face_width: number(ctx, "faceWidth")?,
            backlash: number(ctx, "backlash")?,
        };
        let (solid, data) = bevel_solid(&gear).map_err(|r| r.to_string())?;
        let phase = number(ctx, "phase")?;
        finish_placed(ctx, solid, &[], &data, phase, 0.0)
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        super::pitch_hidden(values)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), serde_json::json!({
            "type": "reference_selection", "label": "Placement",
            "selectionFilter": ["PLANE", "FACE", "DATUM", "EDGE", "VERTEX"], "multiple": false, "default_value": null,
            "hint": "Where the gear's APEX goes, its axis along the placement's normal (or a circular edge's axis); the teeth rise from the apex. A mating gear shares the apex with its axis at 90°. Empty: apex at the origin, axis +Y."
        }));
        for (key, value) in super::involute_fields() {
            if matches!(key.as_str(), "profileShift" | "rootFillet") {
                continue;
            }
            let value = match key.as_str() {
                "module" => serde_json::json!({"type": "number", "label": "Module", "default_value": 2, "exclusiveMinimum": 0,
                    "hint": "Module at the heel (the large end): pitch diameter there / teeth, in model units."}),
                "faceWidth" => serde_json::json!({"type": "number", "label": "Face width", "default_value": 10, "exclusiveMinimum": 0,
                    "hint": "Tooth length along the cone, in model units: at most a third of the cone distance."}),
                _ => value,
            };
            ordered.insert(key.clone(), value);
            if key == "teeth" {
                ordered.insert("mateTeeth".into(), serde_json::json!({"type": "number", "label": "Mating teeth", "default_value": 40, "minimum": 6,
                    "hint": "Teeth of the mating bevel gear (shafts at 90°): pitch cone angle = atan(teeth / mating teeth)."}));
            }
        }
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.BEVEL", "shortName": "GEAR.BEVEL", "longName": "Bevel gear",
            "ribbonPath": "Gears/Bevel and face/Bevel gear", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// `GEAR.ELLIPTICAL` — elliptical (non-circular) gear turning about a focus.
pub mod elliptical {
    use super::*;
    use crate::gears::{elliptical_sizes, elliptical_solid, EllipticalGear};
    use std::f64::consts::{PI, TAU};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let gear = EllipticalGear {
            module: module_of(ctx)?,
            teeth: teeth_of(ctx)?,
            eccentricity: number(ctx, "eccentricity")?,
            pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
            addendum: number(ctx, "addendum")?,
            dedendum: number(ctx, "dedendum")?,
            root_fillet: number(ctx, "rootFillet")?,
            backlash: number(ctx, "backlash")?,
            face_width: number(ctx, "faceWidth")?,
        };
        let (a, e, _, centre_distance) = elliptical_sizes(&gear);
        let phase = number(ctx, "phase")?;
        let (shift, turn) = match ctx.string("position").as_deref().unwrap_or("PIVOT") {
            "PIVOT" => (0.0, phase),
            "MATE" => (centre_distance, mate_phase(phase, a, e)),
            other => return Err(format!("unknown position `{other}`")),
        };
        let (solid, suffixes, data) = elliptical_solid(&gear).map_err(|r| r.to_string())?;
        let mut result = finish_placed(ctx, solid, &suffixes, &data, turn, shift)?;
        if let Some(persistent) = result.persistent_data.as_mut() {
            persistent["gearData"]["semiMajorAxis"] = serde_json::json!(a);
            persistent["gearData"]["centreDistance"] = serde_json::json!(centre_distance);
        }
        Ok(result)
    }

    /// The turn (degrees) of the MATE that meshes with the gear pivoted on the placement and turned by
    /// `phase`: the pitch ellipses touch on the line of centres, so the mate's contact radius is the
    /// centre distance less the gear's, r(π − ψ) = 2a − r(−φ) with r(θ) = a(1 − e²)/(1 + e cos θ), and it
    /// turns the other way (ψ = 0 at φ = 0, ψ = −180° at φ = 180°).
    pub(super) fn mate_phase(phase: f64, a: f64, e: f64) -> f64 {
        if e == 0.0 {
            return -phase;
        }
        let phi = (phase.to_radians() + PI).rem_euclid(TAU) - PI;
        let latus = a * (1.0 - e * e);
        let contact = latus / (1.0 + e * phi.cos());
        let cosine = ((1.0 - latus / (2.0 * a - contact)) / e).clamp(-1.0, 1.0);
        let psi = if phi >= 0.0 { -cosine.acos() } else { cosine.acos() };
        // Whole turns of the gear carry over to the mate.
        psi.to_degrees() - (phase - phi.to_degrees())
    }

    pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
        super::pitch_hidden(values)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), serde_json::json!({
            "type": "reference_selection", "label": "Placement",
            "selectionFilter": ["PLANE", "FACE", "DATUM", "EDGE", "VERTEX"], "multiple": false, "default_value": null,
            "hint": "Where the gear's PIVOT (a focus of its pitch ellipse) goes, its axis along the placement's normal; the near end of the ellipse points along the placement's X. Empty: pivot at the origin, axis +Y."
        }));
        ordered.insert("position".into(), serde_json::json!({"type": "options", "label": "Position", "options": ["PIVOT", "MATE"], "default_value": "PIVOT",
            "hint": "PIVOT: the pivot on the placement. MATE: the meshing partner of a gear with the same fields (phase included) pivoted on the placement: its pivot moved by the centre distance, twice the semi-major axis, along X, and turned the other way to roll with it; an odd tooth count meshes."}));
        for (key, value) in super::involute_fields() {
            if key == "profileShift" {
                continue;
            }
            let value = match key.as_str() {
                "teeth" => serde_json::json!({"type": "number", "label": "Teeth", "default_value": 31, "minimum": 8,
                    "hint": "Number of teeth (8 to 1000); the ellipse is sized so they close. Odd counts let two equal gears mesh."}),
                "backlash" => serde_json::json!({"type": "number", "label": "Backlash", "default_value": 0, "minimum": 0,
                    "hint": "Added to each tooth space along the pitch ellipse, in model units."}),
                _ => value,
            };
            ordered.insert(key.clone(), value);
            if key == "teeth" {
                ordered.insert("eccentricity".into(), serde_json::json!({"type": "number", "label": "Eccentricity", "default_value": 0.25, "minimum": 0,
                    "hint": "Eccentricity of the pitch ellipse (0 to 0.8; 0 is a round gear): the speed ratio of a pair swings between (1 − e)/(1 + e) and its inverse."}));
            }
        }
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.ELLIPTICAL", "shortName": "GEAR.ELLIPTICAL", "longName": "Elliptical gear",
            "ribbonPath": "Gears/Non-circular/Elliptical gear", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

/// The rack's own fields.
fn rack_fields() -> serde_json::Map<String, serde_json::Value> {
    let fields = serde_json::json!({
        "height": {"type": "number", "label": "Height", "default_value": 6, "exclusiveMinimum": 0,
            "hint": "From the pitch line down to the rack's back face, in model units; more than the dedendum."},
        "shift": {"type": "number", "label": "Shift", "default_value": 0,
            "hint": "Slides the rack along its length, in model units (one circular pitch moves it by a whole tooth); use it to mesh with a gear."}
    });
    let serde_json::Value::Object(map) = fields else { unreachable!() };
    map
}

/// The pitch-system fields a choice hides.
fn pitch_hidden(values: &serde_json::Value) -> Vec<&'static str> {
    match values.get("pitchSystem").and_then(serde_json::Value::as_str).unwrap_or("MODULE") {
        "DIAMETRAL_PITCH" => vec!["module", "circularPitch"],
        "CIRCULAR_PITCH" => vec!["module", "diametralPitch"],
        _ => vec!["diametralPitch", "circularPitch"],
    }
}

fn placement_field() -> serde_json::Value {
    serde_json::json!({
        "type": "reference_selection",
        "label": "Placement",
        "selectionFilter": ["PLANE", "FACE", "DATUM", "EDGE", "VERTEX"],
        "multiple": false,
        "default_value": null,
        "hint": "Where the gear goes: a plane or planar face puts its front face there with the axis along the normal; a circular edge puts the axis on the circle's axis; a point moves the origin. Empty: axis +Y through the origin."
    })
}

fn phase_field() -> serde_json::Value {
    serde_json::json!({"type": "number", "label": "Phase", "default_value": 0,
        "hint": "Turns the teeth about the axis, in degrees (tooth 0 starts on the placement's X direction); use it to mesh a mating gear."})
}

/// The shared involute tooth fields, in form order.
fn involute_fields() -> serde_json::Map<String, serde_json::Value> {
    let fields = serde_json::json!({
        "pitchSystem": {"type": "options", "label": "Pitch system", "options": ["MODULE", "DIAMETRAL_PITCH", "CIRCULAR_PITCH"], "default_value": "MODULE",
            "hint": "How the tooth size is given: module (pitch diameter / teeth, in model units), diametral pitch (teeth per inch of pitch diameter, in a millimetre document) or circular pitch (tooth-to-tooth arc on the pitch circle)."},
        "module": {"type": "number", "label": "Module", "default_value": 2, "exclusiveMinimum": 0,
            "hint": "Pitch diameter divided by the tooth count, in model units."},
        "diametralPitch": {"type": "number", "label": "Diametral pitch", "default_value": 12, "exclusiveMinimum": 0,
            "hint": "Teeth per inch of pitch diameter (module = 25.4 / DP mm)."},
        "circularPitch": {"type": "number", "label": "Circular pitch", "default_value": 6.283185307179586, "exclusiveMinimum": 0,
            "hint": "Arc from one tooth to the next on the pitch circle (module = circular pitch / π)."},
        "teeth": {"type": "number", "label": "Teeth", "default_value": 20, "minimum": 3,
            "hint": "Number of teeth (3 or more). Few teeth are built with their real generated undercut."},
        "pressureAngle": {"type": "number", "label": "Pressure angle", "default_value": 20,
            "hint": "Pressure angle of the generating rack, in degrees (10° to 35°; 14.5°, 20° and 25° are common)."},
        "profileShift": {"type": "number", "label": "Profile shift", "default_value": 0,
            "hint": "Profile shift coefficient x: the cutter moved outward by x × module. Positive x thickens the root and removes undercut."},
        "faceWidth": {"type": "number", "label": "Face width", "default_value": 10, "exclusiveMinimum": 0,
            "hint": "Width of the gear along its axis, in model units."},
        "addendum": {"type": "number", "label": "Addendum", "default_value": 1, "exclusiveMinimum": 0,
            "hint": "Addendum coefficient ha*: tip radius = pitch radius + (ha* + x) × module."},
        "dedendum": {"type": "number", "label": "Dedendum", "default_value": 1.25, "exclusiveMinimum": 0,
            "hint": "Dedendum coefficient hf*: root radius = pitch radius − (hf* − x) × module."},
        "rootFillet": {"type": "number", "label": "Root fillet", "default_value": 0.38, "minimum": 0,
            "hint": "Tip radius of the generating cutter as a multiple of the module (ρf*); it shapes the root fillet."},
        "backlash": {"type": "number", "label": "Backlash", "default_value": 0, "minimum": 0,
            "hint": "Circumferential backlash taken off this gear's tooth thickness on the pitch circle, in model units."}
    });
    let serde_json::Value::Object(map) = fields else { unreachable!() };
    map
}


/// `GEAR.WORM_WHEEL` — throated worm wheel, cut by a hob of the mating ZA worm.
pub mod worm_wheel {
    use super::*;
    use crate::gears::{worm_wheel_sizes, worm_wheel_solid, Hand, Worm, WormWheel};

    pub fn execute(ctx: &FeatureContext) -> FeatureResult {
        build(ctx).unwrap_or_else(|error| ctx.fail(error))
    }

    fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
        let module = number(ctx, "module")?;
        let teeth = number(ctx, "teeth")?;
        if teeth.fract() != 0.0 || !(12.0..=400.0).contains(&teeth) {
            return Err(format!("The tooth count must be a whole number from 12 to 400 (it is {teeth})."));
        }
        let starts = number(ctx, "starts")?;
        if starts.fract() != 0.0 || !(1.0..=8.0).contains(&starts) {
            return Err(format!("The worm's starts must be a whole number from 1 to 8 (it is {starts})."));
        }
        let hand = match ctx.string("hand").as_deref().unwrap_or("RIGHT") {
            "RIGHT" => Hand::Right,
            "LEFT" => Hand::Left,
            other => return Err(format!("unknown hand `{other}`")),
        };
        let addendum = number(ctx, "addendum")?;
        let dedendum = number(ctx, "dedendum")?;
        let wheel = WormWheel {
            worm: Worm {
                axial_module: module,
                starts: starts as usize,
                reference_diameter: number(ctx, "diameterQuotient")? * module,
                pressure_angle: number(ctx, "pressureAngle")?.to_radians(),
                // The worm's own tip and root as the wheel's root and tip leave them (0.2 × module clearance).
                addendum: (dedendum - 0.2).max(0.1),
                dedendum: addendum + 0.2,
                length: 1.0,
                hand,
            },
            teeth: teeth as usize,
            addendum,
            dedendum,
            face_width: number(ctx, "faceWidth")?,
            backlash: number(ctx, "backlash")?,
        };
        let (solid, suffixes, data) = worm_wheel_solid(&wheel).map_err(|r| r.to_string())?;
        let (_, centre_distance, _, outside) = worm_wheel_sizes(&wheel);
        let mut result = finish(ctx, solid, &suffixes, &data)?;
        if let Some(persistent) = result.persistent_data.as_mut() {
            persistent["gearData"]["centreDistance"] = serde_json::json!(centre_distance);
            persistent["gearData"]["outsideDiameter"] = serde_json::json!(2.0 * outside);
        }
        Ok(result)
    }

    pub fn schema() -> serde_json::Value {
        let mut ordered = serde_json::Map::new();
        ordered.insert("id".into(), serde_json::json!({"type": "string", "default_value": null, "hint": "Unique identifier for the feature"}));
        ordered.insert("placement".into(), super::placement_field());
        ordered.insert("module".into(), serde_json::json!({"type": "number", "label": "Axial module", "default_value": 2, "exclusiveMinimum": 0,
            "hint": "The worm's axial module (the wheel's transverse module), in model units."}));
        ordered.insert("teeth".into(), serde_json::json!({"type": "number", "label": "Teeth", "default_value": 30, "minimum": 12, "hint": "Wheel teeth (12 to 400)."}));
        ordered.insert("starts".into(), serde_json::json!({"type": "number", "label": "Worm starts", "default_value": 1, "minimum": 1, "hint": "Starts (threads) of the mating worm, 1 to 8."}));
        ordered.insert("diameterQuotient".into(), serde_json::json!({"type": "number", "label": "Worm diameter quotient", "default_value": 10, "exclusiveMinimum": 0,
            "hint": "The mating worm's q: its reference diameter = q × module."}));
        ordered.insert("pressureAngle".into(), serde_json::json!({"type": "number", "label": "Axial pressure angle", "default_value": 20,
            "hint": "The worm's axial pressure angle (ZA: straight flanks in the axial section), 10° to 35°."}));
        ordered.insert("hand".into(), serde_json::json!({"type": "options", "label": "Hand", "options": ["RIGHT", "LEFT"], "default_value": "RIGHT", "hint": "Hand of the worm's thread (the wheel's teeth lean the same way)."}));
        ordered.insert("addendum".into(), serde_json::json!({"type": "number", "label": "Addendum", "default_value": 1, "exclusiveMinimum": 0,
            "hint": "× module, in the mid-plane: the throat diameter is the pitch diameter + 2 × addendum × module."}));
        ordered.insert("dedendum".into(), serde_json::json!({"type": "number", "label": "Dedendum", "default_value": 1.2, "exclusiveMinimum": 0,
            "hint": "× module, in the mid-plane: the hob's tip, the worm's tip plus the clearance."}));
        ordered.insert("faceWidth".into(), serde_json::json!({"type": "number", "label": "Face width", "default_value": 10, "exclusiveMinimum": 0,
            "hint": "Width along the wheel's axis, in model units. It must stay inside the throat (under 1.8 × (worm reference radius − addendum × module)), and the teeth must not come to a point where the throat rises toward the faces."}));
        ordered.insert("backlash".into(), serde_json::json!({"type": "number", "label": "Backlash", "default_value": 0, "minimum": 0,
            "hint": "Axial backlash: the hob's thread is cut this much thicker than the worm's, in model units."}));
        ordered.insert("phase".into(), super::phase_field());
        ordered.insert("boolean".into(), common::optional_boolean_schema());
        serde_json::json!({
            "type": "GEAR.WORM_WHEEL", "shortName": "GEAR.WORM_WHEEL", "longName": "Worm wheel",
            "ribbonPath": "Gears/Worm/Worm wheel", "commandSize": "Large", "displayBuilder": false,
            "inputParamsSchema": serde_json::Value::Object(ordered),
        })
    }
}

