//! APAT — Pattern Component: copies of one or more placed components, laid out
//! like the part Pattern feature (LINEAR or CIRCULAR, the same inputs).
//!
//! The SEED is an ordinary ACOMP the user mates into place. Every other
//! instance is a component record this feature emits itself, placed at
//! `M_n · seed pose` — so it follows the seed, needs no constraints of its own,
//! and is counted by the BOM like any placed part. Instance ids are
//! `{APAT id}.{seed id}.i{n}` (`APAT1.ACOMP3.i2`): `n` is the instance's place in
//! the pattern (the seed is 1), so a suppression or a BOM attribute keyed to it
//! stays on the same place when the count changes. `suppressed` lists those
//! numbers; a number past the count is dormant.
//!
//! The record remembers `M_n` ([`PatternFollow`]); the assembly tail re-poses
//! every instance after the solve, so instances sit on the seed's SOLVED pose
//! in the same run, not one run behind.
//!
//! HOLES mode follows a Hole feature of another component (the DRIVER): the
//! user picks one hole wall (`ACOMP1:S2:P3_Hole_S`) and mates the seed to that
//! hole; every other hole of the same feature gets an instance at the seed's
//! offset from the picked one. The holes are read from the driver part's own
//! snapshot by name (`{sketch}:P{pid}_Hole_{wall}`, a part Pattern copy adding
//! `::{pattern}_{k}`), so a part edit that adds, removes or moves holes adds,
//! removes or moves instances on the next run. The instance number is the
//! hole's key, `1000·(pid + 1) + k` (k = 1 for a hole that is not a pattern
//! copy), so a suppression or a BOM attribute stays on the same hole whatever
//! other holes come and go. Each instance's offset is kept in the driver's
//! frame (the axis of its hole against the picked one: the translation, and
//! the least rotation turning one axis onto the other) and the tail applies it
//! through the driver's solved pose.
//!
//! PATTERN mode follows a part Pattern of the driver the same way: the user
//! picks a face of the patterned feature (on the source or on any copy,
//! `Lug_PX::PC_3`), and every copy of that face gets an instance, numbered as
//! the part numbers its copies (the source is 1). Each copy's map is fitted on
//! its face's carrier control points, which a Pattern copy carries
//! transformed in the same order, and checked on all of them.

use crate::feature_pipeline::component::{compose, create_component, PatternFollow};
use crate::feature_pipeline::features::pattern;
use crate::feature_pipeline::{parts_library, FeatureContext, FeatureResult};
use crate::{restore_solids, AffineTransform};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// `APAT<digits>`: the prefix every instance id carries.
pub fn is_pattern_id(id: &str) -> bool {
    id.strip_prefix("APAT")
        .is_some_and(|digits| !digits.is_empty() && digits.bytes().all(|byte| byte.is_ascii_digit()))
}

/// The instance id of place `n` of `seed` in pattern `pattern_id`.
pub fn instance_id(pattern_id: &str, seed: &str, n: i64) -> String {
    format!("{pattern_id}.{seed}.i{n}")
}

/// `(pattern id, seed id, n)` of an instance id.
pub fn parse_instance_id(id: &str) -> Option<(&str, &str, i64)> {
    let mut parts = id.split('.');
    let (pattern_id, seed, key) = (parts.next()?, parts.next()?, parts.next()?);
    if parts.next().is_some() || !is_pattern_id(pattern_id) {
        return None;
    }
    Some((pattern_id, seed, key.strip_prefix('i')?.parse().ok()?))
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    if !is_pattern_id(&ctx.id) {
        return Err(format!("pattern component id '{}' must have the form APAT<digits>", ctx.id));
    }
    // Seeds: component ids, or any name on a component (a face picked on it).
    let mut seeds: Vec<String> = Vec::new();
    for name in crate::feature_pipeline::features::common::reference_name_array(ctx.param("seeds")) {
        match ctx.scene.owning_component(&name) {
            Some(record) if record.pattern.is_some() => {
                return Err(format!(
                    "'{}' is an instance of pattern {}; pattern its seed instead",
                    record.id,
                    record.pattern.as_ref().map_or("", |p| p.pattern_id.as_str())
                ));
            }
            Some(record) => {
                if !seeds.contains(&record.id) {
                    seeds.push(record.id.clone());
                }
            }
            None => result.unresolved.push(name),
        }
    }
    if seeds.is_empty() {
        return Ok(result);
    }

    let mode = ctx.string("mode").map(|mode| mode.to_uppercase()).unwrap_or_else(|| "LINEAR".into());
    if mode == "HOLES" || mode == "PATTERN" {
        return build_driven(ctx, &seeds, mode == "HOLES", result);
    }
    let linear = mode == "LINEAR";
    let count = pattern::read_count(ctx);
    let suppressed = pattern::suppressed_instances(ctx);
    if suppressed.contains(&1) {
        result.notes.push(format!(
            "{}: instance 1 is the seed and cannot be suppressed; suppress the seed component instead",
            ctx.id
        ));
    }
    let Some(matrices) = pattern::compute_matrices(ctx, linear, count, &mut result)? else {
        return Ok(result);
    };

    for seed in &seeds {
        let record = ctx.scene.components.get(seed).ok_or_else(|| format!("unknown component '{seed}'"))?;
        let entry = parts_library::active_entry(&record.part_name)
            .ok_or_else(|| format!("part '{}' is not in the parts library", record.part_name))?;
        let restored = restore_solids(&entry.snapshot)
            .map_err(|error| format!("part '{}': snapshot did not decode: {error}", record.part_name))?;
        for (offset, matrix) in matrices.iter().enumerate() {
            let n = offset as i64 + 2;
            if suppressed.contains(&n) {
                continue;
            }
            let delta = AffineTransform::new(*matrix)?;
            let pose = crate::feature_pipeline::component::compose(&delta, &record.transform)?;
            let members = restored.solids.iter().map(|s| (s.name.clone(), s.solid.clone())).collect();
            let id = instance_id(&ctx.id, seed, n);
            let (mut instance, added) =
                create_component(&id, &record.part_name, false, record.source.clone(), pose, members)?;
            instance.pattern =
                Some(PatternFollow { pattern_id: ctx.id.clone(), seed: seed.clone(), delta, key: n, driver: None });
            result.added.extend(added);
            result.components.push(instance);
        }
    }
    Ok(result)
}

/// The key of hole `pid` (copy `k` of a part Pattern, 1 for the source).
pub fn hole_key(pid: i64, copy: i64) -> i64 {
    1000 * (pid + 1) + copy
}

/// A hole wall's local name as `(sketch, pid, wall, copy)`:
/// `S2:P3_Hole_S` → `("S2", 3, "S", 1)`, `S2:P3_Hole_S::PAT1_4` → copy 4.
pub fn parse_hole_face(name: &str) -> Option<(&str, i64, &str, i64)> {
    let (base, copy) = match name.rsplit_once("::") {
        Some((base, suffix)) => {
            let (_, k) = suffix.rsplit_once('_')?;
            (base, k.parse::<i64>().ok().filter(|k| (1..1000).contains(k))?)
        }
        None => (name, 1),
    };
    let (head, wall) = base.split_once("_Hole_")?;
    let (sketch, pid) = head.rsplit_once(":P")?;
    if sketch.is_empty() || wall.is_empty() || wall.contains("::") {
        return None;
    }
    Some((sketch, pid.parse().ok()?, wall, copy))
}

/// The least rotation turning unit `from` onto unit `to` (row-major 3×3).
fn least_rotation(from: crate::Vec3, to: crate::Vec3) -> [[f64; 3]; 3] {
    let axis = from.cross(to);
    let cos = from.dot(to).clamp(-1.0, 1.0);
    let sin = axis.length();
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    if sin < 1e-12 {
        if cos > 0.0 {
            return identity;
        }
        // Opposite axes: a half turn about any axis perpendicular to them.
        let u = from.perpendicular().map(|v| v.normalized().unwrap_or(v)).unwrap_or(crate::Vec3::new(1.0, 0.0, 0.0));
        let u = [u.x, u.y, u.z];
        return std::array::from_fn(|r| std::array::from_fn(|c| 2.0 * u[r] * u[c] - identity[r][c]));
    }
    let k = axis.scale(1.0 / sin);
    let k = [k.x, k.y, k.z];
    let skew = [[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]];
    std::array::from_fn(|r| {
        std::array::from_fn(|c| {
            let k2: f64 = (0..3).map(|m| skew[r][m] * skew[m][c]).sum();
            identity[r][c] + sin * skew[r][c] + (1.0 - cos) * k2
        })
    })
}

/// A part Pattern copy's face name as `(base, pattern, k)`:
/// `S1:P1_Hole_S::PAT1_3` → `("S1:P1_Hole_S", "PAT1", 3)`; a name with no
/// copy suffix is the source, `k` = 1 and no pattern.
pub fn parse_pattern_face(name: &str) -> (&str, Option<&str>, i64) {
    if let Some((base, suffix)) = name.rsplit_once("::") {
        if let Some((pattern, k)) = suffix.rsplit_once('_') {
            if let Ok(k) = k.parse::<i64>() {
                if !pattern.is_empty() && k >= 2 {
                    return (base, Some(pattern), k);
                }
            }
        }
    }
    (name, None, 1)
}

/// A frame on three well-spread control points of `surface` (the same
/// indices on every copy): origin, then an orthonormal basis.
fn surface_frame(points: &[crate::Vec3], picks: (usize, usize, usize)) -> Option<[f64; 16]> {
    let (a, b, c) = (points[picks.0], points[picks.1], points[picks.2]);
    let x = b.sub(a).normalized().ok()?;
    let z = x.cross(c.sub(a)).normalized().ok()?;
    let y = z.cross(x);
    Some([x.x, y.x, z.x, a.x, x.y, y.y, z.y, a.y, x.z, y.z, z.z, a.z, 0.0, 0.0, 0.0, 1.0])
}

fn control_points(surface: &crate::NurbsSurface) -> Vec<crate::Vec3> {
    surface.control_points.iter().flatten().filter_map(|point| point.point().ok()).collect()
}

/// The rigid map taking face `from` onto face `to`, its exact copy (the same
/// carrier, transformed): fitted on three spread control points, checked on
/// all of them.
fn copy_transform(from: &crate::NurbsSurface, to: &crate::NurbsSurface) -> Option<AffineTransform> {
    let (p, q) = (control_points(from), control_points(to));
    if p.len() < 3 || p.len() != q.len() {
        return None;
    }
    let far = |from: crate::Vec3, points: &[crate::Vec3]| {
        (0..points.len()).max_by(|&i, &j| points[i].sub(from).length().total_cmp(&points[j].sub(from).length()))
    };
    let b = far(p[0], &p)?;
    let axis = p[b].sub(p[0]).normalized().ok()?;
    let c = (0..p.len()).max_by(|&i, &j| {
        let off = |k: usize| { let v = p[k].sub(p[0]); v.sub(axis.scale(v.dot(axis))).length() };
        off(i).total_cmp(&off(j))
    })?;
    let source = AffineTransform::new(surface_frame(&p, (0, b, c))?).ok()?;
    let copy = AffineTransform::new(surface_frame(&q, (0, b, c))?).ok()?;
    let map = compose(&copy, &source.rigid_inverse().ok()?).ok()?;
    let size = p.iter().map(|v| v.sub(p[0]).length()).fold(1.0f64, f64::max);
    p.iter().zip(&q).all(|(a, b)| map.point(*a).sub(*b).length() <= 1e-7 * size).then_some(map)
}

/// The rigid map in the driver's frame from the picked hole's axis to
/// another hole's: the translation, and the least rotation between them.
fn axis_transform(from: (crate::Vec3, crate::Vec3), to: (crate::Vec3, crate::Vec3)) -> Result<AffineTransform, String> {
    let r = least_rotation(from.1, to.1);
    let o = [from.0.x, from.0.y, from.0.z];
    let t = [to.0.x, to.0.y, to.0.z];
    let mut m = [0.0f64; 16];
    for row in 0..3 {
        for col in 0..3 {
            m[row * 4 + col] = r[row][col];
        }
        m[row * 4 + 3] = t[row] - (0..3).map(|c| r[row][c] * o[c]).sum::<f64>();
    }
    m[15] = 1.0;
    AffineTransform::new(m)
}

/// HOLES and PATTERN modes: one instance per other place the picked face of
/// the driver repeats at, each keyed to that place.
fn build_driven(ctx: &FeatureContext, seeds: &[String], holes: bool, mut result: FeatureResult) -> Result<FeatureResult, String> {
    use crate::assembly_resolve::{resolve_named_selection, SelectionGeometry};
    let Some(picked) = crate::feature_pipeline::features::common::first_reference_name(ctx.param("driver")) else {
        let what = if holes { "a hole" } else { "a face of a patterned feature" };
        result.notes.push(format!("{}: pick {what} of another component to follow", ctx.id));
        return Ok(result);
    };
    let Some(driver) = ctx.scene.owning_component(&picked) else {
        result.unresolved.push(picked);
        return Ok(result);
    };
    if driver.pattern.is_some() || seeds.contains(&driver.id) {
        return Err(format!("{}: follow a placed part other than the seed, not a pattern instance", ctx.id));
    }
    let local = picked.strip_prefix(&format!("{}:", driver.id)).unwrap_or(&picked).to_string();
    let entry = parts_library::active_entry(&driver.part_name)
        .ok_or_else(|| format!("part '{}' is not in the parts library", driver.part_name))?;
    let restored = restore_solids(&entry.snapshot)
        .map_err(|error| format!("part '{}': snapshot did not decode: {error}", driver.part_name))?;
    let faces = || {
        restored.solids.iter().flat_map(|member| {
            member.solid.shells.iter().flat_map(|shell| &shell.faces).filter_map(move |face| Some((face.name.as_deref()?, face, &member.solid)))
        })
    };
    // Each place, by key, as the rigid map (driver frame) from the picked one.
    let mut places: std::collections::BTreeMap<i64, AffineTransform> = std::collections::BTreeMap::new();
    let reference_key;
    if holes {
        let (sketch, pid, wall, copy) = parse_hole_face(&local)
            .ok_or_else(|| format!("{}: '{picked}' is not a hole wall (`…:P<n>_Hole_S`); pick the hole to follow", ctx.id))?;
        reference_key = hole_key(pid, copy);
        let mut axes: std::collections::BTreeMap<i64, (crate::Vec3, crate::Vec3)> = std::collections::BTreeMap::new();
        for (name, _, solid) in faces() {
            let Some((s, p, w, k)) = parse_hole_face(name) else { continue };
            if s != sketch || w != wall || axes.contains_key(&hole_key(p, k)) {
                continue;
            }
            if let Ok(SelectionGeometry::Axis { origin, direction, .. }) = resolve_named_selection(solid, name) {
                axes.insert(hole_key(p, k), (origin, direction));
            }
        }
        let reference = *axes
            .get(&reference_key)
            .ok_or_else(|| format!("{}: hole '{local}' was not found in part '{}'", ctx.id, driver.part_name))?;
        for (&key, &axis) in &axes {
            places.insert(key, axis_transform(reference, axis)?);
        }
    } else {
        let (base, picked_pattern, picked_k) = parse_pattern_face(&local);
        reference_key = picked_k;
        let mut copies: std::collections::BTreeMap<i64, &crate::NurbsSurface> = std::collections::BTreeMap::new();
        let mut pattern = picked_pattern.map(str::to_string);
        for (name, face, _) in faces() {
            let (b, p, k) = parse_pattern_face(name);
            if b != base {
                continue;
            }
            match (p, pattern.as_deref()) {
                (Some(p), None) => pattern = Some(p.to_string()),
                (Some(p), Some(want)) if p != want => continue,
                _ => {}
            }
            copies.entry(k).or_insert(&face.surface);
        }
        if pattern.is_none() {
            return Err(format!(
                "{}: '{picked}' is not repeated by a Pattern in part '{}'; pick a face of a patterned feature",
                ctx.id, driver.part_name
            ));
        }
        let source = *copies
            .get(&1)
            .ok_or_else(|| format!("{}: the pattern's source face '{base}' was not found in part '{}'", ctx.id, driver.part_name))?;
        // Source → each copy, then from the picked copy to each.
        let mut from_source: std::collections::BTreeMap<i64, AffineTransform> = std::collections::BTreeMap::new();
        for (&k, surface) in &copies {
            match copy_transform(source, surface) {
                Some(map) => {
                    from_source.insert(k, map);
                }
                None => result.notes.push(format!("{}: copy {k} of '{base}' is not an exact copy of the source; skipped", ctx.id)),
            }
        }
        let picked_map = from_source
            .get(&reference_key)
            .ok_or_else(|| format!("{}: the picked copy of '{base}' could not be matched to its source", ctx.id))?
            .rigid_inverse()?;
        for (&k, map) in &from_source {
            places.insert(k, compose(map, &picked_map)?);
        }
    }

    let suppressed = pattern::suppressed_instances(ctx);
    if suppressed.contains(&reference_key) {
        let place = if holes { "hole" } else { "place" };
        result.notes.push(format!(
            "{}: instance {reference_key} is the seed's {place} and cannot be suppressed; suppress the seed component instead",
            ctx.id
        ));
    }
    let driver_inverse = driver.transform.rigid_inverse()?;
    for seed in seeds {
        let record = ctx.scene.components.get(seed).ok_or_else(|| format!("unknown component '{seed}'"))?;
        let entry = parts_library::active_entry(&record.part_name)
            .ok_or_else(|| format!("part '{}' is not in the parts library", record.part_name))?;
        let restored = restore_solids(&entry.snapshot)
            .map_err(|error| format!("part '{}': snapshot did not decode: {error}", record.part_name))?;
        for (&key, local_delta) in &places {
            if key == reference_key || suppressed.contains(&key) {
                continue;
            }
            let delta = compose(&compose(&driver.transform, local_delta)?, &driver_inverse)?;
            let pose = compose(&delta, &record.transform)?;
            let members = restored.solids.iter().map(|s| (s.name.clone(), s.solid.clone())).collect();
            let id = instance_id(&ctx.id, seed, key);
            let (mut instance, added) =
                create_component(&id, &record.part_name, false, record.source.clone(), pose, members)?;
            instance.pattern = Some(PatternFollow {
                pattern_id: ctx.id.clone(),
                seed: seed.clone(),
                delta: *local_delta,
                key,
                driver: Some(driver.id.clone()),
            });
            result.added.extend(added);
            result.components.push(instance);
        }
    }
    Ok(result)
}

/// Never offered from a selection: a component selection shows the component
/// actions, not feature offers. Added from the Assembly ribbon.
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    let mut schema = pattern::schema();
    let params = schema["inputParamsSchema"].as_object_mut().expect("the Pattern schema has params");
    params.remove("solids");
    params.remove("booleanMode");
    let mut ordered = serde_json::Map::new();
    ordered.insert("id".into(), serde_json::json!({
        "type": "string",
        "default_value": null,
        "hint": "unique identifier for the pattern (APAT<digits>); instances are named from it"
    }));
    ordered.insert("seeds".into(), serde_json::json!({
        "type": "reference_selection",
        "selectionFilter": ["COMPONENT"],
        "multiple": true,
        "default_value": [],
        "label": "Seed components",
        "hint": "Placed components to pattern; mate the seed, the instances follow it"
    }));
    for (key, value) in params.iter() {
        if key != "id" {
            ordered.insert(key.clone(), value.clone());
        }
    }
    if let Some(suppressed) = ordered.get_mut("suppressed") {
        suppressed["hint"] = serde_json::json!(
            "Instance numbers that place nothing, e.g. 3, 5 (1 is the seed); in Holes mode, the hole keys the instances carry"
        );
    }
    if let Some(mode) = ordered.get_mut("mode") {
        mode["options"] = serde_json::json!(["LINEAR", "CIRCULAR", "HOLES", "PATTERN"]);
        mode["hint"] = serde_json::json!(
            "Pattern type; HOLES follows every hole of another part's Hole feature, PATTERN every copy of another part's Pattern"
        );
    }
    ordered.insert("driver".into(), serde_json::json!({
        "type": "reference_selection",
        "selectionFilter": ["FACE"],
        "multiple": false,
        "default_value": null,
        "label": "Hole or patterned face to follow",
        "hint": "HOLES: a hole of another component, the one the seed is mated to; every hole of its feature gets an instance. PATTERN: a face of a feature that part patterns; every copy gets one"
    }));
    serde_json::json!({
        "type": "APAT",
        "shortName": "APAT",
        "longName": "Pattern Component",
        "ribbonPath": "Home/Assembly/Pattern Component",
        "commandSize": "Large",
        "displayBuilder": false,
        "inputParamsSchema": ordered,
    })
}

