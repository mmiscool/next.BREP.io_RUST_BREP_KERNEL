//! **Flip** a constraint to its other solution (Touch Align facing the other
//! way, a distance on the other side, a gear turning the other way …).
//!
//! The kernel says what a flip is, per type ([`brep_kernel::assembly_flip_plan_json`],
//! `constraints::flip_rule`): the parameter patch and how to SEED the moving
//! component so the solve lands on the other branch. Without the seed the
//! solve starts from the current pose and may simply return to the solution it
//! left (or not converge). The patch and the seeded pose are written as ONE
//! undo step: the pose through `set_many_feature_params` (the checkpoint),
//! the patched assembly block without one, then one rerun, as the drag
//! release writes poses and mechanical references.

use super::component_move::quat_rotate;
use super::transform_gizmo::{
    euler_xyz_deg_from_quat, normalize3, quat_from_axis_angle, quat_from_euler_xyz_deg, quat_mul,
};
use super::*;
use serde_json::{json, Value};

fn triple(value: &Value) -> Option<[f64; 3]> {
    let array = value.as_array()?;
    Some([array.first()?.as_f64()?, array.get(1)?.as_f64()?, array.get(2)?.as_f64()?])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// A unit vector perpendicular to `n`.
fn perpendicular(n: [f64; 3]) -> [f64; 3] {
    let helper = if n[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    normalize3([
        n[1] * helper[2] - n[2] * helper[1],
        n[2] * helper[0] - n[0] * helper[2],
        n[0] * helper[1] - n[1] * helper[0],
    ])
}

impl EngineState {
    /// Flip constraint `id` to its other solution, as one undo step. Returns
    /// the component the seed moved (empty when the flip moves nothing up
    /// front: a mechanical mate, or a path/slot measured from the other end).
    /// `Err` names why the constraint has no other solution.
    pub fn assembly_flip_constraint(&mut self, id: &str) -> Result<String, String> {
        self.ensure_assembly_session();
        let plan: Value = serde_json::from_str(&brep_kernel::assembly_flip_plan_json(id))
            .map_err(|error| format!("flip: {error}"))?;
        if plan["ok"] != true {
            return Err(plan["reason"].as_str().or(plan["error"].as_str()).unwrap_or("this constraint cannot flip").to_string());
        }
        let seed = plan["seed"].as_str().unwrap_or("none").to_string();
        let elements = plan["elements"].as_array().cloned().unwrap_or_default();

        // The patched assembly block (written without a checkpoint below).
        let mut block = self.history.assembly_block().cloned().ok_or("flip: the document has no assembly")?;
        let entry = block["constraints"]
            .as_array_mut()
            .into_iter()
            .flatten()
            .find(|entry| entry["inputParams"]["id"] == id)
            .ok_or_else(|| format!("flip: no constraint '{id}'"))?;
        for (key, value) in plan["patch"].as_object().into_iter().flatten() {
            entry["inputParams"][key] = value.clone();
        }

        let seeded = if seed == "none" { None } else { self.flip_seed(&seed, &elements, &plan)? };
        match &seeded {
            Some((component, translate, rotate_deg)) => {
                let index = self.history.index_of(component).ok_or_else(|| format!("flip: no component '{component}'"))?;
                let mut params = self.history.feature_params(index).unwrap_or_else(|| json!({}));
                if !params["transform"].is_object() {
                    params["transform"] = json!({});
                }
                params["transform"]["translate"] = json!(translate);
                params["transform"]["rotateEulerDeg"] = json!(rotate_deg);
                self.history.break_coalescing();
                self.history.set_many_feature_params(&[(index, params)]);
                self.history.set_assembly_block(block, false);
            }
            None => {
                self.history.break_coalescing();
                self.history.set_assembly_block(block, true);
            }
        }
        self.history.break_coalescing();
        let _ = self.rerun_history();
        Ok(seeded.map(|(component, _, _)| component).unwrap_or_default())
    }

    /// The moving component and its seeded `(translate, rotateEulerDeg)`.
    /// The moving element is the non-fixed one; with both free, the one the
    /// mates leave freer (the last solve's `bodyFreedom`), the second on a tie.
    fn flip_seed(&mut self, seed: &str, elements: &[Value], plan: &Value) -> Result<Option<(String, [f64; 3], [f64; 3])>, String> {
        let freedom = self.assembly_dof_value()["bodyFreedom"].clone();
        let free_of = |element: &Value| -> u64 {
            let component = element["component"].as_str().unwrap_or("");
            freedom[component]["rotations"].as_u64().unwrap_or(0) + freedom[component]["translations"].as_u64().unwrap_or(0)
        };
        let movable: Vec<usize> = (0..elements.len()).filter(|&i| elements[i]["fixed"] != true && elements[i]["component"].as_str().is_some_and(|c| !c.is_empty())).collect();
        let moving = match movable.as_slice() {
            [] => return Err("both components are fixed: unfix one to flip".into()),
            [only] => *only,
            several => *several.iter().max_by_key(|&&i| (free_of(&elements[i]), i)).expect("non-empty"),
        };
        let reference = if moving == 0 { 1.min(elements.len().saturating_sub(1)) } else { 0 };
        let component = elements[moving]["component"].as_str().unwrap_or_default().to_string();
        let info = self.component_info(&component).ok_or_else(|| format!("flip: no component '{component}'"))?;
        let anchor = triple(&elements[moving]["anchor"]).ok_or("flip: the moving element has no anchor")?;
        let own = triple(&elements[moving]["direction"]).map(normalize3);
        let other = triple(&elements[reference]["direction"]).map(normalize3);
        // (rotation quaternion, pivot, extra translation)
        let (rotation, shift) = match seed {
            "turn" => {
                let n = own.or(other).ok_or("flip: no direction to turn about")?;
                (quat_from_axis_angle(perpendicular(n), std::f64::consts::PI), [0.0; 3])
            }
            "turn-about-reference" => {
                let n = other.or(own).ok_or("flip: no direction to turn about")?;
                (quat_from_axis_angle(n, std::f64::consts::PI), [0.0; 3])
            }
            "mirror" => {
                let n = other.ok_or("flip: the reference has no plane to mirror across")?;
                let base = triple(&elements[reference]["anchor"]).ok_or("flip: the reference has no anchor")?;
                let offset = dot([anchor[0] - base[0], anchor[1] - base[1], anchor[2] - base[2]], n);
                ([0.0, 0.0, 0.0, 1.0], [-2.0 * offset * n[0], -2.0 * offset * n[1], -2.0 * offset * n[2]])
            }
            "rotate" => {
                let axis = triple(&plan["angleAxis"]).map(normalize3).or(other).ok_or("flip: no angle axis")?;
                let value = plan["value"].as_f64().unwrap_or(0.0);
                (quat_from_axis_angle(axis, (-2.0 * value).to_radians()), [0.0; 3])
            }
            _ => return Ok(None),
        };
        let q0 = quat_from_euler_xyz_deg(info.rotate_deg);
        let offset = [info.translate[0] - anchor[0], info.translate[1] - anchor[1], info.translate[2] - anchor[2]];
        let turned = quat_rotate(rotation, offset);
        let translate = [anchor[0] + turned[0] + shift[0], anchor[1] + turned[1] + shift[1], anchor[2] + turned[2] + shift[2]];
        let rotate_deg = euler_xyz_deg_from_quat(quat_mul(rotation, q0));
        Ok(Some((component, translate, rotate_deg)))
    }
}

