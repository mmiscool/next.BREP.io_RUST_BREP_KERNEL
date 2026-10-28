use crate::geometry3d::{add3, cross3 as cross, sub3};
use super::transform_gizmo::{
    euler_xyz_deg_from_quat, normalize3, quat_from_axis_angle, quat_from_euler_xyz_deg, quat_mul,
    parse_drag_delta, Quat, TransformDelta,
};
use super::*;

// Component movement uses the shared gizmo at the member-bbox center. Fixed
// components cannot arm it. A drag is solved LIVE: every drag frame asks the
// kernel's dry-run drag solve (`assembly_drag_solve_impl`) where the mates put
// the dragged members, and poses the member displays there display-only (the
// document is untouched until release). A frame whose solve fails holds the
// last good pose; a solve slower than the frame budget skips the next frames
// in proportion, so a large assembly still follows the pointer. Release runs
// one full solve at the final pointer pose and writes the solved poses of
// every moved component in one batch (one undo step); the rerun then finds
// them already solved.
// Arming a component resets feature-transform and dimension gizmos because they
// share one widget slot. All translation and rotation handles are shown together.
// A GROUP (several components moved together) is one rigid body: one gizmo at
// the union member-bbox center on world axes, the same delta composed onto
// every member about that shared pivot, and one batched commit (one undo step).

/// The component-Move controller state (one per engine, like [`TransformArm`]).
pub struct ComponentMoveArm {
    /// The ACOMP feature id the gizmo is armed for (`None` = disarmed) — the
    /// FIRST member of a group.
    pub(super) feature_id: Option<String>,
    /// Every component the gizmo moves, `feature_id` first. One entry for the
    /// single-component Move; two or more for a group move.
    pub(super) group: Vec<String>,
    /// Members that are FIXED and armed by "Unfix and Move": they stay fixed
    /// until a drag commits, and the release writes `isFixed: false` in the
    /// same batch as the new pose (one undo step). Disarming without a drag
    /// changes nothing.
    pub(super) unfix: Vec<String>,
    /// Move Copy selects its members once the background scene rebuild lands.
    select_after_run: bool,
    /// The gizmo pivot — the component's (group's union) member-bbox center, re-synced after
    /// every applied run so the gizmo follows a re-solved (snapped) component.
    pub(super) anchor: [f64; 3],
    /// The in-flight handle drag (grab snapshot + the pending composed pose).
    pub(super) drag: Option<ComponentMoveDrag>,
    /// Pre-drag copies of every display the live solve re-posed, keyed by
    /// solid name — put back before a run's reconcile, on disarm, and when a
    /// release writes nothing.
    pub(super) live_originals: std::collections::HashMap<String, crate::scene::SolidDisplay>,
    /// The pose each live-posed component's displays show now, by component:
    /// a frame whose solve leaves it there re-poses nothing. (The drag solve
    /// answers for every component it solved; on l500 three unmoved 100-part
    /// modules were re-posed and re-uploaded every frame beside the dragged
    /// one.)
    pub(super) live_applied: std::collections::HashMap<String, ([f64; 3], [f64; 3])>,
    /// The per-frame drag solve budget in milliseconds.
    pub(super) budget_ms: f64,
    /// Test hook: the cost every drag solve is charged instead of its
    /// measured wall time (deterministic over-budget behaviour).
    pub(super) solve_cost_override_ms: Option<f64>,
    /// What the last typed move did (the panel's status line).
    pub(super) typed: Option<TypedMove>,
}

/// The outcome of one typed move ([`EngineState::component_move_typed`]):
/// the field, the value typed, how much of it the mates let through along
/// that axis, and the constraints that stopped the rest.
#[derive(Clone, Debug, PartialEq)]
pub struct TypedMove {
    /// `x`, `y`, `z` (mm along the gizmo's axes) or `rx`, `ry`, `rz` (degrees
    /// about them).
    pub field: String,
    pub asked: f64,
    pub applied: f64,
    /// Constraint ids that refused the rest of the move (empty when it all
    /// went through, or when none could be named).
    pub blocking: Vec<String>,
    /// Whether anything was written (one undo step).
    pub written: bool,
}

impl TypedMove {
    /// Whether the mates let the whole typed value through.
    pub fn complete(&self) -> bool {
        (self.applied - self.asked).abs() <= TYPED_TOLERANCE * self.asked.abs().max(1.0)
    }

    /// The panel's status line: what moved, of what was asked, and why not all.
    pub fn summary(&self) -> String {
        let (axis, unit, space) = typed_axis(&self.field).map(|(axis, rotate)| {
            (["X", "Y", "Z"][axis], if rotate { "\u{00b0}" } else { " mm" }, if rotate { "about" } else { "along" })
        }).unwrap_or(("?", "", "along"));
        let shown = |value: f64| crate::formatting::compact_decimal(value, 4);
        let limited = if self.blocking.is_empty() {
            String::new()
        } else {
            format!(", limited by {}", self.blocking.join(", "))
        };
        if !self.written {
            return match self.blocking.as_slice() {
                [] => format!("Not moved {space} {axis}: the mates hold it here."),
                held => format!("Not moved {space} {axis}: held by {}.", held.join(", ")),
            };
        }
        if self.complete() {
            return format!("Moved {}{unit} {space} {axis}.", shown(self.applied));
        }
        format!("Moved {} of {}{unit} {space} {axis}{limited}.", shown(self.applied), shown(self.asked))
    }
}

/// A typed move counts as complete within this fraction of what was typed.
const TYPED_TOLERANCE: f64 = 1e-6;

/// `x|y|z|rx|ry|rz` → (axis index, is a rotation).
fn typed_axis(field: &str) -> Option<(usize, bool)> {
    match field {
        "x" => Some((0, false)),
        "y" => Some((1, false)),
        "z" => Some((2, false)),
        "rx" => Some((0, true)),
        "ry" => Some((1, true)),
        "rz" => Some((2, true)),
        _ => None,
    }
}

/// The per-frame drag solve budget: a solve slower than this skips the next
/// frames in proportion (the display holds the last good pose meanwhile).
pub(super) const DRAG_SOLVE_BUDGET_MS: f64 = 30.0;
/// Never skip more than this many frames after one slow solve.
const DRAG_SOLVE_MAX_SKIP: u32 = 10;

impl Default for ComponentMoveArm {
    fn default() -> Self {
        Self {
            feature_id: None,
            group: Vec::new(),
            unfix: Vec::new(),
            select_after_run: false,
            anchor: [0.0; 3],
            drag: None,
            live_originals: std::collections::HashMap::new(),
            live_applied: std::collections::HashMap::new(),
            budget_ms: DRAG_SOLVE_BUDGET_MS,
            solve_cost_override_ms: None,
            typed: None,
        }
    }
}

/// The live solve's view of one drag: the last good solved poses, whether
/// the latest frame's solve failed, and the frames still to skip.
#[derive(Clone, Default, Debug)]
pub(super) struct LiveSolve {
    /// Component id → solved `(translate, rotate_deg)` from the last solve
    /// that held (every component the solve moved, not only the dragged).
    pub good: Option<std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>>,
    /// The latest attempted solve failed (the display holds `good`).
    pub held: bool,
    /// Frames to skip before the next solve (over-budget back-off).
    pub skip: u32,
    /// Solves run and frames skipped this drag (for the verifier).
    pub solves: u32,
    pub skipped: u32,
}

/// A grab snapshot: the handle, the grab screen point, the gizmo frame AT GRAB
/// (the frozen frame every drag move resolves against — no error
/// accumulation), every member's pose at grab (pivoting about the shared
/// anchor), and the PENDING delta + frame the release commits.
#[derive(Clone)]
pub(super) struct ComponentMoveDrag {
    handle: u32,
    sx: f32,
    sy: f32,
    start: ComponentPose,
    members: Vec<(String, ComponentPose)>,
    pending: Option<(TransformDelta, ComponentPose)>,
    live: LiveSolve,
}

/// A component pose as the gizmo tracks it: the ACOMP `transform` pair plus the
/// gizmo anchor (bbox center) it pivots about.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) struct ComponentPose {
    pub translate: [f64; 3],
    pub rotate_deg: [f64; 3],
    pub anchor: [f64; 3],
}

impl EngineState {
    /// Whether the component Move gizmo is armed (for any component).
    pub fn component_move_armed(&self) -> bool {
        self.component_move.feature_id.is_some()
    }

    /// The armed component feature id (empty when disarmed).
    pub fn component_move_armed_feature(&self) -> String {
        self.component_move.feature_id.clone().unwrap_or_default()
    }

    /// Every component the armed gizmo moves (empty when disarmed).
    pub fn component_move_group(&self) -> Vec<String> {
        self.component_move.group.clone()
    }

    /// The Move action for SEVERAL components (a multi-selection, or faces of
    /// several occurrences): one gizmo that moves them together as a rigid
    /// group. Duplicates count once. FIXED members are left out with one toast
    /// naming them; the rest still arm. One movable member falls back to the
    /// single-component [`Self::component_move_toggle`]; pressing Move again on
    /// the armed group disarms it.
    pub fn component_move_group_toggle(&mut self, ids: &[String]) {
        let mut unique: Vec<String> = Vec::new();
        for id in ids {
            if !unique.contains(id) {
                unique.push(id.clone());
            }
        }
        if unique.len() <= 1 {
            if let Some(id) = unique.first() {
                self.component_move_toggle(id);
            }
            return;
        }
        let mut movable = Vec::new();
        let mut fixed = Vec::new();
        for id in unique {
            match self.component_info(&id) {
                None => self.push_notice(format!("'{id}' is not an assembly component")),
                Some(info) if info.fixed => fixed.push(format!("{} ({id})", info.part_name)),
                Some(_) => movable.push(id),
            }
        }
        if !fixed.is_empty() {
            let verb = if fixed.len() == 1 { "is" } else { "are" };
            self.push_notice(format!(
                "{} {verb} fixed — not moved (unfix to move)",
                fixed.join(", ")
            ));
        }
        match movable.len() {
            0 => {}
            1 => self.component_move_toggle(&movable[0]),
            // The same set, in any order, toggles off.
            _ if self.component_move.group.len() == movable.len()
                && movable.iter().all(|id| self.component_move.group.contains(id)) =>
            {
                self.disarm_transform()
            }
            _ => self.component_move_arm_group(movable),
        }
    }

    /// "Unfix and Move": like [`Self::component_move_group_toggle`], but FIXED
    /// components join the move instead of being skipped. Nothing is unfixed
    /// until a drag commits; the release then writes each fixed member's
    /// `isFixed: false` together with its new pose, as one undo step. Pressing
    /// it again on the same armed set disarms.
    pub fn component_move_unfix_toggle(&mut self, ids: &[String]) {
        let mut members: Vec<String> = Vec::new();
        let mut unfix = Vec::new();
        for id in ids {
            if members.contains(id) {
                continue;
            }
            match self.component_info(id) {
                None => self.push_notice(format!("'{id}' is not an assembly component")),
                Some(info) => {
                    if info.fixed {
                        unfix.push(id.clone());
                    }
                    members.push(id.clone());
                }
            }
        }
        if members.is_empty() {
            return;
        }
        let same = self.component_move.group.len() == members.len()
            && members.iter().all(|id| self.component_move.group.contains(id))
            && self.component_move.unfix.len() == unfix.len();
        if same {
            self.disarm_transform();
            return;
        }
        self.component_move_arm_group(members);
        self.component_move.unfix = unfix;
    }

    /// The Move toggle (context bar / tree action): ARMS the full gizmo (all
    /// handle sets) for `feature_id`, or DISARMS when it is already armed
    /// (arming fresh replaces any other armed component). A FIXED component
    /// refuses with a toast and never arms.
    pub fn component_move_toggle(&mut self, feature_id: &str) {
        let Some(info) = self.component_info(feature_id) else {
            self.push_notice(format!("'{feature_id}' is not an assembly component"));
            return;
        };
        if info.fixed {
            self.push_notice(format!(
                "{} ({feature_id}) is fixed — unfix it to move",
                info.part_name
            ));
            return;
        }
        // Armed for exactly this component (not as one member of a group).
        if self.component_move.group == [feature_id] {
            self.disarm_transform();
        } else {
            self.component_move_arm_widget(feature_id);
        }
    }

    /// The Move Copy action (context bar / tree action): append a SECOND
    /// instance of `source_id`'s part — its `inputParams` cloned (same
    /// `partName`, same pose, any other placement keys riding along) under a
    /// fresh `ACOMP<n>` id and an explicit `isFixed: false` — then select it
    /// and arm the full Move gizmo on it, so the user drags the copy off the
    /// original. No new part is made: both instances reference one
    /// parts-library entry. A FIXED source copies fine (the copy is free).
    /// Returns the new feature id.
    pub fn component_move_copy(&mut self, source_id: &str) -> Result<String, String> {
        let Some(index) = self.history.index_of(source_id).filter(|&index| {
            matches!(
                self.history.feature_type(index).as_deref(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        }) else {
            let error = format!("'{source_id}' is not an assembly component");
            self.push_notice(error.clone());
            return Err(error);
        };
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        let id = self.history.next_feature_id("ACOMP");
        let Some(map) = params.as_object_mut() else {
            return Err(format!("component '{source_id}': malformed inputParams"));
        };
        map.insert("id".into(), serde_json::Value::String(id.clone()));
        map.insert("isFixed".into(), serde_json::Value::Bool(false));
        let feature = serde_json::json!({
            "type": "ACOMP",
            "inputParams": params,
            "persistentData": {}
        });
        self.add_feature(&feature.to_string())?;
        // Select the copy so the context bar's actions (Move again, Delete, …)
        // target the instance the gizmo sits on, not the original.
        self.component_move_arm_widget(&id);
        if self.run_pending() {
            self.component_move.select_after_run = true;
        } else {
            self.select_component(&id);
        }
        Ok(id)
    }

    /// The `arm_transform` ROUTE for ACOMP features (history-panel expand):
    /// a FIXED component silently stays armless (the explicit Move action is
    /// the one that toasts).
    pub(super) fn component_move_arm(&mut self, feature_id: &str) {
        match self.component_info(feature_id) {
            Some(info) if !info.fixed => self.component_move_arm_widget(feature_id),
            _ => {}
        }
    }

    /// Drop the component arm STATE only (the caller owns the widget slot) —
    /// the exclusivity hook `arm_transform` / `arm_dimension` / `disarm_transform`
    /// call before taking the slot for themselves.
    pub(super) fn component_move_reset(&mut self) {
        self.component_live_restore();
        let budget_ms = self.component_move.budget_ms;
        let solve_cost_override_ms = self.component_move.solve_cost_override_ms;
        self.component_move =
            ComponentMoveArm { budget_ms, solve_cost_override_ms, ..ComponentMoveArm::default() };
    }

    /// Arm for `feature_id`: claim the shared widget slot (clearing the
    /// feature gizmo + dimension overlay), pin the anchor at the member-bbox
    /// center, and feed the full handle set.
    fn component_move_arm_widget(&mut self, feature_id: &str) {
        self.component_move_arm_group(vec![feature_id.to_string()]);
    }

    /// Arm for every id of `group` (callers pass at least one, none fixed).
    fn component_move_arm_group(&mut self, group: Vec<String>) {
        // Claim the shared slot WITHOUT disarm_transform (which would also reset
        // the component state we are about to set).
        self.transform_gizmo.feature_id = None;
        self.transform_gizmo.mode = GizmoMode::None;
        self.transform_gizmo.drag = None;
        self.clear_feature_dimension_overlay();

        self.component_live_restore();
        let anchor = self.component_group_anchor(&group);
        self.component_move.feature_id = group.first().cloned();
        self.component_move.group = group;
        self.component_move.unfix.clear();
        self.component_move.select_after_run = false;
        self.component_move.anchor = anchor;
        self.component_move.drag = None;
        self.component_move.typed = None;
        self.feed_component_widget();
        self.dirty = true;
    }

    /// The pivot of `group`: the union member-bbox center (a lone component
    /// without resident members falls back to its translate, as before).
    fn component_group_anchor(&self, group: &[String]) -> [f64; 3] {
        let mut bbox = crate::camera::Aabb::empty();
        for id in group {
            for solid in self.scene.solids() {
                if super::components::owned_by(&solid.name, id) {
                    bbox.union(&solid.bbox);
                }
            }
        }
        if !bbox.is_empty() {
            return bbox.center();
        }
        group
            .first()
            .and_then(|id| self.component_info(id))
            .map(|info| info.translate)
            .unwrap_or([0.0; 3])
    }

    /// The gizmo frame at the armed pose: a single component's own rotated
    /// basis; a group's WORLD axes (a group has no single orientation).
    fn component_move_frame(&self) -> Option<ComponentPose> {
        let anchor = self.component_move.anchor;
        if self.component_move.group.len() > 1 {
            return Some(ComponentPose { translate: anchor, rotate_deg: [0.0; 3], anchor });
        }
        let info = self.component_info(self.component_move.feature_id.as_deref()?)?;
        Some(ComponentPose { translate: info.translate, rotate_deg: info.rotate_deg, anchor })
    }

    /// (Re)feed the widget gizmo at the armed component's current pose+anchor.
    fn feed_component_widget(&mut self) {
        let Some(pose) = self.component_move_frame() else {
            return;
        };
        let json = component_frame_json(&pose);
        let _ = self.widgets.set_transform_json(&json);
    }

    /// Post-run re-sync (the [`finish_apply`] hook, mirroring
    /// `sync_transform_gizmo`): re-anchor at the possibly re-solved member bbox
    /// and re-feed; auto-disarm when the component vanished or became fixed.
    pub(super) fn component_move_sync(&mut self) {
        if self.component_move.feature_id.is_none() {
            return;
        }
        // A member that vanished or became fixed drops out; the gizmo
        // disarms only when nothing movable is left.
        let unfix = &self.component_move.unfix;
        let group: Vec<String> = self
            .component_move
            .group
            .iter()
            .filter(|id| {
                self.component_info(id)
                    .is_some_and(|info| !info.fixed || unfix.contains(id))
            })
            .cloned()
            .collect();
        let Some(id) = group.first().cloned() else {
            self.disarm_transform();
            return;
        };
        // Earlier replies may arrive while a newer run is pending.
        // Consume this once, only after the current scene is available.
        if self.component_move.select_after_run && !self.run_pending() {
            self.component_move.select_after_run = false;
            self.select_component(&id);
        }
        self.component_move.anchor = self.component_group_anchor(&group);
        self.component_move.feature_id = Some(id);
        self.component_move.unfix.retain(|id| group.contains(id));
        self.component_move.group = group;
        self.feed_component_widget();
        self.dirty = true;
    }

    /// Begin a component-gizmo drag at viewport px `(x, y)`; `true` when a
    /// handle was grabbed (the viewport routes the drag here, not the camera).
    pub fn component_press(&mut self, x: f64, y: f64) -> bool {
        let Some(start) = self.component_move_frame() else {
            return false;
        };
        let handle = self.transform_pick(x, y);
        if handle == 0 {
            return false;
        }
        let Some(members) = self.component_move_members() else {
            return false;
        };
        self.widgets.set_transform_active(handle);
        self.component_move.live_applied.clear();
        self.component_move.drag = Some(ComponentMoveDrag {
            handle,
            sx: x as f32,
            sy: y as f32,
            start,
            members,
            pending: None,
            live: LiveSolve::default(),
        });
        // A drag supersedes the last typed move's status line.
        self.component_move.typed = None;
        // The dry-run solve needs the main-side session (once per applied run).
        if self.history_has_assembly() {
            self.ensure_assembly_session();
        }
        self.dirty = true;
        true
    }

    /// Every armed member at its current pose, pivoting about the shared
    /// anchor (`None` when a member is gone).
    fn component_move_members(&self) -> Option<Vec<(String, ComponentPose)>> {
        self.component_move
            .group
            .iter()
            .map(|id| {
                let info = self.component_info(id)?;
                let pose = ComponentPose {
                    translate: info.translate,
                    rotate_deg: info.rotate_deg,
                    anchor: self.component_move.anchor,
                };
                Some((id.clone(), pose))
            })
            .collect()
    }

    /// A TYPED move of the armed component (or group): `value` mm along, or
    /// degrees about, one of the gizmo's axes (`field` = `x|y|z|rx|ry|rz`) —
    /// the same motion as dragging that arrow or ring by exactly `value`. It is
    /// solved against the mates once and written through the drag release's
    /// commit (one undo step, unfix, mechanical references). When the mates
    /// allow only part of it, that part is applied and the report says how
    /// much and which constraints stopped the rest; when they allow none, a
    /// notice names them and nothing is written.
    pub fn component_move_typed(&mut self, field: &str, value: f64) -> Result<TypedMove, String> {
        let (axis, rotate) = typed_axis(field).ok_or_else(|| format!("unknown move field {field:?}"))?;
        if !value.is_finite() {
            return Err(format!("{value} is not a number"));
        }
        if self.component_move.drag.is_some() {
            return Err("a drag is in progress".into());
        }
        let frame = self.component_move_frame().ok_or("nothing is armed to move")?;
        let members = self.component_move_members().ok_or("an armed component is gone")?;
        let mut unit = [0.0; 3];
        unit[axis] = 1.0;
        let direction = normalize3(quat_rotate(quat_from_euler_xyz_deg(frame.rotate_deg), unit));
        let delta = if rotate {
            TransformDelta::Rotate { axis: direction, radians: value.to_radians() }
        } else {
            TransformDelta::Translate([direction[0] * value, direction[1] * value, direction[2] * value])
        };
        if self.history_has_assembly() {
            self.ensure_assembly_session();
        }
        let (solved, blocking) = match self.component_solve_explained(&members, &delta, None, true) {
            Ok(result) => result,
            Err(error) => {
                let report = TypedMove { field: field.into(), asked: value, applied: 0.0, blocking: Vec::new(), written: false };
                self.push_notice(format!("Move: the mates cannot hold any pose for this move ({error})."));
                self.component_move.typed = Some(report.clone());
                return Ok(report);
            }
        };
        let (primary, start) = members.first().cloned().ok_or("nothing is armed to move")?;
        let landed = solved.get(&primary).copied().unwrap_or((start.translate, start.rotate_deg));
        let applied = if rotate {
            let q_start = quat_from_euler_xyz_deg(start.rotate_deg);
            let q = quat_mul(quat_from_euler_xyz_deg(landed.1), [-q_start[0], -q_start[1], -q_start[2], q_start[3]]);
            let along = q[0] * direction[0] + q[1] * direction[1] + q[2] * direction[2];
            let mut degrees = (2.0 * along.atan2(q[3])).to_degrees();
            if degrees > 180.0 {
                degrees -= 360.0;
            } else if degrees < -180.0 {
                degrees += 360.0;
            }
            degrees
        } else {
            let moved = sub3(rigid_delta_point(&start, landed, start.anchor), start.anchor);
            moved[0] * direction[0] + moved[1] * direction[1] + moved[2] * direction[2]
        };
        let stuck = applied.abs() <= TYPED_TOLERANCE * value.abs().max(1.0);
        let written = !stuck && self.component_commit_solved(&members, &solved, frame.anchor);
        if !written {
            self.feed_component_widget();
        }
        let report = TypedMove { field: field.into(), asked: value, applied: if written { applied } else { 0.0 }, blocking, written };
        if !written {
            self.push_notice(format!("Move: {}", report.summary()));
        }
        self.component_move.typed = Some(report.clone());
        self.dirty = true;
        Ok(report)
    }

    /// The last typed move's report (the panel's status line).
    pub fn component_move_typed_report(&self) -> Option<TypedMove> {
        self.component_move.typed.clone()
    }

    /// Whether a component-gizmo drag is in flight.
    pub fn component_move_dragging(&self) -> bool {
        self.component_move.drag.is_some()
    }

    /// Continue the drag: resolve the world delta against the FROZEN grab frame,
    /// compose the pending pose, solve it live (budget permitting) and pose the
    /// member displays and the gizmo where the mates put them.
    pub fn component_drag_to(&mut self, cx: f64, cy: f64) {
        let Some(drag) = self.component_move.drag.clone() else {
            return;
        };
        let cam = gizmo_camera(&self.camera);
        let frame = component_frame_json(&drag.start);
        let json = self.widgets.transform_drag_json_with_frame(
            &cam,
            &frame,
            drag.handle,
            drag.sx,
            drag.sy,
            cx as f32,
            cy as f32,
        );
        let Some(delta) = parse_drag_delta(&json) else {
            return;
        };
        let pending = compose_component_delta(&drag.start, &delta);
        if let Some(live) = self.component_move.drag.as_mut() {
            live.pending = Some((delta, pending));
        }
        let mut live = drag.live.clone();
        if live.skip > 0 {
            live.skip -= 1;
            live.skipped += 1;
        } else {
            let started = now_ms();
            let solved = self.component_drag_solve(&drag.members, &delta, live.good.as_ref());
            let cost = self.component_move.solve_cost_override_ms.unwrap_or(now_ms() - started);
            live.solves += 1;
            match solved {
                Ok(poses) => {
                    live.good = Some(poses);
                    live.held = false;
                }
                Err(_) => live.held = true,
            }
            let budget = self.component_move.budget_ms.max(1e-3);
            if cost > budget {
                live.skip = ((cost / budget).ceil() as u32).saturating_sub(1).min(DRAG_SOLVE_MAX_SKIP);
            }
        }
        // The gizmo rides the primary member's live pose (the pending pose
        // until a solve has held); the gold active-handle highlight survives
        // (only a null feed clears it).
        let frame = self.component_live_frame(&drag, &pending, &live);
        let _ = self.widgets.set_transform_json(&component_frame_json(&frame));
        self.component_live_pose_displays(&drag.members, &live);
        if let Some(state) = self.component_move.drag.as_mut() {
            state.live = live;
        }
        self.dirty = true;
    }

    /// One dry-run solve of the dragged members at `delta`: component id →
    /// solved `(translate, rotate_deg)` for everything the mates moved. `good`
    /// (the last good live poses) is where mechanical mates re-base. A
    /// document without assembly state moves the members freely.
    fn component_drag_solve(
        &mut self,
        members: &[(String, ComponentPose)],
        delta: &TransformDelta,
        good: Option<&std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>>,
    ) -> Result<std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>, String> {
        self.component_solve_explained(members, delta, good, false).map(|(poses, _)| poses)
    }

    /// [`Self::component_drag_solve`], and with `explain` the constraints that
    /// refused the dragged members' target pose (the kernel's conflict search:
    /// one solve per constraint, so only a typed move asks).
    fn component_solve_explained(
        &mut self,
        members: &[(String, ComponentPose)],
        delta: &TransformDelta,
        good: Option<&std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>>,
        explain: bool,
    ) -> Result<(std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>, Vec<String>), String> {
        let free: std::collections::BTreeMap<String, ([f64; 3], [f64; 3])> = members
            .iter()
            .map(|(id, start)| {
                let pose = compose_component_delta(start, delta);
                (id.clone(), (pose.translate, pose.rotate_deg))
            })
            .collect();
        if !self.history_has_assembly() {
            return Ok((free, Vec::new()));
        }
        let poses: serde_json::Map<String, serde_json::Value> = free
            .iter()
            .map(|(id, (translate, rotate_deg))| {
                (id.clone(), serde_json::json!({ "translate": translate, "rotateEulerDeg": rotate_deg }))
            })
            .collect();
        // The last good live poses: mechanical mates re-base there, so each
        // frame couples only the motion since then.
        let reference: serde_json::Map<String, serde_json::Value> = good
            .into_iter()
            .flatten()
            .map(|(id, (translate, rotate_deg))| {
                (id.clone(), serde_json::json!({ "translate": translate, "rotateEulerDeg": rotate_deg }))
            })
            .collect();
        let request = serde_json::json!({
            "poses": poses,
            "unfix": self.component_move.unfix,
            "reference": reference,
            "explain": explain,
        });
        let reply = brep_kernel::assembly_drag_solve_impl(&request.to_string())?;
        let reply: serde_json::Value = serde_json::from_str(&reply).map_err(|error| error.to_string())?;
        let mut solved = std::collections::BTreeMap::new();
        for (id, pose) in reply.get("poses").and_then(|poses| poses.as_object()).into_iter().flatten() {
            let triple = |key: &str| -> Option<[f64; 3]> {
                let values = pose.get(key)?.as_array()?;
                Some([values.first()?.as_f64()?, values.get(1)?.as_f64()?, values.get(2)?.as_f64()?])
            };
            if let (Some(translate), Some(rotate_deg)) = (triple("translate"), triple("rotateEulerDeg")) {
                solved.insert(id.clone(), (translate, rotate_deg));
            }
        }
        let blocking = reply
            .get("blocking")
            .and_then(|ids| ids.as_array())
            .into_iter()
            .flatten()
            .filter_map(|id| id.as_str().map(str::to_string))
            .collect();
        Ok((solved, blocking))
    }

    /// The gizmo frame for this drag frame: a group keeps its world-axis frame
    /// at the anchor carried by the primary member's live delta; a single
    /// component sits at its live pose.
    fn component_live_frame(
        &self,
        drag: &ComponentMoveDrag,
        pending: &ComponentPose,
        live: &LiveSolve,
    ) -> ComponentPose {
        let Some((id, start)) = drag.members.first() else {
            return *pending;
        };
        let Some(&(translate, rotate_deg)) = live.good.as_ref().and_then(|good| good.get(id)) else {
            return *pending;
        };
        let anchor = rigid_delta_point(start, (translate, rotate_deg), start.anchor);
        if drag.members.len() > 1 {
            return ComponentPose { translate: anchor, rotate_deg: [0.0; 3], anchor };
        }
        ComponentPose { translate, rotate_deg, anchor }
    }

    /// Pose every display of every live-solved component at its solved pose
    /// (display-only; the pre-drag copies are kept for the restore).
    fn component_live_pose_displays(&mut self, members: &[(String, ComponentPose)], live: &LiveSolve) {
        let Some(good) = live.good.as_ref() else {
            return;
        };
        for (id, &solved) in good {
            // The grab pose of a dragged member; the document pose otherwise.
            let start = match members.iter().find(|(member, _)| member == id) {
                Some((_, start)) => *start,
                None => match self.component_info(id).map(|info| (info.translate, info.rotate_deg)).or_else(|| self.occurrence_world_pose(id)) {
                    Some((translate, rotate_deg)) => ComponentPose { translate, rotate_deg, anchor: [0.0; 3] },
                    None => continue,
                },
            };
            // Already showing this pose (posed to it last frame, or never
            // moved from where it started): nothing to re-pose or re-upload.
            let shown = self
                .component_move
                .live_applied
                .get(id)
                .copied()
                .unwrap_or((start.translate, start.rotate_deg));
            if same_pose(shown, solved) {
                continue;
            }
            self.component_move.live_applied.insert(id.clone(), solved);
            // A flexible occurrence (`ACOMP1:ACOMP2`) poses its own solids;
            // they come after their instance in the map, so its pose wins.
            let names = if id.contains(':') {
                let prefix = format!("{id}:");
                self.scene.solids().iter().filter(|solid| solid.name.starts_with(&prefix)).map(|solid| solid.name.clone()).collect()
            } else {
                self.member_solid_names(id)
            };
            for name in names {
                let Some(display) = self.scene.solid(&name) else { continue };
                let original = self
                    .component_move
                    .live_originals
                    .entry(name.clone())
                    .or_insert_with(|| display.clone())
                    .clone();
                if let Some(display) = self.scene.solid_mut(&name) {
                    *display = original;
                    pose_display_rigid(display, &start, solved);
                }
            }
        }
        self.dirty = true;
    }

    /// A nested occurrence's world pose as `(translate, rotate_deg)`: its top
    /// component's pose composed with the occurrence's SOLVED pose in it
    /// (`nested_poses`). What a flexible occurrence's live drag pose starts
    /// from. (The live display skipped occurrences, so a flexible
    /// sub-assembly's parts moved rigidly with it during the drag and jumped
    /// to their solved poses at release.)
    fn occurrence_world_pose(&self, path: &str) -> Option<([f64; 3], [f64; 3])> {
        let (top, rest) = path.split_once(':')?;
        let record = self.assembly_components().iter().find(|record| record.id == top)?;
        let local = record.nested_poses.get(rest)?;
        let (a, b) = (&record.transform, local);
        let world: [f64; 16] = std::array::from_fn(|k| (0..4).map(|m| a[(k / 4) * 4 + m] * b[m * 4 + k % 4]).sum());
        let pose = brep_kernel::transform_to_pose_params(&brep_kernel::AffineTransform::new(world).ok()?);
        let triple = |key: &str| -> Option<[f64; 3]> {
            let values = pose.get(key)?.as_array()?;
            Some([values.first()?.as_f64()?, values.get(1)?.as_f64()?, values.get(2)?.as_f64()?])
        };
        Some((triple("translate")?, triple("rotateEulerDeg")?))
    }

    /// Put every live-posed display back exactly (pre-drag pose).
    pub(super) fn component_live_restore(&mut self) {
        self.component_move.live_applied.clear();
        let originals = std::mem::take(&mut self.component_move.live_originals);
        if originals.is_empty() {
            return;
        }
        for (name, original) in originals {
            if let Some(display) = self.scene.solid_mut(&name) {
                *display = original;
            }
        }
        self.dirty = true;
    }

    /// End the drag: COMMIT the pending pose into each moved ACOMP's
    /// `inputParams.transform` (one param write — or one batched write for a
    /// group → one undo entry → one rerun whose constraint tail re-solves; the
    /// post-run sync then re-glues the gizmo to wherever the solve left the
    /// components). A grab that never moved commits nothing.
    pub fn component_release(&mut self) {
        let Some(drag) = self.component_move.drag.take() else {
            return;
        };
        self.widgets.set_transform_active(0);
        self.dirty = true;
        let Some((delta, frame)) = drag.pending else {
            self.component_live_restore();
            return;
        };
        // The true solve at the final pointer pose; a failure keeps the last
        // good live pose, and a drag that never held writes nothing (the
        // pre-drag pose stands).
        let solved = match self.component_drag_solve(&drag.members, &delta, drag.live.good.as_ref()) {
            Ok(poses) => poses,
            Err(_) => match drag.live.good.clone() {
                Some(poses) => poses,
                None => {
                    self.component_live_restore();
                    self.component_move.unfix.clear();
                    self.feed_component_widget();
                    return;
                }
            },
        };
        self.component_commit_solved(&drag.members, &solved, frame.anchor);
    }

    /// Write the solved poses of a drag or a typed move: every component that
    /// moved, a pending unfix, and the mechanical mates' references, in ONE
    /// undo step (then one rerun). `fallback_anchor` places the gizmo when the
    /// primary member has no solved pose. Returns whether anything was written.
    fn component_commit_solved(
        &mut self,
        members: &[(String, ComponentPose)],
        solved: &std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>,
        fallback_anchor: [f64; 3],
    ) -> bool {
        self.component_move.anchor = match members.first() {
            Some((id, start)) => match solved.get(id) {
                Some(&pose) => rigid_delta_point(start, pose, start.anchor),
                None => fallback_anchor,
            },
            None => fallback_anchor,
        };
        let mut edits = Vec::new();
        for (id, &(translate, rotate_deg)) in solved {
            let Some(index) = self.history.index_of(id) else {
                continue;
            };
            // Only what moved is written (a drag the mates fully undid, or a
            // partner they left alone, changes nothing). A pending unfix
            // always writes: the release frees the member.
            let unfix = self.component_move.unfix.contains(id);
            match self.component_info(id) {
                Some(info) if !unfix && !pose_moved((info.translate, info.rotate_deg), (translate, rotate_deg)) => continue,
                None => continue,
                _ => {}
            }
            let pose = ComponentPose { translate, rotate_deg, anchor: [0.0; 3] };
            let mut params = self
                .history
                .feature_params(index)
                .unwrap_or_else(|| serde_json::json!({}));
            if !params.get("transform").map(|t| t.is_object()).unwrap_or(false) {
                if let Some(object) = params.as_object_mut() {
                    object.insert("transform".into(), serde_json::json!({}));
                }
            }
            if let Some(transform) = params.get_mut("transform").and_then(|t| t.as_object_mut()) {
                transform.insert("translate".into(), serde_json::json!(pose.translate));
                transform.insert("rotateEulerDeg".into(), serde_json::json!(pose.rotate_deg));
            }
            if self.component_move.unfix.contains(id) {
                if let Some(object) = params.as_object_mut() {
                    object.insert("isFixed".into(), serde_json::Value::Bool(false));
                }
            }
            edits.push((id.clone(), params));
        }
        // The pending unfix is spent: from here the members are free.
        self.component_move.unfix.clear();
        if edits.is_empty() {
            self.component_live_restore();
            return false;
        }
        // Gears, racks and screws keep their phase across runs (their stored
        // references): re-capture them at the released poses and write them in
        // the SAME undo step as the poses, or the next run would read a
        // multi-turn drag against the pre-drag references and wrap a spin.
        // ONE UNDO STEP PER COMMIT. A param write coalesces on `param:{index}`,
        // so two drags or typed values of the same component in a row would
        // otherwise be one entry, and one undo would take back both (the
        // feature transform gizmo ends its run on release for the same reason).
        self.history.break_coalescing();
        if let Some(block) = self.mechanical_block_at(solved) {
            let resolved: Vec<(usize, serde_json::Value)> = edits
                .iter()
                .filter_map(|(id, params)| Some((self.history.index_of(id)?, params.clone())))
                .collect();
            // The checkpoint here snapshots the pre-drag document (poses AND
            // references); the block follows without one.
            self.history.set_many_feature_params(&resolved);
            self.history.set_assembly_block(block, false);
            self.history.break_coalescing();
            let _ = self.rerun_history();
            return true;
        }
        match edits.as_slice() {
            [] => {}
            [(id, params)] => {
                let _ = self.update_feature_params(id, &params.to_string());
            }
            _ => {
                let _ = self.update_many_feature_params(&edits);
            }
        }
        self.history.break_coalescing();
        true
    }

    /// The assembly block with every mechanical mate's references re-captured
    /// at `solved` (the released poses), or `None` when the document has no
    /// mechanical mate (nothing to refresh).
    fn mechanical_block_at(
        &mut self,
        solved: &std::collections::BTreeMap<String, ([f64; 3], [f64; 3])>,
    ) -> Option<serde_json::Value> {
        if !self.history_has_assembly() {
            return None;
        }
        let poses: serde_json::Map<String, serde_json::Value> = solved
            .iter()
            .map(|(id, (translate, rotate_deg))| {
                (id.clone(), serde_json::json!({ "translate": translate, "rotateEulerDeg": rotate_deg }))
            })
            .collect();
        let reply = brep_kernel::assembly_rebase_mechanical_impl(&serde_json::json!({ "poses": poses }).to_string()).ok()?;
        serde_json::from_str::<serde_json::Value>(&reply).ok().filter(|block| !block.is_null())
    }

    /// The armed component gizmo's logical state for the verifier:
    /// `{armed, feature, group, unfix, anchor}`.
    pub fn component_move_json(&self) -> String {
        serde_json::json!({
            "armed": self.component_move_armed(),
            "feature": self.component_move_armed_feature(),
            "group": self.component_move.group,
            "unfix": self.component_move.unfix,
            "anchor": self.component_move.anchor,
            "typed": self.component_move.typed.as_ref().map(|typed| serde_json::json!({
                "field": typed.field,
                "asked": typed.asked,
                "applied": typed.applied,
                "blocking": typed.blocking,
                "written": typed.written,
                "summary": typed.summary(),
            })),
            "live": self.component_move.drag.as_ref().map(|drag| {
                let poses: serde_json::Map<String, serde_json::Value> = drag
                    .live
                    .good
                    .iter()
                    .flatten()
                    .map(|(id, (translate, rotate_deg))| {
                        (id.clone(), serde_json::json!({ "translate": translate, "rotateEulerDeg": rotate_deg }))
                    })
                    .collect();
                serde_json::json!({
                    "poses": poses,
                    "held": drag.live.held,
                    "solves": drag.live.solves,
                    "skipped": drag.live.skipped,
                })
            }),
        })
        .to_string()
    }
}

/// The widget frame feed for a component pose: origin = the ANCHOR (bbox
/// center), axes = the pose's rotated basis (intrinsic XYZ, the kernel bake).
/// EVERY handle set is shown (center free-move ball + axis arrows + rotation
/// arcs) — move and rotate coexist, no mode switch.
fn component_frame_json(pose: &ComponentPose) -> String {
    let euler = [
        pose.rotate_deg[0].to_radians(),
        pose.rotate_deg[1].to_radians(),
        pose.rotate_deg[2].to_radians(),
    ];
    let x = normalize3(rotate_euler_xyz_f64([1.0, 0.0, 0.0], euler));
    let y = normalize3(rotate_euler_xyz_f64([0.0, 1.0, 0.0], euler));
    let z = normalize3(rotate_euler_xyz_f64([0.0, 0.0, 1.0], euler));
    serde_json::json!({
        "origin": pose.anchor,
        "x": x,
        "y": y,
        "z": z,
        "showCenter": true,
        "showAxes": true,
        "showRings": true,
    })
    .to_string()
}

/// Translation shifts pose and anchor together. Rotation pivots about anchor C:
/// `R' = dR·R`, `translate' = C + dR·(translate − C)`.
pub(super) fn compose_component_delta(
    start: &ComponentPose,
    delta: &TransformDelta,
) -> ComponentPose {
    match delta {
        TransformDelta::Translate(d) => ComponentPose {
            translate: add3(start.translate, *d),
            rotate_deg: start.rotate_deg,
            anchor: add3(start.anchor, *d),
        },
        TransformDelta::Rotate { axis, radians } => {
            let dq = quat_from_axis_angle(*axis, *radians);
            let q0 = quat_from_euler_xyz_deg(start.rotate_deg);
            let rotate_deg = euler_xyz_deg_from_quat(quat_mul(dq, q0));
            let offset = sub3(start.translate, start.anchor);
            ComponentPose {
                translate: add3(start.anchor, quat_rotate(dq, offset)),
                rotate_deg,
                anchor: start.anchor,
            }
        }
    }
}

/// Where a world point riding a component at `start` lands when the component
/// is at `to`: `p' = R_to·R_startᵀ·(p − t_start) + t_to`.
fn rigid_delta_point(start: &ComponentPose, to: ([f64; 3], [f64; 3]), point: [f64; 3]) -> [f64; 3] {
    let q_start = quat_from_euler_xyz_deg(start.rotate_deg);
    let q_to = quat_from_euler_xyz_deg(to.1);
    let q_delta = quat_mul(q_to, [-q_start[0], -q_start[1], -q_start[2], q_start[3]]);
    add3(to.0, quat_rotate(q_delta, sub3(point, start.translate)))
}

/// Whether two `(translate, rotate_deg)` poses differ beyond solver noise: a
/// partner the drag did not move comes back from the solve within ~1e-7 mm
/// (measured on as1-ug-214), and writing that would rerun it for nothing.
fn pose_moved(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> bool {
    (0..3).any(|i| (a.0[i] - b.0[i]).abs() > 1e-6 || (a.1[i] - b.1[i]).abs() > 1e-6)
}

/// Re-pose a display rigidly from the component pose `start` to `to`: the
/// mesh, edge polylines and vertices move, normals rotate, the bbox follows.
/// Two `(translate, rotate_deg)` poses within roundoff of each other.
fn same_pose(a: ([f64; 3], [f64; 3]), b: ([f64; 3], [f64; 3])) -> bool {
    (0..3).all(|k| (a.0[k] - b.0[k]).abs() <= 1e-9 && (a.1[k] - b.1[k]).abs() <= 1e-9)
}

fn pose_display_rigid(display: &mut crate::scene::SolidDisplay, start: &ComponentPose, to: ([f64; 3], [f64; 3])) {
    // A rigid move keeps the shape: the renderer rewrites positions and
    // normals in the buffers it holds (`sync_scene`'s pose path) instead of
    // rebuilding the solid every drag frame.
    if display.shape_revision == 0 {
        display.shape_revision = display.revision;
    }
    // Moved by rewriting its positions, not through a pose matrix.
    display.pose = None;
    display.revision = crate::scene::next_revision();
    let q_start = quat_from_euler_xyz_deg(start.rotate_deg);
    let q_delta = quat_mul(quat_from_euler_xyz_deg(to.1), [-q_start[0], -q_start[1], -q_start[2], q_start[3]]);
    let pose = |p: [f64; 3]| add3(to.0, quat_rotate(q_delta, sub3(p, start.translate)));
    let mut bbox = crate::camera::Aabb::empty();
    for position in &mut display.mesh.positions {
        let posed = pose(position.map(f64::from));
        *position = posed.map(|v| v as f32);
        bbox.expand(posed);
    }
    for normal in &mut display.mesh.normals {
        *normal = quat_rotate(q_delta, normal.map(f64::from)).map(|v| v as f32);
    }
    for edge in &mut display.edges {
        for point in &mut edge.polyline {
            let posed = pose(point.map(f64::from));
            *point = posed.map(|v| v as f32);
            bbox.expand(posed);
        }
    }
    for vertex in &mut display.vertices {
        vertex.position = pose(vertex.position);
        bbox.expand(vertex.position);
    }
    if !bbox.is_empty() {
        display.bbox = bbox;
    }
}

/// Milliseconds on a monotonic-enough clock (the browser has no `Instant`).
/// Shared with the motion-study bake (`simulation_play`), which slices its
/// solving under the same kind of per-frame budget.
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn now_ms() -> f64 {
    static ORIGIN: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    ORIGIN.get_or_init(std::time::Instant::now).elapsed().as_secs_f64() * 1000.0
}

#[cfg(target_arch = "wasm32")]
pub(super) fn now_ms() -> f64 {
    js_sys::Date::now()
}

/// Rotate `v` by quaternion `q`: `v + 2·(q.xyz × (q.xyz × v + w·v))`.
pub(super) fn quat_rotate(q: Quat, v: [f64; 3]) -> [f64; 3] {
    let u = [q[0], q[1], q[2]];
    let w = q[3];
    let t = cross(u, add3(cross(u, v), [w * v[0], w * v[1], w * v[2]]));
    [v[0] + 2.0 * t[0], v[1] + 2.0 * t[1], v[2] + 2.0 * t[2]]
}

