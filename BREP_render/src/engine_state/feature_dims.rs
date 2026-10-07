use brep_kernel::first_reference_name;
use super::*;
use super::sketch_edit_ops::is_plain_number_literal;
use brep_gizmos::hit_region::{point_region, segment_region, HitShape};

/// The world-space overlay group carrying the FD leaders + arrowheads.
const FEATURE_DIM_OVERLAY: &str = "feature-dim-leaders";

/// How clearly a dimension handle must win its own point before it is published
/// as a widget (px, screen). See [`EngineState::dimension_handle_points`].
const GRAB_MARGIN_PX: f64 = 2.0;

/// The role a dimension-gizmo hit region plays. An `Arrow` is a value-drag handle
/// (a linear leader or the angular arc handle, keyed by its `field_key`); `Origin`
/// is the shared origin/center BALL (the mode toggle), keyed by the `field_key` of
/// the FIRST dim anchored there — several dims share one ball (a cube's three
/// leaders), so the name is a stable handle for it rather than a claim that it
/// belongs to that one dim. Both share ONE region list so the pick + the drawn
/// outline can never drift.
#[derive(Clone, Debug)]
enum DimRole {
    Arrow(String),
    Origin(String),
}

impl EngineState {
    /// The armed ◎ gizmo mode: `"none"`, `"transform"`, or `"dimension"`. Drives
    /// the ◎ highlight + the app's dimension-overlay draw / input routing.
    pub fn gizmo_mode(&self) -> &'static str {
        match self.transform_gizmo.mode {
            GizmoMode::None => "none",
            GizmoMode::Transform => "transform",
            GizmoMode::Dimension => "dimension",
        }
    }

    /// Whether the DIMENSION gizmo is armed for THIS feature (drives the ◎
    /// dimension-mode highlight).
    pub fn dimension_armed_for(&self, feature_id: &str) -> bool {
        matches!(self.transform_gizmo.mode, GizmoMode::Dimension)
            && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
    }

    /// The dimension-armed feature id (empty unless in dimension mode).
    pub fn dimension_armed_feature(&self) -> String {
        if matches!(self.transform_gizmo.mode, GizmoMode::Dimension) {
            self.transform_gizmo.feature_id.clone().unwrap_or_default()
        } else {
            String::new()
        }
    }

    /// Arm the DIMENSION gizmo for `feature_id`: hide the transform widget, show
    /// the annotation overlay. Re-arming a different feature moves it.
    pub fn arm_dimension(&mut self, feature_id: &str) {
        // The widget slot is shared — an armed component Move gizmo yields.
        self.component_move_reset();
        self.transform_gizmo.feature_id = Some(feature_id.to_string());
        self.transform_gizmo.mode = GizmoMode::Dimension;
        self.transform_gizmo.drag = None;
        // The transform widget and the dimension overlay are mutually exclusive.
        let _ = self.widgets.set_transform_json("null");
        self.refresh_feature_dimension_overlay();
        self.dirty = true;
    }

    // --- The orange center-sphere ◎ TOGGLE (dimension ↔ transform) ---------
    //
    // A single orange sphere sits at the gizmo center in BOTH modes: the
    // transform gizmo's `HANDLE_CENTER` sphere and the dimension arrows' shared
    // origin sphere project to the same point. Clicking it flips the two modes,
    // mirroring the old app's `CombinedTransformControls` center-handle toggle
    // (pointer-down on `HANDLE_CENTER` calls
    // `toggleDisplayMode`). The viewport routes a bare CLICK here; a DRAG on the
    // center still free-moves via `transform_press` (unchanged).

    /// Whether a screen-px pick in TRANSFORM mode lands on the orange CENTER
    /// free-move sphere (`HANDLE_CENTER`). The viewport uses this to make a bare
    /// click on the center TOGGLE to the dimension arrows (via
    /// [`toggle_to_dimension`](Self::toggle_to_dimension)) instead of swallowing
    /// it as a generic handle click. False in any other gizmo mode.
    pub fn transform_center_pick(&self, x: f64, y: f64) -> bool {
        matches!(self.transform_gizmo.mode, GizmoMode::Transform)
            && self.transform_pick(x, y) == brep_gizmos::transform::HANDLE_CENTER
    }

    /// Whether a screen-px pick in DIMENSION mode lands on an orange ORIGIN
    /// sphere of the armed feature's dimension arrows. Each distinct annotation
    /// draws such a sphere — a LINEAR dim at its `point_a` (a cube's three axis
    /// dims share one, a cone/pyramid draw two), an ANGULAR dim at its arc
    /// `center` (the vertex; its sweep-END sphere is the angle DRAG handle, not a
    /// toggle) — so every one is projected via the camera and hit-tested against
    /// the screen-constant sphere radius. The viewport uses this to TOGGLE back to
    /// the transform gizmo (via [`toggle_to_transform`](Self::toggle_to_transform)),
    /// which is the ONLY way an angular-only feature (a revolve) reaches transform.
    /// False in any other gizmo mode. The hit radius mirrors the gizmo center's own
    /// tolerance (`PX_CENTER_RAD + 2.0`, transform.rs) so the two toggle targets match.
    pub fn dimension_origin_pick(&self, x: f64, y: f64) -> bool {
        // The origin sphere (LINEAR at `point_a`, ANGULAR at the arc `center`) is an
        // `Origin`-role region. 2D-test the cursor against the SAME screen-space
        // regions the outline draws — a point-in-circle test, so what is outlined
        // is exactly what toggles.
        let p = [x as f32, y as f32];
        self.dimension_hit_regions()
            .into_iter()
            .any(|(role, shape)| matches!(role, DimRole::Origin(_)) && shape.contains(p))
    }

    /// Whether a screen-px pick in DIMENSION mode lands on a dimension ARROWHEAD
    /// (a linear leader's orange cone TIP at `point_b`, or an angular arc's orange
    /// sweep-END handle sphere). Returns the grabbed annotation's `field_key` — the
    /// viewport routes a DRAG that starts here to [`feature_dimension_drag`](Self::
    /// feature_dimension_drag), editing that param live (Fix 4). `None` in any other
    /// gizmo mode / when no arrowhead is under the pointer. Distinct from
    /// [`dimension_origin_pick`](Self::dimension_origin_pick): that grabs the SHARED
    /// origin sphere (a mode toggle), this grabs an arrowHEAD (a value edit). The
    /// nearest arrowhead within the screen-constant hit radius wins.
    pub fn dimension_arrow_pick(&self, x: f64, y: f64) -> Option<String> {
        // The NEAREST `Arrow`-role region containing the cursor wins. LINEAR: the
        // WHOLE leader CAPSULE (`point_a → point_b`) is grabbable, so a cursor
        // anywhere on the visible shaft grabs it — even when `point_b` crosses
        // BEHIND the eye (the reported sizeY failure), because in ortho the whole
        // leader still projects (and in perspective the region is front-clipped to
        // its visible part). ANGULAR: the arc sweep-END handle CIRCLE. These are
        // the SAME screen-space regions the outline draws, so what is outlined is
        // exactly what grabs.
        let p = [x as f32, y as f32];
        let mut best: Option<(f32, String)> = None;
        for (role, shape) in self.dimension_hit_regions() {
            if let DimRole::Arrow(key) = role {
                let d = shape.spine_distance(p);
                if d <= shape.radius() && best.as_ref().map(|(bd, _)| d < *bd).unwrap_or(true) {
                    best = Some((d, key));
                }
            }
        }
        best.map(|(_, key)| key)
    }

    /// The authoritative SCREEN-space (viewport-local px) pickable regions of the
    /// armed feature's dimension gizmo, each paired with its [`DimRole`]. The
    /// SINGLE source `dimension_arrow_pick` (its `Arrow` regions), `dimension_origin_pick`
    /// (its `Origin` regions), and `dimension_hit_areas_json` (draws them ALL) all
    /// consume — so the grabbable area is exactly the drawn outline. Projection +
    /// the perspective front-clip happen ONCE in [`brep_gizmos::hit_region`].
    ///   * LINEAR → a leader CAPSULE (`point_a → point_b`, `ARROW_HANDLE_HIT_RAD_PX`)
    ///     with role `Arrow` + an origin CIRCLE (`point_a`, `ORIGIN_SPHERE_RAD_PX + 2`)
    ///     with role `Origin`.
    ///   * ANGULAR → an arc-handle CIRCLE (`arrow_handle_point`,
    ///     `ARROW_HANDLE_HIT_RAD_PX`) with role `Arrow` + an arc-center origin
    ///     CIRCLE (`center`, `ORIGIN_SPHERE_RAD_PX + 2`) with role `Origin`.
    /// Origin balls are deduped by world position (a torus's linear origin + its
    /// angular center coincide) so they match the single drawn sphere. `[]` unless
    /// the DIMENSION gizmo is armed for a feature.
    fn dimension_hit_regions(&self) -> Vec<(DimRole, HitShape)> {
        if !matches!(self.transform_gizmo.mode, GizmoMode::Dimension) {
            return Vec::new();
        }
        let feature = self.dimension_armed_feature();
        if feature.is_empty() {
            return Vec::new();
        }
        let wpp = self.camera.world_per_pixel();
        let arrow_px = crate::feature_dimensions::ARROW_HANDLE_HIT_RAD_PX as f32;
        let origin_px = (crate::feature_dimensions::ORIGIN_SPHERE_RAD_PX + 2.0) as f32;
        let cam = &self.camera;
        let mut out: Vec<(DimRole, HitShape)> = Vec::new();
        let mut origins: Vec<[f64; 3]> = Vec::new();
        for ann in self.feature_dimension_annotations(&feature) {
            match ann.kind {
                crate::feature_dimensions::FeatureDimKind::Linear => {
                    if let Some(shape) = segment_region(cam, ann.point_a, ann.point_b, arrow_px) {
                        out.push((DimRole::Arrow(ann.field_key.clone()), shape));
                    }
                    push_origin_region(&mut out, &mut origins, cam, ann.point_a, origin_px, &ann.field_key);
                }
                crate::feature_dimensions::FeatureDimKind::Angular => {
                    let handle = crate::feature_dimensions::arrow_handle_point(&ann, wpp);
                    if let Some(shape) = point_region(cam, handle, arrow_px) {
                        out.push((DimRole::Arrow(ann.field_key.clone()), shape));
                    }
                    push_origin_region(&mut out, &mut origins, cam, ann.center, origin_px, &ann.field_key);
                }
            }
        }
        out
    }

    /// The grabbable ARROWHEAD of every dimension the armed ◎ gizmo draws, as
    /// `(field_key, (x, y))` in viewport-local px — what a host presses to drag
    /// that param's value.
    ///
    /// The point is the arrowHEAD end of the `Arrow` region
    /// ([`dimension_hit_regions`](Self::dimension_hit_regions)), not its middle:
    /// a LINEAR leader's capsule runs `point_a → point_b` and the whole shaft
    /// grabs, but the cone TIP at `b` is the thing a reader sees and a
    /// walkthrough should be filmed pulling; an ANGULAR dim's region is already
    /// the sweep-end handle circle, so its center is the point.
    ///
    /// Published ONLY where the gizmo's own pick answers with that field, AND
    /// still answers it [`GRAB_MARGIN_PX`] away in each direction — the same
    /// rule [`transform_handle_points`](Self::transform_handle_points) follows,
    /// with a margin the transform widget does not need.
    ///
    /// The margin is not caution, it is a bug this found. A pick is decided by
    /// `spine_distance`, nearest wins and a tie goes to the first annotation, so
    /// where two handles COINCIDE the winner turns on the last bit of an `f32`.
    /// A full-circle torus is exactly that: `arc`'s sweep-end ball sits at 360°,
    /// which is where the `majorRadius` leader points, so the two are the same
    /// screen point. Publishing `dim:arc` there passed an equality test taken in
    /// `f64` and then lost the press, because a rect goes out through JSON and
    /// comes back a few ulps away — and the drag silently resized the torus
    /// instead of sweeping it. Requiring the field to win by a couple of pixels
    /// drops such a handle rather than publishing one that grabs its neighbour.
    ///
    /// Empty in any mode but DIMENSION.
    pub fn dimension_handle_points(&self) -> Vec<(String, (f64, f64))> {
        use brep_gizmos::hit_region::HitShape;
        self.dimension_hit_regions()
            .into_iter()
            .filter_map(|(role, shape)| {
                let DimRole::Arrow(key) = role else { return None };
                let [x, y] = match shape {
                    HitShape::Capsule { b, .. } => b,
                    HitShape::Circle { c, .. } => c,
                };
                let point = (f64::from(x), f64::from(y));
                let grabs = |dx: f64, dy: f64| {
                    self.dimension_arrow_pick(point.0 + dx, point.1 + dy).as_deref()
                        == Some(key.as_str())
                };
                const M: f64 = GRAB_MARGIN_PX;
                let robust = grabs(0.0, 0.0)
                    && grabs(M, 0.0)
                    && grabs(-M, 0.0)
                    && grabs(0.0, M)
                    && grabs(0.0, -M);
                robust.then_some((key, point))
            })
            .collect()
    }

    /// The ORIGIN BALLS the armed ◎ gizmo draws, as `(field_key, (x, y))` in
    /// viewport-local px — the orange spheres a CLICK toggles the gizmo between
    /// its dimension and transform forms at.
    ///
    /// A ball is keyed by the first dim anchored on it, because several share
    /// one: a cube's three leaders all start at its min corner and draw a single
    /// sphere (`radius`/`sizeX`), while a cone or a pyramid draws two — one at
    /// the base centre and one at the top (`radiusBottom` and `radiusTop`). The
    /// name is a stable handle for the ball, not a claim that it belongs to that
    /// dim alone.
    ///
    /// Published under the same rule as the arrowheads: only where the gizmo's
    /// own [`dimension_origin_pick`](Self::dimension_origin_pick) answers at the
    /// point, so a ball hidden behind an arrowhead that outranks it is dropped
    /// rather than published somewhere a click would do something else.
    ///
    /// Empty in any mode but DIMENSION.
    pub fn dimension_origin_points(&self) -> Vec<(String, (f64, f64))> {
        use brep_gizmos::hit_region::HitShape;
        self.dimension_hit_regions()
            .into_iter()
            .filter_map(|(role, shape)| {
                let DimRole::Origin(key) = role else { return None };
                let HitShape::Circle { c, .. } = shape else { return None };
                let point = (f64::from(c[0]), f64::from(c[1]));
                self.dimension_origin_pick(point.0, point.1).then_some((key, point))
            })
            .collect()
    }

    /// DEBUG overlay: the EXACT SCREEN-space (viewport-local px) pickable regions
    /// of the armed feature's dimension gizmo — the SAME regions
    /// `dimension_arrow_pick` + `dimension_origin_pick` 2D-test the cursor against
    /// ([`dimension_hit_regions`](Self::dimension_hit_regions)) — so the red
    /// outline can NEVER drift from the grabbable area. Each item is a
    /// `{ kind:"capsule", a:[x,y], b:[x,y], r }` (linear leaders) or
    /// `{ kind:"circle", c:[x,y], r }` (origin / arc-handle spheres); the app only
    /// offsets by `rect.min`. `[]` unless the DIMENSION gizmo is armed.
    pub fn dimension_hit_areas_json(&self) -> String {
        let regions = self.dimension_hit_regions();
        super::camera_widgets::hit_shapes_json(regions.iter().map(|(_, shape)| shape))
    }

    /// Toggle the armed ◎ gizmo from TRANSFORM to DIMENSION for the currently
    /// transform-armed feature (the orange center-sphere click). No-op unless a
    /// feature is transform-armed.
    pub fn toggle_to_dimension(&mut self) {
        let feature = self.transform_armed_feature();
        if !feature.is_empty() && self.feature_dimension_annotations_json(&feature) != "[]" {
            self.arm_dimension(&feature);
        }
    }

    /// Toggle the armed ◎ gizmo from DIMENSION to TRANSFORM for the currently
    /// dimension-armed feature (the orange origin-sphere click). No-op unless a
    /// feature is dimension-armed.
    pub fn toggle_to_transform(&mut self) {
        let feature = self.dimension_armed_feature();
        if feature.is_empty() {
            return;
        }
        // A PLANE has NO transform — its placement is fully the orientation +
        // `offset_distance` the offset dimension gizmo drives — so it never gets a
        // transform gizmo: the origin-sphere toggle stays in dimension mode. (Other
        // dim features, e.g. a revolve, still toggle to their transform gizmo.)
        if let Some(index) = self.history.index_of(&feature) {
            if self.history.feature_type(index).as_deref() == Some("P") {
                return;
            }
        }
        self.arm_transform(&feature);
    }

    /// The linear dimension annotations for `feature_id` (resolving expression
    /// params against the live history env first). `[]` for a feature type with
    /// no FD-1 builder / a missing feature.
    fn feature_dimension_annotations(
        &self,
        feature_id: &str,
    ) -> Vec<crate::feature_dimensions::FeatureDimAnnotation> {
        let Some(index) = self.history.index_of(feature_id) else {
            return Vec::new();
        };
        let Some(feature_type) = self.history.feature_type(index) else {
            return Vec::new();
        };
        let Some(params) = self.history.feature_params(index) else {
            return Vec::new();
        };
        let resolved = self.resolve_param_expressions(&params);
        // Resolve any scene references the builder needs (extrude profile plane,
        // revolve axis line) from the run report's profiles/axes — keyed off the
        // ORIGINAL params so reference-name strings are read verbatim.
        let refs = self.feature_dimension_refs(&feature_type, &params);
        crate::feature_dimensions::build_annotations_with_refs(&feature_type, &resolved, &refs)
    }

    /// Resolve the scene references a feature-dimension builder needs beyond its
    /// pure params: the extrude/revolve profile PLANE (center + normal) and the
    /// revolve AXIS line. Sourced from the run report the engine already holds —
    /// `sketch_profiles` (the sketch's world profile, which survives being
    /// consumed by the extrude/revolve since only solids honor `removed`) and
    /// `sketch_axes` (a sketch's published axis lines) — with resident-scene
    /// fallbacks for a profile that is a solid FACE (its display-mesh plane, see
    /// [`Self::lookup_profile_plane`]) and an axis that is a solid EDGE (its
    /// polyline). Empty for any other feature type; missing pieces stay `None`
    /// so the builder degrades to `[]` gracefully.
    fn feature_dimension_refs(
        &self,
        feature_type: &str,
        params: &serde_json::Value,
    ) -> crate::feature_dimensions::ResolvedRefs {
        let mut refs = crate::feature_dimensions::ResolvedRefs::default();
        match feature_type {
            "E" => {
                if let Some((center, normal)) = self.lookup_profile_plane(params.get("profile")) {
                    refs.profile_center = Some(center);
                    refs.profile_normal = Some(normal);
                }
            }
            "R" => {
                if let Some((center, normal)) = self.lookup_profile_plane(params.get("profile")) {
                    refs.profile_center = Some(center);
                    refs.profile_normal = Some(normal);
                }
                if let Some((point, dir)) = self.lookup_axis_line(params.get("axis")) {
                    refs.axis_point = Some(point);
                    refs.axis_dir = Some(dir);
                }
            }
            "P" => {
                // The plane feature registers ONE scene frame under its own id; the
                // offset dim hangs off that resolved plane (origin + z-axis normal).
                if let Some(id) = params.get("id").and_then(|v| v.as_str()) {
                    if let Some((_, frame)) =
                        self.construction_frames.iter().find(|(name, _)| name == id)
                    {
                        refs.plane_origin = Some(vec3_to_arr(frame.origin));
                        refs.plane_normal = Some(fd_normalize3(vec3_to_arr(frame.z_axis)));
                        // Screen-constant handle stub for the offset ≈ 0 case (~48 px).
                        refs.plane_dim_length = Some(self.camera.world_per_pixel() * 48.0);
                    }
                }
            }
            _ => {}
        }
        refs
    }

    /// Resolve a `profile` reference param to the world plane the extrude/revolve
    /// gizmo hangs off, as `(center, unit normal)`. A SKETCH profile first (its
    /// outer-loop centroid + `z_axis`), else a resident solid FACE named by the
    /// reference (the area-weighted centroid + outward normal of its display
    /// mesh) — the same two sources the kernel's extrude/revolve accept for
    /// `profile` (`scene.resolve_profile`, then `resolve_face` →
    /// `face_profile`, whose `z_axis` is likewise the OUTWARD face normal, so the
    /// arc's `orient_revolve_axis` sign matches the kernel's sweep direction).
    /// `None` when neither resolves — a face whose owning solid was later
    /// consumed (a boolean target) is no longer in the scene, and the builder
    /// then degrades to `[]` as before.
    fn lookup_profile_plane(
        &self,
        profile_param: Option<&serde_json::Value>,
    ) -> Option<([f64; 3], [f64; 3])> {
        if let Some(profile) = self.lookup_sketch_profile(profile_param) {
            return Some((sketch_profile_centroid(profile), vec3_to_arr(profile.z_axis)));
        }
        let name = first_reference_name(profile_param)?;
        self.scene.face_plane_world(&name)
    }

    /// Resolve a `profile` reference param to the sketch profile the engine holds
    /// (exact name, or the `:PROFILE`-suffixed form — mirrors `SceneMap::resolve_profile`).
    fn lookup_sketch_profile(
        &self,
        profile_param: Option<&serde_json::Value>,
    ) -> Option<&brep_kernel::SketchProfile> {
        let name = first_reference_name(profile_param)?;
        // A committed sketch is surfaced as a render display sheet aliased
        // `{sketch}:FACE`, and profile consumers may reference the `{sketch}:PROFILE`
        // form; both alias the base sketch id the run report keys `sketch_profiles`
        // by. Strip either so the extrude/revolve gizmo resolves the same profile the
        // kernel does (mirrors `common::normalize_profile_alias` / `resolve_profile`).
        let base = name
            .strip_suffix(":FACE")
            .or_else(|| name.strip_suffix(":PROFILE"))
            .unwrap_or(&name);
        self.sketch_profiles
            .iter()
            .find(|(id, _)| id == &name || id == base)
            .map(|(_, profile)| profile)
    }

    /// Resolve an `axis` reference param to a world line `(point, unit direction)`:
    /// a published sketch axis first (`sketch_axes`), else a resident solid EDGE's
    /// polyline endpoints (`scene.edge_polyline_world`). `None` if neither resolves.
    fn lookup_axis_line(
        &self,
        axis_param: Option<&serde_json::Value>,
    ) -> Option<([f64; 3], [f64; 3])> {
        let name = first_reference_name(axis_param)?;
        if let Some((_, axis)) = self.sketch_axes.iter().find(|(id, _)| id == &name) {
            let dir = fd_normalize3(vec3_to_arr(axis.direction));
            return Some((vec3_to_arr(axis.point), dir));
        }
        // Fallback: a resident edge used as an axis — take its polyline endpoints.
        let poly = self.scene.edge_polyline_world(&name)?;
        let first = *poly.first()?;
        let last = *poly.last()?;
        let dir = fd_sub3(last, first);
        if fd_norm3(dir) < 1e-9 {
            return None;
        }
        Some((first, fd_normalize3(dir)))
    }

    /// A copy of `params` with each top-level STRING field evaluated against the
    /// history's `expressions` + `configurator` (the kernel `eval_expression`) and
    /// replaced by its finite numeric result — so an expression-valued param
    /// (e.g. `sizeX: "a + b"`) places its dimension at the resolved length.
    /// Non-numeric strings (ids, enum options) fail to eval and stay verbatim.
    fn resolve_param_expressions(&self, params: &serde_json::Value) -> serde_json::Value {
        let expressions = self.history.expressions();
        let configurator = self.history.configurator();
        let mut out = params.clone();
        if let Some(object) = out.as_object_mut() {
            for value in object.values_mut() {
                if let Some(source) = value.as_str() {
                    if let Ok(number) =
                        brep_kernel::eval_expression(&expressions, &configurator, source)
                    {
                        if number.is_finite() {
                            *value = serde_json::json!(number);
                        }
                    }
                }
            }
        }
        out
    }

    /// The dimension annotations for `feature_id` as JSON:
    /// `[{ fieldKey, pointA, pointB, value, label, mid }]` (world-space points;
    /// `mid` is the leader midpoint the app anchors the label at). `[]` when the
    /// feature type has no FD-1 builder.
    pub fn feature_dimension_annotations_json(&self, feature_id: &str) -> String {
        use crate::feature_dimensions::FeatureDimKind;
        let annotations = self.feature_dimension_annotations(feature_id);
        let wpp = self.camera.world_per_pixel();
        let out: Vec<serde_json::Value> = annotations
            .iter()
            .map(|a| {
                // The chip anchor: a linear leader's midpoint, or an angular arc's
                // mid-sweep point at the screen-constant radius (camera-dependent,
                // so computed here with the live `world_per_pixel`). `kind` lets the
                // app format the chip (`A 234°` for an angular value in DEGREES).
                let (kind, mid) = match a.kind {
                    FeatureDimKind::Linear => ("linear", a.midpoint()),
                    FeatureDimKind::Angular => {
                        ("angular", crate::feature_dimensions::angular_chip_anchor(a, wpp))
                    }
                };
                serde_json::json!({
                    "fieldKey": a.field_key,
                    "pointA": a.point_a,
                    "pointB": a.point_b,
                    "value": a.value,
                    "label": a.label,
                    "mid": mid,
                    "kind": kind,
                })
            })
            .collect();
        serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string())
    }

    /// The `{ mode, feature, annotations }` snapshot the headless verifier reads
    /// (published as `__brepFeatureDim`).
    pub fn feature_dimension_state_json(&self) -> String {
        let feature = self.dimension_armed_feature();
        let annotations: serde_json::Value = if feature.is_empty() {
            serde_json::json!([])
        } else {
            serde_json::from_str(&self.feature_dimension_annotations_json(&feature))
                .unwrap_or_else(|_| serde_json::json!([]))
        };
        serde_json::json!({
            "mode": self.gizmo_mode(),
            "feature": feature,
            "annotations": annotations,
        })
        .to_string()
    }

    /// The `set_overlay` JSON for the `feature-dim-leaders` group — the annotation
    /// leaders + arrowheads for the dimension-armed feature (empty when not in
    /// dimension mode, so a stale group is cleared).
    fn feature_dimension_overlay_json(&self) -> String {
        let feature = self.dimension_armed_feature();
        let annotations = if feature.is_empty() {
            Vec::new()
        } else {
            self.feature_dimension_annotations(&feature)
        };
        let (positions, colors) = crate::feature_dimensions::leaders_buffers(
            &annotations,
            self.camera.world_per_pixel(),
        );
        serde_json::json!({
            "groups": [
                {
                    "name": FEATURE_DIM_OVERLAY,
                    "renderOrder": 10003,
                    "tris": { "positions": positions, "colors": colors },
                }
            ]
        })
        .to_string()
    }

    /// (Re)project the dimension leaders onto the current geometry. Called on arm
    /// + after every param change (drag / value edit / rerun in dimension mode)
    /// + on a material ZOOM ([`Self::ensure_feature_dimension_overlay_current`]).
    /// Remembers the `world_per_pixel` it baked at, which is what lets that
    /// per-frame ensure fire on change ONLY.
    pub fn refresh_feature_dimension_overlay(&mut self) {
        let json = self.feature_dimension_overlay_json();
        let _ = self.set_overlay_json(&json);
        let wpp = self.camera.world_per_pixel();
        self.feature_dim_overlay_wpp = if wpp > 0.0 { wpp } else { f64::MIN_POSITIVE };
    }

    /// Clear the dimension overlay (an empty group), e.g. when disarming or
    /// switching to transform mode.
    pub(super) fn clear_feature_dimension_overlay(&mut self) {
        let _ = self.set_overlay_json(&serde_json::json!({
            "groups": [ { "name": FEATURE_DIM_OVERLAY } ]
        }).to_string());
        self.feature_dim_overlay_wpp = 0.0;
    }

    /// Per-frame upkeep for the DIMENSION gizmo (driven by
    /// [`Self::ensure_overlays_current`]).
    ///
    /// The group is baked into pre-expanded vertices at feed time, and its
    /// rod/cone/origin-sphere sizing — plus the angular arc's entire world
    /// RADIUS (`ANGLE_ARC_RAD_PX × world_per_pixel`) — is screen-constant. So a
    /// zoom that is not followed by a re-bake leaves the handles drawn at the old
    /// pixel size, and the angular sweep handle drawn at the old world position
    /// while [`Self::dimension_hit_regions`] (which projects against the LIVE
    /// camera) grabs at the new one: the outline and the drawn handle drift apart.
    ///
    /// Re-bakes ONLY on a material `world_per_pixel` change
    /// ([`overlay_wpp_stale`](super::overlay_wpp_stale)) — a quiet frame does not
    /// touch the overlay, so there is no per-frame dirty loop. Nothing armed →
    /// nothing baked, and the remembered zoom is dropped so re-arming re-bakes.
    pub(super) fn ensure_feature_dimension_overlay_current(&mut self) {
        if !matches!(self.transform_gizmo.mode, GizmoMode::Dimension)
            || self.dimension_armed_feature().is_empty()
        {
            self.feature_dim_overlay_wpp = 0.0;
            return;
        }
        let wpp = self.camera.world_per_pixel();
        if super::overlay_wpp_stale(self.feature_dim_overlay_wpp, wpp) {
            self.refresh_feature_dimension_overlay();
        }
    }

    /// Finalize a dimension drag with a fresh run of the committed parameters.
    /// Abandon background preview work first so releasing the arrow does not wait
    /// behind an expensive intermediate value. The final run forces this feature
    /// and its consumers to rebuild, rather than replaying the cached handle.
    pub fn feature_dimension_release(&mut self, feature_id: &str) {
        if self.history.index_of(feature_id).is_none() {
            return;
        }
        self.cancel_run_with_notice(false);
        self.rerun_history_forcing(Some(feature_id));
    }

    /// Drag a dimension handle: project the pointer pixel `(x, y)` onto the
    /// annotation's world axis (`pointA → pointB`), take the distance along the
    /// axis from `pointA` as the new value (correcting for any transform scale so
    /// the PARAM — not the scaled world length — is what changes), set the param,
    /// and re-run the history live. Degenerate projections (parallel ray / zero
    /// axis) no-op.
    pub fn feature_dimension_drag(&mut self, feature_id: &str, field_key: &str, x: f64, y: f64) {
        let annotations = self.feature_dimension_annotations(feature_id);
        let Some(annotation) = annotations.iter().find(|a| a.field_key == field_key) else {
            return;
        };
        if annotation.kind == crate::feature_dimensions::FeatureDimKind::Angular {
            if let Some(degrees) = self.angular_drag_degrees(annotation, x, y) {
                self.write_feature_dimension_param(feature_id, field_key, serde_json::json!(degrees));
                // Live-follow: re-bake the world-space leaders onto the rebuilt
                // geometry so the arc tracks the pointer this frame (Fix 3).
                self.refresh_feature_dimension_overlay();
            }
            return;
        }
        let a = annotation.point_a;
        let b = annotation.point_b;
        let axis = fd_sub3(b, a);
        let len = fd_norm3(axis);
        if len < 1e-9 {
            return;
        }
        let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
        let ray = self.camera.pick_ray(x, y);
        let ray_dir = fd_normalize3(ray.dir);
        let Some(t_world) = closest_t_on_axis(a, dir, ray.origin, ray_dir) else {
            return;
        };
        // World distance → param value: correct for the local axis scale via the
        // CURRENT ratio (world length / current param). Under unit scale this is
        // the identity; when the param is ~0 there is no ratio, so use the world
        // distance directly (unit-scale assumption).
        let scale_recip = if annotation.value.abs() > 1e-9 && len > 1e-9 {
            annotation.value / len
        } else {
            1.0
        };
        // Preserve SIGN so a linear dim can be dragged through the origin to the
        // negative side (a directional dim — cube size, height — then extends the
        // other way; the kernel takes |value| for magnitude dims). A small dead-zone
        // keeps it off an exact 0 (a degenerate extent the builders reject).
        let raw = t_world * scale_recip;
        let new_value = if raw >= 0.0 { raw.max(1e-4) } else { raw.min(-1e-4) };
        self.write_feature_dimension_param(feature_id, field_key, serde_json::json!(new_value));
        // Live-follow: re-bake the world-space leaders onto the rebuilt geometry so
        // the arrow tracks the pointer this frame (Fix 3).
        self.refresh_feature_dimension_overlay();
    }

    /// Map a pointer pixel to a swept angle (DEGREES) for an ANGULAR annotation:
    /// search the sweep for the degree whose arc-end projects nearest the pointer
    /// (a coarse 2° pass, then a ±2° refine at 0.25°), snap to 1°, clamp to
    /// `[-360, 360]`. Ported from the overlay `angle` drag. The magnitude is
    /// floored off exactly 0 so a torus `arc` drag never lands on 0 — which the
    /// kernel's `|| 360` falsy fallback would flip to a FULL torus mid-drag.
    /// `None` if the arc never projects in front of the camera.
    ///
    /// `pub(super)`: the assembly-constraint angle-arc drag
    /// (`assembly_overlay.rs`) maps its pointer through this SAME search so the
    /// two angle gizmos share one drag feel.
    pub(super) fn angular_drag_degrees(
        &self,
        ann: &crate::feature_dimensions::FeatureDimAnnotation,
        x: f64,
        y: f64,
    ) -> Option<f64> {
        let radius = crate::feature_dimensions::ANGLE_ARC_RAD_PX * self.camera.world_per_pixel();
        if radius <= 1e-9 {
            return None;
        }
        // A sweep of `deg` and `deg - 360` share the SAME arc-end world point, so
        // the screen-nearest search alone can't tell them apart at the wrap. Break
        // the tie toward the CURRENT value (angle-unwrap
        // continuity) with a tiny bias `~1e-6·Δ°²` — decisive only when screen
        // errors are essentially equal, negligible against any real pointer move.
        let current = ann.value;
        let combined = |screen_err: f64, deg: f64| -> f64 {
            let d = deg - current;
            screen_err + 1e-6 * d * d
        };
        let mut best_deg = current;
        let mut best_err = f64::INFINITY;
        // Coarse sweep over the full range.
        let mut deg = -360.0;
        while deg <= 360.0 {
            if let Some(err) = self.angle_arc_end_err(ann, radius, deg, x, y) {
                let err = combined(err, deg);
                if err < best_err {
                    best_err = err;
                    best_deg = deg;
                }
            }
            deg += 2.0;
        }
        if !best_err.is_finite() {
            return None;
        }
        // Refine around the coarse best.
        let center = best_deg;
        let mut deg = center - 2.0;
        while deg <= center + 2.0 {
            if (-360.0..=360.0).contains(&deg) {
                if let Some(err) = self.angle_arc_end_err(ann, radius, deg, x, y) {
                    let err = combined(err, deg);
                    if err < best_err {
                        best_err = err;
                        best_deg = deg;
                    }
                }
            }
            deg += 0.25;
        }
        let clamped = best_deg.round().clamp(-360.0, 360.0);
        let floored = if clamped.abs() < 0.1 {
            if clamped < 0.0 { -0.1 } else { 0.1 }
        } else {
            clamped
        };
        Some(floored)
    }

    /// Squared screen-pixel distance from `(x, y)` to the arc-end at `deg` for an
    /// angular annotation (`center + rotate(ref_dir, axis, deg) * radius`), or
    /// `None` when that point is behind the camera.
    fn angle_arc_end_err(
        &self,
        ann: &crate::feature_dimensions::FeatureDimAnnotation,
        radius: f64,
        deg: f64,
        x: f64,
        y: f64,
    ) -> Option<f64> {
        let dir = fd_normalize3(crate::feature_dimensions::rotate_about_axis(
            ann.ref_dir,
            ann.axis,
            deg.to_radians(),
        ));
        let p = [
            ann.center[0] + dir[0] * radius,
            ann.center[1] + dir[1] * radius,
            ann.center[2] + dir[2] * radius,
        ];
        let (sx, sy, depth) = self.camera.project(p);
        if depth <= 0.0 {
            return None;
        }
        Some((sx - x) * (sx - x) + (sy - y) * (sy - y))
    }

    /// Edit a dimension value from a label field: a plain numeric literal sets the
    /// param to that number; otherwise the input is treated as an EXPRESSION —
    /// evaluated LIVE against the history's `expressions` + `configurator` (the
    /// kernel `eval_expression`) and, on success, STORED as the expression string
    /// (the kernel re-evaluates it via `ctx.number`, so it stays live). A blank /
    /// bad-expression input no-ops (never corrupts the feature). Re-runs live.
    pub fn feature_dimension_set_value(&mut self, feature_id: &str, field_key: &str, input: &str) {
        let trimmed = input.trim();
        if trimmed.is_empty() {
            return;
        }
        if is_plain_number_literal(trimmed) {
            let Ok(number) = trimmed.parse::<f64>() else {
                return;
            };
            if !number.is_finite() {
                return;
            }
            self.write_feature_dimension_param(feature_id, field_key, serde_json::json!(number));
        } else {
            // Validate the expression before storing it (a bad expression no-ops).
            let expressions = self.history.expressions();
            let configurator = self.history.configurator();
            match brep_kernel::eval_expression(&expressions, &configurator, trimmed) {
                Ok(number) if number.is_finite() => {
                    self.write_feature_dimension_param(
                        feature_id,
                        field_key,
                        serde_json::Value::String(trimmed.to_string()),
                    );
                }
                _ => {}
            }
        }
    }

    /// Set one `inputParams` field of `feature_id` (a number or an expression
    /// string) and re-run the history (which re-projects the leaders in dimension
    /// mode). No-op when the feature is absent.
    fn write_feature_dimension_param(
        &mut self,
        feature_id: &str,
        field_key: &str,
        value: serde_json::Value,
    ) {
        let Some(index) = self.history.index_of(feature_id) else {
            return;
        };
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        let Some(object) = params.as_object_mut() else {
            return;
        };
        // A drag frame whose pointer has not moved (egui reports `dragged()`
        // on every frame the button is held, and the frame loop keeps running
        // while a run is in flight) resolves to the value already written. No
        // run for it: each submit is a whole-history rebuild on the runner.
        if object.get(field_key) == Some(&value) {
            return;
        }
        object.insert(field_key.to_string(), value);
        let _ = self.update_feature_params(feature_id, &params.to_string());
    }
}

// --- FD-1 geometry helpers (self-contained, `fd_` prefixed to avoid clashes) ---

fn fd_sub3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn fd_dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn fd_norm3(v: [f64; 3]) -> f64 {
    fd_dot3(v, v).sqrt()
}

fn fd_normalize3(v: [f64; 3]) -> [f64; 3] {
    let n = fd_norm3(v);
    if n < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

/// Push an origin-ball region at world `p` unless a ball already sits there (dedup
/// by world position, matching `leaders_buffers`' single drawn sphere). The shared
/// point-region builder projects it to a screen circle; a behind-eye (perspective)
/// origin is omitted — invisible, so not grabbable — matching the outline.
fn push_origin_region(
    out: &mut Vec<(DimRole, HitShape)>,
    seen: &mut Vec<[f64; 3]>,
    cam: &crate::view::ViewCamera,
    p: [f64; 3],
    px: f32,
    field_key: &str,
) {
    if seen.iter().any(|o| fd_norm3(fd_sub3(*o, p)) < 1e-6) {
        return;
    }
    seen.push(p);
    if let Some(shape) = point_region(cam, p, px) {
        out.push((DimRole::Origin(field_key.to_string()), shape));
    }
}

/// The parameter `t` of the point on the axis line `a + t*dir` (dir UNIT) closest
/// to the ray `ray_o + s*ray_d` (ray_d UNIT). `None` when the two are parallel
/// (no well-defined projection). `t` is the signed world distance along `dir`
/// from `a`. (`pub(super)`: shared with the constraint distance-arrow drag.)
pub(super) fn closest_t_on_axis(
    a: [f64; 3],
    dir: [f64; 3],
    ray_o: [f64; 3],
    ray_d: [f64; 3],
) -> Option<f64> {
    let w0 = fd_sub3(a, ray_o);
    let b = fd_dot3(dir, ray_d);
    let d = fd_dot3(dir, w0);
    let e = fd_dot3(ray_d, w0);
    let denom = 1.0 - b * b;
    if denom.abs() < 1e-9 {
        return None;
    }
    Some((b * e - d) / denom)
}

fn vec3_to_arr(v: brep_kernel::Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// The world CENTROID of a sketch profile — the average of its outer-loop curve
/// start points (the profile-polygon vertices), approximating the previous
/// face-average-center computation. Falls back to the sketch plane origin when
/// no outer loop is available. Used to anchor the extrude/revolve gizmos on the
/// geometry rather than at a possibly-far sketch-plane origin.
fn sketch_profile_centroid(profile: &brep_kernel::SketchProfile) -> [f64; 3] {
    let mut sum = [0.0f64; 3];
    let mut count = 0usize;
    if let Some(outer) = profile.regions.first().and_then(|region| region.first()) {
        for curve in &outer.curves {
            if let Ok(domain) = curve.domain() {
                if let Ok(point) = curve.evaluate(domain[0]) {
                    sum[0] += point.x;
                    sum[1] += point.y;
                    sum[2] += point.z;
                    count += 1;
                }
            }
        }
    }
    if count > 0 {
        [sum[0] / count as f64, sum[1] / count as f64, sum[2] / count as f64]
    } else {
        vec3_to_arr(profile.origin)
    }
}

