//! Draggable linear and angular annotations for selected feature parameters.
//!
//! Builders place local-space leaders on the feature's geometry and apply the
//! same intrinsic XYZ transform as the kernel, keeping annotations aligned with
//! the rebuilt solid. Each annotation identifies the parameter it edits.

use crate::geometry3d::{cross3, dot3, len3 as norm3, sub3};

use serde_json::Value;

use crate::engine_state::rotate_euler_xyz_f64;

/// A dimension annotation's kind. FD-1 was all [`FeatureDimKind::Linear`]; FD-2
/// adds [`FeatureDimKind::Angular`] (torus `arc`, revolve `angle`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FeatureDimKind {
    /// A linear distance from `point_a` → `point_b`.
    Linear,
    /// An angular sweep of `value` DEGREES about `axis`, measured in the plane
    /// ⟂ `axis` from the zero reference `ref_dir`, centered at `center`. Ported
    /// from the previous app's angle-annotation builder.
    Angular,
}

/// One editable dimension annotation.
///
/// LINEAR: a leader from world `point_a` → `point_b` whose length is the
/// (resolved) value of param `field_key`; dragging projects the pointer onto the
/// `a → b` axis.
///
/// ANGULAR: a screen-constant-radius ARC swept `value` degrees about `axis`
/// (unit), starting from `ref_dir` (unit, in the plane ⟂ `axis`), centered at
/// `center`; dragging maps the pointer to a swept angle. The `point_a`/`point_b`
/// / `midpoint()` linear surface is unused for an angular annotation (both are
/// set to `center`); the app anchors an angular chip at the arc's mid-sweep,
/// which is camera-dependent and computed engine-side.
#[derive(Clone, Debug)]
pub struct FeatureDimAnnotation {
    /// The `inputParams` key this annotation edits (e.g. `sizeX`, `radius`, `arc`).
    pub field_key: String,
    /// LINEAR: the dimension's start point in WORLD space (the leader anchor /
    /// drag base). ANGULAR: `center`.
    pub point_a: [f64; 3],
    /// LINEAR: the dimension's end point in WORLD space (the value-1 handle).
    /// ANGULAR: `center`.
    pub point_b: [f64; 3],
    /// The current (resolved) numeric value of `field_key` (a length for LINEAR,
    /// DEGREES for ANGULAR).
    pub value: f64,
    /// The short display prefix (`X`, `R`, `H`, `A`, `Arc`, …).
    pub label: String,
    /// The annotation kind.
    pub kind: FeatureDimKind,
    /// ANGULAR only: the arc center (the axis-plane vertex) in WORLD space.
    /// `[0;3]` for a linear annotation.
    pub center: [f64; 3],
    /// ANGULAR: the rotation AXIS (unit) the arc sweeps about. LINEAR: `[0;3]`
    /// for feature dims; the assembly DISTANCE overlays stash the BASE-FACE
    /// outward unit normal here (the signed drag axis — see
    /// [`crate::constraint_overlays`]'s perpendicular-foot construction).
    /// `leaders_buffers` ignores it for linear annotations either way.
    pub axis: [f64; 3],
    /// ANGULAR only: the sweep's ZERO reference (unit, in the plane ⟂ `axis`).
    /// `[0;3]` for a linear annotation.
    pub ref_dir: [f64; 3],
}

impl FeatureDimAnnotation {
    /// `pub(crate)`: the assembly-constraint overlay builder
    /// ([`crate::constraint_overlays`]) constructs the SAME annotation shape for
    /// distance constraints so `leaders_buffers` renders constraint arrows with
    /// byte-identical styling (UI-consistency directive — one arrow look).
    pub(crate) fn linear(field_key: &str, a: [f64; 3], b: [f64; 3], value: f64, label: &str) -> Self {
        Self {
            field_key: field_key.to_string(),
            point_a: a,
            point_b: b,
            value,
            label: label.to_string(),
            kind: FeatureDimKind::Linear,
            center: [0.0; 3],
            axis: [0.0; 3],
            ref_dir: [0.0; 3],
        }
    }

    /// An angular annotation: `value` DEGREES swept about `axis` from `ref_dir`,
    /// centered at `center`. `axis` is normalized and `ref_dir` is projected into
    /// the plane ⟂ `axis` then normalized (mirrors the overlay `#createAngle`
    /// pre-processing). A degenerate axis/ref falls back to an arbitrary basis so
    /// the annotation is always renderable.
    ///
    /// `pub(crate)`: also constructed by [`crate::constraint_overlays`] for angle
    /// constraints so the constraint arc reuses this exact gizmo styling.
    pub(crate) fn angular(
        field_key: &str,
        center: [f64; 3],
        axis: [f64; 3],
        ref_dir: [f64; 3],
        value: f64,
        label: &str,
    ) -> Self {
        let axis = normalize_or(axis, [0.0, 1.0, 0.0]);
        // Project the reference into the plane ⟂ axis, then normalize.
        let d = dot3(ref_dir, axis);
        let planar = [
            ref_dir[0] - axis[0] * d,
            ref_dir[1] - axis[1] * d,
            ref_dir[2] - axis[2] * d,
        ];
        let ref_dir = if norm3(planar) <= 1e-9 {
            arbitrary_perpendicular(axis)
        } else {
            normalize_or(planar, arbitrary_perpendicular(axis))
        };
        Self {
            field_key: field_key.to_string(),
            point_a: center,
            point_b: center,
            value,
            label: label.to_string(),
            kind: FeatureDimKind::Angular,
            center,
            axis,
            ref_dir,
        }
    }

    /// The world-space midpoint of the leader — where the app anchors a LINEAR
    /// label. (Angular chips anchor at the arc mid-sweep, computed engine-side
    /// with the camera's `world_per_pixel`.)
    pub fn midpoint(&self) -> [f64; 3] {
        [
            (self.point_a[0] + self.point_b[0]) * 0.5,
            (self.point_a[1] + self.point_b[1]) * 0.5,
            (self.point_a[2] + self.point_b[2]) * 0.5,
        ]
    }
}

/// Resolved scene geometry an annotation builder needs beyond the pure
/// `inputParams` — the profile plane (extrude/revolve) and the axis line
/// (revolve). These are NOT pure params (they resolve scene references), so the
/// engine resolves them from the run report's sketch profiles / axes, or from
/// the resident scene when the reference names a solid face / edge (see
/// `EngineState::feature_dimension_refs`), and hands them in. All fields are
/// optional: a builder that can't source what it needs returns `[]` gracefully.
#[derive(Clone, Debug, Default)]
pub struct ResolvedRefs {
    /// Extrude/revolve: the profile CENTER (world centroid of the outer loop) —
    /// the anchor the distance/angle gizmo hangs off, matching the previous app's
    /// resolved profile-reference-geometry center.
    pub profile_center: Option<[f64; 3]>,
    /// Extrude/revolve: the profile plane NORMAL (unit) — the sketch `+z` basis,
    /// or a face profile's OUTWARD normal (what the kernel's `face_profile` sets
    /// as `z_axis`) — the authoritative sweep/revolve-orientation direction.
    pub profile_normal: Option<[f64; 3]>,
    /// Revolve: a point on the resolved axis LINE (world).
    pub axis_point: Option<[f64; 3]>,
    /// Revolve: the resolved axis line DIRECTION (unit, UNORIENTED — the builder
    /// orients it toward the profile front via the `orient_revolve_axis` port).
    pub axis_dir: Option<[f64; 3]>,
    /// Plane (`P`): the resolved plane frame's ORIGIN (world) — the plane AFTER its
    /// `offset_distance`. The offset dim's un-offset base is `origin − normal·offset`.
    pub plane_origin: Option<[f64; 3]>,
    /// Plane (`P`): the resolved plane frame's unit NORMAL (its z-axis) — the axis
    /// the signed offset dim runs along.
    pub plane_normal: Option<[f64; 3]>,
    /// Plane (`P`): a small world length for the offset dim's handle stub when the
    /// offset is ~0 (a zero-length leader can't be dragged). Screen-constant so the
    /// handle is a consistent size; ignored once the offset is non-zero.
    pub plane_dim_length: Option<f64>,
}

/// Build the linear dimension annotations for a feature `type` from its
/// `input_params`. Dispatches on the primitive-solid type; returns `[]` for any
/// type without an FD-1 builder (extrude / revolve / booleans / etc.).
///
/// `input_params` should already have its numeric fields resolved to numbers
/// (the engine resolves expression strings against the history env before
/// calling this — see `EngineState::feature_dimension_annotations`), but plain
/// numeric strings are tolerated here too so the pure geometry stays testable.
pub fn build_annotations(feature_type: &str, input_params: &Value) -> Vec<FeatureDimAnnotation> {
    build_annotations_with_refs(feature_type, input_params, &ResolvedRefs::default())
}

/// Build the dimension annotations for a feature, given any resolved scene
/// references (`ResolvedRefs`) it needs. The primitive-solid + torus builders
/// ignore `resolved` (pure params + the baked transform); extrude/revolve read
/// the resolved profile plane / axis line and return `[]` when it is
/// unavailable (an unresolved reference — see the builders). The engine resolves
/// the profile plane from a sketch profile OR a resident solid face.
pub fn build_annotations_with_refs(
    feature_type: &str,
    input_params: &Value,
    resolved: &ResolvedRefs,
) -> Vec<FeatureDimAnnotation> {
    let transform = input_params.get("transform");
    match feature_type {
        "P.CU" => build_cube(input_params, transform),
        "P.CY" => build_cylinder(input_params, transform),
        "P.CO" => build_cone(input_params, transform),
        // The sphere feature type is `P.S` (the dispatch keys on `P.S`).
        "P.S" | "P.SP" => build_sphere(input_params, transform),
        "P.PY" => build_pyramid(input_params, transform),
        "P.T" => build_torus(input_params, transform),
        "E" => build_extrude(input_params, resolved),
        "R" => build_revolve(input_params, resolved),
        "P" => build_plane(input_params, resolved),
        _ => Vec::new(),
    }
}

/// Cube (`P.CU`): minimum corner at the local origin, extending `+sizeX/Y/Z`
/// (matches the kernel's `make_box_brep((0,0,0), …)`). Three linear dims from the
/// origin corner along each axis.
fn build_cube(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let sx = resolve_number(params, "sizeX");
    let sy = resolve_number(params, "sizeY");
    let sz = resolve_number(params, "sizeZ");
    let p0 = transform_point(transform, [0.0, 0.0, 0.0]);
    let px = transform_point(transform, [sx, 0.0, 0.0]);
    let py = transform_point(transform, [0.0, sy, 0.0]);
    let pz = transform_point(transform, [0.0, 0.0, sz]);
    vec![
        FeatureDimAnnotation::linear("sizeX", p0, px, sx, "X"),
        FeatureDimAnnotation::linear("sizeY", p0, py, sy, "Y"),
        FeatureDimAnnotation::linear("sizeZ", p0, pz, sz, "Z"),
    ]
}

/// Cylinder (`P.CY`): axis is local `+Y`, base at `y=0`, top at `y=height`;
/// radius along local `+X`. Two dims: radius (base → radial) + height (base → top).
fn build_cylinder(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let radius = resolve_number(params, "radius");
    let height = resolve_number(params, "height");
    let base = transform_point(transform, [0.0, 0.0, 0.0]);
    let top = transform_point(transform, [0.0, height, 0.0]);
    let radial = transform_point(transform, [radius, 0.0, 0.0]);
    vec![
        FeatureDimAnnotation::linear("radius", base, radial, radius, "R"),
        FeatureDimAnnotation::linear("height", base, top, height, "H"),
    ]
}

/// Cone (`P.CO`): base radius at `y=0`, top radius at `y=height` (both along
/// local `+X`), axis along `+Y`. Three dims: radiusBottom / radiusTop / height.
fn build_cone(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let radius_top = resolve_number(params, "radiusTop");
    let radius_bottom = resolve_number(params, "radiusBottom");
    let height = resolve_number(params, "height");
    let base_center = transform_point(transform, [0.0, 0.0, 0.0]);
    let top_center = transform_point(transform, [0.0, height, 0.0]);
    let base_radius = transform_point(transform, [radius_bottom, 0.0, 0.0]);
    let top_radius = transform_point(transform, [radius_top, height, 0.0]);
    vec![
        FeatureDimAnnotation::linear("radiusBottom", base_center, base_radius, radius_bottom, "Rb"),
        FeatureDimAnnotation::linear("radiusTop", top_center, top_radius, radius_top, "Rt"),
        FeatureDimAnnotation::linear("height", base_center, top_center, height, "H"),
    ]
}

/// Sphere (`P.S`): one radial dim from the center along local `+X`.
fn build_sphere(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let radius = resolve_number(params, "radius");
    let center = transform_point(transform, [0.0, 0.0, 0.0]);
    let radial = transform_point(transform, [radius, 0.0, 0.0]);
    vec![FeatureDimAnnotation::linear("radius", center, radial, radius, "R")]
}

/// Pyramid (`P.PY`): centered on its axis — base at `y = -h/2`, apex at `y = h/2`,
/// base edge spanning `±side/2` in local X (at `z = -side/2`). Two dims: the base
/// side edge + the axial height.
fn build_pyramid(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let side = resolve_number(params, "baseSideLength");
    let height = resolve_number(params, "height");
    let half_side = side * 0.5;
    let base_y = -height * 0.5;
    let apex_y = height * 0.5;
    let base_start = transform_point(transform, [-half_side, base_y, -half_side]);
    let base_end = transform_point(transform, [half_side, base_y, -half_side]);
    let base_center = transform_point(transform, [0.0, base_y, 0.0]);
    let apex = transform_point(transform, [0.0, apex_y, 0.0]);
    vec![
        FeatureDimAnnotation::linear("baseSideLength", base_start, base_end, side, "Side"),
        FeatureDimAnnotation::linear("height", base_center, apex, height, "H"),
    ]
}

/// Torus (`P.T`): centered on its axis (local `+Y`) at the origin; the tube
/// circle lies in the local X/Y plane centered at `(majorRadius, 0, 0)`, revolved
/// `arc` DEGREES about `+Y`. Two LINEAR dims — `majorRadius` (center → the tube
/// centerline along `+X`, `R`) and `tubeRadius` (the centerline → the outer wall,
/// `r`) — plus the `arc` sweep as an ANGULAR dim about the local `+Y` axis from
/// the `+X` reference. Ported verbatim from the previous torus-annotation builder.
fn build_torus(params: &Value, transform: Option<&Value>) -> Vec<FeatureDimAnnotation> {
    let major = resolve_number(params, "majorRadius");
    let tube = resolve_number(params, "tubeRadius");
    let arc = clamp_deg(resolve_number(params, "arc"));

    let center = transform_point(transform, [0.0, 0.0, 0.0]);
    let major_point = transform_point(transform, [major, 0.0, 0.0]);
    let tube_point = transform_point(transform, [major + tube, 0.0, 0.0]);
    // The axis / reference are the transformed local +Y / +X (subtract the
    // transformed origin then normalize, so translation cancels; a non-uniform
    // scale skews them — normalization handles magnitude).
    let axis = normalize_or(sub3(transform_point(transform, [0.0, 1.0, 0.0]), center), [0.0, 1.0, 0.0]);
    let start_dir = sub3(major_point, center);

    vec![
        FeatureDimAnnotation::linear("majorRadius", center, major_point, major, "R"),
        FeatureDimAnnotation::linear("tubeRadius", major_point, tube_point, tube, "r"),
        FeatureDimAnnotation::angular("arc", center, axis, start_dir, arc, "Arc"),
    ]
}

/// Extrude (`E`): a LINEAR distance dim from the profile CENTER along the profile
/// plane NORMAL by `distance` (`D`), plus the two-sided `distanceBack` leg along
/// `-normal` (`Db`). Needs the resolved profile center + normal (a sketch
/// profile's, or a resident solid face's — the engine sources both). Ported from
/// the previous extrude-annotation builder.
fn build_extrude(params: &Value, resolved: &ResolvedRefs) -> Vec<FeatureDimAnnotation> {
    let (Some(center), Some(normal)) = (resolved.profile_center, resolved.profile_normal) else {
        return Vec::new();
    };
    let normal = normalize_or(normal, [0.0, 0.0, 1.0]);
    let distance = resolve_number(params, "distance");
    let back = resolve_number(params, "distanceBack");
    let forward = [
        center[0] + normal[0] * distance,
        center[1] + normal[1] * distance,
        center[2] + normal[2] * distance,
    ];
    let backward = [
        center[0] - normal[0] * back,
        center[1] - normal[1] * back,
        center[2] - normal[2] * back,
    ];
    vec![
        FeatureDimAnnotation::linear("distance", center, forward, distance, "D"),
        FeatureDimAnnotation::linear("distanceBack", center, backward, back, "Db"),
    ]
}

/// Plane (`P`): one SIGNED LINEAR offset dim along the plane normal, driving
/// `offset_distance`. The plane sits on either side of its base orientation/
/// reference plane, so the dim runs from the un-offset base
/// (`origin − normal·offset`) to the current plane (`origin`) and is dragged
/// through zero to flip sign (the drag preserves sign; the kernel accepts a
/// negative `offset_distance`). At offset ≈ 0 the leader collapses, so a small
/// `+normal` stub (`plane_dim_length`, screen-constant) keeps it draggable — the
/// value mapping stays 1:1 there. No resolved plane frame → `[]`.
fn build_plane(params: &Value, resolved: &ResolvedRefs) -> Vec<FeatureDimAnnotation> {
    let (Some(origin), Some(normal)) = (resolved.plane_origin, resolved.plane_normal) else {
        return Vec::new();
    };
    let offset = resolve_number(params, "offset_distance");
    // The un-offset base plane point — the dim's zero anchor.
    let base = [
        origin[0] - normal[0] * offset,
        origin[1] - normal[1] * offset,
        origin[2] - normal[2] * offset,
    ];
    // Handle at the current offset; at ~0 use a small +normal stub so the leader
    // has a direction (the drag no-ops on a zero-length axis).
    let extent = if offset.abs() > 1e-6 {
        offset
    } else {
        resolved.plane_dim_length.unwrap_or(1.0)
    };
    let handle = [
        base[0] + normal[0] * extent,
        base[1] + normal[1] * extent,
        base[2] + normal[2] * extent,
    ];
    vec![FeatureDimAnnotation::linear(
        "offset_distance",
        base,
        handle,
        offset,
        "Offset",
    )]
}

/// Revolve (`R`): one ANGULAR dim = `angle` DEGREES swept about the resolved axis
/// line, oriented toward the profile front (the `orient_revolve_axis` port — so
/// the arc rotates the SAME way the solid does), centered at the axis point
/// nearest the profile, zeroed on the radial from that vertex to the profile.
/// Needs the resolved axis line + profile; returns `[]` otherwise. Ported from
/// the previous revolve-annotation builder.
fn build_revolve(params: &Value, resolved: &ResolvedRefs) -> Vec<FeatureDimAnnotation> {
    let (Some(axis_point), Some(axis_dir)) = (resolved.axis_point, resolved.axis_dir) else {
        return Vec::new();
    };
    let Some(profile_center) = resolved.profile_center else {
        return Vec::new();
    };
    let axis = orient_revolve_axis(axis_dir, axis_point, profile_center, resolved.profile_normal);
    let vertex = closest_point_on_line(profile_center, axis_point, axis);
    // The radial from the vertex to the profile (projected ⟂ axis) is the zero
    // reference; the `angular` ctor re-projects + falls back if it degenerates.
    let start_dir = sub3(profile_center, vertex);
    let angle = clamp_deg(resolve_number(params, "angle"));
    vec![FeatureDimAnnotation::angular("angle", vertex, axis, start_dir, angle, "A")]
}

/// The signed revolve axis native Revolve uses: the profile's outward normal
/// selects between the two directions of an unoriented axis edge. Port of
/// `resolveOrientedRevolveAxisDirection`.
pub(crate) fn orient_revolve_axis(
    axis_dir: [f64; 3],
    axis_point: [f64; 3],
    profile_center: [f64; 3],
    profile_normal: Option<[f64; 3]>,
) -> [f64; 3] {
    let axis = normalize_or(axis_dir, [0.0, 1.0, 0.0]);
    let Some(normal) = profile_normal else {
        return axis;
    };
    if norm3(normal) <= 1e-12 {
        return axis;
    }
    let normal = normalize_or(normal, [0.0, 0.0, 1.0]);
    // radial = (profileCenter - axisPoint) projected ⟂ axis.
    let mut radial = sub3(profile_center, axis_point);
    let d = dot3(radial, axis);
    radial = [radial[0] - axis[0] * d, radial[1] - axis[1] * d, radial[2] - axis[2] * d];
    if norm3(radial) <= 1e-12 {
        return axis;
    }
    let c = cross3(axis, radial);
    if dot3(c, normal) < 0.0 {
        [-axis[0], -axis[1], -axis[2]]
    } else {
        axis
    }
}

/// The point on line `(line_point, line_dir)` closest to `point`.
pub(crate) fn closest_point_on_line(
    point: [f64; 3],
    line_point: [f64; 3],
    line_dir: [f64; 3],
) -> [f64; 3] {
    let dir = normalize_or(line_dir, [0.0, 1.0, 0.0]);
    let t = dot3(sub3(point, line_point), dir);
    [
        line_point[0] + dir[0] * t,
        line_point[1] + dir[1] * t,
        line_point[2] + dir[2] * t,
    ]
}

/// Clamp a degree value to `[-360, 360]`.
fn clamp_deg(v: f64) -> f64 {
    v.clamp(-360.0, 360.0)
}

/// Apply a feature's `inputParams.transform` (TRS, `rotationEuler` in DEGREES,
/// `M = T·R·S`) to a LOCAL point → WORLD. Mirrors composing that degree-based TRS
/// matrix and applying it to a point: `world = position + R·(scale ⊙ local)`, with `R` the exact
/// intrinsic XYZ Euler order matrix the kernel bake uses (`rotate_euler_xyz_f64`).
pub(crate) fn transform_point(transform: Option<&Value>, local: [f64; 3]) -> [f64; 3] {
    let position = read_vec3(transform, "position", [0.0, 0.0, 0.0]);
    let rotation_deg = read_vec3(transform, "rotationEuler", [0.0, 0.0, 0.0]);
    let scale = read_vec3(transform, "scale", [1.0, 1.0, 1.0]);
    let scaled = [local[0] * scale[0], local[1] * scale[1], local[2] * scale[2]];
    let euler = [
        rotation_deg[0].to_radians(),
        rotation_deg[1].to_radians(),
        rotation_deg[2].to_radians(),
    ];
    let rotated = rotate_euler_xyz_f64(scaled, euler);
    [
        rotated[0] + position[0],
        rotated[1] + position[1],
        rotated[2] + position[2],
    ]
}

/// Read a `[x, y, z]` from a `transform` sub-field (numbers only; a missing /
/// short array keeps the per-index default).
fn read_vec3(transform: Option<&Value>, key: &str, default: [f64; 3]) -> [f64; 3] {
    crate::json_support::vec3_or(transform.and_then(|t| t.get(key)), default)
}

/// Read `params[key]` as a finite number: a JSON number, else a plain numeric
/// string (e.g. `"12.5"`). Non-numeric / expression strings resolve to `0.0`
/// (the engine pre-resolves expressions before building, so this is only the
/// pure-geometry fallback). Mirrors the numeric input-param resolution fallback.
fn resolve_number(params: &Value, key: &str) -> f64 {
    match params.get(key) {
        Some(Value::Number(n)) => n.as_f64().filter(|v| v.is_finite()).unwrap_or(0.0),
        Some(Value::String(s)) => s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(0.0),
        _ => 0.0,
    }
}

// --- restyled leader geometry (matches the reference dimension-arrows image) --
//
// Each annotation draws a thick SILVER rod (a 3D tube) from the shared origin
// `point_a` out to `point_b`, an ORANGE cone arrowhead at `point_b`, and a
// single ORANGE origin sphere at the shared start point (deduped across the
// annotations that share it — a cube's three axis dims share one corner). The
// geometry is radially symmetric so it needs no camera orientation; it is fed to
// the `feature-dim-leaders` overlay group as flat triangle buffers. Colors are
// display sRGB written ~directly by the overlay shader (with a per-face shade for
// depth), so use hex/255 — no linear conversion.

/// Silver-grey rod shaft color (~0xccced1) — also the angle ARC tube.
const SHAFT_RGB: [f32; 3] = [0.80, 0.81, 0.82];
/// Orange cone + origin/handle-sphere color (#F5A623).
const ORANGE_RGB: [f32; 3] = [0.961, 0.651, 0.137];
/// Red — the angle gizmo's ZERO-reference (drawn DASHED) radial line.
const RED_RGB: [f32; 3] = [0.902, 0.157, 0.157];
/// Green — the angle gizmo's rotation-AXIS line.
const GREEN_RGB: [f32; 3] = [0.204, 0.808, 0.267];

/// Silver-rod shaft radius, CSS pixels (thick, reads as a 3D rod).
const SHAFT_RAD_PX: f64 = 2.2;
/// Arrowhead cone length, CSS pixels.
const CONE_LEN_PX: f64 = 16.0;
/// Arrowhead cone base radius, CSS pixels (fuller 3D cone).
const CONE_RAD_PX: f64 = 6.0;
/// Origin sphere radius, CSS pixels (medium, screen-constant). Shared with
/// `EngineState::dimension_origin_pick` so the click hit-radius matches the drawn
/// sphere exactly.
pub(crate) const ORIGIN_SPHERE_RAD_PX: f64 = 7.0;

/// The angle gizmo's ARC radius, CSS pixels (screen-constant — matching the previous
/// `FEATURE_ANGLE_RADIUS_PX`). Shared with the engine so the drawn arc, the chip
/// anchor (mid-sweep) and the drag hit-search all use the SAME radius.
pub const ANGLE_ARC_RAD_PX: f64 = 120.0;
/// The angle gizmo's ref/axis line radius, CSS pixels (thinner than the arc).
const ANGLE_RAY_RAD_PX: f64 = 1.6;
/// Degrees of sweep per arc tube segment (tessellation of the arc).
const ARC_DEG_PER_SEG: f64 = 4.0;
/// The RED zero-reference line's dash / gap length, CSS pixels.
const DASH_LEN_PX: f64 = 6.0;
const DASH_GAP_PX: f64 = 5.0;

const TUBE_SEGMENTS: usize = 8;
const CONE_SEGMENTS: usize = 16;
const SPHERE_RINGS: usize = 6;
const SPHERE_SECTORS: usize = 10;

/// Build the world-space leader geometry for a set of annotations as flat
/// triangle `(positions, colors)` buffers (9 position + 9 color floats per
/// triangle), ready to feed the `feature-dim-leaders` overlay group as `tris`.
/// Normals are omitted — the overlay parser computes a flat face normal per
/// triangle, and the shader's per-face shade gives the rods/cones/spheres their
/// 3D read. `world_per_pixel` keeps the rod/cone/sphere screen-constant.
pub fn leaders_buffers(
    annotations: &[FeatureDimAnnotation],
    world_per_pixel: f64,
) -> (Vec<f32>, Vec<f32>) {
    let mut tb = TriBuf::default();
    let shaft_rad = SHAFT_RAD_PX * world_per_pixel;
    let cone_len = CONE_LEN_PX * world_per_pixel;
    let cone_rad = CONE_RAD_PX * world_per_pixel;
    let sphere_rad = ORIGIN_SPHERE_RAD_PX * world_per_pixel;

    // Draw the shared origin sphere once per distinct start point.
    let mut origins: Vec<[f64; 3]> = Vec::new();
    let mut add_origin = |tb: &mut TriBuf, a: [f64; 3]| {
        if !origins.iter().any(|o| norm3(sub3(*o, a)) < 1e-6) {
            push_sphere(tb, a, sphere_rad, ORANGE_RGB);
            origins.push(a);
        }
    };

    for ann in annotations {
        match ann.kind {
            FeatureDimKind::Angular => {
                // The arc VERTEX gets the same orange origin sphere as a linear
                // dim's `point_a`, so it is a visible mode-toggle target (the arc's
                // sweep-END orange sphere is the angle DRAG handle, not a toggle).
                // `add_origin` dedups against a linear origin at the same world
                // point (a torus's `majorRadius` origin == this center), so no
                // doubled geometry there.
                add_origin(&mut tb, ann.center);
                push_angle_gizmo(&mut tb, ann, world_per_pixel);
            }
            FeatureDimKind::Linear => {
                let a = ann.point_a;
                let b = ann.point_b;
                let axis = sub3(b, a);
                let len = norm3(axis);
                add_origin(&mut tb, a);
                if len < 1e-9 {
                    continue;
                }
                let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
                // The cone occupies the far end; the rod runs from origin to it.
                let cl = cone_len.min(len * 0.9);
                let shaft_end = [b[0] - dir[0] * cl, b[1] - dir[1] * cl, b[2] - dir[2] * cl];
                push_tube(&mut tb, a, shaft_end, shaft_rad, SHAFT_RGB);
                push_cone(&mut tb, shaft_end, b, cone_rad, ORANGE_RGB);
            }
        }
    }
    (tb.positions, tb.colors)
}

/// Append a PLAIN leader line `a → b` (thin silver rod, NO arrowhead cone / origin
/// sphere — nothing that reads as grabbable) onto existing `(positions, colors)`
/// triangle buffers. The NON-dimensional assembly-constraint overlays (coincident /
/// parallel / …) draw their anchor-to-anchor leaders through this so the line
/// styling stays in this ONE home (same silver + screen-constant radius as the
/// dimension rods, slightly thinner because it carries no handle).
pub fn append_plain_leader(
    positions: &mut Vec<f32>,
    colors: &mut Vec<f32>,
    a: [f64; 3],
    b: [f64; 3],
    world_per_pixel: f64,
) {
    let mut tb = TriBuf {
        positions: std::mem::take(positions),
        colors: std::mem::take(colors),
    };
    push_tube(&mut tb, a, b, ANGLE_RAY_RAD_PX * world_per_pixel, SHAFT_RGB);
    *positions = tb.positions;
    *colors = tb.colors;
}

/// A flat triangle-soup accumulator (positions + per-vertex rgb colors).
#[derive(Default)]
struct TriBuf {
    positions: Vec<f32>,
    colors: Vec<f32>,
}

impl TriBuf {
    fn tri(&mut self, a: [f64; 3], b: [f64; 3], c: [f64; 3], rgb: [f32; 3]) {
        for p in [a, b, c] {
            self.positions
                .extend_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32]);
            self.colors.extend_from_slice(&rgb);
        }
    }
}

/// Push a solid 3D rod (open-ended tube) from `a` to `b` with world `radius`.
fn push_tube(tb: &mut TriBuf, a: [f64; 3], b: [f64; 3], radius: f64, rgb: [f32; 3]) {
    let axis = sub3(b, a);
    let len = norm3(axis);
    if len < 1e-9 || radius <= 0.0 {
        return;
    }
    let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
    let (u, v) = axis_basis(dir);
    let ring = |center: [f64; 3], k: usize| -> [f64; 3] {
        let ang = (k as f64 / TUBE_SEGMENTS as f64) * std::f64::consts::TAU;
        let (c, s) = (ang.cos() * radius, ang.sin() * radius);
        [
            center[0] + u[0] * c + v[0] * s,
            center[1] + u[1] * c + v[1] * s,
            center[2] + u[2] * c + v[2] * s,
        ]
    };
    for k in 0..TUBE_SEGMENTS {
        let a0 = ring(a, k);
        let a1 = ring(a, k + 1);
        let b0 = ring(b, k);
        let b1 = ring(b, k + 1);
        tb.tri(a0, b0, b1, rgb);
        tb.tri(a0, b1, a1, rgb);
    }
}

/// Push a filled arrowhead cone: apex at `tip`, base circle of world `radius`
/// centered at `base` (side facets + a base cap).
fn push_cone(tb: &mut TriBuf, base: [f64; 3], tip: [f64; 3], radius: f64, rgb: [f32; 3]) {
    let axis = sub3(tip, base);
    let len = norm3(axis);
    if len < 1e-9 || radius <= 0.0 {
        return;
    }
    let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
    let (u, v) = axis_basis(dir);
    let ring = |k: usize| -> [f64; 3] {
        let ang = (k as f64 / CONE_SEGMENTS as f64) * std::f64::consts::TAU;
        let (c, s) = (ang.cos() * radius, ang.sin() * radius);
        [
            base[0] + u[0] * c + v[0] * s,
            base[1] + u[1] * c + v[1] * s,
            base[2] + u[2] * c + v[2] * s,
        ]
    };
    let mut prev = ring(0);
    for k in 1..=CONE_SEGMENTS {
        let cur = ring(k);
        tb.tri(tip, prev, cur, rgb); // side facet
        tb.tri(base, cur, prev, rgb); // base cap
        prev = cur;
    }
}

/// Push a filled UV sphere of world `radius` at `center` (flat-shaded facets).
fn push_sphere(tb: &mut TriBuf, center: [f64; 3], radius: f64, rgb: [f32; 3]) {
    if radius <= 0.0 {
        return;
    }
    let point = |ring: usize, sector: usize| -> [f64; 3] {
        let lat = std::f64::consts::PI * (ring as f64 / SPHERE_RINGS as f64)
            - std::f64::consts::FRAC_PI_2;
        let lon = std::f64::consts::TAU * (sector as f64 / SPHERE_SECTORS as f64);
        [
            center[0] + lat.cos() * lon.cos() * radius,
            center[1] + lat.cos() * lon.sin() * radius,
            center[2] + lat.sin() * radius,
        ]
    };
    for r in 0..SPHERE_RINGS {
        for sct in 0..SPHERE_SECTORS {
            let p00 = point(r, sct);
            let p01 = point(r, sct + 1);
            let p10 = point(r + 1, sct);
            let p11 = point(r + 1, sct + 1);
            tb.tri(p00, p10, p11, rgb);
            tb.tri(p00, p11, p01, rgb);
        }
    }
}

/// Push the angle gizmo (image-9 target) for an ANGULAR annotation: a light-grey
/// ARC of screen-constant radius swept from `ref_dir` by `value` degrees about
/// `axis` at `center`, an ORANGE handle SPHERE at the sweep end with an ORANGE
/// CONE just past it along the arc tangent, a RED DASHED zero-reference line along
/// `ref_dir`, and a GREEN line along `axis`. Sizing is screen-constant via
/// `world_per_pixel` so the gizmo stays a fixed pixel size across zoom.
fn push_angle_gizmo(tb: &mut TriBuf, ann: &FeatureDimAnnotation, world_per_pixel: f64) {
    let center = ann.center;
    let axis = ann.axis;
    let start = ann.ref_dir;
    let radius = ANGLE_ARC_RAD_PX * world_per_pixel;
    let ray_rad = ANGLE_RAY_RAD_PX * world_per_pixel;
    let shaft_rad = SHAFT_RAD_PX * world_per_pixel;
    let cone_len = CONE_LEN_PX * world_per_pixel;
    let cone_rad = CONE_RAD_PX * world_per_pixel;
    let sphere_rad = ORIGIN_SPHERE_RAD_PX * world_per_pixel;
    if radius <= 1e-9 {
        return;
    }
    // A full 360° arc would close on itself; clamp the DRAWN sweep just under it
    // (matches the overlay's ±359.9 draw clamp) while the chip still shows the
    // real value.
    let value = ann.value.clamp(-359.9, 359.9);
    let value_rad = value.to_radians();

    // The arc: sample from 0 → value and connect consecutive points with grey
    // tube segments. The point at parameter `t` (radians) is
    // `center + rotate(start, axis, t) * radius`.
    let arc_point = |t: f64| -> [f64; 3] {
        let dir = rotate_about_axis(start, axis, t);
        [
            center[0] + dir[0] * radius,
            center[1] + dir[1] * radius,
            center[2] + dir[2] * radius,
        ]
    };
    let seg_count = ((value.abs() / ARC_DEG_PER_SEG).ceil() as usize).max(2);
    let mut prev = arc_point(0.0);
    for k in 1..=seg_count {
        let t = value_rad * (k as f64 / seg_count as f64);
        let cur = arc_point(t);
        push_tube(tb, prev, cur, shaft_rad, SHAFT_RGB);
        prev = cur;
    }

    // The sweep END: the orange handle sphere sits on the arc, the cone points
    // just past it along the arc tangent (the direction of increasing angle).
    let dir_end = rotate_about_axis(start, axis, value_rad);
    let end_pt = [
        center[0] + dir_end[0] * radius,
        center[1] + dir_end[1] * radius,
        center[2] + dir_end[2] * radius,
    ];
    push_sphere(tb, end_pt, sphere_rad, ORANGE_RGB);
    // Tangent = d/dt rotate = axis × dir_end, signed by the sweep direction.
    let sweep_sign = if value < 0.0 { -1.0 } else { 1.0 };
    let tangent = normalize_or(cross3(axis, dir_end), dir_end);
    let tangent = [tangent[0] * sweep_sign, tangent[1] * sweep_sign, tangent[2] * sweep_sign];
    let cone_tip = [
        end_pt[0] + tangent[0] * cone_len,
        end_pt[1] + tangent[1] * cone_len,
        end_pt[2] + tangent[2] * cone_len,
    ];
    push_cone(tb, end_pt, cone_tip, cone_rad, ORANGE_RGB);

    // RED DASHED zero-reference line from the center out along `ref_dir` to the
    // arc-start radius (short tube dashes with gaps).
    let ref_end = [
        center[0] + start[0] * radius,
        center[1] + start[1] * radius,
        center[2] + start[2] * radius,
    ];
    push_dashed(tb, center, ref_end, ray_rad, RED_RGB, world_per_pixel);

    // GREEN rotation-axis line through the center (a solid tube, both sides).
    let axis_len = radius * 0.7;
    let axis_a = [
        center[0] - axis[0] * axis_len,
        center[1] - axis[1] * axis_len,
        center[2] - axis[2] * axis_len,
    ];
    let axis_b = [
        center[0] + axis[0] * axis_len,
        center[1] + axis[1] * axis_len,
        center[2] + axis[2] * axis_len,
    ];
    push_tube(tb, axis_a, axis_b, ray_rad, GREEN_RGB);
}

/// Push a DASHED line `a → b` as a series of short solid tube segments (dash then
/// gap, in screen-constant CSS px). Used for the angle gizmo's red zero-reference.
fn push_dashed(
    tb: &mut TriBuf,
    a: [f64; 3],
    b: [f64; 3],
    radius: f64,
    rgb: [f32; 3],
    world_per_pixel: f64,
) {
    let axis = sub3(b, a);
    let len = norm3(axis);
    if len < 1e-9 {
        return;
    }
    let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
    let dash = (DASH_LEN_PX * world_per_pixel).max(1e-6);
    let gap = (DASH_GAP_PX * world_per_pixel).max(1e-6);
    let mut s = 0.0;
    while s < len {
        let e = (s + dash).min(len);
        let p0 = [a[0] + dir[0] * s, a[1] + dir[1] * s, a[2] + dir[2] * s];
        let p1 = [a[0] + dir[0] * e, a[1] + dir[1] * e, a[2] + dir[2] * e];
        push_tube(tb, p0, p1, radius, rgb);
        s = e + gap;
    }
}

/// The world-space chip anchor for an ANGULAR annotation: the arc mid-sweep point
/// at the screen-constant radius (`center + rotate(ref_dir, axis, value/2) *
/// radius`) — i.e. `labelAnchor = vertex + bisector * radius`. Camera-dependent
/// (via `world_per_pixel`), so the engine computes it per frame.
pub fn angular_chip_anchor(ann: &FeatureDimAnnotation, world_per_pixel: f64) -> [f64; 3] {
    let radius = ANGLE_ARC_RAD_PX * world_per_pixel;
    let value = ann.value.clamp(-359.9, 359.9);
    let bisector = rotate_about_axis(ann.ref_dir, ann.axis, (value * 0.5).to_radians());
    [
        ann.center[0] + bisector[0] * radius,
        ann.center[1] + bisector[1] * radius,
        ann.center[2] + bisector[2] * radius,
    ]
}

/// Screen-px hit radius for grabbing a dimension arrowHEAD — the cone base plus a
/// little slack, so a click near the drawn arrowhead reliably grabs it. Shared by
/// `EngineState::dimension_arrow_pick`.
// Grab tolerance around a dimension arrowhead. Generous on purpose: egui only
// reports a DRAG once the pointer has already moved a few px past the press, so a
// tight radius makes the arrow feel un-grabbable. This gives a comfortable target.
pub(crate) const ARROW_HANDLE_HIT_RAD_PX: f64 = CONE_RAD_PX + 12.0;

/// The world-space arrowHEAD handle point of an annotation — the drag grab target
/// (`EngineState::dimension_arrow_pick`). LINEAR: the orange cone tip at
/// `point_b`. ANGULAR: the orange sweep-END handle sphere on the arc (`center +
/// rotate(ref_dir, axis, value°) * radius`, value clamped to the drawn ±359.9° so
/// the grab point matches the drawn handle). Camera-dependent for angular (via
/// `world_per_pixel`), so it is computed per frame.
pub(crate) fn arrow_handle_point(
    ann: &FeatureDimAnnotation,
    world_per_pixel: f64,
) -> [f64; 3] {
    match ann.kind {
        FeatureDimKind::Linear => ann.point_b,
        FeatureDimKind::Angular => {
            let radius = ANGLE_ARC_RAD_PX * world_per_pixel;
            let value = ann.value.clamp(-359.9, 359.9);
            let dir = rotate_about_axis(ann.ref_dir, ann.axis, value.to_radians());
            [
                ann.center[0] + dir[0] * radius,
                ann.center[1] + dir[1] * radius,
                ann.center[2] + dir[2] * radius,
            ]
        }
    }
}

/// Normalize `v`, or return `fallback` if `v` is ~zero-length.
fn normalize_or(v: [f64; 3], fallback: [f64; 3]) -> [f64; 3] {
    let n = norm3(v);
    if n < 1e-12 {
        fallback
    } else {
        [v[0] / n, v[1] / n, v[2] / n]
    }
}

/// A stable unit vector ⟂ `direction` (a port of the previous app's arbitrary-perpendicular helper).
fn arbitrary_perpendicular(direction: [f64; 3]) -> [f64; 3] {
    if norm3(direction) <= 1e-12 {
        return [0.0, 0.0, 1.0];
    }
    let seed = if dot3(direction, [0.0, 0.0, 1.0]).abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let mut perp = cross3(direction, seed);
    if norm3(perp) <= 1e-12 {
        perp = cross3(direction, [1.0, 0.0, 0.0]);
    }
    if norm3(perp) <= 1e-12 {
        [1.0, 0.0, 0.0]
    } else {
        normalize_or(perp, [1.0, 0.0, 0.0])
    }
}

/// Rotate `v` by `angle` radians about unit `axis` (Rodrigues). Shared with the
/// engine's angular drag/chip anchoring.
pub fn rotate_about_axis(v: [f64; 3], axis: [f64; 3], angle: f64) -> [f64; 3] {
    crate::geometry3d::rotate3(v, normalize_or(axis, [0.0, 1.0, 0.0]), angle)
}

/// A radially-symmetric perpendicular basis `(u, v)` for a unit `dir`.
fn axis_basis(dir: [f64; 3]) -> ([f64; 3], [f64; 3]) {
    let seed = if dir[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let mut u = cross3(dir, seed);
    let un = norm3(u);
    if un < 1e-9 {
        u = [0.0, 1.0, 0.0];
    } else {
        u = [u[0] / un, u[1] / un, u[2] / un];
    }
    let v = cross3(dir, u);
    let vn = norm3(v).max(1e-9);
    (u, [v[0] / vn, v[1] / vn, v[2] / vn])
}

