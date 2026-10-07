//! Assembly-constraint VIEWPORT overlays — the CONSTRAINT twin of
//! [`crate::feature_dimensions`].
//!
//! Pure builders: the kernel's `assembly_overlay_json` rows (per-constraint world
//! anchors / directions / status / measured value) + the `assembly_state_json`
//! constraint list (for `inputParams` — expression detection + element refs) fold
//! into [`ConstraintOverlay`] records, which bake into the SAME leader/arrow/arc
//! triangle buffers the feature-dimension gizmo draws (via
//! [`crate::feature_dimensions::leaders_buffers`] — nothing re-rolled, per the
//! UI-consistency directive):
//!
//! * **distance** → a LINEAR dimension annotation — the silver rod + orange
//!   cone arrow + orange origin sphere, GRABBABLE (drag edits
//!   `inputParams.distance`, commit auto-solves). Plane-based pairings draw the
//!   TRUE dimension perpendicular to the BASE face (the perpendicular-foot
//!   construction on [`build_distance_annotation`], matching the kernel's
//!   signed `d = (P_other − P_base)·n̂_base` convention); plane-less pairings
//!   keep the plain anchor-to-anchor leader.
//! * **angle** → an ANGULAR annotation (the screen-constant arc + orange sweep-end
//!   handle + red dashed zero reference + green axis), GRABBABLE (drag edits
//!   `inputParams.angle`).
//! * **center** → two plain leaders drawn BY ROLE from the row's `groups`
//!   (the kernel mapper's inferred `[width pair, tab]` element-index groups):
//!   the width span between its two faces, and a leader from that span's
//!   midpoint (which lies on the mid-plane) to the tab's anchor centroid; the
//!   label sits at the width midpoint. Pick order never shapes the drawing.
//! * everything else (coincident / parallel / perpendicular / concentric /
//!   tangent / touch_align / fixed) → a plain anchor-to-anchor leader line +
//!   label anchor only — never a handle.
//!
//! Dragging is DISABLED for a distance/angle whose param is a non-numeric
//! EXPRESSION string (matches how feature dimensions treat expression-driven
//! params); the engine consults [`ConstraintOverlay::draggable`].
//!
//! The interactive state machine (hit regions, drag preview, the
//! `assembly_update_constraint_json` commit) lives in
//! `engine_state/assembly_overlay.rs`; this module stays camera-free and pure so
//! the geometry is unit-testable from canned JSON payloads.

use serde_json::Value;

use crate::feature_dimensions::{
    append_plain_leader, leaders_buffers, FeatureDimAnnotation,
};

/// Which overlay family a constraint renders as.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConstraintOverlayKind {
    /// A grabbable linear dimension arrow pair (`distance`).
    Distance,
    /// A grabbable angle arc (`angle`).
    Angle,
    /// A non-interactive leader + label (every other type).
    Leader,
}

/// One constraint's overlay record: identity + status for the label, the
/// resolved world geometry, and (for the dimensional kinds) the annotation the
/// shared leader renderer draws + the drag machinery grabs.
#[derive(Clone, Debug)]
pub struct ConstraintOverlay {
    /// The constraint id (`DIST3`, `ANGL2`, …) — the mutation-ABI key.
    pub id: String,
    /// The constraint `type` string (`distance`, `coincident`, …).
    pub constraint_type: String,
    /// The type's icon glyph (`brep_kernel::ConstraintTypeDef::icon`) — what
    /// the viewport chip shows in place of the id. Empty for an unknown type,
    /// in which case the chip falls back to the id.
    pub icon: String,
    /// The solve status (`satisfied` / `adjusted` / `error` / …) — drives the
    /// label color via [`status_color`].
    pub status: String,
    /// The human status/solve message (label tooltip).
    pub message: String,
    /// The overlay family.
    pub kind: ConstraintOverlayKind,
    /// The resolved WORLD anchor points (one per selection; empty when the
    /// kernel could not resolve the selections — label-less, geometry-less row).
    pub anchors: Vec<[f64; 3]>,
    /// The element ROLE groups a multi-element type inferred (`groups` on the
    /// overlay row): index groups into `anchors`, role order — center's
    /// `[width pair, tab]`. Empty for the pairing types, whose two anchors ARE
    /// the drawing.
    pub groups: Vec<Vec<usize>>,
    /// Distance/Angle: the dimension annotation (reusing the feature-dim shape so
    /// `leaders_buffers` renders it verbatim). `field_key` is the `inputParams`
    /// key the drag edits (`distance` / `angle`). `None` for `Leader` rows and
    /// for dimensional rows whose anchors did not resolve.
    pub annotation: Option<FeatureDimAnnotation>,
    /// The measured value the kernel evaluated (`value` in the overlay row), for
    /// the label suffix. `None` for non-dimensional rows.
    pub value: Option<f64>,
    /// The value's unit (`"mm"` / `"deg"`), empty when `value` is `None`.
    pub unit: String,
    /// Whether the dimensional handle may be DRAGGED: true only for
    /// Distance/Angle whose current param is numeric (or absent — a first-solve
    /// initialized target). A non-numeric expression string disables the drag.
    pub draggable: bool,
    /// The constraint's `inputParams` (from the state list) — the drag commit
    /// mutates a clone of this (so `elements` / flags ride along unchanged).
    pub input_params: Value,
    /// The referenced element names (`inputParams.elements`) — label hover
    /// highlights these through the existing emphasis machinery.
    pub elements: Vec<String>,
}

impl ConstraintOverlay {
    /// The `inputParams` key a drag on this overlay edits (`distance` / `angle`).
    pub fn field_key(&self) -> Option<&'static str> {
        match self.kind {
            ConstraintOverlayKind::Distance => Some("distance"),
            ConstraintOverlayKind::Angle => Some("angle"),
            ConstraintOverlayKind::Leader => None,
        }
    }

    /// The world-space label anchor: a dimensional annotation's chip anchor (the
    /// leader midpoint, or the angle arc's mid-sweep at the screen-constant
    /// radius — camera-dependent via `world_per_pixel`); a role-grouped row's
    /// first-group centroid (center: the width span's midpoint, on the
    /// mid-plane); a leader row's anchor midpoint (or its single anchor).
    /// `None` when nothing resolved.
    pub fn label_anchor(&self, world_per_pixel: f64) -> Option<[f64; 3]> {
        if let Some(annotation) = &self.annotation {
            return Some(match self.kind {
                ConstraintOverlayKind::Angle => {
                    crate::feature_dimensions::angular_chip_anchor(annotation, world_per_pixel)
                }
                _ => annotation.midpoint(),
            });
        }
        if let Some((span, _)) = self.role_leaders() {
            return Some(co_mid(span.0, span.1));
        }
        match self.anchors.len() {
            0 => None,
            1 => Some(self.anchors[0]),
            _ => {
                let a = self.anchors[0];
                let b = self.anchors[1];
                Some([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5])
            }
        }
    }

    /// The role-drawn leaders of a grouped row (center): the WIDTH span
    /// between the first group's two anchors, and the leader from that span's
    /// midpoint to the second group's anchor centroid. `None` unless the row
    /// carries two groups whose indexes all resolve to anchors (a pairing row,
    /// or a stale `groups` against a shorter anchor list, draws the plain
    /// leader instead).
    fn role_leaders(&self) -> Option<(([f64; 3], [f64; 3]), ([f64; 3], [f64; 3]))> {
        let [width, tab] = self.groups.as_slice() else {
            return None;
        };
        let [w0, w1] = width.as_slice() else {
            return None;
        };
        let (a, b) = (*self.anchors.get(*w0)?, *self.anchors.get(*w1)?);
        if tab.is_empty() {
            return None;
        }
        let mut centroid = [0.0f64; 3];
        for &index in tab {
            let p = self.anchors.get(index)?;
            for k in 0..3 {
                centroid[k] += p[k] / tab.len() as f64;
            }
        }
        Some(((a, b), (co_mid(a, b), centroid)))
    }

    /// The label chip text: the type's icon, then `{value}{unit}` for
    /// dimensional rows (`⟺ 5.25 mm`, `∠ 90°`); the bare icon otherwise. The
    /// id no longer rides in the chip — the app's chip hover names it — so a
    /// crowded assembly reads by picture, not by `DIST3`/`COIN9` codes. A row
    /// whose type has no icon keeps the id in the icon's place.
    pub fn label_text(&self) -> String {
        let lead = if self.icon.is_empty() { self.id.as_str() } else { self.icon.as_str() };
        match self.value {
            Some(value) => {
                let n = crate::formatting::compact_decimal(value, 2);
                if self.unit == "deg" {
                    format!("{lead} {n}\u{00b0}")
                } else if self.unit.is_empty() {
                    format!("{lead} {n}")
                } else {
                    format!("{lead} {n} {}", self.unit)
                }
            }
            None => lead.to_string(),
        }
    }
}

// ---------------------------------------------------------------------------
// Status → color
// ---------------------------------------------------------------------------

/// The requirements-doc §5 status → color vocabulary as display-sRGB `[r,g,b]`
/// (0..1, hex/255 like the leader palette — the overlay shader writes ~directly).
///
/// Overlay-shader color (0..1 floats) for a constraint status — a thin view
/// over the ONE canonical map in [`crate::assembly_status`] (the panel's row
/// labels and the tree rollup consume the same table; unified at Wave-3
/// integration so the vocabulary can never drift).
pub fn status_color(status: &str) -> [f32; 3] {
    let [r, g, b] = crate::assembly_status::status_color_rgb(status);
    [r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0]
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

/// Build the overlay records from the kernel payloads: `overlay_rows` is the
/// parsed `assembly_overlay_json` array; `state_constraints` is the parsed
/// `assembly_state_json`'s `constraints` array (may be `Null` — the builder then
/// has no `inputParams`, so dimensional rows fall back to draggable-with-empty
/// params). Rows without resolved anchors yield status-only records (no
/// geometry, no label anchor); a `fixed` constraint's single anchor yields a
/// label anchor but no leader.
pub fn build_constraint_overlays(
    overlay_rows: &Value,
    state_constraints: &Value,
) -> Vec<ConstraintOverlay> {
    let Some(rows) = overlay_rows.as_array() else {
        return Vec::new();
    };
    rows.iter()
        .filter_map(|row| build_row(row, state_constraints))
        .collect()
}

fn build_row(row: &Value, state_constraints: &Value) -> Option<ConstraintOverlay> {
    let id = row.get("id")?.as_str()?.to_string();
    let constraint_type = row
        .get("type")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let icon = brep_kernel::constraint_type(&constraint_type)
        .map(|def| def.icon.to_string())
        .unwrap_or_default();
    let status = row
        .get("status")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let message = row
        .get("message")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();
    let anchors = read_points(row.get("anchors"));
    let directions = read_dirs(row.get("directions"));
    let geoms = read_strings(row.get("geoms"));
    let groups = read_groups(row.get("groups"));
    let value = row.get("value").and_then(Value::as_f64);
    let unit = row
        .get("unit")
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string();

    // The matching state entry's inputParams (expression check + elements).
    let input_params = state_constraints
        .as_array()
        .and_then(|list| {
            list.iter().find(|entry| {
                entry
                    .get("inputParams")
                    .and_then(|p| p.get("id"))
                    .and_then(Value::as_str)
                    == Some(id.as_str())
            })
        })
        .and_then(|entry| entry.get("inputParams"))
        .cloned()
        .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
    let elements = crate::json_support::string_values(input_params.get("elements"))
        .map(str::to_string)
        .collect();

    let kind = match constraint_type.as_str() {
        "distance" => ConstraintOverlayKind::Distance,
        "angle" => ConstraintOverlayKind::Angle,
        _ => ConstraintOverlayKind::Leader,
    };

    // Dragging: only the dimensional kinds, and only while the param is NOT a
    // non-numeric expression string (absent / number / plain numeric string are
    // all draggable — matching the feature-dimension expression rule).
    let draggable = match kind {
        ConstraintOverlayKind::Leader => false,
        ConstraintOverlayKind::Distance => param_allows_drag(&input_params, "distance"),
        ConstraintOverlayKind::Angle => param_allows_drag(&input_params, "angle"),
    };

    let annotation = match kind {
        ConstraintOverlayKind::Distance => {
            build_distance_annotation(&anchors, &directions, &geoms, value)
        }
        ConstraintOverlayKind::Angle => build_angle_annotation(
            &anchors,
            &directions,
            &geoms,
            value,
            row.get("angleAxis")
                .cloned()
                .and_then(|v| serde_json::from_value(v).ok()),
        ),
        ConstraintOverlayKind::Leader => None,
    };

    Some(ConstraintOverlay {
        id,
        constraint_type,
        icon,
        status,
        message,
        kind,
        anchors,
        groups,
        annotation,
        value: match kind {
            ConstraintOverlayKind::Leader => None,
            _ => value,
        },
        unit,
        draggable,
        input_params,
        elements,
    })
}

/// Whether `params[key]` permits a value drag: absent (first-solve initialized),
/// a JSON number, or a PLAIN numeric string — but NOT a non-numeric expression
/// string (`"a + b"`), which stays authoritative and disables the handle.
fn param_allows_drag(params: &Value, key: &str) -> bool {
    match params.get(key) {
        None | Some(Value::Null) => true,
        Some(Value::Number(_)) => true,
        Some(Value::String(text)) => text.trim().parse::<f64>().is_ok(),
        _ => false,
    }
}

/// The distance constraint's LINEAR annotation.
///
/// PLANE-BASED pairings (at least one element is tagged `"plane"` in the row's
/// `geoms` — the BASE face, element 0 preferred, matching the kernel mapper's
/// base choice) draw the TRUE dimension: the other element's anchor `P` is
/// projected onto the base plane along its outward normal `n̂` — the
/// perpendicular foot `F = P − s·n̂` with `s = (P − Q)·n̂` the SIGNED offset —
/// and the arrow runs `F → P`. That segment is normal to the base face by
/// construction and its length `|s|` IS the constrained distance, so the arrow
/// shows the kernel's signed `d = (P_other − P_base)·n̂_base` convention
/// verbatim (a negative `s` points behind the face). The base normal rides in
/// the annotation's (linear-unused) `axis` field so the drag can measure
/// signed offsets along it — including through the face into negatives, and
/// even when `s = 0` collapses the segment to a point.
///
/// Plane-less pairings (lines/points — no side to be on) keep the plain
/// `anchors[0] → anchors[1]` leader with the unsigned measured value and a
/// zero `axis`. `None` unless both anchors resolved.
fn build_distance_annotation(
    anchors: &[[f64; 3]],
    directions: &[Option<[f64; 3]>],
    geoms: &[String],
    value: Option<f64>,
) -> Option<FeatureDimAnnotation> {
    if anchors.len() < 2 {
        return None;
    }
    // The base plane: the first element tagged "plane" with a usable normal.
    let base = (0..2).find(|&i| {
        geoms.get(i).map(String::as_str) == Some("plane")
            && directions
                .get(i)
                .copied()
                .flatten()
                .is_some_and(|n| co_norm(n) > 1e-9)
    });
    if let Some(base) = base {
        let n = directions[base].expect("base index checked above");
        let len = co_norm(n);
        let n = [n[0] / len, n[1] / len, n[2] / len];
        let q = anchors[base];
        let p = anchors[1 - base];
        let s = co_dot(co_sub(p, q), n);
        let foot = [p[0] - n[0] * s, p[1] - n[1] * s, p[2] - n[2] * s];
        // The kernel's measured `value` equals `s` for both plane arms (same
        // formula over the same anchors); prefer it for label consistency.
        let mut annotation =
            FeatureDimAnnotation::linear("distance", foot, p, value.unwrap_or(s), "D");
        annotation.axis = n;
        return Some(annotation);
    }
    let a = anchors[0];
    let b = anchors[1];
    let value = value.unwrap_or_else(|| co_norm(co_sub(b, a)));
    Some(FeatureDimAnnotation::linear("distance", a, b, value, "D"))
}

/// The angle arc uses the kernel's persistent world reference axis. Rebuilding
/// an axis from `d0 × d1` would reverse it at negative/reflex targets, and lose
/// it at 0/180 degrees. Legacy rows without an axis fall back to the cross.
///
/// The arc VERTEX is the angle's hinge — see [`angle_vertex`].
fn build_angle_annotation(
    anchors: &[[f64; 3]],
    directions: &[Option<[f64; 3]>],
    geoms: &[String],
    value: Option<f64>,
    reference_axis: Option<[f64; 3]>,
) -> Option<FeatureDimAnnotation> {
    if anchors.len() < 2 || directions.len() < 2 {
        return None;
    }
    let d0 = directions[0]?;
    let d1 = directions[1]?;
    let axis = reference_axis.unwrap_or_else(|| co_cross(d0, d1));
    let center = angle_vertex(anchors, d0, d1, geoms);
    // Plane directions are normals. Turn the reference into the first
    // face's section through the hinge so the arc arms lie in the measured
    // planes, rather than sticking out perpendicular to them. Rotating both
    // normals by the same quarter turn preserves the signed sweep and axis.
    // Choose the half-line towards the first face; using only that fixed arm
    // avoids reversing the reference as the second component turns.
    let ref_dir = if geoms.get(0).map(String::as_str) == Some("plane")
        && geoms.get(1).map(String::as_str) == Some("plane")
    {
        let tangent = co_cross(axis, d0);
        if co_dot(tangent, co_sub(anchors[0], center)) < 0.0 {
            co_scale(tangent, -1.0)
        } else {
            tangent
        }
    } else {
        d0
    };
    let value = value.unwrap_or(0.0).clamp(-360.0, 360.0);
    Some(FeatureDimAnnotation::angular(
        "angle", center, axis, ref_dir, value, "A",
    ))
}

/// `|n̂₀ × n̂₁|` below which two planes (or a line and a plane) count as
/// parallel and have no hinge. The solver leaves a satisfied 0°/180° target
/// with a residual around 1e-10, so the cutoff sits well above that: a target
/// of exactly 180° must land on the fallback, not on a hinge 1e10 mm away.
const PARALLEL_SIN: f64 = 1e-6;

/// The arc vertex — the point the angle gizmo rotates about.
///
/// For two PLANAR faces the measured directions are the face NORMALS, and the
/// angle between two planes is hinged on the intersection line of their
/// INFINITE planes. The vertex is the point of that hinge nearest the midpoint
/// of the two face anchors. That foot is invariant under any rotation about
/// the hinge (a rotation about an axis preserves every point's component
/// along it), so editing the angle of a hinge-pinned pair rotates the arc
/// without moving it. That holds for every planar face because the kernel's
/// face anchor is a FIXED point of its body (the boundary box centre in the
/// face's own in-plane basis, `assembly_resolve::planar_face`), not a
/// world-axis box that would slide over a triangle or an L as the body turns.
///
/// A plane paired with a direction carrier (edge, axis face, circle) is
/// hinged where the carrier line pierces the plane. Two carriers meet at the
/// closest-approach midpoint of their lines (the vertex of two edges). Rows
/// without geometry tags (legacy sessions) take that carrier rule too.
///
/// Parallel elements have no hinge: the vertex falls back to the anchor
/// midpoint, which lies on the mid-plane of two parallel faces. Approaching
/// parallel, the true hinge recedes to infinity and the vertex follows it
/// until [`PARALLEL_SIN`] switches to the fallback.
fn angle_vertex(anchors: &[[f64; 3]], d0: [f64; 3], d1: [f64; 3], geoms: &[String]) -> [f64; 3] {
    let is_plane = |i: usize| geoms.get(i).map(String::as_str) == Some("plane");
    let (a0, a1) = (anchors[0], anchors[1]);
    match (is_plane(0), is_plane(1)) {
        (true, true) => plane_hinge_foot(a0, d0, a1, d1).unwrap_or_else(|| co_mid(a0, a1)),
        (true, false) => {
            line_plane_pierce(a1, d1, a0, d0).unwrap_or_else(|| co_mid(a0, a1))
        }
        (false, true) => {
            line_plane_pierce(a0, d0, a1, d1).unwrap_or_else(|| co_mid(a0, a1))
        }
        (false, false) => carrier_closest_midpoint(a0, d0, a1, d1),
    }
}

/// The point on the intersection line of the planes `(p0, n0)` and `(p1, n1)`
/// nearest the midpoint `M` of `p0` and `p1` — the constrained least-squares
/// solution `P = M + α·n̂₀ + β·n̂₁` with `[1 c; c 1]·[α β]ᵀ = [n̂₀·(p0−M), n̂₁·(p1−M)]ᵀ`,
/// `c = n̂₀·n̂₁`. `None` when the planes are parallel ([`PARALLEL_SIN`]).
fn plane_hinge_foot(
    p0: [f64; 3],
    n0: [f64; 3],
    p1: [f64; 3],
    n1: [f64; 3],
) -> Option<[f64; 3]> {
    let (l0, l1) = (co_norm(n0), co_norm(n1));
    if l0 < 1e-9 || l1 < 1e-9 {
        return None;
    }
    let n0 = co_scale(n0, 1.0 / l0);
    let n1 = co_scale(n1, 1.0 / l1);
    let c = co_dot(n0, n1);
    let det = 1.0 - c * c; // = |n̂₀ × n̂₁|²
    if det < PARALLEL_SIN * PARALLEL_SIN {
        return None;
    }
    let m = co_mid(p0, p1);
    let r0 = co_dot(n0, co_sub(p0, m));
    let r1 = co_dot(n1, co_sub(p1, m));
    let alpha = (r0 - c * r1) / det;
    let beta = (r1 - c * r0) / det;
    Some(co_add(m, co_add(co_scale(n0, alpha), co_scale(n1, beta))))
}

/// Where the carrier line `l + t·d` pierces the plane `(p, n)`; `None` when
/// the line runs parallel to the plane ([`PARALLEL_SIN`] on the unit vectors).
fn line_plane_pierce(l: [f64; 3], d: [f64; 3], p: [f64; 3], n: [f64; 3]) -> Option<[f64; 3]> {
    let (ld, ln) = (co_norm(d), co_norm(n));
    if ld < 1e-9 || ln < 1e-9 {
        return None;
    }
    let d = co_scale(d, 1.0 / ld);
    let n = co_scale(n, 1.0 / ln);
    let denom = co_dot(d, n);
    if denom.abs() < PARALLEL_SIN {
        return None;
    }
    let t = co_dot(n, co_sub(p, l)) / denom;
    Some(co_add(l, co_scale(d, t)))
}

/// The midpoint of the closest-approach segment between carrier lines
/// `a + t·da` and `b + s·db`; the anchor midpoint when (near) parallel.
fn carrier_closest_midpoint(a: [f64; 3], da: [f64; 3], b: [f64; 3], db: [f64; 3]) -> [f64; 3] {
    let mid = |p: [f64; 3], q: [f64; 3]| {
        [(p[0] + q[0]) * 0.5, (p[1] + q[1]) * 0.5, (p[2] + q[2]) * 0.5]
    };
    let da_n = co_norm(da);
    let db_n = co_norm(db);
    if da_n < 1e-9 || db_n < 1e-9 {
        return mid(a, b);
    }
    let u = [da[0] / da_n, da[1] / da_n, da[2] / da_n];
    let v = [db[0] / db_n, db[1] / db_n, db[2] / db_n];
    let w0 = co_sub(a, b);
    let b_uv = co_dot(u, v);
    let denom = 1.0 - b_uv * b_uv;
    if denom.abs() < 1e-9 {
        return mid(a, b); // parallel carriers — no unique vertex
    }
    let d = co_dot(u, w0);
    let e = co_dot(v, w0);
    let t = (b_uv * e - d) / denom;
    let s = (e - b_uv * d) / denom;
    let p = [a[0] + u[0] * t, a[1] + u[1] * t, a[2] + u[2] * t];
    let q = [b[0] + v[0] * s, b[1] + v[1] * s, b[2] + v[2] * s];
    mid(p, q)
}

// ---------------------------------------------------------------------------
// Buffers
// ---------------------------------------------------------------------------

/// Bake the whole overlay set into flat world-space triangle `(positions,
/// colors)` buffers (the `tris` shape the overlay group consumes): the
/// dimensional annotations through [`leaders_buffers`] (identical arrow/arc
/// styling), the non-dimensional rows as plain silver leaders
/// ([`append_plain_leader`]). Rows without geometry contribute nothing.
pub fn constraint_overlay_buffers(
    overlays: &[ConstraintOverlay],
    world_per_pixel: f64,
) -> (Vec<f32>, Vec<f32>) {
    let annotations: Vec<FeatureDimAnnotation> = overlays
        .iter()
        .filter_map(|overlay| overlay.annotation.clone())
        .collect();
    let (mut positions, mut colors) = leaders_buffers(&annotations, world_per_pixel);
    for overlay in overlays {
        if overlay.kind != ConstraintOverlayKind::Leader {
            continue;
        }
        if let Some((span, tab)) = overlay.role_leaders() {
            // Role-drawn (center): the width span, then mid-plane → tab.
            append_plain_leader(&mut positions, &mut colors, span.0, span.1, world_per_pixel);
            append_plain_leader(&mut positions, &mut colors, tab.0, tab.1, world_per_pixel);
        } else if overlay.anchors.len() >= 2 {
            append_plain_leader(
                &mut positions,
                &mut colors,
                overlay.anchors[0],
                overlay.anchors[1],
                world_per_pixel,
            );
        }
    }
    (positions, colors)
}

// ---------------------------------------------------------------------------
// JSON + vec helpers (self-contained; `co_` prefixed like the fd_ family)
// ---------------------------------------------------------------------------

fn read_points(value: Option<&Value>) -> Vec<[f64; 3]> {
    value
        .and_then(Value::as_array)
        .map(|list| list.iter().filter_map(read_point3).collect())
        .unwrap_or_default()
}

/// Directions align index-wise with anchors; a JSON `null` (a point-like
/// selection has no direction) stays `None`.
fn read_dirs(value: Option<&Value>) -> Vec<Option<[f64; 3]>> {
    value
        .and_then(Value::as_array)
        .map(|list| list.iter().map(read_point3).collect())
        .unwrap_or_default()
}

/// The row's per-element `geoms` tags (aligned index-wise with anchors);
/// empty when absent — a distance row then has no identifiable base plane and
/// falls back to the plain anchor-to-anchor leader.
fn read_strings(value: Option<&Value>) -> Vec<String> {
    value
        .and_then(Value::as_array)
        .map(|list| {
            list.iter()
                .map(|v| v.as_str().unwrap_or("").to_string())
                .collect()
        })
        .unwrap_or_default()
}

/// The row's role `groups` (index groups into `anchors`); empty when absent.
fn read_groups(value: Option<&Value>) -> Vec<Vec<usize>> {
    value
        .and_then(Value::as_array)
        .map(|groups| {
            groups
                .iter()
                .map(|group| {
                    group
                        .as_array()
                        .map(|list| {
                            list.iter()
                                .filter_map(Value::as_u64)
                                .map(|index| index as usize)
                                .collect()
                        })
                        .unwrap_or_default()
                })
                .collect()
        })
        .unwrap_or_default()
}

fn read_point3(value: &Value) -> Option<[f64; 3]> {
    let list = value.as_array()?;
    Some([
        list.first()?.as_f64()?,
        list.get(1)?.as_f64()?,
        list.get(2)?.as_f64()?,
    ])
}

fn co_mid(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5]
}

fn co_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn co_scale(v: [f64; 3], k: f64) -> [f64; 3] {
    [v[0] * k, v[1] * k, v[2] * k]
}

fn co_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn co_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn co_norm(v: [f64; 3]) -> f64 {
    co_dot(v, v).sqrt()
}

fn co_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

