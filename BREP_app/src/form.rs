//! The generic, schema-driven egui form engine — ONE per-field renderer for BOTH
//! the display-settings dialog AND the schema-driven feature dialogs (the user's
//! principle: the structure of a thing drives the UI; you don't rewrite UI code
//! per field, and you don't write two dialog engines).
//!
//! [`field_input`] emits ONLY the input widget(s) for one [`FormField`] — no
//! leading label, no forced row layout — so the CALLER owns the placement. Both
//! side trees drive it DIRECTLY: the history feature tree (`panels::history`) and
//! the settings tree (`panels::settings`) each render a field as a `tree` LEAF
//! whose node label is the field label and whose right-aligned content is this
//! input. The widget per [`FieldKind`]:
//!   * `Color`   → `color_edit_button_srgb`
//!   * `Bool`    → checkbox
//!   * `Enum`    → combo box
//!   * `Number`/`Range` → slider (bounded settings)
//!   * `Scalar`  → an EXPRESSION-CAPABLE single-line field (unbounded feature
//!     numbers): the user types a plain number OR a variable name / inline
//!     equation (`width * 2`) that the kernel evaluates against the history's
//!     `expressions` sheet; while focused, the mouse wheel steps a pure number.
//!   * `BoundedScalar` → the same field, bounding numeric literals only when
//!     an edit commits; the plugin runtime remains the authoritative validator.
//!   * `Text`    → single-line edit (disabled for a read-only `id`)
//!   * `Vec3`    → three drag values (transform position/rotation/scale)
//!   * `Button`  → an action button; a click is surfaced to the caller by the
//!     field key (via the `clicked` out-param) — e.g. `editSketch` opens the
//!     engine-native sketcher.
//!   * `Reference` → a full-width activation button with the current selection
//!     listed BENEATH it, one line per entity, each with an `✕` to remove it.
//!     The `✕` is pinned to the line's RIGHT edge and the name truncates into
//!     what is left (hover for the whole one) — an entity name is as long as the
//!     modelling history made it, and a line laid out name-first would push its
//!     own remove button off the panel.
//!     Pressing the button is surfaced to the caller by the field key (the same
//!     `clicked` out-param a `Button` uses) — entering the picker needs the
//!     engine, and this file deliberately has none. HOVERING a line reports the
//!     entity NAME on it the same way ([`FieldActions::hovered_entity`]), so the
//!     caller can light that entity in the 3D scene.
//!
//! # Width, and who decides it
//!
//! An input fills the width it is given in a TOP-DOWN layout (the label-above
//! form view: [`crate::form_view`]) and stays COMPACT in a RIGHT-TO-LEFT row
//! (the tree/settings idiom, where the label is the tree node and the input sits
//! at the panel edge). `ui.layout().prefer_right_to_left()` is the one
//! discriminator — the same one `Vec3` has always used to order its components —
//! so neither caller passes a layout flag and neither can drift from the other.
//!
//! A field binds to a `path` (a chain of JSON object keys), so it can read/write
//! a NESTED value (`["transform","position"]`, `["boolean","operation"]`), not
//! just a top-level key. On any change it writes back into `current` (the JSON
//! document being edited — settings JSON, or a feature's `inputParams`) and
//! returns `true`; the caller re-applies / re-runs. There is deliberately NO
//! per-field code, so this file is reusable verbatim by a later `brep-ui` crate.

pub(crate) use brep_render::brep_kernel::reference_names;
use crate::color::rgb_to_hex;
use brep_render::style::{parse_css_hex, FieldKind, FormField};
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The red of a reference line's remove `✕` — the destructive red the history
/// tree's delete affordance uses (theme-independent on purpose: it must read as
/// "removes something" in both light and dark).
const REMOVE_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// The width an input should take: whatever the caller has left in a TOP-DOWN
/// form (label above, full-width field), or `compact` in a RIGHT-TO-LEFT tree
/// row, where the input shares the row with its label.
fn input_width(ui: &egui::Ui, compact: f32) -> f32 {
    if ui.layout().prefer_right_to_left() {
        compact
    } else {
        ui.available_width()
    }
}

/// What the user did to a field that only the CALLER can act on — both of these
/// need an engine (entering the reference picker; lighting an entity in the 3D
/// scene), and this file deliberately has none.
///
/// ONE struct rather than two `&mut Option<String>` out-params: they carry
/// different things (a field KEY vs an entity NAME) but the same type, so
/// adjacent out-params would swap silently at a call site.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct FieldActions {
    /// A [`FieldKind::Button`] was pressed, or a `Reference`'s `Select` was — the
    /// field KEY, because acting on either needs the engine.
    pub clicked: Option<String>,
    /// The pointer is over a reference LINE: the entity NAME that line lists, for
    /// the caller to hover-highlight in the 3D scene.
    pub hovered_entity: Option<String>,
}

/// Emit ONLY the input widget(s) for one field — NO leading label, NO forced row
/// layout (the caller supplies both: the form view puts the label ABOVE and gives
/// the input the full width; the settings tree puts the label in the tree node
/// and this input in the row's right-aligned content). Writes any change back
/// into `current` at the field's `path`, returns
/// `(changed, interactive_widget_rect)`.
///
/// `probe`, when present, additionally receives each OPEN enum item's rect (keyed
/// `"<path>#<variant>"`) and a reference field's activation, per-line and
/// per-line-remove rects (`"<path>#activate"`, `"<path>#line<index>"`,
/// `"<path>#x<index>"`) for the headed verifier.
///
/// `actions` is the out-param for what is NOT a value: a `Button` press, a
/// `Reference`'s `Select` press, and the entity a hovered reference line names.
pub fn field_input(
    ui: &mut egui::Ui,
    field: &FormField,
    current: &mut Value,
    mut probe: Option<&mut HashMap<String, egui::Rect>>,
    actions: &mut FieldActions,
) -> (bool, egui::Rect) {
    let path = &field.path;
    match &field.kind {
        FieldKind::Color => {
            let mut rgb = read_rgb(value_at(current, path));
            let r = ui.color_edit_button_srgb(&mut rgb);
            if r.changed() {
                set_at(current, path, Value::String(rgb_to_hex(rgb)));
            }
            (r.changed(), r.rect)
        }
        FieldKind::Bool => {
            let mut b = value_at(current, path).and_then(Value::as_bool).unwrap_or(false);
            let r = ui.checkbox(&mut b, "");
            if r.changed() {
                set_at(current, path, Value::Bool(b));
            }
            (r.changed(), r.rect)
        }
        FieldKind::Enum { variants } => {
            let orig = value_at(current, path)
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string();
            let mut sel = orig.clone();
            let mut combo = egui::ComboBox::from_id_salt(("form-enum", field.key()));
            if !ui.layout().prefer_right_to_left() {
                combo = combo.width(ui.available_width());
            }
            let combo = combo
                .selected_text(&sel)
                .show_ui(ui, |ui| {
                    for v in variants {
                        let item = ui.selectable_value(&mut sel, v.to_string(), v.as_str());
                        if let Some(map) = probe.as_deref_mut() {
                            map.insert(format!("{}#{}", path.join("."), v), item.rect);
                        }
                    }
                });
            let changed = sel != orig;
            if changed {
                set_at(current, path, Value::String(sel));
            }
            (changed, combo.response.rect)
        }
        FieldKind::Number { min, max, step } | FieldKind::Range { min, max, step } => {
            let mut v = value_at(current, path).and_then(Value::as_f64).unwrap_or(*min);
            let r = ui.add(egui::Slider::new(&mut v, *min..=*max).step_by(*step));
            if r.changed() {
                set_at(current, path, serde_json::json!(v));
            }
            (r.changed(), r.rect)
        }
        FieldKind::Scalar { step } | FieldKind::BoundedScalar { step, .. } => {
            // An EXPRESSION-CAPABLE numeric field — see [`scalar_widget`], which
            // owns the whole behaviour so a `Vec3` component gets exactly the same
            // field.
            let (committed, rect) = scalar_widget(
                ui,
                ("scalar-edit", path.join(".")),
                value_at(current, path),
                *step,
                input_width(ui, 72.0),
            );
            let changed = committed.is_some();
            if let Some(mut value) = committed {
                if let (FieldKind::BoundedScalar { min, max, .. }, Some(number)) = (&field.kind, value.as_f64()) {
                    let bounded = number.clamp(*min, *max);
                    if bounded != number {
                        value = serde_json::json!(bounded);
                    }
                }
                set_at(current, path, value);
            }
            (changed, rect)
        }
        FieldKind::Text { read_only } => {
            let mut s = value_at(current, path)
                .and_then(Value::as_str)
                .unwrap_or("")
                .to_string();
            let width = input_width(ui, ui.spacing().text_edit_width);
            if *read_only {
                let r = ui.add_enabled(
                    false,
                    egui::TextEdit::singleline(&mut s).desired_width(width),
                );
                (false, r.rect)
            } else {
                let r = ui.add(egui::TextEdit::singleline(&mut s).desired_width(width));
                if r.changed() {
                    set_at(current, path, Value::String(s));
                }
                (r.changed(), r.rect)
            }
        }
        FieldKind::Vec3 { step } => {
            // THREE expression-capable scalar fields, not three number-only drag
            // values: a transform component is a numeric param like any other, so
            // `position: ["W/2", 0, 0]` must be typeable and must SURVIVE. The
            // number-only widget showed a stored expression as `0` and wrote all
            // three slots back as numbers on any edit, silently destroying the
            // other two — [`vec3_with_slot`] rewrites ONLY the edited index and
            // keeps its siblings' JSON verbatim.
            let stored = value_at(current, path).cloned();
            let mut edited: Option<(usize, Value)> = None;
            let mut rect = egui::Rect::NOTHING;
            let mut component = |ui: &mut egui::Ui, index: usize, width: f32| {
                let (committed, r) = scalar_widget(
                    ui,
                    ("vec3-edit", path.join("."), index),
                    vec3_slot(stored.as_ref(), index),
                    *step,
                    width,
                );
                if let Some(value) = committed {
                    edited = Some((index, value));
                }
                rect = rect.union(r);
            };
            if ui.layout().prefer_right_to_left() {
                // In a RIGHT-TO-LEFT row (the right-aligned tree / settings content)
                // egui lays widgets from the right, which would show the components
                // as z,y,x. Add them in reverse so they still READ x, y, z.
                for index in [2usize, 1, 0] {
                    component(ui, index, 56.0);
                }
            } else {
                // Label-above form: ONE row of three EQUAL-width fields under the
                // single label, reading x, y, z. `ui.horizontal` is this arm's own —
                // the caller's layout is vertical, so without it the three
                // components would stack.
                let gap = ui.spacing().item_spacing.x;
                let each = ((ui.available_width() - 2.0 * gap) / 3.0).max(24.0);
                ui.horizontal(|ui| {
                    for index in 0..3 {
                        component(ui, index, each);
                    }
                });
            }
            match edited {
                Some((index, value)) => {
                    set_at(current, path, vec3_with_slot(stored.as_ref(), index, value));
                    (true, rect)
                }
                None => (false, rect),
            }
        }
        FieldKind::Button { label } => {
            // An action button binds to no value; a click is surfaced via `clicked`
            // (set to the field key) so the host — the history tree, which holds
            // `&mut EngineState` — can act on it (e.g. `editSketch` → sketch mode).
            //
            // FULL WIDTH, like the reference field's `Select` below: an action
            // button is the primary thing to do in its section (Edit Sketch IS the
            // sketch form), and a content-width button floating at the left of a
            // full-width form reads as a minor control rather than the main one.
            let r = ui.add_sized(
                [ui.available_width(), ui.spacing().interact_size.y],
                egui::Button::new(label.as_str()),
            );
            if r.clicked() {
                actions.clicked = Some(field.key().to_string());
            }
            (false, r.rect)
        }
        FieldKind::Reference { filter, multiple } => {
            // R4 — the reference widget: a full-width activation button with the
            // chosen entities listed BENEATH it, one per line, each with an `✕`
            // to remove it. Nothing expands: the selection is what the user needs
            // to see, so it is never hidden behind a `[+]`.
            //
            // Pressing `Select` reports the field key through `clicked` (exactly
            // as a `Button` does) — entering the modal picker is the ENGINE's job
            // and the caller owns which flavour of picker to enter. Removing a
            // line, by contrast, is a pure edit of `current`, so it happens here
            // and reports `changed` like any other field.
            let names = reference_names(value_at(current, path));
            let pkey = path.join(".");
            let mut removed: Option<usize> = None;
            let mut activate_rect = egui::Rect::NOTHING;
            ui.vertical(|ui| {
                let width = ui.available_width();
                let hint = format!(
                    "▣ Select {}{}",
                    filter.join("/"),
                    if *multiple { " …" } else { "" }
                );
                let select = crate::icon_text::icon_button(ui, &hint);
                let button = ui.add_sized(
                    [width, ui.spacing().interact_size.y],
                    select,
                );
                activate_rect = button.rect;
                if button.clicked() {
                    actions.clicked = Some(field.key().to_string());
                }
                if let Some(map) = probe.as_deref_mut() {
                    map.insert(format!("{pkey}#activate"), button.rect);
                }
                if names.is_empty() {
                    ui.label(egui::RichText::new("(none)").weak());
                    return;
                }
                for (i, name) in names.iter().enumerate() {
                    // The ✕ is allocated FIRST, from the RIGHT edge, and the name
                    // takes what is left over — the same order the form view's
                    // title row uses, for the same reason. An entity name is as
                    // long as the modelling history made it (a fillet on a blend
                    // edge reaches 76 characters), so name-then-button lets the
                    // name push the button clean off the panel: measured 212 pt
                    // past the edge of a 320 pt panel, where an ancestor clip
                    // then eats the click as well as the pixels.
                    ui.horizontal(|ui| {
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                let remove = crate::icon_text::icon_button_colored(
                                    ui,
                                    "✕",
                                    Some(REMOVE_RED),
                                )
                                .stroke(egui::Stroke::new(1.0, REMOVE_RED))
                                .small();
                                let x = ui.add(remove);
                                if let Some(map) = probe.as_deref_mut() {
                                    map.insert(format!("{pkey}#x{i}"), x.rect);
                                }
                                if x.clicked() {
                                    removed = Some(i);
                                }
                                // Back to reading order for the name, inside
                                // whatever width the button left behind. It
                                // TRUNCATES there rather than wrapping: these
                                // names differ in their tails (`[0]` vs `[1]`,
                                // `NZ` vs `NY`), and a `Label` that elides shows
                                // the WHOLE text on hover by itself
                                // (`show_tooltip_when_elided`, on by default),
                                // so the tail is a hover away and adding an
                                // `on_hover_text` here would only stack a second
                                // tooltip on egui's.
                                ui.with_layout(
                                    egui::Layout::left_to_right(egui::Align::Center),
                                    |ui| {
                                        let line = ui.add(
                                            egui::Label::new(format!("• {name}"))
                                                .truncate(),
                                        );
                                        // Hovering the line reports the entity it
                                        // names, so the caller can light it in the
                                        // 3D scene exactly as mousing over it there
                                        // would — a reference list is a list of
                                        // things in the model, and reading which
                                        // `…|BOUNDARY[1]` is which off the name
                                        // alone is what the highlight replaces.
                                        if line.hovered() {
                                            actions.hovered_entity = Some(name.clone());
                                        }
                                        if let Some(map) = probe.as_deref_mut() {
                                            map.insert(
                                                format!("{pkey}#line{i}"),
                                                line.rect,
                                            );
                                        }
                                    },
                                );
                            },
                        );
                    });
                }
            });
            if let Some(i) = removed {
                let mut kept = names;
                kept.remove(i);
                let value = if *multiple {
                    Value::Array(kept.into_iter().map(Value::String).collect())
                } else {
                    Value::String(kept.first().cloned().unwrap_or_default())
                };
                set_at(current, path, value);
                return (true, activate_rect);
            }
            (false, activate_rect)
        }
    }
}

// --- nested JSON read / write ------------------------------------------------

/// Resolve `path` (object keys) to a value inside `root`, if present.
pub(crate) fn value_at<'a>(root: &'a Value, path: &[String]) -> Option<&'a Value> {
    let mut cur = root;
    for seg in path {
        cur = cur.get(seg.as_str())?;
    }
    Some(cur)
}

/// Write `new_val` into `root` at `path`, auto-vivifying intermediate objects
/// (so a feature whose `inputParams` omits `transform` still accepts an edit).
pub(crate) fn set_at(root: &mut Value, path: &[String], new_val: Value) {
    if path.is_empty() {
        *root = new_val;
        return;
    }
    if !root.is_object() {
        *root = Value::Object(serde_json::Map::new());
    }
    let mut cur = root;
    for seg in &path[..path.len() - 1] {
        let obj = cur.as_object_mut().expect("object by construction");
        cur = obj
            .entry(seg.clone())
            .or_insert_with(|| Value::Object(serde_json::Map::new()));
        if !cur.is_object() {
            *cur = Value::Object(serde_json::Map::new());
        }
    }
    cur.as_object_mut()
        .expect("object by construction")
        .insert(path[path.len() - 1].clone(), new_val);
}

// --- Scalar (expression-capable feature number) helpers ----------------------

/// The ONE expression-capable numeric field: a [`FieldKind::Scalar`], and each
/// component of a [`FieldKind::Vec3`].
///
/// It shows a stored NUMBER as text and a stored EXPRESSION (`width * 2`)
/// VERBATIM — never clobbering a string param to `0` the way a number-only
/// `DragValue` does — and lets the user type either. A transient edit buffer
/// lives in egui memory (keyed by `id_source`) and is committed on focus-loss
/// (Enter / click-away), mirroring `panels::expressions`, so a half-typed
/// expression (`width *`) never re-runs the history mid-keystroke. While the
/// field is focused, the mouse wheel STEPS a pure number by `step` (recovering
/// the old drag-value stepping); a scroll notch is a complete, valid edit, so it
/// commits immediately.
///
/// Returns the value to STORE when the edit committed (the caller owns where it
/// goes — a whole param for `Scalar`, one slot of the array for `Vec3`) and the
/// widget rect.
fn scalar_widget(
    ui: &mut egui::Ui,
    id_source: impl std::hash::Hash + std::fmt::Debug,
    stored: Option<&Value>,
    step: f64,
    width: f32,
) -> (Option<Value>, egui::Rect) {
    let buf_id = ui.make_persistent_id(id_source);
    let stored_text = scalar_display(stored);
    // Seed from the live buffer while editing; otherwise from the stored value
    // (an undo / gizmo edit may have changed it out from under us).
    let mut buf = ui
        .data_mut(|d| d.get_temp::<String>(buf_id))
        .unwrap_or_else(|| stored_text.clone());

    let r = ui.add(
        egui::TextEdit::singleline(&mut buf)
            .id(buf_id)
            .desired_width(width),
    );

    let mut committed = None;
    if r.gained_focus() {
        // Start each edit from the current stored value.
        buf = stored_text.clone();
    }
    // Scroll-to-step a PURE number by `step` (wheel up = +step) while the field
    // is focused AND the pointer is over it ("scroll over the field to step
    // it"). A non-numeric expression is un-steppable (`scroll_step_scalar` →
    // `None`) and left untouched — typing still works. Step ONLY when the cursor
    // is actually over this field, so a focused field doesn't swallow panel
    // scrolling while the user scrolls elsewhere to navigate (that would
    // silently edit the value). Two independent "pointer is over me" signals for
    // robustness — a real wheel carries the cursor position on native/desktop;
    // only synthetic (headless-test) wheels lack it.
    let pointer_over_field = r.hovered()
        || ui
            .input(|i| i.pointer.latest_pos())
            .is_some_and(|pos| r.rect.contains(pos));
    if r.has_focus() && pointer_over_field {
        let notches = wheel_notches(ui);
        // EAT the wheel so the enclosing side-panel `ScrollArea` can't ALSO
        // scroll the panel — the field owns the scroll while the cursor is over
        // it. Do this EVERY frame the cursor is here, not only on the notch
        // frame: egui SMOOTHS a wheel notch across several frames, and only the
        // first carries a `MouseWheel` event, so zeroing just that frame let the
        // smoothed TAIL leak into the panel (visible on the desktop build; the
        // web scroll wasn't smoothed so it looked fine). On non-scroll frames
        // these are harmless no-ops. The ScrollArea reads the smoothed delta in
        // its epilogue (after this content), so zeroing here blocks it; we also
        // drop the wheel events.
        ui.input_mut(|i| {
            i.smooth_scroll_delta = egui::Vec2::ZERO;
            i.events
                .retain(|e| !matches!(e, egui::Event::MouseWheel { .. }));
        });
        if notches != 0.0 {
            if let Some(stepped) = scroll_step_scalar(&buf, notches, step) {
                buf = stepped;
                committed = Some(scalar_store(&buf));
            }
        }
    }
    if r.lost_focus() {
        // Commit on Enter / click-away. Skip an EMPTY buffer (a `String("")` is a
        // guaranteed kernel eval error) and a NO-OP (compare by the DISPLAYED
        // text so a whole-float `20.0` vs a typed `20` — same display — doesn't
        // re-run the history for nothing).
        let trimmed = buf.trim();
        if !trimmed.is_empty() && trimmed != stored_text {
            committed = Some(scalar_store(&buf));
        }
        ui.data_mut(|d| d.remove::<String>(buf_id));
    } else if r.has_focus() {
        // Keep the in-progress buffer (incl. any scroll step) across frames.
        ui.data_mut(|d| d.insert_temp(buf_id, buf.clone()));
    } else {
        // Unfocused and not committing: drop any transient buffer so the next
        // edit reseeds from the (possibly externally changed) value.
        ui.data_mut(|d| d.remove::<String>(buf_id));
    }
    (committed, r.rect)
}

/// One slot of a stored vec3 (`[x, y, z]`), or `None` when the param is absent /
/// not an array / short. A slot is read VERBATIM — a number stays a number and an
/// expression string stays that string, so `scalar_display` shows it as authored.
fn vec3_slot(stored: Option<&Value>, index: usize) -> Option<&Value> {
    stored?.as_array()?.get(index).filter(|v| !v.is_null())
}

/// The vec3 to STORE after one component was edited: a full three-element array
/// with `index` replaced and the OTHER TWO slots carried over verbatim. Carrying
/// them is the point — writing all three back from three coerced `f64`s is what
/// silently turned a sibling `"W/2"` into `0`. A missing sibling materializes as
/// `0`, which is what the number-only widget already displayed for it.
fn vec3_with_slot(stored: Option<&Value>, index: usize, value: Value) -> Value {
    let mut out: Vec<Value> = (0..3)
        .map(|i| vec3_slot(stored, i).cloned().unwrap_or(Value::from(0.0)))
        .collect();
    out[index] = value;
    Value::Array(out)
}

/// The text to SHOW for a [`FieldKind::Scalar`] field: a stored NUMBER as its
/// shortest decimal string (`20`, not `20.0`; `20.5` as `20.5` — no precision
/// loss), a stored EXPRESSION string VERBATIM (`width * 2`, never clobbered to
/// `0`), and a missing / other value as empty.
fn scalar_display(value: Option<&Value>) -> String {
    match value {
        Some(Value::String(s)) => s.clone(),
        // Rust's `f64` Display is the shortest round-tripping form and omits a
        // trailing `.0`, so an integer OR whole-float number both show as `20`.
        Some(Value::Number(n)) => n.as_f64().map(|f| f.to_string()).unwrap_or_default(),
        _ => String::new(),
    }
}

/// Turn committed field text into the stored param [`Value`]: a PURE numeric
/// literal becomes a JSON `Number` (clean serialization + the kernel's fast
/// `as_f64` path), anything else becomes a `Value::String` the kernel evaluates
/// against the history `expressions` sheet (`ctx.number` at
/// `feature_pipeline/mod.rs`: a `String` param is `env.eval`'d). Numeric-ness is
/// decided by a strict JSON number parse, so `10.` / `1e` and other half-typed
/// forms stay strings rather than round-tripping through a reformat.
fn scalar_store(text: &str) -> Value {
    let trimmed = text.trim();
    match serde_json::from_str::<Value>(trimmed) {
        Ok(v @ Value::Number(_)) => v,
        _ => Value::String(trimmed.to_string()),
    }
}

/// Apply `notches` mouse-wheel steps of size `step` to a Scalar field's text.
/// Only a PURE numeric literal steps (wheel up = `+step`); a non-numeric
/// expression (`width * 2`) is un-steppable and yields `None` (a no-op — the
/// caller leaves the text alone so typing keeps working).
fn scroll_step_scalar(text: &str, notches: f64, step: f64) -> Option<String> {
    let base: f64 = text.trim().parse().ok()?;
    Some(format_scalar_number(base + notches * step, step))
}

/// Format a stepped number to the STEP's decimal precision so repeated steps do
/// not accumulate binary-float noise (`0.1` steps stay `10.1`, `10.2`, … not
/// `10.299999`). Trailing zeros / dot are trimmed (`10.50 → 10.5`, `9.0 → 9`).
fn format_scalar_number(v: f64, step: f64) -> String {
    let decimals = step_decimals(step);
    let mut s = format!("{:.*}", decimals, v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
        if s.ends_with('.') {
            s.pop();
        }
    }
    s
}

/// Decimal places implied by `step` (`0.5 → 1`, `0.1 → 1`, `0.01 → 2`, `1 → 0`),
/// capped so a pathological step can't ask for absurd precision.
fn step_decimals(step: f64) -> usize {
    let step = step.abs();
    if step == 0.0 || !step.is_finite() {
        return 3;
    }
    let mut d = 0usize;
    let mut s = step;
    while (s - s.round()).abs() > 1e-9 && d < 6 {
        s *= 10.0;
        d += 1;
    }
    d
}

/// Whole mouse-wheel notches scrolled this frame (wheel up = `+`), rounded to an
/// integer notch count. Read from the raw `MouseWheel` events (not
/// `smooth_scroll_delta`) so one physical notch is one discrete step. Browser
/// backends report a conventional wheel detent as roughly 100 pixels (Chromium)
/// or 3 lines (Firefox), whereas native winit reports it as 40 points or 1 line;
/// keep those platform scales separate so the web field doesn't jump by 2–3
/// schema steps for the same wheel movement. The caller gates this on the field
/// being focused.
fn wheel_notches(ui: &egui::Ui) -> f64 {
    let raw: f32 = ui.input(|i| {
        i.events
            .iter()
            .filter_map(|event| match event {
                egui::Event::MouseWheel { unit, delta, .. } => Some(wheel_delta_to_notches(
                    *unit,
                    delta.y,
                    cfg!(target_arch = "wasm32"),
                )),
                _ => None,
            })
            .sum()
    });
    (raw as f64).round()
}

fn wheel_delta_to_notches(unit: egui::MouseWheelUnit, delta_y: f32, web: bool) -> f32 {
    let (lines_per_notch, points_per_notch) = if web { (3.0, 100.0) } else { (1.0, 40.0) };
    match unit {
        egui::MouseWheelUnit::Line => delta_y / lines_per_notch,
        egui::MouseWheelUnit::Point => delta_y / points_per_notch,
        egui::MouseWheelUnit::Page => delta_y * 20.0,
    }
}

/// Read a `#rrggbb` value as `[u8; 3]` for `color_edit_button_srgb`.
fn read_rgb(value: Option<&Value>) -> [u8; 3] {
    let hex = value.and_then(Value::as_str).unwrap_or("#000000");
    let rgb = parse_css_hex(hex).unwrap_or([0.0, 0.0, 0.0]);
    [
        (rgb[0] * 255.0).round() as u8,
        (rgb[1] * 255.0).round() as u8,
        (rgb[2] * 255.0).round() as u8,
    ]
}

