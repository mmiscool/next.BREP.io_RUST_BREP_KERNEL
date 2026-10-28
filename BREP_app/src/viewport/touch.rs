//! Touch-only camera navigation. A single contact keeps the existing tap/drag
//! routing (including sketch tools and gizmos). Two contacts capture navigation
//! until ALL fingers lift; a leftover finger must never become a click or orbit.
use egui::{Event, Pos2, TouchDeviceId, TouchId, TouchPhase};

pub(crate) const TARGET: f32 = 44.0;
pub(super) const CURSOR_OFFSET: f64 = 36.0;


pub(crate) fn enabled(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp::<bool>(egui::Id::new("brep-touch-input"))).unwrap_or(false)
}

/// Below this window width (points) a layout is NARROW even with a mouse.
pub(crate) const NARROW_WIDTH: f32 = 650.0;

/// Touch input or a window narrower than [`NARROW_WIDTH`]: the layouts in
/// which the side panels would cover the view, so a dialog's reference pick
/// takes the view over (`panels::pick_bar`).
pub(crate) fn narrow(ctx: &egui::Context) -> bool {
    enabled(ctx) || ctx.content_rect().width() < NARROW_WIDTH
}

/// Whether the shell drew NO chrome this frame (a focused reference pick,
/// `panels::pick_bar`): the page's own full-screen button, which the toolbar
/// places, must not float over the view then.
pub(crate) fn set_chrome_hidden(ctx: &egui::Context, hidden: bool) {
    ctx.data_mut(|d| d.insert_temp(egui::Id::new("brep-chrome-hidden"), hidden));
}

pub(crate) fn chrome_hidden(ctx: &egui::Context) -> bool {
    ctx.data(|d| d.get_temp::<bool>(egui::Id::new("brep-chrome-hidden"))).unwrap_or(false)
}

/// The page's full-screen button shows only beside the toolbar that places
/// it: never under a modal, never while the chrome is hidden.
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
pub(crate) fn fullscreen_button_visible(ctx: &egui::Context) -> bool {
    !chrome_hidden(ctx) && ctx.memory(|m| m.top_modal_layer().is_none())
}

pub(crate) fn compact(ctx: &egui::Context) -> bool {
    #[cfg(target_arch = "wasm32")]
    {
        enabled(ctx)
    }
    #[cfg(not(target_arch = "wasm32"))]
    { let _ = ctx; false }
}

pub(crate) fn target_size(ctx: &egui::Context, desktop: egui::Vec2) -> egui::Vec2 {
    if enabled(ctx) { desktop.max(egui::Vec2::splat(TARGET)) } else { desktop }
}

pub(crate) fn text_edit_margin(ui: &egui::Ui) -> egui::Margin {
    // egui 0.35 currently ignores TextEdit.min_size.y. Padding must provide
    // the actual contact height; the desktop value is TextEdit's own default.
    let pad = if enabled(ui.ctx()) { ((TARGET - ui.text_style_height(&egui::TextStyle::Body)) * 0.5).ceil().max(2.0) as i8 } else { 2 };
    egui::Margin::symmetric(4, pad)
}

/// The browser's mobile policy selects the sizing layer. Desktop browser width
/// and touch hardware never opt in; web/index.html owns identity and URL override.
pub(crate) fn prepare_browser_input(ui: &mut egui::Ui) {
    #[cfg(target_arch = "wasm32")]
    let active = web_sys::window().and_then(|w| w.document())
        .and_then(|d| d.document_element())
        .is_some_and(|e| e.has_attribute("data-brep-touch"));
    #[cfg(not(target_arch = "wasm32"))]
    let active = false;
    apply_sizing(ui, active);
}

/// Shared browser seam for every focused egui text editor. egui already masks
/// password values before emitting widget output.
#[cfg(target_arch = "wasm32")]
pub(crate) fn publish_editor(ctx: &egui::Context) {
    use wasm_bindgen::{JsCast, JsValue};
    if !enabled(ctx) { return; }
    if let Some(rect) = ctx.data(|d| d.get_temp::<egui::Rect>(egui::Id::new("brep-mobile-fullscreen-slot"))) {
        if let Some(window) = web_sys::window() {
            if let Ok(callback) = js_sys::Reflect::get(window.as_ref(), &JsValue::from_str("__brepMobileFullscreenRect")) {
                if let Ok(callback) = callback.dyn_into::<js_sys::Function>() {
                    let zoom = ctx.zoom_factor();
                    let style = ctx.global_style();
                    let payload = serde_json::json!({ "x": rect.left() * zoom, "y": rect.top() * zoom,
                        "bg": style.visuals.widgets.inactive.bg_fill.to_array(), "fg": style.visuals.text_color().to_array(),
                        "visible": fullscreen_button_visible(ctx) }).to_string();
                    let _ = callback.call1(&JsValue::NULL, &JsValue::from_str(&payload));
                }
            }
        }
    }
    let focused = ctx.memory(|m| m.focused());
    let selection = focused.and_then(|id| egui::TextEdit::load_state(ctx, id))
        .and_then(|state| state.cursor.char_range()).map(|range| range.as_sorted_char_range());
    let payload = ctx.output(|output| {
        if output.ime.is_none() || focused.is_none() { return "null".to_string(); }
        let text = output.events.iter().rev().find_map(|event| event.widget_info().current_text_value.clone());
        let reopen = output.events.iter().any(|event| {
            event.widget_info().current_text_value.is_some() && matches!(event,
                egui::output::OutputEvent::Clicked(_) | egui::output::OutputEvent::DoubleClicked(_) |
                egui::output::OutputEvent::FocusGained(_))
        });
        serde_json::json!({ "id": format!("{:?}", focused.unwrap()), "text": text, "reopen": reopen,
            "selection": selection.map(|range| [range.start, range.end]) }).to_string()
    });
    let cache = egui::Id::new("brep-mobile-editor-output");
    if ctx.data(|d| d.get_temp::<String>(cache)).as_deref() == Some(&payload) { return; }
    if let Some(window) = web_sys::window() {
        if let Ok(callback) = js_sys::Reflect::get(window.as_ref(), &JsValue::from_str("__brepMobileEditor")) {
            if let Ok(callback) = callback.dyn_into::<js_sys::Function>() {
                if callback.call1(&JsValue::NULL, &JsValue::from_str(&payload)).is_ok() {
                    ctx.data_mut(|d| d.insert_temp(cache, payload));
                }
            }
        }
    }
}

fn apply_sizing(ui: &mut egui::Ui, active: bool) {
    let ctx = ui.ctx().clone();
    ctx.data_mut(|d| d.insert_temp(egui::Id::new("brep-touch-input"), active));
    let key = egui::Id::new(("brep-touch-desktop-spacing", format!("{:?}", ctx.theme())));
    let saved = ctx.data(|d| d.get_temp::<egui::style::Spacing>(key));
    if active {
        let mut spacing = saved.unwrap_or_else(|| {
            let spacing = ctx.style_of(ctx.theme()).spacing.clone();
            ctx.data_mut(|d| d.insert_temp(key, spacing.clone()));
            spacing
        });
        spacing.interact_size = spacing.interact_size.max(egui::Vec2::splat(TARGET));
        spacing.button_padding.x = spacing.button_padding.x.max(14.0);
        spacing.item_spacing.y = spacing.item_spacing.y.max(6.0);
        spacing.slider_width = spacing.slider_width.max(120.0);
        spacing.scroll.bar_width = spacing.scroll.bar_width.max(16.0);
        ctx.style_mut_of(ctx.theme(), |style| style.spacing = spacing.clone());
        *ui.spacing_mut() = spacing;
    } else if let Some(spacing) = saved {
        ctx.style_mut_of(ctx.theme(), |style| style.spacing = spacing.clone());
        *ui.spacing_mut() = spacing;
        ctx.data_mut(|d| d.remove::<egui::style::Spacing>(key));
    }
}

/// Broaden the app's handle acquisition without changing engine/render defaults.
/// The caller retains this offset for the whole drag so acquisition never jumps.
pub(super) fn handle_target(state: &brep_render::engine_state::EngineState, x: f64, y: f64) -> Option<(f64, f64)> {
    let mut points: Vec<(f64, f64)> = state.transform_handle_points().into_iter().map(|(_, p)| p).collect();
    points.extend(state.transform_gizmo_anchor());
    points.extend(state.dimension_handle_points().into_iter().map(|(_, p)| p));
    points.extend(state.dimension_origin_points().into_iter().map(|(_, p)| p));
    if let Ok(serde_json::Value::Object(handles)) = serde_json::from_str(&state.constraint_handle_points_json()) {
        points.extend(handles.values().filter_map(|p| Some((p[0].as_f64()?, p[1].as_f64()?))));
    }
    points.into_iter().filter_map(|p| {
        let d = (p.0 - x).hypot(p.1 - y);
        (d <= f64::from(TARGET) * 0.5).then_some((d, p))
    }).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, p)| p)
}

/// The visible cursor leads a drawing finger, then snaps to the nearest visible
/// solved point. All tolerances are screen pixels, independent of model scale.
pub(super) fn sketch_target(state: &brep_render::engine_state::EngineState, x: f64, y: f64, drawing: bool) -> (f64, f64) {
    let y = if drawing { (y - CURSOR_OFFSET).clamp(0.0, state.camera.height.max(0.0)) } else { y };
    let Some(session) = state.sketch_edit_session() else { return (x, y) };
    session.doc.points.iter().filter_map(|p| {
        let world = session.plane.to_world(p.x, p.y);
        if !state.camera.label_anchor_visible(world) { return None; }
        let (sx, sy, _) = state.camera.project(world);
        let distance = (sx - x).hypot(sy - y);
        (distance <= f64::from(TARGET) * 0.5).then_some((distance, (sx, sy)))
    }).min_by(|a, b| a.0.total_cmp(&b.0)).map(|(_, p)| p).unwrap_or((x, y))
}

#[derive(Clone, Copy)]
struct Contact {
    device: TouchDeviceId,
    id: TouchId,
    pos: Pos2,
    admitted: bool,
}

#[derive(Default)]
pub(super) struct TouchNavigation {
    contacts: Vec<Contact>,
    captured: bool,
    last_frame: Option<(u64, bool)>,
}

#[derive(Default)]
pub(super) struct TouchFrame {
    pub suppress_pointer: bool,
    pub started: bool,
    pub motion: Option<(Pos2, Pos2, f64)>,
}

impl TouchNavigation {
    /// egui can lay out the same frame more than once. Consume motion once,
    /// while retaining pointer suppression on every pass, including release.
    pub fn update_for_frame(
        &mut self,
        frame: u64,
        events: &[Event],
        focused: bool,
        any_touches: bool,
        admit: impl Fn(Pos2) -> bool,
    ) -> TouchFrame {
        if let Some((previous, suppressed)) = self.last_frame {
            if previous == frame {
                return TouchFrame {
                    suppress_pointer: suppressed,
                    ..Default::default()
                };
            }
            if previous + 1 != frame {
                // A hidden/docked viewport may have missed touch releases.
                *self = Self::default();
            }
        }
        let result = self.update(events, focused, any_touches, admit);
        self.last_frame = Some((frame, result.suppress_pointer));
        result
    }

    fn pair(&self) -> Option<(Pos2, f32)> {
        let [a, b] = self.contacts.as_slice() else {
            return None;
        };
        if !a.admitted || !b.admitted || a.device != b.device {
            return None;
        }
        Some((a.pos + (b.pos - a.pos) * 0.5, a.pos.distance(b.pos)))
    }

    pub fn update(
        &mut self,
        events: &[Event],
        focused: bool,
        any_touches: bool,
        admit: impl Fn(Pos2) -> bool,
    ) -> TouchFrame {
        let mut result = TouchFrame {
            suppress_pointer: self.captured,
            ..Default::default()
        };
        if !focused {
            result.started = self.contacts.iter().any(|c| c.admitted);
            result.suppress_pointer |= result.started;
            *self = Self::default();
            return result;
        }
        let before = self.pair();
        let mut membership_changed = false;
        for event in events {
            let Event::Touch {
                device_id,
                id,
                phase,
                pos,
                ..
            } = *event
            else {
                continue;
            };
            let index = self
                .contacts
                .iter()
                .position(|c| c.device == device_id && c.id == id);
            match phase {
                TouchPhase::Start => {
                    membership_changed = true;
                    if index.is_none() {
                        self.contacts.push(Contact {
                            device: device_id,
                            id,
                            pos,
                            admitted: pos.is_finite() && admit(pos),
                        });
                    }
                    // Latch even when both fingers start and end in one frame.
                    if self.pair().is_some() && !self.captured {
                        self.captured = true;
                        result.started = true;
                        result.suppress_pointer = true;
                    }
                }
                TouchPhase::Move => {
                    if let Some(index) = index {
                        if pos.is_finite() {
                            self.contacts[index].pos = pos;
                        }
                    }
                }
                TouchPhase::End | TouchPhase::Cancel => {
                    membership_changed = true;
                    if let Some(index) = index {
                        if phase == TouchPhase::Cancel && self.contacts[index].admitted {
                            // Some backends send no synthetic mouse-up on cancel.
                            // Close even a single-finger camera/tool capture.
                            result.started = true;
                            result.suppress_pointer = true;
                            self.captured = true;
                        }
                        self.contacts.remove(index);
                    }
                }
            }
        }
        if self.captured && !membership_changed {
            if let (Some((from, old_distance)), Some((to, distance))) = (before, self.pair()) {
                // Nearly coincident fingers have no reliable scale. Keep panning,
                // then establish a new baseline before accepting pinch ratios.
                let scale = if old_distance >= 4.0 && distance >= 4.0 {
                    f64::from(distance) / f64::from(old_distance)
                } else {
                    1.0
                };
                if from != to || scale != 1.0 {
                    result.motion = Some((from, to, scale));
                }
            }
        }
        // Also recover if the viewport was hidden when the release arrived.
        if !any_touches || self.contacts.is_empty() {
            *self = Self::default();
        }
        result
    }
}

