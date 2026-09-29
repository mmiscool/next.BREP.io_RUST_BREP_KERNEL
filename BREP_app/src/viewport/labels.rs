use super::*;

/// The egui text color for CONSTRAINT annotations (the dimension VALUE labels), read
/// from the engine's single source of truth — the display-settings sketch palette
/// ([`brep_render::style::RenderSettings::sketch_colors`]) — so the label text follows
/// the same (editable) color as the engine-drawn leaders + geometric glyphs.
fn constraint_text_color(settings: &brep_render::style::RenderSettings) -> egui::Color32 {
    rgb_color32(settings.sketch_colors().constraint)
}

/// The egui text color for a dimension the solver named in a CONFLICT — the same
/// palette's conflict red, so the value text matches its own (engine-drawn) leader
/// and the status bar's conflict dot.
fn conflict_text_color(settings: &brep_render::style::RenderSettings) -> egui::Color32 {
    rgb_color32(settings.sketch_colors().conflict)
}

fn rgb_color32(c: u32) -> egui::Color32 {
    egui::Color32::from_rgb((c >> 16) as u8, (c >> 8) as u8, c as u8)
}

/// The ONE font every model-overlaid label draws and measures with: the base
/// `Monospace` text style, its size multiplied by the user's "Label scale" setting
/// ([`brep_render::style::RenderSettings::label_scale`], clamped engine-side to
/// `[0.25, 3.0]`, 1.0 = unchanged).
///
/// EVERY label site routes through here — the read-only chips AND the inline
/// `TextEdit`s AND the [`text_edit_width`] measurement — which is what keeps the
/// four sizes that must move together in lockstep:
///   1. the GLYPHS (this `FontId`),
///   2. the measured EDIT-BOX WIDTH (`text_edit_width`, fed this same `FontId`),
///   3. the chip's PADDING ([`chip_margin`]),
///   4. the WIDGET METRICS egui would otherwise apply unscaled
///      ([`scale_label_spacing`] + the `TextEdit`'s own `margin`) — WITHOUT which
///      a shrinking label is tiny glyphs floating in a fixed-height box.
/// The chip's FRAME then follows for free: egui sizes a `Frame` to its content, so
/// the background box AND the widget's click/hit rect are the widget rect grown by
/// [`chip_margin`] at every scale — the visible chip and the hit target can never
/// come apart. A read-only chip and its editable counterpart therefore TRACK each
/// other (see `chip_height_follows_the_label_scale`), so a label does not visibly
/// jump when it is clicked to edit. Measuring with an unscaled font while drawing
/// with a scaled one would drift the background box and the click target off the
/// glyphs — never resolve `TextStyle::Monospace` directly at a label site.
fn label_font(style: &egui::Style, scale: f32) -> egui::FontId {
    let mut font = egui::TextStyle::Monospace.resolve(style);
    font.size *= scale;
    font
}

/// Scale the `ui.spacing()` metrics that size the WIDGETS INSIDE a label chip, so
/// the chip's HEIGHT follows its glyphs instead of egui's fixed chrome minimums.
/// Call it on the chip `Frame`'s inner `ui` (it mutates only that `Ui`'s cloned
/// style, never the app-wide one), alongside [`label_font`] + [`chip_margin`].
///
/// THE BUG THIS FIXES: `egui::Button` floors its height at
/// `ui.spacing().interact_size.y` (18pt by default — egui's "a button is at least
/// this tall so you can hit it" rule) and pads with `ui.spacing().button_padding`.
/// Neither knows about the label scale, so a 0.5x label drew 6pt glyphs inside a
/// chip that stayed exactly as tall as a 1.0x one — the reported "the label itself
/// stays the same height". Scaling these by the SAME factor as the font keeps the
/// relationship the 1.0 default has (the floor lands at `18 * scale`, a hair above
/// the `~17 * scale` content) at every scale, so the chip is proportional all the
/// way down instead of bottoming out. `item_spacing` is scaled for the same reason
/// — it costs nothing on today's one-widget chips and is the right value the day a
/// chip grows a second widget.
///
/// NOT a `set_height`/`set_min_size`: the chip must stay CONTENT-sized, because
/// its background box and its click/hit rect are both derived from what the widget
/// allocates. Forcing a size would move the box off the glyphs.
///
/// At `scale == 1.0` every one of these is an exact identity (`v * 1.0 == v`), so
/// the default everyone sees is untouched — pinned by
/// `scale_1_0_is_pixel_identical_to_the_unscaled_chip`.
fn scale_label_spacing(ui: &mut egui::Ui, scale: f32) {
    let spacing = ui.spacing_mut();
    spacing.interact_size *= scale;
    spacing.button_padding *= scale;
    spacing.item_spacing *= scale;
}

/// A label chip's `inner_margin`, scaled with the text so a 3x label keeps its
/// padding proportionate instead of hugging the glyphs. `x`/`y` are the base
/// (scale 1.0) paddings in egui points.
///
/// Also used for the sketch-dimension `TextEdit`'s OWN text margin, whose egui
/// default is exactly `Margin::symmetric(4, 2)` — so `chip_margin(4.0, 2.0, 1.0)`
/// reproduces it byte for byte while every other scale finally moves it. (That
/// margin is a `TextEdit` builder field, NOT a `ui.spacing()` one, which is why
/// [`scale_label_spacing`] cannot reach it.)
fn chip_margin(x: f32, y: f32, scale: f32) -> egui::Margin {
    egui::Margin::symmetric((x * scale).round() as i8, (y * scale).round() as i8)
}

/// The width (egui points) to give a dimension value-edit `TextEdit` so its current
/// `text` never wraps: the measured no-wrap width of the string in `font` (same
/// `layout_no_wrap` path the action rail uses), plus a little padding for the caret
/// + inner margin, floored so an emptied box (select-all + delete) stays a usable
/// size. Both the padding and the floor scale with the font, so the box tracks the
/// glyphs at every "Label scale". `font` MUST be the same [`label_font`] the
/// `TextEdit` draws with. Sizing only — value / color / placement are unchanged.
fn text_edit_width(ui: &egui::Ui, text: &str, font: &egui::FontId) -> f32 {
    let measured = ui.ctx().fonts_mut(|f| {
        f.layout_no_wrap(text.to_owned(), font.clone(), egui::Color32::PLACEHOLDER)
            .size()
            .x
    });
    // The base 12pt caret/margin allowance and 24pt floor, scaled by how much this
    // font is bigger than the unscaled base — so an emptied box at 3x is still a
    // usable size for 3x glyphs.
    let base = egui::TextStyle::Monospace.resolve(ui.style()).size;
    let k = if base > 0.0 { font.size / base } else { 1.0 };
    (measured + 12.0 * k).max(24.0 * k)
}

/// The ONE inline value editor a dimension label opens, with every SCALE-dependent
/// input already applied: the box is sized to its text ([`text_edit_width`]) in the
/// [`label_font`], and its own text margin — the `TextEdit` builder field egui
/// defaults to `Margin::symmetric(4, 2)` and which [`scale_label_spacing`] cannot
/// reach — is scaled through [`chip_margin`], so the box's HEIGHT tracks the glyphs
/// instead of adding a fixed 4pt at every scale. Callers add only NON-sizing
/// decoration (`text_color`, `frame`). A caller that passes `Frame::NONE` makes
/// egui ignore the margin entirely, which is equally scale-following.
fn label_text_edit<'t>(
    ui: &egui::Ui,
    buf: &'t mut String,
    font: &egui::FontId,
    scale: f32,
) -> egui::TextEdit<'t> {
    // Size the box to its text so the value never wraps — measured in the SAME
    // scaled font it draws with.
    let width = text_edit_width(ui, buf.as_str(), font);
    egui::TextEdit::singleline(buf)
        .desired_width(width)
        .margin(chip_margin(4.0, 2.0, scale))
        .font(egui::FontSelection::FontId(font.clone()))
}

/// Select the ENTIRE contents of a just-opened inline `TextEdit`. Call on the
/// first frame the editor opens (right after `request_focus`) so a
/// double-click-to-edit starts with everything selected — typing replaces the
/// whole value at once. `char_count` is the field text's char length.
fn select_all_text_edit(ctx: &egui::Context, id: egui::Id, char_count: usize) {
    if let Some(mut state) = egui::TextEdit::load_state(ctx, id) {
        state.cursor.set_char_range(Some(egui::text::CCursorRange::two(
            egui::text::CCursor::new(0),
            egui::text::CCursor::new(char_count),
        )));
        egui::TextEdit::store_state(ctx, id, state);
    }
}

impl Viewport {
    /// Draw the editable dimension labels (S5) over the viewport while in sketch
    /// mode. For each dimensional constraint the engine reports a label anchor in
    /// world space + its display text; we project it to screen and draw a small
    /// clickable value. DOUBLE-clicking opens an inline single-line `TextEdit`
    /// (seeded from the number, or the `valueExpr` when set) with all text
    /// selected; Enter applies via
    /// [`EngineState::sketch_set_dimension_value`], Esc cancels. Dragging a label
    /// repositions it via [`EngineState::sketch_dimension_drag_to`]. The labels ride
    /// in `Order::Middle` areas so they float over the 3D and take pointer priority
    /// over the viewport's own select/place handling.
    pub(super) fn draw_dimension_labels(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        if !state.sketch_mode() {
            self.editing_dim = None;
            return;
        }

        // The dimension labels (id/text/world/value/valueExpr/mode) from the engine.
        let labels: Vec<serde_json::Value> =
            serde_json::from_str(&state.sketch_dimension_labels_json()).unwrap_or_default();

        // Drop an open editor whose constraint no longer has a label (e.g. deleted).
        if let Some((id, _)) = self.editing_dim.as_ref() {
            let key = id.to_string();
            if !labels.iter().any(|l| l["id"].to_string() == key) {
                self.editing_dim = None;
            }
        }
        if labels.is_empty() {
            return;
        }

        // Project every label anchor world→screen in one shot.
        let worlds: Vec<[f64; 3]> = labels
            .iter()
            .map(|l| {
                let w = &l["world"];
                [
                    w[0].as_f64().unwrap_or(0.0),
                    w[1].as_f64().unwrap_or(0.0),
                    w[2].as_f64().unwrap_or(0.0),
                ]
            })
            .collect();
        let screens: Vec<[f64; 4]> = serde_json::to_string(&worlds)
            .ok()
            .and_then(|s| state.world_to_screen_json(&s).ok())
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        if screens.len() != labels.len() {
            return;
        }

        // Editing state pulled out so the draw closures can mutate the buffer without
        // borrowing `self` twice; written back after the loop.
        let fresh = self.dim_edit_fresh;
        self.dim_edit_fresh = false;
        let mut editing = self.editing_dim.take();
        let editing_key = editing.as_ref().map(|(id, _)| id.to_string());

        // The label text color from the live settings (read BEFORE the closures, which
        // must never touch `state`) — follows the editable sketch constraint color.
        let dim_text_color = constraint_text_color(&state.settings);
        // …and the red a CONFLICTING dimension's value is painted in instead (the
        // leaders are colored engine-side; this is the app-drawn text half of the
        // same indication).
        let dim_conflict_color = conflict_text_color(&state.settings);
        // The user's "Label scale" (read BEFORE the closures, which must never touch
        // `state`) — drives the glyphs, the edit-box measurement and the chip padding
        // through the ONE `label_font` / `chip_margin` pair.
        let label_scale = state.settings.label_scale;

        // Deferred engine mutations (never call `state` inside the area closures).
        let mut start_edit: Option<serde_json::Value> = None;
        let mut apply: bool = false;
        let mut cancel: bool = false;
        let mut drag_to: Option<(serde_json::Value, f64, f64)> = None;
        // A label drag ended this frame → reset the sketch undo's per-drag guard (S6a)
        // so the whole drag was one undo step and the next drag starts a fresh one.
        let mut drag_ended = false;

        for (i, label) in labels.iter().enumerate() {
            let scr = screens[i];
            if scr[3] < 0.5 {
                // The engine's ONE label policy (ViewCamera::label_anchor_visible):
                // hidden only when perspective-behind-the-eye or the 3D anchor
                // projects OUTSIDE the viewport (otherwise the egui Area would
                // clamp the chip to the edge). Ortho depth / near / far NEVER
                // cull — never add such a test here.
                continue;
            }
            let cid = label["id"].clone();
            let cid_key = cid.to_string();
            let pos = egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32);
            let is_editing = editing_key.as_deref() == Some(cid_key.as_str());

            let area_id = egui::Id::new(("brep-dim-label", cid_key.clone()));
            egui::Area::new(area_id)
                .order(egui::Order::Middle)
                .fixed_pos(pos)
                .pivot(egui::Align2::CENTER_CENTER)
                .show(ctx, |ui| {
                    let font = label_font(ui.style(), label_scale);
                    egui::Frame::popup(ui.style())
                        .inner_margin(chip_margin(4.0, 2.0, label_scale))
                        .show(ui, |ui| {
                            // The widget metrics egui would otherwise apply
                            // unscaled — without this the chip's HEIGHT is
                            // floored at the 1.0 button height at every scale.
                            scale_label_spacing(ui, label_scale);
                            if is_editing {
                                let buf = &mut editing.as_mut().expect("editing buffer").1;
                                let editor = label_text_edit(ui, buf, &font, label_scale);
                                let resp = ui.add(editor);
                                if fresh {
                                    resp.request_focus();
                                    select_all_text_edit(ui.ctx(), resp.id, buf.chars().count());
                                }
                                let enter =
                                    ui.input(|i| i.key_pressed(egui::Key::Enter));
                                if resp.lost_focus() {
                                    if enter {
                                        apply = true;
                                    } else if !fresh {
                                        cancel = true;
                                    }
                                }
                                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                                    cancel = true;
                                }
                            } else {
                                let text = label["text"].as_str().unwrap_or("").to_string();
                                let color = if label["conflicting"].as_bool() == Some(true) {
                                    dim_conflict_color
                                } else {
                                    dim_text_color
                                };
                                let resp = ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(text)
                                            .font(font.clone())
                                            .color(color),
                                    )
                                    // Never wrap/clip the chip — extend to fit its text.
                                    .wrap_mode(egui::TextWrapMode::Extend)
                                    .sense(egui::Sense::click_and_drag()),
                                );
                                if resp.double_clicked() {
                                    start_edit = Some(cid.clone());
                                }
                                if resp.dragged() {
                                    if let Some(p) = resp.interact_pointer_pos() {
                                        drag_to = Some((
                                            cid.clone(),
                                            (p.x - rect.min.x) as f64,
                                            (p.y - rect.min.y) as f64,
                                        ));
                                    }
                                }
                                if resp.drag_stopped() {
                                    drag_ended = true;
                                }
                            }
                        });
                });
        }

        // --- apply deferred actions (state is free to borrow again here) ---------
        if apply {
            if let Some((id, text)) = editing.take() {
                state.sketch_set_dimension_value(&id, &text);
            }
            self.editing_dim = None;
        } else if cancel {
            self.editing_dim = None;
        } else {
            // Keep the (possibly edited) buffer for the next frame.
            self.editing_dim = editing;
        }

        if let Some((id, lx, ly)) = drag_to {
            state.sketch_dimension_drag_to(&id, lx, ly);
        }
        if drag_ended {
            state.sketch_dimension_drag_end();
        }

        if let Some(id) = start_edit {
            // Seed the field from the display value (diameter shows the diameter),
            // preferring the expression when one is set.
            let seed: serde_json::Value =
                serde_json::from_str(&state.sketch_dimension_value_json(&id)).unwrap_or_default();
            let text = seed
                .get("valueExpr")
                .and_then(|v| v.as_str())
                .map(str::to_string)
                .or_else(|| {
                    seed.get("value")
                        .and_then(|v| v.as_f64())
                        .map(|n| format!("{n}"))
                })
                .unwrap_or_default();
            self.editing_dim = Some((id, text));
            self.dim_edit_fresh = true;
        }

        // Keep animating while a field is open (focus / caret).
        if self.editing_dim.is_some() {
            ctx.request_repaint();
        }
    }

    /// Draw the editable FEATURE-dimension labels (FD-1) over the viewport while
    /// the ◎ is in DIMENSION mode. For each linear param dim the engine reports a
    /// leader midpoint (world) + its value; we project it to screen and draw a
    /// small `"{label} {value}"` chip. DOUBLE-clicking opens an inline `TextEdit`
    /// seeded from the value with all text selected; Enter applies via
    /// [`EngineState::feature_dimension_set_value`]
    /// (numeric literal OR live expression), Esc cancels. Dragging the chip drives
    /// [`EngineState::feature_dimension_drag`] (the dim resizes the param live).
    /// The chips ride `Order::Middle`, taking pointer priority over the viewport's
    /// select/orbit — so a drag that starts on a handle never orbits the camera.
    pub(super) fn draw_feature_dimension_labels(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        if state.gizmo_mode() != "dimension" {
            self.editing_feature_dim = None;
            return;
        }
        let feature = state.dimension_armed_feature();
        if feature.is_empty() {
            self.editing_feature_dim = None;
            return;
        }

        let annotations: Vec<serde_json::Value> =
            serde_json::from_str(&state.feature_dimension_annotations_json(&feature))
                .unwrap_or_default();

        // Drop an open editor whose field no longer has an annotation.
        if let Some((_, field, _)) = self.editing_feature_dim.as_ref() {
            let key = field.clone();
            if !annotations
                .iter()
                .any(|a| a["fieldKey"].as_str() == Some(key.as_str()))
            {
                self.editing_feature_dim = None;
            }
        }
        if annotations.is_empty() {
            return;
        }

        // Project every leader midpoint world→screen in one shot.
        let worlds: Vec<[f64; 3]> = annotations
            .iter()
            .map(|a| {
                let m = &a["mid"];
                [
                    m[0].as_f64().unwrap_or(0.0),
                    m[1].as_f64().unwrap_or(0.0),
                    m[2].as_f64().unwrap_or(0.0),
                ]
            })
            .collect();
        let screens: Vec<[f64; 4]> = serde_json::to_string(&worlds)
            .ok()
            .and_then(|s| state.world_to_screen_json(&s).ok())
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        if screens.len() != annotations.len() {
            return;
        }

        // The user's "Label scale" (read BEFORE the closures, which must never touch
        // `state`).
        let label_scale = state.settings.label_scale;

        let fresh = self.feature_dim_edit_fresh;
        self.feature_dim_edit_fresh = false;
        let mut editing = self.editing_feature_dim.take();
        let editing_key = editing.as_ref().map(|(_, field, _)| field.clone());

        // Deferred engine mutations (never touch `state` inside the area closures).
        let mut start_edit: Option<(String, String)> = None; // (field, seed)
        let mut apply = false;
        let mut cancel = false;
        let mut drag_to: Option<(String, f64, f64)> = None;

        for (i, annotation) in annotations.iter().enumerate() {
            let scr = screens[i];
            if scr[3] < 0.5 {
                // The engine's ONE label policy (ViewCamera::label_anchor_visible):
                // hidden only when perspective-behind-the-eye or the 3D anchor
                // projects OUTSIDE the viewport (otherwise the egui Area would
                // clamp the chip to the edge). Ortho depth / near / far NEVER
                // cull — never add such a test here.
                continue;
            }
            let Some(field) = annotation["fieldKey"].as_str() else {
                continue;
            };
            let field = field.to_string();
            let value = annotation["value"].as_f64().unwrap_or(0.0);
            let prefix = annotation["label"].as_str().unwrap_or("").to_string();
            // An angular dim (torus `arc`, revolve `angle`) shows its value in
            // DEGREES with a trailing `°`; the edit seed stays the bare number.
            let is_angular = annotation["kind"].as_str() == Some("angular");
            let pos = egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32);
            let is_editing = editing_key.as_deref() == Some(field.as_str());

            let area_id = egui::Id::new(("brep-feature-dim", feature.clone(), field.clone()));
            egui::Area::new(area_id)
                .order(egui::Order::Middle)
                .fixed_pos(pos)
                .pivot(egui::Align2::CENTER_CENTER)
                .show(ctx, |ui| {
                    // Dark rounded chip with a thin orange border + orange
                    // monospace text (matches the reference dimension image).
                    let orange = egui::Color32::from_rgb(245, 166, 35);
                    let dark = egui::Color32::from_rgb(20, 20, 20);
                    let font = label_font(ui.style(), label_scale);
                    egui::Frame::new()
                        .fill(dark)
                        .stroke(egui::Stroke::new(1.0, orange))
                        .corner_radius(egui::CornerRadius::same(6))
                        .inner_margin(chip_margin(6.0, 3.0, label_scale))
                        .show(ui, |ui| {
                            // The widget metrics egui would otherwise apply
                            // unscaled — without this the chip's HEIGHT is
                            // floored at the 1.0 button height at every scale.
                            // (This editor passes `Frame::NONE`, which makes egui
                            // ignore the `TextEdit`'s own text margin entirely, so
                            // there is nothing to scale there.)
                            scale_label_spacing(ui, label_scale);
                            if is_editing {
                                let buf = &mut editing.as_mut().expect("editing buffer").2;
                                let editor = label_text_edit(ui, buf, &font, label_scale)
                                    .text_color(orange)
                                    .frame(egui::Frame::NONE);
                                let resp = ui.add(editor);
                                if fresh {
                                    resp.request_focus();
                                    select_all_text_edit(ui.ctx(), resp.id, buf.chars().count());
                                }
                                let enter = ui.input(|i| i.key_pressed(egui::Key::Enter));
                                if resp.lost_focus() {
                                    if enter {
                                        apply = true;
                                    } else if !fresh {
                                        cancel = true;
                                    }
                                }
                                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                                    cancel = true;
                                }
                            } else {
                                let text = if is_angular {
                                    format!("{prefix} {}\u{00b0}", fmt_dim_value(value))
                                } else {
                                    format!("{prefix} {}", fmt_dim_value(value))
                                };
                                let resp = ui.add(
                                    egui::Button::new(
                                        egui::RichText::new(text)
                                            .font(font.clone())
                                            .color(orange),
                                    )
                                    .frame(false)
                                    // Never wrap/clip the chip — extend to fit its text.
                                    .wrap_mode(egui::TextWrapMode::Extend)
                                    .sense(egui::Sense::click_and_drag()),
                                );
                                if resp.double_clicked() {
                                    start_edit = Some((field.clone(), fmt_dim_value(value)));
                                }
                                if resp.dragged() {
                                    if let Some(p) = resp.interact_pointer_pos() {
                                        drag_to = Some((
                                            field.clone(),
                                            (p.x - rect.min.x) as f64,
                                            (p.y - rect.min.y) as f64,
                                        ));
                                    }
                                }
                            }
                        });
                });
        }

        // --- apply deferred actions (state free to borrow again) -----------------
        if apply {
            if let Some((feat, field, text)) = editing.take() {
                state.feature_dimension_set_value(&feat, &field, &text);
            }
            self.editing_feature_dim = None;
        } else if cancel {
            self.editing_feature_dim = None;
        } else {
            self.editing_feature_dim = editing;
        }

        if let Some((field, lx, ly)) = drag_to {
            state.feature_dimension_drag(&feature, &field, lx, ly);
        }

        if let Some((field, seed)) = start_edit {
            self.editing_feature_dim = Some((feature.clone(), field, seed));
            self.feature_dim_edit_fresh = true;
        }

        if self.editing_feature_dim.is_some() {
            ctx.request_repaint();
        }
    }

    /// Draw the ASSEMBLY-CONSTRAINT labels over the viewport: for every cached
    /// constraint overlay the engine reports a world label anchor (leader
    /// midpoint / arc mid-sweep) plus its status-colored text — we project each
    /// to screen and draw a small chip. HOVERING a chip highlights the
    /// constraint's referenced geometry (the engine's element hover — deduped,
    /// with the one-frame viewport yield flag); CLICKING it expands that
    /// constraint's row in the Assembly Constraints panel (the kernel `open`
    /// flag via [`EngineState::constraint_label_clicked`]). The chips are
    /// hidden whenever the engine's overlay cache is empty (Show Constraint
    /// Graphics off, sketch mode, no assembly). During a handle drag the chip
    /// text tracks the live preview value (the cache carries it).
    pub(super) fn draw_constraint_labels(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        let labels: Vec<serde_json::Value> =
            serde_json::from_str(&state.constraint_labels_json()).unwrap_or_default();
        if labels.is_empty() {
            if self.constraint_label_hovered.take().is_some() {
                state.constraint_hover_end();
            }
            return;
        }

        // Project every label anchor world→screen in one shot.
        let worlds: Vec<[f64; 3]> = labels
            .iter()
            .map(|l| {
                let w = &l["world"];
                [
                    w[0].as_f64().unwrap_or(0.0),
                    w[1].as_f64().unwrap_or(0.0),
                    w[2].as_f64().unwrap_or(0.0),
                ]
            })
            .collect();
        let screens: Vec<[f64; 4]> = serde_json::to_string(&worlds)
            .ok()
            .and_then(|s| state.world_to_screen_json(&s).ok())
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        if screens.len() != labels.len() {
            return;
        }

        // The user's "Label scale" (read BEFORE the closures, which must never touch
        // `state`).
        let label_scale = state.settings.label_scale;

        // Deferred engine mutations (never touch `state` inside the area closures).
        let mut hovered: Option<String> = None;
        let mut clicked: Option<String> = None;

        for (i, label) in labels.iter().enumerate() {
            let scr = screens[i];
            if scr[3] < 0.5 {
                // The engine's ONE label policy (ViewCamera::label_anchor_visible):
                // hidden only when perspective-behind-the-eye or the 3D anchor
                // projects OUTSIDE the viewport (otherwise the egui Area would
                // clamp the chip to the edge). Ortho depth / near / far NEVER
                // cull — never add such a test here.
                continue;
            }
            let Some(id) = label["id"].as_str() else {
                continue;
            };
            let text = label["text"].as_str().unwrap_or(id).to_string();
            let message = label["message"].as_str().unwrap_or("").to_string();
            let status = label["status"].as_str().unwrap_or("").to_string();
            // The status → color vocabulary comes from the engine (ONE map).
            let rgb = &label["color"];
            let color = egui::Color32::from_rgb(
                (rgb[0].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[1].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[2].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
            );
            let pos = egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32);
            let area_id = egui::Id::new(("brep-constraint-label", id));
            egui::Area::new(area_id)
                .order(egui::Order::Middle)
                .fixed_pos(pos)
                .pivot(egui::Align2::CENTER_CENTER)
                .show(ctx, |ui| {
                    // Dark rounded chip with a thin status-colored border +
                    // status-colored monospace text (the feature-dim chip look,
                    // colored by the requirements-§5 status vocabulary). The
                    // label-click-SELECTED constraint gets a thicker border.
                    let selected = label["selected"].as_bool().unwrap_or(false);
                    let dark = egui::Color32::from_rgb(20, 20, 20);
                    let font = label_font(ui.style(), label_scale);
                    egui::Frame::new()
                        .fill(dark)
                        .stroke(egui::Stroke::new(if selected { 2.5 } else { 1.0 }, color))
                        .corner_radius(egui::CornerRadius::same(6))
                        .inner_margin(chip_margin(6.0, 3.0, label_scale))
                        .show(ui, |ui| {
                            // The widget metrics egui would otherwise apply
                            // unscaled — without this the chip's HEIGHT is
                            // floored at the 1.0 button height at every scale.
                            scale_label_spacing(ui, label_scale);
                            // The chip leads with the constraint TYPE's icon
                            // (`text` is `{icon} {value}{unit}`). There is no
                            // icon font, so the glyph is drawn as catalogued
                            // artwork, tinted the status colour and sized to
                            // the chip's (label-scaled) font — never as a
                            // character, which would render a box.
                            let (icon, rest) = crate::icon_text::split_caption(&text);
                            let rich = egui::RichText::new(rest).font(font.clone()).color(color);
                            let button = match icon {
                                Some(icon) => {
                                    egui_extras::install_image_loaders(ui.ctx());
                                    let height = ui.fonts_mut(|f| f.row_height(&font));
                                    let mut art = crate::icon_text::image(icon, height);
                                    if icon.mono {
                                        art = art.tint(color);
                                    }
                                    if rest.is_empty() {
                                        egui::Button::new(art)
                                    } else {
                                        egui::Button::new((art, rich))
                                    }
                                }
                                None => egui::Button::new(rich),
                            };
                            let resp = ui.add(
                                button
                                    .frame(false)
                                    // Never wrap/clip the chip — extend to fit its text.
                                    .wrap_mode(egui::TextWrapMode::Extend)
                                    .sense(egui::Sense::click()),
                            );
                            // The id left the chip when the icon took its place,
                            // so the hover is where two same-type constraints
                            // are told apart.
                            let resp = if message.is_empty() {
                                resp.on_hover_text(format!("{id} \u{2014} {status}"))
                            } else {
                                resp.on_hover_text(format!("{id} \u{2014} {status}: {message}"))
                            };
                            if resp.hovered() {
                                hovered = Some(id.to_string());
                            }
                            if resp.clicked() {
                                clicked = Some(id.to_string());
                            }
                        });
                });
        }

        // --- apply deferred actions (state free to borrow again) -----------------
        match hovered {
            Some(id) => {
                // Highlight the referenced geometry; deduped engine-side, and the
                // one-frame yield flag keeps next frame's scene hover off it.
                state.constraint_hover(&id);
                self.constraint_label_hovered = Some(id);
            }
            None => {
                if self.constraint_label_hovered.take().is_some() {
                    state.constraint_hover_end();
                }
            }
        }
        if let Some(id) = clicked {
            state.constraint_label_clicked(&id);
        }
    }

    /// Draw the ACTIVE PMI VIEW's annotation labels: one chip per annotation
    /// at its label anchor, leading with the type's glyph, in the PMI colour
    /// (red for an unresolved annotation, grey for a disabled one, a thicker
    /// border for the open one), sized by the view's text size. Hover
    /// highlights the referenced geometry, a click SELECTS the annotation and a
    /// double click opens its form (as its tree row does), a DRAG moves the
    /// label on its depth plane (one coalesced undo step per drag, never a
    /// history run), and a RIGHT-click selects it and opens its menu — the
    /// entries its PMI tree row's `⋯` offers ([`Self::show_pmi_label_menu`]).
    pub(super) fn draw_pmi_labels(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        let labels: Vec<serde_json::Value> =
            serde_json::from_str(&state.pmi_labels_json()).unwrap_or_default();
        self.pmi_label_hits.clear();
        self.pmi_menu_hits.clear();
        if labels.is_empty() {
            self.pmi_label_menu = None;
            if self.pmi_label_hovered.take().is_some() {
                state.pmi_hover_end();
            }
            if self.pmi_label_dragging.take().is_some() {
                state.pmi_label_drag_end();
            }
            return;
        }
        let worlds: Vec<[f64; 3]> = labels
            .iter()
            .map(|l| {
                let w = &l["world"];
                [
                    w[0].as_f64().unwrap_or(0.0),
                    w[1].as_f64().unwrap_or(0.0),
                    w[2].as_f64().unwrap_or(0.0),
                ]
            })
            .collect();
        let screens: Vec<[f64; 4]> = serde_json::to_string(&worlds)
            .ok()
            .and_then(|s| state.world_to_screen_json(&s).ok())
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        if screens.len() != labels.len() {
            return;
        }
        let label_scale = state.settings.label_scale;

        let mut hovered: Option<String> = None;
        let mut clicked: Option<String> = None;
        let mut double_clicked: Option<String> = None;
        let mut drag_to: Option<(String, f64, f64)> = None;
        let mut drag_ended = false;
        let mut menu_at: Option<(String, egui::Pos2)> = None;
        let mut chip_rects: Vec<(String, egui::Rect)> = Vec::new();

        for (i, label) in labels.iter().enumerate() {
            let scr = screens[i];
            if scr[3] < 0.5 {
                // The engine's ONE label policy (ViewCamera::label_anchor_visible).
                continue;
            }
            let Some(id) = label["id"].as_str() else {
                continue;
            };
            let text = label["text"].as_str().unwrap_or(id).to_string();
            let message = label["message"].as_str().unwrap_or("").to_string();
            let status = label["status"].as_str().unwrap_or("").to_string();
            let open = label["open"].as_bool().unwrap_or(false);
            // Selected in the PMI tree: the same heavier outline an open one
            // carries, the constraint chip's selected accent.
            let selected = label["selected"].as_bool().unwrap_or(false);
            let text_size = label["textSizePt"].as_f64().unwrap_or(12.0);
            let rgb = &label["color"];
            let color = egui::Color32::from_rgb(
                (rgb[0].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[1].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[2].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
            );
            // The view's text size scales the chip on top of the user's label scale.
            let scale = label_scale * (text_size / 12.0) as f32;
            let pos = egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32);
            let area_id = egui::Id::new(("brep-pmi-label", id));
            egui::Area::new(area_id)
                .order(egui::Order::Middle)
                .fixed_pos(pos)
                .pivot(egui::Align2::CENTER_CENTER)
                .show(ctx, |ui| {
                    let dark = egui::Color32::from_rgb(20, 20, 20);
                    let font = label_font(ui.style(), scale);
                    egui::Frame::new()
                        .fill(dark)
                        .stroke(egui::Stroke::new(if open || selected { 2.5 } else { 1.0 }, color))
                        .corner_radius(egui::CornerRadius::same(4))
                        .inner_margin(chip_margin(6.0, 3.0, scale))
                        .show(ui, |ui| {
                            scale_label_spacing(ui, scale);
                            // A dimension chip is its value text; a datum /
                            // frame / error chip leads with the type's glyph,
                            // drawn as catalogued artwork tinted the chip colour.
                            let (icon, rest) = crate::icon_text::split_caption(&text);
                            let rich = egui::RichText::new(rest).font(font.clone()).color(color);
                            let button = match icon {
                                Some(icon) => {
                                    egui_extras::install_image_loaders(ui.ctx());
                                    let height = ui.fonts_mut(|f| f.row_height(&font));
                                    let mut art = crate::icon_text::image(icon, height);
                                    if icon.mono {
                                        art = art.tint(color);
                                    }
                                    if rest.is_empty() {
                                        egui::Button::new(art)
                                    } else {
                                        egui::Button::new((art, rich))
                                    }
                                }
                                None => egui::Button::new(rich),
                            };
                            let resp = ui.add(
                                button
                                    .frame(false)
                                    .wrap_mode(egui::TextWrapMode::Extend)
                                    .sense(egui::Sense::click_and_drag()),
                            );
                            let resp = if message.is_empty() {
                                resp.on_hover_text(format!("{id} \u{2014} {status} (drag to move, click to select, double-click to edit, right-click for its menu)"))
                            } else {
                                resp.on_hover_text(format!("{id} \u{2014} {status}: {message}"))
                            };
                            if resp.hovered() {
                                hovered = Some(id.to_string());
                            }
                            // A double click EDITS and a click SELECTS, as the
                            // annotation's tree row does. The double click is
                            // read first: egui reports its second press as a
                            // click too.
                            if crate::panels::tree::is_double_click(&resp) {
                                double_clicked = Some(id.to_string());
                            } else if resp.clicked() {
                                clicked = Some(id.to_string());
                            }
                            if resp.dragged() {
                                if let Some(p) = resp.interact_pointer_pos() {
                                    drag_to = Some((
                                        id.to_string(),
                                        (p.x - rect.min.x) as f64,
                                        (p.y - rect.min.y) as f64,
                                    ));
                                }
                            }
                            if resp.drag_stopped() {
                                drag_ended = true;
                            }
                            if resp.secondary_clicked() {
                                let at = resp.interact_pointer_pos().unwrap_or(resp.rect.center());
                                menu_at = Some((id.to_string(), at));
                            }
                            chip_rects.push((id.to_string(), resp.rect));
                        });
                });
        }
        self.pmi_label_hits = chip_rects;

        // --- apply deferred actions (state free to borrow again) -----------------
        if let Some((id, lx, ly)) = drag_to {
            self.pmi_label_dragging = Some(id.clone());
            state.pmi_label_drag_to(&id, lx, ly);
        }
        if drag_ended {
            self.pmi_label_dragging = None;
            state.pmi_label_drag_end();
        }
        match hovered {
            Some(id) => {
                state.pmi_hover(&id);
                self.pmi_label_hovered = Some(id);
            }
            None => {
                if self.pmi_label_hovered.take().is_some() {
                    state.pmi_hover_end();
                }
            }
        }
        if let Some(id) = double_clicked {
            state.pmi_label_double_clicked(&id);
        } else if let Some(id) = clicked {
            state.pmi_label_clicked(&id);
        }
        // A right-click SELECTS too, so the chip the menu acts on carries the
        // accent while it is up.
        if let Some((id, _)) = &menu_at {
            state.pmi_label_clicked(id);
        }
        self.show_pmi_label_menu(ctx, state, menu_at);
    }

    /// A PMI label chip's RIGHT-CLICK menu: the annotation's PMI tree row menu
    /// — the same entries its `⋯` offers, run through the same dispatch
    /// (`panels::pmi::run_row_action`), so the chip and the tree cannot offer
    /// different things. Its entries are published as
    /// `pmimenu/menuitem:<annotation id>:<action id>`.
    ///
    /// `request` is a right-click this frame; it is applied AFTER the open
    /// menu has drawn, so the click that asked for a new menu is not also the
    /// click that closes it. A chip no longer drawn (deleted, disabled, off
    /// screen) takes its menu with it.
    fn show_pmi_label_menu(
        &mut self,
        ctx: &egui::Context,
        state: &mut EngineState,
        request: Option<(String, egui::Pos2)>,
    ) {
        let mut chosen: Option<(String, String)> = None;
        if let Some((id, pos)) = self.pmi_label_menu.clone() {
            if self.pmi_label_hits.iter().any(|(chip, _)| *chip == id) {
                let hits = &mut self.pmi_menu_hits;
                // The popup belongs to the chip's own layer — each chip is an
                // `Area` keyed like this one.
                let layer = egui::LayerId::new(egui::Order::Middle, egui::Id::new(("brep-pmi-label", id.as_str())));
                let (action, still_open) = crate::column_tree::action_menu(
                    ctx,
                    layer,
                    egui::Id::new("brep-pmi-label-menu"),
                    pos,
                    &crate::panels::pmi::annotation_actions(),
                    |entry, rect| {
                        hits.insert(format!("menuitem:{id}:{}", entry.id), rect);
                    },
                );
                if !still_open {
                    self.pmi_label_menu = None;
                }
                chosen = action.map(|action| (id, action));
            } else {
                self.pmi_label_menu = None;
            }
        }
        if let Some((id, action)) = chosen {
            if let Err(error) = crate::panels::pmi::run_row_action(state, &id, &action) {
                state.push_notice(format!("PMI: {error}"));
            }
        }
        if let Some(request) = request {
            self.pmi_label_menu = Some(request);
        }
    }

    /// Draw the transform gizmo's axis-end labels (`XC` red, `YC` green, `ZC`
    /// blue) at each cone tip while the ◎ is in TRANSFORM mode.
    ///
    /// PAINTED, not laid out: these are pure display text with no widget behind
    /// them, so they cannot take a press away from the gizmo they annotate.
    ///
    /// THE BUG THIS FIXES: they used to be `egui::Area`s marked
    /// `.interactable(false)`, which is NOT enough. That flag only excludes the
    /// area from `Memory::layer_id_at`; the area still registers its own
    /// `Sense::hover()` widget rect, and egui's `hit_test` filters layers by
    /// `Order::allow_interaction()` alone. A hit in the topmost layer whose
    /// rect covers the whole interact radius DROPS every widget in the layers
    /// behind it — including the viewport, whose `drag_started()` then never
    /// fires. So the label chip, ~20x17 pt, swallowed any press within ~10 pt
    /// of its centre.
    ///
    /// That had a shape: an axis pointing AT or AWAY FROM the camera
    /// foreshortens onto the gizmo origin, and its label lands exactly on the
    /// orange CENTRE free-move handle. Which is to say the centre handle was
    /// inert in precisely the AXIS-ALIGNED standard views — `YC` under TOP,
    /// `ZC` under FRONT, `XC` under RIGHT — and worked under ISO, where the
    /// nearest label sits ~102 pt away. Neither the anchor nor the camera
    /// moved on such a press, because the viewport never saw it.
    pub(super) fn draw_transform_axis_labels(
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        if state.gizmo_mode() != "transform" {
            return;
        }
        let labels: Vec<serde_json::Value> =
            serde_json::from_str(&state.transform_axis_labels_json()).unwrap_or_default();
        if labels.is_empty() {
            return;
        }
        // Project every label anchor world→screen in one shot.
        let worlds: Vec<[f64; 3]> = labels
            .iter()
            .map(|l| {
                let w = &l["world"];
                [
                    w[0].as_f64().unwrap_or(0.0),
                    w[1].as_f64().unwrap_or(0.0),
                    w[2].as_f64().unwrap_or(0.0),
                ]
            })
            .collect();
        let screens: Vec<[f64; 4]> = serde_json::to_string(&worlds)
            .ok()
            .and_then(|s| state.world_to_screen_json(&s).ok())
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();
        if screens.len() != labels.len() {
            return;
        }
        // The user's "Label scale".
        let font = label_font(&ctx.global_style(), state.settings.label_scale);
        // One painter over the viewport, in the layer the chips used to sit in.
        // A layer painter allocates no widget, so nothing here is hit-testable.
        let mut painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Middle,
            egui::Id::new("brep-transform-axis"),
        ));
        painter.set_clip_rect(rect);
        for (i, label) in labels.iter().enumerate() {
            let scr = screens[i];
            if scr[3] < 0.5 {
                // The engine's ONE label policy (ViewCamera::label_anchor_visible):
                // hidden only when perspective-behind-the-eye or the 3D anchor
                // projects OUTSIDE the viewport. Ortho depth / near / far NEVER
                // cull — never add such a test here.
                continue;
            }
            let text = label["text"].as_str().unwrap_or("");
            let rgb = &label["rgb"];
            let color = egui::Color32::from_rgb(
                (rgb[0].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[1].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
                (rgb[2].as_f64().unwrap_or(1.0) * 255.0).round() as u8,
            );
            let pos = egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32);
            painter.text(pos, egui::Align2::CENTER_CENTER, text, font.clone(), color);
        }
    }

    /// DEBUG overlay: draw a 1px RED outline of the EXACT click/hit region of
    /// EVERY currently-shown gizmo handle — arrows AND balls, for EVERY gizmo:
    /// the feature transform widget, the dimension arrows, AND the assembly
    /// component Move gizmo (which feeds the same widget with ◎ mode "none").
    /// The engine exposers ([`EngineState::transform_hit_areas_json`] /
    /// [`EngineState::dimension_hit_areas_json`]) hand back the SAME screen-space
    /// (viewport-local px) regions the hit test 2D-tests the cursor against — each
    /// a `{kind:"capsule", a, b, r}` (axis arrows, dimension leaders) or a
    /// `{kind:"circle", c, r}` (center / origin / grab / arc-handle spheres). The
    /// projection + the perspective front-clip ALREADY happened in the region
    /// builder, so here we only offset by `rect.min` and stroke — there is nothing
    /// to project or clip, and the outline is byte-identical to the pickable region.
    pub(super) fn draw_gizmo_hit_areas(
        &self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &EngineState,
    ) {
        // Debug-only overlay: off unless the "Debug grab handles" setting is on.
        if !state.settings.debug_grab_handles {
            return;
        }
        let json = match state.gizmo_mode() {
            "dimension" => state.dimension_hit_areas_json(),
            // The widget transform gizmo serves BOTH the feature transform mode
            // AND the component Move gizmo (whose ◎ mode is "none"), so anything
            // else asks the widget exposer — it returns `[]` exactly when the
            // widget is hidden, i.e. when nothing is grabbable.
            _ => state.transform_hit_areas_json(),
        };
        let areas: Vec<serde_json::Value> = serde_json::from_str(&json).unwrap_or_default();
        if areas.is_empty() {
            return;
        }
        let stroke = egui::Stroke::new(1.0, egui::Color32::RED);
        let mut painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Foreground,
            egui::Id::new("brep-gizmo-hit-areas"),
        ));
        painter.set_clip_rect(rect);
        let at = |v: &serde_json::Value| {
            egui::pos2(
                rect.min.x + v[0].as_f64().unwrap_or(0.0) as f32,
                rect.min.y + v[1].as_f64().unwrap_or(0.0) as f32,
            )
        };
        for area in &areas {
            let r = area["r"].as_f64().unwrap_or(0.0) as f32;
            match area["kind"].as_str() {
                Some("capsule") => {
                    draw_capsule_outline(&painter, at(&area["a"]), at(&area["b"]), r, stroke);
                }
                Some("circle") => {
                    painter.circle_stroke(at(&area["c"]), r, stroke);
                }
                _ => {}
            }
        }
    }
}

/// Stroke the 1px outline of a screen-space capsule (stadium): the pickable region
/// of a transform axis arrow — the segment `a→b` expanded by radius `px`. Two long
/// parallel edges + a semicircular cap at each end (each cap bulging AWAY from the
/// other end), assembled as ONE closed polyline. A projected segment that collapses
/// to ~a point (axis pointing at / away from the camera) degenerates to a disc of
/// radius `px` — the true region there — so a circle is stroked instead.
fn draw_capsule_outline(
    painter: &egui::Painter,
    a: egui::Pos2,
    b: egui::Pos2,
    px: f32,
    stroke: egui::Stroke,
) {
    let seg = b - a;
    let len = seg.length();
    if len < 1.0 {
        painter.circle_stroke(a, px, stroke);
        return;
    }
    let dir = seg / len;
    let perp = egui::vec2(-dir.y, dir.x) * px; // +perp offset (angle `ang0`)
    let ang0 = perp.y.atan2(perp.x);
    const CAP_SEGS: usize = 8;
    let pi = std::f32::consts::PI;
    let mut pts: Vec<egui::Pos2> = Vec::with_capacity(4 + 2 * CAP_SEGS);
    // +perp long edge: a+perp → b+perp.
    pts.push(a + perp);
    pts.push(b + perp);
    // Cap at b, bulging toward +dir: sweep +perp → -perp (angle ang0 → ang0-π).
    for k in 1..CAP_SEGS {
        let t = ang0 - pi * (k as f32) / (CAP_SEGS as f32);
        pts.push(b + egui::vec2(t.cos(), t.sin()) * px);
    }
    pts.push(b - perp);
    // -perp long edge: b-perp → a-perp.
    pts.push(a - perp);
    // Cap at a, bulging toward -dir: sweep -perp → +perp (angle ang0+π → ang0).
    for k in 1..CAP_SEGS {
        let t = (ang0 + pi) - pi * (k as f32) / (CAP_SEGS as f32);
        pts.push(a + egui::vec2(t.cos(), t.sin()) * px);
    }
    painter.add(egui::Shape::closed_line(pts, stroke));
}

fn fmt_dim_value(value: f64) -> String {
    brep_render::formatting::compact_decimal(value, 4)
}

// --- the transform gizmo's readout ---------------------------------------
//
// A gizmo drag was the one edit in the app that reported NOTHING. The model
// followed the pointer, the number went into `transform`, and the form's
// `Transform` section is collapsed by default — so the distance just dragged
// appeared nowhere on screen. And when the motion refused, the gizmo kept
// tracking the pointer while the geometry stayed at its last good answer, so
// the handles ended up somewhere the model was not (measured at 15.4 mm INSIDE
// the solid) with the only signal a red banner in the History dock, on the far
// side of the window from the pointer.
//
// Both are answered by one chip, at the gizmo, where the eyes already are.

/// The live chip's orange — the dimension-label orange, so a gizmo readout and
/// a dimension readout are the same object to the eye.
const READOUT_ORANGE: egui::Color32 = egui::Color32::from_rgb(245, 166, 35);
/// The chip background, as the dimension chips use.
const READOUT_DARK: egui::Color32 = egui::Color32::from_rgb(20, 20, 20);

impl super::Viewport {
    /// Draw the transform gizmo's readout chip, and the Transform Face pivot
    /// marker.
    ///
    /// Both are PAINTED as non-interactable areas: a chip that took a press
    /// would steal it from the gizmo handle underneath, which is the one thing
    /// an overlay on a live drag must never do (the same rule the XC/YC/ZC axis
    /// labels follow). There is consequently no hit key to document for either.
    pub(super) fn draw_transform_readout(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        self.draw_face_transform_pivot(ctx, rect, state);

        let readout: serde_json::Value =
            serde_json::from_str(&state.transform_readout_json()).unwrap_or(serde_json::Value::Null);
        if readout["active"].as_bool() != Some(true) {
            return;
        }
        let Some(pos) = project_world(state, rect, &readout["origin"]) else {
            return;
        };
        // A REFUSAL IS NOT REPORTED HERE — it already reaches the user TWICE.
        // `panels::history::feature_error_for_user` builds one sentence and
        // shows it in both the form's banner and the tree's error leaf, so a
        // third copy floating over the model said nothing new and said it on
        // the one surface the user needs clear to drag the motion back. The
        // refusal state is still published on `__brepGizmoReadout` for scripts
        // and for the gizmo's own live-follow; only the chip is gone.
        if readout["refused"].as_bool() == Some(true) {
            return;
        }
        let label_scale = state.settings.label_scale;

        // WHAT THE CHIP SAYS: the axis and the distance/angle of a live drag,
        // which is not an error and has no other home on screen.
        let value_text = readout["text"].as_str().unwrap_or_default().to_string();
        if value_text.is_empty() {
            return;
        }
        let lines: Vec<(String, egui::Color32)> = vec![(value_text, READOUT_ORANGE)];

        // CLEAR OF THE HANDLES. The chip is bottom-anchored and sits a fixed
        // distance straight up from the gizmo's origin — far enough to clear
        // the +Y arrow, whose tip is the highest thing the widget draws and
        // which is drawn at a CONSTANT screen size, so one constant clears it
        // at every zoom. Anything closer buries the very handles the user has
        // to grab.
        let anchor = egui::pos2(pos.x, pos.y - 86.0 * label_scale.max(1.0));
        let font = label_font(&ctx.global_style(), label_scale);

        egui::Area::new(egui::Id::new("brep-gizmo-readout"))
            .order(egui::Order::Middle)
            // NEVER interactable: the pointer belongs to the gizmo underneath.
            .interactable(false)
            .fixed_pos(anchor)
            .pivot(egui::Align2::CENTER_BOTTOM)
            .show(ctx, |ui| {
                egui::Frame::new()
                    .fill(READOUT_DARK)
                    .stroke(egui::Stroke::new(1.0, READOUT_ORANGE))
                    .corner_radius(egui::CornerRadius::same(6))
                    .inner_margin(chip_margin(7.0, 4.0, label_scale))
                    .show(ui, |ui| {
                        scale_label_spacing(ui, label_scale);
                        for (text, color) in &lines {
                            ui.label(
                                egui::RichText::new(text.clone())
                                    .font(font.clone())
                                    .color(*color),
                            );
                        }
                    });
            });
    }

    /// Mark the Transform Face PIVOT when the gizmo has moved off it.
    ///
    /// The feature turns about its stored `pivot`, and the gizmo sits at
    /// `pivot + position` — so the two coincide only while the motion is zero.
    /// The moment the user drags, the point the rotation actually happens about
    /// is no longer under the handles and was, before this, nowhere on screen
    /// at all: it existed only as three numbers in the form. Marked only when
    /// the two have come apart, so the common case gains no clutter.
    fn draw_face_transform_pivot(
        &mut self,
        ctx: &egui::Context,
        rect: egui::Rect,
        state: &mut EngineState,
    ) {
        let gizmo: serde_json::Value =
            serde_json::from_str(&state.gizmo_state_json()).unwrap_or(serde_json::Value::Null);
        let (Some(pivot), Some(origin)) = (gizmo["pivot"].as_array(), gizmo["origin"].as_array())
        else {
            return;
        };
        // Apart by enough to be worth two marks on screen.
        let apart = (0..3).any(|axis| {
            let p = pivot.get(axis).and_then(serde_json::Value::as_f64).unwrap_or(0.0);
            let o = origin.get(axis).and_then(serde_json::Value::as_f64).unwrap_or(0.0);
            (p - o).abs() > 1e-6
        });
        if !apart {
            return;
        }
        let Some(pos) = project_world(state, rect, &gizmo["pivot"]) else {
            return;
        };

        // A painted cross-in-a-ring, not a widget: nothing here may take a
        // press from the gizmo.
        let painter = ctx.layer_painter(egui::LayerId::new(
            egui::Order::Middle,
            egui::Id::new("brep-face-transform-pivot"),
        ));
        let scale = state.settings.label_scale;
        let r = 5.0 * scale;
        let stroke = egui::Stroke::new(1.5, READOUT_ORANGE);
        painter.circle_stroke(pos, r, stroke);
        painter.line_segment([pos - egui::vec2(r * 1.8, 0.0), pos + egui::vec2(r * 1.8, 0.0)], stroke);
        painter.line_segment([pos - egui::vec2(0.0, r * 1.8), pos + egui::vec2(0.0, r * 1.8)], stroke);
        painter.text(
            pos + egui::vec2(r * 2.2, -r * 2.2),
            egui::Align2::LEFT_BOTTOM,
            "pivot",
            label_font(&ctx.global_style(), scale),
            READOUT_ORANGE,
        );
    }
}

/// Project a `[x, y, z]` JSON world point to a viewport screen position, or
/// `None` when it is not on screen (the engine's ONE label policy —
/// behind-the-eye or outside the viewport; ortho depth NEVER culls).
fn project_world(
    state: &mut EngineState,
    rect: egui::Rect,
    world: &serde_json::Value,
) -> Option<egui::Pos2> {
    let point = [
        world.get(0)?.as_f64()?,
        world.get(1)?.as_f64()?,
        world.get(2)?.as_f64()?,
    ];
    let json = serde_json::to_string(&[point]).ok()?;
    let screens: Vec<[f64; 4]> = serde_json::from_str(&state.world_to_screen_json(&json).ok()?).ok()?;
    let scr = screens.first()?;
    (scr[3] >= 0.5).then(|| egui::pos2(rect.min.x + scr[0] as f32, rect.min.y + scr[1] as f32))
}

