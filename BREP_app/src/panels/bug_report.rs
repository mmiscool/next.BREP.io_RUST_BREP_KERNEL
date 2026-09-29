//! In-app **Submit Bug** report.
//!
//! A toolbar button opens this flow. It captures, WITHOUT the report dialog in
//! the shot:
//!   * a **screenshot** of the whole application (egui UI + the 3D model) as it
//!     looked the instant the button was pressed,
//!   * the current **model** (the `.nbrep` recipe), and
//!   * a user **description** (+ an optional email), and
//!   * the session's **diagnostics** — which renderer is actually in use and
//!     what it is running on, appended to the description (see
//!     [`compose_description`] for why they travel inside that field),
//! then POSTs them to the public reports endpoint (`v2.brep.io/api/report`).
//! ONE code path runs on both native and wasm.
//!
//! ## Screenshot-before-dialog
//! egui/eframe captures a screenshot of the FRAME (the composited surface, so it
//! includes the 3D viewport, which is an `egui_wgpu` paint callback into egui's
//! frame — see [`crate::viewport`]). We must not let the dialog appear in that
//! frame, so the flow is a small state machine:
//!   1. Button click → snapshot the model, send `ViewportCommand::Screenshot`,
//!      enter [`Phase::Capturing`]. The dialog is NOT drawn while capturing.
//!   2. A later frame delivers `Event::Screenshot`; we encode it to PNG, build a
//!      preview thumbnail, and enter [`Phase::Editing`] — only NOW is the dialog
//!      drawn, so the captured frame(s) never contain it.
//!   3. Submit builds a small multipart body by hand (no extra deps) and fires
//!      it through `ehttp`; the reply marshals back over an mpsc channel +
//!      `request_repaint`, exactly like [`crate::panels::step_parts`].

use crate::automation::hit_keys::HitKeyDoc;
use crate::diagnostics::Diagnostics;
use std::sync::mpsc::Receiver;

use brep_render::engine_state::EngineState;
use crate::icon_text::IconTextUi as _;
use eframe::egui;

/// The public reports endpoint the button posts to (fronted
/// by v2.brep.io). Accepts the multipart fields
/// `description`,`email`,`model`,`screenshot`.
const REPORT_URL: &str = "https://v2.brep.io/api/report";

/// The feature name [`crate::offsite`]'s refusal sentence starts with.
const FEATURE: &str = "Bug report";

/// The server's cap on the `description` field, in CHARACTERS
/// (`description.chars().take(MAX_DESC)` in the endpoint's own
/// `routes/reports.rs`). It truncates the TAIL, and the diagnostics block is at
/// the tail — so [`compose_description`] clamps the user's own text to leave
/// room rather than letting a very long description silently cut the
/// diagnostics off.
const MAX_DESC: usize = 20_000;

/// Frames to wait for the screenshot event before giving up and opening the
/// dialog anyway (so a device that never delivers the capture can't hang the
/// flow). ~1s at 60fps; the persistent offscreen 3D means the capture normally
/// lands within a few frames.
const CAPTURE_TIMEOUT_FRAMES: u32 = 60;

/// The dialog's width in points, on a window wide enough for it.
const DIALOG_WIDTH: f32 = 560.0;

/// How many rows of text the description shows before it scrolls instead of
/// growing.
const DESCRIPTION_ROWS: usize = 12;

/// The description field's own text margin (egui's default, named so the
/// field's cap can count it).
const DESCRIPTION_MARGIN: egui::Margin = egui::Margin::symmetric(4, 2);

/// The height of the dialog's scrolling body on a window with room for it. The
/// body is exactly this tall — never taller, never shorter — so the dialog is
/// ONE size for as long as it is open, whatever is typed into it; on a window
/// too short for it, the body takes what the window has left instead.
const BODY_HEIGHT: f32 = 460.0;

/// The gap above the button row, and between it and the status line.
const FOOTER_GAP: f32 = 10.0;
const STATUS_GAP: f32 = 6.0;

/// Whether the description's Cut / Copy / Paste menu can reach the clipboard.
/// Native eframe turns those requests into the same input as the keyboard
/// shortcuts; the web runner does not handle them at all, and a browser only
/// hands a page the clipboard inside its own copy/paste events — so there the
/// menu says which keys to press instead.
const MENU_REACHES_CLIPBOARD: bool = cfg!(not(target_arch = "wasm32"));

/// Where the flow is between "button pressed" and "dialog closed".
#[derive(Default, PartialEq)]
enum Phase {
    /// Nothing in progress.
    #[default]
    Idle,
    /// Screenshot requested; dialog intentionally hidden so it isn't captured.
    Capturing { frames: u32 },
    /// Screenshot in hand; dialog open, collecting description + email.
    Editing,
    /// POST in flight.
    Sending,
}

pub struct BugReportPanel {
    phase: Phase,
    /// The problem description (required to submit).
    description: String,
    /// Optional reporter email.
    email: String,
    /// A short status / error line under the buttons.
    status: String,
    /// PNG bytes of the pre-dialog screenshot (UI + 3D), if captured.
    screenshot_png: Option<Vec<u8>>,
    /// A preview texture of the screenshot shown in the dialog.
    thumb: Option<egui::TextureHandle>,
    /// The model (`.nbrep`) snapshotted at button-press time.
    model_json: String,
    /// The session diagnostics block, taken from the app's ONE
    /// [`Diagnostics`] at button-press time — the same text the Info window
    /// shows, so a report can never describe a different machine from the one
    /// the user was reading about.
    diagnostics: String,
    /// The in-flight POST reply channel (drained each frame).
    response_rx: Option<Receiver<Result<(), String>>>,
    /// Per-frame widget rects for the headed verifier (wasm only).
    hits: std::collections::HashMap<String, egui::Rect>,
    /// How the description's scroll area stood on the last drawn frame, for the
    /// verifier: whether a long report overflows the field and where it is
    /// scrolled to.
    description_scroll: DescriptionScroll,
}

/// The description field's scroll extent, in points.
#[derive(Clone, Copy, Default, Debug, PartialEq)]
struct DescriptionScroll {
    /// The height of the text.
    content: f32,
    /// The height of the field that shows it.
    viewport: f32,
    /// How far down the text is scrolled.
    offset: f32,
}

impl BugReportPanel {
    pub fn new() -> Self {
        Self {
            phase: Phase::Idle,
            description: String::new(),
            email: String::new(),
            status: String::new(),
            screenshot_png: None,
            thumb: None,
            model_json: String::new(),
            diagnostics: String::new(),
            response_rx: None,
            hits: std::collections::HashMap::new(),
            description_scroll: DescriptionScroll::default(),
        }
    }

    /// Toolbar entry point: snapshot the model + the session diagnostics,
    /// request a screenshot of THIS frame (before the dialog exists), and begin
    /// capturing. Ignored if a report flow is already in progress.
    ///
    /// `diagnostics` is the app's ONE instance — the report renders it here
    /// rather than collecting anything of its own, which is what keeps the
    /// submitted text and the Info window's rows the same rows.
    pub fn request(&mut self, ctx: &egui::Context, state: &EngineState, diagnostics: &Diagnostics) {
        if self.phase != Phase::Idle {
            return;
        }
        self.description.clear();
        self.email.clear();
        self.status.clear();
        self.screenshot_png = None;
        self.thumb = None;
        self.response_rx = None;
        // The model can't change while the modal is open, but snapshot it now so
        // the report reflects exactly the state the user was looking at.
        self.model_json = state.history_request_json();
        self.diagnostics = diagnostics.report_text();
        ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
        self.phase = Phase::Capturing { frames: 0 };
        ctx.request_repaint();
    }

    /// Draw + drive the flow. Called once per frame at ctx level (like the file
    /// dialog). Idempotent while [`Phase::Idle`].
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();

        self.poll_capture(ctx);
        self.drain_response(state);

        if !matches!(self.phase, Phase::Editing | Phase::Sending) {
            return;
        }

        let sending = self.phase == Phase::Sending;
        let mut submit = false;
        let mut cancel = false;
        let modal = egui::Modal::new(egui::Id::new("brep-bug-report")).show(ctx, |ui| {
            let window = ctx.content_rect();
            ui.set_width(DIALOG_WIDTH.min((window.width() - 40.0).max(240.0)));
            // Scroll bars that SHOW whenever there is more to scroll. egui's
            // default floating bars stay invisible until the pointer is over the
            // field, so a report longer than the field read as text that had run
            // out of room, with nothing to say there was more.
            ui.spacing_mut().scroll = egui::style::ScrollStyle::thin();
            // `icon_label`, not `heading`: U+1F41E is a catalogued COLOUR icon, so
            // this draws the real artwork inline with the title instead of the
            // font's monochrome outline.
            ui.icon_label(
                egui::RichText::new("\u{1F41E}  Submit a bug report").heading(),
            );
            ui.add_space(4.0);
            ui.label(
                "Describe what went wrong. Your current model and a screenshot of \
                 the app (UI + 3D view) are attached automatically.",
            );
            ui.add_space(2.0);
            ui.weak("Your report and its screenshot may be shown publicly on the bug list.");
            ui.add_space(8.0);

            // Everything between the heading and the buttons is ONE scrolling
            // body, capped to what the window has left once the heading above
            // and the footer below are counted. However long the description,
            // and with the screenshot and the diagnostics expanded, the dialog
            // stays on screen and Submit stays in it — a dialog that grew with
            // its contents put the button below the bottom of the window.
            //
            // The heading is measured from `min_rect`, which does not depend on
            // where the centred modal lands, so there is no size/position
            // feedback. A modal's ui is only as tall as the modal was LAST frame,
            // so the body gets a child ui whose max rect IS the budget (the
            // same construction as the command palette's list).
            //
            // The body is a FIXED height, not a cap. A capped body still grew
            // with its text up to the cap, and a modal is re-centred from its
            // PREVIOUS frame's size: every frame the text grew a row, the dialog
            // got taller downward before it was moved up, so while the user typed
            // its bottom edge ran off the window (889.5 on an 800-tall window,
            // 250 characters in). A body that is one height from the first frame
            // gives the modal one size, so there is nothing to re-centre and
            // nothing to overflow; the text scrolls inside it instead.
            let chrome = ui.min_rect().height();
            let room = (window.height() - chrome - self.footer_height(ui) - 48.0).max(120.0);
            let body_height = room.min(BODY_HEIGHT);
            let budget = egui::Rect::from_min_size(
                ui.cursor().min,
                egui::vec2(ui.available_width(), body_height),
            );
            ui.scope_builder(egui::UiBuilder::new().max_rect(budget), |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("brep-bug-body")
                    .min_scrolled_height(body_height)
                    .max_height(body_height)
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui));
            });

            ui.add_space(FOOTER_GAP);
            ui.horizontal(|ui| {
                let can_submit = !self.description.trim().is_empty() && !sending;
                let label = if sending { "Sending\u{2026}" } else { "Submit report" };
                let submit_btn = ui.add_enabled(can_submit, egui::Button::new(label));
                self.hit("submit", &submit_btn);
                if submit_btn.clicked() {
                    submit = true;
                }
                let cancel_btn = ui.add_enabled(!sending, egui::Button::new("Cancel"));
                self.hit("cancel", &cancel_btn);
                if cancel_btn.clicked() {
                    cancel = true;
                }
            });
            // The status line's row is always there, empty or not, so a status
            // appearing ("Submitting…") does not change the dialog's size either.
            ui.add_space(STATUS_GAP);
            ui.weak(if self.status.is_empty() { " " } else { self.status.as_str() });
        });

        if submit {
            self.send(ctx);
        } else if cancel || (modal.should_close() && !sending) {
            self.reset();
        }
    }

    /// The height [`Self::show`] draws under the body: the gap, the button row
    /// and the status line's row, which is always drawn. Worked out from the
    /// style the footer is drawn with, so the body's budget is right on the
    /// first frame.
    fn footer_height(&self, ui: &egui::Ui) -> f32 {
        let spacing = ui.spacing();
        let buttons = spacing.interact_size.y.max(
            ui.text_style_height(&egui::TextStyle::Button) + 2.0 * spacing.button_padding.y,
        );
        let status =
            STATUS_GAP + ui.text_style_height(&egui::TextStyle::Body) + 2.0 * spacing.item_spacing.y;
        FOOTER_GAP + buttons + 2.0 * spacing.item_spacing.y + status
    }

    /// The dialog's scrolling body: the description, the email, the attached
    /// screenshot and the diagnostics.
    fn body(&mut self, ui: &mut egui::Ui) {
        ui.label("What happened?");
        self.description_field(ui);

        ui.add_space(6.0);
        ui.label("Email (optional)");
        let email = ui.add(
            egui::TextEdit::singleline(&mut self.email)
                .desired_width(f32::INFINITY)
                .hint_text("so we can follow up — optional"),
        );
        self.hit("field:email", &email);

        ui.add_space(8.0);
        if let Some(tex) = &self.thumb {
            ui.label("Attached screenshot:");
            ui.add_space(2.0);
            // Fit the preview to the dialog width, keeping aspect.
            let size = tex.size_vec2();
            let scale = ((DIALOG_WIDTH - 40.0).min(ui.available_width()) / size.x).min(1.0);
            ui.add(
                egui::Image::new((tex.id(), size * scale))
                    .corner_radius(4.0)
                    .bg_fill(egui::Color32::from_gray(20)),
            );
        } else {
            ui.weak("(screenshot unavailable — the model + description will still be sent)");
        }

        // What the report will carry about this machine, shown before it is
        // sent rather than attached invisibly. Collapsed: it is for the
        // triager, and the user has already read it in the Info window if
        // they wanted to.
        ui.add_space(8.0);
        let diag = egui::CollapsingHeader::new("Diagnostics attached to this report")
            .id_salt("brep-bug-diagnostics")
            .show(ui, |ui| {
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(&self.diagnostics).monospace().small(),
                    )
                    .wrap(),
                );
            });
        self.hits.insert("diagnostics".to_string(), diag.header_response.interact_rect);
    }

    /// The description. It grows with its text up to [`DESCRIPTION_ROWS`] rows
    /// and then scrolls inside its own bar, so a long report never grows the
    /// dialog; the field follows the caret as the user types.
    ///
    /// A bare scroll area is enough here because the body it sits in was given
    /// an explicit budget: the room it measures is that budget, not the size
    /// the modal happened to have last frame.
    fn description_field(&mut self, ui: &mut egui::Ui) {
        let cap = DESCRIPTION_ROWS as f32 * ui.text_style_height(&egui::TextStyle::Body)
            + DESCRIPTION_MARGIN.sum().y;
        let scroll = egui::ScrollArea::vertical()
            .id_salt("brep-bug-description")
            .max_height(cap)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                ui.add(
                    egui::TextEdit::multiline(&mut self.description)
                        .margin(DESCRIPTION_MARGIN)
                        .desired_rows(5)
                        .desired_width(f32::INFINITY)
                        .hint_text("Steps, what you expected, and what actually happened"),
                )
            });
        self.description_scroll = DescriptionScroll {
            content: scroll.content_size.y,
            viewport: scroll.inner_rect.height(),
            offset: scroll.state.offset.y,
        };
        let field = scroll.inner;
        self.hit("field:description", &field);
        field.context_menu(|ui| self.edit_menu(ui, &field));
    }

    /// The description's right-click menu: Cut, Copy and Paste. An egui text
    /// field has no menu of its own, and right-click → Paste is how a lot of
    /// people paste — on a Mac especially, where the reporter looked for it.
    ///
    /// Each entry asks the platform for exactly what its keyboard shortcut
    /// does, and first hands focus back to the field: clicking the menu took
    /// focus away, and a text field ignores a paste it is not focused for. The
    /// platform delivers the action as input on the NEXT frame, by which time
    /// the field has focus again.
    fn edit_menu(&mut self, ui: &mut egui::Ui, field: &egui::Response) {
        let entries = [
            ("menu:cut", "Cut", egui::ViewportCommand::RequestCut),
            ("menu:copy", "Copy", egui::ViewportCommand::RequestCopy),
            ("menu:paste", "Paste", egui::ViewportCommand::RequestPaste),
        ];
        for (key, label, command) in entries {
            let item = ui.add_enabled(MENU_REACHES_CLIPBOARD, egui::Button::new(label));
            self.hit(key, &item);
            if item.clicked() {
                field.request_focus();
                ui.ctx().send_viewport_cmd(command);
                ui.close();
            }
        }
        if !MENU_REACHES_CLIPBOARD {
            ui.weak("In the browser, use Ctrl+X / C / V (Cmd on a Mac).");
        }
    }

    /// While capturing, look for the delivered screenshot; time out gracefully.
    fn poll_capture(&mut self, ctx: &egui::Context) {
        let frames = match &mut self.phase {
            Phase::Capturing { frames } => {
                *frames += 1;
                *frames
            }
            _ => return,
        };
        // eframe injects `Event::Screenshot` into the frame's raw input once the
        // async framebuffer readback completes. Take the newest one.
        let shot = ctx.input(|i| {
            i.raw.events.iter().rev().find_map(|e| match e {
                egui::Event::Screenshot { image, .. } => Some(image.clone()),
                _ => None,
            })
        });
        if let Some(img) = shot {
            self.screenshot_png = encode_png(&img);
            self.thumb = Some(ctx.load_texture(
                "brep-bug-shot",
                (*img).clone(),
                egui::TextureOptions::LINEAR,
            ));
            self.phase = Phase::Editing;
        } else if frames > CAPTURE_TIMEOUT_FRAMES {
            self.status = "(screenshot unavailable)".into();
            self.phase = Phase::Editing;
        } else {
            ctx.request_repaint();
        }
    }

    /// Drain the POST reply: success closes the dialog with a toast; an error
    /// stays open so the user can retry.
    fn drain_response(&mut self, state: &mut EngineState) {
        let Some(rx) = &self.response_rx else { return };
        let Ok(result) = rx.try_recv() else { return };
        self.response_rx = None;
        match result {
            Ok(()) => {
                state.push_notice("Bug report submitted — thank you!".to_string());
                self.reset();
            }
            Err(e) => {
                self.status = crate::offsite::explain(FEATURE, REPORT_URL, format!("Submit failed: {e}"));
                self.phase = Phase::Editing;
            }
        }
    }

    /// Build the multipart body and fire the POST (native + wasm via ehttp).
    fn send(&mut self, ctx: &egui::Context) {
        // On a PLM-hosted app that does not allow the report site, say so
        // instead of letting the browser refuse the request (crate::offsite).
        if let Some(sentence) = crate::offsite::refusal(FEATURE, REPORT_URL) {
            self.status = sentence;
            return;
        }
        self.status = "Submitting\u{2026}".into();
        self.phase = Phase::Sending;

        let (content_type, body) = build_multipart(
            &compose_description(&self.description, &self.diagnostics),
            &self.email,
            &self.model_json,
            self.screenshot_png.as_deref(),
        );
        let mut req = ehttp::Request::post(REPORT_URL, body);
        // `Request::post` sets text/plain; replace it with our multipart type.
        req.headers
            .headers
            .retain(|(k, _)| !k.eq_ignore_ascii_case("content-type"));
        req.headers.headers.push(("Content-Type".to_string(), content_type));

        let (tx, rx) = std::sync::mpsc::channel();
        let ctx = ctx.clone();
        ehttp::fetch(req, move |result| {
            let out = match result {
                Ok(resp) if resp.ok => Ok(()),
                Ok(resp) => Err(format!("HTTP {} {}", resp.status, resp.status_text)),
                Err(err) => Err(err),
            };
            let _ = tx.send(out);
            ctx.request_repaint();
        });
        self.response_rx = Some(rx);
    }

    /// Back to idle, dropping the screenshot + preview texture.
    fn reset(&mut self) {
        self.phase = Phase::Idle;
        self.description.clear();
        self.email.clear();
        self.status.clear();
        self.diagnostics.clear();
        self.screenshot_png = None;
        self.thumb = None;
        self.response_rx = None;
        self.description_scroll = DescriptionScroll::default();
    }

    /// Record a widget's screen rect for the verifier — its VISIBLE part: the
    /// body and the description both scroll, and a click at the centre of a
    /// rect that runs past its scroll area's clip lands on nothing.
    fn hit(&mut self, key: &str, resp: &egui::Response) {
        self.hits.insert(key.to_string(), resp.interact_rect);
    }

    /// Logical state for the headed verifier: which phase + whether a shot was
    /// captured.
    pub fn state_json(&self) -> String {
        let phase = match self.phase {
            Phase::Idle => "idle",
            Phase::Capturing { .. } => "capturing",
            Phase::Editing => "editing",
            Phase::Sending => "sending",
        };
        serde_json::json!({
            "phase": phase,
            "hasScreenshot": self.screenshot_png.is_some(),
            "diagnostics": self.diagnostics,
            "status": self.status,
            "descriptionScroll": {
                "content": self.description_scroll.content,
                "viewport": self.description_scroll.viewport,
                "offset": self.description_scroll.offset,
            },
        })
        .to_string()
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        let map: serde_json::Map<String, serde_json::Value> = self
            .hits
            .iter()
            .map(|(k, r)| {
                (
                    k.clone(),
                    serde_json::json!([r.center().x, r.center().y, r.width(), r.height()]),
                )
            })
            .collect();
        serde_json::Value::Object(map).to_string()
    }
}

/// Encode an egui `ColorImage` (the screenshot) to PNG bytes via the pure-Rust
/// `image` crate (wasm-safe). `None` on a zero-size image or encode error.
fn encode_png(color: &egui::ColorImage) -> Option<Vec<u8>> {
    use image::ImageEncoder;
    let [w, h] = color.size;
    if w == 0 || h == 0 {
        return None;
    }
    let mut rgba = Vec::with_capacity(w * h * 4);
    for px in &color.pixels {
        // Straight (un-premultiplied) sRGBA, matching a normal PNG.
        rgba.extend_from_slice(&px.to_srgba_unmultiplied());
    }
    let mut png = Vec::new();
    image::codecs::png::PngEncoder::new(&mut png)
        .write_image(&rgba, w as u32, h as u32, image::ExtendedColorType::Rgba8)
        .ok()?;
    Some(png)
}

/// The `description` the report actually submits: what the user typed, then the
/// diagnostics block.
///
/// **Why inside `description` and not its own part.** The endpoint
/// (`v2.brep.io/api/report`) parses multipart into a fixed four-field record
/// and its match ends `_ => {}` — an unknown part is accepted and silently
/// DROPPED, never rejected. A `diagnostics` part would therefore submit cleanly
/// and arrive nowhere, which is the worst of both: a client that looks like it
/// reports the renderer and a server that never stores it. `description` is
/// stored verbatim in `report.json` and is what a triager reads first, so the
/// block goes there until the server grows a field of its own.
///
/// APPENDED, never prepended: the server's push notification and its public
/// report list both summarise a report by its FIRST LINE, and a list where every
/// row reads "--- diagnostics ---" tells a reader nothing.
///
/// The user's own text is clamped so the block survives the server's
/// [`MAX_DESC`] tail truncation. A description long enough to hit that cap has
/// already said what it has to say; the diagnostics are the part that cannot be
/// re-derived later.
fn compose_description(description: &str, diagnostics: &str) -> String {
    if diagnostics.is_empty() {
        return description.chars().take(MAX_DESC).collect();
    }
    const SEPARATOR: &str = "\n\n";
    // `chars`, matching the server's own `chars().take(MAX_DESC)`.
    let block_len = diagnostics.chars().count() + SEPARATOR.chars().count();
    let room = MAX_DESC.saturating_sub(block_len);
    let user: String = description.chars().take(room).collect();
    format!("{user}{SEPARATOR}{diagnostics}")
}

/// Hand-build a `multipart/form-data` body (avoids ehttp's `multipart` feature,
/// which pulls `rand`→`getrandom` and would need the wasm `js` feature). The
/// boundary carries a distinctive ASCII prefix plus the payload lengths so it
/// can't collide with the (text) model JSON or the PNG bytes.
fn build_multipart(
    description: &str,
    email: &str,
    model: &str,
    screenshot: Option<&[u8]>,
) -> (String, Vec<u8>) {
    let boundary = format!(
        "----BREPBugReport{:x}x{:x}Boundary",
        model.len(),
        screenshot.map(|s| s.len()).unwrap_or(0)
    );
    let mut body = Vec::new();
    push_text_field(&mut body, &boundary, "description", description);
    push_text_field(&mut body, &boundary, "email", email);
    push_file_field(
        &mut body,
        &boundary,
        "model",
        "model.nbrep",
        "application/json",
        model.as_bytes(),
    );
    if let Some(png) = screenshot {
        push_file_field(&mut body, &boundary, "screenshot", "screenshot.png", "image/png", png);
    }
    body.extend_from_slice(format!("--{boundary}--\r\n").as_bytes());
    (format!("multipart/form-data; boundary={boundary}"), body)
}

fn push_text_field(body: &mut Vec<u8>, boundary: &str, name: &str, value: &str) {
    body.extend_from_slice(
        format!("--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n")
            .as_bytes(),
    );
    body.extend_from_slice(value.as_bytes());
    body.extend_from_slice(b"\r\n");
}

fn push_file_field(
    body: &mut Vec<u8>,
    boundary: &str,
    name: &str,
    filename: &str,
    content_type: &str,
    bytes: &[u8],
) {
    body.extend_from_slice(
        format!(
            "--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"; \
             filename=\"{filename}\"\r\nContent-Type: {content_type}\r\n\r\n"
        )
        .as_bytes(),
    );
    body.extend_from_slice(bytes);
    body.extend_from_slice(b"\r\n");
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "bug", prefix: "field:description", meaning: "the description field (its visible part; right-click opens its Cut / Copy / Paste menu)", command: None },
    HitKeyDoc { panel: "bug", prefix: "menu:cut", meaning: "cut the description's selection, from its right-click menu (native; disabled in the browser)", command: None },
    HitKeyDoc { panel: "bug", prefix: "menu:copy", meaning: "copy the description's selection, from its right-click menu (native; disabled in the browser)", command: None },
    HitKeyDoc { panel: "bug", prefix: "menu:paste", meaning: "paste the clipboard into the description, from its right-click menu (native; disabled in the browser)", command: None },
    HitKeyDoc { panel: "bug", prefix: "field:email", meaning: "the email field", command: None },
    HitKeyDoc { panel: "bug", prefix: "diagnostics", meaning: "expand the diagnostics block the report will carry (the `diagnostics` command returns the same rows, but expanding a header is not what it does)", command: None },
    HitKeyDoc { panel: "bug", prefix: "submit", meaning: "submit the report", command: None },
    HitKeyDoc { panel: "bug", prefix: "cancel", meaning: "close the report", command: None },
];

