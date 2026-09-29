//! The DOCS ANNOTATION overlay: the caption band and the highlight ring that
//! turn a headless screen capture into a frame of a feature walkthrough.
//!
//! # Why the host paints it and not a compositor
//!
//! The obvious place for annotation is after capture, in `BREP_mcp_core`'s
//! image module, next to the cropper and the drawn cursor. It is not there for
//! one reason: **text needs a font and this repo ships none.** `BREP_app/src/fonts.rs`
//! is explicit — the only text face on native Linux is whatever `fc-match
//! monospace` names, read at run time; there is no font file in the checkout to
//! rasterise with, and `image` cannot draw text at all. A compositor would have
//! meant either a new crate (`ab_glyph` / `imageproc`) or a hand-rolled bitmap
//! face, and the project's stance on new dependencies is the one stated in
//! `BREP_kernel/src/io/three_mf/inflate.rs`.
//!
//! The host already has a laid-out, hinted, correctly-fallen-back font: the
//! app's own. So the overlay is painted in egui, on a layer above everything the
//! app drew, in the same frame the harness then renders — and the existing
//! `screenshot` path captures it with no changes at all, including its
//! downscale.
//!
//! The second reason is the highlight. A ring is only worth drawing if it is
//! around the real control, and the real control's rectangle is published every
//! frame by the app itself (`automation::registry::hit_rects`, egui points).
//! Painting inside the egui context means the ring is drawn in exactly those
//! coordinates, with no points-to-pixels conversion to get wrong and no
//! separate scaling to keep in step with `max_width`.
//!
//! # What it does NOT do
//!
//! It draws no widget, publishes no hit rect, touches no document and holds no
//! state the app can see. [`Annotated`] is a wrapper `eframe::App` around
//! `BrepApp` whose `ui` calls the app's and then paints; with no annotation
//! set — which is every script but the walkthroughs — it adds one branch per
//! frame and nothing else.
use eframe::egui;
use serde_json::Value;

/// The accent the ring is drawn in: the same amber a highlight is expected to
/// be, chosen to read against both the app's dark chrome and a light 3D
/// background.
const ACCENT: egui::Color32 = egui::Color32::from_rgb(255, 176, 0);
/// The caption band's ink and ground. OPAQUE: at 235 the status bar's text
/// showed through the band as a ghost line under the caption.
const BAND: egui::Color32 = egui::Color32::from_rgb(12, 14, 18);
const CAPTION_INK: egui::Color32 = egui::Color32::from_rgb(238, 240, 244);
const TITLE_INK: egui::Color32 = ACCENT;

const PAD: f32 = 14.0;
const TITLE_SIZE: f32 = 15.0;
const CAPTION_SIZE: f32 = 21.0;

/// What the overlay draws this frame. Empty = nothing is drawn.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Annotation {
    /// A short lead above the caption — the walkthrough's subject, repeated on
    /// every frame so a still lifted out of the GIF still says what it is of.
    pub title: Option<String>,
    /// The sentence describing what this frame shows.
    pub caption: Option<String>,
    /// Widget keys (as `hit_rects` publishes them) to ring.
    pub highlight: Vec<String>,
    /// Surface points, in egui points, to ring with a circle. A pick in the 3D
    /// view has no published rectangle — the `annotate` tool resolves an
    /// `entity` through the app's own `locate` and hands the projected point
    /// here, so the ring lands on the edge or face the next click takes.
    pub points: Vec<[f32; 2]>,
    /// `(n, total)`, drawn at the right of the band.
    pub step: Option<(u32, u32)>,
}

impl Annotation {
    pub fn is_empty(&self) -> bool {
        self.title.is_none()
            && self.caption.is_none()
            && self.highlight.is_empty()
            && self.points.is_empty()
            && self.step.is_none()
    }

    /// Parse the `annotate` tool's argument object. Unknown keys are refused:
    /// a misspelled `hilight` that silently drew nothing would be exactly the
    /// drift these videos exist to avoid.
    pub fn parse(spec: &Value) -> Result<Self, String> {
        let Some(obj) = spec.as_object() else {
            return Err(format!("annotate: expected an object, got {spec}"));
        };
        let mut out = Annotation::default();
        for (key, value) in obj {
            match key.as_str() {
                "title" | "caption" => {
                    let text = match value {
                        Value::Null => None,
                        Value::String(s) if s.is_empty() => None,
                        Value::String(s) => Some(s.clone()),
                        other => return Err(format!("annotate: `{key}` must be a string, got {other}")),
                    };
                    if key == "title" {
                        out.title = text;
                    } else {
                        out.caption = text;
                    }
                }
                "highlight" => match value {
                    Value::Null => {}
                    Value::String(s) => out.highlight.push(s.clone()),
                    Value::Array(a) => {
                        for item in a {
                            let s = item.as_str().ok_or_else(|| format!("annotate: `highlight` entries must be strings, got {item}"))?;
                            out.highlight.push(s.to_string());
                        }
                    }
                    other => return Err(format!("annotate: `highlight` must be a string or an array of strings, got {other}")),
                },
                "point" => match value {
                    Value::Null => {}
                    Value::Array(a) if a.len() == 2 && a.iter().all(|v| v.is_number()) => {
                        out.points.push([a[0].as_f64().unwrap_or(0.0) as f32, a[1].as_f64().unwrap_or(0.0) as f32]);
                    }
                    Value::Array(a) => {
                        for item in a {
                            let pair = item
                                .as_array()
                                .filter(|p| p.len() == 2 && p.iter().all(|v| v.is_number()))
                                .ok_or_else(|| format!("annotate: `point` entries must be `[x, y]`, got {item}"))?;
                            out.points.push([pair[0].as_f64().unwrap_or(0.0) as f32, pair[1].as_f64().unwrap_or(0.0) as f32]);
                        }
                    }
                    other => return Err(format!("annotate: `point` must be `[x, y]` or an array of them, got {other}")),
                },
                "step" => match value {
                    Value::Null => {}
                    Value::Array(a) if a.len() == 2 => {
                        let n = a[0].as_u64().ok_or("annotate: `step` entries must be whole numbers")?;
                        let total = a[1].as_u64().ok_or("annotate: `step` entries must be whole numbers")?;
                        out.step = Some((n as u32, total as u32));
                    }
                    other => return Err(format!("annotate: `step` must be `[n, total]`, got {other}")),
                },
                other => return Err(format!("annotate: unknown field `{other}` (title | caption | highlight | point | step)")),
            }
        }
        Ok(out)
    }
}

/// Every widget rectangle the app published on its last frame, in egui points.
pub fn published_rects() -> std::collections::BTreeMap<String, [f32; 4]> {
    brep_app::automation::registry::lock().hit_rects(None)
}

/// The keys in `wanted` that the app is not publishing, with the nearest
/// published keys to each, so a renamed widget fails by name.
///
/// This is what stops a walkthrough going quietly wrong: a highlight whose
/// control has been renamed draws nothing at all, and a GIF with no ring looks
/// like a design choice rather than a break.
pub fn unknown_keys(wanted: &[String]) -> Vec<String> {
    let published = published_rects();
    wanted
        .iter()
        .filter(|k| !published.contains_key(k.as_str()))
        .map(|k| {
            let prefix = k.split('/').next().unwrap_or("");
            let near: Vec<&str> = published
                .keys()
                .filter(|p| p.starts_with(prefix))
                .map(String::as_str)
                .take(6)
                .collect();
            if near.is_empty() {
                format!("`{k}` is not a published widget")
            } else {
                format!("`{k}` is not a published widget (nearby: {})", near.join(", "))
            }
        })
        .collect()
}

/// Ring a POINT in the 3D view — a pick, or a gizmo handle whose published rect
/// is zero-size. A dark hairline outside the amber so it survives on a light 3D
/// background as well as on the dark chrome.
fn ring_point(painter: &egui::Painter, centre: egui::Pos2) {
    painter.circle_stroke(centre, 20.0, egui::Stroke::new(2.0, egui::Color32::from_black_alpha(150)));
    painter.circle_stroke(centre, 18.0, egui::Stroke::new(3.0, ACCENT));
}

/// Paint the overlay over whatever the app drew this frame.
pub fn paint(ctx: &egui::Context, note: &Annotation) {
    if note.is_empty() {
        return;
    }
    let screen = ctx.content_rect();
    let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Debug, egui::Id::new("brep-mcp-annotation")));

    // The ring, re-resolved from the LIVE publication every frame: a control
    // that moved between the annotate call and the capture is still ringed
    // where it now is.
    if !note.highlight.is_empty() {
        let rects = published_rects();
        for key in &note.highlight {
            let Some(r) = rects.get(key.as_str()) else { continue };
            // A POINT widget rather than a box: the three VIEWPORT-hosted panels
            // (`gizmo/…`, `constraint/…`, and a sheet's anchors) publish
            // ZERO-SIZE rects, because what they name is a handle in the 3D view
            // with no rectangle of its own — a gizmo arrowhead, a rotation grab,
            // an origin ball. Ringing one as a box draws a six-point square that
            // a reader cannot see, so it gets the same circle a 3D pick gets.
            if r[2] < 1.0 && r[3] < 1.0 {
                ring_point(&painter, egui::pos2(r[0], r[1]));
                continue;
            }
            let rect = egui::Rect::from_min_size(egui::pos2(r[0], r[1]), egui::vec2(r[2], r[3])).expand(3.0);
            // A dark hairline outside the amber so the ring survives on a light
            // 3D background as well as on the dark chrome.
            painter.rect_stroke(
                rect.expand(2.0),
                6.0,
                egui::Stroke::new(2.0, egui::Color32::from_black_alpha(150)),
                egui::StrokeKind::Inside,
            );
            painter.rect_stroke(rect, 5.0, egui::Stroke::new(3.0, ACCENT), egui::StrokeKind::Outside);
        }
    }

    // A pick in the 3D view: a ring where the click will land.
    for p in &note.points {
        ring_point(&painter, egui::pos2(p[0], p[1]));
    }

    if note.title.is_none() && note.caption.is_none() && note.step.is_none() {
        return;
    }

    // The band is laid out from the text up, so a long caption wraps and the
    // band grows rather than the caption being clipped.
    let counter = note.step.map(|(n, total)| format!("{n} / {total}"));
    let counter_galley = counter
        .as_ref()
        .map(|c| painter.layout_no_wrap(c.clone(), egui::FontId::proportional(TITLE_SIZE), ACCENT));
    let counter_w = counter_galley.as_ref().map(|g| g.size().x + PAD).unwrap_or(0.0);
    let text_w = (screen.width() - 2.0 * PAD - counter_w).max(80.0);

    let title_galley = note
        .title
        .as_ref()
        .map(|t| painter.layout(t.clone(), egui::FontId::proportional(TITLE_SIZE), TITLE_INK, text_w));
    let caption_galley = note
        .caption
        .as_ref()
        .map(|c| painter.layout(c.clone(), egui::FontId::proportional(CAPTION_SIZE), CAPTION_INK, text_w));

    let title_h = title_galley.as_ref().map(|g| g.size().y + 4.0).unwrap_or(0.0);
    let caption_h = caption_galley.as_ref().map(|g| g.size().y).unwrap_or(0.0);
    let band_h = (PAD + title_h + caption_h + PAD).max(48.0);
    let band = egui::Rect::from_min_max(
        egui::pos2(screen.min.x, screen.max.y - band_h),
        egui::pos2(screen.max.x, screen.max.y),
    );
    painter.rect_filled(band, 0.0, BAND);
    painter.line_segment(
        [band.left_top(), band.right_top()],
        egui::Stroke::new(2.0, ACCENT),
    );

    let mut y = band.min.y + PAD;
    if let Some(g) = title_galley {
        let h = g.size().y;
        painter.galley(egui::pos2(band.min.x + PAD, y), g, TITLE_INK);
        y += h + 4.0;
    }
    if let Some(g) = caption_galley {
        painter.galley(egui::pos2(band.min.x + PAD, y), g, CAPTION_INK);
    }
    if let Some(g) = counter_galley {
        let size = g.size();
        painter.galley(
            egui::pos2(band.max.x - PAD - size.x, band.min.y + PAD),
            g,
            ACCENT,
        );
    }
}

/// `BrepApp` with the overlay painted over it.
///
/// The harness's state type, so `harness.state()` reaches the app through
/// [`Annotated::app`]. The wrapper exists because egui shapes can only be added
/// while a pass is open: painting between `harness.step()` calls would be
/// cleared by the next `begin_pass`, so the only place the overlay can go is
/// inside an `update`.
pub struct Annotated {
    app: brep_app::app::BrepApp,
    pub note: Annotation,
}

impl Annotated {
    pub fn new(app: brep_app::app::BrepApp) -> Self {
        Self { app, note: Annotation::default() }
    }

    pub fn app(&self) -> &brep_app::app::BrepApp {
        &self.app
    }
}

impl eframe::App for Annotated {
    /// Phase 1 of the app's automation frame. `BrepApp` implements it and the
    /// runner calls it before `ui`, so a wrapper that did not forward it would
    /// silently swallow every queued click.
    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        eframe::App::raw_input_hook(&mut self.app, ctx, raw_input);
    }

    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        eframe::App::logic(&mut self.app, ctx, frame);
    }

    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        eframe::App::ui(&mut self.app, ui, frame);
        paint(ui.ctx(), &self.note);
    }
}

