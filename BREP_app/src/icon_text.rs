//! [`IconText`] — a label that draws catalogued characters as real SVG images,
//! inline with the surrounding text.
//!
//! # Why
//!
//! The app used to draw its icons as characters, from an icon font. A TrueType
//! glyph can only be ONE colour, so multi-colour artwork meant stacking several
//! private-use glyphs at one origin and painting each a different colour. This
//! widget removed that ceiling — a character is drawn from its SVG source
//! ([`crate::icons`]), so an icon carries as many colours as its artwork does —
//! and with every icon site drawing this way, the font itself is gone. An icon
//! character is now only ever a KEY into the catalog.
//!
//! # Drop-in
//!
//! [`IconTextUi::icon_label`] takes exactly what [`egui::Ui::label`] takes and
//! returns exactly what it returns, so converting a call site is a one-token
//! edit — `ui.label(x)` becomes `ui.icon_label(x)`. Text with no catalogued
//! character takes a fast path that IS a plain `Label`, so converting a site
//! that never shows an icon changes nothing about how it lays out.
//!
//! Styling survives the split. The text is resolved to one [`egui::text::LayoutJob`]
//! first, then sliced at icon characters, so every run keeps the exact
//! [`egui::TextFormat`] it was given — `.strong()`, `.weak()`, `.small()`,
//! `.color(…)`, an explicit `.font(…)`, all of it. Each icon is sized from the
//! font of the run it sits in and tinted with that run's colour, so an icon in a
//! `.small()` label is small and an icon in a red label is red.
//!
//! # Layout
//!
//! Segments are emitted into a wrapping horizontal layout, so a line of text
//! with icons in it wraps like ordinary text rather than overflowing. egui's
//! `Label` cooperates: in a wrapped horizontal layout it starts on the current
//! row after the previous widget and continues below.
//!
//! # Image loader
//!
//! Drawing an SVG needs `egui_extras`' image loader on the context. This widget
//! installs it itself, the first time it actually has an icon to draw, rather
//! than making every shell remember to — a host that forgot would show blank
//! gaps where icons should be, with no error. `install_image_loaders` skips
//! loaders already present, so the call is idempotent, and the no-icon fast
//! path never reaches it.

use crate::icons;
use eframe::egui::{
    self,
    text::{ByteIndex, LayoutJob, LayoutSection},
    Align, FontSelection, Response, Sense, TextWrapMode, Widget, WidgetText,
};

/// The `Image` for a catalogued icon, sized to `height` points and its own
/// aspect. Shared by every site that draws artwork instead of a font glyph —
/// this widget, toolbar buttons, tree rows and palette rows — so one icon is the
/// same size and comes from the same texture wherever it appears.
///
/// The image is NOT tinted: monochrome artwork is white in the catalog and needs
/// the caller's text colour multiplied in, which only the caller knows. Colour
/// artwork must never be tinted at all.
pub fn image(icon: &'static icons::Icon, height: f32) -> egui::Image<'static> {
    let size = egui::vec2(height * icon.aspect, height);
    egui::Image::new(egui::ImageSource::Bytes {
        uri: icon.uri.into(),
        bytes: egui::load::Bytes::Static(icon.svg.as_bytes()),
    })
    .fit_to_exact_size(size)
}

/// A [`egui::Button`] whose label's leading catalogued glyph is drawn as
/// artwork instead of as a character — the button counterpart of
/// [`IconTextUi::icon_label`], for the small `✎` / `✕` / `▶` buttons scattered
/// through the panels.
///
/// Monochrome artwork is tinted to the button's LIVE text colour, so hover,
/// pressed and disabled states look exactly as they did when a font drew the
/// glyph. Colour artwork is never tinted. A label with no leading catalogued
/// glyph comes back as a plain text button, so converting a call site that
/// turns out to have no icon changes nothing.
///
/// Build it before calling `ui.add`, since it borrows the `Ui` to install the
/// image loader:
///
/// ```ignore
/// let button = icon_text::icon_button(ui, "✎").small();
/// if ui.add(button).clicked() { … }
/// ```
pub fn icon_button<'a>(ui: &egui::Ui, label: &'a str) -> egui::Button<'a> {
    icon_button_colored(ui, label, None)
}

/// The same, in an explicit colour: `color` paints BOTH the artwork and the
/// text, for a destructive action that has to read red. `None` leaves the
/// button its ambient colours, which is what lets monochrome artwork follow
/// hover and disabled state.
pub fn icon_button_colored<'a>(
    ui: &egui::Ui,
    label: &'a str,
    color: Option<egui::Color32>,
) -> egui::Button<'a> {
    let Some((icon, rest)) = split_leading(label) else {
        return match color {
            Some(c) => egui::Button::new(egui::RichText::new(label).color(c)),
            None => egui::Button::new(label),
        };
    };
    egui_extras::install_image_loaders(ui.ctx());
    let mut art = image(icon, ui.text_style_height(&egui::TextStyle::Body));
    // Colour artwork carries its own colours and is never recoloured; only
    // monochrome artwork takes the caller's.
    if let (Some(c), true) = (color, icon.mono) {
        art = art.tint(c);
    }
    let button = match (rest.is_empty(), color) {
        (true, _) => egui::Button::new(art),
        (false, Some(c)) => egui::Button::new((art, egui::RichText::new(rest).color(c))),
        (false, None) => egui::Button::new((art, rest)),
    };
    // An explicit colour is already applied above; otherwise let the tint follow
    // the widget's live text colour so hover/pressed/disabled still read.
    button.image_tint_follows_text_color(icon.mono && color.is_none())
}

/// A label's leading catalogued icon and the text after it, or `None` when it
/// does not start with one.
fn split_leading(label: &str) -> Option<(&'static icons::Icon, &str)> {
    let mut chars = label.chars();
    let icon = icons::lookup(chars.next()?)?;
    Some((icon, chars.as_str().trim_start()))
}

/// A `selectable_label` whose leading catalogued glyph is drawn as artwork —
/// for the list rows that lead with an icon (the add-feature palette, the file
/// browser). Falls back to a plain `selectable_label` when there is no leading
/// icon, so converting a row that turns out to have none changes nothing.
pub fn selectable_icon_label(ui: &mut egui::Ui, selected: bool, label: &str) -> Response {
    let Some((icon, rest)) = split_leading(label) else {
        return ui.selectable_label(selected, label);
    };
    egui_extras::install_image_loaders(ui.ctx());
    let art = image(icon, ui.text_style_height(&egui::TextStyle::Body));
    ui.add(
        egui::Button::selectable(selected, (art, rest))
            .image_tint_follows_text_color(icon.mono),
    )
}

/// ONE catalogued glyph drawn on its own, in `color` — for a status badge or
/// any other place a bare glyph was previously a coloured `RichText`.
/// `None` when the string is not a single catalogued character.
pub fn glyph(ui: &egui::Ui, glyph: &str, color: egui::Color32) -> Option<egui::Image<'static>> {
    let icon = icons::artwork(glyph)?;
    egui_extras::install_image_loaders(ui.ctx());
    let art = image(icon, ui.text_style_height(&egui::TextStyle::Body));
    // Colour artwork carries its own colours and must not be tinted; monochrome
    // artwork is white in the catalog, so the multiply lands it on `color`.
    Some(if icon.mono { art.tint(color) } else { art })
}

/// The same split, for a caller that must MEASURE a caption the way
/// [`icon_button`] will draw it: the leading icon (if any) and the remaining
/// text. A caption with no leading icon comes back whole.
pub fn split_caption(label: &str) -> (Option<&'static icons::Icon>, &str) {
    match split_leading(label) {
        Some((icon, rest)) => (Some(icon), rest),
        None => (None, label),
    }
}

/// One piece of a split label: either a run of text or a catalogued icon.
enum Segment {
    /// A text run, already carrying its own formatting.
    Text(LayoutJob),
    /// A catalogued icon, plus the format of the run it appeared in — that is
    /// what sizes and colours it.
    Icon(&'static icons::Icon, egui::TextFormat),
}

/// The settings shared by every `Label` a split produces. Held apart from the
/// text so [`IconText::show`] can consume the text into a job and still pass
/// these down.
#[derive(Clone, Copy, Default)]
struct Opts {
    wrap_mode: Option<TextWrapMode>,
    sense: Option<Sense>,
    selectable: Option<bool>,
}

/// A label whose catalogued characters are drawn as SVG images inline with the
/// text. See the module docs; construct with [`IconText::new`] or, at an
/// existing `ui.label` call site, [`IconTextUi::icon_label`].
#[must_use = "widgets do nothing unless you add them to a Ui"]
pub struct IconText {
    text: WidgetText,
    opts: Opts,
}

impl IconText {
    /// Take anything [`egui::Ui::label`] takes.
    pub fn new(text: impl Into<WidgetText>) -> Self {
        Self { text: text.into(), opts: Opts::default() }
    }

    /// Override the wrap mode (default: the `Ui`'s).
    #[inline]
    pub fn wrap_mode(mut self, wrap_mode: TextWrapMode) -> Self {
        self.opts.wrap_mode = Some(wrap_mode);
        self
    }

    /// Wrap long text onto the next line.
    #[inline]
    pub fn wrap(self) -> Self {
        self.wrap_mode(TextWrapMode::Wrap)
    }

    /// Truncate long text with an ellipsis instead of wrapping.
    #[inline]
    pub fn truncate(self) -> Self {
        self.wrap_mode(TextWrapMode::Truncate)
    }

    /// Let the text extend past the available width, growing the parent `Ui`.
    #[inline]
    pub fn extend(self) -> Self {
        self.wrap_mode(TextWrapMode::Extend)
    }

    /// Make the label respond to clicks and/or drags, as [`egui::Label::sense`].
    #[inline]
    pub fn sense(mut self, sense: Sense) -> Self {
        self.opts.sense = Some(sense);
        self
    }

    /// Whether the text can be selected with the mouse, as [`egui::Label::selectable`].
    #[inline]
    pub fn selectable(mut self, selectable: bool) -> Self {
        self.opts.selectable = Some(selectable);
        self
    }

    /// Add to `ui`, returning the union of every segment's response.
    pub fn show(self, ui: &mut egui::Ui) -> Response {
        let Self { text, opts } = self;
        // Resolve styling ONCE, into the same job `Label` would have built.
        // Slicing this (rather than the original string) is what preserves
        // per-run formatting through the split.
        let job = std::sync::Arc::unwrap_or_clone(text.into_layout_job(
            ui.style(),
            FontSelection::Default,
            ui.text_valign(),
        ));

        let segments = split(&job);

        // Fast path: nothing to draw as an image, so BE a plain `Label` —
        // same layout, same response, no nested `Ui`. Most converted call
        // sites take this path, which is what makes the conversion safe to
        // apply wholesale.
        if segments.len() == 1 {
            if let Some(Segment::Text(_)) = segments.first() {
                return opts.label(job).ui(ui);
            }
        }

        // Icons present, so we are about to draw images: make sure the loader
        // that decodes them is on this context. Idempotent — see the module docs.
        egui_extras::install_image_loaders(ui.ctx());

        // Icons present: lay the pieces out as one wrapping line of text. If we
        // are already inside a wrapping horizontal layout, join it rather than
        // nesting — nesting would restart wrapping at the full width and
        // overflow the row we were handed.
        if ui.layout().is_horizontal() && ui.layout().main_wrap() {
            opts.emit(ui, segments)
        } else {
            ui.horizontal_wrapped(|ui| opts.emit(ui, segments)).inner
        }
    }
}

impl Opts {
    /// Draw the segments into `ui`, which is a wrapping horizontal layout.
    fn emit(&self, ui: &mut egui::Ui, segments: Vec<Segment>) -> Response {
        // Text runs must butt up against their icons with no gap, or every icon
        // would sit in a word-space of its own. Restore the caller's spacing
        // afterwards so our next sibling is still separated normally.
        let spacing = ui.spacing().item_spacing;
        ui.spacing_mut().item_spacing.x = 0.0;

        let mut response: Option<Response> = None;
        let mut union = |acc: &mut Option<Response>, r: Response| {
            *acc = Some(match acc.take() {
                Some(prev) => prev | r,
                None => r,
            });
        };

        for segment in segments {
            match segment {
                Segment::Text(job) => union(&mut response, self.label(job).ui(ui)),
                Segment::Icon(icon, format) => {
                    union(&mut response, self.icon(ui, icon, &format));
                }
            }
        }

        ui.spacing_mut().item_spacing = spacing;
        // An empty string yields no segments; hand back an inert zero-size
        // allocation so callers always get a Response, as `ui.label("")` does.
        response.unwrap_or_else(|| ui.allocate_response(egui::Vec2::ZERO, Sense::hover()))
    }

    /// Draw one icon, sized and coloured from the text run it belongs to.
    fn icon(&self, ui: &mut egui::Ui, icon: &'static icons::Icon, format: &egui::TextFormat) -> Response {
        // Match the line box of the run's own font, so the image occupies the
        // space a glyph of that font would have, so the icon lands on the text
        // baseline at text size.
        let height = ui.fonts_mut(|f| f.row_height(&format.font_id));
        let size = egui::vec2(height * icon.aspect, height);

        let mut image = image(icon, height);

        // Monochrome artwork is white in the catalog, so multiplying by the text
        // colour lands it exactly on that colour. Artwork with authored colours
        // is never tinted — that would wash it out.
        if icon.mono {
            image = image.tint(resolve(ui, format.color));
        }

        let response = ui.add_sized(size, image);

        // Put the character itself over the image, invisibly: the icon then
        // still selects, copies and reads out as text, exactly as it does when
        // the font draws it.
        ui.put(
            response.rect,
            egui::Label::new(
                egui::RichText::new(icon.ch)
                    .font(format.font_id.clone())
                    .color(egui::Color32::TRANSPARENT),
            )
            .selectable(self.selectable.unwrap_or(false)),
        ) | response
    }

    /// A `Label` for one text run, carrying the shared settings.
    fn label(&self, job: LayoutJob) -> egui::Label {
        let mut label = egui::Label::new(job);
        if let Some(wrap_mode) = self.wrap_mode {
            label = label.wrap_mode(wrap_mode);
        }
        if let Some(sense) = self.sense {
            label = label.sense(sense);
        }
        if let Some(selectable) = self.selectable {
            label = label.selectable(selectable);
        }
        label
    }
}

impl Widget for IconText {
    fn ui(self, ui: &mut egui::Ui) -> Response {
        self.show(ui)
    }
}

/// [`egui::Color32::PLACEHOLDER`] means "whatever the style says" — `Label`
/// substitutes the real colour when it paints. We tint before that happens, so
/// we have to do the same substitution ourselves; tinting with the placeholder
/// would paint the sentinel colour.
fn resolve(ui: &egui::Ui, color: egui::Color32) -> egui::Color32 {
    if color == egui::Color32::PLACEHOLDER {
        ui.visuals().text_color()
    } else {
        color
    }
}

/// Split a laid-out job at every catalogued character.
///
/// Returns a single `Text` segment when nothing is catalogued, which is the
/// signal for the plain-`Label` fast path.
fn split(job: &LayoutJob) -> Vec<Segment> {
    let mut segments = Vec::new();
    let mut run_start = 0;

    for (at, ch) in job.text.char_indices() {
        let Some(icon) = icons::lookup(ch) else { continue };
        if run_start < at {
            segments.push(Segment::Text(slice(job, run_start, at)));
        }
        segments.push(Segment::Icon(icon, format_at(job, at)));
        run_start = at + ch.len_utf8();
    }

    if segments.is_empty() {
        // No catalogued character anywhere: hand back the job exactly as it
        // came, so the fast path in `show` is a byte-for-byte plain `Label`.
        return vec![Segment::Text(job.clone())];
    }
    if run_start < job.text.len() {
        segments.push(Segment::Text(slice(job, run_start, job.text.len())));
    }
    segments
}

/// The format covering byte `at` — the styling of the run an icon sits in.
fn format_at(job: &LayoutJob, at: usize) -> egui::TextFormat {
    // `format_at_byte` panics on a section-less job; one with text always has
    // sections, but an icon-only job built by hand might not.
    if job.sections.is_empty() {
        return egui::TextFormat::default();
    }
    job.format_at_byte(ByteIndex(at)).clone()
}

/// A sub-job over `job[start..end]`, keeping each overlapping section's format.
fn slice(job: &LayoutJob, start: usize, end: usize) -> LayoutJob {
    // Clone so every scalar setting (halign, justify, break_on_newline, …)
    // carries over, then replace the parts that are per-slice.
    let mut out = job.clone();
    out.text = job.text[start..end].to_owned();
    out.sections = job
        .sections
        .iter()
        .filter_map(|s| {
            let lo = s.byte_range.start.0.max(start);
            let hi = s.byte_range.end.0.min(end);
            (lo < hi).then(|| LayoutSection {
                // Leading space belongs to the section's true start; a section
                // we cut into mid-way must not re-apply it.
                leading_space: if s.byte_range.start.0 >= start { s.leading_space } else { 0.0 },
                byte_range: ByteIndex(lo - start)..ByteIndex(hi - start),
                format: s.format.clone(),
            })
        })
        .collect();
    // `Label` rebuilds these per row it lays out.
    out.first_row_min_height = 0.0;
    out.halign = Align::LEFT;
    out
}

/// `ui.label(…)` → `ui.icon_label(…)`: the drop-in seam. Implemented for
/// [`egui::Ui`], so it is in scope wherever this trait is imported.
pub trait IconTextUi {
    /// As [`egui::Ui::label`], but catalogued characters are drawn as SVG icons.
    fn icon_label(&mut self, text: impl Into<WidgetText>) -> Response;
}

impl IconTextUi for egui::Ui {
    fn icon_label(&mut self, text: impl Into<WidgetText>) -> Response {
        IconText::new(text).show(self)
    }
}


