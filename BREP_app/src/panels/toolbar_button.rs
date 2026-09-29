//! The SINGLE source of truth for toolbar-button styling — shared by the main
//! toolbar (`toolbar.rs`), the context bar (`context_bar.rs`), and the 2D sketcher
//! toolbar (`sketch.rs`). Every toolbar button goes through here, so sizing/style
//! changes land in ONE place. The human label is in the hover TOOLTIP, and every
//! button is SQUARE (min-width == height).
//!
//! Artwork is ALWAYS the catalogued SVG ([`crate::icons`]) — colour icons paint
//! their own colours, monochrome ones are tinted to the button's live text
//! colour. There is no font path: this app ships no icon font, and a glyph
//! character is only ever a KEY into the catalog.
//!
//! There used to be two other routes. Several private-use font glyphs painted at
//! one origin in different colours was the only way to get multi-colour artwork
//! out of a monochrome TrueType face; that went when each became a single colour
//! SVG. Then monochrome glyphs were left to `BrepIcons.ttf` on the reasoning that
//! a rasterised image "would buy nothing" — true in isolation, but it was the
//! last thing keeping a whole font in the binary, so it went too.
use eframe::egui;

/// The catalogued artwork for a button glyph — what lets a plain
/// `button`/`button_enabled`/`toggle` call site render its icon with no per-site
/// plumbing, so the toolbar, the workbench registry and the sketch tool row all
/// light up from the glyph alone. Shared with the tree and palette rows so all
/// three agree on what is an icon.
use crate::icons::artwork;

/// Square edge length of a single-glyph toolbar button (min-width == height).
pub const TOOLBAR_BTN: f32 = 28.0;

/// Shared flow layout for toolbar controls in every workbench. Individual
/// controls wrap at the available width; nested non-wrapping groups would hide
/// their buttons on narrow windows. The parent panel grows to contain each row.
pub fn wrapped<R>(ui: &mut egui::Ui, contents: impl FnOnce(&mut egui::Ui) -> R) -> egui::InnerResponse<R> {
    ui.horizontal_wrapped(contents)
}

/// Maximum painted edge length, independent of legacy SVG canvas padding.
const GLYPH_SIZE: f32 = 17.0;

fn square() -> egui::Vec2 {
    egui::vec2(TOOLBAR_BTN, TOOLBAR_BTN)
}

/// The button for one glyph: its catalogued artwork if it has any, else the
/// characters themselves as text.
///
/// Monochrome artwork is white in the catalog, so `image_tint_follows_text_color`
/// multiplies it by the button's LIVE text colour — which is what keeps hover,
/// pressed and disabled states looking exactly as they did when the font drew
/// the glyph. Colour artwork must never be tinted, so it opts out.
fn glyph_button<'a>(ui: &egui::Ui, glyph: &'a str) -> egui::Button<'a> {
    match artwork(glyph) {
        Some(icon) => {
            // Idempotent, and the only place a toolbar button needs it: a button
            // can be the first thing on screen to draw an SVG.
            egui_extras::install_image_loaders(ui.ctx());
            let height = GLYPH_SIZE / icon.artwork_aspect.max(1.0);
            let image = egui::Image::new(egui::ImageSource::Bytes {
                uri: format!("bytes://brep-toolbar/{}.svg", icon.name).into(),
                bytes: egui::load::Bytes::Static(icon.artwork_svg.as_bytes()),
            })
            .fit_to_exact_size(egui::vec2(height * icon.artwork_aspect, height));
            egui::Button::new(image)
                .image_tint_follows_text_color(icon.mono)
                .min_size(square())
        }
        None => egui::Button::new(glyph).min_size(square()),
    }
}

/// A plain action toolbar button: `glyph` shown, `tooltip` on hover. Square.
pub fn button(ui: &mut egui::Ui, glyph: &str, tooltip: &str) -> egui::Response {
    let button = glyph_button(ui, glyph);
    ui.add(button).on_hover_text(tooltip)
}

/// A toolbar button enabled only when `enabled` (undo/redo, selection actions).
pub fn button_enabled(
    ui: &mut egui::Ui,
    enabled: bool,
    glyph: &str,
    tooltip: &str,
) -> egui::Response {
    let button = glyph_button(ui, glyph);
    ui.add_enabled(enabled, button).on_hover_text(tooltip)
}

/// A toolbar TOGGLE button (draw-tool selection, wireframe): pressed when `selected`.
pub fn toggle(ui: &mut egui::Ui, selected: bool, glyph: &str, tooltip: &str) -> egui::Response {
    let button = glyph_button(ui, glyph).selected(selected);
    ui.add(button).on_hover_text(tooltip)
}

/// The outcome of a toolbar [`select`] combo for one frame.
pub struct SelectResult {
    /// The option `id` the user picked THIS frame — `Some` only on a real change
    /// (differs from `current`); `None` otherwise.
    pub changed: Option<String>,
    /// The combo HEADER's screen rect (the verifier clicks this to open the menu).
    pub header_rect: egui::Rect,
    /// `(id, rect)` per menu item while the menu is OPEN (empty when closed) — the
    /// verifier clicks an item rect to drive a SELECTION.
    pub item_rects: Vec<(String, egui::Rect)>,
}

/// Painted edge of the workbench icon on one switcher row.
const SELECT_ICON: f32 = 18.0;

/// Painted edge of a switcher row's trailing marker. Both markers are square
/// artwork drawn at this size, so the header and every menu item reserve the
/// SAME right-hand slot and their labels sit at the same x.
const SELECT_MARKER: f32 = 12.0;

/// The switcher header's "there is a menu here" chevron (U+25BE) and the tick
/// beside the ACTIVE workbench (U+2713). Both are catalogued artwork, like every
/// other picture in this module: nothing here renders a character as text, so no
/// font has to have it. They used to be `right_text("▾")` / `right_text("✓")`,
/// which drew correctly only where the OS monospace happened to carry them —
/// on wasm, where `fonts::install` falls back to egui's four bundled faces,
/// NEITHER codepoint is covered (`Ubuntu-Light`/`NotoEmoji`/`emoji-icon-font`
/// have no U+25BE, and nothing bundled has U+2713 at all), so both rendered as
/// a tofu box in the browser build.
const SELECT_CHEVRON: &str = "\u{25BE}";
const SELECT_TICK: &str = "\u{2713}";

/// One switcher row: the workbench's own icon, its label, and a right-aligned
/// SLOT for the trailing marker.
///
/// The slot is an empty custom atom, and the marker is painted into the rect it
/// reports, because [`egui::Button::image_tint_follows_text_color`] is a
/// per-BUTTON switch: the workbench icons carry their own colours and must not
/// be tinted, so a monochrome marker sharing the button could not be tinted
/// either. Painting it separately also gets it the row's LIVE text colour —
/// including the selected row's, which is the one place a tick ever appears.
fn switcher_row(label: &str, glyph: &str, marker_slot: egui::Id) -> egui::Button<'static> {
    let icon = artwork(glyph).expect("workbench icon must be catalogued");
    let image = egui::Image::new(egui::ImageSource::Bytes {
        uri: icon.uri.into(),
        bytes: egui::load::Bytes::Static(icon.svg.as_bytes()),
    })
    .fit_to_exact_size(egui::Vec2::splat(SELECT_ICON));
    egui::Button::new((
        image,
        label.to_owned(),
        egui::Atom::grow(),
        egui::Atom::custom(marker_slot, egui::Vec2::splat(SELECT_MARKER)),
    ))
    .image_tint_follows_text_color(icon.mono)
    .wrap_mode(egui::TextWrapMode::Extend)
}

/// Draw one switcher row `width` wide and paint `marker` (if any) into its slot.
fn switcher_row_ui(
    ui: &mut egui::Ui,
    label: &str,
    glyph: &str,
    width: f32,
    selected: bool,
    marker: Option<&str>,
) -> egui::Response {
    let slot = ui.id().with(("switcher-marker", label));
    let out = switcher_row(label, glyph, slot)
        .selected(selected)
        .min_size(egui::vec2(width, TOOLBAR_BTN))
        .atom_ui(ui);
    if let (Some(marker), Some(rect)) = (marker, out.rect(slot)) {
        let icon = artwork(marker).expect("switcher marker must be catalogued");
        let tint = ui.style().interact_selectable(&out.response, selected).text_color();
        egui::Image::new(egui::ImageSource::Bytes {
            uri: icon.uri.into(),
            bytes: egui::load::Bytes::Static(icon.svg.as_bytes()),
        })
        .tint(tint)
        .paint_at(ui, rect);
    }
    out.response
}

/// The width the switcher is drawn at: the NATURAL width of its widest option,
/// so the longest label fits exactly and nothing carries dead space.
///
/// egui measures it, in a throwaway invisible sizing-pass `Ui`, rather than this
/// module re-deriving egui's button padding and atom-gap arithmetic — a copy of
/// that arithmetic would go quietly loose or tight the next time either changes.
/// Every row is measured with its marker slot present, so the header's chevron
/// and an item's tick are both accounted for whichever row is widest.
fn switcher_width(ui: &egui::Ui, options: &[(&str, &str, &str)]) -> f32 {
    let mut probe = egui::Ui::new(
        ui.ctx().clone(),
        ui.id().with("switcher-measure"),
        egui::UiBuilder::new()
            .sizing_pass()
            .invisible()
            .style(ui.style().clone())
            .layer_id(ui.layer_id())
            // Far OFF SCREEN, deliberately. The probe still registers its rows
            // as widgets in this layer, and at the origin those six rects would
            // land on the toolbar, the workbench strip and the top of the dock —
            // over widgets drawn BEFORE the switcher, which the switcher's own
            // click cannot mask. Layout is arithmetic and `fit_to_exact_size` is
            // deterministic, so the measurement is the same wherever it happens.
            .max_rect(egui::Rect::from_min_size(
                egui::pos2(1.0e5, 1.0e5),
                egui::Vec2::splat(1.0e4),
            )),
    );
    options.iter().fold(0.0_f32, |widest, (label, _, glyph)| {
        let slot = probe.id().with(("switcher-marker", *label));
        let row = switcher_row(label, glyph, slot).min_size(egui::vec2(0.0, TOOLBAR_BTN));
        widest.max(row.atom_ui(&mut probe).response.rect.width())
    })
}

/// An icon-and-label toolbar dropdown, as tall as the surrounding buttons and
/// exactly as wide as its widest option needs.
/// Options are `(label, id, glyph)` entries from the workbench registry.
pub fn select(
    ui: &mut egui::Ui,
    id_source: &str,
    current: &str,
    options: &[(&str, &str, &str)],
) -> SelectResult {
    egui_extras::install_image_loaders(ui.ctx());
    let width = switcher_width(ui, options);
    let mut changed = None;
    let mut item_rects = Vec::new();
    let header = ui
        .push_id(("toolbar-select", id_source), |ui| {
            let (label, _, glyph) = options
                .iter()
                .find(|(_, id, _)| *id == current)
                .expect("selected workbench must be registered");
            let response =
                switcher_row_ui(ui, label, glyph, width, false, Some(SELECT_CHEVRON))
                    .on_hover_text("Switch workbench");
            egui::Popup::menu(&response).width(width).show(|ui| {
                for (label, id, glyph) in options {
                    let selected = *id == current;
                    let marker = selected.then_some(SELECT_TICK);
                    let resp = switcher_row_ui(ui, label, glyph, width, selected, marker);
                    item_rects.push(((*id).to_string(), resp.rect));
                    if resp.clicked() {
                        if !selected {
                            changed = Some((*id).to_string());
                        }
                        ui.close();
                    }
                }
            });
            response
        })
        .inner;
    SelectResult {
        changed,
        header_rect: header.rect,
        item_rects,
    }
}

/// One MENU row: a toolbar button's own artwork and its human label, as a
/// selectable row inside a popup. This is what a toolbar menu entry
/// looks like — the same glyph it draws in the row, plus the label that is only
/// a tooltip while it is on the strip.
///
/// `selected` is the entry's live PRESSED state, so menus and toolbar buttons
/// communicate their current state with the same selected fill.
///
/// No marker slot, unlike [`switcher_row`]: a menu of toolbar buttons has no
/// "which one is active" tick to reserve room for — several can be pressed at
/// once (the three display toggles), and the pressed FILL says so.
fn menu_row_button(glyph: &str, label: &str) -> egui::Button<'static> {
    let icon = artwork(glyph).expect("a toolbar button's glyph must be catalogued");
    // The CROPPED artwork at the strip's own painted size and URI, exactly as
    // [`glyph_button`] draws it — not the switcher's padded `svg`, which paints
    // the same glyph visibly smaller. Menu and toolbar artwork share a size.
    let height = GLYPH_SIZE / icon.artwork_aspect.max(1.0);
    let image = egui::Image::new(egui::ImageSource::Bytes {
        uri: format!("bytes://brep-toolbar/{}.svg", icon.name).into(),
        bytes: egui::load::Bytes::Static(icon.artwork_svg.as_bytes()),
    })
    .fit_to_exact_size(egui::vec2(height * icon.artwork_aspect, height));
    egui::Button::new((image, label.to_owned()))
        .image_tint_follows_text_color(icon.mono)
        .wrap_mode(egui::TextWrapMode::Extend)
}

/// Draw one overflow-menu row `width` wide. Returns its response, so the caller
/// publishes the hit-rect and reads the click.
pub fn menu_row(
    ui: &mut egui::Ui,
    glyph: &str,
    label: &str,
    selected: bool,
    width: f32,
) -> egui::Response {
    ui.add(
        menu_row_button(glyph, label)
            .selected(selected)
            .min_size(egui::vec2(width, TOOLBAR_BTN)),
    )
}

/// The width an overflow menu of `rows` — `(glyph, label)` pairs — is drawn at:
/// the NATURAL width of its widest row, measured by egui in a throwaway
/// invisible sizing pass, exactly the way [`switcher_width`] measures the
/// switcher. Re-deriving egui's button padding and atom-gap arithmetic here
/// would go quietly loose or tight the next time either changes.
pub fn menu_width(ui: &egui::Ui, rows: &[(&str, &str)]) -> f32 {
    let mut probe = egui::Ui::new(
        ui.ctx().clone(),
        ui.id().with("menu-measure"),
        egui::UiBuilder::new()
            .sizing_pass()
            .invisible()
            .style(ui.style().clone())
            .layer_id(ui.layer_id())
            // Far OFF SCREEN, for the reason `switcher_width` explains: the
            // probe still registers its rows as widgets in this layer, and at
            // the origin they would land on the toolbar it is measuring FOR.
            .max_rect(egui::Rect::from_min_size(
                egui::pos2(1.0e5, 1.0e5),
                egui::Vec2::splat(1.0e4),
            )),
    );
    rows.iter().fold(0.0_f32, |widest, (glyph, label)| {
        let row = menu_row_button(glyph, label).min_size(egui::vec2(0.0, TOOLBAR_BTN));
        widest.max(probe.add(row).rect.width())
    })
}

