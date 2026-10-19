//! action_rail — the SHARED right-side, single-column, uniform-width action
//! toolbar. It is the ONE renderer behind BOTH the modeling context bar
//! ([`super::context_bar`]) and the in-sketch context bar
//! ([`super::sketch`]), so the two look and behave identically: change the
//! layout here and both update. It draws INTO a caller-owned `ui` (the caller
//! owns the top-right `Area`/`Frame` and any stacking), measures every caption
//! up front to size the column to the WIDEST button, and stretches every button
//! to that shared width (100%).

use std::collections::HashMap;

/// One button on an action rail.
pub struct ActionItem {
    /// Stable key returned when clicked (and used for the verifier hit-rect).
    pub key: String,
    /// The caption drawn on the button.
    pub label: String,
    /// Hover tooltip.
    pub tooltip: String,
    /// Greyed-out + unclickable when false.
    pub enabled: bool,
}

impl ActionItem {
    pub fn new(key: impl Into<String>, label: impl Into<String>, tooltip: impl Into<String>) -> Self {
        Self {
            key: key.into(),
            label: label.into(),
            tooltip: tooltip.into(),
            enabled: true,
        }
    }

    pub fn enabled(mut self, enabled: bool) -> Self {
        self.enabled = enabled;
        self
    }
}

/// Draw a titled single-column toolbar of uniform-width buttons into `ui`.
/// Returns the clicked item's key (if any). Records each button's rect into
/// `hits` (keyed by item key) for the headed verifier. The card's width is the
/// widest button; the optional `title`/`subtitle` wrap within that width.
pub fn action_rail(
    ui: &mut egui::Ui,
    title: Option<&str>,
    subtitle: Option<&str>,
    items: &[ActionItem],
    hits: &mut HashMap<String, egui::Rect>,
) -> Option<String> {
    if crate::viewport::touch::compact(ui.ctx()) {
        return mobile_chooser(ui, title, items, hits);
    }
    if items.is_empty() {
        return None;
    }

    // Width = the widest button caption (+ button padding). Measured via the
    // Button text style; titles are allowed to wrap within it.
    let font = egui::TextStyle::Button.resolve(ui.style());
    let measure = |ui: &egui::Ui, text: &str| -> f32 {
        ui.ctx().fonts_mut(|f| {
            f.layout_no_wrap(text.to_owned(), font.clone(), egui::Color32::PLACEHOLDER)
                .size()
                .x
        })
    };
    // A caption's LEADING glyph is drawn as artwork, not as a character, so it
    // is measured as a square of the text height rather than as text — the
    // column must be sized to what is actually drawn. Without an icon font a
    // character measured here would also be a box.
    // Body, not Button: `icon_text::icon_button` sizes the artwork off Body, and
    // the column must be measured at the size it is actually drawn.
    let icon_w = ui.text_style_height(&egui::TextStyle::Body);
    let mut widest = 0.0f32;
    for item in items {
        let (icon, rest) = crate::icon_text::split_caption(&item.label);
        let text_w = measure(ui, rest);
        widest = widest.max(if icon.is_some() {
            text_w + icon_w + ui.spacing().item_spacing.x
        } else {
            text_w
        });
    }
    let col_w = widest + 2.0 * ui.spacing().button_padding.x;

    ui.set_width(col_w.max(1.0));
    if let Some(title) = title {
        ui.label(egui::RichText::new(title).strong());
    }
    if let Some(subtitle) = subtitle {
        ui.label(egui::RichText::new(subtitle).weak().small());
    }
    ui.add_space(2.0);

    let full = ui.available_width();
    let h = ui.spacing().interact_size.y;
    let mut clicked = None;
    for item in items {
        let resp = ui
            .add_enabled_ui(item.enabled, |ui| {
                let button = crate::icon_text::icon_button(ui, &item.label);
                ui.add_sized([full, h], button)
            })
            .inner
            .on_hover_text(&item.tooltip)
            // egui's plain hover tooltip is for ENABLED widgets only; a disabled
            // item's tooltip is the whole explanation of why it is greyed out.
            .on_disabled_hover_text(&item.tooltip);
        hits.insert(item.key.clone(), resp.rect);
        if resp.clicked() {
            clicked = Some(item.key.clone());
        }
    }
    clicked
}

// The mobile entry uses the SAME palette as Add feature and returns the same
// ActionItem key to the existing caller/dispatcher. Desktop rails stay inline.
const MOBILE_PALETTE: &str = "brep-mobile-actions-palette";
const MOBILE_REQUEST: &str = "brep-mobile-actions-request";
const MOBILE_CHOICE: &str = "brep-mobile-actions-choice";

pub(crate) fn open_mobile_actions(ctx: &egui::Context) {
    ctx.data_mut(|d| d.insert_temp(egui::Id::new(MOBILE_REQUEST), true));
}

pub(crate) fn close_mobile_actions(ctx: &egui::Context) {
    ctx.data_mut(|d| {
        let mut palette = d.get_temp::<crate::palette::Palette>(egui::Id::new(MOBILE_PALETTE)).unwrap_or_default();
        palette.close();
        d.remove::<String>(egui::Id::new(MOBILE_CHOICE));
        d.insert_temp(egui::Id::new(MOBILE_PALETTE), palette);
        d.insert_temp(egui::Id::new(MOBILE_REQUEST), false);
    });
}

fn mobile_chooser(ui: &mut egui::Ui, title: Option<&str>, items: &[ActionItem], hits: &mut HashMap<String, egui::Rect>) -> Option<String> {
    let ctx = ui.ctx().clone();
    let (mut palette, requested) = ctx.data_mut(|d| {
        let palette = d.get_temp::<crate::palette::Palette>(egui::Id::new(MOBILE_PALETTE)).unwrap_or_default();
        let requested = d.get_temp::<bool>(egui::Id::new(MOBILE_REQUEST)).unwrap_or(false);
        d.insert_temp(egui::Id::new(MOBILE_REQUEST), false);
        (palette, requested)
    });
    let choices = || items.iter().map(|item| crate::palette::PaletteItem::new(&item.key, &item.label, vec![item.tooltip.clone()])).collect::<Vec<_>>();
    if requested { palette.open(choices(), title.unwrap_or("Selection actions"), "Search actions"); }
    let chosen = ctx.data_mut(|d| d.remove_temp::<String>(egui::Id::new(MOBILE_CHOICE)))
        .filter(|key| items.iter().any(|item| item.enabled && item.key == *key));
    if items.is_empty() { palette.close(); }
    else if palette.is_open() {
        let disabled = items.iter().filter(|item| !item.enabled).map(|item| (item.key.clone(), item.tooltip.clone())).collect();
        palette.refresh_items(choices(), disabled);
        for (key, rect) in palette.hits() {
            if let Some(key) = key.strip_prefix("item:") { hits.insert(key.to_owned(), *rect); }
        }
    }
    ctx.data_mut(|d| d.insert_temp(egui::Id::new(MOBILE_PALETTE), palette));
    chosen
}

/// Draw at app level, like the feature palette. Keeping the modal outside the
/// entry Area lets its display popup sit above the modal in egui's layer order.
pub(crate) fn show_mobile_actions(ctx: &egui::Context) {
    let mut palette = ctx.data(|d| d.get_temp::<crate::palette::Palette>(egui::Id::new(MOBILE_PALETTE))).unwrap_or_default();
    let chosen = palette.show_with_id(ctx, egui::Id::new("brep-mobile-actions-modal"));
    ctx.data_mut(|d| {
        d.insert_temp(egui::Id::new(MOBILE_PALETTE), palette);
        if let Some(chosen) = chosen { d.insert_temp(egui::Id::new(MOBILE_CHOICE), chosen); }
    });
}

#[cfg(all(feature = "automation", target_arch = "wasm32"))]
pub(crate) fn mobile_actions_json(ctx: &egui::Context, entry: egui::Rect) -> String {
    let palette = ctx.data(|d| d.get_temp::<crate::palette::Palette>(egui::Id::new(MOBILE_PALETTE))).unwrap_or_default();
    serde_json::json!({"open": [entry.left(), entry.top(), entry.width(), entry.height()], "visible": palette.is_open(),
        "rects": serde_json::from_str::<serde_json::Value>(&crate::automation::hit_rects::hits_json(palette.hits().iter())).unwrap_or_default()}).to_string()
}
