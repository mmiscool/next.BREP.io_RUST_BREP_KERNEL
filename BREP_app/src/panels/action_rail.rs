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
