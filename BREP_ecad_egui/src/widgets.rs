//! Shared egui form widgets.
use std::collections::HashMap;

/// ONE dropdown for an enumeration — the form's enum field, for a panel's own
/// state as much as for a schema field (the owner's rule, 2026-10-27: an
/// enumeration is a dropdown, never a row of radio or segmented buttons).
/// `choices` are `(value, id, label)`: the value written to `current` when the
/// entry is chosen, the id the entry's hit key carries, and the text shown.
/// The closed combo shows the chosen value's label (nothing when `current`
/// matches no choice) and is as wide as `width` says (egui's own when `None`,
/// for a combo that shares its row). `hits`, when given, gets the combo's rect
/// under `key` and each OPEN entry's rect under `key#id`, the shape every
/// script clicks a form's enum with (`click_widget key`, then `key#id`).
/// Returns whether the choice changed and the combo's response.
pub fn enum_dropdown<T: PartialEq + Clone>(
    ui: &mut egui::Ui,
    key: &str,
    choices: &[(T, &str, &str)],
    current: &mut T,
    width: Option<f32>,
    mut hits: Option<&mut HashMap<String, egui::Rect>>,
) -> (bool, egui::Response) {
    let shown = choices
        .iter()
        .find(|(value, _, _)| value == current)
        .map_or_else(String::new, |(_, _, label)| label.to_string());
    let mut combo = egui::ComboBox::from_id_salt(("form-enum", key));
    if let Some(width) = width {
        combo = combo.width(width);
    }
    let mut changed = false;
    let combo = combo.selected_text(shown).show_ui(ui, |ui| {
        for (value, id, label) in choices {
            let item = ui.selectable_value(current, value.clone(), *label);
            changed |= item.changed();
            if let Some(map) = hits.as_deref_mut() {
                map.insert(format!("{key}#{id}"), item.rect);
            }
        }
    });
    if let Some(map) = hits.as_deref_mut() {
        map.insert(key.to_string(), combo.response.rect);
    }
    (changed, combo.response)
}
