//! Serialization shared by widget hit-rectangle publishers.

use eframe::egui;
use serde_json::{Map, Value};

/// Publish rectangles as `[left, top, width, height]` in egui points.
/// Iteration order is preserved; when keys repeat, the last rectangle wins.
pub(crate) fn hits_json<'a>(
    hits: impl IntoIterator<Item = (&'a String, &'a egui::Rect)>,
) -> String {
    let map: Map<String, Value> = hits
        .into_iter()
        .map(|(key, rect)| {
            (
                key.clone(),
                serde_json::json!([rect.min.x, rect.min.y, rect.width(), rect.height()]),
            )
        })
        .collect();
    Value::Object(map).to_string()
}

