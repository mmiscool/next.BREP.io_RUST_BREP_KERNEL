//! The active PMI view's annotation graphics: dimension lines, extension
//! lines, arcs, leaders and arrowheads baked into the `pmi-overlay` group
//! from the kernel's presentation layout (the SAME layout the STEP file
//! writes), sized in pixels at bake time and re-baked when the zoom or the
//! viewing direction moves materially; plus the label feed the app's chip
//! pass draws.

use super::*;
use brep_kernel::{pmi_present, pmi_type, PmiLayoutStyle, PmiStatus};

const PMI_OVERLAY_GROUP: &str = "pmi-overlay";
/// Arrowhead length, in pixels.
pub(crate) const ARROW_PX: f64 = 12.0;
/// The reference text cap height at 12 pt, in pixels (frames scale with the
/// view's text size).
pub(crate) const TEXT_PX_AT_12PT: f64 = 11.0;
/// PMI line / arrow colour (amber, distinct from the dimension gizmo's rods
/// and the constraint status colours).
const PMI_RGB: [f32; 3] = [0.98, 0.78, 0.22];
/// An error annotation's chip colour.
const ERROR_RGB: [f32; 3] = [0.97, 0.32, 0.29];
/// A disabled annotation's chip colour.
const DISABLED_RGB: [f32; 3] = [0.55, 0.55, 0.55];
/// The error a balloon shows while its target is hidden in the viewport and
/// the view's hidden list has not caught up (no run has happened).
pub(crate) const LIVE_HIDDEN_MESSAGE: &str = "target hidden in the viewport: show it, or update the view's visibility";

impl EngineState {
    /// Re-bake the overlay from the cached report (empty without an active
    /// view or while a sketch edit is live).
    pub fn refresh_pmi_overlay(&mut self) {
        let Some(active) = self.pmi_active_view.clone() else {
            self.clear_pmi_overlay();
            return;
        };
        if self.sketch_mode() {
            self.clear_pmi_overlay();
            return;
        }
        let Some(view) = self.pmi_report.as_ref().and_then(|report| report.view(&active)).cloned() else {
            self.clear_pmi_overlay();
            return;
        };
        let text_size = self
            .pmi_state()
            .find_view(&active)
            .map(|view| view.display.text_size_pt)
            .unwrap_or(12.0);
        let wpp = self.camera.world_per_pixel();
        let (_, up, view_dir) = self.camera.basis();
        let style = PmiLayoutStyle {
            arrow: ARROW_PX * wpp,
            text_height: TEXT_PX_AT_12PT * wpp * (text_size / 12.0),
            view_dir,
            view_up: up,
            plane: None,
        };
        let mut lines: Vec<f32> = Vec::new();
        let mut line_colors: Vec<f32> = Vec::new();
        let mut tris: Vec<f32> = Vec::new();
        let mut tri_colors: Vec<f32> = Vec::new();
        for row in &view.annotations {
            if !row.enabled || row.status != PmiStatus::Ok || self.balloon_target_hidden_live(row) {
                continue;
            }
            // A picked annotation plane lays the row out in that plane.
            let row_style = PmiLayoutStyle {
                plane: row.plane,
                ..style
            };
            let drawn = pmi_present(&row.geometry, row.label_world, &row.text, &row_style);
            for polyline in &drawn.polylines {
                for pair in polyline.windows(2) {
                    for point in pair {
                        lines.extend(point.iter().map(|c| *c as f32));
                        line_colors.extend_from_slice(&PMI_RGB);
                    }
                }
            }
            for arrow in &drawn.arrows {
                for point in arrow {
                    tris.extend(point.iter().map(|c| *c as f32));
                    tri_colors.extend_from_slice(&PMI_RGB);
                }
                // Both windings, so the arrowhead reads from either side.
                for point in [arrow[0], arrow[2], arrow[1]] {
                    tris.extend(point.iter().map(|c| *c as f32));
                    tri_colors.extend_from_slice(&PMI_RGB);
                }
            }
        }
        let group = if lines.is_empty() && tris.is_empty() {
            serde_json::json!({ "name": PMI_OVERLAY_GROUP })
        } else {
            serde_json::json!({
                "name": PMI_OVERLAY_GROUP,
                "renderOrder": 10001,
                "lines": { "positions": lines, "colors": line_colors },
                "tris": { "positions": tris, "colors": tri_colors },
            })
        };
        let _ = self.set_overlay_json(&serde_json::json!({ "groups": [group] }).to_string());
        self.pmi_overlay_key = Some((if wpp > 0.0 { wpp } else { f64::MIN_POSITIVE }, view_dir));
        self.dirty = true;
    }

    /// Whether a balloon row's target is hidden in the LIVE scene: every
    /// solid its recipe projects onto is invisible now. The kernel resolves
    /// against the view's stored hidden list; a hide made in the viewport
    /// after the run is only known here, so the leader is withheld and the
    /// chip reads as an error until the view's visibility is updated.
    pub(crate) fn balloon_target_hidden_live(&self, row: &brep_kernel::PmiAnnotationReport) -> bool {
        match &row.geometry {
            brep_kernel::PmiGeometry::Leader { balloon: true, anchor: Some(anchor), .. } => {
                !anchor.solids.is_empty()
                    && anchor
                        .solids
                        .iter()
                        .all(|name| self.scene.solid(name).is_none_or(|solid| !solid.visible))
            }
            _ => false,
        }
    }

    fn clear_pmi_overlay(&mut self) {
        if self.pmi_overlay_key.take().is_some() {
            let _ = self.set_overlay_json(
                &serde_json::json!({ "groups": [ { "name": PMI_OVERLAY_GROUP } ] }).to_string(),
            );
            self.dirty = true;
        }
    }

    /// Per-frame upkeep: re-bake when the zoom (>0.5 %) or the viewing
    /// direction (>0.5°) moved, so arrowheads stay pixel-sized and face the
    /// camera.
    pub fn ensure_pmi_overlay_current(&mut self) {
        let Some((baked_wpp, baked_dir)) = self.pmi_overlay_key else {
            if self.pmi_active_view.is_some() {
                self.refresh_pmi_overlay();
            }
            return;
        };
        if self.sketch_mode() {
            self.clear_pmi_overlay();
            return;
        }
        let wpp = self.camera.world_per_pixel();
        let (_, _, view_dir) = self.camera.basis();
        let dot = baked_dir[0] * view_dir[0] + baked_dir[1] * view_dir[1] + baked_dir[2] * view_dir[2];
        if super::overlay_wpp_stale(baked_wpp, wpp) || dot < (0.5f64).to_radians().cos() {
            self.refresh_pmi_overlay();
        }
    }

    /// The label feed for the app's chip pass — the active view's
    /// annotations: `[{id, type, icon, text, status, message, color:[r,g,b],
    /// world:[x,y,z], enabled, open, textSizePt}]`. Empty without an active
    /// view.
    pub fn pmi_labels_json(&self) -> String {
        let Some(active) = self.pmi_active_view.as_deref() else {
            return "[]".into();
        };
        let Some(view) = self.pmi_report.as_ref().and_then(|report| report.view(active)) else {
            return "[]".into();
        };
        let text_size = self
            .pmi_state()
            .find_view(active)
            .map(|view| view.display.text_size_pt)
            .unwrap_or(12.0);
        let rows: Vec<serde_json::Value> = view
            .annotations
            .iter()
            .map(|row| {
                // A balloon whose target was hidden in the viewport AFTER
                // the run (the view's hidden list not yet updated) is an
                // error chip with no leader: the kernel's head is on a body
                // nobody sees.
                let (status, message) = if row.status == PmiStatus::Ok && self.balloon_target_hidden_live(row) {
                    (PmiStatus::Error, LIVE_HIDDEN_MESSAGE.to_string())
                } else {
                    (row.status, row.message.clone())
                };
                let color = if status == PmiStatus::Error {
                    ERROR_RGB
                } else if !row.enabled {
                    DISABLED_RGB
                } else {
                    PMI_RGB
                };
                let icon = pmi_type(&row.kind).map(|def| def.icon).unwrap_or("");
                let text = if status == PmiStatus::Error {
                    format!("{} {}", icon, row.id)
                } else if row.text.is_empty() {
                    format!("{} {}", icon, row.id)
                } else {
                    row.text.clone()
                };
                serde_json::json!({
                    "id": row.id,
                    "type": row.kind,
                    "icon": icon,
                    "text": text,
                    "status": match status { PmiStatus::Ok => "ok", PmiStatus::Error => "error" },
                    "message": message,
                    "color": [color[0], color[1], color[2]],
                    "world": row.label_world,
                    "enabled": row.enabled,
                    "open": self.pmi_open_annotation.as_deref() == Some(row.id.as_str()),
                    "selected": self.pmi_selected_annotation.as_deref() == Some(row.id.as_str()),
                    "textSizePt": text_size,
                })
            })
            .collect();
        serde_json::Value::Array(rows).to_string()
    }
}
