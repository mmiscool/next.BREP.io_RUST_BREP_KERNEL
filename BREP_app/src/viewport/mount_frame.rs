//! The part's MOUNT FRAME, drawn in its own editor: where the footprint's
//! frame is in the model's coordinates.
//!
//! A part's mount (`brep_kernel::part_mount`) moves it onto its footprint
//! wherever it is placed, but never in its own editor, where the features are
//! shown as built. So the editor shows the frame instead: an axis triad at the
//! footprint origin and the seating plane's outline (z = 0 of the mount,
//! round the pads), both carried into model coordinates by M⁻¹. It is an
//! overlay only, painted and never hit-tested, and it shows what a face pick
//! or a typed mount did.

use eframe::egui;

use brep_ecad_core::board::Footprint;
use brep_render::brep_kernel;
use brep_render::engine_state::EngineState;

use super::Viewport;

/// How far the seating outline stands off the pads, in millimetres.
const OUTLINE_MARGIN_MM: f64 = 0.5;

/// What the frame is drawn in: the board's x, y and z colours, and the
/// outline in the seating plane's own.
const AXIS_COLORS: [egui::Color32; 3] = [
    egui::Color32::from_rgb(230, 80, 80),
    egui::Color32::from_rgb(90, 200, 90),
    egui::Color32::from_rgb(90, 140, 240),
];
const OUTLINE_COLOR: egui::Color32 = egui::Color32::from_rgb(230, 190, 60);

/// One segment of the frame in MODEL coordinates, with the colour it is
/// drawn in.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct FrameSegment {
    pub from: [f64; 3],
    pub to: [f64; 3],
    pub color: egui::Color32,
}

/// The frame of a part's `mount` and `pads` blocks, in model coordinates:
/// three axis segments from the footprint origin, then the four sides of the
/// seating outline. `None` when there is no mount to show (no block, or one
/// that does not read as a pose), or no pads to sit on.
pub(crate) fn mount_frame_segments(mount: Option<&serde_json::Value>, pads: Option<&serde_json::Value>) -> Option<Vec<FrameSegment>> {
    let mount = brep_kernel::mount_from_block(mount?).ok()?;
    let footprint = serde_json::from_value::<Footprint>(pads?.clone()).ok()?;
    if footprint.pads.is_empty() {
        return None;
    }
    let inverse = mount.rigid_inverse().ok()?;
    let model = |p: [f64; 3]| {
        let q = inverse.point(brep_kernel::Vec3::new(p[0], p[1], p[2]));
        [q.x, q.y, q.z]
    };
    // Pads are micrometres with y down; the footprint frame is millimetres
    // with y up.
    let (min, max) = footprint.bounds();
    let (x0, x1) = (min.x as f64 / 1_000.0 - OUTLINE_MARGIN_MM, max.x as f64 / 1_000.0 + OUTLINE_MARGIN_MM);
    let (y0, y1) = (-(max.y as f64) / 1_000.0 - OUTLINE_MARGIN_MM, -(min.y as f64) / 1_000.0 + OUTLINE_MARGIN_MM);
    let length = ((x1 - x0).max(y1 - y0) * 0.3).max(1.0);
    let mut out: Vec<FrameSegment> = (0..3)
        .map(|axis| {
            let mut tip = [0.0; 3];
            tip[axis] = length;
            FrameSegment { from: model([0.0; 3]), to: model(tip), color: AXIS_COLORS[axis] }
        })
        .collect();
    let corners = [[x0, y0, 0.0], [x1, y0, 0.0], [x1, y1, 0.0], [x0, y1, 0.0]];
    for i in 0..4 {
        out.push(FrameSegment { from: model(corners[i]), to: model(corners[(i + 1) % 4]), color: OUTLINE_COLOR });
    }
    Some(out)
}

impl Viewport {
    /// Paint the open part's mount frame ([`mount_frame_segments`]) over the
    /// 3D view. A segment with an end off screen is left out.
    pub(super) fn draw_mount_frame(ctx: &egui::Context, rect: egui::Rect, state: &EngineState) {
        let Some(segments) = mount_frame_segments(state.history.mount_block(), state.history.pads_block()) else {
            return;
        };
        let ends: Vec<[f64; 3]> = segments.iter().flat_map(|segment| [segment.from, segment.to]).collect();
        let screens = state.world_to_screen_points(&ends);
        if screens.len() != ends.len() {
            return;
        }
        let mut painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Middle, egui::Id::new("brep-mount-frame")));
        painter.set_clip_rect(rect);
        for (i, segment) in segments.iter().enumerate() {
            let (a, b) = (screens[2 * i], screens[2 * i + 1]);
            if a[3] < 0.5 || b[3] < 0.5 {
                continue;
            }
            let at = |s: [f64; 4]| egui::pos2(rect.min.x + s[0] as f32, rect.min.y + s[1] as f32);
            painter.line_segment([at(a), at(b)], egui::Stroke::new(2.0, segment.color));
        }
    }
}

