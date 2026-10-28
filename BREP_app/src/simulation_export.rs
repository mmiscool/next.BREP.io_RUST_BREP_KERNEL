//! A motion study as a video file: GIF (the `image` crate's encoder, every
//! frame its own palette, looping) or MP4 (H.264 through the pure-Rust
//! `rusty_h264` encoder, muxed by `mp4e`), at a chosen size and fps.
//!
//! The study is baked first if need be, then every baked frame is shown the
//! way playback shows it (poses, clips, camera) and drawn at the export size
//! through the viewport's own render core under the user's display settings
//! (`crate::viewport::Viewport::render_rgb`); the playhead goes back where
//! it stood. Native only: the web app plays studies but writes no video.

use crate::app::BrepApp;
use serde::Serialize;

/// What to export.
#[derive(Debug, Clone, PartialEq)]
pub struct ExportSpec {
    /// The study id; the active one when absent.
    pub study: Option<String>,
    /// `gif` or `mp4`.
    pub format: String,
    pub width: u32,
    pub height: u32,
    /// Frames per second; the study's when absent (every baked frame is a
    /// video frame: the fps sets how fast they play).
    pub fps: Option<u32>,
    /// Where to write the file; nothing written when absent (the bytes are
    /// returned either way).
    pub path: Option<String>,
}

/// What an export came to.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ExportReport {
    pub study: String,
    pub format: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub frames: usize,
    pub bytes: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// The file's first bytes as (lossy) text: `GIF89a`, or an MP4's `ftyp`.
    pub head: String,
}

impl ExportReport {
    /// One line for the pane's notice.
    pub fn summary(&self) -> String {
        match &self.path {
            Some(path) => format!("Wrote {} ({} frames, {}×{} at {} fps, {} bytes)", path, self.frames, self.width, self.height, self.fps, self.bytes),
            None => format!("Exported {} frames, {}×{} at {} fps, {} bytes", self.frames, self.width, self.height, self.fps, self.bytes),
        }
    }
}

/// The size limits: GIF palettes and the H.264 level both have their price.
const SIZE_RANGE: std::ops::RangeInclusive<u32> = 16..=4096;

/// Export the study: bake it, draw every frame, encode, write `path` if given.
pub fn export(app: &mut BrepApp, spec: &ExportSpec) -> Result<(ExportReport, Vec<u8>), String> {
    let format = spec.format.to_ascii_lowercase();
    if format != "gif" && format != "mp4" {
        return Err(format!("no export format '{}': gif or mp4", spec.format));
    }
    if !SIZE_RANGE.contains(&spec.width) || !SIZE_RANGE.contains(&spec.height) {
        return Err(format!("the size must be {}..={} pixels a side, not {}×{}", SIZE_RANGE.start(), SIZE_RANGE.end(), spec.width, spec.height));
    }
    if format == "mp4" && (spec.width % 2 == 1 || spec.height % 2 == 1) {
        return Err(format!("MP4 needs an even width and height, not {}×{}", spec.width, spec.height));
    }
    let engine = app.docs.engine_mut();
    let standing = engine.simulation_playback_json();
    let before = standing["time"].as_f64().unwrap_or(0.0);
    let study = engine.simulation_bake_all(spec.study.as_deref())?;
    let record = engine.simulation()?.and_then(|block| block.study(&study).cloned()).ok_or_else(|| format!("no study '{study}'"))?;
    let fps = spec.fps.unwrap_or(record.fps).max(1);
    let frames = record.frame_count();
    let (width, height) = (spec.width, spec.height);
    // The camera at the export size: the frame fit (near/far to everything
    // drawn) reads the camera's own size.
    let (w0, h0) = (engine.camera.width, engine.camera.height);
    engine.camera.width = width as f64;
    engine.camera.height = height as f64;
    let mut sink = FrameSink::new(&format, width, height, fps)?;
    let mut failure: Option<String> = None;
    for index in 0..frames {
        let time = record.frame_time(index);
        let engine = app.docs.engine_mut();
        if let Err(error) = engine.simulation_seek(Some(&study), time) {
            failure = Some(error);
            break;
        }
        let (camera, _) = engine.fit_camera_and_overlay();
        match app.viewport.render_rgb(app.docs.engine(), &camera, width, height).and_then(|rgb| sink.push(&rgb)) {
            Ok(()) => {}
            Err(error) => {
                failure = Some(format!("frame {index}: {error}"));
                break;
            }
        }
    }
    // The camera's size and the playhead go back whatever happened.
    let engine = app.docs.engine_mut();
    engine.camera.width = w0;
    engine.camera.height = h0;
    let _ = engine.simulation_seek(Some(&study), before);
    if let Some(error) = failure {
        return Err(error);
    }
    let bytes = sink.finish()?;
    let path = match &spec.path {
        Some(path) if !path.trim().is_empty() => {
            let path = std::path::PathBuf::from(path.trim());
            if let Some(parent) = path.parent().filter(|p| !p.as_os_str().is_empty()) {
                std::fs::create_dir_all(parent).map_err(|e| format!("{}: {e}", parent.display()))?;
            }
            std::fs::write(&path, &bytes).map_err(|e| format!("{}: {e}", path.display()))?;
            Some(path.display().to_string())
        }
        _ => None,
    };
    let head = String::from_utf8_lossy(&bytes[..bytes.len().min(16)]).into_owned();
    Ok((ExportReport { study, format, width, height, fps, frames, bytes: bytes.len(), path, head }, bytes))
}

/// Where the frames go: GIF frames kept until the end (the encoder owns its
/// output), or H.264 access units encoded as the frames come and muxed at the
/// end.
pub(crate) enum FrameSink {
    Gif { width: u32, height: u32, delay_ms: u32, frames: Vec<image::Frame> },
    Mp4 { width: u32, height: u32, duration_ms: u32, encoder: rusty_h264_encoder::Encoder, units: Vec<Vec<u8>> },
}

impl FrameSink {
    pub(crate) fn new(format: &str, width: u32, height: u32, fps: u32) -> Result<Self, String> {
        let ms = (1000.0 / fps as f64).round().max(1.0) as u32;
        Ok(match format {
            "gif" => FrameSink::Gif { width, height, delay_ms: ms, frames: Vec::new() },
            "mp4" => {
                let mut config = rusty_h264_encoder::EncoderConfig::new(width as usize, height as usize);
                config.framerate = fps as f32;
                config.gop_size = fps.max(1);
                // About an eighth of a bit per pixel per frame: crisp CAD edges
                // at 960×540 / 30 fps come to ~2 Mbps.
                config.bitrate = ((width as u64 * height as u64 * fps as u64) / 8).clamp(200_000, 40_000_000) as u32;
                let encoder = rusty_h264_encoder::Encoder::new(config).map_err(|e| format!("h264 encoder: {e:?}"))?;
                FrameSink::Mp4 { width, height, duration_ms: ms, encoder, units: Vec::new() }
            }
            other => return Err(format!("no export format '{other}': gif or mp4")),
        })
    }

    /// One frame, packed RGB top row first, `width`×`height`.
    pub(crate) fn push(&mut self, rgb: &[u8]) -> Result<(), String> {
        match self {
            FrameSink::Gif { width, height, delay_ms, frames } => {
                if rgb.len() != (*width * *height * 3) as usize {
                    return Err(format!("frame is {} bytes, not {}×{}×3", rgb.len(), width, height));
                }
                let mut rgba = Vec::with_capacity(rgb.len() / 3 * 4);
                for px in rgb.chunks_exact(3) {
                    rgba.extend_from_slice(&[px[0], px[1], px[2], 255]);
                }
                let image = image::RgbaImage::from_raw(*width, *height, rgba).ok_or("gif frame: size")?;
                let delay = image::Delay::from_numer_denom_ms(((*delay_ms as f64 / 10.0).round().max(1.0) * 10.0) as u32, 1);
                frames.push(image::Frame::from_parts(image, 0, 0, delay));
                Ok(())
            }
            FrameSink::Mp4 { width, height, encoder, units, .. } => {
                let frame = rgb_to_i420(rgb, *width, *height)?;
                units.push(encoder.encode(&frame));
                Ok(())
            }
        }
    }

    pub(crate) fn finish(self) -> Result<Vec<u8>, String> {
        match self {
            FrameSink::Gif { frames, .. } => {
                if frames.is_empty() {
                    return Err("nothing to export: the study has no frames".into());
                }
                let mut out = Vec::new();
                {
                    let mut encoder = image::codecs::gif::GifEncoder::new_with_speed(&mut out, 10);
                    encoder.set_repeat(image::codecs::gif::Repeat::Infinite).map_err(|e| format!("gif: {e}"))?;
                    for frame in frames {
                        encoder.encode_frame(frame).map_err(|e| format!("gif frame: {e}"))?;
                    }
                }
                Ok(out)
            }
            FrameSink::Mp4 { width, height, duration_ms, mut encoder, mut units } => {
                let tail = encoder.flush();
                if !tail.is_empty() {
                    units.push(tail);
                }
                if units.is_empty() {
                    return Err("nothing to export: the study has no frames".into());
                }
                let mut cursor = std::io::Cursor::new(Vec::new());
                {
                    let mut muxer = mp4e::Mp4e::new(&mut cursor);
                    muxer.set_video_track(width, height, mp4e::Codec::AVC);
                    for unit in &units {
                        muxer.encode_video(unit, duration_ms).map_err(|e| format!("mp4: {e:?}"))?;
                    }
                    muxer.flush().map_err(|e| format!("mp4: {e:?}"))?;
                }
                Ok(cursor.into_inner())
            }
        }
    }
}

/// Packed RGB (top row first) to a planar 4:2:0 frame (BT.601 limited range),
/// chroma averaged over each 2×2 block. Even sizes only.
pub(crate) fn rgb_to_i420(rgb: &[u8], width: u32, height: u32) -> Result<rusty_h264_common::YuvFrame, String> {
    let (w, h) = (width as usize, height as usize);
    if rgb.len() != w * h * 3 {
        return Err(format!("frame is {} bytes, not {w}×{h}×3", rgb.len()));
    }
    if w % 2 == 1 || h % 2 == 1 {
        return Err(format!("MP4 needs an even width and height, not {w}×{h}"));
    }
    let mut y = vec![0u8; w * h];
    let (cw, ch) = (w / 2, h / 2);
    let mut u = vec![0u8; cw * ch];
    let mut v = vec![0u8; cw * ch];
    let luma = |r: f64, g: f64, b: f64| (16.0 + 0.257 * r + 0.504 * g + 0.098 * b).round().clamp(16.0, 235.0) as u8;
    for row in 0..h {
        for col in 0..w {
            let p = (row * w + col) * 3;
            y[row * w + col] = luma(rgb[p] as f64, rgb[p + 1] as f64, rgb[p + 2] as f64);
        }
    }
    for row in 0..ch {
        for col in 0..cw {
            let (mut r, mut g, mut b) = (0.0, 0.0, 0.0);
            for (dy, dx) in [(0, 0), (0, 1), (1, 0), (1, 1)] {
                let p = ((row * 2 + dy) * w + col * 2 + dx) * 3;
                r += rgb[p] as f64;
                g += rgb[p + 1] as f64;
                b += rgb[p + 2] as f64;
            }
            let (r, g, b) = (r / 4.0, g / 4.0, b / 4.0);
            u[row * cw + col] = (128.0 - 0.148 * r - 0.291 * g + 0.439 * b).round().clamp(16.0, 240.0) as u8;
            v[row * cw + col] = (128.0 + 0.439 * r - 0.368 * g - 0.071 * b).round().clamp(16.0, 240.0) as u8;
        }
    }
    Ok(rusty_h264_common::YuvFrame { width: w, height: h, y, u, v })
}

