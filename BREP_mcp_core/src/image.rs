//! Screenshot post-processing: PNG encode/decode, crop, downscale, diff, a
//! drawn cursor, and the animated-GIF encoder the docs walkthroughs are
//! written with. Pure CPU, `image` crate, no fonts — the annotation a
//! walkthrough carries (caption, highlight ring) is painted by the HOST, in
//! egui, before the frame is captured, because there is no font here to
//! rasterise with and nothing in the repo to load one from
//! (`BREP_mcp/src/headless.rs`, `annotate`).
pub use image::RgbaImage;
use image::{imageops, ImageBuffer, Rgba};

/// A rectangle in pixels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PxRect {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
}

pub fn decode_png(bytes: &[u8]) -> Result<RgbaImage, String> {
    image::load_from_memory_with_format(bytes, image::ImageFormat::Png)
        .map(|i| i.to_rgba8())
        .map_err(|e| format!("png decode: {e}"))
}

pub fn encode_png(img: &RgbaImage) -> Result<Vec<u8>, String> {
    let mut out = std::io::Cursor::new(Vec::new());
    img.write_to(&mut out, image::ImageFormat::Png)
        .map_err(|e| format!("png encode: {e}"))?;
    Ok(out.into_inner())
}

/// Crop to `rect`, clamped to the image.
pub fn crop(img: &RgbaImage, rect: PxRect) -> RgbaImage {
    let x = rect.x.min(img.width().saturating_sub(1));
    let y = rect.y.min(img.height().saturating_sub(1));
    let w = rect.w.min(img.width() - x).max(1);
    let h = rect.h.min(img.height() - y).max(1);
    imageops::crop_imm(img, x, y, w, h).to_image()
}

/// Downscale so the width fits `max_width` (never upscales).
pub fn scale_to_width(img: &RgbaImage, max_width: u32) -> RgbaImage {
    if img.width() <= max_width || max_width == 0 {
        return img.clone();
    }
    let h = (img.height() as f64 * max_width as f64 / img.width() as f64).round().max(1.0) as u32;
    imageops::resize(img, max_width, h, imageops::FilterType::Triangle)
}

/// Fraction of differing pixels (any channel differs by more than `threshold`)
/// and the bounding box of the difference, after scaling `b` to `a`'s size.
pub struct Diff {
    pub ratio: f64,
    pub bbox: Option<PxRect>,
    pub mask: RgbaImage,
}

pub fn diff(a: &RgbaImage, b: &RgbaImage, threshold: u8) -> Diff {
    let b = if b.dimensions() != a.dimensions() {
        imageops::resize(b, a.width(), a.height(), imageops::FilterType::Triangle)
    } else {
        b.clone()
    };
    let (w, h) = a.dimensions();
    let mut mask: RgbaImage = ImageBuffer::from_pixel(w, h, Rgba([0, 0, 0, 255]));
    let mut count = 0u64;
    let (mut x0, mut y0, mut x1, mut y1) = (u32::MAX, u32::MAX, 0u32, 0u32);
    for y in 0..h {
        for x in 0..w {
            let pa = a.get_pixel(x, y).0;
            let pb = b.get_pixel(x, y).0;
            let differs = (0..3).any(|c| pa[c].abs_diff(pb[c]) > threshold);
            if differs {
                count += 1;
                mask.put_pixel(x, y, Rgba([255, 0, 255, 255]));
                x0 = x0.min(x);
                y0 = y0.min(y);
                x1 = x1.max(x);
                y1 = y1.max(y);
            }
        }
    }
    let bbox = (count > 0).then(|| PxRect { x: x0, y: y0, w: x1 - x0 + 1, h: y1 - y0 + 1 });
    Diff { ratio: count as f64 / (w as f64 * h as f64), bbox, mask }
}

/// Draw a simple arrow cursor with its tip at (x, y): white fill, black
/// outline, so it reads on any background.
///
/// `scale` multiplies the 12 px shape, each cell becoming a `scale`-square
/// block. It is drawn at FULL resolution, before a capture's downscale, so a
/// walkthrough frame that is then squeezed from 1400 px to 1000 px needs a
/// bigger pointer to stay followable — 1x lands at nine pixels tall, which
/// reads as a speck. Ordinary screenshots pass 1 and are unchanged.
pub fn draw_cursor(img: &mut RgbaImage, x: i64, y: i64, pressed: bool, scale: u32) {
    const SHAPE: [&str; 12] = [
        "X...........",
        "XX..........",
        "XOX.........",
        "XOOX........",
        "XOOOX.......",
        "XOOOOX......",
        "XOOOOOX.....",
        "XOOOOOOX....",
        "XOOOOOOOX...",
        "XOOOOXXXXX..",
        "XOOXOX......",
        "XX..XOX.....",
    ];
    let fill = if pressed { Rgba([255, 220, 0, 255]) } else { Rgba([255, 255, 255, 255]) };
    let scale = scale.max(1) as i64;
    for (dy, row) in SHAPE.iter().enumerate() {
        for (dx, ch) in row.chars().enumerate() {
            let colour = match ch {
                'X' => Rgba([0, 0, 0, 255]),
                'O' => fill,
                _ => continue,
            };
            for sy in 0..scale {
                for sx in 0..scale {
                    let px = x + dx as i64 * scale + sx;
                    let py = y + dy as i64 * scale + sy;
                    if px < 0 || py < 0 || px >= img.width() as i64 || py >= img.height() as i64 {
                        continue;
                    }
                    img.put_pixel(px as u32, py as u32, colour);
                }
            }
        }
    }
}

/// One frame of an animated GIF: the picture and how long it is HELD, in
/// milliseconds. GIF stores the delay in hundredths of a second, so a value is
/// rounded to the nearest 10 ms when it is written.
pub struct GifFrame {
    pub image: RgbaImage,
    pub hold_ms: u32,
}

/// Encode an animated GIF that loops forever.
///
/// Every frame must be the same size: GIF's logical screen is one rectangle and
/// a frame that does not fill it would be composited over its neighbour rather
/// than replacing it, which is a silent visual bug, so a mismatch is an error
/// here instead.
///
/// `speed` is the `image`/`gif` quantiser knob, 1..=30 — 1 weighs quality over
/// time, 30 the reverse. Each frame gets its OWN 256-colour palette (NeuQuant),
/// which is what makes a shaded 3D view acceptable at all and also what makes
/// large flat regions shimmer slightly between frames.
pub fn encode_gif(frames: &[GifFrame], speed: i32) -> Result<Vec<u8>, String> {
    let first = frames.first().ok_or("an animated GIF needs at least one frame")?;
    let (w, h) = first.image.dimensions();
    for (i, f) in frames.iter().enumerate() {
        if f.image.dimensions() != (w, h) {
            return Err(format!(
                "frame {i} is {}x{} but frame 0 is {w}x{h}: every GIF frame must be the same size",
                f.image.width(),
                f.image.height()
            ));
        }
    }
    let mut out = Vec::new();
    {
        let mut encoder = image::codecs::gif::GifEncoder::new_with_speed(&mut out, speed.clamp(1, 30));
        encoder
            .set_repeat(image::codecs::gif::Repeat::Infinite)
            .map_err(|e| format!("gif repeat: {e}"))?;
        for f in frames {
            // GIF's unit is a hundredth of a second. Round rather than truncate,
            // and never emit 0 — a zero delay is "as fast as the viewer likes",
            // which is not a hold.
            let cs = ((f.hold_ms as f64) / 10.0).round().max(1.0) as u32;
            let delay = image::Delay::from_numer_denom_ms(cs * 10, 1);
            encoder
                .encode_frame(image::Frame::from_parts(f.image.clone(), 0, 0, delay))
                .map_err(|e| format!("gif frame: {e}"))?;
        }
    }
    Ok(out)
}

/// Is the image (nearly) a single flat colour? A black frame from a failed
/// render trips this; a rendered app never does.
pub fn is_flat(img: &RgbaImage) -> bool {
    let Some(first) = img.pixels().next() else { return true };
    img.pixels().all(|p| p.0[..3] == first.0[..3])
}

