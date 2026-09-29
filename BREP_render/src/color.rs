//! Stable per-solid display colors + color-space helpers.
//!
//! `solid_color_srgb` is a byte-faithful port of the retired artifact renderer's
//! `solidColor`: FNV-1a over the name's UTF-16 code
//! units, hue = ((hash >>> 8) % 360)/360, HSL(hue, 0.45, 0.6). Keeping the hash
//! EXACT means re-baselined artifacts keep the colors users already know.

/// FNV-1a over UTF-16 code units with wrapping 32-bit multiplies — byte-compatible
/// with the original implementation.
fn fnv1a_utf16(name: &str) -> u32 {
    let mut hash: u32 = 2166136261;
    for unit in name.encode_utf16() {
        hash ^= unit as u32;
        hash = hash.wrapping_mul(16777619);
    }
    hash
}

/// HSL → sRGB, the exact formula the retired soft raster used.
fn hsl_to_rgb(h: f64, s: f64, l: f64) -> [f64; 3] {
    let q = if l < 0.5 { l * (1.0 + s) } else { l + s - l * s };
    let p = 2.0 * l - q;
    let channel = |t: f64| -> f64 {
        let t = t.rem_euclid(1.0);
        if t < 1.0 / 6.0 {
            p + (q - p) * 6.0 * t
        } else if t < 0.5 {
            q
        } else if t < 2.0 / 3.0 {
            p + (q - p) * (2.0 / 3.0 - t) * 6.0
        } else {
            p
        }
    };
    [channel(h + 1.0 / 3.0), channel(h), channel(h - 1.0 / 3.0)]
}

/// The stable per-solid display color, in sRGB (0..1 per channel).
pub fn solid_color_srgb(name: &str) -> [f64; 3] {
    let hue = ((fnv1a_utf16(name) >> 8) % 360) as f64 / 360.0;
    hsl_to_rgb(hue, 0.45, 0.6)
}

/// sRGB electro-optical transfer (decode): sRGB component → linear.
pub fn srgb_to_linear(c: f64) -> f64 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

/// A 0xRRGGBB hex color → linear-space RGB (for light/material constants).
pub fn hex_to_linear(hex: u32) -> [f32; 3] {
    hex_to_srgb(hex).map(|channel| srgb_to_linear(channel) as f32)
}

/// A 0xRRGGBB hex color → sRGB-space RGB in 0..1 (for clear colors written to
/// a non-sRGB target, where the bytes should equal the hex exactly).
pub fn hex_to_srgb(hex: u32) -> [f64; 3] {
    [
        ((hex >> 16) & 0xff) as f64 / 255.0,
        ((hex >> 8) & 0xff) as f64 / 255.0,
        (hex & 0xff) as f64 / 255.0,
    ]
}

/// A packed RGB color normalized for single-precision overlay and material buffers.
pub(crate) fn hex_to_srgb_f32(hex: u32) -> [f32; 3] {
    hex_to_srgb(hex).map(|channel| channel as f32)
}

