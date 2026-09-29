//! Font setup — the OS's default monospace font for all text.
//!
//! This repo ships NO font file. It used to ship one, `BrepIcons.ttf`, a
//! monoline icon face that led egui's fallback chain and supplied every
//! specialized UI glyph. Every one of those glyphs is now drawn from its SVG
//! instead ([`crate::icons`] + [`crate::icon_text`]), which is what a
//! multi-colour icon needs anyway — a TrueType glyph can only be one colour —
//! so the face had nothing left to draw and went. An icon character is now only
//! ever a KEY into the icon catalog, never something a font renders.
//!
//! The BINARY still carries four faces, and that is not ours to drop: eframe's
//! `default_fonts` feature is enabled unconditionally in Cargo.toml and pulls
//! `epaint_default_fonts`, whose whole contents are four `include_bytes!`
//! constants — Ubuntu-Light, NotoEmoji-Regular, Hack-Regular and
//! emoji-icon-font. `include_bytes!` runs at compile time, so those four are
//! linked into every native binary and wasm module whether or not the fallback
//! below ever draws with them. That is a licence obligation, not a detail: two
//! of them are conjoined OFL-1.1 / Ubuntu-font-1.0, so a binary release must
//! carry their notices. See THIRD-PARTY-NOTICES.md.
//!
//! The TEXT a user reads comes from none of the four on native — it is the
//! operating system's default monospace font, located and read at runtime via
//! fontconfig, so the UI reads with the user's own system monospace.
//!
//! Platform reality: only native can read OS font files. On wasm (browser) —
//! and on native if the OS monospace can't be located — we fall back to egui's
//! bundled default fonts (eframe feature `default_fonts`) for text + emoji.
//!
//! Installed at app creation from each shell entry point (`lib.rs` wasm `start`,
//! `main.rs` native `run_native`).

use eframe::egui;

/// Install fonts into `ctx`. Call once, at app creation.
pub fn install(ctx: &egui::Context) {
    // Base text = OS monospace when we can read it; otherwise egui's bundled
    // defaults (wasm, or a native machine without fontconfig).
    let fonts = match os_monospace() {
        Some((bytes, index)) => os_mono_defs(bytes, index),
        None => egui::FontDefinitions::default(),
    };
    ctx.set_fonts(fonts);
}

/// A minimal `FontDefinitions` whose ONLY text font is the OS monospace — no
/// bundled fallbacks.
fn os_mono_defs(bytes: Vec<u8>, index: u32) -> egui::FontDefinitions {
    let mut fonts = egui::FontDefinitions::empty();
    let mut data = egui::FontData::from_owned(bytes);
    data.index = index; // honor a .ttc collection face index from fontconfig
    fonts.font_data.insert("os_mono".to_owned(), data.into());
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts.families.insert(family, vec!["os_mono".to_owned()]);
    }
    fonts
}

/// Locate and read the OS default monospace font (path + collection index).
/// Native only, via fontconfig's `fc-match`; returns `None` on any failure so
/// the caller falls back to egui's bundled defaults.
#[cfg(not(target_arch = "wasm32"))]
fn os_monospace() -> Option<(Vec<u8>, u32)> {
    let out = std::process::Command::new("fc-match")
        .args(["--format=%{file}\n%{index}", "monospace"])
        .output()
        .ok()?;
    if !out.status.success() {
        return None;
    }
    let text = String::from_utf8_lossy(&out.stdout);
    let mut lines = text.lines();
    let path = lines.next()?.trim();
    if path.is_empty() {
        return None;
    }
    let index = lines.next().and_then(|l| l.trim().parse().ok()).unwrap_or(0);
    let bytes = std::fs::read(path).ok()?; // read failure => None => default fallback
    Some((bytes, index))
}

/// wasm has no OS font access — always fall back to egui's bundled defaults.
#[cfg(target_arch = "wasm32")]
fn os_monospace() -> Option<(Vec<u8>, u32)> {
    None
}
