//! Text authoring and embedded font outlines. Only the frame is solver geometry.
use super::{SketchDoc, SketchGeometry};
use serde_json::{json, Value};
use ttf_parser::OutlineBuilder;

pub const FONTS: &[&str] = &["Ubuntu Light", "Hack Regular"];

#[derive(Default)]
struct Outline {
    contours: Vec<Vec<[f64; 2]>>,
    current: Vec<[f64; 2]>,
    offset: f64,
}
impl Outline {
    fn p(&self, x: f32, y: f32) -> [f64; 2] {
        [x as f64 + self.offset, y as f64]
    }
}
impl OutlineBuilder for Outline {
    fn move_to(&mut self, x: f32, y: f32) {
        self.current = vec![self.p(x, y)];
    }
    fn line_to(&mut self, x: f32, y: f32) {
        let a = *self.current.last().unwrap();
        let b = self.p(x, y);
        if a == b {
            return;
        }
        self.current.extend([
            [a[0] + (b[0] - a[0]) / 3., a[1] + (b[1] - a[1]) / 3.],
            [
                a[0] + 2. * (b[0] - a[0]) / 3.,
                a[1] + 2. * (b[1] - a[1]) / 3.,
            ],
            b,
        ]);
    }
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let a = *self.current.last().unwrap();
        let q = self.p(x1, y1);
        let b = self.p(x, y);
        self.current.extend([
            [
                a[0] + 2. * (q[0] - a[0]) / 3.,
                a[1] + 2. * (q[1] - a[1]) / 3.,
            ],
            [
                b[0] + 2. * (q[0] - b[0]) / 3.,
                b[1] + 2. * (q[1] - b[1]) / 3.,
            ],
            b,
        ]);
    }
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        self.current
            .extend([self.p(x1, y1), self.p(x2, y2), self.p(x, y)]);
    }
    fn close(&mut self) {
        let a = self.current[0];
        self.line_to((a[0] - self.offset) as f32, a[1] as f32);
        if self.current.len() >= 4 {
            self.contours.push(std::mem::take(&mut self.current));
        }
    }
}

pub fn outlines(text: &str, font: &str) -> Result<Value, String> {
    if text.trim().is_empty() {
        return Err("Enter text containing visible characters".into());
    }
    if text.chars().count() > 256 {
        return Err("Sketch text is limited to 256 characters".into());
    }
    if text.chars().any(char::is_control) {
        return Err("Sketch text must be a single line".into());
    }
    let bytes = match font {
        "Ubuntu Light" => epaint_default_fonts::UBUNTU_LIGHT,
        "Hack Regular" => epaint_default_fonts::HACK_REGULAR,
        _ => return Err("Unknown sketch font".into()),
    };
    let face = ttf_parser::Face::parse(bytes, 0).map_err(|e| format!("Font: {e:?}"))?;
    let mut builder = Outline::default();
    for ch in text.chars() {
        let glyph = face
            .glyph_index(ch)
            .ok_or_else(|| format!("{font} has no glyph for {ch}"))?;
        face.outline_glyph(glyph, &mut builder);
        builder.offset += face.glyph_hor_advance(glyph).unwrap_or(0) as f64;
    }
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in builder.contours.iter().flatten() {
        for i in 0..2 {
            lo[i] = lo[i].min(p[i]);
            hi[i] = hi[i].max(p[i]);
        }
    }
    if (0..2).any(|i| !lo[i].is_finite() || hi[i] <= lo[i]) {
        return Err("Text has no usable outlines".into());
    }
    for p in builder.contours.iter_mut().flatten() {
        for i in 0..2 {
            p[i] = (p[i] - lo[i]) / (hi[i] - lo[i]);
        }
    }
    Ok(json!(builder.contours))
}

pub fn expanded(doc: &SketchDoc) -> Result<SketchDoc, String> {
    let value = serde_json::to_value(doc).map_err(|e| e.to_string())?;
    serde_json::from_value(brep_kernel::feature_pipeline::expand_sketch_text(&value)?)
        .map_err(|e| e.to_string())
}

pub fn object(
    id: Value,
    corners: Vec<Value>,
    text: &str,
    font: &str,
) -> Result<SketchGeometry, String> {
    Ok(SketchGeometry { id, geom_type: "text".into(), points:corners, extra: serde_json::from_value(json!({"construction":true,"text":text,"font":font,"textContours":outlines(text,font)?})).unwrap() })
}

