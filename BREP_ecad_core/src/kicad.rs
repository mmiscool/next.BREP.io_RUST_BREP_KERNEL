//! KiCad 6+ symbol and footprint importers with inherited and multi-unit device support.
use crate::board::{Footprint, Model, Pad, PadShape};
use crate::{Graphic, Pin, Point, Symbol};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Debug)]
enum Expr {
    Atom(String),
    List(Vec<Expr>),
}
impl Expr {
    fn atom(&self) -> &str {
        if let Self::Atom(s) = self { s } else { "" }
    }
    fn list(&self) -> &[Expr] {
        if let Self::List(v) = self { v } else { &[] }
    }
    fn head(&self) -> &str {
        self.list().first().map_or("", Self::atom)
    }
    fn arg(&self, n: usize) -> &str {
        self.list().get(n).map_or("", Self::atom)
    }
    fn child(&self, key: &str) -> Option<&Expr> {
        self.list().iter().find(|e| e.head() == key)
    }
    fn required(&self, key: &str) -> Result<&Expr, String> {
        self.child(key).ok_or_else(|| format!("Missing {key}"))
    }
    fn number(&self, n: usize) -> Result<f64, String> {
        let value: f64 = self
            .arg(n)
            .parse()
            .map_err(|_| format!("Invalid number in {}", self.head()))?;
        if value.is_finite() && value.abs() <= 100_000. {
            Ok(value)
        } else {
            Err("Number out of range".into())
        }
    }
    fn point(&self) -> Result<Point, String> {
        Ok(Point::new(
            (self.number(1)? * 1000.).round() as i32,
            (-self.number(2)? * 1000.).round() as i32,
        ))
    }
}
fn parse(text: &str) -> Result<Expr, String> {
    if text.len() > 32 * 1024 * 1024 {
        return Err("Library exceeds 32 MiB import limit".into());
    }
    let chars: Vec<_> = text.chars().collect();
    let mut i = 0;
    fn whitespace(c: &[char], i: &mut usize) {
        loop {
            while *i < c.len() && c[*i].is_whitespace() {
                *i += 1;
            }
            if c.get(*i) == Some(&';') {
                while *i < c.len() && c[*i] != '\n' {
                    *i += 1;
                }
            } else {
                break;
            }
        }
    }
    fn item(c: &[char], i: &mut usize, depth: usize) -> Result<Expr, String> {
        if depth > 64 {
            return Err("S-expression nesting exceeds 64 levels".into());
        }
        whitespace(c, i);
        match c.get(*i) {
            Some('(') => {
                *i += 1;
                let mut v = vec![];
                loop {
                    whitespace(c, i);
                    match c.get(*i) {
                        Some(')') => {
                            *i += 1;
                            return Ok(Expr::List(v));
                        }
                        None => return Err("Unclosed list".into()),
                        _ => v.push(item(c, i, depth + 1)?),
                    }
                }
            }
            Some('"') => {
                *i += 1;
                let mut s = String::new();
                while let Some(&ch) = c.get(*i) {
                    *i += 1;
                    if ch == '"' {
                        return Ok(Expr::Atom(s));
                    }
                    if ch == '\\' {
                        let escaped = *c.get(*i).ok_or("Unclosed escape")?;
                        *i += 1;
                        s.push(match escaped {
                            'n' => '\n',
                            'r' => '\r',
                            't' => '\t',
                            x => x,
                        });
                    } else {
                        s.push(ch);
                    }
                }
                Err("Unclosed string".into())
            }
            Some(')') | None => Err("Unexpected end of expression".into()),
            _ => {
                let start = *i;
                while *i < c.len() && !c[*i].is_whitespace() && c[*i] != '(' && c[*i] != ')' {
                    *i += 1;
                }
                Ok(Expr::Atom(c[start..*i].iter().collect()))
            }
        }
    }
    let root = item(&chars, &mut i, 0)?;
    whitespace(&chars, &mut i);
    if i != chars.len() {
        return Err("Trailing expressions".into());
    }
    Ok(root)
}
#[derive(Default)]
pub struct ImportReport {
    pub symbols: Vec<Symbol>,
    pub warnings: Vec<String>,
}

pub fn import_library(text: &str, nickname: &str) -> Result<ImportReport, String> {
    let root = parse(text)?;
    if root.head() != "kicad_symbol_lib" {
        return Err("Expected a KiCad .kicad_sym library".into());
    }
    let mut definitions = BTreeMap::new();
    for e in root.list().iter().filter(|e| e.head() == "symbol") {
        if e.arg(1).is_empty() || definitions.insert(e.arg(1), e).is_some() {
            return Err("Empty or duplicate symbol name".into());
        }
    }
    let mut result = ImportReport::default();
    for name in definitions.keys() {
        match resolve(name, nickname, &definitions, &mut BTreeSet::new()) {
            Ok(s) => result.symbols.push(s),
            Err(e) => result
                .warnings
                .push(format!("{nickname}:{name}: skipped — {e}")),
        }
    }
    Ok(result)
}
fn resolve(
    name: &str,
    nickname: &str,
    definitions: &BTreeMap<&str, &Expr>,
    visited: &mut BTreeSet<String>,
) -> Result<Symbol, String> {
    if visited.len() >= 32 || !visited.insert(name.into()) {
        return Err("Cyclic or overly deep inheritance".into());
    }
    let node = definitions
        .get(name)
        .ok_or_else(|| format!("Missing parent {name}"))?;
    let mut symbol = if let Some(parent) = node.child("extends") {
        resolve(parent.arg(1), nickname, definitions, visited)?
    } else {
        Symbol {
            library_id: String::new(),
            reference_prefix: "U".into(),
            description: String::new(),
            graphics: vec![],
            pins: vec![],
            unit_count: 1,
            properties: BTreeMap::new(),
            power_net: None,
            graphic_gates: vec![],
            hide_pin_names: false,
            hide_pin_numbers: false,
        }
    };
    // `(pin_names (offset 0) hide)` before KiCad 8, `(pin_names (hide yes))`
    // since; a derived symbol that says nothing keeps its parent's.
    if let Some(hide) = node.child("pin_names").and_then(hidden) {
        symbol.hide_pin_names = hide;
    }
    if let Some(hide) = node.child("pin_numbers").and_then(hidden) {
        symbol.hide_pin_numbers = hide;
    }
    symbol.library_id = format!("{nickname}:{name}");
    for prop in node.list().iter().filter(|p| p.head() == "property") {
        symbol
            .properties
            .insert(prop.arg(1).into(), prop.arg(2).into());
        match prop.arg(1) {
            "Reference" => symbol.reference_prefix = prop.arg(2).into(),
            "Description" | "ki_description" => symbol.description = prop.arg(2).into(),
            _ => {}
        }
    }
    if node.child("power").is_some() || symbol.power_net.is_some() {
        symbol.power_net = Some(
            symbol
                .properties
                .get("Value")
                .cloned()
                .unwrap_or_else(|| name.into()),
        );
    }
    let mut units: BTreeMap<u32, (Vec<Graphic>, Vec<Pin>)> = BTreeMap::new();
    for unit in node.list().iter().filter(|p| p.head() == "symbol") {
        let mut suffix = unit.arg(1).rsplit('_');
        let style: u32 = suffix
            .next()
            .unwrap_or("")
            .parse()
            .map_err(|_| "Invalid body style")?;
        let number: u32 = suffix
            .next()
            .unwrap_or("")
            .parse()
            .map_err(|_| "Invalid unit number")?;
        if style > 1 {
            continue;
        }
        let (graphics, pins) = units.entry(number).or_default();
        for item in unit.list().iter().skip(2) {
            match item.head() {
                "rectangle" => {
                    let a = item.required("start")?.point()?;
                    let b = item.required("end")?.point()?;
                    graphics.push(Graphic::Path(vec![
                        a,
                        Point::new(b.x, a.y),
                        b,
                        Point::new(a.x, b.y),
                        a,
                    ]));
                }
                "polyline" => graphics.push(Graphic::Path(
                    item.required("pts")?
                        .list()
                        .iter()
                        .skip(1)
                        .map(Expr::point)
                        .collect::<Result<Vec<_>, _>>()?,
                )),
                "circle" => graphics.push(Graphic::Circle {
                    center: item.required("center")?.point()?,
                    radius: (item.required("radius")?.number(1)? * 1000.).round() as i32,
                }),
                "arc" => graphics.push(Graphic::Path(arc_points(
                    item.required("start")?.point()?,
                    item.required("mid")?.point()?,
                    item.required("end")?.point()?,
                ))),
                "bezier" | "curve" => {
                    let p = item
                        .required("pts")?
                        .list()
                        .iter()
                        .skip(1)
                        .map(Expr::point)
                        .collect::<Result<Vec<_>, _>>()?;
                    if p.len() != 4 {
                        return Err("Bezier needs four control points".into());
                    }
                    graphics.push(Graphic::Path(
                        (0..=32)
                            .map(|i| {
                                let t = i as f64 / 32.;
                                let u = 1. - t;
                                let f = |x: bool| {
                                    p.iter()
                                        .zip([u * u * u, 3. * u * u * t, 3. * u * t * t, t * t * t])
                                        .map(|(p, w)| f64::from(if x { p.x } else { p.y }) * w)
                                        .sum::<f64>()
                                        .round() as i32
                                };
                                Point::new(f(true), f(false))
                            })
                            .collect(),
                    ));
                }
                "text" | "text_box" => {
                    let size = item
                        .child("effects")
                        .and_then(|e| e.child("font"))
                        .and_then(|f| f.child("size"))
                        .and_then(|s| s.number(1).ok())
                        .unwrap_or(1.27);
                    graphics.push(Graphic::Text {
                        at: item.required("at")?.point()?,
                        text: item.arg(1).into(),
                        size: (size * 1000.).round().max(1.) as i32,
                    });
                }
                "pin" => {
                    let at = item.required("at")?;
                    let p = at.point()?;
                    let degrees = at.number(3)?;
                    if degrees % 90. != 0. {
                        return Err("Non-orthogonal pin".into());
                    }
                    let len = (item.required("length")?.number(1)? * 1000.).round() as i32;
                    if len < 0 {
                        return Err("Negative pin length".into());
                    }
                    let angle = degrees.to_radians();
                    let direction =
                        Point::new(angle.cos().round() as i32, -angle.sin().round() as i32);
                    let end = p.offset(Point::new(direction.x * len, direction.y * len));
                    let perp = Point::new(-direction.y, direction.x);
                    let offset = |along: i32, side: i32| {
                        end.offset(Point::new(
                            direction.x * along + perp.x * side,
                            direction.y * along + perp.y * side,
                        ))
                    };
                    match item.arg(2) {
                        "line" => {}
                        "inverted" | "inverted_clock" => graphics.push(Graphic::Circle {
                            center: offset(-500, 0),
                            radius: 500,
                        }),
                        "clock" => {}
                        "input_low" | "clock_low" => graphics.push(Graphic::Path(vec![
                            offset(-1270, 0),
                            offset(0, 700),
                            end,
                        ])),
                        "output_low" => graphics.push(Graphic::Path(vec![offset(-1270, 700), end])),
                        other => return Err(format!("Unsupported pin shape {other}")),
                    }
                    if item.arg(2).contains("clock") {
                        graphics.push(Graphic::Path(vec![
                            offset(0, -700),
                            offset(1000, 0),
                            offset(0, 700),
                        ]));
                    }
                    pins.push(Pin {
                        // A KiCad unit is a GATE of one package, never a port
                        // group of its own: see `Pin::gate`.
                        unit: 0,
                        // Set below, when the unit loop places this unit's pins.
                        gate: 0,
                        number: item.required("number")?.arg(1).into(),
                        name: item.required("name")?.arg(1).into(),
                        electrical_type: item.arg(1).into(),
                        at: p,
                        end,
                        hidden: hidden(item).unwrap_or(false),
                    });
                }
                "unit_name" => {}
                other => return Err(format!("Unsupported graphic {other}")),
            }
        }
    }
    // Lay the units of a device out side by side, with shared graphics
    // replicated and each physical pin represented once. Each unit is a GATE
    // (`Pin::gate`, `Symbol::graphic_gates`): a sheet places each on its own as
    // U1A, U1B, …, and they stay one part with one footprint and one port group,
    // so `unit_count` stays 1 and no pin names a unit. A device of one unit is
    // not split at all.
    let count = units.keys().copied().max().unwrap_or(1).max(1);
    let common = units.remove(&0).unwrap_or_default();
    let mut offset = 0;
    for number in 1..=count {
        let (mut graphics, mut pins) = units.remove(&number).unwrap_or_default();
        graphics.extend(common.0.clone());
        if number == 1 {
            pins.extend(common.1.clone());
        }
        let mut xs = vec![0];
        for g in &graphics {
            match g {
                Graphic::Path(p) => xs.extend(p.iter().map(|p| p.x)),
                Graphic::Circle { center, radius } => {
                    xs.extend([center.x - radius, center.x + radius])
                }
                Graphic::Text { at, .. } => xs.push(at.x),
            }
        }
        xs.extend(pins.iter().map(|p| p.at.x));
        let width = xs.iter().max().unwrap() - xs.iter().min().unwrap();
        let shift = Point::new(offset, 0);
        for g in &mut graphics {
            match g {
                Graphic::Path(points) => {
                    for p in points {
                        *p = p.offset(shift);
                    }
                }
                Graphic::Circle { center, .. } => *center = center.offset(shift),
                Graphic::Text { at, .. } => *at = at.offset(shift),
            }
        }
        let gate = if count > 1 { number } else { 0 };
        for p in &mut pins {
            // A single-unit device has no gates to tell apart, and leaving it
            // unset keeps its symbol byte-identical to one written before gates.
            p.gate = gate;
            p.at = p.at.offset(shift);
            p.end = p.end.offset(shift);
        }
        if count > 1 {
            symbol.graphic_gates.resize(symbol.graphics.len(), 0);
            symbol.graphic_gates.extend(graphics.iter().map(|_| gate));
        }
        symbol.graphics.extend(graphics);
        symbol.pins.extend(pins);
        offset += width + 12700;
    }
    let mut pins = BTreeSet::new();
    symbol.pins.retain(|p| pins.insert(p.number.clone()));
    if symbol.pins.iter().any(|p| p.number.is_empty()) {
        return Err("Empty pin number".into());
    }
    if symbol.power_net.is_some() && !symbol.pins.iter().any(|p| p.electrical_type == "power_in") {
        symbol.power_net = None;
    }
    Ok(symbol)
}
/// Whether an item says it is hidden, `None` when it does not say: the bare
/// `hide` atom KiCad 6 and 7 write, or `(hide yes)` / `(hide no)` since KiCad 8.
fn hidden(item: &Expr) -> Option<bool> {
    item.list().iter().find_map(|e| match (e.atom(), e.head()) {
        ("hide", _) => Some(true),
        (_, "hide") => Some(e.arg(1) != "no"),
        _ => None,
    })
}
fn arc_points(a: Point, m: Point, b: Point) -> Vec<Point> {
    let (ax, ay, mx, my, bx, by) = (
        a.x as f64, a.y as f64, m.x as f64, m.y as f64, b.x as f64, b.y as f64,
    );
    let d = 2. * (ax * (my - by) + mx * (by - ay) + bx * (ay - my));
    if d.abs() < 0.01 {
        return vec![a, m, b];
    }
    let (aa, mm, bb) = (ax * ax + ay * ay, mx * mx + my * my, bx * bx + by * by);
    let cx = (aa * (my - by) + mm * (by - ay) + bb * (ay - my)) / d;
    let cy = (aa * (bx - mx) + mm * (ax - bx) + bb * (mx - ax)) / d;
    let start = (ay - cy).atan2(ax - cx);
    let mid = (my - cy).atan2(mx - cx);
    let end = (by - cy).atan2(bx - cx);
    let tau = std::f64::consts::TAU;
    let ccw = (end - start).rem_euclid(tau);
    let sweep = if (mid - start).rem_euclid(tau) <= ccw {
        ccw
    } else {
        ccw - tau
    };
    let r = ((ax - cx).powi(2) + (ay - cy).powi(2)).sqrt();
    (0..=32)
        .map(|i| {
            let t = start + sweep * i as f64 / 32.;
            Point::new(
                (cx + r * t.cos()).round() as i32,
                (cy + r * t.sin()).round() as i32,
            )
        })
        .collect()
}

/// Import a KiCad 6+ `.kicad_mod` footprint. Silkscreen (or, without it, fabrication)
/// outlines are kept. Pads rotated by other than a multiple of 90° are approximated by
/// their bounding box and reported in the warnings.
pub fn import_footprint(text: &str) -> Result<(Footprint, Vec<String>), String> {
    let root = parse(text)?;
    if !matches!(root.head(), "footprint" | "module") {
        return Err("Expected a KiCad .kicad_mod footprint".into());
    }
    let name = root.arg(1).trim().to_owned();
    if name.is_empty() {
        return Err("Footprint name is empty".into());
    }
    let um = |v: f64| (v * 1000.).round() as i32;
    let xy =
        |e: &Expr| -> Result<Point, String> { Ok(Point::new(um(e.number(1)?), um(e.number(2)?))) };
    let mut warnings = vec![];
    let mut pads = vec![];
    // The 3D model, with the placement KiCad gives it on the pads. Read as KiCad 9.0.0
    // reads it (`PCB_IO_KICAD_SEXPR_PARSER::parse3DModel`): each entry in file order,
    // a later one replacing an earlier, and `at`, the offset's name before KiCad 5, in
    // INCHES. Only the first `(model …)` is kept; `hide` and `opacity` are not read.
    let xyz = |entry: &Expr| -> Result<[f64; 3], String> {
        let xyz = entry.required("xyz")?;
        Ok([xyz.number(1)?, xyz.number(2)?, xyz.number(3)?])
    };
    let model = match root.child("model") {
        Some(model) if !model.arg(1).trim().is_empty() => {
            let mut placed = Model {
                path: model.arg(1).trim().to_owned(),
                offset: [0.; 3],
                scale: [1.; 3],
                rotation: [0.; 3],
            };
            for entry in model.list().iter().skip(2) {
                match entry.head() {
                    "offset" => placed.offset = xyz(entry)?,
                    // KiCad multiplies by the float `25.4f`, which lands a one-inch
                    // offset 3.8e-7 mm short; this takes the inch exactly.
                    "at" => placed.offset = xyz(entry)?.map(|inches| inches * 25.4),
                    "scale" => placed.scale = xyz(entry)?,
                    "rotate" => placed.rotation = xyz(entry)?,
                    _ => {}
                }
            }
            Some(placed)
        }
        _ => None,
    };
    let (mut silk, mut fab) = (vec![], vec![]);
    for item in root.list().iter().skip(2) {
        let outline = match item.child("layer").map_or("", |l| l.arg(1)) {
            "F.SilkS" | "F.Silkscreen" => Some(&mut silk),
            "F.Fab" => Some(&mut fab),
            _ => None,
        };
        match item.head() {
            "pad" => {
                let kind = item.arg(2);
                let at = item.required("at")?;
                let rotation = at.number(3).unwrap_or(0.);
                let size = item.required("size")?;
                let (mut w, mut h) = (size.number(1)?, size.number(2)?);
                let half_turn = rotation.rem_euclid(180.);
                if (half_turn - 90.).abs() < 1e-6 {
                    (w, h) = (h, w);
                } else if half_turn > 1e-6 {
                    let (c, s) = (
                        rotation.to_radians().cos().abs(),
                        rotation.to_radians().sin().abs(),
                    );
                    (w, h) = (w * c + h * s, w * s + h * c);
                    warnings.push(format!(
                        "Pad {} rotated {rotation}° is approximated by its bounding box",
                        item.arg(1)
                    ));
                }
                let drill = item.child("drill").and_then(|d| {
                    d.list()
                        .iter()
                        .skip(1)
                        .find_map(|e| e.atom().parse::<f64>().ok())
                });
                let drill = match kind {
                    "thru_hole" | "np_thru_hole" => Some(um(drill
                        .filter(|d| *d > 0.)
                        .ok_or("Through-hole pad without a drill")?)),
                    "smd" | "connect" => None,
                    other => return Err(format!("Unsupported pad type {other}")),
                };
                let shape = match item.arg(3) {
                    "circle" => PadShape::Circle,
                    "oval" => PadShape::Oval,
                    "rect" | "roundrect" | "trapezoid" | "chamfered_rect" => PadShape::Rect,
                    other => {
                        warnings.push(format!(
                            "Pad {} shape {other} is approximated by a rectangle",
                            item.arg(1)
                        ));
                        PadShape::Rect
                    }
                };
                if drill.is_none()
                    && item.child("layers").is_some_and(|l| {
                        let layers: Vec<&str> = l.list().iter().skip(1).map(Expr::atom).collect();
                        layers.contains(&"B.Cu")
                            && !layers.iter().any(|l| *l == "F.Cu" || *l == "*.Cu")
                    })
                {
                    warnings.push(format!(
                        "Bottom-side pad {} is placed on the footprint side",
                        item.arg(1)
                    ));
                }
                let d = drill.unwrap_or(0);
                pads.push(Pad {
                    number: if kind == "np_thru_hole" {
                        String::new()
                    } else {
                        item.arg(1).into()
                    },
                    at: xy(at)?,
                    size: Point::new(um(w).max(d).max(1), um(h).max(d).max(1)),
                    shape,
                    drill,
                    plated: kind != "np_thru_hole",
                });
            }
            "fp_line" => {
                if let Some(o) = outline {
                    o.push(vec![
                        xy(item.required("start")?)?,
                        xy(item.required("end")?)?,
                    ]);
                }
            }
            "fp_rect" => {
                if let Some(o) = outline {
                    let (a, b) = (xy(item.required("start")?)?, xy(item.required("end")?)?);
                    o.push(vec![a, Point::new(b.x, a.y), b, Point::new(a.x, b.y), a]);
                }
            }
            "fp_poly" => {
                if let Some(o) = outline {
                    let mut points = item
                        .required("pts")?
                        .list()
                        .iter()
                        .skip(1)
                        .map(xy)
                        .collect::<Result<Vec<_>, _>>()?;
                    if let Some(&first) = points.first() {
                        points.push(first);
                    }
                    o.push(points);
                }
            }
            "fp_circle" => {
                if let Some(o) = outline {
                    let c = xy(item.required("center")?)?;
                    let e = xy(item.required("end")?)?;
                    let r = f64::from(e.x - c.x).hypot(f64::from(e.y - c.y));
                    o.push(
                        (0..=32)
                            .map(|i| {
                                let t = std::f64::consts::TAU * f64::from(i) / 32.;
                                Point::new(
                                    c.x + (r * t.cos()).round() as i32,
                                    c.y + (r * t.sin()).round() as i32,
                                )
                            })
                            .collect(),
                    );
                }
            }
            "fp_arc" => {
                if let Some(o) = outline {
                    if let Some(mid) = item.child("mid") {
                        o.push(arc_points(
                            xy(item.required("start")?)?,
                            xy(mid)?,
                            xy(item.required("end")?)?,
                        ));
                    } else {
                        warnings.push("Legacy arc without a midpoint skipped".into());
                    }
                }
            }
            _ => {}
        }
    }
    Ok((
        Footprint {
            name,
            pads,
            silk: if silk.is_empty() { fab } else { silk },
            model,
        },
        warnings,
    ))
}

/// Where KiCad puts a footprint's 3D model, as a row-major 4×4 matrix taking a point
/// of the model file to the footprint's 3D frame.
///
/// KiCad 9.0.0 composes `T(offset) · Rz(−rz) · Ry(−ry) · Rx(−rx) · S(scale)`: the
/// scale first, then the rotation's X, Y and Z turns, each by the NEGATED angle, then
/// the offset (`render_3d_opengl.cpp:1070-1074`; the STEP exporter's
/// `STEP_PCB_MODEL::getModelLocation` says "aOrientation is applied -Z*-Y*-X"). Its
/// model unit is the millimetre (`UNITS3D_TO_UNITSPCB` is `IU_PER_MM`), and so is the
/// offset.
///
/// The frame is KiCad's 3D view of the footprint, not its file: x as the footprint's,
/// y the footprint's NEGATED (a `.kicad_mod` points y down, the 3D view up, and KiCad
/// places the footprint at `(x, −y)`), z up from the board's top copper. A pad at
/// `(x, y)` µm in [`Footprint`] coordinates is at `(x / 1000, −y / 1000, 0)` here.
///
/// A turn by a whole multiple of 90° uses exact 0 and ±1, so a model KiCad turns a
/// quarter keeps its faces exactly on the axes.
pub fn model_placement(model: &Model) -> [f64; 16] {
    let turn = |axis: usize, degrees: f64| -> [[f64; 3]; 3] {
        let (c, s) = cos_sin_degrees(-degrees);
        let mut m = [[0.; 3]; 3];
        let (i, j) = ((axis + 1) % 3, (axis + 2) % 3);
        m[axis][axis] = 1.;
        m[i][i] = c;
        m[i][j] = -s;
        m[j][i] = s;
        m[j][j] = c;
        m
    };
    let product = |a: [[f64; 3]; 3], b: [[f64; 3]; 3]| {
        let mut m = [[0.; 3]; 3];
        for (r, row) in m.iter_mut().enumerate() {
            for (c, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| a[r][k] * b[k][c]).sum();
            }
        }
        m
    };
    let [rx, ry, rz] = model.rotation;
    let rotation = product(product(turn(2, rz), turn(1, ry)), turn(0, rx));
    let mut m = [0.; 16];
    for r in 0..3 {
        for c in 0..3 {
            m[r * 4 + c] = rotation[r][c] * model.scale[c];
        }
        m[r * 4 + 3] = model.offset[r];
    }
    m[15] = 1.;
    m
}

/// `(cos, sin)` of an angle in degrees, exact at whole multiples of 90°.
fn cos_sin_degrees(degrees: f64) -> (f64, f64) {
    let turned = degrees.rem_euclid(360.);
    if turned == 0. {
        (1., 0.)
    } else if turned == 90. {
        (0., 1.)
    } else if turned == 180. {
        (-1., 0.)
    } else if turned == 270. {
        (0., -1.)
    } else {
        let radians = degrees.to_radians();
        (radians.cos(), radians.sin())
    }
}

/// How a symbol names the footprint its part takes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FootprintLink {
    /// The symbol's `Footprint` property, `library:name`, as in
    /// `Package_SO:SOIC-8_3.9x4.9mm_P1.27mm`. `library` is empty when the property
    /// names a footprint without one.
    Named { library: String, name: String },
    /// A generic symbol leaves `Footprint` empty; the footprint is chosen, and KiCad
    /// offers those its `ki_fp_filters` patterns match (see [`footprint_filter_matches`]).
    /// Empty when the symbol gives no patterns either.
    Chosen { filters: Vec<String> },
}

/// Follow the first link of KiCad's chain: the footprint `symbol` names, or the
/// patterns a generic symbol offers instead.
pub fn footprint_link(symbol: &Symbol) -> FootprintLink {
    let named = symbol
        .properties
        .get("Footprint")
        .map_or("", |value| value.trim());
    if named.is_empty() {
        let filters = symbol
            .properties
            .get("ki_fp_filters")
            .map_or("", String::as_str);
        return FootprintLink::Chosen {
            filters: filters.split_whitespace().map(str::to_owned).collect(),
        };
    }
    match named.split_once(':') {
        Some((library, name)) => FootprintLink::Named {
            library: library.trim().to_owned(),
            name: name.trim().to_owned(),
        },
        None => FootprintLink::Named {
            library: String::new(),
            name: named.to_owned(),
        },
    }
}

/// Record on `symbol` the footprint its part was given, as KiCad's "Assign
/// Footprints" does: the `Footprint` property becomes `library:name`
/// (`Resistor_SMD:R_0805_2012Metric`), the library NICKNAME first. That property is
/// what a netlist writes ([`crate::Document::kicad_netlist`]), and Pcbnew resolves
/// a footprint by it: a bare `R_0805_2012Metric` names no library and cannot be
/// found. The nickname is the `.pretty` folder's name without the extension,
/// which is the nickname KiCad's own library table gives each of its folders.
/// An empty `library` keeps the bare name, since there is no nickname to give.
pub fn assign_footprint(symbol: &mut Symbol, library: &str, name: &str) {
    let library = library.trim().trim_end_matches(".pretty");
    let id = if library.is_empty() { name.trim().to_owned() } else { format!("{library}:{}", name.trim()) };
    symbol.properties.insert("Footprint".into(), id);
}

/// Whether KiCad 9.0.0 offers the footprint `library:name` for a symbol whose
/// `ki_fp_filters` are `filters` (`FOOTPRINT_FILTER_IT::FootprintFilterMatch`): no
/// patterns offer every footprint; otherwise any one matching is enough. A pattern
/// matches the whole name, ignoring case, with `*` for any run of characters and `?`
/// for one. A pattern holding `:` is matched against `library:name`, any other
/// against the name alone.
pub fn footprint_filter_matches(filters: &[String], library: &str, name: &str) -> bool {
    filters.is_empty()
        || filters.iter().any(|filter| {
            let pattern: Vec<char> = filter.to_lowercase().chars().collect();
            let subject = if filter.contains(':') {
                format!("{library}:{name}")
            } else {
                name.to_owned()
            };
            let subject: Vec<char> = subject.to_lowercase().chars().collect();
            wildcard_matches(&pattern, &subject)
        })
}

/// Anchored `*` / `?` wildcard match, iterative with one backtrack point.
fn wildcard_matches(pattern: &[char], subject: &[char]) -> bool {
    let (mut p, mut s) = (0, 0);
    let mut star: Option<(usize, usize)> = None;
    while s < subject.len() {
        match pattern.get(p) {
            Some('*') => {
                star = Some((p, s));
                p += 1;
            }
            Some(&c) if c == '?' || c == subject[s] => {
                p += 1;
                s += 1;
            }
            _ => match star {
                Some((star_p, star_s)) => {
                    p = star_p + 1;
                    s = star_s + 1;
                    star = Some((star_p, star_s + 1));
                }
                None => return false,
            },
        }
    }
    pattern[p..].iter().all(|&c| c == '*')
}
