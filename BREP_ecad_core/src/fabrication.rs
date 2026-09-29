//! Manufacturing outputs: Gerber X2 layers, Excellon drill files, and a zip bundle.
//!
//! Every file shares one origin, the lower-left corner of the board outline, with
//! positive Y up as both formats expect. Board coordinates are Y down, so Y is
//! flipped here and nowhere else. All layers are viewed from the top, as Gerber
//! requires; bottom-side footprints are already mirrored by their placement.
use crate::board::{Board, Placement, Shape};
use crate::{Document, Point};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

/// Silkscreen stroke width.
pub const SILK_WIDTH: i32 = 150;
/// Stroke width used to draw the board outline.
pub const OUTLINE_WIDTH: i32 = 100;
/// Height of reference designators on the silkscreen.
pub const TEXT_HEIGHT: i32 = 1000;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FabricationFile {
    pub name: String,
    pub contents: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Fabrication {
    pub files: Vec<FabricationFile>,
    /// Problems worth fixing before ordering boards; the files are still complete.
    pub warnings: Vec<String>,
}
impl Fabrication {
    /// All files in one uncompressed zip archive, as most board houses accept.
    pub fn zip(&self) -> Vec<u8> {
        zip(self
            .files
            .iter()
            .map(|f| (f.name.as_str(), f.contents.as_bytes())))
    }
}

impl Document {
    /// Gerber and drill files for the board. `name` prefixes every file name.
    pub fn fabrication(&self, name: &str) -> Result<Fabrication, String> {
        self.validate()?;
        // The zones are refilled on a copy first: what is sent to be made is the
        // fill the copper as it stands asks for, never a stale one the user forgot
        // to refill.
        let netlist = self.netlist();
        let refilled;
        let board = if self.board.zones.is_empty() {
            &self.board
        } else {
            let mut copy = self.board.clone();
            copy.fill_zones(&netlist);
            refilled = copy;
            &refilled
        };
        let base: String = name
            .trim()
            .chars()
            .map(|c| {
                if c.is_ascii_alphanumeric() || "-_.".contains(c) {
                    c
                } else {
                    '_'
                }
            })
            .collect();
        let base = if base.is_empty() {
            "board".into()
        } else {
            base
        };
        let origin = Origin::new(board);
        let last = board.layer_count - 1;
        let mut files = vec![];
        let mut add = |suffix: &str, contents: String| {
            files.push(FabricationFile {
                name: format!("{base}-{suffix}"),
                contents,
            })
        };

        for layer in 0..board.layer_count {
            let (name, extension, function) = if layer == 0 {
                ("F_Cu".to_owned(), "gtl".to_owned(), "Copper,L1,Top")
            } else if layer == last {
                ("B_Cu".into(), "gbl".into(), "Copper,L{},Bot")
            } else {
                (
                    format!("In{layer}_Cu"),
                    format!("g{}", layer + 1),
                    "Copper,L{},Inr",
                )
            };
            let function = function.replace("{}", &(u32::from(layer) + 1).to_string());
            add(
                &format!("{name}.{extension}"),
                copper(board, &origin, layer).finish(&function, "Positive"),
            );
        }
        for (top, side) in [(true, "F"), (false, "B")] {
            let (letter, position) = if top { ('t', "Top") } else { ('b', "Bot") };
            add(
                &format!("{side}_Mask.g{letter}s"),
                mask(board, &origin, top).finish(&format!("Soldermask,{position}"), "Negative"),
            );
            add(
                &format!("{side}_Paste.g{letter}p"),
                paste(board, &origin, top).finish(&format!("Paste,{position}"), "Positive"),
            );
        }
        let mut unsupported = BTreeSet::new();
        for (top, side) in [(true, "F"), (false, "B")] {
            let (letter, position) = if top { ('t', "Top") } else { ('b', "Bot") };
            let layer = legend(self, &origin, top, &mut unsupported);
            add(
                &format!("{side}_Silkscreen.g{letter}o"),
                layer.finish(&format!("Legend,{position}"), "Positive"),
            );
        }
        let mut profile = Gerber::default();
        profile.select(Aperture::Circle(OUTLINE_WIDTH), Some("Profile"));
        let mut outline: Vec<Point> = board.outline.clone();
        outline.push(board.outline[0]);
        profile.stroke(outline.iter().map(|p| origin.point(*p)));
        add("Edge_Cuts.gm1", profile.finish("Profile,NP", "Positive"));

        let (plated, unplated) = drills(board, &origin);
        let layers = board.layer_count;
        add(
            "PTH.drl",
            excellon(&plated, &format!("Plated,1,{layers},PTH"), "Plated,PTH"),
        );
        if !unplated.is_empty() {
            add(
                "NPTH.drl",
                excellon(
                    &unplated,
                    &format!("NonPlated,1,{layers},NPTH"),
                    "NonPlated,NPTH",
                ),
            );
        }

        let mut warnings = vec![];
        if board.zones != self.board.zones {
            warnings.push(
                "The zones' fill was out of date, so these files carry a fresh fill: \
                 fill the zones on the board to see what was sent."
                    .to_owned(),
            );
        }
        let findings = board.drc(&netlist);
        if !findings.is_empty() {
            let unrouted = findings
                .iter()
                .filter(|v| v.kind == crate::board::ViolationKind::Unrouted)
                .count();
            warnings.push(format!(
                "The design rule check has {} finding(s), {unrouted} of them unrouted connections. \
                 Fix them before ordering boards.",
                findings.len()
            ));
        }
        if !unsupported.is_empty() {
            warnings.push(format!(
                "Silkscreen text shows these characters as '?': {}",
                unsupported.iter().collect::<String>()
            ));
        }
        add("pos.csv", self.pick_and_place_csv());
        add("bom.csv", self.electronics_bom_csv());
        let readme = readme(self, &files, &warnings);
        files.push(FabricationFile {
            name: format!("{base}-README.txt"),
            contents: readme,
        });
        Ok(Fabrication { files, warnings })
    }

    /// The pick-and-place (centroid) file, one row per placed part, in the
    /// column convention of JLCPCB's CPL template, which KiCad's own position
    /// export and most assembly houses also accept:
    /// `Designator,Val,Package,Mid X,Mid Y,Rotation,Layer`.
    ///
    /// * Mid X / Mid Y are the footprint's anchor (its origin, where KiCad puts a
    ///   part's position), in millimetres with a `mm` suffix, from the Gerbers'
    ///   own origin: the outline's lower-left corner, Y up.
    /// * Rotation is in degrees, counter-clockwise as seen from the top, 0 to 270.
    ///   A bottom part is reported the same way: the angle of its mirrored
    ///   footprint seen from the top. No assembler-specific bottom-side
    ///   correction is applied.
    /// * Layer is `Top` or `Bottom`.
    ///
    /// Rows are in reference order (`R2` before `R10`). Power symbols have no
    /// part to place and do not appear.
    pub fn pick_and_place_csv(&self) -> String {
        let origin = Origin::new(&self.board);
        let mut rows: Vec<(&str, String)> = vec![];
        for placement in &self.board.placements {
            let Some(component) = self.placed_part(placement) else {
                continue;
            };
            let (x, y) = origin.point(placement.at);
            let rotation = (4 - u32::from(placement.rotation % 4)) % 4 * 90;
            rows.push((
                &component.reference,
                [
                    csv_field(&component.reference),
                    csv_field(&component.value),
                    csv_field(&placement.footprint.name),
                    format!("{}mm", millimetres(x / 1000)),
                    format!("{}mm", millimetres(y / 1000)),
                    rotation.to_string(),
                    if placement.bottom { "Bottom" } else { "Top" }.into(),
                ]
                .join(","),
            ));
        }
        rows.sort_by(|a, b| natural(a.0).cmp(&natural(b.0)));
        let mut out = String::from("Designator,Val,Package,Mid X,Mid Y,Rotation,Layer\n");
        for (_, row) in rows {
            out.push_str(&row);
            out.push('\n');
        }
        out
    }

    /// The electronics bill of materials: one line per value and footprint,
    /// with every reference that uses it, in the column convention of JLCPCB's
    /// BOM template: `Comment,Designator,Footprint,Quantity`. `Comment` is the
    /// part's value, as JLCPCB names it; `Designator` lists the references in
    /// order (`R1,R2,R10`), quoted because it holds commas.
    ///
    /// Every component on the schematic is listed, placed on the board or not:
    /// a part not yet on the board still has to be bought. Its footprint is the
    /// one it would be placed with. Power symbols are not parts and are left out.
    /// Lines are in the order of their first reference.
    pub fn electronics_bom_csv(&self) -> String {
        let mut groups: BTreeMap<(String, String), Vec<&str>> = BTreeMap::new();
        for component in &self.components {
            if component.symbol.power_net.is_some() {
                continue;
            }
            let footprint = self
                .board
                .placements
                .iter()
                .find(|p| p.component == component.id)
                .map(|p| p.footprint.name.clone())
                .or_else(|| component.pads.as_ref().map(|f| f.name.clone()))
                .or_else(|| component.symbol.properties.get("Footprint").cloned())
                .unwrap_or_default();
            groups
                .entry((component.value.clone(), footprint))
                .or_default()
                .push(&component.reference);
        }
        let mut lines: Vec<_> = groups
            .into_iter()
            .map(|((value, footprint), mut references)| {
                references.sort_by(|a, b| natural(a).cmp(&natural(b)));
                (value, footprint, references)
            })
            .collect();
        lines.sort_by(|a, b| natural(a.2[0]).cmp(&natural(b.2[0])));
        let mut out = String::from("Comment,Designator,Footprint,Quantity\n");
        for (value, footprint, references) in lines {
            let _ = writeln!(
                out,
                "{},{},{},{}",
                csv_field(&value),
                csv_field(&references.join(",")),
                csv_field(&footprint),
                references.len()
            );
        }
        out
    }

    /// The component a board placement mounts, unless it is a power symbol.
    fn placed_part(&self, placement: &Placement) -> Option<&crate::Component> {
        self.components
            .iter()
            .find(|c| c.id == placement.component && c.symbol.power_net.is_none())
    }
}

/// What a bundle's files hold, counted by reading the files themselves: pad
/// flashes on the outer copper, drill hits, the outline's extent and the CSV
/// rows. A host reports these beside an export so a caller can compare them
/// with the board without trusting the writer.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReadBack {
    /// `D03` flashes in the top and bottom copper files.
    pub top_flashes: usize,
    pub bottom_flashes: usize,
    /// Hits in the plated and non-plated drill files.
    pub plated_holes: usize,
    pub unplated_holes: usize,
    /// Width and height of the Edge_Cuts drawing in millimetres, measured to
    /// the centre of its stroke.
    pub outline_mm: (f64, f64),
    /// Data rows (not the header) of the pick-and-place and BOM files.
    pub placements: usize,
    pub bom_lines: usize,
}

impl Fabrication {
    /// Count what the files hold; see [`ReadBack`].
    pub fn read_back(&self) -> ReadBack {
        let text = |suffix: &str| {
            self.files
                .iter()
                .find(|f| f.name.ends_with(suffix))
                .map_or("", |f| f.contents.as_str())
        };
        let flashes = |t: &str| t.lines().filter(|l| l.starts_with('X') && l.ends_with("D03*")).count();
        let holes = |t: &str| t.lines().filter(|l| l.starts_with('X')).count();
        let rows = |t: &str| t.lines().count().saturating_sub(1);
        let mut min = (i64::MAX, i64::MAX);
        let mut max = (i64::MIN, i64::MIN);
        for line in text("-Edge_Cuts.gm1").lines() {
            let Some((x, rest)) = line.strip_prefix('X').and_then(|l| l.split_once('Y')) else {
                continue;
            };
            let y = rest.split('D').next().unwrap_or("");
            if let (Ok(x), Ok(y)) = (x.parse::<i64>(), y.parse::<i64>()) {
                min = (min.0.min(x), min.1.min(y));
                max = (max.0.max(x), max.1.max(y));
            }
        }
        let outline_mm = if min.0 <= max.0 {
            ((max.0 - min.0) as f64 / 1e6, (max.1 - min.1) as f64 / 1e6)
        } else {
            (0., 0.)
        };
        ReadBack {
            top_flashes: flashes(text("-F_Cu.gtl")),
            bottom_flashes: flashes(text("-B_Cu.gbl")),
            plated_holes: holes(text("-PTH.drl")),
            unplated_holes: holes(text("-NPTH.drl")),
            outline_mm,
            placements: rows(text("-pos.csv")),
            bom_lines: rows(text("-bom.csv")),
        }
    }
}

/// A CSV field, quoted when it holds a comma, a quote or a line break.
fn csv_field(text: &str) -> String {
    if text.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_owned()
    }
}

/// A sort key that orders the digits in a reference by value: `R2` before `R10`.
fn natural(text: &str) -> Vec<(String, u64)> {
    let mut key = vec![];
    let mut letters = String::new();
    let mut digits = String::new();
    for c in text.chars() {
        if c.is_ascii_digit() {
            digits.push(c);
        } else {
            if !digits.is_empty() {
                key.push((std::mem::take(&mut letters), digits.parse().unwrap_or(u64::MAX)));
                digits.clear();
            }
            letters.push(c);
        }
    }
    key.push((letters, digits.parse().unwrap_or(0)));
    key
}

/// What each file in the bundle is, for the person placing the order.
fn readme(document: &Document, files: &[FabricationFile], warnings: &[String]) -> String {
    let board = &document.board;
    let (min, max) = board.outline_bounds();
    let mut out = String::from("Fabrication files\n=================\n\n");
    let _ = writeln!(
        out,
        "Board: {} copper layer(s), outline {} x {} mm.",
        board.layer_count,
        millimetres(i64::from(max.x) - i64::from(min.x)),
        millimetres(i64::from(max.y) - i64::from(min.y)),
    );
    out.push_str(
        "Units are millimetres. Every file shares one origin, the lower-left corner of the\n\
         board outline, with Y up. All layers are viewed from the top.\n\n",
    );
    for file in files {
        let (_, suffix) = file.name.rsplit_once('-').unwrap_or(("", &file.name));
        let what = match suffix.split('.').next().unwrap_or("") {
            "F_Cu" => "Gerber X2, top copper".to_owned(),
            "B_Cu" => "Gerber X2, bottom copper".to_owned(),
            inner if inner.starts_with("In") && inner.ends_with("_Cu") => {
                format!("Gerber X2, inner copper layer {}", &inner[2..inner.len() - 3])
            }
            "F_Mask" => "Gerber X2, top solder mask (negative: flashes are openings)".into(),
            "B_Mask" => "Gerber X2, bottom solder mask (negative: flashes are openings)".into(),
            "F_Paste" => "Gerber X2, top paste stencil".into(),
            "B_Paste" => "Gerber X2, bottom paste stencil".into(),
            "F_Silkscreen" => "Gerber X2, top silkscreen (legend)".into(),
            "B_Silkscreen" => "Gerber X2, bottom silkscreen (legend)".into(),
            "Edge_Cuts" => "Gerber X2, board outline (profile)".into(),
            "PTH" => "Excellon, plated holes (vias and component holes)".into(),
            "NPTH" => "Excellon, non-plated holes".into(),
            "pos" => "pick and place (centroid), JLCPCB CPL columns".into(),
            "bom" => "bill of materials, JLCPCB BOM columns, grouped by value and footprint".into(),
            _ => continue,
        };
        let _ = writeln!(out, "{:<32} {what}", file.name);
    }
    out.push_str(
        "\nPick and place: Mid X / Mid Y are each footprint's origin. Rotation is degrees\n\
         counter-clockwise seen from the top; a bottom part's is its mirrored footprint's,\n\
         also seen from the top, with no assembler-specific correction.\n",
    );
    if !warnings.is_empty() {
        out.push_str("\nWarnings:\n");
        for warning in warnings {
            let _ = writeln!(out, "- {warning}");
        }
    }
    out
}

/// Maps board micrometres (Y down) to output nanometres (Y up) from the outline's
/// lower-left corner. Nanometres keep half-micrometre pad centres exact.
struct Origin {
    left: i64,
    bottom: i64,
}
impl Origin {
    fn new(board: &Board) -> Self {
        let (min, max) = board.outline_bounds();
        Self {
            left: i64::from(min.x),
            bottom: i64::from(max.y),
        }
    }
    fn point(&self, p: Point) -> (i64, i64) {
        (
            (i64::from(p.x) - self.left) * 1000,
            (self.bottom - i64::from(p.y)) * 1000,
        )
    }
    /// Midpoint of two board points.
    fn middle(&self, a: Point, b: Point) -> (i64, i64) {
        (
            (i64::from(a.x) + i64::from(b.x) - 2 * self.left) * 500,
            (2 * self.bottom - i64::from(a.y) - i64::from(b.y)) * 500,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Aperture {
    Circle(i32),
    Rect(i32, i32),
    Obround(i32, i32),
}

/// One Gerber image. Apertures are collected while drawing and written ahead of
/// the drawing commands when the file is finished.
#[derive(Default)]
struct Gerber {
    apertures: Vec<(Aperture, Option<&'static str>)>,
    current: Option<usize>,
    body: String,
}
impl Gerber {
    fn select(&mut self, aperture: Aperture, function: Option<&'static str>) {
        let key = (aperture, function);
        let index = match self.apertures.iter().position(|a| *a == key) {
            Some(index) => index,
            None => {
                self.apertures.push(key);
                self.apertures.len() - 1
            }
        };
        if self.current != Some(index) {
            self.current = Some(index);
            let _ = writeln!(self.body, "D{}*", index + 10);
        }
    }
    fn flash(&mut self, (x, y): (i64, i64)) {
        let _ = writeln!(self.body, "X{x}Y{y}D03*");
    }
    fn stroke(&mut self, points: impl IntoIterator<Item = (i64, i64)>) {
        for (i, (x, y)) in points.into_iter().enumerate() {
            let operation = if i == 0 { "D02" } else { "D01" };
            let _ = writeln!(self.body, "X{x}Y{y}{operation}*");
        }
    }
    /// Draw a copper-style shape grown by `grow` on every side.
    fn shape(&mut self, origin: &Origin, shape: Shape, grow: i32, function: Option<&'static str>) {
        match shape {
            Shape::Rect { min, max } => {
                self.select(
                    Aperture::Rect(max.x - min.x + 2 * grow, max.y - min.y + 2 * grow),
                    function,
                );
                self.flash(origin.middle(min, max));
            }
            Shape::Capsule { a, b, radius } => {
                let diameter = 2 * (radius + grow);
                if a == b {
                    self.select(Aperture::Circle(diameter), function);
                    self.flash(origin.point(a));
                } else if a.x == b.x || a.y == b.y {
                    let (dx, dy) = (a.x.abs_diff(b.x) as i32, a.y.abs_diff(b.y) as i32);
                    self.select(Aperture::Obround(dx + diameter, dy + diameter), function);
                    self.flash(origin.middle(a, b));
                } else {
                    self.select(Aperture::Circle(diameter), function);
                    self.stroke([origin.point(a), origin.point(b)]);
                }
            }
        }
    }
    fn finish(self, file_function: &str, polarity: &str) -> String {
        let mut out = String::new();
        let _ = writeln!(
            out,
            "%TF.GenerationSoftware,eCAD,Schematic and PCB Studio,{}*%",
            env!("CARGO_PKG_VERSION")
        );
        out.push_str("%TF.SameCoordinates,Original*%\n");
        let _ = writeln!(out, "%TF.FileFunction,{file_function}*%");
        let _ = writeln!(out, "%TF.FilePolarity,{polarity}*%");
        out.push_str("%FSLAX46Y46*%\n%MOMM*%\n%LPD*%\nG01*\nG75*\n");
        for (index, (aperture, function)) in self.apertures.iter().enumerate() {
            if let Some(function) = function {
                let _ = writeln!(out, "%TA.AperFunction,{function}*%");
            }
            let template = match *aperture {
                Aperture::Circle(d) => format!("C,{}", millimetres(d)),
                Aperture::Rect(w, h) => format!("R,{}X{}", millimetres(w), millimetres(h)),
                Aperture::Obround(w, h) => format!("O,{}X{}", millimetres(w), millimetres(h)),
            };
            let _ = writeln!(out, "%ADD{}{template}*%", index + 10);
            if function.is_some() {
                out.push_str("%TD*%\n");
            }
        }
        out.push_str(&self.body);
        out.push_str("M02*\n");
        out
    }
}

fn millimetres(um: impl Into<i64>) -> String {
    let um = um.into();
    let sign = if um < 0 { "-" } else { "" };
    format!("{sign}{}.{:03}", um.abs() / 1000, um.abs() % 1000)
}

/// A drilled pad with copper around the hole, or a plated one of any size.
fn pad_has_copper(placement: &Placement, index: usize) -> bool {
    let pad = &placement.footprint.pads[index];
    match pad.drill {
        Some(drill) if !pad.plated => pad.size.x.min(pad.size.y) > drill,
        _ => true,
    }
}

fn copper(board: &Board, origin: &Origin, layer: u8) -> Gerber {
    let last = board.layer_count - 1;
    let mut gerber = Gerber::default();
    // Zones first, as G36/G37 regions, each piece's holes joined to its outside by
    // cut-ins (`zones::fractured`): the layer stays all dark polarity, so nothing
    // drawn after a zone can be cleared by it. Each region carries the X2 function
    // and net of its zone.
    for zone in board.zones.iter().filter(|z| z.layer == layer && !z.fill.is_empty()) {
        let _ = writeln!(gerber.body, "%TA.AperFunction,Conductor*%");
        let _ = writeln!(gerber.body, "%TO.N,{}*%", zone.net.replace(['*', '%', ','], "_"));
        for piece in &zone.fill {
            let ring = crate::board::zones::fractured(piece);
            gerber.body.push_str("G36*\n");
            gerber.stroke(ring.iter().chain(ring.first()).map(|p| origin.point(*p)));
            gerber.body.push_str("G37*\n");
        }
        gerber.body.push_str("%TD*%\n");
    }
    for placement in &board.placements {
        for (index, pad) in placement.footprint.pads.iter().enumerate() {
            let (from, to) = placement.pad_layers(pad, board.layer_count);
            if !(from..=to).contains(&layer) || !pad_has_copper(placement, index) {
                continue;
            }
            let function = match (pad.drill, pad.plated) {
                (None, _) => "SMDPad,CuDef",
                (Some(_), true) => "ComponentPad",
                (Some(_), false) if layer == 0 || layer == last => "WasherPad",
                (Some(_), false) => continue,
            };
            gerber.shape(origin, placement.pad_shape(pad), 0, Some(function));
        }
    }
    for track in board.tracks.iter().filter(|t| t.layer == layer) {
        gerber.select(Aperture::Circle(track.width), Some("Conductor"));
        gerber.stroke(track.points.iter().map(|p| origin.point(*p)));
    }
    for via in &board.vias {
        gerber.shape(
            origin,
            Shape::circle(via.at, via.diameter / 2),
            0,
            Some("ViaPad"),
        );
    }
    gerber
}

/// Openings over pads on one side. Vias stay covered (tented).
fn mask(board: &Board, origin: &Origin, top: bool) -> Gerber {
    let side = if top { 0 } else { board.layer_count - 1 };
    let mut gerber = Gerber::default();
    for placement in &board.placements {
        for pad in &placement.footprint.pads {
            let (from, to) = placement.pad_layers(pad, board.layer_count);
            let reaches = (from..=to).contains(&side) && (top || side != 0);
            if reaches || (!top && pad.drill.is_some()) {
                gerber.shape(
                    origin,
                    placement.pad_shape(pad),
                    board.rules.mask_expansion,
                    None,
                );
            }
        }
    }
    gerber
}

/// Stencil apertures for surface-mount pads on one side.
fn paste(board: &Board, origin: &Origin, top: bool) -> Gerber {
    let last = board.layer_count - 1;
    let mut gerber = Gerber::default();
    for placement in &board.placements {
        for pad in placement
            .footprint
            .pads
            .iter()
            .filter(|p| p.drill.is_none())
        {
            let (layer, _) = placement.pad_layers(pad, board.layer_count);
            if (top && layer == 0) || (!top && last != 0 && layer == last) {
                gerber.shape(origin, placement.pad_shape(pad), 0, None);
            }
        }
    }
    gerber
}

/// Footprint outlines and reference designators for parts on one side.
fn legend(
    document: &Document,
    origin: &Origin,
    top: bool,
    unsupported: &mut BTreeSet<char>,
) -> Gerber {
    let mut gerber = Gerber::default();
    for placement in document.board.placements.iter().filter(|p| p.bottom != top) {
        for line in &placement.footprint.silk {
            gerber.select(Aperture::Circle(SILK_WIDTH), None);
            gerber.stroke(line.iter().map(|p| origin.point(placement.transform(*p))));
        }
        let Some(component) = document
            .components
            .iter()
            .find(|c| c.id == placement.component)
        else {
            continue;
        };
        let (min, max) = placement.courtyard();
        let anchor = Point::new(
            ((i64::from(min.x) + i64::from(max.x)) / 2) as i32,
            min.y - SILK_WIDTH,
        );
        for stroke in text_strokes(&component.reference, anchor, !top, unsupported) {
            gerber.select(Aperture::Circle(SILK_WIDTH), None);
            gerber.stroke(stroke.iter().map(|p| origin.point(*p)));
        }
    }
    gerber
}

/// Stroke-font glyphs on a 4 × 6 cell, Y up. Each glyph is polylines separated by
/// `;`, each polyline a list of `x,y` pairs.
fn glyph(c: char) -> Option<&'static str> {
    Some(match c {
        '0' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0;1,1 3,5",
        '1' => "1,5 2,6 2,0;1,0 3,0",
        '2' => "0,5 1,6 3,6 4,5 4,4 0,0 4,0",
        '3' => "0,5 1,6 3,6 4,5 4,4 3,3 4,2 4,1 3,0 1,0 0,1;1,3 3,3",
        '4' => "3,0 3,6 0,2 4,2",
        '5' => "4,6 0,6 0,3 3,3 4,2 4,1 3,0 0,0",
        '6' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,2 3,3 0,3",
        '7' => "0,6 4,6 1,0",
        '8' => "1,3 0,4 0,5 1,6 3,6 4,5 4,4 3,3 1,3 0,2 0,1 1,0 3,0 4,1 4,2 3,3",
        '9' => "0,1 1,0 3,0 4,1 4,5 3,6 1,6 0,5 0,4 1,3 4,3",
        'A' => "0,0 0,4 2,6 4,4 4,0;0,3 4,3",
        'B' => "0,3 3,3 4,4 4,5 3,6 0,6 0,0 3,0 4,1 4,2 3,3",
        'C' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1",
        'D' => "0,0 0,6 2,6 4,4 4,2 2,0 0,0",
        'E' => "4,6 0,6 0,0 4,0;0,3 3,3",
        'F' => "4,6 0,6 0,0;0,3 3,3",
        'G' => "4,5 3,6 1,6 0,5 0,1 1,0 3,0 4,1 4,3 2,3",
        'H' => "0,0 0,6;4,0 4,6;0,3 4,3",
        'I' => "1,6 3,6;2,6 2,0;1,0 3,0",
        'J' => "4,6 4,1 3,0 1,0 0,1",
        'K' => "0,0 0,6;4,6 0,2;1,3 4,0",
        'L' => "0,6 0,0 4,0",
        'M' => "0,0 0,6 2,3 4,6 4,0",
        'N' => "0,0 0,6 4,0 4,6",
        'O' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0",
        'P' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3",
        'Q' => "1,0 0,1 0,5 1,6 3,6 4,5 4,1 3,0 1,0;2,2 4,0",
        'R' => "0,0 0,6 3,6 4,5 4,4 3,3 0,3;2,3 4,0",
        'S' => "4,5 3,6 1,6 0,5 0,4 1,3 3,3 4,2 4,1 3,0 1,0 0,1",
        'T' => "0,6 4,6;2,6 2,0",
        'U' => "0,6 0,1 1,0 3,0 4,1 4,6",
        'V' => "0,6 2,0 4,6",
        'W' => "0,6 1,0 2,3 3,0 4,6",
        'X' => "0,0 4,6;0,6 4,0",
        'Y' => "0,6 2,3 4,6;2,3 2,0",
        'Z' => "0,6 4,6 0,0 4,0",
        '-' => "1,3 3,3",
        '_' => "0,0 4,0",
        '+' => "1,3 3,3;2,2 2,4",
        '.' => "2,0 2,0",
        '/' => "0,0 4,6",
        '?' => "0,5 1,6 3,6 4,5 4,4 2,3 2,2;2,0 2,0",
        _ => return None,
    })
}

/// Polylines in board coordinates for `text` centred on `anchor`, with its
/// baseline there. Bottom-side text is mirrored so it reads from below.
fn text_strokes(
    text: &str,
    anchor: Point,
    mirrored: bool,
    unsupported: &mut BTreeSet<char>,
) -> Vec<Vec<Point>> {
    let unit = TEXT_HEIGHT / 6;
    let count = text.chars().count() as i32;
    let width = (count * 5 - 1).max(0) * unit;
    let mut strokes = vec![];
    for (i, c) in text.chars().enumerate() {
        let c = c.to_ascii_uppercase();
        if c == ' ' {
            continue;
        }
        let shape = glyph(c).unwrap_or_else(|| {
            unsupported.insert(c);
            glyph('?').unwrap_or_default()
        });
        let left = i as i32 * 5;
        for polyline in shape.split(';') {
            strokes.push(
                polyline
                    .split(' ')
                    .filter_map(|pair| {
                        let (x, y) = pair.split_once(',')?;
                        let (x, y) = (x.parse::<i32>().ok()?, y.parse::<i32>().ok()?);
                        let offset = (left + x) * unit - width / 2;
                        let offset = if mirrored { -offset } else { offset };
                        Some(Point::new(anchor.x + offset, anchor.y - y * unit))
                    })
                    .collect(),
            );
        }
    }
    strokes
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum Hole {
    Via,
    Component,
}
/// Drill hits grouped by (diameter, kind), in output coordinates.
type Drills = BTreeMap<(i32, Hole), Vec<(i64, i64)>>;

fn drills(board: &Board, origin: &Origin) -> (Drills, Drills) {
    let (mut plated, mut unplated) = (Drills::new(), Drills::new());
    for via in &board.vias {
        plated
            .entry((via.drill, Hole::Via))
            .or_default()
            .push(origin.point(via.at));
    }
    for placement in &board.placements {
        for pad in &placement.footprint.pads {
            if let Some(drill) = pad.drill {
                let target = if pad.plated {
                    &mut plated
                } else {
                    &mut unplated
                };
                target
                    .entry((drill, Hole::Component))
                    .or_default()
                    .push(origin.point(placement.transform(pad.at)));
            }
        }
    }
    (plated, unplated)
}

/// Excellon drill file in the XNC subset, with decimal millimetre coordinates.
fn excellon(holes: &Drills, file_function: &str, plating: &str) -> String {
    let mut out = String::from("M48\n");
    let _ = writeln!(out, "; DRILL file {{eCAD {}}}", env!("CARGO_PKG_VERSION"));
    out.push_str("; FORMAT={-:-/ absolute / metric / decimal}\n");
    let _ = writeln!(
        out,
        "; #@! TF.GenerationSoftware,eCAD,Schematic and PCB Studio,{}",
        env!("CARGO_PKG_VERSION")
    );
    let _ = writeln!(out, "; #@! TF.FileFunction,{file_function}");
    out.push_str("FMAT,2\nMETRIC\n");
    for (tool, (diameter, kind)) in holes.keys().enumerate() {
        let kind = match kind {
            Hole::Via => "ViaDrill",
            Hole::Component => "ComponentDrill",
        };
        let _ = writeln!(out, "; #@! TA.AperFunction,{plating},{kind}");
        let _ = writeln!(out, "T{}C{}", tool + 1, millimetres(*diameter));
    }
    // Absolute coordinates are the XNC default; readers reject G90 after the header.
    out.push_str("%\nG05\n");
    for (tool, hits) in holes.values().enumerate() {
        let _ = writeln!(out, "T{}", tool + 1);
        for (x, y) in hits {
            // Output coordinates are nanometres; drill files use micrometre precision.
            let _ = writeln!(
                out,
                "X{}Y{}",
                millimetres(x.div_euclid(1000)),
                millimetres(y.div_euclid(1000))
            );
        }
    }
    out.push_str("M30\n");
    out
}

/// Bitwise CRC-32 (IEEE), as used by zip.
pub fn crc32(bytes: &[u8]) -> u32 {
    let mut crc = u32::MAX;
    for &byte in bytes {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = if crc & 1 == 1 {
                (crc >> 1) ^ 0xEDB8_8320
            } else {
                crc >> 1
            };
        }
    }
    !crc
}

/// A zip archive with stored (uncompressed) entries and a fixed 1980-01-01
/// timestamp, so identical inputs give identical archives.
pub fn zip<'a>(entries: impl IntoIterator<Item = (&'a str, &'a [u8])>) -> Vec<u8> {
    const DOS_DATE: u16 = (1 << 5) | 1;
    let mut out = vec![];
    let mut directory = vec![];
    let mut count: u16 = 0;
    let push16 = |v: &mut Vec<u8>, x: u16| v.extend_from_slice(&x.to_le_bytes());
    let push32 = |v: &mut Vec<u8>, x: u32| v.extend_from_slice(&x.to_le_bytes());
    for (name, data) in entries {
        let offset = out.len() as u32;
        let (crc, size, name_len) = (crc32(data), data.len() as u32, name.len() as u16);
        for (record, signature) in [(&mut out, 0x0403_4b50), (&mut directory, 0x0201_4b50)] {
            push32(record, signature);
            if signature == 0x0201_4b50 {
                push16(record, 20); // made by
            }
            push16(record, 10); // needed to extract
            push16(record, 0); // flags
            push16(record, 0); // stored
            push16(record, 0); // time
            push16(record, DOS_DATE);
            push32(record, crc);
            push32(record, size);
            push32(record, size);
            push16(record, name_len);
            push16(record, 0); // extra
            if signature == 0x0201_4b50 {
                push16(record, 0); // comment
                push16(record, 0); // disk
                push16(record, 0); // internal attributes
                push32(record, 0); // external attributes
                push32(record, offset);
            }
            record.extend_from_slice(name.as_bytes());
        }
        out.extend_from_slice(data);
        count += 1;
    }
    let (start, length) = (out.len() as u32, directory.len() as u32);
    out.extend_from_slice(&directory);
    push32(&mut out, 0x0605_4b50);
    push16(&mut out, 0);
    push16(&mut out, 0);
    push16(&mut out, count);
    push16(&mut out, count);
    push32(&mut out, length);
    push32(&mut out, start);
    push16(&mut out, 0);
    out
}

/// The entries of a zip archive, name and stored size, read from its central
/// directory in archive order. Empty when `archive` is not a zip this module
/// could have written (no end record, or a directory that runs off the end).
pub fn zip_entries(archive: &[u8]) -> Vec<(String, usize)> {
    let u16_at = |i: usize| -> Option<usize> {
        Some(u16::from_le_bytes(archive.get(i..i + 2)?.try_into().ok()?) as usize)
    };
    let u32_at = |i: usize| -> Option<usize> {
        Some(u32::from_le_bytes(archive.get(i..i + 4)?.try_into().ok()?) as usize)
    };
    let read = || -> Option<Vec<(String, usize)>> {
        let end = archive.len().checked_sub(22)?;
        if u32_at(end)? != 0x0605_4b50 {
            return None;
        }
        let mut entry = u32_at(end + 16)?;
        let mut out = vec![];
        for _ in 0..u16_at(end + 10)? {
            if u32_at(entry)? != 0x0201_4b50 {
                return None;
            }
            let name_len = u16_at(entry + 28)?;
            let name = archive.get(entry + 46..entry + 46 + name_len)?;
            out.push((String::from_utf8_lossy(name).into_owned(), u32_at(entry + 24)?));
            entry += 46 + name_len + u16_at(entry + 30)? + u16_at(entry + 32)?;
        }
        Some(out)
    };
    read().unwrap_or_default()
}
