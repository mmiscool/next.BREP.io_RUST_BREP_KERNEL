//! The BOARD in 3D: the substrate solid, the copper on each layer, and the
//! plated via barrels through it — derived from the document's `pcb` block,
//! never a feature.
//!
//! # Why it is derived and not a feature
//!
//! A board's geometry is a READING of eCAD's board, the way a part's declared
//! connection points are a reading of its `ports` block (the 2026-09-20 ports
//! record). It has no parameters a user would edit in a dialog, no place in the
//! history order — nothing downstream can reference it, because nothing is
//! ordered against it — and it must follow every track the router lays down
//! without an undo step of its own. So it is rebuilt at the TAIL, after the run
//! ([`EngineState::finish_apply`]) and after any write to the `pcb` block, from
//! the block alone. The PCB workbench still creates no feature of its own.
//!
//! It is built HERE rather than in the kernel for two reasons: the kernel is
//! told only that a `pcb` block is PRESENT (`HistoryRequest::pcb_present`) and
//! deliberately neither reads nor rewrites it, and this crate already holds
//! both `brep_ecad_core` and the scene. The cost is stated plainly below.
//!
//! # DISPLAY geometry, not kernel solids
//!
//! Every body this module inserts is a synthesized [`SolidDisplay`] with
//! `source_handle == 0` — triangles, not B-rep. That is the cheap thing, built
//! first and on purpose: it is visible, correctly placed, coloured, listed in
//! the Scene tree, hideable and pickable, and it costs one GPU buffer set per
//! LAYER however many track segments the layer carries. What it cannot do:
//!
//!   * it cannot be cut against — no boolean takes it as a tool or a target;
//!   * it has no mass properties and no measurable exact topology;
//!   * a drawing sheet that draws it falls back to MESH hidden-line for the
//!     whole view, because the analytic pass needs every drawn solid's B-rep;
//!   * it is not what STEP export writes: the export builds the board again as
//!     exact solids ([`step_solids`]), handed straight to the writer and never
//!     entered in the scene or the kernel's registry.
//!
//! Real SCENE solids would have to be minted on the RUNNER thread, where the
//! kernel's solid registry lives, which means a kernel tail, which means the
//! kernel reading eCAD's block. That is a layering decision, not a day's work.
//!
//! # The stack
//!
//! Board coordinates are integer micrometres with +Y DOWN; BREP is millimetres
//! with +Y up, so every point is `x / 1000, -y / 1000` — the same conversion
//! the placement writer makes (`ecad_parts::follow_sheet`), which is what keeps
//! a part's pins over its pads. `board::layer_z` puts F.Cu's plane at 0, B.Cu's
//! at `-SUBSTRATE_THICKNESS` and the inner layers evenly between, so a 2-layer
//! and a 4-layer board come out of the same arithmetic. The substrate fills
//! `[-SUBSTRATE_THICKNESS, 0]`; copper stands `COPPER_THICKNESS` proud of its
//! own plane, outward on the two outer layers and centred on an inner one.

use super::*;
use brep_ecad_core::board::{self, Board, CopperItem, Shape};
use brep_ecad_core::Netlist;
use brep_kernel::{DisplaySolidPayload, Mesh, Vec3};
use std::collections::BTreeMap;

/// Every board body's name leads with this, so the scene, the Scene tree and
/// the stale sweep can tell one apart from a feature's solid. No `:` — that is
/// the occurrence namespace an assembly reads.
pub const BOARD_SOLID_PREFIX: &str = "PCB_";
/// The substrate body's scene name.
pub const SUBSTRATE_SOLID: &str = "PCB_Board";
/// The via-barrel body's scene name.
pub const VIAS_SOLID: &str = "PCB_Vias";

/// Separates a board body's name from the ROLE of one of its faces —
/// `PCB_F.Cu/GND` is the GND copper on F.Cu, `PCB_F.Mask/Opening` is the bare
/// metal the mask leaves open. `:` is the assembly occurrence namespace and `|`
/// is the kernel's edge-name convention, so neither may be borrowed here.
///
/// A face name is a metadata key and the label the UI shows for what the
/// pointer is over, which is what makes net colouring readable WITHOUT a
/// legend: the colour says "these are one net" and hovering says which.
const FACE_SEPARATOR: char = '/';
/// The face name of copper on an island that carries no single schematic net.
const NO_NET_FACE: &str = "(no net)";
/// The face name of the mask's bare-metal openings.
const OPENING_FACE: &str = "Opening";
/// The face name of the mask lacquer itself.
const LACQUER_FACE: &str = "Lacquer";
/// The face name of one of the substrate's side walls, numbered from 1 —
/// `PCB_Board/Side 3`. One wall per side of the outline between two corners,
/// so a rectangular board has four, as a box does.
const SIDE_FACE: &str = "Side";
/// A board outline corner turning by more than this, in radians, gets a
/// vertical display edge down the wall. A corner a polygon rounds with many
/// short sides turns a little at each and gets none, so a rounded board is not
/// combed with verticals.
const CORNER_TURN: f64 = 0.5;

/// The substrate's cut EDGE: the pale fibreglass-and-epoxy core a board shows
/// where it was routed out of its panel, since the green is only on its two
/// faces. Drawn in the green, the edge read as a shadow of the top face and a
/// user looking for the board's thickness did not find it.
const EDGE_COLOR: [f32; 3] = [0xb8 as f32 / 255., 0xa8 as f32 / 255., 0x7a as f32 / 255.];
/// FR-4 green.
const SUBSTRATE_COLOR: [f32; 3] = [0x1c as f32 / 255., 0x5e as f32 / 255., 0x38 as f32 / 255.];
/// Bare copper.
const COPPER_COLOR: [f32; 3] = [0xc0 as f32 / 255., 0x7b as f32 / 255., 0x38 as f32 / 255.];
/// Solder-mask lacquer — darker and glossier than the bare substrate, so a
/// board with the mask shown and one with it hidden do not read the same.
const MASK_COLOR: [f32; 3] = [0x12 as f32 / 255., 0x44 as f32 / 255., 0x26 as f32 / 255.];
/// Silkscreen ink.
const SILK_COLOR: [f32; 3] = [0xec as f32 / 255., 0xec as f32 / 255., 0xe4 as f32 / 255.];

/// How far the mask lacquer stands over the copper it covers, in board
/// micrometres. A CONSTANT and not a [`board::DesignRules`] field, for the
/// reason `SUBSTRATE_THICKNESS` is one: a new always-serialized field puts
/// every `pcb` block written before it out of step with its editor.
const MASK_OVER_COPPER: i32 = 20;
/// How thick the silkscreen ink is drawn, on top of the lacquer.
const SILK_THICKNESS: i32 = 15;

/// Facets per HALF-turn on a round copper end or a via barrel — so eight round
/// a full circle. Small on purpose: a track end is a fraction of a millimetre
/// across and a dense board has thousands of them, so this number multiplies
/// the whole scene's triangle count. The measured figures in the validation
/// record are at this value.
const ARC_FACETS: usize = 4;

/// Board micrometres to millimetres, +Y down to +Y up.
fn xy(p: brep_ecad_core::Point) -> [f64; 2] {
    [f64::from(p.x) / 1000., -f64::from(p.y) / 1000.]
}

/// Millimetres from board micrometres, for a length rather than a position.
fn mm(v: i32) -> f64 {
    f64::from(v) / 1000.
}

/// A triangle soup being built, with flat normals and one face id per triangle.
/// A planar piece — a quad, a cap — shares its corners between its triangles.
/// Triangles are emitted face by face, because the display builder groups each
/// face's triangles by their first contiguous run.
#[derive(Default)]
struct Soup {
    mesh: Mesh,
}

impl Soup {
    fn vertex(&mut self, p: [f64; 3], n: [f64; 3]) -> u32 {
        self.mesh.positions.extend(p);
        self.mesh.normals.extend(n);
        (self.mesh.positions.len() / 3 - 1) as u32
    }

    /// A planar quad as two triangles that SHARE its four corners. A hovered
    /// face is outlined by the mesh edges only one of its triangles uses, so
    /// a corner per triangle would outline the quad's diagonal as well.
    fn quad(&mut self, a: [f64; 3], b: [f64; 3], c: [f64; 3], d: [f64; 3], face: u32) {
        let n = normal(a, b, c);
        let [ia, ib, ic, id] = [a, b, c, d].map(|p| self.vertex(p, n));
        self.mesh.indices.extend([ia, ib, ic, ia, ic, id]);
        self.mesh.face_ids.extend([face, face]);
    }

    /// The walls of the prism `loop_xy` sweeps between `lo` and `hi`. The loop
    /// is counter-clockwise, so the walls face outwards: each quad runs
    /// `a -> b` along the bottom, and the right of that direction — outside a
    /// counter-clockwise loop — is where its normal points
    /// (`every_wall_faces_out_of_the_copper_and_the_board` pins it).
    fn walls(&mut self, loop_xy: &[[f64; 2]], lo: f64, hi: f64, face: u32) {
        for i in 0..loop_xy.len() {
            let a = loop_xy[i];
            let b = loop_xy[(i + 1) % loop_xy.len()];
            self.quad(
                [a[0], a[1], lo],
                [b[0], b[1], lo],
                [b[0], b[1], hi],
                [a[0], a[1], hi],
                face,
            );
        }
    }

    /// The wall of one open run of an outline — `walls` without the quad that
    /// closes the loop — so each side of the board is a face of its own.
    fn walls_open(&mut self, run: &[[f64; 2]], lo: f64, hi: f64, face: u32) {
        for pair in run.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            self.quad([a[0], a[1], lo], [b[0], b[1], lo], [b[0], b[1], hi], [a[0], a[1], hi], face);
        }
    }

    /// One cap of that prism, at `z`, facing `up` or down.
    ///
    /// Its triangles share the loop's corners, for the reason [`Self::quad`]'s
    /// do: hovering the cap outlines the loop, not its triangulation.
    fn cap(&mut self, loop_xy: &[[f64; 2]], triangles: &[[usize; 3]], z: f64, up: bool, face: u32) {
        let n = if up { [0., 0., 1.] } else { [0., 0., -1.] };
        let first = self.mesh.positions.len() as u32 / 3;
        for p in loop_xy {
            self.vertex([p[0], p[1], z], n);
        }
        for &[a, b, c] in triangles {
            let (b, c) = if up { (b, c) } else { (c, b) };
            self.mesh.indices.extend([a, b, c].map(|i| first + i as u32));
            self.mesh.face_ids.push(face);
        }
    }

    /// One piece of a zone's fill swept between `lo` and `hi`, as ONE face: its
    /// caps from the even-odd trapezoids of its rings (`zones::fill_triangles`,
    /// which draws the holes a fan or plain ear clipping cannot), and walls round
    /// the outer ring and every hole, each facing out of the copper.
    fn slab(&mut self, piece: &board::FillPiece, lo: f64, hi: f64, face: u32) {
        let rings: Vec<&[brep_ecad_core::Point]> = std::iter::once(piece.outer.as_slice())
            .chain(piece.holes.iter().map(Vec::as_slice))
            .collect();
        // The caps SHARE their corners: the triangles are watertight, and a
        // hovered face is outlined by the edges only one triangle uses, so a
        // corner per triangle would outline every triangle of the fill.
        let triangles = board::zones::fill_triangles(&rings);
        for (z, up) in [(hi, true), (lo, false)] {
            let n = if up { [0., 0., 1.] } else { [0., 0., -1.] };
            let mut shared: std::collections::HashMap<(u64, u64), u32> = std::collections::HashMap::new();
            for &[a, b, c] in &triangles {
                let ccw = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0) < 0.;
                // Board +y is down, so a turn that is clockwise there is
                // counter-clockwise here.
                let (b, c) = if ccw == up { (b, c) } else { (c, b) };
                let corners = [a, b, c].map(|(x, y)| {
                    *shared
                        .entry((x.to_bits(), y.to_bits()))
                        .or_insert_with(|| self.vertex([x / 1000., -y / 1000., z], n))
                });
                self.mesh.indices.extend(corners);
                self.mesh.face_ids.push(face);
            }
        }
        for (k, ring) in rings.iter().enumerate() {
            let mut ring: Vec<[f64; 2]> = ring.iter().copied().map(xy).collect();
            if (signed_area2(&ring) < 0.) == (k == 0) {
                ring.reverse();
            }
            self.walls(&ring, lo, hi, face);
        }
    }

    /// A whole closed prism — bottom cap, top cap, walls — as ONE face.
    fn prism(&mut self, loop_xy: &[[f64; 2]], lo: f64, hi: f64, face: u32) {
        let triangles = fan_or_ears(loop_xy);
        self.cap(loop_xy, &triangles, lo, false, face);
        self.cap(loop_xy, &triangles, hi, true, face);
        self.walls(loop_xy, lo, hi, face);
    }
}

fn normal(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if len > 1e-12 {
        [n[0] / len, n[1] / len, n[2] / len]
    } else {
        [0., 0., 1.]
    }
}

/// Twice the signed area of `loop_xy` — positive counter-clockwise.
fn signed_area2(loop_xy: &[[f64; 2]]) -> f64 {
    let mut sum = 0.;
    for i in 0..loop_xy.len() {
        let a = loop_xy[i];
        let b = loop_xy[(i + 1) % loop_xy.len()];
        sum += a[0] * b[1] - b[0] * a[1];
    }
    sum
}

/// Triangulate a simple polygon given counter-clockwise, as index triples.
///
/// Ear clipping, because a board outline is an arbitrary simple polygon and a
/// fan is only right for a convex one. Copper shapes ARE convex and clip in one
/// pass, so they pay for the generality in comparisons rather than in passes.
/// A polygon this cannot finish (self-crossing, or a hole the outline was drawn
/// with) falls back to a fan, which draws something wrong rather than nothing —
/// the board is a picture here, and a missing board reads as a bug in the
/// router rather than in its outline.
fn fan_or_ears(loop_xy: &[[f64; 2]]) -> Vec<[usize; 3]> {
    let n = loop_xy.len();
    if n < 3 {
        return Vec::new();
    }
    let fan = || (1..n - 1).map(|i| [0, i, i + 1]).collect::<Vec<_>>();
    let mut remaining: Vec<usize> = (0..n).collect();
    let mut out = Vec::with_capacity(n - 2);
    let convex = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| {
        (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    };
    let inside = |a: [f64; 2], b: [f64; 2], c: [f64; 2], p: [f64; 2]| {
        convex(a, b, p) >= 0. && convex(b, c, p) >= 0. && convex(c, a, p) >= 0.
    };
    // Each pass must clip at least one ear; a pass that clips none is a polygon
    // this cannot handle, and the fan takes over.
    while remaining.len() > 3 {
        let before = remaining.len();
        let mut i = 0;
        while remaining.len() > 3 && i < remaining.len() {
            let (ia, ib, ic) = (
                remaining[(i + remaining.len() - 1) % remaining.len()],
                remaining[i],
                remaining[(i + 1) % remaining.len()],
            );
            let (a, b, c) = (loop_xy[ia], loop_xy[ib], loop_xy[ic]);
            if convex(a, b, c) <= 0. {
                i += 1;
                continue;
            }
            if remaining
                .iter()
                .any(|&p| p != ia && p != ib && p != ic && inside(a, b, c, loop_xy[p]))
            {
                i += 1;
                continue;
            }
            out.push([ia, ib, ic]);
            remaining.remove(i);
        }
        if remaining.len() == before {
            return fan();
        }
    }
    out.push([remaining[0], remaining[1], remaining[2]]);
    out
}

/// A copper primitive's outline in millimetres, counter-clockwise: a rectangle
/// as four corners, a capsule as a stadium closed by two [`ARC_FACETS`] arcs
/// (a circle when its ends coincide).
fn shape_loop(shape: &Shape) -> Vec<[f64; 2]> {
    match *shape {
        Shape::Rect { min, max } => {
            let (a, b) = (xy(min), xy(max));
            let (x0, x1) = (a[0].min(b[0]), a[0].max(b[0]));
            let (y0, y1) = (a[1].min(b[1]), a[1].max(b[1]));
            vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
        }
        Shape::Capsule { a, b, radius } => {
            let (a, b, r) = (xy(a), xy(b), mm(radius).max(1e-4));
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let len = (dx * dx + dy * dy).sqrt();
            // A circle: one closed ring about the single centre.
            if len <= 1e-9 {
                return (0..ARC_FACETS * 2)
                    .map(|i| {
                        let t = std::f64::consts::TAU * i as f64 / (ARC_FACETS * 2) as f64;
                        [a[0] + r * t.cos(), a[1] + r * t.sin()]
                    })
                    .collect();
            }
            // A stadium: the half-ring round `b`, then the half-ring round `a`.
            // `base` is the direction a->b, so the arcs sweep the far side of
            // each end and the two straights close the loop.
            let base = dy.atan2(dx) - std::f64::consts::FRAC_PI_2;
            let mut out = Vec::with_capacity(2 * (ARC_FACETS + 1));
            for (centre, start) in [(b, base), (a, base + std::f64::consts::PI)] {
                for i in 0..=ARC_FACETS {
                    let t = start + std::f64::consts::PI * i as f64 / ARC_FACETS as f64;
                    out.push([centre[0] + r * t.cos(), centre[1] + r * t.sin()]);
                }
            }
            out
        }
    }
}

/// The z span of copper on `layer`: proud of the board on the two outer layers,
/// centred on its own plane on an inner one (where there is no "outward").
fn copper_span(layer: u8, layer_count: u8) -> (f64, f64) {
    let plane = mm(board::layer_z(layer, layer_count));
    let t = mm(board::COPPER_THICKNESS);
    let last = layer_count.saturating_sub(1);
    if layer == 0 {
        (plane, plane + t)
    } else if layer >= last {
        (plane - t, plane)
    } else {
        (plane - t * 0.5, plane + t * 0.5)
    }
}

/// The OUTER plane of one side of the board and the direction that is "out of
/// the board" there: the top side's plane is 0 and its outward `+1`, the bottom
/// side's is `-SUBSTRATE_THICKNESS` and `-1`. Everything above the copper — the
/// mask lacquer, its openings, the silkscreen — is stacked from here, so the
/// two sides come out of one arithmetic the way the copper layers do.
fn side_frame(top: bool) -> (f64, f64) {
    if top {
        (0., 1.)
    } else {
        (mm(-board::SUBSTRATE_THICKNESS), -1.)
    }
}

/// A slab on one side of the board, `from`..`to` micrometres OUT from that
/// side's plane, as an ordered `(lo, hi)` millimetre pair — [`Soup::prism`]
/// wants its caps the right way up, and the bottom side's "out" is `-z`.
fn side_span(top: bool, from: i32, to: i32) -> (f64, f64) {
    let (plane, out) = side_frame(top);
    let (a, b) = (plane + out * mm(from), plane + out * mm(to));
    (a.min(b), a.max(b))
}

/// The top of the mask lacquer, in micrometres out from its side's plane: over
/// the copper, because the lacquer covers the copper it is not open over.
const LACQUER_TOP: i32 = board::COPPER_THICKNESS + MASK_OVER_COPPER;

/// One copper primitive's shape grown by `grow` on every side — a pad's solder
/// mask OPENING, which [`board::DesignRules::mask_expansion`] sizes. The same
/// growth `fabrication`'s Gerber writer applies when it flashes the aperture.
fn grown(shape: Shape, grow: i32) -> Shape {
    match shape {
        Shape::Rect { min, max } => Shape::Rect {
            min: brep_ecad_core::Point::new(min.x - grow, min.y - grow),
            max: brep_ecad_core::Point::new(max.x + grow, max.y + grow),
        },
        Shape::Capsule { a, b, radius } => Shape::Capsule {
            a,
            b,
            radius: radius + grow,
        },
    }
}

/// The board outline in millimetres, wound counter-clockwise for the caps and
/// walls. eCAD stores it in a +Y-down frame, so a polygon wound one way there
/// is wound the other here. `None` for an outline that is not a polygon.
fn oriented_outline(board: &Board) -> Option<Vec<[f64; 2]>> {
    let mut outline: Vec<[f64; 2]> = board.outline.iter().copied().map(xy).collect();
    if outline.len() < 3 {
        return None;
    }
    if signed_area2(&outline) < 0. {
        outline.reverse();
    }
    Some(outline)
}

/// The scene name of one side's solder mask, KiCad's own (`F.Mask` / `B.Mask`).
fn mask_solid(top: bool) -> String {
    format!("{BOARD_SOLID_PREFIX}{}.Mask", if top { "F" } else { "B" })
}

/// The scene name of one side's silkscreen, KiCad's own (`F.SilkS` / `B.SilkS`).
fn silk_solid(top: bool) -> String {
    format!("{BOARD_SOLID_PREFIX}{}.SilkS", if top { "F" } else { "B" })
}

/// The RAILS that have a conventional colour, the spellings of each this
/// recognises, and that colour — consulted before the hash by [`net_color`].
///
/// **The convention is the ATX supply wire code plus the test-lead colours it
/// grew out of**: ground black, the positive supply red, +3.3 V orange, +12 V
/// yellow, −12 V blue. The CHOICE OF HUE is therefore not this project's
/// invention. A wire colour has no hex, though, so the exact point inside each
/// band IS chosen here — and it was chosen by MEASUREMENT, not by taste.
///
/// **Matching is EXACT, case-insensitively, against a spelling in this table.**
/// That is forgiving about the two things eCAD tools actually disagree on —
/// the case of a name and whether a voltage rail carries a `+` — and reckless
/// about nothing: there is no prefix strip, no suffix strip and no substring
/// test, so `AGND`, `DGND`, `GND1`, `VCC_IO` and `GNDBREAK` all fall through to
/// the hash. A board that splits its ground names it split on purpose, and
/// painting `AGND` and `DGND` the one ground colour would put back the very
/// defect this table exists to fix.
///
/// **A row is a RAIL, not a net, so the map is deliberately many-to-one.** A
/// board carrying `VCC` and `+5V` as two DISTINCT nets draws both in the
/// supply red. That is the price of naming the rail rather than the net, and
/// the face name — `PCB_F.Cu/VCC` against `PCB_F.Cu/+5V` — is what separates
/// them, exactly as it does for two hashed nets that collide.
const CURATED_RAILS: &[(&[&str], u32)] = &[
    // Ground is black on a test lead, in an ATX loom and in an automotive
    // harness alike. Drawn as a near-black GREY, not `#000`: the viewport
    // background is `0x0b0d10` and a true black track loses its silhouette
    // where it crosses the board edge.
    (&["GND", "VSS", "0V"], 0x3a3a40),
    // The positive supply is red by the same three conventions, and ATX's red
    // IS +5 V — `VCC` was a 5 V rail before it was a name, so the supply names
    // and the 5 V spellings are one row rather than two shades of one red.
    (&["VCC", "VDD", "+5V", "5V"], 0xcc1111),
    // ATX orange, unmodified — and the row that had to be MEASURED hardest,
    // because bare copper is itself an orange. This clears the `(no net)`
    // default by ΔE00 12.3, which is the smallest gap in the whole set and is
    // reported rather than hidden; a duller orange does not. `#e07000` sits
    // ΔE00 8.0 from bare copper and a 3.3 V track drawn in it would read as
    // unnetted copper.
    (&["+3V3", "3V3", "+3.3V", "3.3V"], 0xff8000),
    // ATX yellow.
    (&["+12V", "12V"], 0xe8d400),
    // ATX blue, its only negative rail. `VEE` is NOT here: it means "the
    // negative supply" at no particular voltage, and giving it this blue would
    // be inventing a convention rather than following one.
    (&["-12V"], 0x1b4fc8),
];

/// A stable colour for a NET: the conventional colour of the RAIL it names if
/// it names one, and otherwise the app's own name-hashed palette — the same
/// function that gives an unpainted BODY its colour, asked about the net's name
/// rather than the body's.
///
/// One function either way, so a net is the same colour on F.Cu as on B.Cu.
/// Two nets share a colour only if they are the same net OR they name the same
/// rail in [`CURATED_RAILS`]; the hash gives no separation guarantee at all,
/// which is why the rails that MUST be told apart are not left to it.
fn net_color(net: &str) -> [f32; 3] {
    curated_rail_color(net)
        .unwrap_or_else(|| crate::color::solid_color_srgb(net))
        .map(|c| c as f32)
}

/// The conventional colour of the rail `net` names, or `None` for a net that
/// names no rail in [`CURATED_RAILS`].
fn curated_rail_color(net: &str) -> Option<[f64; 3]> {
    CURATED_RAILS.iter().find_map(|(spellings, hex)| {
        spellings
            .iter()
            .any(|spelling| spelling.eq_ignore_ascii_case(net))
            .then(|| crate::color::hex_to_srgb(*hex))
    })
}

/// The colour a board body or one of its FACES draws in when the metadata store
/// has none of its own for it — the fallback `sync_colors_from_metadata`
/// reaches for, since a derived body has no producing feature to have been
/// coloured by. `None` for every name that is not a board body's or a face of
/// one; a stored `color` attribute still wins over all of it.
pub fn default_color(name: &str) -> Option<[f32; 3]> {
    let rest = name.strip_prefix(BOARD_SOLID_PREFIX)?;
    // `PCB_<body>/<role>` is a face; anything else is a body. Only the bodies
    // that need more than one colour name their faces at all — the barrels'
    // Barrel and the silkscreen's Ink carry no prefix and inherit their body.
    // The split is on the FIRST separator and no body's name contains one, so a
    // net whose own name does round-trips.
    let Some((body, role)) = rest.split_once(FACE_SEPARATOR) else {
        return Some(body_color(rest));
    };
    // A face that would draw in its BODY'S OWN colour carries NO override.
    // `face_style_palette` re-arms the renderer's whole-mesh fast path only for
    // a solid with no per-face colours at all, so a board with no nets — every
    // document nobody has drawn a schematic for — must not pay a style buffer
    // and a per-face draw for one face that changes nothing.
    let color = face_color(body, role);
    (color != body_color(body)).then_some(color)
}

/// The default colour of a board body, named without [`BOARD_SOLID_PREFIX`].
fn body_color(body: &str) -> [f32; 3] {
    if body == "Board" {
        SUBSTRATE_COLOR
    } else if body.ends_with(".Mask") {
        MASK_COLOR
    } else if body.ends_with(".SilkS") {
        SILK_COLOR
    } else {
        COPPER_COLOR
    }
}

/// The default colour of one face of a board body. The BODY decides how to read
/// the role, so a net that happens to be called `Opening` is still a net.
fn face_color(body: &str, role: &str) -> [f32; 3] {
    if body == "Board" {
        if role.starts_with(SIDE_FACE) {
            EDGE_COLOR
        } else {
            SUBSTRATE_COLOR
        }
    } else if body.ends_with(".Mask") {
        if role == OPENING_FACE {
            COPPER_COLOR
        } else {
            MASK_COLOR
        }
    } else if role == NO_NET_FACE {
        COPPER_COLOR
    } else {
        net_color(role)
    }
}

/// One built body: its scene name, its colour, and the payload to display.
struct Body {
    name: String,
    color: [f32; 3],
    payload: DisplaySolidPayload,
    /// The index of the first DISPLAY-ONLY edge in `payload.edges`: the copper
    /// outline, which is drawn and not picked (see [`copper_outline`]).
    aux_from: usize,
}

fn body(name: String, color: [f32; 3], faces: Vec<String>, soup: Soup) -> Option<Body> {
    if soup.mesh.indices.is_empty() {
        return None;
    }
    Some(Body {
        name,
        color,
        aux_from: usize::MAX,
        payload: DisplaySolidPayload {
            faces: faces
                .into_iter()
                .enumerate()
                .map(|(i, name)| (i as u64, Some(name)))
                .collect(),
            mesh: soup.mesh,
            edges: Vec::new(),
            vertices: Vec::new(),
            chord_tolerance: 0.,
        },
    })
}

/// The substrate: the outline swept through the board's thickness, with its top
/// face, its bottom face and its edge named apart so a user can pick one.
///
/// It is NOT drilled. Every via barrel below sits inside solid substrate rather
/// than in a hole, which is right where it is and wrong about what surrounds
/// it; cutting the holes wants a hole-aware cap triangulation this does not
/// have. Hiding `PCB_Board` in the Scene tree shows the barrels. So there is no
/// hole rim to draw either: the drilled board is the STEP export's.
fn substrate(board: &Board) -> Option<Body> {
    let outline = oriented_outline(board)?;
    let triangles = fan_or_ears(&outline);
    let (lo, hi) = (mm(-board::SUBSTRATE_THICKNESS), 0.);
    let n = outline.len();
    // The outline's SIDES: the runs between its corners. An outline with no
    // corner at all — a round board — is one side all the way round.
    let mut at = corners(&outline);
    if at.is_empty() {
        at.push(0);
    }
    let sides: Vec<Vec<[f64; 2]>> = (0..at.len())
        .map(|k| {
            let (from, to) = (at[k], at[(k + 1) % at.len()]);
            let steps = (to + n - from - 1) % n + 1;
            (0..=steps).map(|i| outline[(from + i) % n]).collect()
        })
        .collect();
    let mut soup = Soup::default();
    soup.cap(&outline, &triangles, hi, true, 0);
    soup.cap(&outline, &triangles, lo, false, 1);
    for (k, side) in sides.iter().enumerate() {
        soup.walls_open(side, lo, hi, 2 + k as u32);
    }
    // Named under the body like every other board face, so the walls can
    // carry a colour of their own ([`default_color`]) and hovering one says
    // whose it is.
    let face = |role: String| format!("{SUBSTRATE_SOLID}{FACE_SEPARATOR}{role}");
    let side = |k: usize| face(format!("{SIDE_FACE} {}", k + 1));
    let mut faces = vec![face("Top".into()), face("Bottom".into())];
    faces.extend((0..sides.len()).map(side));
    let mut body = body(SUBSTRATE_SOLID.to_string(), SUBSTRATE_COLOR, faces.clone(), soup)?;
    // The edges a box has: each side's top and bottom edge, and a vertical
    // where two sides meet — named, in the kernel's `faceA|faceB` spelling, by
    // the two faces each one divides.
    let mut push = |name: String, points: Vec<Vec3>| {
        let id = body.payload.edges.len() as u64;
        body.payload.edges.push((id, Some(name), points));
    };
    for (k, run) in sides.iter().enumerate() {
        for (cap, z) in [(&faces[0], hi), (&faces[1], lo)] {
            let points = run.iter().map(|p| Vec3::new(p[0], p[1], z)).collect();
            push(format!("{cap}|{}", side(k)), points);
        }
    }
    if sides.len() > 1 {
        for k in 0..sides.len() {
            let p = outline[at[k]];
            let before = (k + sides.len() - 1) % sides.len();
            push(
                format!("{}|{}", side(before), side(k)),
                vec![Vec3::new(p[0], p[1], hi), Vec3::new(p[0], p[1], lo)],
            );
        }
    }
    Some(body)
}

/// The indices of the outline's CORNERS: every vertex where the outline turns
/// by more than [`CORNER_TURN`].
fn corners(outline: &[[f64; 2]]) -> Vec<usize> {
    let n = outline.len();
    (0..n)
        .filter(|&i| {
            let (a, b, c) = (outline[(i + n - 1) % n], outline[i], outline[(i + 1) % n]);
            let (u, v) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
            let turn = (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1]);
            turn.abs() > CORNER_TURN
        })
        .collect()
}

/// The display edges of one copper layer: where its copper STOPS — the
/// boundary of the union of every track, pad, via ring and fill on the layer,
/// so a bend in a track, a track landing on a pad and a pad inside a fill draw
/// no seam ([`outlines::union_boundary`]).
///
/// On ONE plane, the copper's outer face (`z`): the other is 35 µm away and
/// lands on the same pixels from every angle but edge-on, and drawing it too
/// doubled the edge instances the frame pays. Measured on the dense synthetic
/// board in the validation record, both planes took a spin's frame from 119 to
/// 230 ms on the headless renderer.
///
/// They are DISPLAY-ONLY ([`Body::aux_from`]): drawn, but never picked. An edge
/// outranks a face under the pointer, and copper is thin enough that its outline
/// is under the pointer wherever the copper is, so a pickable outline would take
/// the hover from the NET face — whose name is what hovering copper is for.
fn copper_outline(regions: &[outlines::Region], z: f64) -> Vec<Vec<Vec3>> {
    outlines::union_boundary(regions)
        .into_iter()
        .map(|line| line.into_iter().map(|p| Vec3::new(p[0], p[1], z)).collect())
        .collect()
}

/// The OUTER face of the copper on `layer`: the top of F.Cu, the underside of
/// B.Cu, and the upper face of an inner layer (which has no outside).
fn copper_face(layer: u8, layer_count: u8) -> f64 {
    let (lo, hi) = copper_span(layer, layer_count);
    if layer > 0 && layer + 1 >= layer_count { lo } else { hi }
}

/// One copper layer: every copper primitive the board carries THROUGH that
/// layer — the tracks on it, the pads that reach it, and each via's annular
/// ring — as one body, so a layer costs the scene one buffer set whatever it
/// carries.
///
/// The layer is ONE body but not one face. Its copper is grouped by NET, one
/// named face per net, and [`default_color`] gives each face the net's own
/// colour — so a trace reads as part of a net at a glance, and hovering it
/// names that net instead of naming the layer. The grouping is why this costs
/// nothing extra: a face is a triangle RANGE, not a buffer, and the body count
/// — the number the viewport lives or dies by — is unchanged.
///
/// `nets` is index-aligned with `copper`. Copper on an island that carries no
/// single schematic net — an unrouted stub, a board with no schematic, or a
/// short between two nets — goes on the [`NO_NET_FACE`] face and stays bare
/// copper, so a board nobody has drawn a schematic for looks exactly as it did.
fn copper_layer(
    board: &Board,
    copper: &[CopperItem],
    nets: &[Option<String>],
    layer: u8,
) -> Option<Body> {
    let (lo, hi) = copper_span(layer, board.layer_count);
    // Grouped in a BTreeMap so the faces come out in one order whatever order
    // the router laid the copper down in: a rebuild that reshuffled them would
    // move every face's metadata key, and a per-net colour the user stored
    // with it.
    let mut by_net: BTreeMap<Option<&str>, (Vec<usize>, Vec<&board::FillPiece>)> = BTreeMap::new();
    for (i, item) in copper.iter().enumerate() {
        if item.layers.0 <= layer && layer <= item.layers.1 {
            by_net.entry(nets[i].as_deref()).or_default().0.push(i);
        }
    }
    // A zone's fill is its net's copper, on the same face as that net's tracks
    // and pads: one colour, one name to hover, and no extra body.
    for zone in board.zones.iter().filter(|z| z.layer == layer) {
        by_net.entry(Some(zone.net.as_str())).or_default().1.extend(&zone.fill);
    }
    let name = format!("{BOARD_SOLID_PREFIX}{}", board::layer_name(layer, board.layer_count));
    let mut soup = Soup::default();
    let mut faces: Vec<String> = Vec::with_capacity(by_net.len());
    // Every shape drawn, for the outline: the net does not matter to where
    // copper stops, so it is one union for the whole layer.
    let mut regions: Vec<outlines::Region> = Vec::new();
    for (net, (items, fill)) in &by_net {
        // The display builder reads a face's triangles as their FIRST
        // contiguous run of face ids, so a net's copper is emitted all at once
        // — and the id is claimed only once something has been drawn under it.
        let face = faces.len() as u32;
        let mut drawn = false;
        for &i in items {
            let outline = shape_loop(&copper[i].shape);
            if outline.len() >= 3 {
                soup.prism(&outline, lo, hi, face);
                regions.push(outlines::Region::Convex(outline));
                drawn = true;
            }
        }
        for piece in fill {
            if piece.outer.len() >= 3 {
                soup.slab(piece, lo, hi, face);
                let rings = std::iter::once(&piece.outer)
                    .chain(&piece.holes)
                    .map(|ring| ring.iter().copied().map(xy).collect())
                    .collect();
                regions.push(outlines::Region::Fill(rings));
                drawn = true;
            }
        }
        if drawn {
            faces.push(format!("{name}{FACE_SEPARATOR}{}", net.unwrap_or(NO_NET_FACE)));
        }
    }
    let mut body = body(name, COPPER_COLOR, faces, soup)?;
    body.aux_from = body.payload.edges.len();
    for line in copper_outline(&regions, copper_face(layer, board.layer_count)) {
        let id = body.payload.edges.len() as u64;
        body.payload.edges.push((id, None, line));
    }
    Some(body)
}

/// Every solder-mask OPENING on one side, as the shape the aperture takes.
///
/// The rule is `fabrication::mask`'s, not a new one: a pad whose copper reaches
/// this side is open, a bottom-side drilled pad is open on the bottom whatever
/// its copper does, and a VIA is tented — covered — which is why no via appears
/// here. The aperture is the pad grown by
/// [`board::DesignRules::mask_expansion`], the same growth the Gerber writer
/// flashes.
fn mask_openings(board: &Board, top: bool) -> impl Iterator<Item = Shape> + '_ {
    let side = if top { 0 } else { board.layer_count.saturating_sub(1) };
    board.placements.iter().flat_map(move |placement| {
        placement.footprint.pads.iter().filter_map(move |pad| {
            let (from, to) = placement.pad_layers(pad, board.layer_count);
            let reaches = (from..=to).contains(&side) && (top || side != 0);
            (reaches || (!top && pad.drill.is_some()))
                .then(|| grown(placement.pad_shape(pad), board.rules.mask_expansion))
        })
    })
}

/// One side's SOLDER MASK: the lacquer sheet over the whole outline, and the
/// bare-metal opening over every pad the mask is open on.
///
/// Which pads those are is `fabrication::mask`'s rule, not a new one: a pad
/// whose copper reaches this side is open, a bottom-side drilled pad is open on
/// the bottom whatever its copper does, and a VIA is tented — covered — which
/// is why no via appears here. The opening is the pad's shape grown by
/// [`board::DesignRules::mask_expansion`], the same growth the Gerber writer
/// flashes.
///
/// **The lacquer is not cut.** An opening is a prism standing `COPPER_THICKNESS`
/// proud of the sheet rather than a hole through it, so it COVERS the lacquer
/// instead of removing it — the same approximation the undrilled substrate
/// makes for a via barrel, and for the same reason: cutting apertures out of
/// the outline polygon wants a hole-aware triangulation this module does not
/// have, and hundreds of pads is exactly the size at which bridging them into
/// the ear clipper stops being cheap.
///
/// **It is built HIDDEN.** Opaque lacquer over the copper hides the traces,
/// which is the thing the 3D board was asked for; a user who wants to see the
/// board as it will be manufactured ticks it on in the Scene tree.
///
/// **A side with no OPENING gets no mask body at all.** Lacquer on its own is
/// the same green the substrate already draws over the same outline, so a board
/// with no pads on this side would gain a hidden body that shows nothing when
/// it is shown. A board nobody has placed a part on therefore still makes
/// exactly the four bodies it made before this.
fn mask_layer(board: &Board, top: bool) -> Option<Body> {
    let outline = oriented_outline(board)?;
    // Every opening this side has, collected first — a side with none gets no
    // body, and the lacquer has to be emitted BEFORE them in one soup because
    // the display builder reads a face's triangles as their first contiguous
    // run of face ids.
    let apertures: Vec<Vec<[f64; 2]>> = mask_openings(board, top)
        .map(|shape| shape_loop(&shape))
        .filter(|loop_xy| loop_xy.len() >= 3)
        .collect();
    if apertures.is_empty() {
        return None;
    }
    let mut soup = Soup::default();
    let (lo, hi) = side_span(top, 0, LACQUER_TOP);
    soup.prism(&outline, lo, hi, 0);
    let (open_lo, open_hi) = side_span(top, 0, LACQUER_TOP + board::COPPER_THICKNESS);
    for loop_xy in &apertures {
        soup.prism(loop_xy, open_lo, open_hi, 1);
    }
    let name = mask_solid(top);
    let faces = vec![
        format!("{name}{FACE_SEPARATOR}{LACQUER_FACE}"),
        format!("{name}{FACE_SEPARATOR}{OPENING_FACE}"),
    ];
    body(name, MASK_COLOR, faces, soup)
}

/// One side's SILKSCREEN: every silk polyline of every footprint mounted on
/// that side, inked on top of the mask.
///
/// A polyline is drawn the way the Gerber writer strokes it — a round pen of
/// `fabrication::SILK_WIDTH` dragged along it — which here is one capsule prism
/// per segment, the same primitive a track segment is. A footprint's silk is a
/// handful of segments, so a whole board's ink is a rounding error beside its
/// copper.
///
/// Reference designators are NOT drawn. The Gerber legend strokes them with
/// `fabrication::text_strokes`, which is private to that module; exporting it
/// is a change to `brep_ecad_core` and not this one.
fn silkscreen(board: &Board, top: bool) -> Option<Body> {
    let (lo, hi) = side_span(top, LACQUER_TOP, LACQUER_TOP + SILK_THICKNESS);
    let mut soup = Soup::default();
    // A footprint is on the FRONT silkscreen when it is not mounted on the
    // bottom — the same test `fabrication::legend` makes.
    for placement in board.placements.iter().filter(|p| p.bottom != top) {
        for line in &placement.footprint.silk {
            for pair in line.windows(2) {
                let shape = Shape::segment(
                    placement.transform(pair[0]),
                    placement.transform(pair[1]),
                    brep_ecad_core::fabrication::SILK_WIDTH,
                );
                let loop_xy = shape_loop(&shape);
                if loop_xy.len() >= 3 {
                    soup.prism(&loop_xy, lo, hi, 0);
                }
            }
        }
    }
    body(silk_solid(top), SILK_COLOR, vec!["Ink".to_string()], soup)
}

/// The plated via barrels: for each via, the tube the plating makes on the wall
/// of its drilled hole, from the top copper plane to the bottom one.
fn via_barrels(board: &Board) -> Option<Body> {
    let (lo, hi) = (mm(-board::SUBSTRATE_THICKNESS), 0.);
    let mut soup = Soup::default();
    let facets = ARC_FACETS * 2;
    for via in &board.vias {
        let centre = xy(via.at);
        let inner = mm(via.drill.max(1) / 2).max(1e-4);
        let outer = inner + mm(board::COPPER_THICKNESS);
        let ring = |r: f64| -> Vec<[f64; 2]> {
            (0..facets)
                .map(|i| {
                    let t = std::f64::consts::TAU * i as f64 / facets as f64;
                    [centre[0] + r * t.cos(), centre[1] + r * t.sin()]
                })
                .collect()
        };
        let (out_ring, in_ring) = (ring(outer), ring(inner));
        soup.walls(&out_ring, lo, hi, 0);
        // The bore, wound the other way so its wall faces into the hole.
        let mut bore = in_ring.clone();
        bore.reverse();
        soup.walls(&bore, lo, hi, 0);
        // The two annular ends.
        for (z, up) in [(hi, true), (lo, false)] {
            for i in 0..facets {
                let j = (i + 1) % facets;
                let (a, b) = (in_ring[i], in_ring[j]);
                let (c, d) = (out_ring[j], out_ring[i]);
                let quad = [
                    [a[0], a[1], z],
                    [b[0], b[1], z],
                    [c[0], c[1], z],
                    [d[0], d[1], z],
                ];
                if up {
                    soup.quad(quad[0], quad[1], quad[2], quad[3], 0);
                } else {
                    soup.quad(quad[3], quad[2], quad[1], quad[0], 0);
                }
            }
        }
    }
    body(VIAS_SOLID.to_string(), COPPER_COLOR, vec!["Barrel".to_string()], soup)
}

/// Every copper primitive on the board, and the NET each one carries.
///
/// The nets come from [`Board::connectivity`], which walks the copper into
/// islands and reads each island's net off the pads it touches — so a track is
/// on a net because it reaches a pad of that net, which is the only sense in
/// which a track HAS one. `None` for an island carrying no net, or more than
/// one, which is a short and must not be painted as though it were a net.
///
/// With no netlist — a board whose document has no placement, so no pad can
/// carry a net — the copper is asked for directly and every item's net is
/// `None`. That skips the island walk entirely, which is why a board nobody has
/// drawn a schematic for pays nothing at all for net colouring.
fn copper_and_nets(
    board: &Board,
    netlist: Option<&Netlist>,
) -> (Vec<CopperItem>, Vec<Option<String>>) {
    let Some(netlist) = netlist else {
        let copper = board.copper();
        let nets = vec![None; copper.len()];
        return (copper, nets);
    };
    let connectivity = board.connectivity(netlist);
    let nets = connectivity
        .islands
        .iter()
        .map(|&island| connectivity.island_net(island).map(str::to_owned))
        .collect();
    (connectivity.items, nets)
}

/// Every body a board contributes: the substrate, each copper layer from F.Cu
/// down to B.Cu, the via barrels, then the two outer surfaces — each side's
/// solder mask and its silkscreen. A layer, a side or a surface carrying
/// nothing contributes no body at all.
///
/// The copper stack comes FIRST and in its original order, so the names a
/// reader already knows (`PCB_Board`, `PCB_F.Cu`, …, `PCB_Vias`) are still the
/// first four of a two-layer board and the surfaces are appended behind them.
fn bodies(board: &Board, netlist: Option<&Netlist>) -> Vec<Body> {
    let (copper, nets) = copper_and_nets(board, netlist);
    let mut out = Vec::new();
    out.extend(substrate(board));
    for layer in 0..board.layer_count.max(1) {
        out.extend(copper_layer(board, &copper, &nets, layer));
    }
    out.extend(via_barrels(board));
    for top in [true, false] {
        out.extend(mask_layer(board, top));
    }
    for top in [true, false] {
        out.extend(silkscreen(board, top));
    }
    out
}

/// Whether a board body is SHOWN the first time it reaches the scene.
///
/// Everything is, except the solder mask: opaque lacquer over the copper hides
/// the traces, and seeing the traces is what the 3D board is for. The mask is
/// built all the same, so ticking it on in the Scene tree costs nothing and
/// needs no rebuild.
fn default_visible(name: &str) -> bool {
    !name.ends_with(".Mask")
}

/// What [`EngineState::refresh_board_geometry`] built, for the tests and the
/// cost figures in the validation record.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BoardBuild {
    pub bodies: usize,
    pub triangles: usize,
    /// Display edges over every body, and the line segments they draw — the
    /// instance count the edge passes pay per frame.
    pub edges: usize,
    pub edge_segments: usize,
}

impl EngineState {
    /// (Re)build the board's 3D bodies from the document's `pcb` block.
    ///
    /// Mirrors [`refresh_committed_sketches`](Self::refresh_committed_sketches)
    /// in shape and for the same reason: `apply_run_output` reconciles the scene
    /// against the RUN's outputs and drops everything else, so a derived body
    /// has to be re-inserted after it. Bodies shown last time and not this time
    /// (the block went away, a layer emptied, the board shrank) are removed.
    ///
    /// Cheap to call: the parsed board is compared against the one the current
    /// bodies were built from and an unchanged block returns without touching
    /// the scene. That is what lets the eCAD host call it on a track edit, which
    /// writes the block and — changing no placement — runs no history.
    ///
    /// A board that DID change is rebuilt and every body is then seated over
    /// whatever is resident under its name. Replacing, not skipping, is the
    /// point: see the seating comment below.
    pub fn refresh_board_geometry(&mut self) -> BoardBuild {
        // The ordinary document carries no `pcb` block and pays one `Option`
        // test per run for the whole of this module.
        let block = self.history.pcb_block();
        if block.is_none() && self.board_displays.is_empty() {
            return BoardBuild::default();
        }
        // The whole DOCUMENT, not just its board: a copper island's net is read
        // off the pads it touches, and which net a pad carries is the
        // SCHEMATIC's answer. The netlist is only asked for when the board has
        // a placement, because a board with no pads can carry no net and the
        // walk would be work for a guaranteed `None`.
        let document = block
            .cloned()
            .and_then(|value| brep_ecad_core::Document::from_value(value).ok());
        let (board, netlist) = match document {
            Some(document) => {
                let netlist = (!document.board.placements.is_empty()).then(|| document.netlist());
                (Some(document.board), netlist)
            }
            None => (None, None),
        };
        // The rebuild test is the board AND the pin-to-net map, because those
        // two together decide every triangle and every face colour this module
        // makes. Renaming a net on the schematic moves no copper, so the board
        // alone would report nothing to do and the traces would keep the old
        // net's colour until the next routed track.
        let pin_nets = netlist
            .as_ref()
            .map(brep_ecad_core::board::pin_nets)
            .unwrap_or_default();
        let rebuilt = board != self.board_built_from || pin_nets != self.board_pin_nets;
        if rebuilt {
            self.board_pin_nets = pin_nets;
            self.rebuild_board_displays(board, netlist.as_ref());
        }
        // Seat the bodies. Two different things bring us here and each needs
        // its own half of the condition:
        //
        // - a REBUILD made new displays, and a body's name does not change with
        //   what is on it — `PCB_F.Cu` is `PCB_F.Cu` whether one track is routed
        //   on it or a thousand. The OLD display is still resident under that
        //   name, so "insert it if it is missing" leaves the stale geometry in
        //   the scene and the 3D board follows the 2D board only once something
        //   else has swept the scene (a reload, or any history run). That was
        //   the staleness the user saw while routing;
        // - a RECONCILE dropped them. `apply_run_output` keeps only the RUN's
        //   own outputs, so after a run the scene has none of these and every
        //   one goes back in. Nothing was rebuilt, so the kept displays are
        //   copied in rather than remade: rebuilding a dense board costs two
        //   orders of magnitude more than copying it, and the kept `revision`
        //   is what lets the renderer reuse its GPU buffers.
        //
        // A schematic edit, which writes the SAME board back, is neither: it
        // rebuilds nothing, finds every body resident, and touches no solid.
        let mut seated = false;
        for index in 0..self.board_displays.len() {
            // What the scene already shows under this name, if anything. Read
            // out by value so the scene is free to be written to below.
            let name = self.board_displays[index].name.clone();
            let resident = self
                .scene
                .solid(&name)
                .map(|solid| (solid.visible, solid.color_override));
            if let Some((visible, _)) = resident {
                // The SCENE is where a hide lives, and a history run wipes the
                // scene — so the kept display takes the hide back with it here,
                // while there is still a resident body to read it off. Without
                // this the mask, which is built hidden, would come back hidden
                // after every feature edit however many times the user ticked
                // it on.
                self.board_displays[index].visible = visible;
            }
            if !rebuilt && resident.is_some() {
                continue;
            }
            let mut fresh = self.board_displays[index].clone();
            if let Some((_, color)) = resident {
                // A rebuild changes the body's GEOMETRY. It does not undo what
                // the user has done TO the body, and replacing the whole
                // display would: a stored colour is meant to win over the
                // board's default, and nothing re-derives it on this path,
                // because `sync_colors_from_metadata` runs in `finish_apply`
                // and no history runs when a track is routed. (The other half
                // of that — the user's HIDE — was carried into the kept display
                // a few lines up, so `fresh` already has it.)
                //
                // `visibility`, the per-FACE hide map, is deliberately NOT
                // carried: `SolidDisplay` says a re-tessellated solid resets to
                // all-visible, and this is a re-tessellation.
                fresh.color_override = color;
            }
            self.scene.insert_solid(fresh);
            seated = true;
        }
        if seated {
            // A body's FACES are coloured too now — a copper layer's net faces,
            // the mask's bare openings — and a face colour the user stored is a
            // metadata record like any other. Resolving them through the one
            // seam that owns colour, rather than carrying them across by hand,
            // is what keeps a painted net painted on the frame it is routed on;
            // the pass is a no-op when nothing moved, so an untouched board
            // costs a compare per face.
            self.sync_colors_from_metadata();
        }
        self.dirty |= seated;
        self.board_build
    }

    /// Build `board`'s bodies, sweeping out any the last board left behind that
    /// this one does not make (the block went away, a layer emptied, the last
    /// via was deleted). It does NOT put the new displays in the scene — the
    /// caller's seating pass does, and must, because a name that survives a
    /// rebuild still holds the OLD body.
    fn rebuild_board_displays(&mut self, board: Option<Board>, netlist: Option<&Netlist>) {
        let built: Vec<Body> = board
            .as_ref()
            .map(|board| bodies(board, netlist))
            .unwrap_or_default();
        let mut build = BoardBuild::default();
        // What each body was SHOWING before this rebuild, so a rebuild does not
        // un-hide a body the user hid — the mask included, which starts hidden
        // and stays hidden only until the user says otherwise.
        let was_visible: std::collections::HashMap<&str, bool> = self
            .board_displays
            .iter()
            .map(|display| (display.name.as_str(), display.visible))
            .collect();
        let displays: Vec<crate::scene::SolidDisplay> = built
            .into_iter()
            .map(|Body { name, color, payload, aux_from }| {
                build.bodies += 1;
                build.triangles += payload.mesh.face_ids.len();
                let mut solid = crate::scene::solid_display_from_payload(&name, payload);
                for edge in solid.edges.iter_mut().skip(aux_from) {
                    edge.aux = true;
                }
                build.edges += solid.edges.len();
                build.edge_segments += solid.edges.iter().map(|e| e.polyline.len().saturating_sub(1)).sum::<usize>();
                solid.color_override = Some(color);
                // Every named face's DEFAULT colour, stamped at build time.
                // The seating pass re-resolves all of them through
                // `sync_colors_from_metadata` — which is what makes a colour the
                // user STORED win — but that pass bumps the `revision` of any
                // solid it CHANGES, and a bumped revision re-uploads the body's
                // buffers. A kept display that did not already carry the colour
                // the pass resolves would therefore be re-uploaded on every
                // single history run, which is the cost the kept displays exist
                // to avoid. So the two are not redundant: the stamp is what
                // keeps the revision stable, the pass is what honours the store.
                for face in &mut solid.faces {
                    face.color_override = default_color(&face.name);
                }
                solid.visible = was_visible
                    .get(name.as_str())
                    .copied()
                    .unwrap_or_else(|| default_visible(&name));
                solid
            })
            .collect();
        let fed: Vec<String> = displays.iter().map(|solid| solid.name.clone()).collect();
        for name in std::mem::replace(&mut self.shown_board_ids, fed) {
            if !self.shown_board_ids.contains(&name) {
                self.scene.remove_solid(&name);
            }
        }
        self.board_displays = displays;
        self.board_built_from = board;
        self.board_build = build;
        self.sync_view_frame();
        self.dirty = true;
    }

    /// What the last [`refresh_board_geometry`](Self::refresh_board_geometry)
    /// built — body and triangle counts, for the cost the record reports.
    pub fn board_build(&self) -> BoardBuild {
        self.board_build
    }

    /// Remember that the user SHOWED or HID a board body, so the kept displays
    /// carry it across a history run.
    ///
    /// A run reconciles the scene against its own outputs and drops every
    /// derived body, so by the time the refresh at the tail of `finish_apply`
    /// runs there is nothing resident to read the state off — and the body
    /// would come back at its build-time default. That matters most for the
    /// solder mask, which is built HIDDEN: without this, showing it would last
    /// until the next feature edit.
    ///
    /// Called from [`set_visible`](Self::set_visible), which is the one seam
    /// the Scene tree and the BOM panel both hide a solid through.
    pub(super) fn remember_board_visibility(&mut self, name: &str, visible: bool) {
        if !name.starts_with(BOARD_SOLID_PREFIX) {
            return;
        }
        if let Some(display) = self
            .board_displays
            .iter_mut()
            .find(|display| display.name == name)
        {
            display.visible = visible;
        }
    }

    /// The document's board as EXACT solids, for STEP export — `None` when the
    /// document has no `pcb` block or its board has no usable outline. Built
    /// from the block every call rather than kept: an export is rare, and the
    /// booleans that drill the holes are not what a track edit should pay for.
    pub(crate) fn board_step_solids(&self) -> Option<step_solids::BoardStepSolids> {
        let document = brep_ecad_core::Document::from_value(self.history.pcb_block()?.clone()).ok()?;
        let netlist = (!document.board.placements.is_empty()).then(|| document.netlist());
        step_solids::board_step_solids(&document.board, netlist.as_ref())
    }

    /// The scene names of the board bodies currently shown, in stack order.
    pub fn board_solid_names(&self) -> &[String] {
        &self.shown_board_ids
    }
}

mod outlines;
mod step_solids;

