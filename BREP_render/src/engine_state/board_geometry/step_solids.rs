//! The board as EXACT solids, for STEP export.
//!
//! The viewport's board bodies (the parent module) are triangles on purpose,
//! and the header there says what that costs; the one cost that mattered to a
//! mechanical engineer was "it is not exported to STEP". This module is the
//! answer, and it is built for export only: a `document_export step` of a
//! board document now carries the board beside its parts.
//!
//! What is made, all as kernel B-rep (never the display mesh):
//!
//!   * `PCB_Board` — the outline swept through `SUBSTRATE_THICKNESS`, DRILLED:
//!     every via and every through-hole pad is a cylindrical hole through it,
//!     which the display substrate is not; and with the board's cutouts milled
//!     through it, as the display substrate also has them;
//!   * `PCB_<layer>/<net>#<n>` — one thin solid per copper primitive (track
//!     segment, pad, via ring) on each layer, `COPPER_THICKNESS` thick, with
//!     true circular arcs for round track ends and pads, and a drilled pad or
//!     via ring a ring round its hole rather than a disc over it. They are NOT fused: a track overlaps the
//!     pad it lands on, so the layer's copper is a set of overlapping solids,
//!     each one closed and valid on its own — the way an unfused eCAD STEP
//!     writes it;
//!   * `PCB_<layer>/<net>#<n>` again for each piece of a zone's FILL: its outer
//!     ring swept through the copper with every hole cut out in one n-ary
//!     subtract, straight-sided because a fill's rings are polygons;
//!   * `PCB_Vias#<n>` — each via's plated barrel, a tube from F.Cu's plane to
//!     B.Cu's.
//!
//! The solder mask and the silkscreen are left out: they are films a few
//! hundredths of a millimetre thick that no mechanical fit reads, and the
//! display keeps the mask hidden by default for the same reason.
//!
//! Round rings and barrels are REVOLVED, not drilled: a rectangle turned a full
//! circle about the hole's axis is the exact annulus, with no boolean. The
//! substrate's holes are cut in one n-ary subtract. On the re-audit's
//! amp-board (127 board solids) the board costs about 0.4 s where cutting every
//! bore with its own boolean cost 6.3 s — the validation record has the split.
//!
//! The zones are REFILLED on a copy first, as the fabrication export does: the
//! fill stored in the document is the one the user last made, and a track drawn
//! after it (the normal "pour, route one more, save") leaves it crossing that
//! track. Written as stored, the file carried copper shorting two nets that the
//! Gerbers of the same document did not (the eCAD third audit's B2). When the
//! refill differs from what is stored, [`BoardStepSolids::notices`] says so.
//!
//! A primitive the kernel refuses (a degenerate shape, a boolean that fails)
//! is SKIPPED and listed in [`BoardStepSolids::skipped`], never allowed to
//! fail the whole export: a board with one odd pad still exports its board,
//! and the export posts a notice naming what is missing.

use super::{copper_and_nets, cutout_rings, mm, oriented_outline, xy, BOARD_SOLID_PREFIX, SUBSTRATE_SOLID, VIAS_SOLID};
use brep_ecad_core::board::{self, Board, CopperRef, Shape};
use brep_ecad_core::Netlist;
use brep_kernel::{
    boolean_operation, extrude_profile_brep, make_arc, make_cylinder_brep, make_line, revolve_profile_brep,
    BooleanOperation, BooleanOptions, BrepSolid, NurbsCurve, Vec3,
};

/// The export colours, as 8-bit sRGB — the display's FR-4 green and bare
/// copper, so the file looks like the board the user was looking at.
pub(crate) const SUBSTRATE_RGB: [u8; 3] = [0x1c, 0x5e, 0x38];
pub(crate) const COPPER_RGB: [u8; 3] = [0xc0, 0x7b, 0x38];

/// The board's exact solids and what could not be made.
#[derive(Default)]
pub(crate) struct BoardStepSolids {
    /// `(body name, solid)` in a stable order: substrate, copper layer by
    /// layer, barrels.
    pub bodies: Vec<(String, BrepSolid)>,
    /// `(body name, colour)` for every body.
    pub colors: Vec<(String, [u8; 3])>,
    /// Primitives the kernel refused, as `"<body>: <error>"`.
    pub skipped: Vec<String>,
    /// What the export should tell the user about the board beyond `skipped`:
    /// that the zones' stored fill was out of date and a fresh one was written.
    pub notices: Vec<String>,
}

/// One drilled hole: centre (mm, +Y up) and diameter (mm). A SLOT is the
/// diameter swept from `at` to `to`; a round hole has `to == at`.
#[derive(Clone, Copy)]
struct Hole {
    at: [f64; 2],
    to: [f64; 2],
    diameter: f64,
    /// How deep a BLIND hole goes, as the z range it opens (mm); `None` for a
    /// hole through the whole board.
    depth: Option<(f64, f64)>,
}
impl Hole {
    fn round(at: [f64; 2], diameter: f64) -> Self {
        Self { at, to: at, diameter, depth: None }
    }
    fn is_round(&self) -> bool {
        (self.to[0] - self.at[0]).hypot(self.to[1] - self.at[1]) <= 1e-9
    }
}

fn point(p: [f64; 2], z: f64) -> Vec3 {
    Vec3::new(p[0], p[1], z)
}

/// A closed straight-sided loop as line curves at height `z`.
fn polygon_curves(loop_xy: &[[f64; 2]], z: f64) -> Result<Vec<NurbsCurve>, String> {
    (0..loop_xy.len())
        .map(|i| make_line(point(loop_xy[i], z), point(loop_xy[(i + 1) % loop_xy.len()], z)))
        .collect()
}

/// A half-turn arc about `centre` at height `z`, from angle `start`.
fn half_arc(centre: [f64; 2], radius: f64, start: f64, z: f64) -> Result<NurbsCurve, String> {
    make_arc(
        point(centre, z),
        Vec3::new(1., 0., 0.),
        Vec3::new(0., 1., 0.),
        radius,
        start,
        start + std::f64::consts::PI,
    )
}

/// A copper primitive's outline as EXACT curves at height `z`: a rectangle as
/// four lines, a capsule as two lines and two half-circles, a circle as two
/// half-circles (the extruder wants at least two curves).
fn shape_curves(shape: &Shape, z: f64) -> Result<Vec<NurbsCurve>, String> {
    match shape.clone() {
        // A polygon's corners as straights, its rounding (a custom pad's, or a
        // mask opening grown from one) faceted on the true outline. Board y
        // points down, so the outline is walked backwards to turn
        // counter-clockwise in millimetres.
        Shape::Polygon { .. } if shape.swept_corners().is_some() => {
            // Exact: each outward corner's arc and the straights between,
            // walked backwards (board y points down) to turn counter-clockwise
            // in millimetres, as the Gerber's stroked region draws it.
            let corners = shape.swept_corners().unwrap_or_default();
            let n = corners.len();
            let mmxy = |(x, y): (f64, f64)| [x / 1000., -y / 1000.];
            let mut curves = vec![];
            for i in (0..n).rev() {
                let (start, _) = corners[i].ends();
                if let board::SweptCorner::Arc { centre, radius, from, sweep } = corners[i] {
                    curves.push(make_arc(
                        point(xy(centre), z),
                        Vec3::new(1., 0., 0.),
                        Vec3::new(0., 1., 0.),
                        mm(radius),
                        -(from + sweep),
                        -from,
                    )?);
                }
                let (to, from_pt) = (mmxy(corners[(i + n - 1) % n].ends().1), mmxy(start));
                if (to[0] - from_pt[0]).hypot(to[1] - from_pt[1]) > 1e-6 {
                    curves.push(make_line(point(from_pt, z), point(to, z))?);
                }
            }
            Ok(curves)
        }
        // A polygon with its corners as its outline, or one whose rounding fills
        // a notch: the outline, faceted on the true curve.
        Shape::Polygon { .. } => {
            let mut ring: Vec<[f64; 2]> = shape.outline(8).into_iter().map(xy).collect();
            ring.reverse();
            if ring.len() < 3 {
                return Err("degenerate polygon".into());
            }
            polygon_curves(&ring, z)
        }
        // Four quarter arcs about the core rectangle's corners, joined by the
        // straights between them (none where the radius takes a whole side).
        Shape::RoundRect { min, max, radius } => {
            let r = mm(radius);
            let (a, b) = (xy(min), xy(max));
            let (x0, x1) = (a[0].min(b[0]) + r, a[0].max(b[0]) - r);
            let (y0, y1) = (a[1].min(b[1]) + r, a[1].max(b[1]) - r);
            if r <= 1e-6 || x1 < x0 - 1e-9 || y1 < y0 - 1e-9 {
                return Err("degenerate rounded rectangle".into());
            }
            let half = std::f64::consts::FRAC_PI_2;
            let at = |c: [f64; 2], t: f64| [c[0] + r * t.cos(), c[1] + r * t.sin()];
            let mut curves = vec![];
            for (k, c) in [[x1, y0], [x1, y1], [x0, y1], [x0, y0]].into_iter().enumerate() {
                let start = half * (k as f64 - 1.);
                curves.push(make_arc(
                    point(c, z),
                    Vec3::new(1., 0., 0.),
                    Vec3::new(0., 1., 0.),
                    r,
                    start,
                    start + half,
                )?);
                let next = [[x1, y1], [x0, y1], [x0, y0], [x1, y0]][k];
                let (from, to) = (at(c, start + half), at(next, start + half));
                if (from[0] - to[0]).hypot(from[1] - to[1]) > 1e-6 {
                    curves.push(make_line(point(from, z), point(to, z))?);
                }
            }
            Ok(curves)
        }
        Shape::Rect { min, max } => {
            let (a, b) = (xy(min), xy(max));
            let (x0, x1) = (a[0].min(b[0]), a[0].max(b[0]));
            let (y0, y1) = (a[1].min(b[1]), a[1].max(b[1]));
            if x1 - x0 <= 1e-6 || y1 - y0 <= 1e-6 {
                return Err("degenerate rectangle".into());
            }
            polygon_curves(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]], z)
        }
        Shape::Capsule { a, b, radius } => {
            let (a, b, r) = (xy(a), xy(b), mm(radius));
            if r <= 1e-6 {
                return Err("zero-radius capsule".into());
            }
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            if (dx * dx + dy * dy).sqrt() <= 1e-6 {
                return Ok(vec![half_arc(a, r, 0., z)?, half_arc(a, r, std::f64::consts::PI, z)?]);
            }
            // The half-ring round `b`, the straight back to `a`'s side, the
            // half-ring round `a`, and the straight home — the same stadium the
            // display draws, with the arcs exact.
            let base = dy.atan2(dx) - std::f64::consts::FRAC_PI_2;
            let at = |c: [f64; 2], t: f64| [c[0] + r * t.cos(), c[1] + r * t.sin()];
            let (pi, far) = (std::f64::consts::PI, base + std::f64::consts::PI);
            Ok(vec![
                half_arc(b, r, base, z)?,
                make_line(point(at(b, base + pi), z), point(at(a, far), z))?,
                half_arc(a, r, far, z)?,
                make_line(point(at(a, far + pi), z), point(at(b, base), z))?,
            ])
        }
    }
}

/// The profile at `lo` swept up to `hi`.
fn prism(curves: &[NurbsCurve], lo: f64, hi: f64) -> Result<BrepSolid, String> {
    // The kernel's extrusion refuses typed; this helper's callers are stringly
    // (the lossy exit above the typed stack).
    extrude_profile_brep(curves, Vec3::new(0., 0., 1.), hi - lo).map_err(String::from)
}

/// An exact ANNULUS about `centre`, `r_in`..`r_out`, from `lo` up to `hi`: a
/// rectangle in the XZ half-plane revolved a full turn about the vertical axis
/// through `centre`. A via ring, a round drilled pad and a via barrel are all
/// this shape, and revolving it costs a fraction of the boolean that would
/// otherwise cut the bore out of a disc.
fn ring(centre: [f64; 2], r_in: f64, r_out: f64, lo: f64, hi: f64) -> Result<BrepSolid, String> {
    if !(r_in > 1e-6 && r_out > r_in + 1e-6 && hi > lo) {
        return Err(format!("degenerate ring r {r_in}..{r_out}"));
    }
    let at = |r: f64, z: f64| Vec3::new(centre[0] + r, centre[1], z);
    let profile = [
        make_line(at(r_in, lo), at(r_out, lo))?,
        make_line(at(r_out, lo), at(r_out, hi))?,
        make_line(at(r_out, hi), at(r_in, hi))?,
        make_line(at(r_in, hi), at(r_in, lo))?,
    ];
    revolve_profile_brep(&profile, point(centre, lo), Vec3::new(0., 0., 1.), std::f64::consts::TAU)
}

/// A drilled copper primitive: a round pad or via ring as an exact [`ring`],
/// anything else as its prism with the drill cut out.
fn drilled(shape: &Shape, hole: Hole, lo: f64, hi: f64) -> Result<BrepSolid, String> {
    if let Shape::Capsule { a, b, radius } = *shape {
        let centred = (xy(a)[0] - hole.at[0]).hypot(xy(a)[1] - hole.at[1]) <= 1e-6;
        if a == b && centred && hole.is_round() {
            return ring(hole.at, hole.diameter / 2., mm(radius), lo, hi);
        }
    }
    let body = prism(&shape_curves(shape, lo)?, lo, hi)?;
    drill(body, &[hole], lo, hi)
}

/// An over-long cylinder through `lo`..`hi` on `hole`: the tool a drill cuts
/// with, clear of both faces so no cap is coplanar with the body's.
fn cutter(hole: &Hole, lo: f64, hi: f64) -> Result<BrepSolid, String> {
    let margin = 1.0;
    // A blind hole: from past the outer face it opens on down (or up) to the
    // plane of the inner layer it stops at.
    let (lo, hi, margin_lo, margin_hi) = match hole.depth {
        Some((d_lo, d_hi)) => (d_lo.max(lo), d_hi.min(hi), if d_lo <= lo { margin } else { 0. }, if d_hi >= hi { margin } else { 0. }),
        None => (lo, hi, margin, margin),
    };
    if hole.depth.is_some() {
        return make_cylinder_brep(
            point(hole.at, lo - margin_lo),
            Vec3::new(0., 0., 1.),
            hole.diameter / 2.,
            hi - lo + margin_lo + margin_hi,
        );
    }
    if !hole.is_round() {
        // A slot: its stadium, exact arcs and straights, swept through.
        let (z0, z1) = (lo - margin, hi + margin);
        let um = |v: f64| (v * 1000.).round() as i32;
        let end = |p: [f64; 2]| brep_ecad_core::Point::new(um(p[0]), -um(p[1]));
        let slot = Shape::Capsule { a: end(hole.at), b: end(hole.to), radius: um(hole.diameter / 2.) };
        return prism(&shape_curves(&slot, z0)?, z0, z1);
    }
    make_cylinder_brep(
        point(hole.at, lo - margin),
        Vec3::new(0., 0., 1.),
        hole.diameter / 2.,
        hi - lo + 2. * margin,
    )
}

/// Cut every hole in `holes` out of `body`.
///
/// All of them in ONE n-ary subtract: the amp-board's 13 holes took 1.1 s as
/// thirteen binary cuts and 0.21 s as one (the validation record's figures).
/// If the kernel refuses the set, the holes are cut one at a time instead, so
/// a refusal costs time rather than the hole.
fn drill(body: BrepSolid, holes: &[Hole], lo: f64, hi: f64) -> Result<BrepSolid, String> {
    if holes.is_empty() {
        return Ok(body);
    }
    let mut operands = Vec::with_capacity(holes.len() + 1);
    operands.push(body.clone());
    for hole in holes {
        operands.push(cutter(hole, lo, hi)?);
    }
    if let Ok(drilled) = brep_kernel::boolean_operation_nary(&operands, BooleanOperation::Subtract) {
        return Ok(drilled);
    }
    let options = BooleanOptions::default();
    let mut body = body;
    for hole in holes {
        body = boolean_operation(&body, &cutter(hole, lo, hi)?, BooleanOperation::Subtract, &options)
            .map_err(|refusal| format!("hole at ({:.3}, {:.3}): {refusal}", hole.at[0], hole.at[1]))?;
    }
    Ok(body)
}

/// Mill the board's cutouts out of the substrate `body`: each cutout's polygon
/// swept clear of both faces, all in one n-ary subtract, one at a time if the
/// kernel refuses the set (as [`drill`] does).
fn cut_out(body: BrepSolid, board: &Board, lo: f64, hi: f64) -> Result<BrepSolid, String> {
    if board.cutouts.is_empty() {
        return Ok(body);
    }
    let (z0, z1) = (lo - 1., hi + 1.);
    let cutters = cutout_rings(board)
        .into_iter()
        .map(|mut ring| {
            // Wound counter-clockwise, as the outline is swept.
            ring.reverse();
            prism(&polygon_curves(&ring, z0)?, z0, z1)
        })
        .collect::<Result<Vec<_>, String>>()?;
    let operands: Vec<BrepSolid> = std::iter::once(body.clone()).chain(cutters.iter().cloned()).collect();
    if let Ok(cut) = brep_kernel::boolean_operation_nary(&operands, BooleanOperation::Subtract) {
        return Ok(cut);
    }
    let options = BooleanOptions::default();
    let mut body = body;
    for (k, cutter) in cutters.iter().enumerate() {
        body = boolean_operation(&body, cutter, BooleanOperation::Subtract, &options)
            .map_err(|refusal| format!("cutout {}: {refusal}", k + 1))?;
    }
    Ok(body)
}

/// One piece of a zone's fill as an exact solid: its outer ring swept through the
/// copper, with every hole cut out of it in one n-ary subtract (one at a time if
/// the kernel refuses the set, as [`drill`] does). Straight edges only — a fill's
/// rings are polygons.
fn fill_solid(piece: &board::FillPiece, lo: f64, hi: f64) -> Result<BrepSolid, String> {
    let ring = |points: &[brep_ecad_core::Point], outward: bool| {
        let mut ring: Vec<[f64; 2]> = points.iter().copied().map(xy).collect();
        if (super::signed_area2(&ring) > 0.) != outward {
            ring.reverse();
        }
        ring
    };
    let body = prism(&polygon_curves(&ring(&piece.outer, true), lo)?, lo, hi)?;
    if piece.holes.is_empty() {
        return Ok(body);
    }
    let margin = 1.0;
    let cutters = piece
        .holes
        .iter()
        .map(|hole| {
            let curves = polygon_curves(&ring(hole, true), lo - margin)?;
            prism(&curves, lo - margin, hi + margin)
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut operands = Vec::with_capacity(cutters.len() + 1);
    operands.push(body.clone());
    operands.extend(cutters.iter().cloned());
    if let Ok(cut) = brep_kernel::boolean_operation_nary(&operands, BooleanOperation::Subtract) {
        return Ok(cut);
    }
    let options = BooleanOptions::default();
    let mut body = body;
    for (k, cutter) in cutters.iter().enumerate() {
        body = boolean_operation(&body, cutter, BooleanOperation::Subtract, &options)
            .map_err(|refusal| format!("fill hole {k}: {refusal}"))?;
    }
    Ok(body)
}

/// The drill a copper primitive surrounds: a via's, or a drilled pad's.
fn item_hole(board: &Board, owner: &CopperRef) -> Option<Hole> {
    match *owner {
        CopperRef::Via(index) => board.vias.get(index).map(|via| Hole::round(xy(via.at), mm(via.drill))),
        CopperRef::Pad { placement, pad } => {
            let placement = board.placements.get(placement)?;
            let pad = placement.footprint.pads.get(pad)?;
            let hole = pad.hole_size()?;
            match placement.pad_hole(pad)? {
                // The slot the drill file routes: the capsule's own width.
                Shape::Capsule { a, b, radius } if a != b => Some(Hole { at: xy(a), to: xy(b), diameter: mm(2 * radius), depth: None }),
                _ => Some(Hole::round(xy(placement.transform(pad.at)), mm(hole.x.min(hole.y)))),
            }
        }
        CopperRef::Track { .. } => None,
    }
}

/// Every hole in the board: each through via's drill and each drilled pad's,
/// right through, and each BLIND via's from the outer face it opens on to the
/// plane of its inner layer. A BURIED via's hole opens on neither face: it would
/// be a closed cavity inside the substrate, which is not cut, and its barrel
/// stands in solid substrate as every barrel does in the display.
fn board_holes(board: &Board) -> Vec<Hole> {
    let count = board.layer_count;
    let blind: Vec<Hole> = board
        .vias
        .iter()
        .filter(|via| via.kind(count) == board::ViaKind::Blind)
        .map(|via| {
            let (from, to) = via.layers(count);
            let depth = (mm(board::layer_z(to, count)), mm(board::layer_z(from, count)));
            Hole { depth: Some(depth), ..Hole::round(xy(via.at), mm(via.drill)) }
        })
        .collect();
    let vias = (0..board.vias.len())
        .filter(|&v| board.vias[v].kind(count) == board::ViaKind::Through)
        .map(CopperRef::Via);
    let pads = board.placements.iter().enumerate().flat_map(|(placement, p)| {
        (0..p.footprint.pads.len()).map(move |pad| CopperRef::Pad { placement, pad })
    });
    vias.chain(pads)
        .filter_map(|owner| item_hole(board, &owner))
        .chain(blind)
        .filter(|hole| hole.diameter > 1e-6)
        .collect()
}

/// A body name's net part: the net, or `no-net` for an island that carries no
/// single one. `/` is the separator the display's face names use, so a net
/// name that contains one has it replaced.
fn net_part(net: Option<&str>) -> String {
    net.map(|net| net.replace('/', "_")).unwrap_or_else(|| "no-net".to_string())
}

/// The board's exact solids. `None` for a board with no usable outline — the
/// same boards the display draws no substrate for.
pub(crate) fn board_step_solids(board: &Board, netlist: Option<&Netlist>) -> Option<BoardStepSolids> {
    let outline = oriented_outline(board)?;
    let mut out = BoardStepSolids::default();
    let refilled;
    let board = if board.zones.is_empty() {
        board
    } else {
        let mut copy = board.clone();
        let empty = Netlist { version: 1, nets: vec![], diagnostics: vec![] };
        copy.fill_zones(netlist.unwrap_or(&empty));
        if copy.zones != board.zones {
            out.notices.push(
                "STEP export: the zones' fill was out of date, so the file carries a fresh fill, as the fabrication \
                 export does. Fill zones (B) on the board to see the copper that was written."
                    .to_owned(),
            );
        }
        refilled = copy;
        &refilled
    };
    let holes = board_holes(board);
    let (board_lo, board_hi) = (mm(-board::SUBSTRATE_THICKNESS), 0.);

    // The substrate.
    let substrate = polygon_curves(&outline, board_lo)
        .and_then(|curves| prism(&curves, board_lo, board_hi))
        .and_then(|body| drill(body, &holes, board_lo, board_hi))
        .and_then(|body| cut_out(body, board, board_lo, board_hi));
    match substrate {
        Ok(body) => {
            out.bodies.push((SUBSTRATE_SOLID.to_string(), body));
            out.colors.push((SUBSTRATE_SOLID.to_string(), SUBSTRATE_RGB));
        }
        Err(error) => out.skipped.push(format!("{SUBSTRATE_SOLID}: {error}")),
    }

    // The copper, layer by layer, one solid per primitive.
    let (copper, nets) = copper_and_nets(board, netlist);
    for layer in 0..board.layer_count.max(1) {
        let (lo, hi) = super::copper_span(layer, board.layer_count);
        let layer_name = format!("{BOARD_SOLID_PREFIX}{}", board::layer_name(layer, board.layer_count));
        let mut count = 0usize;
        for (item, net) in copper.iter().zip(&nets) {
            if !(item.layers.0 <= layer && layer <= item.layers.1) {
                continue;
            }
            count += 1;
            let name = format!("{layer_name}/{}#{count}", net_part(net.as_deref()));
            let solid = match item_hole(board, &item.owner) {
                Some(hole) => drilled(&item.shape, hole, lo, hi),
                None => shape_curves(&item.shape, lo).and_then(|curves| prism(&curves, lo, hi)),
            };
            match solid {
                Ok(body) => {
                    out.colors.push((name.clone(), COPPER_RGB));
                    out.bodies.push((name, body));
                }
                Err(error) => out.skipped.push(format!("{name}: {error}")),
            }
        }
        // Each zone's fill, one solid per piece, named by the zone's net.
        for zone in board.zones.iter().filter(|z| z.layer == layer) {
            for piece in &zone.fill {
                count += 1;
                let name = format!("{layer_name}/{}#{count}", net_part(Some(&zone.net)));
                match fill_solid(piece, lo, hi) {
                    Ok(body) => {
                        out.colors.push((name.clone(), COPPER_RGB));
                        out.bodies.push((name, body));
                    }
                    Err(error) => out.skipped.push(format!("{name}: {error}")),
                }
            }
        }
    }

    // The plated via barrels, as tubes: the bore's wall plus the plating.
    for (index, via) in board.vias.iter().enumerate() {
        let name = format!("{VIAS_SOLID}#{}", index + 1);
        let inner = mm(via.drill) / 2.;
        let outer = inner + mm(board::COPPER_THICKNESS);
        let (from, to) = via.layers(board.layer_count);
        let (lo, hi) = (
            mm(board::layer_z(to, board.layer_count)),
            mm(board::layer_z(from, board.layer_count)),
        );
        let barrel = ring(xy(via.at), inner, outer, lo, hi);
        match barrel {
            Ok(body) => {
                out.colors.push((name.clone(), COPPER_RGB));
                out.bodies.push((name, body));
            }
            Err(error) => out.skipped.push(format!("{name}: {error}")),
        }
    }
    Some(out)
}

