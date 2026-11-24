//! Whether a part's 3D model sits on its own footprint: each pcb connection
//! point against the pad with its pin number (the pin seat check, "Pin
//! seats on pads").
//!
//! The geometry is measured by the caller, which has the part's solids: this
//! module takes what the pin's 3D model puts on the board, already in the
//! footprint's coordinates (micrometres, y down, as the pads are), and says
//! whether it lands on its pad. A miss is a finding, never a refusal.
//!
//! * A **contact body** (a pin's own solid) is read by its SECTION: where it
//!   meets the top copper (the slab `|z| <= TOP_SLAB` in the part's frame) for
//!   a surface-mount pad, and where it runs through the board (`z` in
//!   `[-thickness, 0]`) for a drilled one. The whole section must lie on the
//!   copper, or inside the drill, within [`SEAT_TOLERANCE`]; the miss is the
//!   farthest any of it strays. There is no direction rule: a right-angle
//!   header's pin meets the board only at its tail, and that is what is read.
//! * A point **seated** on a face, edge or vertex (no contact body) is read by
//!   position: the seat projected along the board normal must land on the pad.

use crate::board::{Footprint, Pad, PadShape, Shape};
use crate::Point;

/// How far a seat may stray off its pad or drill and still pass, in
/// micrometres: modelling slack, not a fit.
pub const SEAT_TOLERANCE: i32 = 50;
/// Half the height of the slab a surface-mount pin is read in, about the top
/// copper (z = 0), in micrometres.
pub const TOP_SLAB: i32 = 50;
/// The board thickness a through-hole pin is read through when there is no
/// board to take it from (a part on its own, in Qualify), in micrometres.
pub const ASSUMED_THICKNESS: i32 = 1_600;

/// What a pin's 3D model puts on the board, in footprint coordinates.
#[derive(Clone, Debug, PartialEq)]
pub enum SeatContact {
    /// A point seated on a face, edge or vertex, projected onto the board.
    Seat(Point),
    /// A contact body, by its section.
    Body {
        /// Its section in the top-copper slab.
        top: Vec<Point>,
        /// Its section through the board's thickness.
        through: Vec<Point>,
        /// Its lowest point's height above the top copper, in micrometres
        /// (negative below it).
        lowest: i32,
        /// The thickness `through` was read over, in micrometres.
        thickness: i32,
    },
}

/// The verdict for one pin.
#[derive(Clone, Debug, PartialEq)]
pub enum SeatVerdict {
    /// It lands on (or through) its pad.
    OnPad,
    /// The footprint has no pad with the pin's number.
    NoPad,
    /// Its seat or section strays `by` micrometres off the pad's copper.
    OffPad { by: i32 },
    /// Its section through the board strays `by` micrometres outside the drill.
    OffDrill { by: i32 },
    /// The contact body does not reach the board: its lowest point is
    /// `lowest` micrometres above the top copper.
    NoReach { lowest: i32 },
    /// The contact body meets the top copper but does not run into the
    /// drilled pad's hole.
    NotThrough,
    /// The contact body lands on pads of several pins: a body of the whole
    /// connector rather than of this pin.
    SpansPads { pads: Vec<String> },
}

/// A pad's copper in footprint coordinates.
fn copper(pad: &Pad) -> Shape {
    match pad.corners() {
        Some(corners) => Shape::polygon(corners, pad.effective_corner_radius()),
        None => Shape::rounded_pad(pad.at, pad.size, pad.shape, pad.effective_corner_radius()),
    }
}

/// A drilled pad's hole; `None` for a surface-mount pad.
fn hole(pad: &Pad) -> Option<Shape> {
    let drill = pad.drill?;
    Some(match pad.drill_height {
        Some(height) => Shape::rounded_pad(pad.at, Point::new(drill, height), PadShape::Oval, 0),
        None => Shape::circle(pad.at, drill / 2),
    })
}

/// The farthest any of `points` lies off `shape`, in micrometres (0 inside).
fn excursion(shape: &Shape, points: &[Point]) -> f64 {
    points.iter().map(|p| shape.distance_to_point(*p)).fold(0., f64::max)
}

/// Check pin `pin`'s contact on `footprint`. Several pads sharing the number
/// are alternatives: landing on any one passes, and a miss is measured to the
/// nearest.
pub fn pad_seat_check(footprint: &Footprint, pin: &str, contact: &SeatContact) -> SeatVerdict {
    let pin = pin.trim();
    let pads: Vec<&Pad> = footprint.pads.iter().filter(|pad| pad.number.trim() == pin).collect();
    if pads.is_empty() {
        return SeatVerdict::NoPad;
    }
    let tolerance = f64::from(SEAT_TOLERANCE);
    let round = |by: f64| by.round() as i32;
    match contact {
        SeatContact::Seat(at) => {
            let by = pads.iter().map(|pad| copper(pad).distance_to_point(*at)).fold(f64::INFINITY, f64::min);
            if by <= tolerance { SeatVerdict::OnPad } else { SeatVerdict::OffPad { by: round(by) } }
        }
        SeatContact::Body { top, through, lowest, .. } => {
            // A body standing on several pins' pads is the connector's, not this pin's.
            let mut touched: Vec<String> = footprint
                .pads
                .iter()
                .filter(|pad| !pad.number.trim().is_empty())
                .filter(|pad| top.iter().chain(through).any(|p| copper(pad).distance_to_point(*p) <= 0.))
                .map(|pad| pad.number.trim().to_owned())
                .collect();
            touched.sort();
            touched.dedup();
            if touched.len() > 1 {
                return SeatVerdict::SpansPads { pads: touched };
            }
            let drilled: Vec<(&Pad, Shape)> = pads.iter().filter_map(|pad| Some((*pad, hole(pad)?))).collect();
            if !drilled.is_empty() {
                if through.is_empty() {
                    return if top.is_empty() && *lowest > TOP_SLAB {
                        SeatVerdict::NoReach { lowest: *lowest }
                    } else {
                        SeatVerdict::NotThrough
                    };
                }
                let by = drilled.iter().map(|(_, hole)| excursion(hole, through)).fold(f64::INFINITY, f64::min);
                return if by <= tolerance { SeatVerdict::OnPad } else { SeatVerdict::OffDrill { by: round(by) } };
            }
            if top.is_empty() {
                return if *lowest > TOP_SLAB {
                    SeatVerdict::NoReach { lowest: *lowest }
                } else {
                    // It runs into the board where a surface-mount pad has no hole.
                    let by = pads.iter().map(|pad| excursion(&copper(pad), through)).fold(f64::INFINITY, f64::min);
                    SeatVerdict::OffPad { by: round(by.max(tolerance + 1.)) }
                };
            }
            let by = pads.iter().map(|pad| excursion(&copper(pad), top)).fold(f64::INFINITY, f64::min);
            if by <= tolerance { SeatVerdict::OnPad } else { SeatVerdict::OffPad { by: round(by) } }
        }
    }
}

/// Millimetres from micrometres, two decimals.
fn mm(um: i32) -> String {
    format!("{:.2}", f64::from(um) / 1_000.)
}

impl SeatVerdict {
    pub fn passes(&self) -> bool {
        *self == SeatVerdict::OnPad
    }

    /// The finding in words, `None` when the pin is on its pad. `thickness` is
    /// named when it was assumed rather than read from a board.
    pub fn message(&self, pin: &str, contact: &SeatContact, assumed: bool) -> Option<String> {
        let thickness = match contact {
            SeatContact::Body { thickness, .. } if assumed => format!(" (read through an assumed {} mm board)", mm(*thickness)),
            _ => String::new(),
        };
        Some(match self {
            SeatVerdict::OnPad => return None,
            SeatVerdict::NoPad => format!("Pin {pin} has no pad: the footprint has no pad numbered {pin}"),
            SeatVerdict::OffPad { by } => format!("Pin {pin} seats {} mm outside pad {pin}", mm(*by)),
            SeatVerdict::OffDrill { by } => format!("Pin {pin} passes {} mm outside the drill of pad {pin}{thickness}", mm(*by)),
            SeatVerdict::NoReach { lowest } => {
                format!("Pin {pin} does not reach the board: its contact body ends {} mm above it", mm(*lowest))
            }
            SeatVerdict::NotThrough => format!("Pin {pin} does not go into the drill of pad {pin}{thickness}"),
            SeatVerdict::SpansPads { pads } => format!(
                "Pin {pin}'s contact body lands on pads {}: name the pin's own solid, or seat the point on a face",
                pads.join(", ")
            ),
        })
    }
}

