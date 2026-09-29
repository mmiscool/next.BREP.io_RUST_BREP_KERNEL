//! The content identity of the face set the integrator actually reads.
//!
//! Two `solid_mass_properties` / `solid_signed_volume` calls return the same
//! bits exactly when they see the same geometry in the same order. This hash
//! is that statement, made checkable: it walks precisely the fields the
//! integrator reads and nothing else, in the order it reads them.
//!
//! **What goes in.** Per shell in order, per face in order: the carrier's
//! degrees, knot vectors and control points, `same_sense`, then per loop in
//! order and per coedge in order the pcurve's degree, knots and control
//! points and the coedge's `forward` flag.
//!
//! **What stays out.** `id` and `name` on every record, and the solid's
//! `vertices` and `edges` — `solid_props.rs` never reads them, so a solid
//! whose edge table was renumbered integrates to the same bits and must hash
//! the same. `genus` is topology bookkeeping the integrator never consults.
//!
//! **Order, not set.** Both entry points run a compensated sum over the faces
//! in shell order; permuting the faces permutes the addends and can move the
//! last bits. So the hash is over the SEQUENCE. "The same face set" is a
//! station-signature statement; this is the stronger one that licenses
//! serving a stored value.
//!
//! **Why the bits and not the value.** `f64::to_bits` distinguishes `-0.0`
//! from `0.0` and makes every NaN payload its own key. Both are what the
//! integrator sees: it can consume a negative zero (the sign survives a
//! multiply) and it never normalizes a NaN. Hashing the value would let two
//! inputs the integrator distinguishes share a key.

use crate::{BrepSolid, FaceRecord, NurbsCurve, NurbsSurface, Vec4};

/// The 128-bit content identity of a solid's face set.
///
/// Two hashes are equal when the two solids present the integrator with the
/// same geometry in the same order. It is a hash, so equality is evidence and
/// not proof — but see the width note on [`absorb`]: a consumer that serves a
/// stored value on a hit is betting on 2^-128, and this file owes that bet a
/// mixer worth the number.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct FaceSetIdentity {
    lanes: [u64; 2],
    /// Faces walked, carried beside the digest as a cheap first discriminator
    /// and as the thing a profile line can print.
    pub faces: u64,
}

impl FaceSetIdentity {
    pub fn hex(&self) -> String {
        format!("{:016x}{:016x}", self.lanes[0], self.lanes[1])
    }
}

/// The splitmix64 finalizer (Steele/Lea/Flood, "Fast Splittable Pseudorandom
/// Number Generators"): two multiply-xorshift rounds that avalanche every
/// input bit across all 64 output bits.
#[inline]
fn mix64(mut z: u64) -> u64 {
    z ^= z >> 30;
    z = z.wrapping_mul(0xbf58_476d_1ce4_e5b9);
    z ^= z >> 27;
    z = z.wrapping_mul(0x94d0_49bb_1331_11eb);
    z ^ (z >> 31)
}

/// Two independent 64-bit streams over the same word sequence, distinguished
/// by their seed and their per-word increment.
///
/// **Why not `FxHasher`.** The crate's fast hasher is a rotate-xor-multiply
/// per word with no finalizer; it is the right tool for a `HashMap` bucket
/// index and the wrong one here. Control-point words are highly structured —
/// exact zeros, exact ones, repeated knots, shared coordinates — and a weak
/// mixer's collisions cluster on exactly that kind of input. A collision here
/// does not cost a slow lookup, it serves a WRONG VOLUME silently. splitmix64
/// per word, twice, is a few ns per station's worth of data against an
/// integrator that spends 335 ns per station.
struct TwoLane {
    state: [u64; 2],
}

const SEEDS: [u64; 2] = [0x9e37_79b9_7f4a_7c15, 0xc2b2_ae3d_27d4_eb4f];
const STEPS: [u64; 2] = [0xa076_1d64_78bd_642f, 0xe703_7ed1_a0b4_28db];

impl TwoLane {
    fn new() -> Self {
        Self { state: SEEDS }
    }

    #[inline]
    fn u64(&mut self, value: u64) {
        for lane in 0..2 {
            // Order-sensitive by construction: the state enters the mix, so
            // permuting the words permutes the digest.
            self.state[lane] = mix64(self.state[lane] ^ value).wrapping_add(STEPS[lane]);
        }
    }

    #[inline]
    fn usize(&mut self, value: usize) {
        self.u64(value as u64);
    }

    #[inline]
    fn bool(&mut self, value: bool) {
        self.u64(value as u64);
    }

    /// `to_bits`, not the value: it separates `-0.0` from `0.0` and gives every
    /// NaN payload its own key. Both are what the integrator sees — it can
    /// consume a negative zero (the sign survives a multiply) and it never
    /// normalizes a NaN — so hashing the value would let two inputs the
    /// integrator distinguishes share a key.
    #[inline]
    fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    fn floats(&mut self, values: &[f64]) {
        // Length first: without it `[a], [b, c]` and `[a, b], [c]` are the
        // same word stream.
        self.usize(values.len());
        for value in values {
            self.f64(*value);
        }
    }

    fn control_point(&mut self, point: &Vec4) {
        self.f64(point.x);
        self.f64(point.y);
        self.f64(point.z);
        self.f64(point.w);
    }

    fn finish(self) -> [u64; 2] {
        [mix64(self.state[0]), mix64(self.state[1])]
    }
}

fn hash_curve(lane: &mut TwoLane, curve: &NurbsCurve) {
    lane.usize(curve.degree);
    lane.floats(&curve.knots);
    lane.usize(curve.control_points.len());
    for point in &curve.control_points {
        lane.control_point(point);
    }
}

fn hash_surface(lane: &mut TwoLane, surface: &NurbsSurface) {
    lane.usize(surface.degree_u);
    lane.usize(surface.degree_v);
    lane.floats(&surface.knots_u);
    lane.floats(&surface.knots_v);
    lane.usize(surface.control_points.len());
    for row in &surface.control_points {
        lane.usize(row.len());
        for point in row {
            lane.control_point(point);
        }
    }
}

fn hash_face(lane: &mut TwoLane, face: &FaceRecord) {
    hash_surface(lane, &face.surface);
    lane.bool(face.same_sense);
    lane.usize(face.loops.len());
    for loop_record in &face.loops {
        lane.usize(loop_record.coedges.len());
        for coedge in &loop_record.coedges {
            lane.bool(coedge.forward);
            hash_curve(lane, &coedge.pcurve);
        }
    }
}

/// Hash exactly what the integrator reads, in the order it reads it.
pub fn face_set_identity(solid: &BrepSolid) -> FaceSetIdentity {
    let mut lane = TwoLane::new();
    let mut faces = 0u64;
    lane.usize(solid.shells.len());
    for shell in &solid.shells {
        lane.usize(shell.faces.len());
        for face in &shell.faces {
            faces += 1;
            hash_face(&mut lane, face);
        }
    }
    FaceSetIdentity {
        lanes: lane.finish(),
        faces,
    }
}
