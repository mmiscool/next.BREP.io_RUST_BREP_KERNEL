//! Two-lane splitmix64 identity hashes of curves and surfaces (memo keys).

use crate::{NurbsCurve, NurbsSurface, Vec4};


/// The splitmix64 finalizer (Steele/Lea/Flood, "Fast Splittable Pseudorandom
/// Number Generators"): two multiply-xorshift rounds that avalanche every
/// input bit across all 64 output bits.
#[inline]
pub fn mix64(mut z: u64) -> u64 {
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
pub struct TwoLane {
    state: [u64; 2],
}

pub const SEEDS: [u64; 2] = [0x9e37_79b9_7f4a_7c15, 0xc2b2_ae3d_27d4_eb4f];
pub const STEPS: [u64; 2] = [0xa076_1d64_78bd_642f, 0xe703_7ed1_a0b4_28db];

impl TwoLane {
    pub fn new() -> Self {
        Self { state: SEEDS }
    }

    #[inline]
    pub fn u64(&mut self, value: u64) {
        for lane in 0..2 {
            // Order-sensitive by construction: the state enters the mix, so
            // permuting the words permutes the digest.
            self.state[lane] = mix64(self.state[lane] ^ value).wrapping_add(STEPS[lane]);
        }
    }

    #[inline]
    pub fn usize(&mut self, value: usize) {
        self.u64(value as u64);
    }

    #[inline]
    pub fn bool(&mut self, value: bool) {
        self.u64(value as u64);
    }

    /// `to_bits`, not the value: it separates `-0.0` from `0.0` and gives every
    /// NaN payload its own key. Both are what the integrator sees — it can
    /// consume a negative zero (the sign survives a multiply) and it never
    /// normalizes a NaN — so hashing the value would let two inputs the
    /// integrator distinguishes share a key.
    #[inline]
    pub fn f64(&mut self, value: f64) {
        self.u64(value.to_bits());
    }

    pub fn floats(&mut self, values: &[f64]) {
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

    pub fn finish(self) -> [u64; 2] {
        [mix64(self.state[0]), mix64(self.state[1])]
    }
}

pub fn hash_curve(lane: &mut TwoLane, curve: &NurbsCurve) {
    lane.usize(curve.degree);
    lane.floats(&curve.knots);
    lane.usize(curve.control_points.len());
    for point in &curve.control_points {
        lane.control_point(point);
    }
}

pub fn hash_surface(lane: &mut TwoLane, surface: &NurbsSurface) {
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
