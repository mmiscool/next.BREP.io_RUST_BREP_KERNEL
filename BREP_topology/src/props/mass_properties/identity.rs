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
/// not proof — but see the width note on `absorb`: a consumer that serves a
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
#[allow(unused_imports)]
pub use crate::identity_hash::{hash_curve, hash_surface, mix64, TwoLane, SEEDS, STEPS};

/// Everything the per-face tessellator reads off a face record besides its
/// coedges' shared edge samples: the carrier, `same_sense`, and per loop and
/// coedge in order the sense and the pcurve. `watertight_tessellation::face_memo`
/// keys a face's mesh fragment on this plus the samples' bits.
pub fn hash_face(lane: &mut TwoLane, face: &FaceRecord) {
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

/// The 128-bit content identity of one coedge's USE: the face's carrier, the
/// pcurve, the edge's curve and domain and the coedge's sense — every input
/// of a reading that compares the trim's image with the edge (the refit's
/// detection miss, the challenger's old-trim tracking). Two uses with equal
/// identities read the same number, bit for bit; `reading_memo.rs` keys on it.
pub fn coedge_use_identity(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &crate::topology::EdgeRecord,
    forward: bool,
) -> [u64; 2] {
    let mut lane = TwoLane::new();
    hash_surface(&mut lane, surface);
    hash_curve(&mut lane, pcurve);
    hash_curve(&mut lane, &edge.curve);
    lane.f64(edge.t0);
    lane.f64(edge.t1);
    lane.bool(edge.degenerate);
    lane.bool(forward);
    lane.finish()
}

/// Inputs of assembly's sampled edge-to-carrier residual. IDs, trims and
/// tolerance are absent because the reader does not consult them. Preserve
/// carrier order, represented interval, and every geometric bit.
pub fn carrier_residual_identity(
    curve: &NurbsCurve, t0: f64, t1: f64, surfaces: &[&NurbsSurface],
) -> [u64; 2] {
    let mut lane = TwoLane::new();
    hash_curve(&mut lane, curve);
    lane.f64(t0);
    lane.f64(t1);
    lane.usize(surfaces.len());
    for surface in surfaces { hash_surface(&mut lane, surface); }
    lane.finish()
}

/// `key` with a tolerance folded in: the identity of a reading that is a pure
/// function of a use's content AND one policy value (the refit's candidacy
/// standoff, read against `policy.model`). The same content under another
/// model is another key.
pub fn identity_with_model(key: [u64; 2], model: f64) -> [u64; 2] {
    let mut lane = TwoLane::new();
    lane.u64(key[0]);
    lane.u64(key[1]);
    lane.f64(model);
    lane.finish()
}

/// The 128-bit identity of an EDGE's scope claim (`heal::edge_claim_with`):
/// the identities of the edge's uses in the order the claim is handed them,
/// the order in which it visits their distinct carriers (`ranks`, the first
/// use of each carrier in that visiting order) and the model tolerance.
pub fn edge_claim_identity(uses: &[[u64; 2]], ranks: &[usize], model: f64) -> [u64; 2] {
    let mut lane = TwoLane::new();
    lane.usize(uses.len());
    for key in uses {
        lane.u64(key[0]);
        lane.u64(key[1]);
    }
    lane.usize(ranks.len());
    for &rank in ranks {
        lane.usize(rank);
    }
    lane.f64(model);
    lane.finish()
}

/// The 128-bit content identity of one face's volume contribution about
/// `reference` under the sign reading's correction policy: everything
/// `face_volume_contribution_about` reads (the face as `hash_face` walks it,
/// the reference point's bits). `reading_memo.rs` keys the boolean's
/// assembly sign check on it.
pub fn face_contribution_identity(face: &FaceRecord, reference: crate::Vec3) -> [u64; 2] {
    let mut lane = TwoLane::new();
    hash_face(&mut lane, face);
    lane.f64(reference.x);
    lane.f64(reference.y);
    lane.f64(reference.z);
    lane.finish()
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
