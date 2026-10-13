//! Recentring.
//!
//! The catalogue's decision is to EXTEND our translation control first — the
//! four-operation vendor control passes at offsets 0, 10, 100 and 1000 — and
//! wrap the boolean and fillet entries in a recentre guard only if an offset
//! moves a result.
//!
//! This module is that control, applied at the kernel's own entry points so
//! every case document of the manifest exercises it: with
//! `BREP_TRANSLATION_CONTROL=<offset>` set, [`boolean_operation_with_diagnostics`]
//! and [`fillet_edges_reported`] translate their operands by `(offset,
//! offset, offset)`, run, and translate the result back, at the OUTERMOST
//! entry only (a boolean the fillet runs inside, or the perturbation retry
//! the boolean runs inside itself, sees the already-translated operands). The
//! case gate's comparison against its baseline then reads `Same` or names
//! the row and the number that moved.
//!
//! The control without the guard moved results at every offset (the figures
//! are in the acceptance-cost record of 2026-10-10: box ∪ sphere loses two
//! edges at 1e6, the box fillet comes back incomplete, and manifest rows
//! refuse at 1e4), so the guard is in: [`guard_offset`] recentres an
//! operation whose operands' combined box centre lies further from the
//! origin than [`RECENTRE_FACTOR`] times the box diagonal, by translating
//! the operands so that centre is the origin, running, and translating the
//! result back. The threshold is relative to the body's own size, as the
//! catalogue decides; a body at the origin, or one whose distance is within
//! a few diagonals, never fires it. `BREP_RECENTRE` sets its mode
//! ([`GuardMode`]: on by default since its promotion, `0` off, `always` for
//! the census). The control and the guard nest: a translated control entry
//! still lets the guard fire.
//!
//! A guard that moves a body to the origin changes its placement, and the
//! kernel is not placement-invariant at small offsets: box ∪ sphere at the
//! origin has 16 edges (two degenerate pole edges) and at `(-7, 0, 0)` has
//! 14, with volumes 3.3e-5 either side of the closed form (the record's
//! probe). So the control WITH the guard reads `Same` only where the kernel
//! would read `Same` under that small placement change; the record's census
//! separates the two with the `always` arm.
//!
//! [`boolean_operation_with_diagnostics`]: crate::boolean_operation_with_diagnostics
//! [`fillet_edges_reported`]: crate::fillet_edges_reported

use crate::topology::BrepSolid;
use crate::transform_topology::{transform_curve, transform_surface};
use crate::{Aabb, AffineTransform, Vec3};
use std::cell::Cell;

/// The guard fires when the combined box centre is further from the origin
/// than this many box diagonals. The vendor four-operation control passes at
/// offset 1000 on a body about 20 across (50 diagonals); the manifest
/// control without the guard moves rows at 1e4 (see the record), so the
/// factor sits well under both.
pub(crate) const RECENTRE_FACTOR: f64 = 32.0;

/// The pure translation by `offset`.
pub(crate) fn translation(offset: Vec3) -> Result<AffineTransform, String> {
    AffineTransform::new([
        1.0, 0.0, 0.0, offset.x, 0.0, 1.0, 0.0, offset.y, 0.0, 0.0, 1.0, offset.z, 0.0, 0.0, 0.0, 1.0,
    ])
}

/// `solid` moved by `offset`: vertices, edge curves and carriers translated,
/// ids, names, loops, coedges and pcurves untouched, the mass cache dropped.
/// Unlike [`crate::transform_brep`] this does not validate the result: the
/// operand of a boolean may carry issues its healing stage repairs, and a
/// translation adds none.
pub(crate) fn translate_solid(solid: &BrepSolid, offset: Vec3) -> Result<BrepSolid, String> {
    let transform = translation(offset)?;
    let mut result = solid.clone();
    result.mass_properties_cache = Default::default();
    for vertex in &mut result.vertices {
        vertex.point = transform.point(vertex.point);
    }
    for edge in &mut result.edges {
        edge.curve = transform_curve(&edge.curve, transform)?;
    }
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            face.surface = transform_surface(&face.surface, transform)?;
        }
    }
    Ok(result)
}

/// The control's offset, `BREP_TRANSLATION_CONTROL=<d>` read once; `None`
/// when unset, empty, zero or unparsable, and always `None` in the browser.
#[cfg(not(target_arch = "wasm32"))]
fn translation_control_offset() -> Option<Vec3> {
    static OFFSET: std::sync::OnceLock<Option<Vec3>> = std::sync::OnceLock::new();
    *OFFSET.get_or_init(|| {
        let value = std::env::var("BREP_TRANSLATION_CONTROL").ok()?;
        let offset: f64 = value.trim().parse().ok()?;
        (offset != 0.0 && offset.is_finite()).then(|| Vec3::new(offset, offset, offset))
    })
}

#[cfg(target_arch = "wasm32")]
fn translation_control_offset() -> Option<Vec3> {
    None
}

/// How the guard is armed. `BREP_RECENTRE` reads it: empty or `0` is `Off`
/// (the A/B lever), `always` is `Always` (every boolean and fillet recentred
/// whatever its distance: the census's forced arm, which separates a row
/// the guard moved from a row that moves under any small placement change),
/// anything else or unset is `On` — the promoted default, on the census in
/// the acceptance-cost record of 2026-10-10 (every manifest operation
/// recentred reads the baseline's 83 correct / 11 success / 1 refused).
/// Never read in the browser, where the mode is the default.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum GuardMode {
    Off,
    On,
    Always,
}

const DEFAULT_GUARD_MODE: GuardMode = GuardMode::On;

#[cfg(not(target_arch = "wasm32"))]
fn guard_mode_from_env() -> GuardMode {
    static MODE: std::sync::OnceLock<GuardMode> = std::sync::OnceLock::new();
    *MODE.get_or_init(|| match std::env::var("BREP_RECENTRE").as_deref() {
        Err(_) => DEFAULT_GUARD_MODE,
        Ok("") | Ok("0") => GuardMode::Off,
        Ok("always") => GuardMode::Always,
        Ok(_) => GuardMode::On,
    })
}

#[cfg(target_arch = "wasm32")]
fn guard_mode_from_env() -> GuardMode {
    DEFAULT_GUARD_MODE
}

thread_local! {
    /// A test's override of the mode on its own thread (the environment is
    /// process-wide and read once).
    static GUARD_MODE_OVERRIDE: Cell<Option<GuardMode>> = const { Cell::new(None) };
}

fn guard_mode() -> GuardMode {
    GUARD_MODE_OVERRIDE.with(|cell| cell.get()).unwrap_or_else(guard_mode_from_env)
}




/// The bounds of a solid's vertices and surface control nets; infinite when
/// a surface's box cannot be read (the guard then never fires).
pub(crate) fn solid_bounds(solid: &BrepSolid) -> Aabb {
    let mut bounds = Aabb::from_points(solid.vertices.iter().map(|vertex| vertex.point));
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        match Aabb::from_surface_controls(&face.surface) {
            Ok(face_bounds) => bounds.include(face_bounds),
            Err(_) => {
                return Aabb {
                    minimum: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
                    maximum: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
                }
            }
        }
    }
    bounds
}

/// The translation the guard applies to an operation over `bounds`: minus
/// the box centre when that centre is more than [`RECENTRE_FACTOR`]
/// diagonals from the origin, `None` otherwise, when the guard is off, when
/// a guarded entry is already open on this thread, or when the bounds are
/// not finite.
pub(crate) fn guard_offset(bounds: Aabb) -> Option<Vec3> {
    let mode = guard_mode();
    if mode == GuardMode::Off || GUARD_DEPTH.with(|depth| depth.get()) > 0 {
        return None;
    }
    let centre = bounds.minimum.add(bounds.maximum).scale(0.5);
    let diagonal = bounds.diagonal();
    let distance = centre.length();
    if !(diagonal.is_finite() && diagonal > 0.0 && distance.is_finite()) {
        return None;
    }
    (mode == GuardMode::Always || distance > RECENTRE_FACTOR * diagonal).then(|| centre.scale(-1.0))
}

/// One `recentre.fired` line on stderr under `BREP_PROFILE`, the switch the
/// boolean's and the mass integrator's profiles read.
pub(crate) fn profile_fired(entry: &str, offset: Vec3, bounds: Aabb) {
    if std::env::var("BREP_PROFILE").is_ok() {
        eprintln!(
            "recentre.fired entry={entry} distance={:.4e} diagonal={:.4e}",
            offset.length(),
            bounds.diagonal()
        );
    }
}

thread_local! {
    /// How many translated CONTROL entries are open on this thread: the
    /// control applies only when none is.
    static CONTROL_DEPTH: Cell<u32> = const { Cell::new(0) };
    /// How many GUARDED entries are open: the guard fires only when none
    /// is (the recentred operands sit at the origin anyway).
    static GUARD_DEPTH: Cell<u32> = const { Cell::new(0) };
}

/// The offset an entry point should translate by: the control's, when it is
/// set and no translated control entry is already open on this thread.
pub(crate) fn control_offset_at_entry() -> Option<Vec3> {
    if CONTROL_DEPTH.with(|depth| depth.get()) > 0 {
        return None;
    }
    translation_control_offset()
}

/// Which of the two translating callers opened an entry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum EntryKind {
    Control,
    Guard,
}

/// A translated entry, open until dropped.
pub(crate) struct TranslatedEntry(EntryKind);

impl TranslatedEntry {
    pub(crate) fn open(kind: EntryKind) -> Self {
        let depth = match kind {
            EntryKind::Control => &CONTROL_DEPTH,
            EntryKind::Guard => &GUARD_DEPTH,
        };
        depth.with(|depth| depth.set(depth.get() + 1));
        Self(kind)
    }
}

impl Drop for TranslatedEntry {
    fn drop(&mut self) {
        let depth = match self.0 {
            EntryKind::Control => &CONTROL_DEPTH,
            EntryKind::Guard => &GUARD_DEPTH,
        };
        depth.with(|depth| depth.set(depth.get().saturating_sub(1)));
    }
}

