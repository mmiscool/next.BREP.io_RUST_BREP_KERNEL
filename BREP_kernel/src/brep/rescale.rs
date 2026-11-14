//! Working scale: operations on a model much smaller than one unit run on a
//! copy scaled up by a power of two, and their result is scaled back.
//!
//! The kernel's identity, weld and coincidence bands are tuned for models of
//! unit size and larger: the model scale they are measured against
//! ([`crate::tolerance::solid_scale`]) is floored at 1, and beside it stand
//! absolute floors (the weld floor 1e-5, the coincidence floor 1e-6, the fit
//! accuracy 2e-6). A part of size 0.001 sits almost wholly inside those bands:
//! a cube of side 0.001 united with a copy raised 3e-6 came back as the cube
//! alone, reporting valid, and a cylinder of radius 0.005 would not grow by
//! 0.001 where the same shape a thousand times larger builds exactly.
//!
//! No floor is moved here. Under `BREP_WORKING_SCALE` (default on, `=0` off) an entry whose operands'
//! combined box diagonal is below 1/16 multiplies every coordinate by the power
//! of two that brings the diagonal into `[1, 2)`, runs, and multiplies the
//! result by the inverse. A power of two changes exponents only, so both
//! scalings are exact: the result is bit for bit the one the same shape of
//! that larger size gives, and every band keeps the proportion to the part it
//! has at unit size. Operands of diagonal 1/16 and above are never touched:
//! between 1/16 and 1 a model keeps the absolute bands, which stand up to 16
//! times coarser against it than against a unit model.
//!
//! A carrier parameter that is itself a length scales with the model: a plane
//! is built over `[0, width] x [0, height]`, and left at that domain a plane
//! scaled 2^20 times would move 2^20 units per unit of parameter, which the
//! parametric bands do not expect either. Its knots and its pcurves'
//! coordinates take the same power of two.
//!
//! Lengths the caller passes (an offset distance) are scaled with the
//! operands. The tolerance policy is not: it is the kernel's working precision
//! and applies at the working scale. Refusal messages name working-scale
//! coordinates and say by what factor.
//!
//! The guard reads the operands' box alone, at every entry: a boolean an
//! offset or a fillet runs inside itself on a piece that small is
//! scaled like any other, whatever the size of the model it came from.

use crate::topology::BrepSolid;
use crate::{Aabb, KernelRefusal, NurbsCurve, NurbsSurface, Vec4};

/// `BREP_WORKING_SCALE` (default on, `=0` off): operations on a model of box
/// diagonal below [`FIRING_DIAGONAL`] run at a power-of-two working scale.
pub(crate) fn enabled() -> bool {
    crate::switch!("BREP_WORKING_SCALE", true)
}

/// The diagonal a scaled model is brought to: into
/// `[WORKING_DIAGONAL, 2 * WORKING_DIAGONAL)`. It is the floor of
/// [`crate::tolerance::solid_scale`]: under it the bands stop following the
/// model.
pub(crate) const WORKING_DIAGONAL: f64 = 1.0;

/// Models whose box diagonal is below this run at working scale. A model
/// between this and [`WORKING_DIAGONAL`] is left as it is: the absolute bands
/// stand at most 16 times coarser against it than against a unit model, and
/// its result is the one it has always had.
pub(crate) const FIRING_DIAGONAL: f64 = 0.0625;

/// The smallest diagonal the guard scales. Below it the model is no larger
/// than round-off of ordinary coordinates and is left to the entry's own
/// refusals.
const SMALLEST_DIAGONAL: f64 = 1e-150;

/// The power of two an operation over `bounds` is scaled by: the one that
/// brings the box diagonal into `[1, 2)` when it is below
/// [`FIRING_DIAGONAL`]; `None` for a diagonal at or above it, with the switch off, or when the bounds are not
/// finite. It reads the operands' box and nothing else, so an operation an
/// outer one runs inside itself is scaled by the same rule whoever called it;
/// operands already at working scale have a diagonal of at least 1 and are
/// left alone.
pub(crate) fn guard_factor(bounds: Aabb) -> Option<f64> {
    if !enabled() {
        return None;
    }
    factor_for_diagonal(bounds.diagonal())
}

/// See [`guard_factor`]; the switch is not consulted.
pub(crate) fn factor_for_diagonal(diagonal: f64) -> Option<f64> {
    if !(diagonal.is_finite() && diagonal >= SMALLEST_DIAGONAL && diagonal < FIRING_DIAGONAL) {
        return None;
    }
    // frexp by hand: the exponent e with diagonal * 2^e in [1, 2).
    let mut factor = 1.0f64;
    let mut scaled = diagonal;
    while scaled < WORKING_DIAGONAL {
        scaled *= 2.0;
        factor *= 2.0;
    }
    Some(factor)
}

/// The combined bounds of `solids` (vertices and surface control nets).
pub(crate) fn bounds_of<'a>(solids: impl IntoIterator<Item = &'a BrepSolid>) -> Option<Aabb> {
    let mut all: Option<Aabb> = None;
    for solid in solids {
        if solid.vertices.is_empty() && solid.shells.iter().all(|shell| shell.faces.is_empty()) {
            continue;
        }
        let bounds = crate::recentre::solid_bounds(solid);
        match &mut all {
            Some(all) => all.include(bounds),
            None => all = Some(bounds),
        }
    }
    all
}

fn scale_controls(points: &[Vec4], factor: f64) -> Vec<Vec4> {
    // Control points are stored weighted (x·w, y·w, z·w, w): the scaled point
    // has the same weight.
    points.iter().map(|p| Vec4 { x: p.x * factor, y: p.y * factor, z: p.z * factor, w: p.w }).collect()
}

fn scale_curve(curve: &NurbsCurve, factor: f64) -> Result<NurbsCurve, String> {
    NurbsCurve::new(curve.degree, curve.knots.clone(), scale_controls(&curve.control_points, factor))
}

/// Whether `surface` is parameterised by length along u and along v: the
/// point moves one unit per unit of parameter (a plane built over
/// `[0, width] x [0, height]`). Such a parameter is a length of the model and
/// is scaled with it; a parameter that is not (an angle, a `[0, 1]` blend) is
/// left alone. Read from chords at a three-by-three lattice of the domain.
fn length_parameters(surface: &NurbsSurface) -> (bool, bool) {
    let (Some(&u0), Some(&u1), Some(&v0), Some(&v1)) =
        (surface.knots_u.first(), surface.knots_u.last(), surface.knots_v.first(), surface.knots_v.last())
    else {
        return (false, false);
    };
    let (du, dv) = ((u1 - u0) / 64.0, (v1 - v0) / 64.0);
    if !(du > 0.0 && dv > 0.0) {
        return (false, false);
    }
    let (mut along_u, mut along_v) = (true, true);
    for i in 0..3 {
        for j in 0..3 {
            let u = u0 + (u1 - u0) * (0.25 + 0.25 * i as f64);
            let v = v0 + (v1 - v0) * (0.25 + 0.25 * j as f64);
            let (Ok(here), Ok(next_u), Ok(next_v)) =
                (surface.evaluate(u, v), surface.evaluate(u + du, v), surface.evaluate(u, v + dv))
            else {
                return (false, false);
            };
            along_u &= (next_u.sub(here).length() / du - 1.0).abs() <= UNIT_SPEED_BAND;
            along_v &= (next_v.sub(here).length() / dv - 1.0).abs() <= UNIT_SPEED_BAND;
        }
    }
    (along_u, along_v)
}

/// How far from one the chord per unit of parameter may stand for the
/// parameter to be read as a length.
const UNIT_SPEED_BAND: f64 = 1e-9;

/// `surface` scaled by `factor`, with which of its parameters were scaled.
fn scale_surface(surface: &NurbsSurface, factor: f64) -> Result<(NurbsSurface, bool, bool), String> {
    let (along_u, along_v) = length_parameters(surface);
    let knots = |knots: &[f64], scaled: bool| -> Vec<f64> {
        if scaled { knots.iter().map(|knot| knot * factor).collect() } else { knots.to_vec() }
    };
    let scaled = NurbsSurface::new(
        surface.degree_u,
        surface.degree_v,
        knots(&surface.knots_u, along_u),
        knots(&surface.knots_v, along_v),
        surface.control_points.iter().map(|row| scale_controls(row, factor)).collect(),
    )?;
    Ok((scaled, along_u, along_v))
}

/// `solid` scaled about the origin by `factor`: vertices, edge curves and
/// carriers multiplied; a carrier parameter that is a length
/// ([`length_parameters`]) is multiplied too, with the pcurves' coordinates
/// along it. Ids, names, loops, edge parameters and pcurve domains are
/// untouched, and the mass cache is dropped. Not validated: an operand may
/// carry issues the operation's healing repairs, and a uniform scaling adds
/// none. Exact when `factor` is a power of two.
pub(crate) fn scale_solid(solid: &BrepSolid, factor: f64) -> Result<BrepSolid, String> {
    let mut result = solid.clone();
    result.mass_properties_cache = Default::default();
    for vertex in &mut result.vertices {
        vertex.point = vertex.point.scale(factor);
    }
    for edge in &mut result.edges {
        edge.curve = scale_curve(&edge.curve, factor)?;
    }
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            let (surface, along_u, along_v) = scale_surface(&face.surface, factor)?;
            face.surface = surface;
            if !(along_u || along_v) {
                continue;
            }
            let (fu, fv) = (if along_u { factor } else { 1.0 }, if along_v { factor } else { 1.0 });
            for coedge in face.loops.iter_mut().flat_map(|loop_record| &mut loop_record.coedges) {
                // Pcurve controls are weighted (u·w, v·w, 0, w).
                let controls = coedge.pcurve.control_points.iter().map(|p| Vec4 { x: p.x * fu, y: p.y * fv, z: p.z, w: p.w }).collect();
                coedge.pcurve = NurbsCurve::new(coedge.pcurve.degree, coedge.pcurve.knots.clone(), controls)?;
            }
        }
    }
    Ok(result)
}

/// [`scale_solid`] with a failure named as a refusal of `entry`.
pub(crate) fn scale_operand(solid: &BrepSolid, factor: f64, entry: &str) -> Result<BrepSolid, KernelRefusal> {
    scale_solid(solid, factor).map_err(|message| {
        KernelRefusal::internal(
            crate::KernelStage::Collect,
            "working_scale",
            format!("{entry}: the working scale could not scale a body by {factor}: {message}"),
        )
    })
}

/// A refusal raised at working scale, saying so: its coordinates and lengths
/// are `factor` times the model's.
pub(crate) fn name_scale(error: KernelRefusal, factor: f64) -> KernelRefusal {
    error.with_message(|message| format!("{message} (lengths at the working scale, {factor} times the model's)"))
}

/// The factor for an operation on `source` that moves its faces by
/// `distance`: measured on the box grown by the distance on every side, so a
/// small seed grown far is not scaled past unit size.
pub(crate) fn offset_factor(source: &BrepSolid, distance: f64) -> Option<(f64, Aabb)> {
    let mut bounds = bounds_of([source])?;
    let reach = distance.abs();
    if !reach.is_finite() {
        return None;
    }
    let pad = crate::Vec3::new(reach, reach, reach);
    bounds = Aabb { minimum: bounds.minimum.sub(pad), maximum: bounds.maximum.add(pad) };
    guard_factor(bounds).map(|factor| (factor, bounds))
}

/// An offset's source and distance at the working scale.
pub(crate) struct ScaledOffset {
    pub(crate) factor: f64,
    pub(crate) source: BrepSolid,
    pub(crate) distance: f64,
}

impl ScaledOffset {
    /// `None` when the guard does not fire for `source` moved by `distance`.
    pub(crate) fn of(source: &BrepSolid, distance: f64, entry: &str) -> Result<Option<Self>, KernelRefusal> {
        let Some((factor, bounds)) = offset_factor(source, distance) else {
            return Ok(None);
        };
        profile_fired(entry, factor, bounds);
        let source = scale_operand(source, factor, entry)?;
        Ok(Some(Self { factor, source, distance: distance * factor }))
    }

    /// A result of the scaled run, back at the model's scale.
    pub(crate) fn back(&self, solid: &BrepSolid, entry: &str) -> Result<BrepSolid, KernelRefusal> {
        scale_operand(solid, 1.0 / self.factor, entry)
    }

    pub(crate) fn refusal(&self, error: KernelRefusal) -> KernelRefusal {
        name_scale(error, self.factor)
    }
}

/// One `working_scale.fired` line on stderr under `BREP_PROFILE`.
pub(crate) fn profile_fired(entry: &str, factor: f64, bounds: Aabb) {
    if crate::profile::profile_enabled() {
        profile_line!("working_scale.fired entry={entry} factor={factor} diagonal={:.4e}", bounds.diagonal());
    }
}

