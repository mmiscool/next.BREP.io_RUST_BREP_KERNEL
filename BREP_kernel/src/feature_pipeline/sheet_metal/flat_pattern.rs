//! Flat-pattern (unfold) → 2D vector export (DXF R12 + SVG).
//!
//! A [`SheetTree`] laid out FLAT (every bend opened) is a set of coplanar 2D
//! regions: each flat's outline, the bend-allowance strip filling the gap between
//! a parent flat and its folded child, and every flat's hole loops. This module
//! walks the tree at `fold = 0` — the SAME geometry [`super::evaluate`] would
//! build at `fold = 0.0`, but as pure 2D polylines instead of an exact-BREP solid
//! — and serializes the result to DXF (R12 ASCII) and SVG.
//!
//! ## Why a bespoke 2D walk (not `folded_flat_placements` / `evaluate`)
//!
//! [`super::evaluate::folded_flat_placements`] is hardcoded to `fold = 1.0` (the
//! FOLDED layout — wrong for a flat pattern), and the fold-parametric frame math
//! (`build_bend` / `Frame`) is private to the `evaluate` module. Extracting the
//! outline from the `evaluate(tree, 0)` BREP solid is possible but heavy: that
//! solid keeps coplanar plate/band faces UN-merged, so its "top" is many faces
//! whose union boundary would have to be reconstructed. The 2D layout is far
//! simpler and is derived DIRECTLY from the tree, matching `build_bend`'s
//! `fold = 0` branch exactly:
//!   * outward in-plane direction of a CCW fold edge `a -> b` is `out = (eʸ, -eˣ)`,
//!   * the child flat's frame is `origin = a + out·allowance`, `u = out`, `v = ê`,
//!   * the allowance gap is `allowance` wide; the fold centerline runs down its
//!     middle, at the edge offset outward by `allowance / 2`.
//!
//! ## Clipping to the material region
//!
//! A [`Hole`](super::tree::Hole) baked by SM.CUTOUT may reach OUTSIDE its flat's
//! outline — the feature calls that an edge notch, and it is how a trim takes a
//! bite out of a blank rather than punching a window in it. Neither the outline
//! nor the hole loop is then a cut boundary on its own: the eaten stretch of the
//! outline is air, and the stretch of the hole loop beyond the outline bounds
//! nothing at all. The folded solid resolves this with a boolean; the 2D walk
//! resolves it in [`Region`], which samples the flat's outline and every hole
//! loop once, splits each drawn line at every crossing, and keeps only the pieces
//! with metal on exactly one side. A blank nothing overhangs, and a hole that
//! sits inside one, come through the clip verbatim.
//!
//! The clip refuses exactly one thing: a cut that eats into a FOLD edge, where
//! the bend's own lines would have to be split with it. See
//! [`Region::check_fold_edge_uncut`].
//!
//! ## Output line categories / roles
//!
//! Every emitted line falls into one of three [`LoopRole`]s (DXF layer + colour +
//! linetype and SVG stroke follow from it):
//!   * [`LoopRole::Cut`] — the continuous OUTER cut profile of the flat blank:
//!     every outline edge that is not a fold PLUS the two SIDE edges of each bend
//!     region (`a -> a+out·A`, `b -> b+out·A`). Solid blue-green (layer `CUT`).
//!   * [`LoopRole::Fold`] — one bend CENTERLINE per fold, down the middle of the
//!     allowance gap. Red PHANTOM (layer `FOLD`).
//!   * [`LoopRole::BendInternal`] — the two TANGENT lines per bend (parent-side
//!     `a -> b`, child-side `a+out·A -> b+out·A`): the fold-zone boundaries where
//!     each flat meets the bend. Internal, NOT cut. Cyan DASHED (layer `TANGENT`).
//!
//! A flat's fold-carrying outline edge is therefore emitted as the dashed-cyan
//! parent tangent, never as a solid cut, and a child's seam edge (its attachment
//! to the parent bend) is suppressed from the cut profile too — so a 0° bend
//! (no gap) reads as one continuous blank with no line at the seam.
//!
//! Each non-degenerate bend also gets a short TEXT [`Annotation`] on layer `BEND`
//! stating the fold direction and angle (`"UP 90° R1"` / `"DOWN 45° R2"`), drawn
//! yellow at a span-scaled height. A drawn collar is not developable, so — exactly
//! as the `fold = 0` plate does — only its enlarged hole opening shows (no collar
//! wall region).

use super::tree::{Flat, HoleBendKind, SheetTree};
use crate::{point_in_polygon, segment_intersection, NurbsCurve, Vec2};

/// A 2D segment (its two endpoints) in the common root-plane frame.
type Seg = ([f64; 2], [f64; 2]);

/// What KIND of line a polyline is. Drives the DXF layer / colour / linetype and
/// the SVG stroke style.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoopRole {
    /// A material boundary that is physically cut: the outer outline, a hole, or
    /// the side edge of a bend region. Solid blue-green.
    Cut,
    /// A bend fold line — the centerline down the allowance gap between two flats.
    /// Red PHANTOM.
    Fold,
    /// An INTERNAL fold-zone boundary: the tangent line where a flat meets a bend
    /// (parent-side or child-side). Inside the part, not cut. Cyan DASHED.
    BendInternal,
}

/// One 2D loop (or open line) of the flat pattern, in the flat plane's own
/// coordinates (the root flat's local frame; +Y up). Closed loops are cut
/// outlines / holes; the open cut runs, tangent lines and fold centerlines reuse
/// this same type.
#[derive(Clone, Debug)]
pub struct Polyline {
    /// Vertices, head-to-tail (a closed loop's closing edge is implicit).
    pub points: Vec<[f64; 2]>,
    /// Closed cut loop vs open line (cut run / tangent / fold centerline).
    pub closed: bool,
    /// Which line category this is.
    pub role: LoopRole,
}

/// A short shop-floor bend label placed at a fold line's midpoint (fold
/// direction + angle, e.g. `"UP 90° R1"`).
#[derive(Clone, Debug)]
pub struct Annotation {
    /// Label anchor (the fold-line midpoint) in the flat plane's own
    /// coordinates (root frame, +Y up) — the SAME frame the [`Polyline`]s use.
    pub pos: [f64; 2],
    /// Text baseline orientation, degrees CCW from +X in the flat-plane frame,
    /// normalized to `(-90, 90]` so the label always reads upright. DXF (Y-up)
    /// consumes this directly as its `50` rotation; SVG (Y-down) negates it.
    pub rotation_deg: f64,
    /// The rendered label (uses a real `°` glyph; the DXF writer rewrites it to
    /// the AutoCAD `%%d` degree code for ANSI-clean AC1009 output).
    pub text: String,
}

/// The full flat-pattern export: cut loops / runs, tangent lines and fold
/// centerlines (all [`Polyline`]s) plus the per-bend [`Annotation`]s, in the one
/// common root-plane frame. Produced by [`build`]; consumed by [`to_dxf`] /
/// [`to_svg`].
#[derive(Clone, Debug)]
pub struct FlatPattern {
    /// Cut outlines / holes (closed), cut runs / tangent lines / fold centerlines
    /// (open), all coplanar.
    pub polylines: Vec<Polyline>,
    /// One bend label per non-degenerate fold.
    pub annotations: Vec<Annotation>,
}

/// A 2D rigid placement: local `(x, y)` maps to `o + x·u + y·v` (all root-plane
/// coordinates). Composes down the flat tree so every flat's outline lands in one
/// common plane.
#[derive(Clone, Copy)]
struct Placement {
    o: [f64; 2],
    u: [f64; 2],
    v: [f64; 2],
}

impl Placement {
    /// The root flat sits in its own local frame unchanged.
    fn identity() -> Self {
        Placement { o: [0.0, 0.0], u: [1.0, 0.0], v: [0.0, 1.0] }
    }
    /// Map a local point into the root plane.
    fn map(&self, p: [f64; 2]) -> [f64; 2] {
        [
            self.o[0] + p[0] * self.u[0] + p[1] * self.v[0],
            self.o[1] + p[0] * self.u[1] + p[1] * self.v[1],
        ]
    }
    /// Map a local DIRECTION (no translation) into the root plane.
    fn map_dir(&self, d: [f64; 2]) -> [f64; 2] {
        [
            d[0] * self.u[0] + d[1] * self.v[0],
            d[0] * self.u[1] + d[1] * self.v[1],
        ]
    }
}

/// The flat pattern of `tree`: the outer cut profile (outline runs + bend side
/// edges) on layer `CUT`, one fold CENTERLINE per bend on layer `FOLD`, the
/// fold-zone TANGENT lines on layer `TANGENT`, and one bend `Annotation` per
/// fold, all laid out coplanar (the `fold = 0` layout).
pub fn build(tree: &SheetTree) -> Result<FlatPattern, String> {
    if tree.thickness <= 0.0 {
        return Err(format!(
            "sheet-metal flat pattern: thickness must be positive, got {}",
            tree.thickness
        ));
    }
    let mut out = FlatPattern { polylines: Vec::new(), annotations: Vec::new() };
    walk(&tree.root, Placement::identity(), tree.thickness, None, &mut out)?;
    if out.polylines.is_empty() {
        return Err("sheet-metal flat pattern: tree produced no geometry".into());
    }
    Ok(out)
}

/// Emit `flat`'s cut profile + holes, then recurse across every bend (outline
/// flanges and straight hole-rim flaps), unfolding each child into the same
/// plane. `seam` — when `flat` is a child — is the root-coord segment along which
/// it attaches to its parent's bend; that edge is a fold-zone boundary (drawn by
/// the parent's `emit_bend` as the child-side tangent), so it is suppressed from
/// this flat's cut profile.
fn walk(
    flat: &Flat,
    place: Placement,
    thickness: f64,
    seam: Option<Seg>,
    out: &mut FlatPattern,
) -> Result<(), String> {
    let n = flat.outline.len();

    // 0. The flat's MATERIAL region: its outline and every hole loop, sampled once
    //    in flat-local coordinates. A cut may overhang the outline (SM.CUTOUT calls
    //    that an edge notch), so neither the outline nor a hole loop is a cut
    //    boundary on its own — every emitted line below is clipped to the part of
    //    it that actually separates material from air.
    let region = Region::sample(flat)?;

    // 1. The outer cut profile. Every outline edge is a solid cut EXCEPT one that
    //    carries a bend (its tangent is drawn by `emit_bend`) or one that is this
    //    flat's seam to its parent (the parent's `emit_bend` draws it as the
    //    child-side tangent). Those are dropped here so they never render as a cut.
    emit_outline(flat, &region, place, seam, out)?;

    // 2. Hole loops (outer + islands), clipped to the blank. A blind pocket's rim
    //    is emitted as a cut loop too — consistent with the `fold = 0` plate, which
    //    carries the same hole; a through/blind distinction is not drawn in v1.
    for hole in &flat.holes {
        emit_loop(&hole.outer, &region, place, out)?;
        for island in &hole.islands {
            emit_loop(island, &region, place, out)?;
        }
    }

    // 3 + 4. Bends: outline flanges (on `edges`) and straight hole-rim flaps.
    //        Each emits the two bend SIDE edges (cut), the two TANGENT lines and a
    //        fold CENTERLINE + annotation (unless a 0° bend, which has no
    //        allowance gap) and an unfolded child placement to recurse into. A
    //        drawn collar is not developable — its wall is skipped (its enlarged
    //        opening already shows as a hole loop above), matching the `fold = 0`
    //        plate.
    let mut children: Vec<(&Flat, Placement, Seg)> = Vec::new();
    for index in 0..n {
        let Some(bend) = flat.edges.get(index).and_then(|edge| edge.bend.as_ref()) else {
            continue;
        };
        let a = flat.outline[index];
        let b = flat.outline[(index + 1) % n];
        let allowance = bend.allowance(thickness);
        emit_bend(
            a,
            b,
            allowance,
            bend.angle_deg,
            bend.inside_radius,
            &bend.child,
            place,
            &mut children,
            out,
        );
    }
    for hole_bend in &flat.hole_bends {
        let HoleBendKind::Straight { child } = &hole_bend.kind else {
            continue; // a collar has no developable wall (see the note above)
        };
        let hole = flat.holes.get(hole_bend.hole).ok_or_else(|| {
            format!(
                "sheet-metal flat pattern: hole bend `{}` references missing hole {} on flat `{}`",
                hole_bend.id, hole_bend.hole, flat.id
            )
        })?;
        let curve = hole.outer.get(hole_bend.segment).ok_or_else(|| {
            format!(
                "sheet-metal flat pattern: hole bend `{}` references missing segment {} of hole {}",
                hole_bend.id, hole_bend.segment, hole_bend.hole
            )
        })?;
        let (mut a, mut b) = curve_endpoints_2d(curve)?;
        if hole_bend.reversed {
            std::mem::swap(&mut a, &mut b);
        }
        // Allowance for the transient hole-rim bend — same neutral-fiber formula
        // as `Bend::allowance` (tree.rs), without cloning the child subtree.
        let allowance = bend_allowance(
            hole_bend.angle_deg,
            hole_bend.inside_radius,
            hole_bend.k_factor,
            thickness,
        );
        emit_bend(
            a,
            b,
            allowance,
            hole_bend.angle_deg,
            hole_bend.inside_radius,
            child,
            place,
            &mut children,
            out,
        );
    }

    for (child, child_place, child_seam) in children {
        walk(child, child_place, thickness, Some(child_seam), out)?;
    }
    Ok(())
}

/// Emit `flat`'s outer cut profile. An outline edge that carries a bend or is the
/// flat's seam to its parent is a fold-zone boundary (drawn elsewhere as a
/// tangent), so it is NOT a cut. With no such edge the whole outline is one closed
/// cut loop; otherwise each maximal run of consecutive cut edges is emitted as an
/// open cut chain (the side edges from `emit_bend` bridge the runs into the full
/// closed profile). Every run is CLIPPED to `region`: the stretch of an outline
/// edge that a cut has eaten is air, not a cut line (the hole loop's own arc
/// across the opening is the cut there, emitted by [`emit_loop`]).
fn emit_outline(
    flat: &Flat,
    region: &Region,
    place: Placement,
    seam: Option<Seg>,
    out: &mut FlatPattern,
) -> Result<(), String> {
    let n = flat.outline.len();
    if n == 0 {
        return Ok(());
    }

    // Which outline edges are fold-zone boundaries (dropped from the cut profile)?
    let suppressed: Vec<bool> = (0..n)
        .map(|i| {
            let is_bend = flat
                .edges
                .get(i)
                .and_then(|edge| edge.bend.as_ref())
                .is_some();
            let is_seam = seam.is_some_and(|s| {
                let pa = place.map(flat.outline[i]);
                let pb = place.map(flat.outline[(i + 1) % n]);
                seg_matches(pa, pb, s)
            });
            is_bend || is_seam
        })
        .collect();

    // A fold line the cut has eaten into would leave `emit_bend` drawing a tangent
    // and two side edges across air — the bend region is not developable there at
    // all. SM.CUTOUT refuses a region that enters a bend-wedge footprint, so this
    // is unreachable through the feature; say so loudly rather than draw it.
    for (i, &is_fold) in suppressed.iter().enumerate() {
        if is_fold {
            region.check_fold_edge_uncut(
                flat.outline[i],
                flat.outline[(i + 1) % n],
                &flat.id,
                flat.edges.get(i).map_or("", |edge| edge.id.as_str()),
            )?;
        }
    }

    if suppressed.iter().all(|&drop| !drop) {
        // No fold-zone boundary — the whole outline is one closed cut loop.
        let outline = flat.densified_outline()?;
        push_clipped(&outline, true, region, place, LoopRole::Cut, out);
        return Ok(());
    }

    // Emit each maximal run of consecutive cut edges as an open chain. Start just
    // past the first suppressed edge so a run never wraps across a boundary.
    let start = suppressed.iter().position(|&drop| drop).unwrap();
    let mut run: Vec<[f64; 2]> = Vec::new();
    for step in 0..n {
        let i = (start + 1 + step) % n;
        if suppressed[i] {
            flush_run(&mut run, region, place, out);
        } else {
            let pts = edge_points(flat, i)?;
            if run.is_empty() {
                run.extend(pts);
            } else {
                // Drop the shared vertex where this edge meets the previous one.
                run.extend(pts.into_iter().skip(1));
            }
        }
    }
    flush_run(&mut run, region, place, out);
    Ok(())
}

/// Push the accumulated cut run (≥ 2 points), clipped to the material region, and
/// reset it.
fn flush_run(run: &mut Vec<[f64; 2]>, region: &Region, place: Placement, out: &mut FlatPattern) {
    if run.len() >= 2 {
        push_clipped(run, false, region, place, LoopRole::Cut, out);
    }
    run.clear();
}

/// Sample a closed exact-curve loop (a hole's outer or island loop) and push the
/// stretches of it that really bound material. A loop wholly inside the blank is
/// emitted unchanged; one that overhangs the outline (an edge notch) contributes
/// only its inboard arc, and one wholly outside contributes nothing.
fn emit_loop(
    curves: &[NurbsCurve],
    region: &Region,
    place: Placement,
    out: &mut FlatPattern,
) -> Result<(), String> {
    let points = sample_loop(curves)?;
    push_clipped(&points, true, region, place, LoopRole::Cut, out);
    Ok(())
}

/// Clip one flat-local chain to the material region and push what survives,
/// mapped into the root plane. An untouched chain is pushed verbatim.
fn push_clipped(
    points: &[[f64; 2]],
    closed: bool,
    region: &Region,
    place: Placement,
    role: LoopRole,
    out: &mut FlatPattern,
) {
    for (run, run_closed) in region.clip(points, closed) {
        if run.len() < 2 || (run_closed && run.len() < 3) {
            continue;
        }
        out.polylines.push(Polyline {
            points: run.iter().map(|p| place.map(*p)).collect(),
            closed: run_closed,
            role,
        });
    }
}

/// The flat-local points of outline edge `i`, endpoints INCLUSIVE — a straight
/// edge is its two vertices; a curved edge (keyed in [`Flat::outline_curves`] by
/// its edge id) is 17 samples of its exact curve.
fn edge_points(flat: &Flat, i: usize) -> Result<Vec<[f64; 2]>, String> {
    let n = flat.outline.len();
    let start = flat.outline[i];
    let end = flat.outline[(i + 1) % n];
    match flat
        .edges
        .get(i)
        .and_then(|edge| flat.outline_curves.get(&edge.id))
    {
        Some(curve) => {
            let [t0, t1] = curve.domain()?;
            let mut pts = Vec::with_capacity(17);
            for step in 0..=16 {
                let t = t0 + (t1 - t0) * (step as f64) / 16.0;
                let p = curve.evaluate(t)?;
                pts.push([p.x, p.y]);
            }
            Ok(pts)
        }
        None => Ok(vec![start, end]),
    }
}

/// Whether the (root-coord) segment `pa -> pb` is the same segment as `seg`,
/// regardless of direction.
fn seg_matches(pa: [f64; 2], pb: [f64; 2], seg: Seg) -> bool {
    let (s0, s1) = seg;
    (near(pa, s0) && near(pb, s1)) || (near(pa, s1) && near(pb, s0))
}

/// Two 2D points within a sub-micron tolerance (sheet-metal features are ≥ mm).
fn near(p: [f64; 2], q: [f64; 2]) -> bool {
    (p[0] - q[0]).abs() < 1e-6 && (p[1] - q[1]).abs() < 1e-6
}

/// Emit a bend on the parent-local edge `a -> b`: the two SIDE edges (cut), the
/// two TANGENT lines (parent-side `a -> b` and child-side `a' -> b'`, internal),
/// the fold CENTERLINE + annotation, and push the child's unfolded placement.
/// Mirrors `build_bend`'s `fold = 0` branch: the child lies just past the
/// allowance gap (spacing = the neutral-fiber allowance `A`), so `a' = a + out·A`,
/// `b' = b + out·A`, and the centerline runs down the middle of the gap
/// (`out·A/2`). A 0° bend (no gap) emits none of those lines — the child seam is
/// suppressed on both flats, so the two read as one continuous blank.
fn emit_bend<'a>(
    a: [f64; 2],
    b: [f64; 2],
    allowance: f64,
    angle_deg: f64,
    inside_radius: f64,
    child: &'a Flat,
    place: Placement,
    children: &mut Vec<(&'a Flat, Placement, Seg)>,
    out: &mut FlatPattern,
) {
    let e = normalized([b[0] - a[0], b[1] - a[1]]);
    if e == [0.0, 0.0] {
        return; // degenerate fold edge — nothing to unfold
    }
    let out_dir = [e[1], -e[0]]; // in-plane outward = right of a -> b (CCW)
    let a2 = [a[0] + out_dir[0] * allowance, a[1] + out_dir[1] * allowance];
    let b2 = [b[0] + out_dir[0] * allowance, b[1] + out_dir[1] * allowance];

    if allowance > 1e-9 {
        // The two SIDE edges — the outer cut profile continuing through the bend.
        out.polylines.push(open_line(place.map(a), place.map(a2), LoopRole::Cut));
        out.polylines.push(open_line(place.map(b), place.map(b2), LoopRole::Cut));
        // The two TANGENT lines — the fold-zone boundaries where each flat meets
        // the bend (parent-side `a -> b`, child-side `a' -> b'`). Internal.
        out.polylines
            .push(open_line(place.map(a), place.map(b), LoopRole::BendInternal));
        out.polylines
            .push(open_line(place.map(a2), place.map(b2), LoopRole::BendInternal));
        // The fold CENTERLINE: the edge a -> b shifted outward by half the
        // allowance, i.e. down the middle of the parent/child gap.
        let half = allowance * 0.5;
        let c_a = [a[0] + out_dir[0] * half, a[1] + out_dir[1] * half];
        let c_b = [b[0] + out_dir[0] * half, b[1] + out_dir[1] * half];
        out.polylines
            .push(open_line(place.map(c_a), place.map(c_b), LoopRole::Fold));

        // The bend label at the centerline midpoint, oriented to read along the
        // fold. Direction (root frame) fixes the rotation; the sign of the bend
        // angle fixes UP vs DOWN.
        let mid = place.map([(c_a[0] + c_b[0]) * 0.5, (c_a[1] + c_b[1]) * 0.5]);
        let dir = place.map_dir(e);
        let rotation_deg = readable_angle(dir[1].atan2(dir[0]).to_degrees());
        out.annotations.push(Annotation {
            pos: mid,
            rotation_deg,
            text: bend_label(angle_deg, inside_radius),
        });
    }

    // The child attaches along its near edge `a' -> b'` (root coords). Suppress it
    // from the child's cut profile even for a 0° bend (a' == a, b' == b), so no
    // line is drawn at a flush seam.
    let seam = (place.map(a2), place.map(b2));
    children.push((
        child,
        Placement {
            o: place.map(a2),
            u: place.map_dir(out_dir),
            v: place.map_dir(e),
        },
        seam,
    ));
}

/// A two-point open polyline of the given role.
fn open_line(p0: [f64; 2], p1: [f64; 2], role: LoopRole) -> Polyline {
    Polyline { points: vec![p0, p1], closed: false, role }
}

/// A shop-floor bend label: fold DIRECTION + magnitude (+ inside radius).
/// Direction convention matches [`super::tree::Bend::angle_deg`]: `+` folds
/// toward the flat's −normal side = `"DOWN"`, `−` toward +normal = `"UP"`. Uses a
/// real `°` glyph (the DXF writer rewrites it to `%%d`).
fn bend_label(angle_deg: f64, inside_radius: f64) -> String {
    let dir = if angle_deg < 0.0 { "UP" } else { "DOWN" };
    format!(
        "{dir} {}° R{}",
        trim_number(angle_deg.abs()),
        trim_number(inside_radius)
    )
}

/// Format a length/angle for a label: two decimals, trailing zeros (and a bare
/// dot) stripped — `90.0 -> "90"`, `45.5 -> "45.5"`, `1.25 -> "1.25"`.
fn trim_number(v: f64) -> String {
    let s = format!("{v:.2}");
    let trimmed = s.trim_end_matches('0').trim_end_matches('.');
    trimmed.to_string()
}

/// Fold the text baseline angle (degrees) into `(-90, 90]` — a line has no
/// direction, so this keeps the label upright whichever way the fold edge runs.
fn readable_angle(mut deg: f64) -> f64 {
    deg %= 360.0;
    if deg > 180.0 {
        deg -= 360.0;
    } else if deg <= -180.0 {
        deg += 360.0;
    }
    if deg > 90.0 {
        deg -= 180.0;
    } else if deg <= -90.0 {
        deg += 180.0;
    }
    deg
}


/// Sample a closed exact-curve loop (a hole's outer or island loop, control points
/// at local `z = 0`) into a FLAT-LOCAL 2D polygon — 16 points per curve,
/// head-to-tail, the closing edge implicit.
fn sample_loop(curves: &[NurbsCurve]) -> Result<Vec<[f64; 2]>, String> {
    let mut points = Vec::new();
    for curve in curves {
        let [t0, t1] = curve.domain()?;
        for step in 0..16 {
            let t = t0 + (t1 - t0) * (step as f64) / 16.0;
            let p = curve.evaluate(t)?;
            points.push([p.x, p.y]);
        }
    }
    if points.len() < 3 {
        return Err("sheet-metal flat pattern: degenerate hole loop".into());
    }
    Ok(points)
}

/// The two flat-local endpoints of a hole-rim segment (its curve's domain ends).
fn curve_endpoints_2d(curve: &NurbsCurve) -> Result<([f64; 2], [f64; 2]), String> {
    let [t0, t1] = curve.domain()?;
    let p0 = curve.evaluate(t0)?;
    let p1 = curve.evaluate(t1)?;
    Ok(([p0.x, p0.y], [p1.x, p1.y]))
}

/// Neutral-fiber bend allowance `|angle| · (midRadius + (k − 0.5)·t)` — the same
/// formula as `Bend::allowance` (tree.rs), for the transient hole-rim bend.
fn bend_allowance(angle_deg: f64, inside_radius: f64, k_factor: f64, thickness: f64) -> f64 {
    let mid = (inside_radius + thickness * 0.5).max(1e-6);
    let neutral = mid + (k_factor - 0.5) * thickness;
    angle_deg.to_radians().abs() * neutral
}

// ===========================================================================
// The material region — clipping the drawn lines to what actually bounds metal
// ===========================================================================

/// Node-merge / crossing tolerance for the 2D clip. Sheet-metal features are
/// millimetres and the loops are exact curves sampled at f64, so a crossing
/// nearer than this to a shared vertex IS that vertex.
const CLIP_TOL: f64 = 1e-9;

/// How far off a candidate line the material test is taken. Large enough that
/// [`crate::point_in_polygon`] never reports `boundary` against the very loop the
/// line came from (which sits exactly `CLIP_TOL` ≪ this away), small enough that
/// it cannot step across a different loop: every crossing has already split the
/// line, so the nearest other boundary is at least half a piece away.
const PROBE_EPS: f64 = 1e-6;

/// One flat's MATERIAL region in flat-local coordinates: the outline polygon and
/// every hole loop, sampled once.
///
/// The tree stores a cut as a [`Hole`](super::tree::Hole) whose loop may reach
/// OUTSIDE the outline — SM.CUTOUT's edge notch. The folded solid resolves that
/// with a boolean; the 2D walk has to resolve it here, or the flat pattern draws
/// a blank that was never notched plus a stray closed loop hanging off its edge.
struct Region {
    /// The blank's boundary, densified (16 samples per curved outline segment).
    outline: Ring,
    /// Per hole: its outer loop and the island loops that keep material inside it.
    holes: Vec<(Ring, Vec<Ring>)>,
}

/// One sampled closed loop, kept in both the forms the clip needs — the `Vec2`
/// ring [`crate::point_in_polygon`] takes and the raw vertices the splitter walks
/// — plus its bounding box. A perforated panel puts hundreds of loops in a
/// [`Region`] and every drawn line is tested against all of them, so the box is
/// what keeps that from being quadratic in the loop COUNT times their size.
struct Ring {
    points: Vec<[f64; 2]>,
    ring: Vec<Vec2>,
    min: [f64; 2],
    max: [f64; 2],
}

impl Ring {
    fn new(points: Vec<[f64; 2]>) -> Self {
        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        for p in &points {
            for axis in 0..2 {
                min[axis] = min[axis].min(p[axis]);
                max[axis] = max[axis].max(p[axis]);
            }
        }
        let ring = points.iter().map(|p| Vec2 { x: p[0], y: p[1] }).collect();
        Ring { points, ring, min, max }
    }

    /// Strictly inside this loop (a point on its boundary is neither in nor out —
    /// the probe points [`Region::bounds_material`] tests never sit there).
    fn contains(&self, p: [f64; 2]) -> bool {
        if p[0] < self.min[0] - CLIP_TOL
            || p[0] > self.max[0] + CLIP_TOL
            || p[1] < self.min[1] - CLIP_TOL
            || p[1] > self.max[1] + CLIP_TOL
        {
            return false;
        }
        point_in_polygon(Vec2 { x: p[0], y: p[1] }, &self.ring, CLIP_TOL) == "in"
    }

    /// Whether the box `min..max` can reach this loop at all.
    fn box_overlaps(&self, min: [f64; 2], max: [f64; 2]) -> bool {
        min[0] <= self.max[0] + CLIP_TOL
            && max[0] >= self.min[0] - CLIP_TOL
            && min[1] <= self.max[1] + CLIP_TOL
            && max[1] >= self.min[1] - CLIP_TOL
    }
}

impl Region {
    /// Sample `flat`'s outline and hole loops into one clippable region.
    fn sample(flat: &Flat) -> Result<Self, String> {
        let outline = Ring::new(flat.densified_outline()?);
        let mut holes = Vec::with_capacity(flat.holes.len());
        for hole in &flat.holes {
            let outer = Ring::new(sample_loop(&hole.outer)?);
            let islands = hole
                .islands
                .iter()
                .map(|island| sample_loop(island).map(Ring::new))
                .collect::<Result<Vec<_>, _>>()?;
            holes.push((outer, islands));
        }
        Ok(Region { outline, holes })
    }

    /// Every loop of this region: the outline, then each hole with its islands.
    fn rings(&self) -> impl Iterator<Item = &Ring> {
        std::iter::once(&self.outline).chain(
            self.holes
                .iter()
                .flat_map(|(outer, islands)| std::iter::once(outer).chain(islands)),
        )
    }

    /// Whether `p` — which must not lie ON any loop — is metal: inside the blank,
    /// and not inside a hole except where one of that hole's islands restores it
    /// (the even-odd sketch-region semantics [`Hole::islands`] records).
    fn material(&self, p: [f64; 2]) -> bool {
        if !self.outline.contains(p) {
            return false;
        }
        !self.holes.iter().any(|(outer, islands)| {
            outer.contains(p) && !islands.iter().any(|island| island.contains(p))
        })
    }

    /// Whether the straight piece `a -> b` — split at every crossing already, so
    /// no other loop passes through its interior — separates metal from air.
    /// Sampling both sides also drops a line that bounds nothing: a hole edge
    /// outside the blank, or an outline edge a cut has eaten through.
    fn bounds_material(&self, a: [f64; 2], b: [f64; 2]) -> bool {
        let d = [b[0] - a[0], b[1] - a[1]];
        let length = (d[0] * d[0] + d[1] * d[1]).sqrt();
        if length <= CLIP_TOL {
            return false;
        }
        let eps = PROBE_EPS.min(length * 0.25);
        let n = [-d[1] / length * eps, d[0] / length * eps];
        let mid = [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        self.material([mid[0] + n[0], mid[1] + n[1]])
            != self.material([mid[0] - n[0], mid[1] - n[1]])
    }

    /// Refuse when a cut has eaten into a fold edge. The bend hanging off that
    /// edge is drawn from the edge's two raw endpoints — tangent lines, side edges
    /// and centerline all span the FULL edge — so a partly eaten fold line would
    /// silently draw a bend across air.
    ///
    /// This is REACHABLE, on a knife edge: SM.CUTOUT's guard rejects a region that
    /// ENTERS a bend-wedge footprint, so a cut is refused as soon as it passes the
    /// fold tangent, but a cut that stops EXACTLY on it is admitted (measured on
    /// the reporter's fork: a cut to y = 106.999 cuts, y = 107.001 is refused by
    /// SM.CUTOUT, y = 107.000 lands here). Drawing that case right means splitting
    /// the fold line — the eaten stretch bounds the notch against an uncut bend
    /// strip and is a CUT, the rest stays the tangent — and splitting the bend
    /// annotation with it. Until that exists, refuse: the clip alone would drop
    /// the coincident stretch from both the outline and the hole loop and leave a
    /// gap in the profile.
    fn check_fold_edge_uncut(
        &self,
        a: [f64; 2],
        b: [f64; 2],
        flat_id: &str,
        edge_id: &str,
    ) -> Result<(), String> {
        let cut = self
            .split(a, b)
            .into_iter()
            .any(|(pa, pb)| !self.bounds_material(pa, pb));
        if cut {
            return Err(format!(
                "sheet-metal flat pattern: a cut on flat `{flat_id}` eats into fold edge \
                 `{edge_id}` — a fold line cannot be cut (the bend's tangent lines, side \
                 edges and centerline all span the whole edge)"
            ));
        }
        Ok(())
    }

    /// Split `a -> b` at every point where another loop of this region crosses it,
    /// returning the pieces head-to-tail (endpoints preserved exactly).
    fn split(&self, a: [f64; 2], b: [f64; 2]) -> Vec<([f64; 2], [f64; 2])> {
        let av = Vec2 { x: a[0], y: a[1] };
        let bv = Vec2 { x: b[0], y: b[1] };
        let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        if length <= CLIP_TOL {
            return Vec::new();
        }
        let parameter_tol = (CLIP_TOL / length).min(0.25);
        let min = [a[0].min(b[0]), a[1].min(b[1])];
        let max = [a[0].max(b[0]), a[1].max(b[1])];
        let mut cuts: Vec<f64> = Vec::new();
        for loop_ring in self.rings() {
            // Cheap rejects: a whole loop, then a segment, whose box misses this
            // line's box cannot cross it.
            if !loop_ring.box_overlaps(min, max) {
                continue;
            }
            let poly = &loop_ring.points;
            for i in 0..poly.len() {
                let (c, d) = (poly[i], poly[(i + 1) % poly.len()]);
                if c[0].min(d[0]) > max[0] + CLIP_TOL
                    || c[0].max(d[0]) < min[0] - CLIP_TOL
                    || c[1].min(d[1]) > max[1] + CLIP_TOL
                    || c[1].max(d[1]) < min[1] - CLIP_TOL
                {
                    continue;
                }
                let hit = segment_intersection(
                    av,
                    bv,
                    Vec2 { x: c[0], y: c[1] },
                    Vec2 { x: d[0], y: d[1] },
                    CLIP_TOL,
                );
                // A shared vertex lands on 0 or 1 and is not an interior split; a
                // collinear overlap returns nothing here and is split instead by
                // the transverse segments at its two ends.
                if let Some([t, _]) = hit {
                    if t > parameter_tol && t < 1.0 - parameter_tol {
                        cuts.push(t);
                    }
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        let mut parameters = vec![0.0];
        for t in cuts {
            if t - parameters[parameters.len() - 1] > parameter_tol {
                parameters.push(t);
            }
        }
        if 1.0 - parameters[parameters.len() - 1] <= parameter_tol {
            parameters.pop();
        }
        parameters.push(1.0);
        let at = |t: f64| -> [f64; 2] {
            match t {
                t if t == 0.0 => a,
                t if t == 1.0 => b,
                t => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t],
            }
        };
        parameters
            .windows(2)
            .map(|pair| (at(pair[0]), at(pair[1])))
            .collect()
    }

    /// Clip one flat-local chain to the material boundary. Returns the surviving
    /// runs as `(points, closed)`. A chain no cut touches comes back verbatim —
    /// same vertices, same order, same closed flag — so a hole that sits inside
    /// its blank, and a blank nothing overhangs, are drawn exactly as before.
    fn clip(&self, points: &[[f64; 2]], closed: bool) -> Vec<(Vec<[f64; 2]>, bool)> {
        let n = points.len();
        if n < 2 || (closed && n < 3) {
            return Vec::new();
        }
        let mut pieces: Vec<([f64; 2], [f64; 2], bool)> = Vec::new();
        let mut split_any = false;
        let last = if closed { n } else { n - 1 };
        for i in 0..last {
            let (a, b) = (points[i], points[(i + 1) % n]);
            let parts = self.split(a, b);
            split_any |= parts.len() != 1;
            for (pa, pb) in parts {
                let keep = self.bounds_material(pa, pb);
                pieces.push((pa, pb, keep));
            }
        }
        if pieces.is_empty() {
            return Vec::new();
        }
        if pieces.iter().all(|piece| piece.2) {
            if !split_any {
                return vec![(points.to_vec(), closed)];
            }
            let mut chain: Vec<[f64; 2]> = pieces.iter().map(|piece| piece.0).collect();
            if !closed {
                chain.push(pieces[pieces.len() - 1].1);
            }
            return vec![(chain, closed)];
        }
        // Something was dropped. Start just past a dropped piece so an open run
        // never wraps across the gap (an open chain already starts at its head).
        let start = if closed {
            pieces.iter().position(|piece| !piece.2).unwrap() + 1
        } else {
            0
        };
        let mut runs = Vec::new();
        let mut run: Vec<[f64; 2]> = Vec::new();
        for step in 0..pieces.len() {
            let (a, b, keep) = pieces[(start + step) % pieces.len()];
            if keep {
                if run.is_empty() {
                    run.push(a);
                }
                run.push(b);
            } else if run.len() >= 2 {
                runs.push((std::mem::take(&mut run), false));
            } else {
                run.clear();
            }
        }
        if run.len() >= 2 {
            runs.push((run, false));
        }
        runs
    }
}

/// Unit vector, or `[0, 0]` for a (near-)zero input.
fn normalized(d: [f64; 2]) -> [f64; 2] {
    let len = (d[0] * d[0] + d[1] * d[1]).sqrt();
    if len > 1e-12 {
        [d[0] / len, d[1] / len]
    } else {
        [0.0, 0.0]
    }
}

// ===========================================================================
// Writers
// ===========================================================================

/// DXF layer for a loop role.
fn layer(role: LoopRole) -> &'static str {
    match role {
        LoopRole::Cut => "CUT",
        LoopRole::Fold => "FOLD",
        LoopRole::BendInternal => "TANGENT",
    }
}

/// DXF layer for bend annotations (kept off the line layers so the labels can be
/// toggled independently).
const BEND_LAYER: &str = "BEND";

/// The exact blue-green (`#00B3A4`) of the cut profile, as a 24-bit RGB int for
/// the DXF `420` true-colour code (blue-green is not a standard ACI).
const CUT_RGB: u32 = 0x00B3A4;

/// DXF style for a role: `(ACI fallback colour, optional 24-bit true colour,
/// linetype name)`. The ACI keeps old readers coloured; the true colour (when
/// present) gives the exact blue-green to readers that understand `420`.
fn dxf_style(role: LoopRole) -> (i32, Option<u32>, &'static str) {
    match role {
        LoopRole::Cut => (3, Some(CUT_RGB), "CONTINUOUS"), // green fallback + blue-green
        LoopRole::Fold => (1, None, "PHANTOM"),            // red, phantom
        LoopRole::BendInternal => (4, None, "DASHED"),     // cyan, dashed
    }
}

/// The DXF entity colour + linetype group codes for a role (`62` before `420` so
/// strict R12 readers still get an ACI).
fn dxf_color_ltype(role: LoopRole) -> String {
    let (aci, tc, lt) = dxf_style(role);
    let mut s = format!("6\n{lt}\n62\n{aci}\n");
    if let Some(c) = tc {
        s.push_str(&format!("420\n{c}\n"));
    }
    s
}

/// Axis-aligned bounds `[min_x, min_y, max_x, max_y]` over every polyline point,
/// or `None` when there is no geometry. Drives the SVG viewport and the text
/// height (a fraction of the pattern span) for both writers.
fn bounds(polys: &[Polyline]) -> Option<[f64; 4]> {
    let mut min_x = f64::INFINITY;
    let mut min_y = f64::INFINITY;
    let mut max_x = f64::NEG_INFINITY;
    let mut max_y = f64::NEG_INFINITY;
    for poly in polys {
        for p in &poly.points {
            min_x = min_x.min(p[0]);
            min_y = min_y.min(p[1]);
            max_x = max_x.max(p[0]);
            max_y = max_y.max(p[1]);
        }
    }
    if min_x.is_finite() {
        Some([min_x, min_y, max_x, max_y])
    } else {
        None
    }
}

/// Annotation text height, ~5% of the pattern span (min 1 unit), from its bounds.
fn text_height(b: &[f64; 4]) -> f64 {
    let span = (b[2] - b[0]).max(b[3] - b[1]).max(1.0);
    (span * 0.05).max(1.0)
}

/// Serialize a flat pattern to a minimal DXF R12 (AC1009) ASCII document: a
/// HEADER (`$ACADVER` + a span-scaled `$LTSCALE`), a TABLES section defining the
/// `CONTINUOUS`/`DASHED`/`PHANTOM` linetypes, and an ENTITIES section of
/// heavyweight `POLYLINE`/`VERTEX`/`SEQEND` loops (LWPOLYLINE does not exist in
/// R12) plus a plain `TEXT` entity per bend (no MTEXT). Cut lines go on layer
/// `CUT` (blue-green), fold centerlines on `FOLD` (red phantom), tangent lines on
/// `TANGENT` (cyan dashed), bend labels on `BEND` (yellow); readers auto-create
/// referenced layers.
pub fn to_dxf(fp: &FlatPattern) -> String {
    let bbox = bounds(&fp.polylines);
    let span = bbox
        .map(|b| (b[2] - b[0]).max(b[3] - b[1]).max(1.0))
        .unwrap_or(1.0);
    let ltscale = (span * 0.01).max(0.1);
    let height = bbox.map(|b| text_height(&b)).unwrap_or(1.0);

    let mut s = String::new();
    // HEADER — version + a span-scaled global linetype scale so the dashed /
    // phantom patterns read at any part size.
    s.push_str("0\nSECTION\n2\nHEADER\n9\n$ACADVER\n1\nAC1009\n");
    s.push_str(&format!("9\n$LTSCALE\n40\n{ltscale:.3}\n"));
    s.push_str("0\nENDSEC\n");
    // TABLES — the linetypes the entities reference (49: + = dash, − = gap).
    s.push_str("0\nSECTION\n2\nTABLES\n0\nTABLE\n2\nLTYPE\n70\n3\n");
    s.push_str("0\nLTYPE\n2\nCONTINUOUS\n70\n64\n3\nSolid line\n72\n65\n73\n0\n40\n0.0\n");
    s.push_str(
        "0\nLTYPE\n2\nDASHED\n70\n64\n3\nDashed __ __ __\n72\n65\n73\n2\n40\n0.75\n49\n0.5\n49\n-0.25\n",
    );
    s.push_str(
        "0\nLTYPE\n2\nPHANTOM\n70\n64\n3\nPhantom ___ _ _ ___\n72\n65\n73\n6\n40\n2.5\n49\n1.25\n49\n-0.25\n49\n0.25\n49\n-0.25\n49\n0.25\n49\n-0.25\n",
    );
    s.push_str("0\nENDTAB\n0\nENDSEC\n");
    // ENTITIES.
    s.push_str("0\nSECTION\n2\nENTITIES\n");
    for poly in &fp.polylines {
        let layer = layer(poly.role);
        s.push_str("0\nPOLYLINE\n8\n");
        s.push_str(layer);
        s.push('\n');
        s.push_str(&dxf_color_ltype(poly.role));
        // 66/1 = vertices follow; 70 bit 1 = closed polyline.
        s.push_str("66\n1\n70\n");
        s.push_str(if poly.closed { "1" } else { "0" });
        s.push('\n');
        for p in &poly.points {
            s.push_str("0\nVERTEX\n8\n");
            s.push_str(layer);
            s.push_str(&format!("\n10\n{:.6}\n20\n{:.6}\n30\n0.0\n", p[0], p[1]));
        }
        s.push_str("0\nSEQEND\n");
    }
    // Bend labels as plain R12 TEXT (yellow, ACI 2), centered on the fold midpoint
    // (72/1 with the alignment point 11/21) and rotated (50) to read along the
    // fold. DXF is Y-up like the source data, so `rotation_deg` and the point map
    // through directly. The `°` glyph is rewritten to the AutoCAD `%%d` code.
    for ann in &fp.annotations {
        let text = ann.text.replace('°', "%%d");
        s.push_str("0\nTEXT\n8\n");
        s.push_str(BEND_LAYER);
        s.push_str("\n62\n2\n");
        s.push_str(&format!("10\n{:.6}\n20\n{:.6}\n30\n0.0\n", ann.pos[0], ann.pos[1]));
        s.push_str(&format!("40\n{height:.6}\n1\n{text}\n"));
        s.push_str(&format!("50\n{:.6}\n72\n1\n", ann.rotation_deg));
        s.push_str(&format!("11\n{:.6}\n21\n{:.6}\n31\n0.0\n", ann.pos[0], ann.pos[1]));
    }
    s.push_str("0\nENDSEC\n0\nEOF\n");
    s
}

/// Serialize a flat pattern to a self-contained SVG sized to the pattern bounds
/// (millimetre units), Y-flipped so the +Y-up flat plane reads right-side-up, over
/// a dark CAD canvas so the coloured layers read. Cut lines draw solid blue-green;
/// fold centerlines draw red with a phantom (long-short-short) dash; tangent lines
/// draw cyan dashed; each bend gets a yellow `<text>` label rotated to read along
/// its fold line.
pub fn to_svg(fp: &FlatPattern) -> String {
    let Some(b @ [min_x, min_y, max_x, max_y]) = bounds(&fp.polylines) else {
        // No geometry — an empty but valid SVG.
        return String::from(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"0mm\" height=\"0mm\" viewBox=\"0 0 0 0\"></svg>\n",
        );
    };
    let span = (max_x - min_x).max(max_y - min_y).max(1.0);
    let margin = span * 0.02;
    let width = (max_x - min_x) + 2.0 * margin;
    let height = (max_y - min_y) + 2.0 * margin;
    let stroke = (span * 0.003).max(0.05);
    let dash = (span * 0.02).max(0.5);
    let font = text_height(&b);

    // Span-scaled dash patterns. Phantom = long, gap, short, gap, short, gap.
    let long = dash * 2.0;
    let short = dash * 0.5;
    let phantom =
        format!("{long:.6} {dash:.6} {short:.6} {dash:.6} {short:.6} {dash:.6}");
    let dashed = format!("{:.6} {:.6}", dash, dash * 0.6);

    // Local -> SVG: shift into the padded box and flip Y (data +Y up -> SVG down).
    let sx = |x: f64| x - min_x + margin;
    let sy = |y: f64| max_y - y + margin;

    let mut s = String::new();
    s.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width:.6}mm\" height=\"{height:.6}mm\" viewBox=\"0 0 {width:.6} {height:.6}\">\n"
    ));
    // Dark canvas so the blue-green / red / cyan / yellow layers are legible.
    s.push_str(&format!(
        "<rect x=\"0\" y=\"0\" width=\"{width:.6}\" height=\"{height:.6}\" fill=\"#151515\"/>\n"
    ));
    s.push_str(&format!("<g fill=\"none\" stroke-width=\"{stroke:.6}\">\n"));
    for poly in &fp.polylines {
        if poly.points.is_empty() {
            continue;
        }
        let mut d = String::new();
        for (i, p) in poly.points.iter().enumerate() {
            d.push_str(if i == 0 { "M " } else { " L " });
            d.push_str(&format!("{:.6} {:.6}", sx(p[0]), sy(p[1])));
        }
        if poly.closed {
            d.push_str(" Z");
        }
        match poly.role {
            LoopRole::Cut => {
                s.push_str(&format!("<path d=\"{d}\" stroke=\"#00b3a4\"/>\n"));
            }
            LoopRole::Fold => {
                s.push_str(&format!(
                    "<path d=\"{d}\" stroke=\"#ff4d4d\" stroke-dasharray=\"{phantom}\"/>\n"
                ));
            }
            LoopRole::BendInternal => {
                s.push_str(&format!(
                    "<path d=\"{d}\" stroke=\"#22ccff\" stroke-dasharray=\"{dashed}\"/>\n"
                ));
            }
        }
    }
    // Bend labels. Translate to the (Y-flipped) midpoint, then rotate: the flip
    // negates the data-frame angle (SVG rotate() is CW / Y-down). `text-anchor`
    // center + a small upward `dy` seats the label just above the fold line.
    // `<text>` needs its own `fill` (the group is `fill="none"`); yellow.
    for ann in &fp.annotations {
        s.push_str(&format!(
            "<text transform=\"translate({:.6} {:.6}) rotate({:.6})\" \
text-anchor=\"middle\" dy=\"{:.6}\" fill=\"#ffd400\" font-size=\"{font:.6}\">{}</text>\n",
            sx(ann.pos[0]),
            sy(ann.pos[1]),
            -ann.rotation_deg,
            -font * 0.3,
            ann.text,
        ));
    }
    s.push_str("</g>\n</svg>\n");
    s
}

