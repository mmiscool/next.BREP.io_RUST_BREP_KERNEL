use super::*;
use super::frame::{rotate_about, Frame};
use super::plate::build_flat_plate;

/// Build a bend wedge for `bend` on the parent edge `a -> b` (parent-local 2D),
/// and return `(wedge_solid, child_frame)`. `fold` scales the bend angle so
/// `fold = 0` opens every bend flat.
pub(super) fn build_bend(
    bend: &Bend,
    a: [f64; 2],
    b: [f64; 2],
    parent: Frame,
    thickness: f64,
    fold: f64,
) -> Result<(Option<BrepSolid>, Frame), String> {
    // Positive `angle_deg` folds the leg toward the flat's −normal side; the
    // feature layer's `useOppositeCenterline` chooses the side by sign. Kept
    // positive so the wedge `revolve` sweeps a proper (non-inverted) solid.
    let theta = bend.angle_deg.to_radians() * fold;
    let half_t = thickness * 0.5;
    let r_mid = bend.mid_radius(thickness);
    let r_in = (r_mid - half_t).max(1e-6);
    let r_out = r_mid + half_t;

    // Parent-edge world geometry: start point, edge direction, in-plane outward
    // normal (CCW ⇒ right of a→b is outward), and the flat normal.
    let ea = parent.point(a[0], a[1], 0.0);
    let eb = parent.point(b[0], b[1], 0.0);
    let edge_len = eb.sub(ea).length();
    if !(edge_len > 1e-9) {
        return Err(format!("sheet-metal: bend `{}` on a degenerate edge", bend.id));
    }
    let e_hat = eb.sub(ea).normalized()?; // along the fold edge (canonical +Y)
    let n_hat = parent.w; // flat normal (radial at angle 0, canonical +X)

    // --- Unfolded (flat pattern): the bend opens into a flat allowance strip so
    // the developed sheet stays ONE connected piece, and the child lies coplanar
    // just past it (spacing = the neutral-fiber bend allowance). ---
    if theta.abs() < 1e-9 {
        let out0 = e_hat.cross(n_hat); // in-plane outward (fold direction)
        let allowance = bend.allowance(thickness);
        let child_frame = Frame {
            o: ea.add(out0.scale(allowance)),
            u: out0,
            v: e_hat,
            w: n_hat,
        };
        if allowance < 1e-9 {
            return Ok((None, child_frame)); // 0° bend: flats simply abut
        }
        let strip_frame = Frame {
            o: ea,
            u: out0,
            v: e_hat,
            w: n_hat,
        };
        let strip = build_flat_plate(&strip_flat(bend, allowance, edge_len), thickness, strip_frame)?;
        return Ok((Some(strip), child_frame));
    }

    // Signed fold: the revolve must stay a POSITIVE angle to sweep a proper solid.
    // The fold SIDE is carried by mirroring the wedge frame through the sheet
    // plane: `side = −1` flips the radial column (`u = −n̂`) AND the axial column
    // (`v = −ê`, origin moved to the far edge end) so the frame stays a proper
    // rotation (det +1 — `transform_brep` never sees a reflection) while the
    // sweep still leaves the flat through the SAME in-plane outward direction
    // `ê × n̂`. Positive `angle_deg` folds toward −n̂; a negative angle (SM.F's
    // `useOppositeCenterline`, or a path turning the other way in SM.CF) folds
    // toward +n̂ — the exact mirror, wall outside the edge on both sides.
    let side = if theta >= 0.0 { 1.0 } else { -1.0 };
    let mag = theta.abs();
    let n_eff = n_hat.scale(side);

    // Bend axis: parallel to the edge, offset -Rm·n_eff from the fold edge.
    let axis_a = ea.sub(n_eff.scale(r_mid));
    // Wedge placement. The canonical revolve sweeps its cross-section from
    // radial +u toward −w (right-handed about v), so the frame is chosen per
    // fold SIDE so the material always sweeps FORWARD off the hinge (toward
    // ê×n̂): a negative fold anchors at the b-end with the axial direction
    // reversed (v = −ê), which flips the sweep sense while keeping the frame a
    // proper rotation (no reflection). The positive branch is unchanged.
    let wedge_frame = if side >= 0.0 {
        Frame {
            o: axis_a,
            u: n_eff,
            v: e_hat,
            w: n_eff.cross(e_hat),
        }
    } else {
        let axis_b = eb.sub(n_eff.scale(r_mid));
        let v = e_hat.scale(-1.0);
        Frame {
            o: axis_b,
            u: n_eff,
            v,
            w: n_eff.cross(v),
        }
    };
    let wedge = match build_bend_wedge(bend, r_in, r_out, edge_len, mag)? {
        Some(wedge) => Some(transform_brep(&wedge, wedge_frame.affine()?, false)?),
        None => None,
    };

    // Child frame: the parent tangent rotated by `side·mag` about the fold edge
    // (the fold sense flips with the side, matching the revolve sense above),
    // placed at the angle-`mag` mid-tangent line, extending FORWARD along the
    // rotated circumference. `w = side·n_child (= u × v)` keeps the child frame
    // a proper rotation on both sides.
    let n_child = rotate_about(n_eff, e_hat, side * mag);
    let out0 = e_hat.cross(n_hat);
    let out_child = rotate_about(out0, e_hat, side * mag);
    let child_edge_a = axis_a.add(n_child.scale(r_mid));
    let child_frame = Frame {
        o: child_edge_a,
        u: out_child,
        v: e_hat,
        w: n_child.scale(side),
    };

    Ok((wedge, child_frame))
}

/// The flattened bend region: a rectangular strip `allowance × length` filling
/// the gap between the parent and child flats in the flat pattern.
///
/// ## Flat-pattern bend-line naming contract
///
/// The strip's id is `{bendId}:BAND:{DIR}:{angle}`, so [`build_flat_plate`]
/// names its faces:
/// * `{bendId}:BAND:{DIR}:{angle}:A` — the band's top face (sheet +normal side),
/// * `{bendId}:BAND:{DIR}:{angle}:B` — the bottom face,
/// * `{bendId}:BAND:{DIR}:{angle}:SIDE:s{i}` — the end walls (s1/s3 are the
///   fold-seam interfaces and are consumed by the union; s0/s2 survive on the
///   sheet rim).
///
/// `DIR` is `DOWN` when the authored fold goes toward the sheet's −normal (B)
/// side (positive `angle_deg`) and `UP` toward +normal (negative angle);
/// `{angle}` is `|angle_deg|` in degrees via `f64` Display (shortest exact
/// round-trip — an expression-driven angle like `100/3` prints all its digits).
/// The two FOLD LINES are the band faces' boundary edges shared with the
/// parent/child plate faces; the flat-pattern union keeps coplanar faces
/// un-merged (see [`evaluate`]) so those edges survive, and the derived edge
/// naming pass (`{faceA}|{faceB}[n]`) makes them selectable. Hole-rim flaps
/// route through the same machinery, so their bands carry the same names; a 0°
/// bend has no allowance (flats simply abut — no band), and a drawn collar is
/// not developable (no band; the flat pattern shows the enlarged opening).
fn strip_flat(bend: &Bend, allowance: f64, length: f64) -> Flat {
    let direction = if bend.angle_deg >= 0.0 { "DOWN" } else { "UP" };
    Flat {
        id: format!("{}:BAND:{direction}:{}", bend.id, bend.angle_deg.abs()),
        outline: vec![[0.0, 0.0], [allowance, 0.0], [allowance, length], [0.0, length]],
        edges: (0..4)
            .map(|i| Edge {
                id: format!("s{i}"),
                bend: None,
            })
            .collect(),
        holes: Vec::new(),
        hole_bends: Vec::new(),
        outline_curves: Default::default(),
    }
}

/// The annular wedge: revolve the through-thickness cross-section rectangle
/// `(Rin,0)-(Rout,0)-(Rout,L)-(Rin,L)` about the canonical +Y axis through the
/// origin, by `theta`. Profile point `(radial, axial)` → `Vec3(radial, axial, 0)`
/// (same convention as `hole.rs`).
fn build_bend_wedge(
    bend: &Bend,
    r_in: f64,
    r_out: f64,
    length: f64,
    theta: f64,
) -> Result<Option<BrepSolid>, String> {
    // A (near-)zero fold produces no wedge (flat pattern handles spacing itself).
    if theta.abs() < 1e-9 {
        return Ok(None);
    }
    let p = |radial: f64, axial: f64| Vec3::new(radial, axial, 0.0);
    let profile = vec![
        make_line(p(r_in, 0.0), p(r_out, 0.0))?,   // axial end @ y=0
        make_line(p(r_out, 0.0), p(r_out, length))?, // outer cylinder (Rout)
        make_line(p(r_out, length), p(r_in, length))?, // axial end @ y=L
        make_line(p(r_in, length), p(r_in, 0.0))?,  // inner cylinder (Rin)
    ];
    // side_names aligned to profile curves; cap_names = the two angular caps
    // (angle 0 / angle θ) — left None so the union consumes them at the plate
    // interfaces.
    let side_names = vec![
        Some(format!("{}:END:0", bend.id)),
        Some(format!("{}:A", bend.id)), // outer (top) bend face
        Some(format!("{}:END:L", bend.id)),
        Some(format!("{}:B", bend.id)), // inner bend face
    ];
    revolve_profile_brep_named(
        &profile,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        theta,
        &side_names,
        &[None, None],
    )
    .map(Some)
}
