use super::*;
use super::bend::build_bend;
use super::collar::{build_collar, segment_endpoints_2d};
use super::frame::Frame;
use super::plate::build_flat_plate;

/// One flat's placement in the FOLDED (`fold = 1`) part, plus the local data a
/// profile-mapping consumer (SM.CUTOUT) needs: the world frame, the outline
/// polygon, and the bend-wedge footprint strips adjoining the flat (regions a
/// cut loop must not enter — the fold arcs live there).
pub struct FlatPlacement {
    /// The flat's tree id (`Flat::id`) — the key for mutating it back.
    pub id: String,
    /// World frame: local `(x, y, z)` maps to `origin + x·u + y·v + z·w`.
    pub origin: Vec3,
    pub u: Vec3,
    pub v: Vec3,
    /// The flat normal (thickness direction).
    pub w: Vec3,
    /// The flat's closed CCW footprint polygon (local 2D). Curved outline
    /// segments ([`Flat::outline_curves`]) are DENSIFIED (16 samples each) so
    /// the consumers' polygon overlap/extent tests see the true bulge, not the
    /// chord skeleton — every consumer is a permissive containment/extent test,
    /// never index-aligned with `Flat::outline`.
    pub outline: Vec<[f64; 2]>,
    /// Bend-wedge footprints adjoining this flat, in ITS local 2D frame.
    pub wedge_strips: Vec<WedgeStrip>,
}

/// A conservative bend-wedge footprint: the strip beyond the directed edge
/// `a -> b` (on its RIGHT side), reaching `reach` outward. The parent flat gets
/// one per folded edge; the child flat gets one along its attach seam.
pub struct WedgeStrip {
    pub a: [f64; 2],
    pub b: [f64; 2],
    /// Outward reach: the bend's OUTER radius (>= the wedge's true in-plane reach).
    pub reach: f64,
    /// The bend id, for error messages.
    pub bend_id: String,
}

/// Walk `tree` at `fold = 1.0` and return every flat's world placement (the
/// same frames [`evaluate`] places plates with) plus bend-strip metadata.
/// Bend wedges built during the walk are discarded — this reuses [`build_bend`]
/// verbatim so the frames can never diverge from the geometry.
pub fn folded_flat_placements(tree: &SheetTree) -> Result<Vec<FlatPlacement>, String> {
    let mut placements = Vec::new();
    let root = Frame::from_placement(&tree.root_transform);
    collect_placements(&tree.root, root, tree.thickness, None, &mut placements)?;
    Ok(placements)
}

/// Recursive body of [`folded_flat_placements`]. `seam` is the child-side strip
/// of the bend that placed this flat (None for the root).
fn collect_placements(
    flat: &Flat,
    frame: Frame,
    thickness: f64,
    seam: Option<WedgeStrip>,
    out: &mut Vec<FlatPlacement>,
) -> Result<(), String> {
    let mut placement = FlatPlacement {
        id: flat.id.clone(),
        origin: frame.o,
        u: frame.u,
        v: frame.v,
        w: frame.w,
        outline: flat.densified_outline()?,
        wedge_strips: seam.into_iter().collect(),
    };
    let n = flat.outline.len();
    let mut children = Vec::new();
    for (index, edge) in flat.edges.iter().enumerate() {
        let Some(bend) = &edge.bend else { continue };
        let a = flat.outline[index];
        let b = flat.outline[(index + 1) % n];
        let r_out = bend.mid_radius(thickness) + thickness * 0.5;
        // Parent-side strip: beyond the fold edge a -> b (outward = right, CCW).
        placement.wedge_strips.push(WedgeStrip {
            a,
            b,
            reach: r_out,
            bend_id: bend.id.clone(),
        });
        let (_wedge, child_frame) = build_bend(bend, a, b, frame, thickness, 1.0)?;
        let seam_len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        // Child-side strip: the seam runs along child-local v from (0,0) to
        // (0, L); the wedge lies on the -u side, i.e. right of (0,L) -> (0,0).
        let child_seam = WedgeStrip {
            a: [0.0, seam_len],
            b: [0.0, 0.0],
            reach: r_out,
            bend_id: bend.id.clone(),
        };
        children.push((&*bend.child, child_frame, child_seam));
    }
    // Hole-rim flaps are flats too: recurse into every straight hole-bend child
    // (mirroring [`emit_hole_bend`]'s frame construction) so consumers like
    // SM.CUTOUT can target walls hanging off hole flaps. The parent gets the
    // flap's bend strip (the fold arc reaches INTO the opening); a collar has
    // no child flat — nothing to place. KNOWN GAP (documented, loud-free): a
    // collar's annular bend footprint is not represented as wedge strips, so a
    // flat-pattern cut crossing a collar rim is not rejected; the cut bakes
    // into the plate and leaves the collar untouched.
    for hole_bend in &flat.hole_bends {
        let HoleBendKind::Straight { child } = &hole_bend.kind else {
            continue;
        };
        let hole = flat.holes.get(hole_bend.hole).ok_or_else(|| {
            format!(
                "sheet-metal: hole bend `{}` references missing hole {} on flat `{}`",
                hole_bend.id, hole_bend.hole, flat.id
            )
        })?;
        let curve = hole.outer.get(hole_bend.segment).ok_or_else(|| {
            format!(
                "sheet-metal: hole bend `{}` references missing segment {} of hole {}",
                hole_bend.id, hole_bend.segment, hole_bend.hole
            )
        })?;
        let (mut a, mut b) = segment_endpoints_2d(curve)?;
        if hole_bend.reversed {
            std::mem::swap(&mut a, &mut b);
        }
        let bend = Bend {
            id: hole_bend.id.clone(),
            angle_deg: hole_bend.angle_deg,
            inside_radius: hole_bend.inside_radius,
            k_factor: hole_bend.k_factor,
            child: child.clone(),
        };
        let r_out = bend.mid_radius(thickness) + thickness * 0.5;
        placement.wedge_strips.push(WedgeStrip {
            a,
            b,
            reach: r_out,
            bend_id: bend.id.clone(),
        });
        let (_wedge, child_frame) = build_bend(&bend, a, b, frame, thickness, 1.0)?;
        let seam_len = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2)).sqrt();
        let child_seam = WedgeStrip {
            a: [0.0, seam_len],
            b: [0.0, 0.0],
            reach: r_out,
            bend_id: bend.id.clone(),
        };
        children.push((&**child, child_frame, child_seam));
    }
    out.push(placement);
    for (child, child_frame, child_seam) in children {
        collect_placements(child, child_frame, thickness, Some(child_seam), out)?;
    }
    Ok(())
}


/// Build the exact-BREP solid for `tree` at the given `fold` fraction
/// (`1.0` folded, `0.0` flat). Returns one merged solid.
pub fn evaluate(tree: &SheetTree, fold: f64) -> Result<BrepSolid, String> {
    if tree.thickness <= 0.0 {
        return Err(format!(
            "sheet-metal: thickness must be positive, got {}",
            tree.thickness
        ));
    }
    guard_curved_edge_bends(&tree.root)?;
    let mut parts: Vec<BrepSolid> = Vec::new();
    let root = Frame::from_placement(&tree.root_transform);
    emit_flat(&tree.root, root, tree.thickness, fold, &mut parts)?;
    // Flat pattern (fold ≈ 0): keep coplanar faces UN-merged so every bend's
    // allowance strip survives as named `{bendId}:BAND:…` faces and the fold
    // lines stay real BREP edges (see [`strip_flat`] for the naming contract).
    // Folded (fold ≠ 0): merge as before — the coplanar plate/wedge interfaces
    // are construction seams, not annotations.
    let mut solid = union_all(parts, fold.abs() >= 1e-9)?;
    // Name every plate/bend edge `{faceA}|{faceB}[n]` (sorted face names + a
    // per-pair counter) so the SM body's edges become pickable by name — the app
    // resolves an edge selection strictly by its scene name. In particular a
    // vertical thickness CORNER edge between two `{flat}:SIDE:{e}` walls becomes
    // `{flat}:SIDE:{eA}|{flat}:SIDE:{eB}[n]`, which the corner fillet/chamfer
    // feature parses back into that adjacent SIDE-face pair. This pass is the
    // shared assembly point for every SM feature (all call `evaluate`), at both
    // fold=0 and fold=1. It is purely ADDITIVE — `extrude_profile_brep` leaves SM
    // edges unnamed — and deterministic given SM's deterministic face names +
    // union topology. The UNFOLD feature re-stamps via `register_added`; that pass
    // is idempotent (it re-derives the same `{faceA}|{faceB}[n]` names), so
    // unfold's output stays byte-identical.
    crate::feature_pipeline::features::common::stamp_derived_edge_names(&mut solid);
    Ok(solid)
}

/// Refuse (loudly, before any geometry) a bend anchored on a CURVED outline
/// segment: the whole bend model — fold line, wedge revolve axis, setback
/// surgery, allowance strip — assumes a straight hinge. An outline segment is
/// curved iff its edge id keys [`Flat::outline_curves`]; flange setback surgery
/// keeps the fold segment's ORIGINAL id, so a graft onto a curved edge is
/// always caught here regardless of which surgery path produced it.
fn guard_curved_edge_bends(flat: &Flat) -> Result<(), String> {
    for edge in &flat.edges {
        let Some(bend) = &edge.bend else { continue };
        if flat.outline_curves.contains_key(&edge.id) {
            return Err(format!(
                "sheet-metal: edge `{}` of flat `{}` is a curved outline edge — \
                 flanges/hems fold straight edges only",
                edge.id, flat.id
            ));
        }
        guard_curved_edge_bends(&bend.child)?;
    }
    for hole_bend in &flat.hole_bends {
        if let HoleBendKind::Straight { child } = &hole_bend.kind {
            guard_curved_edge_bends(child)?;
        }
    }
    Ok(())
}

/// Emit the plate for `flat` (placed by `frame`) plus, recursively, every bend
/// hanging off its edges and the child flats they reach.
fn emit_flat(
    flat: &Flat,
    frame: Frame,
    thickness: f64,
    fold: f64,
    parts: &mut Vec<BrepSolid>,
) -> Result<(), String> {
    parts.push(build_flat_plate(flat, thickness, frame)?);

    for (index, edge) in flat.edges.iter().enumerate() {
        let Some(bend) = &edge.bend else { continue };
        let n = flat.outline.len();
        let a = flat.outline[index];
        let b = flat.outline[(index + 1) % n];
        let (wedge, child_frame) = build_bend(bend, a, b, frame, thickness, fold)?;
        if let Some(wedge) = wedge {
            parts.push(wedge);
        }
        emit_flat(&bend.child, child_frame, thickness, fold, parts)?;
    }
    for hole_bend in &flat.hole_bends {
        emit_hole_bend(flat, hole_bend, frame, thickness, fold, parts)?;
    }
    Ok(())
}

/// Emit the geometry of one hole-rim fold: a straight window segment folds a
/// child wall INTO the opening through the same [`build_bend`] machinery as an
/// outline flange; a circular loop grows a [`build_collar`] revolve.
fn emit_hole_bend(
    flat: &Flat,
    hole_bend: &HoleBend,
    frame: Frame,
    thickness: f64,
    fold: f64,
    parts: &mut Vec<BrepSolid>,
) -> Result<(), String> {
    let hole = flat.holes.get(hole_bend.hole).ok_or_else(|| {
        format!(
            "sheet-metal: hole bend `{}` references missing hole {} on flat `{}`",
            hole_bend.id, hole_bend.hole, flat.id
        )
    })?;
    match &hole_bend.kind {
        HoleBendKind::Straight { child } => {
            let curve = hole.outer.get(hole_bend.segment).ok_or_else(|| {
                format!(
                    "sheet-metal: hole bend `{}` references missing segment {} of hole {}",
                    hole_bend.id, hole_bend.segment, hole_bend.hole
                )
            })?;
            let (mut a, mut b) = segment_endpoints_2d(curve)?;
            if hole_bend.reversed {
                std::mem::swap(&mut a, &mut b);
            }
            // Reuse the outline-bend machinery through a transient Bend record —
            // the (possibly reversed) segment direction already puts the opening
            // on the fold side (`ê × n̂`).
            let bend = Bend {
                id: hole_bend.id.clone(),
                angle_deg: hole_bend.angle_deg,
                inside_radius: hole_bend.inside_radius,
                k_factor: hole_bend.k_factor,
                child: child.clone(),
            };
            let (wedge, child_frame) = build_bend(&bend, a, b, frame, thickness, fold)?;
            if let Some(wedge) = wedge {
                parts.push(wedge);
            }
            emit_flat(child, child_frame, thickness, fold, parts)
        }
        HoleBendKind::Collar { leg } => {
            if let Some(collar) =
                build_collar(hole_bend, &hole.outer, *leg, frame, thickness, fold)?
            {
                parts.push(collar);
            }
            Ok(())
        }
    }
}

/// Union every part into one solid (base ∪ wedge ∪ child ∪ …). Empty/degenerate
/// parts are skipped. `merge_coplanar` merges the coplanar plate/bend interface
/// faces (the folded body); the flat pattern passes `false` so the bend BAND
/// faces — same plane as the plates on purpose — keep their identity.
fn union_all(parts: Vec<BrepSolid>, merge_coplanar: bool) -> Result<BrepSolid, String> {
    let options = BooleanOptions {
        merge_coplanar_faces: merge_coplanar,
        // Keep every PLANAR thickness/wall face per outline segment even when the
        // folded body coalesces coplanar faces: the big top/bottom plate faces
        // (`:A`/`:B`, and a blind pocket floor) still merge, but a plate side
        // wall (`:SIDE:`), a bend transverse end cap (`:END:`), a hole-bore or
        // island wall (`:CUTOUT:`), and a collar wall (`:wall:`) each stay their
        // own face. Fusing two collinear thickness faces (e.g. a plate side wall
        // meeting a flange's bend end in the same transverse plane) would erase
        // the segment boundary a later flange needs to attach to. Relief/corner
        // treatments bake into the outline (`:SIDE:`) or holes (`:CUTOUT:`), so
        // this list covers them too. The flat pattern passes
        // `merge_coplanar = false`, so the list is inert there.
        keep_unmerged_name_substrs: vec![
            ":SIDE:".to_string(),
            ":END:".to_string(),
            ":CUTOUT:".to_string(),
            ":wall:".to_string(),
        ],
        ..BooleanOptions::default()
    };
    let mut current: Option<BrepSolid> = None;
    for part in parts {
        if part.shells.is_empty() {
            continue; // a zero-fold wedge, etc.
        }
        current = Some(match current.take() {
            None => part,
            Some(running) => {
                let running_handle = crate::register_solid_value(running);
                let part_handle = crate::register_solid_value(part);
                let folded = crate::with_two_registered_solids(
                    running_handle,
                    part_handle,
                    |run, add| boolean_operation(run, add, BooleanOperation::Union, &options),
                );
                crate::free_registered_solid(running_handle);
                crate::free_registered_solid(part_handle);
                folded.map_err(|error| format!("sheet-metal union failed: {error}"))?
            }
        });
    }
    current.ok_or_else(|| "sheet-metal: tree produced no geometry".to_string())
}
