use super::*;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    run(ctx, FlangeOptions::default())
}

/// Shared entry — SM.F calls with defaults, SM.HEM with its preset.
pub fn run(ctx: &FeatureContext, opts: FlangeOptions) -> FeatureResult {
    match build(ctx, opts) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext, opts: FlangeOptions) -> Result<FeatureResult, String> {
    // --- Resolve every pick: same body, parsed anchor each ---
    let names = selection_names(ctx);
    if names.is_empty() {
        // No hinge selected: pass through so the caller can repair/re-dispatch.
        return Ok(FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone()));
    }
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    let mut handle: Option<u32> = None;
    let mut targets: Vec<Target> = Vec::new();
    for name in &names {
        match resolve_selection(ctx, name)? {
            None => result.unresolved.push(name.clone()),
            Some((found, target)) => {
                match handle {
                    Some(existing) if existing != found => {
                        return Err(
                            "sheet-metal flange: every selected edge must belong to ONE sheet-metal body"
                                .into(),
                        )
                    }
                    _ => handle = Some(found),
                }
                if targets.contains(&target) {
                    return Err(format!("sheet-metal flange: `{name}` is selected twice"));
                }
                targets.push(target);
            }
        }
    }
    if !result.unresolved.is_empty() {
        return Ok(result);
    }
    let handle = handle.expect("at least one resolved selection");
    let Some(mut tree) = sheet_metal::get_tree(handle) else {
        return Err("sheet-metal flange: selected face is not on a sheet-metal body".into());
    };
    let body_name = solid_name_for(ctx, handle)
        .ok_or("sheet-metal flange: could not resolve the target body name")?;

    // --- Fold parameters (shared by every pick) ---
    let angle_deg = opts
        .angle_deg
        .unwrap_or_else(|| ctx.number("angle").unwrap_or(90.0));
    let flip = ctx
        .param("useOppositeCenterline")
        .and_then(|v| v.as_bool())
        .unwrap_or(false);
    let signed_angle = if flip { -angle_deg } else { angle_deg };

    let thickness = tree.thickness;
    let requested_radius = ctx.number("bendRadius").unwrap_or(0.0);
    let inside_radius = if requested_radius > 0.0 {
        requested_radius
    } else if opts.knife_zero_radius || tree.default_inside_radius <= 0.0 {
        KNIFE_RADIUS
    } else {
        tree.default_inside_radius
    };
    let k_factor = tree.default_k_factor;

    // flangeLength is measured from the `flangeLengthReference` datum (the
    // SolidWorks length reference): the straight leg is the given length minus
    // the mold-line setback tan(|θ|/2)·R (Inner Virtual Sharp) or
    // tan(|θ|/2)·(R+t) (Outer Virtual Sharp); Tangent to Bend takes the length
    // as the leg itself (zero setback).
    let raw_length = ctx.number("flangeLength").unwrap_or(opts.default_leg);
    let reference = match opts.length_reference {
        Some(fixed) => fixed.to_string(),
        None => ctx
            .param("flangeLengthReference")
            .and_then(|v| v.as_str())
            .unwrap_or("Outer Virtual Sharp")
            .to_string(),
    };
    // Tangent to Bend measures from where the bend ends; SolidWorks only allows
    // it for bends ≥ 90° (below that the tangent datum sits past the free rim),
    // so we refuse it loudly rather than build a surprising leg.
    if reference == "Tangent to Bend" && angle_deg.abs() < 90.0 {
        return Err(format!(
            "sheet-metal flange: the `Tangent to Bend` length reference needs a bend of at least 90° (got {:.1}°); use `Inner Virtual Sharp` or `Outer Virtual Sharp` for a shallower flange",
            angle_deg.abs()
        ));
    }
    let tan_half = (angle_deg.abs() * 0.5).to_radians().tan().max(0.0);
    let reference_setback = match reference.as_str() {
        "Inner Virtual Sharp" => inside_radius * tan_half,
        "Outer Virtual Sharp" => (inside_radius + thickness) * tan_half,
        "Tangent to Bend" => 0.0,
        other => {
            return Err(format!(
                "sheet-metal flange: unknown flangeLengthReference `{other}` (Inner Virtual Sharp | Outer Virtual Sharp | Tangent to Bend)"
            ))
        }
    };
    let leg = raw_length - reference_setback;
    if !(leg > GEOM_EPS) {
        return Err(format!(
            "sheet-metal flange: flangeLength {raw_length} minus the {reference} setback {reference_setback:.4} leaves no leg; lengthen it or use the `Tangent to Bend` reference"
        ));
    }

    // Flange position: `inset` names how far the fold line moves INTO the
    // material (Material Inside = t+R keeps the whole flange within the
    // original footprint at 90°; Material Outside = R; Bend Outside = 0) and a
    // positive `offset` pushes it back OUT.
    let inset = ctx
        .param("inset")
        .and_then(|v| v.as_str())
        .unwrap_or("Material Inside");
    let inset_shift = match inset {
        "Material Inside" => thickness + inside_radius,
        "Material Outside" => inside_radius,
        "Bend Outside" => 0.0,
        other => {
            return Err(format!(
                "sheet-metal flange: unknown inset `{other}` (Material Inside | Material Outside | Bend Outside)"
            ))
        }
    };
    let offset = ctx.number("offset").unwrap_or(0.0);

    // Bend relief: default `none` (the retired engine had no bend relief);
    // width defaults to the thickness, depth (past the fold line) to the
    // neutral-fiber bend allowance |θ|·(R + k·t).
    let relief = match ctx
        .param("reliefType")
        .and_then(|v| v.as_str())
        .unwrap_or("none")
    {
        "none" => ReliefType::None,
        "rectangular" => ReliefType::Rectangular,
        "obround" => ReliefType::Obround,
        "tear" => ReliefType::Tear,
        other => {
            return Err(format!(
                "sheet-metal flange: unknown reliefType `{other}` (none | rectangular | obround | tear)"
            ))
        }
    };
    let relief_width = match ctx.number("reliefWidth").unwrap_or(0.0) {
        width if width > 0.0 => width,
        _ => thickness,
    };
    let allowance = signed_angle.to_radians().abs() * (inside_radius + k_factor * thickness);
    let relief_depth = match ctx.number("reliefDepth").unwrap_or(0.0) {
        depth if depth > 0.0 => depth,
        _ => allowance,
    };
    let corner = match ctx
        .param("cornerType")
        .and_then(|v| v.as_str())
        .unwrap_or("open")
    {
        "open" => CornerType::Open,
        "closed" => CornerType::Closed,
        "miter" => CornerType::Miter,
        other => {
            return Err(format!(
                "sheet-metal flange: unknown cornerType `{other}` (open | closed | miter)"
            ))
        }
    };

    let geometry = FlangeParams {
        signed_angle,
        inside_radius,
        k_factor,
        thickness,
        leg,
        shift: inset_shift - offset,
        setback_start: ctx.number("edgeStartSetback").unwrap_or(0.0).max(0.0),
        setback_end: ctx.number("edgeEndSetback").unwrap_or(0.0).max(0.0),
        relief,
        relief_width,
        relief_depth,
        corner,
    };

    // --- Graft one bend per pick ---
    // Outline picks graft per-flat JOINTLY (the corner treatment must see every
    // pick on a flat at once); hole picks splice their loop's curve list, so
    // same-loop picks process highest-segment first to keep earlier indices
    // valid. Bend/flat NAMES still follow selection order (`k`).
    let multi = targets.len() > 1;
    let bend_name = |k: usize| {
        if multi {
            format!("{}:bend{k}", ctx.id)
        } else {
            format!("{}:bend", ctx.id)
        }
    };
    let child_name = |k: usize| {
        if multi {
            format!("{}:flat{k}", ctx.id)
        } else {
            format!("{}:flat", ctx.id)
        }
    };

    let mut outline_groups: Vec<(&str, Vec<OutlinePick>)> = Vec::new();
    for (k, target) in targets.iter().enumerate() {
        let Target::Outline { flat_id, edge_id } = target else { continue };
        let pick = OutlinePick {
            selection: k,
            edge_id: edge_id.clone(),
            bend_id: bend_name(k),
            child_id: child_name(k),
        };
        match outline_groups.iter_mut().find(|(id, _)| *id == flat_id) {
            Some((_, picks)) => picks.push(pick),
            None => outline_groups.push((flat_id, vec![pick])),
        }
    }
    for (flat_id, picks) in &outline_groups {
        let flat = tree.root.find_by_id_mut(flat_id)
            .ok_or_else(|| format!("sheet-metal flange: flat `{flat_id}` not found in the body"))?;
        graft_outline_group(flat, picks, &geometry)?;
    }

    let mut hole_order: Vec<usize> = (0..targets.len())
        .filter(|&k| matches!(targets[k], Target::Hole { .. }))
        .collect();
    hole_order.sort_by_key(|&k| match &targets[k] {
        Target::Hole { hole, segment, .. } => (*hole, std::cmp::Reverse(*segment)),
        Target::Outline { .. } => unreachable!("filtered to hole picks"),
    });
    for &k in &hole_order {
        let Target::Hole { flat_id, hole, segment } = &targets[k] else {
            unreachable!("filtered to hole picks")
        };
        let flat = tree.root.find_by_id_mut(flat_id)
            .ok_or_else(|| format!("sheet-metal flange: flat `{flat_id}` not found in the body"))?;
        graft_hole_flange(flat, *hole, *segment, &geometry, bend_name(k), child_name(k))?;
    }

    // --- Re-evaluate the mutated tree; the new body replaces the old (same name) ---
    let added = sheet_metal::register_folded(tree, &body_name)?;
    result.added.push(added);
    result.removed = vec![body_name];
    Ok(result)
}

// ---------------------------------------------------------------------------
// Selection resolution
// ---------------------------------------------------------------------------

/// Every string in the `faces` reference selection, in order.
fn selection_names(ctx: &FeatureContext) -> Vec<String> {
    ctx.param("faces")
        .and_then(|v| v.as_array())
        .map(|array| {
            array
                .iter()
                .filter_map(|entry| entry.as_str().map(str::to_string))
                .collect()
        })
        .unwrap_or_default()
}

/// Resolve one pick to `(body handle, target)`; `None` = unresolved (the caller may
/// repair). Face picks parse directly; a solid EDGE pick (`{A}|{B}`) uses
/// whichever adjacent face parses as a flange anchor.
fn resolve_selection(
    ctx: &FeatureContext,
    name: &str,
) -> Result<Option<(u32, Target)>, String> {
    if let Some(face) = ctx.scene.resolve_face(name) {
        if name.contains(":ISLAND:") {
            return Err(format!(
                "sheet-metal flange: `{name}` is a cutout ISLAND wall — island rims cannot fold"
            ));
        }
        let target = parse_target(name).ok_or_else(|| {
            format!(
                "sheet-metal flange: `{name}` is not a sheet-metal side face (…:SIDE:…) or hole bore segment (…:CUTOUT:hole:segment)"
            )
        })?;
        return Ok(Some((face.handle, target)));
    }
    if name.contains('|') {
        if let Some(edge) = ctx.scene.resolve_edge(name) {
            let base = strip_dedup_suffix(name);
            let mut candidates: Vec<Target> = base.split('|').filter_map(parse_target).collect();
            candidates.dedup_by(|a, b| a == b);
            return match candidates.len() {
                1 => Ok(Some((edge.handle, candidates.pop().expect("one candidate")))),
                0 => Err(format!(
                    "sheet-metal flange: edge `{name}` does not border a sheet-metal side/bore face"
                )),
                _ => Err(format!(
                    "sheet-metal flange: edge `{name}` borders two flangeable faces — pick the face itself"
                )),
            };
        }
    }
    Ok(None)
}

/// Parse a face name into its flange anchor: `{flat}:SIDE:{edge}` (outline) or
/// `{flat}:CUTOUT:{hole}:{segment}` (hole bore wall). The loop-level cap name
/// `{flat}:CUTOUT:{hole}` does NOT parse — the pick must name a segment.
fn parse_target(name: &str) -> Option<Target> {
    let base = strip_dedup_suffix(name);
    if let Some((flat, rest)) = base.split_once(":CUTOUT:") {
        if flat.is_empty() {
            return None;
        }
        let mut fields = rest.split(':');
        let hole = fields.next()?.parse::<usize>().ok()?;
        let segment = fields.next()?.parse::<usize>().ok()?;
        if fields.next().is_some() {
            return None;
        }
        return Some(Target::Hole {
            flat_id: flat.to_string(),
            hole,
            segment,
        });
    }
    let (flat, edge) = base.split_once(":SIDE:")?;
    if flat.is_empty() || edge.is_empty() {
        return None;
    }
    Some(Target::Outline {
        flat_id: flat.to_string(),
        edge_id: edge.to_string(),
    })
}

/// Strip a trailing collision-dedup suffix `[n]` from a scene name.
fn strip_dedup_suffix(name: &str) -> &str {
    if name.ends_with(']') {
        if let Some(open) = name.rfind('[') {
            if name[open + 1..name.len() - 1]
                .chars()
                .all(|c| c.is_ascii_digit())
            {
                return &name[..open];
            }
        }
    }
    name
}

/// The scene-map solid name currently bound to `handle` (reverse lookup).
fn solid_name_for(ctx: &FeatureContext, handle: u32) -> Option<String> {
    ctx.scene
        .solids
        .iter()
        .find(|(_, bound)| **bound == handle)
        .map(|(name, _)| name.clone())
}

/// The new wall for a (possibly corner-treated) band. Local x runs outward
/// from the bend (`0` = attach seam, `leg` = free rim), local y along the
/// hinge (`0` = band start, `span` = band end).
///
/// * [`WallEnd::Flat`] ends square at the band boundary.
/// * [`WallEnd::Extend`] (closed corners) stretches the wall past the band
///   boundary — past the START toward `y = −e`, past the END toward
///   `y = span + e` — covering the corner opening (the BEND never extends:
///   the tab hangs square off the wall plane with a free top edge).
/// * [`WallEnd::Miter`] cuts the wall 45°: full width at the attach seam,
///   `leg` shorter at the free rim (the classic mitered box-pan end).
pub(super) fn corner_child_flat(
    id: String,
    leg: f64,
    span: f64,
    start: WallEnd,
    end: WallEnd,
    bend_id: &str,
) -> Result<Flat, String> {
    let miter_cut = |wall: WallEnd| match wall {
        WallEnd::Miter => leg,
        _ => 0.0,
    };
    let (cut0, cut1) = (miter_cut(start), miter_cut(end));
    if cut0 + cut1 > span + GEOM_EPS {
        return Err(format!(
            "sheet-metal flange: the 45° miter cuts ({cut0:.4} + {cut1:.4}) on `{bend_id}` consume the {span:.4}-wide wall — shorten the leg or use cornerType `open`"
        ));
    }
    let ext = |wall: WallEnd| match wall {
        WallEnd::Extend(by) => by,
        _ => 0.0,
    };
    let (y0, y1) = (-ext(start), span + ext(end));
    // CCW with the attach seam along x = 0; a miter that exactly consumes the
    // free rim collapses the tip segment (dropped below).
    let raw = [
        [0.0, y0],
        [leg, y0 + cut0],
        [leg, y1 - cut1],
        [0.0, y1],
    ];
    let mut outline: Vec<[f64; 2]> = Vec::new();
    for point in raw {
        let distinct = outline.last().map_or(true, |last: &[f64; 2]| {
            ((point[0] - last[0]).powi(2) + (point[1] - last[1]).powi(2)).sqrt() > GEOM_EPS
        });
        if distinct {
            outline.push(point);
        }
    }
    if outline.len() < 3 {
        return Err(format!(
            "sheet-metal flange: wall outline for `{bend_id}` degenerates"
        ));
    }
    let edges = (0..outline.len())
        .map(|i| Edge {
            id: format!("{id}:e{i}"),
            bend: None,
        })
        .collect();
    Ok(Flat {
        id,
        outline,
        edges,
        holes: Vec::new(),
        hole_bends: Vec::new(),
        outline_curves: Default::default(),
    })
}
