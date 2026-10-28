//! What the 3D roughing strategies share: the resolved level inputs (stock
//! slice, part shadow, feasible set per level) and the stock-aware link.

use super::dilate::checked;
use super::levels::{merged_section, overhang_pieces, plan_levels, resolve_slabs, stock_slice, LevelPlan};
use crate::collision::{simulate_checked, Collision, CollisionAgainst, CollisionOptions, CollisionPart};
use crate::cutter::{ToolPart, ToolProfile};
use crate::link::{connect, LinkContext, LinkOptions, Pass};
use crate::model::{Containment, Depths, Diagnostic, Heights, Motion, OperationReport, Point3, RetractPolicy, Toolpath};
use crate::planar::machining::{invalid, OpContext};
use crate::planar::{Loop, Region};
use crate::stock::ResolvedStock;
use crate::stock_sim::removal::RemovalOptions;
use crate::stock_sim::{DexelGrid, DexelOptions};
use crate::{CamError, Runner, Tolerance};

/// The material situation of one Z level.
pub struct LevelGeometry {
    pub z: f64,
    /// Top of the slab this level removes (the level above, or the top of cut).
    pub z_top: f64,
    pub stock: Region,
    pub shadow: Region,
    /// Material to remove: stock minus the part shadow (and its allowance).
    pub material: Region,
    pub feasible: Region,
}

/// Levels and per-level regions of a roughing operation.
pub struct RoughingInput {
    pub plan: LevelPlan,
    pub levels: Vec<LevelGeometry>,
    pub stock_top: f64,
    /// Levels left uncut because the tool above its flutes could not be
    /// checked (see `reach_check`).
    pub warnings: Vec<Diagnostic>,
    /// The stock deep levels' passes are checked against (`finish_stock_aware`).
    pub reach: Option<ReachCheck>,
}

/// Resolve the levels. `load` grows the feasible set's outer boundary so the
/// first front sits in the air; roughing by offsets passes `stepover`.
#[allow(clippy::too_many_arguments)]
pub fn resolve_levels(
    ctx: &OpContext<'_>,
    depths: &Depths,
    fine_step: f64,
    containment: &Containment,
    stock: &ResolvedStock,
    rest: Option<&DexelGrid>,
    load: f64,
    runner: &mut Runner<'_>,
) -> Result<RoughingInput, CamError> {
    let tol = &ctx.tolerance;
    let r = ctx.radius();
    let radial = ctx.operation.stock_to_leave.radial;
    let axial = ctx.operation.stock_to_leave.axial;
    let margin = crate::pocket2d::wall_margin(tol, ctx.operation.tolerance);
    let stock_top = match (stock, rest) {
        (ResolvedStock::Rest(_), Some(g)) => g.z_range[1],
        (s, _) => s.top_z().ok_or_else(|| invalid(ctx.strategy, "rest stock needs the previous setup's simulated stock"))?,
    };
    if ctx.heights.clearance <= stock_top + tol.distance_eps {
        return Err(invalid(ctx.strategy, "clearance height must be above the stock"));
    }
    let top = depths.top.or(ctx.heights.top).unwrap_or(stock_top).min(stock_top);
    let bottom = depths.bottom.or(ctx.heights.bottom).unwrap_or(ctx.mesh.bounds.min.z) + axial;
    if bottom > top + tol.distance_eps {
        return Err(invalid(ctx.strategy, format!("bottom of cut {bottom} is above the top of cut {top}")));
    }
    let targets = &ctx.operation.targets;
    let mesh = ctx.mesh;
    let is_target = |face: u32| {
        let f = &mesh.faces[face as usize];
        !f.protected_only && targets.target_solids.contains(&f.solid)
    };
    let plan = plan_levels(mesh, is_target, top, bottom, depths, fine_step, axial, tol)?;
    let mut levels = Vec::with_capacity(plan.levels.len());
    let mut above = top;
    // The shadow of a level is everything of the protected mesh (fixtures
    // too) above it, not just its own slab: the cutter and its shank reach
    // up, so an overhang higher up (the roof of a horizontal hole) bars the
    // positions under it. It is accumulated top-down: each band between two
    // levels adds sections at its ends, three interior heights and every
    // flat in it. A bulge thinner than the sampling between two samples is
    // left to the pass certificate, which refuses it by name.
    let mut shadow_above = Region::empty();
    let mut sampled_to = mesh.bounds.max.z + tol.distance_eps;
    // The projections of every downward-facing surface above the level,
    // accumulated top-down band by band (see `overhang_pieces`).
    let mut overhang = Region::empty();
    let mut overhang_to = f64::INFINITY;
    runner.stage("roughing-levels", "Resolve roughing levels", 0, plan.levels.len() as u64)?;
    // The shadows accumulate top-down; each level's feasible set and material
    // depend on its own stock and shadow only, so they are resolved after the
    // walk (`resolve_slabs`) and joined in order. An error of the walk at a
    // level is reported after any of the levels above it, as the single loop
    // did.
    let mut slabs: Vec<(f64, f64, Region, Region)> = Vec::with_capacity(plan.levels.len());
    let mut walk_error = None;
    for &z in plan.levels.iter() {
        match shadow_walk_step(ctx, stock, rest, &plan, z, &mut above, &mut sampled_to, &mut shadow_above, &mut overhang, &mut overhang_to) {
            Ok(slab) => slabs.push(slab),
            Err(e) => {
                walk_error = Some(e);
                break;
            }
        }
    }
    let resolved = resolve_slabs(&slabs, r, load, radial, margin, ctx.operation.tolerance, containment, tol, runner)?;
    for (i, ((z, slab_top, stock_region, shadow), done)) in slabs.into_iter().zip(resolved).enumerate() {
        let (feasible, material) = done?;
        levels.push(LevelGeometry { z, z_top: slab_top, stock: stock_region, shadow, material, feasible });
        runner.work(1, "roughing-levels", i as u64 + 1, plan.levels.len() as u64)?;
    }
    if let Some(e) = walk_error {
        return Err(e);
    }
    let (reach, warnings) = reach_check(ctx, stock, rest, &mut levels);
    Ok(RoughingInput { plan, levels, stock_top, warnings, reach })
}

/// The stock a deep roughing checks its passes against (`keep_clear`): the
/// op's starting stock as the dexel grid the simulation builds, so a pass is
/// kept exactly when the simulation finds its shank and holder clear.
pub struct ReachCheck {
    grid: DexelGrid,
}

/// The reach check of `levels`: none when no level lies more than a cutting
/// length below the top (the tool above its flutes never meets the stock),
/// else the op's stock as a grid. When that grid cannot be built, the deeper
/// levels are left uncut and say so.
fn reach_check(ctx: &OpContext<'_>, stock: &ResolvedStock, rest: Option<&DexelGrid>, levels: &mut [LevelGeometry]) -> (Option<ReachCheck>, Vec<Diagnostic>) {
    let tol = &ctx.tolerance;
    let flutes = ctx.tool.geometry.cutting_length;
    let (Some(top), Some(deepest)) = (levels.first().map(|l| l.z_top), levels.last().map(|l| l.z)) else {
        return (None, Vec::new());
    };
    if top <= deepest + flutes + tol.distance_eps {
        return (None, Vec::new());
    }
    let grid = DexelGrid::from_stock(stock, &DexelOptions::default(), rest, None);
    match grid {
        Ok(grid) => (Some(ReachCheck { grid }), Vec::new()),
        Err(e) => {
            let mut warnings = Vec::new();
            for lv in levels.iter_mut().filter(|l| l.z + flutes < top - tol.distance_eps && !l.feasible.is_empty()) {
                lv.feasible = Region::empty();
                warnings.push(Diagnostic::new(
                    "roughing-reach-unchecked",
                    format!("level Z={:.3} left uncut: the tool above its {flutes:.1} mm flutes could not be checked against the stock ({e:?})", lv.z),
                ));
            }
            (None, warnings)
        }
    }
}


/// How much wider than the tool its shank and holder are checked: a shank as
/// wide as the cutter runs in the column the flutes cut, at exactly their
/// radius, so a graze must fall on the safe side of the tangency test.
const REACH_MARGIN: f64 = 1e-3;

/// The shank- and holder-vs-stock motions of `motions` run on `grid` (which
/// they then have cut), as the simulation checks them, the shank and holder
/// `REACH_MARGIN` wider. A motion whose lowest point is a cutting length or
/// more below the stock top is checked; above that the tool above its flutes
/// clears the stock, and the motion only cuts.
fn reach_hits(grid: &mut DexelGrid, ctx: &OpContext<'_>, motions: &[Motion], runner: &mut Runner<'_>) -> Result<Vec<Collision>, CamError> {
    let mut profile = ToolProfile::new(&ctx.cutter, ctx.tool.holder.as_ref(), 0.01);
    for p in profile.points.iter_mut().filter(|p| p.part != ToolPart::Cutting) {
        p.radius += REACH_MARGIN;
    }
    let removal = RemovalOptions::default();
    let clear_above = grid.z_range[1] - ctx.tool.geometry.cutting_length + ctx.tolerance.distance_eps;
    let mut hits = Vec::new();
    let mut previous: Option<Point3> = None;
    let mut k = 0;
    while k < motions.len() {
        let deep = |m: &Motion, from: Option<Point3>| m.end().is_some_and(|e| from.map_or(e.z, |p| p.z.min(e.z)) < clear_above);
        let start = previous;
        let is_deep = deep(&motions[k], start);
        let mut end = k;
        let mut at = start;
        while end < motions.len() && deep(&motions[end], at) == is_deep {
            at = motions[end].end().or(at);
            end += 1;
        }
        // Each run starts where the one before left the tool.
        let lead = usize::from(start.is_some());
        let run: Vec<Motion> = start.map(Motion::rapid).into_iter().chain(motions[k..end].iter().cloned()).collect();
        if is_deep {
            let sim = simulate_checked(grid, None, &ctx.cutter, &profile, &run, &ctx.tolerance, &removal, &CollisionOptions::default(), runner)?;
            hits.extend(
                sim.collisions
                    .into_iter()
                    .filter(|c| matches!(c.part, CollisionPart::Shank | CollisionPart::Holder) && c.against == CollisionAgainst::Stock && c.motion_index >= lead)
                    .map(|c| Collision { motion_index: c.motion_index - lead + k, ..c }),
            );
        } else {
            crate::stock_sim::removal::simulate(grid, &ctx.cutter, &run, &ctx.tolerance, &removal, runner)?;
        }
        previous = at;
        k = end;
    }
    Ok(hits)
}

/// The passes in cutting order with every stretch whose shank or holder
/// would ride in the stock (as it stands after the passes before) left out:
/// each pass enters from above the stock and cuts on a copy of the grid; a
/// hit splits it, a long stretch is first divided so only its colliding part
/// is lost. The cut length left per level is reported.
fn keep_clear(ctx: &OpContext<'_>, check: &ReachCheck, passes: &[Pass], report: &mut OperationReport, runner: &mut Runner<'_>) -> Result<Vec<Pass>, CamError> {
    let above = check.grid.z_range[1] + ctx.operation.linking.safe_distance.max(ctx.tolerance.distance_eps);
    let fine = 2.0 * check.grid.cell_x.max(check.grid.cell_y);
    let feed = ctx.feeds.cutting_feed;
    let mut grid = check.grid.clone();
    let mut out = Vec::with_capacity(passes.len());
    let mut lost: Vec<(f64, f64)> = Vec::new();
    for pass in passes {
        let mut queue: std::collections::VecDeque<Vec<Point3>> = std::collections::VecDeque::from([pass.points.clone()]);
        while let Some(points) = queue.pop_front() {
            if points.len() < 2 {
                continue;
            }
            let p0 = points[0];
            let mut motions = vec![Motion::rapid(Point3::new(p0.x, p0.y, above)), Motion::plunge(p0, ctx.feeds.plunge_feed)];
            motions.extend(points[1..].iter().map(|p| Motion::cut(*p, feed)));
            let mut trial = grid.clone();
            let hits = reach_hits(&mut trial, ctx, &motions, runner)?;
            if hits.is_empty() {
                grid = trial;
                out.push(Pass::new(points, pass.path_id));
                continue;
            }
            // Segment k is motion k + 2; a hit on the entry loses the first.
            let bad = |k: usize| hits.iter().any(|h| h.motion_index == k + 2 || (k == 0 && h.motion_index < 2));
            let long = (0..points.len() - 1).any(|k| bad(k) && points[k].distance(points[k + 1]) > fine);
            if long {
                let mut refined = vec![p0];
                for k in 0..points.len() - 1 {
                    let (a, b) = (points[k], points[k + 1]);
                    let n = if bad(k) { (a.distance(b) / fine).ceil().max(1.0) as usize } else { 1 };
                    refined.extend((1..=n).map(|s| a.add(b.sub(a).scale(s as f64 / n as f64))));
                }
                queue.push_front(refined);
                continue;
            }
            let mut pieces = Vec::new();
            let mut piece = vec![p0];
            for k in 0..points.len() - 1 {
                if bad(k) {
                    lost.push((points[k].z, points[k].distance(points[k + 1])));
                    if piece.len() >= 2 {
                        pieces.push(std::mem::take(&mut piece));
                    }
                    piece = vec![points[k + 1]];
                } else {
                    piece.push(points[k + 1]);
                }
            }
            if piece.len() >= 2 {
                pieces.push(piece);
            }
            for p in pieces.into_iter().rev() {
                queue.push_front(p);
            }
        }
    }
    let flutes = ctx.tool.geometry.cutting_length;
    let mut levels: Vec<(f64, f64)> = Vec::new();
    for (z, len) in lost {
        match levels.iter_mut().find(|(lz, _)| (*lz - z).abs() <= ctx.tolerance.distance_eps) {
            Some(l) => l.1 += len,
            None => levels.push((z, len)),
        }
    }
    for (z, len) in levels {
        report.warn(Diagnostic::new("roughing-beyond-reach", format!("{len:.1} mm of cut at Z={z:.3} left: stock above is beyond the tool's {flutes:.1} mm reach")));
    }
    Ok(out)
}

/// One level of the top-down shadow walk of `resolve_levels`: the level's
/// stock slice and shadow, `(z, slab top, stock, shadow)`, advancing the
/// accumulated state.
#[allow(clippy::too_many_arguments)]
fn shadow_walk_step(
    ctx: &OpContext<'_>,
    stock: &ResolvedStock,
    rest: Option<&DexelGrid>,
    plan: &LevelPlan,
    z: f64,
    above: &mut f64,
    sampled_to: &mut f64,
    shadow_above: &mut Region,
    overhang: &mut Region,
    overhang_to: &mut f64,
) -> Result<(f64, f64, Region, Region), CamError> {
    let tol = &ctx.tolerance;
    let mesh = ctx.mesh;
    let axial = ctx.operation.stock_to_leave.axial;
    {
        let slab_top = above.max(z);
        let stock_region = stock_slice(stock, rest, z, slab_top, tol)?;
        let band_top = *sampled_to;
        let mut heights: Vec<f64> = (0..=4).map(|k| z + (band_top - z) * k as f64 / 4.0).collect();
        for h in heights.iter_mut() {
            if plan.flats.iter().any(|f| (*f - *h).abs() <= 1e-6) {
                *h -= 1e-4;
            }
        }
        heights.extend(input_flats(plan, z, band_top));
        heights.retain(|h| *h > z && *h <= band_top);
        for h in heights {
            if h < mesh.bounds.min.z - tol.distance_eps || h > mesh.bounds.max.z + tol.distance_eps {
                continue;
            }
            let section = merged_section(mesh, h, |_| true, tol)?;
            if !section.is_empty() {
                *shadow_above = if shadow_above.is_empty() { section } else { checked("shadow above the level", shadow_above, &section, || shadow_above.union(&section, tol))? };
            }
        }
        *sampled_to = z;
        // The level's own section, and the one the axial stock reaches down to.
        let mut own = Region::empty();
        // On a flat's plane the section is ambiguous; material exactly at the
        // cut's height cannot hit a flat-bottomed cutter, so the sample moves
        // just above the plane (the level at a top face then faces over it;
        // the pass certificate still proves every point against the floor).
        let on_flat = |h: f64| if plan.flats.iter().any(|f| (*f - h).abs() <= 1e-6) { h + 1e-4 } else { h };
        for h in if axial > tol.distance_eps { vec![on_flat(z - axial), on_flat(z)] } else { vec![on_flat(z)] } {
            if h < mesh.bounds.min.z - tol.distance_eps || h > mesh.bounds.max.z + tol.distance_eps {
                continue;
            }
            let section = merged_section(mesh, h, |_| true, tol)?;
            if !section.is_empty() {
                own = if own.is_empty() { section } else { checked("the level's own section", &own, &section, || own.union(&section, tol))? };
            }
        }
        let mut shadow = if shadow_above.is_empty() { own } else if own.is_empty() { shadow_above.clone() } else { checked("the level's shadow", shadow_above, &own, || shadow_above.union(&own, tol))? };
        let pieces = overhang_pieces(mesh, z, *overhang_to, tol);
        *overhang_to = z;
        if !pieces.is_empty() {
            let mut loops = std::mem::take(&mut overhang.loops);
            loops.extend(pieces);
            let all = Region { loops };
            *overhang = checked("overhang union", &all, &Region::empty(), || all.union(&Region::empty(), tol))?;
        }
        if !overhang.is_empty() {
            // The shadow grows by the loops of the overhang beyond the sampled
            // sections that reach out of it by more than a quarter of the
            // machining tolerance. The feasible set keeps the cutter the
            // radius plus the allowance plus the wall margin (the machining
            // tolerance) off the shadow, so a dropped loop's material stays
            // more than the radius away. What is dropped is rounding and the
            // slivers between a sampled section's facets and a flank's
            // projection (the real bracket: 448 such loops, one with a 6.5e-7
            // stub the planar union could not stitch). A thin leaning flank
            // that reaches past that distance anywhere is kept however narrow.
            // The loops of the difference lie on the shadow's own boundary
            // pieces, so the union never meets a near-copy of a wall.
            let beyond = checked("overhang beyond the sampled shadow", overhang, &shadow, || overhang.difference(&shadow, tol))?;
            shadow = grow_by_overhangs(shadow, beyond, 0.25 * ctx.operation.tolerance.max(tol.point_eps), tol)?;
        }
        *above = z;
        Ok((z, slab_top, stock_region, shadow))
    }
}

/// `shadow` united with the loops of `beyond` (the overhang outside it) that
/// reach more than `reach_out` from it anywhere (checked at every vertex and
/// edge midpoint); loops within that distance are dropped (see
/// `resolve_levels`).
pub fn grow_by_overhangs(shadow: Region, beyond: Region, reach_out: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let reaches = |l: &Loop| l.segments.iter().any(|seg| [seg.start, seg.midpoint()].iter().any(|q| shadow.nearest_boundary(*q).map_or(true, |(_, d)| d > reach_out)));
    let loops: Vec<Loop> = beyond.loops.into_iter().filter(|l| l.signed_area().abs() > tol.area_eps && reaches(l)).collect();
    if !loops.iter().any(|l| l.signed_area() > 0.0) {
        return Ok(shadow);
    }
    let reach = Region { loops };
    checked("shadow with overhangs", &shadow, &reach, || shadow.union(&reach, tol))
}

/// Flats in the band `(z, top]`, each sampled just below and just above its
/// plane, never on it: on a face's own plane the walls ending exactly there
/// may or may not cross by rounding (a tessellated hole wall did, the block's
/// planar walls did not, and the section was the hole alone).
fn input_flats(plan: &LevelPlan, z: f64, top: f64) -> Vec<f64> {
    plan.flats.iter().filter(|f| **f > z && **f <= top + 1e-3).flat_map(|f| [*f - 1e-4, *f + 1e-4]).collect()
}

/// Link certified passes with the stock in mind: no keep-down links (the
/// linker proves them against the part, not the stock), retracts at least
/// the safe distance above the stock top, then the chord filter and the arc
/// fitter. `helix_diameter` overrides the operation's when positive.
/// With a reach check (`RoughingInput::reach`), the passes first lose what
/// the tool above its flutes cannot cut (`keep_clear`), and the linked path
/// is then run whole on the starting stock: a link or entry that still meets
/// it cuts the pass at the contact, and the passes are linked again.
#[allow(clippy::too_many_arguments)]
pub fn finish_stock_aware(ctx: &OpContext<'_>, passes: &[Pass], stock_top: f64, helix_diameter: Option<f64>, mut report: OperationReport, reach: Option<&ReachCheck>, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
    let Some(check) = reach else {
        return link_stock_aware(ctx, passes, stock_top, helix_diameter, report, runner);
    };
    let mut kept = keep_clear(ctx, check, passes, &mut report, runner)?;
    for _ in 0..4 {
        let path = link_stock_aware(ctx, &kept, stock_top, helix_diameter, report.clone(), runner)?;
        let mut grid = check.grid.clone();
        let mut hits = reach_hits(&mut grid, ctx, &path.motions, runner)?;
        if hits.is_empty() {
            return Ok(path);
        }
        // One cut per colliding motion (its first contact).
        hits.dedup_by_key(|h| h.motion_index);
        for h in &hits {
            let id = path.motions.get(h.motion_index).and_then(|m| m.path_id);
            let nearest = kept
                .iter()
                .enumerate()
                .filter(|(_, p)| id.is_none() || p.path_id == id)
                .flat_map(|(i, p)| p.points.iter().enumerate().map(move |(k, q)| (i, k, q.distance(h.position))))
                .min_by(|a, b| a.2.total_cmp(&b.2));
            let Some((i, k, _)) = nearest else { continue };
            let pass = kept.remove(i);
            let (before, after) = (pass.points[..k].to_vec(), pass.points[k + 1..].to_vec());
            for piece in [after, before] {
                if piece.len() >= 2 {
                    kept.insert(i, Pass::new(piece, pass.path_id));
                }
            }
            report.warn(Diagnostic::new("roughing-beyond-reach", format!("the entry or link at ({:.3}, {:.3}, {:.3}) left: stock above is beyond the tool's {:.1} mm reach", h.position.x, h.position.y, h.position.z, ctx.tool.geometry.cutting_length)));
        }
    }
    Err(invalid(ctx.strategy, "the tool above its flutes still meets the stock after linking around it"))
}

/// Link the passes (`finish_stock_aware` without a reach check).
fn link_stock_aware(ctx: &OpContext<'_>, passes: &[Pass], stock_top: f64, helix_diameter: Option<f64>, mut report: OperationReport, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
    if passes.is_empty() {
        return Err(invalid(ctx.strategy, "no passes to link (empty or unreachable region)"));
    }
    let safe = ctx.operation.linking.safe_distance.max(ctx.tolerance.distance_eps);
    let mut heights: Heights = ctx.heights.clone();
    heights.retract = heights.retract.max(stock_top + safe).min(heights.clearance);
    heights.feed = heights.feed.max(stock_top + safe).min(heights.clearance);
    let mut params = ctx.operation.linking.clone();
    params.keep_down_distance = 0.0;
    if params.retract_policy == RetractPolicy::MinimumRetract {
        params.retract_policy = RetractPolicy::RetractHeight;
    }
    if let Some(d) = helix_diameter {
        if d > 0.0 {
            params.ramp.helix_diameter = d;
        }
    }
    let options = LinkOptions { chord_tolerance: ctx.operation.tolerance, reorder: false, allow_reverse: false, ..LinkOptions::default() };
    let linked = crate::link::with_current_pre_op_stock(|stock| {
        let link_ctx = LinkContext { mesh: ctx.mesh, cutter: &ctx.cutter, index: ctx.index, tolerance: ctx.tolerance, heights: &heights, feeds: &ctx.feeds, params: &params, options, stock };
        connect(passes, &link_ctx, runner)
    })?;
    report.warnings.extend(linked.report.warnings);
    report.counts.samples += linked.report.counts.samples;
    report.counts.candidates += linked.report.counts.candidates;
    report.counts.contacts += linked.report.counts.contacts;
    let mut path = Toolpath::new(ctx.operation.id.clone(), ctx.tool.id.clone());
    path.motions = linked.motions;
    path.report = report;
    let filtered = crate::filter::filter_toolpath(&path, ctx.tolerance.distance_eps)?;
    let mut out = if ctx.operation.linking.fit_arcs { crate::arcfit::fit_toolpath(&filtered, &crate::arcfit::ArcFitOptions::with_tolerance(ctx.operation.tolerance))? } else { filtered };
    out.report.counts.triangles = ctx.mesh.triangle_count() as u64;
    out.report.counts.passes = passes.len() as u64;
    out.report.counts.motions = out.motions.len() as u64;
    out.report.finish();
    Ok(out)
}

/// Shared validation of a roughing operation's tolerance object.
pub fn check_tolerance(t: &Tolerance) -> Result<(), CamError> {
    t.validate()?;
    Ok(())
}
