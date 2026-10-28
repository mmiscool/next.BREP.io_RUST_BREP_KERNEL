//! Z levels and per-level regions for 3D roughing (rows 3 to 5): depth
//! levels with flat-area detection and fine step-downs, the part shadow of a
//! slab, stock slices from the setup stock or the simulated dexel stock, the
//! rest material a larger reference tool leaves, and the tool-centre feasible
//! set with containment. All regions are in machine XY at the level's Z.

use super::dilate::{checked, circle_loop, grow, merge_pieces, shrink};
use crate::model::{Containment, ContainmentMode, Depths, Point2, Point3};
use crate::planar::extract::{chain_directed, section_at_z};
use crate::planar::{Loop, Region, Segment, Vec2};
use crate::stock::ResolvedStock;
use crate::stock_sim::DexelGrid;
use crate::{CamError, MeshSnapshot, Runner, Tolerance};

fn invalid(message: impl std::fmt::Display) -> CamError {
    CamError::InvalidInput(format!("adaptive levels: {message}"))
}

/// The Z levels of a roughing operation, descending, and the flats found.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelPlan {
    pub levels: Vec<f64>,
    /// Machine Z of the upward flat faces found between bottom and top.
    pub flats: Vec<f64>,
}

/// Depth levels from `top` to `bottom` (`planar::machining::depth_levels`),
/// plus one level exactly `axial` above every upward horizontal face of the
/// target bodies within the range (flat-area detection), and, when
/// `fine_step` is positive, intermediate levels no farther apart than it
/// between a flat's level and the main level above it. `include(face)`
/// selects the target faces.
pub fn plan_levels<F: Fn(u32) -> bool>(mesh: &MeshSnapshot, include: F, top: f64, bottom: f64, depths: &Depths, fine_step: f64, axial: f64, tol: &Tolerance) -> Result<LevelPlan, CamError> {
    let mut levels = crate::planar::machining::depth_levels(top, bottom, depths, "adaptive")?;
    let mut flats: Vec<f64> = Vec::new();
    for t in &mesh.triangles {
        if !include(t.face) || !t.is_horizontal(tol) || t.normal.z <= 0.0 {
            continue;
        }
        let z = t.a.z;
        if z <= bottom + tol.distance_eps || z >= top - tol.distance_eps {
            continue;
        }
        if !flats.iter().any(|f| (f - z).abs() <= tol.distance_eps) {
            flats.push(z);
        }
    }
    flats.sort_by(|a, b| b.total_cmp(a));
    for &f in &flats {
        let level = f + axial;
        if level >= top - tol.distance_eps || level <= bottom + tol.distance_eps {
            continue;
        }
        // Levels landing within the axial allowance above a flat would cut
        // into it: snap them up to the flat's level.
        levels.retain(|l| !(*l > level - tol.distance_eps && *l < level + axial + tol.distance_eps) || (*l - level).abs() <= tol.distance_eps);
        if !levels.iter().any(|l| (l - level).abs() <= tol.distance_eps) {
            levels.push(level);
        }
        if fine_step > 0.0 {
            levels.sort_by(|a, b| b.total_cmp(a));
            if let Some(&above) = levels.iter().filter(|l| **l > level + tol.distance_eps).min_by(|a, b| a.total_cmp(b)) {
                let span = above - level;
                let n = (span / fine_step - 1e-9).ceil().max(1.0) as usize;
                if n > 100_000 {
                    return Err(invalid("fine step-down level budget exceeded"));
                }
                for i in 1..n {
                    let z = above - span * i as f64 / n as f64;
                    if !levels.iter().any(|l| (l - z).abs() <= tol.distance_eps) {
                        levels.push(z);
                    }
                }
            }
        }
    }
    levels.sort_by(|a, b| b.total_cmp(a));
    levels.dedup_by(|a, b| (*a - *b).abs() <= tol.distance_eps);
    Ok(LevelPlan { levels, flats })
}

/// A section with collinear consecutive pieces merged (a refined mesh cuts a
/// flat wall into hundreds of collinear slivers).
pub fn merged_section<F: Fn(u32) -> bool>(mesh: &MeshSnapshot, z: f64, include: F, tol: &Tolerance) -> Result<Region, CamError> {
    let s = section_at_z(mesh, z, include, tol).map_err(|e| CamError::InvalidInput(format!("the part's section at z = {z}: {e}")))?;
    let loops: Vec<Loop> = s.loops.iter().map(|l| Loop::new(merge_pieces(&l.segments, tol))).filter(|l| l.segments.len() >= 2).collect();
    let region = Region { loops };
    // A section with negative total area is a cut exactly at a face's plane
    // (some walls ending there crossed by rounding, others did not); it says
    // nothing reliable about that height, and the callers sample around it.
    if region.area() < -tol.area_eps {
        return Ok(Region::empty());
    }
    Ok(region)
}

/// The XY projections of the pieces of the mesh's downward-facing triangles
/// between heights `z_lo` and `z_hi` (counter-clockwise convex polygons;
/// slivers below the area epsilon are left out). Material above a level
/// that is not over the level's own section rests on a downward-facing
/// surface, so the level's section together with these projections over
/// everything above it is the exact shadow; the sampled sections between
/// levels miss what leans out between two samples (a turbine blade's
/// leaning flank, refused by the pass certificate as a 1.13 mm gouge).
pub fn overhang_pieces(mesh: &MeshSnapshot, z_lo: f64, z_hi: f64, tol: &Tolerance) -> Vec<Loop> {
    let mut out = Vec::new();
    for t in &mesh.triangles {
        if t.normal.z >= 0.0 || t.max_z() <= z_lo || t.min_z() >= z_hi {
            continue;
        }
        // Clip the triangle to the slab (Sutherland-Hodgman, two planes).
        let mut poly: Vec<Point3> = t.vertices().to_vec();
        for (h, keep_above) in [(z_lo, true), (z_hi, false)] {
            if !h.is_finite() {
                continue;
            }
            let inside = |p: &Point3| if keep_above { p.z >= h } else { p.z <= h };
            let mut next = Vec::with_capacity(poly.len() + 2);
            for k in 0..poly.len() {
                let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
                if inside(&a) {
                    next.push(a);
                }
                if inside(&a) != inside(&b) {
                    next.push(a.lerp(b, (h - a.z) / (b.z - a.z)));
                }
            }
            poly = next;
            if poly.len() < 3 {
                break;
            }
        }
        let mut pts: Vec<Point2> = Vec::with_capacity(poly.len());
        for q in &poly {
            let q = Point2::new(q.x, q.y);
            if pts.last().map_or(true, |l: &Point2| l.distance(q) > tol.point_eps) {
                pts.push(q);
            }
        }
        while pts.len() > 1 && pts[0].distance(pts[pts.len() - 1]) <= tol.point_eps {
            pts.pop();
        }
        if pts.len() < 3 {
            continue;
        }
        let mut l = Loop::polygon(&pts);
        if !l.is_ccw() {
            l = l.reversed();
        }
        if l.signed_area() > tol.area_eps {
            out.push(l);
        }
    }
    out
}

/// The part's shadow over the slab `[z_cut, z_top]`: the union of the
/// sections of the protected mesh at `z_cut - axial` (so a cut at `z_cut`
/// stays `axial` above sloped faces), at `z_cut`, and at three heights up to
/// `z_top`. A bulge between the sampled heights is wider than every sample;
/// the exact certificate of the passes catches it as an error.
pub fn part_shadow<F: Fn(u32) -> bool + Copy>(mesh: &MeshSnapshot, include: F, z_cut: f64, z_top: f64, axial: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let mut heights = vec![z_cut - axial, z_cut];
    let span = (z_top - z_cut).max(0.0);
    for k in 1..=3 {
        heights.push(z_cut + span * k as f64 / 3.0);
    }
    let mut out = Region::empty();
    for z in heights {
        if z < mesh.bounds.min.z - tol.distance_eps || z > mesh.bounds.max.z + tol.distance_eps {
            continue;
        }
        let section = merged_section(mesh, z, include, tol)?;
        if section.is_empty() {
            continue;
        }
        out = if out.is_empty() { section } else { out.union(&section, tol)? };
    }
    Ok(out)
}

/// The region of the stock at height `z` (an empty region above or below it).
pub fn stock_slice(stock: &ResolvedStock, rest: Option<&DexelGrid>, z_lo: f64, z_hi: f64, tol: &Tolerance) -> Result<Region, CamError> {
    match stock {
        ResolvedStock::Box(b) => {
            let bounds = b.machine_bounds();
            if z_hi < bounds.min.z - tol.distance_eps || z_lo > bounds.max.z + tol.distance_eps {
                return Ok(Region::empty());
            }
            // Footprint: the convex hull of the eight transformed corners.
            let mut pts: Vec<Point2> = Vec::new();
            for &x in &[b.bounds.min.x, b.bounds.max.x] {
                for &y in &[b.bounds.min.y, b.bounds.max.y] {
                    for &zz in &[b.bounds.min.z, b.bounds.max.z] {
                        let p = b.to_machine.apply_point(Point3::new(x, y, zz));
                        pts.push(Point2::new(p.x, p.y));
                    }
                }
            }
            let hull = convex_hull(&pts, tol);
            if hull.len() < 3 {
                return Err(invalid("degenerate stock footprint"));
            }
            Ok(Region::new(vec![Loop::polygon(&hull)], tol)?)
        }
        ResolvedStock::Cylinder(c) => {
            let bounds = c.machine_bounds();
            if z_hi < bounds.min.z - tol.distance_eps || z_lo > bounds.max.z + tol.distance_eps {
                return Ok(Region::empty());
            }
            if (c.axis.z.abs() - 1.0).abs() > tol.angle_eps {
                return Err(invalid("a tilted cylinder stock has no constant slice; use a stock body"));
            }
            Ok(Region::new(vec![circle_loop(Point2::new(c.base.x, c.base.y), c.radius, true)], tol)?)
        }
        ResolvedStock::Mesh(m) => {
            // The widest of the slab's sections, as the shadow does.
            let mut out = Region::empty();
            for k in 0..=2 {
                let z = z_lo + (z_hi - z_lo) * k as f64 / 2.0;
                if z < m.bounds.min.z - tol.distance_eps || z > m.bounds.max.z + tol.distance_eps {
                    continue;
                }
                let s = merged_section(m, z, |_| true, tol)?;
                if !s.is_empty() {
                    out = if out.is_empty() { s } else { out.union(&s, tol)? };
                }
            }
            Ok(out)
        }
        ResolvedStock::Rest(_) => {
            let grid = rest.ok_or_else(|| invalid("rest stock needs the previous setup's simulated stock"))?;
            grid_slab_region(grid, z_lo, z_hi, tol)
        }
    }
}

/// Convex hull (monotone chain), counter-clockwise, duplicates removed.
pub fn convex_hull(points: &[Point2], tol: &Tolerance) -> Vec<Point2> {
    let mut pts: Vec<Point2> = points.to_vec();
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup_by(|a, b| a.distance(*b) <= tol.point_eps);
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: Point2, a: Point2, b: Point2| (a.sub(o)).cross(b.sub(o));
    let mut lower: Vec<Point2> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= tol.area_eps {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<Point2> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= tol.area_eps {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// The columns of a dexel grid holding material anywhere in `[z_lo, z_hi]`,
/// inflated by one cell (a partially filled neighbour counts as material, so
/// the region never falls short of the stock), as the exact polygon of those
/// cells: once-used cell edges chained with the material on the left.
pub fn grid_slab_region(grid: &DexelGrid, z_lo: f64, z_hi: f64, tol: &Tolerance) -> Result<Region, CamError> {
    let (nx, ny) = (grid.nx as usize, grid.ny as usize);
    if nx == 0 || ny == 0 || grid.columns.len() != nx * ny {
        return Err(invalid("invalid dexel grid layout"));
    }
    let raw: Vec<bool> = grid.columns.iter().map(|c| c.has_material(z_lo, z_hi)).collect();
    let filled = |ix: isize, iy: isize| -> bool {
        for dy in -1..=1isize {
            for dx in -1..=1isize {
                let (x, y) = (ix + dx, iy + dy);
                if x >= 0 && y >= 0 && x < nx as isize && y < ny as isize && raw[y as usize * nx + x as usize] {
                    return true;
                }
            }
        }
        false
    };
    let corner = |ix: isize, iy: isize| Point2::new(grid.origin.x + ix as f64 * grid.cell_x, grid.origin.y + iy as f64 * grid.cell_y);
    let mut segments: Vec<Segment> = Vec::new();
    for iy in -1..=ny as isize {
        for ix in -1..=nx as isize {
            if !filled(ix, iy) {
                continue;
            }
            // Material on the left: bottom edge +x, right edge +y, top edge -x, left edge -y.
            if !filled(ix, iy - 1) {
                segments.push(Segment::line(corner(ix, iy), corner(ix + 1, iy)));
            }
            if !filled(ix + 1, iy) {
                segments.push(Segment::line(corner(ix + 1, iy), corner(ix + 1, iy + 1)));
            }
            if !filled(ix, iy + 1) {
                segments.push(Segment::line(corner(ix + 1, iy + 1), corner(ix, iy + 1)));
            }
            if !filled(ix - 1, iy) {
                segments.push(Segment::line(corner(ix, iy + 1), corner(ix, iy)));
            }
        }
    }
    if segments.is_empty() {
        return Ok(Region::empty());
    }
    let loops = chain_directed(segments, tol)?;
    let merged: Vec<Loop> = loops.iter().map(|l| Loop::new(merge_pieces(&l.segments, tol))).collect();
    Ok(Region::new(merged, tol)?)
}

/// What a tool of radius `reference` cannot remove from the material
/// `material` of a level: the material minus everything within `reference`
/// of the reference tool's own feasible set (`feasible_ref`, built like the
/// current tool's with the reference radius). The inside corners and narrows.
pub fn reference_tool_rest(material: &Region, feasible_ref: &Region, reference: f64, chord_tol: f64, tol: &Tolerance) -> Result<Region, CamError> {
    if feasible_ref.is_empty() {
        return Ok(material.clone());
    }
    let reach = grow(feasible_ref, reference, chord_tol, tol)?;
    Ok(material.difference(&reach, tol)?)
}

/// The tool-centre feasible set of a level: the stock slice grown by
/// `r + load + margin` (so the first front sits in the air around the stock)
/// minus the part shadow grown by `r + allowance`, intersected with the
/// containment boundary when given (tool centre inside, tool inside, or tool
/// outside).
#[allow(clippy::too_many_arguments)]
pub fn feasible_set(stock: &Region, shadow: &Region, r: f64, load: f64, allowance: f64, chord_tol: f64, containment: Option<&Containment>, tol: &Tolerance) -> Result<Region, CamError> {
    let mut t = grow(stock, r + load, chord_tol, tol)?;
    if !shadow.is_empty() {
        let keep_off = grow(shadow, r + allowance, chord_tol, tol)?;
        t = checked("feasible = grown stock - grown shadow", &t, &keep_off, || t.difference(&keep_off, tol))?;
    }
    if let Some(c) = containment {
        if !c.boundary.is_empty() {
            let loops: Vec<Loop> = c.boundary.iter().filter(|l| l.len() >= 3).map(|l| Loop::polygon(l)).collect();
            let mut bound = Region::from_unoriented(loops, tol)?;
            let d = match c.mode {
                ContainmentMode::ToolCenterInside => c.offset,
                ContainmentMode::ToolInside => c.offset - r,
                ContainmentMode::ToolOutside => c.offset + r,
            };
            if d > tol.distance_eps {
                bound = grow(&bound, d, chord_tol, tol)?;
            } else if d < -tol.distance_eps {
                bound = shrink(&bound, -d, chord_tol, tol)?;
            }
            t = checked("feasible & containment", &t, &bound, || t.intersection(&bound, tol))?;
        }
    }
    Ok(t)
}

/// Contour-parallel loops of a feasible set at `stepover`, outermost first.
/// Ring `k` is the set shrunk by `k·stepover` from the set itself, so the
/// rings are independent: with the `parallel` feature they are shrunk a wave
/// of pool-width at a time, and the rings past the first empty one are
/// discarded unread (errors too), so the result is the serial one.
pub fn offset_rings(feasible: &Region, stepover: f64, chord_tol: f64, tol: &Tolerance, strategy: &str, runner: &mut Runner<'_>) -> Result<Vec<Region>, CamError> {
    const BUDGET: usize = 100_000;
    let mut out = vec![feasible.clone()];
    #[cfg(feature = "parallel")]
    let wave = crate::par::threads();
    #[cfg(not(feature = "parallel"))]
    let wave = 1;
    let mut k = 1usize;
    while k < BUDGET {
        let ks: Vec<usize> = (k..(k + wave).min(BUDGET)).collect();
        let one = |&k: &usize, _: &mut Runner<'_>| shrink(feasible, k as f64 * stepover, chord_tol, tol);
        #[cfg(feature = "parallel")]
        let shrunk = crate::par::map_ordered(&ks, runner, "offset-rings", one)?;
        #[cfg(not(feature = "parallel"))]
        let shrunk: Vec<_> = ks.iter().map(|k| one(k, runner)).collect();
        for r in shrunk {
            let r = r?;
            if r.is_empty() {
                return Ok(out);
            }
            out.push(r);
        }
        k += ks.len();
    }
    Err(crate::planar::machining::invalid(strategy, "offset ring budget exceeded"))
}

/// Each level's `(feasible, material)` from its `(z, slab top, stock, shadow)`
/// (see `resolve_levels`): the levels are independent, so with the `parallel`
/// feature they are resolved on the pool, in level order.
#[allow(clippy::too_many_arguments)]
pub fn resolve_slabs(slabs: &[(f64, f64, Region, Region)], r: f64, load: f64, radial: f64, margin: f64, chord_tol: f64, containment: &Containment, tol: &Tolerance, runner: &mut Runner<'_>) -> Result<Vec<Result<(Region, Region), CamError>>, CamError> {
    let one = |(_, _, stock_region, shadow): &(f64, f64, Region, Region), _: &mut Runner<'_>| -> Result<(Region, Region), CamError> {
        let feasible = if stock_region.is_empty() { Region::empty() } else { feasible_set(stock_region, shadow, r, load, radial + margin, chord_tol, Some(containment), tol)? };
        let material = if stock_region.is_empty() || shadow.is_empty() {
            stock_region.clone()
        } else {
            let keep = grow(shadow, radial, chord_tol, tol)?;
            checked("material = stock - grown shadow", stock_region, &keep, || stock_region.difference(&keep, tol))?
        };
        Ok((feasible, material))
    };
    #[cfg(feature = "parallel")]
    let resolved = crate::par::map_ordered(slabs, runner, "roughing-levels", one)?;
    #[cfg(not(feature = "parallel"))]
    let resolved: Vec<_> = slabs.iter().map(|s| one(s, runner)).collect();
    Ok(resolved)
}
