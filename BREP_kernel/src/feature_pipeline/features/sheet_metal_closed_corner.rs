//! Close the free ends of two perpendicular sibling flanges. Edits live in the
//! sheet tree, so replay, subsequent sheet features and unfolding retain them.
//! Bend sectors are preserved; reliefs cut the wall ends at the bend tangency.
use super::common;
use crate::feature_pipeline::sheet_metal::{self, Flat, Hole, PlaneCut, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult, SelectionProbe};
use crate::{make_arc, make_line, Vec3};
use std::f64::consts::{PI, TAU};

const EPS: f64 = 1e-7;

pub fn context_applicable(probe: &SelectionProbe) -> bool {
    probe.all_sheet_metal && probe.faces == 2 && probe.edges == 0
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(format!("sheet-metal closed corner: {error}")),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // `bends` (the rounded bend faces, NX style) is the dialog's selection. Documents
    // saved before it picked the flanges' broad faces into `faces`; that field stays
    // a hidden compatibility read and accepts either kind.
    let bend_picks = common::reference_names(ctx.param("bends"));
    let legacy = bend_picks.is_empty();
    let picks = if legacy {
        common::reference_names(ctx.param("faces"))
    } else {
        bend_picks
    };
    if picks.len() != 2 {
        return Err("select the two bend faces that meet at the corner".into());
    }
    let mut handle = None;
    let mut names = Vec::new();
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    for pick in &picks {
        let Some(face) = ctx.scene.resolve_face(pick) else {
            result.unresolved.push(pick.clone());
            continue;
        };
        if handle.is_some_and(|h| h != face.handle) {
            return Err("both bends must belong to the same sheet-metal body".into());
        }
        handle = Some(face.handle);
        // Drop the face index and a mirrored part's `::{feature}` namespaces.
        let name = pick.split("::").next().unwrap_or(pick);
        let name = name.split('[').next().unwrap_or(name);
        let id = name
            .strip_suffix(":A")
            .or_else(|| name.strip_suffix(":B"))
            .ok_or("select the bend faces, not side faces or bend ends")?;
        names.push(id.to_string());
    }
    if !result.unresolved.is_empty() {
        return Ok(result);
    }
    let handle = handle.ok_or("missing sheet-metal body")?;
    let mut tree = sheet_metal::get_tree(handle).ok_or("selection is not sheet metal")?;
    // Each bend names the wall it carries; a legacy pick may already name the wall.
    let mut ids = Vec::new();
    for name in &names {
        match bend_child(&tree.root, name) {
            Some(child) => ids.push(child),
            None if legacy && tree.root.find_by_id(name).is_some() => ids.push(name.clone()),
            None => return Err("select the two bend faces that meet at the corner, not the flat faces".into()),
        }
    }
    let body = ctx
        .scene
        .solids
        .iter()
        .find(|(_, h)| **h == handle)
        .map(|(name, _)| name.clone())
        .ok_or("body is no longer in the scene")?;
    let treatment = ctx
        .param("treatment")
        .and_then(|v| v.as_str())
        .unwrap_or("U Cutout");
    let centres = match ctx
        .param("reliefOrigin")
        .and_then(|v| v.as_str())
        .unwrap_or("Bend tangent")
    {
        "Bend tangent" => false,
        "Bend centers" => true,
        _ => return Err("unknown relief origin".into()),
    };
    let overlap = ctx.param("overlap").and_then(|v| v.as_str()).unwrap_or("First over second");
    if !matches!(overlap, "First over second" | "Second over first" | "Mitre (weld gap)") {
        return Err("unknown overlap option".into());
    }
    let options = Options {
        treatment: treatment.to_string(),
        centres,
        reverse: overlap == "Second over first",
        mitre: overlap == "Mitre (weld gap)",
        gap: number(ctx, "gap", 0.1)?,
        size: if treatment == "Closed" || (treatment == "V Cutout" && !centres) {
            0.0
        } else {
            number(ctx, "cutoutSize", 2.0)?
        },
        depth: if matches!(treatment, "U Cutout" | "V Cutout") {
            number(ctx, "cutoutDepth", 2.0)?
        } else {
            0.0
        },
        v_angles: if treatment == "V Cutout" {
            [number(ctx, "vAngleFirst", 45.0)?, number(ctx, "vAngleSecond", 45.0)?]
        } else {
            [45.0; 2]
        },
        v_lengths: if treatment == "V Cutout" && centres {
            [number(ctx, "vLengthFirst", 3.0)?, number(ctx, "vLengthSecond", 3.0)?]
        } else {
            [0.0; 2]
        },
    };
    apply(&mut tree, [&ids[0], &ids[1]], &options)?;
    result
        .added
        .push(sheet_metal::register_folded(tree, &body)?);
    result.removed.push(body);
    Ok(result)
}

fn number(ctx: &FeatureContext, name: &str, default: f64) -> Result<f64, String> {
    let value = if ctx.param(name).is_some() {
        ctx.number(name)?
    } else {
        default
    };
    if !value.is_finite() || value < 0.0 {
        return Err(format!("{name} must be finite and nonnegative"));
    }
    Ok(value)
}

struct Options {
    treatment: String,
    /// Neither wall overlaps: both are cut on the mitre (bisector) plane, each
    /// `gap / 2` short of it, leaving an even weld gap between the two faces.
    mitre: bool,
    /// NX "Bend Centers": U/V reliefs centred where the bend centrelines
    /// cross, cutting the base plate, both bend zones and both walls.
    centres: bool,
    reverse: bool,
    gap: f64,
    size: f64,
    depth: f64,
    v_angles: [f64; 2],
    v_lengths: [f64; 2],
}

/// The id of the wall reached across the bend `bend_id`, searching nested walls.
fn bend_child(flat: &Flat, bend_id: &str) -> Option<String> {
    flat.edges.iter().filter_map(|e| e.bend.as_ref()).find_map(|bend| {
        if bend.id == bend_id {
            Some(bend.child.id.clone())
        } else {
            bend_child(&bend.child, bend_id)
        }
    })
}

/// Locate sibling hinges that actually share an endpoint, including nested walls.
fn pair<'a>(flat: &'a Flat, ids: [&str; 2]) -> Option<(&'a Flat, usize, usize)> {
    let a = flat
        .edges
        .iter()
        .position(|e| e.bend.as_ref().is_some_and(|b| b.child.id == ids[0]));
    let b = flat
        .edges
        .iter()
        .position(|e| e.bend.as_ref().is_some_and(|b| b.child.id == ids[1]));
    if let (Some(a), Some(b)) = (a, b) {
        return Some((flat, a, b));
    }
    for edge in &flat.edges {
        if let Some(bend) = &edge.bend {
            if let Some(found) = pair(&bend.child, ids) {
                return Some(found);
            }
        }
    }
    None
}

fn apply(tree: &mut SheetTree, ids: [&str; 2], options: &Options) -> Result<(), String> {
    if ids[0] == ids[1] {
        return Err("select two different flanges".into());
    }
    if !matches!(
        options.treatment.as_str(),
        "Closed" | "Circular Cutout" | "U Cutout" | "V Cutout"
    ) {
        return Err("unknown corner treatment".into());
    }
    if !options.gap.is_finite() || options.gap < 0.0 {
        return Err("gap must be finite and nonnegative".into());
    }
    let (parent, a, b) =
        pair(&tree.root, ids).ok_or("flanges must share a parent flat and a corner")?;
    let n = parent.outline.len();
    let (p, q) = if (a + 1) % n == b {
        (a, b)
    } else if (b + 1) % n == a {
        (b, a)
    } else {
        return Err("the selected hinges do not share a corner".into());
    };
    let point = |i: usize| parent.outline[i % n];
    let d = |i: usize| {
        let (a, b) = (point(i), point(i + 1));
        [b[0] - a[0], b[1] - a[1]]
    };
    let (dp, dq) = (d(p), d(q));
    let (lp, lq) = (dp[0].hypot(dp[1]), dq[0].hypot(dq[1]));
    if lp <= EPS
        || lq <= EPS
        || (dp[0] * dq[0] + dp[1] * dq[1]).abs() > EPS * lp * lq
        || dp[0] * dq[1] - dp[1] * dq[0] <= 0.0
    {
        return Err("only convex perpendicular corners are supported".into());
    }
    let bp = parent.edges[p].bend.as_ref().unwrap();
    let bq = parent.edges[q].bend.as_ref().unwrap();
    if bp.angle_deg * bq.angle_deg <= 0.0
        || bp.angle_deg.abs() >= 180.0 - EPS
        || bq.angle_deg.abs() >= 180.0 - EPS
    {
        return Err("both bends must fold the same way, by less than 180 degrees".into());
    }
    // Square 90/90 overlap corners keep their original construction (saved
    // documents rebuild unchanged); any other angle, and the mitre, trim each
    // wall by a plane through the other wall's geometry.
    let square = (bp.angle_deg.abs() - 90.0).abs() <= EPS && (bp.angle_deg - bq.angle_deg).abs() <= EPS;
    let general = options.mitre || !square || FORCE_PLANE_CUTS.with(|force| force.get());
    let through = if options.reverse { ids[1] } else { ids[0] };
    let corner = Corner {
        parent: parent.id.clone(),
        a_p: point(p),
        c: point(q),
        t_p: [dp[0] / lp, dp[1] / lp],
        t_q: [dq[0] / lq, dq[1] / lq],
        spans: [lp, lq],
        bends: [bp.id.clone(), bq.id.clone()],
        walls: [bp.child.id.clone(), bq.child.id.clone()],
        allowances: [bp.allowance(tree.thickness), bq.allowance(tree.thickness)],
        p_first: bp.child.id == ids[0],
    };
    let (bp_child, bq_child) = (bp.child.id.clone(), bq.child.id.clone());
    let edits = [
        (bp.child.id.clone(), false, lp, bq.inside_radius),
        (bq.child.id.clone(), true, lq, bp.inside_radius),
    ];
    let thickness = tree.thickness;
    let cuts = if general {
        Some(corner_planes(tree, [&bp_child, &bq_child], through, options)?)
    } else {
        None
    };
    for (id, start, span, radius) in edits {
        let flat = tree.root.find_by_id_mut(&id).unwrap();
        // Only plain rectangular walls can be extended by moving one end. The
        // OTHER end may already be closed (and relieved) by an earlier corner,
        // so earlier holes stay and only this end must be untreated.
        if flat.outline.len() != 4
            || flat.edges.len() != 4
            || !flat.hole_bends.is_empty()
            || !flat.outline_curves.is_empty()
            || flat.edges.iter().any(|e| e.bend.is_some())
        {
            return Err("use plain rectangular flanges before adding further bends".into());
        }
        let leg = flat.outline[1][0];
        let o = &flat.outline;
        let square = leg > EPS
            && o[0][0].abs() <= EPS
            && (o[1][0] - leg).abs() <= EPS
            && (o[2][0] - leg).abs() <= EPS
            && o[3][0].abs() <= EPS
            && (o[0][1] - o[1][1]).abs() <= EPS
            && (o[2][1] - o[3][1]).abs() <= EPS
            && o[0][1] < o[2][1];
        if !square {
            return Err("flanges must have square rectangular ends".into());
        }
        let untouched = if start { o[0][1].abs() <= EPS } else { (o[2][1] - span).abs() <= EPS };
        if !untouched {
            return Err("this flange end already has a closed corner".into());
        }
        let end = if let Some(cuts) = &cuts {
            // Extend past the trimming plane at every corner of the wall's
            // section, then let the plane cut the end; reliefs sit where the
            // plane crosses the hinge line on the midplane.
            let cut = &cuts[usize::from(id != bp_child)];
            let [px, py, pz] = cut.point;
            let [nx, ny, nz] = cut.normal;
            if ny.abs() <= 1e-9 {
                return Err("the two walls never meet".into());
            }
            let y_at = |x: f64, z: f64| py - (nx * (x - px) + nz * (z - pz)) / ny;
            let ys = [y_at(0.0, -0.5 * thickness), y_at(0.0, 0.5 * thickness), y_at(leg, -0.5 * thickness), y_at(leg, 0.5 * thickness)];
            let margin = 0.05 * thickness;
            let reach = if start {
                ys.iter().copied().fold(f64::INFINITY, f64::min).min(0.0) - margin
            } else {
                ys.iter().copied().fold(f64::NEG_INFINITY, f64::max).max(span) + margin
            };
            let hinge = y_at(0.0, 0.0);
            if (start && hinge > 0.5 * span) || (!start && hinge < 0.5 * span) {
                return Err("the gap cuts away the whole wall end".into());
            }
            for i in if start { [0, 1] } else { [2, 3] } {
                flat.outline[i][1] = reach;
            }
            flat.end_cuts.push(cut.clone());
            hinge
        } else {
            let extension = if id == through {
                radius + thickness
            } else {
                radius - options.gap
            };
            if extension < -EPS {
                return Err("gap exceeds the available corner extension (inside bend radius)".into());
            }
            let end = if start { -extension } else { span + extension };
            for i in if start { [0, 1] } else { [2, 3] } {
                flat.outline[i][1] = end;
            }
            end
        };
        if options.treatment != "Closed" && !options.centres {
            let depth = if options.treatment == "Circular Cutout" {
                options.size * 0.5
            } else {
                options.depth
            };
            let width = if options.treatment == "V Cutout" {
                // Angles belong to selection order, independently of which wall overlaps.
                let angle = options.v_angles[usize::from(id != ids[0])];
                if !angle.is_finite() || angle <= 0.0 || angle >= 90.0 {
                    return Err("V angles must be finite and strictly between 0 and 90 degrees".into());
                }
                depth * angle.to_radians().tan()
            } else {
                options.size * 0.5
            };
            if !depth.is_finite()
                || !width.is_finite()
                || depth <= EPS
                || width <= EPS
                || depth >= leg - EPS
                || width >= span - EPS
            {
                return Err(
                    "cutout dimensions must be positive and smaller than the flange leg and span"
                        .into(),
                );
            }
            // A cut centered on the wall's bend-tangent corner opens to both free
            // boundaries. Exact arcs remain exact through fold and flat export.
            let at = |x: f64, y: f64| Vec3::new(x, end + y, 0.0);
            let curves = match options.treatment.as_str() {
                "Circular Cutout" => vec![make_arc(
                    at(0.0, 0.0),
                    Vec3::new(1.0, 0.0, 0.0),
                    Vec3::new(0.0, 1.0, 0.0),
                    width,
                    0.0,
                    TAU,
                )?],
                "U Cutout" => {
                    if depth < width {
                        return Err("U cutout depth must be at least half its width".into());
                    }
                    vec![
                        make_line(at(-width, -width), at(depth - width, -width))?,
                        make_arc(
                            at(depth - width, 0.0),
                            Vec3::new(1.0, 0.0, 0.0),
                            Vec3::new(0.0, 1.0, 0.0),
                            width,
                            -PI / 2.0,
                            PI / 2.0,
                        )?,
                        make_line(at(depth - width, width), at(-width, width))?,
                        make_line(at(-width, width), at(-width, -width))?,
                    ]
                }
                _ => vec![
                    make_line(at(-depth, -2.0 * width), at(depth, 0.0))?,
                    make_line(at(depth, 0.0), at(-depth, 2.0 * width))?,
                    make_line(at(-depth, 2.0 * width), at(-depth, -2.0 * width))?,
                ],
            };
            flat.holes.push(Hole::through(curves));
        }
    }
    if options.centres {
        centre_reliefs(tree, &corner, options)?;
    }
    Ok(())
}

thread_local! {
    /// Tests: build square overlap corners through the plane cuts too, so the
    /// two constructions can be compared on the same corner.
    static FORCE_PLANE_CUTS: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// The trimming plane of each wall (`walls` = [p side, q side]) in that wall's
/// own frame, from the folded placements. Overlap: the through wall stops on
/// the other wall's far face, the butting wall `gap` short of the through
/// wall's near face. Mitre: both stop `gap / 2` short of the plane that bisects
/// the two walls.
fn corner_planes(tree: &SheetTree, walls: [&str; 2], through: &str, options: &Options) -> Result<[PlaneCut; 2], String> {
    use crate::Vec3;
    let placements = sheet_metal::evaluate::folded_flat_placements(tree)?;
    let find = |id: &str| placements.iter().find(|p| p.id == id).ok_or_else(|| format!("wall `{id}` has no placement"));
    let (wp, wq) = (find(walls[0])?, find(walls[1])?);
    let flat = |id: &str| tree.root.find_by_id(id).ok_or_else(|| format!("wall `{id}` is gone"));
    let centre = |placement: &sheet_metal::evaluate::FlatPlacement, f: &Flat| {
        let (mut x, mut y) = (0.0, 0.0);
        for p in &f.outline {
            x += p[0];
            y += p[1];
        }
        let k = f.outline.len() as f64;
        placement.origin.add(placement.u.scale(x / k)).add(placement.v.scale(y / k))
    };
    let (cp, cq) = (centre(wp, flat(walls[0])?), centre(wq, flat(walls[1])?));
    // a_P points from P's midplane toward Q, a_Q from Q's toward P.
    let toward = |from: &sheet_metal::evaluate::FlatPlacement, other: Vec3| {
        if other.sub(from.origin).dot(from.w) >= 0.0 { from.w } else { from.w.scale(-1.0) }
    };
    let (ap, aq) = (toward(wp, cq), toward(wq, cp));
    let half = 0.5 * tree.thickness;
    let g = options.gap;
    let world: [(Vec3, Vec3); 2] = if options.mitre {
        // f(x) = (x - oQ)·aQ - (x - oP)·aP: positive in P, negative in Q.
        let grad = aq.sub(ap);
        let m = grad.length();
        if m <= 1e-9 {
            return Err("the two walls are parallel".into());
        }
        let e = grad.scale(1.0 / m);
        let f = |x: Vec3| x.sub(wq.origin).dot(aq) - x.sub(wp.origin).dot(ap);
        let on = |level: f64| wp.origin.add(e.scale((level - f(wp.origin)) / m));
        [(on(0.5 * g * m), e.scale(-1.0)), (on(-0.5 * g * m), e)]
    } else {
        let far_face_of_q = (wq.origin.sub(aq.scale(half)), aq.scale(-1.0));
        let near_face_of_p = (wp.origin.add(ap.scale(half + g)), ap.scale(-1.0));
        let far_face_of_p = (wp.origin.sub(ap.scale(half)), ap.scale(-1.0));
        let near_face_of_q = (wq.origin.add(aq.scale(half + g)), aq.scale(-1.0));
        if through == walls[0] {
            [far_face_of_q, near_face_of_p]
        } else {
            [near_face_of_q, far_face_of_p]
        }
    };
    let local = |placement: &sheet_metal::evaluate::FlatPlacement, (point, normal): (Vec3, Vec3)| {
        let d = point.sub(placement.origin);
        PlaneCut {
            point: [d.dot(placement.u), d.dot(placement.v), d.dot(placement.w)],
            normal: [normal.dot(placement.u), normal.dot(placement.v), normal.dot(placement.w)],
        }
    };
    Ok([local(wp, world[0]), local(wq, world[1])])
}

/// The corner as the relief needs it, in the parent's flat frame: hinge `p`
/// ends at the corner `c`, hinge `q` starts there.
struct Corner {
    parent: String,
    a_p: [f64; 2],
    c: [f64; 2],
    t_p: [f64; 2],
    t_q: [f64; 2],
    spans: [f64; 2],
    bends: [String; 2],
    walls: [String; 2],
    allowances: [f64; 2],
    /// The first selection is the `p` side (selection order owns the V values).
    p_first: bool,
}

/// Cut the U or V relief centred on the bend-centreline crossing through the
/// parent plate, both bend zones and both walls (owner contract 2026-10-28).
fn centre_reliefs(tree: &mut SheetTree, k: &Corner, options: &Options) -> Result<(), String> {
    let out = |t: [f64; 2]| [t[1], -t[0]];
    let (out_p, out_q) = (out(k.t_p), out(k.t_q));
    let [a_p, a_q] = k.allowances;
    let o = [
        k.c[0] + 0.5 * (a_p * out_p[0] + a_q * out_q[0]),
        k.c[1] + 0.5 * (a_p * out_p[1] + a_q * out_q[1]),
    ];
    let r = 0.5 * options.size;
    let shape = match options.treatment.as_str() {
        "U Cutout" => {
            if r <= EPS || options.depth < options.size - EPS {
                return Err("a U relief needs a positive diameter and a length at least that diameter".into());
            }
            let d = [out_p[0] + out_q[0], out_p[1] + out_q[1]];
            let norm = d[0].hypot(d[1]);
            relief::slot(o, [d[0] / norm, d[1] / norm], r, options.depth)
        }
        "V Cutout" => {
            let pick = |values: [f64; 2]| if k.p_first { values } else { [values[1], values[0]] };
            let [alpha_p, alpha_q] = pick(options.v_angles);
            let [len_p, len_q] = pick(options.v_lengths);
            if alpha_p < 0.0 || alpha_q < 0.0 || alpha_p + alpha_q >= 90.0 {
                return Err("V angles must be nonnegative and sum to less than 90 degrees".into());
            }
            if len_p <= EPS || len_q <= EPS {
                return Err("V flank lengths must be positive".into());
            }
            let rot = |v: [f64; 2], a: f64| {
                let (s, c) = a.to_radians().sin_cos();
                [v[0] * c - v[1] * s, v[0] * s + v[1] * c]
            };
            let (f_p, f_q) = (rot(k.t_p, -alpha_p), rot([-k.t_q[0], -k.t_q[1]], alpha_q));
            let e_p = [o[0] + len_p * f_p[0], o[1] + len_p * f_p[1]];
            let e_q = [o[0] + len_q * f_q[0], o[1] + len_q * f_q[1]];
            if f_p[0] * f_q[1] - f_p[1] * f_q[0] > 0.0 {
                relief::hull(o, r, e_p, e_q)?
            } else {
                relief::hull(o, r, e_q, e_p)?
            }
        }
        _ => return Err("Bend centers reliefs are U or V cutouts".into()),
    };
    let overlaps = |loop_: &[relief::Seg], lo: [f64; 2], hi: [f64; 2]| {
        let mut kept = loop_.to_vec();
        for (n, bound) in [([-1.0, 0.0], -lo[0]), ([1.0, 0.0], hi[0]), ([0.0, -1.0], -lo[1]), ([0.0, 1.0], hi[1])] {
            kept = relief::clip(&kept, n, bound);
        }
        kept
    };
    let bounds = |flat: &Flat| {
        flat.outline.iter().fold([[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]], |[lo, hi], p| {
            [[lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])]]
        })
    };
    // The parent plate, in its own frame.
    let parent = tree.root.find_by_id_mut(&k.parent).ok_or("corner flat is gone")?;
    let [lo, hi] = bounds(parent);
    if !overlaps(&shape, lo, hi).is_empty() {
        parent.holes.push(Hole::through(relief::curves(&shape)?));
    }
    // Each band (x across from the parent fold line, y along the hinge from
    // its start) and each wall (x across from the band's far side).
    let hinge_starts = [k.a_p, k.c];
    let outs = [out_p, out_q];
    for side in 0..2 {
        let band = relief::into_frame(&shape, hinge_starts[side], outs[side]);
        let allowance = k.allowances[side];
        let delta = 0.25 * allowance;
        let clipped = overlaps(&band, [-delta, -delta], [allowance + delta, k.spans[side] + delta]);
        if !clipped.is_empty() {
            let parent = tree.root.find_by_id_mut(&k.parent).ok_or("corner flat is gone")?;
            let bend = parent
                .edges
                .iter_mut()
                .filter_map(|e| e.bend.as_mut())
                .find(|b| b.id == k.bends[side])
                .ok_or("corner bend is gone")?;
            bend.reliefs.push(Hole::through(relief::curves(&clipped)?));
        }
        let start = hinge_starts[side];
        let wall_origin = [start[0] + allowance * outs[side][0], start[1] + allowance * outs[side][1]];
        let wall_shape = relief::into_frame(&shape, wall_origin, outs[side]);
        let wall = tree.root.find_by_id_mut(&k.walls[side]).ok_or("corner wall is gone")?;
        let [lo, hi] = bounds(wall);
        if !overlaps(&wall_shape, lo, hi).is_empty() {
            wall.holes.push(Hole::through(relief::curves(&wall_shape)?));
        }
    }
    Ok(())
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
        "type":"SM.CLOSED_CORNER", "shortName":"SM.CC", "longName":"SM Closed Corner",
        "ribbonPath":"Home/Sheet Metal/Closed Corner", "commandSize":"Compact", "displayBuilder":false,
        "inputParamsSchema": {
            "id":{"type":"string","default_value":null},
            "bends":{"label":"Bends","type":"reference_selection","selectionFilter":["FACE"],"multiple":true,"default_value":null,
                "hint":"Select the rounded bend faces of two adjacent flanges folded the same way (any angles below 180°). Selection order controls overlap and V angles."},
            "faces":{"type":"reference_selection","selectionFilter":["FACE"],"multiple":true,"default_value":null,
                "hint":"Flange faces picked by documents saved before bend selection; kept so they still open."},
            "treatment":{"label":"Corner treatment","type":"options","options":["U Cutout","V Cutout"],"default_value":"U Cutout"},
            "reliefOrigin":{"label":"Relief origin","type":"options","options":["Bend tangent","Bend centers"],"default_value":"Bend tangent","hint":"Bend tangent: relief cut into each wall end. Bend centers (NX): one relief centred where the bend centrelines cross, cutting the plate, both bends and both walls."},
            "overlap":{"type":"options","options":["First over second","Second over first","Mitre (weld gap)"],"default_value":"First over second","hint":"Which flange extends across the other's end, or Mitre: neither overlaps, both are cut on the bisector plane with the gap between them for a weld."},
            "gap":{"type":"number","default_value":0.1,"min":0,"hint":"Overlap: clearance between the butting end and the overlapping flange. Mitre: the weld gap between the two mitred faces."},
            "cutoutSize":{"label":"Cutout diameter / width","type":"number","default_value":2,"min":0,"hint":"Circular diameter, or U width at the bend tangent."},
            "cutoutDepth":{"label":"Cutout depth","type":"number","default_value":2,"min":0,"hint":"U/V reach along the flange from the bend tangent."},
            "vAngleFirst":{"label":"V angle on first flange","type":"number","default_value":45,"min":0,"max":90,"hint":"Angle in degrees between the first selected flange's V edge and the direction away from its bend; strictly between 0 and 90."},
            "vAngleSecond":{"label":"V angle on second flange","type":"number","default_value":45,"min":0,"max":90,"hint":"Independent V edge angle on the second selected flange; strictly between 0 and 90 degrees."},
            "vLengthFirst":{"label":"V length on first flange","type":"number","default_value":3,"min":0,"hint":"Bend centers: length of the V flank measured from the first selected bend's centreline, from the apex."},
            "vLengthSecond":{"label":"V length on second flange","type":"number","default_value":3,"min":0,"hint":"Bend centers: length of the second V flank, from the apex."}
        }
    })
}

/// The schema form displays only the dimensions used by the active treatment.
/// Saved documents may still carry the legacy flange `faces` and the Closed or
/// Circular treatments; the dialog never shows the former and offers U and V.
pub fn hidden_params(values: &serde_json::Value) -> Vec<&'static str> {
    let centres = values.get("reliefOrigin").and_then(|v| v.as_str()) == Some("Bend centers");
    let mut hidden = match values.get("treatment").and_then(|v| v.as_str()).unwrap_or("U Cutout") {
        "Closed" => vec!["cutoutSize", "cutoutDepth", "vAngleFirst", "vAngleSecond", "reliefOrigin"],
        "Circular Cutout" => vec!["cutoutDepth", "vAngleFirst", "vAngleSecond", "reliefOrigin"],
        "U Cutout" => vec!["vAngleFirst", "vAngleSecond"],
        // At the bend centres a V has a rounded apex (diameter) and a length per flank.
        "V Cutout" if centres => vec!["cutoutDepth"],
        "V Cutout" => vec!["cutoutSize"],
        _ => vec![],
    };
    if !(centres && values.get("treatment").and_then(|v| v.as_str()) == Some("V Cutout")) {
        hidden.extend(["vLengthFirst", "vLengthSecond"]);
    }
    hidden.push("faces");
    hidden
}

mod relief;

