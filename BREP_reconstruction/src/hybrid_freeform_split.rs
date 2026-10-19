//! Split a freeform segmentation region that holds several carriers.
//!
//! Lane J row 7 (2026-10-15). Two equal cylinders crossing at right angles
//! touch tangentially where their intersection curves cross, so the kernel
//! region grower joins both walls into one region no single carrier fits.
//! On an exact mesh the segmenter's precision peel separates them; once the
//! vertices carry noise above the peel's bar the region stays `Freeform` and
//! the mixed builder facets every triangle of it. Recognition run on that
//! region alone, at the segmenter's own acceptance bar, proposes the
//! carriers; the hybrid builder still validates every shared boundary and
//! demotes what it cannot trim.
use crate::{recognize_surfaces_with_attempt_budget, Mesh, RecognitionOptions, SamplingMode};
use brep_kernel::{MeshRegion, MeshSegmentation, RegionCarrier, SegmentOptions, Vec3};

/// Smallest freeform region worth a recognition pass.
const MIN_SPLIT_TRIANGLES: usize = 64;

/// Work budget of the recognition pass. Its cost grows with the hypotheses
/// it draws, steeply where nothing fits: a 1 868-triangle region jittered
/// past the bar took 4 s at 128 hypotheses, 43 s at 256 and 123 s at the
/// recognizer's default 512, and still assigned nothing; 64 hypotheses lost
/// the carriers of the 2 229-triangle fixture. With the hypotheses and
/// refinement steps capped the work is linear in the region, and a region
/// above the triangle cap is not tried at all: it stays as the segmenter
/// left it, exactly as without the split.
///
/// The hypothesis cap bounds one extraction round only: the recognizer keeps
/// extracting while any candidate reaches its minimum support, and on a
/// torus jittered past the bar every round peels a small patch (one
/// 5 430-triangle region took 224 s and assigned nothing). So the pass also
/// runs under a budget of hypotheses over all its rounds; a region whose
/// budget runs out is left as the segmenter left it. Up to the budget the
/// pass is the unbudgeted one, hypothesis for hypothesis. (Raising the
/// minimum support instead is not a filter: it also drops every feature
/// component below it, and lost the walls of five jittered crossings.)
const SPLIT_MAX_HYPOTHESES: usize = 128;

/// The pass draws hypotheses for planes and cylinders only: the split
/// proposes cylinders, and planes stay among the kinds so a flat patch is
/// not taken for a cylinder of large radius. On a jittered torus every kind
/// was drawn at 16.8 ms per hypothesis against 2.0 on a crossing, most of it
/// fitting candidates that lost; planes and cylinders alone took the
/// 5 430-triangle torus region from 70.7 s to 17.7 s and every crossing the
/// split rebuilds stayed rebuilt.
const SPLIT_KINDS: [crate::SurfaceType; 3] = [crate::SurfaceType::Plane, crate::SurfaceType::Cylinder, crate::SurfaceType::Cone];

/// Lane J row 7c, promoted ON 2026-10-16: a region row 7's plane/cylinder
/// stage leaves without two cylinders goes on to a stage that draws cones and
/// proposes by exclusive support ([`propose`]). BREP_HYBRID_SPLIT_CONES=0
/// restores row 7 alone.
fn split_cones_enabled() -> bool {
    std::env::var("BREP_HYBRID_SPLIT_CONES").map_or(true, |v| v != "0")
}

/// Lane J row 7d, promoted ON 2026-10-16: a region bordered by curved strips of the segmenter's own
/// is split when the carriers the split accepts hold every triangle of every
/// strip, and the strips join the pieces. BREP_HYBRID_SPLIT_STRIPS=0 restores
/// row 7c's decline.
fn split_strips_enabled() -> bool {
    std::env::var("BREP_HYBRID_SPLIT_STRIPS").map_or(true, |v| v != "0")
}

/// Sized from measurement, not tuned: the most hypotheses an unbudgeted
/// plane/cylinder pass drew on a region it then divided, over the 190
/// census rows the split tries, was 1 864 (the radius-10 d0.005 jitter-1.0
/// crossing); the budget is twice that, rounded up to whole rounds of
/// [`SPLIT_MAX_HYPOTHESES`]: ceil(2 * 1 864 / 128) * 128 = 3 840.
const SPLIT_MAX_ATTEMPTS: usize = 3_840;

/// The cone stage's own budget, by the same rule: the most hypotheses an
/// unbudgeted cone stage drew on a region it then divided, over the 190
/// rows, was 78 (`occt_countersunk_hole_s10_d0.001_jitter-1.0`); twice that
/// rounded up to whole rounds is 256. A crossing jittered past the bar,
/// which stage 1 leaves with fewer than two cylinders, spent 11.6 s in the
/// stage at the stage-1 budget and divides nothing.
const SPLIT_CONE_STAGE_MAX_ATTEMPTS: usize = 256;

/// The cheap pre-test. A cylinder's normals lie on the great circle normal
/// to its axis; a torus's or a sphere's cover the sphere. A region is tried
/// only when bands of [`BAND_HALF_WIDTH_DEG`] about at most four such great
/// circles hold [`BAND_MIN_COVERAGE`] of its triangle normals. Over the 190
/// rows every region the split divides held at least 0.958, every jittered
/// torus and sphere region (and the gearbox keychain's) at most 0.740; the
/// bar is the midpoint, 0.85, rounded.
const BAND_HALF_WIDTH_DEG: f64 = 10.0;
const BAND_MIN_COVERAGE: f64 = 0.85;

const SPLIT_MAX_REFINEMENT_ITERATIONS: usize = 40;
const MAX_SPLIT_TRIANGLES: usize = 32_768;

/// What the split did, for the conversion report.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct SplitReport {
    /// Largest vertex deviation from a carrier the split wrote (zero when it
    /// wrote none): the noise the mesh carries.
    pub(super) noise: f64,
    /// Freeform regions split into carrier regions.
    pub(super) split_regions: usize,
    /// Freeform regions tried and left as they were (fewer than two
    /// carriers, or under 19 in 20 triangles assigned).
    pub(super) declined_regions: usize,
    /// Freeform regions above [`MAX_SPLIT_TRIANGLES`], not tried.
    pub(super) over_budget_regions: usize,
    /// Freeform regions whose recognition ran out of [`SPLIT_MAX_ATTEMPTS`],
    /// left as they were.
    pub(super) budget_hit_regions: usize,
    /// Freeform regions the normal-band pre-test declined before recognition.
    pub(super) band_declined_regions: usize,
}

/// Regions below this size beside a split region are slivers it may absorb.
const SLIVER_TRIANGLES: usize = 8;

/// A group of touching slivers is absorbed only below this size and
/// area-weighted tilt; a cap fragment run fanned from its rim lies on the
/// wall's carrier too, but faces it at a right angle (89.2 to 89.6 degrees
/// on the jittered crossed cylinders, where a lone wall sliver reads up to
/// 78.8).
const SLIVER_GROUP_TRIANGLES: usize = 24;
const SLIVER_GROUP_TILT_DEG: f64 = 85.0;

pub(super) fn split(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    segmentation: &mut MeshSegmentation,
) -> SplitReport {
    let mut report = SplitReport::default();
    if indices.len() != segmentation.triangle_region_ids.len() * 3 {
        return report;
    }
    let candidate = |region: &MeshRegion| {
        matches!(region.carrier, RegionCarrier::Freeform) && region.triangle_count >= MIN_SPLIT_TRIANGLES
    };
    report.over_budget_regions = segmentation
        .regions
        .iter()
        .filter(|region| candidate(region) && region.triangle_count > MAX_SPLIT_TRIANGLES)
        .count();
    let candidate = |region: &MeshRegion| candidate(region) && region.triangle_count <= MAX_SPLIT_TRIANGLES;
    if !segmentation.regions.iter().any(candidate) {
        return report;
    }
    let debug = std::env::var("BREP_DEBUG_SPLIT").is_ok();
    let mut changed = false;
    let edges = {
        let mut edges = std::collections::HashMap::<(u32, u32), Vec<usize>>::new();
        for t in 0..segmentation.triangle_region_ids.len() {
            for c in 0..3 {
                let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                edges.entry((a.min(b), a.max(b))).or_default().push(t);
            }
        }
        edges
    };
    for region in segmentation.regions.clone() {
        if !candidate(&region) {
            continue;
        }
        let ids: Vec<usize> = segmentation
            .triangle_region_ids
            .iter()
            .enumerate()
            .filter_map(|(t, &r)| (r == region.id).then_some(t))
            .collect();
        // A region the segmenter already bordered with curved strips of its
        // own, too large to absorb as slivers, is left as it is: the builder
        // demotes every cylinder/cylinder edge that is neither a shared
        // ruling nor an equal-radius crossing, and the strips' carriers are
        // fitted apart from the walls the split would write. On the radius-1
        // d0.005 jitter-0.1 crossing strips of 15, 30 and 33 triangles took
        // all the split pieces with them (one triangle more faceted than
        // without the split); on every crossing the split rebuilds, its curved
        // neighbours held at most 5 triangles.
        //
        // Row 7d: the strips are collected instead, with the curved strips
        // bordering those in turn, and the region goes on when every strip is
        // smaller than the region; after the carriers are chosen each strip
        // joins them only if they hold every one of its triangles (below),
        // so no strip seam is left beside a split piece.
        let curved_neighbours = |tris: &[usize], strips: &std::collections::BTreeSet<u32>| {
            let mut found = std::collections::BTreeSet::<u32>::new();
            for &t in tris {
                for c in 0..3 {
                    let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                    for &n in &edges[&(a.min(b), a.max(b))] {
                        let r = segmentation.triangle_region_ids[n];
                        let Some(other) = segmentation.regions.get(r as usize) else { continue };
                        if r != region.id
                            && !strips.contains(&r)
                            && other.triangle_count >= SLIVER_TRIANGLES
                            && !matches!(other.carrier, RegionCarrier::Plane { .. } | RegionCarrier::Freeform)
                        {
                            found.insert(r);
                        }
                    }
                }
            }
            found
        };
        let mut strips = std::collections::BTreeSet::<u32>::new();
        let mut strip_triangles = Vec::<(u32, Vec<usize>)>::new();
        let mut frontier = curved_neighbours(&ids, &strips);
        let mut strip_declined = None;
        while let Some(r) = frontier.pop_first() {
            let count = segmentation.regions[r as usize].triangle_count;
            if !split_strips_enabled() || count >= ids.len() {
                strip_declined = Some((r, count));
                break;
            }
            strips.insert(r);
            let tris: Vec<usize> = segmentation
                .triangle_region_ids
                .iter()
                .enumerate()
                .filter_map(|(t, &s)| (s == r).then_some(t))
                .collect();
            frontier.extend(curved_neighbours(&tris, &strips));
            strip_triangles.push((r, tris));
        }
        if let Some((strip, count)) = strip_declined {
            if debug {
                eprintln!("[split] region {} tris {} borders curved region {strip} of {count}, left as segmented", region.id, ids.len());
            }
            report.declined_regions += 1;
            continue;
        }
        let normals: Vec<crate::Vec3> = ids
            .iter()
            .filter_map(|&t| {
                let p = [0, 1, 2].map(|c| {
                    let at = indices[3 * t + c] as usize * 3;
                    crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
                });
                (p[1] - p[0]).cross(p[2] - p[0]).normalized()
            })
            .collect();
        let coverage = normal_band_coverage(&normals, BAND_HALF_WIDTH_DEG.to_radians().sin());
        if debug {
            eprintln!("[split] region {} tris {} band coverage {coverage:.4}", region.id, ids.len());
        }
        if coverage < BAND_MIN_COVERAGE {
            report.band_declined_regions += 1;
            continue;
        }
        let mut remap = std::collections::HashMap::<u32, u32>::new();
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for &t in &ids {
            let mut tri = [0u32; 3];
            for c in 0..3 {
                let v = indices[3 * t + c];
                tri[c] = *remap.entry(v).or_insert_with(|| {
                    let at = v as usize * 3;
                    vertices.push(crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2]));
                    (vertices.len() - 1) as u32
                });
            }
            triangles.push(tri);
        }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for p in &vertices {
            for (axis, value) in [p.x, p.y, p.z].into_iter().enumerate() {
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
            }
        }
        let diagonal = (0..3).map(|a| (high[a] - low[a]).powi(2)).sum::<f64>().sqrt();
        let recognition = RecognitionOptions {
            distance_tolerance: options.fit_tolerance * diagonal,
            relative_tolerance: 0.0,
            normal_tolerance: options.normal_tolerance_deg.to_radians(),
            sampling: SamplingMode::Vertices,
            max_hypotheses: SPLIT_MAX_HYPOTHESES,
            max_refinement_iterations: SPLIT_MAX_REFINEMENT_ITERATIONS,
            ..RecognitionOptions::default()
        };
        let mesh = Mesh::new(vertices, triangles);
        let tolerance = recognition.distance_tolerance;
        let cos_gate = options.deflection_angle_deg.to_radians().cos();
        // Stage 1 is row 7's pass: planes and cylinders, cylinders holding a
        // tenth of the region. Only a region it leaves without two such
        // cylinders goes on (row 7c; BREP_HYBRID_SPLIT_CONES=0 turns it off) to a
        // pass that draws cones too and proposes by exclusive support, so
        // every region row 7 divides is divided exactly as before.
        let mut stages = vec![false];
        if split_cones_enabled() {
            stages.push(true);
        }
        let mut chosen = None;
        let mut out_of_budget = false;
        for cones in stages {
            let kinds = if cones { &SPLIT_KINDS[..] } else { &SPLIT_KINDS[..2] };
            let started = std::time::Instant::now();
            let cap = if cones { SPLIT_CONE_STAGE_MAX_ATTEMPTS } else { SPLIT_MAX_ATTEMPTS };
            let budgeted = recognize_surfaces_with_attempt_budget(&mesh, &recognition, cap, kinds);
            if debug {
                eprintln!(
                    "[split] region {} tris {} tol {:.3e} cones {cones} in {:.2}s work {:?}",
                    region.id,
                    ids.len(),
                    tolerance,
                    started.elapsed().as_secs_f64(),
                    budgeted.as_ref().map(|(_, work)| *work).ok()
                );
            }
            if std::env::var("BREP_SPLIT_COMPARE").is_ok() {
                compare_budget(&mesh, &recognition, &budgeted, region.id, (&ids, positions, indices, cos_gate), cones);
            }
            let Ok((result, work)) = budgeted else { break };
            if work.exhausted {
                out_of_budget = true;
                break;
            }
            let carriers = propose(&result.regions, &ids, positions, indices, tolerance, cos_gate, cones);
            if carriers.len() >= 2 {
                chosen = Some(carriers);
                break;
            }
        }
        if out_of_budget {
            report.budget_hit_regions += 1;
            continue;
        }
        let Some(mut carriers) = chosen else {
            report.declined_regions += 1;
            continue;
        };
        // One cone fitted twice (two pieces of a frustum's one wall, their
        // noisy apexes 0.023 apart at chord 0.1) is two carriers the builder
        // cannot join: their shared edge is no coaxial ring, and on the
        // radius-1 d0.1 jitter-0.1 frustum that demoted three more triangles
        // than leaving the region as it was. Such a region is left as it is.
        let cones: Vec<crate::ConeSurface> = carriers
            .iter()
            .filter_map(|(c, _)| if let crate::AnalyticSurface::Cone(cone) = c { Some(*cone) } else { None })
            .collect();
        let twin = cones.iter().enumerate().any(|(i, a)| {
            cones[i + 1..].iter().any(|b| {
                a.axis.dot(b.axis).abs() >= (0.5_f64).to_radians().cos() && (a.half_angle - b.half_angle).abs() <= (0.5_f64).to_radians()
            })
        });
        if twin {
            if debug {
                eprintln!("[split] region {} has one cone fitted twice, left as segmented", region.id);
            }
            report.declined_regions += 1;
            continue;
        }
        // Two noise-fitted cylinders of one radius crossing each other are
        // made one exact crossing: a common radius and axes through one
        // point. Fitted apart, their radii and axis offsets differ by ~1e-3
        // of the radius, the builder's section ellipses then miss the
        // tangent-point vertices by as much, and at radius 10 that failed
        // topology validation.
        if let [(crate::AnalyticSurface::Cylinder(first), _), (crate::AnalyticSurface::Cylinder(second), _)] = &mut carriers[..] {
            if (first.radius - second.radius).abs() <= tolerance && first.axis.cross(second.axis).length() > 0.1 {
                let (a1, a2) = (first.axis, second.axis);
                let w0 = first.axis_origin - second.axis_origin;
                let (bb, d, e) = (a1.dot(a2), a1.dot(w0), a2.dot(w0));
                let denominator = 1.0 - bb * bb;
                let p1 = first.axis_origin + a1 * ((bb * e - d) / denominator);
                let p2 = second.axis_origin + a2 * ((e - bb * d) / denominator);
                if (p1 - p2).length() <= tolerance {
                    let center = (p1 + p2) * 0.5;
                    let radius = 0.5 * (first.radius + second.radius);
                    first.axis_origin = center;
                    second.axis_origin = center;
                    first.radius = radius;
                    second.radius = radius;
                }
            }
        }
        // Every region triangle goes to the carrier holding all its vertices
        // closest (within the acceptance bar), its normal within the region
        // grower's dihedral gate; 19 in 20 must, or the region is left as it
        // was.
        let points = |t: usize| {
            [0, 1, 2].map(|c| {
                let at = indices[3 * t + c] as usize * 3;
                crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
            })
        };
        // Two cylinders of one radius whose axes cross: the planes of their
        // section, through the crossing point normal to a1 + a2 and a1 - a2,
        // cut space into four wedges, and in each the union's wall is the
        // cylinder whose axis the wedge runs along (|d.a1| against |d.a2|).
        // Near a tangent point both carriers hold a triangle within the bar
        // over a long stretch, and the nearer one by distance is noise; the
        // wedge of its centroid is not.
        let crossing = |a: usize, b: usize| -> Option<(crate::Vec3, crate::Vec3, crate::Vec3)> {
            let (crate::AnalyticSurface::Cylinder(first), crate::AnalyticSurface::Cylinder(second)) = (carriers[a].0, carriers[b].0) else {
                return None;
            };
            if (first.radius - second.radius).abs() > tolerance || first.axis.cross(second.axis).length() <= 0.1 {
                return None;
            }
            let (a1, a2) = (first.axis, second.axis);
            let w0 = first.axis_origin - second.axis_origin;
            let (bb, d, e) = (a1.dot(a2), a1.dot(w0), a2.dot(w0));
            let denominator = 1.0 - bb * bb;
            let p1 = first.axis_origin + a1 * ((bb * e - d) / denominator);
            let p2 = second.axis_origin + a2 * ((e - bb * d) / denominator);
            ((p1 - p2).length() <= tolerance).then_some(((p1 + p2) * 0.5, a1, a2))
        };
        let fit_within = |p: [crate::Vec3; 3], cos_gate: f64| -> Option<(f64, usize)> {
            let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalized()?;
            let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
            let mut held: Vec<(f64, usize)> = Vec::new();
            for (c, (surface, _)) in carriers.iter().enumerate() {
                let worst = p.iter().map(|&q| surface.signed_distance(q).abs()).fold(0.0_f64, f64::max);
                if worst > tolerance || !surface.normal_at(centroid).is_some_and(|n| n.dot(normal).abs() >= cos_gate) {
                    continue;
                }
                held.push((worst, c));
            }
            held.sort_by(|a, b| a.0.total_cmp(&b.0));
            if let [(_, a), (_, b), ..] = held[..] {
                if let Some((center, a1, a2)) = crossing(a, b) {
                    let d = centroid - center;
                    let pick = if d.dot(a1).abs() >= d.dot(a2).abs() { a } else { b };
                    return held.into_iter().find(|h| h.1 == pick);
                }
            }
            held.into_iter().next()
        };
        let fit = |p: [crate::Vec3; 3]| fit_within(p, cos_gate);
        // Row 7d: a strip joins the split only when the accepted carriers
        // hold every one of its triangles, within the bar and the grower's
        // gate; one triangle they do not hold leaves the region as it was.
        if let Some((strip, tris)) = strip_triangles.iter().find(|(_, tris)| !tris.iter().all(|&t| fit(points(t)).is_some())) {
            if debug {
                eprintln!("[split] region {} strip {strip} of {} not held by the carriers, left as segmented", region.id, tris.len());
            }
            report.declined_regions += 1;
            continue;
        }
        if debug && !strip_triangles.is_empty() {
            eprintln!("[split] region {} absorbs strips {:?}", region.id, strip_triangles.iter().map(|(r, tris)| (*r, tris.len())).collect::<Vec<_>>());
        }
        let ids: Vec<usize> = ids.iter().copied().chain(strip_triangles.iter().flat_map(|(_, tris)| tris.iter().copied())).collect();
        let mut owner: Vec<(usize, usize)> = ids.iter().filter_map(|&t| fit(points(t)).map(|(_, c)| (t, c))).collect();
        let assigned = owner.len();
        if debug {
            for (c, (surface, support)) in carriers.iter().enumerate() {
                eprintln!("[split]   carrier {c} support {support} {surface:?}");
            }
            eprintln!("[split]   assigned {assigned} of {}", ids.len());
        }
        if carriers.len() < 2 || assigned * 20 < ids.len() * 19 {
            report.declined_regions += 1;
            continue;
        }
        // The region holds no cap: the grower walled every cap off at its
        // rim. So a remaining region triangle within the bar of a carrier is
        // a jittered wall sliver (see below) and goes to that carrier.
        let claimed: std::collections::HashSet<usize> = owner.iter().map(|o| o.0).collect();
        for &t in &ids {
            if !claimed.contains(&t) {
                if let Some((_, c)) = fit_within(points(t), -1.0) {
                    owner.push((t, c));
                }
            }
        }
        // Slivers the grower walled off inside the same walls (a jittered
        // triangle tilted past the dihedral gate reads as a one-triangle
        // plane, or a two-triangle sphere) join the carrier holding their
        // vertices closest when every edge they share with another region
        // borders this one. Their normals are no evidence either way: a
        // sliver's jittered normal tilts up to ~80 degrees, as far as a cap
        // triangle with all three corners on the rim does; enclosure tells
        // them apart, since a cap piece borders the rest of its cap.
        let mut small = std::collections::BTreeMap::<u32, Vec<usize>>::new();
        for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
            if r != region.id && segmentation.regions.get(r as usize).is_some_and(|other| other.triangle_count < SLIVER_TRIANGLES) {
                small.entry(r).or_default().push(t);
            }
        }
        // Touching slivers are judged together: a run of cap fragments is
        // bounded by the wall alone too, and when the cap is fanned from its
        // rim every one of its triangles lies on the wall's carrier.
        let sliver_of = |t: usize| small.contains_key(&segmentation.triangle_region_ids[t]);
        let mut grouped = std::collections::HashSet::<usize>::new();
        for members in small.values() {
            if grouped.contains(&members[0]) {
                continue;
            }
            let mut group = vec![members[0]];
            grouped.insert(members[0]);
            let mut enclosed = true;
            let mut at = 0;
            while at < group.len() {
                let t = group[at];
                at += 1;
                for c in 0..3 {
                    let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                    for &n in &edges[&(a.min(b), a.max(b))] {
                        if sliver_of(n) {
                            if grouped.insert(n) {
                                group.push(n);
                            }
                        } else if segmentation.triangle_region_ids[n] != region.id && !strips.contains(&segmentation.triangle_region_ids[n]) {
                            enclosed = false;
                        }
                    }
                }
            }
            if !enclosed {
                continue;
            }
            let fits: Option<Vec<(usize, usize)>> = group
                .iter()
                .map(|&t| fit_within(points(t), -1.0).map(|(_, c)| (t, c)))
                .collect();
            let Some(fits) = fits else { continue };
            // Area-weighted tilt of the group from the carriers it lies on.
            let (mut tilted, mut area) = (0.0, 0.0);
            for &(t, c) in &fits {
                let p = points(t);
                let cross = (p[1] - p[0]).cross(p[2] - p[0]);
                let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                if let (Some(n), Some(m)) = (carriers[c].0.normal_at(centroid), cross.normalized()) {
                    tilted += cross.length() * n.dot(m).abs().min(1.0).acos().to_degrees();
                    area += cross.length();
                }
            }
            let tilt = if area > 0.0 { tilted / area } else { 90.0 };
            if debug {
                let p = points(group[0]);
                eprintln!("[split]   sliver group of {} at {:.3},{:.3},{:.3} tilt {tilt:.1}", group.len(), p[0].x, p[0].y, p[0].z);
            }
            // A rim-fanned cap of an n-segment rim has n - 2 triangles, so a
            // group of one or two (a lone jittered sliver reads up to 87
            // degrees) is no cap whatever its tilt.
            if group.len() < SLIVER_GROUP_TRIANGLES && (group.len() <= 2 || tilt <= SLIVER_GROUP_TILT_DEG) {
                owner.extend(fits);
            }
        }
        if debug {
            eprintln!("[split]   with slivers {}", owner.len());
        }
        // At a tangent point both carriers hold the triangles there, so the
        // nearest one can leave a one- or two-triangle island of one carrier
        // inside the other's wall. An island under the sliver size goes to
        // the carrier its neighbours hold when that carrier holds it too.
        let mut carrier_of: std::collections::HashMap<usize, usize> = owner.iter().copied().collect();
        for _ in 0..8 {
            let mut moved = false;
            let mut seen = std::collections::HashSet::<usize>::new();
            for &(start, _) in &owner {
                let c = carrier_of[&start];
                if !seen.insert(start) {
                    continue;
                }
                let mut island = vec![start];
                let mut around = std::collections::BTreeMap::<usize, usize>::new();
                let mut at = 0;
                while at < island.len() {
                    let t = island[at];
                    at += 1;
                    for k in 0..3 {
                        let (a, b) = (indices[3 * t + k], indices[3 * t + (k + 1) % 3]);
                        for &n in &edges[&(a.min(b), a.max(b))] {
                            match carrier_of.get(&n) {
                                Some(&m) if m == c => {
                                    if seen.insert(n) {
                                        island.push(n);
                                    }
                                }
                                Some(&m) => *around.entry(m).or_default() += 1,
                                None => {}
                            }
                        }
                    }
                }
                if island.len() >= SLIVER_TRIANGLES || around.len() != 1 {
                    continue;
                }
                let (&other, _) = around.iter().next().unwrap();
                let surface = carriers[other].0;
                if island.iter().all(|&t| points(t).iter().all(|&q| surface.signed_distance(q).abs() <= tolerance)) {
                    for &t in &island {
                        carrier_of.insert(t, other);
                    }
                    moved = true;
                }
            }
            if !moved {
                break;
            }
        }
        for entry in &mut owner {
            entry.1 = carrier_of[&entry.0];
        }
        // One region per edge-connected piece of each carrier, as the
        // segmenter itself leaves them: the walls of crossing cylinders are
        // cut into pieces that touch only at the tangent points, and one
        // region spanning two pieces makes their seam one closed loop that
        // the builder cannot split at those points.
        let mut pieces: Vec<(usize, Vec<usize>)> = Vec::new();
        let mut placed = std::collections::HashSet::<usize>::new();
        for &(start, c) in &owner {
            if !placed.insert(start) {
                continue;
            }
            let mut piece = vec![start];
            let mut at = 0;
            while at < piece.len() {
                let t = piece[at];
                at += 1;
                for k in 0..3 {
                    let (a, b) = (indices[3 * t + k], indices[3 * t + (k + 1) % 3]);
                    for &n in &edges[&(a.min(b), a.max(b))] {
                        if carrier_of.get(&n) == Some(&c) && placed.insert(n) {
                            piece.push(n);
                        }
                    }
                }
            }
            piece.sort_unstable();
            pieces.push((c, piece));
        }
        if debug {
            for (c, piece) in &pieces {
                eprintln!("[split]   piece of carrier {c}: {} tris", piece.len());
            }
        }
        let kv = |p: crate::Vec3| Vec3::new(p.x, p.y, p.z);
        for (c, members) in pieces {
            let surface = &carriers[c].0;
            if !matches!(surface, crate::AnalyticSurface::Cylinder(_) | crate::AnalyticSurface::Cone(_)) {
                continue;
            }
            let mut outward = 0.0;
            let mut max_deviation = 0.0_f64;
            let mut sum_sq = 0.0;
            let mut samples = 0usize;
            let mut max_angle = 0.0_f64;
            let mut area = 0.0;
            let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for &t in &members {
                let p = points(t);
                let cross = (p[1] - p[0]).cross(p[2] - p[0]);
                area += 0.5 * cross.length();
                let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                if let (Some(n), Some(normal)) = (surface.normal_at(centroid), cross.normalized()) {
                    outward += n.dot(normal);
                    max_angle = max_angle.max(n.dot(normal).abs().min(1.0).acos().to_degrees());
                }
                for q in p {
                    let d = surface.signed_distance(q).abs();
                    max_deviation = max_deviation.max(d);
                    sum_sq += d * d;
                    samples += 1;
                    low = Vec3::new(low.x.min(q.x), low.y.min(q.y), low.z.min(q.z));
                    high = Vec3::new(high.x.max(q.x), high.y.max(q.y), high.z.max(q.z));
                }
            }
            let id = segmentation.regions.len() as u32;
            segmentation.regions.push(MeshRegion {
                id,
                triangle_count: members.len(),
                area,
                bbox_min: low,
                bbox_max: high,
                carrier: match surface {
                    crate::AnalyticSurface::Cone(cone) => RegionCarrier::Cone {
                        apex: kv(cone.apex),
                        axis_dir: kv(cone.axis),
                        half_angle_rad: cone.half_angle,
                        sense: if outward >= 0.0 { 1 } else { -1 },
                    },
                    crate::AnalyticSurface::Cylinder(cylinder) => RegionCarrier::Cylinder {
                        axis_point: kv(cylinder.axis_origin),
                        axis_dir: kv(cylinder.axis),
                        radius: cylinder.radius,
                        sense: if outward >= 0.0 { 1 } else { -1 },
                    },
                    _ => unreachable!("only cylinders and cones are proposed"),
                },
                max_deviation,
                rms_deviation: (sum_sq / samples.max(1) as f64).sqrt(),
                max_normal_angle_deg: max_angle,
            });
            for &t in &members {
                segmentation.triangle_region_ids[t] = id;
            }
            report.noise = report.noise.max(max_deviation);
        }
        report.split_regions += 1;
        changed = true;
    }
    if changed {
        compact(segmentation);
    }
    report
}

/// Drop emptied regions and renumber the rest densely, in order.
fn compact(segmentation: &mut MeshSegmentation) {
    let mut counts = vec![0usize; segmentation.regions.len()];
    for &r in &segmentation.triangle_region_ids {
        if let Some(count) = counts.get_mut(r as usize) {
            *count += 1;
        }
    }
    let mut renumber = vec![u32::MAX; segmentation.regions.len()];
    let mut regions = Vec::new();
    for region in std::mem::take(&mut segmentation.regions) {
        let count = counts[region.id as usize];
        if count == 0 {
            continue;
        }
        renumber[region.id as usize] = regions.len() as u32;
        let mut region = region;
        region.id = regions.len() as u32;
        // A freeform remainder keeps its triangles but not its old count.
        region.triangle_count = count;
        regions.push(region);
    }
    for r in &mut segmentation.triangle_region_ids {
        if let Some(&new) = renumber.get(*r as usize) {
            *r = new;
        }
    }
    segmentation.regions = regions;
}

/// The carriers a region is split into, from the pass's accepted surfaces.
///
/// Lane J row 7c: cylinders and cones, pieces of one carrier merged,
/// cylinders before cones and each by size. A candidate is kept only when it
/// holds at least [`MIN_SPLIT_TRIANGLES`] of the region's triangles that no
/// carrier kept before it holds (all three vertices within the bar, the
/// normal within the grower's gate). A countersunk hole's bore holds 256 of
/// 3 766 triangles (6.8 %) no cone holds, and was dropped by the share of a
/// tenth this replaces; the cones the pass fits on a crossing wall (122 to 294
/// triangles) and its redundant sub-wall cylinders are held whole by the two
/// walls, and are dropped as the share dropped them.
fn propose(
    accepted: &[crate::SurfaceRegion],
    ids: &[usize],
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    cos_gate: f64,
    cones: bool,
) -> Vec<(crate::AnalyticSurface, usize)> {
    let mut candidates: Vec<(crate::AnalyticSurface, usize)> = Vec::new();
    let mut regions = accepted.to_vec();
    regions.sort_by(|a, b| b.triangle_indices.len().cmp(&a.triangle_indices.len()));
    if !cones {
        // Row 7: cylinders holding a tenth of the region, pieces merged.
        for r in &regions {
            if r.triangle_indices.len() * 10 < ids.len() || !matches!(r.surface, crate::AnalyticSurface::Cylinder(_)) {
                continue;
            }
            if let Some(entry) = candidates.iter_mut().find(|(c, _)| same_cylinder(c, &r.surface, tolerance)) {
                entry.1 += r.triangle_indices.len();
            } else {
                candidates.push((r.surface, r.triangle_indices.len()));
            }
        }
        return candidates;
    }
    for r in &regions {
        if !matches!(r.surface, crate::AnalyticSurface::Cylinder(_) | crate::AnalyticSurface::Cone(_)) {
            continue;
        }
        if let Some(entry) = candidates
            .iter_mut()
            .find(|(c, _)| same_cylinder(c, &r.surface, tolerance) || same_cone(c, &r.surface, tolerance))
        {
            entry.1 += r.triangle_indices.len();
        } else {
            candidates.push((r.surface, r.triangle_indices.len()));
        }
    }
    let is_cone = |s: &crate::AnalyticSurface| matches!(s, crate::AnalyticSurface::Cone(_));
    candidates.sort_by(|a, b| is_cone(&a.0).cmp(&is_cone(&b.0)).then(b.1.cmp(&a.1)));
    let holds = |surface: &crate::AnalyticSurface, t: usize| -> bool {
        let p = [0, 1, 2].map(|c| {
            let at = indices[3 * t + c] as usize * 3;
            crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
        });
        let Some(normal) = (p[1] - p[0]).cross(p[2] - p[0]).normalized() else { return false };
        let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
        p.iter().all(|&q| surface.signed_distance(q).abs() <= tolerance)
            && surface.normal_at(centroid).is_some_and(|n| n.dot(normal).abs() >= cos_gate)
    };
    let mut held = vec![false; ids.len()];
    let mut kept = Vec::new();
    for (surface, support) in candidates {
        // A fit the pass accepted on a handful of triangles is not a carrier,
        // however many more it holds at a wide bar: a 6-triangle cylinder
        // fit, skewed across a countersunk hole, held 64 at s10's 7.6e-2.
        if support < MIN_SPLIT_TRIANGLES {
            continue;
        }
        let own: Vec<usize> = (0..ids.len()).filter(|&i| !held[i] && holds(&surface, ids[i])).collect();
        if own.len() < MIN_SPLIT_TRIANGLES {
            continue;
        }
        for i in own {
            held[i] = true;
        }
        kept.push((surface, support));
    }
    kept
}

/// Two fitted cones are one carrier: parallel axes, one opening angle, and
/// apexes within the bar of each other.
fn same_cone(a: &crate::AnalyticSurface, b: &crate::AnalyticSurface, tolerance: f64) -> bool {
    let (crate::AnalyticSurface::Cone(a), crate::AnalyticSurface::Cone(b)) = (a, b) else {
        return false;
    };
    a.axis.dot(b.axis) >= (0.5_f64).to_radians().cos()
        && (a.half_angle - b.half_angle).abs() <= (0.5_f64).to_radians()
        && (a.apex - b.apex).length() <= tolerance
}

fn same_cylinder(a: &crate::AnalyticSurface, b: &crate::AnalyticSurface, tolerance: f64) -> bool {
    let (crate::AnalyticSurface::Cylinder(a), crate::AnalyticSurface::Cylinder(b)) = (a, b) else {
        return false;
    };
    let offset = b.axis_origin - a.axis_origin;
    let off_axis = (offset - a.axis * offset.dot(a.axis)).length();
    a.axis.dot(b.axis).abs() >= (0.5_f64).to_radians().cos()
        && (a.radius - b.radius).abs() <= tolerance
        && off_axis <= tolerance
}

/// Debug evidence for the split's work budget (BREP_SPLIT_COMPARE): reruns
/// the region without a budget and prints both runs' work, every accepted
/// surface, and whether the budgeted run's surfaces are the unbudgeted run's
/// (same carrier, same triangles) when its budget did not run out.
fn compare_budget(
    mesh: &Mesh,
    recognition: &RecognitionOptions,
    budgeted: &Result<(crate::RecognitionResult, crate::RecognitionWork), crate::RecognitionError>,
    region: u32,
    (ids, positions, indices, cos_gate): (&[usize], &[f64], &[u32], f64),
    cones: bool,
) {
    let kinds = if cones { &SPLIT_KINDS[..] } else { &SPLIT_KINDS[..2] };
    let triangles = ids.len();
    let started = std::time::Instant::now();
    let unbounded = recognize_surfaces_with_attempt_budget(mesh, recognition, usize::MAX, kinds);
    let old_seconds = started.elapsed().as_secs_f64();
    for (arm, result) in [("old", &unbounded), ("new", budgeted)] {
        match result {
            Ok((result, work)) => {
                eprintln!(
                    "[split-compare] region {region} tris {triangles} {arm} work attempts {} rounds {} exhausted {}",
                    work.attempts, work.rounds, work.exhausted
                );
                for r in &result.regions {
                    eprintln!(
                        "[split-compare] region {region} tris {triangles} {arm} {:?} {} share {:.4}",
                        r.surface.surface_type(),
                        r.triangle_indices.len(),
                        r.triangle_indices.len() as f64 / triangles as f64
                    );
                }
            }
            Err(error) => eprintln!("[split-compare] region {region} tris {triangles} {arm} error {error:?}"),
        }
    }
    let identical = match (&unbounded, budgeted) {
        (Ok((old, _)), Ok((new, _))) => old == new,
        (Err(old), Err(new)) => old == new,
        _ => false,
    };
    let exhausted = budgeted.as_ref().is_ok_and(|(_, work)| work.exhausted);
    eprintln!(
        "[split-compare] region {region} tris {triangles} budget-exhausted {exhausted} identical {identical} old_seconds {old_seconds:.2}"
    );
    // What the split uses from a pass: its proposals (`propose`). Against
    // the every-kind pass.
    let proposals = |result: &Result<(crate::RecognitionResult, crate::RecognitionWork), crate::RecognitionError>| {
        result
            .as_ref()
            .map(|(result, _)| propose(&result.regions, ids, positions, indices, recognition.distance_tolerance, cos_gate, cones))
            .unwrap_or_default()
            .into_iter()
            .map(|(surface, support)| (format!("{:?}", surface.surface_type()), support))
            .collect::<Vec<_>>()
    };
    let every_kind = recognize_surfaces_with_attempt_budget(mesh, recognition, usize::MAX, &[]);
    let (restricted, all) = (proposals(budgeted), proposals(&every_kind));
    eprintln!(
        "[split-compare] region {region} tris {triangles} proposals restricted {restricted:?} every-kind {all:?} same {}",
        restricted == all
    );
    // Where the pass spends its time, in a third run: timings land in the
    // fit diagnostics and would make the two runs above compare unequal.
    if std::env::var("BREP_SPLIT_PHASES").is_ok() {
        let timed = RecognitionOptions {
            collect_phase_timings: true,
            ..recognition.clone()
        };
        let started = std::time::Instant::now();
        let cap = if cones { SPLIT_CONE_STAGE_MAX_ATTEMPTS } else { SPLIT_MAX_ATTEMPTS };
        if let Ok((result, work)) = recognize_surfaces_with_attempt_budget(mesh, &timed, cap, kinds) {
            let mut sums = [0.0_f64; 6];
            for r in &result.regions {
                let t = r.diagnostics.phase_timings;
                for (sum, value) in sums.iter_mut().zip([
                    t.metadata_inspection_seconds,
                    t.candidate_generation_seconds,
                    t.candidate_evaluation_seconds,
                    t.region_growth_seconds,
                    t.refinement_seconds,
                    t.validation_seconds,
                ]) {
                    *sum += value.unwrap_or(0.0);
                }
            }
            let wall = started.elapsed().as_secs_f64();
            eprintln!(
                "[split-phases] region {region} tris {triangles} attempts {} rounds {} wall {wall:.2} ms/attempt {:.2} metadata {:.2} generation {:.2} evaluation {:.2} growth {:.2} refinement {:.2} validation {:.2}",
                work.attempts,
                work.rounds,
                1e3 * wall / work.attempts.max(1) as f64,
                sums[0],
                sums[1],
                sums[2],
                sums[3],
                sums[4],
                sums[5]
            );
        }
    }
}

/// Share of the triangles whose normals lie within a band of half-width
/// asin(`sin_band`) about the great circle normal to one of up to four axes,
/// the axes chosen greedily among cross products of deterministic normal
/// pairs. A cylinder's normals all lie on the great circle normal to its
/// axis; a torus's or a sphere's cover the sphere.
fn normal_band_coverage(normals: &[crate::Vec3], sin_band: f64) -> f64 {
    if normals.len() < 2 {
        return 0.0;
    }
    let mut state = 0x9E37_79B9_7F4A_7C15_u64;
    let mut next = |n: usize| {
        state = state.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        ((state >> 33) as usize) % n
    };
    let mut axes = Vec::new();
    for _ in 0..256 {
        let (a, b) = (normals[next(normals.len())], normals[next(normals.len())]);
        if let Some(axis) = a.cross(b).normalized().filter(|_| a.cross(b).length() > 0.2) {
            axes.push(axis);
        }
    }
    let mut covered = vec![false; normals.len()];
    for _ in 0..4 {
        let best = axes.iter().max_by_key(|axis| {
            normals.iter().zip(&covered).filter(|(n, &c)| !c && n.dot(**axis).abs() <= sin_band).count()
        });
        let Some(&axis) = best else { break };
        for (n, c) in normals.iter().zip(covered.iter_mut()) {
            if n.dot(axis).abs() <= sin_band {
                *c = true;
            }
        }
    }
    covered.iter().filter(|&&c| c).count() as f64 / normals.len() as f64
}

/// After the noisy cap join: a cap fragment under [`SLIVER_TRIANGLES`] that
/// borders a plane region of at least that size joins it when the larger
/// plane holds every one of its vertices within the measured noise. The
/// join only merges runs of small fragments with each other; a two-triangle
/// piece of a 126-triangle cap at the rim of an 11 402-triangle crossing
/// stayed a plane of its own, tilted 0.28 rad off the wall's axis, and the
/// builder demoted every wall for it. A freeform fragment the large plane
/// holds joins it the same way: a two-triangle one on the rim of the radius-1
/// d0.001 crossing failed its wall's edge, and the demotion ran through all
/// four wall pieces.
pub(super) fn absorb_plane_fragments(segmentation: &mut MeshSegmentation, positions: &[f64], indices: &[u32], noise: f64) {
    if !(noise > 0.0) || indices.len() != segmentation.triangle_region_ids.len() * 3 {
        return;
    }
    let point = |v: u32| {
        let at = v as usize * 3;
        Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    };
    let mut edges = std::collections::HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..segmentation.triangle_region_ids.len() {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut members = std::collections::BTreeMap::<u32, Vec<usize>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        members.entry(r).or_default().push(t);
    }
    let mut changed = false;
    for (&fragment, triangles) in &members {
        let Some(region) = segmentation.regions.get(fragment as usize) else { continue };
        if !matches!(region.carrier, RegionCarrier::Plane { .. } | RegionCarrier::Freeform) || triangles.len() >= SLIVER_TRIANGLES {
            continue;
        }
        let mut neighbours = std::collections::BTreeSet::new();
        for &t in triangles {
            for c in 0..3 {
                let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                for &n in &edges[&(a.min(b), a.max(b))] {
                    let r = segmentation.triangle_region_ids[n];
                    if r != fragment {
                        neighbours.insert(r);
                    }
                }
            }
        }
        let host = neighbours.into_iter().find(|&r| {
            let Some(other) = segmentation.regions.get(r as usize) else { return false };
            let RegionCarrier::Plane { origin, normal } = other.carrier else { return false };
            other.triangle_count >= SLIVER_TRIANGLES
                && triangles.iter().all(|&t| {
                    (0..3).all(|c| point(indices[3 * t + c]).sub(origin).dot(normal).abs() <= noise)
                })
        });
        if let Some(host) = host {
            for &t in triangles {
                segmentation.triangle_region_ids[t] = host;
            }
            changed = true;
        }
    }
    if changed {
        compact(segmentation);
    }
}
