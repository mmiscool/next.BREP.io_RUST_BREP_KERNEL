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
    brep_kernel::switch!(live "BREP_HYBRID_SPLIT_CONES", true)
}


/// Lane J row 7d, promoted ON 2026-10-16: a region bordered by curved strips of the segmenter's own
/// is split when the carriers the split accepts hold every triangle of every
/// strip, and the strips join the pieces. BREP_HYBRID_SPLIT_STRIPS=0 restores
/// row 7c's decline.
fn split_strips_enabled() -> bool {
    brep_kernel::switch!(live "BREP_HYBRID_SPLIT_STRIPS", true)
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

/// The cone stage's budget under
/// [`crate::hybrid_region_brep::HybridSwitches::cone_stage_budget`]: on census
/// record kernel_countersunk_hole_s10_d0.005_jitter-1.0 (an 894-triangle
/// countersink over a 96-triangle bore) the stage ran out at 256 and the row
/// kept 970 faceted triangles; at 2 048 it rebuilds with none.
const SPLIT_CONE_STAGE_BUDGET_ATTEMPTS: usize = 2_048;

/// Under the larger cone-stage budget, the smallest half angle a proposed
/// cone may have; a narrower one is a cylinder drawn as a cone.
const CONE_STAGE_MIN_HALF_ANGLE_DEG: f64 = 1.0;

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
    /// Runs of shredded cone strips gathered into one cone after the split
    /// (`HybridSwitches::gather_cone_strips`).
    pub(super) gathered_cone_runs: usize,
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

/// `lone_cone` is [`crate::hybrid_region_brep::HybridSwitches::split_lone_cone`].
pub(super) fn split(
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
    segmentation: &mut MeshSegmentation,
    lone_cone: bool,
    cone_bore: bool,
    probe_crease: bool,
    cone_budget: bool,
    bore_strips: bool,
    lone_share: bool,
    merge_refit: bool,
    noise_bar: bool,
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
    // Under `noise_bar`: the freeform dust the segmenter leaves on a
    // jittered body (regions under the split floor sharing edges with one
    // another) is gathered into one region, which the passes below then
    // draw as any other (census record kernel_stepped_shaft_s10_d0.1_jitter-1.0:
    // 28 freeform regions of 2-38 triangles, 216 of the 252, and no region
    // reached the split).
    // Only on a body no region of which reaches the split: where one does,
    // the dust beside it is that region's strips, bands and slivers (census
    // record occt_countersunk_hole_s10_d0.05_jitter-0.1 went from 0 to 272
    // faceted with its bands gathered away from its bore).
    if noise_bar && merge_refit && sphere_bands_to_cylinders(segmentation, positions, indices) > 0 {
        changed = true;
    }
    let mut gathered = std::collections::HashSet::<u32>::new();
    // A gather the split then declines is undone (its dust goes back to the
    // regions it came from, which later passes join as caps and slivers):
    // left gathered, census record occt_cylinder_s1_d0.1_jitter-0.3's caps
    // stayed facets with the wall, 120 -> 145 faceted.
    let mut gather_undo: Vec<(u32, Vec<(usize, u32)>, Vec<(usize, usize)>, (usize, RegionCarrier))> = Vec::new();
    let mut split_done = std::collections::HashSet::<u32>::new();
    if noise_bar && merge_refit && !segmentation.regions.iter().any(candidate) {
        // On a body no region of which, of any kind, reaches the floor, the
        // dust is every region: a coarse jittered wall's two-triangle
        // facets read as planes, cylinders and tori to the segmenter
        // (census record kernel_cylinder_s1_d0.1_jitter-0.1: 36 plane, 7
        // freeform and 3 torus regions, the largest of 8 triangles).
        // On a jittered body with regions over the floor too, every region
        // under it is dust as well (a jittered frustum's caps are plane chips
        // and freeform pieces beside one cone region of 119): the body reads
        // as jittered when every plane region of 16 triangles or more fits
        // no better than GATHER_NOISY_PLANE of the mesh's diagonal (2-6e-4
        // on the jittered bodies; a T-junction mesh's planes fit exactly, and
        // gathered there its exact cylinders were refitted at the noise bar,
        // census record occt_crossed_cylinders_s1_d0.05_tjunction 7.7e-7 ->
        // 1.9e-5 volume error).
        let mesh_diagonal = {
            let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
            for p in positions.chunks_exact(3) {
                for k in 0..3 {
                    low[k] = low[k].min(p[k]);
                    high[k] = high[k].max(p[k]);
                }
            }
            (0..3).map(|k| (high[k] - low[k]).powi(2)).sum::<f64>().sqrt()
        };
        let planes: Vec<&MeshRegion> = segmentation
            .regions
            .iter()
            .filter(|r| matches!(r.carrier, RegionCarrier::Plane { .. }) && r.triangle_count >= GATHERED_CARRIER_TRIANGLES)
            .collect();
        // With no plane region of 16, the plane chips of two triangles or
        // more stand in: the body reads as jittered when more than half of
        // them fit no better than GATHER_NOISY_CHIP (census record
        // kernel_cylinder_s10_d0.1_jitter-0.1: the wall is one cylinder of
        // 64 and each cap a dozen chips of 1-10 triangles, 6 of 9 such
        // chips 0.9-3.5 bars off; a clean or T-junction mesh's chips fit
        // exactly).
        let jittered = if planes.is_empty() {
            let chips: Vec<&MeshRegion> = segmentation
                .regions
                .iter()
                .filter(|r| matches!(r.carrier, RegionCarrier::Plane { .. }) && r.triangle_count >= 2)
                .collect();
            let noisy = chips.iter().filter(|r| r.max_deviation > GATHER_NOISY_CHIP * mesh_diagonal).count();
            !chips.is_empty() && noisy * 2 > chips.len()
        } else {
            planes.iter().all(|r| r.max_deviation > GATHER_NOISY_PLANE * mesh_diagonal)
        };
        let all_dust = jittered || !segmentation.regions.iter().any(|r| r.triangle_count >= MIN_SPLIT_TRIANGLES);
        if debug {
            let read: Vec<(usize, f64)> = planes.iter().map(|r| (r.triangle_count, r.max_deviation / mesh_diagonal)).collect();
            eprintln!("[split] dust gather: jittered {jittered} all dust {all_dust}, planes of 16 (triangles, deviation / diagonal) {read:?}");
        }
        let dust: Vec<bool> = segmentation
            .regions
            .iter()
            .map(|r| {
                (all_dust || matches!(r.carrier, RegionCarrier::Freeform)) && r.triangle_count > 0 && r.triangle_count < MIN_SPLIT_TRIANGLES
            })
            .collect();
        let mut parent: Vec<usize> = (0..segmentation.regions.len()).collect();
        fn find(parent: &mut [usize], mut i: usize) -> usize {
            while parent[i] != i {
                parent[i] = parent[parent[i]];
                i = parent[i];
            }
            i
        }
        for tris in edges.values() {
            if let [t0, t1] = tris[..] {
                let (a, b) = (segmentation.triangle_region_ids[t0] as usize, segmentation.triangle_region_ids[t1] as usize);
                if a != b && dust.get(a).copied().unwrap_or(false) && dust.get(b).copied().unwrap_or(false) {
                    let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                    if ra != rb {
                        parent[ra] = rb;
                    }
                }
            }
        }
        let mut groups = std::collections::BTreeMap::<usize, Vec<usize>>::new();
        for (i, &is_dust) in dust.iter().enumerate() {
            if is_dust {
                let root = find(&mut parent, i);
                groups.entry(root).or_default().push(i);
            }
        }
        for members in groups.values() {
            let total: usize = members.iter().map(|&i| segmentation.regions[i].triangle_count).sum();
            // On a jittered body a gather as small as a cap is kept (a
            // jittered frustum's two caps gather apart, 30-40 triangles each,
            // beside the cone region).
            let floor = if jittered || all_dust { GATHERED_CARRIER_TRIANGLES } else { MIN_SPLIT_TRIANGLES };
            // A group one region already holds three in four of is that
            // region's face with a chip beside it, which the later passes
            // join (gathered, census record occt_stepped_shaft_s1_d0.005_jitter-1.0's
            // cap of 59 and one chip were refitted at the noise bar: volume
            // error 5.9e-6 -> 1.0e-4).
            let largest = members.iter().map(|&i| segmentation.regions[i].triangle_count).max().unwrap_or(0);
            // Two regions are no dust (census record
            // kernel_pointed_cone_s10_d0.02_jitter-0.1: the segmenter's cone
            // of 32 and plane of 30, which build as they are).
            if members.len() < 3 || total < floor || largest * 4 >= total * 3 {
                continue;
            }
            let host = *members.iter().max_by_key(|&&i| segmentation.regions[i].triangle_count).unwrap();
            let host_id = segmentation.regions[host].id;
            let member_ids: std::collections::HashSet<u32> = members.iter().map(|&i| segmentation.regions[i].id).collect();
            let undo_ids: Vec<(usize, u32)> = segmentation.triangle_region_ids.iter().enumerate().filter(|(_, r)| member_ids.contains(r)).map(|(t, &r)| (t, r)).collect();
            let undo_counts: Vec<(usize, usize)> = members.iter().map(|&i| (i, segmentation.regions[i].triangle_count)).collect();
            gather_undo.push((host_id, undo_ids, undo_counts, (host, segmentation.regions[host].carrier.clone())));
            // The gathered region is freeform whatever its host was.
            segmentation.regions[host].carrier = RegionCarrier::Freeform;
            for &i in members {
                if i == host {
                    continue;
                }
                let id = segmentation.regions[i].id;
                for r in segmentation.triangle_region_ids.iter_mut() {
                    if *r == id {
                        *r = host_id;
                    }
                }
                segmentation.regions[i].triangle_count = 0;
            }
            segmentation.regions[host].triangle_count = total;
            gathered.insert(host_id);
            changed = true;
            if debug {
                eprintln!("[split] gathered {} dust regions into region {host_id} ({total} tris)", members.len());
            }
        }
    }
    if !segmentation.regions.iter().any(candidate) && gathered.is_empty() {
        if changed {
            compact(segmentation);
        }
        return report;
    }
    for region in segmentation.regions.clone() {
        if !candidate(&region) && !gathered.contains(&region.id) {
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
        // A gathered dust region takes no curved strips: its curved
        // neighbour is the body's wall (a jittered frustum's cap dust beside
        // its cone of 119), which stays as it is.
        let mut frontier = if gathered.contains(&region.id) { Default::default() } else { curved_neighbours(&ids, &strips) };
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
        let mut recognition = RecognitionOptions::default();
        recognition.distance_tolerance = options.fit_tolerance * diagonal;
        recognition.relative_tolerance = 0.0;
        recognition.normal_tolerance = options.normal_tolerance_deg.to_radians();
        recognition.sampling = SamplingMode::Vertices;
        recognition.max_hypotheses = SPLIT_MAX_HYPOTHESES;
        recognition.max_refinement_iterations = SPLIT_MAX_REFINEMENT_ITERATIONS;
        recognition.torus_fast = crate::hypothesis_switch::torus_fast();
        let mesh = Mesh::new(vertices, triangles);
        let tolerance = recognition.distance_tolerance;
        let cos_gate = options.deflection_angle_deg.to_radians().cos();
        // A region whose normals cover no band is no cylinder or cone wall
        // and the stages below are not drawn on it; under `noise_bar` it is
        // drawn once, ungated, for a torus at the noise bars as a lone
        // carrier (the jitter-1.0 d0.1 tori: one freeform region of ~3 000
        // triangles the segmenter's torus fit does not hold, which a torus
        // drawn at six bars holds 94-97 of 100 of).
        // A band-declined region the whole-primitive path will take as it
        // is (the strays beside it each under its stray bar and together
        // at most 1 % of the mesh: `whole_closed_primitive`, row 10) is left
        // to it, which fits every vertex and tests the residual as noise:
        // drawn here instead, 20 sphere rows that path built moved in
        // volume (under 3x either way, e.g. census record
        // occt_sphere_s1_d0.1_jitter-0.1 1.1e-4 -> 2.7e-4).
        let band_declined = coverage < BAND_MIN_COVERAGE && !gathered.contains(&region.id);
        let whole_takes_it = {
            let strays = segmentation.regions.iter().filter(|other| other.id != region.id);
            !strays.clone().any(|other| other.triangle_count >= WHOLE_STRAY_TRIANGLES)
                && 100 * strays.map(|other| other.triangle_count).sum::<usize>() <= segmentation.triangle_region_ids.len()
        };
        if band_declined && !(noise_bar && merge_refit && !whole_takes_it) {
            report.band_declined_regions += 1;
            continue;
        }
        // Stage 1 is row 7's pass: planes and cylinders, cylinders holding a
        // tenth of the region. Only a region it leaves without two such
        // cylinders goes on (row 7c; BREP_HYBRID_SPLIT_CONES=0 turns it off) to a
        // pass that draws cones too and proposes by exclusive support, so
        // every region row 7 divides is divided exactly as before.
        // Under `probe_crease` a region the stages divide nothing of is
        // drawn again with hypothesis probes held to one wall
        // (`RecognitionOptions::probe_crease_gate`); a region they divide is
        // divided as without it (gated from the start, the jittered kernel
        // crossings' fits moved and their volume errors grew up to 8 times).
        let gates: &[bool] = if probe_crease { &[false, true] } else { &[false] };
        let mut chosen = None;
        let mut out_of_budget = false;
        let mut bore_bridges: Vec<(u32, Vec<usize>)> = Vec::new();
        // Under `merge_refit` the first pass whose carriers' fits hold less
        // than 19 in 20 of the region proposes once more with its cylinder
        // fits of one wall refitted as one; that proposal waits here and the
        // region takes it only when the passes as they are decline it.
        let mut merged_fallback: Option<(Vec<(crate::AnalyticSurface, usize)>, Vec<crate::CylinderSurface>)> = None;
        // Set, to its bar in split bars, when `merged_fallback` is the
        // noise-bar pass's proposal.
        let mut noise_fallback: Option<f64> = None;
        // The strips beside the region that the noise-bar proposal's bore
        // holds (the cone stage's `bore_strips`, unconditional here).
        let mut noise_bridges: Vec<(u32, Vec<usize>)> = Vec::new();
        for &gated in gates {
            // A gathered dust region is not drawn at the bar either: the
            // stages spent the region's attempt budget there and the noise
            // draw got 20 attempts (census record
            // occt_stepped_shaft_s10_d0.1_jitter-0.3, no plane fitted).
            if band_declined || gathered.contains(&region.id) {
                break;
            }
            recognition.probe_crease_gate = gated;
            if gated {
                recognition.hypothesis_probe_triangles = GATED_PROBE_TRIANGLES;
            }
            let mut stages = vec![false];
            if split_cones_enabled() {
                stages.push(true);
            }
            let stages_left = |cones: bool| !cones && split_cones_enabled();
            let mut fallback = None;
            out_of_budget = false;
            for cones in stages {
                // The strips belong to the stage that found them.
                bore_bridges.clear();
                let kinds = if cones { &SPLIT_KINDS[..] } else { &SPLIT_KINDS[..2] };
                let started = std::time::Instant::now();
                let cap = match (cones, cone_budget) {
                    (true, true) => SPLIT_CONE_STAGE_BUDGET_ATTEMPTS,
                    (true, false) => SPLIT_CONE_STAGE_MAX_ATTEMPTS,
                    (false, _) => SPLIT_MAX_ATTEMPTS,
                };
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
                let mut carriers = propose(&result.regions, &ids, positions, indices, tolerance, cos_gate, cones, &[]);
                // Under `merge_refit` the first pass whose carriers' fits
                // hold less than 19 in 20 of the region has its cylinder fits
                // of one wall refitted as one ([`merge_refit_cylinders`]) and
                // proposes again into `merged_fallback`; the pass itself goes
                // on as it is. Taken in the pass's place the merged walls
                // (assigned at 2 bars) moved crossings the passes already
                // split: census records occt_crossed_cylinders_s10_d0.02_jitter-0.3
                // and occt_crossed_cylinders_s1_d0.005_jitter-1.0 went from 0
                // to 396 and 431 faceted, and three rows' volume errors grew
                // tenfold. A pass whose fits cover the region is not merged
                // (two fits of 355 and 262 triangles on the 1 889 of
                // kernel_crossed_cylinders_s1_d0.1_jitter-0.1 are carriers
                // that fail the assignment, and are).
                let fits_cover = carriers.iter().map(|(_, support)| support).sum::<usize>() * 20 >= ids.len() * 19;
                if merge_refit && !fits_cover && merged_fallback.is_none() {
                    let (mut accepted, walls) = merge_refit_cylinders(&result.regions, &mesh, &recognition, tolerance, MERGE_REFIT_FIT_BARS);
                    // A merged wall's support is the region it holds at the
                    // merge bar, not its fits' triangles: on
                    // kernel_crossed_cylinders_s1_d0.05_jitter-0.3 the pass's
                    // cylinder fits covered a quarter of the region.
                    let bar = MERGE_REFIT_FIT_BARS * tolerance;
                    for r in accepted.iter_mut() {
                        if !walls.iter().any(|w| r.surface == crate::AnalyticSurface::Cylinder(*w)) {
                            continue;
                        }
                        let fits = r.triangle_indices.len();
                        r.triangle_indices = (0..ids.len()).filter(|&i| holds(&r.surface, ids[i], positions, indices, bar, cos_gate)).collect();
                        if debug {
                            eprintln!("[split] merged wall fits {fits} holds {} at 1 bar, {} at {MERGE_REFIT_FIT_BARS} bars (gated {gated} cones {cones})",
                                (0..ids.len()).filter(|&i| holds(&r.surface, ids[i], positions, indices, tolerance, cos_gate)).count(), r.triangle_indices.len());
                        }
                    }
                    if !walls.is_empty() {
                        let merged = propose(&accepted, &ids, positions, indices, tolerance, cos_gate, cones, &walls);
                        if merged.len() >= 2 {
                            merged_fallback = Some((merged, walls));
                        }
                    }
                }
                // Under `cone_budget` the stage runs to the end on regions it
                // used to run out on, and a cylinder drawn as a cone (half
                // angle 6.7e-4, apex 1 492 away on the radius-1 crossing of
                // census record occt_crossed_cylinders_s1_d0.001_jitter-0.3)
                // took the region from the gated pass's two cylinders.
                if cones && cone_budget {
                    carriers.retain(|(carrier, _)| {
                        !matches!(carrier, crate::AnalyticSurface::Cone(cone) if cone.half_angle < CONE_STAGE_MIN_HALF_ANGLE_DEG.to_radians())
                    });
                }
                // Under `cone_bore` a cone the cone stage proposes alone that
                // holds less than 19 in 20 of the region takes a cylinder on its
                // axis through the rest (the bore below a countersink, which
                // comes as 16-triangle fits under the carrier floor: census
                // record kernel_countersunk_hole_s1_d0.005_jitter-0.1, cone 894 of
                // 958 triangles).
                // Under `bore_strips` the bore may also take the small plane
                // regions beside the region that it holds and the cone does
                // not (the 2-triangle strips the segmenter cut out of the
                // radius-1 bore of census record
                // kernel_countersunk_hole_s1_d0.005_jitter-1.0): they join the
                // split as strips, and the bore's pieces are counted with them.
                if cones && cone_bore {
                    if let [(crate::AnalyticSurface::Cone(cone), support)] = carriers[..] {
                        if support * 20 < ids.len() * 19 {
                            let bridges = |bore: &crate::AnalyticSurface| {
                                bore_strip_regions(bore, &cone, &ids, region.id, &*segmentation, &edges, positions, indices, tolerance, cos_gate, cos_gate, f64::INFINITY, None, false)
                            };
                            let bridges: Option<&dyn Fn(&crate::AnalyticSurface) -> Option<Vec<(u32, Vec<usize>)>>> =
                                if bore_strips { Some(&bridges) } else { None };
                            if let Some((bore, held, found)) = coaxial_bore(&cone, &ids, positions, indices, tolerance, cos_gate, bridges) {
                                if debug {
                                    eprintln!("[split] region {} cone of {support} takes a coaxial bore {:?} with {} strips", region.id, (bore, held), found.len());
                                }
                                carriers.push((bore, held));
                                bore_bridges = found;
                            }
                        }
                    }
                }
                // Under `lone_share` a lone cone holding 3 in 4 of the region
                // takes it too ([`LONE_SHARE`]); what it leaves stays in the
                // region for [`assemble_cone_bore`].
                // Under `noise_bar` a cone that holds 19 in 20 of the region
                // at the top of the noise-bar ladder is the region's wall
                // with its jitter, and the noise-bar pass takes it whole: at
                // the share, census record kernel_frustum_s1_d0.1_jitter-0.1's
                // cone of 334 took the region and left 67 triangles freeform
                // (0 -> 506 faceted beside BREP_HYBRID_ASSEMBLE_CONE_BORE).
                let held_share = |surface: &crate::AnalyticSurface| {
                    let held = ids.iter().filter(|&&t| holds(surface, t, positions, indices, tolerance, cos_gate)).count();
                    let noise_held = || {
                        let bar = NOISE_BAR_LADDER[NOISE_BAR_LADDER.len() - 1] * tolerance;
                        ids.iter().filter(|&&t| holds(surface, t, positions, indices, bar, cos_gate)).count() * 20 >= ids.len() * 19
                    };
                    held * LONE_SHARE.1 >= ids.len() * LONE_SHARE.0 && !(noise_bar && merge_refit && noise_held())
                };
                let lone = cones
                    && lone_cone
                    && matches!(&carriers[..], [(surface @ crate::AnalyticSurface::Cone(_), support)] if *support * 20 >= ids.len() * 19
                        || (lone_share && held_share(surface)));
                // Under the lone-cone switch, row 7's cylinders that hold less
                // than 19 in 20 of the region (two 394-triangle fits on the
                // 3 925-triangle countersink of census record
                // occt_countersunk_hole_s10_d0.001_tjunction) wait for the cone
                // stage, and are taken unless it proposes a lone cone (on the
                // jittered crossings its two-carrier proposals replaced row 7's
                // and 7 census rows went from 0 to 2 279-11 963 faceted).
                let covered = carriers.iter().map(|(_, support)| support).sum::<usize>() * 20 >= ids.len() * 19;
                if !cones && lone_cone && carriers.len() >= 2 && !covered && stages_left(cones) {
                    fallback = Some(carriers);
                    continue;
                }
                // A deferred row 7 choice gives way only to a lone cone.
                if lone || (carriers.len() >= 2 && fallback.is_none()) {
                    chosen = Some(carriers);
                    break;
                }
            }
            if chosen.is_none() && fallback.is_some() {
                // Row 7's choice, as without the switch, whatever the cone stage
                // spent.
                chosen = fallback;
                out_of_budget = false;
                bore_bridges.clear();
            }
            if chosen.is_some() {
                break;
            }
        }
        // Under `noise_bar` a choice of one cone fitted twice (declined
        // below) goes to the noise-bar draws instead, which may hold the
        // wall with one cone: on census record occt_frustum_s1_d0.1_jitter-0.1
        // the cone stage fitted the 258-triangle wall twice and the region
        // stayed 420 facets.
        if noise_bar && merge_refit && chosen.as_ref().is_some_and(|carriers| twin_cones(carriers)) {
            if debug {
                eprintln!("[split] region {} has one cone fitted twice, drawn at the noise bars", region.id);
            }
            chosen = None;
            merged_fallback = None;
        }
        // Under `noise_bar` a region every pass leaves without a merged
        // proposal is drawn once more, ungated, for planes and cylinders at
        // the noise bar ([`NOISE_BAR_FIT_BARS`]), and every cylinder fit
        // of that pass is a wall at that bar: refitted with the fits of its
        // wall ([`merge_refit_cylinders`]), supported by what it holds at the
        // bar, and proposed as the merged fallback. On census record
        // kernel_crossed_cylinders_s1_d0.02_jitter-1.0 the vertices lie up
        // to 0.016 off the walls against a 0.008 bar, and the passes at the
        // bar accepted no fit at all (rounds 0).
        // The passes may have spent their budget on the region (the gated
        // pass exhausts it drawing nothing on this record); a proposal
        // taken below clears that.
        for &noise_bars in &NOISE_BAR_LADDER {
            if !(noise_bar && merge_refit && chosen.is_none()) {
                break;
            }
            // A proposal from a lower bar that holds the region stands.
            if merged_fallback.as_ref().is_some_and(|(merged, _)| {
                ids.iter().filter(|&&t| merged.iter().any(|(s, _)| holds(s, t, positions, indices, noise_fallback.unwrap_or(1.0) * tolerance, -1.0))).count() * 20
                    >= ids.len() * 19
            }) {
                break;
            }
            if merged_fallback.is_some() && noise_fallback.is_none() {
                // The batch-39 merged proposal: not this pass's to replace.
                break;
            }
            // Planes and cylinders first, as batch 42 drew them; cones join
            // the draw only when that leaves no proposal holding the region
            // (drawn together, the cone hypotheses took the budget on census
            // record occt_crossed_cylinders_s10_d0.1_jitter-0.3 and the
            // second wall came back as a cone whose edge with the first
            // failed; drawn alone they take a jittered frustum wall).
            // The cylinder draw's carriers at this bar, carried into the cone
            // draw's dedup: the two draws never meet, and on census record
            // occt_countersunk_hole_s10_d0.05_jitter-0.3 the cylinder draw
            // found the bore (72 of its own) and the cone draw the countersink
            // (180) and the same bore as a cone, each a lone carrier.
            let mut draw_cylinders: Vec<(crate::AnalyticSurface, Vec<bool>)> = Vec::new();
            // The lone kinds are drawn one at a time, each with the whole
            // attempt budget (drawn together, the sphere of census record
            // occt_sphere_s1_d0.05_jitter-1.0 that holds 804 of 842 at six
            // bars was not among the hypotheses).
            // A gathered dust region is drawn for planes and cylinders only:
            // with the cone set, a tilted wall fit drawn as a cone joined the
            // two walls of census record occt_stepped_shaft_s10_d0.1_jitter-0.3
            // and the builder demoted every cylinder/cone edge (248 -> 313).
            let kind_sets: Vec<&[crate::SurfaceType]> = if band_declined {
                vec![&NOISE_LONE_KINDS[..1], &NOISE_LONE_KINDS[1..]]
            } else if gathered.contains(&region.id) {
                // Planes and cylinders first, cones only when that holds
                // nothing (a jittered pointed cone's wall).
                vec![&SPLIT_KINDS[..2], &SPLIT_KINDS[..]]
            } else {
                vec![&SPLIT_KINDS[..2], &SPLIT_KINDS[..]]
            };
            for kinds in kind_sets {
                if merged_fallback.as_ref().zip(noise_fallback).is_some_and(|((merged, _), bars)| {
                    bars == noise_bars
                        && ids.iter().filter(|&&t| merged.iter().any(|(s, _)| holds(s, t, positions, indices, bars * tolerance, -1.0))).count() * 20
                            >= ids.len() * 19
                }) {
                    break;
                }
                let mut wide = recognition.clone();
                wide.probe_crease_gate = false;
                wide.distance_tolerance = noise_bars * tolerance;
                let started = std::time::Instant::now();
                // Cones too: a jittered frustum wall (census record
                // kernel_frustum_s1_d0.1_jitter-0.1, 524 faceted) is one cone
                // 1.4 bars off its vertices that no pass at the bar holds.
                let budgeted = recognize_surfaces_with_attempt_budget(&mesh, &wide, SPLIT_MAX_ATTEMPTS, kinds);
                if debug {
                    eprintln!(
                        "[split] region {} tris {} noise-bar pass tol {:.3e} kinds {} in {:.2}s work {:?}",
                        region.id,
                        ids.len(),
                        wide.distance_tolerance,
                        kinds.len(),
                        started.elapsed().as_secs_f64(),
                        budgeted.as_ref().map(|(_, work)| *work).ok()
                    );
                }
                if let Ok((mut result, _)) = budgeted {
                    // The recognizer fits each component whole before it
                    // draws, whatever the kinds asked for (attempts 0): a
                    // whole fit of a kind this draw does not take is fitted
                    // again as each kind it does, cones first. On census
                    // record occt_pointed_cone_s10_d0.1_jitter-0.1 the wall's
                    // band of 74 came as a torus centred 854 above the body
                    // at every bar, and no kind of the draw took it.
                    if let [whole] = &mut result.regions[..] {
                        if whole.triangle_indices.len() == ids.len() && !kinds.contains(&whole.surface.surface_type()) {
                            let selection: Vec<usize> = (0..ids.len()).collect();
                            for &kind in kinds.iter().rev() {
                                let hint = crate::SurfaceHint::KnownType { surface_type: kind };
                                if let Ok(fit) = crate::reconstruct_surface(&mesh, &selection, &hint, &wide) {
                                    if debug {
                                        eprintln!("[split]   whole {:?} fit refitted as {:?}", whole.surface.surface_type(), kind);
                                    }
                                    whole.surface = fit.surface;
                                    break;
                                }
                            }
                        }
                    }
                    if debug {
                        for r in &result.regions {
                            let held = (0..ids.len()).filter(|&i| holds(&r.surface, ids[i], positions, indices, noise_bars * tolerance, -1.0)).count();
                            eprintln!("[split]   gathered draw fit of {} tris holds {held} of {} at {noise_bars} bars: {:?}", r.triangle_indices.len(), ids.len(), r.surface);
                        }
                    }
                    let (accepted, mut walls) = merge_refit_cylinders(&result.regions, &mesh, &recognition, tolerance, noise_bars);
                    for r in &accepted {
                        if let crate::AnalyticSurface::Cylinder(c) = r.surface {
                            if !walls.contains(&c) {
                                walls.push(c);
                            }
                        }
                    }
                    // Every cylinder and cone is supported by what it holds at
                    // the noise bar with no normal gate, largest first; a
                    // carrier three in four of whose triangles a larger one
                    // already holds is that carrier's fit tilted by the noise
                    // (four cylinders of radius 0.88-0.99 tilted up to 4 degrees
                    // beside the two walls on the record above) and is dropped,
                    // and so is one holding fewer than [`MIN_SPLIT_TRIANGLES`] of
                    // its own (the cone stage's exclusive-support rule). A lone
                    // carrier that holds 19 in 20 of the region is a proposal
                    // too (the frustum wall above).
                    let bar = noise_bars * tolerance;
                    let gathered_gate = if gathered.contains(&region.id) { GATHERED_NORMAL_GATE_DEG.to_radians().cos() } else { -1.0 };
                    // On a gathered dust region a cylinder fit of radius over
                    // the region's diagonal is a cap the recognizer drew as a
                    // cylinder (a plane is a cylinder of infinite radius): it
                    // is read as the plane through its triangles (census
                    // record kernel_cylinder_s1_d0.1_jitter-0.1: its two caps
                    // came as cylinders of radius ~1 500 and ~600 holding 30
                    // each, and no plane fit at all).
                    let accepted: Vec<crate::SurfaceRegion> = accepted
                        .into_iter()
                        .map(|mut r| {
                            // Any curved fit whose triangles all face one way
                            // (normals within GATHERED_FLAT_DEG of their mean)
                            // is a flat cap too: the recognizer draws a flat
                            // ring as a cone of near-90-degree half angle or a
                            // torus thousands off (census records
                            // occt_stepped_shaft_s1_d0.1_jitter-0.1,
                            // kernel_stepped_shaft_s1_d0.1_jitter-0.1,
                            // occt_cylinder_s1_d0.1_jitter-0.1).
                            let flat = gathered.contains(&region.id) && !r.triangle_indices.is_empty() && !matches!(r.surface, crate::AnalyticSurface::Plane(_)) && {
                                let normals: Vec<crate::Vec3> = r.triangle_indices.iter().filter_map(|&t| {
                                    let [a, b, c] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                                    (b - a).cross(c - a).normalized()
                                }).collect();
                                let sum = normals.iter().fold(crate::Vec3::new(0.0, 0.0, 0.0), |acc, &n| acc + n);
                                sum.normalized().is_some_and(|mean| normals.iter().all(|n| n.dot(mean) >= GATHERED_FLAT_DEG.to_radians().cos()))
                                    // A cone whose half angle is within
                                    // GATHERED_FLAT_CONE_DEG of 90 degrees is
                                    // a plane whatever its facets' spread: on
                                    // census record
                                    // kernel_pointed_cone_s1_d0.1_jitter-0.3 the
                                    // base of 30 came as a cone of half angle
                                    // 89.93 degrees, one of its fan's slim
                                    // triangles tilted 36 degrees by the jitter.
                                    || matches!(r.surface, crate::AnalyticSurface::Cone(cone) if cone.half_angle >= (90.0 - GATHERED_FLAT_CONE_DEG).to_radians())
                            };
                            if flat || matches!(r.surface, crate::AnalyticSurface::Cylinder(_)) {
                                let cylinder_too_big = matches!(r.surface, crate::AnalyticSurface::Cylinder(cylinder) if cylinder.radius > diagonal);
                                if gathered.contains(&region.id) && (flat || cylinder_too_big) && !r.triangle_indices.is_empty() {
                                    let (mut centroid, mut normal, mut count) = (crate::Vec3::new(0.0, 0.0, 0.0), crate::Vec3::new(0.0, 0.0, 0.0), 0.0);
                                    for &t in &r.triangle_indices {
                                        let [a, b, c] = mesh.triangles[t].map(|v| mesh.vertices[v as usize]);
                                        centroid = centroid + (a + b + c) * (1.0 / 3.0);
                                        normal = normal + (b - a).cross(c - a);
                                        count += 1.0;
                                    }
                                    if let Some(normal) = normal.normalized() {
                                        r.surface = crate::AnalyticSurface::Plane(crate::PlaneSurface { origin: centroid * (1.0 / count), normal });
                                    }
                                }
                            }
                            r
                        })
                        .collect();
                    let mut held_by: Vec<(crate::AnalyticSurface, Vec<bool>)> = accepted
                        .iter()
                        .filter(|r| match r.surface {
                            // On a gathered dust region a cylinder whose
                            // radius exceeds the region's diagonal is a cap
                            // drawn as a cylinder (census record
                            // occt_stepped_shaft_s1_d0.1_jitter-0.3: a fit of
                            // radius ~1 000 with its axis across the body held
                            // 53 of 230 and joined the two walls, which the
                            // builder demoted at every edge, 247 -> 270).
                            crate::AnalyticSurface::Cylinder(cylinder) => !gathered.contains(&region.id) || cylinder.radius <= diagonal,
                            // A gathered dust region holds its caps (the
                            // segmenter chipped them into one- and
                            // two-triangle planes), so planes are carriers
                            // there: on census record
                            // occt_stepped_shaft_s10_d0.1_jitter-0.3 the two
                            // walls held 96 and 93 of the 238 gathered and
                            // the proposal of the two was assigned 189, under
                            // 19 in 20, the caps left over.
                            crate::AnalyticSurface::Plane(_) => gathered.contains(&region.id),
                            // A torus only on the band-declined path: on
                            // census record occt_frustum_s1_d0.1_jitter-0.3
                            // the plane/cylinder draw at six bars returned a
                            // torus holding the whole frustum wall, which
                            // built as a torus (a false kind).
                            // And only a ring torus (its hole open): the
                            // spindle tori the draw fits to a jittered sphere
                            // held it whole, and the sphere rows that built
                            // through the whole-primitive path went to facets
                            // (census records kernel_sphere_s10_d0.05_jitter-1.0
                            // and three more, 0 -> 384 and 0 -> 866).
                            crate::AnalyticSurface::Torus(torus) => band_declined && torus.major_radius > torus.minor_radius,
                            // A sphere likewise only there (census records
                            // occt_sphere_s1_d0.05_jitter-1.0 and three more:
                            // one freeform region of 842-856 of the 866 the
                            // segmenter's sphere fit does not hold, a sphere
                            // drawn at six bars holds 804-807 of it, radius
                            // 2.001 against the record's 2).
                            crate::AnalyticSurface::Sphere(_) => band_declined,
                            // A cone under the cone stage's half-angle floor is
                            // a cylinder drawn as a cone; the cylinder draw's
                            // own fit of it is carried in below instead.
                            // Not on a gathered dust region (its walls are
                            // cylinders; a wall fit tilted by the noise drawn
                            // as a cone joined the two walls of census record
                            // occt_stepped_shaft_s10_d0.1_jitter-0.3 and the
                            // builder demoted every cylinder/cone edge).
                            crate::AnalyticSurface::Cone(cone) => cone.half_angle >= CONE_STAGE_MIN_HALF_ANGLE_DEG.to_radians() && (!gathered.contains(&region.id) || kinds.len() == 3),
                            _ => false,
                        })
                        .map(|r| (r.surface, (0..ids.len()).map(|i| holds(&r.surface, ids[i], positions, indices, bar, gathered_gate)).collect()))
                        .collect();
                    if kinds.len() == 2 {
                        draw_cylinders = held_by.clone();
                    } else {
                        // Only a cylinder on a cone's axis is carried in (a
                        // bore): every cylinder of that draw carried in
                        // outranked the countersink cone of census record
                        // kernel_countersunk_hole_s1_d0.005_jitter-1.0 with
                        // partial wall fits and the row went back to facets.
                        let draw_cones: Vec<crate::ConeSurface> = held_by.iter().filter_map(|(s, _)| if let crate::AnalyticSurface::Cone(c) = s { Some(*c) } else { None }).collect();
                        for (surface, held) in &draw_cylinders {
                            let crate::AnalyticSurface::Cylinder(cylinder) = surface else { continue };
                            let coaxial = draw_cones.iter().any(|cone| {
                                let d = cylinder.axis_origin - cone.apex;
                                cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
                                    && (d - cone.axis * d.dot(cone.axis)).length() <= bar
                            });
                            if coaxial && !held_by.iter().any(|(s, _)| same_cylinder(s, surface, tolerance)) {
                                held_by.push((*surface, held.clone()));
                            }
                        }
                    }
                    let count = |held: &Vec<bool>| held.iter().filter(|&&h| h).count();
                    let is_cone = |s: &crate::AnalyticSurface| matches!(s, crate::AnalyticSurface::Cone(_));
                    held_by.sort_by(|a, b| is_cone(&a.0).cmp(&is_cone(&b.0)).then(count(&b.1).cmp(&count(&a.1))));
                    let mut merged: Vec<(crate::AnalyticSurface, usize)> = Vec::new();
                    let mut taken = vec![false; ids.len()];
                    // A cylinder under the carrier floor that is coaxial with
                    // a cone of the draw waits: it may be that cone's bore.
                    let mut bores: Vec<(crate::AnalyticSurface, Vec<bool>, usize)> = Vec::new();
                    for (surface, held) in &held_by {
                        let support = count(held);
                        let own = (0..ids.len()).filter(|&i| held[i] && !taken[i]).count();
                        if debug {
                            eprintln!("[split] noise-bar carrier holds {support} at {noise_bars} bars, {own} its own ({})", match surface { crate::AnalyticSurface::Cylinder(_) => "cylinder", crate::AnalyticSurface::Cone(_) => "cone", crate::AnalyticSurface::Torus(_) => "torus", crate::AnalyticSurface::Sphere(_) => "sphere", crate::AnalyticSurface::Plane(_) => "plane", _ => "other" });
                        }
                        // A gathered region's carriers are the small faces
                        // of a small body (caps of 30-60 triangles), under
                        // the split's carrier floor.
                        let floor = if gathered.contains(&region.id) { GATHERED_CARRIER_TRIANGLES } else { MIN_SPLIT_TRIANGLES };
                        if own * 4 < support || own < floor {
                            if own >= MIN_BORE_TRIANGLES && own * 4 >= support {
                                if let crate::AnalyticSurface::Cylinder(_) = surface {
                                    bores.push((*surface, held.clone(), support));
                                }
                            }
                            walls.retain(|w| *surface != crate::AnalyticSurface::Cylinder(*w));
                            continue;
                        }
                        for i in 0..ids.len() {
                            taken[i] |= held[i];
                        }
                        merged.push((*surface, support));
                    }
                    // The cone stage's bore, from the draw: a cylinder on a
                    // kept cone's axis (axes within 2 degrees, the cylinder's
                    // axis line within the bar of the apex) holding at least
                    // [`MIN_BORE_TRIANGLES`] of its own joins the proposal.
                    // On census record occt_countersunk_hole_s10_d0.05_jitter-0.3
                    // the cone held 180 of 252 and the bore 29, under the
                    // carrier floor of 64.
                    // A gathered region whose draw holds one plane and
                    // nothing else is a pointed cone's base, its wall a
                    // fan the recognizer drew nothing on (census record
                    // kernel_pointed_cone_s1_d0.1_jitter-0.1: 96 attempts,
                    // 0 rounds): the cone is constructed from the base and
                    // the tip, axis on the base normal through the base's
                    // centre, apex at the wall vertex farthest from the
                    // base when it lies on that axis, half angle the mean
                    // over the wall's vertices, and kept when it holds the
                    // rest of the region at the bar.
                    // Only in the cone stage and only when its draw fitted no
                    // cone: where the recognizer draws the wall, its fit
                    // stands. The base is the plane/cylinder draw's lone
                    // plane when the cone draw fitted nothing at all.
                    let base_plane = if gathered.contains(&region.id) && kinds.len() == 3 && !merged.iter().any(|(s, _)| matches!(s, crate::AnalyticSurface::Cone(_))) {
                        let draw_planes: Vec<crate::AnalyticSurface> = draw_cylinders.iter().map(|(s, _)| *s).filter(|s| matches!(s, crate::AnalyticSurface::Plane(_))).collect();
                        match (&merged[..], &draw_planes[..]) {
                            ([(plane @ crate::AnalyticSurface::Plane(_), _)], _) => Some(*plane),
                            ([], [plane]) => Some(*plane),
                            _ => None,
                        }
                    } else {
                        None
                    };
                    if let Some(base_surface) = base_plane {
                        if merged.is_empty() {
                            let support = (0..ids.len()).filter(|&i| holds(&base_surface, ids[i], positions, indices, bar, gathered_gate)).count();
                            merged.push((base_surface, support));
                        }
                        if let crate::AnalyticSurface::Plane(plane) = base_surface {
                            let base_tris: Vec<usize> = (0..ids.len()).filter(|&i| holds(&base_surface, ids[i], positions, indices, bar, gathered_gate)).map(|i| ids[i]).collect();
                            let wall_tris: Vec<usize> = ids.iter().copied().filter(|t| !base_tris.contains(t)).collect();
                            let unique = |tris: &[usize]| -> Vec<crate::Vec3> {
                                let mut seen = std::collections::BTreeSet::<u32>::new();
                                tris.iter()
                                    .flat_map(|&t| (0..3).map(move |m| indices[3 * t + m]))
                                    .filter(|v| seen.insert(*v))
                                    .map(|v| {
                                        let at = v as usize * 3;
                                        crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
                                    })
                                    .collect()
                            };
                            let (base, wall) = (unique(&base_tris), unique(&wall_tris));
                            if !base.is_empty() && wall_tris.len() >= GATHERED_CARRIER_TRIANGLES {
                                let centre = base.iter().fold(crate::Vec3::new(0.0, 0.0, 0.0), |acc, &p| acc + p) * (1.0 / base.len() as f64);
                                let tip = wall.iter().copied().max_by(|a, b| (*a - centre).dot(plane.normal).abs().total_cmp(&(*b - centre).dot(plane.normal).abs()));
                                if let Some(tip) = tip {
                                    let h = (tip - centre).dot(plane.normal);
                                    let axis = if h < 0.0 { plane.normal } else { plane.normal * -1.0 };
                                    let apex = centre + plane.normal * h;
                                    let on_axis = ((tip - centre) - plane.normal * h).length() <= bar * 4.0;
                                    let (mut sum, mut n) = (0.0, 0.0);
                                    for p in &wall {
                                        let v = *p - apex;
                                        let height = v.dot(axis);
                                        if height > bar {
                                            sum += ((v - axis * height).length() / height).atan();
                                            n += 1.0;
                                        }
                                    }
                                    if on_axis && n > 0.0 {
                                        let cone = crate::AnalyticSurface::Cone(crate::ConeSurface { apex, axis, half_angle: sum / n });
                                        let held = wall_tris.iter().filter(|&&t| holds(&cone, t, positions, indices, bar, gathered_gate)).count();
                                        if debug {
                                            eprintln!("[split] gathered region {} constructed cone holds {held} of {} wall tris: {:?}", region.id, wall_tris.len(), cone);
                                        }
                                        if held * 20 >= wall_tris.len() * 19 {
                                            merged.push((cone, held));
                                        }
                                    }
                                }
                            }
                        }
                    }
                    let mut strips_found: Vec<(u32, Vec<usize>)> = Vec::new();
                    for (surface, held, support) in bores {
                        let crate::AnalyticSurface::Cylinder(cylinder) = surface else { continue };
                        let coaxial = merged.iter().any(|(s, _)| match s {
                            crate::AnalyticSurface::Cone(cone) => {
                                let d = cylinder.axis_origin - cone.apex;
                                cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
                                    && (d - cone.axis * d.dot(cone.axis)).length() <= bar
                            }
                            _ => false,
                        });
                        if !coaxial {
                            continue;
                        }
                        let own = (0..ids.len()).filter(|&i| held[i] && !taken[i]).count();
                        if own < MIN_BORE_TRIANGLES {
                            continue;
                        }
                        if debug {
                            eprintln!("[split] noise-bar cone takes the draw's coaxial bore holding {support}, {own} its own");
                        }
                        for i in 0..ids.len() {
                            taken[i] |= held[i];
                        }
                        merged.push((surface, support));
                        walls.push(cylinder);
                    }
                    // A lone cone with no draw bore beside it takes a bore
                    // constructed on its axis through the triangles it does
                    // not hold (the cone stage's `coaxial_bore`, at this
                    // bar): on census record
                    // kernel_countersunk_hole_s1_d0.1_jitter-0.1 the draws fit
                    // the countersink whole (92 of 118) but the bore only as
                    // pieces of 7-14, under the bore floor.
                    let lone_cone = {
                        let cones: Vec<crate::ConeSurface> = merged.iter().filter_map(|(s, _)| if let crate::AnalyticSurface::Cone(c) = s { Some(*c) } else { None }).collect();
                        match cones[..] {
                            [cone] if !merged.iter().any(|(s, _)| match s {
                                crate::AnalyticSurface::Cylinder(cylinder) => {
                                    let d = cylinder.axis_origin - cone.apex;
                                    cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos() && (d - cone.axis * d.dot(cone.axis)).length() <= bar
                                }
                                _ => false,
                            }) => Some(cone),
                            _ => None,
                        }
                    };
                    if debug && lone_cone.is_some() {
                        eprintln!("[split] noise-bar proposal of {} carriers has a lone cone and no bore", merged.len());
                    }
                    if let Some(cone) = lone_cone {
                        // The bore's pieces between the strips beside the
                        // region are one ring (the strips the bore search
                        // below takes) when the bore alone is in pieces under
                        // the bore floor (kernel_countersunk_hole_s1_d0.1_jitter-0.1);
                        // tried first without them, as a strip the bridge
                        // takes can fail the ring (bridged first, census
                        // record kernel_countersunk_hole_s1_d0.1_jitter-0.3
                        // stayed facets).
                        let bridges = |bore: &crate::AnalyticSurface| {
                            bore_strip_regions(bore, &cone, &ids, region.id, &*segmentation, &edges, positions, indices, NOISE_ASSIGN_SLACK_BARS * bar, -1.0, cos_gate, NOISE_STRIP_NEAR_BARS * bar, Some(NOISE_BAND_TRIANGLES), false)
                        };
                        let constructed = coaxial_bore(&cone, &ids, positions, indices, bar, cos_gate, None)
                            .or_else(|| coaxial_bore(&cone, &ids, positions, indices, bar, cos_gate, Some(&bridges)));
                        if let Some((bore, held, _)) = constructed {
                            if debug {
                                eprintln!("[split] noise-bar cone takes a constructed coaxial bore {bore:?} holding {held}");
                            }
                            if let crate::AnalyticSurface::Cylinder(cylinder) = bore {
                                walls.push(cylinder);
                            }
                            merged.push((bore, held));
                        }
                    }
                    // A bore's ring (any kept cylinder on a kept cone's axis:
                    // the one kept above under the floor, or a draw cylinder
                    // above it) is broken by triangles of the small regions
                    // beside this one (census record
                    // occt_countersunk_hole_s1_d0.1_jitter-0.3: bore pieces of
                    // 10, 34, 26, 1 and 2, every one within 3e-2 of the bore;
                    // the six full-height strips of its coarse bore are plane
                    // regions of 2-3 triangles); the strips it holds join the
                    // split as in the cone stage. Held with no normal gate as
                    // everything at this bar, but a strip faces the bore only
                    // within the grower's gate: with no gate the cap fragment
                    // the strips reach faced it too and the search was
                    // declined whole.
                    let cones: Vec<crate::ConeSurface> = merged.iter().filter_map(|(s, _)| if let crate::AnalyticSurface::Cone(c) = s { Some(*c) } else { None }).collect();
                    // A bore fitted apart from its cone has its axis a
                    // fraction of a degree off the cone's (0.66 degrees on
                    // census record occt_countersunk_hole_s10_d0.05_jitter-1.0,
                    // against the builder's 0.57-degree gate on a cylinder/cone
                    // edge); it is snapped onto the cone's axis line, as two
                    // crossing cylinders are snapped to one centre below.
                    for (surface, _) in merged.iter_mut() {
                        let crate::AnalyticSurface::Cylinder(cylinder) = surface else { continue };
                        let Some(cone) = cones.iter().find(|cone| {
                            let d = cylinder.axis_origin - cone.apex;
                            cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
                                && (d - cone.axis * d.dot(cone.axis)).length() <= bar
                        }) else { continue };
                        let d = cylinder.axis_origin - cone.apex;
                        let snapped = crate::CylinderSurface {
                            axis_origin: cone.apex + cone.axis * d.dot(cone.axis),
                            axis: if cone.axis.dot(cylinder.axis) >= 0.0 { cone.axis } else { cone.axis * -1.0 },
                            radius: cylinder.radius,
                        };
                        for w in walls.iter_mut() {
                            if *w == *cylinder {
                                *w = snapped;
                            }
                        }
                        *cylinder = snapped;
                    }
                    for (surface, _) in &merged {
                        let crate::AnalyticSurface::Cylinder(cylinder) = surface else { continue };
                        let Some(cone) = cones.iter().find(|cone| {
                            let d = cylinder.axis_origin - cone.apex;
                            cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
                                && (d - cone.axis * d.dot(cone.axis)).length() <= bar
                        }) else { continue };
                        if let Some(found) = bore_strip_regions(surface, cone, &ids, region.id, &*segmentation, &edges, positions, indices, NOISE_ASSIGN_SLACK_BARS * bar, -1.0, cos_gate, NOISE_STRIP_NEAR_BARS * bar, Some(NOISE_BAND_TRIANGLES), debug) {
                            if debug {
                                eprintln!("[split] noise-bar bore takes {} strips", found.len());
                            }
                            strips_found.extend(found);
                        }
                    }
                    walls.retain(|w| merged.iter().any(|(s, _)| *s == crate::AnalyticSurface::Cylinder(*w)));
                    let held_all = |merged: &Vec<(crate::AnalyticSurface, usize)>, bars: f64| {
                        ids.iter().filter(|&&t| merged.iter().any(|(s, _)| holds(s, t, positions, indices, bars * tolerance, gathered_gate))).count()
                    };
                    // A band-declined region is one surface (its normals
                    // cover no band, so no wall meets another in it): only
                    // the carrier holding most of it is a proposal (census
                    // record occt_sphere_s1_d0.05_jitter-1.0: two partial
                    // sphere fits of 384 and 337 at three bars made a
                    // two-carrier proposal that held 623 of 842, and the
                    // lone sphere of 804 at six bars never proposed).
                    if band_declined {
                        merged.truncate(1);
                    }
                    // A lone carrier proposes when it holds 19 in 20 within
                    // the assignment slack (the assignment below reaches
                    // that far): the torus of census record
                    // kernel_torus_s1_d0.1_jitter-1.0 holds 2 787 of 2 977 at
                    // six bars, 93.6 in 100.
                    // A gathered dust region joins several faces' dust, so a
                    // lone carrier is no proposal there: on census record
                    // occt_cylinder_s1_d0.1_jitter-0.3 the lone wall took the
                    // caps' dust with it and the builder demoted it at the cap
                    // chips (120 -> 145 faceted).
                    // Since the gathered normal gate (batch 58) a curved lone
                    // carrier holds no cap dust facing along it, and one that
                    // holds 19 in 20 of a gathered region at the noise bar
                    // itself, not only within the slack, is the region's one
                    // face (census record kernel_annular_tube_s1_d0.1_jitter-0.1:
                    // the bore's 17 chips gathered into 64, all held by one
                    // cylinder at three bars, and the gather was declined).
                    let lone_holds = merged.len() == 1
                        && (!gathered.contains(&region.id)
                            || matches!(merged[0].0, crate::AnalyticSurface::Plane(_))
                            || held_all(&merged, noise_bars) * 20 >= ids.len() * 19)
                        && held_all(&merged, noise_bars * NOISE_ASSIGN_SLACK_BARS) * 20 >= ids.len() * 19;
                    // A higher bar's proposal replaces a lower one's only when
                    // it holds more of the region.
                    let better = merged_fallback
                        .as_ref()
                        .zip(noise_fallback)
                        .is_none_or(|((before, _), bars)| held_all(&merged, noise_bars) > held_all(before, bars));
                    if (merged.len() >= 2 || lone_holds) && better {
                        merged_fallback = Some((merged, walls));
                        noise_fallback = Some(noise_bars);
                        noise_bridges = strips_found;
                    }
                }
            }
        }
        // A region the passes decline takes the merged proposal, as does one
        // whose chosen carriers hold less than 19 in 20 of it at the bar:
        // the assignment below holds at most what the carriers hold, so
        // such a choice is declined as it is (two fits of 355 and 262
        // triangles hold 1 480 of the 1 889 of census record
        // kernel_crossed_cylinders_s1_d0.1_jitter-0.1). A lone cone
        // ([`LONE_SHARE`]) is not a choice this replaces.
        let mut merged_walls: Vec<crate::CylinderSurface> = Vec::new();
        let mut noise_region: Option<f64> = None;
        let chosen_short = chosen.as_ref().is_some_and(|carriers: &Vec<(crate::AnalyticSurface, usize)>| {
            !(carriers.len() == 1 && lone_cone)
                && ids.iter().filter(|&&t| carriers.iter().any(|(s, _)| holds(s, t, positions, indices, tolerance, cos_gate))).count() * 20
                    < ids.len() * 19
        });
        if chosen.is_none() || chosen_short {
            if let Some((merged, walls)) = merged_fallback.take() {
                if debug {
                    eprintln!("[split] region {} takes the merged proposal of {} carriers", region.id, merged.len());
                }
                chosen = Some(merged);
                merged_walls = walls;
                noise_region = noise_fallback;
                out_of_budget = false;
                bore_bridges = if noise_fallback.is_some() { std::mem::take(&mut noise_bridges) } else { Vec::new() };
            }
        }
        if out_of_budget {
            report.budget_hit_regions += 1;
            continue;
        }
        let Some(mut carriers) = chosen else {
            if band_declined {
                report.band_declined_regions += 1;
            } else {
                report.declined_regions += 1;
            }
            continue;
        };
        // A region on the merged proposal is assigned at the bar the walls
        // were refitted at (read before the crossing snap below moves the
        // carriers).
        let merged_carrier = !merged_walls.is_empty()
            && carriers.iter().any(|(c, _)| merged_walls.iter().any(|w| *c == crate::AnalyticSurface::Cylinder(*w)))
            && carriers.iter().all(|(c, _)| matches!(c, crate::AnalyticSurface::Cylinder(_)));
        if (bore_strips || noise_region.is_some()) && carriers.len() == 2 {
            for (r, tris) in std::mem::take(&mut bore_bridges) {
                if strips.insert(r) {
                    strip_triangles.push((r, tris));
                }
            }
        }
        // One cone fitted twice (two pieces of a frustum's one wall, their
        // noisy apexes 0.023 apart at chord 0.1) is two carriers the builder
        // cannot join: their shared edge is no coaxial ring, and on the
        // radius-1 d0.1 jitter-0.1 frustum that demoted three more triangles
        // than leaving the region as it was. Such a region is left as it is.
        if twin_cones(&carriers) {
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
        let one_bar = tolerance;
        let tolerance = if let Some(bars) = noise_region {
            bars * tolerance
        } else if merged_carrier {
            MERGE_REFIT_FIT_BARS * tolerance
        } else {
            tolerance
        };
        // A noise-bar region is assigned with no normal gate (its jittered
        // facet normals are no evidence); the region holds no cap.
        let noise_region = noise_region.is_some();
        let cos_gate = if gathered.contains(&region.id) { GATHERED_NORMAL_GATE_DEG.to_radians().cos() } else if noise_region { -1.0 } else { cos_gate };
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
        let fit_within = |p: [crate::Vec3; 3], cos_gate: f64, tolerance: f64| -> Option<(f64, usize)> {
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
        let fit = |p: [crate::Vec3; 3]| fit_within(p, cos_gate, tolerance);
        // A noise-bar region's strips are held within the assignment slack
        // (the strips the segmenter cut from a jittered ring sit just past
        // the bar as the ring's own triangles do).
        let strip_bar = if noise_region { NOISE_ASSIGN_SLACK_BARS * tolerance } else { tolerance };
        let fit_strip = |p: [crate::Vec3; 3]| fit_within(p, cos_gate, strip_bar);
        // Row 7d: a strip joins the split only when the accepted carriers
        // hold every one of its triangles, within the bar and the grower's
        // gate; one triangle they do not hold leaves the region as it was.
        // Under `lone_cone` a strip the lone cone does not
        // hold is that cone's neighbour (a bore piece meeting the
        // countersink on a coaxial ring) and stays as it is.
        let lone = carriers.len() == 1 && lone_cone;
        if lone {
            strip_triangles.retain(|(_, tris)| tris.iter().all(|&t| fit(points(t)).is_some()));
        }
        if let Some((strip, tris)) = strip_triangles.iter().find(|(_, tris)| !tris.iter().all(|&t| fit_strip(points(t)).is_some())) {
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
        let on_strips: std::collections::HashSet<usize> = strip_triangles.iter().flat_map(|(_, tris)| tris.iter().copied()).collect();
        let mut owner: Vec<(usize, usize)> = ids
            .iter()
            .filter_map(|&t| (if on_strips.contains(&t) { fit_strip(points(t)) } else { fit(points(t)) }).map(|(_, c)| (t, c)))
            .collect();
        // A noise-bar region's triangle that no carrier holds at the bar
        // but exactly one holds within NOISE_ASSIGN_SLACK_BARS of it goes to
        // that carrier: at six bars the jitter leaves a few wall triangles
        // just past the bar on their own face (census records
        // occt_countersunk_hole_s1_d0.05_jitter-1.0 and
        // occt_countersunk_hole_s1_d0.02_jitter-1.0: 18 and 17 triangles at
        // 1.05 to 1.3 bars of the bore or the cone, 1.3 off the other, and
        // the proposals held 264 of 282 and 234 of 251, under 19 in 20).
        if noise_region {
            let claimed: std::collections::HashSet<usize> = owner.iter().map(|o| o.0).collect();
            let slack = NOISE_ASSIGN_SLACK_BARS * tolerance;
            for &t in &ids {
                if claimed.contains(&t) {
                    continue;
                }
                let p = points(t);
                let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                let facing = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
                let within: Vec<usize> = carriers
                    .iter()
                    .enumerate()
                    .filter(|(_, (s, _))| p.iter().all(|&q| s.signed_distance(q).abs() <= slack))
                    // On a gathered region the slack keeps the region's
                    // normal gate (a coarse cap's rim triangles lie on the
                    // wall cylinder too).
                    .filter(|(_, (s, _))| {
                        cos_gate <= -1.0 || s.normal_at(centroid).zip(facing).is_some_and(|(n, m)| n.dot(m).abs() >= cos_gate)
                    })
                    .map(|(c, _)| c)
                    .collect();
                if let [c] = within[..] {
                    if debug {
                        let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                        let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
                        let angle = carriers[c].0.normal_at(centroid).zip(normal).map(|(n, m)| n.dot(m).abs().min(1.0).acos().to_degrees());
                        let worst = p.iter().map(|&q| carriers[c].0.signed_distance(q).abs()).fold(0.0_f64, f64::max);
                        eprintln!("[split]     slack tri {t} (region {}) to carrier {c} at {:.2} bars, normal {:?} degrees off, centroid ({:.3},{:.3},{:.3})", segmentation.triangle_region_ids[t], worst / tolerance, angle.map(|a| (a * 10.0).round() / 10.0), centroid.x, centroid.y, centroid.z);
                    }
                    owner.push((t, c));
                }
            }
        }
        let assigned = owner.len();
        if debug {
            for (c, (surface, support)) in carriers.iter().enumerate() {
                eprintln!("[split]   carrier {c} support {support} {surface:?}");
            }
            eprintln!("[split]   assigned {assigned} of {}", ids.len());
            if noise_region {
                let claimed: std::collections::HashSet<usize> = owner.iter().map(|o| o.0).collect();
                for &t in ids.iter().filter(|t| !claimed.contains(t)) {
                    let p = points(t);
                    let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                    let dists: Vec<String> = carriers.iter().map(|(s, _)| format!("{:.3e}", p.iter().map(|&q| s.signed_distance(q).abs()).fold(0.0_f64, f64::max))).collect();
                    let station = carriers.iter().find_map(|(s, _)| if let crate::AnalyticSurface::Cone(cone) = s { Some((centroid - cone.apex).dot(cone.axis)) } else { None });
                    eprintln!("[split]     unassigned tri {t} (region {}) centroid ({:.3},{:.3},{:.3}) station {:?} worst distance to carriers {:?} bar {:.3e}", segmentation.triangle_region_ids[t], centroid.x, centroid.y, centroid.z, station.map(|s| (s * 1000.0).round() / 1000.0), dists, tolerance);
                }
            }
        }
        let share = if lone && lone_share { LONE_SHARE } else { (19, 20) };
        if (carriers.len() < 2 && !lone) || assigned * share.1 < ids.len() * share.0 {
            report.declined_regions += 1;
            continue;
        }
        // The region holds no cap: the grower walled every cap off at its
        // rim. So a remaining region triangle within the bar of a carrier is
        // a jittered wall sliver (see below) and goes to that carrier.
        let claimed: std::collections::HashSet<usize> = owner.iter().map(|o| o.0).collect();
        for &t in &ids {
            if !claimed.contains(&t) {
                if let Some((_, c)) = fit_within(points(t), -1.0, tolerance) {
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
        let sliver_triangles = if noise_region { NOISE_SLIVER_TRIANGLES } else { SLIVER_TRIANGLES };
        let sliver_group_triangles = if noise_region { NOISE_SLIVER_GROUP_TRIANGLES } else { SLIVER_GROUP_TRIANGLES };
        let mut small = std::collections::BTreeMap::<u32, Vec<usize>>::new();
        for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
            if r != region.id && segmentation.regions.get(r as usize).is_some_and(|other| other.triangle_count < sliver_triangles) {
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
                            if debug && enclosed {
                                let p = points(t);
                                eprintln!("[split]   sliver group from region {} at {:.3},{:.3},{:.3} not enclosed: borders region {} ({} tris)", segmentation.triangle_region_ids[t], p[0].x, p[0].y, p[0].z, segmentation.triangle_region_ids[n], segmentation.regions.get(segmentation.triangle_region_ids[n] as usize).map_or(0, |r| r.triangle_count));
                            }
                            enclosed = false;
                        }
                    }
                }
            }
            if !enclosed {
                continue;
            }
            // A noise-bar region's slivers are held within the assignment
            // slack, as its strips are (the dust on the jitter-1.0 tori sits
            // up to 1.1 bars off the torus).
            let fits: Option<Vec<(usize, usize)>> = group
                .iter()
                .map(|&t| fit_within(points(t), -1.0, strip_bar).map(|(_, c)| (t, c)))
                .collect();
            let Some(fits) = fits else {
                if debug {
                    let (t, p) = group.iter().map(|&t| (t, points(t))).find(|(_, p)| fit_within(*p, -1.0, strip_bar).is_none()).unwrap();
                    let dists: Vec<String> = carriers.iter().map(|(s, _)| format!("{:.3e}", p.iter().map(|&q| s.signed_distance(q).abs()).fold(0.0_f64, f64::max))).collect();
                    eprintln!("[split]   sliver group of {} from region {} not held: tri {t} at {:.3},{:.3},{:.3} worst distance to carriers {:?} bar {:.3e}", group.len(), segmentation.triangle_region_ids[group[0]], p[0].x, p[0].y, p[0].z, dists, strip_bar);
                }
                continue;
            };
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
            // In a region a merged wall carries, a group whose vertices spread
            // along its carrier's axis by more than the bar is no cap either, whatever
            // its tilt: a cap is fanned from a rim at one axial station, while
            // a fold the jitter raised in a wall (a 3-triangle plane at 88
            // degrees spanning 0.33 along the z wall of census record
            // kernel_crossed_cylinders_s1_d0.05_jitter-0.3) runs along it, and
            // left as a plane region it failed the wall's edge.
            let along_axis = merged_carrier && {
                let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
                for &(t, c) in &fits {
                    let (origin, axis) = match carriers[c].0 {
                        crate::AnalyticSurface::Cylinder(cylinder) => (cylinder.axis_origin, cylinder.axis),
                        crate::AnalyticSurface::Cone(cone) => (cone.apex, cone.axis),
                        _ => continue,
                    };
                    for q in points(t) {
                        let s = (q - origin).dot(axis);
                        low = low.min(s);
                        high = high.max(s);
                    }
                }
                high - low > tolerance
            };
            // In a noise-bar region neither rule reads: the jitter tilts
            // dust and spreads a cap's rim along the axis past the bar
            // (0.04 against 0.024 on the record above). There a group joins
            // when every triangle's centroid lies on its carrier at the
            // bar: a cap fanned from its rim has its vertices on the wall
            // and its centroids well inside it, dust on the wall has both.
            let centroids_on = noise_region
                && fits.iter().all(|&(t, c)| {
                    let p = points(t);
                    carriers[c].0.signed_distance((p[0] + p[1] + p[2]) * (1.0 / 3.0)).abs() <= tolerance
                });
            let joins = if noise_region { centroids_on } else { group.len() <= 2 || tilt <= SLIVER_GROUP_TILT_DEG || along_axis };
            if group.len() < sliver_group_triangles && joins {
                owner.extend(fits);
            } else if debug {
                eprintln!("[split]   sliver group of {} from region {} not joined: centroids on {centroids_on}, group floor {sliver_group_triangles}", group.len(), segmentation.triangle_region_ids[group[0]]);
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
        // A noise-bar carrier is refitted over every triangle it holds (the
        // region's, the strips', the bands' and the slivers'): the draw's fit
        // came from a fraction of its face at the bar (census record
        // occt_countersunk_hole_s1_d0.02_jitter-1.0: the bore from 31 of the
        // 84 triangles it holds, radius 1.036 against the record's 1.0, past
        // its chord bar once built). A refit of the carrier's own kind
        // replaces it, and a cylinder on a kept cone's axis is snapped back
        // onto the refitted cone's axis line (the snap above).
        // Two crossing cylinders are not refitted: each was refitted as one
        // wall over its fits' union at two bars and snapped to one exact
        // crossing above, and refitted apart over what they hold they come
        // out of it (census record occt_crossed_cylinders_s10_d0.1_jitter-0.3,
        // volume error 9.7e-4 -> 1.7e-3 with the refit).
        let crossing_pair = matches!(
            &carriers[..],
            [(crate::AnalyticSurface::Cylinder(first), _), (crate::AnalyticSurface::Cylinder(second), _)]
                if first.axis.cross(second.axis).length() > 0.1
        );
        if noise_region && !crossing_pair {
            let mut wide = recognition.clone();
            wide.probe_crease_gate = false;
            // At the assignment slack: the strips and slivers the carrier
            // holds lie up to that far off it.
            wide.distance_tolerance = strip_bar;
            let refits: Vec<Option<crate::AnalyticSurface>> = (0..carriers.len())
                .map(|c| {
                    let members: Vec<usize> = owner.iter().filter(|o| o.1 == c).map(|o| o.0).collect();
                    // The carrier's triangles as a mesh of their own (the
                    // region's mesh above holds neither strips nor slivers).
                    let mut remap = std::collections::HashMap::<u32, u32>::new();
                    let mut vertices = Vec::new();
                    let mut triangles = Vec::new();
                    for &t in &members {
                        let mut tri = [0u32; 3];
                        for k in 0..3 {
                            let v = indices[3 * t + k];
                            tri[k] = *remap.entry(v).or_insert_with(|| {
                                let at = v as usize * 3;
                                vertices.push(crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2]));
                                (vertices.len() - 1) as u32
                            });
                        }
                        triangles.push(tri);
                    }
                    let own = Mesh::new(vertices, triangles);
                    let all: Vec<usize> = (0..members.len()).collect();
                    let hint = crate::SurfaceHint::InitialGuess { surface: carriers[c].0, trust: crate::MetadataTrust::InitialGuess };
                    let refit = crate::reconstruct_surface(&own, &all, &hint, &wide);
                    if debug {
                        eprintln!("[split]   carrier {c} refit over {} tris: {}", members.len(), refit.as_ref().map(|f| format!("{:?} deviation {:.3e}", f.surface, f.metrics.max_error)).unwrap_or_else(|e| format!("{e:?}")));
                    }
                    // A refitted plane passes through the mean of the
                    // carrier's vertices (its least-squares offset for the
                    // fitted normal): the fit reached by refinement from the
                    // carrier sits off it, and on census record
                    // occt_cylinder_s10_d0.005_jitter-1.0 the gathered base,
                    // fanned from its 63 rim vertices, refitted 0.016 above
                    // their mean (volume error 1.9e-4 -> 5.2e-4).
                    let refit = refit.ok().map(|f| match f.surface {
                        crate::AnalyticSurface::Plane(plane) if !own.vertices.is_empty() => {
                            let mean = own.vertices.iter().fold(crate::Vec3::new(0.0, 0.0, 0.0), |sum, &q| sum + q) * (1.0 / own.vertices.len() as f64);
                            crate::AnalyticSurface::Plane(crate::PlaneSurface { origin: mean, normal: plane.normal })
                        }
                        surface => surface,
                    });
                    refit.filter(|r| std::mem::discriminant(r) == std::mem::discriminant(&carriers[c].0))
                })
                .collect();
            for (c, refit) in refits.into_iter().enumerate() {
                if let Some(refit) = refit {
                    carriers[c].0 = refit;
                }
            }
            let cones: Vec<crate::ConeSurface> = carriers
                .iter()
                .filter_map(|(c, _)| if let crate::AnalyticSurface::Cone(cone) = c { Some(*cone) } else { None })
                .collect();
            for (surface, _) in carriers.iter_mut() {
                let crate::AnalyticSurface::Cylinder(cylinder) = surface else { continue };
                let Some(cone) = cones.iter().find(|cone| {
                    let d = cylinder.axis_origin - cone.apex;
                    cone.axis.dot(cylinder.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
                        && (d - cone.axis * d.dot(cone.axis)).length() <= tolerance
                }) else {
                    continue;
                };
                let d = cylinder.axis_origin - cone.apex;
                cylinder.axis_origin = cone.apex + cone.axis * d.dot(cone.axis);
                cylinder.axis = if cone.axis.dot(cylinder.axis) >= 0.0 { cone.axis } else { cone.axis * -1.0 };
            }
        }
        // A gathered region of one cone and one plane is a pointed cone or a
        // frustum's wall with its base: the cone's axis is snapped onto the
        // plane's normal through the base's centroid and its half angle
        // refitted about that axis (census record
        // kernel_pointed_cone_s10_d0.05_jitter-0.3: the coarse jittered
        // wall's fit leaned 5 degrees off the base normal and the builder
        // rejected the base ring as no revolve ring, station spread 3.7).
        if gathered.contains(&region.id) {
            let cones: Vec<usize> = (0..carriers.len()).filter(|&c| matches!(carriers[c].0, crate::AnalyticSurface::Cone(_))).collect();
            let planes: Vec<usize> = (0..carriers.len()).filter(|&c| matches!(carriers[c].0, crate::AnalyticSurface::Plane(_))).collect();
            if let ([k], [q]) = (&cones[..], &planes[..]) {
                if let (crate::AnalyticSurface::Cone(cone), crate::AnalyticSurface::Plane(plane)) = (carriers[*k].0, carriers[*q].0) {
                    let axis = if plane.normal.dot(cone.axis) >= 0.0 { plane.normal } else { plane.normal * -1.0 };
                    if axis.dot(cone.axis) >= GATHERED_CONE_SNAP_DEG.to_radians().cos() {
                        // Unique vertices: a cap fanned from its rim repeats
                        // rim vertices unevenly across its triangles.
                        let unique = |c: usize| -> Vec<crate::Vec3> {
                            let mut seen = std::collections::BTreeSet::<u32>::new();
                            owner
                                .iter()
                                .filter(|o| o.1 == c)
                                .flat_map(|o| (0..3).map(move |m| indices[3 * o.0 + m]))
                                .filter(|v| seen.insert(*v))
                                .map(|v| {
                                    let at = v as usize * 3;
                                    crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
                                })
                                .collect()
                        };
                        let base = unique(*q);
                        let wall = unique(*k);
                        if !base.is_empty() && !wall.is_empty() {
                            let centroid = base.iter().fold(crate::Vec3::new(0.0, 0.0, 0.0), |acc, &p| acc + p) * (1.0 / base.len() as f64);
                            // The apex at the height of the wall's farthest
                            // vertex from the base along the axis (a pointed
                            // cone's tip); a frustum keeps its fitted apex
                            // height.
                            let tip = wall.iter().map(|&p| (p - centroid).dot(axis)).fold(0.0_f64, |m, h| if h.abs() > m.abs() { h } else { m });
                            let fitted = (cone.apex - centroid).dot(axis);
                            let height = if (fitted - tip).abs() <= tolerance * 4.0 || wall.iter().any(|&p| ((p - centroid).dot(axis) - tip).abs() <= tolerance && ((p - centroid) - axis * (p - centroid).dot(axis)).length() <= tolerance * 4.0) { tip } else { fitted };
                            let apex = centroid + axis * height;
                            let (mut sum, mut n) = (0.0, 0.0);
                            for p in &wall {
                                let v = *p - apex;
                                let h = v.dot(axis);
                                let r = (v - axis * h).length();
                                if h.abs() > 1e-12 {
                                    sum += (r / h.abs()).atan();
                                    n += 1.0;
                                }
                            }
                            if n > 0.0 {
                                let snapped = crate::ConeSurface { apex, axis, half_angle: sum / n };
                                if debug {
                                    eprintln!("[split]   cone snapped onto the base normal: {:?}", snapped);
                                }
                                carriers[*k].0 = crate::AnalyticSurface::Cone(snapped);
                            }
                        }
                    }
                }
            }
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
                if noise_region && piece.len() < SLIVER_TRIANGLES {
                    for &t in piece {
                        let p = points(t);
                        let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                        let dists: Vec<String> = carriers.iter().map(|(s, _)| format!("{:.3e}", p.iter().map(|&q| s.signed_distance(q).abs()).fold(0.0_f64, f64::max))).collect();
                        let station = carriers.iter().find_map(|(s, _)| if let crate::AnalyticSurface::Cone(cone) = s { Some((centroid - cone.apex).dot(cone.axis)) } else { None });
                        eprintln!("[split]     tri {t} centroid ({:.3},{:.3},{:.3}) station {:?} worst distance to carriers {:?} owner {c}", centroid.x, centroid.y, centroid.z, station.map(|s| (s * 1000.0).round() / 1000.0), dists);
                    }
                }
            }
        }
        // A noise-bar region whose assignment leaves a piece under the
        // sliver size is left as segmented: such a piece is a triangle or
        // two the nearer carrier took at the bar inside the other's wall,
        // and built it made an open edge that failed the whole rebuild
        // (census record kernel_crossed_cylinders_s1_d0.05_jitter-1.0 at
        // six bars: pieces of 315, 285, 8, 1, 457, 35 and 1, and the row
        // went from 2 402 faceted to a faceted repair of all 2 878).
        // A piece under the floor whose every vertex lies within one bar of
        // its own carrier and that no other carrier holds at the noise bar
        // is on its face, not inside another's wall, and is kept (census
        // record occt_countersunk_hole_s10_d0.1_jitter-0.1 at three bars:
        // two two-triangle pieces of the cone at its rim with the bore, at
        // 8e-3 to 2e-2 of the cone and reaching 2.5 off the bore, against a
        // bar of 0.23; their rim vertices lie on the bore too).
        let on_its_face = |c: usize, piece: &[usize]| {
            piece.iter().all(|&t| {
                let p = points(t);
                p.iter().all(|&q| carriers[c].0.signed_distance(q).abs() <= one_bar)
                    && !carriers.iter().enumerate().any(|(k, (s, _))| k != c && p.iter().all(|&q| s.signed_distance(q).abs() <= tolerance))
            })
        };
        if noise_region && pieces.iter().any(|(c, piece)| piece.len() < SLIVER_TRIANGLES && !on_its_face(*c, piece)) {
            if debug {
                eprintln!("[split] region {} has a piece under {SLIVER_TRIANGLES} triangles at the noise bar, left as segmented", region.id);
            }
            report.declined_regions += 1;
            continue;
        }
        if debug && noise_region {
            for (c, piece) in &pieces {
                if piece.len() < SLIVER_TRIANGLES {
                    eprintln!("[split]   piece of carrier {c} of {} tris under the floor kept, on its face", piece.len());
                }
            }
        }
        let kv = |p: crate::Vec3| Vec3::new(p.x, p.y, p.z);
        for (c, members) in pieces {
            let surface = &carriers[c].0;
            if !matches!(surface, crate::AnalyticSurface::Cylinder(_) | crate::AnalyticSurface::Cone(_) | crate::AnalyticSurface::Torus(_) | crate::AnalyticSurface::Sphere(_) | crate::AnalyticSurface::Plane(_)) {
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
            if debug && noise_region {
                eprintln!("[split]   piece of carrier {c}: {} tris, outward {outward:.3e}, max deviation {max_deviation:.3e}", members.len());
            }
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
                    crate::AnalyticSurface::Torus(torus) => RegionCarrier::Torus {
                        center: kv(torus.center),
                        axis_dir: kv(torus.axis),
                        major_radius: torus.major_radius,
                        minor_radius: torus.minor_radius,
                        sense: if outward >= 0.0 { 1 } else { -1 },
                    },
                    crate::AnalyticSurface::Sphere(sphere) => RegionCarrier::Sphere {
                        center: kv(sphere.center),
                        radius: sphere.radius,
                        sense: if outward >= 0.0 { 1 } else { -1 },
                    },
                    crate::AnalyticSurface::Plane(plane) => RegionCarrier::Plane { origin: kv(plane.origin), normal: kv(plane.normal) },
                    _ => unreachable!("only cylinders, cones, tori, spheres and planes are proposed"),
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
        split_done.insert(region.id);
        changed = true;
    }
    for (host_id, undo_ids, undo_counts, (host, carrier)) in gather_undo {
        // A gathered region the split rebuilt gives the triangles no
        // carrier took back to the regions they were gathered from, with
        // their carriers: on census record
        // kernel_countersunk_hole_s10_d0.02_jitter-0.1 the gather took the
        // block's four side faces (two-triangle planes) with the dust, no
        // carrier of the countersink held them, and left in the gathered
        // region they were built as eight facets.
        if split_done.contains(&host_id) {
            let mut returned = 0;
            for (t, r) in undo_ids {
                if r != host_id && segmentation.triangle_region_ids[t] == host_id {
                    segmentation.triangle_region_ids[t] = r;
                    returned += 1;
                }
            }
            if debug && returned > 0 {
                eprintln!("[split] gathered region {host_id} returns {returned} unassigned triangles to their regions");
            }
            continue;
        }
        segmentation.regions[host].carrier = carrier;
        for (t, r) in undo_ids {
            segmentation.triangle_region_ids[t] = r;
        }
        for (i, count) in undo_counts {
            segmentation.regions[i].triangle_count = count;
        }
        if debug {
            eprintln!("[split] gather into region {host_id} declined, undone");
        }
    }
    if changed {
        compact(segmentation);
    }
    report
}

/// Drop emptied regions and renumber the rest densely, in order.
pub(super) fn compact(segmentation: &mut MeshSegmentation) {
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
    merged: &[crate::CylinderSurface],
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
    // A merged wall ([`merge_refit_cylinders`]) holds at the bar it was
    // refitted at.
    let holds = |surface: &crate::AnalyticSurface, t: usize| {
        let bar = if merged.iter().any(|w| *surface == crate::AnalyticSurface::Cylinder(*w)) { MERGE_REFIT_FIT_BARS * tolerance } else { tolerance };
        holds(surface, t, positions, indices, bar, cos_gate)
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

/// Whether `surface` holds triangle `t`: every vertex within the bar, its
/// normal within the grower's gate of the surface normal at its centroid.
fn holds(surface: &crate::AnalyticSurface, t: usize, positions: &[f64], indices: &[u32], tolerance: f64, cos_gate: f64) -> bool {
    let p = [0, 1, 2].map(|c| {
        let at = indices[3 * t + c] as usize * 3;
        crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    });
    let Some(normal) = (p[1] - p[0]).cross(p[2] - p[0]).normalized() else { return false };
    let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
    p.iter().all(|&q| surface.signed_distance(q).abs() <= tolerance)
        && surface.normal_at(centroid).is_some_and(|n| n.dot(normal).abs() >= cos_gate)
}

/// The gated pass of `split_probe_crease`: the triangles a
/// hypothesis probe first grows to. On census record
/// occt_crossed_cylinders_s1_d0.005_jitter-0.1 each of the four wall pieces
/// holds about 103 long triangles, so the default 128 spans two walls even
/// across the crease gate (near the tangent points the walls meet at a
/// dihedral under it); 16 and 32 both hold a sample to one wall.
const GATED_PROBE_TRIANGLES: usize = 32;

/// [`split`]'s `lone_share` (under
/// [`crate::hybrid_region_brep::HybridSwitches::assemble_cone_bore`]): the
/// share (numerator, denominator) of its region a lone cone must hold to
/// take it; [`assemble_cone_bore`] then gathers what it left.
const LONE_SHARE: (usize, usize) = (3, 4);

/// [`assemble_cone_bore`]: how far (degrees) a bore's own fitted axis may
/// tilt from the cone's.
const BORE_AXIS_TILT_DEG: f64 = 1.0;

/// [`assemble_cone_bore`]: how far (in bars) a bore's own fitted axis may
/// pass from the cone's at the bore.
const BORE_AXIS_OFFSET_BARS: f64 = 4.0;

/// [`merge_refit_cylinders`]: how far (degrees) two fits' axes may tilt and
/// still be one wall.
const MERGE_REFIT_TILT_DEG: f64 = 2.0;

/// [`merge_refit_cylinders`]: how far (in bars) two fits' axis lines and
/// radii may differ and still be one wall.
const MERGE_REFIT_BARS: f64 = 4.0;

/// [`split`]'s `merge_refit`: the cylinder fits a recognition pass accepted
/// that are one wall by a loose test (axes within [`MERGE_REFIT_TILT_DEG`],
/// axis lines and radii within [`MERGE_REFIT_BARS`] bars) are refitted as one
/// cylinder over their joined triangles, which replaces them when the refit
/// is a cylinder; the merged walls' surfaces are returned beside. On census record kernel_crossed_cylinders_s1_d0.05_jitter-0.3
/// each of the two crossing walls came as fits of 35-128 triangles whose
/// axis lines lay up to 2.1e-2 apart (bar 8.1e-3), none held a tenth of the
/// 2 590-triangle region, and it stayed faceted.
fn merge_refit_cylinders(
    accepted: &[crate::SurfaceRegion],
    mesh: &Mesh,
    recognition: &RecognitionOptions,
    tolerance: f64,
    fit_bars: f64,
) -> (Vec<crate::SurfaceRegion>, Vec<crate::CylinderSurface>) {
    let mut walls = Vec::new();
    let cylinder = |r: &crate::SurfaceRegion| if let crate::AnalyticSurface::Cylinder(c) = r.surface { Some(c) } else { None };
    let one_wall = |a: &crate::CylinderSurface, b: &crate::CylinderSurface| {
        let bar = MERGE_REFIT_BARS * tolerance;
        let d = b.axis_origin - a.axis_origin;
        a.axis.dot(b.axis).abs() >= MERGE_REFIT_TILT_DEG.to_radians().cos()
            && (a.radius - b.radius).abs() <= bar
            && (d - a.axis * d.dot(a.axis)).length() <= bar
    };
    let mut order: Vec<usize> = (0..accepted.len()).collect();
    order.sort_by(|&a, &b| accepted[b].triangle_indices.len().cmp(&accepted[a].triangle_indices.len()));
    let mut taken = vec![false; accepted.len()];
    let mut out = Vec::new();
    for &lead in &order {
        if taken[lead] {
            continue;
        }
        taken[lead] = true;
        let Some(first) = cylinder(&accepted[lead]) else {
            out.push(accepted[lead].clone());
            continue;
        };
        // The group grows from the lead: every untaken fit that is one wall
        // with the current cylinder joins, the union is refitted, and the
        // refit (its axis no longer one fragment's) is tested again.
        let mut wide = recognition.clone();
        wide.distance_tolerance = fit_bars * tolerance;
        let mut current = first;
        let mut union: Vec<usize> = accepted[lead].triangle_indices.clone();
        let mut joined: Vec<usize> = Vec::new();
        let mut surface = None;
        loop {
            let group: Vec<usize> = order
                .iter()
                .copied()
                .filter(|&o| !taken[o] && !joined.contains(&o) && cylinder(&accepted[o]).is_some_and(|c| one_wall(&current, &c)))
                .collect();
            if group.is_empty() {
                break;
            }
            let mut grown = union.clone();
            for &o in &group {
                grown.extend(accepted[o].triangle_indices.iter().copied());
            }
            grown.sort_unstable();
            grown.dedup();
            let refit = crate::reconstruct_surface(
                mesh,
                &grown,
                &crate::SurfaceHint::InitialGuess { surface: crate::AnalyticSurface::Cylinder(current), trust: crate::MetadataTrust::InitialGuess },
                &wide,
            );
            if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
                eprintln!("[split] merge {} + {} fits ({} tris): {}", union.len(), group.len(), grown.len(), refit.as_ref().map(|f| format!("{:?}", f.surface)).unwrap_or_else(|e| format!("{e}")));
            }
            let Ok(crate::SurfaceFitResult { surface: crate::AnalyticSurface::Cylinder(fitted), .. }) = refit else { break };
            current = fitted;
            union = grown;
            joined.extend(group);
            surface = Some(fitted);
        }
        let Some(fitted) = surface else {
            out.push(accepted[lead].clone());
            continue;
        };
        for &o in &joined {
            taken[o] = true;
        }
        let mut merged = accepted[lead].clone();
        merged.surface = crate::AnalyticSurface::Cylinder(fitted);
        merged.triangle_indices = union;
        out.push(merged);
        walls.push(fitted);
    }
    (out, walls)
}

/// [`merge_refit_cylinders`]: the bar (in split bars) a merged wall is
/// refitted at, and its region assigned at.
const MERGE_REFIT_FIT_BARS: f64 = 2.0;
/// [`HybridSwitches::split_noise_bar`]: the bar, in split bars, at which a
/// region the noise-bar pass proposes is held and assigned, with no normal
/// gate. On census record kernel_crossed_cylinders_s1_d0.02_jitter-1.0 the
/// true walls hold 95 % of the region within 2 bars and all of it within 3,
/// and the grower's 30-degree gate alone drops 11 % (a tenth of the jittered
/// facet normals tilt past 35 degrees).
const NOISE_BAR_FIT_BARS: f64 = 3.0;
/// The noise-bar pass's ladder: when the proposal at one bar holds less
/// than 19 in 20 of the region by distance alone, the region is drawn again
/// at the next. The jitter the census carries is a multiple of the chord,
/// and the split bar a fraction of it: 2.5 bars on d0.02 at jitter 1.0 (the
/// record above), 3.7 bars on d0.1 at jitter 0.3 (census record
/// kernel_crossed_cylinders_s1_d0.1_jitter-0.3, 1 833 faceted, whose 3-bar
/// proposal held 1 268 of 1 433 and was declined).
const NOISE_BAR_LADDER: [f64; 2] = [NOISE_BAR_FIT_BARS, 2.0 * NOISE_BAR_FIT_BARS];
/// [`HybridSwitches::split_noise_bar`]: in a region the noise-bar pass
/// proposes, a neighbouring region under this many triangles is a sliver
/// (the grower walls off runs of up to 25 jittered triangles on the record
/// above against [`SLIVER_TRIANGLES`]), and an enclosed group of slivers
/// under [`NOISE_SLIVER_GROUP_TRIANGLES`] joins the carrier holding it when
/// every triangle's centroid lies on that carrier at the bar: a 30-triangle
/// cap fanned from its rim is a sliver by this floor and lies on the wall at
/// the bar with every vertex on the rim, but its centroids lie well inside
/// the wall, while wall dust has its centroids on the wall.
const NOISE_SLIVER_TRIANGLES: usize = 32;
/// [`HybridSwitches::split_noise_bar`]: a strip the noise bore does not hold
/// declines the strip search only when it lies within this many noise bars
/// of the bore (a ring piece sagging past the bar); farther, it is another
/// face's fragment the ring reached.
const NOISE_STRIP_NEAR_BARS: f64 = 4.0;
/// [`HybridSwitches::split_noise_bar`]: a region beside the split region
/// under this many triangles, of any kind, joins the split as a strip when
/// the bore or the cone holds its every triangle at the bar (freeform bands
/// of 10-32 triangles along the bore wall on the d0.05 / d0.1 jitter-0.1
/// countersinks).
const NOISE_BAND_TRIANGLES: usize = 48;
/// [`HybridSwitches::split_noise_bar`]: a region triangle no carrier holds
/// at the noise bar goes to the one carrier holding it within this many
/// noise bars (jitter leaves a few wall triangles just past the bar).
const NOISE_ASSIGN_SLACK_BARS: f64 = 1.5;
/// [`HybridSwitches::split_noise_bar`]: the kinds drawn at the noise bars on
/// a region whose normals cover no band (no cylinder or cone wall): a lone
/// torus or a lone sphere (a ring torus holds no sphere 19 in 20, a sphere
/// no ring torus; the larger hold wins the dedup).
const NOISE_LONE_KINDS: [crate::SurfaceType; 2] = [crate::SurfaceType::Torus, crate::SurfaceType::Sphere];
/// `whole_closed_primitive`'s stray bar (lane J row 10): a stray region
/// beside a whole primitive has fewer triangles than this, and the strays
/// together hold at most 1 % of the mesh.
const WHOLE_STRAY_TRIANGLES: usize = 8;
/// [`HybridSwitches::split_noise_bar`]: a carrier of a gathered dust region
/// holds at least this many of its own (the caps of a small jittered body
/// hold 30-60 triangles, under [`MIN_SPLIT_TRIANGLES`]).
const GATHERED_CARRIER_TRIANGLES: usize = 16;
/// [`HybridSwitches::split_noise_bar`]: a gathered dust region holds and
/// assigns with a normal gate this wide: a coarse body's caps are fanned
/// from their rim, so every cap vertex lies on the wall cylinder and only
/// the facet normal tells a cap triangle from a wall one (census record
/// kernel_cylinder_s1_d0.1_jitter-0.1: the wall held all 124 triangles
/// with no normal gate).
const GATHERED_NORMAL_GATE_DEG: f64 = 60.0;
/// [`HybridSwitches::split_noise_bar`]: a curved fit on a gathered region all
/// of whose triangle normals lie within this many degrees of their mean is a
/// flat cap drawn as a curved surface, and is read as a plane.
const GATHERED_FLAT_DEG: f64 = 30.0;
/// [`HybridSwitches::split_noise_bar`]: a sphere region of the segmenter's
/// that is a coarse cylinder band. A band whose vertices all lie on its two
/// rim circles is fitted at its vertices by the sphere through both rings as
/// well as by the cylinder, and only the facets tell them apart: their
/// centroids lie near the cylinder (its chord sag) and inside the sphere by
/// the band's bulge. A sphere region refitted as a cylinder whose facet
/// centroids lie under half as far from the cylinder as from the sphere
/// becomes that cylinder (census record occt_stepped_shaft_s10_d0.1_jitter-0.1:
/// the radius-20 wall of 84 came as a sphere, which built once its caps did,
/// at a volume error of 0.30). Returns how many regions it changed.
fn sphere_bands_to_cylinders(segmentation: &mut MeshSegmentation, positions: &[f64], indices: &[u32]) -> usize {
    let point = |v: u32| {
        let at = v as usize * 3;
        crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    };
    let debug = std::env::var("BREP_DEBUG_SPLIT").is_ok();
    let mut members = std::collections::BTreeMap::<u32, Vec<usize>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        members.entry(r).or_default().push(t);
    }
    let mut changed = 0;
    for region in segmentation.regions.iter_mut() {
        let RegionCarrier::Sphere { center, radius, sense } = region.carrier else { continue };
        let Some(tris) = members.get(&region.id) else { continue };
        if tris.len() < GATHERED_CARRIER_TRIANGLES {
            continue;
        }
        let mut remap = std::collections::HashMap::<u32, u32>::new();
        let (mut vertices, mut triangles) = (Vec::new(), Vec::new());
        for &t in tris {
            let mut tri = [0u32; 3];
            for c in 0..3 {
                let v = indices[3 * t + c];
                tri[c] = *remap.entry(v).or_insert_with(|| {
                    vertices.push(point(v));
                    (vertices.len() - 1) as u32
                });
            }
            triangles.push(tri);
        }
        let diagonal = region.bbox_max.sub(region.bbox_min).length();
        let mut options = RecognitionOptions::default();
        options.distance_tolerance = 1.0e-2 * diagonal;
        options.relative_tolerance = 0.0;
        options.normal_tolerance = std::f64::consts::FRAC_PI_2;
        options.sampling = SamplingMode::Vertices;
        let selection: Vec<usize> = (0..triangles.len()).collect();
        let mesh = Mesh::new(vertices, triangles);
        let hint = crate::SurfaceHint::KnownType { surface_type: crate::SurfaceType::Cylinder };
        let Ok(fit) = crate::reconstruct_surface(&mesh, &selection, &hint, &options) else { continue };
        let crate::AnalyticSurface::Cylinder(cylinder) = fit.surface else { continue };
        let sphere = crate::AnalyticSurface::Sphere(crate::SphereSurface { center: crate::Vec3::new(center.x, center.y, center.z), radius });
        let (mut on_sphere, mut on_cylinder) = (0.0, 0.0);
        for t in &mesh.triangles {
            let [a, b, c] = t.map(|v| mesh.vertices[v as usize]);
            let centroid = (a + b + c) * (1.0 / 3.0);
            on_sphere += sphere.signed_distance(centroid).abs();
            on_cylinder += fit.surface.signed_distance(centroid).abs();
        }
        if debug {
            eprintln!("[split] sphere region {} ({}): facet centroids {on_sphere:.3e} off the sphere, {on_cylinder:.3e} off a cylinder of radius {:.4e}", region.id, tris.len(), cylinder.radius);
        }
        if !(on_cylinder * 2.0 < on_sphere) {
            continue;
        }
        let middle = mesh.vertices.iter().fold(crate::Vec3::new(0.0, 0.0, 0.0), |sum, &q| sum + q) * (1.0 / mesh.vertices.len() as f64);
        let on_axis = cylinder.axis_origin + cylinder.axis * (middle - cylinder.axis_origin).dot(cylinder.axis);
        region.carrier = RegionCarrier::Cylinder {
            axis_point: Vec3::new(on_axis.x, on_axis.y, on_axis.z),
            axis_dir: Vec3::new(cylinder.axis.x, cylinder.axis.y, cylinder.axis.z),
            radius: cylinder.radius,
            sense,
        };
        region.max_deviation = mesh.vertices.iter().map(|&q| fit.surface.signed_distance(q).abs()).fold(0.0, f64::max);
        changed += 1;
    }
    changed
}

/// Two cones among `carriers` within half a degree of one axis and one half
/// angle: one cone fitted twice.
fn twin_cones(carriers: &[(crate::AnalyticSurface, usize)]) -> bool {
    let cones: Vec<crate::ConeSurface> = carriers
        .iter()
        .filter_map(|(c, _)| if let crate::AnalyticSurface::Cone(cone) = c { Some(*cone) } else { None })
        .collect();
    cones.iter().enumerate().any(|(i, a)| {
        cones[i + 1..].iter().any(|b| {
            a.axis.dot(b.axis).abs() >= (0.5_f64).to_radians().cos() && (a.half_angle - b.half_angle).abs() <= (0.5_f64).to_radians()
        })
    })
}
/// [`HybridSwitches::split_noise_bar`]: a cone fit on a gathered region whose
/// half angle lies within this many degrees of 90 is a flat cap too, however
/// its facets spread.
const GATHERED_FLAT_CONE_DEG: f64 = 1.0;
/// [`HybridSwitches::split_noise_bar`]: a body whose plane regions of 16
/// triangles or more all fit worse than this fraction of the mesh's diagonal
/// is jittered, and its dust is every region under the split's floor.
const GATHER_NOISY_PLANE: f64 = 1.0e-4;
/// [`HybridSwitches::split_noise_bar`]: the jitter test's bar for a plane
/// chip of a few triangles, against the mesh's diagonal: a clean or
/// T-junction mesh's chips fit to round-off, a jittered one's by its noise,
/// which on a two-triangle strip is a fraction of a plane region's (census
/// record occt_annular_tube_s10_d0.1_jitter-0.1: the bore's strips and the
/// top cap's chips 5e-6 to 2.2e-4 of the diagonal, most under
/// GATHER_NOISY_PLANE's 1e-4).
const GATHER_NOISY_CHIP: f64 = 1.0e-6;
/// [`HybridSwitches::split_noise_bar`]: a gathered region's cone within this
/// many degrees of its plane's normal is snapped onto it.
const GATHERED_CONE_SNAP_DEG: f64 = 10.0;
const NOISE_SLIVER_GROUP_TRIANGLES: usize = 96;

/// The fewest triangles in each piece a coaxial bore holds.
const MIN_BORE_TRIANGLES: usize = 16;

/// A cylinder on `cone`'s axis through the region triangles the cone does
/// not hold, with its support: the radius is the median distance from the
/// axis of the vertices of those that face away from it within the gate.
/// It is a carrier only when each edge-connected piece it holds has at
/// least [`MIN_BORE_TRIANGLES`] and it holds, with the cone, 19 in 20 of
/// the region.
fn coaxial_bore(
    cone: &crate::ConeSurface,
    ids: &[usize],
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    cos_gate: f64,
    bridges: Option<&dyn Fn(&crate::AnalyticSurface) -> Option<Vec<(u32, Vec<usize>)>>>,
) -> Option<(crate::AnalyticSurface, usize, Vec<(u32, Vec<usize>)>)> {
    let surface = crate::AnalyticSurface::Cone(*cone);
    let rest: Vec<usize> = ids.iter().copied().filter(|&t| !holds(&surface, t, positions, indices, tolerance, cos_gate)).collect();
    let point = |v: u32| {
        let at = v as usize * 3;
        crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    };
    let radial = |q: crate::Vec3| {
        let d = q - cone.apex;
        d - cone.axis * d.dot(cone.axis)
    };
    let mut radii = Vec::new();
    for &t in &rest {
        let p = [0, 1, 2].map(|c| point(indices[3 * t + c]));
        let Some(normal) = (p[1] - p[0]).cross(p[2] - p[0]).normalized() else { continue };
        let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
        if radial(centroid).normalized().is_some_and(|r| r.dot(normal).abs() >= cos_gate) {
            radii.extend(p.iter().map(|&q| radial(q).length()));
        }
    }
    if radii.is_empty() {
        return None;
    }
    radii.sort_by(f64::total_cmp);
    let bore = crate::AnalyticSurface::Cylinder(crate::CylinderSurface {
        axis_origin: cone.apex,
        axis: cone.axis,
        radius: radii[radii.len() / 2],
    });
    let own: Vec<usize> = rest.iter().copied().filter(|&t| holds(&bore, t, positions, indices, tolerance, cos_gate)).collect();
    let support = ids.len() - rest.len();
    if std::env::var("BREP_DEBUG_SPLIT").is_ok() {
        eprintln!("[split]   coaxial bore radius {:.4e}: cone holds {support} of {}, bore {} of the rest {}", radii[radii.len() / 2], ids.len(), own.len(), rest.len());
    }
    if own.len() < MIN_BORE_TRIANGLES || (support + own.len()) * 20 < ids.len() * 19 {
        return None;
    }
    // Each edge-connected piece the bore holds is a whole strip of at least
    // [`MIN_BORE_TRIANGLES`]: smaller pieces are slivers that become regions
    // of their own and fail as rulings (census record
    // occt_countersunk_hole_s10_d0.02_jitter-0.3: 22 triangles of a
    // 124-triangle piece of the countersink in pieces of 2-6, which demoted
    // the cone with them).
    // Under `bore_strips` the strips beside the region that the bore holds
    // count as bore triangles here: a bore piece between two strips is a
    // piece of one ring, not a sliver.
    let found = bridges.and_then(|bridges| bridges(&bore)).unwrap_or_default();
    let held = own.len();
    let own: Vec<usize> = own.into_iter().chain(found.iter().flat_map(|(_, tris)| tris.iter().copied())).collect();
    let mut by_edge = std::collections::HashMap::<(u32, u32), Vec<usize>>::new();
    for (i, &t) in own.iter().enumerate() {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    let mut piece = vec![usize::MAX; own.len()];
    for start in 0..own.len() {
        if piece[start] != usize::MAX {
            continue;
        }
        piece[start] = start;
        let (mut stack, mut size) = (vec![start], 0);
        while let Some(i) = stack.pop() {
            size += 1;
            let t = own[i];
            for c in 0..3 {
                let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
                for &j in &by_edge[&(a.min(b), a.max(b))] {
                    if piece[j] == usize::MAX {
                        piece[j] = start;
                        stack.push(j);
                    }
                }
            }
        }
        if size < MIN_BORE_TRIANGLES {
            return None;
        }
    }
    Some((bore, held, found))
}

/// [`split`]'s `bore_strips`: the small plane regions (under
/// [`SLIVER_TRIANGLES`]) reached by edges from region `region` or from one
/// another, every triangle of which `bore` holds and `cone` holds none.
/// With `bands` (the noise-bar path), regions of any kind under that many
/// triangles, every triangle of which the bore or the cone holds: the
/// segmenter leaves freeform bands of 10-32 triangles along the bore wall
/// into the cone's rim beside the split region (the d0.05 / d0.1
/// jitter-0.1 countersinks), at which the builder demoted the bore.
/// None when a reached strip faces the bore (every triangle's normal within
/// `faces_gate` of the bore's; `cos_gate` gates the hold) but the bore does not hold it: the bore would
/// take only part of its ring and the strips left beside it are plane
/// edges the builder demotes the bore at (the d0.05 countersinks, whose
/// 24-strip bores' chords sag past the bar: census record
/// occt_countersunk_hole_s1_d0.05_jitter-0.1, 242 -> 278 faceted).
#[allow(clippy::too_many_arguments)]
fn bore_strip_regions(
    bore: &crate::AnalyticSurface,
    cone: &crate::ConeSurface,
    ids: &[usize],
    region: u32,
    segmentation: &MeshSegmentation,
    edges: &std::collections::HashMap<(u32, u32), Vec<usize>>,
    positions: &[f64],
    indices: &[u32],
    tolerance: f64,
    cos_gate: f64,
    faces_gate: f64,
    near: f64,
    bands: Option<usize>,
    debug: bool,
) -> Option<Vec<(u32, Vec<usize>)>> {
    let cone = crate::AnalyticSurface::Cone(*cone);
    // A strip faces the bore when its normal lies within `faces_gate` of
    // the bore's and its vertices within `near` of the bore: a facing strip
    // the bore does not hold is a ring piece sagging past the bar, not a
    // fragment of another face reached through the ring (the outer-wall
    // fragment 3.2 away on census record occt_countersunk_hole_s1_d0.1_jitter-0.3).
    let faces = |t: usize| {
        let p = [0, 1, 2].map(|c| {
            let at = indices[3 * t + c] as usize * 3;
            crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
        });
        let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
        p.iter().all(|&q| bore.signed_distance(q).abs() <= near)
            && (p[1] - p[0]).cross(p[2] - p[0]).normalized().is_some_and(|normal| {
                bore.normal_at(centroid).is_some_and(|n| n.dot(normal).abs() >= faces_gate)
            })
    };
    let mut members = std::collections::BTreeMap::<u32, Vec<usize>>::new();
    for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
        if r != region
            && segmentation.regions.get(r as usize).is_some_and(|other| match bands {
                Some(band_triangles) => other.triangle_count < band_triangles,
                None => matches!(other.carrier, RegionCarrier::Plane { .. }) && other.triangle_count < SLIVER_TRIANGLES,
            })
        {
            members.entry(r).or_default().push(t);
        }
    }
    let mut seen = std::collections::BTreeSet::<u32>::new();
    let mut found = Vec::new();
    let mut frontier: Vec<usize> = ids.to_vec();
    while let Some(t) = frontier.pop() {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            for &n in &edges[&(a.min(b), a.max(b))] {
                let r = segmentation.triangle_region_ids[n];
                let Some(tris) = members.get(&r) else { continue };
                if !seen.insert(r) {
                    continue;
                }
                if tris.iter().all(|&s| {
                    let by_bore = holds(bore, s, positions, indices, tolerance, cos_gate);
                    let by_cone = holds(&cone, s, positions, indices, tolerance, cos_gate);
                    if bands.is_some() { by_bore || by_cone } else { by_bore && !by_cone }
                }) {
                    frontier.extend(tris.iter().copied());
                    found.push((r, tris.clone()));
                } else if tris.iter().all(|&s| faces(s)) {
                    if debug && bands.is_some() {
                        let worst = |surface: &crate::AnalyticSurface| tris.iter().map(|&s| [0, 1, 2].map(|c| { let at = indices[3 * s + c] as usize * 3; crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2]) }).iter().map(|&q| surface.signed_distance(q).abs()).fold(0.0_f64, f64::max)).fold(0.0_f64, f64::max);
                        eprintln!("[split]   strip search declined at region {r} ({} tris, facing, not held): worst distance to bore {:.3e} cone {:.3e} bar {:.3e}", tris.len(), worst(bore), worst(&cone), tolerance);
                    }
                    return None;
                } else if debug && bands.is_some() {
                    eprintln!("[split]   strip search skips region {r} ({} tris): not held, not facing", tris.len());
                }
            }
        }
    }
    Some(found)
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
            .map(|(result, _)| propose(&result.regions, ids, positions, indices, recognition.distance_tolerance, cos_gate, cones, &[]))
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
        let mut timed = recognition.clone();
        timed.collect_phase_timings = true;
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
/// four wall pieces. With `curved`, a fragment on any curved carrier joins
/// the same way (`HybridSwitches::absorb_curved_slivers`). True when a
/// fragment joined.
pub(super) fn absorb_plane_fragments(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    noise: f64,
    curved: bool,
) -> bool {
    if !(noise > 0.0) || indices.len() != segmentation.triangle_region_ids.len() * 3 {
        return false;
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
        if !(curved || matches!(region.carrier, RegionCarrier::Plane { .. } | RegionCarrier::Freeform))
            || triangles.len() >= SLIVER_TRIANGLES
        {
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
    changed
}

/// [`crate::hybrid_region_brep::HybridSwitches::assemble_cone_bore`]: the
/// smallest cone region the assembly starts from.
const ASSEMBLE_CONE_TRIANGLES: usize = 64;

/// [`crate::hybrid_region_brep::HybridSwitches::assemble_cone_bore`]: the
/// largest curved region the assembly gathers around a cone (a fragment of
/// it, as the cone-strip gather takes).
const ASSEMBLE_FRAGMENT_TRIANGLES: usize = 32;

/// A cone and the bore beneath it, assembled across the regions the
/// segmenter and the split left them in. From each cone region of at least
/// [`ASSEMBLE_CONE_TRIANGLES`], the regions reached by edges that are
/// freeform, plane strips under [`SLIVER_TRIANGLES`] or curved fragments of
/// at most [`ASSEMBLE_FRAGMENT_TRIANGLES`] are gathered. The triangles there
/// the cone does not hold and that face its axis square are fitted with one
/// cylinder on its own axis, within [`BORE_AXIS_TILT_DEG`] and
/// [`BORE_AXIS_OFFSET_BARS`] of the cone's; it must hold every one of them
/// (the whole ring). Then every gathered triangle must go to the cone or the
/// bore (within the bar; one the gate refuses goes to the nearer carrier
/// within the bar, as the split's slivers do), or nothing is changed. The
/// cone region takes its triangles and each edge-connected piece of the bore
/// becomes a cylinder region. Returns how many cones were assembled.
pub(super) fn assemble_cone_bore(
    segmentation: &mut MeshSegmentation,
    positions: &[f64],
    indices: &[u32],
    options: &SegmentOptions,
) -> usize {
    let triangle_count = segmentation.triangle_region_ids.len();
    if indices.len() != triangle_count * 3 {
        return 0;
    }
    let debug = std::env::var("BREP_DEBUG_SPLIT").is_ok();
    let point = |v: u32| {
        let at = v as usize * 3;
        crate::Vec3::new(positions[at], positions[at + 1], positions[at + 2])
    };
    let points = |t: usize| [0, 1, 2].map(|c| point(indices[3 * t + c]));
    let mut edges = std::collections::HashMap::<(u32, u32), Vec<usize>>::new();
    for t in 0..triangle_count {
        for c in 0..3 {
            let (a, b) = (indices[3 * t + c], indices[3 * t + (c + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let cos_gate = options.deflection_angle_deg.to_radians().cos();
    let chord = crate::hybrid_whole_freeform::chord_spans(positions, indices, options.weld_tolerance);
    let mut assembled = 0;
    let cones: Vec<u32> = segmentation
        .regions
        .iter()
        .filter(|r| matches!(r.carrier, RegionCarrier::Cone { .. }) && r.triangle_count >= ASSEMBLE_CONE_TRIANGLES)
        .map(|r| r.id)
        .collect();
    for c in cones {
        let Some(region) = segmentation.regions.get(c as usize) else { continue };
        let RegionCarrier::Cone { apex, axis_dir, half_angle_rad, .. } = region.carrier else { continue };
        let cone = crate::ConeSurface {
            apex: crate::Vec3::new(apex.x, apex.y, apex.z),
            axis: crate::Vec3::new(axis_dir.x, axis_dir.y, axis_dir.z),
            half_angle: half_angle_rad,
        };
        let cone_surface = crate::AnalyticSurface::Cone(cone);
        let gathered_region = |r: u32| {
            segmentation.regions.get(r as usize).is_some_and(|other| match other.carrier {
                RegionCarrier::Freeform => true,
                RegionCarrier::Plane { .. } => other.triangle_count < SLIVER_TRIANGLES,
                _ => other.triangle_count <= ASSEMBLE_FRAGMENT_TRIANGLES,
            })
        };
        let mut members = std::collections::BTreeMap::<u32, Vec<usize>>::new();
        for (t, &r) in segmentation.triangle_region_ids.iter().enumerate() {
            members.entry(r).or_default().push(t);
        }
        let own: Vec<usize> = members.get(&c).cloned().unwrap_or_default();
        let mut seen = std::collections::BTreeSet::<u32>::new();
        let mut frontier = own.clone();
        let mut gathered = Vec::<usize>::new();
        while let Some(t) = frontier.pop() {
            for k in 0..3 {
                let (a, b) = (indices[3 * t + k], indices[3 * t + (k + 1) % 3]);
                for &n in &edges[&(a.min(b), a.max(b))] {
                    let r = segmentation.triangle_region_ids[n];
                    if r == c || seen.contains(&r) || !gathered_region(r) {
                        continue;
                    }
                    seen.insert(r);
                    let tris: Vec<usize> = members.get(&r).cloned().unwrap_or_default();
                    frontier.extend(tris.iter().copied());
                    gathered.extend(tris);
                }
            }
        }
        if gathered.is_empty() {
            continue;
        }
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for &t in own.iter().chain(&gathered) {
            for q in points(t) {
                for (axis, value) in [q.x, q.y, q.z].into_iter().enumerate() {
                    low[axis] = low[axis].min(value);
                    high[axis] = high[axis].max(value);
                }
            }
        }
        let diagonal = (0..3).map(|a| (high[a] - low[a]).powi(2)).sum::<f64>().sqrt();
        let tolerance = options.fit_tolerance * diagonal;
        let radial = |q: crate::Vec3| {
            let d = q - cone.apex;
            d - cone.axis * d.dot(cone.axis)
        };
        let square = (cone.half_angle * 0.5).cos();
        let on_cone: Vec<bool> = gathered.iter().map(|&t| holds(&cone_surface, t, positions, indices, tolerance, cos_gate)).collect();
        let facing: Vec<usize> = gathered
            .iter()
            .zip(&on_cone)
            .filter(|(_, &held)| !held)
            .map(|(&t, _)| t)
            .filter(|&t| {
                let p = points(t);
                let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                (p[1] - p[0]).cross(p[2] - p[0]).normalized().zip(radial(centroid).normalized()).is_some_and(|(n, r)| n.dot(r).abs() >= square)
            })
            .collect();
        if facing.len() < MIN_BORE_TRIANGLES {
            if debug {
                eprintln!("[cone-bore] cone {c} ({}): {} gathered, {} face the axis, left", own.len(), gathered.len(), facing.len());
            }
            continue;
        }
        let mut radii: Vec<f64> = facing.iter().flat_map(|&t| points(t).map(|q| radial(q).length())).collect();
        radii.sort_by(f64::total_cmp);
        let start = crate::AnalyticSurface::Cylinder(crate::CylinderSurface {
            axis_origin: cone.apex,
            axis: cone.axis,
            radius: radii[radii.len() / 2],
        });
        // The bore's own fit, on a mesh of its facing triangles.
        let mut remap = std::collections::HashMap::<u32, u32>::new();
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for &t in &facing {
            let mut tri = [0u32; 3];
            for k in 0..3 {
                let v = indices[3 * t + k];
                tri[k] = *remap.entry(v).or_insert_with(|| {
                    vertices.push(point(v));
                    (vertices.len() - 1) as u32
                });
            }
            triangles.push(tri);
        }
        let mut recognition = RecognitionOptions::default();
        recognition.distance_tolerance = tolerance;
        recognition.relative_tolerance = 0.0;
        recognition.normal_tolerance = options.normal_tolerance_deg.to_radians();
        recognition.sampling = SamplingMode::Vertices;
        // The fit leaves out the triangles with a chord corner (a
        // T-junction vertex sits inside the bore by its chord's sag) while
        // enough are left: fitted through them, the bore of census record
        // kernel_countersunk_hole_s10_d0.1_tjunction came out at radius
        // 9.9995 against the countersink's exact rim of 10, and with the
        // switch alone the builder demoted every cylinder/cone edge.
        let clean: Vec<usize> = facing
            .iter()
            .enumerate()
            .filter(|(_, &t)| (0..3).all(|k| !chord.contains_key(&indices[3 * t + k])))
            .map(|(i, _)| i)
            .collect();
        let selection: Vec<usize> = if clean.len() >= MIN_BORE_TRIANGLES { clean } else { (0..triangles.len()).collect() };
        let fitted = crate::reconstruct_surface(
            &Mesh::new(vertices, triangles),
            &selection,
            &crate::SurfaceHint::InitialGuess { surface: start, trust: crate::MetadataTrust::InitialGuess },
            &recognition,
        )
        .ok()
        .map(|fit| fit.surface);
        let Some(crate::AnalyticSurface::Cylinder(fitted)) = fitted else {
            if debug {
                eprintln!("[cone-bore] cone {c}: no cylinder through {} facing triangles", facing.len());
            }
            continue;
        };
        let middle = facing.iter().flat_map(|&t| points(t)).fold(crate::Vec3::new(0.0, 0.0, 0.0), |sum, q| sum + q)
            * (1.0 / (3 * facing.len()) as f64);
        let on_fitted = fitted.axis_origin + fitted.axis * (middle - fitted.axis_origin).dot(fitted.axis);
        if fitted.axis.dot(cone.axis).abs() < BORE_AXIS_TILT_DEG.to_radians().cos() || radial(on_fitted).length() > BORE_AXIS_OFFSET_BARS * tolerance {
            if debug {
                eprintln!("[cone-bore] cone {c}: bore axis off the cone's ({:?})", fitted);
            }
            continue;
        }
        let bore = crate::AnalyticSurface::Cylinder(fitted);
        let ring = facing.iter().filter(|&&t| holds(&bore, t, positions, indices, tolerance, cos_gate)).count();
        if ring < facing.len() {
            if debug {
                eprintln!("[cone-bore] cone {c}: bore holds {ring} of its {} facing triangles, left", facing.len());
            }
            continue;
        }
        // Every gathered triangle to a carrier, or nothing changes.
        let carriers = [cone_surface, bore];
        // A corner the mesh put on a chord between two of its neighbours (a
        // T-junction vertex) sits inside the carrier by that chord's sag, so
        // the chord's ends are measured in its place (the span whose ends
        // the carrier holds best): on census record
        // kernel_countersunk_hole_s10_d0.1_tjunction two countersink
        // triangles held a chord midpoint 0.233 inside the cone against a
        // bar of 0.076, and the assembly was declined. Left unmeasured, a
        // bore triangle with a chord corner at mid-height kept only its two
        // rim corners, which the cone holds as well as the bore.
        let nearest = |t: usize, gate: f64| -> Option<usize> {
            let p = points(t);
            let normal = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
            let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
            let distance = |s: &crate::AnalyticSurface, k: usize| match chord.get(&indices[3 * t + k]) {
                Some(spans) => spans
                    .iter()
                    .map(|&(a, b)| s.signed_distance(point(a)).abs().max(s.signed_distance(point(b)).abs()))
                    .fold(f64::INFINITY, f64::min),
                None => s.signed_distance(p[k]).abs(),
            };
            carriers
                .iter()
                .enumerate()
                .filter_map(|(i, s)| {
                    let worst = (0..3).map(|k| distance(s, k)).fold(0.0_f64, f64::max);
                    let faces = normal.is_some_and(|m| s.normal_at(centroid).is_some_and(|n| n.dot(m).abs() >= gate));
                    (worst <= tolerance && faces).then_some((worst, i))
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, i)| i)
        };
        let owner: Option<Vec<(usize, usize)>> = gathered
            .iter()
            .map(|&t| nearest(t, cos_gate).or_else(|| nearest(t, -1.0)).map(|i| (t, i)))
            .collect();
        let Some(owner) = owner else {
            if debug {
                let left = gathered.iter().filter(|&&t| nearest(t, -1.0).is_none()).count();
                eprintln!("[cone-bore] cone {c}: {left} of {} gathered triangles on neither carrier, left", gathered.len());
                for &t in gathered.iter().filter(|&&t| nearest(t, -1.0).is_none()) {
                    let p = points(t);
                    let worst: Vec<f64> = carriers.iter().map(|s| p.iter().map(|&q| s.signed_distance(q).abs()).fold(0.0_f64, f64::max)).collect();
                    eprintln!("[cone-bore]   tri {t} region {} corners {:?} worst {:?} bar {tolerance:.3e}", segmentation.triangle_region_ids[t], p, worst);
                }
            }
            continue;
        };
        if debug {
            eprintln!(
                "[cone-bore] cone {c} ({}) takes {} gathered triangles: {} to the cone, {} to a bore of radius {:.4e}",
                own.len(),
                owner.len(),
                owner.iter().filter(|o| o.1 == 0).count(),
                owner.iter().filter(|o| o.1 == 1).count(),
                fitted.radius
            );
        }
        for &(t, i) in &owner {
            if i == 0 {
                segmentation.triangle_region_ids[t] = c;
            }
        }
        // One cylinder region per edge-connected piece of the bore.
        let bore_tris: std::collections::BTreeSet<usize> = owner.iter().filter(|o| o.1 == 1).map(|o| o.0).collect();
        let mut placed = std::collections::BTreeSet::<usize>::new();
        let kv = |p: crate::Vec3| Vec3::new(p.x, p.y, p.z);
        for &start in &bore_tris {
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
                        if bore_tris.contains(&n) && placed.insert(n) {
                            piece.push(n);
                        }
                    }
                }
            }
            let (mut outward, mut max_deviation, mut sum_sq, mut area, mut max_angle) = (0.0, 0.0_f64, 0.0, 0.0, 0.0_f64);
            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
            for &t in &piece {
                let p = points(t);
                let cross = (p[1] - p[0]).cross(p[2] - p[0]);
                area += 0.5 * cross.length();
                let centroid = (p[0] + p[1] + p[2]) * (1.0 / 3.0);
                if let (Some(n), Some(m)) = (bore.normal_at(centroid), cross.normalized()) {
                    outward += n.dot(m);
                    max_angle = max_angle.max(n.dot(m).abs().min(1.0).acos().to_degrees());
                }
                for q in p {
                    let d = bore.signed_distance(q).abs();
                    max_deviation = max_deviation.max(d);
                    sum_sq += d * d;
                    lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
                    hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
                }
            }
            let id = segmentation.regions.len() as u32;
            segmentation.regions.push(MeshRegion {
                id,
                triangle_count: piece.len(),
                area,
                bbox_min: lo,
                bbox_max: hi,
                carrier: RegionCarrier::Cylinder {
                    axis_point: kv(fitted.axis_origin),
                    axis_dir: kv(fitted.axis),
                    radius: fitted.radius,
                    sense: if outward >= 0.0 { 1 } else { -1 },
                },
                max_deviation,
                rms_deviation: (sum_sq / (3 * piece.len()).max(1) as f64).sqrt(),
                max_normal_angle_deg: max_angle,
            });
            for &t in &piece {
                segmentation.triangle_region_ids[t] = id;
            }
        }
        assembled += 1;
    }
    if assembled > 0 {
        compact(segmentation);
    }
    assembled
}
