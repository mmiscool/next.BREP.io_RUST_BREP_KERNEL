use super::*;

fn face_normal(face: &FaceRecord, u: f64, v: f64) -> Result<Vec3, String> {
    let normal = match face.surface.normal(u, v) {
        Ok(normal) => normal,
        Err(error) => {
            // POLE RESCUE: at a collapsed parameterization pole (sphere pole,
            // cone apex) du×dv vanishes and `normal()` errors — which used to
            // propagate and kill the ENTIRE boolean (trial 9: a sphere placed
            // TANGENT at its pole dies with "Vec3.normalized: zero-length
            // vector" before any fragment is classified). The GEOMETRIC
            // normal is well-defined and continuous there; recover it by
            // stepping slightly inside the domain (at the pole every azimuth
            // shares the same limit normal, so the offset direction does not
            // matter). Escape hatch: BREP_POLE_NORMAL_RESCUE=0.
            if !error.contains("zero-length")
                || std::env::var("BREP_POLE_NORMAL_RESCUE").as_deref() == Ok("0")
            {
                return Err(error);
            }
            let [u0, u1] = face.surface.domain_u()?;
            let [v0, v1] = face.surface.domain_v()?;
            let step_u = (u1 - u0) * 1e-4;
            let step_v = (v1 - v0) * 1e-4;
            let inner_u = u.clamp(u0 + step_u, u1 - step_u);
            let inner_v = v.clamp(v0 + step_v, v1 - step_v);
            let mut recovered = None;
            for (cu, cv) in [(u, inner_v), (inner_u, v), (inner_u, inner_v)] {
                if let Ok(normal) = face.surface.normal(cu, cv) {
                    recovered = Some(normal);
                    break;
                }
            }
            match recovered {
                Some(normal) => normal,
                None => return Err(error),
            }
        }
    };
    Ok(if face.same_sense {
        normal
    } else {
        normal.scale(-1.0)
    })
}

/// UV band equivalent to a spatial band of `spatial` at (u, v), derived
/// from the local surface derivative magnitudes (`tolerance.rs`
/// `surface_uv_tolerance`).  Capped to a fraction of the smaller domain
/// span so a pole or collapsed direction cannot widen the band into the
/// whole face.
/// `BREP_CLASSIFY_THIN_BODY_BAND` (default off, `=1` on): see
/// [`SolidClassifier::classify`].
fn thin_body_band_enabled() -> bool {
    crate::switch!("BREP_CLASSIFY_THIN_BODY_BAND", false)
}

/// `BREP_CLASSIFY_THIN_BODY_BAND_CHECK` (default off, `=1` on): measure the
/// thin-body band and report where it would change the answer, keeping it.
fn thin_body_band_check() -> bool {
    crate::switch!("BREP_CLASSIFY_THIN_BODY_BAND_CHECK", false)
}

fn face_uv_tolerance(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> f64 {
    surface_uv_band(&face.surface, u, v, spatial)
}

/// [`face_uv_tolerance`] for a bare carrier.
pub fn surface_uv_band(surface: &crate::NurbsSurface, u: f64, v: f64, spatial: f64) -> f64 {
    let Ok(derivatives) = surface.derivatives(u, v, 1) else {
        return spatial;
    };
    let band = crate::tolerance::surface_uv_tolerance(
        spatial,
        derivatives[1][0].length(),
        derivatives[0][1].length(),
    );
    let cap = match (surface.domain_u(), surface.domain_v()) {
        (Ok([u0, u1]), Ok([v0, v1])) => ((u1 - u0).min(v1 - v0) * 0.05).max(1e-12),
        _ => f64::INFINITY,
    };
    band.min(cap)
}

/// `BREP_CLASSIFY_METRIC_TRIM` (default on; `=0` reads the isotropic band as
/// before): see [`metric_trim`].
fn metric_trim_enabled() -> bool {
    switch!("BREP_CLASSIFY_METRIC_TRIM", true)
}

/// `BREP_CLASSIFY_METRIC_TRIM_CHECK` (default off): compute the old read and
/// the metric read side by side and print each ray hit where they differ.
fn metric_trim_check() -> bool {
    switch!("BREP_CLASSIFY_METRIC_TRIM_CHECK", false)
}

/// The trim read of a ray's crossing at `(u, v)`, with its `Boundary` band
/// measured in space along each parameter direction. [`face_uv_tolerance`]
/// is one uv radius, `spatial / min(|Su|, |Sv|)`: on a wall 5e-3 tall and a
/// dozen long (|Su| 11, |Sv| 8.8e-3) a 1e-6 band read 1.1e-4 in u, 1.25e-3 in
/// space, and every ray from a point within a millimetre of the wall's corner
/// edge read `Boundary` there and gave up ("no clean ray direction").
///
/// The spatial band is the ellipse with semi-axes `spatial / |Su|` and
/// `spatial / |Sv|` about the hit. When the two differ by more than 4×, a hit
/// the isotropic band reads `Boundary` is read again with the ellipse's minor
/// radius at its centre and at its eight tips every 45°. It stays `Boundary`
/// unless all nine reads agree on one side. A straight trim within the
/// ellipse misses all nine only if it cuts off a cap between two tips, at
/// least `cos 22.5°` (0.92) of the band from the hit. Hits on a near-isotropic
/// chart and hits the old band reads `Inside` or `Outside` read as before.
fn metric_trim(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> Result<PolygonClass, String> {
    let band = face_uv_tolerance(face, u, v, spatial);
    let old = parameter_point_in_face(face, Vec2 { x: u, y: v }, band)?;
    let check = metric_trim_check();
    if old != PolygonClass::Boundary || !(metric_trim_enabled() || check) {
        return Ok(old);
    }
    let Ok(derivatives) = face.surface.derivatives(u, v, 1) else {
        return Ok(old);
    };
    let cap = match (face.surface.domain_u(), face.surface.domain_v()) {
        (Ok([u0, u1]), Ok([v0, v1])) => ((u1 - u0).min(v1 - v0) * 0.05).max(1e-12),
        _ => f64::INFINITY,
    };
    let along_u = crate::tolerance::parametric_tolerance(spatial, derivatives[1][0].length()).min(cap);
    let along_v = crate::tolerance::parametric_tolerance(spatial, derivatives[0][1].length()).min(cap);
    let minor = along_u.min(along_v);
    if !(minor > 0.0) || along_u.max(along_v) <= 4.0 * minor {
        return Ok(old);
    }
    let centre = parameter_point_in_face(face, Vec2 { x: u, y: v }, minor)?;
    let mut answer = centre;
    if centre != PolygonClass::Boundary {
        for step in 0..8 {
            let angle = step as f64 * std::f64::consts::FRAC_PI_4;
            let tip = Vec2 { x: u + along_u * angle.cos(), y: v + along_v * angle.sin() };
            if parameter_point_in_face(face, tip, minor)? != centre {
                answer = PolygonClass::Boundary;
                break;
            }
        }
    }
    if check && answer != old {
        eprintln!(
            "classify metric trim: face {} uv ({u:.9}, {v:.9}) band {band:.3e} axes ({along_u:.3e}, {along_v:.3e}) old {old:?} metric {answer:?}",
            face.id
        );
    }
    Ok(if metric_trim_enabled() { answer } else { old })
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum PointClass {
    In,
    Out,
    On,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct PointClassification {
    pub class: PointClass,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub on_normal: Option<Vec3>,
}

/// Per-thread counters for the classifier, printed by
/// [`classify_profile_report`] when `BREP_PROFILE` is set: how many point
/// queries ran, how many face projections and trim tests the On band cost,
/// how many rays were cast and how many face intersections and trim tests
/// they cost, and the wall time of each part. When the switch is off every
/// counter is one thread-local flag test.
#[derive(Default, Clone, Copy)]
pub struct ClassifyProfile {
    pub enabled: bool,
    pub classify_calls: u64,
    pub band_candidates: u64,
    pub band_project_ms: f64,
    pub band_trim_tests: u64,
    pub band_trim_ms: f64,
    pub rays: u64,
    pub ray_faces: u64,
    pub ray_csi_ms: f64,
    /// Ray-face intersections the exact lane seeded (`exact_ray_enabled`) and
    /// the ones it declined to the sampled search.
    pub ray_exact_faces: u64,
    pub ray_fallback_faces: u64,
    pub ray_trim_tests: u64,
    pub ray_trim_ms: f64,
    pub coincident_calls: u64,
    pub coincident_ms: f64,
    pub classifier_builds: u64,
    pub classifier_build_ms: f64,
}

thread_local! {
    static CLASSIFY_PROFILE: std::cell::Cell<ClassifyProfile> =
        const { std::cell::Cell::new(ClassifyProfile {
            enabled: false, classify_calls: 0, band_candidates: 0, band_project_ms: 0.0,
            band_trim_tests: 0, band_trim_ms: 0.0, rays: 0, ray_faces: 0, ray_csi_ms: 0.0,
            ray_exact_faces: 0, ray_fallback_faces: 0, ray_trim_tests: 0, ray_trim_ms: 0.0, coincident_calls: 0, coincident_ms: 0.0,
            classifier_builds: 0, classifier_build_ms: 0.0,
        }) };
}

impl crate::profile::Counters for ClassifyProfile {
    fn armed(&self) -> bool {
        self.enabled
    }
}

fn classify_profile(update: impl FnOnce(&mut ClassifyProfile)) {
    crate::profile::update(&CLASSIFY_PROFILE, update)
}

fn classify_profile_started() -> Option<web_time::Instant> {
    crate::profile::started(&CLASSIFY_PROFILE)
}

use crate::profile::elapsed_ms as classify_elapsed_ms;

/// Arm the classifier counters (reset to zero) when `BREP_PROFILE` is set.
pub fn classify_profile_begin() {
    let enabled = crate::profile::profile_enabled();
    CLASSIFY_PROFILE.with(|cell| {
        cell.set(ClassifyProfile {
            enabled,
            ..ClassifyProfile::default()
        })
    });
}

/// Print the counters gathered since [`classify_profile_begin`] under
/// `label` and disarm them. Silent when profiling is off.
pub fn classify_profile_report(label: &str) {
    let p = CLASSIFY_PROFILE.with(|cell| cell.get());
    if !p.enabled {
        return;
    }
    profile_line!(
        "classify.profile {label} calls={} builds={} build_ms={:.2} band: candidates={} project_ms={:.2} trim_tests={} trim_ms={:.2} rays={} ray_faces={} csi_ms={:.2} exact_faces={} fallback_faces={} ray_trim_tests={} ray_trim_ms={:.2} coincident: calls={} ms={:.2}",
        p.classify_calls, p.classifier_builds, p.classifier_build_ms, p.band_candidates,
        p.band_project_ms, p.band_trim_tests, p.band_trim_ms, p.rays, p.ray_faces,
        p.ray_csi_ms, p.ray_exact_faces, p.ray_fallback_faces, p.ray_trim_tests, p.ray_trim_ms, p.coincident_calls, p.coincident_ms,
    );
    CLASSIFY_PROFILE.with(|cell| cell.set(ClassifyProfile::default()));
}

/// The exact ray lane (lane B row 1, 2026-10-17): a ray against an ANALYTIC
/// face is seeded with its closed-form roots (`crate::exact_line_surface`)
/// instead of the sampled search's 32-sample scan, then solved and filtered
/// by the same code. Default ON since its on/off census (production 96/96,
/// bench 52/52 bodies and 1197/1197 lines) read identical;
/// `BREP_CLASSIFY_EXACT_RAY=0` (or `off`) restores the sampled search.
fn exact_ray_enabled() -> bool {
    switch!("BREP_CLASSIFY_EXACT_RAY", true)
}

/// One ray-face intersection of the ray pass: the exact lane's seeds when it
/// is on and the face admits them, the sampled search otherwise. Counted on
/// the profile as `exact_faces` / `fallback_faces` (never timed).
fn ray_face_intersections(
    sub_ray: &NurbsCurve,
    surface: &crate::NurbsSurface,
    tolerance: f64,
) -> Result<Vec<crate::CurveSurfaceIntersection>, String> {
    if exact_ray_enabled() {
        if let Some(seeds) = crate::exact_line_surface::exact_line_surface_seeds(sub_ray, surface, tolerance) {
            classify_profile(|p| p.ray_exact_faces += 1);
            return crate::curve_surface_intersection::intersect_curve_surface_from_seeds(sub_ray, surface, tolerance, seeds);
        }
        classify_profile(|p| p.ray_fallback_faces += 1);
    }
    // A ray proved clear of the face: the sampled search's answer is known to be empty (the check
    // mode runs the search anyway and stops on a disagreement).
    let check = crate::curve_clear::check_mode();
    if check || ray_clear_enabled() {
        let clear = crate::curve_clear::curve_clear_of_surface(sub_ray, surface, tolerance);
        if check {
            let hits = intersect_curve_surface(sub_ray, surface, tolerance)?;
            if clear && !hits.is_empty() {
                crate::curve_clear::disagreement("classifier ray", hits.len());
            }
            return Ok(hits);
        }
        if clear {
            return Ok(Vec::new());
        }
    }
    crate::curve_surface_intersection::intersect_ray_surface(sub_ray, surface, tolerance)
}

/// `BREP_CLASSIFY_RAY_CLEAR` (default on since its promotion; `=0` restores the search alone): a ray the sampled search would take against a freeform
/// face is first proved clear of it by hull boxes (`crate::curve_clear`); most such searches find
/// nothing, each after many Newton starts that open with a global projection.
fn ray_clear_enabled() -> bool {
    switch!("BREP_CLASSIFY_RAY_CLEAR", true)
}

/// Point-in-solid classification with per-solid precomputation: face
/// bounds, the solid box, and a face BVH are built once so repeated
/// queries (boolean fragment selection asks once per fragment) prune to
/// the few faces a probe point or ray can actually touch.
pub struct SolidClassifier<'a> {
    faces: Vec<&'a FaceRecord>,
    face_boxes: Vec<Aabb>,
    bounds: Aabb,
    bvh: Bvh,
    tolerance: f64,
    /// `(region, cap)`: within `region` the On and coincident-On bands reach
    /// at most `cap` (the feature-scale design's B6/B6′; empty unless the boolean's
    /// `BREP_FEATURE_SCALE_SELECT` sets it with [`Self::with_coincident_caps`]).
    coincident_caps: Vec<(Aabb, f64)>,
}

/// Clip the segment `start + s·direction, s ∈ [0, length]` to `bounds`
/// (slab test).  Returns the clipped `[s_entry, s_exit]` span, or None when
/// the segment misses the box.
fn clip_segment_to_aabb(
    start: Vec3,
    direction: Vec3,
    length: f64,
    bounds: &Aabb,
) -> Option<[f64; 2]> {
    let mut s0 = 0.0f64;
    let mut s1 = length;
    for axis in 0..3 {
        let (origin, delta, minimum, maximum) = match axis {
            0 => (start.x, direction.x, bounds.minimum.x, bounds.maximum.x),
            1 => (start.y, direction.y, bounds.minimum.y, bounds.maximum.y),
            _ => (start.z, direction.z, bounds.minimum.z, bounds.maximum.z),
        };
        if delta.abs() <= 1e-15 {
            if origin < minimum || origin > maximum {
                return None;
            }
            continue;
        }
        let mut near = (minimum - origin) / delta;
        let mut far = (maximum - origin) / delta;
        if near > far {
            std::mem::swap(&mut near, &mut far);
        }
        s0 = s0.max(near);
        s1 = s1.min(far);
        if s0 > s1 {
            return None;
        }
    }
    Some([s0, s1])
}

/// `BREP_CLASSIFY_PLANAR_TRIM_BOXES` (default on; `=0` restores): a planar
/// face's box is its boundary's (the control points of the edges its loops
/// use), not its carrier's. A linear function over a plane region takes its
/// extremes on the region's boundary, so the region lies in the boundary's
/// box. A boolean's planar faces keep the carrier they were cut from: on a
/// molded bottle cap 180 of 484 planes span up to 85 times their trims, every
/// ray met about 100 face boxes against 14 on the same body read back from
/// STEP, and 4096 points took 57 s against 8 s.
fn planar_trim_boxes_enabled() -> bool {
    switch!("BREP_CLASSIFY_PLANAR_TRIM_BOXES", true)
}

/// The box of a planar face's boundary curves, when every coedge's edge is
/// known; `None` for a curved face, a face with no loop, or a missing edge.
fn planar_trim_box(face: &FaceRecord, edges: &std::collections::HashMap<u64, &crate::topology::EdgeRecord>) -> Option<Aabb> {
    if edges.is_empty() || face.loops.is_empty() || !face.surface.is_affine().unwrap_or(false) {
        return None;
    }
    let mut trimmed = Aabb::empty();
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        let edge = edges.get(&coedge.edge_id)?;
        trimmed.include(Aabb::from_curve_controls(&edge.curve).ok()?);
    }
    Some(trimmed)
}

impl<'a> SolidClassifier<'a> {
    pub fn new(solid: &'a BrepSolid, tolerance: f64) -> Result<Self, String> {
        let build_started = classify_profile_started();
        let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
        let edges: std::collections::HashMap<u64, &crate::topology::EdgeRecord> =
            if planar_trim_boxes_enabled() { solid.edges.iter().map(|edge| (edge.id, edge)).collect() } else { Default::default() };
        let face_boxes = faces
            .iter()
            .map(|face| match planar_trim_box(face, &edges) {
                Some(trimmed) => Ok(trimmed),
                None => Aabb::from_surface_controls(&face.surface),
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut bounds = Aabb::empty();
        for face_box in &face_boxes {
            bounds.include(*face_box);
        }
        let bvh = Bvh::build(&face_boxes);
        classify_profile(|p| {
            p.classifier_builds += 1;
            p.classifier_build_ms += classify_elapsed_ms(build_started);
        });
        Ok(Self {
            faces,
            face_boxes,
            bounds,
            bvh,
            tolerance,
            coincident_caps: Vec::new(),
        })
    }

    /// The orientation a chord through `through` along `direction` reads, or
    /// `None` when the chord cannot tell (`BREP_BOOLEAN_SIGN_CHORDS`).
    ///
    /// The chord runs from outside the faces' box to outside it, so a closed,
    /// consistently oriented boundary is entered and left in turn: its hits,
    /// ordered along the chord, alternate between outward normals against
    /// the chord and along it, and their number is even. `Some(true)` when
    /// they do and the first is an entry (material lies behind the normals,
    /// as on a solid of positive volume), `Some(false)` when they do and the
    /// first is an exit (inside out). `None` when any hit is tangential,
    /// grazes (`|n · d|` under 1e-2), falls in a trim's boundary band, sits
    /// within the On band of another hit, when the hits do not alternate or
    /// are odd in number (a root the finder lost, or a face reversed against
    /// its neighbours), or when no hit lies within the On band of `through`
    /// (the aimed face itself was missed).
    pub fn chord_orientation(&self, through: Vec3, direction: Vec3) -> Result<Option<bool>, String> {
        let on_tolerance = self.tolerance * 10.0;
        let direction = direction.normalized()?;
        let middle = self.bounds.minimum.add(self.bounds.maximum).scale(0.5);
        let reach = self.bounds.diagonal() + through.sub(middle).length() + 1.0;
        let start = through.sub(direction.scale(reach));
        let length = 2.0 * reach;
        let end = start.add(direction.scale(length));
        let mut candidates = Vec::new();
        self.bvh.intersecting_segment(start, end, on_tolerance, &mut candidates);
        candidates.sort_unstable();
        // (distance along the chord, the outward normal along it).
        let mut hits: Vec<(f64, bool)> = Vec::new();
        let mut aimed = false;
        for &index in &candidates {
            let face = self.faces[index];
            let margin = on_tolerance.max(self.face_boxes[index].diagonal() * 1e-3);
            let Some([span_start, span_end]) =
                clip_segment_to_aabb(start, direction, length, &self.face_boxes[index].expanded(margin))
            else {
                continue;
            };
            let span_start = (span_start - margin).max(0.0);
            let span_end = (span_end + margin).min(length);
            if span_end - span_start <= 1e-12 {
                continue;
            }
            let sub_ray = make_line(start.add(direction.scale(span_start)), start.add(direction.scale(span_end)))?;
            for intersection in ray_face_intersections(&sub_ray, &face.surface, self.tolerance)? {
                if intersection.tangential {
                    return Ok(None);
                }
                let uv = Vec2 { x: intersection.u, y: intersection.v };
                match parameter_point_in_face(face, uv, face_uv_tolerance(face, uv.x, uv.y, on_tolerance))? {
                    PolygonClass::Outside => continue,
                    PolygonClass::Boundary => return Ok(None),
                    PolygonClass::Inside => {}
                }
                let along = face_normal(face, uv.x, uv.y)?.dot(direction);
                if along.abs() < 1e-2 {
                    return Ok(None);
                }
                if intersection.point.sub(through).length() <= on_tolerance {
                    aimed = true;
                }
                hits.push((intersection.point.sub(start).dot(direction), along > 0.0));
            }
        }
        if !aimed || hits.is_empty() || hits.len() % 2 == 1 {
            return Ok(None);
        }
        hits.sort_by(|a, b| a.0.total_cmp(&b.0));
        for pair in hits.windows(2) {
            if pair[1].0 - pair[0].0 <= on_tolerance || pair[1].1 == pair[0].1 {
                return Ok(None);
            }
        }
        Ok(Some(!hits[0].1))
    }

    /// Cheap On-band probe: is the point within the classifier's On
    /// tolerance of ANY face carrier (trim not checked — conservative)?
    /// Used to guard adjacency propagation: fragments near the other
    /// solid's surface need the full normal-based On decision, everything
    /// else can inherit its fate from a neighbor.
    pub fn near_surface(&self, point: Vec3) -> Result<bool, String> {
        let on_tolerance = self.tolerance * 10.0;
        if !self.bounds.expanded(on_tolerance).contains(point) {
            return Ok(false);
        }
        let mut candidates = Vec::new();
        self.bvh
            .containing_point(point, on_tolerance, &mut candidates);
        for &index in &candidates {
            let projection = project_point_to_surface(&self.faces[index].surface, point)?;
            if projection.distance <= on_tolerance {
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Conservative carrier-proximity probe: is `point` within `band` of ANY
    /// face's carrier surface (trim NOT checked)?  A strict superset of the
    /// `On` verdict (which additionally requires the point to fall in a face's
    /// trim), so callers that want to skip every point that *might* be on or
    /// near the boundary — the semantic oracle's On-skip — can rely on a `true`
    /// here to mean "not safely In/Out".  `band` is taken explicitly so the
    /// caller can widen it to a size-relative width and absorb near-coincidence
    /// gaps (Golovanov §4.13 derived tolerances).
    pub fn within_band(&self, point: Vec3, band: f64) -> Result<bool, String> {
        if !self.bounds.expanded(band).contains(point) {
            return Ok(false);
        }
        let mut candidates = Vec::new();
        self.bvh.containing_point(point, band, &mut candidates);
        for &index in &candidates {
            let projection = project_point_to_surface(&self.faces[index].surface, point)?;
            if projection.distance <= band {
                return Ok(true);
            }
        }
        Ok(false)
    }

    pub fn classify(&self, point: Vec3) -> Result<PointClassification, String> {
    let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::CLASSIFY);
        classify_profile(|p| p.classify_calls += 1);
        let tolerance = self.tolerance;
        if !self.bounds.expanded(tolerance).contains(point) {
            return Ok(PointClassification {
                class: PointClass::Out,
                on_normal: None,
            });
        }
        // FEATURE SCALE B6′ (`BREP_FEATURE_SCALE_SELECT`; no caps without it):
        // the On band (1e-6) reaches across a slab 1e-6 thick, so the middle
        // of the hairline slot's strips read On its walls, not In the saw.
        let on_tolerance = self.capped_band(point, tolerance * 10.0);
        if !self.coincident_caps.is_empty() && env_read!("BREP_DEBUG_FEATURE_SCALE").is_some() {
            eprintln!("feature_scale: classify at {point:?} on band {on_tolerance:.3e} (uncapped {:.3e}, {} caps)", tolerance * 10.0, self.coincident_caps.len());
        }
        // THIN-BODY BAND (`BREP_CLASSIFY_THIN_BODY_BAND`, default off; with
        // `BREP_CLASSIFY_THIN_BODY_BAND_CHECK` the band is measured and a
        // differing answer reported, the answer kept): where the body is
        // thinner than four On bands at a face the point lies within the band
        // of, the band is capped at a quarter of the body's local thickness
        // there, measured exactly along the face's inward normal from the
        // point's foot to the next face. A lens 1.8e-6 thick read a point
        // 4.7e-7 outside it On its wall (the band is 1e-6), so inside.
        let (thin_on, thin_check) = (thin_body_band_enabled(), thin_body_band_check());
        if thin_on || thin_check {
            if let Some(cap) = self.thin_body_cap(point, on_tolerance)? {
                if cap < on_tolerance {
                    // The rays then read crossings at a tenth of the cap: at
                    // the classifier's own tolerance two crossings of a wall
                    // 8e-7 across merge into one.
                    let ray_tolerance = (0.1 * cap).min(self.tolerance);
                    if thin_on {
                        return self.classify_banded(point, cap, ray_tolerance);
                    }
                    let kept = self.classify_banded(point, on_tolerance, self.tolerance)?;
                    let capped = self.classify_banded(point, cap, ray_tolerance)?;
                    if kept.class != capped.class {
                        eprintln!(
                            "thin-body band check: {point:?} reads {:?} at the band {on_tolerance:.3e}, {:?} at the thickness cap {cap:.3e}",
                            kept.class, capped.class
                        );
                    }
                    return Ok(kept);
                }
            }
        }
        self.classify_banded(point, on_tolerance, self.tolerance)
    }

    /// The least quarter of the body's local thickness at every face whose
    /// band `band` reaches the point inside its trim, where that thickness is
    /// under four bands: from the point's foot on the face, along the face's
    /// inward normal, to the nearest other boundary point within four bands
    /// (a face's own surface is passed over at its foot). `None` where the
    /// body is no thinner than that.
    fn thin_body_cap(&self, point: Vec3, band: f64) -> Result<Option<f64>, String> {
        let mut candidates = Vec::new();
        self.bvh.containing_point(point, band, &mut candidates);
        let mut cap: Option<f64> = None;
        let reach = 4.0 * band;
        for &index in &candidates {
            let face = self.faces[index];
            let projection = project_point_to_surface(&face.surface, point)?;
            if projection.distance > band {
                continue;
            }
            let uv = Vec2 { x: projection.u, y: projection.v };
            if parameter_point_in_face(face, uv, face_uv_tolerance(face, projection.u, projection.v, band))? == PolygonClass::Outside {
                continue;
            }
            let Ok(normal) = face_normal(face, projection.u, projection.v) else { continue };
            let inward = normal.scale(-1.0);
            // The probe starts a thousandth of a band inside, past its own
            // wall (no self-hit to exclude, at any distance from the origin),
            // and reads the hits at that resolution (the intersection merges
            // hits within ten of its tolerance; at the classifier's own a
            // wall 8e-7 across would merge with its foot).
            let offset = 1e-3 * band;
            let resolution = 1e-3 * band;
            let foot = projection.point;
            let from = foot.add(inward.scale(offset));
            let to = foot.add(inward.scale(reach));
            let mut hits = Vec::new();
            self.bvh.intersecting_segment(from, to, band, &mut hits);
            let ray = make_line(from, to)?;
            for &other in &hits {
                let other_face = self.faces[other];
                for hit in ray_face_intersections(&ray, &other_face.surface, resolution)? {
                    let distance = hit.point.sub(foot).length();
                    if !(distance > offset) || distance > reach {
                        continue;
                    }
                    let trim = parameter_point_in_face(
                        other_face,
                        Vec2 { x: hit.u, y: hit.v },
                        face_uv_tolerance(other_face, hit.u, hit.v, resolution),
                    )?;
                    if trim != PolygonClass::Outside {
                        let quarter = distance / 4.0;
                        cap = Some(cap.map_or(quarter, |c: f64| c.min(quarter)));
                    }
                }
            }
        }
        Ok(cap)
    }

    /// [`Self::classify`] at the On band `on_tolerance`, its rays' crossings
    /// read at `tolerance`.
    fn classify_banded(&self, point: Vec3, on_tolerance: f64, tolerance: f64) -> Result<PointClassification, String> {
        let mut candidates = Vec::new();
        self.bvh
            .containing_point(point, on_tolerance, &mut candidates);
        candidates.sort_unstable();
        // All faces the point lies On, split by whether it sits in the trim
        // interior or within the boundary band.  Where the point is on an
        // edge or vertex shared by several faces, the classification normal
        // is the normalized sum of their normals (Golovanov §4.11/§6.3) —
        // a single face's normal is ambiguous there, and falling through to
        // ray casting made the answer direction-dependent.
        let mut interior_normals: Vec<Vec3> = Vec::new();
        let mut boundary_normals: Vec<Vec3> = Vec::new();
        for &index in &candidates {
            let face = self.faces[index];
            let project_started = classify_profile_started();
            let projection = project_point_to_surface(&face.surface, point)?;
            classify_profile(|p| {
                p.band_candidates += 1;
                p.band_project_ms += classify_elapsed_ms(project_started);
            });
            if projection.distance > on_tolerance {
                continue;
            }
            let uv_tolerance = face_uv_tolerance(face, projection.u, projection.v, on_tolerance);
            let trim_started = classify_profile_started();
            let trim = parameter_point_in_face(
                face,
                Vec2 {
                    x: projection.u,
                    y: projection.v,
                },
                uv_tolerance,
            )?;
            classify_profile(|p| {
                p.band_trim_tests += 1;
                p.band_trim_ms += classify_elapsed_ms(trim_started);
            });
            match trim {
                PolygonClass::Inside => {
                    interior_normals.push(face_normal(face, projection.u, projection.v)?)
                }
                PolygonClass::Boundary => {
                    boundary_normals.push(face_normal(face, projection.u, projection.v)?)
                }
                PolygonClass::Outside => {}
            }
        }
        let pool = if interior_normals.is_empty() {
            &boundary_normals
        } else {
            &interior_normals
        };
        if !pool.is_empty() {
            let mut sum = Vec3::default();
            for normal in pool {
                sum = sum.add(*normal);
            }
            // A near-zero sum means opposed normals (knife edge, coincident
            // back-to-back faces): genuinely ambiguous, let the ray casting
            // below decide.
            if sum.length() > 1e-3 {
                return Ok(PointClassification {
                    class: PointClass::On,
                    on_normal: Some(sum.normalized()?),
                });
            }
        }
        let directions = [
            Vec3::new(0.577215, 0.618034, 0.532088),
            Vec3::new(-0.707107, 0.267949, 0.654321),
            Vec3::new(0.316228, -0.741657, 0.585786),
            Vec3::new(-0.414214, -0.552786, -0.723607),
            Vec3::new(0.9482, 0.11893, -0.29456),
            Vec3::new(-0.13947, 0.90271, -0.40718),
            Vec3::new(0.62361, -0.33912, -0.70414),
            Vec3::new(0.20912, 0.51293, 0.83261),
        ];
        let ray_length = self.bounds.diagonal() * 3.0 + 1.0;
        // RAY-AGREEMENT VOTING: a single ray's parity flips when the root
        // finder loses one of two close crossings through a thin feature
        // (trial 533's 00000231-p4: a ray pierced a curved flange twice ~3 mm
        // apart, intersect_curve_surface returned one root, and a point 2 mm
        // clear of the material classified In). One lost root is a
        // direction-specific accident, so require TWO clean directions to
        // AGREE before trusting the parity; on disagreement keep sampling
        // directions and return the first verdict confirmed twice. Escape
        // hatch: BREP_CLASSIFY_RAY_AGREE=0 restores first-clean-ray.
        let require_agreement = std::env::var("BREP_CLASSIFY_RAY_AGREE").as_deref() != Ok("0");
        let mut verdicts: Vec<PointClass> = Vec::new();
        'directions: for direction in directions {
            classify_profile(|p| p.rays += 1);
            let direction = direction.normalized()?;
            let ray_end = point.add(direction.scale(ray_length));
            candidates.clear();
            self.bvh
                .intersecting_segment(point, ray_end, on_tolerance, &mut candidates);
            candidates.sort_unstable();
            let mut crossings = 0;
            for &index in &candidates {
                let face = self.faces[index];
                // Intersect the ray with this face over the SHORT span the
                // ray actually spends near the face's bounding box, not the
                // whole solid-diagonal-scale ray.  The root finder seeds one
                // Newton start per curve sample segment, so a long ray gives
                // a small face a single seed — a ray that crosses a small
                // curved face twice within one sample segment (e.g. a probe
                // from a tangency line through a blend tube) silently loses
                // a crossing and flips the parity.  Clipping the ray to the
                // face box makes the seed density match the face scale.
                let margin = on_tolerance.max(self.face_boxes[index].diagonal() * 1e-3);
                let Some([span_start, span_end]) = clip_segment_to_aabb(
                    point,
                    direction,
                    ray_length,
                    &self.face_boxes[index].expanded(margin),
                ) else {
                    continue;
                };
                let span_start = (span_start - margin).max(0.0);
                let span_end = (span_end + margin).min(ray_length);
                if span_end - span_start <= 1e-12 {
                    continue;
                }
                let sub_ray = make_line(
                    point.add(direction.scale(span_start)),
                    point.add(direction.scale(span_end)),
                )?;
                let csi_started = classify_profile_started();
                let intersections = ray_face_intersections(&sub_ray, &face.surface, tolerance)?;
                classify_profile(|p| {
                    p.ray_faces += 1;
                    p.ray_csi_ms += classify_elapsed_ms(csi_started);
                });
                for intersection in intersections {
                    if intersection.point.sub(point).length() <= on_tolerance {
                        let trim_started = classify_profile_started();
                        let trim = parameter_point_in_face(
                            face,
                            Vec2 {
                                x: intersection.u,
                                y: intersection.v,
                            },
                            face_uv_tolerance(face, intersection.u, intersection.v, on_tolerance),
                        )?;
                        classify_profile(|p| {
                            p.ray_trim_tests += 1;
                            p.ray_trim_ms += classify_elapsed_ms(trim_started);
                        });
                        if trim != PolygonClass::Outside {
                            return Ok(PointClassification {
                                class: PointClass::On,
                                on_normal: Some(face_normal(face, intersection.u, intersection.v)?),
                            });
                        }
                        continue;
                    }
                    if intersection.tangential {
                        continue 'directions;
                    }
                    let trim_started = classify_profile_started();
                    let trim_class = metric_trim(face, intersection.u, intersection.v, on_tolerance)?;
                    classify_profile(|p| {
                        p.ray_trim_tests += 1;
                        p.ray_trim_ms += classify_elapsed_ms(trim_started);
                    });
                    if std::env::var("BREP_DEBUG_CLASSIFY").is_ok() {
                        eprintln!(
                            "classify ray dir=({:.3},{:.3},{:.3}) face={} hit=({:.4},{:.4},{:.4}) t3d={:.4} uv=({:.6},{:.6}) trim={:?}",
                            direction.x, direction.y, direction.z,
                            face.id,
                            intersection.point.x, intersection.point.y, intersection.point.z,
                            intersection.point.sub(point).length(),
                            intersection.u, intersection.v,
                            trim_class
                        );
                    }
                    match trim_class {
                        PolygonClass::Boundary => continue 'directions,
                        PolygonClass::Inside => crossings += 1,
                        PolygonClass::Outside => {}
                    }
                }
            }
            let verdict = if crossings % 2 == 1 {
                PointClass::In
            } else {
                PointClass::Out
            };
            if std::env::var("BREP_DEBUG_CLASSIFY").is_ok() {
                eprintln!(
                    "classify verdict dir=({:.3},{:.3},{:.3}) crossings={crossings} -> {verdict:?}",
                    direction.x, direction.y, direction.z
                );
            }
            if !require_agreement || verdicts.contains(&verdict) {
                return Ok(PointClassification {
                    class: verdict,
                    on_normal: None,
                });
            }
            verdicts.push(verdict);
        }
        // Every direction bailed (tangential/boundary) or, under agreement
        // voting, the clean directions never confirmed one another. A single
        // unconfirmed verdict is still far better than an error.
        if let Some(&verdict) = verdicts.last() {
            return Ok(PointClassification {
                class: verdict,
                on_normal: None,
            });
        }
        Err("classifyPointVsSolid: no clean ray direction found".into())
    }

    /// Wider-band On probe for near-coincident faces.  A model built from
    /// noisy input carries faces that are geometrically coincident with the
    /// other solid's boundary yet separated by a gap that scales with the
    /// model (a few microns), not with the absolute model tolerance.  Such a
    /// gap slips a face fragment's interior test point past the fixed On band
    /// (`classify`), so it reads as a stray In/Out.  This probe re-checks
    /// whether `point` sits in the trim INTERIOR of a coincident boundary
    /// face within a band derived from the solid's size (Golovanov §4.13
    /// derived tolerances), returning the mean outward normal of the
    /// coincident faces when it does.  It only ever RESCUES an On verdict —
    /// callers keep the tight-band In/Out otherwise — so the general
    /// classification path is unchanged.  Interior-only (boundary hits are
    /// ignored) so it fires only on a genuine surface overlap, never on mere
    /// proximity to an edge.
    /// Caps the coincident-On band within each region (see
    /// `coincident_caps`): a test point a sliver's width from the other
    /// solid's face is a feature, not noise to rescue to On.
    pub fn with_coincident_caps(mut self, caps: Vec<(Aabb, f64)>) -> Self {
        self.coincident_caps = caps;
        self
    }

    /// `band` capped by every `coincident_caps` region within its reach of
    /// `point`, strictly (a point exactly at a cap's distance is a feature).
    fn capped_band(&self, point: Vec3, band: f64) -> f64 {
        self.coincident_caps
            .iter()
            .filter(|(region, _)| region.expanded(band).contains(point))
            .fold(band, |band, &(_, cap)| band.min(cap * (1.0 - 1e-9)))
    }

    pub fn coincident_on_normal(&self, point: Vec3) -> Result<Option<Vec3>, String> {
        let started = classify_profile_started();
        let result = self.coincident_on_normal_inner(point);
        classify_profile(|p| {
            p.coincident_calls += 1;
            p.coincident_ms += classify_elapsed_ms(started);
        });
        result
    }

    fn coincident_on_normal_inner(&self, point: Vec3) -> Result<Option<Vec3>, String> {
        let band = (self.tolerance * 10.0).max(self.bounds.diagonal() * 1e-7);
        // A point at a capped band's edge is a feature, not noise: strictly
        // inside the cap only (`< cap`, as the projection test below reads
        // `> band` as off).
        let band = self.capped_band(point, band);
        if !self.bounds.expanded(band).contains(point) {
            return Ok(None);
        }
        let mut candidates = Vec::new();
        self.bvh.containing_point(point, band, &mut candidates);
        let mut interior_normals: Vec<Vec3> = Vec::new();
        for &index in &candidates {
            let face = self.faces[index];
            let projection = project_point_to_surface(&face.surface, point)?;
            if projection.distance > band {
                continue;
            }
            let uv_tolerance = face_uv_tolerance(face, projection.u, projection.v, band);
            if let PolygonClass::Inside = parameter_point_in_face(
                face,
                Vec2 {
                    x: projection.u,
                    y: projection.v,
                },
                uv_tolerance,
            )? {
                interior_normals.push(face_normal(face, projection.u, projection.v)?);
            }
        }
        if interior_normals.is_empty() {
            return Ok(None);
        }
        let mut sum = Vec3::default();
        for normal in &interior_normals {
            sum = sum.add(*normal);
        }
        if sum.length() <= 1e-3 {
            return Ok(None);
        }
        Ok(Some(sum.normalized()?))
    }
}

pub fn classify_point(
    point: Vec3,
    solid: &BrepSolid,
    tolerance: f64,
) -> Result<PointClassification, String> {
    let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::CLASSIFY);
    classify_profile_begin();
    let result = SolidClassifier::new(solid, tolerance)?.classify(point);
    classify_profile_report("classify_point");
    result
}




