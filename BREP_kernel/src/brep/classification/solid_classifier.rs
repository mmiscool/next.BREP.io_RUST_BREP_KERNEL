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
fn face_uv_tolerance(face: &FaceRecord, u: f64, v: f64, spatial: f64) -> f64 {
    surface_uv_band(&face.surface, u, v, spatial)
}

/// [`face_uv_tolerance`] for a bare carrier.
pub(crate) fn surface_uv_band(surface: &crate::NurbsSurface, u: f64, v: f64, spatial: f64) -> f64 {
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
pub(crate) struct ClassifyProfile {
    pub enabled: bool,
    pub classify_calls: u64,
    pub band_candidates: u64,
    pub band_project_ms: f64,
    pub band_trim_tests: u64,
    pub band_trim_ms: f64,
    pub rays: u64,
    pub ray_faces: u64,
    pub ray_csi_ms: f64,
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
            ray_trim_tests: 0, ray_trim_ms: 0.0, coincident_calls: 0, coincident_ms: 0.0,
            classifier_builds: 0, classifier_build_ms: 0.0,
        }) };
}

fn classify_profile(update: impl FnOnce(&mut ClassifyProfile)) {
    CLASSIFY_PROFILE.with(|cell| {
        let mut profile = cell.get();
        if profile.enabled {
            update(&mut profile);
            cell.set(profile);
        }
    });
}

fn classify_profile_started() -> Option<web_time::Instant> {
    CLASSIFY_PROFILE
        .with(|cell| cell.get().enabled)
        .then(web_time::Instant::now)
}

fn classify_elapsed_ms(started: Option<web_time::Instant>) -> f64 {
    started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0)
}

/// Arm the classifier counters (reset to zero) when `BREP_PROFILE` is set.
pub(crate) fn classify_profile_begin() {
    let enabled = std::env::var("BREP_PROFILE").is_ok();
    CLASSIFY_PROFILE.with(|cell| {
        cell.set(ClassifyProfile {
            enabled,
            ..ClassifyProfile::default()
        })
    });
}

/// Print the counters gathered since [`classify_profile_begin`] under
/// `label` and disarm them. Silent when profiling is off.
pub(crate) fn classify_profile_report(label: &str) {
    let p = CLASSIFY_PROFILE.with(|cell| cell.get());
    if !p.enabled {
        return;
    }
    eprintln!(
        "classify.profile {label} calls={} builds={} build_ms={:.2} band: candidates={} project_ms={:.2} trim_tests={} trim_ms={:.2} rays={} ray_faces={} csi_ms={:.2} ray_trim_tests={} ray_trim_ms={:.2} coincident: calls={} ms={:.2}",
        p.classify_calls, p.classifier_builds, p.classifier_build_ms, p.band_candidates,
        p.band_project_ms, p.band_trim_tests, p.band_trim_ms, p.rays, p.ray_faces,
        p.ray_csi_ms, p.ray_trim_tests, p.ray_trim_ms, p.coincident_calls, p.coincident_ms,
    );
    CLASSIFY_PROFILE.with(|cell| cell.set(ClassifyProfile::default()));
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

impl<'a> SolidClassifier<'a> {
    pub fn new(solid: &'a BrepSolid, tolerance: f64) -> Result<Self, String> {
        let build_started = classify_profile_started();
        let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
        let face_boxes = faces
            .iter()
            .map(|face| Aabb::from_surface_controls(&face.surface))
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
        })
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
        classify_profile(|p| p.classify_calls += 1);
        let tolerance = self.tolerance;
        if !self.bounds.expanded(tolerance).contains(point) {
            return Ok(PointClassification {
                class: PointClass::Out,
                on_normal: None,
            });
        }
        let on_tolerance = tolerance * 10.0;
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
                let intersections = intersect_curve_surface(&sub_ray, &face.surface, tolerance)?;
                classify_profile(|p| {
                    p.ray_faces += 1;
                    p.ray_csi_ms += classify_elapsed_ms(csi_started);
                });
                for intersection in intersections {
                    if intersection.point.sub(point).length() <= tolerance * 10.0 {
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
                    let trim_class = parameter_point_in_face(
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
    classify_profile_begin();
    let result = SolidClassifier::new(solid, tolerance)?.classify(point);
    classify_profile_report("classify_point");
    result
}
