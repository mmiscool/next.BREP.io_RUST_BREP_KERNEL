use super::*;

/// Mean of a section's sampled curve points — the section CENTROID used both
/// to place the section on the guide and to project it onto the guide.
fn guided_section_centroid(curves: &[NurbsCurve]) -> Result<Vec3, KernelRefusal> {
    let mut sum = Vec3::default();
    let mut count = 0usize;
    for curve in curves {
        let [start, end] = curve.domain().or_refuse(KernelStage::Collect, "domain")?;
        for index in 0..16 {
            sum = sum.add(
                curve
                    .evaluate(start + (end - start) * index as f64 / 16.0)
                    .or_refuse(KernelStage::Collect, "evaluate")?,
            );
            count += 1;
        }
    }
    if count == 0 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "empty_section",
            "guidedLoft: a section has no sampleable curves",
        ));
    }
    Ok(sum.scale(1.0 / count as f64))
}

/// Loft a set of loft-compatible cross-sections so the loft's SPINE follows a
/// GUIDE curve (§5.8) instead of the straight centroid-to-centroid path.
///
/// V1 is TRANSLATION-ONLY: at each station along the guide the two bracketing
/// sections are blended per control point (homogeneous, `w` blended too) into a
/// compatible intermediate section, which is then RIGIDLY TRANSLATED so its
/// centroid lands on the guide.  The sections keep their OWN orientation — they
/// bend along the guide but are not rotated into its moving frame (that is
/// `loft_profile_brep_guided_frame`).  The heavy lifting (skin surfaces, shared
/// edges, planar end caps, winding normalization, validation) is delegated to
/// `loft_profile_brep`, so the guided sections inherit all of its guarantees.
///
/// Clear `Err` on: fewer than 2 sections, incompatible sections, a degenerate
/// guide, sections that do not project monotonically onto the guide, coincident
/// stations, or a downstream loft failure.
pub fn loft_profile_brep_guided(
    sections: &[Vec<NurbsCurve>],
    guide: &NurbsCurve,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    loft_profile_brep_guided_core(sections, guide, name, false)
}

/// Rotation-to-frame guided loft (§5.8): like `loft_profile_brep_guided`, but
/// intermediate sections ROTATE with the guide's rotation-minimizing moving
/// frame (double-reflection RMF, the same frames the path sweep uses) instead
/// of keeping their world orientation.
///
/// Each user section is expressed in the LOCAL frame at its own guide station,
/// the local representations are blended, and the blend is mapped back through
/// the frame at each output station.  Because localize→reconstruct through the
/// SAME frame is the identity, every user section is still reproduced EXACTLY
/// at its own station — the rotation only shapes the flow between sections.
/// The relative rotation between two stations is independent of the arbitrary
/// initial frame normal (a start-normal change conjugates every frame by the
/// same constant), so the result is deterministic.
pub fn loft_profile_brep_guided_frame(
    sections: &[Vec<NurbsCurve>],
    guide: &NurbsCurve,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    loft_profile_brep_guided_core(sections, guide, name, true)
}

/// Rotation-minimizing frames over one ordered parameter strip of the guide.
/// All arrays are aligned with `params` (normalized [0, 1] guide parameters).
struct GuidedFrames {
    params: Vec<f64>,
    points: Vec<Vec3>,
    tangents: Vec<Vec3>,
    r_axes: Vec<Vec3>,
    s_axes: Vec<Vec3>,
}

impl GuidedFrames {
    /// Index of the frame at normalized parameter `t` (must be one of the
    /// parameters the strip was marched over).
    fn index_of(&self, t: f64) -> Result<usize, KernelRefusal> {
        let lower = self.params.partition_point(|p| *p < t - 1e-9);
        if lower < self.params.len() && (self.params[lower] - t).abs() <= 1e-9 {
            Ok(lower)
        } else {
            Err(KernelRefusal::internal(
                KernelStage::Refine,
                "frame_index",
                format!("guidedLoft: no frame marched at parameter {t}"),
            ))
        }
    }
}

/// March double-reflection RMF frames over the union of the output stations
/// and the user-section stations so section localization and station
/// reconstruction share ONE consistent strip (mirrors the path sweep's frame
/// propagation, including the coincident-station and drift re-orthogonalize
/// guards).
fn guided_frames(
    guide: &NurbsCurve,
    g0: f64,
    g1: f64,
    station_params: &[f64],
    section_params: &[f64],
) -> Result<GuidedFrames, KernelRefusal> {
    let mut params: Vec<f64> = station_params
        .iter()
        .chain(section_params.iter())
        .copied()
        .collect();
    params.sort_by(|a, b| a.partial_cmp(b).expect("guide params are finite"));
    params.dedup_by(|a, b| (*a - *b).abs() <= 1e-12);

    let count = params.len();
    let mut points = Vec::with_capacity(count);
    let mut tangents = Vec::with_capacity(count);
    for (index, t) in params.iter().enumerate() {
        let derivatives = guide
            .derivatives(g0 + (g1 - g0) * t, 1)
            .or_refuse(KernelStage::Refine, "guide_derivatives")?;
        let tangent = derivatives[1].normalized().map_err(|_| {
            KernelRefusal::input(
                KernelStage::Classify,
                "guide_tangent",
                format!("guidedLoft: guide tangent is degenerate at station {index}"),
            )
        })?;
        points.push(derivatives[0]);
        tangents.push(tangent);
    }

    let mut r_axes = Vec::with_capacity(count);
    let mut s_axes = Vec::with_capacity(count);
    let r0 = tangents[0]
        .perpendicular()
        .or_refuse(KernelStage::Refine, "frame_start")?; // any unit vector ⟂ T0
    s_axes.push(
        tangents[0]
            .cross(r0)
            .normalized()
            .or_refuse(KernelStage::Refine, "frame_start")?,
    );
    r_axes.push(r0);
    for index in 0..count - 1 {
        let t_next = tangents[index + 1];
        let v1 = points[index + 1].sub(points[index]);
        let c1 = v1.dot(v1);
        let r_candidate = if c1 <= 1e-18 {
            r_axes[index]
        } else {
            let reflected_r = r_axes[index].sub(v1.scale(2.0 / c1 * v1.dot(r_axes[index])));
            let reflected_t = tangents[index].sub(v1.scale(2.0 / c1 * v1.dot(tangents[index])));
            let v2 = t_next.sub(reflected_t);
            let c2 = v2.dot(v2);
            if c2 <= 1e-18 {
                reflected_r
            } else {
                reflected_r.sub(v2.scale(2.0 / c2 * v2.dot(reflected_r)))
            }
        };
        let r_next = r_candidate
            .sub(t_next.scale(r_candidate.dot(t_next)))
            .normalized()
            .map_err(|_| {
                KernelRefusal::internal(
                    KernelStage::Refine,
                    "frame_march",
                    format!("guidedLoft: frame degenerated at station {index}"),
                )
            })?;
        s_axes.push(
            t_next
                .cross(r_next)
                .normalized()
                .or_refuse(KernelStage::Refine, "frame_march")?,
        );
        r_axes.push(r_next);
    }
    Ok(GuidedFrames {
        params,
        points,
        tangents,
        r_axes,
        s_axes,
    })
}

fn loft_profile_brep_guided_core(
    sections: &[Vec<NurbsCurve>],
    guide: &NurbsCurve,
    name: Option<&str>,
    rotate_to_frame: bool,
) -> Result<BrepSolid, KernelRefusal> {
    // Loft carries no face names; accept `name` for ABI symmetry with the other
    // builders (the app stamps names onto the emitted face order).
    let _ = name;
    let tolerance = 1e-6;
    let section_count = sections.len();
    if section_count < 2 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "section_count",
            "guidedLoft: need at least 2 sections",
        ));
    }

    let curve_count = validate_sections(sections, tolerance, "guidedLoft", true)?;

    // --- 2. Guide validity + centroid projection to arc-params uᵢ ∈ [0, 1].
    let [g0, g1] = guide.domain().or_refuse(KernelStage::Collect, "guide")?;
    if (g1 - g0).abs() <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "guide_domain",
            "guidedLoft: guide domain is degenerate",
        ));
    }
    let guide_start = guide.evaluate(g0).or_refuse(KernelStage::Collect, "guide")?;
    let mut guide_extent = 0.0_f64;
    for index in 1..=8 {
        let point = guide
            .evaluate(g0 + (g1 - g0) * index as f64 / 8.0)
            .or_refuse(KernelStage::Collect, "guide")?;
        guide_extent = guide_extent.max(point.sub(guide_start).length());
    }
    if guide_extent <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "guide_extent",
            "guidedLoft: guide curve is degenerate (no spatial extent)",
        ));
    }
    let mut u_list = Vec::with_capacity(section_count);
    for section in sections {
        let centroid = guided_section_centroid(section)?;
        let projection = crate::project_point_to_curve(guide, centroid)
            .or_refuse(KernelStage::Classify, "project")?;
        let u = ((projection.u - g0) / (g1 - g0)).clamp(0.0, 1.0);
        u_list.push(u);
    }

    // Sections must project STRICTLY monotonically onto the guide (either
    // direction); a decreasing projection is normalized to increasing by
    // reversing the section order (the loft is symmetric in section order — this
    // just flips which cap is top / bottom).
    let increasing = u_list.windows(2).all(|pair| pair[1] > pair[0] + tolerance);
    let decreasing = u_list.windows(2).all(|pair| pair[1] < pair[0] - tolerance);
    if !increasing && !decreasing {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "monotone_projection",
            "guidedLoft: sections do not project monotonically onto the guide",
        ));
    }
    let mut ordered_sections: Vec<Vec<NurbsCurve>> = sections.to_vec();
    let mut ordered_u = u_list;
    if decreasing {
        ordered_sections.reverse();
        ordered_u.reverse();
    }
    let u_first = ordered_u[0];
    let u_last = ordered_u[section_count - 1];
    if u_last - u_first <= tolerance {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "coincident_stations",
            "guidedLoft: sections project to coincident guide stations",
        ));
    }

    // --- 3. Sample the guide at M = max(24, 6·nSections) stations spanning the
    //        sections' projected range.  At each station blend the bracketing
    //        sections (homogeneous, per control point), then place the blend:
    //        translation mode moves its centroid onto the guide; frame mode
    //        blends LOCAL (per-frame) coordinates and reconstructs through the
    //        station's RMF frame, so the sections rotate with the guide.
    let station_count = 24usize.max(6 * section_count);
    let station_params: Vec<f64> = (0..station_count)
        .map(|station| {
            let frac = station as f64 / (station_count - 1) as f64;
            u_first + (u_last - u_first) * frac
        })
        .collect();

    // Frame mode: one RMF strip over {stations ∪ user stations}, then each
    // user section expressed in the frame at its own station.  The homogeneous
    // control points store frame-local coordinates (weights untouched), so the
    // blend loop below is identical for both modes.
    let frames = if rotate_to_frame {
        Some(guided_frames(guide, g0, g1, &station_params, &ordered_u)?)
    } else {
        None
    };
    let blend_sources: Vec<Vec<NurbsCurve>> = if let Some(frames) = &frames {
        let mut localized = Vec::with_capacity(section_count);
        for (section_index, section) in ordered_sections.iter().enumerate() {
            let frame = frames.index_of(ordered_u[section_index])?;
            let origin = frames.points[frame];
            let (r, s, t_axis) = (
                frames.r_axes[frame],
                frames.s_axes[frame],
                frames.tangents[frame],
            );
            let mut local_section = Vec::with_capacity(curve_count);
            for curve in section {
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|point| {
                        let weight = point.w;
                        let local = Vec3::new(point.x / weight, point.y / weight, point.z / weight)
                            .sub(origin);
                        Vec4 {
                            x: local.dot(r) * weight,
                            y: local.dot(s) * weight,
                            z: local.dot(t_axis) * weight,
                            w: weight,
                        }
                    })
                    .collect();
                local_section.push(
                    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                        .or_refuse(KernelStage::Refine, "local_curve")?,
                );
            }
            localized.push(local_section);
        }
        localized
    } else {
        ordered_sections.clone()
    };

    let mut blended_sections: Vec<Vec<NurbsCurve>> = Vec::with_capacity(station_count);
    for &t in &station_params {
        // Locate the interval [ordered_u[i], ordered_u[i+1]] containing t.
        let mut interval = 0usize;
        while interval + 1 < section_count - 1 && ordered_u[interval + 1] <= t {
            interval += 1;
        }
        let u_lo = ordered_u[interval];
        let u_hi = ordered_u[interval + 1];
        let span = u_hi - u_lo;
        if span <= tolerance {
            // The same text as the whole-run check above; this one re-checks
            // each bracketing pair behind the strict-monotonicity gate.
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "coincident_interval",
                "guidedLoft: sections project to coincident guide stations",
            ));
        }
        let f = ((t - u_lo) / span).clamp(0.0, 1.0);
        let section_lo = &blend_sources[interval];
        let section_hi = &blend_sources[interval + 1];
        // Per-curve, per-control-point homogeneous linear blend.  Because loft
        // compatibility already forced matching weights across the input
        // sections, the blended weight equals the shared weight for every
        // station — so all blended sections remain mutually loft-compatible.
        let mut blended: Vec<NurbsCurve> = Vec::with_capacity(curve_count);
        for curve_index in 0..curve_count {
            let curve_lo = &section_lo[curve_index];
            let curve_hi = &section_hi[curve_index];
            let control_points = curve_lo
                .control_points
                .iter()
                .zip(&curve_hi.control_points)
                .map(|(a, b)| Vec4 {
                    x: a.x * (1.0 - f) + b.x * f,
                    y: a.y * (1.0 - f) + b.y * f,
                    z: a.z * (1.0 - f) + b.z * f,
                    w: a.w * (1.0 - f) + b.w * f,
                })
                .collect();
            blended.push(
                NurbsCurve::new(curve_lo.degree, curve_lo.knots.clone(), control_points)
                    .or_refuse(KernelStage::Refine, "blend_curve")?,
            );
        }
        let placed: Vec<NurbsCurve> = if let Some(frames) = &frames {
            // Reconstruct frame-local coordinates through the station's frame.
            let frame = frames.index_of(t)?;
            let origin = frames.points[frame];
            let (r, s, t_axis) = (
                frames.r_axes[frame],
                frames.s_axes[frame],
                frames.tangents[frame],
            );
            let mut placed = Vec::with_capacity(curve_count);
            for curve in &blended {
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|point| {
                        let weight = point.w;
                        let world = origin
                            .add(r.scale(point.x / weight))
                            .add(s.scale(point.y / weight))
                            .add(t_axis.scale(point.z / weight));
                        Vec4 {
                            x: world.x * weight,
                            y: world.y * weight,
                            z: world.z * weight,
                            w: weight,
                        }
                    })
                    .collect();
                placed.push(
                    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                        .or_refuse(KernelStage::Refine, "placed_curve")?,
                );
            }
            placed
        } else {
            // Rigidly translate the blend so its centroid lands on G(t).
            let blended_centroid = guided_section_centroid(&blended)?;
            let guide_point = guide
                .evaluate(g0 + (g1 - g0) * t)
                .or_refuse(KernelStage::Refine, "guide_station")?;
            let delta = guide_point.sub(blended_centroid);
            let mut placed = Vec::with_capacity(curve_count);
            for curve in &blended {
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|point| Vec4 {
                        x: point.x + point.w * delta.x,
                        y: point.y + point.w * delta.y,
                        z: point.z + point.w * delta.z,
                        w: point.w,
                    })
                    .collect();
                placed.push(
                    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                        .or_refuse(KernelStage::Refine, "translated_curve")?,
                );
            }
            placed
        };
        blended_sections.push(placed);
    }

    // --- 4. Loft through the guided sections (side walls + planar end caps).
    // The inner loft's own class rides through the wrapper.
    loft_profile_brep(&blended_sections).map_err(|error| {
        error.with_message(|error| {
            format!("guidedLoft: loft through guided sections failed: {error}")
        })
    })
}
