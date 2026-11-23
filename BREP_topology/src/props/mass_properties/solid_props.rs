pub use super::*;
use super::mass_profile;
use web_time::Instant;

/// The static facts a `mass.face` line carries beside the route: the
/// recognized carrier kind, the patch degrees and closed directions, and the
/// trim's size. Read only when `BREP_PROFILE_MASS_FACES` armed the lines.
fn face_line_meta(face: &FaceRecord) -> (&'static str, usize, usize, bool, bool, usize, usize) {
    // `kind_label` spells the general carrier with spaces; the profile line is
    // whitespace-tokenized, so collapse it to one word.
    let kind = match face.surface.analytic().map(|analytic| analytic.kind_label()) {
        Some("Surface of revolution") => "Revolution",
        Some(label) => label,
        None => "Fitted",
    };
    let (closed_u, closed_v) = face.surface.closed_directions().unwrap_or((false, false));
    let coedges = face
        .loops
        .iter()
        .map(|loop_record| loop_record.coedges.len())
        .sum();
    (
        kind,
        face.surface.degree_u,
        face.surface.degree_v,
        closed_u,
        closed_v,
        face.loops.len(),
        coedges,
    )
}

/// Emit one `mass.face` line for the face just integrated. `before` is the
/// counter snapshot taken ahead of its work and `started` the wall clock at
/// the same point; both are `None` unless the lines are armed.
fn emit_face_line(
    call: &str,
    shell: usize,
    face_index: usize,
    face: &FaceRecord,
    before: Option<super::profile::MassProfile>,
    started: Option<Instant>,
) {
    let (Some(before), Some(started)) = (before, started) else {
        return;
    };
    let wall_ms = started.elapsed().as_secs_f64() * 1_000.0;
    let (kind, degree_u, degree_v, closed_u, closed_v, loops, coedges) = face_line_meta(face);
    super::profile::emit_face(
        call, shell, face_index, kind, degree_u, degree_v, closed_u, closed_v, loops, coedges,
        before, wall_ms,
    );
}

pub fn solid_mass_properties(solid: &BrepSolid) -> Result<MassProperties, String> {
    let token = super::profile::begin("solid_mass_properties");
    let result = solid_mass_properties_cached(solid);
    super::profile::end(token);
    if result.is_ok() && sign_terms_oracle_enabled() && solid.kind == crate::BodyKind::Solid
        && !solid.shells.is_empty() && solid.shells.iter().all(|shell| !shell.faces.is_empty())
    {
        verify_solid_sign_terms(solid)?;
    }
    if let (Some(dump), Ok(properties)) = (super::profile::identity_dump(), result.as_ref()) {
        super::profile::emit_identity(
            dump,
            "solid_mass_properties",
            solid,
            &[
                ("volume_bits", properties.volume),
                ("area_bits", properties.surface_area),
            ],
        );
    }
    result
}

/// Validate the memo against the current geometry on EVERY read: topology
/// records are publicly editable, and copies may have been transformed or healed.
fn solid_mass_properties_cached(solid: &BrepSolid) -> Result<MassProperties, String> {
    let identity = face_set_identity(solid);
    if let Some((cached_identity, properties)) = solid.mass_properties_cache.entry.get() {
        if cached_identity == identity {
            if super::profile::mass_profile_enabled() {
                let measured = solid_mass_properties_inner(solid)?;
                assert_eq!(properties.volume.to_bits(), measured.volume.to_bits(),
                    "mass cache changed volume");
                assert_eq!(properties.surface_area.to_bits(), measured.surface_area.to_bits(),
                    "mass cache changed area");
                eprintln!("mass.cache hit faces={} verified=true", identity.faces);
            }
            return Ok(properties);
        }
    }
    // Never retain a failed computation, nor an old entry for modified geometry.
    solid.mass_properties_cache.entry.set(None);
    let properties = solid_mass_properties_inner(solid)?;
    solid.mass_properties_cache.entry.set(Some((identity, properties)));
    Ok(properties)
}

pub fn solid_mass_properties_inner(solid: &BrepSolid) -> Result<MassProperties, String> {
    let mut properties = MassProperties {
        surface_area: 0.0,
        volume: 0.0,
    };
    let mut volume_compensation = 0.0;
    let face_lines = super::profile::face_lines_enabled();
    for (shell_index, shell) in solid.shells.iter().enumerate() {
        if shell.faces.is_empty() {
            continue;
        }
        // Use the same local reference as the signed-volume orientation gate.
        // World-origin flux terms can dwarf a small translated solid's volume;
        // even tiny quadrature or boundary errors then dominate their sum.
        let reference = shell_volume_reference(shell)?;
        let kinds = [Integrand::Area, Integrand::VolumeAbout(reference)];
        for (face_index, face) in shell.faces.iter().enumerate() {
            mass_profile(|p| p.faces += 1);
            let line_before = face_lines.then(super::profile::snapshot);
            let line_started = face_lines.then(Instant::now);
            let gate_started = super::profile::profile_started();
            let (area, volume) = if let Some(values) = biperiodic_band_integral(face, &kinds)? {
                mass_profile(|p| {
                    p.route_biperiodic += 1;
                    p.gate_ms += super::profile::elapsed_ms(gate_started);
                });
                (values[0], values[1] / 3.0)
            } else if !is_affine(&face.surface)? && !is_untrimmed(face)? {
                mass_profile(|p| {
                    p.trimmed_faces += 1;
                    p.gate_ms += super::profile::elapsed_ms(gate_started);
                });
                // Preserve the shared cell decomposition and integration pass.
                let values = integrate_trimmed_multi(face, &kinds)?;
                (values[0], values[1] / 3.0)
            } else if is_affine(&face.surface)? {
                mass_profile(|p| {
                    p.route_affine += 1;
                    p.gate_ms += super::profile::elapsed_ms(gate_started);
                });
                // One parameter-space area and one planar metric for both
                // numbers, instead of `face_area` and
                // `face_volume_contribution_about` each walking the trim.
                affine_area_and_volume_about(face, reference)?
            } else {
                mass_profile(|p| {
                    p.route_untrimmed += 1;
                    p.gate_ms += super::profile::elapsed_ms(gate_started);
                });
                // One station sweep carrying both integrands, instead of one
                // sweep per integrand.
                let started = super::profile::profile_started();
                let values = integrate_untrimmed_multi(face, &kinds)?;
                mass_profile(|p| p.untrimmed_ms += super::profile::elapsed_ms(started));
                (values[0], values[1] / 3.0)
            };
            emit_face_line(
                "solid_mass_properties",
                shell_index,
                face_index,
                face,
                line_before,
                line_started,
            );
            properties.surface_area += area;
            let next = properties.volume + volume;
            if properties.volume.abs() >= volume.abs() {
                volume_compensation += (properties.volume - next) + volume;
            } else {
                volume_compensation += (volume - next) + properties.volume;
            }
            properties.volume = next;
        }
    }
    properties.volume += volume_compensation;
    Ok(properties)
}

/// Exact signed volume. A single nonempty shell shares its area/volume
/// measurement with subsequent property queries. Multiple shells retain the
/// original per-shell compensated sum: substituting the across-face sum can
/// change the last bits and an orientation gate must not change its answer.
pub fn solid_signed_volume(solid: &BrepSolid) -> Result<f64, String> {
    let token = super::profile::begin("solid_signed_volume");
    let result = if solid.shells.len() == 1 && !solid.shells[0].faces.is_empty() {
        match solid_mass_properties_cached(solid) {
            Ok(properties) => {
                if super::profile::mass_profile_enabled() {
                    let original = solid_signed_volume_uncached(solid)
                        .expect("shared mass measurement changed a signed-volume refusal");
                    assert_eq!(properties.volume.to_bits(), original.to_bits(),
                        "shared mass measurement changed signed volume");
                }
                Ok(properties.volume)
            }
            // Area computation must not introduce a new signed-volume refusal.
            Err(_) => solid_signed_volume_uncached(solid),
        }
    } else {
        solid_signed_volume_uncached(solid)
    };
    super::profile::end(token);
    if result.is_ok() && sign_terms_oracle_enabled() && solid.kind == crate::BodyKind::Solid {
        verify_solid_sign_terms(solid)?;
    }
    if let (Some(dump), Ok(volume)) = (super::profile::identity_dump(), result.as_ref()) {
        super::profile::emit_identity(dump, "solid_signed_volume", solid, &[("signed_bits", *volume)]);
    }
    result
}

/// The margin, as a fraction of the sum of the face contributions' magnitudes,
/// inside which [`solid_signed_volume_for_sign`] trusts no reading of its own
/// and integrates exactly. The skipped corrections are zero to rounding
/// (collinear control points, 1e-12 of the extent), so this is a very wide
/// safety margin, not a measured error.
const SIGN_READING_MARGIN: f64 = 1e-6;

/// [`solid_signed_volume_for_sign`] for a caller that compares the result with
/// `bar` and nothing else (the boolean assembly's closed tail, the shell's
/// orientation, the direct edit's inversion check — 2026-10-17): the memoised
/// sign reading, whose verdict against the bar is the exact integral's.
/// `BREP_SIGN_READERS_EXACT=1` restores the exact read at those call sites,
/// for an on/off census. Read once.
pub fn signed_volume_against(solid: &BrepSolid, bar: f64) -> Result<f64, String> {
    static EXACT: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *EXACT.get_or_init(|| std::env::var("BREP_SIGN_READERS_EXACT").as_deref() == Ok("1")) {
        return solid_signed_volume(solid);
    }
    solid_signed_volume_for_sign(solid, bar)
}

/// [`solid_signed_volume`] read for its SIGN against `bar`, as the boolean's
/// assembly reads it (`volume <= tolerance³` refuses an inverted shell).
///
/// The exact integration spends most of a trimmed wall's time on the sliver
/// correction behind every chord of its rim pcurves; where such a pcurve is a
/// straight line in uv (a bore's rims, 85 % of the plate case's 105 ms check:
/// 256 chords per wall half, 442 368 correction stations in all) the sliver
/// has no area and the correction is zero to rounding, so this reading skips
/// it (`trimmed::with_straight_corrections_skipped`). Face volume and vector
/// area terms are integrated together and retained by content, then evaluated
/// about the same shell reference as `solid_signed_volume`.
///
/// Native two-sphere supports with the certified trim scope can return a
/// positive lower bound; `BREP_SPHERE_SIGN_BOUND=0|off` keeps the old path. Only a
/// comparison against `bar` is meaningful for that result; it is never stored
/// in the full mass cache. If the bound cannot prove `> bar`, the old path runs.
///
/// The verdict is the exact check's: when the reading sits within
/// `SIGN_READING_MARGIN` of the magnitudes of the bar, the exact value is
/// returned instead, bit for bit. Callers compare the result with `bar`.
pub fn solid_signed_volume_for_sign(solid: &BrepSolid, bar: f64) -> Result<f64, String> {
    let enabled = match std::env::var("BREP_SPHERE_SIGN_BOUND") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    };
    let result = solid_signed_volume_for_sign_with_bound(solid, bar, enabled);
    if result.is_ok() && sign_terms_oracle_enabled() {
        verify_solid_sign_terms(solid)?;
    }
    result
}

pub fn solid_signed_volume_for_sign_with_bound(solid: &BrepSolid, bar: f64, enabled: bool) -> Result<f64, String> {
    if enabled {
        if let Some(lower) = super::sphere_sign::positive_volume_bound(solid) {
            if bar.is_finite() && lower > bar {
                if crate::profile::profile_enabled() {
                    profile_line!("mass.sign_certificate kind=trimmed_spheres lower={lower:.9} bar={bar:.3e}");
                }
                return Ok(lower);
            }
        }
    }
    if sliver_bound_enabled() {
        if let Some(lower) = sliver_bound_certificate(solid, bar) {
            return Ok(lower);
        }
    }
    let token = super::profile::begin("solid_signed_volume_for_sign");
    let outcome = solid_sign_terms_reading(solid);
    super::profile::end(token);
    let (reading, magnitude) = outcome?;
    if (reading - bar).abs() > SIGN_READING_MARGIN * magnitude {
        return Ok(reading);
    }
    solid_signed_volume(solid)
}

/// The quadtree depth of [`solid_sign_coarse_certificate`]: boundary cells
/// split this often, against four times for the exact reading.
pub const COARSE_SIGN_DEPTH: usize = 1;

/// `BREP_SEW_SIGN_COARSE_THIN` (default on since the 2026-11-20 tier-P
/// promotion; `=0` restores the coarse quadtree on every face): in
/// [`solid_sign_coarse_certificate`], a face whose trim cannot hold one whole
/// coarse cell ([`trim_holds_no_coarse_cell`]) is read by its exact route
/// instead of the coarse quadtree. The coarse pass reads nothing of such a
/// face and bounds all of it by its net, so this only replaces a bound with
/// the reading's own terms.
pub fn sign_coarse_thin_enabled() -> bool {
    crate::switch!("BREP_SEW_SIGN_COARSE_THIN", true)
}

/// Whether no cell of the coarse quadtree at `depth` can lie inside the
/// face's trim, so that the coarse pass would integrate none of it. The
/// quadtree's root cells are the carrier's knot spans, halved `depth` times;
/// a region narrower in u (or in v) than those cells on every span it meets
/// holds none whole. The region is boxed by its pcurves' control points
/// (with positive weights each pcurve lies in their hull). A routing test
/// only: either answer leaves the certificate sound. `false` when the box
/// cannot be read, or lies off the carrier's spans (an unwrapped trim).
fn trim_holds_no_coarse_cell(face: &FaceRecord, depth: usize) -> bool {
    let Ok((u_breaks, v_breaks)) = surface_breaks(&face.surface) else {
        return false;
    };
    let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
        for control in &coedge.pcurve.control_points {
            let Ok(point) = control.point() else {
                return false;
            };
            if !(control.w > 0.0) || !point.x.is_finite() || !point.y.is_finite() {
                return false;
            }
            low = [low[0].min(point.x), low[1].min(point.y)];
            high = [high[0].max(point.x), high[1].max(point.y)];
        }
    }
    let cell = 0.5f64.powi(depth.min(60) as i32);
    let narrow = |breaks: &[f64], low: f64, high: f64| -> bool {
        let mut met = false;
        for pair in breaks.windows(2) {
            if pair[1] <= low || pair[0] >= high {
                continue;
            }
            met = true;
            if !(high - low < (pair[1] - pair[0]) * cell) {
                return false;
            }
        }
        met
    };
    low[0] <= high[0] && low[1] <= high[1]
        && (narrow(&u_breaks, low[0], high[0]) || narrow(&v_breaks, low[1], high[1]))
}

/// [`solid_sign_coarse_certificate`]'s reading and the half-width of the
/// interval it certifies.
#[derive(Clone, Copy, Debug)]
pub struct CoarseSign {
    /// The faces' contributions with every boundary cell left at the coarse
    /// depth skipped.
    pub reading: f64,
    /// The skipped regions' bound plus the rounding slack: the volume lies
    /// in `reading ± bound`.
    pub bound: f64,
    /// The skipped parameter-plane measure, summed over the faces (a census
    /// figure).
    pub skipped: f64,
}

/// A CERTIFICATE for the sign of [`solid_signed_volume`] read with a coarse
/// quadtree: `Some(CoarseSign)` whose `reading ± bound` holds the volume, or
/// `None` when no bound can be stated (the caller then reads the sign
/// exactly).
///
/// Every face is integrated by its exact route, at the exact route's
/// stations, except that a trimmed face's boundary cells refine only to
/// `depth` and the partial cells left there are NOT integrated
/// ([`super::trimmed::with_coarse_quadtree`]). The full cells, the chord
/// corrections and every other route are term for term the exact reading's.
/// Under `BREP_SEW_SIGN_COARSE_THIN` ([`sign_coarse_thin_enabled`]) a face
/// whose trim cannot hold one whole cell at `depth`
/// ([`trim_holds_no_coarse_cell`]) is read whole by its exact route, with no
/// skipped region and so no bound: a thin strip of a full-turn carrier, whose
/// coarse cells are all boundary cells, otherwise adds its whole area times
/// the net's `n · r` to the bound.
///
/// The bound, per face. The volume flux is
/// `(1/3) ∫∫_R (S − ref)·(S_u × S_v) du dv` over the trimmed region `R`.
/// A skipped region `K` (the trim fragments clipped to the skipped cells)
/// therefore contributes at most `(1/3) |K| · sup|S − ref| · sup|S_u × S_v|`
/// in magnitude. Here:
/// - `|K|` is the sum of the fragments' absolute shoelace areas. That is
///   exactly their measure for simple trim loops: a simple loop's winding
///   number is 0 or ±1, and the clip keeps it.
/// - `|S − ref| ≤ |S − local| + |local − ref| ≤ r + |local − ref|`.
/// - `|S_u × S_v| ≤ n`.
/// - `(n, r)` are the control-net bounds about `local` of the knot span
///   holding `K` ([`surface_flux_bounds_on_span`]: by local support only that
///   span's `(p+1)(q+1)` controls weigh there). A cell that is not inside one
///   span, or that lies past the domain, takes the whole net's
///   ([`surface_flux_bounds`], valid over the domain and its periodic wraps).
///   A skipped region past an OPEN end of the carrier declines, because no
///   net bound holds on the extension.
///
/// So the true integral of the skipped regions lies within `Σ` of those
/// bounds of zero. The reading is what the exact reading would give with
/// those regions removed. A sign is decided only when `|reading ∓ bar|`
/// exceeds the bound.
///
/// What this certifies: the sign of any integration that agrees with the
/// exact reading on the cells and corrections both share, and reads the
/// skipped regions within their true bound. The exact reading's own
/// quadrature of those regions (finer cells and triangle cubature) is not
/// itself bounded here.
///
/// Declined (`None`):
/// - a face on the hopped unwrap route, which chooses its region by an area
///   read over cells the coarse pass skips;
/// - a carrier with no net bound (a non-positive weight), or a call whose
///   kinds carry no volume reference;
/// - any non-finite value.
///
/// Rounding: the reading re-associates no station within a face. A skipped
/// cell only removes addends, so `1e-9` of the faces' scale covers the
/// difference in summation order, as in [`sliver_bound_certificate`].
/// Nothing is memoised.
pub fn solid_sign_coarse_certificate(solid: &BrepSolid, depth: usize) -> Option<CoarseSign> {
    let (mut reading, mut bound, mut scale, mut skipped) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    // Early out: a sign is settled only when |reading| exceeds the bound, and
    // a body's volume is at most that of a box holding its carriers' controls
    // (their convex hulls hold the faces). Once the bound passes that box,
    // nothing is left to settle. Stopping only falls through to the exact
    // path, so this never weakens the certificate.
    let reach = control_box_volume(solid);
    // The faces in falling order of what their skipped cells could add to the
    // bound (the whole net's `n · r` over the carrier's domain, read from
    // controls alone), so that a body the certificate cannot settle reaches
    // the early out after a few faces rather than most of them. On the
    // bench's spiroid, a few cone walls (integrand zero about their apex,
    // net bound large) carry most of the bound. The order changes only the
    // summation order, which the slack covers.
    let mut faces = Vec::new();
    for shell in &solid.shells {
        let reference = shell_volume_reference(shell).ok()?;
        for face in &shell.faces {
            let local = face_volume_reference(face).unwrap_or(reference);
            faces.push((face, local, reference, bound_potential(&face.surface, local)));
        }
    }
    faces.sort_by(|a, b| b.3.total_cmp(&a.3));
    {
        let thin_route = sign_coarse_thin_enabled();
        for &(face, local, reference, _) in &faces {
            let (terms, measure) = if thin_route && trim_holds_no_coarse_cell(face, depth) {
                // Read whole by its exact route: bound zero, the reading's own terms.
                (face_volume_terms_about(face, local), super::trimmed::CoarseMeasure { depth, ..Default::default() })
            } else {
                super::trimmed::with_coarse_quadtree(depth, || face_volume_terms_about(face, local))
            };
            let terms = terms.ok()?;
            if measure.declined || !terms.iter().all(|v| v.is_finite()) {
                return None;
            }
            let offset = local.sub(reference);
            let area = Vec3::new(terms[1], terms[2], terms[3]);
            let contribution = terms[0] + offset.dot(area);
            let face_bound = (measure.flux + measure.normal * offset.length()) / 3.0;
            reading += contribution;
            bound += face_bound;
            skipped += measure.skipped;
            scale += terms[0].abs() + offset.length() * area.length() + face_bound;
            if !(bound <= reach) {
                return None;
            }
        }
    }
    let bound = bound + 1e-9 * scale;
    [reading, bound, skipped].iter().all(|v| v.is_finite()).then_some(CoarseSign { reading, bound, skipped })
}

/// The whole net's `n · r` about `center` times the carrier's parameter
/// domain area: an ordering key for [`solid_sign_coarse_certificate`], not a
/// bound it relies on. Infinite where no net bound holds, so such a face
/// (which declines) comes first.
fn bound_potential(surface: &crate::NurbsSurface, center: Vec3) -> f64 {
    let span = |knots: &[f64], degree: usize| -> f64 {
        match (knots.get(degree), knots.len().checked_sub(degree + 1).and_then(|i| knots.get(i))) {
            (Some(low), Some(high)) => (high - low).abs(),
            _ => f64::INFINITY,
        }
    };
    let area = span(&surface.knots_u, surface.degree_u) * span(&surface.knots_v, surface.degree_v);
    match surface_flux_bounds(surface, center) {
        Some((n, r)) if (n * r * area).is_finite() => n * r * area,
        _ => f64::INFINITY,
    }
}

/// The volume of the axis-aligned box holding every face carrier's control
/// points; infinite when there are none or any is not finite.
fn control_box_volume(solid: &BrepSolid) -> f64 {
    let (mut low, mut high) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for control in face.surface.control_points.iter().flatten() {
            let Ok(point) = control.point() else { return f64::INFINITY };
            for (axis, value) in [point.x, point.y, point.z].into_iter().enumerate() {
                low[axis] = low[axis].min(value);
                high[axis] = high[axis].max(value);
            }
        }
    }
    let volume = (0..3).map(|axis| high[axis] - low[axis]).product::<f64>();
    if volume.is_finite() { volume } else { f64::INFINITY }
}

/// `BREP_SIGN_SLIVER_BOUND` (on by default since 2026-11-03,
/// tier-P promotion; `0`/`off` restores the reading alone): before the sign
/// reading, try a certificate that integrates every face's cells exactly as
/// the reading does but skips the surface stations of its chord corrections,
/// bounding them instead (see [`sliver_bound_certificate`]). Read on every
/// call.
fn sliver_bound_enabled() -> bool {
    switch!(live "BREP_SIGN_SLIVER_BOUND", true)
}


/// A lower bound on the sign reading that proves its verdict, or `None`.
///
/// The reading sums, per face, the cells' quadrature plus the chord
/// corrections' quadrature `Σ w_k f(Q_k)` (`trimmed::curved_boundary_correction`,
/// positive Gauss weights times the sliver jacobian). Its correction is at most
/// `Σ |w_k| · sup |f|`: the first factor is the face's sliver measure
/// (`trimmed::with_sliver_measure`, read from the pcurves alone), the second
/// comes from the carrier's control net ([`surface_flux_bounds`]): the volume
/// integrand `(P - local) · N` is at most `r · n`, the vector area's `N` at most
/// `n`, so a face's contribution about the shell reference moves by at most
/// `measure · n · (r + |local - reference|) / 3`. Faces the reading already
/// holds exact terms for are taken as they are (bound zero); faces whose route
/// reads its own corrections are integrated exactly, as the reading does.
///
/// The certificate stands only when the bounded sum clears the bar by the
/// reading's own margin, with every bound and a rounding slack added to the
/// magnitude: then the reading would return a value above the bar without its
/// exact fallback, and this returns a lower bound on that value. Otherwise
/// `None`, and the reading runs as before (an error anywhere also yields
/// `None`, so the reading reports it).
///
/// Under `BREP_SIGN_CANONICAL_LOOPS` ([`sign_canonical_loops_mode`]) every face
/// is read in its canonical rotation ([`canonical_loops`]), so that a face a
/// later boolean hands back with its loops started one coedge further on keys
/// and reads the same terms as before.
pub fn sliver_bound_certificate(solid: &BrepSolid, bar: f64) -> Option<f64> {
    match sign_canonical_loops_mode() {
        crate::env_read::Mode::Off => sliver_bound_reading(solid, bar, false),
        crate::env_read::Mode::On => sliver_bound_reading(solid, bar, true),
        crate::env_read::Mode::Check => {
            let given = sliver_bound_reading(solid, bar, false);
            let canonical = sliver_bound_reading(solid, bar, true);
            canonical_loops_check(solid, given, canonical);
            given
        }
    }
}

/// `BREP_SIGN_CANONICAL_LOOPS` (default on since the 2026-11-21 tier-P
/// promotion; `0|off` reads every face as given; `check` reads both and prints
/// one `mass.sign_canonical_check` line per certificate, the given rotation
/// deciding): see [`sliver_bound_certificate`].
pub fn sign_canonical_loops_mode() -> crate::env_read::Mode {
    crate::mode_switch!(live "BREP_SIGN_CANONICAL_LOOPS", true)
}


/// `face` with each loop started at the coedge that makes its sequence of
/// `(pcurve identity, sense)` least (loop order kept): the same region, read
/// from the same pcurves, whichever coedge a boolean happened to start the
/// loop at. Borrowed when every loop already starts there.
pub fn canonical_loops(face: &FaceRecord) -> std::borrow::Cow<'_, FaceRecord> {
    let starts: Vec<usize> = face
        .loops
        .iter()
        .map(|loop_record| {
            let keys: Vec<([u64; 2], bool)> = loop_record
                .coedges
                .iter()
                .map(|coedge| {
                    let mut lane = super::identity::TwoLane::new();
                    super::identity::hash_curve(&mut lane, &coedge.pcurve);
                    (lane.finish(), coedge.forward)
                })
                .collect();
            let n = keys.len();
            (0..n)
                .min_by(|&a, &b| (0..n).map(|k| keys[(a + k) % n]).cmp((0..n).map(|k| keys[(b + k) % n])))
                .unwrap_or(0)
        })
        .collect();
    if starts.iter().all(|&start| start == 0) {
        return std::borrow::Cow::Borrowed(face);
    }
    let mut rotated = face.clone();
    for (loop_record, start) in rotated.loops.iter_mut().zip(starts) {
        loop_record.coedges.rotate_left(start);
    }
    std::borrow::Cow::Owned(rotated)
}

/// The key the sign certificate's memo holds `face`'s terms under: its content
/// identity about its own volume reference (`reference` when it has none).
pub fn sign_face_identity(face: &FaceRecord, reference: Vec3) -> [u64; 2] {
    let local = face_volume_reference(face).unwrap_or(reference);
    super::identity::face_contribution_identity(face, local)
}

/// The check mode of `BREP_SIGN_CANONICAL_LOOPS`: both certificates' verdicts,
/// and per face the worst relative difference between its bounded terms read
/// in the given and the canonical rotation (its contribution about the shell
/// reference against the face's own magnitude, and its vector area).
fn canonical_loops_check(solid: &BrepSolid, given: Option<f64>, canonical: Option<f64>) {
    let (mut faces, mut rotated, mut worst_contribution, mut worst_area, mut unread) = (0usize, 0usize, 0.0f64, 0.0f64, 0usize);
    let mut worst_face = String::from("-");
    let mut worst_of_bound = 0.0f64;
    // The certificate's margin is relative to the faces' summed magnitudes, so
    // each face's difference is read against that too.
    let (mut worst_absolute, mut total_magnitude) = (0.0f64, 0.0f64);
    super::trimmed::with_straight_corrections_skipped(|| {
        for shell in &solid.shells {
            let Ok(reference) = shell_volume_reference(shell) else { continue };
            for face in &shell.faces {
                faces += 1;
                if let Some(local) = face_volume_reference(face) {
                    if let Ok(terms) = face_volume_terms_about(face, local) {
                        let area = Vec3::new(terms[1], terms[2], terms[3]);
                        total_magnitude += (terms[0] + local.sub(reference).dot(area)).abs();
                    }
                }
                let canon = canonical_loops(face);
                let std::borrow::Cow::Owned(canon) = canon else { continue };
                rotated += 1;
                let read = |face: &FaceRecord| {
                    let local = face_volume_reference(face).unwrap_or(reference);
                    let offset = local.sub(reference).length();
                    let (terms, bound) = match bounded_face_terms(face, local) {
                        Some((terms, per_offset, fixed, _)) => (terms, fixed + per_offset * offset),
                        None => (face_volume_terms_about(face, local).ok()?, 0.0),
                    };
                    let area = Vec3::new(terms[1], terms[2], terms[3]);
                    Some((terms[0] + local.sub(reference).dot(area), area, terms[0].abs() + offset * area.length(), bound))
                };
                match (read(face), read(&canon)) {
                    (Some((c0, a0, m0, b0)), Some((c1, a1, m1, b1))) => {
                        // Both readings leave the same region's corrections out and bound them:
                        // their difference must lie inside the two bounds together.
                        worst_of_bound = worst_of_bound.max((c0 - c1).abs() / (b0 + b1).max(f64::MIN_POSITIVE));
                        worst_absolute = worst_absolute.max((c0 - c1).abs());
                        let magnitude = m0.max(m1).max(f64::MIN_POSITIVE);
                        let relative = (c0 - c1).abs() / magnitude;
                        if relative > worst_contribution {
                            worst_contribution = relative;
                            let (closed_u, closed_v) = face.surface.closed_directions().unwrap_or((false, false));
                            worst_face = format!(
                                "{}:{}{}:loops{}",
                                face.surface.analytic().map_or("freeform", |kind| kind.kind_label()).replace(' ', "_"),
                                if closed_u { "U" } else { "u" },
                                if closed_v { "V" } else { "v" },
                                face.loops.len()
                            );
                        }
                        worst_area = worst_area.max(a0.sub(a1).length() / a0.length().max(a1.length()).max(f64::MIN_POSITIVE));
                    }
                    _ => unread += 1,
                }
            }
        }
    });
    let verdict = |value: Option<f64>| if value.is_some() { "certified" } else { "none" };
    // Both certify, or both leave the sign to the reading: anything else is a
    // disagreement to look at, though either outcome stays sound.
    let agree = given.is_some() == canonical.is_some();
    eprintln!(
        "mass.sign_canonical_check{} given={} canonical={} faces={faces} rotated={rotated} unread={unread} worst_rel_contribution={worst_contribution:.3e} worst_rel_area={worst_area:.3e} worst_face={worst_face} difference_of_bound={worst_of_bound:.3e} difference_of_shell={:.3e} thread={}",
        if agree { "" } else { " DISAGREE" },
        verdict(given),
        verdict(canonical),
        worst_absolute / total_magnitude.max(f64::MIN_POSITIVE),
        std::thread::current().name().unwrap_or("-"),
    );
}

/// A face's bounded sign terms read about `from` (the canonical rotation's
/// reference), restated about `to` (the given rotation's): the volume term moves
/// by `(from - to) · area`, exactly as the divergence theorem moves it, and the
/// correction bound is recomputed about `to` from the same control net and the
/// same skipped measure (`per_offset = measure · n / 3`, `fixed = per_offset ·
/// r`, with `(n, r)` the net's bounds about the point), so the certificate
/// reads what the given rotation reads, to rounding.
fn rebase_bounded(
    surface: &crate::NurbsSurface,
    from: Vec3,
    to: Vec3,
    (terms, per_offset, fixed, rounding): ([f64; 4], f64, f64, f64),
) -> Option<([f64; 4], f64, f64, f64)> {
    let terms = rebase_terms(from, to, terms);
    if per_offset == 0.0 && fixed == 0.0 {
        return Some((terms, 0.0, 0.0, rounding));
    }
    let (n_from, _) = surface_flux_bounds(surface, from)?;
    let (n_to, r_to) = surface_flux_bounds(surface, to)?;
    let measure = 3.0 * per_offset / n_from;
    let per_offset = measure * n_to / 3.0;
    let fixed = per_offset * r_to;
    [per_offset, fixed].iter().all(|v| v.is_finite()).then_some((terms, per_offset, fixed, rounding))
}

/// Volume and vector-area terms about `from`, restated about `to`.
fn rebase_terms(from: Vec3, to: Vec3, terms: [f64; 4]) -> [f64; 4] {
    let area = Vec3::new(terms[1], terms[2], terms[3]);
    [terms[0] + from.sub(to).dot(area), terms[1], terms[2], terms[3]]
}

fn sliver_bound_reading(solid: &BrepSolid, bar: f64, canonical: bool) -> Option<f64> {
    if !bar.is_finite() {
        return None;
    }
    let token = super::profile::begin("solid_sign_sliver_bound");
    // `BREP_SIGN_PREFETCH`: a large solid's face terms start on idle threads;
    // the loop below takes each finished one in place of computing it.
    let mut prefetched = SignPrefetch::spawn(solid, canonical);
    let outcome = super::trimmed::with_straight_corrections_skipped(|| -> Option<(f64, f64, f64, f64, [usize; 3])> {
        let (mut reading, mut magnitude, mut bound, mut scale) = (0.0, 0.0, 0.0, 0.0);
        let mut counts = [0usize; 3]; // exact terms held, bounded, integrated exactly
        let mut slot = 0usize;
        for shell in &solid.shells {
            let reference = shell_volume_reference(shell).ok()?;
            let (mut volume, mut compensation) = (0.0f64, 0.0f64);
            for face in &shell.faces {
                // The terms are read (and memoised) in the canonical rotation about its
                // own reference, then restated about the given rotation's, so every
                // number below is the given rotation's to rounding.
                let given = face_volume_reference(face).unwrap_or(reference);
                let held = if canonical { canonical_loops(face) } else { std::borrow::Cow::Borrowed(face) };
                let read_face: &FaceRecord = &held;
                let local = if canonical { face_volume_reference(read_face).unwrap_or(reference) } else { given };
                let identity = super::identity::face_contribution_identity(read_face, local);
                let moved = [local.x, local.y, local.z].map(f64::to_bits) != [given.x, given.y, given.z].map(f64::to_bits);
                let offset = given.sub(reference);
                let ready = prefetched.as_mut().and_then(|prefetched| prefetched.claim(slot));
                slot += 1;
                let (ready_bounded, ready_exact) = match ready {
                    Some((bounded, exact)) => (Some(bounded), exact),
                    None => (None, None),
                };
                let (terms, face_bound, face_scale) = match crate::reading_memo::peek_sign_terms(identity) {
                    Some(terms) => {
                        counts[0] += 1;
                        let terms = terms?;
                        (if moved { rebase_terms(local, given, terms) } else { terms }, 0.0, 0.0)
                    }
                    None => match crate::reading_memo::memoized_bounded_sign_terms(identity, || {
                        prefetched_or(ready_bounded, || bounded_face_terms(read_face, local))
                    }) {
                        Some(bounded) => {
                            counts[1] += 1;
                            let (terms, per_offset, fixed, rounding) =
                                if moved { rebase_bounded(&read_face.surface, local, given, bounded)? } else { bounded };
                            (terms, fixed + per_offset * offset.length(), rounding)
                        }
                        None => {
                            counts[2] += 1;
                            let terms = crate::reading_memo::memoized_sign_terms(identity, || {
                                prefetched_or(ready_exact, || face_volume_terms_about(read_face, local).ok())
                            })?;
                            (if moved { rebase_terms(local, given, terms) } else { terms }, 0.0, 0.0)
                        }
                    },
                };
                let area = Vec3::new(terms[1], terms[2], terms[3]);
                let contribution = terms[0] + offset.dot(area);
                magnitude += contribution.abs();
                bound += face_bound;
                scale += terms[0].abs() + offset.length() * area.length() + face_bound + face_scale * (1.0 + offset.length());
                let next = volume + contribution;
                if volume.abs() >= contribution.abs() {
                    compensation += (volume - next) + contribution;
                } else {
                    compensation += (contribution - next) + volume;
                }
                volume = next;
            }
            reading += volume + compensation;
        }
        Some((reading, magnitude, bound, scale, counts))
    });
    super::profile::end(token);
    if let Some(prefetched) = prefetched {
        prefetched.report();
    }
    let (reading, magnitude, bound, scale, counts) = outcome?;
    // Rounding: the reading adds each correction station to a running total
    // the cells started, and the certificate does not; a face's stations number
    // well under 1e6, so their re-association moves a slot by far less than
    // 1e-9 of the face's scale.
    let slack = 1e-9 * scale;
    let lower = reading - bound - slack;
    let certified = [reading, magnitude, bound, scale, lower].iter().all(|v| v.is_finite())
        && lower > bar
        && lower - bar > SIGN_READING_MARGIN * (magnitude + bound + slack);
    if crate::profile::profile_enabled() {
        profile_line!(
            "mass.sign_certificate kind=sliver_bound certified={certified} cells={reading:.9e} bound={bound:.3e} slack={slack:.3e} magnitude={magnitude:.3e} held={} bounded={} exact={}",
            counts[0], counts[1], counts[2]
        );
    }
    certified.then_some(lower)
}

/// `BREP_SIGN_PREFETCH` (default on; `0` off, `check` compares): the sliver
/// certificate's face terms for a solid of [`SIGN_PREFETCH_MIN_FACES`] or more
/// faces are computed on idle pool threads ahead of its face loop
/// ([`crate::claim_slots`]). Each helper computes exactly what the loop would
/// on a memo miss — [`bounded_face_terms`], and [`face_volume_terms_about`]
/// when that declines — under the same straight-chord flag, on a clone of the
/// face, about the same local reference. The loop is unchanged: it consults
/// the memos in the same order, and only where it would have computed a term
/// does it take the helper's instead (or compute it, when the helper has not
/// finished; it never waits). `check` computes every term on the loop as before
/// and compares it with the helper's (`mass.sign_prefetch` under
/// `BREP_PROFILE`). A helper's face-level profile counters are not added to
/// the loop's `mass.profile` line.
const SIGN_PREFETCH_MIN_FACES: usize = 64;

fn sign_prefetch_mode() -> crate::env_read::Mode {
    #[cfg(not(test))]
    {
        crate::mode_switch!("BREP_SIGN_PREFETCH", true)
    }
}


type BoundedTerms = Option<([f64; 4], f64, f64, f64)>;
type SignReady = (BoundedTerms, Option<Option<[f64; 4]>>);

struct SignJob {
    face: FaceRecord,
    local: Vec3,
}

fn sign_job(job: SignJob) -> SignReady {
    super::trimmed::with_straight_corrections_skipped(|| {
        let bounded = bounded_face_terms(&job.face, job.local);
        let exact = bounded.is_none().then(|| face_volume_terms_about(&job.face, job.local).ok());
        (bounded, exact)
    })
}

#[derive(Clone, Copy, Default)]
struct SignPrefetchTally {
    check: bool,
    hits: usize,
    compared: usize,
    disagree: usize,
}

thread_local! {
    /// The tally of the certificate loop running on this thread, if prefetched.
    static SIGN_PREFETCH_TALLY: std::cell::Cell<Option<SignPrefetchTally>> = const { std::cell::Cell::new(None) };
}

struct SignPrefetch {
    slots: crate::claim_slots::ClaimSlots<SignJob, SignReady>,
    previous: Option<SignPrefetchTally>,
}

impl SignPrefetch {
    fn spawn(solid: &BrepSolid, canonical: bool) -> Option<Self> {
        let mode = sign_prefetch_mode();
        let faces: usize = solid.shells.iter().map(|shell| shell.faces.len()).sum();
        if mode == crate::env_read::Mode::Off
            || faces < SIGN_PREFETCH_MIN_FACES
            || !crate::claim_slots::helpers_available()
        {
            return None;
        }
        let mut jobs = Vec::with_capacity(faces);
        for shell in &solid.shells {
            let reference = shell_volume_reference(shell).ok()?;
            for face in &shell.faces {
                let face = if canonical { canonical_loops(face).into_owned() } else { face.clone() };
                let local = face_volume_reference(&face).unwrap_or(reference);
                jobs.push(SignJob { face, local });
            }
        }
        let slots = crate::claim_slots::ClaimSlots::spawn(jobs, sign_job)?;
        let tally = SignPrefetchTally { check: mode == crate::env_read::Mode::Check, ..Default::default() };
        let previous = SIGN_PREFETCH_TALLY.with(|cell| cell.replace(Some(tally)));
        Some(Self { slots, previous })
    }

    fn claim(&mut self, index: usize) -> Option<SignReady> {
        self.slots.claim(index)
    }

    fn report(self) {
        let tally = SIGN_PREFETCH_TALLY.with(|cell| cell.replace(self.previous)).unwrap_or_default();
        if crate::profile::profile_enabled() {
            profile_line!(
                "mass.sign_prefetch faces={} check={} taken={} claimed={} hits={} compared={} disagree={}",
                self.slots.len(),
                u8::from(tally.check),
                self.slots.taken,
                self.slots.claimed,
                tally.hits,
                tally.compared,
                tally.disagree
            );
        }
    }
}

/// A helper's value when the loop holds one for this face, else `compute`;
/// in check mode always `compute`, compared with the helper's to the bit
/// (shortest round-trip rendering of every float).
fn prefetched_or<T: std::fmt::Debug>(ready: Option<T>, compute: impl FnOnce() -> T) -> T {
    let Some(value) = ready else {
        return compute();
    };
    let check = SIGN_PREFETCH_TALLY.with(|cell| cell.get()).is_some_and(|tally| tally.check);
    if !check {
        SIGN_PREFETCH_TALLY.with(|cell| cell.set(cell.get().map(|mut tally| { tally.hits += 1; tally })));
        return value;
    }
    let fresh = compute();
    let agrees = format!("{fresh:?}") == format!("{value:?}");
    SIGN_PREFETCH_TALLY.with(|cell| {
        cell.set(cell.get().map(|mut tally| {
            tally.compared += 1;
            tally.disagree += usize::from(!agrees);
            tally
        }))
    });
    fresh
}

/// One face's sign terms with the chord corrections bounded, as
/// `(terms, per_offset, fixed, rounding)`: the face's contribution about a
/// shell reference `ref` lies within `fixed + per_offset · |local - ref|` of
/// the reading's, and `rounding` is a scale for the certificate's slack.
/// `None` when the face's route reads its own corrections, a bound is not
/// finite, or the integration fails (the reading then integrates it).
pub fn bounded_face_terms(face: &FaceRecord, local: Vec3) -> Option<([f64; 4], f64, f64, f64)> {
    let (terms, measure) = super::trimmed::with_sliver_measure(|| face_volume_terms_about(face, local));
    let terms = terms.ok()?;
    if measure.declined || !terms.iter().all(|v| v.is_finite()) {
        return None;
    }
    if measure.measure == 0.0 {
        return Some((terms, 0.0, 0.0, measure.complement / 3.0));
    }
    let (n, r) = surface_flux_bounds(&face.surface, local)?;
    let per_offset = measure.measure * n / 3.0;
    let fixed = per_offset * r;
    [per_offset, fixed].iter().all(|v| v.is_finite()).then_some((terms, per_offset, fixed, measure.complement / 3.0))
}

/// Bounds from the control net of `surface`, valid over its whole domain (and
/// its periodic wraps): `n >= |S_u × S_v|` and `r >= |S - center|`.
///
/// With homogeneous controls `Q = (w (P - center), w)`, all weights positive:
/// `S - center` is a convex combination of the `P - center`, so `r` is their
/// largest length; the homogeneous derivative `A_u` is a convex combination
/// of `p (Q_{i+1} - Q_i) / (u_{i+p+1} - u_{i+1})` (spline differentiation,
/// partition of unity of the degree-lowered basis), so its point part and its
/// weight part are at most the largest lengths of those differences' parts;
/// and `S_u = (X_u - w_u (S - center)) / w` with `w >= ` the smallest weight.
/// `None` for a non-positive or non-finite weight or control.
pub fn surface_flux_bounds(surface: &crate::NurbsSurface, center: Vec3) -> Option<(f64, f64)> {
    let net = &surface.control_points;
    let rows = net.len();
    let columns = net.first()?.len();
    surface_flux_bounds_over(surface, center, 0..rows, 0..columns)
}

/// [`surface_flux_bounds`] over one knot span `[u_i, u_{i+1}) × [v_j,
/// v_{j+1})`: there only the controls `i-p..=i` × `j-q..=j` carry basis
/// weight (local support), so the same argument over that sub-net bounds
/// `|S_u × S_v|` and `|S - center|` on the span. `None` for an index
/// outside the net (the caller then takes the whole-net bound).
pub fn surface_flux_bounds_on_span(surface: &crate::NurbsSurface, center: Vec3, span_u: usize, span_v: usize) -> Option<(f64, f64)> {
    let (p, q) = (surface.degree_u, surface.degree_v);
    if span_u < p || span_v < q {
        return None;
    }
    surface_flux_bounds_over(surface, center, span_u - p..span_u + 1, span_v - q..span_v + 1)
}

fn surface_flux_bounds_over(
    surface: &crate::NurbsSurface,
    center: Vec3,
    row_range: std::ops::Range<usize>,
    column_range: std::ops::Range<usize>,
) -> Option<(f64, f64)> {
    let net = &surface.control_points;
    let rows = net.len();
    let columns = net.first()?.len();
    if rows == 0 || columns == 0 || net.iter().any(|row| row.len() != columns)
        || row_range.is_empty() || column_range.is_empty() || row_range.end > rows || column_range.end > columns
    {
        return None;
    }
    let relative = |i: usize, j: usize| -> Option<[f64; 4]> {
        let q = net[i][j];
        let values = [q.x - center.x * q.w, q.y - center.y * q.w, q.z - center.z * q.w, q.w];
        (values.iter().all(|v| v.is_finite()) && q.w > 0.0).then_some(values)
    };
    let mut w_min = f64::INFINITY;
    let mut r = 0.0f64;
    for i in row_range.clone() {
        for j in column_range.clone() {
            let q = relative(i, j)?;
            w_min = w_min.min(q[3]);
            r = r.max((q[0] * q[0] + q[1] * q[1] + q[2] * q[2]).sqrt() / q[3]);
        }
    }
    // (largest |X_d|, largest |w_d|) in one parameter direction.
    let derivative = |along_u: bool| -> Option<(f64, f64)> {
        let (degree, knots, count, range) = if along_u {
            (surface.degree_u, &surface.knots_u, rows, row_range.clone())
        } else {
            (surface.degree_v, &surface.knots_v, columns, column_range.clone())
        };
        if knots.len() < count + degree + 1 {
            return None;
        }
        let (mut point, mut weight) = (0.0f64, 0.0f64);
        if degree == 0 {
            return Some((0.0, 0.0));
        }
        // The derivative's controls over the range: consecutive differences
        // of the controls in it (all of them for the whole net).
        for k in range.start..range.end.saturating_sub(1) {
            let span = knots[k + degree + 1] - knots[k + 1];
            if !(span > 0.0) {
                continue;
            }
            let factor = degree as f64 / span;
            let others = if along_u { column_range.clone() } else { row_range.clone() };
            for other in others {
                let (a, b) = if along_u {
                    (relative(k, other)?, relative(k + 1, other)?)
                } else {
                    (relative(other, k)?, relative(other, k + 1)?)
                };
                let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2], b[3] - a[3]];
                point = point.max(factor * (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt());
                weight = weight.max(factor * d[3].abs());
            }
        }
        Some((point, weight))
    };
    let (xu, wu) = derivative(true)?;
    let (xv, wv) = derivative(false)?;
    let su = (xu + wu * r) / w_min;
    let sv = (xv + wv * r) / w_min;
    let n = su * sv;
    // A little headroom for the evaluator's own rounding.
    let (n, r) = (n * (1.0 + 1e-9), r * (1.0 + 1e-9));
    (n.is_finite() && r.is_finite() && w_min.is_finite()).then_some((n, r))
}

/// Retain the exact path's shell references and compensated face order.
/// Each face's volume flux is affine in the reference: retain its local
/// volume and vector area, then translate the term to the shell reference.
/// Unchanged faces survive both id renumbering and a moving shell reference.
fn solid_sign_terms_reading(solid: &BrepSolid) -> Result<(f64, f64), String> {
    super::trimmed::with_straight_corrections_skipped(|| {
        let (mut reading, mut magnitude) = (0.0, 0.0);
        for (shell_index, shell) in solid.shells.iter().enumerate() {
            let (volume, faces) = shell_volume_sign_reading(shell, shell_index)?;
            reading += volume;
            magnitude += faces;
        }
        Ok((reading, magnitude))
    })
}

fn sign_terms_oracle_enabled() -> bool {
    switch!(live "BREP_SIGN_TERMS_ORACLE", false)
}

/// Diagnostic census only; the shipped reading and the sphere certificate
/// stay unchanged. The cold oracle never uses the mass cache or reading memo.
fn verify_solid_sign_terms(solid: &BrepSolid) -> Result<(), String> {
    let token = super::profile::begin("solid_sign_terms_oracle");
    let reading = solid_sign_terms_reading(solid).map(|pair| pair.0);
    super::profile::end(token);
    check_sign_terms(solid, reading?)
}

pub fn check_sign_terms(solid: &BrepSolid, reading: f64) -> Result<(), String> {
    let exact = solid_signed_volume_uncached(solid)?;
    let error = (reading - exact).abs();
    let bar = 1e-9 * exact.abs();
    if !reading.is_finite() || !exact.is_finite() || error > bar {
        return Err(format!("sign terms oracle: retained {reading:.17e}, whole-body {exact:.17e}, error {error:.3e} exceeds {bar:.3e}"));
    }
    super::profile::emit_sign_oracle(solid, reading, exact)?;
    Ok(())
}

/// [`shell_signed_volume`]'s sum under the caller's correction policy, with
/// the sum of the face contributions' magnitudes beside it (the scale the
/// sign reading's margin is read against).
fn shell_volume_sign_reading(shell: &ShellRecord, shell_index: usize) -> Result<(f64, f64), String> {
    let reference = shell_volume_reference(shell)?;
    if crate::profile::profile_enabled() {
        profile_line!(
            "mass.sign_reference faces={} reference=({:.9},{:.9},{:.9})",
            shell.faces.len(), reference.x, reference.y, reference.z
        );
    }
    let mut volume = 0.0;
    let mut compensation = 0.0;
    let mut magnitude = 0.0;
    let face_lines = super::profile::face_lines_enabled();
    for (face_index, face) in shell.faces.iter().enumerate() {
        mass_profile(|p| p.faces += 1);
        let line_before = face_lines.then(super::profile::snapshot);
        let line_started = face_lines.then(Instant::now);
        // V_f(r) = V_f(a) + (a-r) · A_f / 3. Integrate both terms at
        // the SAME quadrature stations, including trim corrections. Using a
        // boundary-only vector area would miss the integrator's numerical
        // residual and could change the sign reading on inherited trims.
        // Anchor each face locally for accuracy far from the world origin.
        let local = face_volume_reference(face).unwrap_or(reference);
        let hash_started = super::profile::profile_started();
        let identity = super::identity::face_contribution_identity(face, local);
        let hash_ms = super::profile::elapsed_ms(hash_started);
        let mut reread = false;
        let terms = match crate::reading_memo::memoized_sign_terms(identity, || {
            reread = true;
            face_volume_terms_about(face, local).ok()
        }) {
            Some(terms) => terms,
            None => face_volume_terms_about(face, local)?,
        };
        let contribution = terms[0] + local.sub(reference).dot(Vec3::new(terms[1], terms[2], terms[3]));
        super::profile::sign_face(shell_index, face, reread, hash_ms);
        emit_face_line("shell_volume_sign_reading", shell_index, face_index, face, line_before, line_started);
        magnitude += contribution.abs();
        let next = volume + contribution;
        if volume.abs() >= contribution.abs() {
            compensation += (volume - next) + contribution;
        } else {
            compensation += (contribution - next) + volume;
        }
        volume = next;
    }
    Ok((volume + compensation, magnitude))
}

pub fn solid_signed_volume_uncached(solid: &BrepSolid) -> Result<f64, String> {
    let mut volume = 0.0;
    for shell in &solid.shells {
        volume += shell_signed_volume(shell)?;
    }
    Ok(volume)
}

pub fn shell_volume_reference(shell: &ShellRecord) -> Result<Vec3, String> {
    shell.faces.iter().find_map(face_volume_reference)
        .ok_or_else(|| "mass_properties: shell has no finite geometric reference".to_string())
}

/// One finite authored point, with the same fallbacks as the exact integral.
pub fn face_volume_reference(face: &FaceRecord) -> Option<Vec3> {
    let finite = |point: Vec3| point.x.is_finite() && point.y.is_finite() && point.z.is_finite();
    if let Some(coedge) = face
        .loops
        .first()
        .and_then(|loop_record| loop_record.coedges.first())
    {
        if let Ok(domain) = coedge.pcurve.domain() {
            if let Ok(uv) = coedge.pcurve.evaluate(domain[0]) {
                if let Ok(point) = face.surface.evaluate(uv.x, uv.y) {
                    if finite(point) {
                        return Some(point);
                    }
                }
            }
        }
    }
    if let Ok((u_breaks, v_breaks)) = surface_breaks(&face.surface) {
        let u = 0.5 * (u_breaks[0] + u_breaks[u_breaks.len() - 1]);
        let v = 0.5 * (v_breaks[0] + v_breaks[v_breaks.len() - 1]);
        if let Ok(point) = face.surface.evaluate(u, v) {
            if finite(point) {
                return Some(point);
            }
        }
    }
    for row in &face.surface.control_points {
        for control in row {
            if let Ok(point) = control.point() {
                if finite(point) {
                    return Some(point);
                }
            }
        }
    }
    None
}

/// Exact signed volume of one closed shell. Multi-shell tessellation uses this
/// to preserve the authored material orientation: exterior shells contribute
/// positively, while a void boundary contributes negatively.
pub fn shell_signed_volume(shell: &ShellRecord) -> Result<f64, String> {
    let reference = shell_volume_reference(shell)?;

    // A closed shell's divergence-theorem volume is independent of origin,
    // but evaluating each face about the world origin can cancel enormous
    // translated face terms down to a tiny cavity volume. Anchor the
    // integrand on an authored boundary point and compensate the remaining
    // face sum so the sign is stable for small, far-translated shells.
    let mut volume = 0.0;
    let mut compensation = 0.0;
    let face_lines = super::profile::face_lines_enabled();
    for (face_index, face) in shell.faces.iter().enumerate() {
        mass_profile(|p| p.faces += 1);
        let line_before = face_lines.then(super::profile::snapshot);
        let line_started = face_lines.then(Instant::now);
        let contribution = face_volume_contribution_about(face, reference)?;
        emit_face_line(
            "shell_signed_volume",
            0,
            face_index,
            face,
            line_before,
            line_started,
        );
        let next = volume + contribution;
        if volume.abs() >= contribution.abs() {
            compensation += (volume - next) + contribution;
        } else {
            compensation += (contribution - next) + volume;
        }
        volume = next;
    }
    Ok(volume + compensation)
}

/// Exact moments for affine faces via Green's theorem over the trim
/// pcurves: ∬ g du dv = ∮ G dv with G(u,v) = ∫ g dt.  On an affine carrier
/// every moment integrand is a low-degree polynomial, so the inner Gauss
/// antiderivative is exact and the boundary quadrature has the same quality
/// as `parameter_space_area` — no trim-polygon sampling error.  The loop
/// winding supplies the orientation sign, matching the affine volume path.
pub(super) fn affine_moment(face: &FaceRecord, kind: Integrand) -> Result<f64, String> {
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let points = &face.surface.control_points;
    let p00 = points[0][0].point()?;
    let p10 = points[1][0].point()?;
    let p01 = points[0][1].point()?;
    let du = p10.sub(p00).scale(1.0 / (u1 - u0));
    let dv = p01.sub(p00).scale(1.0 / (v1 - v0));
    let weighted_normal = du.cross(dv);
    let g = |u: f64, v: f64| {
        let point = p00.add(du.scale(u - u0)).add(dv.scale(v - v0));
        integrand_value(kind, point, weighted_normal)
    };
    let inner = |u: f64, v: f64| {
        let half = (u - u0) * 0.5;
        let middle = (u + u0) * 0.5;
        let mut sum = 0.0;
        for index in 0..GAUSS_X.len() {
            sum += GAUSS_W[index] * g(middle + half * GAUSS_X[index], v);
        }
        sum * half
    };
    let mut total = 0.0;
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            for pair in curve_breaks(&coedge.pcurve)?.windows(2) {
                // The inner antiderivative is a polynomial in u on an affine
                // carrier and is exact at any order; the boundary walk is the
                // pcurve's own, so it takes the pcurve's panels (`rule`).
                for panel in rule::curve_panels(&coedge.pcurve, pair[0], pair[1])? {
                    for (parameter, weight) in panel.stations() {
                        let (point, tangent) = coedge.pcurve.deriv1(parameter)?;
                        total += weight * inner(point.x, point.y) * tangent.y;
                    }
                }
            }
        }
    }
    // The loops closed as the area closes them (`closed_parameter_space_area`):
    // an open boundary integral moves with the uv origin. Along a straight
    // chord the integrand is a polynomial of degree at most four, which the
    // eight-station rule integrates exactly.
    for (end, start) in loop_closing_chords(face)? {
        let step = start.y - end.y;
        if step == 0.0 {
            continue;
        }
        for index in 0..GAUSS_X.len() {
            let fraction = 0.5 * (GAUSS_X[index] + 1.0);
            let u = end.x + (start.x - end.x) * fraction;
            let v = end.y + step * fraction;
            total += 0.5 * GAUSS_W[index] * inner(u, v) * step;
        }
    }
    Ok(total)
}

pub fn face_moment(face: &FaceRecord, kind: Integrand) -> Result<f64, String> {
    if is_affine(&face.surface)? {
        return affine_moment(face, kind);
    }
    if let Some(values) = biperiodic_band_integral(face, &[kind])? {
        return Ok(values[0]);
    }
    if is_untrimmed(face)? {
        integrate_untrimmed(face, kind)
    } else {
        integrate_trimmed(face, kind)
    }
}

/// Symmetric-matrix product `A[i][j] = M[k][i] * M[k][j]` reused by the Jacobi
/// sweep — kept tiny and explicit rather than pulling in a matrix crate.
pub fn mat3_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                r[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    r
}

pub fn mat3_transpose(a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    let mut r = [[0.0f64; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            r[i][j] = a[j][i];
        }
    }
    r
}

/// Classic Jacobi eigenvalue iteration for a SYMMETRIC 3×3 matrix. Repeatedly
/// applies a Givens rotation in the plane of the largest off-diagonal element,
/// each rotation zeroing that element, until the matrix is diagonal to machine
/// precision. Returns `(eigenvalues, vectors)` where `vectors` holds the
/// eigenvectors as COLUMNS (`vectors[i][k]` is component `i` of eigenvector
/// `k`), unsorted. For a symmetric matrix Jacobi is unconditionally
/// convergent and the accumulated rotations stay orthonormal, so the columns
/// are mutually orthonormal by construction.
pub(super) fn jacobi_eigen_symmetric_3x3(matrix: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    // Symmetrize defensively against tiny asymmetry from round-off upstream.
    let mut a = [
        [matrix[0][0], 0.0, 0.0],
        [0.0, matrix[1][1], 0.0],
        [0.0, 0.0, matrix[2][2]],
    ];
    a[0][1] = 0.5 * (matrix[0][1] + matrix[1][0]);
    a[1][0] = a[0][1];
    a[0][2] = 0.5 * (matrix[0][2] + matrix[2][0]);
    a[2][0] = a[0][2];
    a[1][2] = 0.5 * (matrix[1][2] + matrix[2][1]);
    a[2][1] = a[1][2];

    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let scale = a[0][0].abs() + a[1][1].abs() + a[2][2].abs() + 1.0;
    for _sweep in 0..64 {
        // Pick the largest off-diagonal magnitude.
        let pairs = [(0usize, 1usize), (0, 2), (1, 2)];
        let (mut p, mut q, mut best) = (0usize, 1usize, 0.0f64);
        for &(i, j) in &pairs {
            if a[i][j].abs() > best {
                best = a[i][j].abs();
                p = i;
                q = j;
            }
        }
        if best <= 1e-18 * scale {
            break;
        }
        // Angle that zeroes a[p][q]: t = tan(theta) is the smaller root of
        // t² + 2·theta·t − 1 = 0 with theta = (a_qq − a_pp)/(2 a_pq).
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = if theta == 0.0 {
            1.0
        } else {
            theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt())
        };
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        // Givens rotation J: J[p][p]=J[q][q]=c, J[p][q]=s, J[q][p]=−s.
        let mut j = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        j[p][p] = c;
        j[q][q] = c;
        j[p][q] = s;
        j[q][p] = -s;
        // A ← Jᵀ A J   (drives a[p][q] to zero), V ← V J.
        a = mat3_mul(mat3_transpose(j), mat3_mul(a, j));
        v = mat3_mul(v, j);
    }
    ([a[0][0], a[1][1], a[2][2]], v)
}

/// Principal axes/moments from a centroidal inertia tensor (Golovanov §8.11).
/// Diagonalizes the symmetric tensor, sorts the eigenpairs by moment
/// ASCENDING, returns each eigenvector as a ROW (`axes[i]` pairs with
/// `moments[i]`) normalized to unit length, and fixes the sign of the third
/// axis so the frame is right-handed (determinant +1).
pub fn principal_frame(inertia: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
    let (values, vectors) = jacobi_eigen_symmetric_3x3(inertia);
    // Column k of `vectors` is the eigenvector for `values[k]`.
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]));
    let mut moments = [0.0f64; 3];
    let mut axes = [[0.0f64; 3]; 3];
    for (slot, &k) in order.iter().enumerate() {
        moments[slot] = values[k];
        let mut axis = [vectors[0][k], vectors[1][k], vectors[2][k]];
        let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        if length > 0.0 {
            axis = [axis[0] / length, axis[1] / length, axis[2] / length];
        }
        axes[slot] = axis;
    }
    // Right-hand the frame: if axis0 × axis1 points opposite axis2, flip axis2.
    let cross = [
        axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1],
        axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2],
        axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0],
    ];
    let det = cross[0] * axes[2][0] + cross[1] * axes[2][1] + cross[2] * axes[2][2];
    if det < 0.0 {
        axes[2] = [-axes[2][0], -axes[2][1], -axes[2][2]];
    }
    (moments, axes)
}

/// Area, volume, centroid, and centroidal inertia (unit density).  Area and
/// volume use the same exact paths as `solid_mass_properties`; the moment
/// integrals use divergence-theorem surface quadrature (exact for untrimmed
/// spans, trim-polygon scanline accuracy for trimmed faces).
pub fn solid_mass_properties_full(solid: &BrepSolid) -> Result<FullMassProperties, String> {
    let base = solid_mass_properties(solid)?;
    let token = super::profile::begin("solid_mass_properties_full.moments");
    let result = solid_mass_properties_full_moments(solid, base);
    super::profile::end(token);
    result
}

/// `BREP_MASS_MOMENT_BAND_ROUTE` (default on; `=0` restores the old route): the moment
/// integrals take the bi-periodic band route first, as the area and volume
/// read in [`solid_mass_properties_inner`] and [`face_moment`] already do.
/// Without it a trimmed torus band (two rim loops round the carrier) goes
/// straight to the trimmed cell walk, whose raw polygons collapse to a sliver
/// on such a band: the face's moments read about zero while its volume reads
/// in full, so the centroid is off and moves by a multiple of any
/// translation (a 107-face part with three such bands: 2.4e-3 of the offset
/// along x and z, and hundreds of millimetres along the torus axis at 1000).
pub fn moment_band_route_on() -> bool {
    crate::switch!("BREP_MASS_MOMENT_BAND_ROUTE", true)
}

/// `BREP_MASS_MOMENT_LOCAL_FRAME` (default on; `=0` reads about the world origin): the
/// first and second moments are read about a point of the body instead of
/// the world origin. In the world frame a moment integrand grows with the
/// square of the distance, so a shell whose faces do not close exactly (an
/// imported tolerant body: net vector area 1.4e-4 mm² on a 78-face part)
/// moves its centroid by `t²/2 · Σn / V` under a translation `t` (0.11 mm at
/// 1000 on that part), and the inertia's parallel-axis step cancels digits.
/// About a point that moves with the body, both are translation invariant.
pub fn moment_local_frame_on() -> bool {
    crate::switch!("BREP_MASS_MOMENT_LOCAL_FRAME", true)
}

fn solid_mass_properties_full_moments(
    solid: &BrepSolid,
    base: MassProperties,
) -> Result<FullMassProperties, String> {
    const MOMENT_KINDS: [Integrand; 9] = [
        Integrand::MomentX,
        Integrand::MomentY,
        Integrand::MomentZ,
        Integrand::SecondXX,
        Integrand::SecondYY,
        Integrand::SecondZZ,
        Integrand::ProductXY,
        Integrand::ProductXZ,
        Integrand::ProductYZ,
    ];
    // `BREP_MASS_MOMENT_LOCAL_FRAME`: every moment about one point of the
    // body (the first non-empty shell's volume reference, the point the
    // volume is read about), and the centroid that point plus the local one.
    let origin = if moment_local_frame_on() {
        solid.shells.iter().find(|shell| !shell.faces.is_empty()).map(shell_volume_reference).transpose()?
    } else {
        None
    };
    let local_kinds = origin.map(|r| {
        [
            Integrand::MomentAbout(0, r),
            Integrand::MomentAbout(1, r),
            Integrand::MomentAbout(2, r),
            Integrand::SecondAbout(0, r),
            Integrand::SecondAbout(1, r),
            Integrand::SecondAbout(2, r),
            Integrand::ProductAbout(0, r),
            Integrand::ProductAbout(1, r),
            Integrand::ProductAbout(2, r),
        ]
    });
    let kinds: &[Integrand; 9] = local_kinds.as_ref().unwrap_or(&MOMENT_KINDS);
    let mut moments = [0.0f64; 3];
    let mut seconds = [0.0f64; 3];
    let mut products = [0.0f64; 3];
    for shell in &solid.shells {
        for face in &shell.faces {
            mass_profile(|p| p.faces += 1);
            let band = if moment_band_route_on() {
                biperiodic_band_integral(face, kinds)?
            } else {
                None
            };
            let results = if let Some(values) = band {
                mass_profile(|p| p.route_biperiodic += 1);
                values
            } else if !is_affine(&face.surface)? && !is_untrimmed(face)? {
                mass_profile(|p| p.trimmed_faces += 1);
                // One cell decomposition, all nine moment integrands per
                // station, instead of nine full passes per face.
                integrate_trimmed_multi(face, kinds)?
            } else {
                kinds
                    .iter()
                    .map(|kind| face_moment(face, *kind))
                    .collect::<Result<Vec<_>, _>>()?
            };
            moments[0] += results[0];
            moments[1] += results[1];
            moments[2] += results[2];
            seconds[0] += results[3];
            seconds[1] += results[4];
            seconds[2] += results[5];
            products[0] += results[6];
            products[1] += results[7];
            products[2] += results[8];
        }
    }
    let volume = base.volume;
    if volume.abs() <= 1e-30 {
        return Err("solid_mass_properties_full: non-positive volume".into());
    }
    // The centroid in the frame the moments were read in (the world, or
    // about `origin`).
    let centroid = Vec3::new(
        moments[0] / volume,
        moments[1] / volume,
        moments[2] / volume,
    );
    // Inertia about the frame's origin, then parallel-axis down to the centroid.
    let ixx =
        seconds[1] + seconds[2] - volume * (centroid.y * centroid.y + centroid.z * centroid.z);
    let iyy =
        seconds[0] + seconds[2] - volume * (centroid.x * centroid.x + centroid.z * centroid.z);
    let izz =
        seconds[0] + seconds[1] - volume * (centroid.x * centroid.x + centroid.y * centroid.y);
    let ixy = -(products[0] - volume * centroid.x * centroid.y);
    let ixz = -(products[1] - volume * centroid.x * centroid.z);
    let iyz = -(products[2] - volume * centroid.y * centroid.z);
    let inertia = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]];
    let (principal_moments, principal_axes) = principal_frame(inertia);
    let centroid = origin.map_or(centroid, |r| r.add(centroid));
    Ok(FullMassProperties {
        surface_area: base.surface_area,
        volume,
        centroid,
        inertia,
        principal_moments,
        principal_axes,
    })
}


