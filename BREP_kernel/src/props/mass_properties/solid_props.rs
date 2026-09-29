use super::*;
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

fn solid_mass_properties_inner(solid: &BrepSolid) -> Result<MassProperties, String> {
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
    if let (Some(dump), Ok(volume)) = (super::profile::identity_dump(), result.as_ref()) {
        super::profile::emit_identity(dump, "solid_signed_volume", solid, &[("signed_bits", *volume)]);
    }
    result
}

fn solid_signed_volume_uncached(solid: &BrepSolid) -> Result<f64, String> {
    let mut volume = 0.0;
    for shell in &solid.shells {
        volume += shell_signed_volume(shell)?;
    }
    Ok(volume)
}

pub(super) fn shell_volume_reference(shell: &ShellRecord) -> Result<Vec3, String> {
    let finite = |point: Vec3| point.x.is_finite() && point.y.is_finite() && point.z.is_finite();
    for face in &shell.faces {
        if let Some(coedge) = face
            .loops
            .first()
            .and_then(|loop_record| loop_record.coedges.first())
        {
            if let Ok(domain) = coedge.pcurve.domain() {
                if let Ok(uv) = coedge.pcurve.evaluate(domain[0]) {
                    if let Ok(point) = face.surface.evaluate(uv.x, uv.y) {
                        if finite(point) {
                            return Ok(point);
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
                    return Ok(point);
                }
            }
        }
        for row in &face.surface.control_points {
            for control in row {
                if let Ok(point) = control.point() {
                    if finite(point) {
                        return Ok(point);
                    }
                }
            }
        }
    }
    Err("mass_properties: shell has no finite geometric reference".to_string())
}

/// Exact signed volume of one closed shell. Multi-shell tessellation uses this
/// to preserve the authored material orientation: exterior shells contribute
/// positively, while a void boundary contributes negatively.
pub(crate) fn shell_signed_volume(shell: &ShellRecord) -> Result<f64, String> {
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

pub(super) fn face_moment(face: &FaceRecord, kind: Integrand) -> Result<f64, String> {
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
pub(super) fn mat3_mul(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
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

pub(super) fn mat3_transpose(a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
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
pub(super) fn principal_frame(inertia: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
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
    let mut moments = [0.0f64; 3];
    let mut seconds = [0.0f64; 3];
    let mut products = [0.0f64; 3];
    for shell in &solid.shells {
        for face in &shell.faces {
            mass_profile(|p| p.faces += 1);
            let results = if !is_affine(&face.surface)? && !is_untrimmed(face)? {
                mass_profile(|p| p.trimmed_faces += 1);
                // One cell decomposition, all nine moment integrands per
                // station, instead of nine full passes per face.
                integrate_trimmed_multi(face, &MOMENT_KINDS)?
            } else {
                MOMENT_KINDS
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
    let centroid = Vec3::new(
        moments[0] / volume,
        moments[1] / volume,
        moments[2] / volume,
    );
    // Inertia about the origin, then parallel-axis down to the centroid.
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
    Ok(FullMassProperties {
        surface_area: base.surface_area,
        volume,
        centroid,
        inertia,
        principal_moments,
        principal_axes,
    })
}

