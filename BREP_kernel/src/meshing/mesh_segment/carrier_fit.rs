use super::*;

// ---------------------------------------------------------------------------
// Small linear algebra helpers
// ---------------------------------------------------------------------------

/// Jacobi eigendecomposition of a symmetric 3×3 matrix.  Returns the
/// eigenvalues in ascending order with their (unit) eigenvectors.
fn eigen_symmetric3(mut m: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let mut v = [[0.0_f64; 3]; 3];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    let scale = m
        .iter()
        .flat_map(|row| row.iter())
        .fold(0.0_f64, |acc, value| acc.max(value.abs()))
        .max(1e-300);
    for _ in 0..64 {
        // Largest off-diagonal element.
        let (mut p, mut q, mut largest) = (0usize, 1usize, m[0][1].abs());
        if m[0][2].abs() > largest {
            p = 0;
            q = 2;
            largest = m[0][2].abs();
        }
        if m[1][2].abs() > largest {
            p = 1;
            q = 2;
            largest = m[1][2].abs();
        }
        if largest <= 1e-15 * scale {
            break;
        }
        let apq = m[p][q];
        let theta = (m[q][q] - m[p][p]) / (2.0 * apq);
        let t = if theta >= 0.0 {
            1.0 / (theta + (1.0 + theta * theta).sqrt())
        } else {
            1.0 / (theta - (1.0 + theta * theta).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let (app, aqq) = (m[p][p], m[q][q]);
        m[p][p] = app - t * apq;
        m[q][q] = aqq + t * apq;
        m[p][q] = 0.0;
        m[q][p] = 0.0;
        for i in 0..3 {
            if i != p && i != q {
                let (aip, aiq) = (m[i][p], m[i][q]);
                m[i][p] = c * aip - s * aiq;
                m[p][i] = m[i][p];
                m[i][q] = s * aip + c * aiq;
                m[q][i] = m[i][q];
            }
            let (vip, viq) = (v[i][p], v[i][q]);
            v[i][p] = c * vip - s * viq;
            v[i][q] = s * vip + c * viq;
        }
    }
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| m[a][a].total_cmp(&m[b][b]));
    let values = [
        m[order[0]][order[0]],
        m[order[1]][order[1]],
        m[order[2]][order[2]],
    ];
    let vectors = [
        Vec3::new(v[0][order[0]], v[1][order[0]], v[2][order[0]]),
        Vec3::new(v[0][order[1]], v[1][order[1]], v[2][order[1]]),
        Vec3::new(v[0][order[2]], v[1][order[2]], v[2][order[2]]),
    ];
    (values, vectors)
}

fn sym3_accumulate(m: &mut [[f64; 3]; 3], d: Vec3, weight: f64) {
    let v = [d.x, d.y, d.z];
    for i in 0..3 {
        for j in 0..3 {
            m[i][j] += weight * v[i] * v[j];
        }
    }
}

/// Least-squares (Kåsa) circle through 2D points; centered internally for
/// conditioning.  Returns (center_x, center_y, radius).
fn fit_circle_2d(points: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let inv = 1.0 / points.len() as f64;
    let (mx, my) = points
        .iter()
        .fold((0.0, 0.0), |(sx, sy), &(x, y)| (sx + x * inv, sy + y * inv));
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];
    for &(px, py) in points {
        let x = px - mx;
        let y = py - my;
        let row = [x, y, 1.0];
        let b = x * x + y * y;
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * b;
        }
    }
    let solution = solve_small::<3>(ata, atb, 3).ok()?;
    let cx = 0.5 * solution[0];
    let cy = 0.5 * solution[1];
    let r_squared = solution[2] + cx * cx + cy * cy;
    if !(r_squared > 0.0) || !r_squared.is_finite() {
        return None;
    }
    Some((cx + mx, cy + my, r_squared.sqrt()))
}

/// Flip a direction so its largest-magnitude component is positive
/// (deterministic reporting for orientation-free axes).
fn canonical_dir(a: Vec3) -> Vec3 {
    let (ax, ay, az) = (a.x.abs(), a.y.abs(), a.z.abs());
    let key = if ax >= ay && ax >= az {
        a.x
    } else if ay >= az {
        a.y
    } else {
        a.z
    };
    if key < 0.0 {
        a.scale(-1.0)
    } else {
        a
    }
}

// ---------------------------------------------------------------------------
// Region fitting
// ---------------------------------------------------------------------------

pub(super) struct CandidateFit {
    pub(super) carrier: RegionCarrier,
    pub(super) max_dev: f64,
    pub(super) rms_dev: f64,
    pub(super) max_normal_angle_deg: f64,
}

fn deviation_stats(devs: &[f64]) -> (f64, f64) {
    let mut max_abs = 0.0_f64;
    let mut sum_sq = 0.0_f64;
    for &d in devs {
        max_abs = max_abs.max(d.abs());
        sum_sq += d * d;
    }
    let rms = if devs.is_empty() {
        0.0
    } else {
        (sum_sq / devs.len() as f64).sqrt()
    };
    (max_abs, rms)
}

/// The normal-agreement gate ignores the worst-aligned triangles up to this
/// cumulative fraction of the region area.  Needle triangles (e.g. at the
/// pole rings of a tessellated sphere) have chordal normals that deviate
/// tens of degrees from the surface normal while their vertices still lie
/// exactly on the carrier; a worst-case gate over them would reject perfect
/// fits.  The area-weighted trim mirrors the app's area-weighted RANSAC
/// sampling, which never gated on worst-case slivers either.
const NORMAL_GATE_AREA_TRIM: f64 = 1e-3;

/// Area-weighted orientation sense of the mesh normals against a carrier
/// normal field, plus the area-trimmed worst normal angle after sense
/// correction.  `carrier_normal` may return `None` where the field is
/// singular (e.g. a centroid on the axis); such triangles are skipped.
fn normal_agreement<F>(data: &MeshData, tri_ids: &[u32], carrier_normal: F) -> (i8, f64)
where
    F: Fn(&TriData) -> Option<Vec3>,
{
    let mut alignment = 0.0;
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        if let Some(n) = carrier_normal(tri) {
            alignment += tri.area * tri.normal.dot(n);
        }
    }
    let sense: i8 = if alignment >= 0.0 { 1 } else { -1 };
    let sign = sense as f64;
    let mut entries: Vec<(f64, f64)> = Vec::with_capacity(tri_ids.len());
    let mut total_area = 0.0;
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        if let Some(n) = carrier_normal(tri) {
            let cosine = tri.normal.dot(n.scale(sign)).clamp(-1.0, 1.0);
            entries.push((cosine.acos(), tri.area));
            total_area += tri.area;
        }
    }
    entries.sort_by(|a, b| b.0.total_cmp(&a.0));
    let budget = total_area * NORMAL_GATE_AREA_TRIM;
    let mut spent = 0.0;
    let mut max_angle = 0.0_f64;
    for (angle, area) in entries {
        if spent + area <= budget {
            spent += area;
            continue;
        }
        max_angle = angle;
        break;
    }
    (sense, max_angle.to_degrees())
}

pub(super) fn region_vertices(data: &MeshData, tri_ids: &[u32]) -> Vec<usize> {
    let mut verts: Vec<usize> = tri_ids
        .iter()
        .flat_map(|&t| data.tris[t as usize].verts)
        .collect();
    verts.sort_unstable();
    verts.dedup();
    verts
}

pub(super) fn region_bbox(data: &MeshData, vert_ids: &[usize]) -> (Vec3, Vec3) {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for &v in vert_ids {
        let p = data.verts[v];
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    (low, high)
}

/// Area-weighted mean and centered covariance of the triangle unit normals.
fn normal_statistics(data: &MeshData, tri_ids: &[u32]) -> Option<(Vec3, [[f64; 3]; 3])> {
    let mut total_area = 0.0;
    let mut mean = Vec3::default();
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        total_area += tri.area;
        mean = mean.add(tri.normal.scale(tri.area));
    }
    if !(total_area > 0.0) {
        return None;
    }
    mean = mean.scale(1.0 / total_area);
    let mut covariance = [[0.0_f64; 3]; 3];
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        sym3_accumulate(&mut covariance, tri.normal.sub(mean), tri.area / total_area);
    }
    Some((mean, covariance))
}

pub(super) fn fit_plane(data: &MeshData, tri_ids: &[u32], vert_ids: &[usize]) -> Option<CandidateFit> {
    if vert_ids.len() < 3 {
        return None;
    }
    let inv = 1.0 / vert_ids.len() as f64;
    let centroid = vert_ids
        .iter()
        .fold(Vec3::default(), |acc, &v| acc.add(data.verts[v].scale(inv)));
    let mut covariance = [[0.0_f64; 3]; 3];
    for &v in vert_ids {
        sym3_accumulate(&mut covariance, data.verts[v].sub(centroid), 1.0);
    }
    let (_, vectors) = eigen_symmetric3(covariance);
    let mut normal = vectors[0];
    // Orient with the mesh normals.
    let mut alignment = 0.0;
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        alignment += tri.area * tri.normal.dot(normal);
    }
    if alignment < 0.0 {
        normal = normal.scale(-1.0);
    }
    let devs: Vec<f64> = vert_ids
        .iter()
        .map(|&v| data.verts[v].sub(centroid).dot(normal))
        .collect();
    let (max_dev, rms_dev) = deviation_stats(&devs);
    let (_, max_normal_angle_deg) = normal_agreement(data, tri_ids, |_| Some(normal));
    Some(CandidateFit {
        carrier: RegionCarrier::Plane {
            origin: centroid,
            normal,
        },
        max_dev,
        rms_dev,
        max_normal_angle_deg,
    })
}

pub(super) fn fit_cylinder(data: &MeshData, tri_ids: &[u32], vert_ids: &[usize]) -> Option<CandidateFit> {
    let (_, covariance) = normal_statistics(data, tri_ids)?;
    let (values, vectors) = eigen_symmetric3(covariance);
    // Normals must actually fan around an axis: with (near-)parallel normals
    // the axis is undefined and the region is a plane's business.
    if values[1] <= 1e-10 * values[2].max(1e-300) {
        return None;
    }
    let axis = canonical_dir(vectors[0]);
    let u = axis.perpendicular().ok()?;
    let v = axis.cross(u);
    let inv = 1.0 / vert_ids.len() as f64;
    let centroid = vert_ids.iter().fold(Vec3::default(), |acc, &vid| {
        acc.add(data.verts[vid].scale(inv))
    });
    let projected: Vec<(f64, f64)> = vert_ids
        .iter()
        .map(|&vid| {
            let d = data.verts[vid].sub(centroid);
            (d.dot(u), d.dot(v))
        })
        .collect();
    let (cx, cy, radius) = fit_circle_2d(&projected)?;
    let axis_point = centroid.add(u.scale(cx)).add(v.scale(cy));
    let devs: Vec<f64> = vert_ids
        .iter()
        .map(|&vid| {
            let d = data.verts[vid].sub(axis_point);
            let radial = d.sub(axis.scale(d.dot(axis)));
            radial.length() - radius
        })
        .collect();
    let (max_dev, rms_dev) = deviation_stats(&devs);
    let radial_dir = |tri: &TriData| -> Option<Vec3> {
        let d = tri.centroid.sub(axis_point);
        let radial = d.sub(axis.scale(d.dot(axis)));
        radial.normalized().ok()
    };
    let (sense, max_normal_angle_deg) = normal_agreement(data, tri_ids, radial_dir);
    Some(CandidateFit {
        carrier: RegionCarrier::Cylinder {
            axis_point,
            axis_dir: axis,
            radius,
            sense,
        },
        max_dev,
        rms_dev,
        max_normal_angle_deg,
    })
}

/// Gauss–Newton polish of a cone (apex, axis, half-angle) on the region
/// vertices.  The seed estimates come from facet tangent planes and facet
/// normals, which carry a small chordal bias — the vertices themselves lie
/// exactly on the surface, so a few Newton steps on the point-to-cone
/// distance remove that bias.
fn refine_cone(
    data: &MeshData,
    vert_ids: &[usize],
    mut apex: Vec3,
    mut axis: Vec3,
    mut half_angle: f64,
) -> (Vec3, Vec3, f64) {
    for _ in 0..12 {
        let Ok(e1) = axis.perpendicular() else {
            break;
        };
        let e2 = axis.cross(e1);
        let (sin_a, cos_a) = half_angle.sin_cos();
        let mut ata = [[0.0_f64; 6]; 6];
        let mut atb = [0.0_f64; 6];
        for &vid in vert_ids {
            let d = data.verts[vid].sub(apex);
            let h = d.dot(axis);
            let radial = d.sub(axis.scale(h));
            let rho = radial.length();
            if rho <= 1e-14 * data.diag.max(1e-12) {
                continue;
            }
            let w = radial.scale(1.0 / rho);
            let residual = rho * cos_a - h * sin_a;
            // ∂r/∂apex = −w·cosα + a·sinα ; ∂r/∂axis-tilt = C(d·e) with
            // C = −((h/ρ)cosα + sinα) ; ∂r/∂α = −ρ·sinα − h·cosα.
            let j_apex = axis.scale(sin_a).sub(w.scale(cos_a));
            let c_tilt = -((h / rho) * cos_a + sin_a);
            let row = [
                j_apex.x,
                j_apex.y,
                j_apex.z,
                c_tilt * d.dot(e1),
                c_tilt * d.dot(e2),
                -rho * sin_a - h * cos_a,
            ];
            for i in 0..6 {
                for j in 0..6 {
                    ata[i][j] += row[i] * row[j];
                }
                atb[i] -= row[i] * residual;
            }
        }
        let Ok(step) = solve_small::<6>(ata, atb, 6) else {
            break;
        };
        apex = apex.add(Vec3::new(step[0], step[1], step[2]));
        let tilted = axis.add(e1.scale(step[3])).add(e2.scale(step[4]));
        axis = match tilted.normalized() {
            Ok(a) => a,
            Err(_) => break,
        };
        half_angle = (half_angle + step[5]).clamp(1e-6, std::f64::consts::FRAC_PI_2 - 1e-6);
        let step_size = (step.iter().map(|s| s * s).sum::<f64>()).sqrt();
        if step_size < 1e-12 * data.diag.max(1e-12) {
            break;
        }
    }
    (apex, axis, half_angle)
}

pub(super) fn fit_cone(
    data: &MeshData,
    tri_ids: &[u32],
    vert_ids: &[usize],
    tol_abs: f64,
) -> Option<CandidateFit> {
    // Every tangent plane of a cone passes through the apex:
    // minimize Σ A (n·apex − n·centroid)².
    let mut ata = [[0.0_f64; 3]; 3];
    let mut atb = [0.0_f64; 3];
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        let n = [tri.normal.x, tri.normal.y, tri.normal.z];
        let b = tri.normal.dot(tri.centroid);
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += tri.area * n[i] * n[j];
            }
            atb[i] += tri.area * n[i] * b;
        }
    }
    let apex_solution = solve_small::<3>(ata, atb, 3).ok()?;
    let apex = Vec3::new(apex_solution[0], apex_solution[1], apex_solution[2]);

    let (_, covariance) = normal_statistics(data, tri_ids)?;
    let (values, vectors) = eigen_symmetric3(covariance);
    if values[1] <= 1e-10 * values[2].max(1e-300) {
        return None;
    }
    let mut axis = vectors[0];
    // Point the axis from the apex into the region.
    let mut projection = 0.0;
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        projection += tri.area * tri.centroid.sub(apex).dot(axis);
    }
    if projection < 0.0 {
        axis = axis.scale(-1.0);
    }

    let mut angle_sum = 0.0;
    let mut angle_count = 0usize;
    for &vid in vert_ids {
        let d = data.verts[vid].sub(apex);
        let h = d.dot(axis);
        if h < -tol_abs {
            return None; // Points behind the apex: not one cone nappe.
        }
        // A pointed cone's apex vertex has no direction to measure.
        if d.length() <= tol_abs {
            continue;
        }
        let radial = d.sub(axis.scale(h)).length();
        angle_sum += radial.atan2(h.max(0.0));
        angle_count += 1;
    }
    if angle_count == 0 {
        return None;
    }
    let half_angle = angle_sum / angle_count as f64;
    if !(half_angle > 1e-4) || half_angle > std::f64::consts::FRAC_PI_2 - 1e-4 {
        return None;
    }
    let deviations = |apex: Vec3, axis: Vec3, half_angle: f64| -> Vec<f64> {
        let (sin_a, cos_a) = half_angle.sin_cos();
        vert_ids
            .iter()
            .map(|&vid| {
                let d = data.verts[vid].sub(apex);
                let h = d.dot(axis);
                let radial = d.sub(axis.scale(h)).length();
                radial * cos_a - h * sin_a
            })
            .collect()
    };
    let seed = (apex, axis, half_angle);
    let refined = refine_cone(data, vert_ids, apex, axis, half_angle);
    // The polish is rank deficient when the vertices span a single ring
    // (a pointed cone: one ring plus its apex), and an ill-conditioned step
    // can throw a tangent-plane seed that already fits away.  Keep whichever
    // fits the vertices better.
    let seed_devs = deviations(seed.0, seed.1, seed.2);
    let refined_devs = deviations(refined.0, refined.1, refined.2);
    let seed_stats = deviation_stats(&seed_devs);
    let refined_stats = deviation_stats(&refined_devs);
    let refined_is_better = refined.2 > 1e-4
        && refined.2 <= std::f64::consts::FRAC_PI_2 - 1e-4
        && refined_stats.0.is_finite()
        && refined_stats.0 <= seed_stats.0;
    let ((apex, axis, half_angle), (max_dev, rms_dev)) = if refined_is_better {
        (refined, refined_stats)
    } else {
        (seed, seed_stats)
    };
    let (sin_a, cos_a) = half_angle.sin_cos();
    let cone_normal = |tri: &TriData| -> Option<Vec3> {
        let d = tri.centroid.sub(apex);
        let radial = d.sub(axis.scale(d.dot(axis)));
        let w = radial.normalized().ok()?;
        Some(w.scale(cos_a).sub(axis.scale(sin_a)))
    };
    let (sense, max_normal_angle_deg) = normal_agreement(data, tri_ids, cone_normal);
    Some(CandidateFit {
        carrier: RegionCarrier::Cone {
            apex,
            axis_dir: axis,
            half_angle_rad: half_angle,
            sense,
        },
        max_dev,
        rms_dev,
        max_normal_angle_deg,
    })
}

pub(super) fn fit_sphere(data: &MeshData, tri_ids: &[u32], vert_ids: &[usize]) -> Option<CandidateFit> {
    if vert_ids.len() < 4 {
        return None;
    }
    let inv = 1.0 / vert_ids.len() as f64;
    let centroid = vert_ids
        .iter()
        .fold(Vec3::default(), |acc, &v| acc.add(data.verts[v].scale(inv)));
    let mut ata = [[0.0_f64; 4]; 4];
    let mut atb = [0.0_f64; 4];
    for &vid in vert_ids {
        let p = data.verts[vid].sub(centroid);
        let row = [p.x, p.y, p.z, 1.0];
        let b = p.length_squared();
        for i in 0..4 {
            for j in 0..4 {
                ata[i][j] += row[i] * row[j];
            }
            atb[i] += row[i] * b;
        }
    }
    let solution = solve_small::<4>(ata, atb, 4).ok()?;
    let local_center = Vec3::new(0.5 * solution[0], 0.5 * solution[1], 0.5 * solution[2]);
    let r_squared = solution[3] + local_center.length_squared();
    if !(r_squared > 0.0) || !r_squared.is_finite() {
        return None;
    }
    let radius = r_squared.sqrt();
    let center = centroid.add(local_center);
    let devs: Vec<f64> = vert_ids
        .iter()
        .map(|&vid| data.verts[vid].sub(center).length() - radius)
        .collect();
    let (max_dev, rms_dev) = deviation_stats(&devs);
    let radial = |tri: &TriData| -> Option<Vec3> { tri.centroid.sub(center).normalized().ok() };
    let (sense, max_normal_angle_deg) = normal_agreement(data, tri_ids, radial);
    Some(CandidateFit {
        carrier: RegionCarrier::Sphere {
            center,
            radius,
            sense,
        },
        max_dev,
        rms_dev,
        max_normal_angle_deg,
    })
}

/// Alternating axis refinement for the torus fit.  Every torus normal line
/// is coplanar with the axis, so the axis minimizes
/// Σ A [((centroid − c) × n) · a]²; the a-step is a smallest-eigenvector
/// problem and the c-step a 3×3 least squares (rank-deficient along `a`,
/// pinned to the seed's axial station by a small Tikhonov term).
fn refine_torus_axis(
    data: &MeshData,
    tri_ids: &[u32],
    seed_axis: Vec3,
    seed_point: Vec3,
) -> (Vec3, Vec3) {
    let mut axis = seed_axis;
    let mut point = seed_point;
    for _ in 0..32 {
        // c-step.
        let mut ata = [[0.0_f64; 3]; 3];
        let mut atb = [0.0_f64; 3];
        for &t in tri_ids {
            let tri = &data.tris[t as usize];
            let w = tri.normal.cross(axis);
            let beta = tri.centroid.cross(tri.normal).dot(axis);
            let row = [w.x, w.y, w.z];
            for i in 0..3 {
                for j in 0..3 {
                    ata[i][j] += tri.area * row[i] * row[j];
                }
                atb[i] += tri.area * row[i] * beta;
            }
        }
        let trace = ata[0][0] + ata[1][1] + ata[2][2];
        let lambda = trace * 1e-9 + 1e-30;
        let pin = seed_point.dot(axis);
        let a_row = [axis.x, axis.y, axis.z];
        for i in 0..3 {
            for j in 0..3 {
                ata[i][j] += lambda * a_row[i] * a_row[j];
            }
            atb[i] += lambda * pin * a_row[i];
        }
        let Ok(solution) = solve_small::<3>(ata, atb, 3) else {
            break;
        };
        let new_point = Vec3::new(solution[0], solution[1], solution[2]);
        // a-step.
        let mut m = [[0.0_f64; 3]; 3];
        for &t in tri_ids {
            let tri = &data.tris[t as usize];
            let moment = tri.centroid.sub(new_point).cross(tri.normal);
            sym3_accumulate(&mut m, moment, tri.area);
        }
        let (_, vectors) = eigen_symmetric3(m);
        let mut new_axis = vectors[0];
        if new_axis.dot(axis) < 0.0 {
            new_axis = new_axis.scale(-1.0);
        }
        let converged = new_axis.sub(axis).length() < 1e-14
            && new_point.sub(point).length() < 1e-12 * data.diag.max(1e-12);
        axis = new_axis;
        point = new_point;
        if converged {
            break;
        }
    }
    (axis, point)
}

fn torus_candidate(
    data: &MeshData,
    tri_ids: &[u32],
    vert_ids: &[usize],
    axis: Vec3,
    point: Vec3,
) -> Option<CandidateFit> {
    // Meridian coordinates (radial distance, axial station) must trace the
    // tube circle.
    let meridian: Vec<(f64, f64)> = vert_ids
        .iter()
        .map(|&vid| {
            let d = data.verts[vid].sub(point);
            let z = d.dot(axis);
            let rho = d.sub(axis.scale(z)).length();
            (rho, z)
        })
        .collect();
    let (major_radius, z0, minor_radius) = fit_circle_2d(&meridian)?;
    let floor = 1e-9 * data.diag.max(1e-12);
    if !(major_radius > floor) || !(minor_radius > floor) {
        return None;
    }
    let center = point.add(axis.scale(z0));
    let devs: Vec<f64> = meridian
        .iter()
        .map(|&(rho, z)| ((rho - major_radius).powi(2) + (z - z0).powi(2)).sqrt() - minor_radius)
        .collect();
    let (max_dev, rms_dev) = deviation_stats(&devs);
    let tube_normal = |tri: &TriData| -> Option<Vec3> {
        let d = tri.centroid.sub(center);
        let z = d.dot(axis);
        let radial = d.sub(axis.scale(z));
        let w = radial.normalized().ok()?;
        let spine = center.add(w.scale(major_radius));
        tri.centroid.sub(spine).normalized().ok()
    };
    let (sense, max_normal_angle_deg) = normal_agreement(data, tri_ids, tube_normal);
    Some(CandidateFit {
        carrier: RegionCarrier::Torus {
            center,
            axis_dir: canonical_dir(axis),
            major_radius,
            minor_radius,
            sense,
        },
        max_dev,
        rms_dev,
        max_normal_angle_deg,
    })
}

pub(super) fn fit_torus(data: &MeshData, tri_ids: &[u32], vert_ids: &[usize]) -> Option<CandidateFit> {
    let mut total_area = 0.0;
    let mut centroid = Vec3::default();
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        total_area += tri.area;
        centroid = centroid.add(tri.centroid.scale(tri.area));
    }
    if !(total_area > 0.0) {
        return None;
    }
    centroid = centroid.scale(1.0 / total_area);

    let mut seeds = Vec::new();
    if let Some((_, covariance)) = normal_statistics(data, tri_ids) {
        let (_, vectors) = eigen_symmetric3(covariance);
        seeds.push(vectors[0]);
    }
    let mut moment_matrix = [[0.0_f64; 3]; 3];
    for &t in tri_ids {
        let tri = &data.tris[t as usize];
        let moment = tri.centroid.sub(centroid).cross(tri.normal);
        sym3_accumulate(&mut moment_matrix, moment, tri.area);
    }
    let (_, moment_vectors) = eigen_symmetric3(moment_matrix);
    seeds.push(moment_vectors[0]);

    let mut best: Option<CandidateFit> = None;
    for seed in seeds {
        if !(seed.length() > 0.5) {
            continue; // Eigenvector extraction failed.
        }
        let (axis, point) = refine_torus_axis(data, tri_ids, seed, centroid);
        if let Some(candidate) = torus_candidate(data, tri_ids, vert_ids, axis, point) {
            let better = match &best {
                None => true,
                Some(current) => candidate.max_dev < current.max_dev,
            };
            if better {
                best = Some(candidate);
            }
        }
    }
    best
}

/// A plain seed fit further off than this fraction of its own tube radius, or
/// than the cascade tolerance `gate`, is not a torus the vertex refinement
/// would rescue; such a seed costs no more than it did before.
const TUBE_SEED_FRACTION: f64 = 0.05;
/// Most steps a vertex refinement takes; a converging fit stops long before.
pub(super) const TORUS_REFINE_ITERATIONS: usize = 60;

/// [`fit_torus`], then refined on the VERTICES. The normal-line axis is
/// estimated from FACET normals at centroids, which a chordal tessellation
/// tilts: good enough for the cascade's acceptance tolerance (which keeps
/// the plain fit, so what the cascade accepts is unchanged), three orders
/// short of the precision bar the tangent-compound peel judges a seed by (a
/// quarter-tube fillet patch read 1.5e-2 against a 4e-4 bar). The vertices
/// sit on the surface; the refinement is kept only when it is better.
pub(super) fn fit_torus_refined(
    data: &MeshData,
    tri_ids: &[u32],
    vert_ids: &[usize],
    gate: f64,
) -> Option<CandidateFit> {
    let plain = fit_torus(data, tri_ids, vert_ids)?;
    let tube = match plain.carrier {
        RegionCarrier::Torus { minor_radius, .. } => minor_radius,
        _ => return Some(plain),
    };
    if !(plain.max_dev <= gate) || !(plain.max_dev <= TUBE_SEED_FRACTION * tube) {
        return Some(plain);
    }
    match refine_torus_carrier(data, tri_ids, vert_ids, &plain.carrier, TORUS_REFINE_ITERATIONS) {
        Some(refined) if refined.max_dev < plain.max_dev => Some(refined),
        _ => Some(plain),
    }
}

/// The torus through `tri_ids`' vertices, refined from `prior` (a
/// neighbouring group's carrier, or a plain fit): a warm start converges in a
/// few steps where a cold fit per growth wave would not.
pub(super) fn refine_torus_carrier(
    data: &MeshData,
    tri_ids: &[u32],
    vert_ids: &[usize],
    prior: &RegionCarrier,
    iterations: usize,
) -> Option<CandidateFit> {
    let RegionCarrier::Torus { center, axis_dir, major_radius, minor_radius, .. } = *prior else {
        return None;
    };
    let points = vert_ids.iter().map(|&vid| data.verts[vid]).collect::<Vec<_>>();
    let (axis, point) = refine_torus_on_vertices(
        &points,
        center,
        axis_dir,
        major_radius,
        minor_radius,
        iterations,
    )?;
    torus_candidate(data, tri_ids, vert_ids, axis, point)
}

/// Levenberg–Marquardt on the seven torus parameters (centre, axis
/// direction, major and minor radius), minimizing the vertices' distances
/// to the tube, from a starting torus. Each step is linearized about the
/// current torus with the analytic Jacobian (one pass over the vertices, not
/// fourteen for central differences): with `d = p − c`, `z = d·a`, `ρ` the
/// distance from the axis, `q` the distance from the tube's spine and `n` the
/// unit vector from the spine to `p`, the residual `q − r` moves by `−n·e`
/// for a centre shift along `e`, by `z (d·e) R / (ρ q)` for an axis tilt
/// toward `e`, by `−(ρ − R)/q` for the major radius and by `−1` for the
/// minor. Returns the refined axis and centre; the radii are re-fitted
/// exactly by the caller's meridian circle fit.
fn refine_torus_on_vertices(
    points: &[Vec3],
    center: Vec3,
    axis: Vec3,
    major_radius: f64,
    minor_radius: f64,
    iterations: usize,
) -> Option<(Vec3, Vec3)> {
    if points.len() < 7 {
        return None;
    }
    #[derive(Clone, Copy)]
    struct Torus {
        center: Vec3,
        axis: Vec3,
        major: f64,
        minor: f64,
    }
    let cost_of = |torus: &Torus| -> f64 {
        points
            .iter()
            .map(|&p| {
                let d = p.sub(torus.center);
                let z = d.dot(torus.axis);
                let rho = d.sub(torus.axis.scale(z)).length();
                let residual = ((rho - torus.major).powi(2) + z * z).sqrt() - torus.minor;
                residual * residual
            })
            .sum()
    };
    let mut torus = Torus {
        center,
        axis: axis.normalized().ok()?,
        major: major_radius,
        minor: minor_radius,
    };
    let scale = major_radius.abs().max(minor_radius.abs()).max(1e-12);
    let mut current = cost_of(&torus);
    let mut lambda = 1e-3;
    // A seed that is not a torus stalls rather than converging; stop it
    // after a few steps that each gain less than a percent.
    let mut stalled = 0;
    for _ in 0..iterations {
        let e1 = torus.axis.perpendicular().ok()?;
        let e2 = torus.axis.cross(e1);
        let mut jtj = [[0.0_f64; 7]; 7];
        let mut jtr = [0.0_f64; 7];
        for &p in points {
            let d = p.sub(torus.center);
            let z = d.dot(torus.axis);
            let radial = d.sub(torus.axis.scale(z));
            let rho = radial.length();
            let q = ((rho - torus.major).powi(2) + z * z).sqrt();
            if !(rho > 1e-12 * scale) || !(q > 1e-12 * scale) {
                continue;
            }
            let w = radial.scale(1.0 / rho);
            let n = w.scale((rho - torus.major) / q).add(torus.axis.scale(z / q));
            let tilt = z * torus.major / (rho * q);
            let row = [
                -n.dot(e1),
                -n.dot(e2),
                -n.dot(torus.axis),
                tilt * d.dot(e1),
                tilt * d.dot(e2),
                -(rho - torus.major) / q,
                -1.0,
            ];
            let residual = q - torus.minor;
            for i in 0..7 {
                jtr[i] -= row[i] * residual;
                for j in 0..7 {
                    jtj[i][j] += row[i] * row[j];
                }
            }
        }
        let mut improved = false;
        for _ in 0..8 {
            let mut damped = jtj;
            for (i, row) in damped.iter_mut().enumerate() {
                row[i] += lambda * jtj[i][i].max(1e-30);
            }
            let Ok(delta) = crate::fit::solve_small::<7>(damped, jtr, 7) else {
                lambda *= 10.0;
                continue;
            };
            let Ok(axis) = torus
                .axis
                .add(e1.scale(delta[3]))
                .add(e2.scale(delta[4]))
                .normalized()
            else {
                lambda *= 10.0;
                continue;
            };
            let trial = Torus {
                center: torus
                    .center
                    .add(e1.scale(delta[0]))
                    .add(e2.scale(delta[1]))
                    .add(torus.axis.scale(delta[2])),
                axis,
                major: torus.major + delta[5],
                minor: torus.minor + delta[6],
            };
            let trial_cost = cost_of(&trial);
            if trial_cost < current {
                let step = delta.iter().map(|value| value.abs()).fold(0.0, f64::max);
                stalled = if trial_cost > 0.99 * current { stalled + 1 } else { 0 };
                torus = trial;
                current = trial_cost;
                lambda = (lambda * 0.3).max(1e-12);
                improved = true;
                if step < 1e-13 * scale {
                    return Some((torus.axis, torus.center));
                }
                break;
            }
            lambda *= 10.0;
        }
        if !improved || stalled >= 3 {
            break;
        }
    }
    Some((torus.axis, torus.center))
}
