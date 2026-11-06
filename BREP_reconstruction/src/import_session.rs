//! Interactive mesh import: the phases of a reconstruction as data an
//! application can show while it runs, and the per-region refit a user drives
//! from them.
//!
//! The one-shot conversion ([`crate::stl_conversion`]) is unchanged by this
//! module: it reads the same segmentation the mixed builder starts from
//! ([`crate::segment_for_hybrid`], per SOURCE triangle) and reports it, and it
//! refits one region at a time on its own triangles. Nothing here is a second
//! segmentation or a second classifier.
//!
//! * [`analyze_regions`] — the regions the builder will try to fit, each with
//!   the carrier the segmentation proposed (or none: the region stays
//!   triangles) and the residual against the bar it had to meet.
//! * [`fit_region`] — refit ONE region as a chosen kind (or the best of all
//!   kinds) against a user tolerance: plane, cylinder, cone, sphere, torus
//!   through RANSAC's constrained fit, or a least-squares B-spline surface
//!   ([`fit_nurbs_region`]) where no analytic surface works.

use crate::stl_conversion::{segmentation_options, StlConversionOptions};
use crate::{
    reconstruct_surface, AnalyticSurface, Mesh, RecognitionOptions, SurfaceHint, SurfaceType, Vec3,
};
use brep_kernel::{RegionCarrier, UNASSIGNED_REGION};
use serde::{Deserialize, Serialize};

/// What a region is (proposed or fitted) as.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum RegionKind {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    /// A least-squares B-spline surface.
    Nurbs,
    /// No surface: the region stays mesh triangles.
    Faceted,
}

impl RegionKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
            Self::Sphere => "sphere",
            Self::Torus => "torus",
            Self::Nurbs => "NURBS surface",
            Self::Faceted => "triangles",
        }
    }

    fn of_carrier(carrier: &RegionCarrier) -> Self {
        match carrier {
            RegionCarrier::Plane { .. } => Self::Plane,
            RegionCarrier::Cylinder { .. } => Self::Cylinder,
            RegionCarrier::Cone { .. } => Self::Cone,
            RegionCarrier::Sphere { .. } => Self::Sphere,
            RegionCarrier::Torus { .. } => Self::Torus,
            RegionCarrier::Freeform => Self::Faceted,
        }
    }
}

/// One region of the segmentation the mixed builder starts from.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ImportRegion {
    /// Segmentation region id (the value in [`ImportRegions::triangle_regions`]).
    pub id: u32,
    /// Source triangles in it.
    pub triangles: usize,
    pub area: f64,
    /// The proposed kind; [`RegionKind::Faceted`] when no carrier met the bar.
    pub kind: RegionKind,
    /// The segmentation's carrier (`Freeform` when faceted).
    pub carrier: RegionCarrier,
    /// Largest vertex distance from the carrier (0 for a faceted region:
    /// nothing was accepted to measure against).
    pub max_deviation: f64,
    pub rms_deviation: f64,
    pub max_normal_angle_deg: f64,
    /// The absolute distance a carrier had to meet: the relative fit tolerance
    /// times the region's bounding-box diagonal.
    pub fit_bar: f64,
    /// Bounding-box diagonal of the region, mm.
    pub diagonal: f64,
    /// Why the region stays triangles, in words; `None` when it is proposed
    /// as a surface.
    pub reason: Option<String>,
}

/// The segmentation phase of an import.
#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ImportRegions {
    /// Region id of every source triangle ([`UNASSIGNED_REGION`] for a
    /// degenerate triangle no region took).
    pub triangle_regions: Vec<u32>,
    pub regions: Vec<ImportRegion>,
    /// The relative fit tolerance and normal gate the segmentation used.
    pub fit_tolerance: f64,
    pub normal_tolerance_deg: f64,
}

impl ImportRegions {
    /// Source triangles of region `id`, ascending.
    pub fn triangles_of(&self, id: u32) -> Vec<u32> {
        self.triangle_regions
            .iter()
            .enumerate()
            .filter(|(_, &region)| region == id)
            .map(|(t, _)| t as u32)
            .collect()
    }
}

/// The regions the mixed builder will start from, per source triangle.
///
/// `positions`/`indices` are the welded buffers the conversion reads (the
/// reader's output); `options` the conversion's own, so the segmentation is
/// the one the builder runs.
pub fn analyze_regions(
    positions: &[f64],
    indices: &[u32],
    options: &StlConversionOptions,
) -> Result<ImportRegions, String> {
    let segment_options = segmentation_options(positions, options);
    let segmentation = crate::segment_for_hybrid(positions, indices, &segment_options)?;
    let triangle_count = indices.len() / 3;
    if segmentation.triangle_region_ids.len() != triangle_count {
        return Err(format!(
            "the segmentation labelled {} triangles of {triangle_count}",
            segmentation.triangle_region_ids.len()
        ));
    }
    let regions = segmentation
        .regions
        .iter()
        .filter(|region| region.triangle_count > 0)
        .map(|region| {
            let (low, high) = (region.bbox_min, region.bbox_max);
            let diagonal = ((high.x - low.x).powi(2) + (high.y - low.y).powi(2) + (high.z - low.z).powi(2)).sqrt();
            let fit_bar = segment_options.fit_tolerance * diagonal;
            let kind = RegionKind::of_carrier(&region.carrier);
            let reason = (kind == RegionKind::Faceted).then(|| {
                if region.triangle_count < segment_options.min_region_triangles {
                    format!(
                        "too few triangles to fit ({} < {})",
                        region.triangle_count, segment_options.min_region_triangles
                    )
                } else {
                    format!(
                        "no plane, cylinder, cone, sphere or torus within {} mm ({:.3}% of the region size) \
                         and {:.1}° of the triangle normals",
                        short(fit_bar),
                        segment_options.fit_tolerance * 100.0,
                        segment_options.normal_tolerance_deg
                    )
                }
            });
            ImportRegion {
                id: region.id,
                triangles: region.triangle_count,
                area: region.area,
                kind,
                carrier: region.carrier.clone(),
                max_deviation: region.max_deviation,
                rms_deviation: region.rms_deviation,
                max_normal_angle_deg: region.max_normal_angle_deg,
                fit_bar,
                diagonal,
                reason,
            }
        })
        .collect();
    Ok(ImportRegions {
        triangle_regions: segmentation.triangle_region_ids,
        regions,
        fit_tolerance: segment_options.fit_tolerance,
        normal_tolerance_deg: segment_options.normal_tolerance_deg,
    })
}

/// A distance in mm for a message: three significant figures.
fn short(value: f64) -> String {
    if value == 0.0 || !value.is_finite() {
        return format!("{value}");
    }
    let digits = (2 - value.abs().log10().floor() as i32).max(0) as usize;
    format!("{value:.digits$}")
}

/// What to fit a region as.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FitKind {
    /// The simplest analytic kind within tolerance, else a NURBS surface.
    #[default]
    Auto,
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurbs,
}

impl FitKind {
    pub const ALL: [FitKind; 7] = [
        FitKind::Auto,
        FitKind::Plane,
        FitKind::Cylinder,
        FitKind::Cone,
        FitKind::Sphere,
        FitKind::Torus,
        FitKind::Nurbs,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Auto => "Best fit",
            Self::Plane => "Plane",
            Self::Cylinder => "Cylinder",
            Self::Cone => "Cone",
            Self::Sphere => "Sphere",
            Self::Torus => "Torus",
            Self::Nurbs => "NURBS surface",
        }
    }

    fn surface_type(self) -> Option<SurfaceType> {
        match self {
            Self::Plane => Some(SurfaceType::Plane),
            Self::Cylinder => Some(SurfaceType::Cylinder),
            Self::Cone => Some(SurfaceType::Cone),
            Self::Sphere => Some(SurfaceType::Sphere),
            Self::Torus => Some(SurfaceType::Torus),
            Self::Auto | Self::Nurbs => None,
        }
    }
}

/// Refit one region.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RegionFitRequest {
    /// The region's source triangles.
    pub triangles: Vec<u32>,
    pub kind: FitKind,
    /// Largest vertex distance accepted, mm.
    pub tolerance: f64,
    /// Largest angle between a triangle normal and the surface normal at its
    /// centroid, degrees (analytic kinds).
    pub normal_tolerance_deg: f64,
}

/// One kind tried by [`fit_region`].
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FitAttempt {
    pub kind: RegionKind,
    /// `None` when the kind could not be fitted at all.
    pub max_deviation: Option<f64>,
    pub max_normal_angle_deg: Option<f64>,
    pub within_tolerance: bool,
    pub note: Option<String>,
}

/// The result of [`fit_region`].
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RegionFit {
    /// The kind kept: the requested one, or for [`FitKind::Auto`] the first
    /// within tolerance (or the closest when none is).
    pub kind: Option<RegionKind>,
    pub within_tolerance: bool,
    pub max_deviation: f64,
    pub rms_deviation: f64,
    pub max_normal_angle_deg: f64,
    pub tolerance: f64,
    pub surface: Option<AnalyticSurface>,
    pub nurbs: Option<NurbsFit>,
    /// Every kind tried, in order.
    pub attempts: Vec<FitAttempt>,
    /// The result in words, for the panel.
    pub message: String,
}

/// The region's triangles as a mesh of their own (vertices renumbered).
fn region_mesh(positions: &[f64], indices: &[u32], triangles: &[u32]) -> Result<Mesh, String> {
    let vertex_count = positions.len() / 3;
    let mut map = std::collections::HashMap::<u32, u32>::new();
    let mut vertices = Vec::new();
    let mut local = Vec::with_capacity(triangles.len());
    for &t in triangles {
        let t = t as usize;
        if 3 * t + 2 >= indices.len() {
            return Err(format!("triangle {t} is outside the mesh"));
        }
        let mut corners = [0u32; 3];
        for (k, corner) in corners.iter_mut().enumerate() {
            let v = indices[3 * t + k];
            if v as usize >= vertex_count {
                return Err(format!("triangle {t} names vertex {v} outside the mesh"));
            }
            *corner = *map.entry(v).or_insert_with(|| {
                let p = &positions[3 * v as usize..3 * v as usize + 3];
                vertices.push(Vec3::new(p[0], p[1], p[2]));
                (vertices.len() - 1) as u32
            });
        }
        local.push(corners);
    }
    if local.is_empty() {
        return Err("the region has no triangles".into());
    }
    Ok(Mesh::new(vertices, local))
}

fn triangle_normal(mesh: &Mesh, triangle: [u32; 3]) -> Option<(Vec3, Vec3)> {
    let [a, b, c] = triangle.map(|v| mesh.vertices[v as usize]);
    let normal = (b - a).cross(c - a).normalized()?;
    Some(((a + b + c) * (1.0 / 3.0), normal))
}

/// Deviation of the mesh from an analytic surface: (max, rms) vertex
/// distance and the largest triangle-normal angle (degrees, orientation-free).
fn analytic_deviation(mesh: &Mesh, surface: AnalyticSurface) -> (f64, f64, f64) {
    let mut max = 0.0f64;
    let mut sum = 0.0;
    for &p in &mesh.vertices {
        let d = surface.signed_distance(p).abs();
        max = max.max(d);
        sum += d * d;
    }
    let rms = (sum / mesh.vertices.len().max(1) as f64).sqrt();
    let mut angle = 0.0f64;
    for &triangle in &mesh.triangles {
        let Some((centroid, normal)) = triangle_normal(mesh, triangle) else { continue };
        if let Some(surface_normal) = surface.normal_at(centroid) {
            let cos = normal.dot(surface_normal).abs().min(1.0);
            angle = angle.max(cos.acos().to_degrees());
        }
    }
    (max, rms, angle)
}

fn mesh_diagonal(mesh: &Mesh) -> f64 {
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for p in &mesh.vertices {
        low = Vec3::new(low.x.min(p.x), low.y.min(p.y), low.z.min(p.z));
        high = Vec3::new(high.x.max(p.x), high.y.max(p.y), high.z.max(p.z));
    }
    (high - low).length()
}

/// The best analytic `kind` for the region, measured against nothing yet:
/// RANSAC's constrained fit with its gates opened to the region's size, so a
/// fit that misses the user's tolerance still comes back with its residual.
fn fit_analytic(mesh: &Mesh, kind: SurfaceType) -> Result<AnalyticSurface, String> {
    let diagonal = mesh_diagonal(mesh).max(f64::MIN_POSITIVE);
    let mut options = RecognitionOptions::default();
    options.distance_tolerance = diagonal;
    options.relative_tolerance = 0.0;
    options.normal_tolerance = 89.0f64.to_radians();
    options.minimum_support = 1;
    options.minimum_support_area = 0.0;
    options.deterministic_seed = Some(0);
    options.respect_features = false;
    options.collect_phase_timings = false;
    options.threads = 1;
    // Welded mesh vertices are the samples ON the design surface; facet
    // centroids lie on chords (the conversion's own default).
    options.sampling = crate::SamplingMode::Vertices;
    let triangles: Vec<usize> = (0..mesh.triangles.len()).collect();
    reconstruct_surface(mesh, &triangles, &SurfaceHint::KnownType { surface_type: kind }, &options)
        .map(|fit| fit.surface)
        .map_err(|error| error.to_string())
}

fn kind_of(surface_type: SurfaceType) -> RegionKind {
    match surface_type {
        SurfaceType::Plane => RegionKind::Plane,
        SurfaceType::Cylinder => RegionKind::Cylinder,
        SurfaceType::Cone => RegionKind::Cone,
        SurfaceType::Sphere => RegionKind::Sphere,
        SurfaceType::Torus => RegionKind::Torus,
    }
}

/// Refit one region as `request.kind` against `request.tolerance`.
pub fn fit_region(positions: &[f64], indices: &[u32], request: &RegionFitRequest) -> RegionFit {
    let mut fit = RegionFit {
        kind: None,
        within_tolerance: false,
        max_deviation: f64::INFINITY,
        rms_deviation: f64::INFINITY,
        max_normal_angle_deg: 0.0,
        tolerance: request.tolerance,
        surface: None,
        nurbs: None,
        attempts: Vec::new(),
        message: String::new(),
    };
    if !(request.tolerance.is_finite() && request.tolerance > 0.0) {
        fit.message = "the tolerance must be a positive distance".into();
        return fit;
    }
    let mesh = match region_mesh(positions, indices, &request.triangles) {
        Ok(mesh) => mesh,
        Err(error) => {
            fit.message = error;
            return fit;
        }
    };
    let analytic: Vec<SurfaceType> = match request.kind {
        FitKind::Auto => vec![
            SurfaceType::Plane,
            SurfaceType::Cylinder,
            SurfaceType::Cone,
            SurfaceType::Sphere,
            SurfaceType::Torus,
        ],
        FitKind::Nurbs => Vec::new(),
        kind => vec![kind.surface_type().unwrap()],
    };
    // The best attempt so far, by (within tolerance first, then distance).
    let mut best: Option<(bool, f64)> = None;
    for surface_type in analytic {
        let kind = kind_of(surface_type);
        match fit_analytic(&mesh, surface_type) {
            Ok(surface) => {
                let (max, rms, angle) = analytic_deviation(&mesh, surface);
                let within = max <= request.tolerance && angle <= request.normal_tolerance_deg;
                fit.attempts.push(FitAttempt {
                    kind,
                    max_deviation: Some(max),
                    max_normal_angle_deg: Some(angle),
                    within_tolerance: within,
                    note: None,
                });
                let better = match best {
                    None => true,
                    Some((best_within, best_max)) => (within && !best_within) || (within == best_within && !best_within && max < best_max),
                };
                if better {
                    best = Some((within, max));
                    fit.kind = Some(kind);
                    fit.surface = Some(surface);
                    fit.max_deviation = max;
                    fit.rms_deviation = rms;
                    fit.max_normal_angle_deg = angle;
                    fit.within_tolerance = within;
                }
                // Auto keeps the SIMPLEST kind within tolerance.
                if within && request.kind == FitKind::Auto {
                    break;
                }
            }
            Err(error) => fit.attempts.push(FitAttempt {
                kind,
                max_deviation: None,
                max_normal_angle_deg: None,
                within_tolerance: false,
                note: Some(error),
            }),
        }
    }
    let try_nurbs = request.kind == FitKind::Nurbs || (request.kind == FitKind::Auto && !fit.within_tolerance);
    if try_nurbs {
        match fit_nurbs_region(&mesh, request.tolerance) {
            Ok(nurbs) => {
                let within = nurbs.max_deviation <= request.tolerance;
                fit.attempts.push(FitAttempt {
                    kind: RegionKind::Nurbs,
                    max_deviation: Some(nurbs.max_deviation),
                    max_normal_angle_deg: None,
                    within_tolerance: within,
                    note: Some(format!("{} × {} control points", nurbs.count_u, nurbs.count_v)),
                });
                let better = match best {
                    None => true,
                    Some((best_within, best_max)) => (within && !best_within) || (within == best_within && !best_within && nurbs.max_deviation < best_max),
                } || request.kind == FitKind::Nurbs;
                if better {
                    fit.kind = Some(RegionKind::Nurbs);
                    fit.surface = None;
                    fit.max_deviation = nurbs.max_deviation;
                    fit.rms_deviation = nurbs.rms_deviation;
                    fit.max_normal_angle_deg = 0.0;
                    fit.within_tolerance = within;
                    fit.nurbs = Some(nurbs);
                }
            }
            Err(error) => fit.attempts.push(FitAttempt {
                kind: RegionKind::Nurbs,
                max_deviation: None,
                max_normal_angle_deg: None,
                within_tolerance: false,
                note: Some(error),
            }),
        }
    }
    fit.message = match fit.kind {
        None => {
            let notes = fit
                .attempts
                .iter()
                .filter_map(|attempt| attempt.note.as_ref().map(|note| format!("{}: {note}", attempt.kind.label())))
                .collect::<Vec<_>>();
            format!("No surface could be fitted. {}", notes.join("; "))
        }
        Some(kind) if fit.within_tolerance => format!(
            "Fits as a {} — largest deviation {} mm (tolerance {} mm).",
            kind.label(),
            short(fit.max_deviation),
            short(request.tolerance)
        ),
        Some(kind) => {
            let normal = kind != RegionKind::Nurbs
                && fit.max_deviation <= request.tolerance
                && fit.max_normal_angle_deg > request.normal_tolerance_deg;
            if normal {
                format!(
                    "Closest is a {}, within {} mm, but its normals differ from the triangles' by {:.2}° (limit {:.2}°).",
                    kind.label(),
                    short(fit.max_deviation),
                    fit.max_normal_angle_deg,
                    request.normal_tolerance_deg
                )
            } else {
                format!(
                    "Does not fit: the closest is a {} at {} mm, over the {} mm tolerance.",
                    kind.label(),
                    short(fit.max_deviation),
                    short(request.tolerance)
                )
            }
        }
    };
    fit
}

/// A least-squares B-spline surface fitted to a region.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct NurbsFit {
    pub degree: usize,
    pub count_u: usize,
    pub count_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    /// `count_u * count_v` control points, `u`-major (`[i * count_v + j]`).
    pub control_points: Vec<[f64; 3]>,
    /// Largest distance from a vertex to the surface point at its fitting
    /// parameter — an upper bound on its distance to the surface.
    pub max_deviation: f64,
    pub rms_deviation: f64,
}

impl NurbsFit {
    /// The fit as a kernel surface (unit weights).
    pub fn to_kernel_surface(&self) -> Result<brep_kernel::NurbsSurface, String> {
        let rows = (0..self.count_u)
            .map(|i| {
                (0..self.count_v)
                    .map(|j| {
                        let p = self.control_points[i * self.count_v + j];
                        brep_kernel::Vec4::from_point(brep_kernel::Vec3::new(p[0], p[1], p[2]), 1.0)
                    })
                    .collect()
            })
            .collect();
        brep_kernel::NurbsSurface::new(
            self.degree,
            self.degree,
            self.knots_u.clone(),
            self.knots_v.clone(),
            rows,
        )
    }

    /// The surface point at `(u, v)` in `[0, 1]²`.
    pub fn evaluate(&self, u: f64, v: f64) -> [f64; 3] {
        let (span_u, basis_u) = basis(&self.knots_u, self.degree, self.count_u, u);
        let (span_v, basis_v) = basis(&self.knots_v, self.degree, self.count_v, v);
        let mut point = [0.0; 3];
        for (a, bu) in basis_u.iter().enumerate() {
            for (b, bv) in basis_v.iter().enumerate() {
                let p = self.control_points[(span_u - self.degree + a) * self.count_v + span_v - self.degree + b];
                for k in 0..3 {
                    point[k] += bu * bv * p[k];
                }
            }
        }
        point
    }
}

/// Clamped uniform knots for `count` control points of `degree` on [0, 1].
fn clamped_knots(count: usize, degree: usize) -> Vec<f64> {
    let interior = count - degree - 1;
    let mut knots = vec![0.0; degree + 1];
    knots.extend((1..=interior).map(|k| k as f64 / (interior + 1) as f64));
    knots.extend(std::iter::repeat(1.0).take(degree + 1));
    knots
}

/// The span index and the `degree + 1` non-zero basis values at `t`
/// (Cox–de Boor, The NURBS Book A2.1/A2.2).
fn basis(knots: &[f64], degree: usize, count: usize, t: f64) -> (usize, Vec<f64>) {
    let t = t.clamp(knots[degree], knots[count]);
    let span = if t >= knots[count] {
        count - 1
    } else {
        let mut low = degree;
        let mut high = count;
        while high - low > 1 {
            let mid = (low + high) / 2;
            if t < knots[mid] {
                high = mid;
            } else {
                low = mid;
            }
        }
        low
    };
    let mut values = vec![0.0; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    values[0] = 1.0;
    for j in 1..=degree {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let denominator = right[r + 1] + left[j - r];
            let temp = if denominator == 0.0 { 0.0 } else { values[r] / denominator };
            values[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        values[j] = saved;
    }
    (span, values)
}

/// Principal axes of a point set: centroid and three unit axes, largest
/// spread first (Jacobi on the 3×3 covariance).
fn principal_frame(points: &[Vec3]) -> (Vec3, [Vec3; 3]) {
    let n = points.len().max(1) as f64;
    let centroid = points.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, &p| sum + p) * (1.0 / n);
    let mut c = [[0.0f64; 3]; 3];
    for &p in points {
        let d = [p.x - centroid.x, p.y - centroid.y, p.z - centroid.z];
        for i in 0..3 {
            for j in 0..3 {
                c[i][j] += d[i] * d[j];
            }
        }
    }
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut largest) = (0, 1, 0.0f64);
        for i in 0..3 {
            for j in i + 1..3 {
                if c[i][j].abs() > largest {
                    largest = c[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if largest <= 1e-300 {
            break;
        }
        let theta = 0.5 * (2.0 * c[p][q]).atan2(c[q][q] - c[p][p]);
        let (s, co) = theta.sin_cos();
        let mut next = c;
        for k in 0..3 {
            next[k][p] = co * c[k][p] - s * c[k][q];
            next[k][q] = s * c[k][p] + co * c[k][q];
        }
        let rotated = next;
        for k in 0..3 {
            next[p][k] = co * rotated[p][k] - s * rotated[q][k];
            next[q][k] = s * rotated[p][k] + co * rotated[q][k];
        }
        c = next;
        for row in &mut v {
            let (a, b) = (row[p], row[q]);
            row[p] = co * a - s * b;
            row[q] = s * a + co * b;
        }
    }
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| c[b][b].total_cmp(&c[a][a]));
    let axis = |k: usize| Vec3::new(v[0][k], v[1][k], v[2][k]);
    (centroid, [axis(order[0]), axis(order[1]), axis(order[2])])
}

/// Dense symmetric positive-definite solve for several right-hand sides
/// (Cholesky). `matrix` is `n × n` row-major.
fn cholesky_solve(mut matrix: Vec<f64>, n: usize, rhs: &mut [Vec<f64>]) -> Result<(), String> {
    for j in 0..n {
        let mut diagonal = matrix[j * n + j];
        for k in 0..j {
            diagonal -= matrix[j * n + k] * matrix[j * n + k];
        }
        if !(diagonal > 0.0) {
            return Err("the least-squares system is singular".into());
        }
        let diagonal = diagonal.sqrt();
        matrix[j * n + j] = diagonal;
        for i in j + 1..n {
            let mut value = matrix[i * n + j];
            for k in 0..j {
                value -= matrix[i * n + k] * matrix[j * n + k];
            }
            matrix[i * n + j] = value / diagonal;
        }
    }
    for b in rhs.iter_mut() {
        for i in 0..n {
            let mut value = b[i];
            for k in 0..i {
                value -= matrix[i * n + k] * b[k];
            }
            b[i] = value / matrix[i * n + i];
        }
        for i in (0..n).rev() {
            let mut value = b[i];
            for k in i + 1..n {
                value -= matrix[k * n + i] * b[k];
            }
            b[i] = value / matrix[i * n + i];
        }
    }
    Ok(())
}

/// Control-net sizes tried, coarse to fine; the first whose fit meets the
/// tolerance is kept (the finest otherwise).
const NURBS_NETS: [usize; 9] = [4, 5, 6, 8, 10, 12, 16, 20, 24];
const NURBS_DEGREE: usize = 3;

/// Fit a cubic B-spline surface to a region's vertices by least squares.
///
/// The vertices are parametrized by projection onto the region's best-fit
/// plane, so the region must not fold over itself seen from that plane (a
/// closed tube, a deep cup): such a region is refused with the reason. A
/// small smoothing term keeps control points with no vertex under them
/// determined. Nets grow from 4 × 4 until the largest deviation meets
/// `tolerance`.
pub fn fit_nurbs_region(mesh: &Mesh, tolerance: f64) -> Result<NurbsFit, String> {
    let points = &mesh.vertices;
    if points.len() < 4 {
        return Err("a NURBS surface needs at least four vertices".into());
    }
    let (centroid, [e1, e2, e3]) = principal_frame(points);
    let uv: Vec<[f64; 2]> = points
        .iter()
        .map(|&p| {
            let d = p - centroid;
            [d.dot(e1), d.dot(e2)]
        })
        .collect();
    // Fold check: the triangles must keep one winding in the projection.
    // Scan noise flips the odd sliver, so a fold is a flipped AREA beyond a
    // thousandth of the whole, not one flipped triangle.
    let mut positive = 0.0;
    let mut negative = 0.0;
    for triangle in &mesh.triangles {
        let [a, b, c] = triangle.map(|v| uv[v as usize]);
        let area = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if area > 0.0 {
            positive += area;
        } else {
            negative -= area;
        }
    }
    let total = positive + negative;
    let surface_area: f64 = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.map(|v| points[v as usize]);
            0.5 * (b - a).cross(c - a).length()
        })
        .sum();
    // Twice the projected area against twice the true area: a wall seen
    // edge-on (a tube's) projects to nothing, a fold cancels itself.
    if total <= 1e-3 * 2.0 * surface_area || positive.min(negative) > 1e-3 * total {
        return Err(
            "the region folds over itself seen from its best-fit plane (a tube or a deep cup); \
             split it, or fit it as a cylinder, cone, sphere or torus"
                .into(),
        );
    }
    let _ = e3;
    let (mut low, mut high) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &uv {
        for k in 0..2 {
            low[k] = low[k].min(p[k]);
            high[k] = high[k].max(p[k]);
        }
    }
    let span = [(high[0] - low[0]).max(f64::MIN_POSITIVE), (high[1] - low[1]).max(f64::MIN_POSITIVE)];
    let params: Vec<[f64; 2]> = uv
        .iter()
        .map(|p| [(p[0] - low[0]) / span[0], (p[1] - low[1]) / span[1]])
        .collect();
    let mut last: Option<NurbsFit> = None;
    for &net in &NURBS_NETS {
        let fit = nurbs_least_squares(points, &params, net, net)?;
        let done = fit.max_deviation <= tolerance;
        last = Some(fit);
        if done {
            break;
        }
    }
    Ok(last.unwrap())
}

fn nurbs_least_squares(points: &[Vec3], params: &[[f64; 2]], count_u: usize, count_v: usize) -> Result<NurbsFit, String> {
    let degree = NURBS_DEGREE.min(count_u - 1).min(count_v - 1);
    let knots_u = clamped_knots(count_u, degree);
    let knots_v = clamped_knots(count_v, degree);
    let n = count_u * count_v;
    let mut normal = vec![0.0; n * n];
    let mut rhs = vec![vec![0.0; n]; 3];
    let mut rows = Vec::with_capacity(points.len());
    for (point, param) in points.iter().zip(params) {
        let (span_u, basis_u) = basis(&knots_u, degree, count_u, param[0]);
        let (span_v, basis_v) = basis(&knots_v, degree, count_v, param[1]);
        let mut row = Vec::with_capacity((degree + 1) * (degree + 1));
        for (a, bu) in basis_u.iter().enumerate() {
            for (b, bv) in basis_v.iter().enumerate() {
                row.push(((span_u - degree + a) * count_v + span_v - degree + b, bu * bv));
            }
        }
        for &(i, wi) in &row {
            for &(j, wj) in &row {
                normal[i * n + j] += wi * wj;
            }
            rhs[0][i] += wi * point.x;
            rhs[1][i] += wi * point.y;
            rhs[2][i] += wi * point.z;
        }
        rows.push(row);
    }
    // Smoothing: second differences of the control net in both directions,
    // weighted to a small fraction of the data term.
    let trace = (0..n).map(|i| normal[i * n + i]).sum::<f64>() / n as f64;
    let weight = 1e-6 * trace.max(1e-12);
    let mut add_difference = |cells: [usize; 3]| {
        let coefficients = [1.0, -2.0, 1.0];
        for (a, &i) in cells.iter().enumerate() {
            for (b, &j) in cells.iter().enumerate() {
                normal[i * n + j] += weight * coefficients[a] * coefficients[b];
            }
        }
    };
    for i in 0..count_u {
        for j in 1..count_v - 1 {
            add_difference([i * count_v + j - 1, i * count_v + j, i * count_v + j + 1]);
        }
    }
    for j in 0..count_v {
        for i in 1..count_u - 1 {
            add_difference([(i - 1) * count_v + j, i * count_v + j, (i + 1) * count_v + j]);
        }
    }
    // A tiny ridge keeps an all-empty corner of the net determined.
    for i in 0..n {
        normal[i * n + i] += 1e-12 * trace.max(1e-12);
    }
    cholesky_solve(normal, n, &mut rhs)?;
    let control_points: Vec<[f64; 3]> = (0..n).map(|i| [rhs[0][i], rhs[1][i], rhs[2][i]]).collect();
    let mut max = 0.0f64;
    let mut sum = 0.0;
    for (point, row) in points.iter().zip(&rows) {
        let mut s = [0.0; 3];
        for &(i, w) in row {
            for k in 0..3 {
                s[k] += w * control_points[i][k];
            }
        }
        let d = Vec3::new(s[0], s[1], s[2]).distance(*point);
        max = max.max(d);
        sum += d * d;
    }
    Ok(NurbsFit {
        degree,
        count_u,
        count_v,
        knots_u,
        knots_v,
        control_points,
        max_deviation: max,
        rms_deviation: (sum / points.len() as f64).sqrt(),
    })
}

#[path = "import_session/nurbs_face.rs"]
mod nurbs_face;
pub use nurbs_face::{apply_nurbs_faces, NurbsFaceChoice, NurbsFaceOutcome};

/// [`principal_frame`] on kernel vectors.
fn principal_frame_kernel(points: &[brep_kernel::Vec3]) -> (brep_kernel::Vec3, [brep_kernel::Vec3; 3]) {
    let converted: Vec<Vec3> = points.iter().map(|p| Vec3::new(p.x, p.y, p.z)).collect();
    let (c, axes) = principal_frame(&converted);
    let k = |v: Vec3| brep_kernel::Vec3::new(v.x, v.y, v.z);
    (k(c), axes.map(k))
}

/// The accepted import with the user's NURBS faces: the conversion's STEP
/// with each chosen region's facets replaced by one NURBS face
/// ([`apply_nurbs_faces`]). The STEP is returned unchanged when no choice
/// applies. Only a single-body import takes them (a body is matched by its
/// mesh vertices; a multi-body document is refused whole, with the reason).
pub fn apply_region_choices(
    step_text: &str,
    positions: &[f64],
    indices: &[u32],
    choices: &[NurbsFaceChoice],
) -> Result<(String, Vec<NurbsFaceOutcome>), String> {
    let mut solids = brep_kernel::import_step(step_text)?;
    if solids.len() != 1 {
        let reason = format!("the import holds {} bodies; NURBS faces are placed on single-body imports", solids.len());
        return Ok((step_text.to_string(), choices.iter().map(|_| NurbsFaceOutcome { applied: false, max_deviation: None, control_points: None, reason: Some(reason.clone()) }).collect()));
    }
    let outcomes = apply_nurbs_faces(&mut solids[0], positions, indices, choices);
    if !outcomes.iter().any(|outcome| outcome.applied) {
        return Ok((step_text.to_string(), outcomes));
    }
    let report = brep_kernel::export_step_report(&solids, "Imported mesh", "MM", "")?;
    let text = report.text;
    // The file must read back as the body it was written from.
    let back = brep_kernel::import_step(&text)?;
    if back.len() != 1 || !back[0].validate().is_empty() {
        return Err("the STEP written with the NURBS faces does not read back as one valid body".into());
    }
    Ok((text, outcomes))
}

/// Unassigned triangles' region id, re-exported for callers colouring them.
pub const UNASSIGNED: u32 = UNASSIGNED_REGION;

