//! XT geometry read into kernel terms (Parasolid XT Format Reference §5.2):
//! analytic carriers by their frames, B-curves and B-surfaces as clamped
//! NURBS, the evaluators the translator needs for the geometry STEP has no
//! word for (SP-curves on their surfaces), and each surface's signed
//! distance, which refines intersection curves.
//!
//! Units: XT is in metres (§4.2.1); everything here is read into
//! millimetres, positions and radii alike. Parameters keep XT's meaning: a
//! parameter that is a length in XT (a plane's u and v, a cylinder's or
//! cone's axial v, a line's t) is still in metres, and the evaluators scale
//! it, so an SP-curve's 2D curve maps through its surface unchanged.

use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

use super::reader::{Node, XtFile};

/// XT lengths are metres; the kernel's are millimetres.
pub const MM_PER_METRE: f64 = 1000.0;

pub fn vec3(v: [f64; 3]) -> Vec3 {
    Vec3::new(v[0], v[1], v[2])
}

/// A position in the file, in mm.
pub fn position(v: [f64; 3]) -> Vec3 {
    vec3(v).scale(MM_PER_METRE)
}

fn required_position(node: &Node, field: &str) -> Result<Vec3, String> {
    node.vector(field)
        .map(position)
        .ok_or_else(|| format!("{} {} has no {field}", node.type_name(), node.index))
}

fn required_direction(node: &Node, field: &str) -> Result<Vec3, String> {
    node.vector(field)
        .map(vec3)
        .ok_or_else(|| format!("{} {} has no {field}", node.type_name(), node.index))?
        .normalized()
}

fn required_real(node: &Node, field: &str) -> Result<f64, String> {
    node.real(field)
        .ok_or_else(|| format!("{} {} has no {field}", node.type_name(), node.index))
}

fn required_length(node: &Node, field: &str) -> Result<f64, String> {
    required_real(node, field).map(|r| r * MM_PER_METRE)
}

/// A right-handed frame: origin (mm), z (axis or normal) and x; y = z × x.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub origin: Vec3,
    pub z: Vec3,
    pub x: Vec3,
}

impl Frame {
    pub fn y(&self) -> Vec3 {
        self.z.cross(self.x)
    }

    fn read(node: &Node, origin: &str, z: &str) -> Result<Self, String> {
        let origin = required_position(node, origin)?;
        let z = required_direction(node, z)?;
        let x = match node.vector("x_axis") {
            Some(x) => vec3(x),
            None => z.perpendicular()?,
        };
        // Square x against z: written frames are unit and orthogonal to the
        // file's resolution, which is all the STEP lane needs them to be.
        let x = x.sub(z.scale(x.dot(z))).normalized()?;
        Ok(Self { origin, z, x })
    }

    fn radial(&self, u: f64) -> Vec3 {
        self.x.scale(u.cos()).add(self.y().scale(u.sin()))
    }
}

/// An XT surface, as far as the translator reads it (mm).
#[derive(Clone, Debug)]
pub enum Surface {
    /// R(u,v) = P + uX + vY; natural normal = normal.
    Plane(Frame),
    /// R(u,v) = P + r(X cos u + Y sin u) + vA.
    Cylinder { frame: Frame, radius: f64 },
    /// R(u,v) = P + vA + (X cos u + Y sin u)(r + v tan a). The reference
    /// (§5.2.2.3) writes −vA and says the axis points away from the half in
    /// use; written files have the radius growing along the axis (the
    /// parameterisation above), and the natural normal points outwards.
    Cone { frame: Frame, radius: f64, sin: f64, cos: f64 },
    /// R(u,v) = C + (X cos u + Y sin u) r cos v + r A sin v.
    Sphere { frame: Frame, radius: f64 },
    /// R(u,v) = C + (X cos u + Y sin u)(a + b cos v) + b A sin v.
    Torus { frame: Frame, major: f64, minor: f64 },
    /// A B-surface, clamped.
    Nurbs(NurbsSurface),
    /// An offset of `base` by `distance` along its natural normal (times the
    /// base's sense, §5.2.2.8).
    Offset { base: Box<Surface>, base_sense: bool, distance: f64 },
    /// The profile spun right-handedly about the axis (§5.2.2.11).
    Spun { profile: Curve, axis_point: Vec3, axis: Vec3 },
    /// The section swept along a direction (§5.2.2.10).
    Swept { section: Curve, direction: Vec3 },
}

/// An XT curve, as far as the translator reads it (mm).
#[derive(Clone, Debug)]
pub enum Curve {
    Line { point: Vec3, direction: Vec3 },
    Circle { frame: Frame, radius: f64 },
    Ellipse { frame: Frame, major: f64, minor: f64 },
    /// A B-curve, clamped. A 2D B-curve (an SP-curve's) has z = 0 and is in
    /// its surface's parameters, unscaled.
    Nurbs(NurbsCurve),
}

impl Curve {
    pub fn point(&self, t: f64) -> Result<Vec3, String> {
        Ok(match self {
            Curve::Line { point, direction } => point.add(direction.scale(t * MM_PER_METRE)),
            Curve::Circle { frame, radius } => frame.origin.add(frame.radial(t).scale(*radius)),
            Curve::Ellipse { frame, major, minor } => frame
                .origin
                .add(frame.x.scale(major * t.cos()))
                .add(frame.y().scale(minor * t.sin())),
            Curve::Nurbs(curve) => curve.evaluate(t)?,
        })
    }

    /// The parameter range a whole closed curve, or a bounded one, covers.
    pub fn natural_range(&self) -> Option<[f64; 2]> {
        match self {
            Curve::Line { .. } => None,
            Curve::Circle { .. } | Curve::Ellipse { .. } => Some([0.0, std::f64::consts::TAU]),
            Curve::Nurbs(curve) => curve.domain().ok(),
        }
    }
}

impl Surface {
    /// Point (mm) at XT parameters (u, v).
    pub fn point(&self, u: f64, v: f64) -> Result<Vec3, String> {
        const M: f64 = MM_PER_METRE;
        Ok(match self {
            Surface::Plane(f) => f.origin.add(f.x.scale(u * M)).add(f.y().scale(v * M)),
            Surface::Cylinder { frame, radius } => frame.origin.add(frame.radial(u).scale(*radius)).add(frame.z.scale(v * M)),
            Surface::Cone { frame, radius, sin, cos } => frame
                .origin
                .add(frame.z.scale(v * M))
                .add(frame.radial(u).scale(radius + v * M * sin / cos)),
            Surface::Sphere { frame, radius } => frame
                .origin
                .add(frame.radial(u).scale(radius * v.cos()))
                .add(frame.z.scale(radius * v.sin())),
            Surface::Torus { frame, major, minor } => frame
                .origin
                .add(frame.radial(u).scale(major + minor * v.cos()))
                .add(frame.z.scale(minor * v.sin())),
            Surface::Nurbs(surface) => surface.evaluate(u, v)?,
            Surface::Offset { base, base_sense, distance } => {
                let normal = base.natural_normal(u, v)?;
                let sign = if *base_sense { 1.0 } else { -1.0 };
                base.point(u, v)?.add(normal.scale(sign * distance))
            }
            Surface::Spun { profile, axis_point, axis } => {
                let c = profile.point(u)?;
                let z = axis_point.add(axis.scale(c.sub(*axis_point).dot(*axis)));
                let r = c.sub(z);
                z.add(r.scale(v.cos())).add(axis.cross(r).scale(v.sin()))
            }
            Surface::Swept { section, direction } => section.point(u)?.add(direction.scale(v * M)),
        })
    }

    /// The unit natural normal, dP/du × dP/dv, by central differences on the
    /// exact evaluator: used only to offset and to orient, never as geometry.
    pub fn natural_normal(&self, u: f64, v: f64) -> Result<Vec3, String> {
        if let Surface::Nurbs(surface) = self {
            return surface.normal(u, v);
        }
        let h = 1e-7;
        let du = self.point(u + h, v)?.sub(self.point(u - h, v)?);
        let dv = self.point(u, v + h)?.sub(self.point(u, v - h)?);
        // Unit tangents first: on a small surface the two differences are
        // ~1e-7 mm and their product falls under the zero-length floor.
        du.normalized()?.cross(dv.normalized()?).normalized()
    }

    /// Whether the surface is a plane, cylinder, cone, sphere or torus.
    pub fn is_analytic(&self) -> bool {
        matches!(
            self,
            Surface::Plane(_) | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }
        )
    }

    /// The signed distance (mm) from `x` to the surface and its gradient (a
    /// unit vector), where the surface has one the reader can compute; the
    /// sign is arbitrary but consistent with the gradient.
    pub fn distance(&self, x: Vec3) -> Option<(f64, Vec3)> {
        let unit = |v: Vec3| v.normalized().ok();
        match self {
            Surface::Plane(f) => Some((x.sub(f.origin).dot(f.z), f.z)),
            Surface::Cylinder { frame, radius } => {
                let d = x.sub(frame.origin);
                let radial = d.sub(frame.z.scale(d.dot(frame.z)));
                let e = unit(radial)?;
                Some((radial.length() - radius, e))
            }
            Surface::Cone { frame, radius, sin, cos } => {
                // In the half-plane through the axis, the cone is the line
                // rho = r + a tan(angle); its unit normal is (cos, -sin).
                let d = x.sub(frame.origin);
                let a = d.dot(frame.z);
                let radial = d.sub(frame.z.scale(a));
                let e = unit(radial)?;
                let rho = radial.length();
                Some(((rho - radius) * cos - a * sin, e.scale(*cos).sub(frame.z.scale(*sin))))
            }
            Surface::Sphere { frame, radius } => {
                let d = x.sub(frame.origin);
                Some((d.length() - radius, unit(d)?))
            }
            Surface::Torus { frame, major, minor } => {
                let d = x.sub(frame.origin);
                let radial = d.sub(frame.z.scale(d.dot(frame.z)));
                let tube = frame.origin.add(unit(radial)?.scale(*major));
                let to = x.sub(tube);
                Some((to.length() - minor, unit(to)?))
            }
            Surface::Nurbs(surface) => {
                let foot = crate::project_point_to_surface(surface, x).ok()?;
                let normal = surface.normal(foot.u, foot.v).ok()?;
                Some((x.sub(foot.point).dot(normal), normal))
            }
            Surface::Offset { base, base_sense, distance } => {
                // The offset lies where the base's distance, measured along
                // the base's natural normal, is the offset.
                let (f, g) = base.distance(x)?;
                let normal_side = if *base_sense { *distance } else { -distance };
                // Orient the base distance with the natural normal: the
                // analytic distances above grow outwards, as their natural
                // normals point; a B-surface's is measured along its normal.
                Some((f - normal_side, g))
            }
            // A swept surface is its section's cylinder: the nearest point
            // is the section's nearest across the sweep, then slid along it.
            // Measured along the natural normal, dC/du × sweep.
            Surface::Swept { section, direction } => {
                let d = unit(*direction)?;
                let across = |p: Vec3| p.sub(d.scale(p.dot(d)));
                let u = nearest_parameter(section, [true, true], |c| across(c.sub(x)).length())?;
                let c = extended_point(section, u).ok()?;
                let foot = c.add(d.scale(x.sub(c).dot(d)));
                let normal = unit(curve_tangent(section, u)?.cross(d))?;
                Some((x.sub(foot).dot(normal), normal))
            }
            // A spun surface in the half-plane through the axis and `x` is
            // its profile's (height, radius) trace: the nearest point is that
            // trace's nearest to x's (height, radius), turned to x's side.
            // Measured along the natural normal, dP/du × dP/dv.
            Surface::Spun { profile, axis_point, axis } => {
                let k = unit(*axis)?;
                let split = |p: Vec3| {
                    let d = p.sub(*axis_point);
                    let h = d.dot(k);
                    (h, d.sub(k.scale(h)))
                };
                let (h, radial) = split(x);
                let e = unit(radial)?;
                // A profile is not continued past an end on the axis (a
                // dome's pole): past it the trace folds back on itself.
                let off_axis = |t: f64| profile.point(t).is_ok_and(|c| split(c).1.length() > 1e-6);
                let extend = profile.natural_range().map_or([true, true], |[a, b]| [off_axis(a), off_axis(b)]);
                let u = nearest_parameter(profile, extend, |c| {
                    let (hc, rc) = split(c);
                    (hc - h).hypot(rc.length() - radial.length())
                })?;
                let (hc, rc) = split(extended_point(profile, u).ok()?);
                let foot = axis_point.add(k.scale(hc)).add(e.scale(rc.length()));
                let t = curve_tangent(profile, u)?;
                // The profile's tangent, turned with its point onto x's side.
                let along = k.scale(t.dot(k)).add(e.scale(unit(rc).map_or(0.0, |r| t.dot(r))));
                let normal = unit(along.cross(k.cross(e)))?;
                Some((x.sub(foot).dot(normal), normal))
            }
        }
    }
}

/// A point of `curve` at `t`, a bounded B-curve continued past its ends
/// along its end tangents at its end speeds: Parasolid extends the sections
/// of swept and spun surfaces so (a SolidWorks part's blend spine runs
/// 123 mm along a swept section's straight extension).
/// A closed B-curve is not extended: it wraps round its period.
pub fn extended_point(curve: &Curve, t: f64) -> Result<Vec3, String> {
    let Curve::Nurbs(nurbs) = curve else { return curve.point(t) };
    if let Some([a, b]) = closed_range(curve) {
        return nurbs.evaluate(a + (t - a).rem_euclid(b - a));
    }
    let [a, b] = nurbs.domain()?;
    let h = 1e-6 * (b - a);
    if t < a {
        let p = nurbs.evaluate(a)?;
        let velocity = nurbs.evaluate(a + h)?.sub(p).scale(1.0 / h);
        Ok(p.add(velocity.scale(t - a)))
    } else if t > b {
        let p = nurbs.evaluate(b)?;
        let velocity = p.sub(nurbs.evaluate(b - h)?).scale(1.0 / h);
        Ok(p.add(velocity.scale(t - b)))
    } else {
        nurbs.evaluate(t)
    }
}

/// A B-curve's range when its ends meet (to 1e-6 mm): a closed section or
/// profile, which is searched round its period and never extended.
fn closed_range(curve: &Curve) -> Option<[f64; 2]> {
    let Curve::Nurbs(nurbs) = curve else { return None };
    let [a, b] = nurbs.domain().ok()?;
    (b > a && nurbs.evaluate(a).ok()?.sub(nurbs.evaluate(b).ok()?).length() <= 1e-6).then_some([a, b])
}

/// The parameter of `curve` (extended, [`extended_point`]) where `distance`
/// (from some fixed point) is least: the best of eight samples in every knot
/// span (fewer, but at least two, when spans are many), refined by Brent's
/// method between its neighbouring samples. A closed curve is searched over
/// its period, a bounded B-curve over its range and as far again past each
/// end `extend` names, a line over ±1 km about its point.
pub fn nearest_parameter(curve: &Curve, extend: [bool; 2], distance: impl Fn(Vec3) -> f64) -> Option<f64> {
    let f = |t: f64| extended_point(curve, t).ok().map_or(f64::INFINITY, &distance);
    // The samples, and the bounds the search may not pass: a B-curve's ends
    // it is not extended past. A closed curve's search is unbounded (its
    // foot just below the seam lies before the first sample) and wraps.
    let (mut floor, mut ceiling) = (f64::NEG_INFINITY, f64::INFINITY);
    let closed = closed_range(curve);
    let samples: Vec<f64> = match curve {
        Curve::Nurbs(nurbs) => {
            let [a, b] = nurbs.domain().ok()?;
            let extend = if closed.is_some() { [false, false] } else { extend };
            if closed.is_none() {
                if !extend[0] {
                    floor = a;
                }
                if !extend[1] {
                    ceiling = b;
                }
            }
            let mut knots: Vec<f64> = nurbs.knots.iter().copied().filter(|k| (a..=b).contains(k)).collect();
            knots.dedup();
            let spans = knots.len().saturating_sub(1).max(1);
            let per_span = (1024 / spans).clamp(2, 8);
            let mut samples = Vec::with_capacity(spans * per_span + 2 * 8 * spans + 1);
            // Each extension is as long as the range, sampled as densely as
            // the range is on average.
            let outside = (8 * spans).min(256);
            if extend[0] {
                samples.extend((0..outside).map(|k| a - (b - a) + (b - a) * k as f64 / outside as f64));
            }
            for pair in knots.windows(2) {
                samples.extend((0..per_span).map(|k| pair[0] + (pair[1] - pair[0]) * k as f64 / per_span as f64));
            }
            samples.push(b);
            if extend[1] {
                samples.extend((1..=outside).map(|k| b + (b - a) * k as f64 / outside as f64));
            }
            samples
        }
        _ => {
            // Thirty-two samples round a circle or ellipse, as many along a
            // line's ±1 km.
            let [a, b] = curve.natural_range().unwrap_or([-1000.0, 1000.0]);
            (0..=32).map(|k| a + (b - a) * k as f64 / 32.0).collect()
        }
    };
    let best = (0..samples.len()).min_by(|&i, &j| f(samples[i]).total_cmp(&f(samples[j])))?;
    // The neighbouring samples bracket the minimum; at the first or last
    // sample, the bracket runs as far again outwards (a closed curve's foot
    // just below its seam, a B-curve's foot past its last sample on its
    // extension), within the bounds.
    let t = samples[best];
    let below = if best > 0 { samples[best - 1] } else { t - (samples[1.min(samples.len() - 1)] - t) };
    let above = if best + 1 < samples.len() { samples[best + 1] } else { t + (t - samples[best.saturating_sub(1)]) };
    let (lo, hi) = (below.max(floor), above.min(ceiling));
    let tolerance = 1e-12 * (samples[samples.len() - 1] - samples[0]).abs().max(1e-300);
    let t = brent_minimum(&f, lo, hi, tolerance);
    let t = match closed {
        Some([a, b]) => a + (t - a).rem_euclid(b - a),
        None => t,
    };
    f(t).is_finite().then_some(t)
}

/// Brent's minimisation of `f` over [lo, hi] (golden sections and
/// parabolic steps), to an absolute parameter tolerance `tolerance`.
fn brent_minimum(f: &impl Fn(f64) -> f64, mut lo: f64, mut hi: f64, tolerance: f64) -> f64 {
    let golden = 0.5 * (3.0 - 5f64.sqrt());
    let mut x = lo + golden * (hi - lo);
    let (mut w, mut v) = (x, x);
    let mut fx = f(x);
    let (mut fw, mut fv) = (fx, fx);
    let (mut d, mut e) = (0.0f64, 0.0f64);
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        let tol1 = tolerance + 1e-10 * x.abs();
        let tol2 = 2.0 * tol1;
        if (x - mid).abs() <= tol2 - 0.5 * (hi - lo) {
            break;
        }
        let mut golden_step = true;
        if e.abs() > tol1 {
            // A parabola through x, w and v.
            let r = (x - w) * (fx - fv);
            let mut q = (x - v) * (fx - fw);
            let mut p = (x - v) * q - (x - w) * r;
            q = 2.0 * (q - r);
            if q > 0.0 {
                p = -p;
            }
            q = q.abs();
            if p.abs() < (0.5 * q * e).abs() && p > q * (lo - x) && p < q * (hi - x) {
                e = d;
                d = p / q;
                let u = x + d;
                if u - lo < tol2 || hi - u < tol2 {
                    d = if mid >= x { tol1 } else { -tol1 };
                }
                golden_step = false;
            }
        }
        if golden_step {
            e = if x >= mid { lo - x } else { hi - x };
            d = golden * e;
        }
        let u = if d.abs() >= tol1 { x + d } else if d > 0.0 { x + tol1 } else { x - tol1 };
        let fu = f(u);
        if fu <= fx {
            if u >= x {
                lo = x;
            } else {
                hi = x;
            }
            (v, fv, w, fw, x, fx) = (w, fw, x, fx, u, fu);
        } else {
            if u < x {
                lo = u;
            } else {
                hi = u;
            }
            if fu <= fw || w == x {
                (v, fv, w, fw) = (w, fw, u, fu);
            } else if fu <= fv || v == x || v == w {
                (v, fv) = (u, fu);
            }
        }
    }
    x
}

/// A curve's unit tangent at `t` (extended, [`extended_point`]), by central
/// differences.
fn curve_tangent(curve: &Curve, t: f64) -> Option<Vec3> {
    let h = 1e-7;
    extended_point(curve, t + h).ok()?.sub(extended_point(curve, t - h).ok()?).normalized().ok()
}

/// Read the surface node `node`.
pub fn surface(file: &XtFile, node: &Node, depth: usize) -> Result<Surface, String> {
    if depth > 8 {
        return Err("surfaces nest more than 8 deep".into());
    }
    Ok(match node.type_name() {
        "PLANE" => Surface::Plane(Frame::read(node, "pvec", "normal")?),
        "CYLINDER" => Surface::Cylinder { frame: Frame::read(node, "pvec", "axis")?, radius: required_length(node, "radius")? },
        "CONE" => Surface::Cone {
            frame: Frame::read(node, "pvec", "axis")?,
            radius: required_length(node, "radius")?,
            sin: required_real(node, "sin_half_angle")?,
            cos: required_real(node, "cos_half_angle")?,
        },
        "SPHERE" => Surface::Sphere { frame: Frame::read(node, "centre", "axis")?, radius: required_length(node, "radius")? },
        "TORUS" => {
            let major = required_length(node, "major_radius")?;
            let minor = required_length(node, "minor_radius")?;
            if major <= 0.0 {
                return Err(format!(
                    "TORUS {} is a lemon torus (major radius {major} mm), which has no STEP surface",
                    node.index
                ));
            }
            Surface::Torus { frame: Frame::read(node, "centre", "axis")?, major, minor }
        }
        "B_SURFACE" => {
            let nurbs = file
                .follow(node, "nurbs")
                .ok_or_else(|| format!("B_SURFACE {} has no NURBS_SURF", node.index))?;
            Surface::Nurbs(nurbs_surface(file, nurbs)?)
        }
        "OFFSET_SURF" => {
            let base_node = file
                .follow(node, "surface")
                .ok_or_else(|| format!("OFFSET_SURF {} has no underlying surface", node.index))?;
            Surface::Offset {
                base: Box::new(surface(file, base_node, depth + 1)?),
                base_sense: base_node.char("sense") != Some(b'-'),
                distance: required_length(node, "offset")?,
            }
        }
        "SPUN_SURF" => {
            let profile = file
                .follow(node, "profile")
                .ok_or_else(|| format!("SPUN_SURF {} has no profile", node.index))?;
            Surface::Spun {
                profile: curve(file, profile)?,
                axis_point: required_position(node, "base")?,
                axis: required_direction(node, "axis")?,
            }
        }
        "SWEPT_SURF" => {
            let section = file
                .follow(node, "section")
                .ok_or_else(|| format!("SWEPT_SURF {} has no section", node.index))?;
            Surface::Swept { section: curve(file, section)?, direction: required_direction(node, "sweep")? }
        }
        other => {
            return Err(format!(
                "surface {} is a {other}, which is not read yet",
                node.int("node_id").unwrap_or(node.index as i64)
            ))
        }
    })
}

/// Read the curve node `node` (not INTERSECTION, SP_CURVE or TRIMMED_CURVE,
/// which the translator handles through their parts), in mm.
pub fn curve(file: &XtFile, node: &Node) -> Result<Curve, String> {
    Ok(match node.type_name() {
        "LINE" => Curve::Line { point: required_position(node, "pvec")?, direction: required_direction(node, "direction")? },
        "CIRCLE" => Curve::Circle { frame: Frame::read(node, "centre", "normal")?, radius: required_length(node, "radius")? },
        "ELLIPSE" => Curve::Ellipse {
            frame: Frame::read(node, "centre", "normal")?,
            major: required_length(node, "major_radius")?,
            minor: required_length(node, "minor_radius")?,
        },
        "B_CURVE" => {
            let nurbs = file
                .follow(node, "nurbs")
                .ok_or_else(|| format!("B_CURVE {} has no NURBS_CURVE", node.index))?;
            Curve::Nurbs(nurbs_curve(file, nurbs, MM_PER_METRE)?)
        }
        other => return Err(format!("curve {} is a {other}, which is not read as a basis curve", node.index)),
    })
}

/// The 2D B-curve of an SP-curve, in its surface's parameters (unscaled).
pub fn parameter_curve(file: &XtFile, node: &Node) -> Result<NurbsCurve, String> {
    if node.type_name() != "B_CURVE" {
        return Err(format!("an SP-curve's 2D curve is a {}, not a B-curve", node.type_name()));
    }
    let nurbs = file
        .follow(node, "nurbs")
        .ok_or_else(|| format!("B_CURVE {} has no NURBS_CURVE", node.index))?;
    nurbs_curve(file, nurbs, 1.0)
}

/// Distinct knots and multiplicities, expanded. The knot and multiplicity
/// nodes may be longer than the `count` the curve states, with the rest
/// unset; only the first `count` are the knot vector.
fn expanded_knots(file: &XtFile, node: &Node, count: &str, knots: &str, mult: &str) -> Result<Vec<f64>, String> {
    let mut values = file
        .follow(node, knots)
        .ok_or_else(|| format!("{} {} has no {knots}", node.type_name(), node.index))?
        .reals("knots");
    let mut mults = file
        .follow(node, mult)
        .ok_or_else(|| format!("{} {} has no {mult}", node.type_name(), node.index))?
        .ints("mult");
    if let Some(n) = node.int(count).and_then(|n| usize::try_from(n).ok()) {
        if n > values.len() || n > mults.len() {
            return Err(format!("{} {} states {n} knots and has {}", node.type_name(), node.index, values.len().min(mults.len())));
        }
        values.truncate(n);
        mults.truncate(n);
    }
    if values.len() != mults.len() || values.iter().any(|v| !v.is_finite()) {
        return Err(format!("{} {} has {} knots and {} multiplicities", node.type_name(), node.index, values.len(), mults.len()));
    }
    // A multiplicity is at most degree + 1, and no degree XT writes nears
    // this bound; the vertex arrays bound the total long before it.
    const MAX_MULTIPLICITY: i64 = 64;
    let mut out = Vec::new();
    for (value, mult) in values.iter().zip(mults) {
        if !(1..=MAX_MULTIPLICITY).contains(&mult) {
            return Err(format!("{} {} has a knot of multiplicity {mult}", node.type_name(), node.index));
        }
        out.extend(std::iter::repeat(*value).take(mult as usize));
    }
    Ok(out)
}

/// Homogeneous control points from a vertex array, positions scaled by
/// `scale`: rational vertices are stored premultiplied by their weight, with
/// the weight last (§5.2.1.4).
fn control_points(values: &[f64], dim: usize, rational: bool, scale: f64) -> Result<Vec<Vec4>, String> {
    if dim == 0 || values.len() % dim != 0 || values.iter().any(|v| !v.is_finite()) {
        return Err(format!("a B-spline vertex array of {} values does not divide into {dim}-vectors", values.len()));
    }
    let space = if rational { dim - 1 } else { dim };
    if !(2..=3).contains(&space) {
        return Err(format!("B-spline vertices of dimension {dim} are not read"));
    }
    Ok(values
        .chunks(dim)
        .map(|c| {
            let w = if rational { c[dim - 1] } else { 1.0 };
            let z = if space == 3 { c[2] } else { 0.0 };
            // A rational vertex is already x·w, y·w, z·w.
            Vec4 { x: c[0] * scale, y: c[1] * scale, z: z * scale, w }
        })
        .collect())
}

fn nurbs_curve(file: &XtFile, node: &Node, scale: f64) -> Result<NurbsCurve, String> {
    let degree = node.int("degree").unwrap_or(0);
    let degree = usize::try_from(degree).ok().filter(|&d| d >= 1).ok_or_else(|| format!("NURBS_CURVE {} has degree {degree}", node.index))?;
    let dim = node.int("vertex_dim").unwrap_or(0).max(0) as usize;
    let rational = node.logical("rational").unwrap_or(false);
    let vertices = file
        .follow(node, "bspline_vertices")
        .ok_or_else(|| format!("NURBS_CURVE {} has no vertices", node.index))?
        .reals("vertices");
    let count = node.int("n_vertices").and_then(|n| usize::try_from(n).ok()).unwrap_or(0) * dim;
    let vertices = if count > 0 && count <= vertices.len() { &vertices[..count] } else { &vertices[..] };
    let points = control_points(vertices, dim, rational, scale)?;
    let knots = expanded_knots(file, node, "n_knots", "knots", "knot_mult")?;
    let (knots, points) = clamp(degree, knots, points)?;
    NurbsCurve::new(degree, knots, points)
}

fn nurbs_surface(file: &XtFile, node: &Node) -> Result<NurbsSurface, String> {
    let read_degree = |field: &str| {
        let d = node.int(field).unwrap_or(0);
        usize::try_from(d).ok().filter(|&d| d >= 1).ok_or_else(|| format!("NURBS_SURF {} has {field} {d}", node.index))
    };
    let (degree_u, degree_v) = (read_degree("u_degree")?, read_degree("v_degree")?);
    let nu = node.int("n_u_vertices").unwrap_or(0).max(0) as usize;
    let nv = node.int("n_v_vertices").unwrap_or(0).max(0) as usize;
    let dim = node.int("vertex_dim").unwrap_or(0).max(0) as usize;
    let rational = node.logical("rational").unwrap_or(false);
    let vertices = file
        .follow(node, "bspline_vertices")
        .ok_or_else(|| format!("NURBS_SURF {} has no vertices", node.index))?
        .reals("vertices");
    let count = nu * nv * dim;
    let vertices = if count > 0 && count <= vertices.len() { &vertices[..count] } else { &vertices[..] };
    let points = control_points(vertices, dim, rational, MM_PER_METRE)?;
    if nv == 0 || points.len() != nu * nv {
        return Err(format!("NURBS_SURF {} has {} vertices for a {nu} by {nv} net", node.index, points.len()));
    }
    // The net is stored with v varying fastest: row i holds u index i
    // (checked on corpus faces: the transposed reading misses the faces'
    // vertices by orders of magnitude more).
    let rows: Vec<Vec<Vec4>> = points.chunks(nv).map(<[Vec4]>::to_vec).collect();
    let knots_u = expanded_knots(file, node, "n_u_knots", "u_knots", "u_knot_mult")?;
    let knots_v = expanded_knots(file, node, "n_v_knots", "v_knots", "v_knot_mult")?;
    // Clamp each direction by clamping every row (v) and every column (u).
    let mut clamped_v_knots = None;
    let mut rows_v = Vec::with_capacity(rows.len());
    for row in rows {
        let (k, r) = clamp(degree_v, knots_v.clone(), row)?;
        clamped_v_knots = Some(k);
        rows_v.push(r);
    }
    let knots_v = clamped_v_knots.ok_or_else(|| format!("NURBS_SURF {} has no rows", node.index))?;
    let columns = rows_v.first().map_or(0, Vec::len);
    let mut clamped_u_knots = None;
    let mut columns_u = Vec::with_capacity(columns);
    for j in 0..columns {
        let column: Vec<Vec4> = rows_v.iter().map(|row| row[j]).collect();
        let (k, c) = clamp(degree_u, knots_u.clone(), column)?;
        clamped_u_knots = Some(k);
        columns_u.push(c);
    }
    let knots_u = clamped_u_knots.ok_or_else(|| format!("NURBS_SURF {} has no columns", node.index))?;
    let nu = columns_u.first().map_or(0, Vec::len);
    let net: Vec<Vec<Vec4>> = (0..nu).map(|i| columns_u.iter().map(|column| column[i]).collect()).collect();
    NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, net)
}

/// A B-spline restricted to its valid range [t_p, t_(m-p)] with clamped
/// (Bézier) ends. XT stores periodic B-splines unclamped (§5.2.1.4); the
/// kernel's NURBS are clamped. Knot insertion to full multiplicity at each
/// end of the valid range leaves the curve unchanged on it.
pub fn clamp(degree: usize, knots: Vec<f64>, points: Vec<Vec4>) -> Result<(Vec<f64>, Vec<Vec4>), String> {
    let p = degree;
    if points.len() < p + 1 {
        return Err(format!("a degree {p} B-spline has only {} vertices", points.len()));
    }
    if knots.windows(2).any(|w| !(w[1] >= w[0])) {
        return Err("a B-spline's knots decrease".into());
    }
    if knots.len() != points.len() + p + 1 {
        return Err(format!(
            "a degree {p} B-spline with {} vertices needs {} knots, has {}",
            points.len(),
            points.len() + p + 1,
            knots.len()
        ));
    }
    let (start, end) = (knots[p], knots[knots.len() - 1 - p]);
    if !(end > start) {
        return Err(format!("a B-spline's valid range [{start}, {end}] is empty"));
    }
    let clamped_at = |knots: &[f64], value: f64, from_start: bool| {
        let run = if from_start {
            knots.iter().take_while(|&&k| k == value).count()
        } else {
            knots.iter().rev().take_while(|&&k| k == value).count()
        };
        run >= p + 1
    };
    if clamped_at(&knots, start, true) && clamped_at(&knots, end, false) {
        return Ok((knots, points));
    }
    let (mut knots, mut points) = (knots, points);
    for value in [start, end] {
        let have = knots.iter().filter(|&&k| k == value).count();
        for _ in have..p {
            (knots, points) = insert_knot(p, &knots, &points, value);
        }
    }
    // `first` .. `first + p` are the last p copies of start, and `last` the
    // p-th copy of end from the left; the curve passes through vertex
    // `first - 1` at start and vertex `last - p` at end.
    let first = knots.iter().rposition(|&k| k == start).expect("start was inserted") + 1 - p;
    let last = knots.iter().position(|&k| k == end).expect("end was inserted") + p - 1;
    let lo = first - 1;
    let hi = last - p + 1;
    let mut new_knots = vec![start];
    new_knots.extend_from_slice(&knots[first..=last]);
    new_knots.push(end);
    let new_points = points[lo..hi].to_vec();
    if new_knots.len() != new_points.len() + p + 1 {
        return Err("clamping a periodic B-spline went wrong".into());
    }
    Ok((new_knots, new_points))
}

/// Boehm insertion of one knot.
fn insert_knot(p: usize, knots: &[f64], points: &[Vec4], value: f64) -> (Vec<f64>, Vec<Vec4>) {
    // The span k with knots[k] <= value < knots[k+1], within the valid range.
    let n = points.len();
    let mut k = p;
    while k + 1 < n && knots[k + 1] <= value {
        k += 1;
    }
    let mut new_points = Vec::with_capacity(n + 1);
    for i in 0..=n {
        let point = if i + p <= k {
            points[i]
        } else if i > k {
            points[i - 1]
        } else {
            let denominator = knots[i + p] - knots[i];
            let a = if denominator == 0.0 { 0.0 } else { (value - knots[i]) / denominator };
            points[i - 1].scale(1.0 - a).add(points[i].scale(a))
        };
        new_points.push(point);
    }
    let mut new_knots = knots.to_vec();
    new_knots.insert(k + 1, value);
    (new_knots, new_points)
}
