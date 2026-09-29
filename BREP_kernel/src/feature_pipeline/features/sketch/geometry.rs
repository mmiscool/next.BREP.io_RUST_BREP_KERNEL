use super::*;

/// A directed loop segment in solved sketch UV. `a`→`b` is the natural traversal
/// direction; the chainer reverses via [`NurbsCurve::reversed`] when it walks a
/// segment tail-first. `name` is the source sketch-edge name (`{sketchId}:G{gid}`),
/// carried through so the profile-consumers can stamp byte-exact sidewall names.
pub(super) struct Segment {
    pub(super) a: [f64; 2],
    pub(super) b: [f64; 2],
    pub(super) kind: SegKind,
    pub(super) name: Option<String>,
    /// The source geometry's numeric id + the LOOP id persisted on it. Carried so
    /// the assembled loops can resolve a STABLE per-loop identity (see
    /// [`super::loop_ids`]) that survives adding/removing edges.
    pub(super) ids: SegmentIds,
}

/// The identity a segment inherits from its sketch geometry: `gid` is the
/// geometry's own numeric id, `loop_id` the loop id persisted on it by a previous
/// commit (`None` on geometry that has never been assigned one).
#[derive(Clone, Copy, Default)]
pub(super) struct SegmentIds {
    pub(super) gid: Option<u64>,
    pub(super) loop_id: Option<u64>,
}

pub(super) enum SegKind {
    Line,
    /// CCW arc from `start_angle` sweeping `sweep` (radians) about `center`.
    Arc {
        center: [f64; 2],
        radius: f64,
        start_angle: f64,
        sweep: f64,
    },
    /// Chained cubic Bézier spline: `3n+1` control points in UV
    /// (bezier geometry — every 3 points a new cubic span,
    /// C0-joined at the shared anchors). Exact as a single clamped degree-3
    /// B-spline with interior knots of multiplicity 3 at the span joins.
    Bezier { controls: Vec<[f64; 2]> },
    /// Half of a full ellipse: the exact rational-quadratic arc from `+major`
    /// to `−major` through `+minor` (`upper == false`) or back through
    /// `−minor` (`upper == true`) about `center`. `major`/`minor` are the UV
    /// half-axis VECTORS (`a·û` / `b·v̂`) — the ellipse is the affine image of
    /// the unit circle under `(cu, cv) ↦ center + cu·major + cv·minor`.
    EllipseHalf {
        center: [f64; 2],
        major: [f64; 2],
        minor: [f64; 2],
        upper: bool,
    },
}

impl Segment {
    /// The 3D curve for this segment placed in `frame`, oriented `a`→`b` when
    /// `reversed` is false, else `b`→`a`.
    pub(super) fn to_curve(&self, frame: &Frame, reversed: bool) -> Result<NurbsCurve, String> {
        let natural = match &self.kind {
            SegKind::Line => make_line(
                frame.to_3d(self.a[0], self.a[1]),
                frame.to_3d(self.b[0], self.b[1]),
            )?,
            SegKind::Arc {
                center,
                radius,
                start_angle,
                sweep,
            } => make_arc(
                frame.to_3d(center[0], center[1]),
                frame.x_axis,
                frame.y_axis,
                *radius,
                *start_angle,
                start_angle + sweep,
            )?,
            SegKind::Bezier { controls } => bezier_chain_curve(controls, frame)?,
            SegKind::EllipseHalf {
                center,
                major,
                minor,
                upper,
            } => ellipse_half_curve(*center, *major, *minor, *upper, frame)?,
        };
        if reversed {
            natural.reversed()
        } else {
            Ok(natural)
        }
    }
}

/// One clamped degree-3 B-spline through a chain of cubic Bézier spans
/// (`3n+1` control points): interior knots at the span joins with
/// multiplicity 3 make each span EXACTLY the source cubic. Span k occupies
/// parameter [k, k+1].
fn bezier_chain_curve(controls: &[[f64; 2]], frame: &Frame) -> Result<NurbsCurve, String> {
    let span_count = (controls.len() - 1) / 3;
    if span_count == 0 || controls.len() != span_count * 3 + 1 {
        return Err(format!(
            "sketch: bezier needs 3n+1 control points, got {}",
            controls.len()
        ));
    }
    let mut knots = vec![0.0; 4];
    for join in 1..span_count {
        knots.extend_from_slice(&[join as f64; 3]);
    }
    knots.extend_from_slice(&[span_count as f64; 4]);
    let points = controls
        .iter()
        .map(|p| Vec4::from_point(frame.to_3d(p[0], p[1]), 1.0))
        .collect();
    NurbsCurve::new(3, knots, points)
}

/// One half of a full ellipse as an EXACT rational-quadratic NURBS: the affine
/// image (unit circle → ellipse) of the standard 9-control-point rational full
/// circle's half (`make_arc`'s representation — two 90° spans, corner weights
/// √2/2, knots `[0,0,0,½,½,1,1,1]`). Only the control POINTS pass through the
/// affine map `(cu, cv) ↦ center + cu·major + cv·minor` (then the plane frame);
/// the weights are invariant under affine maps, so the curve is the exact
/// ellipse.
fn ellipse_half_curve(
    center: [f64; 2],
    major: [f64; 2],
    minor: [f64; 2],
    upper: bool,
    frame: &Frame,
) -> Result<NurbsCurve, String> {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    // (unit-circle coords, weight): lower half sweeps θ ∈ [0, π], upper [π, 2π].
    let unit: [([f64; 2], f64); 5] = if upper {
        [
            ([-1.0, 0.0], 1.0),
            ([-1.0, -1.0], w),
            ([0.0, -1.0], 1.0),
            ([1.0, -1.0], w),
            ([1.0, 0.0], 1.0),
        ]
    } else {
        [
            ([1.0, 0.0], 1.0),
            ([1.0, 1.0], w),
            ([0.0, 1.0], 1.0),
            ([-1.0, 1.0], w),
            ([-1.0, 0.0], 1.0),
        ]
    };
    let points = unit
        .iter()
        .map(|&([cu, cv], weight)| {
            let u = center[0] + cu * major[0] + cv * minor[0];
            let v = center[1] + cu * major[1] + cv * minor[1];
            Vec4::from_point(frame.to_3d(u, v), weight)
        })
        .collect();
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0], points)
}

/// A point on the ellipse `center + cos(θ)·major + sin(θ)·minor` (UV).
pub(super) fn ellipse_point(center: [f64; 2], major: [f64; 2], minor: [f64; 2], theta: f64) -> [f64; 2] {
    [
        center[0] + theta.cos() * major[0] + theta.sin() * minor[0],
        center[1] + theta.cos() * major[1] + theta.sin() * minor[1],
    ]
}

/// A fully closed loop = its directed segments in traversal order.
pub(super) struct Loop {
    /// `(segment, reversed)` in head-to-tail order.
    pub(super) entries: Vec<(Segment, bool)>,
    /// Sampled UV polygon, for outer-vs-hole containment classification.
    pub(super) polygon: Vec<[f64; 2]>,
    /// The loop's STABLE identity, resolved by [`super::loop_ids::resolve`] after
    /// assembly. `None` only for a loop whose geometry carries no numeric id at
    /// all (a hand-built test profile).
    pub(super) loop_id: Option<u64>,
}

/// A whole circle as a self-contained loop of TWO semicircle arcs — a profile
/// consumer (`extrude_profile_brep`) needs ≥ 2 curves, so a single closed circle
/// curve won't do. Starts at angle 0 (`center + (r,0)`) CCW, matching legacy:930-935.
/// Both arcs carry the circle's single source edge name.
pub(super) fn circle_loop(
    center: [f64; 2],
    radius: f64,
    name: Option<String>,
    ids: SegmentIds,
) -> Loop {
    let point_at = |angle: f64| [center[0] + radius * angle.cos(), center[1] + radius * angle.sin()];
    let lower = Segment {
        a: point_at(0.0),
        b: point_at(PI),
        kind: SegKind::Arc {
            center,
            radius,
            start_angle: 0.0,
            sweep: PI,
        },
        name: name.clone(),
        ids,
    };
    let upper = Segment {
        a: point_at(PI),
        b: point_at(TAU),
        kind: SegKind::Arc {
            center,
            radius,
            start_angle: PI,
            sweep: PI,
        },
        name,
        ids,
    };
    let polygon = sample_circle_polygon(center, radius);
    Loop {
        entries: vec![(lower, false), (upper, false)],
        polygon,
        loop_id: None,
    }
}

/// A whole ellipse as a self-contained loop of TWO half-ellipse curves — the
/// circle convention ([`circle_loop`]): a profile consumer needs ≥ 2 curves.
/// Starts at `center + major` (θ = 0) CCW; both halves carry the ellipse's
/// single source edge name. `major`/`minor` are the UV half-axis vectors.
pub(super) fn ellipse_loop(
    center: [f64; 2],
    major: [f64; 2],
    minor: [f64; 2],
    name: Option<String>,
    ids: SegmentIds,
) -> Loop {
    let lower = Segment {
        a: ellipse_point(center, major, minor, 0.0),
        b: ellipse_point(center, major, minor, PI),
        kind: SegKind::EllipseHalf {
            center,
            major,
            minor,
            upper: false,
        },
        name: name.clone(),
        ids,
    };
    let upper = Segment {
        a: ellipse_point(center, major, minor, PI),
        b: ellipse_point(center, major, minor, TAU),
        kind: SegKind::EllipseHalf {
            center,
            major,
            minor,
            upper: true,
        },
        name,
        ids,
    };
    let polygon = sample_ellipse_polygon(center, major, minor);
    Loop {
        entries: vec![(lower, false), (upper, false)],
        polygon,
        loop_id: None,
    }
}

/// Sampled UV polygon of a whole ellipse for containment classification —
/// the [`sample_circle_polygon`] convention (16 uniform-angle samples).
fn sample_ellipse_polygon(center: [f64; 2], major: [f64; 2], minor: [f64; 2]) -> Vec<[f64; 2]> {
    (0..16)
        .map(|step| ellipse_point(center, major, minor, TAU * (step as f64 / 16.0)))
        .collect()
}

/// Uniform-parameter polyline over a chained cubic Bézier (`3n+1` controls),
/// `per_span` interior steps per span, endpoints included.
pub(super) fn sample_bezier_polyline(controls: &[[f64; 2]], per_span: usize) -> Vec<[f64; 2]> {
    let span_count = (controls.len().saturating_sub(1)) / 3;
    let mut points = Vec::with_capacity(span_count * per_span + 1);
    for span in 0..span_count {
        let p0 = controls[span * 3];
        let p1 = controls[span * 3 + 1];
        let p2 = controls[span * 3 + 2];
        let p3 = controls[span * 3 + 3];
        let start = if span == 0 { 0 } else { 1 };
        for step in start..=per_span {
            let t = step as f64 / per_span as f64;
            let mt = 1.0 - t;
            let x = mt * mt * mt * p0[0]
                + 3.0 * mt * mt * t * p1[0]
                + 3.0 * mt * t * t * p2[0]
                + t * t * t * p3[0];
            let y = mt * mt * mt * p0[1]
                + 3.0 * mt * mt * t * p1[1]
                + 3.0 * mt * t * t * p2[1]
                + t * t * t * p3[1];
            points.push([x, y]);
        }
    }
    if points.is_empty() {
        points.push(controls.first().copied().unwrap_or([0.0, 0.0]));
    } else {
        points.insert(0, controls[0]);
    }
    points
}

fn sample_circle_polygon(center: [f64; 2], radius: f64) -> Vec<[f64; 2]> {
    (0..16)
        .map(|step| {
            let angle = TAU * (step as f64 / 16.0);
            [
                center[0] + radius * angle.cos(),
                center[1] + radius * angle.sin(),
            ]
        })
        .collect()
}
