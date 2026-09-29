//! Six pole-free coordinate charts for a spherical carrier — the cube atlas.
//!
//! A sphere in this kernel is ONE exact rational-NURBS surface of revolution
//! over a polar `(azimuth, latitude)` domain.  That domain is not a chart in the
//! differential-geometry sense: it is singular at the two poles (a whole
//! parameter line collapses to one point, and `Su × Sv` vanishes there) and it
//! is periodic in `u`, so a region straddling the seam is not a connected set of
//! parameters at all.  Every trimming, splitting and tessellation decision taken
//! in that domain inherits both defects, and the kernel carries a long tail of
//! special cases — pole caps, seam bands, seam walls, unwrapped covers — that
//! exist only to work around them.
//!
//! This module supplies the alternative: an ATLAS of six regular charts, one per
//! cube face, that between them cover the sphere with no singular point and no
//! periodic identification.  Chart `k` maps the square `(s, t) ∈ [-1, 1]²` to
//!
//! ```text
//!     p(s, t) = centre + radius · normalize(n_k + s·e_k + t·f_k)
//! ```
//!
//! with `(e_k, f_k, n_k)` a right-handed orthonormal frame — the central
//! projection of a cube face onto its circumscribed sphere.  The map is a
//! diffeomorphism on the closed square, its Jacobian never degenerates, and it
//! is exact: every image point lies on the sphere to the accuracy of one
//! `normalize`, so NOTHING about the carrier's geometry is approximated by using
//! it.  The map does contain a square root, so it is deliberately NOT offered as
//! a NURBS patch — the exact rational sphere surface remains the geometry of
//! record and the charts are a computational device layered over it.
//!
//! # What the atlas is aligned to
//!
//! The cube basis is the sphere's own recognition frame, so `basis[2]` is the
//! polar axis and the two degenerate poles land at the CENTRES of the `±z`
//! charts.  A former pole is then an ordinary interior point of a regular chart:
//! a cut through it is no more special than a cut anywhere else.  Because the
//! basis is read off the surface, two call sites looking at the same surface
//! always build the same atlas — a prerequisite for the shared-sample rules
//! below.
//!
//! # Sharing, and why it is the whole problem
//!
//! Charts are internal.  They must mint no face, no edge, no name, and above all
//! no crack.  The watertight tessellator's guarantee is that every edge is
//! sampled ONCE and both adjacent faces consume the same positions; an
//! artificial chart boundary has to inherit exactly that guarantee.  Three rules
//! do it, and they are the reason this module owns the sample generation rather
//! than leaving it to each consumer:
//!
//! 1. **A cube edge is subdivided once.**  [`cube_edge_parameters`] returns the
//!    subdivision of a cube edge in the edge's OWN parameter `q`, and both
//!    charts that meet along that edge read the same list through
//!    [`ChartSide::edge`].  Neither chart may add a point of its own.
//! 2. **A corner belongs to no chart.**  The eight cube corners are named by
//!    [`corner_index`], and the three charts and three cube edges meeting at one
//!    all address it by that name, so ownership is never ambiguous.
//! 3. **A crossing is computed once.**  Where a trim polyline leaves one chart,
//!    [`arc_chart_crossings`] returns the crossing as a function of the segment
//!    alone — never of which chart is asking — so the two charts that share it
//!    get the identical point, and the same value can be pushed back into the
//!    SHARED edge-sample table so the neighbouring face gets it too.
//!
//! # Classification
//!
//! [`SphericalRegion`] decides inside/outside for a trimmed spherical face
//! without reference to any chart, by the signed solid angle each trim loop
//! subtends at the query point (Van Oosterom & Strackee's formula).  It is
//! pole-free and seam-free by construction: a slit — the seam traversed once
//! each way, or a collapsed pole loop — cancels exactly, and needs no special
//! case.  Trimming and triangulation share this one classifier so they cannot
//! disagree about where the material is.

use crate::{AnalyticSurface, NurbsSurface, Vec3};

/// Charts in the atlas: one per cube face.
pub const CHART_COUNT: usize = 6;
/// Cube edges, each shared by exactly two charts.
pub const CUBE_EDGE_COUNT: usize = 12;
/// Cube corners, each shared by exactly three charts and three cube edges.
pub const CUBE_CORNER_COUNT: usize = 8;

/// How far along a segment a chart crossing must be to count as one, as a
/// fraction of the segment.  Anything nearer an end is that end.
const ENDPOINT_LAMBDA: f64 = 1e-9;

/// Relative slack used when asking whether a direction lies in a chart or on a
/// cube edge.  Chart membership is a comparison of coordinates that are exactly
/// equal on a boundary, so the slack only has to absorb rounding.
const CHART_EPSILON: f64 = 1e-12;

/// One cube-face chart: `direction(s, t) = normal + s·tangent_s + t·tangent_t`,
/// with `tangent_s × tangent_t = normal`, so `∂p/∂s × ∂p/∂t` points OUT of the
/// sphere everywhere on the chart.
#[derive(Clone, Copy, Debug)]
pub struct Chart {
    pub normal: Vec3,
    pub tangent_s: Vec3,
    pub tangent_t: Vec3,
    /// Basis axis the chart faces along, `0..3`.
    pub axis: usize,
    /// `+1` or `-1`: which end of that axis.
    pub sign: f64,
}

/// Which side of a chart square a cube edge is: the chart's other coordinate
/// runs along the edge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChartSide {
    /// `s = +1`
    SPlus,
    /// `s = -1`
    SMinus,
    /// `t = +1`
    TPlus,
    /// `t = -1`
    TMinus,
}

impl ChartSide {
    pub const ALL: [ChartSide; 4] = [
        ChartSide::SPlus,
        ChartSide::SMinus,
        ChartSide::TPlus,
        ChartSide::TMinus,
    ];

    /// The cube edge this side of chart `chart` lies on, and the sign relating
    /// the chart's free coordinate `f` to the edge parameter: `q = sign · f`.
    ///
    /// This is the single place the two charts meeting along a cube edge agree
    /// about its identity and its direction, so a subdivision computed once in
    /// `q` reaches both of them unchanged.
    pub fn edge(self, chart: usize) -> (usize, f64) {
        let axis = chart / 2;
        let sign = if chart % 2 == 0 { 1.0 } else { -1.0 };
        let b = (axis + 1) % 3;
        let c = (axis + 2) % 3;
        match self {
            // Varying axis c; fixed axes in canonical order (c+1)%3 = axis,
            // (c+2)%3 = b.
            ChartSide::SPlus => (cube_edge_index(c, sign, sign), 1.0),
            ChartSide::SMinus => (cube_edge_index(c, sign, -sign), 1.0),
            // Varying axis b; fixed axes in canonical order (b+1)%3 = c,
            // (b+2)%3 = axis.  The chart's s maps to the edge parameter with
            // the chart's own sign.
            ChartSide::TPlus => (cube_edge_index(b, 1.0, sign), sign),
            ChartSide::TMinus => (cube_edge_index(b, -1.0, sign), sign),
        }
    }

    /// `(s, t)` of the point at free coordinate `f` on this side.
    pub fn coords(self, f: f64) -> (f64, f64) {
        match self {
            ChartSide::SPlus => (1.0, f),
            ChartSide::SMinus => (-1.0, f),
            ChartSide::TPlus => (f, 1.0),
            ChartSide::TMinus => (f, -1.0),
        }
    }
}

/// Canonical index of the cube edge whose VARYING axis is `varying` and whose
/// two other axes `(varying+1) % 3` and `(varying+2) % 3` are pinned to the
/// given signs.  Twelve edges: three varying axes times four sign pairs.
pub fn cube_edge_index(varying: usize, sign_first: f64, sign_second: f64) -> usize {
    varying * 4 + usize::from(sign_first > 0.0) * 2 + usize::from(sign_second > 0.0)
}

/// Inverse of [`cube_edge_index`]: `(varying axis, sign of (v+1)%3, sign of (v+2)%3)`.
pub fn cube_edge_parts(index: usize) -> (usize, f64, f64) {
    let varying = index / 4;
    let rest = index % 4;
    (
        varying,
        if rest & 2 != 0 { 1.0 } else { -1.0 },
        if rest & 1 != 0 { 1.0 } else { -1.0 },
    )
}

/// Canonical index of the cube corner with the given per-axis signs.
pub fn corner_index(signs: [f64; 3]) -> usize {
    usize::from(signs[0] > 0.0) | (usize::from(signs[1] > 0.0) << 1) | (usize::from(signs[2] > 0.0) << 2)
}

/// The corner at the `q = +1` (`positive_end`) or `q = -1` end of a cube edge.
pub fn cube_edge_corner(edge: usize, positive_end: bool) -> usize {
    let (varying, first, second) = cube_edge_parts(edge);
    let mut signs = [0.0; 3];
    signs[varying] = if positive_end { 1.0 } else { -1.0 };
    signs[(varying + 1) % 3] = first;
    signs[(varying + 2) % 3] = second;
    corner_index(signs)
}

/// The shared subdivision of EVERY cube edge, in the edge parameter `q ∈ [-1, 1]`.
///
/// Uniform in ARC ANGLE rather than in `q`: a cube edge's direction at `q` makes
/// an angle `atan(q/√2)` with the edge's midpoint, so equal angle steps give
/// equal chord sag, which is what a chord tolerance actually asks for.  The list
/// is symmetric, strictly increasing, and pinned to `±1` at the ends (the two
/// corners), and it is a pure function of `divisions` — which is why both charts
/// meeting along the edge can compute it independently and still agree
/// bit-for-bit.
pub fn cube_edge_parameters(divisions: usize) -> Vec<f64> {
    let divisions = divisions.max(1);
    let half = std::f64::consts::SQRT_2.recip().atan();
    (0..=divisions)
        .map(|index| {
            if index == 0 {
                -1.0
            } else if index == divisions {
                1.0
            } else {
                let angle = -half + 2.0 * half * index as f64 / divisions as f64;
                std::f64::consts::SQRT_2 * angle.tan()
            }
        })
        .collect()
}

/// The interior grid of a chart square, in the chart coordinate `s` (or `t`).
///
/// A chart spans 90° each way and its coordinate is the TANGENT of the angle, so
/// equal angle steps — the thing a chord tolerance actually constrains — are
/// `tan` of a uniform division, not a uniform division.  Interior points are free
/// (no other chart reads them), so this list is independent of
/// [`cube_edge_parameters`]; sizing both from the same angular step is what keeps
/// the triangles the same size on either side of a chart boundary.
pub fn chart_grid_parameters(divisions: usize) -> Vec<f64> {
    let divisions = divisions.max(2);
    let half = std::f64::consts::FRAC_PI_4;
    (1..divisions)
        .map(|index| (-half + 2.0 * half * index as f64 / divisions as f64).tan())
        .collect()
}

/// The angular step a chord tolerance allows on a sphere of `radius`: a chord
/// subtending `θ` sags `radius·(1 − cos(θ/2))`.
fn angular_step(radius: f64, chord_tolerance: f64) -> Option<f64> {
    if !(chord_tolerance > 0.0) || !(radius > 0.0) {
        return None;
    }
    let ratio = 1.0 - (chord_tolerance / radius).min(1.0);
    let step = 2.0 * ratio.clamp(-1.0, 1.0).acos();
    (step > 0.0 && step.is_finite()).then_some(step)
}

/// Divisions across a chart square (90°) at `chord_tolerance`.
///
/// Sized against the CELL DIAGONAL, not the cell side: a Delaunay triangle over a
/// square grid has the diagonal as its longest edge, and it is that chord whose
/// sag the tolerance is about.  Sizing by the side leaves every triangle a factor
/// of two over tolerance, so the refinement pass fires on all of them and rebuilds
/// the grid it was meant to correct — three times the triangles and a crop of
/// slivers for the same accuracy.
pub fn chart_grid_divisions(radius: f64, chord_tolerance: f64) -> usize {
    let divisions = match angular_step(radius, chord_tolerance) {
        Some(step) => (((std::f64::consts::FRAC_PI_2 * std::f64::consts::SQRT_2) / step).ceil()
            as usize)
            .clamp(2, 96),
        None => 16,
    };
    // EVEN, so `0` is a grid value and the chart's CENTRE is a mesh vertex.
    //
    // The six chart centres are the sphere's extreme points along its own basis —
    // two of them are the former poles. The polar domain always had a vertex at a
    // pole, and a mesh that stops short of one is visibly flat there and fails any
    // check on the solid's extent. At a fine tolerance the shortfall is invisible;
    // at a coarse one (a 40 mm ball at 1 mm chord: five divisions) the nearest
    // grid point sits 12.6 degrees off the pole, a whole millimetre short.
    divisions + divisions % 2
}

/// Divisions per cube edge that hold the chord sag of a `radius` sphere under
/// `chord_tolerance`.  A chord subtending `θ` sags `radius·(1 − cos(θ/2))`, so
/// `θ = 2·acos(1 − tolerance/radius)`; the edge arc is `2·atan(1/√2)` long.
pub fn cube_edge_divisions(radius: f64, chord_tolerance: f64) -> usize {
    let arc = 2.0 * std::f64::consts::SQRT_2.recip().atan();
    match angular_step(radius, chord_tolerance) {
        // The same diagonal-aware step [`chart_grid_divisions`] uses. A cube edge
        // is a 1-D chain and would meet the tolerance at the plain step, but a
        // boundary sampled coarser than the interior it abuts leaves a row of
        // thin triangles along every chart edge — matching the two steps is what
        // makes a chart boundary invisible in the mesh as well as in the topology.
        Some(step) => (((arc * std::f64::consts::SQRT_2) / step).ceil() as usize).clamp(2, 80),
        None => 14,
    }
}

/// A sphere's pole-free cube atlas: centre, radius, and the right-handed basis
/// the six charts are built on.
#[derive(Clone, Copy, Debug)]
pub struct SphereAtlas {
    pub centre: Vec3,
    pub radius: f64,
    /// `basis[2]` is the polar axis, so the two degenerate poles of the stored
    /// polar domain sit at the centres of charts 4 and 5.
    pub basis: [Vec3; 3],
}

impl SphereAtlas {
    /// The atlas of a surface that IS a sphere, however it is parameterized.
    ///
    /// Routed through [`AnalyticSurface::sphere_frame`], so a REFLECTED sphere —
    /// which recognizes as a general `Revolution`, not as `Sphere` — gets an
    /// atlas exactly as a direct one does.  Matching on the `Sphere` variant
    /// here would silently drop every mirrored ball.
    pub fn of_surface(surface: &NurbsSurface) -> Option<Self> {
        Self::of_analytic(surface.analytic()?)
    }

    pub fn of_analytic(analytic: &AnalyticSurface) -> Option<Self> {
        let (centre, radius, basis) = analytic.sphere_frame()?;
        (radius > 0.0).then_some(Self {
            centre,
            radius,
            basis,
        })
    }

    /// Chart `index`: `axis = index / 2`, facing `+` for even and `-` for odd.
    ///
    /// The tangents are picked so `tangent_s × tangent_t = normal` for all six,
    /// which makes every chart's `∂p/∂s × ∂p/∂t` point out of the sphere — the
    /// atlas has ONE orientation, so a triangle wound counter-clockwise in any
    /// chart faces outward in every other.
    pub fn chart(&self, index: usize) -> Chart {
        let axis = index / 2;
        let sign = if index % 2 == 0 { 1.0 } else { -1.0 };
        let b = (axis + 1) % 3;
        let c = (axis + 2) % 3;
        Chart {
            normal: self.basis[axis].scale(sign),
            tangent_s: self.basis[b].scale(sign),
            tangent_t: self.basis[c],
            axis,
            sign,
        }
    }

    /// Unnormalized chart direction — the point of the cube face itself.
    pub fn direction(&self, chart: usize, s: f64, t: f64) -> Vec3 {
        let chart = self.chart(chart);
        chart
            .normal
            .add(chart.tangent_s.scale(s))
            .add(chart.tangent_t.scale(t))
    }

    /// The sphere point at chart coordinates `(s, t)`.  Exact: the result lies on
    /// the sphere to the accuracy of one `normalize`.
    pub fn point(&self, chart: usize, s: f64, t: f64) -> Result<Vec3, String> {
        let direction = self.direction(chart, s, t).normalized()?;
        Ok(self.centre.add(direction.scale(self.radius)))
    }

    /// Outward unit normal at chart coordinates `(s, t)` — for a sphere the
    /// normal IS the radial direction, so no derivative is ever needed and there
    /// is no pole at which one degenerates.
    pub fn normal_at(&self, chart: usize, s: f64, t: f64) -> Result<Vec3, String> {
        self.direction(chart, s, t).normalized()
    }

    /// Whether the surface's OWN `Su x Sv` points out of the sphere, sampled at
    /// the middle of its parameter domain — for a polar sphere the equator, as
    /// far from either degenerate pole as the domain allows.
    ///
    /// The trim convention (material to the LEFT of the directed boundary) is
    /// stated against this normal, not against the face's sense, and a reflected
    /// sphere has it pointing inward — so no consumer may assume it.
    pub fn parameterization_is_outward(&self, surface: &NurbsSurface) -> Result<bool, String> {
        let [u0, u1] = surface.domain_u()?;
        let [v0, v1] = surface.domain_v()?;
        let (point, du, dv) = surface.deriv1(0.5 * (u0 + u1), 0.5 * (v0 + v1))?;
        Ok(du.cross(dv).dot(point.sub(self.centre)) > 0.0)
    }

    /// Basis coordinates of `point` relative to the centre.
    pub fn axis_coordinates(&self, point: Vec3) -> [f64; 3] {
        let d = point.sub(self.centre);
        [
            d.dot(self.basis[0]),
            d.dot(self.basis[1]),
            d.dot(self.basis[2]),
        ]
    }

    /// The chart a point belongs to when only one answer is wanted: the chart
    /// whose face the point projects furthest onto, lowest index breaking a tie.
    /// Deterministic, so it can be used as a key.
    pub fn locate(&self, point: Vec3) -> usize {
        let x = self.axis_coordinates(point);
        let mut best = 0usize;
        let mut best_value = f64::NEG_INFINITY;
        for chart in 0..CHART_COUNT {
            let axis = chart / 2;
            let sign = if chart % 2 == 0 { 1.0 } else { -1.0 };
            let value = sign * x[axis];
            if value > best_value {
                best_value = value;
                best = chart;
            }
        }
        best
    }

    /// Chart coordinates of `point` in `chart`, or `None` when the point is on
    /// the far side of the sphere from it.  A point outside the chart's square
    /// still returns coordinates (with `|s| > 1` or `|t| > 1`); use
    /// [`Self::contains`] to ask about membership.
    pub fn coordinates(&self, chart: usize, point: Vec3) -> Option<(f64, f64)> {
        let x = self.axis_coordinates(point);
        let chart = self.chart(chart);
        let b = (chart.axis + 1) % 3;
        let c = (chart.axis + 2) % 3;
        let w = chart.sign * x[chart.axis];
        if !(w > 0.0) {
            return None;
        }
        Some((chart.sign * x[b] / w, x[c] / w))
    }

    /// Whether `point` lies in `chart`'s closed square, within a relative slack.
    pub fn contains(&self, chart: usize, point: Vec3) -> bool {
        match self.coordinates(chart, point) {
            Some((s, t)) => {
                let slack = 1.0 + CHART_EPSILON;
                s.abs() <= slack && t.abs() <= slack
            }
            None => false,
        }
    }

    /// Every chart whose closed square contains `point`: one in a chart
    /// interior, two on a cube edge, three at a corner.  Ascending index, so the
    /// answer is order-independent.
    pub fn charts_containing(&self, point: Vec3) -> Vec<usize> {
        (0..CHART_COUNT)
            .filter(|chart| self.contains(*chart, point))
            .collect()
    }

    /// The cube edge `point` lies on, with its edge parameter `q`, or `None`
    /// when the point is in a chart interior.
    ///
    /// A point sits on a cube edge exactly when the two LARGEST of its three
    /// basis coordinates are equal in magnitude; `q` is the third coordinate
    /// scaled so those two are `±1`.  Returning `q` — rather than either chart's
    /// own coordinate — is what lets a crossing be filed against the edge once
    /// and read back by both charts.
    pub fn edge_of(&self, point: Vec3, relative_tolerance: f64) -> Option<(usize, f64)> {
        self.edges_containing(point, relative_tolerance)
            .into_iter()
            .next()
    }

    /// EVERY cube edge `point` lies on: one along an edge, three at a corner,
    /// none in a chart interior.
    ///
    /// Callers that have to decide whether two points share an edge must use
    /// this rather than [`Self::edge_of`]: a corner lies on three edges at once,
    /// and picking one of them canonically would make a segment running INTO a
    /// corner look as if it left the edge it is on.
    pub fn edges_containing(&self, point: Vec3, relative_tolerance: f64) -> Vec<(usize, f64)> {
        let x = self.axis_coordinates(point);
        let scale = x[0].abs().max(x[1].abs()).max(x[2].abs());
        if !(scale > 0.0) {
            return Vec::new();
        }
        let tolerance = relative_tolerance.max(CHART_EPSILON) * scale;
        let mut found = Vec::new();
        // The varying axis is the one whose magnitude is NOT tied for largest.
        for varying in 0..3 {
            let b = (varying + 1) % 3;
            let c = (varying + 2) % 3;
            let pinned = x[b].abs().min(x[c].abs());
            if (x[b].abs() - x[c].abs()).abs() <= tolerance
                && pinned >= x[varying].abs() - tolerance
                && pinned > 0.0
            {
                let magnitude = 0.5 * (x[b].abs() + x[c].abs());
                found.push((
                    cube_edge_index(varying, x[b], x[c]),
                    (x[varying] / magnitude).clamp(-1.0, 1.0),
                ));
            }
        }
        found
    }

    /// The sphere point on cube edge `edge` at edge parameter `q`.
    pub fn edge_point(&self, edge: usize, q: f64) -> Result<Vec3, String> {
        let (varying, first, second) = cube_edge_parts(edge);
        let direction = self.basis[varying]
            .scale(q)
            .add(self.basis[(varying + 1) % 3].scale(first))
            .add(self.basis[(varying + 2) % 3].scale(second))
            .normalized()?;
        Ok(self.centre.add(direction.scale(self.radius)))
    }

    /// The parameters `λ ∈ (0, 1)` at which the great-circle arc from `start` to
    /// `end` crosses a chart boundary, ascending.
    ///
    /// A chart boundary is one of the six planes `|x_i| = |x_j|` through the
    /// centre, so the crossing is the root of a function that is LINEAR along
    /// the chord — no Newton iteration, no seeding, and nothing that could
    /// converge onto an extrapolated surface.  `λ` interpolates the chord
    /// `(1−λ)·d_start + λ·d_end`; the crossing point is that direction
    /// normalized back onto the sphere, which is exactly where the arc meets the
    /// plane.
    ///
    /// The result depends only on the two endpoints, never on which chart is
    /// asking, so both charts sharing the crossing derive the identical point.
    /// A plane crossed AWAY from the cube edge it carries (the other coordinate
    /// is larger there) is not a chart boundary at that point and is dropped.
    pub fn arc_chart_crossings(&self, start: Vec3, end: Vec3) -> Vec<f64> {
        let a = self.axis_coordinates(start);
        let b = self.axis_coordinates(end);
        let scale = (0..3)
            .map(|k| a[k].abs().max(b[k].abs()))
            .fold(0.0, f64::max);
        if !(scale > 0.0) {
            return Vec::new();
        }
        // A segment that LIES IN one of the six planes evaluates that plane's
        // function as rounding noise, and the sign of noise flips at random. Every
        // flip brackets a "root", and each one splits the segment at a point no
        // neighbouring face holds — a T-junction, on the very trim that runs along
        // a chart boundary. A crossing has to be transversal, so a bracket counts
        // only when the function is meaningfully non-zero at an end.
        let significant = 1e-9 * scale;
        let mut crossings = Vec::new();
        for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
            for combination in [1.0f64, -1.0] {
                let g0 = a[i] - combination * a[j];
                let g1 = b[i] - combination * b[j];
                if (g0 > 0.0) == (g1 > 0.0) || g0 == g1 {
                    continue;
                }
                if g0.abs().max(g1.abs()) <= significant {
                    continue;
                }
                let lambda = g0 / (g0 - g1);
                // A crossing AT an endpoint is not a crossing: the endpoint is
                // already a vertex, and splitting there mints a second one a few
                // ulps away that no neighbouring face holds. This is the common
                // case, not a corner one — `sample_all_edges` has already put a
                // sample on the boundary, so the segments either side of it each
                // report a root at their shared end.
                if !(lambda > ENDPOINT_LAMBDA && lambda < 1.0 - ENDPOINT_LAMBDA) {
                    continue;
                }
                let at = |k: usize| a[k] + (b[k] - a[k]) * lambda;
                let tied = at(i).abs().max(at(j).abs());
                let third = at(3 - i - j).abs();
                // Only a crossing where the tied pair is the LARGEST pair is on
                // a cube edge; elsewhere the plane runs through a chart's
                // interior and means nothing.
                if tied >= third - CHART_EPSILON * scale && tied > 0.0 {
                    crossings.push(lambda);
                }
            }
        }
        crossings.sort_by(f64::total_cmp);
        crossings.dedup_by(|x, y| (*x - *y).abs() <= 1e-12);
        crossings
    }

    /// Split a closed polyline on the sphere so that no segment crosses a chart
    /// boundary, returning `(point, chart)` for every segment: the vertices of
    /// the segment and the single chart it lies in.
    ///
    /// Crossing points are inserted at the exact positions
    /// [`Self::arc_chart_crossings`] reports, so a vertex on a cube edge belongs
    /// to both adjacent charts with identical coordinates.
    pub fn split_polyline(&self, points: &[Vec3]) -> Result<Vec<ChartSegment>, String> {
        let mut segments = Vec::new();
        for index in 0..points.len() {
            segments.extend(self.split_segment(points[index], points[(index + 1) % points.len()])?);
        }
        Ok(segments)
    }

    /// [`Self::split_polyline`] for ONE segment.  A caller holding individual
    /// boundary segments must use this: handing a two-point slice to the polyline
    /// form would close it and emit the segment twice, once each way.
    pub fn split_segment(&self, start: Vec3, end: Vec3) -> Result<Vec<ChartSegment>, String> {
        let mut segments = Vec::new();
        {
            if start.sub(end).length() <= 0.0 {
                return Ok(segments);
            }
            let crossings = self.arc_chart_crossings(start, end);
            let da = self.axis_coordinates(start);
            let db = self.axis_coordinates(end);
            let mut cursor = start;
            let mut previous = 0.0f64;
            for lambda in crossings.iter().copied().chain(std::iter::once(1.0)) {
                let next = if lambda >= 1.0 {
                    end
                } else {
                    let direction = self.basis[0]
                        .scale(da[0] + (db[0] - da[0]) * lambda)
                        .add(self.basis[1].scale(da[1] + (db[1] - da[1]) * lambda))
                        .add(self.basis[2].scale(da[2] + (db[2] - da[2]) * lambda))
                        .normalized()?;
                    self.centre.add(direction.scale(self.radius))
                };
                let final_piece = lambda >= 1.0;
                // A crossing that landed on the far endpoint adds nothing but a
                // duplicate vertex; the final piece reaches that endpoint anyway.
                let distinct = next.sub(cursor).length() > 0.0
                    && (final_piece || next.sub(end).length() > 0.0);
                if distinct {
                    let middle = 0.5 * (previous + lambda);
                    let probe = self.basis[0]
                        .scale(da[0] + (db[0] - da[0]) * middle)
                        .add(self.basis[1].scale(da[1] + (db[1] - da[1]) * middle))
                        .add(self.basis[2].scale(da[2] + (db[2] - da[2]) * middle));
                    segments.push(ChartSegment {
                        start: cursor,
                        end: next,
                        chart: self.locate(self.centre.add(probe)),
                    });
                }
                cursor = next;
                previous = lambda;
            }
        }
        Ok(segments)
    }
}

/// One piece of a trim polyline, already confined to a single chart.
#[derive(Clone, Copy, Debug)]
pub struct ChartSegment {
    pub start: Vec3,
    pub end: Vec3,
    pub chart: usize,
}

/// Whether the minor great-circle arcs `a0→a1` and `b0→b1` cross, for unit
/// directions.
///
/// The two arcs' plane normals are NORMALIZED before they are crossed, so the
/// magnitude of the result is the sine of the angle between the planes — a
/// meaningful quantity with a meaningful threshold. Crossing the raw normals
/// instead makes that magnitude scale with both arcs' lengths, so a short probe
/// against a short boundary sample yields a direction that is mostly rounding
/// noise, and the crossing decision becomes a coin toss. Parity cannot survive
/// that: one miscounted crossing inverts the answer for a whole region.
///
/// The arcs are half-open at their end, so a crossing exactly at a shared vertex
/// of a polyline is counted once rather than twice.
///
/// Both normals arrive precomputed: `na` once per LEG (it depends only on the
/// path, not on the arc it is tested against) and `nb` once per REGION (it is a
/// property of the boundary arc). A trim query counts crossings over every arc of
/// the region six times over, so computing either here put two square roots per
/// arc per query in the innermost loop of the boolean.
fn arcs_cross(a0: Vec3, a1: Vec3, na: Vec3, arc: &Arc) -> bool {
    let (b0, b1, nb) = (arc.start, arc.end, arc.normal);
    if nb.dot(nb) <= 0.0 {
        // Ends that name no plane — antipodal or coincident. Not a crossing.
        return false;
    }
    // Straddle test first, and it settles most arcs: any crossing point lies on
    // BOTH great circles, so it satisfies `na · p == 0`. An arc whose two ends are
    // strictly on one side of the leg's plane contains no such point, and the same
    // holds for the leg against the arc's plane. Equality is left to the full test
    // below — an end exactly ON the other plane is a crossing the half-open
    // convention still has to adjudicate.
    let straddles = |n: Vec3, x0: Vec3, x1: Vec3| {
        let (d0, d1) = (n.dot(x0), n.dot(x1));
        !((d0 > 0.0 && d1 > 0.0) || (d0 < 0.0 && d1 < 0.0))
    };
    if !straddles(na, b0, b1) || !straddles(nb, a0, a1) {
        return false;
    }
    let line = na.cross(nb);
    if line.length() <= 1e-9 {
        // The two great circles coincide, or meet at too shallow an angle to
        // locate: an overlap is not a transversal crossing and must not flip
        // parity.
        return false;
    }
    let Ok(unit) = line.normalized() else {
        return false;
    };
    let within = |p: Vec3, x0: Vec3, x1: Vec3, n: Vec3| -> bool {
        x0.cross(p).dot(n) >= 0.0 && p.cross(x1).dot(n) > 0.0
    };
    [unit, unit.scale(-1.0)]
        .into_iter()
        .any(|p| within(p, a0, a1, na) && within(p, b0, b1, nb))
}

/// One directed boundary arc, with the great-circle plane it lies in.
#[derive(Clone, Copy, Debug)]
struct Arc {
    start: Vec3,
    end: Vec3,
    /// The unit normal of `start × end`, or zero when the ends name no plane.
    /// Precomputed: every crossing count in every query walks every arc.
    normal: Vec3,
}

/// The trim of a spherical face, as directed boundary arcs on the unit sphere,
/// ready to classify points without touching any chart or parameter domain.
///
/// Containment is decided by CROSSING PARITY against a seed the region carries:
/// one point placed just to the material side of one boundary arc.  The parity of
/// the crossings between the seed and a query decides the query, and both the
/// seed and the test are ordinary great-circle geometry — no parameter domain, no
/// pole, no seam, and no reliance on a signed-area formula whose branch collapses
/// when the query sits opposite a small loop (which is exactly the near-tangent
/// pocket, and exactly where it was wrong).
///
/// The seed encodes the BREP convention in the one place it belongs: material
/// lies to the LEFT of the directed boundary as seen from the FACE normal — the
/// face's own normal, which for a cavity wall points into the sphere, not out of
/// it.
#[derive(Clone, Debug, Default)]
pub struct SphericalRegion {
    /// Directed boundary arcs as unit directions from the sphere centre, with
    /// slits already cancelled.
    arcs: Vec<Arc>,
    /// A unit direction known to be material, or `None` when there is no
    /// boundary at all (the whole sphere) or none usable.
    seed: Option<Vec3>,
}

impl SphericalRegion {
    /// Build the region from the face's trim loops given as closed 3D polylines.
    ///
    /// `outward_face_normal` says whether the FACE's normal points out of the
    /// sphere — `same_sense == parameterization_is_outward`, never `same_sense`
    /// alone and never the surface normal alone.
    pub fn new(centre: Vec3, polylines: &[Vec<Vec3>], outward_face_normal: bool) -> Self {
        let mut segments = Vec::new();
        for polyline in polylines {
            for index in 0..polyline.len() {
                segments.push((polyline[index], polyline[(index + 1) % polyline.len()]));
            }
        }
        Self::from_segments(centre, &segments, outward_face_normal)
    }

    /// As [`Self::new`], from the directed boundary segments themselves.
    ///
    /// A consumer that has one loop joining TWO rims through the seam — a ball
    /// drilled through — MUST use this: concatenating that loop's surviving
    /// coedges and closing the ring bridges rim to rim with two chords that are
    /// not reverses of each other, and the region that comes out is not the one
    /// the face means.
    pub fn from_segments(centre: Vec3, segments: &[(Vec3, Vec3)], outward_face_normal: bool) -> Self {
        use std::collections::HashMap;
        type Key = [u64; 3];
        let key = |p: Vec3| -> Key { [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()] };
        let mut counts: HashMap<(Key, Key), usize> = HashMap::new();
        let mut directed: Vec<(Vec3, Vec3)> = Vec::new();
        for &(a, b) in segments {
            if key(a) == key(b) {
                // A collapsed pole loop: every sample is the same point.
                continue;
            }
            *counts.entry((key(a), key(b))).or_insert(0) += 1;
            directed.push((a, b));
        }
        // Cancel each directed segment against a matching reverse. The seam of a
        // trimmed ball is used once each way by the SAME face, so both uses go;
        // what is left is the material boundary and nothing else.
        let mut budget: HashMap<(Key, Key), usize> = HashMap::new();
        for (&(from, to), &forward) in &counts {
            let backward = counts.get(&(to, from)).copied().unwrap_or(0);
            budget.insert((from, to), forward.saturating_sub(backward.min(forward)));
        }
        let unit = |p: Vec3| p.sub(centre).normalized().ok();
        let mut arcs: Vec<Arc> = Vec::new();
        for (a, b) in directed {
            let entry = budget.entry((key(a), key(b))).or_insert(0);
            if *entry == 0 {
                continue;
            }
            *entry -= 1;
            if let (Some(a), Some(b)) = (unit(a), unit(b)) {
                if a.sub(b).length() > 0.0 {
                    arcs.push(Arc {
                        start: a,
                        end: b,
                        normal: a.cross(b).normalized().unwrap_or_default(),
                    });
                }
            }
        }
        let seed = Self::seed_from(&arcs, outward_face_normal);
        Self { arcs, seed }
    }

    /// A unit direction just to the MATERIAL side of the longest boundary arc.
    ///
    /// The offset is bisected until stepping the same distance to the other side
    /// crosses the boundary exactly once: that is the proof the seed cleared the
    /// arc it came from and reached no further boundary, so it is inside the
    /// material and not merely near it. A region thinner than the first offset is
    /// found by the bisection rather than mis-seeded.
    fn seed_from(arcs: &[Arc], outward_face_normal: bool) -> Option<Vec3> {
        let longest = arcs.iter().copied().max_by(|x, y| {
            x.start
                .sub(x.end)
                .length()
                .total_cmp(&y.start.sub(y.end).length())
        })?;
        let (a, b) = (longest.start, longest.end);
        let middle = a.add(b).normalized().ok()?;
        let along = b.sub(a);
        let tangent = along.sub(middle.scale(along.dot(middle)));
        let normal = if outward_face_normal {
            middle
        } else {
            middle.scale(-1.0)
        };
        let left = normal.cross(tangent).normalized().ok()?;
        // Start well inside the arc's own sampling step, so the common case
        // settles on the first try: this runs once per region build and a region
        // is built once per trim query, in the innermost loop of the boolean.
        let mut step = 0.1 * a.sub(b).length().max(1e-9);
        for _ in 0..24 {
            let inside = middle.add(left.scale(step)).normalized().ok()?;
            let outside = middle.sub(left.scale(step)).normalized().ok()?;
            if Self::crossings(arcs, inside, outside) == 1 {
                return Some(inside);
            }
            step *= 0.5;
        }
        None
    }

    fn crossings(arcs: &[Arc], from: Vec3, to: Vec3) -> usize {
        // The leg's own plane, once for the whole walk: a leg that names no plane
        // crosses nothing, which is what testing each arc against it used to
        // conclude one arc at a time.
        let Ok(leg) = from.cross(to).normalized() else {
            return 0;
        };
        arcs.iter()
            .filter(|arc| arcs_cross(from, to, leg, arc))
            .count()
    }

    /// Crossing parity along a TWO-LEG path `from → via → to`.
    ///
    /// One straight leg is not enough. Two nearly antipodal endpoints do not name
    /// a great circle at all — their cross product is noise — and a single leg
    /// that grazes a boundary vertex can count a crossing twice or not at all.
    /// Routing through a via point that is far from both endpoints makes each leg
    /// well conditioned, and taking three different via points and a majority
    /// makes a single grazed vertex harmless.
    fn parity(arcs: &[Arc], from: Vec3, to: Vec3) -> bool {
        let mut even = 0usize;
        let mut odd = 0usize;
        for index in 0..3 {
            let Some(via) = Self::via_point(from, to, index) else {
                continue;
            };
            let count = Self::crossings(arcs, from, via) + Self::crossings(arcs, via, to);
            if count % 2 == 0 {
                even += 1;
            } else {
                odd += 1;
            }
        }
        even >= odd
    }

    /// A unit direction well away from both endpoints, deterministic in `index`.
    fn via_point(from: Vec3, to: Vec3, index: usize) -> Option<Vec3> {
        let axis = from.cross(to);
        let base = match axis.normalized() {
            Ok(unit) => unit,
            // Antipodal or coincident: any direction off the pair will do.
            Err(_) => from.perpendicular().ok()?,
        };
        let other = base.cross(from).normalized().ok()?;
        let angle = std::f64::consts::TAU * index as f64 / 3.0;
        base.scale(angle.cos())
            .add(other.scale(angle.sin()))
            .normalized()
            .ok()
    }

    /// Whether the face has no material boundary at all — an untrimmed ball,
    /// whose only "loops" were the seam slit and the two collapsed poles.
    pub fn is_whole_sphere(&self) -> bool {
        self.arcs.is_empty()
    }

    /// Whether this region can answer at all: either it has no boundary, or the
    /// seed search cleared one.
    ///
    /// A region with arcs but no seed must make its CALLER decline — answering
    /// anyway would report every point material, which in the mesh is silent
    /// double coverage and in the trim query is a face that swallowed the ball.
    /// A classifier that cannot decide has to say so.
    pub fn is_decidable(&self) -> bool {
        self.arcs.is_empty() || self.seed.is_some()
    }

    /// Whether the sphere point `point` is material.  Pole-free and seam-free:
    /// the query never enters a parameter domain.
    pub fn contains(&self, centre: Vec3, point: Vec3) -> bool {
        let Ok(probe) = point.sub(centre).normalized() else {
            return false;
        };
        let Some(seed) = self.seed else {
            // No boundary at all: the face is the whole ball. A region that HAS a
            // boundary but no seed is not decidable and the caller must have
            // declined already ([`Self::is_decidable`]).
            return true;
        };
        Self::parity(&self.arcs, seed, probe)
    }

    /// How many boundary arcs separate `point` from the region's seed.  A caller
    /// letting one probe speak for a whole flood region prefers the probe whose
    /// count is smallest — it is the one furthest from a grazing arc.
    pub fn separation(&self, centre: Vec3, point: Vec3) -> usize {
        let Ok(probe) = point.sub(centre).normalized() else {
            return usize::MAX;
        };
        match self.seed {
            Some(seed) => Self::via_point(seed, probe, 0)
                .map(|via| {
                    Self::crossings(&self.arcs, seed, via) + Self::crossings(&self.arcs, via, probe)
                })
                .unwrap_or(usize::MAX),
            None => 0,
        }
    }
}

/// Collapse points that are the same point to ONE value.
///
/// Trim polylines assembled from per-coedge evaluations name a shared vertex
/// twice — once from each side — and the two evaluations agree only to rounding.
/// [`SphericalRegion`] cancels a slit by matching a segment against its exact
/// reverse, which those two nearly-equal values would defeat, so a consumer that
/// did not read its points from one shared table runs them through here first.
/// The tolerance is relative to the point cloud's own extent.
pub fn canonicalize_points(points: &mut [Vec3], relative_tolerance: f64) {
    let scale = points
        .iter()
        .map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs()))
        .fold(0.0, f64::max)
        .max(1.0);
    let cell = (relative_tolerance * scale).max(f64::MIN_POSITIVE);
    let mut table: std::collections::HashMap<[i64; 3], Vec3> = std::collections::HashMap::new();
    for point in points.iter_mut() {
        let base = [
            (point.x / cell).round() as i64,
            (point.y / cell).round() as i64,
            (point.z / cell).round() as i64,
        ];
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let probe = [base[0] + dx, base[1] + dy, base[2] + dz];
                    if let Some(&existing) = table.get(&probe) {
                        if existing.sub(*point).length() <= cell {
                            found = Some(existing);
                            break 'search;
                        }
                    }
                }
            }
        }
        match found {
            Some(existing) => *point = existing,
            None => {
                table.insert(base, *point);
            }
        }
    }
}

