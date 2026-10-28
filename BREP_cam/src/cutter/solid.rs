//! The tool solid: ONE representation of the physical tool for preview,
//! material-removal simulation and holder collision.
//!
//! A [`ToolProfile`] is the tool's radius-versus-height polyline from the tip
//! (height 0) up through the cutting profile, the shank and the holder, each
//! vertex tagged with the part it belongs to. Everything else derives from it:
//! [`ToolProfile::radius_at_height`] is the collision envelope,
//! [`ToolProfile::mesh`] revolves it into a closed triangle mesh, and
//! [`ToolProfile::swept_mesh`] builds the volume swept along a linear move.
//! Meshes are plain arrays (no BREP topology) and build identically on every
//! target; the same inputs give the same bytes.

use super::{Cutter, CutterProfile, SegmentKind};
use crate::model::{Bounds3, Holder, Point3};
use serde::{Deserialize, Serialize};

/// Which part of the tool a profile point or triangle belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ToolPart {
    Cutting,
    Shank,
    Holder,
}

/// One vertex of the radius-versus-height polyline.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProfilePoint {
    pub radius: f64,
    pub height: f64,
    pub part: ToolPart,
}

/// The tool as a revolved polyline, tip at the origin, axis +Z.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolProfile {
    /// Heights never decrease from the first point (the tip, radius 0 for
    /// pointed tools or the flat bottom's edge) to the last (the top).
    pub points: Vec<ProfilePoint>,
}

/// Default chordal deviation of a revolved arc in model units.
pub const DEFAULT_CHORD: f64 = 0.01;
/// Default number of segments around the axis.
pub const DEFAULT_RADIAL_SEGMENTS: usize = 32;

impl ToolProfile {
    /// Profile of `cutter` (plus `holder`, stacked on top of the shank), arcs
    /// sampled so no chord deviates more than `chord` from the true profile.
    pub fn new(cutter: &Cutter, holder: Option<&Holder>, chord: f64) -> Self {
        let chord = if chord.is_finite() && chord > 0.0 { chord } else { DEFAULT_CHORD };
        let mut points: Vec<ProfilePoint> = Vec::new();
        // A repeated position re-tags the point with the later part, so the
        // band above it (attributed to its lower point) belongs to that part.
        let mut push = |radius: f64, height: f64, part: ToolPart| {
            let p = ProfilePoint { radius: radius.max(0.0), height, part };
            match points.last_mut() {
                Some(last) if last.radius == p.radius && last.height == p.height => last.part = part,
                _ => points.push(p),
            }
        };
        for seg in cutter.segments() {
            let [r0, r1] = seg.radius_range;
            let [h0, h1] = seg.height_range;
            match seg.kind {
                SegmentKind::Flat | SegmentKind::Cone | SegmentKind::Cylinder | SegmentKind::ConeInverted => {
                    push(r0, h0, ToolPart::Cutting);
                    push(r1, h1, ToolPart::Cutting);
                }
                SegmentKind::Sphere | SegmentKind::Torus | SegmentKind::SphereUpper => {
                    // Arc of radius rho about (ring_radius, z_offset) from the
                    // angle at r0 to the angle at r1, measured from straight down;
                    // an upper arc continues past the equator (angle > 90°).
                    let rho = seg.profile_radius;
                    let center_r = if seg.kind == SegmentKind::Torus { seg.ring_radius } else { 0.0 };
                    let upper = seg.kind == SegmentKind::SphereUpper;
                    let angle_at = |r: f64| {
                        let a = ((r - center_r) / rho).clamp(-1.0, 1.0).asin();
                        if upper { std::f64::consts::PI - a } else { a }
                    };
                    let (a0, a1) = (angle_at(r0), angle_at(r1));
                    let steps = arc_steps(rho, (a1 - a0).abs(), chord);
                    push(r0, h0, ToolPart::Cutting);
                    for i in 1..steps {
                        let a = a0 + (a1 - a0) * i as f64 / steps as f64;
                        push(center_r + rho * a.sin(), seg.z_offset - rho * a.cos(), ToolPart::Cutting);
                    }
                    push(r1, h1, ToolPart::Cutting);
                }
            }
        }
        let cutting_top = cutter.cutting_length();
        let shank_r = cutter.shaft_radius();
        let shank_top = cutting_top + cutter.shaft_length();
        if cutter.shaft_length() > 0.0 {
            push(shank_r, cutting_top, ToolPart::Shank);
            push(shank_r, shank_top, ToolPart::Shank);
        }
        if let Some(holder) = holder {
            let mut base = shank_top;
            for seg in &holder.segments {
                push(0.5 * seg.lower_diameter, base, ToolPart::Holder);
                base += seg.height;
                push(0.5 * seg.upper_diameter, base, ToolPart::Holder);
            }
        }
        Self { points }
    }

    pub fn top_height(&self) -> f64 {
        self.points.last().map_or(0.0, |p| p.height)
    }

    pub fn max_radius(&self) -> f64 {
        self.points.iter().map(|p| p.radius).fold(0.0, f64::max)
    }

    /// The largest radius the tool (cutting part, shank and holder) occupies at
    /// height `h`: the collision envelope. `None` outside `[0, top_height]`.
    pub fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !h.is_finite() || h < 0.0 || h > self.top_height() || self.points.is_empty() {
            return None;
        }
        let mut best: Option<f64> = None;
        for w in self.points.windows(2) {
            let (a, b) = (w[0], w[1]);
            if h < a.height.min(b.height) || h > a.height.max(b.height) {
                continue;
            }
            let r = if (b.height - a.height).abs() <= f64::EPSILON * (1.0 + h.abs()) {
                a.radius.max(b.radius)
            } else {
                a.radius + (b.radius - a.radius) * (h - a.height) / (b.height - a.height)
            };
            best = Some(best.map_or(r, |x: f64| x.max(r)));
        }
        best
    }

    /// Height bands `[h0, h1]` (local, tip = 0) where the tool's radius is at
    /// least `r`, each with the part owning it, in ascending order and merged
    /// where adjacent bands share a part. A column at distance `r` from the
    /// axis is touched exactly on these bands.
    pub fn height_bands_reaching(&self, r: f64) -> Vec<([f64; 2], ToolPart)> {
        let mut out: Vec<([f64; 2], ToolPart)> = Vec::new();
        let mut push = |band: [f64; 2], part: ToolPart| {
            if band[1] < band[0] {
                return;
            }
            match out.last_mut() {
                Some((last, lp)) if *lp == part && last[1] >= band[0] => last[1] = last[1].max(band[1]),
                _ => out.push((band, part)),
            }
        };
        for w in self.points.windows(2) {
            let (a, b) = (w[0], w[1]);
            if b.height < a.height {
                continue;
            }
            let (ra, rb) = (a.radius, b.radius);
            if ra < r && rb < r {
                continue;
            }
            if b.height == a.height {
                // A horizontal step: the step's disc touches at exactly this height.
                if ra.max(rb) >= r {
                    push([a.height, a.height], a.part);
                }
                continue;
            }
            // Radius is linear in height along the segment: solve radius >= r.
            let (mut h0, mut h1) = (a.height, b.height);
            if (rb - ra).abs() > 0.0 {
                let h_cross = a.height + (r - ra) * (b.height - a.height) / (rb - ra);
                if ra < r {
                    h0 = h_cross.max(a.height);
                } else if rb < r {
                    h1 = h_cross.min(b.height);
                }
            }
            push([h0, h1], a.part);
        }
        out
    }

    /// Local bounds of the tool solid.
    pub fn bounds(&self) -> Bounds3 {
        let r = self.max_radius();
        Bounds3::new(Point3::new(-r, -r, 0.0), Point3::new(r, r, self.top_height()))
    }

    /// Revolve the profile into a closed triangle mesh with `radial_segments`
    /// around the axis (at least 3), tip at `origin`.
    pub fn mesh(&self, radial_segments: usize) -> ToolMesh {
        self.swept_mesh(Point3::ZERO, Point3::ZERO, radial_segments)
    }

    /// One-sided radii of the polyline at height `h`: the largest radius just
    /// below `h` and just above it (they differ at a step such as a holder
    /// collar). `None` where the polyline has no point at or around `h`.
    fn sided_radii(&self, h: f64) -> Option<(f64, f64)> {
        let mut below: Option<f64> = None;
        let mut above: Option<f64> = None;
        let bump = |slot: &mut Option<f64>, r: f64| *slot = Some(slot.map_or(r, |x: f64| x.max(r)));
        for w in self.points.windows(2) {
            let (a, b) = (w[0], w[1]);
            let (lo, hi) = (a.height.min(b.height), a.height.max(b.height));
            if h < lo || h > hi {
                continue;
            }
            if hi > lo {
                let r = a.radius + (b.radius - a.radius) * (h - a.height) / (b.height - a.height);
                if h > lo {
                    bump(&mut below, r);
                }
                if h < hi {
                    bump(&mut above, r);
                }
                if h == lo {
                    bump(&mut above, r);
                }
                if h == hi {
                    bump(&mut below, r);
                }
            } else {
                // A horizontal step at exactly h joins the part below (ending at
                // a) to the part above (starting at b).
                bump(&mut below, a.radius);
                bump(&mut above, b.radius);
            }
        }
        if let Some(first) = self.points.first() {
            if h == first.height {
                bump(&mut above, first.radius);
                bump(&mut below, first.radius);
            }
        }
        if let Some(last) = self.points.last() {
            if h == last.height {
                bump(&mut below, last.radius);
                bump(&mut above, last.radius);
            }
        }
        match (below, above) {
            (None, None) => None,
            (b, a) => Some((b.or(a).unwrap(), a.or(b).unwrap())),
        }
    }

    /// Part owning the polyline at height `h` (the highest-indexed point at or
    /// below `h`), so an envelope keeps its preview colours.
    pub fn part_at(&self, h: f64) -> ToolPart {
        self.points.iter().rev().find(|p| p.height <= h).or(self.points.first()).map_or(ToolPart::Cutting, |p| p.part)
    }

    /// The vertical sweep of this profile over `[0, dz]`: the upper envelope
    /// of the profile and its copy lifted by `dz`, as a polyline with the
    /// steps of both. Exactly the solid swept by an axial move of length `dz`.
    pub fn vertical_envelope(&self, dz: f64) -> ToolProfile {
        if !(dz > 0.0) || !dz.is_finite() || self.points.is_empty() {
            return self.clone();
        }
        let mut heights: Vec<f64> = self.points.iter().flat_map(|p| [p.height, p.height + dz]).collect();
        heights.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        heights.dedup();
        let mut out: Vec<ProfilePoint> = Vec::new();
        let mut push = |radius: f64, height: f64, part: ToolPart| {
            let p = ProfilePoint { radius, height, part };
            match out.last_mut() {
                Some(last) if last.radius == p.radius && last.height == p.height => last.part = part,
                _ => out.push(p),
            }
        };
        for h in heights {
            let orig = self.sided_radii(h);
            let lifted = self.sided_radii(h - dz);
            let pick = |o: Option<f64>, l: Option<f64>| match (o, l) {
                (Some(a), Some(b)) => Some(a.max(b)),
                (a, b) => a.or(b),
            };
            let below = pick(orig.map(|r| r.0), lifted.map(|r| r.0));
            let above = pick(orig.map(|r| r.1), lifted.map(|r| r.1));
            let part = if orig.is_some() { self.part_at(h) } else { self.part_at(h - dz) };
            if let Some(b) = below {
                push(b, h, part);
            }
            if let Some(a) = above {
                push(a, h, part);
            }
        }
        ToolProfile { points: out }
    }

    /// The volume swept by the tool moving linearly from `start` to `end`
    /// (tool reference points), as a closed mesh.
    ///
    /// Construction: the profile's [`vertical_envelope`](Self::vertical_envelope)
    /// over the move's vertical extent, placed at the lower of the two heights,
    /// swept horizontally between the two XY positions; each ring becomes a
    /// stadium (back half circle at the start side, front half circle at the
    /// end side, two straight edges). This is EXACT for horizontal moves, for
    /// axial moves (plunges, retracts) and for a zero-length move (the tool
    /// itself), and CONSERVATIVE (a superset) for moves with both a horizontal
    /// and a vertical component. Exact sloped removal needs a per-column floor
    /// evaluation, not a mesh.
    pub fn swept_mesh(&self, start: Point3, end: Point3, radial_segments: usize) -> ToolMesh {
        let n = radial_segments.max(3);
        let d = end.sub(start);
        let dz = d.z.abs();
        let envelope = self.vertical_envelope(dz);
        let z0 = start.z.min(end.z);
        let a = Point3::new(start.x, start.y, z0);
        let b = Point3::new(end.x, end.y, z0);
        let dxy = Point3::new(d.x, d.y, 0.0);
        // Unit direction of travel in XY; an axial or zero move uses +X.
        let dir = dxy.normalized().unwrap_or(Point3::X);
        let moving = dxy.length() > 0.0;
        let half = (n / 2).max(1);
        let ring_len = if moving { 2 * (half + 1) } else { n };
        let mut mesh = ToolMesh::default();
        let mut rings: Vec<(u32, &ProfilePoint)> = Vec::with_capacity(envelope.points.len());
        for p in &envelope.points {
            let base = mesh.positions.len() as u32;
            mesh.push_ring(a, b, dir, moving, p.radius, p.height, n, half);
            rings.push((base, p));
        }
        let Some((first_base, first)) = rings.first().copied() else {
            return mesh;
        };
        // Bottom cap: the lowest ring closed inward (a point ring is skipped by
        // the degenerate filter, so a pointed tool is closed by the next band).
        if first.radius > 0.0 {
            mesh.cap(first_base, ring_len, first.part, true);
        }
        // Bands between consecutive rings, attributed to the lower point's part.
        for w in rings.windows(2) {
            let ((ra, pa), (rb, _)) = (w[0], w[1]);
            mesh.band(ra, rb, ring_len, pa.part);
        }
        if let Some((last_base, last)) = rings.last().copied() {
            if last.radius > 0.0 {
                mesh.cap(last_base, ring_len, last.part, false);
            }
        }
        mesh.finish_groups();
        mesh
    }
}

/// Number of arc steps so the chord deviation stays within `chord`.
fn arc_steps(rho: f64, sweep: f64, chord: f64) -> usize {
    if !(rho > 0.0) || !(sweep > 0.0) {
        return 1;
    }
    let max_step = 2.0 * (1.0 - (chord / rho).min(1.0)).acos();
    if !(max_step > 0.0) {
        return 1;
    }
    ((sweep / max_step).ceil() as usize).clamp(1, 4096)
}

/// A triangle group of one tool part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolMeshGroup {
    pub part: ToolPart,
    /// Triangle index range `[start, end)` into `indices / 3`.
    pub start: u32,
    pub end: u32,
}

/// A plain closed triangle mesh of the tool, outward-wound.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolMesh {
    pub positions: Vec<Point3>,
    /// Three vertex indices per triangle, counter-clockwise seen from outside.
    pub indices: Vec<u32>,
    pub groups: Vec<ToolMeshGroup>,
    #[serde(skip)]
    pending: Vec<(ToolPart, u32)>,
}

impl ToolMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn triangle(&self, i: usize) -> [Point3; 3] {
        let t = &self.indices[3 * i..3 * i + 3];
        [self.positions[t[0] as usize], self.positions[t[1] as usize], self.positions[t[2] as usize]]
    }

    /// Signed volume by the divergence theorem; positive for outward winding.
    pub fn signed_volume(&self) -> f64 {
        (0..self.triangle_count())
            .map(|i| {
                let [a, b, c] = self.triangle(i);
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    pub fn bounds(&self) -> Bounds3 {
        Bounds3::from_points(self.positions.iter().copied())
    }

    /// Every geometric edge is used exactly twice, once in each direction.
    /// Vertices are welded by exact position first, because rings of a pointed
    /// tip repeat the apex and a still ring's seam repeats no vertex.
    pub fn is_closed(&self) -> bool {
        let mut keyed: Vec<([u64; 3], u32)> = self
            .positions
            .iter()
            .enumerate()
            .map(|(i, p)| ([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()], i as u32))
            .collect();
        keyed.sort_unstable();
        let mut canon = vec![0u32; self.positions.len()];
        let mut i = 0;
        while i < keyed.len() {
            let mut j = i;
            while j < keyed.len() && keyed[j].0 == keyed[i].0 {
                canon[keyed[j].1 as usize] = keyed[i].1;
                j += 1;
            }
            i = j;
        }
        let mut edges: Vec<(u32, u32)> = Vec::with_capacity(self.indices.len());
        for t in self.indices.chunks_exact(3) {
            let (a, b, c) = (canon[t[0] as usize], canon[t[1] as usize], canon[t[2] as usize]);
            edges.push((a, b));
            edges.push((b, c));
            edges.push((c, a));
        }
        edges.sort_unstable();
        if edges.windows(2).any(|w| w[0] == w[1]) {
            return false;
        }
        edges.iter().all(|&(a, b)| edges.binary_search(&(b, a)).is_ok())
    }

    /// A ring of the profile point `(radius, height)`. A still tool gets `n`
    /// points around the axis at `start`; a moving tool gets the stadium: the
    /// back half circle at `start` (n/2 + 1 points), then the front half circle at
    /// `end` (n/2 + 1 points), all 2·(n/2 + 1) points counter-clockwise seen from above.
    fn push_ring(&mut self, start: Point3, end: Point3, dir: Point3, moving: bool, radius: f64, height: f64, n: usize, half: usize) {
        use std::f64::consts::PI;
        let side = Point3::new(-dir.y, dir.x, 0.0);
        let at = |c: Point3, ang: f64| {
            c.add(dir.scale(radius * ang.cos())).add(side.scale(radius * ang.sin())).add(Point3::new(0.0, 0.0, height))
        };
        if !moving {
            for i in 0..n {
                self.positions.push(at(start, 2.0 * PI * i as f64 / n as f64));
            }
            return;
        }
        // Back half at start: angles from +90° (left side) through 180° to 270° (right side).
        for i in 0..=half {
            self.positions.push(at(start, 0.5 * PI + PI * i as f64 / half as f64));
        }
        // Front half at end: from 270° (right) through 0° to 90° (left).
        for i in 0..=half {
            self.positions.push(at(end, 1.5 * PI + PI * i as f64 / half as f64));
        }
    }

    fn push_triangle(&mut self, part: ToolPart, a: u32, b: u32, c: u32) {
        if a == b || b == c || a == c {
            return;
        }
        let [pa, pb, pc] = [self.positions[a as usize], self.positions[b as usize], self.positions[c as usize]];
        if pb.sub(pa).cross(pc.sub(pa)).length_squared() <= 0.0 {
            return;
        }
        self.indices.extend([a, b, c]);
        self.pending.push((part, (self.indices.len() / 3 - 1) as u32));
    }

    /// Quads between ring `a` (lower) and ring `b` (upper), wound outward.
    fn band(&mut self, a: u32, b: u32, len: usize, part: ToolPart) {
        let len = len as u32;
        for i in 0..len {
            let j = (i + 1) % len;
            self.push_triangle(part, a + i, a + j, b + j);
            self.push_triangle(part, a + i, b + j, b + i);
        }
    }

    /// Fan over ring `base`; `down` caps the bottom (normal −Z), else the top.
    fn cap(&mut self, base: u32, len: usize, part: ToolPart, down: bool) {
        let center = self.positions.len() as u32;
        let mut c = Point3::ZERO;
        for i in 0..len {
            c = c.add(self.positions[(base + i as u32) as usize]);
        }
        self.positions.push(c.scale(1.0 / len as f64));
        let len = len as u32;
        for i in 0..len {
            let j = (i + 1) % len;
            if down {
                self.push_triangle(part, center, base + j, base + i);
            } else {
                self.push_triangle(part, center, base + i, base + j);
            }
        }
    }

    /// Collapse the per-triangle part tags into contiguous groups.
    fn finish_groups(&mut self) {
        let mut groups: Vec<ToolMeshGroup> = Vec::new();
        for &(part, index) in &self.pending {
            match groups.last_mut() {
                Some(g) if g.part == part && g.end == index => g.end = index + 1,
                _ => groups.push(ToolMeshGroup { part, start: index, end: index + 1 }),
            }
        }
        self.groups = groups;
        self.pending.clear();
    }

    /// Group for triangle `i`, if tagged.
    pub fn part_of(&self, i: u32) -> Option<ToolPart> {
        self.groups.iter().find(|g| g.start <= i && i < g.end).map(|g| g.part)
    }
}
