//! Constant-Z cutter sweep against triangle surfaces. Intervals are normalized
//! along a finite fiber. The full lower profile and the finite shank participate.
//! Numerical solves are bounded and deterministic; no elapsed-time budgets.
use crate::{Bounds3, CamError, Contact, ContactType, Cutter, CutterProfile, MeshSnapshot,
    Point3, ProjectionMode, Runner, Tolerance, TriangleIndex};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction { X, Y }

#[derive(Clone, Debug, PartialEq)]
pub struct Interval {
    pub lower: f64,
    pub upper: f64,
    pub lower_contact: Contact,
    pub upper_contact: Contact,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Fiber {
    pub direction: Direction,
    pub start: Point3,
    pub end: Point3,
    pub intervals: Vec<Interval>,
}

impl Fiber {
    pub fn new(direction: Direction, start: Point3, end: Point3) -> Self {
        Self { direction, start, end, intervals: Vec::new() }
    }
    pub fn coordinate(&self) -> f64 {
        match self.direction { Direction::X => self.start.y, Direction::Y => self.start.x }
    }
}

/// A generation borrows an immutable, validated C0 snapshot. The index, when
/// supplied, must belong to this exact snapshot, as for all TriangleIndex users.
pub struct PushContext<'a> {
    pub mesh: &'a MeshSnapshot,
    pub cutter: &'a Cutter,
    pub index: Option<&'a dyn TriangleIndex>,
    pub tolerance: Tolerance,
    /// Additional radial stock allowance (already cutter-location compensation).
    pub allowance: f64,
}

fn invalid(message: &str) -> CamError { CamError::InvalidInput(message.into()) }

impl PushContext<'_> {
    pub fn validate(&self) -> Result<(), CamError> {
        self.cutter.validate()?;
        self.tolerance.validate()?;
        if !self.allowance.is_finite() || self.allowance < 0.0 { return Err(invalid("push allowance must be finite and non-negative")); }
        if self.mesh.triangles.iter().any(|t| t.face as usize >= self.mesh.faces.len() || !t.bounds.is_finite() || !t.normal.is_finite() || t.vertices().iter().any(|p| !p.is_finite())) {
            return Err(invalid("push mesh contains invalid coordinates or face references"));
        }
        Ok(())
    }

    pub fn fiber(&self, input: &Fiber, runner: &mut Runner<'_>) -> Result<Fiber, CamError> {
        self.validate()?;
        let d = input.end.sub(input.start);
        let axial = match input.direction { Direction::X => d.y == 0.0 && d.x > 0.0, Direction::Y => d.x == 0.0 && d.y > 0.0 };
        if !axial || d.z != 0.0 || !input.start.is_finite() || !input.end.is_finite() {
            return Err(invalid("fiber must have finite increasing axis coordinates and constant Z"));
        }
        let intervals = self.segment(input.start, input.end, runner)?;
        Ok(Fiber { intervals, ..input.clone() })
    }

    /// Surface collision intervals along an arbitrary horizontal segment. This
    /// is also used to check every complete contour chord, not only endpoints.
    pub fn segment(&self, start: Point3, end: Point3, runner: &mut Runner<'_>) -> Result<Vec<Interval>, CamError> {
        runner.token().check()?;
        self.validate()?;
        if !start.is_finite() || !end.is_finite() || start.z != end.z { return Err(invalid("sweep requires a finite horizontal segment")); }
        let length = start.distance_xy(end);
        if length <= self.tolerance.distance_eps { return Err(invalid("sweep segment is too short")); }
        let u = end.sub(start).scale(1.0 / length);
        let v = Point3::new(-u.y, u.x, 0.0);
        let radius = self.cutter.radius().max(self.cutter.shaft_radius()) + self.allowance;
        let bounds = Bounds3::new(Point3::new(start.x.min(end.x)-radius, start.y.min(end.y)-radius, start.z),
            Point3::new(start.x.max(end.x)+radius, start.y.max(end.y)+radius, start.z+self.cutter.total_height()));
        let mode = if u.y == 0.0 { ProjectionMode::Yz } else if u.x == 0.0 { ProjectionMode::Xz } else { ProjectionMode::Xyz };
        // Winding needs the complete line/solid cross-section, including
        // entry/exit faces beyond a short diagonal segment's cutter envelope.
        // Axis projections already keep that direction unbounded.
        let query_bounds=if mode==ProjectionMode::Xyz {
            Bounds3::new(Point3::new(self.mesh.bounds.min.x,self.mesh.bounds.min.y,start.z),
                Point3::new(self.mesh.bounds.max.x,self.mesh.bounds.max.y,bounds.max.z))
        } else {bounds};
        let candidates = match self.index {
            Some(index) => index.query(&query_bounds, mode),
            None => (0..self.mesh.triangle_count() as u32).collect(),
        };
        let mut result = Vec::new();
        let mut crossings: std::collections::BTreeMap<String, Vec<(f64, i32, u32)>> = std::collections::BTreeMap::new();
        // Pass 1: the solid cross-sections along the line (cheap), giving the
        // stretches where the line runs through material.
        let mut solids: Vec<(u32, [Point3; 3])> = Vec::with_capacity(candidates.len());
        for (n, id) in candidates.iter().copied().enumerate() {
            runner.work(1, "push-triangles", n as u64, candidates.len() as u64)?;
            let t = self.mesh.triangles.get(id as usize).ok_or_else(|| invalid("push index contains an invalid triangle id"))?;
            if !mode.overlaps(&query_bounds, &t.bounds) { continue; }
            let local = t.vertices().map(|p| { let d = p.sub(start); Point3::new(d.dot(u), d.dot(v), d.z) });
            // Fill solid cross-sections as well as surface contact bands. Group by
            // body so overlapping solids cannot cancel one another's winding.
            let face = &self.mesh.faces[t.face as usize];
            let cross = section_above(local, 0.0);
            for pair in cross.windows(2) {
                let (a,b) = (pair[0],pair[1]);
                if (a.y <= 0.0 && b.y > 0.0) || (b.y <= 0.0 && a.y > 0.0) {
                    let x = a.x+(b.x-a.x)*(-a.y)/(b.y-a.y);
                    let normal = t.normal.dot(u);
                    if normal != 0.0 { crossings.entry(face.solid.clone()).or_default().push((x, if normal < 0.0 { 1 } else { -1 }, id)); }
                    break;
                }
            }
            if mode.overlaps(&bounds, &t.bounds) { solids.push((id, local)); }
        }
        let mut material = Vec::new();
        for (_, mut events) in crossings {
            events.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.2.cmp(&b.2)));
            let mut winding = 0i32;
            let mut entry = 0.0;
            let mut entry_id = 0;
            for (x, change, id) in events {
                if winding == 0 { entry=x; entry_id=id; }
                winding += change;
                if winding == 0 && x > entry {
                    let lower=(entry/length).max(0.0); let upper=(x/length).min(1.0);
                    if upper > lower {
                        material.push(Interval { lower, upper,
                            lower_contact: Contact { triangle_id: Some(entry_id), contact_type: ContactType::Facet, ..Contact::default() },
                            upper_contact: Contact { triangle_id: Some(id), contact_type: ContactType::Facet, ..Contact::default() } });
                    }
                }
            }
            if winding != 0 { return Err(invalid("push cross-section is open or inconsistently oriented")); }
        }
        // Pass 2: the cutter's contact band with each triangle in reach. A
        // triangle blocks the line at most over its projection widened by the
        // cutter's largest radius (and the outward padding); when that lies
        // strictly inside a stretch of material it cannot change the merged
        // result (`merge` keeps the outermost endpoints and their contacts),
        // and its solve is skipped.
        // The union so far (material, then each solved band), as disjoint
        // pieces keyed by their lower end. A triangle whose bound lies
        // strictly inside one piece cannot move the final merge. Triangles
        // nearest the line are solved first (they block the widest bands),
        // so the union grows fastest; `merge` sorts, so the order of solving
        // does not change the result.
        let mut union: std::collections::BTreeMap<u64, (f64, f64)> = std::collections::BTreeMap::new();
        let key = |x: f64| x.to_bits() ^ if x.is_sign_negative() { u64::MAX } else { 1u64 << 63 };
        let add = |union: &mut std::collections::BTreeMap<u64, (f64, f64)>, lo: f64, hi: f64| {
            let (mut lo, mut hi) = (lo, hi);
            let overlapping: Vec<u64> = union.range(..=key(hi)).rev().take_while(|(_, p)| p.1 >= lo).map(|(k, _)| *k).collect();
            for k in overlapping {
                let p = union.remove(&k).unwrap();
                lo = lo.min(p.0);
                hi = hi.max(p.1);
            }
            union.insert(key(lo), (lo, hi));
        };
        for m in &material { add(&mut union, m.lower, m.upper); }
        let pad = radius + 2.0 * self.tolerance.distance_eps;
        let mut order: Vec<(f64, usize)> = solids.iter().enumerate().map(|(i, (_, local))| {
            let ymin = local.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
            let ymax = local.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
            (if ymin > 0.0 { ymin } else if ymax < 0.0 { -ymax } else { 0.0 }, i)
        }).collect();
        order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        for (_, i) in order {
            let (id, local) = solids[i];
            let xmin = local.iter().map(|p| p.x).fold(f64::INFINITY, f64::min);
            let xmax = local.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max);
            let (blo, bhi) = ((xmin - pad) / length, (xmax + pad) / length);
            if let Some((_, piece)) = union.range(..key(blo)).next_back() {
                if piece.0 < blo && bhi < piece.1 { continue; }
            }
            let solved = triangle_interval(local, self.cutter, self.allowance, self.tolerance, runner)?;
            if let Some((lo, hi, lower_point, upper_point)) = solved {
                let lo = (lo / length).max(0.0);
                let hi = (hi / length).min(1.0);
                if hi > lo {
                    add(&mut union, lo, hi);
                    let contact = Contact { contact_type: ContactType::NumericFallback, triangle_id: Some(id),
                        detail: Some("bounded convex slice support solve (profile, facet and edge)".into()), ..Contact::default() };
                    let world = |p:Point3| start.add(u.scale(p.x)).add(v.scale(p.y)).add(Point3::new(0.0,0.0,p.z));
                    let mut lower_contact=contact.clone(); lower_contact.point=Some(world(lower_point));
                    let mut upper_contact=contact; upper_contact.point=Some(world(upper_point));
                    result.push(Interval { lower: lo, upper: hi, lower_contact, upper_contact });
                }
            }
        }
        result.extend(material);
        merge(&mut result, self.tolerance.distance_eps / length);
        Ok(result)
    }
}

pub fn merge(intervals: &mut Vec<Interval>, epsilon: f64) {
    intervals.sort_by(|a,b| a.lower.total_cmp(&b.lower).then(a.upper.total_cmp(&b.upper))
        .then(a.lower_contact.triangle_id.cmp(&b.lower_contact.triangle_id)));
    let mut merged: Vec<Interval> = Vec::new();
    for item in intervals.drain(..) {
        if let Some(last) = merged.last_mut() {
            if item.lower <= last.upper + epsilon {
                if item.upper > last.upper { last.upper = item.upper; last.upper_contact = item.upper_contact; }
                continue;
            }
        }
        merged.push(item);
    }
    *intervals = merged;
}

pub fn batch(context: &PushContext<'_>, direction: Direction, fibers: &[Fiber], runner: &mut Runner<'_>) -> Result<Vec<Fiber>, CamError> {
    context.validate()?;
    if fibers.iter().any(|f| f.direction != direction) { return Err(invalid("batch fibers have mixed directions")); }
    runner.stage("push-batch", "Push fiber batch", 0, fibers.len() as u64)?;
    #[cfg(not(feature = "parallel"))]
    let output = {
        let mut output = Vec::with_capacity(fibers.len());
        for (i, fiber) in fibers.iter().enumerate() {
            output.push(context.fiber(fiber, runner)?);
            runner.work(1, "push-fibers", (i+1) as u64, fibers.len() as u64)?;
        }
        output
    };
    // Fibers are independent: solved in parallel, assembled in fiber order,
    // the first error (in fiber order) returned.
    #[cfg(feature = "parallel")]
    let output = crate::par::map_ordered(fibers, runner, "push-fibers", |fiber, worker| context.fiber(fiber, worker))?
        .into_iter()
        .collect::<Result<Vec<Fiber>, CamError>>()?;
    runner.stage("push-complete", "Fiber batch complete", output.len() as u64, fibers.len() as u64)?;
    Ok(output)
}

// Cross-section of a triangle at z. Duplicated vertices are harmless; include
// all three edges at horizontal facets so their filled interior is protected.
fn section(t: [Point3; 3], z: f64) -> Vec<Point3> {
    let mut p = Vec::new();
    for (a,b) in [(t[0],t[1]), (t[1],t[2]), (t[2],t[0])] {
        if a.z == z { p.push(a); }
        if (a.z < z && b.z > z) || (a.z > z && b.z < z) { p.push(a.lerp(b, (z-a.z)/(b.z-a.z))); }
    }
    p
}

// Cross-section of a triangle just above z (a vertex at z counts as below),
// for the winding: a line on a floor, top face or edge at exactly z then sees
// each wall standing on the plane once, and the plane's own facets not at all.
fn section_above(t: [Point3; 3], z: f64) -> Vec<Point3> {
    let mut p = Vec::new();
    for (a,b) in [(t[0],t[1]), (t[1],t[2]), (t[2],t[0])] {
        if (a.z <= z) != (b.z <= z) { p.push(if a.z == z { a } else if b.z == z { b } else { a.lerp(b, (z-a.z)/(b.z-a.z)) }); }
    }
    p
}

// Intersect a segment's disc dilation with the local X axis. End discs plus
// its two offset sides cover vertex, horizontal/general edge and facet contact.
fn capsule(a: Point3, b: Point3, radius: f64, range: &mut [f64; 2]) {
    for p in [a,b] {
        if p.y.abs() <= radius {
            let dx = ((radius-p.y.abs())*(radius+p.y.abs())).max(0.0).sqrt();
            range[0] = range[0].min(p.x-dx); range[1] = range[1].max(p.x+dx);
        }
    }
    let d = b.sub(a); let length = d.x.hypot(d.y);
    if length == 0.0 || d.y == 0.0 { return; }
    for sign in [-1.0,1.0] {
        let x = a.x-sign*radius*d.y/length; let y = a.y+sign*radius*d.x/length;
        let s = -y/d.y;
        if (0.0..=1.0).contains(&s) {
            let value = x+s*d.x; range[0] = range[0].min(value); range[1] = range[1].max(value);
        }
    }
}

fn slice(t: [Point3; 3], z: f64, cutter: &Cutter, allowance: f64) -> (f64, [f64; 2]) {
    let points = section(t,z);
    let radius = cutter.max_radius_at_height(z).unwrap_or(0.0)+allowance;
    let min_y = points.iter().map(|p| p.y).fold(f64::INFINITY,f64::min);
    let max_y = points.iter().map(|p| p.y).fold(f64::NEG_INFINITY,f64::max);
    let separation = if min_y > 0.0 { min_y } else if max_y < 0.0 { -max_y } else { 0.0 };
    let mut range = [f64::INFINITY,f64::NEG_INFINITY];
    for a in &points { for b in &points { capsule(*a,*b,radius,&mut range); } }
    (radius-separation, range)
}

// Recover the mesh witness at the winning support height. This records the
// actual vertex/edge/facet point, rather than the cutter-axis boundary point.
fn slice_contact(t:[Point3;3],z:f64,cutter:&Cutter,allowance:f64,right:bool)->Point3 {
    let points=section(t,z);let radius=cutter.max_radius_at_height(z).unwrap_or(0.0)+allowance;
    let mut best=if right {f64::NEG_INFINITY}else{f64::INFINITY};let mut witness=points[0];
    let mut accept=|x:f64,p:Point3| {if (right && x>best)||(!right && x<best) {best=x;witness=p;}};
    for a in &points {
        if a.y.abs()<=radius {let dx=((radius-a.y.abs())*(radius+a.y.abs())).max(0.0).sqrt();accept(a.x+if right {dx}else{-dx},*a);}
        for b in &points {
            let d=b.sub(*a);let length=d.x.hypot(d.y);if length==0.0 || d.y==0.0 {continue;}
            for sign in [-1.0,1.0] {
                let x=a.x-sign*radius*d.y/length;let y=a.y+sign*radius*d.x/length;let s=-y/d.y;
                if (0.0..=1.0).contains(&s) {accept(x+s*d.x,a.lerp(*b,s));}
            }
        }
    }
    witness
}

// Golden-section maximization on a concave slice support function. Piecewise
// profile and triangle height breakpoints isolate convex domains. 80 steps
// reduce even a metre-scale bracket below f64 resolution in model units.
fn maximum(mut a: f64, mut b: f64, f: impl Fn(f64)->f64) -> (f64,f64) {
    let ratio = (5.0_f64.sqrt()-1.0)*0.5;
    let mut c = b-ratio*(b-a); let mut d = a+ratio*(b-a);
    let mut fc = f(c); let mut fd = f(d);
    for _ in 0..80 {
        if fc < fd { a=c; c=d; fc=fd; d=a+ratio*(b-a); fd=f(d); }
        else { b=d; d=c; fd=fc; c=b-ratio*(b-a); fc=f(c); }
    }
    [(a,f(a)),(b,f(b)),(c,fc),(d,fd)].into_iter().max_by(|a,b| a.1.total_cmp(&b.1)).unwrap()
}

fn triangle_interval(t: [Point3; 3], cutter: &Cutter, allowance: f64, tol: Tolerance, runner: &mut Runner<'_>) -> Result<Option<(f64,f64,Point3,Point3)>,CamError> {
    let lo = t.iter().map(|p| p.z).fold(f64::INFINITY,f64::min).max(0.0);
    let hi = t.iter().map(|p| p.z).fold(f64::NEG_INFINITY,f64::max).min(cutter.total_height());
    if hi < lo { return Ok(None); }
    let mut levels = vec![lo,hi];
    levels.extend(t.iter().map(|p| p.z).filter(|z| *z>lo && *z<hi));
    for s in cutter.segments() { levels.extend(s.height_range.into_iter().filter(|z| *z>lo && *z<hi)); }
    levels.push(cutter.cutting_length().clamp(lo,hi));
    levels.sort_by(f64::total_cmp); levels.dedup();
    let mut range = [f64::INFINITY,f64::NEG_INFINITY];
    let mut support_z=[lo,lo];
    for z in &levels {
        let (_, r) = slice(t,*z,cutter,allowance);
        if r[0]<range[0] {range[0]=r[0];support_z[0]=*z;}
        if r[1]>range[1] {range[1]=r[1];support_z[1]=*z;}
    }
    for ab in levels.windows(2) {
        runner.work(1,"push-solve",0,0)?;
        let start = if ab[0] == cutter.cutting_length() && cutter.shaft_radius() != cutter.radius() { ab[0].next_up() } else { ab[0] };
        if start > ab[1] { continue; }
        let clear = |z| slice(t,z,cutter,allowance).0;
        let (peak, feasible) = maximum(start,ab[1],clear);
        if feasible < 0.0 { continue; }
        let mut lower=start; let mut upper=ab[1];
        if clear(lower)<0.0 {
            let mut inside=peak;
            for _ in 0..64 { let mid=(lower+inside)*0.5; if clear(mid)<0.0 { lower=mid; } else { inside=mid; } }
            lower=inside;
        }
        if clear(upper)<0.0 {
            let mut inside=peak;
            for _ in 0..64 { let mid=(upper+inside)*0.5; if clear(mid)<0.0 { upper=mid; } else { inside=mid; } }
            upper=inside;
        }
        let (left_z, left)=maximum(lower,upper,|z| -slice(t,z,cutter,allowance).1[0]);
        let (right_z, right)=maximum(lower,upper,|z| slice(t,z,cutter,allowance).1[1]);
        if -left<range[0] {range[0]=-left;support_z[0]=left_z;}
        if right>range[1] {range[1]=right;support_z[1]=right_z;}
    }
    if range[0].is_finite() && range[1].is_finite() {
        // Outward numeric padding, never silently shrink a blocked interval.
        Ok(Some((range[0]-tol.distance_eps,range[1]+tol.distance_eps,
            slice_contact(t,support_z[0],cutter,allowance,false),slice_contact(t,support_z[1],cutter,allowance,true))))
    } else { Ok(None) }
}

