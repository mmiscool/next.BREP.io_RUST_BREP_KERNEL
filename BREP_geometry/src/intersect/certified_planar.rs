//! Complete projected polynomial-curve intersection cover. Unresolved boxes,
//! tangencies, overlaps and budget exhaustion decline rather than returning
//! a partial list. This module is not used by the normal intersection path.
use crate::{CurveCurveIntersection, NurbsCurve};
use std::ops::{Add, Sub, Mul, Neg};

#[derive(Clone, Copy, Debug)]
struct I { lo: f64, hi: f64 }
impl I {
    fn point(x: f64) -> Self { Self { lo: x, hi: x } }
    fn range(lo: f64, hi: f64) -> Self { Self { lo, hi } }
    fn mid(self) -> f64 { self.lo * 0.5 + self.hi * 0.5 }
    fn finite(self) -> bool { self.lo.is_finite() && self.hi.is_finite() && self.lo <= self.hi }
    fn contains_zero(self) -> bool { self.lo <= 0.0 && self.hi >= 0.0 }
    fn divide(self, rhs: Self) -> Option<Self> {
        if rhs.contains_zero() { return None; }
        if rhs.lo == 1. && rhs.hi == 1. { return Some(self); }
        let inverse = Self::range((1.0 / rhs.hi).next_down(), (1.0 / rhs.lo).next_up());
        let result = self * inverse;
        result.finite().then_some(result)
    }
}
// Error-free transforms avoid adding a full ulp when an operation was exact.
// Products near underflow retain the conservative one-ulp enclosure.
fn sum(a:f64,b:f64)->I {
    let value=a+b;
    if !value.is_finite() {return I::range(value.next_down(),value.next_up());}
    let back=value-a;
    let error=(a-(value-back))+(b-back);
    I::range(if error<0. {value.next_down()} else {value},if error>0. {value.next_up()} else {value})
}
fn product(a:f64,b:f64)->I {
    if a==0. || b==0. {return I::point(0.);}
    let value=a*b;
    if !value.is_finite() || value.abs()<f64::MIN_POSITIVE {
        return I::range(value.next_down(),value.next_up());
    }
    let error=a.mul_add(b,-value);
    // A rounded-to-zero residual can hide an underflowed nonzero error.
    if error==0. && value.abs()<f64::MIN_POSITIVE/f64::EPSILON/f64::EPSILON {
        return I::range(value.next_down(),value.next_up());
    }
    I::range(if error<0. {value.next_down()} else {value},if error>0. {value.next_up()} else {value})
}
impl Add for I { type Output=Self; fn add(self,b:Self)->Self { Self::range(sum(self.lo,b.lo).lo,sum(self.hi,b.hi).hi) } }
impl Neg for I { type Output=Self; fn neg(self)->Self { Self::range(-self.hi,-self.lo) } }
impl Sub for I { type Output=Self; fn sub(self,b:Self)->Self { self + (-b) } }
impl Mul for I {
    type Output=Self;
    fn mul(self,b:Self)->Self {
        let v=[product(self.lo,b.lo),product(self.lo,b.hi),product(self.hi,b.lo),product(self.hi,b.hi)];
        Self::range(v.into_iter().map(|x|x.lo).fold(f64::INFINITY,f64::min),
            v.into_iter().map(|x|x.hi).fold(f64::NEG_INFINITY,f64::max))
    }
}
type P=[I;3];
fn blend(a:P,b:P,t:I)->P { [0,1,2].map(|i| t*a[i]+(I::point(1.)-t)*b[i]) }
fn midpoint(controls:&[P])->P {
    let mut work=controls.to_vec();
    while work.len()>1 { work=work.windows(2).map(|p|blend(p[0],p[1],I::point(0.5))).collect(); }
    work[0]
}

#[derive(Clone)]
struct Span { controls:Vec<P>, tangents:Vec<P>, start:f64, end:f64 }
impl Span {
    fn split(self)->Option<(Self,Self)> {
        let middle=self.start*0.5+self.end*0.5;
        if middle<=self.start || middle>=self.end { return None; }
        let (mut dl,mut dr)=(Vec::new(),Vec::new());
        if !self.tangents.is_empty() {
            // Subdivide the derivative polynomial independently: subtracting
            // deep position enclosures loses the small tangent to cancellation.
            let mut work=self.tangents;
            dl.push(work[0]);dr.push(*work.last()?);
            while work.len()>1 {
                work=work.windows(2).map(|p|blend(p[0],p[1],I::point(0.5))).collect();
                dl.push(work[0]);dr.push(*work.last()?);
            }
            dr.reverse();
            // Each child uses its own unit parameter, so its derivative is
            // half the parent's derivative restricted to that half interval.
            for point in dl.iter_mut().chain(&mut dr) {
                for value in point { *value=*value*I::point(0.5); }
            }
        }
        let mut work=self.controls;
        let mut left=vec![work[0]];let mut right=vec![*work.last()?];
        while work.len()>1 {
            work=work.windows(2).map(|p|blend(p[0],p[1],I::point(0.5))).collect();
            left.push(work[0]);right.push(*work.last()?);
        }
        right.reverse();
        Some((Self{controls:left,tangents:dl,start:self.start,end:middle},Self{controls:right,tangents:dr,start:middle,end:self.end}))
    }
    fn bounds(&self)->P {
        [0,1,2].map(|i|I::range(self.controls.iter().map(|p|p[i].lo).fold(f64::INFINITY,f64::min),
            self.controls.iter().map(|p|p[i].hi).fold(f64::NEG_INFINITY,f64::max)))
    }
    fn derivative(&self)->P {
        if !self.tangents.is_empty() {
            return [0,1,2].map(|i|I::range(
                self.tangents.iter().map(|p|p[i].lo).fold(f64::INFINITY,f64::min),
                self.tangents.iter().map(|p|p[i].hi).fold(f64::NEG_INFINITY,f64::max)));
        }
        let n=I::point((self.controls.len()-1) as f64);
        [0,1,2].map(|i| {
            let d:Vec<_>=self.controls.windows(2).map(|p|n*(p[1][i]-p[0][i])).collect();
            I::range(d.iter().map(|x|x.lo).fold(f64::INFINITY,f64::min),d.iter().map(|x|x.hi).fold(f64::NEG_INFINITY,f64::max))
        })
    }
}

/// Bezier extraction with outward-rounded control arithmetic, including the
/// knot ratios. Only constant positive weights are supported here.
fn spans(curve:&NurbsCurve,derivative_net:bool,origin:Option<[f64;3]>)->Option<Vec<Span>> {
    curve.domain().ok()?;
    let p=curve.degree;let knots=&curve.knots;let n=curve.control_points.len();
    if p==0 || p>8 || n>4096 || knots.len()!=n+p+1 { return None; }
    let weight=curve.control_points.first()?.w;
    if !weight.is_finite() || weight<=0. || curve.control_points.iter().any(|v|v.w!=weight) { return None; }
    let (lo,hi)=(*knots.first()?,*knots.last()?);
    if !(hi>lo) || knots.iter().any(|v|!v.is_finite() || v.abs()>1e100)
        || knots.windows(2).any(|k|k[0]>k[1]) || knots[..=p].iter().any(|v|*v!=lo)
        || knots[knots.len()-p-1..].iter().any(|v|*v!=hi) { return None; }
    // Extraction starts at the first nonempty span only for canonical
    // clamping. Extra endpoint knots leave unused controls; including them
    // would certify intersections of a phantom zero-domain segment.
    if knots[p+1]<=lo || knots[knots.len()-p-2]>=hi {return None;}
    let points:Vec<P>=curve.control_points.iter().map(|v| {
        let xyz=[v.x,v.y,v.z];
        let a=I::point(xyz[0]).divide(I::point(weight))?;
        let b=I::point(xyz[1]).divide(I::point(weight))?;
        let c=I::point(xyz[2]).divide(I::point(weight))?;
        let point=[a,b,c];
        if !point.iter().all(|v|v.lo.abs().max(v.hi.abs())<1e100) {return None;}
        Some(match origin {
            Some(origin)=>[0,1,2].map(|i|point[i]-I::point(origin[i])),
            None=>point,
        })
    }).collect::<Option<_>>()?;
    let (mut a,mut b)=(p,p+1);let m=knots.len()-1;let zero=[I::point(0.);3];
    let mut controls=points[..=p].to_vec();let mut result=Vec::new();
    while b<m {
        let first=b;while b<m && knots[b+1]==knots[b] {b+=1;}
        let mult=b-first+1;if b<m && mult>p {return None;}
        let mut next=vec![zero;p+1];
        if mult<p {
            let numerator=I::point(knots[b])-I::point(knots[a]);
            let mut alpha=vec![I::point(0.);p];
            for j in ((mult+1)..=p).rev() {
                let x=numerator.divide(I::point(knots[a+j])-I::point(knots[a]))?;
                alpha[j-mult-1]=I::range(x.lo.max(0.),x.hi.min(1.));
                if !alpha[j-mult-1].finite() {return None;}
            }
            let r=p-mult;
            for j in 1..=r {
                let s=mult+j;
                for k in (s..=p).rev() {controls[k]=blend(controls[k],controls[k-1],alpha[k-s]);}
                if b<m {next[r-j]=controls[p];}
            }
        }
        let tangents=if derivative_net {
            let n=I::point((controls.len()-1) as f64);
            controls.windows(2).map(|pair|[0,1,2].map(|i|n*(pair[1][i]-pair[0][i]))).collect()
        } else {Vec::new()};
        result.push(Span{controls,tangents,start:knots[a],end:knots[b]});
        if b>=m {break;}
        for i in p-mult..=p {next[i]=points[b-p+i];}
        controls=next;a=b;b+=1;
    }
    Some(result)
}

fn separated(a:&Span,b:&Span,axes:[usize;2])->bool {
    let chord=|s:&Span| {
        let first=s.controls.first().unwrap();let last=s.controls.last().unwrap();
        [last[axes[0]].mid()-first[axes[0]].mid(),last[axes[1]].mid()-first[axes[1]].mid()]
    };
    let ca=chord(a);let cb=chord(b);
    for axis in [[1.,0.],[0.,1.],[-ca[1],ca[0]],[-cb[1],cb[0]]] {
        let bounds=|s:&Span| {
            let dots:Vec<_>=s.controls.iter().map(|p|p[axes[0]]*I::point(axis[0])+p[axes[1]]*I::point(axis[1])).collect();
            I::range(dots.iter().map(|v|v.lo).fold(f64::INFINITY,f64::min),dots.iter().map(|v|v.hi).fold(f64::NEG_INFINITY,f64::max))
        };
        let x=bounds(a);let y=bounds(b);
        if x.finite() && y.finite() && (x.hi<y.lo || y.hi<x.lo) {return true;}
    }
    false
}

enum Read { Empty, Unique([I;2]), Unknown }
fn krawczyk(a:&Span,b:&Span,axes:[usize;2])->Read {
    let da=a.derivative();let db=b.derivative();
    let j=[[da[axes[0]],-db[axes[0]]],[da[axes[1]],-db[axes[1]]]];
    let d=j[0][0].mid()*j[1][1].mid()-j[0][1].mid()*j[1][0].mid();
    if !d.is_finite() || d==0. {return Read::Unknown;}
    let c=[[j[1][1].mid()/d,-j[0][1].mid()/d],[-j[1][0].mid()/d,j[0][0].mid()/d]];
    if c.iter().flatten().any(|v|!v.is_finite() || v.abs()>1e100) {return Read::Unknown;}
    if (I::point(c[0][0])*I::point(c[1][1])-I::point(c[0][1])*I::point(c[1][0])).contains_zero() {return Read::Unknown;}
    let pa=midpoint(&a.controls);let pb=midpoint(&b.controls);let f=[pa[axes[0]]-pb[axes[0]],pa[axes[1]]-pb[axes[1]]];
    let k=[0,1].map(|i| {
        let mut value=I::point(0.5)-I::point(c[i][0])*f[0]-I::point(c[i][1])*f[1];
        for col in 0..2 {
            let matrix=I::point(if i==col {1.} else {0.})-I::point(c[i][0])*j[0][col]-I::point(c[i][1])*j[1][col];
            value=value+matrix*I::range(-0.5,0.5);
        }
        value
    });
    if k.iter().any(|x|!x.finite()) {return Read::Unknown;}
    if k.iter().any(|x|x.hi<0. || x.lo>1.) {Read::Empty}
    else if k.iter().all(|x|x.lo>0. && x.hi<1.) {Read::Unique(k)}
    else {Read::Unknown}
}

// Recenter before interval evaluation so common world offsets do not turn
// a narrow parameter enclosure into a needlessly broad position interval.
fn enclosure(span:&Span,t:I)->P {
    let origin=span.controls[0].map(|x|I::point(x.mid()));
    let mut work:Vec<P>=span.controls.iter().map(|p|[0,1,2].map(|i|p[i]-origin[i])).collect();
    while work.len()>1 {work=work.windows(2).map(|p|blend(p[1],p[0],t)).collect();}
    [0,1,2].map(|i|work[0][i]+origin[i])
}
fn small(bounds:P,tolerance:f64)->bool {
    if !bounds.iter().all(|x|x.finite()) {return false;}
    let width=bounds.into_iter().fold(I::point(0.),|sum,x|sum+(I::point(x.hi)-I::point(x.lo)));
    width.hi<=product(tolerance,0.25).lo
}
// An outward L1 bound is conservative for the Euclidean distance and does
// not lose tiny nonzero separations by squaring them below the float range.
pub(crate) fn distance_upper(a:crate::Vec3,b:crate::Vec3)->f64 {
    let deltas=[(a.x,b.x),(a.y,b.y),(a.z,b.z)];
    deltas.into_iter().fold(I::point(0.),|sum,(a,b)| {
        let d=I::point(a)-I::point(b);
        sum+I::point(d.lo.abs().max(d.hi.abs()))
    }).hi
}
fn inside(point:crate::Vec3,bounds:P,tolerance:f64)->bool {
    [point.x,point.y,point.z].into_iter().zip(bounds).all(|(v,b)|v>=b.lo-tolerance*0.0625 && v<=b.hi+tolerance*0.0625)
}

// Once a rectangle has exactly one root, retain that certificate while
// tightening its position. Its root may lie on a later subdivision boundary;
// the children need only cover it, not prove existence separately.
fn tighten_unique(a:&Span,b:&Span,axes:[usize;2],tolerance:f64)->Option<(P,P)> {
    let mut boxes=vec![(a.clone(),b.clone())];
    for _ in 0..80 {
        let mut live=Vec::new();
        let mut ea=[I::range(f64::INFINITY,f64::NEG_INFINITY);3];let mut eb=ea;
        for (a,b) in boxes {
            if separated(&a,&b,axes) {continue;}
            let (pa,pb)=match krawczyk(&a,&b,axes) {
                Read::Empty=>continue,
                Read::Unique(k)=>(enclosure(&a,k[0]),enclosure(&b,k[1])),
                Read::Unknown=>(a.bounds(),b.bounds()),
            };
            for i in 0..3 {
                ea[i]=I::range(ea[i].lo.min(pa[i].lo),ea[i].hi.max(pa[i].hi));
                eb[i]=I::range(eb[i].lo.min(pb[i].lo),eb[i].hi.max(pb[i].hi));
            }
            live.push((a,b));
        }
        if live.is_empty() || live.len()>2048 {return None;}
        if small(ea,tolerance) && small(eb,tolerance) {return Some((ea,eb));}
        boxes=Vec::with_capacity(live.len()*2);
        for (a,b) in live {
            let size=|s:&Span|s.bounds().iter().map(|v|v.hi-v.lo).fold(0.0,f64::max);
            if size(&a)>=size(&b) {let (x,y)=a.split()?;boxes.push((x,b.clone()));boxes.push((y,b));}
            else {let (x,y)=b.split()?;boxes.push((a.clone(),x));boxes.push((a,y));}
        }
    }
    None
}

fn refine(first:&NurbsCurve,second:&NurbsCurve,a:&Span,b:&Span,k:[I;2],axes:[usize;2],tolerance:f64)->Option<CurveCurveIntersection> {
    let mut s=a.start+(a.end-a.start)*k[0].mid();let mut t=b.start+(b.end-b.start)*k[1].mid();
    let pick=|p:crate::Vec3| {let v=[p.x,p.y,p.z];[v[axes[0]],v[axes[1]]]};
    for _ in 0..32 {
        let (pa,da)=first.deriv1(s).ok()?;let (pb,db)=second.deriv1(t).ok()?;
        let f=pick(pa.sub(pb));let da=pick(da);let db=pick(db);
        let d=da[0]*db[1]-da[1]*db[0];if d==0. || !d.is_finite() {return None;}
        let ds=(-f[0]*db[1]+f[1]*db[0])/d;let dt=(da[0]*f[1]-da[1]*f[0])/d;
        let ns=(s+ds).clamp(a.start,a.end);let nt=(t+dt).clamp(b.start,b.end);
        if ns==s && nt==t {break;} s=ns;t=nt;
    }
    let p=first.evaluate(s).ok()?;let q=second.evaluate(t).ok()?;let gap=distance_upper(p,q);
    (gap.is_finite() && gap<=tolerance).then_some(CurveCurveIntersection{s,t,point:p,gap,tangential:false})
}

/// Returns all isolated regular crossings only when every parameter rectangle
/// was excluded or certified to contain exactly one root. `None` includes
/// unresolved boundary roots and positive-dimensional overlaps. The caller
/// must retain its ordinary path on `None`; a partial answer is never exposed.
pub fn intersections(first:&NurbsCurve,second:&NurbsCurve,tolerance:f64)->Option<Vec<CurveCurveIntersection>> {
    let enabled=crate::switch!("BREP_CERTIFIED_PROFILE_DERIVATIVE_NET",true);
    let check=crate::switch!("BREP_CERTIFIED_PROFILE_DERIVATIVE_NET_CHECK",false);
    let result=intersections_with_frames(first,second,tolerance,enabled);
    if check {
        let alternate=intersections_with_frames(first,second,tolerance,!enabled);
        let signature=|roots:&Option<Vec<CurveCurveIntersection>>| roots.as_ref().map(|roots| {
            roots.iter().map(|r| (r.s.to_bits(),r.t.to_bits(),r.point.x.to_bits(),
                r.point.y.to_bits(),r.point.z.to_bits(),r.gap.to_bits(),r.tangential)).collect::<Vec<_>>()
        });
        if signature(&result)!=signature(&alternate) {
            let (old,new)=if enabled {(&alternate,&result)} else {(&result,&alternate)};
            eprintln!("CERTIFIED_PROFILE_DERIVATIVE_NET old={:?} new={:?} applied={enabled}",
                old.as_ref().map(Vec::len),new.as_ref().map(Vec::len));
        }
    }
    result
}

fn intersections_with_frames(first:&NurbsCurve,second:&NurbsCurve,tolerance:f64,derivative_net:bool)->Option<Vec<CurveCurveIntersection>> {
    let base=intersections_impl(first,second,tolerance,derivative_net,false);
    if base.is_some() {return base;}
    let enabled=crate::switch!("BREP_CERTIFIED_PROFILE_LOCAL_FRAME",true);
    let check=crate::switch!("BREP_CERTIFIED_PROFILE_LOCAL_FRAME_CHECK",false);
    if !enabled && !check {return base;}
    let candidate=intersections_impl(first,second,tolerance,derivative_net,true);
    if check && candidate.is_some() {
        eprintln!("CERTIFIED_PROFILE_LOCAL_FRAME old=None new={:?} applied={enabled}",
            candidate.as_ref().map(Vec::len));
    }
    if enabled {candidate} else {base}
}

fn intersections_impl(first:&NurbsCurve,second:&NurbsCurve,tolerance:f64,derivative_net:bool,local_frame:bool)->Option<Vec<CurveCurveIntersection>> {
    if !tolerance.is_finite() || tolerance<=0. {return None;}
    let mut low=[f64::INFINITY;3];let mut high=[f64::NEG_INFINITY;3];
    for p in first.control_points.iter().chain(&second.control_points) {
        let p=p.point().ok()?;
        for (i,v) in [p.x,p.y,p.z].into_iter().enumerate() {low[i]=low[i].min(v);high[i]=high[i].max(v);}
    }
    let axes=[[0,1],[0,2],[1,2]].into_iter().max_by(|a,b| {
        let area=|axes:[usize;2]|(high[axes[0]]-low[axes[0]])*(high[axes[1]]-low[axes[1]]);
        area(*a).total_cmp(&area(*b))
    })?;
    let initial_origin=local_frame.then(||[0,1,2].map(|i|low[i]*0.5+high[i]*0.5));
    let a=spans(first,derivative_net,initial_origin)?;
    let b=spans(second,derivative_net,initial_origin)?;
    let mut stack=Vec::new();
    let pairs=a.len().checked_mul(b.len())?;
    if pairs>65536 {return None;}
    if local_frame && pairs.checked_mul(first.control_points.len().checked_add(second.control_points.len())?)?>1_048_576 {
        return None;
    }
    for (i,x) in a.iter().enumerate() {for (j,y) in b.iter().enumerate() {
        if let Some(origin)=initial_origin {
            // Re-extract, rather than translate already widened controls.
            // Each original parameter rectangle retains one shared frame
            // through all descendants, preserving its equation and Jacobian.
            let px=midpoint(&x.controls);let py=midpoint(&y.controls);
            let origin=[0,1,2].map(|k|origin[k]+px[k].mid()*0.5+py[k].mid()*0.5);
            let xs=spans(first,derivative_net,Some(origin))?;
            let ys=spans(second,derivative_net,Some(origin))?;
            stack.push((xs[i].clone(),ys[j].clone(),0,Some(origin)));
        } else {
            stack.push((x.clone(),y.clone(),0,None));
        }
    }}
    let mut roots=Vec::new();let mut boxes=0;
    while let Some((a,b,depth,origin))=stack.pop() {
        boxes+=1;if boxes>65536 {return None;}
        if separated(&a,&b,axes) {continue;}
        match krawczyk(&a,&b,axes) {
            Read::Empty=>continue,
            Read::Unique(k)=>{
                let (mut ea,mut eb)=tighten_unique(&a,&b,axes,tolerance)?;
                if let Some(origin)=origin {
                    ea=[0,1,2].map(|i|ea[i]+I::point(origin[i]));
                    eb=[0,1,2].map(|i|eb[i]+I::point(origin[i]));
                    // Restoring a large world offset can widen an enclosure
                    // by a world-coordinate ulp; local precision is insufficient.
                    if !small(ea,tolerance) || !small(eb,tolerance) {return None;}
                }
                let root=refine(first,second,&a,&b,k,axes,tolerance)?;
                if !inside(root.point,ea,tolerance) || !inside(second.evaluate(root.t).ok()?,eb,tolerance) {return None;}
                roots.push(root);continue;
            }
            Read::Unknown=>{}
        }
        if depth>=80 {return None;}
        let size=|s:&Span|s.bounds().iter().map(|v|v.hi-v.lo).fold(0.0,f64::max);
        if size(&a)>=size(&b) {let (x,y)=a.split()?;stack.push((x,b.clone(),depth+1,origin));stack.push((y,b,depth+1,origin));}
        else {let (x,y)=b.split()?;stack.push((a.clone(),x,depth+1,origin));stack.push((a,y,depth+1,origin));}
    }
    roots.sort_by(|a,b|a.s.total_cmp(&b.s).then(a.t.total_cmp(&b.t)));
    Some(roots)
}

