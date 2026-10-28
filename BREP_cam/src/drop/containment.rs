//! Shared projected-surface containment for every drop-family strategy.
//! Slope is the unsigned facet angle to machine XY; Z limits are on the part
//! surface (before cutter/stock offsets). All protected geometry still takes
//! part in drop projection, even when excluded from the machining region.
use crate::*;
use crate::drop::finish::invalid;

/// Resolve implicit offset boundaries once using the whole target-body outline.
/// Face/slope/Z selections clip the machining region separately, never the outline.
pub(crate) fn boundary_region(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,tolerance:Tolerance,c:&Containment)->Result<Region,CamError> {
    c.validate().map_err(CamError::InvalidInput)?;
    let radius=cutter.radius()+operation.stock_to_leave.radial;
    let offset=c.offset+match c.mode {ContainmentMode::ToolCenterInside=>0.,ContainmentMode::ToolInside=>-radius,ContainmentMode::ToolOutside=>radius};
    let boundary=if c.boundary.is_empty() && offset!=0. {
        let silhouette=crate::planar::extract::silhouette(mesh,|id|{let f=&mesh.faces[id as usize];!f.protected_only && operation.targets.target_solids.contains(&f.solid) && !operation.targets.protected_solids.contains(&f.solid)},&tolerance)?;
        if silhouette.loops.is_empty() {return Err(invalid("target silhouette is empty"));}
        silhouette.loops.iter().map(|l|l.segments.iter().map(|s|s.start).collect()).collect()
    } else {c.boundary.clone()};
    for l in &boundary {
        let area=l.iter().zip(l.iter().cycle().skip(1)).take(l.len()).map(|(a,b)|a.x*b.y-a.y*b.x).sum::<f64>().abs();
        if area<=tolerance.area_eps {return Err(invalid("containment loop has zero projected area"));}
    }
    Ok(Region {boundary,drive:vec![],avoid:vec![],offset,avoid_radius:radius+tolerance.distance_eps,selected:false,epsilon:tolerance.distance_eps,drive_index:Default::default()})
}
pub(crate) fn region(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,tolerance:Tolerance,c:&Containment)->Result<Region,CamError> {
    let mut region=boundary_region(operation,cutter,mesh,tolerance,c)?;
    let mut drive=Vec::new();let mut avoid=Vec::new();
    for t in &mesh.triangles {
        let f=&mesh.faces[t.face as usize];
        let matches=|r:&TargetRef|r.solid==f.solid && r.face.as_ref().map_or(true,|id|id==&f.face);
        let poly:Vec<_>=t.vertices().iter().map(|p|Point2::new(p.x,p.y)).collect();
        if f.protected_only || operation.targets.protected_solids.contains(&f.solid) || operation.targets.avoid_faces.iter().any(matches) {avoid.push(poly);}
        if f.protected_only || !operation.targets.target_solids.contains(&f.solid) || (!operation.targets.drive_faces.is_empty() && !operation.targets.drive_faces.iter().any(matches)) {continue;}
        let angle=t.normal.z.abs().clamp(0.,1.).acos().to_degrees();
        if c.slope_deg.map_or(false,|r|angle<r[0]-tolerance.angle_eps||angle>r[1]+tolerance.angle_eps) {continue;}
        let mut vertices=t.vertices().to_vec();
        if let Some([lo,hi])=c.z_range {vertices=clip_z(&clip_z(&vertices,lo,true),hi,false);}
        if vertices.len()>=3 {drive.push(vertices.iter().map(|p|Point2::new(p.x,p.y)).collect());}
    }
    if drive.is_empty() {return Err(invalid("containment has no reachable drive triangles"));}
    region.drive=drive;region.avoid=avoid;region.selected=!operation.targets.drive_faces.is_empty()||c.slope_deg.is_some()||c.z_range.is_some();Ok(region)
}
fn clip_z(poly:&[Point3],z:f64,above:bool)->Vec<Point3> {
    let mut out=Vec::new();
    for (&a,&b) in poly.iter().zip(poly.iter().cycle().skip(1)).take(poly.len()) {
        let inside=|p:Point3|if above {p.z>=z}else{p.z<=z};let ia=inside(a);let ib=inside(b);
        if ia {out.push(a);}if ia!=ib {out.push(a.lerp(b,(z-a.z)/(b.z-a.z)));}
    }out
}
pub(crate) struct Region {pub(crate) boundary:Vec<Vec<Point2>>,pub(crate) drive:Vec<Vec<Point2>>,avoid:Vec<Vec<Point2>>,pub(crate) offset:f64,avoid_radius:f64,selected:bool,epsilon:f64,
    /// Bucket index of `drive`, built on the first clip (after `drive` is set).
    drive_index:std::sync::OnceLock<DriveIndex>}

/// Uniform buckets over the drive polygons' XY boxes. A polygon adds a drive
/// interval to a clipped segment only where the segment comes within
/// `epsilon` of it, so polygons whose box is farther than that from the
/// segment's box are skipped; the rest are taken in their original order.
#[derive(Clone,Debug)]
struct DriveIndex {lo:Point2,cell:f64,nx:usize,ny:usize,buckets:Vec<Vec<u32>>,boxes:Vec<[f64;4]>,margin:f64}
impl DriveIndex {
    fn new(drive:&[Vec<Point2>],epsilon:f64)->Self {
        let margin=2.0*epsilon.max(0.0);
        let boxes:Vec<[f64;4]>=drive.iter().map(|l|{let mut b=[f64::INFINITY,f64::INFINITY,f64::NEG_INFINITY,f64::NEG_INFINITY];for p in l {b[0]=b[0].min(p.x);b[1]=b[1].min(p.y);b[2]=b[2].max(p.x);b[3]=b[3].max(p.y);}b}).collect();
        let finite:Vec<&[f64;4]>=boxes.iter().filter(|b|b.iter().all(|v|v.is_finite())).collect();
        let lo=Point2::new(finite.iter().map(|b|b[0]).fold(f64::INFINITY,f64::min),finite.iter().map(|b|b[1]).fold(f64::INFINITY,f64::min));
        let hi=Point2::new(finite.iter().map(|b|b[2]).fold(f64::NEG_INFINITY,f64::max),finite.iter().map(|b|b[3]).fold(f64::NEG_INFINITY,f64::max));
        let side=((drive.len() as f64).sqrt().ceil() as usize).clamp(1,2048);
        let extent=(hi.x-lo.x).max(hi.y-lo.y);
        let cell=if extent.is_finite() && extent>0.0 {extent/side as f64} else {1.0};
        let (nx,ny)=if extent.is_finite() {((((hi.x-lo.x)/cell).floor() as usize+1).min(side+1),(((hi.y-lo.y)/cell).floor() as usize+1).min(side+1))} else {(1,1)};
        let mut index=Self {lo:if lo.x.is_finite() {lo} else {Point2::new(0.,0.)},cell,nx,ny,buckets:vec![Vec::new();nx*ny],boxes,margin};
        for (i,b) in index.boxes.clone().iter().enumerate() {
            // A non-finite box goes everywhere (never skipped).
            let (x0,y0,x1,y1)=if b.iter().all(|v|v.is_finite()) {index.range(b[0]-margin,b[1]-margin,b[2]+margin,b[3]+margin)} else {(0,0,nx-1,ny-1)};
            for y in y0..=y1 {for x in x0..=x1 {index.buckets[y*nx+x].push(i as u32);}}
        }
        index
    }
    fn range(&self,x0:f64,y0:f64,x1:f64,y1:f64)->(usize,usize,usize,usize) {
        let f=|v:f64,o:f64,n:usize|(((v-o)/self.cell).floor().max(0.0) as usize).min(n-1);
        (f(x0,self.lo.x,self.nx),f(y0,self.lo.y,self.ny),f(x1,self.lo.x,self.nx),f(y1,self.lo.y,self.ny))
    }
    /// Drive polygons that may come within `epsilon` of segment `a`-`b`, ascending.
    fn candidates(&self,a:Point3,b:Point3)->Vec<usize> {
        if ![a.x,a.y,b.x,b.y].iter().all(|v|v.is_finite()) {return (0..self.boxes.len()).collect();}
        let (sx0,sy0,sx1,sy1)=(a.x.min(b.x)-self.margin,a.y.min(b.y)-self.margin,a.x.max(b.x)+self.margin,a.y.max(b.y)+self.margin);
        let (x0,y0,x1,y1)=self.range(sx0,sy0,sx1,sy1);
        let mut out:Vec<usize>=Vec::new();
        for y in y0..=y1 {for x in x0..=x1 {for &i in &self.buckets[y*self.nx+x] {
            let bx=&self.boxes[i as usize];
            if !bx.iter().all(|v|v.is_finite()) || (bx[0]<=sx1 && bx[2]>=sx0 && bx[1]<=sy1 && bx[3]>=sy0) {out.push(i as usize);}
        }}}
        out.sort_unstable();out.dedup();out
    }
}
impl Region {
    /// Waterlines already carry surface selection; apply only boundary limits.
    pub(crate) fn clip_boundary(&self,a:Point3,b:Point3,runner:&mut Runner<'_>)->Result<Vec<(Point3,Point3)>,CamError> {
        if self.boundary.is_empty() {Ok(vec![(a,b)])} else {self.clip(a,b,runner)}
    }
    pub(crate) fn from_polygons(drive:Vec<Vec<Point2>>,epsilon:f64)->Self {Self {boundary:vec![],drive,avoid:vec![],offset:0.,avoid_radius:0.,selected:true,epsilon,drive_index:Default::default()}}

    pub(crate) fn contains(&self,p:Point2)->bool {
        let drive=(!self.boundary.is_empty() && !self.selected) || self.drive.iter().any(|l|inside(l,p)||boundary_distance(l,p)<=self.epsilon);
        self.contains_with_drive(p,drive)
    }
    fn contains_with_drive(&self,p:Point2,drive:bool)->bool {
        let boundary=if self.boundary.is_empty() {drive} else {
            let base=self.boundary.iter().filter(|l|inside(l,p)).count()%2==1;
            let distance=self.boundary.iter().map(|l|boundary_distance(l,p)).fold(f64::INFINITY,f64::min);
            if self.offset<0. {base && distance+self.epsilon>=-self.offset} else {base || distance<=self.offset+self.epsilon}
        };
        boundary && (!self.selected || drive) && !self.avoid.iter().any(|l|inside(l,p)||boundary_distance(l,p)<self.avoid_radius)
    }
    /// The parameter intervals of `a`-`b` inside drive polygon `l` (or within
    /// `epsilon` of it).
    pub(crate) fn drive_cut_intervals(&self,a:Point3,b:Point3,l:&[Point2],out:&mut Vec<(f64,f64)>) {
        let at=|t|if t==0. {a} else if t==1. {b} else {a.lerp(b,t)};
        let mut cuts=vec![0.,1.];
        for (&c,&d) in l.iter().zip(l.iter().cycle().skip(1)).take(l.len()) {edge_cuts(a,b,c,d,0.,self.epsilon,&mut cuts);}
        cuts.sort_by(f64::total_cmp);cuts.dedup();
        for pair in cuts.windows(2) {let mid=at((pair[0]+pair[1])*0.5);let xy=Point2::new(mid.x,mid.y);
            if inside(l,xy)||boundary_distance(l,xy)<=self.epsilon {out.push((pair[0],pair[1]));}
        }
    }
    pub(crate) fn clip(&self,a:Point3,b:Point3,runner:&mut Runner<'_>)->Result<Vec<(Point3,Point3)>,CamError> {
        let needs_drive=self.boundary.is_empty()||self.selected;
        let mut drive_intervals:Vec<(f64,f64)>=Vec::new();
        if needs_drive {
            let index=self.drive_index.get_or_init(||DriveIndex::new(&self.drive,self.epsilon));
            for l in index.candidates(a,b).into_iter().map(|i|&self.drive[i]) {
                self.drive_cut_intervals(a,b,l,&mut drive_intervals);
                runner.work(1,"finish-clip-drive",0,0)?;
            }
            drive_intervals.sort_by(|a,b|a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
            let mut union:Vec<(f64,f64)>=Vec::new();
            for (lo,hi) in drive_intervals {if let Some(last)=union.last_mut() {if lo<=last.1+self.epsilon/a.distance_xy(b).max(self.epsilon) {last.1=last.1.max(hi);continue;}}union.push((lo,hi));}
            drive_intervals=union;
        }
        let mut cuts=vec![0.,1.];
        for &(lo,hi) in &drive_intervals {cuts.push(lo);cuts.push(hi);}
        for (loops,radius) in [(&self.boundary,self.offset.abs()),(&self.avoid,self.avoid_radius)] {
            for l in loops {for (&c,&d) in l.iter().zip(l.iter().cycle().skip(1)).take(l.len()) {
                edge_cuts(a,b,c,d,radius,self.epsilon,&mut cuts);runner.work(1,"finish-clip-edges",cuts.len() as u64,0)?;
            }}
        }
        cuts.sort_by(f64::total_cmp);cuts.dedup();let mut out:Vec<(Point3,Point3)>=Vec::new();
        let at=|t|if t==0. {a} else if t==1. {b} else {a.lerp(b,t)};
        for pair in cuts.windows(2) {let t=(pair[0]+pair[1])*0.5;let mid=at(t);
            let k=drive_intervals.partition_point(|p|p.1<t);
            let drive=!needs_drive || drive_intervals.get(k).map_or(false,|p|p.0<=t&&t<=p.1);
            if self.contains_with_drive(Point2::new(mid.x,mid.y),drive) {
                let p=at(pair[0]);let q=at(pair[1]);
                if let Some(last)=out.last_mut() {if last.1==p {last.1=q;continue;}}
                out.push((p,q));
            }
        }
        Ok(out)
    }
}
fn inside(l:&[Point2],p:Point2)->bool {
    let mut yes=false;
    for (a,b) in l.iter().zip(l.iter().cycle().skip(1)).take(l.len()) {
        if (a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x {yes=!yes;}
    }
    yes
}
fn boundary_distance(l:&[Point2],p:Point2)->f64 {
    l.iter().zip(l.iter().cycle().skip(1)).take(l.len()).map(|(a,b)|{
        let dx=b.x-a.x;let dy=b.y-a.y;let q=dx*dx+dy*dy;
        let t=if q==0. {0.} else {((p.x-a.x)*dx+(p.y-a.y)*dy)/q}.clamp(0.,1.);
        (p.x-a.x-t*dx).hypot(p.y-a.y-t*dy)
    }).fold(f64::INFINITY,f64::min)
}
fn edge_cuts(a:Point3,b:Point3,c:Point2,d:Point2,r:f64,epsilon:f64,cuts:&mut Vec<f64>) {
    let (vx,vy)=(b.x-a.x,b.y-a.y);let (ex,ey)=(d.x-c.x,d.y-c.y);let len=ex.hypot(ey);
    let det=vx*ey-vy*ex;
    if det!=0. {
        for side in if r>0. {vec![-r,0.,r]} else {vec![0.]} {
            let (cx,cy)=if len==0. {(c.x,c.y)} else {(c.x-side*ey/len,c.y+side*ex/len)};
            let t=((cx-a.x)*ey-(cy-a.y)*ex)/det;
            let u=((cx-a.x)*vy-(cy-a.y)*vx)/det;
            let te=epsilon/vx.hypot(vy).max(epsilon);let ue=epsilon/len.max(epsilon);
            if t>=-te && t<=1.+te && u>=-ue && u<=1.+ue {cuts.push(t.clamp(0.,1.));}
        }
    }
    if r>0. {for p in [c,d] {
        let q=vx*vx+vy*vy; if q==0. {continue;}
        let dot=(a.x-p.x)*vx+(a.y-p.y)*vy;let near=-dot/q;
        let dist=(a.x+near*vx-p.x).hypot(a.y+near*vy-p.y);
        if dist<=r {let delta=((r-dist)*(r+dist)/q).sqrt();for t in [near-delta,near+delta] {if (0.0..=1.0).contains(&t){cuts.push(t);}}}
    }}
}

