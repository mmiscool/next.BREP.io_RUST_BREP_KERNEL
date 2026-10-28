//! Shared, transactional finishing path clipping, projection and motion assembly.
//! Strategies supply source curves; this module protects the full mesh and
//! certifies every emitted cut chord using the drop-cutter engine.
use crate::*;
use crate::drop::containment::Region;
use crate::drop::{DropCutter,path::{Span,Sampling,Dilated}};

pub const MAX_SOURCE_SEGMENTS: usize = 100_000;

pub(crate) fn invalid(message: &str) -> CamError { CamError::InvalidInput(message.into()) }

/// Inputs shared by all drop-family finishing strategies. It borrows the shared
/// model and never defines a second operation, toolpath or mesh representation.
pub struct FinishContext<'a> {
    pub operation: &'a Operation,
    pub cutter: &'a Cutter,
    pub mesh: &'a MeshSnapshot,
    pub index: Option<&'a dyn TriangleIndex>,
    pub heights: &'a Heights,
    pub feeds: &'a FeedsSpeeds,
    pub tolerance: Tolerance,
}
impl<'a> FinishContext<'a> {
    pub fn validate(&self) -> Result<(),CamError> {
        let op=self.operation;
        op.validate().map_err(CamError::InvalidInput)?;
        self.tolerance.validate()?;
        if !op.enabled {return Err(invalid("finishing operation is disabled"));}
        if op.stock_to_leave.radial<0. || op.stock_to_leave.axial<0. {return Err(invalid("protected finishing requires nonnegative stock allowance"));}
        if let Some(axis)=op.tool_axis {if axis.normalized().unwrap().distance(Point3::Z)>self.tolerance.angle_eps {return Err(invalid("finishing requires machine +Z tool axis"));}}
        self.heights().validate().map_err(CamError::InvalidInput)?;
        self.feeds().validate().map_err(CamError::InvalidInput)?;
        DropCutter::new(self.mesh,self.cutter,self.index,self.tolerance)?;
        if self.heights().clearance<=self.mesh.bounds.max.z+op.stock_to_leave.axial+self.tolerance.distance_eps {
            return Err(invalid("clearance must exceed the full protected mesh and allowance"));
        }
        for solid in op.targets.protected_bodies() {
            if !self.mesh.faces.iter().any(|f|f.solid==solid) {return Err(CamError::UnresolvedTarget {solid,face:None});}
        }
        for face in op.targets.drive_faces.iter().chain(&op.targets.avoid_faces) {
            if self.mesh.resolve_faces(face).is_empty() {return Err(CamError::UnresolvedTarget {solid:face.solid.clone(),face:face.face.clone()});}
        }
        Ok(())
    }
    pub fn heights(&self) -> &Heights {self.operation.heights.as_ref().unwrap_or(self.heights)}
    pub fn feeds(&self) -> &FeedsSpeeds {self.operation.feeds.as_ref().unwrap_or(self.feeds)}
    pub fn sampling(&self) -> Sampling {
        Sampling {sample_spacing:0.5,min_sample_spacing:self.tolerance.sample_eps,chord_error:self.operation.tolerance*0.5,
            floor_z:self.heights().bottom.unwrap_or(self.mesh.bounds.min.z),..Default::default()}
    }
    /// Projected machining extent, useful for deriving default radii/centres.
    pub fn bounds(&self,containment:&Containment) -> Result<Bounds3,CamError> {
        let region=self.region(containment)?;
        let source=if region.boundary.is_empty() {&region.drive} else {&region.boundary};
        let b=Bounds3::from_points(source.iter().flatten().map(|p|Point3::new(p.x,p.y,0.)));
        if b.is_empty() || !b.is_finite() {return Err(invalid("finishing region is empty or nonfinite"));}
        let e=region.offset.max(0.);Ok(Bounds3::new(Point3::new(b.min.x-e,b.min.y-e,b.min.z),Point3::new(b.max.x+e,b.max.y+e,b.max.z)))
    }
    pub(crate) fn region(&self,c:&Containment) -> Result<Region,CamError> {
        self.validate()?;
        crate::drop::containment::region(self.operation,self.cutter,self.mesh,self.tolerance,c)
    }

    pub fn generate(&self,containment:&Containment,paths:&[Vec<Span>],runner:&mut Runner<'_>) -> Result<Toolpath,CamError> {
        let path=self.generate_unlinked(containment,paths,runner)?;
        crate::strategy_linking::finish(&path,self.operation,self.cutter,self.mesh,self.index,self.heights,self.feeds,self.tolerance,runner)
    }
    pub(crate) fn generate_unlinked(&self,containment:&Containment,paths:&[Vec<Span>],runner:&mut Runner<'_>) -> Result<Toolpath,CamError> {
        let region=self.region(containment)?;
        if paths.is_empty() {return Err(invalid("finishing source paths are empty"));}
        runner.stage("finish-clip","Clip finishing curves",0,paths.len() as u64)?;
        let mut groups:Vec<Vec<Span>>=Vec::new();let mut total=0usize;
        for path in paths {
            if path.is_empty() {return Err(invalid("finishing source curve is empty"));}
            let mut current:Vec<Span>=Vec::new();
            for span in path {
                span.validate(&self.tolerance)?;
                let source=linearize(span,self.operation.tolerance*0.25)?;
                for (a,b) in source {
                    let clipped=region.clip(a,b,runner)?;
                    for (a,b) in clipped {
                        if a.distance_xy(b)<=self.tolerance.sample_eps {continue;}
                        if current.last().map_or(false,|s| s.ends().1 != a) {groups.push(std::mem::take(&mut current));}
                        current.push(Span::Line {start:a,end:b});total+=1;
                        if total>MAX_SOURCE_SEGMENTS {return Err(invalid("finishing source segment work budget exhausted"));}
                    }
                }
            }
            if !current.is_empty() {groups.push(current);}
        }
        if groups.is_empty() {return Err(invalid("finishing source curves have no reachable machining region"));}
        let profile=Dilated {base:self.cutter,pad:self.operation.stock_to_leave.radial};
        let drop=DropCutter::from_profile(self.mesh,&profile,self.index,self.tolerance)?;
        let mut out=Toolpath::new(self.operation.id.clone(),self.operation.tool.clone());
        out.report.counts.triangles=self.mesh.triangle_count() as u64;
        let settings=self.sampling();settings.validate()?;
        let mut previous:Option<Point3>=None;
        for (id,group) in groups.iter().enumerate() {
            let (projected,report)=drop.adaptive(group,&settings,runner)?;
            let mut points:Vec<Point3>=Vec::new();let mut contacts=0;
            for path in projected {
                for p in path {contacts+=usize::from(p.contact.is_some());let p=Point3::new(p.x,p.y,p.z+self.operation.stock_to_leave.axial);
                    if let Some(last)=points.last_mut() {if last.x==p.x && last.y==p.y {last.z=last.z.max(p.z);continue;}}
                    points.push(p);
                }
            }
            if points.len()<2 || contacts==0 {return Err(invalid("finishing curve has no reachable surface"));}
            // Close the vertical seam too: each adjacent span may have raised
            // the same XY endpoint by a different conservative chord allowance.
            let last=points.len()-1;
            if points[0].x==points[last].x && points[0].y==points[last].y {
                let z=points[0].z.max(points[last].z);points[0].z=z;points[last].z=z;
            }
            // Dropped spans may share a raised endpoint; raising it preserves
            // both previously certified adjacent cut chords.
            if points.iter().any(|p|p.z>=self.heights().clearance) {return Err(invalid("finishing path exceeds clearance"));}
            if let Some(p)=previous {out.motions.push(Motion::retract(Point3::new(p.x,p.y,self.heights().clearance),Feed::RAPID).with_path(id as u32));}
            let a=points[0];
            out.motions.push(Motion::rapid(Point3::new(a.x,a.y,self.heights().clearance)).with_path(id as u32));
            out.motions.push(Motion::plunge(a,self.feeds().plunge_feed).with_path(id as u32));
            for &p in points.iter().skip(1) {out.motions.push(Motion::cut(p,self.feeds().cutting_feed).with_path(id as u32));}
            previous=points.last().copied();
            out.report.counts.samples+=report.counts.samples;out.report.counts.candidates+=report.counts.candidates;out.report.counts.contacts+=report.counts.contacts;
            out.report.warnings.extend(report.warnings);
            if out.report.counts.samples>settings.max_samples {return Err(invalid("finishing sample work budget exhausted"));}
            runner.work(1,"finish-project",id as u64+1,groups.len() as u64)?;
        }
        if let Some(p)=previous {out.motions.push(Motion::retract(Point3::new(p.x,p.y,self.heights().clearance),Feed::RAPID));}
        out.report.counts.passes=groups.len() as u64;out.report.counts.motions=out.motions.len() as u64;
        out.report.approximations.push(ApproximationLimit {kind:"finishing-linearization-and-lift".into(),bound:Some(self.operation.tolerance),detail:Some("Source arcs chorded within tolerance/4; whole cut chords cleared with endpoint lift at most tolerance/2".into())});
        out.report.finish();runner.stage("finish-complete","Finishing path generated",groups.len() as u64,groups.len() as u64)?;Ok(out)
    }
}

pub fn polyline(points:&[Point2]) -> Result<Vec<Span>,CamError> {
    if points.len()<2 || !points.iter().all(Point2::is_finite) {return Err(invalid("source polyline needs at least two finite points"));}
    Ok(points.windows(2).map(|p|Span::Line {start:Point3::new(p[0].x,p[0].y,0.),end:Point3::new(p[1].x,p[1].y,0.)}).collect())
}
fn linearize(span:&Span,error:f64) -> Result<Vec<(Point3,Point3)>,CamError> {
    if matches!(span,Span::Line {..}) {return Ok(vec![span.ends()]);}
    let (a,_)=span.ends();let radius=if let Span::Arc {center,..}=span {a.distance_xy(*center)} else {unreachable!()};
    let angle=2.*(1.-(error/radius).min(1.)).acos();
    let steps=(span.length2d()/(radius*angle)).ceil().max(1.);
    if !steps.is_finite() || steps>MAX_SOURCE_SEGMENTS as f64 {return Err(invalid("arc linearization exceeds work budget"));}
    Ok((0..steps as usize).map(|i|(span.point_at(i as f64/steps),span.point_at((i+1) as f64/steps))).collect())
}
