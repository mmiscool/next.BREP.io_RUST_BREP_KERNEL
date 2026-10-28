//! Radial finishing: deterministic spoke paths, clipped and fully protected.
use crate::*;
use crate::drop::{finish::{FinishContext,invalid,MAX_SOURCE_SEGMENTS},path::Span};

pub fn source_paths(p:&RadialParams,bounds:Bounds3)->Result<Vec<Vec<Span>>,CamError> {
    if !p.center.is_finite() || !p.inner_radius.is_finite() || p.inner_radius<0. || !p.angular_step_deg.is_finite() || p.angular_step_deg<=0. || p.angular_step_deg>360. {
        return Err(invalid("radial needs a finite center, nonnegative inner radius and angular step in (0,360]"));
    }
    let outer=p.outer_radius.unwrap_or_else(|| [bounds.min.x,bounds.max.x].iter().flat_map(|x|[bounds.min.y,bounds.max.y].map(|y|(x-p.center.x).hypot(y-p.center.y))).fold(0.,f64::max));
    if !outer.is_finite() || outer<=p.inner_radius {return Err(invalid("radial outer radius must exceed inner radius"));}
    let n=(360./p.angular_step_deg).ceil();if n>MAX_SOURCE_SEGMENTS as f64 {return Err(invalid("radial pass work budget exceeded"));}
    Ok((0..n as usize).map(|i|{
        let angle=(i as f64*p.angular_step_deg).to_radians();let (s,c)=angle.sin_cos();
        vec![Span::Line {start:Point3::new(p.center.x+p.inner_radius*c,p.center.y+p.inner_radius*s,0.),end:Point3::new(p.center.x+outer*c,p.center.y+outer*s,0.)}]
    }).collect())
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Radial(p)=>p,_=>return Err(invalid("radial generator requires Radial parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};
    let paths=source_paths(p,ctx.bounds(&p.containment)?)?;
    ctx.generate(&p.containment,&paths,runner)
}
