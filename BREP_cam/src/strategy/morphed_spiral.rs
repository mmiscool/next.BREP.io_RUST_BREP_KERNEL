//! Continuous polygon-to-centre spiral for a single convex contained region.
use crate::*;
use crate::drop::finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS};

pub fn source_path(p:&MorphedSpiralParams,error:f64)->Result<Vec<Point2>,CamError> {
    p.stepover.validate().map_err(CamError::InvalidInput)?;p.containment.validate().map_err(CamError::InvalidInput)?;
    if p.containment.boundary.len()!=1 || !error.is_finite() || error<=0. {return Err(invalid("morphed spiral requires one explicit convex boundary and positive tolerance"));}
    let mut polygon=p.containment.boundary[0].clone();
    if polygon.first()==polygon.last() {polygon.pop();}
    if polygon.len()<3 {return Err(invalid("morphed spiral boundary needs three vertices"));}
    let mut winding=0.;
    for (&a,&b) in polygon.iter().zip(polygon.iter().cycle().skip(1)).take(polygon.len()) {
        for v in &polygon {
            let cross=(b.x-a.x)*(v.y-a.y)-(b.y-a.y)*(v.x-a.x);
            if cross!=0. {if winding==0. {winding=cross.signum();} else if winding*cross<0. {return Err(invalid("morphed spiral currently supports convex boundaries without holes"));}}
        }
    }
    if winding==0. {return Err(invalid("morphed spiral boundary has zero area"));}
    let center=Point2::new(polygon.iter().map(|p|p.x).sum::<f64>()/polygon.len() as f64,polygon.iter().map(|p|p.y).sum::<f64>()/polygon.len() as f64);
    let radius=polygon.iter().map(|p|(p.x-center.x).hypot(p.y-center.y)).fold(0.,f64::max);
    let turns=(radius/p.stepover.distance).max(1.);
    if !turns.is_finite() || turns>MAX_SOURCE_SEGMENTS as f64 {return Err(invalid("morphed spiral turn work budget exhausted"));}
    let lengths:Vec<_>=polygon.iter().zip(polygon.iter().cycle().skip(1)).take(polygon.len()).map(|(a,b)|(b.x-a.x).hypot(b.y-a.y)).collect();
    let perimeter=lengths.iter().sum::<f64>();let mut out=vec![polygon[0]];let mut phase=0.;
    while phase<turns {
        for (i,&length) in lengths.iter().enumerate() {
            if phase>=turns {break;}if length==0. {continue;}
            let width=length/perimeter;let dt=width.min(turns-phase);let fraction=dt/width;
            let a=polygon[i];let b=polygon[(i+1)%polygon.len()];
            // Quadratic interpolation deviation: |delta_scale*delta_edge|/4.
            let n=((dt/turns*length*fraction/(4.*error)).sqrt().ceil() as usize).max(1);
            if out.len()+n>MAX_SOURCE_SEGMENTS {return Err(invalid("morphed spiral source work budget exhausted"));}
            for j in 1..=n {let t=j as f64/n as f64;let scale=(1.-(phase+dt*t)/turns).max(0.);
                let x=a.x+(b.x-a.x)*fraction*t;let y=a.y+(b.y-a.y)*fraction*t;
                out.push(Point2::new(center.x+scale*(x-center.x),center.y+scale*(y-center.y)));
            }
            phase+=dt;
        }
    }
    *out.last_mut().unwrap()=center;Ok(out)
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::MorphedSpiral(p)=>p,_=>return Err(invalid("morphed spiral generator requires MorphedSpiral parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    ctx.generate(&p.containment,&[polyline(&source_path(p,operation.tolerance*0.25)?)?],runner)
}
