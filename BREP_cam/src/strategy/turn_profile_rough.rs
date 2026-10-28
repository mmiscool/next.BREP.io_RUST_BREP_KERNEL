use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Toolpath,Strategy,Motion,Feed,Point2,Tolerance};
use crate::planar::clip_line;
use crate::turning::{TurningGeometry,profile::TurningProfile,profile_path::*,generate::*};
/// Rough only. Axial cuts stop at the first protected offset; a later free
/// interval is an undercut and refuses rather than silently leaving it. Radial
/// plunges attempt such regions, subject to the complete insert sweep check.
pub fn generate(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,stickout:f64,profile:&TurningProfile,input:&ProfileInput,direction:RoughingDirection,tol:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let params=match &op.strategy {Strategy::TurnProfile(p) if p.roughing && !p.finishing=>p,_=>return Err(invalid("rough profile entry requires roughing only; use profile_finish::generate_combined for both"))};
    let c=context(op,setup,tool,g,stock,profile,input,tol,runner)?;
    let forbidden=rough_forbidden(&c,op,&tol,runner)?;
    let margin=op.tolerance*0.5;
    let mut lines=Vec::new();
    match direction {
        RoughingDirection::Axial=>{
            let from=if c.outside {stock.radius+c.nose} else {input.pilot_radius-c.nose};
            let to=if c.outside {c.min_x+c.nose+op.stock_to_leave.radial+2.*margin} else {c.max_x-c.nose-op.stock_to_leave.radial-2.*margin};
            let span=if c.outside{from-to}else{to-from};
            if span<=tol.point_eps {return Err(invalid("no radial stock remains for axial profile roughing"));}
            let count=(span/params.depth_of_cut).ceil();
            if !count.is_finite() || count>100_000. {return Err(invalid("profile rough pass budget exceeded"));}
            for i in 1..=count as usize {
                let distance=(i as f64*params.depth_of_cut).min(span);let x=from+if c.outside{-distance}else{distance};
                lines.push((Point2::new(x,c.safe_z),Point2::new(x,c.back)));
            }
        }
        RoughingDirection::Radial=>{
            if params.depth_of_cut>2.*c.nose {return Err(invalid("radial profile step exceeds nose diameter and would leave uncut stripes"));}
            let count=((c.front-c.back)/params.depth_of_cut).ceil();
            if !count.is_finite() || count>100_000. {return Err(invalid("profile rough pass budget exceeded"));}
            for i in 0..=count as usize {
                let z=(c.front-i as f64*params.depth_of_cut).max(c.back);
                lines.push((Point2::new(c.clear_x,z),Point2::new(if c.outside{0.}else{stock.radius+c.nose},z)));
            }
        }
    }
    let mut path=c.path;path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));
    for (i,(a,b)) in lines.iter().enumerate() {
        runner.work(forbidden.canonical_segments(&tol).len() as u64,"turn-profile-rough",i as u64,lines.len() as u64)?;
        let intervals=clip_line(&forbidden,*a,*b,&tol);
        let t=intervals.first().map_or(1.,|v|v.0);
        if t<=tol.point_eps {return Err(invalid("rough profile entry is inside protected allowance"));}
        if !intervals.is_empty() {
            let tail=intervals.last().unwrap().1;
            if tail<1.-tol.point_eps || intervals.windows(2).any(|w|w[1].0-w[0].1>tol.point_eps) {return Err(invalid(if direction==RoughingDirection::Axial {"axial profile undercut is unreachable; select radial roughing or another tool"} else {"radial profile obstruction hides material behind the first protected interval"}));}
        }
        if direction==RoughingDirection::Radial && intervals.is_empty() {return Err(invalid("radial profile slice misses selected protected section"));}
        let end=Point2::new(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t);
        if direction==RoughingDirection::Axial {
            path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));
            path.motions.push(Motion::rapid(p3(*a)));
        } else {path.motions.push(Motion::rapid(p3(*a)));}
        path.motions.push(Motion::cut(p3(end),c.feeds.cutting_feed).with_path(i as u32));
        // Reverse the cut to its proven-clear entry before any lateral link.
        path.motions.push(Motion::retract(p3(*a),Feed::RAPID));
        if direction==RoughingDirection::Axial {path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));}
    }
    path.motions.push(Motion::rapid(point(c.clear_x,c.safe_z)));
    path.report.counts.passes=lines.len() as u64;
    finish(path,&c.envelope,profile,stock,stickout,op.tolerance*0.25,runner)
}
