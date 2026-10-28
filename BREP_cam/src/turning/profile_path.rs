//! Derived XZ profile selection and C5 planar adapters. No offset engine here.
use crate::{CamError,Operation,Setup,Tool,StockCylinder,Runner,Point2,Point3,Tolerance,Toolpath,FeedsSpeeds,Strategy};
use crate::planar::{Chain,Loop,Region,Segment};
use super::{TurningGeometry,ToolEnvelope,profile::TurningProfile,generate::*};
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum RoughingDirection { Axial, Radial }
/// Derived feature/stock geometry. Chain coordinates are (physical radius,Z),
/// ordered front to back. `pilot_radius` certifies an existing through bore in
/// the otherwise cylindrical stock; zero means solid bar, never inferred bore.
#[derive(Clone,Debug)]
pub struct ProfileInput {pub contour:Chain,pub pilot_radius:f64}
pub(crate) struct Context {
    pub path:Toolpath,pub envelope:ToolEnvelope,pub feeds:FeedsSpeeds,
    pub material:Region,pub contour:Chain,pub nose:f64,pub clear_x:f64,pub safe_z:f64,
    pub front:f64,pub back:f64,pub min_x:f64,pub max_x:f64,pub outside:bool,
}
pub(crate) fn p2(p:Point3)->Point2 {Point2::new(p.x,p.z)}
pub(crate) fn p3(p:Point2)->Point3 {point(p.x,p.y)}
pub fn chain_from_edges(edges:&[super::profile::ProfileEdge])->Chain {
    use super::profile::ProfileEdge;
    Chain::open(edges.iter().map(|e|match *e {
        ProfileEdge::Line{start,end}=>Segment::line(p2(start),p2(end)),
        ProfileEdge::Arc{start,end,centre,clockwise,..}=>Segment::arc(p2(start),p2(end),p2(centre),!clockwise),
    }).collect())
}
pub(crate) fn context(op:&Operation,setup:&Setup,tool:&Tool,g:&TurningGeometry,stock:&StockCylinder,profile:&TurningProfile,input:&ProfileInput,tol:Tolerance,runner:&mut Runner<'_>)->Result<Context,CamError> {
    tol.validate()?;runner.token().check()?;
    let params=match &op.strategy {Strategy::TurnProfile(p)=>p,_=>return Err(invalid("profile turning requires TurnProfile parameters"))};
    if !params.depth_of_cut.is_finite() || params.depth_of_cut<=0. || ![op.stock_to_leave.radial,op.stock_to_leave.axial].iter().all(|v|v.is_finite() && *v>=0.) {return Err(invalid("invalid profile depth or stock allowance"));}
    let (path,envelope,feeds)=prepare(op,setup,tool,g,profile)?;
    super::profile::bar_section(stock,stock.height)?;
    let nose=nose(g)?;
    if nose<=op.tolerance || input.contour.closed || input.contour.segments.is_empty() || input.contour.segments.len()>512 || profile.loops.iter().map(Vec::len).sum::<usize>()>1024 {return Err(invalid("profile needs positive nose radius and a bounded open contour"));}
    let mut contour=input.contour.clone();contour.validate(&tol)?;
    // Extracted meridians explicitly repeat their closure point. C5 loops
    // close implicitly, so omit only exactly zero-length closure/adjacent edges.
    let material=Region::from_unoriented(profile.loops.iter().map(|l| {
        Loop::new(l.iter().zip(l.iter().cycle().skip(1)).take(l.len()).filter(|(a,b)|a!=b).map(|(a,b)|Segment::line(p2(*a),p2(*b))).collect())
    }).collect(),&tol)?;
    let mut min_x=f64::INFINITY;let mut max_x=f64::NEG_INFINITY;
    let mut front=f64::NEG_INFINITY;let mut back=f64::INFINITY;
    for s in &contour.segments {
        let (low,high)=s.bounds();min_x=min_x.min(low.x);max_x=max_x.max(high.x);front=front.max(high.y);back=back.min(low.y);
        if s.length()/op.tolerance>100_000. {return Err(invalid("turning contour sampling budget exceeded"));}
        let points=s.discretize(op.tolerance*0.125);
        for p in points {
            runner.work(1,"turn-profile-selection",0,0)?;
            if p.x<0. || material.nearest_boundary(p).map_or(true,|(_,d)|d>profile.chord_bound+op.tolerance*0.5) {return Err(invalid("selected turning contour is not on the protected profile"));}
        }
        // Canonical arcs are monotone between axis extrema.
        if s.canonical(&tol).iter().any(|c|c.end.y>c.start.y+tol.point_eps) {return Err(invalid("turning contour must run front to back without axial folds"));}
    }
    if front-back<=tol.point_eps {return Err(invalid("profile selection has no axial extent"));}
    if front>stock.base.z+stock.height+tol.point_eps || back<stock.base.z-tol.point_eps || max_x>stock.radius+tol.point_eps {return Err(invalid("selected profile lies outside stock"));}
    let clearance=op.linking.safe_distance.max(op.tolerance);
    let min_tool=envelope.vertices.iter().map(|p|p.x-envelope.reference_to_tip.x).fold(f64::INFINITY,f64::min);
    let max_tool=envelope.vertices.iter().map(|p|p.x-envelope.reference_to_tip.x).fold(f64::NEG_INFINITY,f64::max);
    let min_z=envelope.vertices.iter().map(|p|p.z-envelope.reference_to_tip.z).fold(f64::INFINITY,f64::min);
    let clear_x=if params.outside {stock.radius-min_tool+clearance} else {
        if !input.pilot_radius.is_finite() || input.pilot_radius<=0. || input.pilot_radius>min_x || input.pilot_radius>=stock.radius {return Err(invalid("internal profiling requires a certified existing pilot bore"));}
        let x=input.pilot_radius-max_tool-clearance;
        if x+min_tool < -input.pilot_radius+clearance || x<0. {return Err(invalid("turning tool does not fit pilot bore clearance"));}x
    };
    let safe_z=stock.base.z+stock.height-min_z+clearance;
    Ok(Context{path,envelope,feeds,material,contour,nose,clear_x,safe_z,front,back,min_x,max_x,outside:params.outside})
}
/// Elliptical X/Z rough allowance via affine coordinates around C5's circular
/// offset. Axis-aligned walls retain the requested X or Z allowance; corners
/// blend elliptically. Zero axes receive only the tolerance margin. Ellipse
/// chords are bounded before the final circular nose offset.
pub(crate) fn rough_forbidden(c:&Context,op:&Operation,tol:&Tolerance,runner:&mut Runner<'_>)->Result<Region,CamError> {
    let margin=op.tolerance*0.5;
    let mut region=c.material.clone();
    if op.stock_to_leave.radial>0. || op.stock_to_leave.axial>0. {
        let x=op.stock_to_leave.radial.max(margin);let z=op.stock_to_leave.axial.max(margin);
        let scaled:Vec<_>=region.loops.iter().map(|l|Loop::polygon(&l.discretize(op.tolerance*0.125).iter().map(|p|Point2::new(p.x/x,p.y/z)).collect::<Vec<_>>())).collect();
        let count=scaled.iter().map(|l|l.segments.len()).sum::<usize>();
        runner.work((count*count) as u64,"turn-profile-allowance",0,0)?;
        let scale_tol=Tolerance::scaled_for_diagonal(region.bounds().1.x.abs().max(region.bounds().1.y.abs())/x.min(z));
        let expanded=Region::new(scaled,&scale_tol)?.offset(1.,&scale_tol)?;
        let loops=expanded.loops.iter().map(|l|Loop::polygon(&l.discretize(op.tolerance*0.125/x.max(z)).iter().map(|p|Point2::new(p.x*x,p.y*z)).collect::<Vec<_>>())).collect();
        region=Region::new(loops,tol)?;
    }
    let count=region.loops.iter().map(|l|l.segments.len()).sum::<usize>();
    if count>4096 {return Err(invalid("turning allowance offset budget exceeded"));}
    runner.work((count*count) as u64,"turn-profile-nose-offset",0,0)?;
    let result=region.offset(c.nose+margin,tol)?;runner.token().check()?;Ok(result)
}
