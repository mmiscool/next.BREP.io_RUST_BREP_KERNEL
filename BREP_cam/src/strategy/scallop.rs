//! Constant CL-surface stepover / prescribed cusp for planar, spherical and
//! horizontal cylindrical patches, with CL-grid distance fronts for freeform
//! constant surface stepover. Cusp control requires verified analytic curvature.
use crate::*;
use crate::drop::finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS};

#[derive(Clone,Copy,Debug)]
enum Surface {
    Plane {sx:f64,sy:f64,z0:f64},
    Sphere {center:Point3,radius:f64,concave:bool},
    Cylinder {center:Point3,radius:f64,along_x:bool},
    RimmedCavity {center:Point3,radius:f64,z0:f64},
}

fn solve(rows:&[Vec<f64>],rhs:&[f64])->Option<Vec<f64>> {
    let n=rows.first()?.len();let mut a=vec![vec![0.;n+1];n];
    for (row,&v) in rows.iter().zip(rhs) {for i in 0..n {for j in 0..n {a[i][j]+=row[i]*row[j];}a[i][n]+=row[i]*v;}}
    for i in 0..n {
        let pivot=(i..n).max_by(|&j,&k|a[j][i].abs().total_cmp(&a[k][i].abs()))?;
        if a[pivot][i].abs()<1e-12 {return None;}a.swap(i,pivot);let scale=a[i][i];
        for j in i..=n {a[i][j]/=scale;}
        for k in 0..n {if k==i {continue;}let scale=a[k][i];for j in i..=n {a[k][j]-=scale*a[i][j];}}
    }
    Some(a.iter().map(|row|row[n]).collect())
}
fn fit(ctx:&FinishContext<'_>,runner:&mut Runner<'_>)->Result<Surface,CamError> {
    let mut triangles=Vec::new();let mut points=Vec::new();
    // Fit only facets whose XY projection intersects the containment bounds.
    // Exact separating axes avoid treating an annular flat rim as part of the
    // cavity patch, without guessing the active face from a handful of probes.
    let p=match &ctx.operation.strategy {Strategy::Scallop(p)=>p,_=>unreachable!()};
    let bounds=ctx.bounds(&p.containment)?;
    let overlaps=|t:&Triangle| {
        let corners=[Point2::new(bounds.min.x,bounds.min.y),Point2::new(bounds.max.x,bounds.min.y),Point2::new(bounds.max.x,bounds.max.y),Point2::new(bounds.min.x,bounds.max.y)];
        let vertices=t.vertices();let mut axes=vec![Point2::new(1.,0.),Point2::new(0.,1.)];
        for i in 0..3 {let a=vertices[i];let b=vertices[(i+1)%3];axes.push(Point2::new(a.y-b.y,b.x-a.x));}
        axes.into_iter().all(|n| {
            let project=|x:f64,y:f64|n.x*x+n.y*y;
            let (lo,hi)=vertices.iter().map(|v|project(v.x,v.y)).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),v|(lo.min(v),hi.max(v)));
            let (a,b)=corners.iter().map(|v|project(v.x,v.y)).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),v|(lo.min(v),hi.max(v)));
            lo<=b && a<=hi
        })
    };
    for t in &ctx.mesh.triangles {
        let face=&ctx.mesh.faces[t.face as usize];
        let selected=ctx.operation.targets.drive_faces.is_empty() || ctx.operation.targets.drive_faces.iter().any(|f|f.solid==face.solid && f.face.as_ref().map_or(true,|id|id==&face.face));
        if selected && overlaps(t) && !face.protected_only && ctx.operation.targets.target_solids.contains(&face.solid) && t.area_vector().z>ctx.tolerance.area_eps {
            triangles.push(t);points.extend(t.vertices());
        }
        runner.work(1,"scallop-fit",t.id as u64,ctx.mesh.triangle_count() as u64)?;
    }
    if points.len()<3 {return Err(invalid("scallop requires upward-oriented drive facets"));}
    let origin=Bounds3::from_points(points.iter().copied()).center();let scale=ctx.mesh.bounds.diagonal();
    if !scale.is_finite() || scale<=ctx.tolerance.distance_eps {return Err(invalid("scallop drive surface is degenerate"));}
    let normalized:Vec<_>=points.iter().map(|p|p.sub(origin).scale(1./scale)).collect();
    let eps=ctx.tolerance.distance_eps*4.;
    let rows:Vec<_>=normalized.iter().map(|p|vec![p.x,p.y,1.]).collect();let rhs:Vec<_>=normalized.iter().map(|p|p.z).collect();
    if let Some(p)=solve(&rows,&rhs) {
        if normalized.iter().all(|v|((p[0]*v.x+p[1]*v.y+p[2])-v.z).abs()*scale<=eps) {return Ok(Surface::Plane {sx:p[0],sy:p[1],z0:origin.z+scale*p[2]-p[0]*origin.x-p[1]*origin.y});}
    }
    let mut coarse=None;
    let rows:Vec<_>=normalized.iter().map(|p|vec![2.*p.x,2.*p.y,2.*p.z,1.]).collect();let rhs:Vec<_>=normalized.iter().map(|p|p.length_squared()).collect();
    if let Some(p)=solve(&rows,&rhs) {
        let c=Point3::new(p[0],p[1],p[2]);let radius=(p[3]+c.length_squared()).sqrt()*scale;let center=origin.add(c.scale(scale));
        if radius.is_finite() && points.iter().all(|v|(v.distance(center)-radius).abs()<=eps) {
            match certify_mesh_sag(&triangles,center,radius,None,ctx.operation.tolerance*0.5) {
                Ok(concave)=>return Ok(Surface::Sphere {center,radius,concave}),
                Err(error)=>coarse=Some(error),
            }
        }
    }
    for along_x in [true,false] {
        let rows:Vec<_>=normalized.iter().map(|p|vec![2.*if along_x {p.y}else{p.x},2.*p.z,1.]).collect();
        let rhs:Vec<_>=normalized.iter().map(|p|if along_x {p.y*p.y+p.z*p.z}else{p.x*p.x+p.z*p.z}).collect();
        if let Some(p)=solve(&rows,&rhs) {
            let radius=(p[2]+p[0]*p[0]+p[1]*p[1]).sqrt()*scale;
            let center=if along_x {Point3::new(0.,origin.y+p[0]*scale,origin.z+p[1]*scale)}else{Point3::new(origin.x+p[0]*scale,0.,origin.z+p[1]*scale)};
            if radius.is_finite() && points.iter().all(|v|((if along_x {v.y-center.y}else{v.x-center.x}).hypot(v.z-center.z)-radius).abs()<=eps) {
                match certify_mesh_sag(&triangles,center,radius,Some(along_x),ctx.operation.tolerance*0.5) {
                    Ok(false)=>return Ok(Surface::Cylinder {center,radius,along_x}),
                    Ok(true)=>coarse=Some(invalid("concave cylinder cusp is not supported")),
                    Err(error)=>coarse=Some(error),
                }
            }
        }
    }
    if let Some(error)=coarse {return Err(error);}
    Err(invalid("scallop curvature engine currently supports one plane, sphere or machine-X/Y cylinder; freeform patches require a general surface-offset front"))
}
// A single analytic carrier cannot describe a spherical pocket and its flat
// rim. Fit each selected upward face independently, then accept only the
// compatible horizontal-plane / concave-sphere combination. Other mixtures
// retain the original refusal; no general freeform cusp claim is made.
fn fit_rimmed_cavity(ctx:&FinishContext<'_>,runner:&mut Runner<'_>)->Result<Surface,CamError> {
    let mut plane=None;let mut sphere=None;
    for (id,face) in ctx.mesh.faces.iter().enumerate() {
        if face.protected_only || !ctx.operation.targets.target_solids.contains(&face.solid) ||
            (!ctx.operation.targets.drive_faces.is_empty() && !ctx.operation.targets.drive_faces.iter().any(|f|f.solid==face.solid&&f.face.as_ref().map_or(true,|v|v==&face.face))) ||
            !ctx.mesh.triangles.iter().any(|t|t.face as usize==id&&t.area_vector().z>ctx.tolerance.area_eps) {continue;}
        let mut operation=ctx.operation.clone();operation.targets.drive_faces=vec![TargetRef::face(&face.solid,&face.face)];
        let local=FinishContext{operation:&operation,..*ctx};
        match fit(&local,runner)? {
            Surface::Plane{sx,sy,z0} if sx.abs()<ctx.tolerance.angle_eps&&sy.abs()<ctx.tolerance.angle_eps=>{
                if plane.is_some_and(|z:f64|(z-z0).abs()>ctx.tolerance.distance_eps*4.) {return Err(invalid("scallop mixed cavity has multiple rim heights"));}plane=Some(z0);
            },
            Surface::Sphere{center,radius,concave:true}=>{
                if sphere.is_some_and(|(c,r):(Point3,f64)|c.distance(center)>ctx.tolerance.distance_eps*4.||(r-radius).abs()>ctx.tolerance.distance_eps*4.) {return Err(invalid("scallop mixed cavity has multiple spherical carriers"));}sphere=Some((center,radius));
            },
            _=>return Err(invalid("scallop mixed cusp requires a horizontal rim and one concave sphere")),
        }
    }
    let (Some(z0),Some((center,radius)))=(plane,sphere) else{return Err(invalid("scallop mixed cusp requires both rim and cavity"));};
    if z0<=center.z-radius || z0>=center.z-ctx.tolerance.distance_eps || radius<=ctx.cutter.radius() {return Err(invalid("scallop mixed cavity must lie below its sphere equator and fit the ball"));}
    Ok(Surface::RimmedCavity{center,radius,z0})
}
fn cavity_tip(rho:f64,center:Point3,radius:f64,z0:f64,ball:f64)->f64 {
    let opening=(radius*radius-(center.z-z0).powi(2)).sqrt();
    let transition=opening*(radius-ball)/radius;
    if rho<transition {center.z-ball-((radius-ball).powi(2)-rho*rho).max(0.).sqrt()}
    else if rho<opening {z0-ball+(ball*ball-(opening-rho).powi(2)).max(0.).sqrt()}
    else {z0}
}
// Bound the complete radial residual between rings, not just midpoint cusp.
// On each interval the spherical target is monotone increasing. For a cutter
// ring, its largest lower-envelope height is at the farther endpoint. Thus
// cutter_max - target_min bounds every point of that interval. Subdivision
// only tightens this bound; failure at the cap refuses the requested cusp.
fn certify_cavity_rings(rings:&[f64],max:f64,center:Point3,radius:f64,z0:f64,ball:f64,cusp:f64,runner:&mut Runner<'_>)->Result<(),CamError> {
    let opening=(radius*radius-(center.z-z0).powi(2)).sqrt();
    let target=|q:f64|if q<opening{center.z-(radius*radius-q*q).sqrt()}else{z0};
    let mut stack=vec![(0.,max,0)];
    while let Some((lo,hi,depth))=stack.pop(){
        runner.work(1,"scallop-radial-cusp",0,0)?;
        let bound=rings.iter().filter_map(|&q|{let d=(lo-q).abs().max((hi-q).abs());if d>=ball{None}else{Some(cavity_tip(q,center,radius,z0,ball)+ball-(ball*ball-d*d).sqrt()-target(lo))}}).fold(f64::INFINITY,f64::min);
        if bound<=cusp {continue;}
        if depth>=20 {return Err(invalid("mixed scallop radial cusp could not be certified"));}
        let mid=(lo+hi)*0.5;stack.push((lo,mid,depth+1));stack.push((mid,hi,depth+1));
    }Ok(())
}
fn certify_mesh_sag(triangles:&[&Triangle],center:Point3,radius:f64,axis:Option<bool>,limit:f64)->Result<bool,CamError> {
    let mut concave=None;
    for t in triangles {
        let n=t.area_vector().normalized().ok_or_else(||invalid("degenerate scallop facet"))?;
        let inward=n.dot(t.a.sub(center))<0.;
        if concave.is_some_and(|v|v!=inward) {return Err(invalid("analytic scallop patch has inconsistent orientation"));}concave=Some(inward);
        let distance=if let Some(along_x)=axis {
            let radial=if along_x {Point3::new(0.,n.y,n.z)}else{Point3::new(n.x,0.,n.z)};
            let norm=radial.length();if norm==0. {continue;}radial.dot(t.a.sub(center)).abs()/norm
        }else {n.dot(t.a.sub(center)).abs()};
        if radius-distance>limit {return Err(invalid(&format!("drive mesh sag {} exceeds scallop mesh budget {limit}; retessellate more finely",radius-distance)));}
    }
    Ok(concave.unwrap_or(false))
}
fn plane_step(tool:f64,cusp:Option<f64>,step:f64)->Result<f64,CamError> {
    if let Some(h)=cusp {
        if !h.is_finite() || h<=0. || h>=tool {return Err(invalid("cusp height must lie strictly between zero and ball radius"));}
        Ok(2.*(2.*tool*h-h*h).sqrt())
    }else if step>0. && step<2.*tool {Ok(step)}else {Err(invalid("CL stepover must be smaller than the ball diameter"))}
}
fn round_step(part:f64,tool:f64,cusp:Option<f64>,step:f64)->Result<f64,CamError> {
    let s=part+tool;
    if let Some(h)=cusp {
        plane_step(tool,Some(h),step)?;
        let max_cusp=2.*part*tool/((part*part+2.*part*tool).sqrt()+part);
        if h>=max_cusp {return Err(invalid("requested cusp exceeds the neighboring cutter-sphere intersection limit"));}
        // Distance from either sphere centre to the cusp point at R+h is r.
        let cos=(s*s+(part+h).powi(2)-tool*tool)/(2.*s*(part+h));
        let result=2.*s*cos.clamp(-1.,1.).acos();
        if result<=0. {return Err(invalid("cusp height is below numeric resolution"));}Ok(result)
    }else {plane_step(tool,None,step)}
}
fn concave_step(part:f64,tool:f64,cusp:Option<f64>,step:f64)->Result<f64,CamError> {
    let s=part-tool;if s<=0. {return Err(invalid("ball does not fit inside concave spherical patch"));}
    if let Some(h)=cusp {
        plane_step(tool,Some(h),step)?;
        let cos=(s*s+(part-h).powi(2)-tool*tool)/(2.*s*(part-h));
        let value=2.*s*cos.clamp(-1.,1.).acos();
        if value<=0. {return Err(invalid("concave cusp is below numeric resolution"));}Ok(value)
    }else {plane_step(tool,None,step)}
}
fn circle(center:Point2,rx:f64,ry:f64,ux:Point2,uy:Point2,error:f64)->Result<Vec<Point2>,CamError> {
    let radius=rx.max(ry);let step=(8.*error/radius).sqrt().min(std::f64::consts::FRAC_PI_8);
    let n=(std::f64::consts::TAU/step).ceil().max(8.);
    if !n.is_finite() || n>MAX_SOURCE_SEGMENTS as f64 {return Err(invalid("scallop ring work budget exceeded"));}
    let mut out:Vec<_>=(0..n as usize).map(|i|{let a=std::f64::consts::TAU*i as f64/n;Point2::new(center.x+rx*a.cos()*ux.x+ry*a.sin()*uy.x,center.y+rx*a.cos()*ux.y+ry*a.sin()*uy.y)}).collect();out.push(out[0]);Ok(out)
}
fn source(ctx:&FinishContext<'_>,p:&ScallopParams,surface:Surface,cusp:Option<f64>,runner:&mut Runner<'_>)->Result<Vec<Vec<crate::drop::path::Span>>,CamError> {
    let bounds=ctx.bounds(&p.containment)?;let region=ctx.region(&p.containment)?;let r=ctx.cutter.radius();let err=ctx.operation.tolerance*0.125;
    let mut curves=Vec::new();
    match surface {
        Surface::Plane {sx,sy,..}=>{
            let center=bounds.center();let norm=(1.+sx*sx+sy*sy).sqrt();
            let u=Point3::new(1.,0.,sx).normalized().unwrap();let n=Point3::new(-sx,-sy,1.).scale(1./norm);let v=n.cross(u);
            let max=[bounds.min.x,bounds.max.x].iter().flat_map(|x|[bounds.min.y,bounds.max.y].map(|y|{
                let d=Point3::new(x-center.x,y-center.y,sx*(x-center.x)+sy*(y-center.y));d.length()
            })).fold(0.,f64::max);
            let step=plane_step(r,cusp,p.stepover.distance)?;let count=(max/step+0.5).ceil() as usize;
            if count>MAX_SOURCE_SEGMENTS {return Err(invalid("scallop pass work budget exceeded"));}
            for i in 0..count {let radius=(i as f64+0.5)*step;
                curves.push(circle(Point2::new(center.x,center.y),radius,radius,Point2::new(u.x,u.y),Point2::new(v.x,v.y),err)?);
            }
        }
        Surface::Sphere {center,radius,concave}=>{
            let s=if concave {radius-r}else{radius+r};
            if s<=0. {return Err(invalid("ball does not fit inside concave spherical patch"));}
            let step=if concave {concave_step(radius,r,cusp,p.stepover.distance)?}else{round_step(radius,r,cusp,p.stepover.distance)?};
            let max=[bounds.min.x,bounds.max.x].iter().flat_map(|x|[bounds.min.y,bounds.max.y].map(|y|(x-center.x).hypot(y-center.y))).fold(0.,f64::max);
            if max>=s {return Err(invalid("scallop sphere containment reaches beyond the upper CL hemisphere"));}
            let max_angle=(max/s).asin();let delta=step/s;let count=(max_angle/delta+0.5).ceil() as usize;
            if count>MAX_SOURCE_SEGMENTS {return Err(invalid("scallop pass work budget exceeded"));}
            for i in 0..count {let angle=(i as f64+0.5)*delta;if angle>=std::f64::consts::FRAC_PI_2 {break;}
                let rho=s*angle.sin();curves.push(circle(Point2::new(center.x,center.y),rho,rho,Point2::new(1.,0.),Point2::new(0.,1.),err)?);
            }
        }
        Surface::RimmedCavity {center,radius,z0}=>{
            let opening=(radius*radius-(center.z-z0).powi(2)).sqrt();
            let slope=opening/(center.z-z0);let stretch=(1.+slope*slope).sqrt();
            let h=cusp.ok_or_else(||invalid("mixed cavity requires an explicit cusp budget"))?;
            let step=plane_step(r,Some(h*0.25/stretch),p.stepover.distance)?/stretch;
            let max=[bounds.min.x,bounds.max.x].iter().flat_map(|x|[bounds.min.y,bounds.max.y].map(|y|(x-center.x).hypot(y-center.y))).fold(0.,f64::max);
            let count=(max/step+0.5).ceil() as usize;
            if count>MAX_SOURCE_SEGMENTS {return Err(invalid("mixed scallop ring budget exceeded"));}
            let rings:Vec<_>=(0..count).map(|i|(i as f64+0.5)*step).collect();
            certify_cavity_rings(&rings,max,center,radius,z0,r,h*0.5,runner)?;
            for rho in rings {curves.push(circle(Point2::new(center.x,center.y),rho,rho,Point2::new(1.,0.),Point2::new(0.,1.),err.min(h*0.125/stretch))?);}
        }
        Surface::Cylinder {center,radius,along_x}=>{
            let s=radius+r;let step=round_step(radius,r,cusp,p.stepover.distance)?;let delta=step/s;
            let (lo,hi,c)=if along_x {(bounds.min.y,bounds.max.y,center.y)}else{(bounds.min.x,bounds.max.x,center.x)};
            if (lo-c).abs()>=s || (hi-c).abs()>=s {return Err(invalid("scallop cylinder containment exceeds the upper CL half-cylinder"));}
            let lo=((lo-c)/s).asin();let hi=((hi-c)/s).asin();let count=((hi-lo)/delta).ceil() as usize;
            if count>MAX_SOURCE_SEGMENTS {return Err(invalid("scallop pass work budget exceeded"));}
            for i in 0..=count {let angle=(lo+i as f64*delta).min(hi);let value=c+s*angle.sin();
                curves.push(if along_x {vec![Point2::new(bounds.min.x,value),Point2::new(bounds.max.x,value)]}else{vec![Point2::new(value,bounds.min.y),Point2::new(value,bounds.max.y)]});
            }
        }
    }
    if !p.inside_out {curves.reverse();}
    let mut out=Vec::new();let mut segments=0;
    for (id,mut curve) in curves.into_iter().enumerate() {
        if p.stepover.direction==CutDirection::Conventional || (p.stepover.direction==CutDirection::Either && id%2==1) {curve.reverse();}
        let spans=polyline(&curve)?;let mut nonempty=false;
        for span in &spans {let (a,b)=span.ends();if !region.clip(a,b,runner)?.is_empty() {nonempty=true;break;}}
        if nonempty {segments+=spans.len();if segments>MAX_SOURCE_SEGMENTS {return Err(invalid("scallop source segment budget exceeded"));}out.push(spans);}
    }
    if out.is_empty() {return Err(invalid("scallop region contains no reachable pass"));}Ok(out)
}

pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Scallop(p)=>p,_=>return Err(invalid("scallop generator requires Scallop parameters"))};
    generate_with_cusp(operation,cutter,mesh,index,heights,feeds,tolerance,runner,p.cusp_height)
}
/// Direct override API; generate() uses the persisted ScallopParams.cusp_height.
pub fn generate_with_cusp(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,cusp:Option<f64>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::Scallop(p)=>p,_=>return Err(invalid("scallop generator requires Scallop parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    if cutter.kind()!=CutterKind::Ball {return Err(invalid("scallop surface spacing currently requires a ball cutter"));}
    if operation.stock_to_leave.radial!=0. || operation.stock_to_leave.axial!=0. {return Err(invalid("scallop surface spacing currently requires zero stock allowance"));}
    if cusp.is_none() {plane_step(cutter.radius(),None,p.stepover.distance)?;}
    let surface=match fit(&ctx,runner) {
        Ok(surface)=>surface,
        Err(CamError::Cancelled)=>return Err(CamError::Cancelled),
        Err(error)=>{
            if cusp.is_some() {
                match fit_rimmed_cavity(&ctx,runner) {Ok(surface)=>surface,Err(CamError::Cancelled)=>return Err(CamError::Cancelled),Err(_)=>return Err(error)}
            } else {return crate::drop::geodesic::generate(&ctx,p,runner); }
        }
    };
    let paths=source(&ctx,p,surface,cusp,runner)?;
    let mut result=ctx.generate(&p.containment,&paths,runner)?;
    for motion in &result.motions {
        if let MotionKind::Cut {to}|MotionKind::Plunge {to}=motion.kind {
            let r=cutter.radius();
            let expected=match surface {
                Surface::Plane {sx,sy,z0}=>sx*to.x+sy*to.y+z0+r*((1.+sx*sx+sy*sy).sqrt()-1.),
                Surface::Sphere {center,radius,concave}=>{
                    let s=if concave {radius-r}else{radius+r};let sign=if concave {-1.}else{1.};
                    sign*(s*s-(to.x-center.x).powi(2)-(to.y-center.y).powi(2)).sqrt()+center.z-r
                },
                Surface::RimmedCavity {center,radius,z0}=>cavity_tip((to.x-center.x).hypot(to.y-center.y),center,radius,z0,r),
                Surface::Cylinder {center,radius,along_x}=>((radius+r).powi(2)-(if along_x {to.y-center.y}else{to.x-center.x}).powi(2)).sqrt()+center.z-r,
            };
            if !expected.is_finite() || (to.z-expected).abs()>operation.tolerance {
                return Err(invalid(&format!("protected geometry or projection prevented the requested analytic scallop surface at {to:?}: expected Z {expected}, surface {surface:?}")));
            }
        }
    }
    if matches!(surface,Surface::RimmedCavity{..}) {
        result.report.approximations.push(ApproximationLimit{kind:"mixed-cavity-radial-cusp".into(),bound:cusp.map(|h|h*0.5),detail:Some("Interval-certified ideal circular-ring radial residual; polygonization and mesh/drop projection retain the separate operation-tolerance budget".into())});
    }
    result.report.approximations.push(ApproximationLimit {kind:"verified-analytic-scallop-surface".into(),bound:Some(operation.tolerance),detail:Some(format!("Surface {surface:?}; CL-surface step {}; requested cusp {cusp:?}; mesh sag <= tolerance/2",p.stepover.distance))});
    Ok(result)
}
