//! Exact axial carriers with tessellated trim bounds; no generic offsetting.
use crate::{CamError,MeshSnapshot,Point3,Runner,StockCylinder};
use serde::{Serialize,Deserialize};
use std::f64::consts::TAU;
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub enum AxialCarrier {Plane{z:f64},Line{a:Point3,b:Point3},Circle{centre:Point3,radius:f64}}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct AxialFace {pub solid:String,pub face:String,pub carrier:AxialCarrier}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub enum ProfileEdge {Line{start:Point3,end:Point3},Arc{start:Point3,end:Point3,centre:Point3,radius:f64,clockwise:bool}}
impl ProfileEdge {
    pub fn start(&self)->Point3 {match *self {Self::Line{start,..}|Self::Arc{start,..}=>start}}
    pub fn end(&self)->Point3 {match *self {Self::Line{end,..}|Self::Arc{end,..}=>end}}
    pub fn reversed(&self)->Self {match *self {Self::Line{start,end}=>Self::Line{start:end,end:start},Self::Arc{start,end,centre,radius,clockwise}=>Self::Arc{start:end,end:start,centre,radius,clockwise:!clockwise}}}
    pub fn sample(&self,chord:f64)->Result<Vec<Point3>,CamError> {
        if !chord.is_finite() || chord<=0. {return Err(invalid("profile chord must be positive"));}
        match *self {
            Self::Line{start,end}=>Ok(vec![start,end]),
            Self::Arc{start,end,centre,radius,clockwise}=>{
                if !radius.is_finite() || radius<=0. {return Err(invalid("invalid profile arc radius"));}
                let a=(start.z-centre.z).atan2(start.x-centre.x);let b=(end.z-centre.z).atan2(end.x-centre.x);
                let sweep=if clockwise {(a-b).rem_euclid(TAU)} else {(b-a).rem_euclid(TAU)};
                let count=(sweep/(2.*(1.-(chord/radius).min(1.)).acos())).ceil().max(1.);
                if !count.is_finite() || count>100_000. {return Err(invalid("profile arc work budget exceeded"));}
                let mut points=Vec::new();
                for i in 0..=count as usize {let t=a+if clockwise{-1.}else{1.}*sweep*i as f64/count;points.push(Point3::new(centre.x+radius*t.cos(),0.,centre.z+radius*t.sin()));}
                points[0]=start;*points.last_mut().unwrap()=end;Ok(points)
            }
        }
    }
}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
pub struct TurningProfile {pub edges:Vec<ProfileEdge>,pub loops:Vec<Vec<Point3>>,pub chord_bound:f64}
impl TurningProfile {
    pub fn section_area(&self)->f64 {self.loops.iter().map(|p|p.iter().zip(p.iter().cycle().skip(1)).take(p.len()).map(|(a,b)|a.x*b.z-b.x*a.z).sum::<f64>().abs()*0.5).sum()}
}
#[derive(Clone,Debug,PartialEq)]
pub struct BarSection {pub polygon:Vec<Point3>,pub chuck_z:f64}
/// Shared stock cylinder, validated on spindle Z; stick-out is measured back
/// from its front end, so the clamped portion is explicit collision geometry.
pub fn bar_section(stock:&StockCylinder,stick_out:f64)->Result<BarSection,CamError> {
    if !stock.base.is_finite() || stock.base.x!=0. || stock.base.y!=0. || !stock.axis.is_finite() || stock.axis.distance(Point3::Z)>1e-9 || !stock.radius.is_finite() || stock.radius<=0. || !stock.height.is_finite() || stock.height<=0. || !stick_out.is_finite() || stick_out<=0. || stick_out>stock.height {return Err(invalid("lathe bar must be coaxial, finite, and have valid stick-out"));}
    let front=stock.base.z+stock.height;
    if !front.is_finite() {return Err(invalid("lathe bar extent overflow"));}
    Ok(BarSection{polygon:vec![Point3::new(0.,0.,stock.base.z),Point3::new(stock.radius,0.,stock.base.z),Point3::new(stock.radius,0.,front),Point3::new(0.,0.,front)],chuck_z:front-stick_out})
}
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}

/// Extract the upper (X>=0) meridian. All faces must have an adapter-certified
/// coaxial analytic carrier. A cap must have one radial interval. Complex trims
/// that cannot join into closed sections refuse with a face/section diagnostic.
pub fn extract(carriers:&[AxialFace],mesh:&MeshSnapshot,chord:f64,runner:&mut Runner<'_>)->Result<TurningProfile,CamError> {
    runner.token().check()?;
    if !chord.is_finite() || chord<=0. || carriers.is_empty() {return Err(invalid("turning extraction needs carriers and positive chord"));}
    let mut edges=Vec::new();let mut edge_solids=Vec::new();
    for f in &mesh.faces {if !f.protected_only && !carriers.iter().any(|c|c.solid==f.solid && c.face==f.face) {return Err(invalid(&format!("non-axisymmetric or unsupported face {}/{}",f.solid,f.face)));}}
    for c in carriers {
        let before=edges.len();
        let mut low=f64::INFINITY;let mut high=f64::NEG_INFINITY;let mut min_r=f64::INFINITY;let mut max_r=f64::NEG_INFINITY;
        let mut axial_intervals=Vec::new();
        for t in &mesh.triangles {
            runner.work(1,"turning-profile",0,0)?;
            if mesh.faces.get(t.face as usize).map_or(false,|f|f.solid==c.solid && f.face==c.face && !f.protected_only) {
                let vertices=t.vertices();
                let a=vertices.iter().map(|p|p.z).fold(f64::INFINITY,f64::min);
                let b=vertices.iter().map(|p|p.z).fold(f64::NEG_INFINITY,f64::max);
                axial_intervals.push((a,b));
                for p in vertices {low=low.min(p.z);high=high.max(p.z);let r=p.x.hypot(p.y);min_r=min_r.min(r);max_r=max_r.max(r);}
            }
        }
        if !low.is_finite() {return Err(invalid(&format!("missing turning face {}/{}",c.solid,c.face)));}
        match c.carrier {
            AxialCarrier::Plane{z}=>{
                if (high-low).abs()>chord || !z.is_finite() {return Err(invalid(&format!("nonplanar cap {}/{}",c.solid,c.face)));}
                // Cap triangulations need not contain an axial vertex. Intersect
                // filled triangles with Y=0 instead of taking vertex radii.
                let mut intervals=Vec::new();
                for t in &mesh.triangles {
                    runner.work(1,"turning-cap-section",0,0)?;
                    if !mesh.faces.get(t.face as usize).map_or(false,|f|f.solid==c.solid && f.face==c.face) {continue;}
                    let vertices=t.vertices();let mut hits=Vec::new();
                    for i in 0..3 {let a=vertices[i];let b=vertices[(i+1)%3];
                        if a.y.abs()<=chord*0.001 {hits.push(a.x);}
                        if (a.y<0. && b.y>0.) || (a.y>0. && b.y<0.) {hits.push(a.x+(b.x-a.x)*(-a.y)/(b.y-a.y));}
                    }
                    if hits.len()>=2 {let a=hits.iter().copied().fold(f64::INFINITY,f64::min).max(0.);let b=hits.iter().copied().fold(f64::NEG_INFINITY,f64::max);if b>a {intervals.push((a,b));}}
                }
                intervals.sort_by(|a,b|a.0.total_cmp(&b.0));
                let &(start,mut end)=intervals.first().ok_or_else(||invalid("empty turning cap section"))?;
                for &(a,b) in &intervals {if a>end+chord {return Err(invalid(&format!("disconnected cap intervals {}/{}",c.solid,c.face)));}end=end.max(b);}
                edges.push(ProfileEdge::Line{start:Point3::new(start,0.,z),end:Point3::new(end,0.,z)});
            }
            AxialCarrier::Line{a,b}=>{
                if !a.is_finite() || !b.is_finite() || (b.z-a.z).abs()<=1e-12 {return Err(invalid("invalid axial line carrier"));}
                let radius=|z|a.x+(b.x-a.x)*(z-a.z)/(b.z-a.z);
                // A boolean can leave disconnected axial bands on one carrier.
                // Taking only the min/max Z bridges an annular groove.
                axial_intervals.sort_by(|a,b|a.0.total_cmp(&b.0));
                let mut bands:Vec<(f64,f64)>=Vec::new();
                for (a,b) in axial_intervals {
                    if let Some(last)=bands.last_mut().filter(|last|a<=last.1+chord*0.001) {last.1=last.1.max(b);}else{bands.push((a,b));}
                }
                for (a,b) in bands {if b>a {edges.push(ProfileEdge::Line{start:Point3::new(radius(a),0.,a),end:Point3::new(radius(b),0.,b)});}}
            }
            AxialCarrier::Circle{centre,radius}=>{
                if !centre.is_finite() || !radius.is_finite() || radius<=0. || centre.x<0. || (centre.x>0. && centre.x<=radius) || low<centre.z-radius-chord || high>centre.z+radius+chord {return Err(invalid("invalid axial circle carrier"));}
                for sign in [1.,-1.] {
                    if sign<0. && (centre.x==0. || min_r>=centre.x-chord) {continue;}
                    if sign>0. && max_r<=centre.x+chord {continue;}
                    let point=|z:f64|Point3::new(centre.x+sign*(radius*radius-(z-centre.z).powi(2)).max(0.).sqrt(),0.,z);
                    edges.push(ProfileEdge::Arc{start:point(low.max(centre.z-radius)),end:point(high.min(centre.z+radius)),centre,radius,clockwise:sign<0.});
                }
            }
        }
        edge_solids.extend(std::iter::repeat_n(c.solid.clone(),edges.len()-before));
    }
    // Close material intervals on the spindle before walking any loop. Waiting
    // until the seed and current endpoint both lie on-axis makes closure depend
    // on which face happened to seed traversal (a groove often seeds off-axis).
    let mut poles=std::collections::BTreeMap::<String,Vec<Point3>>::new();
    for (edge,solid) in edges.iter().zip(&edge_solids) {
        for p in [edge.start(),edge.end()] {if p.x.abs()<=chord {poles.entry(solid.clone()).or_default().push(p);}}
    }
    for (solid,mut points) in poles {
        points.sort_by(|a,b|a.z.total_cmp(&b.z));let mut odd=Vec::new();let mut i=0;
        while i<points.len() {let mut j=i+1;while j<points.len() && points[j].distance(points[i])<=chord {j+=1;}
            if (j-i)%2==1 {odd.push(points[i]);}i=j;
        }
        if odd.len()%2!=0 {return Err(invalid(&format!("unpaired turning spindle endpoint on {solid}")));}
        for pair in odd.chunks_exact(2) {edges.push(ProfileEdge::Line{start:pair[0],end:pair[1]});}
    }
    let mut unused:Vec<usize>=(0..edges.len()).collect();let mut loops=Vec::new();let mut ordered=Vec::new();
    while let Some(seed)=unused.pop() {
        let first=edges[seed].clone();let start=first.start();let mut end=first.end();let mut points=first.sample(chord)?;ordered.push(first);
        loop {
            runner.work(1,"turning-profile-chain",0,0)?;
            if end.distance(start)<=chord {break;}
            let matches:Vec<_>=unused.iter().enumerate().filter_map(|(j,&i)| {
                if edges[i].start().distance(end)<=chord {Some((j,false))} else if edges[i].end().distance(end)<=chord {Some((j,true))} else {None}
            }).collect();
            if matches.is_empty() {
                return Err(invalid(&format!("turning face trims do not form closed meridian sections at {end:?}")));
            }
            if matches.len()>1 {return Err(invalid("ambiguous turning section junction"));}
            let (j,reverse)=matches[0];let i=unused.remove(j);let edge=if reverse{edges[i].reversed()}else{edges[i].clone()};
            let samples=edge.sample(chord)?;points.extend(samples.into_iter().skip(1));end=edge.end();ordered.push(edge);
        }
        if points.len()<3 {return Err(invalid("degenerate turning section"));}
        loops.push(points);
    }
    runner.token().check()?;Ok(TurningProfile{edges:ordered,loops,chord_bound:chord})
}

/// Resolve a shared bar or solid-stock snapshot into its turning section.
/// Mesh stock requires the same adapter certification as the finished part;
/// unresolved rest stock must first be supplied by simulation as a snapshot.
pub fn stock_profile(stock:&crate::ResolvedStock,carriers:&[AxialFace],stickout:f64,chord:f64,runner:&mut Runner<'_>)->Result<(TurningProfile,f64),CamError> {
    match stock {
        crate::ResolvedStock::Cylinder(c)=>{
            let bar=bar_section(c,stickout)?;
            let edges=bar.polygon.iter().zip(bar.polygon.iter().cycle().skip(1)).take(bar.polygon.len()).map(|(a,b)|ProfileEdge::Line{start:*a,end:*b}).collect();
            Ok((TurningProfile{edges,loops:vec![bar.polygon],chord_bound:0.},bar.chuck_z))
        }
        crate::ResolvedStock::Mesh(mesh)=>{
            let profile=extract(carriers,mesh,chord,runner)?;
            let (back,front)=profile.loops.iter().flatten().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),p|(a.min(p.z),b.max(p.z)));
            if !stickout.is_finite() || stickout<=0. || stickout>front-back {return Err(invalid("invalid solid-stock stick-out"));}
            Ok((profile,front-stickout))
        }
        _=>Err(invalid("turning stock needs a coaxial cylinder or certified solid snapshot")),
    }
}
