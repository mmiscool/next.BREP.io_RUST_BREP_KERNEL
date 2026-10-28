//! Conservative revolved silhouette of a mixed-feature body in spindle space.
//! Between mesh Z stations the radial norm of each intersected triangle edge
//! is convex. Joining the largest endpoint radii therefore encloses every
//! azimuth, including flats and cross holes, instead of turning into them.
use crate::{CamError, MeshSnapshot, Point3, Runner};
use super::profile::{ProfileEdge, TurningProfile};

pub fn extract(mesh:&MeshSnapshot,chord:f64,runner:&mut Runner<'_>)->Result<TurningProfile,CamError> {
    if !chord.is_finite() || chord<=0. {return Err(CamError::InvalidInput("turning envelope needs a positive mesh chord".into()));}
    let triangles:Vec<_>=mesh.triangles.iter().filter(|t|mesh.faces.get(t.face as usize).is_some_and(|f|!f.protected_only)).collect();
    let mut stations:Vec<f64>=triangles.iter().flat_map(|t|t.vertices().into_iter().flat_map(|p|[p.z-chord,p.z+chord])).collect();
    if stations.iter().any(|z|!z.is_finite()) {return Err(CamError::InvalidInput("nonfinite turning envelope".into()));}
    stations.sort_by(f64::total_cmp);stations.dedup_by(|a,b|(*a-*b).abs()<1e-10);
    let mut spans:Vec<_>=triangles.iter().map(|t| {
        let v=t.vertices();(*t,v.iter().map(|p|p.z-chord).fold(f64::INFINITY,f64::min),v.iter().map(|p|p.z+chord).fold(f64::NEG_INFINITY,f64::max))
    }).collect();
    spans.sort_by(|a,b|a.1.total_cmp(&b.1));
    let mut cursor=0;let mut active:Vec<usize>=Vec::new();
    let mut outline:Vec<Point3>=Vec::new();
    for band in stations.windows(2) {
        let [lo,hi]=[band[0],band[1]];let middle=lo+(hi-lo)*0.5;
        let mut radii=[0.0_f64;2];let mut found=false;
        while cursor<spans.len() && spans[cursor].1<middle {active.push(cursor);cursor+=1;}
        active.retain(|&i|spans[i].2>middle);
        for &i in &active {
            let triangle=spans[i].0;
            runner.work(1,"turning-envelope",0,0)?;
            let v=triangle.vertices();
            for i in 0..3 {
                let (a,b)=(v[i],v[(i+1)%3]);
                if middle<=a.z.min(b.z)-chord || middle>=a.z.max(b.z)+chord {continue;}
                found=true;
                for (j,z) in [lo,hi].into_iter().enumerate() {
                    // Radial projection is 1-Lipschitz. Expand in Z as well
                    // as radius so the input mesh chord is covered even on
                    // steep shoulders; radial padding alone cannot do that.
                    for sample_z in [z-chord,z+chord] {
                        let r=if a.z==b.z {a.x.hypot(a.y).max(b.x.hypot(b.y))} else {
                            let p=a.lerp(b,((sample_z-a.z)/(b.z-a.z)).clamp(0.,1.));p.x.hypot(p.y)
                        };
                        radii[j]=radii[j].max(r);
                    }
                }
            }
        }
        if !found {continue;}
        for point in [Point3::new(radii[0]+chord,0.,lo),Point3::new(radii[1]+chord,0.,hi)] {
            if outline.last().is_some_and(|p|p.distance(point)<1e-9) {continue;}
            if outline.len()>=2 {
                let a=outline[outline.len()-2];let b=outline[outline.len()-1];
                let ab=b.sub(a);let bc=point.sub(b);
                // Never simplify inward, even when a very small bend meets
                // the collinearity threshold at a tiny requested chord.
                let outside=point.z==a.z || a.x+(point.x-a.x)*(b.z-a.z)/(point.z-a.z)>=b.x;
                if outside && (ab.x*bc.z-ab.z*bc.x).abs()<1e-10 && ab.dot(bc)>=0. {outline.pop();}
            }
            outline.push(point);
        }
    }
    if outline.len()<2 {return Err(CamError::InvalidInput("empty turning envelope".into()));}
    let mut polygon=vec![Point3::new(0.,0.,outline[0].z)];polygon.extend(outline);
    polygon.push(Point3::new(0.,0.,polygon.last().unwrap().z));polygon.push(polygon[0]);
    let edges=polygon.windows(2).filter(|p|p[0].distance(p[1])>1e-10).map(|p|ProfileEdge::Line{start:p[0],end:p[1]}).collect();
    // `chord` is the input mesh approximation, not a claim that the enclosing
    // revolved stock finishes the milled features. Those belong to live ops.
    Ok(TurningProfile{edges,loops:vec![polygon],chord_bound:chord})
}
