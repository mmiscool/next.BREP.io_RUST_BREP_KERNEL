//! Certified straight convex plane–plane network strips; other geometry marches.
use super::*;


fn enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_ANALYTIC_NETWORK_CYLINDER").ok().as_deref() == Some("1"))
}

fn finite(point: Vec3) -> bool { point.x.is_finite() && point.y.is_finite() && point.z.is_finite() }

struct PlaneChart {
    origin: Vec3,
    u: Vec3,
    v: Vec3,
    twist: Vec3,
    normal: Vec3,
    normal_length: f64,
    gram: [f64; 3],
    determinant: f64,
    domain_u: [f64;2],
    domain_v: [f64;2],
}

impl PlaneChart {
    fn new(surface: &NurbsSurface, bar: f64) -> Option<Self> {
        if surface.degree_u != 1 || surface.degree_v != 1
            || surface.knots_u.len()!=4 || surface.knots_v.len()!=4
            || surface.control_points.len() != 2 || surface.control_points.iter().any(|row| row.len() != 2)
            || surface.control_points.iter().flatten().any(|p| p.w != 1.0 || !finite(Vec3::new(p.x,p.y,p.z)))
        { return None; }
        let [u0,u1]=surface.domain_u().ok()?;
        let [v0,v1]=surface.domain_v().ok()?;
        if ![u0,u1,v0,v1].iter().all(|v| v.is_finite()) || !(u1>u0 && v1>v0)
            || surface.knots_u != [u0,u0,u1,u1] || surface.knots_v != [v0,v0,v1,v1] { return None; }
        let domain_u=[u0,u1];let domain_v=[v0,v1];
        let point = |i: usize,j: usize| { let p=surface.control_points[i][j]; Vec3::new(p.x,p.y,p.z) };
        let origin=point(0,0);
        let u=point(1,0).sub(origin);
        let v=point(0,1).sub(origin);
        let twist=point(1,1).sub(origin).sub(u).sub(v);
        // Positive unit-weight bilinear bases bound the entire stored chart.
        if !finite(twist) || !(twist.length() <= bar) { return None; }
        let gram=[u.dot(u),u.dot(v),v.dot(v)];
        let determinant=gram[0]*gram[2]-gram[1]*gram[1];
        if !determinant.is_finite() || !(determinant > 1e-24*gram[0]*gram[2]) { return None; }
        let cross=u.cross(v);
        let normal_length=cross.length();
        let normal=cross.normalized().ok()?;
        if !normal_length.is_finite() || !finite(normal) { return None; }
        Some(Self { origin,u,v,twist,normal,normal_length,gram,determinant,domain_u,domain_v })
    }

    fn uv(&self, point: Vec3) -> Option<Vec3> {
        let delta=point.sub(self.origin);
        let (a,b)=(delta.dot(self.u),delta.dot(self.v));
        let uv=Vec3::new((self.gram[2]*a-self.gram[1]*b)/self.determinant,
                        (self.gram[0]*b-self.gram[1]*a)/self.determinant,0.0);
        finite(uv).then_some(uv)
    }

    fn line_pcurve(&self, points: [Vec3;2], radius: f64, bar: f64) -> Option<NurbsCurve> {
        let normalized=[self.uv(points[0])?,self.uv(points[1])?];
        let native=normalized.map(|p| Vec3::new(self.domain_u[0]+p.x*(self.domain_u[1]-self.domain_u[0]),
            self.domain_v[0]+p.y*(self.domain_v[1]-self.domain_v[0]),0.0));
        if native.iter().any(|p| !finite(*p)) { return None; }
        // Certify the stored native controls, including cancellation when a
        // short UV span has a large parameter origin, not just the inverse.
        let uv=native.map(|p| Vec3::new((p.x-self.domain_u[0])/(self.domain_u[1]-self.domain_u[0]),
            (p.y-self.domain_v[0])/(self.domain_v[1]-self.domain_v[0]),0.0));
        let umax=uv[0].x.abs().max(uv[1].x.abs());
        let vmax=uv[0].y.abs().max(uv[1].y.abs());
        let mut inverse_error=0.0_f64;
        for i in 0..2 {
            let image=self.origin.add(self.u.scale(uv[i].x)).add(self.v.scale(uv[i].y));
            let residual=image.sub(points[i]).length();
            if !residual.is_finite() { return None; }
            inverse_error=inverse_error.max(residual);
        }
        // On the entire extended UV line, |u| and |v| are endpoint-bounded.
        // S=O+uU+vV+uvT and Su×Sv=U×V+u(U×T)+v(T×V).
        let normal_delta=umax*self.u.cross(self.twist).length()+vmax*self.twist.cross(self.v).length();
        if !(normal_delta < self.normal_length) { return None; }
        let normal_error=2.0*normal_delta/(self.normal_length-normal_delta);
        let error=inverse_error+self.twist.length()*umax*vmax+radius*normal_error;
        if !error.is_finite() || !(error <= bar) { return None; }
        crate::make_line(native[0],native[1]).ok()
    }
}

pub(in crate::blend) struct CertifiedCylinder {
    pub(in crate::blend) rows: FittedRows,
    pub(in crate::blend) stations: Vec<Station>,
    pub(in crate::blend) parameters: Vec<f64>,
    pub(in crate::blend) vertex_indices: [usize;2],
    pub(in crate::blend) station_ts: Vec<f64>,
}

/// Construct the circular section and its exact extrusion, without a fit or
/// station solve. The certificate covers the stored nets and overshot rails.
pub(in crate::blend) fn construct(
    edge: &EdgeRecord, first: &BlendMate, second: &BlendMate,
    constant: Option<f64>, overshoot: f64, ends: [f64;2],
) -> Option<CertifiedCylinder> {
    if !enabled() { return None; }
    let radius=constant?;
    if !(radius.is_finite() && radius>0.0 && overshoot.is_finite() && overshoot>=0.0)
        || first.rho.abs()!=radius || second.rho.abs()!=radius
        || (first.rho>0.0)==first.face.same_sense || (second.rho>0.0)==second.face.same_sense
        || first.loop_index!=0 || second.loop_index!=0
        || edge.start_vertex_id==edge.end_vertex_id
        || !(edge.t0.is_finite() && edge.t1.is_finite() && edge.t0>=0.0 && edge.t1<=1.0 && edge.t1>edge.t0)
        || edge.curve.degree!=1 || edge.curve.knots!=[0.0,0.0,1.0,1.0] || edge.curve.control_points.len()!=2
        || edge.curve.control_points.iter().any(|p| p.w!=1.0 || !finite(Vec3::new(p.x,p.y,p.z)))
    { return None; }
    let cp=&edge.curve.control_points;
    let e0=Vec3::new(cp[0].x,cp[0].y,cp[0].z);
    let e1=Vec3::new(cp[1].x,cp[1].y,cp[1].z);
    let chord=e1.sub(e0);
    let bar=crate::KernelTolerances::for_scale(chord.length().max(radius),1e-7).convergence;
    if !(bar.is_finite() && radius>bar) { return None; }
    let a=PlaneChart::new(&first.face.surface,bar)?;
    let b=PlaneChart::new(&second.face.surface,bar)?;
    let dot=a.normal.dot(b.normal);
    if !(dot.abs()<=1e-13) { return None; }
    let (aa,bb)=(a.normal.dot(a.normal),b.normal.dot(b.normal));
    let det=aa*bb-dot*dot;
    let solve=|x:f64,y:f64| a.normal.scale((bb*x-dot*y)/det).add(b.normal.scale((aa*y-dot*x)/det));
    let axis=a.normal.cross(b.normal).normalized().ok()?;
    let travel=axis.scale(chord.dot(axis));
    let base=e0.add(solve(-a.normal.dot(e0.sub(a.origin)),-b.normal.dot(e0.sub(b.origin))));
    let center0=base.add(solve(first.rho,second.rho));
    if !finite(travel) || !finite(center0) || !(travel.length()>bar) { return None; }
    let (station_ts,vertex_indices)=crate::blend::edge::open_station_parameters(edge,overshoot,ends,16,[false,false]);
    if station_ts.iter().any(|t| !t.is_finite()) || station_ts.windows(2).any(|p| !(p[1]>p[0]))
        || vertex_indices[0]>=vertex_indices[1] { return None; }
    let (lo,hi)=(*station_ts.first()?,*station_ts.last()?);
    for t in [lo,hi] {
        let original=e0.add(chord.scale(t));
        let projected=base.add(travel.scale(t));
        if !(original.sub(projected).length()<=bar) { return None; }
    }
    let center_start=center0.add(travel.scale(lo));
    let shift=travel.scale(hi-lo);
    let p1=center_start.sub(a.normal.scale(first.rho));
    let p2=center_start.sub(b.normal.scale(second.rho));
    let d1=a.normal.scale(-first.rho/radius);
    let d2=b.normal.scale(-second.rho/radius);
    let cosine=d1.dot(d2);
    let weight=((1.0+cosine)*0.5).sqrt();
    let apex=center_start.add(d1.add(d2).scale(radius/(1.0+cosine)));
    if !finite(shift) || !finite(apex) || !(shift.length()>bar && weight.is_finite() && weight>0.0) { return None; }
    // Large translations must not round away the section radius or travel.
    if !(p1.sub(center_start).add(a.normal.scale(first.rho)).length()<=bar)
        || !(p2.sub(center_start).add(b.normal.scale(second.rho)).length()<=bar) { return None; }
    for start in [center_start,p1,p2,apex] {
        let moved=start.add(shift).sub(start);
        if !finite(moved) || !(moved.length()>bar && moved.sub(shift).length()<=bar) { return None; }
    }
    let cr_pcurve=a.line_pcurve([p1,p1.add(shift)],radius,bar)?;
    let cs_pcurve=b.line_pcurve([p2,p2.add(shift)],radius,bar)?;
    let section=NurbsCurve::new(2,vec![0.0,0.0,0.0,1.0,1.0,1.0],vec![
        Vec4::from_point(p1,1.0),Vec4::from_point(apex,weight),Vec4::from_point(p2,1.0),
    ]).ok()?;
    let cylinder=crate::make_extrusion(&section,shift).ok()?;
    // The network uses u along the edge, v along the circular section.
    let surface=NurbsSurface::new(1,2,cylinder.knots_v.clone(),cylinder.knots_u.clone(),
        (0..2).map(|j| (0..3).map(|i| cylinder.control_points[i][j]).collect()).collect()).ok()?;
    let cr=crate::make_line(p1,p1.add(shift)).ok()?;
    let cs=crate::make_line(p2,p2.add(shift)).ok()?;
    let center=crate::make_line(center_start,center_start.add(shift)).ok()?;
    let parameters:Vec<f64>=station_ts.iter().map(|t| (t-lo)/(hi-lo)).collect();
    let mut stations=Vec::with_capacity(parameters.len());
    for &s in &parameters {
        let u1=cr_pcurve.evaluate(s).ok()?;
        let u2=cs_pcurve.evaluate(s).ok()?;
        stations.push(Station { uv1:[u1.x,u1.y],uv2:[u2.x,u2.y],p1:cr.evaluate(s).ok()?,p2:cs.evaluate(s).ok()?,
            center:center.evaluate(s).ok()?,weight,apex:apex.add(shift.scale(s)) });
    }
    let rows=FittedRows {surface,cr,cs,cr_pcurve,cs_pcurve,u_domain:[0.0,1.0],center:Some(center),
        exact_extrusion:true,vertex_stations:Some([parameters[vertex_indices[0]],parameters[vertex_indices[1]]]),revolution_axis:None};
    crate::blend::record_count("count.network.analytic_cylinder",1);
    Some(CertifiedCylinder {rows,stations,parameters,vertex_indices,station_ts})
}
