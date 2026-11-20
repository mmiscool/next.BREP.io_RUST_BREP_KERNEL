use super::*;

// Isolate every real zero of a scalar Bezier numerator. Variation-diminishing
// bounds discard root-free intervals and certify single crossing intervals.
// Unresolved multiple roots decline the whole reduction, never hide a branch.
fn roots(mut c: Vec<f64>, a: f64, b: f64, depth: usize, out: &mut Vec<f64>) -> Option<()> {
    if c.iter().any(|x| !x.is_finite()) || c.iter().all(|x| *x == 0.0) { return None; }
    if c[0] == 0.0 {
        out.push(a);
        let n = c.len()-1;
        c = (0..n).map(|i| c[i+1]*n as f64/(i+1) as f64).collect();
        return roots(c,a,b,depth,out);
    }
    if c[c.len()-1] == 0.0 {
        out.push(b);
        let n = c.len()-1;
        c = (0..n).map(|i| c[i]*n as f64/(n-i) as f64).collect();
        return roots(c,a,b,depth,out);
    }
    let signs: Vec<_> = c.iter().filter(|x| **x != 0.0).map(|x| x.is_sign_positive()).collect();
    let variations = signs.windows(2).filter(|s| s[0] != s[1]).count();
    if variations == 0 { return Some(()); }
    if depth >= 52 || (a+b)*0.5 == a || (a+b)*0.5 == b { return None; }
    if variations == 1 {
        let eval = |t: f64| {
            let mut p = c.clone();
            for n in (1..p.len()).rev() { for i in 0..n { p[i] = (1.0-t)*p[i]+t*p[i+1]; } }
            p[0]
        };
        let (mut lo,mut hi) = (0.0,1.0);
        for _ in 0..60 {
            let mid = (lo+hi)*0.5;
            let f = eval(mid);
            if f == 0.0 { lo=mid; hi=mid; break; }
            if f.is_sign_positive() == c[0].is_sign_positive() { lo=mid; } else { hi=mid; }
        }
        out.push(a+(b-a)*(lo+hi)*0.5);
        return Some(());
    }
    let mut left = vec![c[0]];
    let mut right = vec![*c.last()?];
    for n in (1..c.len()).rev() {
        for i in 0..n { c[i] = (c[i]+c[i+1])*0.5; }
        left.push(c[0]); right.push(c[n-1]);
    }
    right.reverse();
    let middle = (a+b)*0.5;
    roots(left,a,middle,depth+1,out)?;
    roots(right,middle,b,depth+1,out)
}

/// Plane/extrusion section in the finite sweep band. A transverse section is
/// an affine image of the whole profile. If its control hull leaves that
/// band, decline to general SSI rather than return an unclipped section.
/// A parallel plane is reduced to all roots of the profile's scalar height;
/// coincident intervals and unresolved tangencies retain the general path.
pub fn plane_extrusion_section(extrusion: &ExtrusionStructure, origin: Vec3, normal: Vec3, tolerance: f64) -> Option<Vec<NurbsCurve>> {
    if !tolerance.is_finite() || tolerance <= 0.0 { return None; }
    let normal = normal.normalized().ok()?;
    let d = extrusion.direction;
    let dot = normal.dot(d);
    let length = extrusion.sweep_domain[1]-extrusion.sweep_domain[0];
    if !length.is_finite() || length <= 0.0 || !dot.is_finite() { return None; }
    let profile = &extrusion.profile;
    if dot.abs() > 1e-10*d.length() {
        let mut controls = Vec::new();
        for p in &profile.control_points {
            let point = p.point().ok()?;
            let s = origin.sub(point).dot(normal)/dot;
            if !s.is_finite() || s < 0.0 || s > length { return None; }
            let q = point.add(d.scale(s));
            let residual = q.sub(origin).dot(normal).abs();
            if !residual.is_finite() || residual > tolerance { return None; }
            controls.push(crate::Vec4 { x:q.x*p.w,y:q.y*p.w,z:q.z*p.w,w:p.w });
        }
        return Some(vec![NurbsCurve::new(profile.degree,profile.knots.clone(),controls).ok()?]);
    }
    // A nearly parallel direction is not parallel: its section can travel
    // arbitrarily far, so only roundoff-sized dot products take this lane.
    if dot.abs() > 32.0*f64::EPSILON*d.length() { return None; }
    // Angular roundoff can accumulate over a long sweep. Reserve the
    // remaining positional budget for the profile root's residual.
    let drift = dot.abs()*length;
    if !drift.is_finite() || drift > tolerance { return None; }
    let coefficients: Vec<_> = profile.control_points.iter().map(|p| {
        let value=Vec3::new(p.x-origin.x*p.w,p.y-origin.y*p.w,p.z-origin.z*p.w).dot(normal);
        // Trigonometric endpoint construction can leave a roundoff residue
        // at a closed seam. Zero only that arithmetic uncertainty, capped
        // well inside the caller's contact band; larger near misses decline.
        let roundoff=64.0*f64::EPSILON*(Vec3::new(p.x,p.y,p.z).length()+origin.length()*p.w);
        if value.abs()<=roundoff.min(0.01*tolerance*p.w) {0.0} else {value}
    }).collect();
    let pieces = crate::curve::bezier_decompose_raw(&profile.knots,&coefficients,profile.degree,0.0,|a,b,t|a*(1.0-t)+b*t)?;
    let weights: Vec<_> = profile.control_points.iter().map(|p|p.w).collect();
    let weight_pieces=crate::curve::bezier_decompose_raw(&profile.knots,&weights,profile.degree,0.0,|a,b,t|a*(1.0-t)+b*t)?;
    let domain = profile.domain().ok()?;
    let spans: Vec<_> = profile.knots.windows(2).filter(|k| k[1]>k[0] && k[0]>=domain[0] && k[1]<=domain[1]).collect();
    if pieces.len()!=spans.len() { return None; }
    let mut parameters = Vec::new();
    for ((piece,weights),span) in pieces.into_iter().zip(weight_pieces).zip(spans) {
        // A root-free numerator is not proof of separation within the caller's
        // tolerance. Its extrema and ends must stay outside that contact band.
        // The maximum positive denominator control is a conservative bound.
        let band=tolerance*weights.iter().copied().fold(0.0_f64,f64::max);
        for endpoint in [piece[0],*piece.last()?] {
            if endpoint != 0.0 && endpoint.abs()<=band { return None; }
        }
        let derivative: Vec<_>=piece.windows(2).map(|c|(c[1]-c[0])*(piece.len()-1) as f64).collect();
        if (piece[0]==0.0 && derivative.first().is_some_and(|d|d.abs()<=band))
            || (*piece.last()?==0.0 && derivative.last().is_some_and(|d|d.abs()<=band)) { return None; }
        if derivative.iter().all(|x|*x==0.0) {
            if piece[0].abs()<=band { return None; }
        } else {
            let mut critical=Vec::new(); roots(derivative,0.0,1.0,0,&mut critical)?;
            for t in critical {
                if t==0.0 || t==1.0 { continue; }
                let mut c=piece.clone();
                for n in (1..c.len()).rev() { for i in 0..n {c[i]=c[i]*(1.0-t)+c[i+1]*t;} }
                if c[0].abs()<=band { return None; }
            }
        }
        roots(piece,span[0],span[1],0,&mut parameters)?;
    }
    parameters.sort_by(f64::total_cmp);
    parameters.dedup_by(|a,b| (*a-*b).abs()<=64.0*f64::EPSILON*(domain[1]-domain[0]));
    // A closed profile's two endpoint parameters name one geometric ruling.
    if parameters.first().is_some_and(|t|*t==domain[0]) && parameters.last().is_some_and(|t|*t==domain[1]) {
        let first=profile.evaluate(domain[0]).ok()?; let last=profile.evaluate(domain[1]).ok()?;
        let roundoff=64.0*f64::EPSILON*first.length().max(last.length()).max(d.length()*length);
        if first.sub(last).length()<=roundoff.min(tolerance) { parameters.pop(); }
    }
    let points=parameters.iter().map(|t|profile.evaluate(*t).ok()).collect::<Option<Vec<_>>>()?;
    for (i,p) in points.iter().enumerate() {
        if points[..i].iter().any(|q|p.sub(*q).length()<=tolerance) { return None; }
    }
    parameters.into_iter().map(|t| {
        let p = profile.evaluate(t).ok()?;
        let end = p.add(d.scale(length));
        let residual = p.sub(origin).dot(normal).abs();
        let end_residual = end.sub(origin).dot(normal).abs();
        if !residual.is_finite() || residual+drift > tolerance
            || !end_residual.is_finite() || end_residual > tolerance { return None; }
        crate::make_line(p,end).ok()
    }).collect()
}
