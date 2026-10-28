//! Deterministic adaptive Z bands, measured on corresponding closed CL loops.
use crate::{CamError,Point3,Runner};
use super::weave::Loop;
use std::collections::BTreeMap;

fn point_at(points:&[Point3],fraction:f64)->Point3 {
    let length: f64=points.windows(2).map(|p|p[0].distance_xy(p[1])).sum();
    let target=length*fraction;let mut used=0.0;
    for pair in points.windows(2) {
        let span=pair[0].distance_xy(pair[1]);
        if used+span>=target && span>0.0 {return pair[0].lerp(pair[1],(target-used)/span);}
        used+=span;
    }
    points[0]
}

/// Compare equal arc-length positions from each canonical closed-loop start.
/// This also detects lateral loop motion and changing loop shape, not only area.
pub fn departure(a:&[Loop],m:&[Loop],b:&[Loop])->f64 {
    if a.len()!=m.len() || a.len()!=b.len() {return f64::INFINITY;}
    let mut maximum: f64=0.0;
    for ((a,m),b) in a.iter().zip(m).zip(b) {
        if a.hole!=m.hole || a.hole!=b.hole {return f64::INFINITY;}
        let count=a.points.len().max(m.points.len()).max(b.points.len()).max(32);
        for i in 0..count {
            let t=i as f64/count as f64;
            let expected=point_at(&a.points,t).lerp(point_at(&b.points,t),0.5);
            maximum=maximum.max(expected.distance(point_at(&m.points,t)));
        }
    }
    maximum
}

/// Refine each input band until its midpoint CL departure is within tolerance.
/// Topology changes and exhausted work budgets are explicit errors. This is a
/// sampled geometric departure criterion, not an analytic stock-removal model.
pub fn refine(initial:&[f64],limit:f64,max_levels:usize,runner:&mut Runner<'_>,mut sample:impl FnMut(f64,&mut Runner<'_>)->Result<Vec<Loop>,CamError>)->Result<Vec<f64>,CamError> {
    if initial.is_empty() || !limit.is_finite() || limit<=0.0 || initial.iter().any(|z|!z.is_finite()) || initial.windows(2).any(|p|p[0]<=p[1]) || max_levels<initial.len() {
        return Err(CamError::InvalidInput("invalid adaptive Z controls".into()));
    }
    let mut cache=BTreeMap::new();
    for z in initial {cache.insert(z.to_bits(),sample(*z,runner)?);}
    let mut stack:Vec<_>=initial.windows(2).rev().map(|p|(p[0],p[1],0u32)).collect();
    let mut output=vec![initial[0]];
    while let Some((top,bottom,depth))=stack.pop() {
        runner.stage("contour-adaptive-z","Check waterline band departure",output.len() as u64,max_levels as u64)?;
        if cache.len()>=max_levels.saturating_mul(2) {return Err(CamError::InvalidInput("adaptive Z sample budget exceeded".into()));}
        let mid=bottom+(top-bottom)*0.5;
        if mid==top || mid==bottom {return Err(CamError::InvalidInput("adaptive Z resolution exhausted".into()));}
        let middle=sample(mid,runner)?;
        let error=departure(&cache[&top.to_bits()],&middle,&cache[&bottom.to_bits()]);
        cache.insert(mid.to_bits(),middle);
        if error>limit {
            if depth>=32 || output.len()+stack.len()+2>=max_levels {return Err(CamError::InvalidInput("adaptive Z cannot resolve topology/departure within level budget".into()));}
            stack.push((mid,bottom,depth+1));stack.push((top,mid,depth+1));
        } else {output.push(bottom);}
    }
    Ok(output)
}

