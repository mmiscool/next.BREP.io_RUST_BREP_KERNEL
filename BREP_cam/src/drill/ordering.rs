//! Deterministic open-path nearest neighbour followed by budgeted 2-opt.
use crate::{CamError, Point3, Runner};

/// Order a SINGLE tool's hole positions. Stable ties retain original indices.
/// `swap_budget` counts candidate swaps, not elapsed time. The start is fixed;
/// the final hole is free (no artificial trip back to the starting point).
pub fn order(points:&[Point3],start:Point3,swap_budget:u64,runner:&mut Runner<'_>)->Result<Vec<usize>,CamError> {
    runner.token().check()?;
    if !start.is_finite() || points.iter().any(|p|!p.is_finite()) {
        return Err(CamError::InvalidInput("hole ordering requires finite positions".into()));
    }
    let mut used=vec![false;points.len()];let mut result=Vec::with_capacity(points.len());let mut at=start;
    for _ in 0..points.len() {
        let mut best=None;let mut distance=f64::INFINITY;
        for (i,p) in points.iter().enumerate() {
            runner.work(1,"hole-neighbour",0,0)?;
            if !used[i] {let d=at.distance_xy(*p);if best.is_none() || d<distance {best=Some(i);distance=d;}}
        }
        let i=best.unwrap();result.push(i);used[i]=true;at=points[i];
    }
    let mut work=0;
    loop {
        let mut improved=false;
        for i in 0..result.len() {
            for j in i+1..result.len() {
                if work==swap_budget {runner.token().check()?;return Ok(result);}
                work+=1;runner.work(1,"hole-two-opt",work,swap_budget)?;
                let a=if i==0 {start} else {points[result[i-1]]};
                let b=points[result[i]];let c=points[result[j]];
                let mut before=a.distance_xy(b);let mut after=a.distance_xy(c);
                if j+1<result.len() {let d=points[result[j+1]];before+=c.distance_xy(d);after+=b.distance_xy(d);}
                if after<before {result[i..=j].reverse();improved=true;}
            }
        }
        if !improved {break;}
    }
    runner.token().check()?;Ok(result)
}
