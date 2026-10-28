//! Boundary-only weave on a rectangular (possibly nonuniform) fiber grid.
//! Interval endpoints are CL vertices. Grid intersections supply material
//! occupancy, not machinable cell-face loops. Ambiguous cells are refused.
use std::collections::{BTreeMap,BTreeSet};
use crate::{CamError, Point3, Runner, Tolerance};
use crate::push::{Direction, Fiber};

#[derive(Clone, Debug, PartialEq)]
pub struct Loop {
    pub points: Vec<Point3>,
    pub hole: bool,
    pub area: f64,
}

fn error(s:&str)->CamError { CamError::InvalidInput(format!("waterline weave: {s}")) }

pub fn area(p:&[Point3])->f64 {
    p.iter().zip(p.iter().cycle().skip(1)).take(p.len()).map(|(a,b)| a.x*b.y-b.x*a.y).sum::<f64>()*0.5
}

pub fn contains(p:&[Point3], q:Point3)->bool {
    let mut inside=false;
    for (a,b) in p.iter().zip(p.iter().cycle().skip(1)).take(p.len()) {
        if (a.y>q.y)!=(b.y>q.y) && q.x<(b.x-a.x)*(q.y-a.y)/(b.y-a.y)+a.x { inside=!inside; }
    }
    inside
}

pub fn reconstruct(x:&[Fiber],y:&[Fiber],tol:Tolerance,runner:&mut Runner<'_>)->Result<Vec<Loop>,CamError> {
    reconstruct_counted(x,y,tol,false,runner).map(|(loops,_)|loops)
}

/// `reconstruct`, plus the number of free gaps closed. With `close`, a free
/// gap between two blocked intervals of one fiber that lies inside a single
/// grid edge is closed instead of refused: at the tip of a sharp free wedge,
/// where two blocked regions meet at an angle, such a gap has two CL vertices
/// on one edge at every grid spacing, so refinement cannot resolve it.
/// Closing keeps the tool out of a gap narrower than the grid, which only
/// removes positions. A blocked interval inside one edge is always refused
/// (dropping it could gouge).
pub fn reconstruct_counted(x:&[Fiber],y:&[Fiber],tol:Tolerance,close:bool,runner:&mut Runner<'_>)->Result<(Vec<Loop>,usize),CamError> {
    tol.validate()?;
    if x.len()<2 || y.len()<2 { return Err(error("two fibers per axis are required")); }
    let mut xs:Vec<_>=x.iter().collect(); let mut ys:Vec<_>=y.iter().collect();
    xs.sort_by(|a,b|a.coordinate().total_cmp(&b.coordinate()));
    ys.sort_by(|a,b|a.coordinate().total_cmp(&b.coordinate()));
    if xs.iter().any(|f| f.direction!=Direction::X) || ys.iter().any(|f| f.direction!=Direction::Y) {
        return Err(error("wrong fiber direction"));
    }
    let z=xs[0].start.z;
    for fibers in [&xs,&ys] {
        if fibers.windows(2).any(|p|p[1].coordinate()-p[0].coordinate()<=tol.distance_eps) { return Err(error("duplicate or unordered grid coordinates")); }
        for f in fibers {
            if !f.start.is_finite() || !f.end.is_finite() || f.start.z!=z || f.end.z!=z { return Err(error("non-finite or inconsistent fiber plane")); }
            let mut previous=-1.0;
            for interval in &f.intervals {
                if !interval.lower.is_finite() || !interval.upper.is_finite() || interval.lower<0.0 || interval.upper>1.0 || interval.upper<=interval.lower || interval.lower<=previous {
                    return Err(error("intervals must be finite, disjoint and sorted"));
                }
                previous=interval.upper;
            }
        }
    }
    // Each edge has at most one CL vertex. More means a sub-grid component or
    // unresolved narrow gap: no arbitrary pairing that could lose an island.
    // Work follows the boundary, not the Cartesian product of mostly empty
    // fibers. Fine grids around small curved parts otherwise exhaust a dense
    // cell budget while containing only a few thousand actual boundary cells.
    const MAX_BOUNDARY_WORK:usize=4_000_000;
    if xs.len().saturating_add(ys.len())>MAX_BOUNDARY_WORK {return Err(error("fiber work budget exceeded"));}
    let mut points=Vec::new();
    let mut edges=BTreeMap::<(u8,usize,usize),usize>::new();
    let mut closed=0usize;
    for (axis,fibers,opposite) in [(0u8,&xs,&ys),(1u8,&ys,&xs)] {
        let coordinates:Vec<_>=opposite.iter().map(|f|f.coordinate()).collect();
        let along=|f:&Fiber,t:f64| { let p=f.start.lerp(f.end,t); if axis==0 {p.x} else {p.y} };
        let on_grid=|c:f64| { let i=coordinates.partition_point(|v|*v<c); (i>0 && (c-coordinates[i-1]).abs()<=tol.distance_eps) || (i<coordinates.len() && (c-coordinates[i]).abs()<=tol.distance_eps) };
        for (row,f) in fibers.iter().enumerate() {
            let mut spans:Vec<(f64,f64)>=Vec::with_capacity(f.intervals.len());
            for interval in &f.intervals {
                if let Some(last)=spans.last_mut() {
                    let (a,b)=(along(f,last.1),along(f,interval.lower));
                    if close && !on_grid(a) && !on_grid(b) && coordinates.partition_point(|v|*v<a)==coordinates.partition_point(|v|*v<b) {
                        last.1=interval.upper; closed+=1; continue;
                    }
                }
                spans.push((interval.lower,interval.upper));
            }
            for (lower,upper) in spans {
                for t in [lower,upper] {
                    let p=f.start.lerp(f.end,t); let coord=if axis==0 {p.x} else {p.y};
                    let i=coordinates.partition_point(|v|*v<coord);
                    if i==0 || i==coordinates.len() { return Err(error("open boundary at grid extent")); }
                    if (coord-coordinates[i-1]).abs()<=tol.distance_eps || (coord-coordinates[i]).abs()<=tol.distance_eps {
                        return Err(error(&format!("CL vertex coincides with grid intersection; refine or translate grid (axis {axis}, coordinate {coord}, neighbors {} / {})",coordinates[i-1],coordinates[i])));
                    }
                    if edges.insert((axis,row,i-1),points.len()).is_some() { return Err(error("multiple boundary crossings in one grid edge; refine")); }
                    points.push(p);
                    if points.len()>MAX_BOUNDARY_WORK {return Err(error("boundary work budget exceeded"));}
                    runner.work(1,"waterline-boundary",points.len() as u64,0)?;
                }
            }
        }
    }
    let mut adjacency=vec![Vec::<usize>::new();points.len()];
    let mut cells=BTreeSet::new();
    for &(axis,row,col) in edges.keys() {
        let (r,c)=if axis==0 {(row,col)}else{(col,row)};
        if r<xs.len()-1 && c<ys.len()-1 {cells.insert((r,c));}
        if axis==0 && r>0 {cells.insert((r-1,c));}
        if axis==1 && c>0 {cells.insert((r,c-1));}
        if cells.len()>MAX_BOUNDARY_WORK {return Err(error("boundary cell work budget exceeded"));}
    }
    for (i,&(row,col)) in cells.iter().enumerate() {
        runner.work(1,"waterline-weave",i as u64,cells.len() as u64)?;
        let around=[(0,row,col),(1,col+1,row),(0,row+1,col),(1,col,row)];
        let hits:Vec<_>=around.iter().filter_map(|key|edges.get(key).copied()).collect();
        match hits.len() {
            2=> {adjacency[hits[0]].push(hits[1]); adjacency[hits[1]].push(hits[0]);},
            _=>return Err(error("ambiguous cell topology; refine")),
        }
    }
    if adjacency.iter().any(|n|n.len()!=2) { return Err(error("open or non-manifold boundary graph")); }
    let mut visited=vec![false;points.len()]; let mut loops=Vec::<Loop>::new();
    for first in 0..points.len() {
        if visited[first] {continue;}
        let mut current=first; let mut previous=usize::MAX; let mut path=Vec::new();
        loop {
            runner.work(1,"waterline-traverse",current as u64,points.len() as u64)?;
            if visited[current] { if current==first {break;} return Err(error("boundary cycle revisits another cycle")); }
            visited[current]=true; path.push(points[current]);
            let next=if adjacency[current][0]!=previous {adjacency[current][0]} else {adjacency[current][1]};
            previous=current; current=next;
        }
        let signed=area(&path);
        if path.len()<3 || signed.abs()<=tol.area_eps {return Err(error("degenerate boundary cycle"));}
        loops.push(Loop {points:path,hole:false,area:signed});
    }
    for i in 0..loops.len() {
        runner.work(1,"waterline-classify",i as u64,loops.len() as u64)?;
        let nesting=(0..loops.len()).filter(|j|*j!=i && contains(&loops[*j].points,loops[i].points[0])).count();
        let hole=nesting%2==1;
        if (loops[i].area<0.0)!=hole {loops[i].points.reverse();}
        let start=(0..loops[i].points.len()).min_by(|a,b| {
            let a=loops[i].points[*a]; let b=loops[i].points[*b]; a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y))
        }).unwrap();
        loops[i].points.rotate_left(start);
        let first=loops[i].points[0]; loops[i].points.push(first);
        loops[i].hole=hole; loops[i].area=area(&loops[i].points);
    }
    loops.sort_by(|a,b|b.area.abs().total_cmp(&a.area.abs()).then(a.points[0].x.total_cmp(&b.points[0].x)).then(a.points[0].y.total_cmp(&b.points[0].y)));
    Ok((loops,closed))
}
