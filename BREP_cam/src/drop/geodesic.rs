//! Surface-distance fronts on a drop-cutter CL grid. A triangular fast-marching
//! update minimizes interpolated arrival time plus travel across each facet.
//! Iso-distance contours split at front collisions; planar grid topology keeps
//! distinct contour components from crossing. Coverage is checked by a second
//! multi-source solve from the extracted fronts, with a cell-interior margin.
use crate::*;
use crate::drop::{DropCutter,finish::{FinishContext,invalid,polyline,MAX_SOURCE_SEGMENTS}};
use std::cmp::Ordering;
use std::collections::{BinaryHeap,BTreeMap,BTreeSet};

const MAX_GRID:usize=250_000;
struct Grid {
    points:Vec<Point3>, valid:Vec<bool>, triangles:Vec<[usize;3]>, incident:Vec<Vec<usize>>,
    boundary:Vec<usize>, max_edge:f64, counts:WorkCounts,
}
#[derive(Clone,Copy)]
struct Event {distance:f64,id:usize}
impl PartialEq for Event {fn eq(&self,b:&Self)->bool {self.distance==b.distance&&self.id==b.id}}
impl Eq for Event {}
impl Ord for Event {fn cmp(&self,b:&Self)->Ordering {b.distance.total_cmp(&self.distance).then(b.id.cmp(&self.id))}}
impl PartialOrd for Event {fn partial_cmp(&self,b:&Self)->Option<Ordering> {Some(self.cmp(b))}}

impl Grid {
    fn build(ctx:&FinishContext<'_>,p:&ScallopParams,runner:&mut Runner<'_>)->Result<Self,CamError> {
        let b=ctx.bounds(&p.containment)?;let region=ctx.region(&p.containment)?;
        let spacing=(ctx.operation.tolerance*0.25).min(p.stepover.distance/8.);
        let cols=((b.max.x-b.min.x)/spacing).ceil()+1.;
        let rows=((b.max.y-b.min.y)/spacing).ceil()+1.;
        if !cols.is_finite()||!rows.is_finite()||cols<2.||rows<2.||cols*rows>MAX_GRID as f64 {return Err(invalid("freeform CL grid exceeds its deterministic work budget; reduce region or relax tolerance"));}
        let (nx,ny)=(cols as usize,rows as usize);
        let drop=DropCutter::new(ctx.mesh,ctx.cutter,ctx.index,ctx.tolerance)?;
        let mut points=vec![Point3::ZERO;nx*ny];let mut valid=vec![false;nx*ny];let mut counts=WorkCounts::default();
        runner.stage("scallop-cl-grid","Sample protected cutter-location surface",0,(nx*ny) as u64)?;
        for j in 0..ny {for i in 0..nx {
            let x=b.min.x+(b.max.x-b.min.x)*i as f64/(nx-1) as f64;
            let y=b.min.y+(b.max.y-b.min.y)*j as f64/(ny-1) as f64;let id=j*nx+i;
            if region.contains(Point2::new(x,y)) {
                let sample=drop.point(x,y,ctx.sampling().floor_z,runner)?;
                counts.samples+=1;counts.candidates+=sample.candidate_count;counts.contacts+=sample.contact_count;
                if sample.point.contact.is_none() {return Err(invalid("freeform CL grid contains an unreachable region"));}
                points[id]=sample.point.position();valid[id]=true;
            }
            runner.work(1,"scallop-cl-grid",id as u64+1,(nx*ny) as u64)?;
        }}
        let mut triangles=Vec::new();let mut incident=vec![Vec::new();nx*ny];let mut edges:BTreeMap<(usize,usize),usize>=BTreeMap::new();let mut max_edge:f64=0.;
        for j in 0..ny-1 {for i in 0..nx-1 {
            let a=j*nx+i;let c=a+nx;
            for t in [[a,a+1,c+1],[a,c+1,c]] {
                if t.iter().all(|&id|valid[id]) {
                    // Valid corners alone can bridge a reentrant boundary.
                    // Require each complete cell edge to remain in the region.
                    let mut contained=true;
                    for (u,v) in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])] {
                        let (a,b)=(points[u],points[v]);let clips=region.clip(a,b,runner)?;
                        if clips.len()!=1 || clips[0].0.distance_xy(a)>ctx.tolerance.distance_eps || clips[0].1.distance_xy(b)>ctx.tolerance.distance_eps {contained=false;break;}
                    }
                    if !contained {continue;}
                    let tid=triangles.len();for id in t {incident[id].push(tid);}
                    for (u,v) in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])] {let e=(u.min(v),u.max(v));*edges.entry(e).or_default()+=1;max_edge=max_edge.max(points[u].distance(points[v]));}
                    triangles.push(t);
                }
            }
        }}
        if triangles.is_empty() || max_edge>ctx.operation.tolerance*0.5 {
            return Err(invalid("CL surface grid is too steep/discontinuous for the requested tolerance; refine the region or tolerance"));
        }
        // A point-only sliver has no surface cells and cannot silently disappear.
        if valid.iter().enumerate().any(|(i,&v)|v&&incident[i].is_empty()) {return Err(invalid("CL region has an unresolved sub-cell sliver"));}
        let boundary:BTreeSet<_>=edges.iter().filter(|(_,count)|**count==1).flat_map(|(&(a,b),_)|[a,b]).collect();
        Ok(Self {points,valid,triangles,incident,boundary:boundary.into_iter().collect(),max_edge,counts})
    }
    fn march(&self,seeds:&[(usize,f64)],runner:&mut Runner<'_>)->Result<Vec<f64>,CamError> {
        let mut distance=vec![f64::INFINITY;self.points.len()];let mut accepted=vec![false;self.points.len()];let mut heap=BinaryHeap::new();
        for &(id,d) in seeds {if d<distance[id] {distance[id]=d;heap.push(Event {distance:d,id});}}
        let mut visited=0;
        while let Some(Event {distance:d,id})=heap.pop() {
            if accepted[id]||d!=distance[id] {continue;}accepted[id]=true;visited+=1;
            for &tid in &self.incident[id] {
                let tri=self.triangles[tid];
                for v in tri {
                    if accepted[v] {continue;}
                    let other:Vec<_>=tri.iter().copied().filter(|&u|u!=v).collect();let (a,b)=(other[0],other[1]);let mut best=distance[v];
                    for u in [a,b] {if accepted[u] {best=best.min(distance[u]+self.points[u].distance(self.points[v]));}}
                    if accepted[a]&&accepted[b] {
                        best=best.min(triangle_update(self.points[v],self.points[a],self.points[b],distance[a],distance[b]));
                    }
                    if best<distance[v] {distance[v]=best;heap.push(Event {distance:best,id:v});}
                }
            }
            runner.work(1,"scallop-surface-distance",visited,self.points.len() as u64)?;
        }
        if self.valid.iter().enumerate().any(|(i,&v)|v&&!distance[i].is_finite()) {return Err(invalid("disconnected CL surface region has no distance seed"));}
        Ok(distance)
    }
    fn contours(&self,field:&[f64],level:f64,runner:&mut Runner<'_>)->Result<Front,CamError> {
        let mut level=level;
        // Avoid the degree-four ambiguity of a level passing exactly through a
        // sampled vertex. The shift is far below the geometric tolerance.
        while field.iter().any(|&d|d==level) {level=f64::from_bits(level.to_bits()+1);}
        let mut positions:BTreeMap<(usize,usize),Point3>=BTreeMap::new();let mut graph:BTreeMap<(usize,usize),Vec<(usize,usize)>>=BTreeMap::new();let mut seeds=Vec::new();
        for (tid,t) in self.triangles.iter().enumerate() {
            runner.work(1,"scallop-contours",tid as u64,self.triangles.len() as u64)?;
            let mut crossing=Vec::new();
            for (a,b) in [(t[0],t[1]),(t[1],t[2]),(t[2],t[0])] {
                if (field[a]<level)!=(field[b]<level) {
                    let k=(a.min(b),a.max(b));let alpha=(level-field[k.0])/(field[k.1]-field[k.0]);
                    let p=self.points[k.0].lerp(self.points[k.1],alpha);positions.insert(k,p);crossing.push(k);
                }
            }
            if crossing.len()==2 {
                let (a,b)=(crossing[0],crossing[1]);graph.entry(a).or_default().push(b);graph.entry(b).or_default().push(a);
                for &v in t {seeds.push((v,segment_distance(self.points[v],positions[&a],positions[&b])));}
            }
        }
        if graph.values().any(|n|n.len()!=2) {return Err(invalid("CL offset front is open or non-manifold"));}
        let mut used=BTreeSet::new();let mut paths=Vec::new();
        for &start in graph.keys() {
            if used.contains(&start) {continue;}let mut path=Vec::new();let mut current=start;let mut prev=None;
            loop {
                if !used.insert(current) {if current!=start {return Err(invalid("CL offset fronts collided ambiguously"));}break;}
                path.push(positions[&current]);let next=graph[&current].iter().copied().find(|n|Some(*n)!=prev).unwrap();prev=Some(current);current=next;
            }
            if path.len()<3 {return Err(invalid("CL offset front collapsed below grid resolution"));}
            path.push(path[0]);paths.push(path);
        }
        Ok(Front {level,paths,seeds})
    }
}
struct Front {level:f64,paths:Vec<Vec<Point3>>,seeds:Vec<(usize,f64)>}
fn triangle_update(v:Point3,a:Point3,b:Point3,da:f64,db:f64)->f64 {
    let edge=b.sub(a);let length=edge.length();let delta=(db-da)/length;
    if delta.abs()>=1. {return f64::INFINITY;}
    let q=v.sub(a);let along=q.dot(edge)/length;let height=(q.length_squared()-along*along).max(0.).sqrt();
    let t=((along-delta*height/(1.-delta*delta).sqrt())/length).clamp(0.,1.);
    let candidate=da+(db-da)*t+v.distance(a.lerp(b,t));
    if candidate>=da.max(db) {candidate}else{f64::INFINITY}
}
fn segment_distance(p:Point3,a:Point3,b:Point3)->f64 {
    let d=b.sub(a);let q=d.length_squared();if q==0. {return p.distance(a);}p.distance(a.lerp(b,(p.sub(a).dot(d)/q).clamp(0.,1.)))
}

pub(crate) fn generate(ctx:&FinishContext<'_>,p:&ScallopParams,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    if p.cusp_height.is_some() {return Err(invalid("freeform scallop currently requires CL-surface stepover; cusp-height control is available on verified analytic patches"));}
    // Reuse C0's deterministic tree even when the caller did not prepare one.
    let local=if ctx.index.is_none() {Some(TreeIndex::build_with_runner(ctx.mesh,TreeOptions::default(),runner)?)}else{None};
    let ctx=FinishContext {operation:ctx.operation,cutter:ctx.cutter,mesh:ctx.mesh,index:ctx.index.or_else(||local.as_ref().map(|i|i as &dyn TriangleIndex)),heights:ctx.heights,feeds:ctx.feeds,tolerance:ctx.tolerance};
    let grid=Grid::build(&ctx,p,runner)?;
    let field=grid.march(&grid.boundary.iter().map(|&id|(id,0.)).collect::<Vec<_>>(),runner)?;
    let max=field.iter().copied().filter(|v|v.is_finite()).fold(0.,f64::max);let tiny=(ctx.operation.tolerance*0.01).min(max*0.01);
    let mut fronts=Vec::new();let mut level=tiny;
    while level<max {let front=grid.contours(&field,level,runner)?;if !front.paths.is_empty(){fronts.push(front);}level+=p.stepover.distance;
        if fronts.len()>1000 {return Err(invalid("scallop front work budget exhausted"));}
    }
    if fronts.is_empty() {return Err(invalid("CL region has no nonempty offset front"));}
    let target=p.stepover.distance*0.5+ctx.operation.tolerance-grid.max_edge;let mut cleanup=0;
    let coverage_max=loop {
        let seeds:Vec<_>=fronts.iter().flat_map(|f|f.seeds.iter().copied()).collect();let coverage=grid.march(&seeds,runner)?;
        let (id,&worst)=coverage.iter().enumerate().filter(|(id,_)|grid.valid[*id]).max_by(|a,b|a.1.total_cmp(b.1).then(a.0.cmp(&b.0))).unwrap();
        if worst<=target {break worst+grid.max_edge;}
        if cleanup>=32 {return Err(invalid("scallop coverage could not be certified within cleanup-front budget"));}
        let new_level=(field[id]-tiny).max(tiny*0.5);
        if fronts.iter().any(|f|(f.level-new_level).abs()<tiny*0.1) {return Err(invalid("scallop coverage needs finer CL sampling"));}
        let front=grid.contours(&field,new_level,runner)?;if front.paths.is_empty(){return Err(invalid("scallop terminal region has no resolvable cleanup front"));}fronts.push(front);cleanup+=1;
    };
    fronts.sort_by(|a,b|a.level.total_cmp(&b.level));if p.inside_out {fronts.reverse();}
    let mut paths=Vec::new();let mut segments=0;
    for front in fronts {for mut path in front.paths {
        if p.stepover.direction==CutDirection::Conventional || (p.stepover.direction==CutDirection::Either && paths.len()%2==1) {path.reverse();}
        // Marching triangles can leave nearly coincident crossings near a
        // vertex. Remove them before adaptive drop's degenerate-span filter,
        // preserving a continuous closed source loop.
        path.pop();let mut reduced:Vec<Point3>=Vec::new();
        for point in path {if reduced.last().map_or(true,|last|last.distance_xy(point)>ctx.tolerance.sample_eps*2.) {reduced.push(point);}}
        while reduced.len()>1 && reduced.last().unwrap().distance_xy(reduced[0])<=ctx.tolerance.sample_eps*2. {reduced.pop();}
        if reduced.len()<3 {return Err(invalid("scallop contour is below the path sampling resolution"));}
        reduced.push(reduced[0]);
        let points:Vec<_>=reduced.iter().map(|p|Point2::new(p.x,p.y)).collect();segments+=points.len();
        if segments>MAX_SOURCE_SEGMENTS {return Err(invalid("scallop extracted contour work budget exhausted"));}
        paths.push(polyline(&points)?);
    }}
    let mut out=ctx.generate(&p.containment,&paths,runner)?;
    out.report.counts.samples+=grid.counts.samples;out.report.counts.candidates+=grid.counts.candidates;out.report.counts.contacts+=grid.counts.contacts;
    out.report.approximations.push(ApproximationLimit {kind:"freeform-cl-surface-fronts".into(),bound:Some(ctx.operation.tolerance),detail:Some(format!("{} CL nodes; {} surface cells; max cell edge {}; coverage bound {}; requested CL step {}; cleanup fronts {}",grid.valid.iter().filter(|v|**v).count(),grid.triangles.len(),grid.max_edge,coverage_max,p.stepover.distance,cleanup))});
    Ok(out)
}
