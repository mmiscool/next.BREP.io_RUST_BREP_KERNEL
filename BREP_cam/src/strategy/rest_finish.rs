//! Reference-ball accessibility rest finishing. A surface point and its facet
//! normal define the unique tangent ball center. If dropping that center raises
//! the reference tool, the point is unreachable by that ball. The current ball
//! must reach it. Stock mode instead uses residual material at dexel centres.
use crate::*;
use crate::drop::{DropCutter,finish::{FinishContext,invalid,MAX_SOURCE_SEGMENTS},containment::Region,path::Span};
fn gap(p:Point3,n:Point3,r:f64,drop:&DropCutter<'_>,runner:&mut Runner<'_>)->Result<f64,CamError> {
    let center=p.add(n.scale(r));let tip=center.z-r;
    Ok(drop.point(center.x,center.y,tip-1.,runner)?.point.z-tip)
}
enum Probe<'a> { Reference {radius:f64,drop:&'a DropCutter<'a>,threshold:f64}, Stock {grid:&'a DexelGrid,needed:Vec<bool>} }
impl Probe<'_> {
    fn needed(&self,p:Point3,n:Point3,runner:&mut Runner<'_>)->Result<bool,CamError> {
        match self {
            Self::Reference {radius,drop,threshold}=>Ok(gap(p,n,*radius,drop,runner)?>*threshold),
            Self::Stock {grid,needed}=>{let x=((p.x-grid.origin.x)/grid.cell_x).floor();let y=((p.y-grid.origin.y)/grid.cell_y).floor();Ok(x>=0. && y>=0. && x<(grid.nx as f64) && y<(grid.ny as f64) && needed[y as usize*grid.nx as usize+x as usize])}
        }
    }
}
// Check every cell touched by a leaf's projected CL bounding rectangle. This
// deliberately omits boundary leaves rather than bridging a non-rest cell.
fn stock_cells(vertices:[Point3;3],normal:Point3,radius:f64,grid:&DexelGrid,needed:&[bool])->(bool,bool) {
    let v=vertices.map(|p|p.add(normal.scale(radius)));
    let xmin=v.iter().map(|p|((p.x-grid.origin.x)/grid.cell_x).floor()).fold(f64::INFINITY,f64::min);
    let xmax=v.iter().map(|p|((p.x-grid.origin.x)/grid.cell_x).floor()).fold(f64::NEG_INFINITY,f64::max);
    let ymin=v.iter().map(|p|((p.y-grid.origin.y)/grid.cell_y).floor()).fold(f64::INFINITY,f64::min);
    let ymax=v.iter().map(|p|((p.y-grid.origin.y)/grid.cell_y).floor()).fold(f64::NEG_INFINITY,f64::max);
    let mut all=xmin>=0. && ymin>=0. && xmax<grid.nx as f64 && ymax<grid.ny as f64;let mut any=false;
    if xmax<0. || ymax<0. || xmin>=grid.nx as f64 || ymin>=grid.ny as f64 {return (false,false);}
    for y in ymin.max(0.) as usize..=ymax.min(grid.ny as f64-1.) as usize {for x in xmin.max(0.) as usize..=xmax.min(grid.nx as f64-1.) as usize {let b=needed[y*grid.nx as usize+x];all&=b;any|=b;}}
    (all,any)
}
fn selected(operation:&Operation,mesh:&MeshSnapshot,t:&crate::mesh::Triangle)->bool {
    let f=&mesh.faces[t.face as usize];!f.protected_only && operation.targets.target_solids.contains(&f.solid) && (operation.targets.drive_faces.is_empty() || operation.targets.drive_faces.iter().any(|r|r.solid==f.solid&&r.face.as_ref().map_or(true,|id|id==&f.face)))
}
// Budgets, measured on the C19 thin-wall job (145 x 100 x 70, 832 triangles,
// job grid 388 x 270 = 104 760 columns, 0.1 left everywhere against a 0.05
// minimum rest, so the whole part is rest region; C16 record 2026-10-20,
// brep-vm): 807 013 raster tests in 0.004 s, 8 281 508 classification calls
// (1 596 231 leaves, 3 557 148 distinct gap drops) in about 31 s serial, the
// operation 40 s serial and 15 s with feature `parallel` on 6 cores (143 s
// before the per-triangle drop memo and the drive-polygon index).
/// Classification calls (subdivision to the stepover): about 2.4 times the
/// thin wall's, some 3 minutes at its rate.
const MAX_REST_CLASSIFY:usize=20_000_000;
/// Columns a stock grid may hold for rest finishing (the dexel layer's own
/// default ceiling): the per-column work below is the column's intervals.
const MAX_REST_COLUMNS:usize=4_000_000;
/// Triangle-over-column tests of the surface raster: about a second at the
/// thin wall's rate.
const MAX_REST_RASTER:u64=200_000_000;

/// Per column of `grid`: whether its stock stands more than `minimum` above
/// the selected part surface at the column centre (the highest vertical-ray
/// hit). The surface is rasterised triangle by triangle over the columns
/// whose centres its XY bounds can contain, widened by one column (the ray
/// test is exact, so a centre a full column outside a triangle's bounds
/// never hits it): the same heights as testing every column against every
/// triangle, at the cost of the selected part's footprint only, so columns
/// away from the target cost nothing.
pub(crate) fn stock_rest_needed(operation:&Operation,mesh:&MeshSnapshot,grid:&DexelGrid,minimum:f64,runner:&mut Runner<'_>)->Result<Vec<bool>,CamError> {
    if grid.columns.len()>MAX_REST_COLUMNS {return Err(invalid("rest stock grid budget exhausted"));}
    for (i,col) in grid.columns.iter().enumerate() {
        runner.work(1,"rest-stock",i as u64,grid.columns.len() as u64)?;
        let mut last=f64::NEG_INFINITY;for &[lo,hi] in &col.intervals {if !lo.is_finite() || !hi.is_finite() || lo>=hi || lo<last {return Err(invalid("invalid rest stock grid intervals"));}last=hi;}
    }
    let mut surface=vec![f64::NEG_INFINITY;grid.columns.len()];
    let (nx,ny)=(grid.nx as i64,grid.ny as i64);
    // Column index whose centre is the first at or after `v` along an axis.
    let first=|v:f64,origin:f64,cell:f64|((v-origin)/cell-0.5).ceil();
    let mut work=0u64;
    for t in &mesh.triangles {
        runner.work(1,"rest-stock-surface",0,0)?;
        if !selected(operation,mesh,t) {continue;}
        let (x0,x1)=(t.a.x.min(t.b.x).min(t.c.x),t.a.x.max(t.b.x).max(t.c.x));
        let (y0,y1)=(t.a.y.min(t.b.y).min(t.c.y),t.a.y.max(t.b.y).max(t.c.y));
        if ![x0,x1,y0,y1].iter().all(|v|v.is_finite()) {return Err(invalid("rest finishing mesh is not finite"));}
        let ix0=((first(x0,grid.origin.x,grid.cell_x)-1.0).max(0.0)) as i64;
        let ix1=((first(x1,grid.origin.x,grid.cell_x)+1.0).min(nx as f64-1.0)) as i64;
        let iy0=((first(y0,grid.origin.y,grid.cell_y)-1.0).max(0.0)) as i64;
        let iy1=((first(y1,grid.origin.y,grid.cell_y)+1.0).min(ny as f64-1.0)) as i64;
        if ix0>ix1 || iy0>iy1 {continue;}
        work+=((ix1-ix0+1)*(iy1-iy0+1)) as u64;
        if work>MAX_REST_RASTER {return Err(invalid("rest stock surface raster budget exhausted"));}
        for iy in iy0..=iy1 {for ix in ix0..=ix1 {
            let k=(iy*nx+ix) as usize;
            if grid.columns[k].top().is_none() {continue;}
            let (x,y)=grid.center(ix as u32,iy as u32);
            if let Some(z)=crate::stock_sim::vertical_ray_hit(t.a,t.b,t.c,x,y) {surface[k]=surface[k].max(z);}
        }}
    }
    Ok(grid.columns.iter().zip(&surface).map(|(col,&s)|s.is_finite() && col.top().is_some_and(|z|z-s>minimum)).collect())
}

/// Gap drops of one source triangle's subdivision, by exact point: the
/// leaves share vertices, and a repeated drop at the same point (same normal
/// and radius within one triangle) returns the same value.
type GapMemo=std::collections::HashMap<[u64;3],f64>;
fn memo_gap(p:Point3,normal:Point3,small:f64,current:&DropCutter<'_>,memo:&mut GapMemo,runner:&mut Runner<'_>)->Result<f64,CamError> {
    let key=[p.x.to_bits(),p.y.to_bits(),p.z.to_bits()];
    if let Some(&g)=memo.get(&key) {return Ok(g);}
    let g=gap(p,normal,small,current,runner)?;memo.insert(key,g);Ok(g)
}
fn classify(vertices:[Point3;3],normal:Point3,resolution:f64,depth:usize,small:f64,current:&DropCutter<'_>,probe:&Probe<'_>,epsilon:f64,polygons:&mut Vec<Vec<Point2>>,work:&mut usize,memo:&mut GapMemo,runner:&mut Runner<'_>)->Result<(),CamError> {
    *work+=1;runner.work(1,"rest-classify",*work as u64,0)?;
    if *work>MAX_REST_CLASSIFY {return Err(invalid("rest classification budget exhausted"));}
    let stock_all=if let Probe::Stock {grid,needed}=probe {let (all,any)=stock_cells(vertices,normal,small,grid,needed);if !any {return Ok(());}all} else {true};
    let [a,b,c]=vertices;
    let edges=[a.distance(b),b.distance(c),c.distance(a)];
    if edges.iter().copied().fold(0.,f64::max)>resolution {
        if depth>=24 {return Err(invalid("rest subdivision depth exhausted"));}
        let k=if edges[0]>=edges[1]&&edges[0]>=edges[2]{0}else if edges[1]>=edges[2]{1}else{2};
        let (a,b,c)=(vertices[k],vertices[(k+1)%3],vertices[(k+2)%3]);let m=a.lerp(b,0.5);
        for v in [[a,m,c],[m,b,c]] {classify(v,normal,resolution,depth+1,small,current,probe,epsilon,polygons,work,memo,runner)?;}return Ok(());
    }
    if !stock_all {return Ok(());}
    let centroid=a.add(b).add(c).scale(1./3.);
    // All vertices and the centroid must qualify. Mixed cells are omitted;
    // the reported resolution bounds the boundary discretization, not stock.
    for p in [a,b,c,centroid] {
        if !probe.needed(p,normal,runner)? || memo_gap(p,normal,small,current,memo,runner)?>epsilon {return Ok(());}
    }
    polygons.push(vertices.iter().map(|p|{let center=p.add(normal.scale(small));Point2::new(center.x,center.y)}).collect());Ok(())
}
pub fn generate(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,reference_tool:&Cutter)->Result<Toolpath,CamError> {
    generate_source(operation,cutter,mesh,index,heights,feeds,tolerance,runner,Some(reference_tool),None)
}
/// Stock is the previous simulation in the same machine coordinate system.
pub fn generate_from_stock(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,stock:&DexelGrid)->Result<Toolpath,CamError> {
    generate_source(operation,cutter,mesh,index,heights,feeds,tolerance,runner,None,Some(stock))
}
fn generate_source(operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>,reference_tool:Option<&Cutter>,stock:Option<&DexelGrid>)->Result<Toolpath,CamError> {
    let p=match &operation.strategy {Strategy::RestFinish(p)=>p,_=>return Err(invalid("rest finishing requires RestFinish parameters"))};
    let ctx=FinishContext {operation,cutter,mesh,index,heights,feeds,tolerance};ctx.validate()?;
    if !matches!(cutter,Cutter::Ball(_)) {return Err(invalid("rest finishing requires a current ball"));}
    if operation.stock_to_leave.radial!=0. || operation.stock_to_leave.axial!=0. {return Err(invalid("rest finishing requires zero stock allowance"));}
    let current=DropCutter::new(mesh,cutter,index,tolerance)?;
    let reference=reference_tool.map(|tool|DropCutter::new(mesh,tool,index,tolerance)).transpose()?;
    let resolution=p.stepover.distance.max(tolerance.sample_eps*4.);
    let probe=match p.source {
        RestSource::ReferenceTool=>{
            let tool=reference_tool.ok_or_else(||invalid("reference rest source requires a reference tool"))?;
            if !matches!(tool,Cutter::Ball(_)) || tool.radius()<=cutter.radius() {return Err(invalid("reference rest finishing requires a larger reference ball"));}
            Probe::Reference {radius:tool.radius(),drop:reference.as_ref().unwrap(),threshold:p.minimum_rest.max(tolerance.distance_eps)}
        },
        RestSource::Stock=>{
            let grid=stock.ok_or_else(||invalid("stock rest source requires simulated stock"))?;
            if !grid.origin.is_finite() || !grid.cell_x.is_finite() || !grid.cell_y.is_finite() || grid.cell_x<=0. || grid.cell_y<=0. || grid.nx==0 || grid.ny==0 || grid.nx as u64*grid.ny as u64!=grid.columns.len() as u64 {return Err(invalid("invalid rest stock grid layout"));}
            let needed=stock_rest_needed(operation,mesh,grid,p.minimum_rest.max(tolerance.distance_eps),runner)?;
            if !needed.iter().any(|v|*v) {return Err(invalid("no reachable stock rest region"));}
            Probe::Stock {grid,needed}
        }
    };
    let mut sources:Vec<([Point3;3],Point3)>=Vec::new();
    for t in &mesh.triangles {
        if !selected(operation,mesh,t) {continue;}
        let n=if t.normal.z<0. {t.normal.scale(-1.)}else{t.normal};if n.z<=tolerance.angle_eps {continue;}
        // Cheap centroid/vertex exclusion avoids tessellating broad planar
        // facets whose reference tangent balls are all accessible.
        let mut needed=false;
        if matches!(probe,Probe::Stock {..}) {needed=true;} else {for q in [t.a,t.b,t.c,t.centroid()] {needed|=probe.needed(q,n,runner)?;}}
        if needed {sources.push((t.vertices(),n));}
    }
    // Each source triangle is classified on its own (its subdivision, gap
    // memo and leaves); the leaves are joined in source order, and the work
    // budget is summed in that order, so the parallel build (feature
    // `parallel`) equals the serial one.
    let one=|(v,n):&([Point3;3],Point3),worker:&mut Runner<'_>|{
        let (mut leaves,mut work,mut memo)=(Vec::new(),0usize,GapMemo::new());
        let result=classify(*v,*n,resolution,0,cutter.radius(),&current,&probe,operation.tolerance*0.25,&mut leaves,&mut work,&mut memo,worker);
        (result,leaves,work)
    };
    #[cfg(feature="parallel")]
    let classified=crate::par::map_ordered(&sources,runner,"rest-classify",one)?;
    #[cfg(not(feature="parallel"))]
    let classified={let mut out=Vec::with_capacity(sources.len());for s in &sources {let r=one(s,runner);let failed=r.0.is_err();out.push(r);if failed {break;}}out};
    let mut polygons=Vec::new();let mut work=0usize;
    for (result,leaves,used) in classified {
        work+=used;
        if work>MAX_REST_CLASSIFY {return Err(invalid("rest classification budget exhausted"));}
        result?;
        polygons.extend(leaves);
    }
    if polygons.is_empty() {return Err(invalid("no reachable rest region"));}
    let region=Region::from_polygons(polygons,tolerance.distance_eps);

    let angle=p.pass_angle_deg.to_radians();let (s,c)=angle.sin_cos();
    let uv=|p:Point2|Point2::new(p.x*c+p.y*s,-p.x*s+p.y*c);
    let xy=|u:f64,v:f64|Point3::new(u*c-v*s,u*s+v*c,0.);
    let points:Vec<_>=region.drive.iter().flatten().copied().map(uv).collect();
    let lo=Point2::new(points.iter().map(|p|p.x).fold(f64::INFINITY,f64::min),points.iter().map(|p|p.y).fold(f64::INFINITY,f64::min));
    let hi=Point2::new(points.iter().map(|p|p.x).fold(f64::NEG_INFINITY,f64::max),points.iter().map(|p|p.y).fold(f64::NEG_INFINITY,f64::max));
    let count=((hi.y-lo.y)/p.stepover.distance).ceil() as usize;if count>MAX_SOURCE_SEGMENTS {return Err(invalid("rest pass budget exhausted"));}
    let mut paths=Vec::new();for i in 0..=count {let v=(lo.y+i as f64*p.stepover.distance).min(hi.y);for (a,b) in region.clip(xy(lo.x,v),xy(hi.x,v),runner)? {if a.distance_xy(b)>tolerance.sample_eps {paths.push(vec![Span::Line {start:a,end:b}]);}}}
    let mut out=ctx.generate(&p.containment,&paths,runner)?;
    let (kind,detail,bound)=match &probe {
        Probe::Reference {..}=>("reference-ball-rest-region","Tangent-ball accessibility on mesh facets; all leaf vertices and centroid qualify. Mixed boundary cells omitted. Ball tools and zero allowances only.",resolution),
        Probe::Stock {grid,..}=>("simulated-stock-rest-region","Residual stock height above the selected part at dexel centres; piecewise cell classification, complete leaf CL bounding rectangles plus surface vertices and centroid qualify. Mixed cells omitted. XY stock sampling and mesh projection are approximate; ball tools, zero allowances, same machine frame required.",resolution.max(grid.cell_x).max(grid.cell_y))
    };
    out.report.approximations.push(ApproximationLimit {kind:kind.into(),bound:Some(bound),detail:Some(detail.into())});Ok(out)
}
