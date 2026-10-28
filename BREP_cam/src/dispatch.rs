//! ONE entry point over every strategy: the generator the application's
//! worker calls per operation, without knowing which lane wrote the strategy
//! or what inputs it takes.
//!
//! Each lane's `generate` has its own signature — the drop-cutter finishes
//! take a cutter, heights and feeds; the waterline family takes the setup,
//! the tool and its limits; the 2.5D family a [`PlanarInput`] of regions;
//! drilling the recognised holes — so this module builds those inputs from
//! the operation, the setup, the document's tools and the mesh snapshot, and
//! dispatches exhaustively on the strategy. Advanced model inputs arrive in
//! GenerateContext; missing geometry is a named refusal, never a guessed path.
//! Pattern expansion preserves program sections through generate_expanded.
//!
//! Hole recognition needs the kernel (the adapter's revolved faces), which
//! lives on the worker side; the caller recognises holes there and passes
//! them in, so this module stays kernel-free.

use crate::cutter::{Cutter, CutterError};
use crate::error::CamError;
use crate::holes::Hole;
use crate::index::TriangleIndex;
use crate::mesh::MeshSnapshot;
use crate::model::{FeedsSpeeds, Heights, Operation, Setup, Strategy, TargetRef, Tool, Toolpath};
use crate::numeric::Tolerance;
use crate::planar::extract::{face_outline, silhouette};
use crate::planar::machining::{PlanarInput, PlanarRegion};
use crate::Runner;

pub mod chains;

/// What one operation's generation reads.
pub struct GenerateInput<'a> {
    pub operation: &'a Operation,
    pub setup: &'a Setup,
    /// The document's tools: the operation's, and a rest finish's reference tool.
    pub tools: &'a [Tool],
    pub mesh: &'a MeshSnapshot,
    pub index: Option<&'a dyn TriangleIndex>,
    pub tolerance: Tolerance,
    /// The holes recognised on the target bodies (drilling); empty otherwise.
    pub holes: &'a [Hole],
}

fn invalid(message: impl Into<String>) -> CamError {
    CamError::InvalidInput(message.into())
}

/// Resolve every picked face before generating any drilling motion. A face
/// must identify a recognized hole on the same body; duplicate picks do not
/// duplicate a hole. With no picks the established all-holes behavior remains.
fn selected_holes<'a>(operation: &Operation, holes: &'a [Hole]) -> Result<std::borrow::Cow<'a, [Hole]>, CamError> {
    let picks = &operation.targets.drive_faces;
    if picks.is_empty() { return Ok(std::borrow::Cow::Borrowed(holes)); }
    let matches = |pick: &TargetRef, hole: &Hole| pick.solid == hole.solid
        && pick.face.as_ref().is_some_and(|face| hole.faces.contains(face));
    for pick in picks {
        if !holes.iter().any(|hole| matches(pick, hole)) {
            return Err(invalid(format!("{}: picked face {}/{} is not a recognised hole", operation.id, pick.solid, pick.face.as_deref().unwrap_or("<body>"))));
        }
    }
    Ok(std::borrow::Cow::Owned(holes.iter().filter(|hole| picks.iter().any(|pick| matches(pick, hole))).cloned().collect()))
}

/// The operation's tool, from the document's tools.
pub fn tool_of<'a>(operation: &Operation, tools: &'a [Tool]) -> Result<&'a Tool, CamError> {
    tools
        .iter()
        .find(|t| t.id == operation.tool)
        .ok_or_else(|| invalid(format!("operation {} names tool `{}`, which the document does not have", operation.id, operation.tool)))
}

/// The heights an operation cuts at: its own override, else the setup's.
pub fn heights_of(operation: &Operation, setup: &Setup) -> Heights {
    operation.heights.clone().unwrap_or_else(|| setup.heights.clone())
}

/// The feeds an operation cuts at: its own override, else the tool's.
pub fn feeds_of(operation: &Operation, tool: &Tool) -> FeedsSpeeds {
    operation.feeds.clone().unwrap_or_else(|| tool.feeds.clone())
}

fn cutter_of(tool: &Tool) -> Result<Cutter, CamError> {
    Cutter::from_tool(tool).map_err(|e: CutterError| CamError::Cutter(e))
}

/// The 2.5D input for an operation: one planar region per drive face (its
/// projected outline, at the face's top) and, for an operation with no
/// drive faces, the targets' silhouette (the projected union of every
/// triangle, at the body's top); the stock top at the mesh's top. Nothing to
/// outline is the model's empty-target error.
pub fn planar_input(input: &GenerateInput<'_>) -> Result<PlanarInput, CamError> {
    let op = input.operation;
    let mesh = input.mesh;
    let all: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
    let bounds = mesh.bounds_of(&all);
    let mut regions = Vec::new();
    let faces: Vec<TargetRef> = if op.targets.drive_faces.is_empty() {
        op.targets.target_solids.iter().map(|s| TargetRef::solid(s.clone())).collect()
    } else {
        op.targets.drive_faces.clone()
    };
    for target in &faces {
        let face_ids = mesh.resolve_faces(target);
        if face_ids.is_empty() {
            return Err(CamError::UnresolvedTarget { solid: target.solid.clone(), face: target.face.clone() });
        }
        let what = target.face.as_deref().unwrap_or(&target.solid).to_string();
        let (region, z) = if target.is_whole_body() {
            let region = silhouette(mesh, |f| face_ids.contains(&f), &input.tolerance)
                .map_err(|e| invalid(format!("{}: the silhouette of {what} could not be built: {e:?}", op.id)))?;
            let triangles = mesh.triangles_of_faces(&face_ids);
            (region, mesh.bounds_of(&triangles).max.z)
        } else {
            let (region, z_range) = face_outline(mesh, |f| face_ids.contains(&f), &input.tolerance)
                .map_err(|e| invalid(format!("{}: the outline of {what} could not be built: {e:?}", op.id)))?;
            if matches!(op.strategy,Strategy::Slot(_)|Strategy::Chamfer2D(_)) && z_range[1]-z_range[0]>input.tolerance.distance_eps {
                return Err(invalid(format!("{}: selected face must be horizontal",op.strategy.kind_name())));
            }
            (region, z_range[1])
        };
        if region.loops.is_empty() {
            continue;
        }
        regions.push(PlanarRegion { region, z, upward: true });
    }
    if regions.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: op.id.as_str().to_string() });
    }
    // A contour follows the selected face boundary, or the whole-body
    // silhouette. Keep loop winding: internal and external boundaries differ.
    let chains = if let Strategy::Slot(params)=&op.strategy {
        if op.targets.drive_faces.is_empty() {return Err(invalid("slot: select a horizontal rectangular slot floor"));}
        regions.iter().map(|r|slot_floor_chain(r,params.width,&input.tolerance)).collect::<Result<Vec<_>,_>>()?
    } else if matches!(op.strategy, Strategy::Contour2D(_)|Strategy::Chamfer2D(_)) {
        regions.iter().flat_map(|r| r.region.loops.iter().map(move |l|
            crate::planar::machining::PlanarChain { chain: crate::planar::Chain::from_loop(l), z: r.z }
        )).collect()
    } else { Vec::new() };
    Ok(PlanarInput { regions, chains, circles: Vec::new(), stock_top: Some(bounds.max.z) })
}

/// Prefer explicitly planned geometry, never silently replace it with a silhouette.
fn planar_for<'a>(input:&GenerateInput<'_>,context:&GenerateContext<'a>)->Result<std::borrow::Cow<'a,PlanarInput>,CamError> {
    if let Some((_,geometry))=context.resolved_planar {
        return Ok(std::borrow::Cow::Borrowed(geometry));
    }
    let geometry=if matches!(input.operation.strategy,Strategy::Contour2D(_)|Strategy::Slot(_)|Strategy::Trace(_)|Strategy::Chamfer2D(_)|Strategy::Engrave(_)) {chains::selected(input,context)?}else{planar_input(input)?};
    Ok(std::borrow::Cow::Owned(geometry))
}

/// Resolve a straight rectangular floor only. Width is checked against the
/// selected geometry; a bounding box alone is not evidence of a slot.
fn slot_floor_chain(region: &PlanarRegion, width: f64, tol: &Tolerance) -> Result<crate::planar::machining::PlanarChain, CamError> {
    use crate::planar::{Chain, Segment, Vec2};
    use crate::Point2;
    let fail = || invalid("slot: selected floor must be a horizontal rectangle matching slot width");
    if region.region.loops.len()!=1 || !width.is_finite() || width<=0.0 { return Err(fail()); }
    let l=&region.region.loops[0];
    if l.segments.is_empty() || l.segments.iter().any(|s|s.arc.is_some()) { return Err(fail()); }
    let edge=l.segments.iter().max_by(|a,b|a.start.distance(a.end).total_cmp(&b.start.distance(b.end))).unwrap();
    let length=edge.start.distance(edge.end);
    if length<=tol.point_eps { return Err(fail()); }
    let u=Point2::new((edge.end.x-edge.start.x)/length,(edge.end.y-edge.start.y)/length);
    let v=Point2::new(-u.y,u.x);
    let project=|p:Point2| { let d=p.sub(edge.start); (d.dot(u),d.dot(v)) };
    let mut bounds=[f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY];
    for s in &l.segments { let (x,y)=project(s.start); bounds[0]=bounds[0].min(x);bounds[1]=bounds[1].max(x);bounds[2]=bounds[2].min(y);bounds[3]=bounds[3].max(y); }
    let [x0,x1,y0,y1]=bounds;
    let dx=x1-x0;let dy=y1-y0;let eps=tol.distance_eps;
    if dx<=width+eps || (dy-width).abs()>eps { return Err(fail()); }
    for s in &l.segments {
        let (ax,ay)=project(s.start);let (bx,by)=project(s.end);
        let vertical=(ax-bx).abs()<=eps && ((ax-x0).abs()<=eps || (ax-x1).abs()<=eps);
        let horizontal=(ay-by).abs()<=eps && ((ay-y0).abs()<=eps || (ay-y1).abs()<=eps);
        if !vertical && !horizontal {return Err(fail());}
    }
    if (l.signed_area().abs()-dx*dy).abs()>eps*(dx+dy) {return Err(fail());}
    let point=|x:f64|Point2::new(edge.start.x+u.x*x+v.x*(y0+y1)*0.5,edge.start.y+u.y*x+v.y*(y0+y1)*0.5);
    Ok(crate::planar::machining::PlanarChain {chain:Chain::open(vec![Segment::line(point(x0+width*0.5),point(x1-width*0.5))]),z:region.z})
}

/// Additional model data that cannot be reconstructed safely from triangles.
/// The worker resolves these by identity; no arbitrary previous path is chosen.
#[derive(Default)]
pub struct GenerateContext<'a> {
    /// Exact machine-space geometry from recognition/planning, bound to its operation.
    pub resolved_planar: Option<(&'a crate::OperationId, &'a PlanarInput)>,
    pub chain_geometry: Option<&'a dyn chains::ChainGeometry>,
    pub machine: Option<&'a crate::Machine>,
    pub flow_faces: &'a [&'a dyn crate::flow::ParametricFace],
    /// Explicit inner operation, in machine coordinates (unwrapped for RotaryWrap).
    pub source: Option<&'a crate::program::OperationInput>,
    /// MultiAxisFinish source definition to generate on this mesh when no cached path
    /// is supplied. Only self-contained three-axis finishing strategies qualify.
    pub source_operation: Option<&'a Operation>,
    /// Resolved pattern sources, with their original tool/setup identities.
    pub available: &'a [crate::program::OperationInput],
    pub rest_stock: Option<&'a crate::DexelGrid>,
    /// Adapter-resolved stock in machine coordinates (required for FromSolid).
    pub rough_stock: Option<&'a crate::stock::ResolvedStock>,
    /// Explicit larger reference tool for Pocket3D rest roughing. The roughing
    /// model currently has no reference-tool id; never choose one implicitly.
    pub rough_reference_tool: Option<&'a crate::ToolId>,
    pub turning: Option<&'a TurningInput<'a>>,
    pub mill_turn: Option<&'a MillTurnInput<'a>>,
    /// The stock as it stands before this operation (lane C7, `stock_sim::job`),
    /// in the setup's machine frame: the linkers' in-process stock seed.
    /// `None`: the linkers keep above the conservative material top.
    pub pre_op_stock: Option<&'a crate::StockEnvelope>,
}

/// Geometry supplement supplied by the model adapter/feature selection.
/// A supplied profile must include all protected axisymmetric material. Otherwise
/// the public extractor builds it from adapter-certified carriers and the mesh.
pub struct TurningInput<'a> {
    pub tool_id: &'a crate::ToolId,
    pub tool_geometry: &'a crate::turning::TurningGeometry,
    pub carriers: &'a [crate::turning::profile::AxialFace],
    pub profile: Option<&'a crate::turning::profile::TurningProfile>,
    pub contour: Option<&'a crate::turning::profile_path::ProfileInput>,
    pub stick_out: f64,
    pub groove_center_z: Option<f64>,
    pub thread_radius: Option<f64>,
}

/// Live-tool selection is distinct from Strategy: the shared model has no
/// polar/cylindrical strategy variants. Inputs are explicit compensated CL
/// contours and separated rotating/fixed meshes, as required by C11's API.
pub enum MillTurnInput<'a> {
    IndexedDrill { hole: &'a Hole, indexed_mesh: &'a MeshSnapshot },
    Polar { contour: &'a crate::planar::Chain, z:f64, part:&'a MeshSnapshot, fixed:&'a MeshSnapshot },
    Cylindrical { contour: &'a crate::planar::Chain, radius:f64, part:&'a MeshSnapshot, fixed:&'a MeshSnapshot },
}

fn refused(input:&GenerateInput<'_>,why:&str)->CamError {
    invalid(format!("{}: {}: {why}",input.operation.id,input.operation.strategy.kind_name()))
}
fn resolved_machine<'a>(input:&GenerateInput<'_>,context:&GenerateContext<'a>)->Result<Option<&'a crate::Machine>,CamError> {
    match (&input.setup.machine,context.machine) {
        (Some(id),Some(machine)) if *id==machine.id=>Ok(Some(machine)),
        (None,None)=>Ok(None),
        (Some(_),None)=>Err(refused(input,"setup machine must be resolved in GenerateContext")),
        _=>Err(refused(input,"resolved machine identity does not match the setup")),
    }
}
// Mesh and CL inputs here are MACHINE coordinates, not WCS-local S.
// Passing setup.wcs.origin would subtract it twice from table pivots.
fn machine_chain(input:&GenerateInput<'_>,context:&GenerateContext<'_>)->Result<Option<crate::kinematics::Chain>,CamError> {
    Ok(resolved_machine(input,context)?.map(|m|crate::kinematics::Chain::from_machine(m,crate::Point3::ZERO)).transpose()?)
}
fn validated_inner_path<'a>(input:&GenerateInput<'_>,context:&'a GenerateContext<'_>)->Result<&'a Toolpath,CamError> {
    let s=context.source.ok_or_else(||refused(input,"requires an explicit resolved source operation/path in GenerateContext"))?;
    let op=input.operation;
    if !s.operation.enabled || s.operation.id==op.id || s.operation.setup!=op.setup || s.operation.tool!=op.tool || s.tool.id!=op.tool || s.toolpath.operation!=s.operation.id || s.toolpath.tool!=s.tool.id || s.operation.targets!=op.targets || !s.toolpath.report.is_usable() || s.toolpath.motions.is_empty() || s.tool!=*tool_of(op,input.tools)? || !s.toolpath.motions.iter().any(|m|m.kind.is_cutting()) || s.toolpath.motions.iter().any(|m|!m.kind.is_finite()) {
        return Err(refused(input,"source must be a distinct enabled operation with matching setup, tool, targets and usable path"));
    }
    if s.toolpath.motions.iter().any(|m|!matches!(m.kind,crate::MotionKind::Rapid{..}|crate::MotionKind::Cut{..}|crate::MotionKind::Plunge{..}|crate::MotionKind::Retract{..}|crate::MotionKind::LeadIn{..}|crate::MotionKind::LeadOut{..}|crate::MotionKind::Link{..}|crate::MotionKind::Comment{..})) {
        return Err(refused(input,"source must contain linear machining moves; arcs, cycles, probes and machine-control motions cannot be reinterpreted"));
    }
    Ok(&s.toolpath)
}

/// Generate an explicitly selected finishing operation, never a guessed default
/// or a recursive dependency. Cached and generated paths share the same checks.
fn inner_path<'a>(input:&GenerateInput<'_>,context:&'a GenerateContext<'_>,runner:&mut Runner<'_>)->Result<std::borrow::Cow<'a,Toolpath>,CamError> {
    use std::borrow::Cow;
    let Some(source)=context.source_operation else {return validated_inner_path(input,context).map(Cow::Borrowed);};
    if context.source.is_some() {return Err(refused(input,"supply either a source operation or a resolved source path, not both"));}
    let op=input.operation;
    if !source.enabled || source.id==op.id || source.setup!=op.setup || source.tool!=op.tool || source.targets!=op.targets {
        return Err(refused(input,"source must be a distinct enabled operation with matching setup, tool, targets and usable path"));
    }
    if !matches!(source.strategy,Strategy::Parallel(_)|Strategy::Scallop(_)|Strategy::Pencil(_)|Strategy::Radial(_)|Strategy::Spiral(_)|Strategy::MorphedSpiral(_)|Strategy::Project(_)|Strategy::Morph(_)|Strategy::Contour3D(_)|Strategy::SteepShallow(_)|Strategy::Horizontal(_)|Strategy::Ramp(_)) {
        return Err(refused(input,"source operation must be a self-contained three-axis finishing strategy"));
    }
    let source_input=GenerateInput {operation:source,..*input};
    // A source that fails says so: the finish's report names its source.
    let path=generate(&source_input,runner).map_err(|e| match e {CamError::Cancelled=>CamError::Cancelled,e=>refused(input,&format!("source operation {} ({}) failed: {e}",source.id,source.strategy.kind_name()))})?;
    let generated=crate::program::OperationInput {operation:source.clone(),tool:tool_of(source,input.tools)?.clone(),toolpath:path};
    let resolved=GenerateContext {source:Some(&generated),..Default::default()};
    validated_inner_path(input,&resolved)?;
    Ok(Cow::Owned(generated.toolpath))
}

/// Stock bounds are MODEL-space inputs to resolve_stock. Recover them from
/// target vertices, not by inverse-transforming the inflated machine AABB.
fn rough_stock(input:&GenerateInput<'_>,context:&GenerateContext<'_>)->Result<crate::stock::ResolvedStock,CamError> {
    input.setup.model_to_machine.validate().map_err(|e|refused(input,&format!("invalid stock transform: {e}")))?;
    if let Some(stock)=context.rough_stock {return Ok(stock.clone());}
    let mut bounds=crate::Bounds3::empty();
    let inverse=input.setup.model_to_machine.inverse();
    let targets=if input.setup.targets.is_empty() {input.operation.targets.target_solids.iter().map(TargetRef::solid).collect()} else {input.setup.targets.clone()};
    for target in targets.into_iter().filter(|_|matches!(input.setup.stock,crate::Stock::BoundsOffset{..})) {
        let faces=input.mesh.resolve_faces(&target);
        if faces.is_empty() {return Err(CamError::UnresolvedTarget {solid:target.solid,face:target.face});}
        for id in input.mesh.triangles_of_faces(&faces) {
            let triangle=&input.mesh.triangles[id as usize];
            if input.mesh.faces[triangle.face as usize].protected_only {continue;}
            for p in triangle.vertices() {bounds.include(inverse.apply_point(p));}
        }
    }
    crate::stock::resolve_stock(input.setup,&bounds,None).map_err(|e|refused(input,&format!("stock resolution: {e}; FromSolid needs rough_stock in GenerateContext")))
}

fn roughing_path(input:&GenerateInput<'_>,context:&GenerateContext<'_>,tool:&Tool,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    use crate::{RestSource,Stock};
    let op=input.operation;let setup=input.setup;let mesh=input.mesh;let index=input.index;let tol=input.tolerance;
    let (rest,source)=match &op.strategy {
        Strategy::Adaptive2D(p)=>(p.rest_machining,p.rest_source),
        Strategy::Adaptive3D(p)=>(p.rest_machining,p.rest_source),
        Strategy::Pocket3D(p)=>(p.rest_machining,p.rest_source),
        _=>return Err(refused(input,"not a roughing strategy")),
    };
    let grid=if rest && source==RestSource::Stock {Some(context.rest_stock.ok_or_else(||refused(input,"rest machining requires simulated stock in GenerateContext.rest_stock"))?)} else {None};
    if rest && source==RestSource::ReferenceTool && !matches!(op.strategy,Strategy::Pocket3D(_)) {
        return Err(refused(input,"reference-tool rest is supported by Pocket3D; adaptive rest requires RestSource::Stock"));
    }
    if matches!(setup.stock,Stock::FromRest{..}) && grid.is_none() {
        return Err(refused(input,"FromRest setup stock requires rest_machining with RestSource::Stock and a simulated grid"));
    }
    if matches!(op.strategy,Strategy::Adaptive2D(_)) {
        let mut planar=planar_input(input)?;
        let stock=rough_stock(input,context)?;
        planar.stock_top=grid.map(|g|g.z_range[1]).or_else(||stock.top_z());
        return crate::adaptive2d::generate(op,setup,tool,mesh,index,&planar,grid,tol,runner);
    }
    let stock=rough_stock(input,context)?;
    match &op.strategy {
        Strategy::Adaptive3D(_)=>crate::adaptive3d::generate(op,setup,tool,mesh,index,&stock,grid,tol,runner),
        Strategy::Pocket3D(_) if rest && source==RestSource::ReferenceTool=>{
            let id=context.rough_reference_tool.ok_or_else(||refused(input,"reference-tool rest requires explicit rough_reference_tool in GenerateContext"))?;
            let reference=input.tools.iter().find(|t|t.id==*id).ok_or_else(||refused(input,"roughing reference tool is not in the document"))?;
            let cutter=cutter_of(reference)?;
            crate::rest_rough::generate(op,setup,tool,mesh,index,&stock,crate::rest_rough::RestInput::ReferenceTool(&cutter),tol,runner)
        },
        Strategy::Pocket3D(_)=>crate::pocket3d::generate(op,setup,tool,mesh,index,&stock,grid,tol,runner),
        _=>Err(refused(input,"not a 3D roughing strategy")),
    }
}

/// Pattern output retains separate operations/tools/WCS sections. The single
/// Toolpath API refuses this loss of information; workers use this entry point
/// for program expansion and pass its setups to program::assemble.
pub fn generate_expanded(input:&GenerateInput<'_>,context:&GenerateContext<'_>,runner:&mut Runner<'_>)->Result<Vec<crate::program::SetupInput>,CamError> {
    if let Strategy::Pattern(p)=&input.operation.strategy {
        let output=if matches!(p.kind,crate::PatternKind::WcsOffsets{..}) {
            crate::pattern::generate::wcs_setups(input.operation,input.setup,context.available,input.mesh,input.tolerance,runner)?
        }else{
            vec![crate::program::SetupInput {setup:input.setup.clone(),operations:crate::pattern::generate::generate(input.operation,input.setup,context.available,input.mesh,input.tolerance,runner)?}]
        };
        if output.is_empty()||output.iter().any(|s|s.operations.is_empty()||s.operations.iter().any(|o|o.toolpath.motions.is_empty()||!o.toolpath.report.is_usable())) {return Err(refused(input,"pattern expansion produced no usable operations"));}return Ok(output);
    }
    let tool=tool_of(input.operation,input.tools)?.clone();let path=generate_with_context(input,context,runner)?;
    Ok(vec![crate::program::SetupInput {setup:input.setup.clone(),operations:vec![crate::program::OperationInput {operation:input.operation.clone(),tool,toolpath:path}]}])
}

/// Default outside selection follows a single connected certified meridian.
/// Disconnected sections, bores and axial folds require an explicit selection.
fn turning_outer_contour(profile:&crate::turning::profile::TurningProfile,tol:&Tolerance)->Result<crate::turning::profile_path::ProfileInput,CamError> {
    let mut edges:Vec<_>=profile.edges.iter().filter(|e| e.start().x>tol.point_eps && e.end().x>tol.point_eps).cloned().collect();
    let start=edges.iter().flat_map(|e|[e.start(),e.end()]).max_by(|a,b|a.z.total_cmp(&b.z).then(a.x.total_cmp(&b.x))).ok_or_else(||invalid("turning: no outside meridian; select a profile"))?;
    let mut at=start;let mut selected=Vec::new();
    while !edges.is_empty() {
        let candidates:Vec<_>=edges.iter().enumerate().filter_map(|(i,e)| {
            let e=if e.start().distance(at)<=tol.point_eps {e.clone()}else if e.end().distance(at)<=tol.point_eps {e.reversed()}else{return None};
            (e.end().z<=at.z+tol.point_eps).then_some((i,e))
        }).collect();
        if candidates.len()!=1 {return Err(invalid("turning: outside meridian is disconnected, ambiguous or folds; select a profile"));}
        let (i,e)=candidates.into_iter().next().unwrap();at=e.end();edges.remove(i);selected.push(e);
    }
    if start.z-at.z<=tol.point_eps {return Err(invalid("turning: outside meridian has no axial extent"));}
    Ok(crate::turning::profile_path::ProfileInput{contour:crate::turning::profile_path::chain_from_edges(&selected),pilot_radius:0.})
}

fn turning_path(input:&GenerateInput<'_>,context:&GenerateContext<'_>,tool:&Tool,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    use crate::{MachineKind,Strategy,Stock,StockCylinder};
    use crate::turning;
    let machine=resolved_machine(input,context)?.ok_or_else(||refused(input,"requires a resolved lathe or mill-turn machine; generic setup is a mill"))?;
    if !matches!(machine.kind,MachineKind::Lathe|MachineKind::MillTurn{..}) {return Err(refused(input,"requires a lathe or mill-turn setup, not a milling setup"));}
    let geometry=context.turning.ok_or_else(||refused(input,"requires certified turning profile/carriers, insert geometry and stick-out in GenerateContext"))?;
    if *geometry.tool_id!=tool.id {return Err(refused(input,"turning geometry belongs to a different tool"));}
    let setup=input.setup;let op=input.operation;let transform=&setup.model_to_machine;
    let stock=match &setup.stock {Stock::Cylinder{base,axis,radius,height}=>StockCylinder{base:transform.apply_point(*base),axis:transform.apply_vector(*axis),radius:*radius,height:*height},_=>return Err(refused(input,"requires cylindrical setup stock with explicit stick-out"))};
    turning::profile::bar_section(&stock,geometry.stick_out)?;
    let extracted;
    let profile=if let Some(p)=geometry.profile {p}else{extracted=turning::profile::extract(geometry.carriers,input.mesh,op.tolerance*0.125,runner)?;&extracted};
    let g=geometry.tool_geometry;let stick=geometry.stick_out;
    match &op.strategy {
        Strategy::TurnFace(_)=>turning::face::generate(op,setup,tool,g,&stock,stick,profile,runner),
        Strategy::TurnPartOff(_)=>turning::part_off::generate(op,setup,tool,g,&stock,stick,profile,runner),
        Strategy::TurnGroove(_)=>turning::groove::generate(op,setup,tool,g,&stock,stick,profile,geometry.groove_center_z.ok_or_else(||refused(input,"requires selected groove centre Z"))?,runner),
        Strategy::TurnThread(_)=>turning::thread_generation::generate(op,setup,tool,g,&stock,stick,profile,geometry.thread_radius.ok_or_else(||refused(input,"requires selected thread nominal radius"))?,runner).map(|p|p.canned),
        Strategy::TurnProfile(p)=>{
            let derived;
            let contour=if let Some(contour)=geometry.contour {contour}else if p.outside && op.targets.chains.is_empty() && op.targets.drive_faces.is_empty() {
                derived=turning_outer_contour(profile,&input.tolerance)?;&derived
            }else{return Err(refused(input,"requires selected front-to-back profile contour and explicit pilot-bore radius"));};
            let direction=match p.direction {crate::RoughingDirection::Axial=>turning::profile_path::RoughingDirection::Axial,crate::RoughingDirection::Radial=>turning::profile_path::RoughingDirection::Radial};
            match (p.roughing,p.finishing) {
                (true,true)=>turning::profile_finish::generate_combined(op,setup,tool,g,&stock,stick,profile,contour,direction,input.tolerance,runner),
                (true,false)=>turning::profile_rough::generate(op,setup,tool,g,&stock,stick,profile,contour,direction,input.tolerance,runner),
                (false,true)=>turning::profile_finish::generate(op,setup,tool,g,&stock,stick,profile,contour,input.tolerance,runner),
                _=>Err(refused(input,"enable roughing or finishing")),
            }
        },
        _=>Err(refused(input,"not a turning strategy")),
    }
}

/// Live axial milling keeps the compensated Cartesian path and asks the
/// controller's polar interpolator to drive spindle C. No synthetic C angles
/// are attached to a path that was generated in a different frame.
fn live_milling_path(input:&GenerateInput<'_>,context:&GenerateContext<'_>,tool:&Tool,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let machine=resolved_machine(input,context)?.ok_or_else(||refused(input,"live tooling requires a resolved mill-turn machine"))?;
    crate::turning::millturn::chain(machine,crate::Point3::ZERO)?;
    if input.mesh.faces.iter().any(|f|f.protected_only) {
        return Err(refused(input,"automatic polar milling needs an explicit rotating-part/fixed-fixture selection when protected bodies are present"));
    }
    if matches!(&machine.kind,crate::MachineKind::MillTurn{c_axis,..} if c_axis.limits_deg.is_some()) {
        return Err(refused(input,"automatic polar milling requires unrestricted C travel; use an explicit indexed selection"));
    }
    let op=input.operation;
    if op.tool_axis.is_some_and(|axis|axis.distance(crate::Point3::Z)>input.tolerance.angle_eps) {
        return Err(refused(input,"axial live milling requires the spindle Z tool axis"));
    }
    let feeds=op.feeds.as_ref().unwrap_or(&tool.feeds);
    if feeds.css {return Err(refused(input,"live milling requires fixed tool spindle rpm"));}
    if !matches!(op.strategy,Strategy::Contour2D(_)|Strategy::Trace(_)) {return Err(refused(input,"live milling strategy requires a supported axial contour"));}
    let geometry=planar_for(input,context)?;
    let mut path=match op.strategy {
        Strategy::Contour2D(_)=>crate::contour2d::generate(op,input.setup,tool,input.mesh,input.index,geometry.as_ref(),input.tolerance,runner),
        Strategy::Trace(_)=>crate::trace::generate(op,input.setup,tool,input.mesh,input.index,geometry.as_ref(),input.tolerance,runner),
        _=>return Err(refused(input,"live milling strategy requires a supported axial contour")),
    }?;
    let mut previous:Option<crate::Point3>=None;
    for motion in &path.motions {
        if let Some(p)=motion.kind.end() {
            crate::turning::millturn::validate_position(machine,crate::Point3::new(p.x.hypot(p.y),0.,p.z),input.tolerance)?;
            if motion.kind.is_cutting() {
                if let Some(from)=previous {
                    let samples=match motion.kind {
                        crate::MotionKind::ArcCw{to,center,plane,pitch}|crate::MotionKind::ArcCcw{to,center,plane,pitch}=> {
                            if plane!=crate::ArcPlane::Xy {return Err(refused(input,"automatic polar milling requires XY arcs"));}
                            // Bound the complete circular arc, not merely its
                            // endpoints, against physical radial travel.
                            let radius=(from.x-center.x).hypot(from.y-center.y);
                            for r in [(center.x.hypot(center.y)-radius).max(0.),center.x.hypot(center.y)+radius] {
                                for z in [from.z,to.z] {crate::turning::millturn::validate_position(machine,crate::Point3::new(r,0.,z),input.tolerance)?;}
                            }
                            let mut points=vec![from];
                            points.extend(crate::multiaxis::linearize_arc(from,to,center,plane,pitch,matches!(motion.kind,crate::MotionKind::ArcCw{..}),op.tolerance*0.125));points
                        },
                        _=>vec![from,p],
                    };
                    for pair in samples.windows(2) {
                        runner.work(1,"live-polar-check",0,0)?;
                        let edge=crate::planar::Segment::line(crate::Point2::new(pair[0].x,pair[0].y),crate::Point2::new(pair[1].x,pair[1].y));
                        if edge.distance(crate::Point2::new(0.,0.))<=op.tolerance {return Err(refused(input,"polar live contour crosses the spindle singularity"));}
                    }
                }
            }
            previous=Some(p);
        }
    }
    path.motions.insert(0,crate::Motion::new(crate::MotionKind::PolarInterpolation{enabled:true},crate::Feed::RAPID));
    path.motions.push(crate::Motion::new(crate::MotionKind::PolarInterpolation{enabled:false},crate::Feed::RAPID));
    path.report.counts.motions=path.motions.len() as u64;
    Ok(path)
}

fn mill_turn_path(input:&GenerateInput<'_>,context:&GenerateContext<'_>,selection:&MillTurnInput<'_>,tool:&Tool,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    if input.operation.setup!=input.setup.id {return Err(refused(input,"operation setup identity mismatch"));}
    let machine=resolved_machine(input,context)?.ok_or_else(||refused(input,"live tooling requires a resolved mill-turn machine"))?;
    crate::turning::millturn::chain(machine,crate::Point3::ZERO)?;
    match selection {
        MillTurnInput::IndexedDrill{hole,indexed_mesh}=>{
            if !matches!(input.operation.strategy,Strategy::Drill(_)){return Err(refused(input,"indexed live drilling requires Drill strategy"));}
            selected_holes(input.operation,std::slice::from_ref(*hole))?;
            crate::turning::millturn::drilling::generate_hole(input.operation,input.setup,machine,tool,hole,indexed_mesh,input.tolerance,runner).map(|p|p.canned)
        },
        MillTurnInput::Polar{contour,z,part,fixed}=>{
            if !matches!(input.operation.strategy,Strategy::Contour2D(_)|Strategy::Trace(_)){return Err(refused(input,"polar live milling requires Contour2D or Trace with a compensated contour"));}
            crate::turning::millturn::milling::polar(input.operation,machine,tool,contour,*z,part,fixed,input.tolerance,runner).map(|p|p.0)
        },
        MillTurnInput::Cylindrical{contour,radius,part,fixed}=>{
            if !matches!(input.operation.strategy,Strategy::Contour2D(_)|Strategy::Trace(_)){return Err(refused(input,"cylindrical live milling requires Contour2D or Trace with a compensated contour"));}
            crate::turning::millturn::milling::cylindrical(input.operation,machine,tool,contour,*radius,part,fixed,input.tolerance,runner).map(|p|p.0)
        },
    }
}

/// Generate one operation's toolpath.
pub fn generate(input: &GenerateInput<'_>, runner: &mut Runner<'_>) -> Result<Toolpath, CamError> {
    generate_with_context(input,&GenerateContext::default(),runner)
}

/// Generate with resolved model inputs. Existing single-operation arms are unchanged.
pub fn generate_with_context(input:&GenerateInput<'_>,context:&GenerateContext<'_>,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    // The linkers below read the pre-operation stock from this scope (lane C4).
    // ... and the finishing adapter the tool's holder (lane C16's shank and
    // holder certification against that stock).
    let holder=input.tools.iter().find(|t|t.id==input.operation.tool).and_then(|t|t.holder.as_ref());
    crate::link::with_pre_op_stock(context.pre_op_stock,||crate::strategy_linking::with_tool_holder(holder,||generate_in_context(input,context,runner)))
}

fn generate_in_context(input:&GenerateInput<'_>,context:&GenerateContext<'_>,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    runner.token().check()?;
    if context.resolved_planar.is_some() && !matches!(input.operation.strategy,Strategy::Face(_)|Strategy::Pocket2D(_)|Strategy::Contour2D(_)|Strategy::Slot(_)|Strategy::Trace(_)|Strategy::Chamfer2D(_)|Strategy::Engrave(_)|Strategy::Bore(_)|Strategy::Circular(_)|Strategy::ThreadMill(_)) {
        return Err(refused(input,"resolved planar geometry is not supported by this strategy"));
    }
    if let Some((operation,_))=context.resolved_planar {
        if operation!=&input.operation.id {return Err(refused(input,"resolved planar geometry belongs to another operation"));}
        if !input.operation.targets.chains.is_empty() {return Err(refused(input,"supply resolved planar geometry or stored chains, not both"));}
    }
    if matches!(input.operation.strategy,Strategy::ManualNc(_)) {return crate::pattern::manual_nc::generate(input.operation,runner);}
    if matches!(input.operation.strategy,Strategy::Pattern(_)) {return Err(refused(input,"requires generate_expanded to preserve source tool and WCS sections"));}
    let op = input.operation;
    let setup = input.setup;
    let mesh = input.mesh;
    let index = input.index;
    let tol = input.tolerance;
    if !op.targets.chains.is_empty() && !matches!(op.strategy,Strategy::Contour2D(_)|Strategy::Slot(_)|Strategy::Trace(_)|Strategy::Chamfer2D(_)|Strategy::Engrave(_)) {
        return Err(refused(input,"selected chains are not supported by this strategy"));
    }
    if op.targets.target_solids.is_empty() && op.targets.chains.is_empty() {
        return Err(CamError::EmptyTarget { operation_id: op.id.as_str().to_string() });
    }
    let tool = tool_of(op, input.tools)?;
    let heights = heights_of(op, setup);
    let feeds = feeds_of(op, tool);
    if let Some(selection)=context.mill_turn {return mill_turn_path(input,context,selection,tool,runner);}
    if op.live_tool==Some(true) && !context.machine.is_some_and(|m|matches!(m.kind,crate::MachineKind::MillTurn{..})) {
        return Err(refused(input,"live tooling requires a resolved mill-turn machine"));
    }
    if let Some(machine)=context.machine {
        resolved_machine(input,context)?;
        if matches!(machine.kind,crate::MachineKind::MillTurn{..}) && !op.strategy.is_turning() && !op.strategy.is_targetless() && op.live_tool.unwrap_or(true) {
            return live_milling_path(input,context,tool,runner);
        }
        if matches!(machine.kind,crate::MachineKind::Lathe) && matches!(op.strategy,Strategy::Drill(_)) {
            let holes = selected_holes(op, input.holes)?;
            return crate::turning::drill::generate(op,setup,tool,&holes,mesh,tol,runner).map(|p|p.canned);
        }
        if matches!(machine.kind,crate::MachineKind::Lathe|crate::MachineKind::MillTurn{..}) && !matches!(op.strategy,Strategy::TurnFace(_)|Strategy::TurnProfile(_)|Strategy::TurnGroove(_)|Strategy::TurnThread(_)|Strategy::TurnPartOff(_)) {
            return Err(refused(input,"this setup requires an explicit live-tool selection or a turning strategy"));
        }
    }
    if let Strategy::RestFinish(p)=&op.strategy {if p.source==crate::RestSource::Stock {
        let stock=context.rest_stock.ok_or_else(||refused(input,"stock rest finishing requires the previous simulated DexelGrid"))?;
        return crate::rest_finish::generate_from_stock(op,&cutter_of(tool)?,mesh,index,&heights,&feeds,tol,runner,stock);
    }}
    match &op.strategy {
        // --- 3D finishing on the drop cutter ---
        Strategy::Parallel(_) => crate::parallel::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Scallop(_) => crate::scallop::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Pencil(_) => crate::pencil::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Radial(_) => crate::radial::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Spiral(_) => crate::spiral::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::MorphedSpiral(_) => crate::morphed_spiral::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Project(_) => crate::project::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::Morph(_) => crate::morph::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        Strategy::RestFinish(p) => {
            let reference = input
                .tools
                .iter()
                .find(|t| t.id == p.reference_tool)
                .ok_or_else(|| invalid(format!("{}: reference tool `{}` is not in the document", op.id, p.reference_tool)))?;
            crate::rest_finish::generate(op, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner, &cutter_of(reference)?)
        }
        // --- the waterline family ---
        Strategy::Contour3D(_) => crate::waterline::contour::generate(op, setup, tool, mesh, tol, Default::default(), runner),
        Strategy::SteepShallow(_) => crate::waterline::steep_shallow::generate(op, setup, tool, mesh, tol, Default::default(), runner),
        Strategy::Horizontal(_) => crate::waterline::horizontal::generate(op, setup, tool, mesh, tol, runner),
        Strategy::Ramp(_) => crate::waterline::ramp::generate(op, setup, tool, mesh, tol, Default::default(), runner),
        // --- multi-axis ---
        Strategy::Swarf(_) => crate::swarf::generate(op, &cutter_of(tool)?, tool.holder.as_ref(), mesh, index, &heights, &feeds, tol, runner),
        Strategy::MultiAxisContour(_) => {
            crate::multiaxis_contour::generate(op, &cutter_of(tool)?, tool.holder.as_ref(), mesh, index, &heights, &feeds, tol, runner)
        }
        // --- 2.5D on planar regions ---
        Strategy::Face(_) => crate::face::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Pocket2D(_) => crate::pocket2d::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Contour2D(_) => crate::contour2d::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Slot(_) => crate::slot::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Trace(_) => crate::trace::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Chamfer2D(_) => crate::chamfer2d::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Engrave(_) => crate::engrave::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Bore(_) => crate::bore::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::Circular(_) => crate::circular::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        Strategy::ThreadMill(_) => crate::thread_mill::generate(op, setup, tool, mesh, index, planar_for(input,context)?.as_ref(), tol, runner),
        // --- drilling: the canned stream (cycles); the simulation lane expands it ---
        Strategy::Drill(_) => {
            let holes = selected_holes(op, input.holes)?;
            if holes.is_empty() {
                return Err(invalid(format!("{}: no hole was recognised on its target bodies", op.id)));
            }
            crate::drill::strategy::generate(op, setup, tool, &holes, mesh, tol, runner).map(|out| out.canned)
        }
        // --- probing (lane C14) and the rotary finish (lane C10) ---
        Strategy::ProbeWcs(_) => crate::probe_wcs::generate(op, setup, tool, mesh, tol, runner),
        Strategy::ProbeGeometry(_) => crate::probe_geometry::generate(op, setup, tool, mesh, tol, runner),
        Strategy::RotaryFinish(p) => crate::rotary::generate_rotary_finish(op, p, &cutter_of(tool)?, mesh, index, &heights, &feeds, tol, runner),
        // --- operations requiring additional resolved model inputs ---
        Strategy::MultiAxisFinish(p) => {
            let inner=inner_path(input,context,runner)?;
            let chain=machine_chain(input,context)?;
            let machine=chain.as_ref().map(|chain|crate::multiaxis_finish::MachineCheck {chain,options:Default::default()});
            crate::multiaxis_finish::generate(op,p,&inner,&cutter_of(tool)?,tool.holder.as_ref(),mesh,index,machine.as_ref(),&heights,&feeds,tol,runner)
        },
        Strategy::Adaptive2D(_) | Strategy::Adaptive3D(_) | Strategy::Pocket3D(_) => {
            roughing_path(input,context,tool,runner)
        }
        Strategy::Flow(_) => {
            if context.flow_faces.is_empty(){return Err(refused(input,"requires selected parametric face samplers from the geometry adapter"));}
            crate::flow::generate(op,&cutter_of(tool)?,mesh,index,&heights,&feeds,tol,runner,context.flow_faces)
        },
        Strategy::RotaryWrap(p) => {
            if context.source_operation.is_some() {return Err(refused(input,"requires a precomputed unwrapped source path; source_operation is only for MultiAxisFinish"));}
            crate::rotary::wrap_toolpath(op,p,validated_inner_path(input,context)?,&cutter_of(tool)?,mesh,index,tol,runner)
        },
        Strategy::TurnFace(_) | Strategy::TurnProfile(_) | Strategy::TurnGroove(_) | Strategy::TurnThread(_) | Strategy::TurnPartOff(_) => {
            turning_path(input,context,tool,runner)
        }
        Strategy::ManualNc(_) => crate::pattern::manual_nc::generate(op,runner),
        Strategy::Pattern(_) => Err(refused(input,"requires generate_expanded to preserve source tool and WCS sections")),
    }
}

