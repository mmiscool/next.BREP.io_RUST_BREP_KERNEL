//! The CAM simulation player (lane C17, rows 2-3): a section of the CAM pane
//! that loads a setup's generated operations into a program, plays it
//! against the stock (C7's Z-dexel for 3-axis streams, the tri-dexel when a
//! tool axis tilts), draws the stock, the tool at the scrub position and the
//! collision / gouge markers in the viewport, compares the stock with the
//! part (deviation colour map with a legend), reads the cycle time, stops at
//! a collision when asked, and exports the simulated stock as STL.
//!
//! The simulation advances on the UI thread in bounded chunks per frame
//! (`budget_per_frame` motions) so the pane stays responsive; the runner
//! Command/Reply that moves it to the worker is requested from C1.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use brep_cam::collision::{simulate_checked, simulate_checked_on_machine, CollisionOptions};
use brep_cam::cutter::ToolProfile;
use brep_cam::model::{Bounds3, Machine, Point3, RigidTransform, Setup, Stock};
use brep_cam::kinematics::post::{resolve_program, ResolveOptions};
use brep_cam::kinematics::Chain;
use brep_cam::multiaxis::sim::{
    check_machine, DriverOptions, MachineCheckOptions, Player, PlayState, Position, RangeCutter, SimDriver, StockModel, Timeline, TriDexel,
    TriDexelOptions,
};
use brep_render::runner::{CamSimJob, CamSimRequest};
use brep_cam::program::{assemble, OperationInput, ProgramInput, SetupInput};
use brep_cam::stock::resolve_stock;
use brep_cam::stock_sim::mesh::StockMesh;
use brep_cam::stock_sim::removal::RemovalOptions;
use brep_cam::stock_sim::{DexelGrid, DexelOptions};
use brep_cam::verify::{deviation_map, DeviationMap, DeviationOptions};
use brep_cam::stock_sim::remaining::{remaining_stock_mesh, RemainingStockMesh};
use brep_cam::{BruteForceIndex, Cutter, MeshBuilder, MeshSnapshot, NullSink, Runner, Tolerance, TreeIndex, TreeOptions};
use brep_render::cam_sim::{marks_group, remaining_group, remaining_legend, stock_group, stock_stl, tool_group, Marker, MARKS_GROUP, STOCK_GROUP, TOOL_GROUP};
use brep_render::engine_state::EngineState;
use egui;
use serde_json::{json, Value};

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::cam::CamDocument;
use crate::panels::file::{SaveBody, SaveRequest};

/// One operation's range in the program with its cutter.
#[derive(Clone, Debug)]
pub struct RangeTool {
    pub operation: String,
    pub first: usize,
    pub count: usize,
    pub cutter: Option<Cutter>,
    pub profile: Option<ToolProfile>,
    pub turning: Option<brep_cam::multiaxis::sim::player::TurningRange>,
    /// The operation's tolerance: the remaining-stock bands' edge.
    pub tolerance: f64,
}

/// A loaded simulation.
pub struct SimSession {
    pub setup: String,
    pub player: Player,
    pub driver: SimDriver,
    pub ranges: Vec<RangeTool>,
    pub to_model: RigidTransform,
    /// Collision and gouge markers found by C7's checked simulation (3-axis
    /// streams) at load, with the motions they belong to.
    pub marks: Vec<Marker>,
    pub collision_motions: BTreeSet<usize>,
    /// Every collision found: (motion index in the program, part or body, against).
    pub collisions: Vec<(usize, String, String)>,
    /// The ranges with their cutters, as the stepping function takes them.
    pub range_cutters: Vec<RangeCutter>,
    /// The stock as loaded (the runner's Load and a reload after a lost session).
    load_stock: StockModel,
    /// The machine the session simulates on (sent with the runner's Load).
    sim_machine: Option<Machine>,
    /// `Some` when the background runner holds the stock (C17b); `None`
    /// simulates inline on the UI thread in bounded chunks.
    pub remote: Option<RemoteSim>,
    pub multi_axis: bool,
    /// The part in machine coordinates (deviation).
    pub part: Option<MeshSnapshot>,
    pub deviation: Option<DeviationMap>,
    pub legend: Option<Value>,
    mesh: StockMesh,
    mesh_version: Option<u64>,
    /// The part's index, built once for the remaining-stock overlay.
    part_index: Option<TreeIndex>,
    /// The stock coloured by remaining material (C7's remaining_stock_mesh),
    /// for the stock version and tolerance in `remaining_key`.
    pub remaining: Option<RemainingStockMesh>,
    remaining_key: Option<(u64, u64)>,
    /// Distinguishes sessions in the overlay's stock key (versions restart at 0).
    epoch: u64,
}

static NEXT_EPOCH: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

/// Remaining-stock band edge when an operation has no usable tolerance.
pub const DEFAULT_TOLERANCE: f64 = 0.05;

/// The runner-side session's bookkeeping on the main side.
#[derive(Debug, Default)]
pub struct RemoteSim {
    pub session: u64,
    seq: u64,
    /// The chunk on the runner: (seq, target).
    pub in_flight: Option<(u64, usize)>,
    /// Sequence numbers whose replies are dropped (cancelled chunks).
    dropped: BTreeSet<u64>,
    /// The runner acknowledged the Load.
    pub loaded: bool,
    /// Chunks submitted and replies accepted (for the state blob).
    pub submitted: u64,
    pub accepted: u64,
    pub cancelled: u64,
    /// The runner refused a chunk: the player stops here with the reason and
    /// submits nothing more until the simulation is loaded again. (It used to
    /// resubmit the refused chunk every frame and never catch up.)
    pub failed: Option<String>,
}

static NEXT_SESSION: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);

impl SimSession {
    /// The stock mesh for the driver's current version (rebuilt on change).
    pub fn mesh(&mut self) -> &StockMesh {
        if self.mesh_version != Some(self.driver.version) {
            self.mesh = self.driver.stock.mesh();
            self.mesh_version = Some(self.driver.version);
        }
        &self.mesh
    }

    /// The range (and its tool) a motion belongs to.
    pub fn range_of(&self, motion: usize) -> Option<&RangeTool> {
        self.ranges.iter().find(|r| motion >= r.first && motion < r.first + r.count)
    }

    /// Whether the stock shows the player's completed count with nothing
    /// in flight.
    pub fn caught_up(&self) -> bool {
        self.driver.applied == self.player.completed() && self.remote.as_ref().map_or(true, |r| r.in_flight.is_none() && r.loaded)
    }

    /// Bring the stock toward the player's completed count: inline, at most
    /// `budget` motions now; on the runner, fold landed replies and submit
    /// the next chunk of at most `chunk` motions. `true` while not caught up.
    pub fn drive(&mut self, state: &mut EngineState, budget: usize, chunk: usize) -> Result<bool, String> {
        let target = self.player.completed();
        let tolerance = Tolerance::default();
        let Some(remote) = self.remote.as_mut() else {
            if self.driver.applied != target {
                self.driver
                    .advance_ranges(&self.player.timeline, &self.range_cutters, target, budget, &tolerance, &mut |_| true)
                    .map_err(|e| e.to_string())?;
            }
            return Ok(self.driver.applied != target);
        };
        let _ = state.cam_sim_poll_progress();
        if let Some(failed) = &remote.failed {
            return Err(failed.clone());
        }
        while let Some(reply) = state.cam_sim_poll() {
            if reply.session != remote.session || remote.dropped.remove(&reply.seq) || remote.in_flight.map(|f| f.0) != Some(reply.seq) {
                continue;
            }
            remote.in_flight = None;
            if let Some(error) = reply.error {
                if error.contains("no simulation session") {
                    // The runner lost the session (a worker restart): load again.
                    remote.loaded = false;
                    continue;
                }
                remote.failed = Some(error.clone());
                self.player.pause();
                return Err(error);
            }
            remote.accepted += 1;
            remote.loaded = true;
            let d = &mut self.driver;
            d.applied = reply.applied;
            d.version = reply.version;
            d.removed_volume = reply.removed_volume;
            d.unsupported = reply.unsupported;
            d.warnings = reply.warnings;
            if let Some(stock) = reply.stock {
                d.stock = stock;
                self.mesh_version = None;
            }
        }
        if remote.in_flight.is_none() {
            remote.seq += 1;
            let seq = remote.seq;
            let request = if !remote.loaded {
                Some(CamSimRequest::Load {
                    timeline: self.player.timeline.clone(),
                    stock: self.load_stock.clone(),
                    ranges: self.range_cutters.clone(),
                    options: self.driver.options.clone(),
                    machine: self.sim_machine.clone(),
                })
            } else if self.driver.applied != target {
                Some(CamSimRequest::Advance { target, budget: chunk, stock: true, known_version: self.driver.version })
            } else {
                None
            };
            if let Some(request) = request {
                if let CamSimRequest::Load { .. } = &request {
                    // The view restarts from the loaded stock.
                    self.driver.stock = self.load_stock.clone();
                    self.driver.applied = 0;
                    self.driver.version = 0;
                    self.driver.removed_volume = 0.0;
                    self.driver.unsupported.clear();
                    self.driver.warnings.clear();
                    self.mesh_version = None;
                }
                if state.cam_sim_submit(CamSimJob { session: remote.session, seq, request }) {
                    remote.in_flight = Some((seq, target));
                    remote.submitted += 1;
                } else {
                    return Err("the runner refused the simulation command".into());
                }
            }
        }
        Ok(!self.caught_up())
    }

    /// Stop simulating: the chunk in flight is cancelled (between two
    /// motions on the native thread) and its reply dropped; the stock stays
    /// the last accepted one and the transport pauses there.
    pub fn cancel(&mut self, state: &mut EngineState) -> bool {
        let mut stopped = false;
        if let Some(remote) = self.remote.as_mut() {
            if let Some((seq, _)) = remote.in_flight.take() {
                state.cam_sim_cancel();
                remote.dropped.insert(seq);
                remote.cancelled += 1;
                stopped = true;
            }
        }
        let applied = self.driver.applied;
        if self.player.completed() != applied || stopped {
            self.player.pause();
            self.player.position = if applied == 0 { Position { motion: 0, fraction: 0.0 } } else { Position { motion: applied - 1, fraction: 1.0 } };
            stopped = true;
        }
        stopped
    }

    /// Compare the stock with the part: the deviation map and its legend.
    pub fn compare(&mut self, options: &DeviationOptions, scale: f64) -> Result<(), String> {
        let part = self.part.as_ref().ok_or("no part mesh: the setup's target bodies are not on screen")?;
        let grid = match &self.driver.stock {
            StockModel::Dexel(g) => g.clone(),
            StockModel::TriDexel(t) => t.to_dexel_grid(),
            StockModel::Turning(t) => t.view.clone(),
        };
        let index = BruteForceIndex::build(part);
        let map = deviation_map(&grid, part, &index, options);
        self.legend = Some(brep_render::cam_sim::deviation_legend(&map, scale));
        if let StockModel::Turning(stock)=&self.driver.stock {
            use brep_cam::planar::{Region,Loop};
            let tol=Tolerance::default();
            if !options.tolerance.is_finite() || options.tolerance<=0. {return Err("turning deviation tolerance must be positive".into());}
            let (target_min,target_max)=stock.target.bounds();let (_,stock_max)=stock.section.bounds();
            let domain=Region::new(vec![Loop::rectangle(brep_cam::Point2::new(0.,target_min.y),brep_cam::Point2::new(stock_max.x.max(target_max.x)+1.,stock_max.y.max(target_max.y)+1.))],&tol).map_err(|e|e.to_string())?;
            let machined=stock.section.intersection(&domain,&tol).map_err(|e|e.to_string())?;
            let rear=stock.section.difference(&domain,&tol).map_err(|e|e.to_string())?;
            let missing=stock.target.difference(&machined,&tol).map_err(|e|e.to_string())?;
            let excess=machined.difference(&stock.target,&tol).map_err(|e|e.to_string())?;
            let outer=stock.target.offset(options.tolerance,&tol).map_err(|e|e.to_string())?;
            let inner=stock.target.offset(-options.tolerance,&tol).map_err(|e|e.to_string())?;
            let within=machined.difference(&outer,&tol).map_err(|e|e.to_string())?.is_empty() && inner.difference(&machined,&tol).map_err(|e|e.to_string())?.is_empty();
            let rear_volume=brep_cam::turning::sim::StockState::new(rear,tol).map_err(|e|e.to_string())?.volume();
            self.legend.as_mut().unwrap()["axial"]=json!({"withinTolerance":within,"tolerance":options.tolerance,"machinedFromZ":target_min.y,"machinedExcessArea":excess.area(),"machinedMissingArea":missing.area(),"rearRemainingVolume":rear_volume,"totalRemainingVolume":stock.volume(),"scope":"certified target meridian from its rear face forward; rear bar extension reported separately"});
        }
        self.deviation = Some(map);
        Ok(())
    }

    /// The tolerance of the operation at the playback position.
    pub fn tolerance_here(&self) -> f64 {
        self.range_of(self.player.position.motion).map_or(DEFAULT_TOLERANCE, |r| r.tolerance)
    }

    /// Colour the stock as it stands by remaining material against the part
    /// (C7's remaining_stock_mesh, band edges from the operation tolerance at
    /// the playback position). Recomputed only when the stock version or the
    /// tolerance changed; `true` when it was.
    pub fn refresh_remaining(&mut self) -> Result<bool, String> {
        let tolerance = self.tolerance_here();
        let key = (self.driver.version, tolerance.to_bits());
        if self.remaining_key == Some(key) {
            return Ok(false);
        }
        let part = self.part.as_ref().ok_or("no part mesh: the setup's target bodies are not on screen")?;
        let index = self.part_index.get_or_insert_with(|| TreeIndex::build(part, TreeOptions::default()));
        let grid = match &self.driver.stock {
            StockModel::Dexel(g) => g.clone(),
            StockModel::TriDexel(t) => t.to_dexel_grid(),
            StockModel::Turning(t) => t.view.clone(),
        };
        let options = DeviationOptions { tolerance, max_distance: 1_000.0 };
        self.remaining = Some(remaining_stock_mesh(&grid, part, index, &options));
        self.remaining_key = Some(key);
        Ok(true)
    }

    /// JSON for `cam_sim_state`.
    pub fn state_json(&self) -> Value {
        let p = &self.player;
        let op = p.timeline.operation_of(p.position.motion).map(|i| p.timeline.operations[i].operation.to_string());
        json!({
            "setup": self.setup,
            "multiAxis": self.multi_axis,
            "playing": p.state == PlayState::Playing,
            "speed": p.speed,
            "motion": p.position.motion,
            "fraction": p.position.fraction,
            "time": p.time(),
            "totalSeconds": p.timeline.total_seconds,
            "progress": p.progress(),
            "motions": p.timeline.len(),
            "completed": p.completed(),
            "applied": self.driver.applied,
            "caughtUp": self.caught_up(),
            "runner": self.remote.as_ref().map(|r| json!({ "session": r.session, "loaded": r.loaded, "inFlight": r.in_flight.map(|f| f.1), "submitted": r.submitted, "accepted": r.accepted, "cancelled": r.cancelled, "failed": r.failed })),
            "removedVolume": self.driver.removed_volume,
            "stockVolume": self.driver.stock.volume(),
            "unsupported": self.driver.unsupported,
            "warnings": self.driver.warnings.len(),
            "operation": op,
            "operations": p.timeline.operations.iter().map(|r| json!({ "operation": r.operation.to_string(), "first": r.first, "count": r.count, "seconds": p.timeline.end_times.get(r.first + r.count.saturating_sub(1)).copied().unwrap_or(0.0) - p.timeline.start_time(r.first) })).collect::<Vec<_>>(),
            "marks": self.marks.len(),
            "collisionMotions": self.collision_motions.iter().collect::<Vec<_>>(),
            "collisions": self.collisions.iter().map(|(m, p, a)| json!({ "motion": m, "part": p, "against": a })).collect::<Vec<_>>(),
            "deviation": self.legend,
            "stockVersion": self.driver.version,
            "remaining": self.remaining.as_ref().map(|r| {
                let mut legend = remaining_legend(r);
                legend["stockVersion"] = json!(self.remaining_key.map(|k| k.0));
                legend
            }),
        })
    }
}

/// A target body's display mesh as a CAM snapshot in MODEL coordinates.
fn snapshot_of_solids(state: &EngineState, names: &[String], tolerance: Tolerance) -> Option<MeshSnapshot> {
    let mut builder = MeshBuilder::new(tolerance);
    let mut any = false;
    for name in names {
        let Some(solid) = state.scene.solid(name) else { continue };
        let mesh = &solid.mesh;
        let positions: Vec<f64> = mesh.positions.iter().flat_map(|p| [p[0] as f64, p[1] as f64, p[2] as f64]).collect();
        let max_face = mesh.face_ids.iter().copied().max().map_or(0, |m| m as usize + 1);
        let face_names: Vec<String> = (0..max_face).map(|i| solid.faces.get(i).map(|f| f.name.clone()).unwrap_or_else(|| format!("f{i}"))).collect();
        if builder.add_body_buffers(name, false, &positions, &mesh.indices, &mesh.face_ids, &face_names).is_ok() {
            any = true;
        }
    }
    if any {
        Some(builder.finish())
    } else {
        None
    }
}

/// Model-space bounds of the named solids on screen.
fn bounds_of_solids(state: &EngineState, names: &[String]) -> Bounds3 {
    let mut b = Bounds3::empty();
    for name in names {
        if let Some(solid) = state.scene.solid(name) {
            for p in &solid.mesh.positions {
                b.include(Point3::new(p[0] as f64, p[1] as f64, p[2] as f64));
            }
        }
    }
    b
}

/// Build the session for `setup_id` (or the setup of the first generated
/// operation) from the engine's generation results.
///
/// `toolpaths` replaces an operation's generated toolpath for the session
/// (by operation id; an operation with one needs no generation result).
pub fn build_session(
    state: &EngineState,
    doc: &CamDocument,
    setup_id: Option<&str>,
    cell: f64,
    machine: Option<&Machine>,
    toolpaths: &BTreeMap<String, brep_cam::model::Toolpath>,
) -> Result<SimSession, String> {
    let toolpath_of = |id: &str| -> Option<brep_cam::model::Toolpath> {
        toolpaths.get(id).cloned().or_else(|| state.cam.results.get(id).and_then(|r| r.toolpath.clone()))
    };
    let generated: Vec<&brep_cam::model::Operation> = doc
        .operations
        .iter()
        .filter(|op| toolpaths.contains_key(op.id.as_str()) || state.cam.results.get(op.id.as_str()).map_or(false, |r| r.toolpath.is_some()))
        .collect();
    if generated.is_empty() {
        return Err("nothing to simulate: generate an operation first".into());
    }
    let setup_id = match setup_id {
        Some(id) => id.to_string(),
        None => generated[0].setup.to_string(),
    };
    let setup: &Setup = doc.setup(&setup_id).ok_or_else(|| format!("no setup `{setup_id}`"))?;
    let ops: Vec<&brep_cam::model::Operation> = generated.into_iter().filter(|op| op.setup.as_str() == setup_id).collect();
    if ops.is_empty() {
        return Err(format!("setup `{setup_id}` has no generated operation"));
    }
    let mut inputs = Vec::new();
    let mut multi_axis = false;
    for op in &ops {
        let tool = doc.tool(op.tool.as_str()).ok_or_else(|| format!("operation `{}` names a missing tool", op.id))?;
        let toolpath = toolpath_of(op.id.as_str()).ok_or_else(|| format!("operation `{}` has no toolpath", op.id))?;
        if toolpath.motions.iter().any(|m| m.tool_axis.map_or(false, |a| a.normalized().map_or(false, |u| u.distance(Point3::Z) > 1e-9))) {
            multi_axis = true;
        }
        inputs.push(OperationInput { operation: (*op).clone(), tool: tool.clone(), toolpath });
    }
    let turning_program=inputs.iter().any(|i|matches!(i.operation.strategy,brep_cam::Strategy::TurnFace(_)|brep_cam::Strategy::TurnProfile(_)));
    let turning_machine=if turning_program {machine.cloned().or_else(||doc.setup_machine(&setup_id))}else{None};
    let program = assemble(&ProgramInput { name: format!("sim-{setup_id}"), machine:turning_machine, setups: vec![SetupInput { setup: setup.clone(), operations: inputs }], ..Default::default() })
        .map_err(|e| format!("program: {e}"))?;
    // The machine the session checks: the one given to the player, else the
    // setup's (C17c); its feeds and accelerations time the program.
    let setup_machine = doc.setup_machine(&setup_id);
    let machine = machine.cloned().or(setup_machine);
    let machine = machine.as_ref();
    let timeline = Timeline::from_program(&program, machine.unwrap_or(&Machine::default()));
    // Ranges with their cutters.
    let mut ranges = Vec::new();
    let mut turning_target = None;
    for (section, range) in program.sections.iter().zip(&timeline.operations) {
        let tool = program.tools.iter().find(|t| t.id == section.tool).ok_or("program tool missing")?;
        let tolerance = doc.operation(range.operation.as_str()).map(|o| o.tolerance).filter(|t| t.is_finite() && *t > 0.0).unwrap_or(DEFAULT_TOLERANCE);
        if tool.kind == brep_cam::ToolKind::LatheInsert {
            use brep_cam::planar::{Region,Loop,Segment};
            let operation=doc.operation(section.operation.as_str()).ok_or("turning operation missing")?;
            if !matches!(operation.strategy,brep_cam::Strategy::TurnFace(_)|brep_cam::Strategy::TurnProfile(_)) {return Err("axial simulation supports face/profile turning only".into());}
            let certified=state.cam.results[section.operation.as_str()].turning_profile.as_ref().ok_or("turning result has no certified meridian; regenerate")?;
            let target=Region::from_unoriented(certified.loops.iter().map(|lp|Loop::new(lp.iter().zip(lp.iter().cycle().skip(1)).take(lp.len()).filter(|(a,b)|a!=b).map(|(a,b)|Segment::line(brep_cam::Point2::new(a.x,a.z),brep_cam::Point2::new(b.x,b.z))).collect())).collect(),&Tolerance::default()).map_err(|e|e.to_string())?;
            if turning_target.as_ref().is_some_and(|p|p!=&target) {return Err("turning ranges have different protected profiles".into());}
            turning_target=Some(target.clone());
            let protected=target.offset(-operation.tolerance,&Tolerance::default()).map_err(|e|e.to_string())?;
            let geometry=tool.turning_geometry.as_ref().ok_or_else(||format!("tool {} has no insert geometry",tool.id))?;
            if tool.holder.is_some() {return Err("turning simulation requires an explicit axial holder envelope; mill holder stacks are unsupported".into());}
            let envelope=brep_cam::turning::envelope(tool,geometry,operation.tolerance*0.1).map_err(|e|e.to_string())?;
            let stick=setup.turning_stick_out.ok_or("setup has no stick-out")?;
            let stock=match setup.stock {brep_cam::Stock::Cylinder{base,axis,radius,height}=>brep_cam::StockCylinder{base:setup.model_to_machine.apply_point(base),axis:setup.model_to_machine.apply_vector(axis),radius,height},_=>return Err("turning simulation requires cylindrical stock".into())};
            let chuck_z=brep_cam::turning::profile::bar_section(&stock,stick).map_err(|e|e.to_string())?.chuck_z;
            let turning=brep_cam::multiaxis::sim::player::TurningRange{tool:tool.id.clone(),envelope,holders:Vec::new(),protected,chuck_z};
            ranges.push(RangeTool{operation:range.operation.to_string(),first:range.first,count:range.count,cutter:None,profile:None,turning:Some(turning),tolerance});
        } else {
            let cutter = Cutter::from_tool(tool).map_err(|e| format!("tool `{}`: {e}", tool.id))?;
            let profile = ToolProfile::new(&cutter, tool.holder.as_ref(), 0.01);
            ranges.push(RangeTool { operation: range.operation.to_string(), first: range.first, count: range.count, cutter:Some(cutter), profile:Some(profile), turning:None, tolerance });
        }
    }
    // Stock from the setup and the target bodies on screen.
    let mut target_names: Vec<String> = setup.targets.iter().map(|t| t.solid.clone()).collect();
    for op in &ops {
        for s in &op.targets.target_solids {
            if !target_names.contains(s) {
                target_names.push(s.clone());
            }
        }
    }
    // The stock generation planned against: the setup's start as a generated
    // operation's job resolved it from the bodies' f64 tessellation. The
    // display mesh is f32, an ULP off at the stock's faces, and a carry grows
    // that skin into a full-height wall the shank then "hits".
    let start = ops.iter().find_map(|op| state.cam.results.get(op.id.as_str()).and_then(|r| r.setup_start.as_ref()));
    let target_bounds = start.and_then(|s| s.target_bounds).unwrap_or_else(|| bounds_of_solids(state, &target_names));
    let tolerance = Tolerance::default();
    let from_solid = match &setup.stock {
        Stock::FromSolid { solid } => snapshot_of_solids(state, std::slice::from_ref(solid), tolerance),
        _ => None,
    };
    let resolved = resolve_stock(setup, &target_bounds, from_solid.as_ref()).map_err(|e| e.to_string())?;
    let part = snapshot_of_solids(state, &target_names, tolerance).map(|m| brep_cam::mesh::to_machine(setup, &m));
    // A setup continuing from another starts from that setup's simulated
    // stock carried into this frame: the stock generation planned against
    // (lane C7's job_stock, before this setup's first operation).
    let carried = match &resolved {
        brep_cam::ResolvedStock::Rest(_) if start.is_some_and(|s| s.carried.is_some()) => start.and_then(|s| s.carried.clone()),
        brep_cam::ResolvedStock::Rest(_) => {
            let mesh = part.as_ref().ok_or("the carried stock needs the target bodies on screen")?;
            let previous = crate::panels::cam_generate::setup_history(state, doc, setup);
            let own_machine = doc.setup_machine(setup.id.as_str());
            let job = brep_cam::JobStock { setup, machine: own_machine.as_ref(), tools: &doc.tools, mesh,
                fallback_targets: &target_names, earlier: &[], previous: &previous, tolerance };
            let mut sink = NullSink;
            let result = brep_cam::job_stock(&mut brep_cam::PreOpStockCache::new(previous.len() + 1), &job, &mut Runner::detached(&mut sink))
                .map_err(|e| format!("carried stock from the previous setup: {e}"))?;
            Some(result.stock.grid)
        }
        _ => None,
    };
    // An indexed or rotary stream cuts in the part frame through the
    // machine's chain (C7's `*_on_machine`), which only the Z-dexel does.
    let chain = brep_cam::multiaxis::sim::sim_chain(machine);
    if brep_cam::multiaxis::sim::uses_rotary(&timeline.motions) {
        multi_axis = false;
    }
    let stock = if let Some(target)=turning_target {
        // A mill-turn setup turns, then live-mills the turned stock (the driver
        // switches to its Z-dexel view); turning after milling cannot follow.
        let first_milling=ranges.iter().filter(|r|r.turning.is_none()).map(|r|r.first).min();
        if first_milling.is_some_and(|m|ranges.iter().any(|r|r.turning.is_some()&&r.first>m)) {return Err("turning after milling in one setup is not simulated: split the setup after the first milling operation".into());}
        let brep_cam::ResolvedStock::Cylinder(cylinder)=&resolved else {return Err("turning simulation requires cylinder stock".into());};
        let section=brep_cam::planar::Region::new(vec![brep_cam::planar::Loop::rectangle(brep_cam::Point2::new(0.,cylinder.base.z),brep_cam::Point2::new(cylinder.radius,cylinder.base.z+cylinder.height))],&tolerance).map_err(|e|e.to_string())?;
        let view=DexelGrid::from_stock(&resolved,&DexelOptions{cell,..Default::default()},None,None).map_err(|e|e.to_string())?;
        let mut axial=brep_cam::multiaxis::sim::player::TurningStock{section,target,view};axial.refresh_view();
        StockModel::Turning(axial)
    } else if let Some(grid) = carried {
        if multi_axis {
            StockModel::TriDexel(TriDexel::from_dexel(&grid, &TriDexelOptions { cell, ..Default::default() }).map_err(|e| format!("{e:?}"))?)
        } else {
            StockModel::Dexel(grid)
        }
    } else if multi_axis {
        StockModel::TriDexel(TriDexel::from_stock(&resolved, &TriDexelOptions { cell, ..Default::default() }, None, None).map_err(|e| e.to_string())?)
    } else {
        StockModel::Dexel(DexelGrid::from_stock(&resolved, &DexelOptions { cell, ..Default::default() }, None, None).map_err(|e| format!("{e:?}"))?)
    };
    // Collision and gouge pre-pass (3-axis streams): C7's checked simulation
    // per operation on a scratch grid.
    let mut marks = Vec::new();
    let mut collision_motions = BTreeSet::new();
    let mut collisions: Vec<(usize, String, String)> = Vec::new();
    if let StockModel::Dexel(grid) = &stock {
        let mut scratch = grid.clone();
        for r in &ranges {
            let motions: Vec<brep_cam::model::Motion> = timeline.motions[r.first..r.first + r.count].to_vec();
            let mut sink = NullSink;
            let options = CollisionOptions { stop_at_first: false, ..Default::default() };
            let mut runner = Runner::detached(&mut sink);
            let checked = match &chain {
                Some(chain) => simulate_checked_on_machine(&mut scratch, None, r.cutter.as_ref().unwrap(), r.profile.as_ref().unwrap(), &motions, &tolerance, &RemovalOptions::default(), &options, chain, &mut runner),
                None => simulate_checked(&mut scratch, None, r.cutter.as_ref().unwrap(), r.profile.as_ref().unwrap(), &motions, &tolerance, &RemovalOptions::default(), &options, &mut runner),
            }
            .map_err(|_| "collision check cancelled".to_string())?;
            for c in checked.collisions {
                // The cutting part meeting material on a non-cutting move is a
                // gouge; the shank or holder touching anything is a collision.
                let gouge = matches!(c.part, brep_cam::CollisionPart::CuttingOnRapid);
                marks.push(Marker { at: c.contact, gouge });
                collision_motions.insert(r.first + c.motion_index);
                collisions.push((r.first + c.motion_index, format!("{:?}", c.part), format!("{:?}", c.against)));
            }
        }
    }
    // Machine collision (C10c): the machine's link solids along the resolved
    // motion, against each other and the stock as loaded. The first hit is
    // marked (on the part, in setup coordinates) and joins the stop list.
    let mut machine_notes = Vec::new();
    if let Some(machine) = machine.filter(|m| !m.collision.is_empty()) {
        // The same zero work offset generation and the removal use.
        match Chain::from_machine(machine, Point3::ZERO) {
            Ok(chain) => match resolve_program(&program, &chain, &ResolveOptions::default()) {
                Ok(resolved) => {
                    let tri = match &stock {
                        StockModel::TriDexel(t) => Some(t.clone()),
                        StockModel::Dexel(g) => TriDexel::from_dexel(g, &TriDexelOptions { cell, ..Default::default() }).ok(),
                        StockModel::Turning(t) => TriDexel::from_dexel(&t.view, &TriDexelOptions { cell, ..Default::default() }).ok(),
                    };
                    let mut sink = NullSink;
                    match check_machine(&chain, &machine.collision, &resolved.motions, tri.as_ref(), &MachineCheckOptions::default(), &mut Runner::detached(&mut sink)) {
                        Ok(report) => {
                            if let Some(hit) = report.first {
                                let motion = resolved.motions[hit.block..].iter().find_map(|b| b.source).unwrap_or(hit.block);
                                let on_part = chain.world_to_part(&hit.angles_deg, hit.point);
                                marks.push(Marker { at: on_part, gouge: false });
                                collision_motions.insert(motion);
                                collisions.push((motion, hit.body.clone(), hit.against.clone()));
                            }
                        }
                        Err(e) => machine_notes.push(format!("machine check: {e}")),
                    }
                }
                Err(e) => machine_notes.push(format!("machine kinematics: {e}")),
            },
            Err(e) => machine_notes.push(format!("machine: {e}")),
        }
    }
    let range_cutters: Vec<RangeCutter> = ranges.iter().map(|r| RangeCutter { first: r.first, count: r.count, cutter: r.cutter, profile: r.profile.clone(), turning:r.turning.clone() }).collect();
    if matches!(&stock,StockModel::Turning(_)) {
        let mut checked=SimDriver::new(stock.clone(),DriverOptions::default()).map_err(|e|e.to_string())?;
        // The turning motions only: live milling after them is the player's.
        let turned=range_cutters.iter().filter(|r|r.turning.is_none()).map(|r|r.first).min().unwrap_or(timeline.len());
        checked.advance_ranges(&timeline,&range_cutters,turned,usize::MAX,&tolerance,&mut |_|true).map_err(|e|format!("turning checked replay: {e}"))?;
    }
    let load_stock = stock.clone();
    let mut driver = SimDriver::new(stock, DriverOptions::default()).map_err(|e| e.to_string())?.with_chain(chain);
    for note in machine_notes {
        driver.warnings.push((0, brep_cam::model::Diagnostic::new("machine-check", note)));
    }
    Ok(SimSession {
        setup: setup_id,
        player: Player::new(timeline),
        driver,
        ranges,
        to_model: setup.machine_to_model(),
        marks,
        collision_motions,
        collisions,
        range_cutters,
        load_stock,
        sim_machine: machine.cloned(),
        remote: None,
        multi_axis,
        part,
        deviation: None,
        legend: None,
        mesh: StockMesh::default(),
        mesh_version: None,
        part_index: None,
        remaining: None,
        remaining_key: None,
        epoch: NEXT_EPOCH.fetch_add(1, std::sync::atomic::Ordering::Relaxed),
    })
}

/// How many frames' budgets one runner chunk may hold (see `pump`).
const RUNNER_CHUNK_GROWTH: usize = 8;

/// The pane section's state.
pub struct CamSimPanel {
    hits: HashMap<String, egui::Rect>,
    pub session: Option<SimSession>,
    pub speed: f64,
    pub stop_at_collision: bool,
    pub cell: f64,
    pub budget_per_frame: usize,
    /// `budget_per_frame` was set explicitly (`cam_sim_set budgetPerFrame`):
    /// runner chunks are then exactly that size, never grown.
    pub budget_pinned: bool,
    pub show_deviation: bool,
    /// Colour the stock by remaining material at the playback point.
    pub show_remaining: bool,
    /// Seconds since the remaining-stock overlay was last recomputed.
    remaining_age: f64,
    pub notice: Option<String>,
    /// The stock, tool and marks groups last sent to the viewport.
    overlay: [String; 3],
    /// What the sent stock group was built from; the stock (the whole grid's
    /// mesh) is rebuilt and resent only when it changes, never per frame for
    /// a moving tool.
    overlay_stock_key: Option<String>,
    /// While the stock is catching up or playing, a newer stock version waits
    /// until this time to be resent. A tri-dexel stock group is hundreds of
    /// thousands of vertices and the viewport re-parses it, so resending it
    /// for every 64-motion chunk made the redraw, not the simulation, set the
    /// pace (about 0.85 s per chunk on the 4th-axis part). The stock is
    /// always resent once the player settles.
    overlay_stock_next: Option<web_time::Instant>,
    /// Stock groups built (automation: `overlayStockBuilds`).
    overlay_stock_builds: u64,
    /// An export waiting for the shell's save door.
    pub pending_save: Option<SaveRequest>,
    /// `true` after a load so `cam_sim_state` can report the last error.
    pub error: Option<String>,
    /// The machine the next load checks collisions on (from `cam_sim_load`;
    /// the setup dialog has no machine field yet).
    pub machine: Option<Machine>,
    /// Toolpaths that replace operations' generated ones (by operation id),
    /// kept for later loads like `machine`.
    pub toolpaths: BTreeMap<String, brep_cam::model::Toolpath>,
    /// Simulate inline even when the runner can (tests, comparison).
    pub force_inline: bool,
}

impl Default for CamSimPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl CamSimPanel {
    pub fn new() -> Self {
        Self {
            hits: HashMap::new(),
            session: None,
            speed: 1.0,
            stop_at_collision: true,
            cell: 0.5,
            budget_per_frame: 64,
            budget_pinned: false,
            show_deviation: false,
            show_remaining: false,
            remaining_age: 0.0,
            notice: None,
            overlay: Default::default(),
            overlay_stock_key: None,
            overlay_stock_next: None,
            overlay_stock_builds: 0,
            pending_save: None,
            error: None,
            machine: None,
            toolpaths: BTreeMap::new(),
            force_inline: false,
        }
    }

    /// Load (or reload) the simulation of a setup.
    pub fn load(&mut self, state: &mut EngineState, doc: &CamDocument, setup_id: Option<&str>) -> Result<(), String> {
        if let Some(old) = self.session.as_mut() {
            old.cancel(state);
            if let Some(r) = &old.remote {
                let _ = state.cam_sim_submit(CamSimJob { session: r.session, seq: u64::MAX, request: CamSimRequest::Drop });
            }
        }
        match build_session(state, doc, setup_id, self.cell, self.machine.as_ref(), &self.toolpaths) {
            Ok(mut s) => {
                if !self.force_inline {
                    // Probe the runner with the Load itself: a runner that
                    // cannot simulate declines and the session stays inline.
                    let session = NEXT_SESSION.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    let mut remote = RemoteSim { session, ..Default::default() };
                    remote.seq = 1;
                    let load = CamSimRequest::Load {
                        timeline: s.player.timeline.clone(),
                        stock: s.load_stock.clone(),
                        ranges: s.range_cutters.clone(),
                        options: s.driver.options.clone(),
                        machine: s.sim_machine.clone(),
                    };
                    if state.cam_sim_submit(CamSimJob { session, seq: 1, request: load }) {
                        remote.in_flight = Some((1, 0));
                        remote.submitted = 1;
                        s.remote = Some(remote);
                    }
                }
                s.player.set_speed(self.speed).ok();
                s.player.stop_at_collision = self.stop_at_collision;
                self.notice = Some(format!(
                    "{} motions, {:.1} s, {}{}",
                    s.player.timeline.len(),
                    s.player.timeline.total_seconds,
                    if matches!(&s.driver.stock, StockModel::Turning(_)) { "axisymmetric stock" } else if s.multi_axis { "tri-dexel stock" } else { "Z-dexel stock" },
                    if s.marks.is_empty() { String::new() } else { format!(", {} collision marks", s.marks.len()) }
                ));
                self.session = Some(s);
                self.error = None;
                Ok(())
            }
            Err(e) => {
                self.error = Some(e.clone());
                self.notice = Some(e.clone());
                Err(e)
            }
        }
    }

    pub fn unload(&mut self, state: &mut EngineState) {
        if let Some(mut s) = self.session.take() {
            s.cancel(state);
            if let Some(r) = &s.remote {
                let _ = state.cam_sim_submit(CamSimJob { session: r.session, seq: u64::MAX, request: CamSimRequest::Drop });
            }
        }
        self.overlay = Default::default();
        self.overlay_stock_key = None;
    }

    /// Stop simulating (see [`SimSession::cancel`]).
    pub fn cancel(&mut self, state: &mut EngineState) -> bool {
        let stopped = self.session.as_mut().map_or(false, |s| s.cancel(state));
        if stopped {
            self.notice = Some("simulation stopped: the stock shows the last finished motion".into());
        }
        stopped
    }

    /// Advance the transport by `dt` seconds and the stock toward it; `true`
    /// when another frame is needed. The app calls it once per frame, drawn
    /// or not: called from the pane, a simulation loaded while the CAM pane
    /// was not drawn never read the runner's reply and never caught up.
    pub fn pump(&mut self, state: &mut EngineState, dt: f64) -> bool {
        let stop_at_collision = self.stop_at_collision;
        let budget = self.budget_per_frame;
        // The runner cuts a 64-motion chunk in tens of milliseconds and then
        // waits for the next frame, so a long catch-up was paced by frames
        // (about 0.4 s each on the thin wall), not by cutting. Off the UI
        // thread a chunk may hold RUNNER_CHUNK_GROWTH frames' budgets; a Stop
        // still cancels it between two motions natively, and the browser drops
        // the chunk when it lands.
        let chunk = if self.budget_pinned { budget } else { budget.saturating_mul(RUNNER_CHUNK_GROWTH) };
        let Some(s) = self.session.as_mut() else { return false };
        self.remaining_age += dt;
        let before = s.player.position;
        let moved = s.player.advance(dt);
        if moved && stop_at_collision && s.player.state == PlayState::Playing {
            // Pause at the first collision motion entered since the last frame.
            let entered = (before.motion..=s.player.position.motion).find(|m| s.collision_motions.contains(m) && (*m > before.motion || before.fraction == 0.0));
            if let Some(m) = entered {
                s.player.pause();
                s.player.position = brep_cam::multiaxis::sim::Position { motion: m, fraction: 0.0 };
                self.notice = Some(format!("stopped at motion {m}: collision"));
            }
        }
        match s.drive(state, budget, chunk) {
            Ok(pending) => pending || s.player.state == PlayState::Playing,
            Err(e) => {
                self.notice = Some(e);
                false
            }
        }
    }

    /// Push the viewport groups (stock, tool, marks) when they changed.
    pub fn push_overlay(&mut self, state: &mut EngineState) {
        // The remaining-stock colours follow the playback point: on every stock
        // change while paused, at most twice a second while playing.
        if self.show_remaining {
            if let Some(s) = self.session.as_mut() {
                let playing = s.player.state == PlayState::Playing;
                if !playing || self.remaining_age >= 0.5 {
                    match s.refresh_remaining() {
                        Ok(true) => self.remaining_age = 0.0,
                        Ok(false) => {}
                        Err(e) => {
                            self.notice = Some(e);
                            self.show_remaining = false;
                        }
                    }
                }
            }
        }
        let show_remaining = self.show_remaining;
        let mut built = None;
        let groups = match self.session.as_mut() {
            None => {
                self.overlay_stock_key = None;
                [STOCK_GROUP, TOOL_GROUP, MARKS_GROUP].map(|name| Some(json!({ "name": name }).to_string()))
            }
            Some(s) => {
                let to_model = s.to_model;
                let remaining = show_remaining && s.remaining.is_some();
                let legend = if self.show_deviation { s.legend.as_ref().map(|l| l.to_string()) } else { None };
                let look = format!("{}:{}:{:?}:{:?}", s.epoch, remaining, s.remaining_key, legend);
                let key = format!("{look}:{}", s.driver.version);
                // Only the stock version moved: hold it back while the player
                // is still busy and the last resend's interval has not passed.
                let settled = s.caught_up() && s.player.state != PlayState::Playing;
                let held = !settled
                    && self.overlay_stock_key.as_deref().is_some_and(|k| k.rsplit_once(':').map(|(l, _)| l) == Some(look.as_str()))
                    && self.overlay_stock_next.is_some_and(|t| web_time::Instant::now() < t);
                // An unchanged stock is neither copied nor compared: it is the
                // whole grid's mesh (80 MB of JSON on a thin-wall part), and
                // cloning it every frame cost about 0.1 s per frame.
                let stock = if self.overlay_stock_key.as_deref() == Some(key.as_str()) || held {
                    None
                } else {
                    built = Some(web_time::Instant::now());
                    self.overlay_stock_builds += 1;
                    self.overlay_stock_key = Some(key);
                    match (&s.remaining, remaining) {
                        (Some(r), true) => remaining_group(r, &to_model),
                        _ => {
                            let deviation = if self.show_deviation { s.deviation.clone() } else { None };
                            let scale = s.legend.as_ref().and_then(|l| l["scale"].as_f64()).unwrap_or(1.0);
                            stock_group(s.mesh(), &to_model, deviation.as_ref().map(|d| (d, scale)))
                        }
                    }
                    .to_string()
                    .into()
                };
                let tool = match (s.player.pose(), s.range_of(s.player.position.motion)) {
                    (Some((tip, axis)), Some(r)) if r.profile.is_some() => tool_group(r.profile.as_ref().unwrap(), tip, axis, &to_model),
                    _ => json!({ "name": TOOL_GROUP }),
                };
                [stock, Some(tool.to_string()), Some(marks_group(&s.marks, &to_model).to_string())]
            }
        };
        // Groups are upserted by name: send only the ones that changed.
        let changed: Vec<usize> = (0..3).filter(|&i| groups[i].as_ref().is_some_and(|g| *g != self.overlay[i])).collect();
        if !changed.is_empty() {
            let body: Vec<&str> = changed.iter().map(|&i| groups[i].as_deref().unwrap_or_default()).collect();
            let _ = state.widgets.set_overlay_json(&format!("{{\"groups\":[{}]}}", body.join(",")));
            state.dirty = true;
            for (slot, group) in self.overlay.iter_mut().zip(groups) {
                if let Some(group) = group {
                    *slot = group;
                }
            }
        }
        if let Some(start) = built {
            // The next mid-run resend waits at least twice what this one cost,
            // so redrawing takes at most a third of the time.
            let cost = start.elapsed();
            self.overlay_stock_next = Some(web_time::Instant::now() + (cost * 2).max(web_time::Duration::from_millis(500)));
        }
    }

    /// The simulated stock as STL text (model coordinates).
    pub fn export_stl(&mut self) -> Option<String> {
        let s = self.session.as_mut()?;
        let to_model = s.to_model;
        let name = format!("stock-{}", s.setup);
        Some(stock_stl(s.mesh(), &to_model, &name))
    }

    pub fn state_json(&self) -> Value {
        json!({
            "loaded": self.session.is_some(),
            "speed": self.speed,
            "stopAtCollision": self.stop_at_collision,
            "cell": self.cell,
            "showDeviation": self.show_deviation,
            "showRemaining": self.show_remaining,
            "notice": self.notice,
            "error": self.error,
            "session": self.session.as_ref().map(|s| s.state_json()),
            "overlayStockBuilds": self.overlay_stock_builds,
        })
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// The section's widget rects, merged into the CAM pane's blob.
    pub fn hit_rects(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// The pane section.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, doc: &CamDocument, setup_hint: Option<&str>) {
        self.hits.clear();
        let header = egui::CollapsingHeader::new("Simulation").default_open(true).show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                let load = ui.button(if self.session.is_some() { "Reload" } else { "Load" }).on_hover_text("Simulate the generated operations of the selected setup");
                self.hits.insert("cam:sim:load".into(), load.rect);
                if load.clicked() {
                    let _ = self.load(state, doc, setup_hint);
                }
                let busy = self.session.as_ref().map_or(false, |s| !s.caught_up());
                let stop = ui.add_enabled(busy, egui::Button::new("Stop")).on_hover_text("Stop simulating: keep the stock of the last finished motion");
                self.hits.insert("cam:sim:cancel".into(), stop.rect);
                if stop.clicked() {
                    self.cancel(state);
                }
                let has = self.session.is_some();
                let playing = self.session.as_ref().map_or(false, |s| s.player.state == PlayState::Playing);
                let play = ui.add_enabled(has, egui::Button::new(if playing { "Pause" } else { "Play" }));
                self.hits.insert("cam:sim:play".into(), play.rect);
                if play.clicked() {
                    if let Some(s) = self.session.as_mut() {
                        s.player.toggle();
                    }
                }
                let back = ui.add_enabled(has, egui::Button::new("|<"));
                self.hits.insert("cam:sim:step:back".into(), back.rect);
                if back.clicked() {
                    if let Some(s) = self.session.as_mut() {
                        s.player.step_back();
                    }
                }
                let fwd = ui.add_enabled(has, egui::Button::new(">|"));
                self.hits.insert("cam:sim:step:forward".into(), fwd.rect);
                if fwd.clicked() {
                    if let Some(s) = self.session.as_mut() {
                        s.player.step_forward();
                    }
                }
            });
            if let Some(s) = self.session.as_mut() {
                let mut progress = s.player.progress();
                let slider = ui.add(egui::Slider::new(&mut progress, 0.0..=1.0).show_value(false).text("scrub"));
                self.hits.insert("cam:sim:scrub".into(), slider.rect);
                if slider.changed() {
                    s.player.pause();
                    s.player.scrub(progress);
                }
                ui.horizontal_wrapped(|ui| {
                    let mut speed = self.speed;
                    let drag = ui.add(egui::DragValue::new(&mut speed).range(0.1..=100.0).speed(0.1).prefix("speed ×"));
                    self.hits.insert("cam:sim:speed".into(), drag.rect);
                    if drag.changed() && s.player.set_speed(speed).is_ok() {
                        self.speed = speed;
                    }
                    let stop = ui.checkbox(&mut self.stop_at_collision, "stop at collision");
                    self.hits.insert("cam:sim:stop".into(), stop.rect);
                    s.player.stop_at_collision = self.stop_at_collision;
                });
                let op = s.player.timeline.operation_of(s.player.position.motion).map(|i| s.player.timeline.operations[i].operation.to_string()).unwrap_or_default();
                let time = s.player.time();
                let total = s.player.timeline.total_seconds;
                let line = ui.label(format!(
                    "{op}  {:.1} s / {:.1} s  motion {}/{}  applied {}  removed {:.1}  {}",
                    time,
                    total,
                    s.player.position.motion + 1,
                    s.player.timeline.len(),
                    s.driver.applied,
                    s.driver.removed_volume,
                    if s.caught_up() { "" } else { "(simulating…)" }
                ));
                self.hits.insert("cam:sim:readout".into(), line.rect);
                if !s.marks.is_empty() {
                    ui.colored_label(egui::Color32::from_rgb(230, 90, 60), format!("{} collision / gouge marks (red points)", s.marks.len()));
                }
                if !s.driver.unsupported.is_empty() {
                    ui.colored_label(egui::Color32::from_rgb(230, 160, 60), format!("{} tilted motions not cut by the Z-dexel", s.driver.unsupported.len()));
                }
                ui.horizontal_wrapped(|ui| {
                    let compare = ui.button("Compare to part").on_hover_text("Deviation of the simulated stock from the part: excess blue, gouge red");
                    self.hits.insert("cam:sim:deviation".into(), compare.rect);
                    if compare.clicked() {
                        match s.compare(&DeviationOptions::default(), 1.0) {
                            Ok(()) => self.show_deviation = true,
                            Err(e) => self.notice = Some(e),
                        }
                    }
                    let shown = ui.add_enabled(s.deviation.is_some(), egui::Checkbox::new(&mut self.show_deviation, "colour map"));
                    self.hits.insert("cam:sim:deviation:show".into(), shown.rect);
                    let remaining = ui
                        .add_enabled(s.part.is_some(), egui::Checkbox::new(&mut self.show_remaining, "Remaining stock"))
                        .on_hover_text("Colour the stock at the playback point by the material left over the part: red gouge, green within the operation tolerance, blue up to 10× it, yellow beyond");
                    self.hits.insert("cam:sim:remaining".into(), remaining.rect);
                    let export = ui.button("Export stock (STL)");
                    self.hits.insert("cam:sim:export".into(), export.rect);
                    if export.clicked() {
                        let name = format!("stock-{}.stl", s.setup);
                        let solid = format!("stock-{}", s.setup);
                        let to_model = s.to_model;
                        let stl = stock_stl(s.mesh(), &to_model, &solid);
                        self.pending_save = Some(SaveRequest { title: "Export the simulated stock".into(), file_name: name, body: SaveBody::Text(stl) });
                    }
                });
                if let (true, Some(r)) = (self.show_remaining, &s.remaining) {
                    let tol = r.tolerance;
                    let (_, _, gouge) = r.counts();
                    let line = ui.label(format!(
                        "remaining stock (mm): red < {:.3}  green ±{:.3}  blue ≤ {:.3}  yellow > {:.3}   max excess {:.3}{}",
                        -tol,
                        tol,
                        10.0 * tol,
                        10.0 * tol,
                        r.max.max(0.0),
                        if gouge > 0 { format!("   {gouge} gouged vertices") } else { String::new() }
                    ));
                    self.hits.insert("cam:sim:remaining:legend".into(), line.rect);
                }
                if let Some(legend) = &s.legend {
                    let line = ui.label(format!(
                        "deviation: min {:.3}  max {:.3}  tolerance ±{:.3}  (green within, blue excess, red gouge)",
                        legend["min"].as_f64().unwrap_or(0.0),
                        legend["max"].as_f64().unwrap_or(0.0),
                        legend["tolerance"].as_f64().unwrap_or(0.0)
                    ));
                    self.hits.insert("cam:sim:legend".into(), line.rect);
                }
            }
            if let Some(notice) = &self.notice {
                let label = ui.label(egui::RichText::new(notice).weak());
                self.hits.insert("cam:sim:notice".into(), label.rect);
            }
        });
        self.hits.insert("cam:sim:header".into(), header.header_response.rect);
        self.push_overlay(state);
    }
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "cam", prefix: "cam:sim:header", meaning: "the Simulation section's header", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:load", meaning: "load (or reload) the simulation of the selected setup's generated operations", command: Some("cam_sim_load") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:cancel", meaning: "stop simulating: the stock keeps the last finished motion", command: Some("cam_sim_cancel") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:play", meaning: "play or pause the simulation", command: Some("cam_sim_play") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:step:back", meaning: "step to the start of the current (or previous) motion", command: Some("cam_sim_step") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:step:forward", meaning: "step to the end of the current (or next) motion", command: Some("cam_sim_step") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:scrub", meaning: "the scrub slider over the program's time", command: Some("cam_sim_scrub") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:speed", meaning: "playback speed relative to machining time", command: Some("cam_sim_set") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:stop", meaning: "pause when a collision motion is entered", command: Some("cam_sim_set") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:readout", meaning: "operation, time, motion, applied count and removed volume", command: Some("cam_sim_state") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:deviation", meaning: "compare the simulated stock with the part (deviation colour map)", command: Some("cam_sim_deviation") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:deviation:show", meaning: "show or hide the deviation colour map", command: Some("cam_sim_set") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:remaining", meaning: "colour the stock by remaining material at the playback point (bands from the operation tolerance)", command: Some("cam_sim_set") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:remaining:legend", meaning: "the remaining-stock legend: band edges in mm and the largest excess", command: Some("cam_sim_state") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:export", meaning: "export the simulated stock as STL through the save door", command: Some("cam_sim_export") },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:legend", meaning: "the deviation legend", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:sim:notice", meaning: "what the last simulation action said", command: None },
];

