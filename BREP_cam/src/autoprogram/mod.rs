//! Automatic programming (lane C13 rows 3 and 4): recognised features ->
//! a setup's ordered operations through the technology table, tools from
//! the library, feeds from lane C12, generation through the existing
//! strategies, program assembly, and the report of what was recognised,
//! programmed and skipped.
//!
//! Ordering: facing, roughing by descending cutter diameter, finishing
//! (including chamfers), then holes: spotting, drilling, tapping/thread milling.
//! Tool reuse is stable within a phase and cannot override roughing diameter
//! or hole dependencies. Tool number and ID break ties deterministically.

use crate::features::techdb::{Fit, OperationRule, TechDb};
use crate::features::{Feature, FeatureKind, Recognized, WallLine};
use crate::feeds::WorkMaterial;
use crate::holes::Hole;
use crate::library::LibraryUnits;
use crate::model::{
    Chamfer2dParams, Contour2dParams, CutDirection, Depths, FaceParams, Lead, LeadKind, Machine, MachineKind,
    Operation, OperationId, Pocket2dParams, Setup, SlotParams, Stepover, Strategy, TargetSelection, Tool, ToolId,
    ToolKind, Toolpath,
};
use crate::planar::machining::{PlanarChain, PlanarInput, PlanarRegion};
use crate::planar::{chain_capsule, Chain, Loop, Region, Vec2};
use crate::{CamError, Cutter, CutterProfile, MeshSnapshot, Runner, Tolerance, TriangleIndex};
use serde::{Deserialize, Serialize};

mod hole_ops;
pub mod multi;
pub mod access;
pub mod binding;
pub mod report;
pub use report::{report, Report};

/// What the planner reads.
pub struct AutoInput<'a> {
    pub recognized: &'a Recognized,
    pub setup: &'a Setup,
    /// The tool library (every tool must validate; see `library::validate_tool`).
    pub tools: &'a [Tool],
    /// The machine for feeds and speeds (`None`: a generic 3-axis mill).
    pub machine: Option<&'a Machine>,
    pub material: WorkMaterial,
    pub techdb: &'a TechDb,
    pub units: LibraryUnits,
    /// Machine Z of the stock top when known.
    pub stock_top: Option<f64>,
}

/// The geometry a planned operation is generated from.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum OperationGeometry {
    Planar(PlanarInput),
    Holes(Vec<Hole>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct PlannedOperation {
    pub operation: Operation,
    pub tool: ToolId,
    pub geometry: OperationGeometry,
    pub feature: String,
    pub role: String,
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Skipped {
    pub feature: String,
    pub kind: String,
    pub reason: String,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Plan {
    pub operations: Vec<PlannedOperation>,
    pub skipped: Vec<Skipped>,
    pub tool_changes: u32,
}

fn invalid(m: impl std::fmt::Display) -> CamError {
    CamError::InvalidInput(format!("autoprogram: {m}"))
}

fn phase(role: &str) -> u8 {
    match role {
        "face" => 0,
        "rough" => 1,
        "finish" | "chamfer" => 2,
        "spot" => 3,
        "drill" => 4,
        "tap" => 5,
        _ => 6,
    }
}

fn material_name(m: WorkMaterial) -> &'static str {
    match m {
        WorkMaterial::Aluminium => "aluminium",
        WorkMaterial::MildSteel => "mildSteel",
        WorkMaterial::AlloySteel => "alloySteel",
        WorkMaterial::StainlessSteel => "stainlessSteel",
        WorkMaterial::CastIron => "castIron",
        WorkMaterial::Brass => "brass",
        WorkMaterial::Titanium => "titanium",
        WorkMaterial::Plastics => "plastics",
        WorkMaterial::Wood => "wood",
        WorkMaterial::Mdf => "mdf",
    }
}

/// The feature's depth, width constraints and corner radius for tool fitting.
struct Fitting<'a> {
    depth: f64,
    /// The tool must fit this region inward by its radius (plus a hair).
    region: Option<&'a Region>,
    corner_radius: Option<f64>,
    /// The tool diameter may not exceed this (slot width, chamfer cone size...).
    max_diameter: Option<f64>,
    exact_diameter: Option<f64>,
}

/// Choose a tool for a rule: the largest of the rule's kinds that fits.
fn choose_tool<'a>(rule: &OperationRule, tools: &'a [Tool], fit: &Fitting<'_>, clearance: Option<&(Region, Region)>, tol: &Tolerance) -> Result<&'a Tool, String> {
    let mut candidates: Vec<&Tool> = tools.iter().filter(|t| rule.tool.kinds.contains(&t.kind)).collect();
    if candidates.is_empty() {
        return Err(format!("the library has no {:?} tool", rule.tool.kinds));
    }
    candidates.sort_by(|a, b| b.geometry.diameter.total_cmp(&a.geometry.diameter).then(a.number.cmp(&b.number)));
    let mut why = Vec::new();
    for t in candidates {
        let d = t.geometry.diameter;
        if t.geometry.cutting_length + tol.distance_eps < fit.depth {
            why.push(format!("{}: cutting length {} below the depth {}", t.id, t.geometry.cutting_length, fit.depth));
            continue;
        }
        // An island finisher needs room for the whole cutter, not only
        // its centre line. Even a "largest" rule cannot override this.
        if let Some((outline, allowed)) = clearance {
            let margin = crate::pocket2d::wall_margin(tol, rule.tolerance);
            let envelope = outline.offset(d + rule.stock_to_leave.radial + 2.0 * margin, tol)
                .and_then(|expanded| expanded.difference(allowed, tol));
            match envelope {
                Ok(outside) if outside.area() <= tol.area_eps => {},
                Ok(_) => { why.push(format!("{}: cutter sweep does not fit around the island", t.id)); continue; },
                Err(e) => { why.push(format!("{}: island clearance could not be certified: {e}", t.id)); continue; },
            }
        }
        match rule.tool.fit {
            Fit::Largest => return Ok(t),
            Fit::ExactDiameter => {
                let want = fit.exact_diameter.ok_or("no exact diameter for the feature")?;
                if (d - want).abs() <= 1e-6 {
                    return Ok(t);
                }
                why.push(format!("{}: diameter {} is not {}", t.id, d, want));
            }
            Fit::LargestFitting => {
                if let Some(m) = fit.max_diameter {
                    if d > m + 1e-9 {
                        why.push(format!("{}: diameter {} above {}", t.id, d, m));
                        continue;
                    }
                }
                if let Some(r) = fit.corner_radius {
                    if 0.5 * d > r + 1e-9 {
                        why.push(format!("{}: radius {} above the corner radius {}", t.id, 0.5 * d, r));
                        continue;
                    }
                }
                if let Some(region) = fit.region {
                    let inner = region.offset(-(0.5 * d + crate::pocket2d::wall_margin(tol, rule.tolerance)), tol).map_err(|e| e.to_string())?;
                    if inner.is_empty() {
                        why.push(format!("{}: diameter {} does not fit the region", t.id, d));
                        continue;
                    }
                }
                return Ok(t);
            }
        }
    }
    Err(format!("no fitting tool: {}", why.join("; ")))
}

/// A closed loop as a planar chain (the loop's own direction).
fn loop_chain(l: &Loop) -> Chain {
    Chain { segments: l.segments.clone(), closed: true }
}

/// The region a pocket is cleared over: closed pockets their floor; open
/// pockets and steps the floor extended past every open edge by the tool
/// radius plus the overhang, cut back behind every planar wall.
fn clearing_region(floor: &Region, open_edges: &[Chain], walls: &[WallLine], extend: f64, tol: &Tolerance) -> Result<Region, CamError> {
    if open_edges.is_empty() {
        return Ok(floor.clone());
    }
    let mut region = floor.clone();
    for e in open_edges {
        let cap = chain_capsule(e, extend, tol)?;
        region = region.union(&cap, tol)?;
    }
    let big = 1.0e4;
    for w in walls {
        let t = w.inward.perp();
        let corners = [
            w.point.sub(t.scale(big)),
            w.point.add(t.scale(big)),
            w.point.add(t.scale(big)).add(w.inward.scale(big)),
            w.point.sub(t.scale(big)).add(w.inward.scale(big)),
        ];
        let mut half = Loop::polygon(&corners);
        if !half.is_ccw() {
            half = half.reversed();
        }
        region = region.intersection(&Region::new(vec![half], tol)?, tol)?;
    }
    Ok(region)
}

/// Open-edge overhang is empty space only until it reaches another feature.
/// Clip it against known higher material before the pocket strategy offsets it
/// by the cutter radius; planar wall half-planes alone cannot protect a curved
/// neighbour at the end of an open edge.
fn protect_higher_material(mut region: Region, feature: &Feature, floor_z: f64, recognized: &Recognized, tol: &Tolerance) -> Result<Region, CamError> {
    for other in &recognized.features {
        if other.id == feature.id { continue; }
        let (outline, z) = match &other.kind {
            FeatureKind::Face { region, z } => (region, *z),
            FeatureKind::Boss { outline, top_z, .. } => (outline, *top_z),
            FeatureKind::Pocket { floor, floor_z, .. } | FeatureKind::Step { floor, floor_z, .. } => (floor, *floor_z),
            _ => continue,
        };
        if z > floor_z + tol.distance_eps && !outline.loops.is_empty() { region = region.difference(outline, tol)?; }
    }
    Ok(region)
}

/// Plan the setup's operations.
pub fn plan(input: &AutoInput<'_>) -> Result<Plan, CamError> {
    let tol = Tolerance::default();
    input.techdb.validate().map_err(invalid)?;
    crate::library::validate_program_tools(input.tools).map_err(invalid)?;
    let generic = Machine::new("generic-mill", "Generic 3-axis mill", MachineKind::Mill3);
    let machine = input.machine.unwrap_or(&generic);
    let factors = input.techdb.factors(material_name(input.material));
    let mut planned: Vec<(u8, u32, usize, PlannedOperation)> = Vec::new();
    let mut skipped = Vec::new();
    for (fi, feature) in input.recognized.features.iter().enumerate() {
        let kind = feature.kind.name();
        if !feature.machinable {
            skipped.push(Skipped { feature: feature.id.clone(), kind: kind.into(), reason: feature.reason.clone().unwrap_or_else(|| "not machinable in this setup".into()) });
            continue;
        }
        if matches!(feature.kind, FeatureKind::Fillet { .. }) {
            skipped.push(Skipped { feature: feature.id.clone(), kind: kind.into(), reason: "fillets need a 3D finishing strategy or a form cutter; not programmed".into() });
            continue;
        }
        let Some(rule) = input.techdb.rule(kind) else {
            skipped.push(Skipped { feature: feature.id.clone(), kind: kind.into(), reason: format!("the technology table has no rule for {kind}") });
            continue;
        };
        let mut produced = 0;
        let mut notes_for_skip = Vec::new();
        for op_rule in &rule.operations {
            let result=if op_rule.strategy=="drill" && matches!(feature.kind,FeatureKind::Hole(_)) {
                hole_ops::plan_hole(input,feature,op_rule,machine)
            } else {plan_operation(input, feature, op_rule, machine, &factors, &tol).map(|p|vec![p])};
            match result {
                Ok(ops) => { for p in ops {
                    p.operation.validate().map_err(|m| invalid(format!("{}: {m}", p.operation.id)))?;
                    let tool_number = input.tools.iter().find(|t| t.id == p.tool).map_or(0, |t| t.number);
                    planned.push((phase(&p.role), tool_number, fi, p));
                    produced += 1;
                }}
                Err(reason) => notes_for_skip.push(format!("{}: {reason}", op_rule.name)),
            }
        }
        if produced == 0 {
            skipped.push(Skipped { feature: feature.id.clone(), kind: kind.into(), reason: notes_for_skip.join("; ") });
        } else if !notes_for_skip.is_empty() {
            // Partial: record on the produced operations.
            for (_, _, i, p) in planned.iter_mut() {
                if *i == fi {
                    p.notes.extend(notes_for_skip.iter().cloned());
                }
            }
        }
    }
    // Phase, descending roughing diameter, then tool and feature order.
    // Reuse across phases never overrides the roughing diameter priority.
    let diameter=|p:&PlannedOperation| input.tools.iter().find(|t|t.id==p.tool).unwrap().geometry.diameter;
    planned.sort_by(|a, b| a.0.cmp(&b.0).then_with(||if a.0==1 {diameter(&b.3).total_cmp(&diameter(&a.3))}else{std::cmp::Ordering::Equal}).then(a.1.cmp(&b.1)).then(a.3.tool.as_str().cmp(b.3.tool.as_str())).then(a.2.cmp(&b.2)));
    let mut ordered: Vec<PlannedOperation> = Vec::new();
    let mut last_tool: Option<ToolId> = None;
    let mut i = 0;
    while i < planned.len() {
        let ph = planned[i].0;
        let mut j = i;
        while j < planned.len() && planned[j].0 == ph {
            j += 1;
        }
        let mut group: Vec<PlannedOperation> = planned.drain(i..j).map(|(_, _, _, p)| p).collect();
        if let Some(lt) = last_tool.as_ref().filter(|_|ph!=1) {
            // Stable: pull the continuing tool's operations to the front.
            let (same, rest): (Vec<_>, Vec<_>) = group.into_iter().partition(|p| &p.tool == lt);
            group = same.into_iter().chain(rest).collect();
        }
        if let Some(last) = group.last() {
            last_tool = Some(last.tool.clone());
        }
        ordered.extend(group);
        // `drain` removed the group; the next group now starts at `i`.
        let _ = &mut i;
    }
    let tool_changes = ordered.windows(2).filter(|w| w[0].tool != w[1].tool).count() as u32;
    Ok(Plan { operations: ordered, skipped, tool_changes })
}

/// The enclosing pocket with this island filled in. Other islands remain
/// protected holes. Recognition constructs the boss outline from this exact
/// reversed floor loop, so identity does not depend on feature ordering.
fn island_clearance(feature: &Feature, recognized: &Recognized, tol: &Tolerance) -> Result<Option<(Region, Region)>, CamError> {
    let FeatureKind::Boss { outline, base_z, .. } = &feature.kind else { return Ok(None) };
    // A standalone boss can be too close to another raised feature even
    // without a recognized enclosing pocket. Bound its cutter sweep against
    // all known higher material, as well as any enclosing floor boundary.
    let (lo,hi)=outline.bounds();
    let mut allowed=Region::new(vec![Loop::rectangle(lo.sub(crate::Point2::new(1.0e4,1.0e4)),hi.add(crate::Point2::new(1.0e4,1.0e4)))],tol)?;
    'parent: for parent in &recognized.features {
        if parent.solid != feature.solid { continue; }
        let (FeatureKind::Pocket { floor, floor_z, .. } | FeatureKind::Step { floor, floor_z, .. }) = &parent.kind else { continue };
        if (base_z - floor_z).abs() > tol.distance_eps { continue; }
        for (i, boundary) in floor.loops.iter().enumerate() {
            if !boundary.is_ccw() && outline.loops == vec![boundary.reversed()] {
                allowed = floor.clone();
                allowed.loops.remove(i);
                break 'parent;
            }
        }
    }
    allowed=protect_higher_material(allowed,feature,*base_z,recognized,tol)?;
    Ok(Some((outline.clone(),allowed)))
}

fn plan_operation(input: &AutoInput<'_>, feature: &Feature, rule: &OperationRule, machine: &Machine, factors: &crate::features::techdb::MaterialFactors, tol: &Tolerance) -> Result<PlannedOperation, String> {
    let setup = input.setup;
    let stock_top = input.stock_top;
    let id = OperationId::new(format!("{}-{}", feature.id, rule.name));
    let mut notes = Vec::new();
    let targets = TargetSelection::solids([feature.solid.clone()]);
    // Geometry and fitting per feature kind.
    let (fitting, strategy_kind) = match &feature.kind {
        FeatureKind::OuterProfile { top_z,bottom_z,.. } => (Fitting {depth:stock_top.unwrap_or(*top_z)-bottom_z+rule.break_through,region:None,corner_radius:None,max_diameter:None,exact_diameter:None},"outer-profile"),
        FeatureKind::Face { region, z } => {
            let top = stock_top.unwrap_or(z + input.techdb.face_allowance);
            (Fitting { depth: top - z, region: Some(region), corner_radius: None, max_diameter: None, exact_diameter: None }, "face")
        }
        FeatureKind::Pocket { floor, floor_z, top_z, corner_radius, .. } => (Fitting { depth: top_z - floor_z, region: Some(floor), corner_radius: *corner_radius, max_diameter: None, exact_diameter: None }, "pocket"),
        FeatureKind::Step { floor, floor_z, top_z, .. } => (Fitting { depth: top_z - floor_z, region: Some(floor), corner_radius: None, max_diameter: None, exact_diameter: None }, "step"),
        FeatureKind::Slot { width, floor_z, top_z, .. } => (Fitting { depth: top_z - floor_z, region: None, corner_radius: None, max_diameter: Some(*width), exact_diameter: None }, "slot"),
        FeatureKind::Boss { top_z, base_z, .. } => (Fitting { depth: top_z - base_z, region: None, corner_radius: None, max_diameter: None, exact_diameter: None }, "boss"),
        FeatureKind::Hole(h) => (Fitting { depth: h.depth, region: None, corner_radius: None, max_diameter: None, exact_diameter: Some(h.diameter) }, "hole"),
        FeatureKind::Chamfer { width, .. } => (Fitting { depth: width + rule.tip_offset, region: None, corner_radius: None, max_diameter: None, exact_diameter: None }, "chamfer"),
        FeatureKind::Fillet { .. } => return Err("fillets need a 3D finishing strategy or a form cutter; not programmed".into()),
    };
    let _ = strategy_kind;
    if rule.strategy == "drill" {return Err("drill rule requires the compound-hole planner".into());}
    let tool=choose_tool(rule, input.tools, &fitting, island_clearance(feature, input.recognized, tol).map_err(|e| e.to_string())?.as_ref(), tol)?;
    if matches!(feature.kind,FeatureKind::Boss{..}) {
        notes.push("boss contour finishes walls only; pocket-around requires the separately programmed enclosing floor (external stock boundary is not inferred)".into());
    }
    let d = tool.geometry.diameter;
    let stepover = (rule.stepover * d * factors.stepover).max(tol.sample_eps);
    let step_down = rule.step_down * d * factors.step_down;
    let depths = |top: f64, bottom: f64| Depths { top: Some(top), bottom: Some(bottom), step_down, finishing_step_down: 0.0, finishing_passes: 0, depth_first: true };
    let mut linking = crate::model::LinkingParams::default();
    linking.ramp.kind = rule.ramp;
    if rule.lead > 0.0 {
        linking.lead_in = Lead { kind: LeadKind::Arc, radius: rule.lead * d, sweep_deg: 90.0, linear_length: 0.0, vertical_radius: 0.0 };
        linking.lead_out = linking.lead_in;
    }
    let (strategy, geometry) = match (&feature.kind, rule.strategy.as_str()) {
        (FeatureKind::OuterProfile {outline,top_z,bottom_z,walls},"contour2D")=>{
            if *walls!=crate::features::ProfileWalls::Vertical {return Err("outer profile requires certified vertical walls".into());}
            // Bounds-offset stock uses model axes. Its bottom allowance
            // proves contact only when model +Z is machine +Z.
            let aligned=setup.model_to_machine.rotation[2].iter().zip([0.,0.,1.]).all(|(a,b)|(a-b).abs()<tol.angle_eps);
            let on_stock_bottom=aligned && match &setup.stock {
                crate::Stock::BoundsOffset{bottom,..}=>*bottom==0.,
                crate::Stock::Box{min,..}=>(setup.model_to_machine.apply_point(*min).z-bottom_z).abs()<=tol.distance_eps,
                _=>false,
            };
            let breakthrough=if on_stock_bottom {rule.break_through}else{0.};
            let params=Contour2dParams{depths:depths(stock_top.unwrap_or(*top_z),bottom_z-breakthrough),outside:true,roughing_passes:rule.roughing_passes,roughing_stepover:stepover,finishing_passes:rule.finishing_passes.max(1),finishing_overlap:0.,tabs:Vec::new(),direction:CutDirection::Climb};
            (Strategy::Contour2D(params),PlanarInput{chains:outline.loops.iter().map(|l|PlanarChain{chain:loop_chain(l),z:*bottom_z}).collect(),stock_top,..Default::default()})
        }
        (FeatureKind::Face { region, z }, "face") => {
            let top = stock_top.unwrap_or(z + input.techdb.face_allowance);
            if top <= *z + tol.distance_eps { return Err("stock top is at or below the face; no facing allowance to remove".into()); }
            let params = FaceParams { depths: depths(top, *z), stepover: Stepover { distance: stepover, direction: CutDirection::Climb }, pass_angle_deg: 0.0, pass_extension: rule.extension, zig_zag: rule.zig_zag };
            (Strategy::Face(params), PlanarInput { regions: vec![PlanarRegion { region: region.clone(), z: *z, upward: true }], stock_top, ..Default::default() })
        }
        (FeatureKind::Pocket { floor, floor_z, top_z, open_edges, walls, .. }, "pocket2D") | (FeatureKind::Step { floor, floor_z, top_z, open_edges, walls, .. }, "pocket2D") => {
            let region = clearing_region(floor, open_edges, walls, 0.5 * d + input.techdb.open_pocket_overhang, tol).map_err(|e| e.to_string())?;
            let region = if open_edges.is_empty() { region } else {
                protect_higher_material(region, feature, *floor_z, input.recognized, tol).map_err(|e| e.to_string())?
            };
            // A single floor-finishing pass can reach outside the previous
            // rougher's sweep. Clear those stock slivers before the shank
            // descends into them, using the feature's roughing depth rule.
            let mut pocket_depths=depths(stock_top.unwrap_or(*top_z).max(*top_z), *floor_z);
            if pocket_depths.top.unwrap()-floor_z > tool.geometry.cutting_length + tol.distance_eps && pocket_depths.step_down <= 0. {
                let rough_step=input.techdb.rule(feature.kind.name()).and_then(|r|r.operations.iter().find(|r|r.strategy=="pocket2D" && r.role=="rough" && r.step_down>0.)).map(|r|r.step_down)
                    .ok_or("single-depth pocket finishing exceeds flute reach; supply a roughing step-down rule")?;
                pocket_depths.step_down=rough_step*d*factors.step_down;
                notes.push("stock exceeds flute reach; clear finishing overhang in roughing-sized depth levels".into());
            }
            if pocket_depths.step_down>0. { pocket_depths.step_down=pocket_depths.step_down.min(tool.geometry.cutting_length); }
            let params = Pocket2dParams { depths: pocket_depths, stepover: Stepover { distance: stepover, direction: CutDirection::Climb }, offset_pattern: true, wall_finish: false, keep_islands: true };
            (Strategy::Pocket2D(params), PlanarInput { regions: vec![PlanarRegion { region, z: *floor_z, upward: true }], stock_top, ..Default::default() })
        }
        (FeatureKind::Pocket { floor, floor_z, top_z, .. }, "contour2D") => {
            // Outer loops are inside contours; island loops outside contours.
            let mut chains = Vec::new();
            for l in &floor.loops {
                chains.push(PlanarChain { chain: loop_chain(l), z: *floor_z });
            }
            let mut wall_depths=depths(stock_top.unwrap_or(*top_z).max(*top_z),*floor_z);
            if wall_depths.step_down<=0. && wall_depths.top.unwrap()-floor_z>tool.geometry.cutting_length+tol.distance_eps {
                return Err("wall finishing exceeds flute reach; supply a positive step-down rule".into());
            }
            if wall_depths.step_down>0. {wall_depths.step_down=wall_depths.step_down.min(tool.geometry.cutting_length);}
            let params = Contour2dParams { depths: wall_depths, outside: false, roughing_passes: rule.roughing_passes, roughing_stepover: stepover, finishing_passes: rule.finishing_passes.max(1), finishing_overlap: 0.0, tabs: Vec::new(), direction: CutDirection::Climb };
            // Islands (clockwise loops) are machined on the outside: split them
            // into their own input so each chain carries the right side.
            let outer: Vec<PlanarChain> = chains.iter().filter(|c| Loop::new(c.chain.segments.clone()).is_ccw()).cloned().collect();
            let islands: Vec<PlanarChain> = chains.iter().filter(|c| !Loop::new(c.chain.segments.clone()).is_ccw()).cloned().collect();
            if !islands.is_empty() {
                notes.push(format!("{} island wall(s) are finished by the island's own boss contour", islands.len()));
            }
            (Strategy::Contour2D(params), PlanarInput { chains: outer, stock_top, ..Default::default() })
        }
        (FeatureKind::Slot { centre, width, floor_z, top_z }, "slot") => {
            let params = SlotParams { depths: depths(*top_z, *floor_z), width: *width, trochoidal: false };
            (Strategy::Slot(params), PlanarInput { chains: vec![PlanarChain { chain: centre.clone(), z: *floor_z }], stock_top, ..Default::default() })
        }
        (FeatureKind::Boss { outline, top_z, base_z, .. }, "contour2D") => {
            let chains: Vec<PlanarChain> = outline.loops.iter().map(|l| PlanarChain { chain: loop_chain(l), z: *base_z }).collect();
            let params = Contour2dParams { depths: depths(*top_z, *base_z), outside: true, roughing_passes: rule.roughing_passes, roughing_stepover: stepover, finishing_passes: rule.finishing_passes.max(1), finishing_overlap: 0.0, tabs: Vec::new(), direction: CutDirection::Climb };
            (Strategy::Contour2D(params), PlanarInput { chains, stock_top, ..Default::default() })
        }
        (FeatureKind::Chamfer { chain, width, z, .. }, "chamfer2D") => {
            let params = Chamfer2dParams { width: *width, tip_offset: rule.tip_offset };
            (Strategy::Chamfer2D(params), PlanarInput { chains: vec![PlanarChain { chain: chain.clone(), z: *z }], stock_top, ..Default::default() })
        }
        (FeatureKind::Face { .. }, "contour2D") => return Err("outer profile: recognition supplies a top outline but no certified profile bottom or stock boundary; select an explicit contour depth (tabs default off)".into()),
        (kind, strategy) => return Err(format!("the rule's strategy {strategy} does not apply to a {}", kind.name())),
    };
    let mut op = Operation::new(id, format!("{} {}", feature.kind.name(), rule.name), setup.id.clone(), tool.id.clone(), strategy);
    op.targets = targets;
    op.tolerance = rule.tolerance;
    op.stock_to_leave = rule.stock_to_leave;
    op.linking = linking;
    op.feeds = feeds_for(tool, machine, input, Some(stepover), &mut notes);
    Ok(PlannedOperation { operation: op, tool: tool.id.clone(), geometry: OperationGeometry::Planar(geometry), feature: feature.id.clone(), role: rule.role.clone(), notes })
}

/// Feeds from lane C12 for the tool and material; the tool's own feeds
/// (with a note) when the library tool carries no cutting data.
fn feeds_for(tool: &Tool, machine: &Machine, input: &AutoInput<'_>, engagement: Option<f64>, notes: &mut Vec<String>) -> Option<crate::model::FeedsSpeeds> {
    match crate::feeds::calculate(tool, machine, input.units, input.material, engagement) {
        Ok(calc) => {
            notes.extend(calc.adjustments.iter().map(|a| format!("feeds: {a}")));
            Some(calc.feeds)
        }
        Err(e) => {
            notes.push(format!("feeds: {e}; the tool's own feeds are used"));
            None
        }
    }
}

/// One operation's generation outcome. Drilling keeps its canned cycles in
/// `toolpath` and the expanded motions in `expanded` (for simulation).
#[derive(Clone, Debug, PartialEq)]
pub struct GeneratedOperation {
    pub operation: OperationId,
    pub toolpath: Option<Toolpath>,
    pub expanded: Option<Toolpath>,
    pub error: Option<String>,
}

impl GeneratedOperation {
    /// The motions to simulate: the expanded stream when there is one.
    pub fn simulated(&self) -> Option<&Toolpath> {
        self.expanded.as_ref().or(self.toolpath.as_ref())
    }
}

/// Generate every planned operation through its strategy. Per-operation
/// failures are recorded, not fatal; cancellation is.
pub fn generate(plan: &Plan, input: &AutoInput<'_>, mesh: &MeshSnapshot, index: Option<&dyn TriangleIndex>, tol: Tolerance, runner: &mut Runner<'_>) -> Result<Vec<GeneratedOperation>, CamError> {
    let mut out = Vec::with_capacity(plan.operations.len());
    for (i, p) in plan.operations.iter().enumerate() {
        runner.stage("autoprogram-generate", &p.operation.name, i as u64, plan.operations.len() as u64)?;
        let tool = input.tools.iter().find(|t| t.id == p.tool).ok_or_else(|| invalid(format!("tool {} missing", p.tool)))?;
        let op = &p.operation;
        let result: Result<(Toolpath, Option<Toolpath>), CamError> = match (&op.strategy, &p.geometry) {
            (Strategy::Face(_)|Strategy::Pocket2D(_)|Strategy::Contour2D(_)|Strategy::Slot(_)|Strategy::Chamfer2D(_)|Strategy::Circular(_)|Strategy::ThreadMill(_), OperationGeometry::Planar(g)) => {
                let dispatch_input=crate::dispatch::GenerateInput {operation:op,setup:input.setup,tools:input.tools,mesh,index,tolerance:tol,holes:&[]};
                let context=crate::dispatch::GenerateContext {resolved_planar:Some((&op.id,g)),machine:if input.setup.machine.is_some(){input.machine}else{None},..Default::default()};
                crate::dispatch::generate_with_context(&dispatch_input,&context,runner).map(|t|(t,None))
            },
            (Strategy::Drill(_), OperationGeometry::Holes(h)) => crate::drill::strategy::generate(op, input.setup, tool, h, mesh, tol, runner).map(|d| (d.canned, Some(d.expanded))),
            (s, _) => Err(invalid(format!("no generator for {}", s.kind_name()))),
        };
        match result {
            Ok((path, expanded)) => out.push(GeneratedOperation { operation: op.id.clone(), toolpath: Some(path), expanded, error: None }),
            Err(CamError::Cancelled) => return Err(CamError::Cancelled),
            Err(e) => out.push(GeneratedOperation { operation: op.id.clone(), toolpath: None, expanded: None, error: Some(e.to_string()) }),
        }
    }
    Ok(out)
}

/// The program input of the generated operations (failed ones left out),
/// ready for `program::assemble` and the posts.
pub fn program_input(plan: &Plan, generated: &[GeneratedOperation], input: &AutoInput<'_>, name: &str) -> crate::program::ProgramInput {
    let mut operations = Vec::new();
    for p in &plan.operations {
        let Some(g) = generated.iter().find(|g| g.operation == p.operation.id) else { continue };
        let Some(path) = &g.toolpath else { continue };
        let Some(tool) = input.tools.iter().find(|t| t.id == p.tool) else { continue };
        operations.push(crate::program::OperationInput { operation: p.operation.clone(), tool: tool.clone(), toolpath: path.clone() });
    }
    crate::program::ProgramInput {
        name: name.to_string(),
        number: None,
        units: match input.units {
            LibraryUnits::Millimeters => crate::program::Units::Mm,
            LibraryUnits::Inches => crate::program::Units::In,
        },
        machine: input.machine.cloned(),
        setups: vec![crate::program::SetupInput { setup: input.setup.clone(), operations }],
        ..Default::default()
    }
}

/// The cutter of a planned operation's tool (for simulation).
pub fn cutter_of(plan: &Plan, input: &AutoInput<'_>, operation: &OperationId) -> Option<Cutter> {
    let p = plan.operations.iter().find(|p| &p.operation.id == operation)?;
    let tool = input.tools.iter().find(|t| t.id == p.tool)?;
    let c = Cutter::from_tool(tool).ok()?;
    let _ = c.radius();
    Some(c)
}

