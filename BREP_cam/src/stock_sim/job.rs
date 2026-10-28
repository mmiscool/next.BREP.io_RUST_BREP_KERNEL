//! Pre-operation stock for one generation job.
//!
//! The app hands the runner one operation at a time. This module turns what
//! the job carries — the setup, its earlier operations with their current
//! toolpaths, and (for `Stock::FromRest`) the setups it continues from — into
//! the stock as it stands before the operation, through one
//! [`PreOpStockCache`] the runner keeps per document session:
//! - every setup in the chain is resolved from its own stock definition,
//!   sized from the target bodies in the job's machine snapshot;
//! - each earlier setup's final stock is carried into the next setup's frame
//!   ([`carry_from_setup`]); an oblique carry adds its named approximation;
//! - the job's setup is replayed through the operations before this one.
//! A failure is a named [`PreOpStockError`]; the caller links with the
//! conservative material top and puts [`unavailable_warning`] on the
//! operation. It never falls back to the raw stock silently.

use super::carry::carry_from_setup;
use super::preop::{PlannedOp, PreOpStockCache, PreOpStockError, PreOperationStock};
use super::removal::RemovalOptions;
use super::{DexelGrid, DexelOptions};
use crate::cancel::Runner;
use crate::mesh::MeshSnapshot;
use crate::model::{ApproximationLimit, Bounds3, Diagnostic, Machine, Operation, Setup, TargetRef, Tool, Toolpath};
use crate::numeric::Tolerance;
use crate::stock::{resolve_stock, ResolvedStock};
use serde::{Deserialize, Serialize};

/// A setup the job's setup continues from (`Stock::FromRest`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SetupHistory {
    pub setup: Setup,
    /// The setup's machine, resolved.
    #[serde(default)]
    pub machine: Option<Machine>,
    /// The setup's operations in program order with their current toolpaths
    /// (`None`: not generated, failed, or stale).
    #[serde(default)]
    pub operations: Vec<(Operation, Option<Toolpath>)>,
}

/// What a generation job knows about the stock before its operation.
pub struct JobStock<'a> {
    pub setup: &'a Setup,
    pub machine: Option<&'a Machine>,
    pub tools: &'a [Tool],
    /// The job's machine snapshot (the setup's machine frame).
    pub mesh: &'a MeshSnapshot,
    /// Target bodies to size a stock from when a setup lists none (the operation's).
    pub fallback_targets: &'a [String],
    /// The setup's operations BEFORE this one, in program order.
    pub earlier: &'a [(Operation, Option<Toolpath>)],
    /// The setups this one continues from, oldest first (empty unless its stock is `FromRest`).
    pub previous: &'a [SetupHistory],
    pub tolerance: Tolerance,
}

/// The stock before the job's operation, and what the carry approximated.
#[derive(Clone, Debug, PartialEq)]
pub struct JobStockResult {
    pub stock: PreOperationStock,
    pub limits: Vec<ApproximationLimit>,
    pub start: SetupStart,
}

/// The stock the job's setup starts from, as the job resolved it from its own
/// snapshot: the simulation loads this so it cuts the stock generation planned
/// against (a display mesh's f32 bounds differ by an ULP, and a carry grows
/// that skin into a wall).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SetupStart {
    /// Model-space bounds of the setup's targets (`Stock::BoundsOffset` only).
    #[serde(default)]
    pub target_bounds: Option<Bounds3>,
    /// The stock carried in from the setup it continues from (`Stock::FromRest` only).
    #[serde(default)]
    pub carried: Option<DexelGrid>,
}

/// The grid settings a job uses: a cell from the snapshot's size (stable for
/// every operation of a setup, so the cache's keys match).
pub fn job_dexel_options(mesh: &MeshSnapshot) -> DexelOptions {
    let d = mesh.bounds.diagonal();
    let cell = if d.is_finite() && d > 0.0 { (d / 500.0).clamp(0.05, 5.0) } else { 0.5 };
    DexelOptions { cell, ..Default::default() }
}

/// Whether a generation job builds pre-operation stock: three-axis milling
/// on a mill or a mill-turn. Turning, probing, manual NC, patterns and
/// multi-axis strategies do not link over a Z-dexel stock.
pub fn uses_pre_op_stock(strategy: &crate::Strategy, machine: Option<&Machine>) -> bool {
    use crate::Strategy as S;
    if machine.is_some_and(|m| matches!(m.kind, crate::MachineKind::Lathe)) {
        return false;
    }
    !matches!(
        strategy,
        S::ProbeWcs(_)
            | S::ProbeGeometry(_)
            | S::Pattern(_)
            | S::ManualNc(_)
            | S::Swarf(_)
            | S::MultiAxisContour(_)
            | S::RotaryWrap(_)
            | S::RotaryFinish(_)
            | S::MultiAxisFinish(_)
            | S::TurnFace(_)
            | S::TurnProfile(_)
            | S::TurnGroove(_)
            | S::TurnThread(_)
            | S::TurnPartOff(_)
    )
}

/// The warning an operation carries when its pre-operation stock is unavailable.
pub fn unavailable_warning(error: &PreOpStockError) -> Diagnostic {
    Diagnostic::new(
        "pre-operation-stock-unavailable",
        format!("pre-operation stock unavailable: {error}; links keep above the stock top"),
    )
}

/// Model-space bounds of `targets` (protected-only faces excluded) read from
/// the snapshot, which is in `frame`'s machine coordinates.
fn target_bounds(mesh: &MeshSnapshot, frame: &Setup, targets: &[TargetRef]) -> Result<Bounds3, PreOpStockError> {
    let inverse = frame.model_to_machine.inverse();
    let mut bounds = Bounds3::empty();
    for target in targets {
        let faces = mesh.resolve_faces(target);
        if faces.is_empty() {
            return Err(PreOpStockError::InvalidInput(format!("stock target body {:?} is not in the job's snapshot", target.solid)));
        }
        for id in mesh.triangles_of_faces(&faces) {
            let triangle = &mesh.triangles[id as usize];
            if mesh.faces[triangle.face as usize].protected_only {
                continue;
            }
            for p in triangle.vertices() {
                bounds.include(inverse.apply_point(p));
            }
        }
    }
    Ok(bounds)
}

fn resolve(job: &JobStock<'_>, setup: &Setup, continues_from: Option<&Setup>) -> Result<(ResolvedStock, Option<Bounds3>), PreOpStockError> {
    let targets: Vec<TargetRef> =
        if setup.targets.is_empty() { job.fallback_targets.iter().map(TargetRef::solid).collect() } else { setup.targets.clone() };
    let bounds = if matches!(setup.stock, crate::Stock::BoundsOffset { .. }) { Some(target_bounds(job.mesh, job.setup, &targets)?) } else { None };
    let stock = resolve_stock(setup, &bounds.unwrap_or_else(Bounds3::empty), None)
        .map_err(|e| PreOpStockError::InvalidInput(format!("setup {:?}: {e} (a stock body is not carried into pre-operation stock)", setup.id.as_str())))?;
    if let ResolvedStock::Rest(name) = &stock {
        match continues_from {
            Some(p) if p.id.as_str() == name.as_str() => {}
            _ => {
                return Err(PreOpStockError::InvalidInput(format!(
                    "setup {:?} continues from setup {:?}, which the job does not carry",
                    setup.id.as_str(),
                    name.as_str()
                )))
            }
        }
    }
    Ok((stock, bounds))
}

fn planned<'a>(ops: &'a [(Operation, Option<Toolpath>)], tools: &'a [Tool]) -> Result<Vec<PlannedOp<'a>>, PreOpStockError> {
    ops.iter()
        .map(|(operation, toolpath)| {
            let tool = tools.iter().find(|t| t.id == operation.tool).ok_or_else(|| {
                PreOpStockError::InvalidInput(format!("operation {:?}: its tool {:?} is not in the document", operation.id.as_str(), operation.tool.as_str()))
            })?;
            Ok(PlannedOp { operation, tool, toolpath: toolpath.as_ref() })
        })
        .collect()
}

/// The stock before the job's operation.
pub fn job_stock(cache: &mut PreOpStockCache, job: &JobStock<'_>, runner: &mut Runner<'_>) -> Result<JobStockResult, PreOpStockError> {
    let options = job_dexel_options(job.mesh);
    let removal = RemovalOptions::default();
    let mut carried: Option<DexelGrid> = None;
    let mut limits = Vec::new();
    let mut before: Option<&Setup> = None;
    for (k, history) in job.previous.iter().enumerate() {
        let next = job.previous.get(k + 1).map_or(job.setup, |h| &h.setup);
        let (initial, _) = resolve(job, &history.setup, before)?;
        let ops = planned(&history.operations, job.tools)?;
        let c = carry_from_setup(cache, &history.setup, history.machine.as_ref(), &initial, carried.as_ref(), &options, &ops, &job.tolerance, &removal, runner, next)?;
        limits.extend(c.limits);
        carried = Some(c.grid);
        before = Some(&history.setup);
    }
    let (initial, target_bounds) = resolve(job, job.setup, before)?;
    let ops = planned(job.earlier, job.tools)?;
    let stock = cache.stock_before(job.setup, job.machine, &initial, carried.as_ref(), &options, &ops, ops.len(), &job.tolerance, &removal, runner)?;
    Ok(JobStockResult { stock, limits, start: SetupStart { target_bounds, carried } })
}

