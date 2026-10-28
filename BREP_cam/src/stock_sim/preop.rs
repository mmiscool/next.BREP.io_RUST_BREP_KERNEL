//! Pre-operation stock: a setup's stock as it stands before operation N.
//!
//! The stock before operation N is the setup's initial stock replayed through
//! its operations 0..N-1, in order, on the setup's machine chain (so indexed
//! and mill-turn programs cut in the part frame). The result is the dexel grid
//! and a [`StockEnvelope`] over it, which the linker and the dispatcher use as
//! "material that may still be there".
//!
//! [`PreOpStockCache`] keys every result by the stock definition, the grid and
//! removal settings, the machine and setup frame, and the ordered list of
//! (tool, toolpath) hashes of the operations replayed; regenerating operation
//! N therefore replays only from the longest cached unchanged prefix.
//!
//! Never a silent fallback to the raw stock: an earlier operation that is
//! ungenerated, generated with errors, or not simulatable is a named refusal.

use super::envelope::StockEnvelope;
use super::removal::{simulate, simulate_on_machine, RemovalOptions};
use super::{DexelGrid, DexelOptions, StockSimError};
use crate::cancel::{Cancelled, Runner};
use crate::cutter::{Cutter, CutterError};
use crate::kinematics::Chain;
use crate::model::{Machine, Operation, RigidTransform, Setup, Tool, Toolpath};
use crate::numeric::Tolerance;
use crate::stock::ResolvedStock;

/// One operation of the setup, in program order, with its current toolpath.
#[derive(Clone, Copy, Debug)]
pub struct PlannedOp<'a> {
    pub operation: &'a Operation,
    pub tool: &'a Tool,
    /// `None`: not generated yet.
    pub toolpath: Option<&'a Toolpath>,
}

/// The stock before an operation.
#[derive(Clone, Debug, PartialEq)]
pub struct PreOperationStock {
    /// Machine-frame dexel stock (the frame `ResolvedStock` is given in).
    pub grid: DexelGrid,
    /// Envelope over `grid` in the same frame (identity link frame).
    pub envelope: StockEnvelope,
    /// Operations taken from the cache.
    pub cached_ops: usize,
    /// Operations replayed for this request.
    pub replayed_ops: usize,
}

/// Why the stock before an operation could not be built.
#[derive(Clone, Debug, PartialEq)]
pub enum PreOpStockError {
    /// An earlier enabled operation has no usable toolpath (missing, or generated with errors).
    Ungenerated { operation: String },
    /// An earlier operation's toolpath cannot be simulated in a Z-dexel stock.
    Unsimulatable { operation: String, motions: Vec<usize>, reason: String },
    /// An earlier operation's tool has no milling cutter profile.
    Cutter { operation: String, error: CutterError },
    /// The machine has no milling kinematics chain (e.g. a lathe).
    Kinematics(String),
    /// The initial grid could not be built.
    Grid(StockSimError),
    InvalidInput(String),
    Cancelled,
}

impl std::fmt::Display for PreOpStockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PreOpStockError::Ungenerated { operation } => {
                write!(f, "operation {operation:?} before this one has no usable toolpath; generate it first")
            }
            PreOpStockError::Unsimulatable { operation, reason, .. } => {
                write!(f, "operation {operation:?} before this one cannot be simulated: {reason}")
            }
            PreOpStockError::Cutter { operation, error } => write!(f, "operation {operation:?}: {error}"),
            PreOpStockError::Kinematics(m) => write!(f, "machine kinematics: {m}"),
            PreOpStockError::Grid(e) => write!(f, "{e}"),
            PreOpStockError::InvalidInput(m) => write!(f, "{m}"),
            PreOpStockError::Cancelled => write!(f, "cancelled"),
        }
    }
}

impl std::error::Error for PreOpStockError {}

impl From<Cancelled> for PreOpStockError {
    fn from(_: Cancelled) -> Self {
        PreOpStockError::Cancelled
    }
}

type Key = [u64; 2];

/// FNV-1a with two offset bases: a stable 128-bit content hash.
fn hash(parts: &[&[u8]]) -> Key {
    let mut out = [0xcbf2_9ce4_8422_2325_u64, 0x6c62_272e_07bb_0142_u64];
    for h in out.iter_mut() {
        for part in parts {
            for &b in part.iter() {
                *h ^= b as u64;
                *h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
            *h ^= 0xff;
            *h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    out
}

#[derive(Clone, Debug)]
struct Entry {
    base: Key,
    ops: Vec<Key>,
    grid: DexelGrid,
}

/// Bounded cache of pre-operation stock states. Eviction is oldest first
/// (insertion order), so behaviour is deterministic.
#[derive(Clone, Debug, Default)]
pub struct PreOpStockCache {
    max_entries: usize,
    entries: Vec<Entry>,
}

impl PreOpStockCache {
    /// A cache keeping at most `max_entries` states (at least one).
    pub fn new(max_entries: usize) -> Self {
        Self { max_entries: max_entries.max(1), entries: Vec::new() }
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    /// The stock of `setup` before `ops[n]` (`n == ops.len()`: after all of
    /// them). `ops` are the setup's operations in program order; `initial` is
    /// the resolved stock in the setup's machine frame; `rest` the previous
    /// setup's grid for `ResolvedStock::Rest`; `machine` the setup's machine
    /// (its chain is built with the setup's work-offset origin; `None` replays
    /// frame-free, refusing any nonzero rotation by name).
    #[allow(clippy::too_many_arguments)]
    pub fn stock_before(
        &mut self,
        setup: &Setup,
        machine: Option<&Machine>,
        initial: &ResolvedStock,
        rest: Option<&DexelGrid>,
        options: &DexelOptions,
        ops: &[PlannedOp<'_>],
        n: usize,
        tol: &Tolerance,
        removal: &RemovalOptions,
        runner: &mut Runner<'_>,
    ) -> Result<PreOperationStock, PreOpStockError> {
        if n > ops.len() {
            return Err(PreOpStockError::InvalidInput(format!("operation index {n} is past the setup's {} operations", ops.len())));
        }
        if let Some(foreign) = ops[..n].iter().find(|o| o.operation.setup != setup.id) {
            return Err(PreOpStockError::InvalidInput(format!(
                "operation {:?} belongs to setup {:?}, not {:?}",
                foreign.operation.id.as_str(),
                foreign.operation.setup.as_str(),
                setup.id.as_str()
            )));
        }
        let chain = match machine {
            Some(m) => Some(Chain::from_machine(m, setup.wcs.origin).map_err(|e| PreOpStockError::Kinematics(format!("{e}")))?),
            None => None,
        };
        let rest_json = rest.map(|g| serde_json::to_vec(g).unwrap_or_default()).unwrap_or_default();
        let base = hash(&[
            format!("{initial:?}").as_bytes(),
            format!("{options:?}").as_bytes(),
            format!("{machine:?}").as_bytes(),
            format!("{:?}{:?}", setup.model_to_machine, setup.wcs.origin).as_bytes(),
            &rest_json,
            format!("{removal:?}{tol:?}").as_bytes(),
        ]);
        let keys: Vec<Key> = ops[..n]
            .iter()
            .map(|o| {
                let tool = serde_json::to_vec(o.tool).unwrap_or_default();
                let path = o.toolpath.map(|t| serde_json::to_vec(&t.motions).unwrap_or_default()).unwrap_or_default();
                let enabled: &[u8] = if o.operation.enabled { b"on" } else { b"off" };
                hash(&[enabled, &tool, &path])
            })
            .collect();
        // Longest cached prefix of this request.
        let best = self
            .entries
            .iter()
            .filter(|e| e.base == base && e.ops.len() <= n && keys[..e.ops.len()] == e.ops[..])
            .max_by_key(|e| e.ops.len());
        let (mut grid, start) = match best {
            Some(e) => (e.grid.clone(), e.ops.len()),
            None => (DexelGrid::from_stock(initial, options, rest, None).map_err(PreOpStockError::Grid)?, 0),
        };
        if start == n && best.is_some() {
            let envelope = StockEnvelope::new(&grid, RigidTransform::identity()).map_err(PreOpStockError::Grid)?;
            return Ok(PreOperationStock { grid, envelope, cached_ops: n, replayed_ops: 0 });
        }
        for (k, o) in ops[start..n].iter().enumerate() {
            runner.stage("pre-operation-stock", "replaying an earlier operation", k as u64, (n - start) as u64)?;
            if !o.operation.enabled {
                continue;
            }
            let path = match o.toolpath {
                Some(t) if t.report.is_usable() => t,
                _ => return Err(PreOpStockError::Ungenerated { operation: o.operation.id.as_str().to_string() }),
            };
            let cutter = Cutter::from_tool(o.tool).map_err(|error| PreOpStockError::Cutter { operation: o.operation.id.as_str().to_string(), error })?;
            let report = match &chain {
                Some(c) => simulate_on_machine(&mut grid, &cutter, &path.motions, tol, removal, c, runner)?,
                None => simulate(&mut grid, &cutter, &path.motions, tol, removal, runner)?,
            };
            if !report.unsupported.is_empty() {
                let reason = report
                    .warnings
                    .iter()
                    .find(|w| w.code == "motion-not-simulated")
                    .and_then(|w| w.detail.clone())
                    .unwrap_or_else(|| "motion not simulated".into());
                return Err(PreOpStockError::Unsimulatable { operation: o.operation.id.as_str().to_string(), motions: report.unsupported, reason });
            }
        }
        // Every operation after the cached prefix (disabled ones included: their key is part of the prefix).
        let replayed_ops = n - start;
        self.entries.push(Entry { base, ops: keys, grid: grid.clone() });
        while self.entries.len() > self.max_entries {
            self.entries.remove(0);
        }
        let envelope = StockEnvelope::new(&grid, RigidTransform::identity()).map_err(PreOpStockError::Grid)?;
        Ok(PreOperationStock { grid, envelope, cached_ops: start, replayed_ops })
    }
}

