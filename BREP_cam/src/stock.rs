//! Stock resolved into MACHINE coordinates for one setup: the shapes the
//! material-removal simulation (lane C7) starts from. No simulation here.

use crate::mesh::MeshSnapshot;
use crate::model::{Bounds3, Point3, RigidTransform, Setup, SetupId, Stock};

/// A box given in its own (model) frame plus the transform into machine space.
/// Kept oriented so a rotated setup does not inflate the stock to its AABB.
#[derive(Clone, Debug, PartialEq)]
pub struct StockBox {
    /// The box in model coordinates (a `Stock::BoundsOffset`'s offsets follow
    /// the setup's frame: see [`bounds_offset_box`]).
    pub bounds: Bounds3,
    /// Model-to-machine.
    pub to_machine: RigidTransform,
}

/// A `Stock::BoundsOffset`'s box, kept ORIENTED in the model frame (no
/// axis-aligned inflation on a rotated setup), with the offsets following the
/// setup: each machine axis's offsets (`x` both sides of machine X, `y` of
/// machine Y, `bottom` / `top` below / above machine Z) are applied along the
/// model axis that maps closest to that machine axis, on the side its sign
/// gives — so a setup whose Z is the model's +Y adds its top offset along the
/// model's +Y, and an upside-down part gets its top offset above it on the
/// machine. Exact whenever the setup's axes are the model's axes permuted or
/// reversed (every face or edge pick); for a frame skewed off the model axes
/// the offsets go along the nearest model axes. `None` for another stock kind
/// or no targets.
pub fn bounds_offset_box(setup: &Setup, target_bounds_model: &Bounds3) -> Option<StockBox> {
    let Stock::BoundsOffset { x, y, top, bottom } = &setup.stock else { return None };
    if target_bounds_model.is_empty() {
        return None;
    }
    let r = &setup.model_to_machine.rotation;
    let mut min = [target_bounds_model.min.x, target_bounds_model.min.y, target_bounds_model.min.z];
    let mut max = [target_bounds_model.max.x, target_bounds_model.max.y, target_bounds_model.max.z];
    // (low side, high side) of each machine axis.
    let offsets = [(*x, *x), (*y, *y), (*bottom, *top)];
    let mut used = [false; 3];
    // Machine Z claims its model axis first: top and bottom matter most.
    for machine_axis in [2usize, 0, 1] {
        let (lo, hi) = offsets[machine_axis];
        // The model axis whose image is closest to this machine axis (each used once).
        let model_axis = (0..3)
            .filter(|i| !used[*i])
            .max_by(|a, b| r[machine_axis][*a].abs().partial_cmp(&r[machine_axis][*b].abs()).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap_or(machine_axis);
        used[model_axis] = true;
        if r[machine_axis][model_axis] >= 0.0 {
            min[model_axis] -= lo;
            max[model_axis] += hi;
        } else {
            min[model_axis] -= hi;
            max[model_axis] += lo;
        }
    }
    Some(StockBox { bounds: Bounds3::new(Point3::new(min[0], min[1], min[2]), Point3::new(max[0], max[1], max[2])), to_machine: setup.model_to_machine })
}

impl StockBox {
    /// Axis-aligned machine-space bounds (exact for axis-aligned setups).
    pub fn machine_bounds(&self) -> Bounds3 {
        self.to_machine.apply_bounds(&self.bounds)
    }

    /// Whether a MACHINE point is inside the stock (closed).
    pub fn contains(&self, machine_point: Point3) -> bool {
        self.bounds.contains_point(self.to_machine.inverse().apply_point(machine_point))
    }

    pub fn volume(&self) -> f64 {
        let s = self.bounds.size();
        if self.bounds.is_empty() {
            0.0
        } else {
            s.x * s.y * s.z
        }
    }
}

/// A cylinder in MACHINE coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct StockCylinder {
    pub base: Point3,
    /// Unit axis direction.
    pub axis: Point3,
    pub radius: f64,
    pub height: f64,
}

impl StockCylinder {
    pub fn machine_bounds(&self) -> Bounds3 {
        let a = self.axis;
        let top = self.base.add(a.scale(self.height));
        let ext = Point3::new(
            self.radius * (1.0 - a.x * a.x).max(0.0).sqrt(),
            self.radius * (1.0 - a.y * a.y).max(0.0).sqrt(),
            self.radius * (1.0 - a.z * a.z).max(0.0).sqrt(),
        );
        Bounds3::new(self.base.min(top).sub(ext), self.base.max(top).add(ext))
    }

    pub fn contains(&self, machine_point: Point3) -> bool {
        let d = machine_point.sub(self.base);
        let along = d.dot(self.axis);
        if along < 0.0 || along > self.height {
            return false;
        }
        let radial = d.sub(self.axis.scale(along)).length();
        radial <= self.radius
    }

    pub fn volume(&self) -> f64 {
        std::f64::consts::PI * self.radius * self.radius * self.height
    }
}

/// The setup's stock in machine coordinates.
#[derive(Clone, Debug, PartialEq)]
pub enum ResolvedStock {
    Box(StockBox),
    Cylinder(StockCylinder),
    /// A body used as stock, already tessellated and transformed into machine space.
    Mesh(MeshSnapshot),
    /// The rest material of another setup; the simulation chains to that setup's result.
    Rest(SetupId),
}

impl ResolvedStock {
    /// Machine-space bounds; `None` for `Rest` (known only after the previous setup ran).
    pub fn machine_bounds(&self) -> Option<Bounds3> {
        match self {
            ResolvedStock::Box(b) => Some(b.machine_bounds()),
            ResolvedStock::Cylinder(c) => Some(c.machine_bounds()),
            ResolvedStock::Mesh(m) => Some(m.bounds),
            ResolvedStock::Rest(_) => None,
        }
    }

    /// Analytic containment; `None` where it needs a volume model (mesh, rest).
    pub fn contains(&self, machine_point: Point3) -> Option<bool> {
        match self {
            ResolvedStock::Box(b) => Some(b.contains(machine_point)),
            ResolvedStock::Cylinder(c) => Some(c.contains(machine_point)),
            ResolvedStock::Mesh(_) | ResolvedStock::Rest(_) => None,
        }
    }

    /// Top of stock in machine Z, when known.
    pub fn top_z(&self) -> Option<f64> {
        self.machine_bounds().map(|b| b.max.z)
    }
}

/// Why a stock definition could not be resolved.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum StockError {
    /// The definition failed its own validation (message from `Stock::validate`).
    Invalid(String),
    /// `BoundsOffset` with no target geometry to grow.
    NoTargets,
    /// `FromSolid` named a body the caller did not supply.
    MissingSolid(String),
}

impl std::fmt::Display for StockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StockError::Invalid(m) => write!(f, "stock: {m}"),
            StockError::NoTargets => write!(f, "stock: the setup has no target geometry to size the stock from"),
            StockError::MissingSolid(s) => write!(f, "stock: body {s:?} was not supplied"),
        }
    }
}

impl std::error::Error for StockError {}

/// Resolve `setup.stock` into machine coordinates. `target_bounds_model` is
/// the model-space bounds of the setup's target bodies; `from_solid` is the
/// model-space snapshot of the stock body for `Stock::FromSolid`. The setup
/// transform is applied here and nowhere else for stock.
pub fn resolve_stock(
    setup: &Setup,
    target_bounds_model: &Bounds3,
    from_solid: Option<&MeshSnapshot>,
) -> Result<ResolvedStock, StockError> {
    setup.stock.validate().map_err(StockError::Invalid)?;
    let t = &setup.model_to_machine;
    match &setup.stock {
        Stock::BoundsOffset { .. } => Ok(ResolvedStock::Box(bounds_offset_box(setup, target_bounds_model).ok_or(StockError::NoTargets)?)),
        Stock::Box { min, max } => Ok(ResolvedStock::Box(StockBox { bounds: Bounds3::new(*min, *max), to_machine: *t })),
        Stock::Cylinder { base, axis, radius, height } => {
            let axis = axis.normalized().ok_or_else(|| StockError::Invalid("cylinder axis is zero".into()))?;
            Ok(ResolvedStock::Cylinder(StockCylinder {
                base: t.apply_point(*base),
                axis: t.apply_vector(axis),
                radius: *radius,
                height: *height,
            }))
        }
        Stock::FromSolid { solid } => {
            let mesh = from_solid.ok_or_else(|| StockError::MissingSolid(solid.clone()))?;
            Ok(ResolvedStock::Mesh(crate::mesh::to_machine(setup, mesh)))
        }
        Stock::FromRest { setup: previous } => Ok(ResolvedStock::Rest(previous.clone())),
    }
}

