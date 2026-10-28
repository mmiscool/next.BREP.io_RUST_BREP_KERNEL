//! Durable identity of an automatically programmed feature. Geometry is
//! refreshed from exact bodies after edits; a lost or ambiguous feature is
//! refused rather than silently machining a different region.
use super::*;
use crate::features::Feature;
use crate::model::{Point2, TargetSelection, ToolGeometry};
use std::collections::{BTreeMap, BTreeSet};

/// Only persistent recognition identity belongs in the recipe. In particular,
/// do not flatten a Hole's own `kind` into the feature enum's `kind` tag.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FeatureIdentity {
    pub id: String,
    pub solid: String,
    pub kind: String,
    pub faces: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Binding {
    pub geometry: OperationGeometry,
    pub setup: Setup,
    pub targets: TargetSelection,
    pub tool_geometry: ToolGeometry,
    pub tool_kind: ToolKind,
    /// Hex strings on the wire: JSON/JavaScript numbers cannot retain all
    /// 64 fingerprint bits across the browser worker boundary.
    #[serde(with = "fingerprints")]
    pub body_fingerprints: BTreeMap<String, u64>,
    pub feature: FeatureIdentity,
    /// XY position and dimensions identify the feature while its depth may
    /// change. Face identities must match as well.
    pub key_dimensions: Vec<f64>,
    pub planned_name: String,
    pub role: String,
    pub strategy: Strategy,
    pub operation_tolerance: f64,
}

mod fingerprints {
    use super::*;
    pub fn serialize<S: serde::Serializer>(value: &BTreeMap<String, u64>, serializer: S) -> Result<S::Ok, S::Error> {
        value.iter().map(|(k, v)| (k, format!("{v:016x}"))).collect::<BTreeMap<_, _>>().serialize(serializer)
    }
    pub fn deserialize<'de, D: serde::Deserializer<'de>>(deserializer: D) -> Result<BTreeMap<String, u64>, D::Error> {
        BTreeMap::<String, String>::deserialize(deserializer)?.into_iter().map(|(k, v)| {
            u64::from_str_radix(&v, 16).map(|v| (k, v)).map_err(serde::de::Error::custom)
        }).collect()
    }
}

pub fn key_dimensions(feature: &Feature) -> Vec<f64> {
    let bounds = |(a, b): (Point2, Point2)| vec![a.x, a.y, b.x, b.y];
    match &feature.kind {
        FeatureKind::Face { region, .. } => bounds(region.bounds()),
        FeatureKind::Pocket { floor, islands, .. } => { let mut v = bounds(floor.bounds()); v.push(*islands as f64); v },
        FeatureKind::Step { floor, .. } => bounds(floor.bounds()),
        FeatureKind::OuterProfile { outline, .. } | FeatureKind::Boss { outline, .. } => bounds(outline.bounds()),
        FeatureKind::Slot { centre, width, .. } => { let mut v = bounds(crate::planar::Loop::new(centre.segments.clone()).bounds()); v.push(*width); v },
        FeatureKind::Hole(h) => vec![h.top.x, h.top.y, h.diameter],
        FeatureKind::Chamfer { chain, width, .. } => { let mut v = bounds(crate::planar::Loop::new(chain.segments.clone()).bounds()); v.push(*width); v },
        FeatureKind::Fillet { chain, radius, .. } => { let mut v = bounds(crate::planar::Loop::new(chain.segments.clone()).bounds()); v.push(*radius); v },
    }
}

impl Binding {
    pub fn new(planned: &PlannedOperation, feature: &Feature, setup: &Setup, tool: &Tool, body_fingerprints: BTreeMap<String, u64>) -> Self {
        Self { geometry: planned.geometry.clone(), setup: setup.clone(), targets: planned.operation.targets.clone(),
            tool_geometry: tool.geometry.clone(), tool_kind: tool.kind, body_fingerprints, feature: FeatureIdentity { id: feature.id.clone(), solid: feature.solid.clone(), kind: feature.kind.name().into(), faces: feature.faces.clone() }, key_dimensions: key_dimensions(feature),
            planned_name: planned.operation.name.clone(), role: planned.role.clone(), strategy: planned.operation.strategy.clone(), operation_tolerance: planned.operation.tolerance }
    }

    pub fn validate_inputs(&self, operation: &Operation, setup: &Setup, tool: &Tool) -> Result<(), String> {
        if self.setup.model_to_machine != setup.model_to_machine || self.setup.targets != setup.targets || self.setup.protected != setup.protected || self.setup.stock != setup.stock {
            return Err("automatic programming setup geometry changed; re-run Program part automatically".into());
        }
        if self.targets != operation.targets || self.tool_geometry != tool.geometry || self.tool_kind != tool.kind || self.strategy != operation.strategy || self.operation_tolerance != operation.tolerance {
            return Err("automatic programming targets, cutter, tolerance or passes changed; re-run Program part automatically".into());
        }
        Ok(())
    }

    pub fn matches(&self, feature: &Feature, tol: &Tolerance) -> bool {
        let keys = key_dimensions(feature);
        self.feature.solid == feature.solid && self.feature.kind == feature.kind.name()
            && self.feature.faces.iter().collect::<BTreeSet<_>>() == feature.faces.iter().collect::<BTreeSet<_>>()
            && keys.len() == self.key_dimensions.len()
            && keys.iter().zip(&self.key_dimensions).all(|(a, b)| (a - b).abs() <= tol.distance_eps.max(1e-6))
    }

    /// Apply the same whole-part extension protection as the preview after
    /// re-recognizing changed geometry in the worker's current setup frame.
    pub fn rebind_with_mesh(&self, recognized: &Recognized, mesh: &MeshSnapshot, operation: &Operation, setup: &Setup, tools: &[Tool], machine: Option<&Machine>, stock_top: Option<f64>, tol: Tolerance) -> Result<(Operation, OperationGeometry), String> {
        let (op, mut geometry) = self.rebind(recognized, operation, setup, tools, machine, stock_top, tol)?;
        if matches!(op.strategy, Strategy::Pocket2D(_)) {
            let feature = recognized.features.iter().find(|f| self.matches(f, &tol)).ok_or("rebound feature disappeared")?;
            super::access::constrain_rebound(&mut geometry, feature, mesh, &tol).map_err(|e| e.to_string())?;
        }
        Ok((op, geometry))
    }

    /// Rebind only the operation's own rule, using its current cutter. Hole
    /// sequences need the full library to retain pilot/recess/thread intent.
    pub fn rebind(&self, recognized: &Recognized, operation: &Operation, setup: &Setup, tools: &[Tool], machine: Option<&Machine>, stock_top: Option<f64>, tol: Tolerance) -> Result<(Operation, OperationGeometry), String> {
        let matches: Vec<_> = recognized.features.iter().filter(|f| self.matches(f, &tol)).collect();
        if matches.len() != 1 {
            return Err(format!("feature {} no longer recognised; re-run Program part automatically{}", self.feature.id, if matches.len() > 1 { " (ambiguous identity)" } else { "" }));
        }
        let feature = matches[0];
        let own: Vec<_> = tools.iter().filter(|t| t.id == operation.tool).cloned().collect();
        let library = if matches!(feature.kind, FeatureKind::Hole(_)) { tools } else { own.as_slice() };
        let db = TechDb::builtin();
        let planned = plan(&AutoInput { recognized, setup, tools: library, machine, material: WorkMaterial::Aluminium,
            techdb: &db, units: LibraryUnits::Millimeters, stock_top }).map_err(|e| e.to_string())?;
        let updated = planned.operations.iter().find(|p| p.feature == feature.id && p.operation.name == self.planned_name && p.role == self.role && p.tool == operation.tool)
            .ok_or_else(|| format!("feature {} cannot use its programmed tool/rule after model change; re-run Program part automatically", self.feature.id))?;
        let mut op = operation.clone();
        op.strategy = updated.operation.strategy.clone();
        Ok((op, updated.geometry.clone()))
    }
}

/// Exact recognition shared by the preview/worker boundary.
#[cfg(feature = "kernel")]
pub fn recognize_bodies(bodies: &[(String, brep_kernel::BrepSolid)], setup: &Setup, mesh: &MeshSnapshot, chord: f64, tol: Tolerance, runner: &mut Runner<'_>) -> Result<Recognized, CamError> {
    let mut carriers = Vec::new(); let mut regions = Vec::new(); let mut revolved = Vec::new();
    for (name, solid) in bodies {
        if !setup.targets.iter().any(|t| &t.solid == name) { continue; }
        let body = crate::kernel_adapter::BodyInput { name, solid, protected_only: false };
        carriers.extend(crate::kernel_adapter::features::face_carriers(&body, setup, &tol));
        regions.extend(crate::kernel_adapter::planar::horizontal_faces(solid, &setup.model_to_machine, chord, &tol));
        revolved.extend(crate::kernel_adapter::revolved_faces(&body, setup));
    }
    let holes = crate::holes::recognize(&revolved, mesh, tol, runner)?;
    let recognized = crate::features::recognize(&crate::features::RecognitionInput { mesh, carriers: &carriers, regions: &regions, holes: &holes, tolerance: tol })?;
    Ok(recognized)
}

