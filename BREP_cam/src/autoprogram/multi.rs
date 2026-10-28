//! Additional fixed-axis setups are proposals, certified by fresh recognition.
//! A refusal is never converted into machinable geometry by rotating its label.
use super::*;
use crate::model::{Bounds3, Point3, RigidTransform, Stock};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis { PositiveX, NegativeX, PositiveY, NegativeY, NegativeZ }
impl Axis {
    pub const ALL: [Self; 5] = [Self::PositiveX, Self::NegativeX, Self::PositiveY, Self::NegativeY, Self::NegativeZ];
    pub fn name(self) -> &'static str { match self { Self::PositiveX => "+X", Self::NegativeX => "-X", Self::PositiveY => "+Y", Self::NegativeY => "-Y", Self::NegativeZ => "-Z" } }
    pub fn vector(self) -> Point3 { match self { Self::PositiveX => Point3::X, Self::NegativeX => Point3::X.scale(-1.), Self::PositiveY => Point3::Y, Self::NegativeY => Point3::Y.scale(-1.), Self::NegativeZ => Point3::Z.scale(-1.) } }
}

/// Exact recognition for a proposed orientation. Bounds include the initial
/// stock: later stock can only shrink, so these heights remain conservative.
#[derive(Clone, Debug)]
pub struct Candidate { pub axis: Axis, pub setup: Setup, pub recognized: Recognized, pub stock_top: Option<f64> }
#[derive(Clone, Debug)]
pub struct SetupPlan {
    pub axis: Option<Axis>,
    pub setup: Setup,
    pub recognized: Recognized,
    pub plan: Plan,
    /// First-setup feature IDs satisfied here, each assigned at most once.
    pub covered: Vec<String>,
}
#[derive(Clone, Debug)]
pub struct MultiPlan { pub setups: Vec<SetupPlan>, pub unreached: Vec<Skipped> }

pub fn candidate_setup(first: &Setup, axis: Axis, model_stock: Bounds3) -> Result<Setup, CamError> {
    let inverse = first.machine_to_model();
    let z = axis.vector();
    let x = if z.x.abs() > 0.5 { Point3::Y } else { Point3::X };
    let mut setup = first.clone();
    setup.id = format!("{}-auto-{}", first.id, axis.name()).into();
    setup.name = format!("{} · {} approach", first.name, axis.name());
    setup.model_to_machine = RigidTransform::from_frame(inverse.apply_point(Point3::ZERO), inverse.apply_vector(x), inverse.apply_vector(z)).map_err(invalid)?;
    setup.stock = Stock::FromRest { setup: first.id.clone() };
    let bounds = setup.model_to_machine.apply_bounds(&model_stock);
    let gap = (first.heights.clearance - first.heights.retract).abs().max(5.);
    setup.heights.clearance = bounds.max.z + 2. * gap;
    setup.heights.retract = bounds.max.z + gap;
    setup.heights.feed = bounds.max.z + 1.;
    setup.heights.top = None;
    setup.heights.bottom = None;
    Ok(setup)
}

/// Only an opening direction parallel to a candidate axis can certify the
/// original floor. A different side face is not proof that an undercut was cut.
pub fn needed_axes(recognized: &Recognized) -> Vec<Axis> {
    Axis::ALL.into_iter().filter(|axis| recognized.features.iter().any(|f| !f.machinable && f.direction.dot(axis.vector()) > 1. - 1e-6)).collect()
}

fn same_feature(original: &Feature, candidate: &Feature) -> bool {
    let family = |kind: &FeatureKind| match kind { FeatureKind::Pocket { .. } | FeatureKind::Step { .. } | FeatureKind::Slot { .. } => "floor", _ => kind.name() };
    original.solid == candidate.solid && !original.faces.is_empty()
        && original.faces.first() == candidate.faces.first()
        && family(&original.kind) == family(&candidate.kind)
        && candidate.machinable && candidate.direction.dot(Point3::Z) > 1. - 1e-6
}

pub fn plan_candidates<F>(input: &AutoInput<'_>, first: Plan, candidates: &[Candidate], mut planner: F) -> Result<MultiPlan, CamError>
where F: FnMut(&AutoInput<'_>) -> Result<Plan, CamError> {
    let mut pending: Vec<_> = input.recognized.features.iter().filter(|f| !f.machinable).collect();
    let mut planning_refusals=std::collections::BTreeMap::<String,Vec<String>>::new();
    let mut setups = vec![SetupPlan { axis: None, setup: input.setup.clone(), recognized: input.recognized.clone(), covered: input.recognized.features.iter().filter(|f| first.operations.iter().any(|p| p.feature == f.id)).map(|f| f.id.clone()).collect(), plan: first }];
    for axis in Axis::ALL {
        let Some(candidate) = candidates.iter().find(|c| c.axis == axis) else { continue };
        let mut pairs = Vec::new();
        for original in &pending {
            if original.direction.dot(axis.vector()) < 1. - 1e-6 { continue; }
            let matches: Vec<_> = candidate.recognized.features.iter().filter(|f| same_feature(original, f)).collect();
            if matches.len() == 1 { pairs.push((original.id.clone(), matches[0].id.clone())); }
        }
        if pairs.is_empty() { continue; }
        let mut setup = candidate.setup.clone();
        setup.stock = Stock::FromRest { setup: setups.last().unwrap().setup.id.clone() };
        let mut plan = planner(&AutoInput { recognized: &candidate.recognized, setup: &setup, stock_top: candidate.stock_top, ..*input })?;
        for (original,id) in &pairs {
            if plan.operations.iter().any(|p| &p.feature==id) {continue;}
            let reasons:Vec<_>=plan.skipped.iter().filter(|s| &s.feature==id).map(|s|s.reason.as_str()).collect();
            let reason=if reasons.is_empty() {"the planner produced no operations".into()}else{reasons.join("; ")};
            planning_refusals.entry(original.clone()).or_default().push(format!("{}: {reason}",axis.name()));
        }
        pairs.retain(|(_, id)| plan.operations.iter().any(|p| &p.feature == id));
        if pairs.is_empty() { continue; }
        plan.operations.retain(|p| pairs.iter().any(|(_, id)| id == &p.feature));
        plan.skipped.retain(|s| pairs.iter().any(|(_, id)| id == &s.feature));
        let mut recognized = candidate.recognized.clone();
        recognized.features.retain(|f| pairs.iter().any(|(_, id)| id == &f.id));
        // IDs are preview identities, namespaced without weakening the durable
        // face/kind/key-dimension identity used by Binding::matches.
        let prefix = format!("auto{}:", axis.name());
        for feature in &mut recognized.features { feature.id = format!("{prefix}{}", feature.id); }
        for op in &mut plan.operations {
            op.feature = format!("{prefix}{}", op.feature);
            op.operation.id = format!("{prefix}{}", op.operation.id).into();
            op.operation.setup = setup.id.clone();
        }
        for skipped in &mut plan.skipped { skipped.feature = format!("{prefix}{}", skipped.feature); }
        plan.tool_changes = plan.operations.windows(2).filter(|w| w[0].tool != w[1].tool).count() as u32;
        let covered: Vec<_> = pairs.into_iter().map(|(original, _)| original).collect();
        pending.retain(|f| !covered.contains(&f.id));
        setups.push(SetupPlan { axis: Some(axis), setup, recognized, plan, covered });
    }
    let unreached = pending.into_iter().map(|f| {
        if let Some(reasons)=planning_refusals.get(&f.id) {
            return Skipped{feature:f.id.clone(),kind:f.kind.name().into(),reason:format!("axis-aligned access found for feature {}, but no machining plan: {}; choose fitting tools/rules or skip",f.id,reasons.join("; "))};
        }
        let mut reasons = vec![f.reason.clone().unwrap_or_else(|| "access is uncertified".into())];
        for candidate in candidates {
            for found in candidate.recognized.features.iter().filter(|c| c.solid == f.solid && c.faces.first() == f.faces.first()) {
                if let Some(reason) = &found.reason { if !reasons.contains(reason) { reasons.push(reason.clone()); } }
            }
        }
        Skipped { feature: f.id.clone(), kind: f.kind.name().into(), reason: format!(
            "no additional axis-aligned setup certifies feature {}: {}; requires 3D/5-axis review or skip", f.id, reasons.join("; ")) }
    }).collect();
    Ok(MultiPlan { setups, unreached })
}

