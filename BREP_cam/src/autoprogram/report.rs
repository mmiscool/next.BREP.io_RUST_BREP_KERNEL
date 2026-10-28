//! The report for the UI (lane C13 row 4): what was recognised, what was
//! programmed with which operations and tools, what was skipped and why,
//! and how each generated operation ended.

use super::{GeneratedOperation, Plan, Skipped};
use crate::features::Recognized;
use crate::model::{OperationId, ToolId};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FeatureLine {
    pub id: String,
    pub kind: String,
    pub solid: String,
    pub faces: Vec<String>,
    pub machinable: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
    /// Operations programmed for the feature, in program order.
    pub operations: Vec<OperationId>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OperationLine {
    pub id: OperationId,
    pub name: String,
    pub feature: String,
    pub role: String,
    pub strategy: String,
    pub tool: ToolId,
    /// `planned`, `generated` or `failed`.
    pub status: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    pub motions: u64,
    pub warnings: u64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notes: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Report {
    pub features: Vec<FeatureLine>,
    pub operations: Vec<OperationLine>,
    pub skipped: Vec<Skipped>,
    pub tool_changes: u32,
    pub unassigned_faces: Vec<(String, String)>,
    pub generated: u32,
    pub failed: u32,
}

/// Assemble the report; `generated` may be empty (a plan not yet generated).
pub fn report(recognized: &Recognized, plan: &Plan, generated: &[GeneratedOperation]) -> Report {
    let mut features = Vec::new();
    for f in &recognized.features {
        features.push(FeatureLine {
            id: f.id.clone(),
            kind: f.kind.name().to_string(),
            solid: f.solid.clone(),
            faces: f.faces.clone(),
            machinable: f.machinable,
            reason: f.reason.clone(),
            operations: plan.operations.iter().filter(|p| p.feature == f.id).map(|p| p.operation.id.clone()).collect(),
        });
    }
    let mut operations = Vec::new();
    let (mut ok, mut failed) = (0u32, 0u32);
    for p in &plan.operations {
        let g = generated.iter().find(|g| g.operation == p.operation.id);
        let (status, error, motions, warnings) = match g {
            None => ("planned", None, 0, 0),
            Some(GeneratedOperation { toolpath: Some(t), .. }) => {
                ok += 1;
                ("generated", None, t.motions.len() as u64, t.report.warnings.len() as u64)
            }
            Some(GeneratedOperation { error, .. }) => {
                failed += 1;
                ("failed", error.clone(), 0, 0)
            }
        };
        operations.push(OperationLine {
            id: p.operation.id.clone(),
            name: p.operation.name.clone(),
            feature: p.feature.clone(),
            role: p.role.clone(),
            strategy: p.operation.strategy.kind_name().to_string(),
            tool: p.tool.clone(),
            status: status.to_string(),
            error,
            motions,
            warnings,
            notes: p.notes.clone(),
        });
    }
    Report {
        features,
        operations,
        skipped: plan.skipped.clone(),
        tool_changes: plan.tool_changes,
        unassigned_faces: recognized.unassigned.clone(),
        generated: ok,
        failed,
    }
}
