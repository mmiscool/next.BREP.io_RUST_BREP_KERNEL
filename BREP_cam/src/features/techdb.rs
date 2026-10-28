//! The technology table (lane C13 row 2): data, not code. A feature type
//! maps to an ordered list of operation rules; each rule names the strategy,
//! how to choose the tool, and the cutting parameters as fractions of the
//! chosen tool's diameter; per-material factors scale stepover and step
//! down. The built-in table is `BREP_cam/data/techdb.json`; a shop's own
//! table loads through [`TechDb::from_json`].

use crate::model::{RampKind, StockToLeave, ToolKind};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const BUILTIN_TECHDB: &str = include_str!("../../data/techdb.json");

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct TechDb {
    pub format: String,
    pub version: u32,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub units: String,
    /// Stock assumed above a face to face when the stock top is unknown.
    #[serde(default = "default_face_allowance")]
    pub face_allowance: f64,
    /// How far the tool runs out past an open pocket's open edges.
    #[serde(default = "default_overhang")]
    pub open_pocket_overhang: f64,
    #[serde(default)]
    pub material_factors: BTreeMap<String, MaterialFactors>,
    pub rules: Vec<Rule>,
}

fn default_face_allowance() -> f64 {
    2.0
}

fn default_overhang() -> f64 {
    1.0
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct MaterialFactors {
    pub stepover: f64,
    pub step_down: f64,
}

impl Default for MaterialFactors {
    fn default() -> Self {
        Self { stepover: 1.0, step_down: 1.0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct Rule {
    /// Feature kind name (`features::FeatureKind::name`).
    pub feature: String,
    pub operations: Vec<OperationRule>,
}

/// How a tool is picked from the library for one operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Fit {
    /// The largest tool of the kinds.
    Largest,
    /// The largest tool that fits the feature (corner radius, slot width,
    /// pocket width) with a cutting length at least the depth.
    LargestFitting,
    /// The tool whose diameter equals the feature's (holes).
    ExactDiameter,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct ToolRule {
    pub kinds: Vec<ToolKind>,
    pub fit: Fit,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", deny_unknown_fields)]
pub struct OperationRule {
    pub name: String,
    /// Strategy kind name: `face`, `pocket2D`, `contour2D`, `slot`,
    /// `chamfer2D`, `drill`.
    pub strategy: String,
    /// `face`, `rough`, `finish`, `drill`, `chamfer` (orders the program).
    pub role: String,
    pub tool: ToolRule,
    /// Stepover as a fraction of the tool diameter.
    pub stepover: f64,
    /// Step down as a fraction of the tool diameter (`0`: one level).
    pub step_down: f64,
    pub stock_to_leave: StockToLeave,
    #[serde(default)]
    pub finishing_passes: u32,
    #[serde(default)]
    pub roughing_passes: u32,
    /// Lead arc radius as a fraction of the tool diameter (`0`: none).
    #[serde(default)]
    pub lead: f64,
    #[serde(default = "default_ramp")]
    pub ramp: RampKind,
    /// Facing: extension past the stock, zig-zag.
    #[serde(default)]
    pub extension: f64,
    #[serde(default = "default_true")]
    pub zig_zag: bool,
    /// Chamfer tip offset below the chamfer's lower edge.
    #[serde(default)]
    pub tip_offset: f64,
    /// Extra outside-profile depth when the part reaches the stock bottom.
    #[serde(default, skip_serializing_if="is_zero")]
    pub break_through: f64,
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
}

fn is_zero(v:&f64)->bool {*v==0.}

fn default_ramp() -> RampKind {
    RampKind::Plunge
}

fn default_true() -> bool {
    true
}

fn default_tolerance() -> f64 {
    0.01
}

impl TechDb {
    pub fn builtin() -> TechDb {
        Self::from_json(BUILTIN_TECHDB).expect("the built-in technology table validates")
    }

    pub fn from_json(text: &str) -> Result<TechDb, String> {
        let db: TechDb = serde_json::from_str(text).map_err(|e| e.to_string())?;
        db.validate()?;
        Ok(db)
    }

    pub fn to_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string_pretty(self).map(|s| s + "\n").map_err(|e| e.to_string())
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.format != "brep-techdb" || self.version != 1 {
            return Err("unsupported technology table format/version".into());
        }
        if !self.face_allowance.is_finite() || self.face_allowance < 0.0 || !self.open_pocket_overhang.is_finite() || self.open_pocket_overhang < 0.0 {
            return Err("face allowance and open pocket overhang must be finite and non-negative".into());
        }
        for (m, f) in &self.material_factors {
            if !(f.stepover > 0.0 && f.stepover.is_finite() && f.step_down > 0.0 && f.step_down.is_finite()) {
                return Err(format!("material {m}: factors must be finite and positive"));
            }
        }
        if self.rules.is_empty() {
            return Err("the technology table has no rules".into());
        }
        let mut seen = std::collections::BTreeSet::new();
        for r in &self.rules {
            if !seen.insert(r.feature.as_str()) {
                return Err(format!("duplicate rule for feature {}", r.feature));
            }
            if r.operations.is_empty() {
                return Err(format!("rule for {} has no operations", r.feature));
            }
            let mut names = std::collections::BTreeSet::new();
            for o in &r.operations {
                if o.name.trim().is_empty() || !names.insert(o.name.as_str()) {
                    return Err(format!("rule for {}: empty or duplicate operation name", r.feature));
                }
                if !matches!(o.strategy.as_str(), "face" | "pocket2D" | "contour2D" | "slot" | "chamfer2D" | "drill") {
                    return Err(format!("rule for {}: unknown strategy {}", r.feature, o.strategy));
                }
                if !matches!(o.role.as_str(), "face" | "rough" | "finish" | "drill" | "chamfer") {
                    return Err(format!("rule for {}: unknown role {}", r.feature, o.role));
                }
                if o.tool.kinds.is_empty() {
                    return Err(format!("rule for {}: operation {} names no tool kinds", r.feature, o.name));
                }
                for (n, v) in [("stepover", o.stepover), ("step down", o.step_down), ("lead", o.lead), ("extension", o.extension), ("tip offset", o.tip_offset), ("breakthrough", o.break_through)] {
                    if !v.is_finite() || v < 0.0 {
                        return Err(format!("rule for {}: {n} must be finite and non-negative", r.feature));
                    }
                }
                if o.stepover > 1.0 {
                    return Err(format!("rule for {}: stepover fraction above 1", r.feature));
                }
                if !o.tolerance.is_finite() || o.tolerance <= 0.0 {
                    return Err(format!("rule for {}: tolerance must be positive", r.feature));
                }
                o.stock_to_leave.validate()?;
                if o.stock_to_leave.radial < 0.0 || o.stock_to_leave.axial < 0.0 {
                    return Err(format!("rule for {}: stock to leave must not be negative", r.feature));
                }
            }
        }
        Ok(())
    }

    /// The rule for a feature kind name.
    pub fn rule(&self, feature: &str) -> Option<&Rule> {
        self.rules.iter().find(|r| r.feature == feature)
    }

    /// Factors for a material (kebab-case or camelCase name), identity when absent.
    pub fn factors(&self, material: &str) -> MaterialFactors {
        let camel: String = {
            let mut out = String::new();
            let mut up = false;
            for ch in material.chars() {
                if ch == '-' || ch == '_' {
                    up = true;
                } else if up {
                    out.extend(ch.to_uppercase());
                    up = false;
                } else {
                    out.push(ch);
                }
            }
            out
        };
        self.material_factors.get(material).or_else(|| self.material_factors.get(&camel)).copied().unwrap_or_default()
    }
}
