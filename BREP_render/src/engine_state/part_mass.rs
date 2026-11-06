//! MASS PROPERTIES OF A PART — computed from the part's geometry and the
//! density the PART carries, never typed in and never stored per occurrence.
//!
//! # One source
//!
//! Material and density are attributes of the PART definition: the
//! `partAttributes` record on the part's own document (see `bom.rs`), edited
//! in the part's Properties dialog or the BOM's part columns. Everything that
//! shows a mass — the BOM row, the Info window, the BOM export, the MCP
//! readers — resolves it here, so they cannot disagree:
//!
//! * a solid's density is its own metadata `density` when one is set (a
//!   multi-material part's per-body override), else its PART's `Density`,
//!   else the density of the part's `Material` when that names a material
//!   this build knows (`material_density`), else none;
//! * a part's volume is the volume of the solids ONE placement of it
//!   produces (every placement is the same rigid copy), from the existing
//!   measurement cache (`object_info_json`), so nothing re-executes the history
//!   per frame and a value is simply pending until the runner answers;
//! * a part's mass is `Σ volume × density`. A part with NO geometry (a
//!   purchased or library part with nothing to measure) may carry a stored
//!   `Mass` instead: that is the only time a mass is typed, and it is still a
//!   part attribute.
//!
//! Units: density in g/cm³ (what material tables quote), volume in mm³ (the
//! kernel's length unit), mass in kg.
//!
//! # Old documents
//!
//! A document saved before this rule may hold `Mass` / `Material` / `Density`
//! on its OCCURRENCES (the ACOMP's `bom` record, or the `occurrenceAttributes`
//! key an older batch path wrote). `migrate_occurrence_part_attributes`
//! lifts them onto the part as the document opens, and says so once.

use super::*;
use serde_json::Value;

/// The part attribute keys that describe the PART's mass properties. None of
/// them may be stored on an occurrence.
pub const PART_ONLY_KEYS: [&str; 4] = ["Mass", "Material", "Density", "Volume"];

/// Derived — computed from geometry, never stored.
pub const VOLUME_KEY: &str = "Volume";
/// Computed for a part with geometry; a stored override only without.
pub const MASS_KEY: &str = "Mass";
/// The part's density in g/cm³.
pub const DENSITY_KEY: &str = "Density";
/// The part's material name; resolves a density when no `Density` is set.
pub const MATERIAL_KEY: &str = "Material";

pub const DENSITY_UNIT: &str = "g/cm³";
pub const MASS_UNIT: &str = "kg";
pub const VOLUME_UNIT: &str = "mm³";

/// mm³ × g/cm³ → kg: 1 mm³ = 1e-3 cm³, 1 g = 1e-3 kg.
const MM3_G_CM3_TO_KG: f64 = 1e-6;

/// Built-in densities (g/cm³) for common material names, most specific
/// first: the first key found as a whole word (or word run) of the
/// material's name wins, so `Stainless Steel 304` is stainless and
/// `6061-T6` is aluminium.
const MATERIAL_DENSITIES: &[(&str, f64)] = &[
    ("stainless", 7.93),
    ("cast iron", 7.20),
    ("tool steel", 7.80),
    ("steel", 7.85),
    ("iron", 7.87),
    ("aluminium", 2.70),
    ("aluminum", 2.70),
    ("6061", 2.70),
    ("6063", 2.69),
    ("7075", 2.81),
    ("2024", 2.78),
    ("brass", 8.50),
    ("bronze", 8.80),
    ("copper", 8.96),
    ("titanium", 4.43),
    ("magnesium", 1.74),
    ("zinc", 7.14),
    ("nickel", 8.90),
    ("lead", 11.34),
    ("abs", 1.04),
    ("pla", 1.24),
    ("petg", 1.27),
    ("nylon", 1.14),
    ("polycarbonate", 1.20),
    ("acetal", 1.41),
    ("delrin", 1.41),
    ("pom", 1.41),
    ("hdpe", 0.95),
    ("polypropylene", 0.90),
    ("pvc", 1.38),
    ("acrylic", 1.18),
    ("pmma", 1.18),
    ("ptfe", 2.20),
    ("glass", 2.50),
    ("oak", 0.75),
    ("pine", 0.50),
    ("plywood", 0.60),
    ("wood", 0.60),
];

/// The built-in density (g/cm³) for a material NAME, or `None` when the name
/// matches nothing this build knows (then the part needs an explicit
/// `Density`).
pub fn material_density(material: &str) -> Option<f64> {
    let words: Vec<String> = material
        .to_lowercase()
        .split(|c: char| !c.is_alphanumeric())
        .filter(|word| !word.is_empty())
        .map(str::to_string)
        .collect();
    if words.is_empty() {
        return None;
    }
    let padded = format!(" {} ", words.join(" "));
    MATERIAL_DENSITIES
        .iter()
        .find(|(key, _)| padded.contains(&format!(" {key} ")))
        .map(|&(_, density)| density)
}

/// A mass in kg as a reader wants it: kilograms from 1 kg up, grams below.
pub fn format_mass_kg(kg: f64) -> String {
    if kg.abs() >= 1.0 {
        format!("{kg:.3} kg")
    } else {
        format!("{:.3} g", kg * 1000.0)
    }
}

/// A volume in mm³ as a reader wants it: cm³ from 1 cm³ up, mm³ below.
pub fn format_volume_mm3(mm3: f64) -> String {
    if mm3.abs() >= 1000.0 {
        format!("{:.3} cm³", mm3 / 1000.0)
    } else {
        format!("{mm3:.3} mm³")
    }
}

/// The display text of a DERIVED part field (`Mass`, `Volume`) with its
/// unit; `None` for any other field or a non-number.
pub fn format_derived(field: &str, value: &Value) -> Option<String> {
    let number = value.as_f64()?;
    match field {
        MASS_KEY => Some(format_mass_kg(number)),
        VOLUME_KEY => Some(format_volume_mm3(number)),
        _ => None,
    }
}

/// Where a resolved density came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DensitySource {
    /// The solid's own metadata `density` (a per-body override).
    Solid,
    /// The part's `Density` attribute.
    Part,
    /// The built-in density of the part's `Material`.
    Material,
}

impl DensitySource {
    pub fn as_str(self) -> &'static str {
        match self {
            DensitySource::Solid => "solid",
            DensitySource::Part => "part",
            DensitySource::Material => "material",
        }
    }
}

/// A positive finite number from a JSON number or numeric string.
fn positive_number(value: &Value) -> Option<f64> {
    let number = match value {
        Value::Number(number) => number.as_f64(),
        Value::String(text) => text.trim().parse::<f64>().ok(),
        _ => None,
    }?;
    (number.is_finite() && number > 0.0).then_some(number)
}

/// A part's density from its attribute record: its `Density`, else its
/// `Material`'s built-in density.
pub fn part_density(attributes: &Value) -> Option<(f64, DensitySource)> {
    if let Some(density) = attributes.get(DENSITY_KEY).and_then(positive_number) {
        return Some((density, DensitySource::Part));
    }
    attributes
        .get(MATERIAL_KEY)
        .and_then(Value::as_str)
        .and_then(material_density)
        .map(|density| (density, DensitySource::Material))
}

/// What a mass reading is.
#[derive(Clone, Debug, PartialEq)]
pub enum MassStatus {
    /// Volume × density over the part's geometry.
    Computed,
    /// A part with no geometry: its stored `Mass`.
    Override,
    /// A measurement is still in flight.
    Pending,
    /// The geometry is measured but some solid has no density.
    NoDensity,
    /// No geometry and no stored mass.
    NoGeometry,
    /// The kernel could not measure a solid.
    Failed(String),
}

impl MassStatus {
    pub fn as_str(&self) -> &'static str {
        match self {
            MassStatus::Computed => "computed",
            MassStatus::Override => "override",
            MassStatus::Pending => "pending",
            MassStatus::NoDensity => "no-density",
            MassStatus::NoGeometry => "no-geometry",
            MassStatus::Failed(_) => "failed",
        }
    }

    /// The sentence a reader sees when there is no number.
    pub fn note(&self) -> Option<String> {
        match self {
            MassStatus::Computed | MassStatus::Override => None,
            MassStatus::Pending => Some("measuring".into()),
            MassStatus::NoDensity => {
                Some("no density: set the part's Material or Density".into())
            }
            MassStatus::NoGeometry => {
                Some("no geometry: set the part's Mass".into())
            }
            MassStatus::Failed(message) => Some(format!("not measurable: {message}")),
        }
    }
}

/// One part's (or one document's) mass properties.
#[derive(Clone, Debug, PartialEq)]
pub struct MassReport {
    /// Σ solid volume (mm³); `None` with no geometry or while pending.
    pub volume_mm3: Option<f64>,
    /// Mass (kg); `None` when it cannot be given — `status` says why.
    pub mass_kg: Option<f64>,
    /// The density (g/cm³) when every solid resolves the SAME one (`None`
    /// for a mixed-density part or when none resolves).
    pub density_g_cm3: Option<f64>,
    /// Where that density came from.
    pub density_source: Option<DensitySource>,
    /// How many solids the reading covers.
    pub solid_count: usize,
    pub status: MassStatus,
}

impl MassReport {
    /// The reading as JSON, units in the key names:
    /// `{volumeMm3, massKg, densityGcm3, densitySource, massStatus, massNote}`.
    pub fn to_json(&self) -> Value {
        serde_json::json!({
            "volumeMm3": self.volume_mm3,
            "massKg": self.mass_kg,
            "densityGcm3": self.density_g_cm3,
            "densitySource": self.density_source.map(DensitySource::as_str),
            "solidCount": self.solid_count,
            "massStatus": self.status.as_str(),
            "massNote": self.status.note(),
        })
    }
}

impl EngineState {
    /// The part attribute record that owns `solid`: the innermost part along
    /// its component chain (`ACOMP1:ACOMP2:Body` is the part ACOMP2 places
    /// inside the part ACOMP1 places), or the open document's own record for
    /// a solid with no chain. `None` when the chain cannot be resolved.
    pub(crate) fn owning_part_attributes(&self, solid: &str) -> Option<Value> {
        let (chain, _local) = brep_kernel::split_component_namespace(solid);
        let Some((&head, rest)) = chain.split_first() else {
            return Some(self.document_part_attributes());
        };
        let part_name = self
            .assembly_components
            .iter()
            .find(|component| component.id == head)
            .map(|component| component.part_name.clone())
            .or_else(|| {
                let index = self.history.index_of(head)?;
                self.history
                    .feature_params(index)?
                    .get("partName")?
                    .as_str()
                    .map(str::to_string)
            })?;
        let mut document = self
            .history
            .parts_library()
            .get(&part_name)?
            .get("document")?;
        for segment in rest {
            let inner = document
                .get("features")?
                .as_array()?
                .iter()
                .find(|feature| feature["inputParams"]["id"].as_str() == Some(segment))?;
            let inner_part = inner["inputParams"]["partName"].as_str()?;
            document = document
                .get("partsLibrary")?
                .get(inner_part)?
                .get("document")?;
        }
        Some(
            document
                .get(PART_ATTRIBUTES)
                .filter(|value| value.is_object())
                .cloned()
                .unwrap_or_else(|| Value::Object(serde_json::Map::new())),
        )
    }

    /// The density (g/cm³) a solid's mass is computed with, and where it came
    /// from: its own metadata `density`, else its part's (see the module).
    pub fn solid_density(&self, solid: &str) -> Option<(f64, DensitySource)> {
        if let Some(density) = self
            .metadata
            .attribute(solid, "density")
            .and_then(|text| text.trim().parse::<f64>().ok())
            .filter(|density| density.is_finite() && *density > 0.0)
        {
            return Some((density, DensitySource::Solid));
        }
        part_density(&self.owning_part_attributes(solid)?)
    }

    /// Measurable scene solids (sketch sheets have no volume).
    fn measurable_solids<'a>(&'a self, filter: impl Fn(&str) -> bool + 'a) -> Vec<String> {
        self.scene
            .solids()
            .iter()
            .filter(|solid| !solid.is_sketch && filter(&solid.name))
            .map(|solid| solid.name.clone())
            .collect()
    }

    /// The solids ONE placement of `part_name` produces at the top level: its
    /// first occurrence's (every occurrence is a rigid copy of the same part).
    fn part_solids(&self, part_name: &str) -> Vec<String> {
        let Some(first) = self
            .assembly_components
            .iter()
            .find(|component| component.part_name == part_name)
            .map(|component| component.id.clone())
        else {
            return Vec::new();
        };
        self.chain_solids(&first)
    }

    /// The solids under one occurrence CHAIN (`ACOMP9` or `ACOMP9:ACOMP1`).
    fn chain_solids(&self, chain: &str) -> Vec<String> {
        let prefix = format!("{chain}:");
        self.measurable_solids(|name| name.starts_with(&prefix))
    }

    /// The mass properties of what one occurrence chain places — a NESTED
    /// BOM line (`ACOMP9:ACOMP1` is the part ACOMP1 places inside the
    /// sub-assembly ACOMP9 places), each solid with its own part's density.
    /// Asks for missing measurements.
    pub fn chain_mass(&mut self, chain: &str) -> MassReport {
        let solids = self.chain_solids(chain);
        self.measure_solids(&solids);
        self.chain_mass_cached(chain)
    }

    /// [`Self::chain_mass`] from what is already measured.
    pub fn chain_mass_cached(&self, chain: &str) -> MassReport {
        let solids = self.chain_solids(chain);
        // The chain's own part record (a solid name under it resolves to it).
        let attributes = self
            .owning_part_attributes(&format!("{chain}:_"))
            .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
        self.mass_over(&solids, &attributes)
    }

    /// Ask for every placed part's measurements; `true` while any is still in
    /// flight (an export waits on this rather than writing blanks).
    pub fn bom_mass_pending(&mut self) -> bool {
        let parts: Vec<String> =
            self.assembly_components.iter().map(|c| c.part_name.clone()).collect();
        self.measure_parts(parts.iter().map(String::as_str));
        parts
            .iter()
            .any(|part| self.part_mass_cached(part).status == MassStatus::Pending)
    }

    /// Whether the part has geometry to measure (then its mass is computed and
    /// a typed `Mass` is refused).
    pub fn part_has_geometry(&self, part_name: &str) -> bool {
        !self.part_solids(part_name).is_empty()
    }

    /// Whether the open document has geometry to measure.
    pub fn document_has_geometry(&self) -> bool {
        !self.measurable_solids(|_| true).is_empty()
    }

    /// A solid's volume (mm³) off the measurement cache, WITHOUT asking:
    /// `Ok(None)` while no answer is cached (see [`Self::measure_solids`]).
    fn solid_volume_cached(&self, solid: &str) -> Result<Option<f64>, String> {
        let Some(text) = self.info_cache.get(solid) else {
            return Ok(None);
        };
        let info: Value = serde_json::from_str(text).unwrap_or(Value::Null);
        if info.get("ok").and_then(Value::as_bool) != Some(true) {
            return Err(info
                .get("message")
                .and_then(Value::as_str)
                .unwrap_or("no measurement")
                .to_string());
        }
        info.get("volume")
            .and_then(Value::as_f64)
            .map(Some)
            .ok_or_else(|| "no volume".to_string())
    }

    /// Ask the runner to measure every solid not measured yet (one query per
    /// solid, deduplicated by the measurement lane itself). An Inline runner
    /// answers here; a threaded one on a later frame.
    fn measure_solids(&mut self, solids: &[String]) {
        for solid in solids {
            if !self.info_cache.contains_key(solid) {
                let _ = self.object_info_raw_json(solid);
            }
        }
    }

    /// Ask for the measurements the named parts' masses need — what a panel
    /// calls before it reads [`Self::part_bom_record_cached`] for each row.
    pub fn measure_parts<'a>(&mut self, parts: impl IntoIterator<Item = &'a str>) {
        for part in parts {
            let solids = self.part_solids(part);
            self.measure_solids(&solids);
        }
    }

    /// Mass properties over `solids`, falling back to `attributes`' stored
    /// `Mass` when there are none.
    fn mass_over(&self, solids: &[String], attributes: &Value) -> MassReport {
        if solids.is_empty() {
            let stored = attributes.get(MASS_KEY).and_then(positive_number);
            return MassReport {
                volume_mm3: None,
                mass_kg: stored,
                density_g_cm3: None,
                density_source: None,
                solid_count: 0,
                status: if stored.is_some() {
                    MassStatus::Override
                } else {
                    MassStatus::NoGeometry
                },
            };
        }
        let mut volume = 0.0;
        let mut mass = 0.0;
        let mut pending = false;
        let mut failure: Option<String> = None;
        let mut missing_density = false;
        let mut densities: Vec<(f64, DensitySource)> = Vec::new();
        for solid in solids {
            match self.solid_volume_cached(solid) {
                Ok(Some(v)) => {
                    volume += v;
                    match self.solid_density(solid) {
                        Some((density, source)) => {
                            mass += v * density * MM3_G_CM3_TO_KG;
                            densities.push((density, source));
                        }
                        None => missing_density = true,
                    }
                }
                Ok(None) => pending = true,
                Err(message) => {
                    failure.get_or_insert(format!("{solid}: {message}"));
                }
            }
        }
        let uniform = densities
            .first()
            .copied()
            .filter(|(first, _)| densities.iter().all(|(d, _)| (d - first).abs() <= 1e-12));
        let status = if let Some(message) = failure {
            MassStatus::Failed(message)
        } else if pending {
            MassStatus::Pending
        } else if missing_density {
            MassStatus::NoDensity
        } else {
            MassStatus::Computed
        };
        MassReport {
            volume_mm3: matches!(status, MassStatus::Computed | MassStatus::NoDensity)
                .then_some(volume),
            mass_kg: (status == MassStatus::Computed).then_some(mass),
            density_g_cm3: uniform.map(|(density, _)| density),
            density_source: uniform.map(|(_, source)| source),
            solid_count: solids.len(),
            status,
        }
    }

    /// A parts-library part's mass properties (see the module), asking for
    /// any measurement still missing.
    pub fn part_mass(&mut self, part_name: &str) -> MassReport {
        let solids = self.part_solids(part_name);
        self.measure_solids(&solids);
        self.part_mass_cached(part_name)
    }

    /// [`Self::part_mass`] from what is already measured (never asks; a
    /// solid not measured yet reads as pending).
    pub fn part_mass_cached(&self, part_name: &str) -> MassReport {
        let solids = self.part_solids(part_name);
        let attributes = self.part_attributes(part_name);
        let mut report = self.mass_over(&solids, &attributes);
        if solids.is_empty() {
            // No geometry: still say what density the part names.
            if let Some((density, source)) = part_density(&attributes) {
                report.density_g_cm3 = Some(density);
                report.density_source = Some(source);
            }
        }
        report
    }

    /// The OPEN document's own mass properties: every measurable solid it
    /// shows (a part's bodies; an assembly's placed components, each with its
    /// own part's density).
    pub fn document_mass(&mut self) -> MassReport {
        let solids = self.measurable_solids(|_| true);
        self.measure_solids(&solids);
        self.document_mass_cached()
    }

    /// [`Self::document_mass`] from what is already measured.
    pub fn document_mass_cached(&self) -> MassReport {
        let solids = self.measurable_solids(|_| true);
        let attributes = self.document_part_attributes();
        let mut report = self.mass_over(&solids, &attributes);
        if solids.is_empty() {
            if let Some((density, source)) = part_density(&attributes) {
                report.density_g_cm3 = Some(density);
                report.density_source = Some(source);
            }
        }
        report
    }

    /// A part's BOM record: its stored attributes with the DERIVED mass
    /// properties folded in — `Volume` (mm³) and `Mass` (kg) as computed, and
    /// `Density` as resolved when the part names only a `Material`. A `Mass`
    /// stored on a part with geometry is not what the BOM shows: the computed
    /// value replaces it, or the key is absent when there is none yet.
    pub fn part_bom_record(&mut self, part_name: &str) -> Value {
        let report = self.part_mass(part_name);
        fold_mass(self.part_attributes(part_name), &report)
    }

    /// [`Self::part_bom_record`] from what is already measured, plus the
    /// reading itself (a panel row shows the note when there is no number).
    pub fn part_bom_record_cached(&self, part_name: &str) -> (Value, MassReport) {
        let report = self.part_mass_cached(part_name);
        (fold_mass(self.part_attributes(part_name), &report), report)
    }

    /// The open document's own record with its derived mass properties — the
    /// same fold as [`Self::part_bom_record`].
    pub fn document_bom_record(&mut self) -> Value {
        let attributes = self.document_part_attributes();
        let report = self.document_mass();
        fold_mass(attributes, &report)
    }

    /// Refuse a write a part attribute may not take: `Volume` is derived, and
    /// `Mass` is computed whenever there is geometry to compute it from.
    pub(crate) fn check_part_attribute_write(
        &self,
        key: &str,
        value: &Value,
        has_geometry: bool,
    ) -> Result<(), String> {
        let clearing =
            value.is_null() || matches!(value, Value::String(text) if text.is_empty());
        if key == VOLUME_KEY && !clearing {
            return Err("Volume is computed from the part's geometry and cannot be set".into());
        }
        if key == MASS_KEY && !clearing && has_geometry {
            return Err(
                "Mass is computed from the part's geometry and density: set the part's Material or Density instead (a Mass can be typed only for a part with no geometry)"
                    .into(),
            );
        }
        Ok(())
    }
}

/// Fold a mass reading into an attribute record (see
/// [`EngineState::part_bom_record`]).
fn fold_mass(mut attributes: Value, report: &MassReport) -> Value {
    let Some(record) = attributes.as_object_mut() else {
        return attributes;
    };
    record.remove(VOLUME_KEY);
    if report.status != MassStatus::Override {
        record.remove(MASS_KEY);
    }
    if let Some(volume) = report.volume_mm3 {
        record.insert(VOLUME_KEY.into(), Value::from(volume));
    }
    if let Some(mass) = report.mass_kg {
        record.insert(MASS_KEY.into(), Value::from(mass));
    }
    if !record.contains_key(DENSITY_KEY) {
        if let (Some(density), Some(DensitySource::Material)) =
            (report.density_g_cm3, report.density_source)
        {
            record.insert(DENSITY_KEY.into(), Value::from(density));
        }
    }
    attributes
}

/// Lift part-only attributes (`Mass`, `Material`, `Density`) that an older
/// document stored on its OCCURRENCES onto the PART, in place, before the
/// document is loaded. Returns the notices to show (one per part whose
/// occurrences disagreed or whose part already said otherwise, plus one
/// summary of what moved).
///
/// * Both occurrence records are swept: the BOM's `bom` record and the
///   `occurrenceAttributes` key an older batch path wrote (its other keys are
///   folded into `bom`, which wins on a clash).
/// * Equal values move to the part when the part does not already carry the
///   key; the occurrences lose them either way.
/// * Disagreeing occurrences: the FIRST occurrence's value goes to the part
///   (unless the part already carries one), and the disagreement is reported
///   once for that part.
/// * Every embedded part document is swept too (a sub-assembly's own
///   occurrences), and a changed entry is re-stamped as an attribute edit is.
pub fn migrate_occurrence_part_attributes(document: &mut Value) -> Vec<String> {
    let mut notices = Vec::new();
    migrate_level(document, "", &mut notices);
    notices
}

fn migrate_level(document: &mut Value, path: &str, notices: &mut Vec<String>) {
    // (part, key) -> [(occurrence id, value)] in history order.
    let mut found: Vec<(String, String, Vec<(String, Value)>)> = Vec::new();
    if let Some(features) = document.get_mut("features").and_then(Value::as_array_mut) {
        for feature in features {
            let is_acomp = feature
                .get("type")
                .and_then(Value::as_str)
                .is_some_and(super::components::is_acomp_feature_type);
            if !is_acomp {
                continue;
            }
            let Some(params) = feature.get_mut("inputParams").and_then(Value::as_object_mut) else {
                continue;
            };
            let id = params.get("id").and_then(Value::as_str).unwrap_or("").to_string();
            let part = params.get("partName").and_then(Value::as_str).unwrap_or("").to_string();
            // Fold the legacy key into the real record (the real one wins).
            if let Some(Value::Object(legacy)) = params.remove("occurrenceAttributes") {
                let record = params
                    .entry(super::bom::OCCURRENCE_ATTRIBUTES.to_string())
                    .or_insert_with(|| Value::Object(serde_json::Map::new()));
                if !record.is_object() {
                    *record = Value::Object(serde_json::Map::new());
                }
                let record = record.as_object_mut().expect("normalized");
                for (key, value) in legacy {
                    record.entry(key).or_insert(value);
                }
            }
            let Some(record) = params
                .get_mut(super::bom::OCCURRENCE_ATTRIBUTES)
                .and_then(Value::as_object_mut)
            else {
                continue;
            };
            for key in ["Mass", "Material", "Density", "Volume"] {
                let Some(value) = record.remove(key) else { continue };
                let blank = value.is_null() || matches!(&value, Value::String(t) if t.trim().is_empty());
                if blank || key == VOLUME_KEY || part.is_empty() {
                    continue;
                }
                match found.iter_mut().find(|(p, k, _)| *p == part && k == key) {
                    Some((_, _, values)) => values.push((id.clone(), value)),
                    None => found.push((part.clone(), key.to_string(), vec![(id.clone(), value)])),
                }
            }
            if record.is_empty() {
                params.remove(super::bom::OCCURRENCE_ATTRIBUTES);
            }
        }
    }

    let mut moved: Vec<String> = Vec::new();
    let mut touched: std::collections::BTreeSet<String> = std::collections::BTreeSet::new();
    for (part, key, values) in &found {
        let label = if path.is_empty() { part.clone() } else { format!("{path}/{part}") };
        let Some(entry) = document
            .get_mut("partsLibrary")
            .and_then(|library| library.get_mut(part))
            .filter(|entry| entry.get("document").is_some_and(Value::is_object))
        else {
            notices.push(format!(
                "BOM: {key} on {} occurrence(s) of '{label}' was dropped: the part is not in the parts library",
                values.len()
            ));
            continue;
        };
        let first = &values[0].1;
        let disagree = values.iter().any(|(_, value)| value != first);
        let part_document = entry.get_mut("document").expect("filtered");
        let existing = part_document
            .get(PART_ATTRIBUTES)
            .and_then(|record| record.get(key))
            .cloned();
        let shown = |value: &Value| match value {
            Value::String(text) => format!("'{text}'"),
            other => other.to_string(),
        };
        match existing {
            Some(existing) if &existing != first || disagree => {
                notices.push(format!(
                    "BOM: '{label}' occurrences carried {key} {} — the part keeps its own {key} {}",
                    values.iter().map(|(id, value)| format!("{id}={}", shown(value))).collect::<Vec<_>>().join(", "),
                    shown(&existing),
                ));
            }
            Some(_) => {}
            None => {
                let object = part_document.as_object_mut().expect("filtered");
                let record = object
                    .entry(PART_ATTRIBUTES.to_string())
                    .or_insert_with(|| Value::Object(serde_json::Map::new()));
                if !record.is_object() {
                    *record = Value::Object(serde_json::Map::new());
                }
                record
                    .as_object_mut()
                    .expect("normalized")
                    .insert(key.clone(), first.clone());
                touched.insert(part.clone());
                if disagree {
                    notices.push(format!(
                        "BOM: '{label}' occurrences disagreed on {key} ({}) — the part takes {} from {}",
                        values.iter().map(|(id, value)| format!("{id}={}", shown(value))).collect::<Vec<_>>().join(", "),
                        shown(first),
                        values[0].0,
                    ));
                } else {
                    moved.push(format!("{key} {} → part '{label}'", shown(first)));
                }
            }
        }
    }

    // Recurse into every embedded part document, then re-stamp what changed.
    if let Some(library) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) {
        for (name, entry) in library.iter_mut() {
            let Some(part_document) = entry.get_mut("document").filter(|d| d.is_object()) else {
                continue;
            };
            let before = notices.len();
            let label = if path.is_empty() { name.clone() } else { format!("{path}/{name}") };
            let changed_inside = migrate_level_changed(part_document, &label, notices);
            if changed_inside || notices.len() != before {
                touched.insert(name.clone());
            }
        }
        for name in &touched {
            if let Some(entry) = library.get_mut(name) {
                let text = entry["document"].to_string();
                entry["sourceSignature"] = Value::String(super::document_signature(&text));
            }
        }
    }

    if !moved.is_empty() {
        notices.push(format!(
            "BOM: mass properties moved from occurrences to their parts: {} (save to keep)",
            moved.join("; ")
        ));
    }
}

/// [`migrate_level`] on a nested document, reporting whether it changed.
fn migrate_level_changed(document: &mut Value, path: &str, notices: &mut Vec<String>) -> bool {
    let before = document.clone();
    migrate_level(document, path, notices);
    *document != before
}

