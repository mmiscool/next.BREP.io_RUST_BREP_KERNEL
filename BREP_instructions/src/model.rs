//! The `instructions` block: everything an instructions document authors.
//!
//! An instructions document places its target assembly as one grounded
//! component and keeps the authored instructions in this top-level block. The
//! block is plain data: a tree of steps. A step adds parts and may hold child
//! steps, the group it installs: the children build the group (and see only
//! it), then the step places it over what its earlier siblings built. Reading
//! order is children first, then their parent, depth first. A sub-assembly is
//! such a step: its parts' steps are its children. Views are PMI views of the
//! document, named by id; everything else a step shows is here.
//!
//! Schema 1 kept chapters (one per assembly level) of flat steps; it reads as
//! the same tree ([`Instructions::from_value`]).

use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashSet;

/// The document key the block is stored under.
pub const BLOCK_KEY: &str = "instructions";

/// The block's schema version.
pub const SCHEMA: u32 = 2;

/// The `documentClass` value of an instructions document.
pub const DOCUMENT_CLASS: &str = "instructions";

/// The file extension of an instructions document, dot included.
pub const EXTENSION: &str = ".ibrep";

/// The part attribute that marks a fastener (bool).
pub const FASTENER_ATTRIBUTE: &str = "Fastener";
/// The part attribute holding a fastener's torque, as text with its unit.
pub const TORQUE_ATTRIBUTE: &str = "Torque";
/// The part attribute that marks a serial-tracked part (bool).
pub const SERIAL_TRACKED_ATTRIBUTE: &str = "Serial_Tracked";

/// The steps-per-page counts a template offers.
pub const STEPS_PER_PAGE: [u8; 3] = [1, 2, 4];

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Instructions {
    pub schema: u32,
    /// The assembly revision these instructions describe. Authoritative: the
    /// server derives what the instructions reference from this field alone.
    pub target: Target,
    #[serde(default)]
    pub template: Template,
    #[serde(default)]
    pub front: FrontMatter,
    /// The top-level steps, in order; each may hold children ([`Step::steps`]).
    #[serde(default)]
    pub steps: Vec<Step>,
    #[serde(default)]
    pub id_counter: u64,
    /// A re-target waiting for the author's approval. Release is refused while
    /// it is set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retarget: Option<Retarget>,
}

/// One revision of the target assembly.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Target {
    /// The store key the assembly was placed from: `part/<id>/rev/<rev>` in a
    /// PLM session, the file's key otherwise.
    pub source_key: String,
    #[serde(default)]
    pub part_number: String,
    /// The revision label, for display.
    #[serde(default)]
    pub revision: String,
    /// The placed document's content signature.
    #[serde(default)]
    pub signature: String,
    /// The assembly's name, for titles.
    #[serde(default)]
    pub name: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum Orientation {
    #[default]
    Portrait,
    Landscape,
}

/// The printed page template, one per document.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Template {
    /// 1, 2 or 4.
    pub steps_per_page: u8,
    pub orientation: Orientation,
    /// A paper-size key (`A4`, `Letter`, …).
    pub paper: String,
}

impl Default for Template {
    fn default() -> Self {
        Self { steps_per_page: 1, orientation: Orientation::Portrait, paper: "A4".into() }
    }
}

/// The cover and the lists before the steps.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrontMatter {
    #[serde(default)]
    pub title: String,
    #[serde(default)]
    pub subtitle: String,
    /// The step whose view is the cover picture; the last step when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cover_step: Option<String>,
    pub parts_list: bool,
    pub tools_list: bool,
}

impl Default for FrontMatter {
    fn default() -> Self {
        Self { title: String::new(), subtitle: String::new(), cover_step: None, parts_list: true, tools_list: true }
    }
}

/// A schema-1 chapter: the steps of one assembly level, read only to migrate.
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "camelCase")]
struct ChapterV1 {
    id: String,
    /// The occurrence path of the assembly the chapter built; `""` for the
    /// target assembly itself.
    #[serde(default)]
    assembly: String,
    #[serde(default)]
    title: String,
    #[serde(default)]
    steps: Vec<Step>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Step {
    pub id: String,
    pub title: String,
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub notes: String,
    #[serde(default)]
    pub adds: Vec<Add>,
    /// The PMI view (camera, exploded state, balloons) the step shows.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub view: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub tools: Vec<ToolUse>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub torque: Option<Torque>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub callouts: Vec<Callout>,
    /// Insertion-motion arrows: each played as a short move home.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub arrows: Vec<Arrow>,
    /// Reserved for a motion clip attached to the step.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub motion: Option<Value>,
    #[serde(default)]
    pub signoff: Signoff,
    /// The author captured the view's camera from the viewport: it is kept
    /// through every regeneration until Reset view. Otherwise the engine
    /// generates the camera from the step's parts and its parent's direction.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub camera_captured: bool,
    /// Child steps, in order: the group this step installs. They are read
    /// before it and see only the group's own parts.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub steps: Vec<Step>,
}

/// Parts one step adds: one part, at one or more occurrences.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Add {
    /// Occurrence paths, relative to the document (`ACOMP1:ACOMP3`).
    pub occurrences: Vec<String>,
    /// The part: its PLM part id when it was placed from a PLM revision
    /// (what the server and the thumbnails key on), else its name.
    pub part: String,
    /// The display name, when it differs from `part` (a PLM part): what the
    /// pane, the parts list and the player show. See [`Add::display_name`].
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    #[serde(default)]
    pub find_number: String,
    /// A finished sub-assembly placed as one part.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub subassembly: bool,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub fastener: bool,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub serial_tracked: bool,
}

impl Add {
    pub fn quantity(&self) -> usize {
        self.occurrences.len()
    }

    /// What to show for the part: its name, else `part` (an add written
    /// before names were carried, or a part that is its own name).
    pub fn display_name(&self) -> &str {
        if self.name.is_empty() {
            &self.part
        } else {
            &self.name
        }
    }

    /// An add for `part` named `name` (the name is written only when it
    /// differs).
    pub fn named(occurrences: Vec<String>, part: &str, name: &str) -> Self {
        Self {
            occurrences,
            part: part.to_string(),
            name: if name == part { String::new() } else { name.to_string() },
            ..Self::default()
        }
    }

    /// The same BOM row: the same part under the same find number, placed
    /// the same way (a sub-assembly is not the part of the same name).
    fn same_row(&self, other: &Add) -> bool {
        self.part == other.part && self.find_number == other.find_number && self.subassembly == other.subassembly
    }
}

impl Step {
    /// An empty step with `id` and `title`.
    pub fn new(id: String, title: String) -> Self {
        Self { id, title, ..Self::default() }
    }

    /// Whether `step_id` is this step or one of its descendants.
    pub fn contains(&self, step_id: &str) -> bool {
        self.id == step_id || self.steps.iter().any(|s| s.contains(step_id))
    }

    /// Whether this step places a sub-assembly at `occurrence`.
    fn places(&self, occurrence: &str) -> bool {
        self.adds.iter().any(|a| a.subassembly && a.occurrences.iter().any(|o| o == occurrence))
    }

    /// Every occurrence this step and its descendants add.
    pub fn occurrences_with_descendants<'a>(&'a self, out: &mut Vec<&'a str>) {
        out.extend(self.adds.iter().flat_map(|a| a.occurrences.iter().map(String::as_str)));
        for child in &self.steps {
            child.occurrences_with_descendants(out);
        }
    }

    /// One BOM row per part: adds of the same row (see `Add::same_row`)
    /// become one add holding every occurrence, in first-add order. Merged
    /// steps, moved parts and documents written one add per occurrence all
    /// read as `8 × bolt`, never eight rows of `1 × bolt`.
    pub fn coalesce_adds(&mut self) {
        let mut rows: Vec<Add> = Vec::with_capacity(self.adds.len());
        for mut add in std::mem::take(&mut self.adds) {
            match rows.iter_mut().find(|row| row.same_row(&add)) {
                Some(row) => {
                    row.occurrences.append(&mut add.occurrences);
                    row.fastener |= add.fastener;
                    row.serial_tracked |= add.serial_tracked;
                }
                None => rows.push(add),
            }
        }
        self.adds = rows;
    }
}

/// A tool a step uses: a tools-library entry by id, with a snapshot of its
/// name and spec so printed and offline copies need no server.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolUse {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_id: Option<String>,
    pub name: String,
    #[serde(default)]
    pub spec: String,
    /// Added without the library (no session); listed for linking.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub unlinked: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TorqueSource {
    /// Defaulted from the fastener part's attribute.
    Part,
    /// Set on the step by the author.
    Override,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Torque {
    /// As authored (`12`, `10-12`).
    pub value: String,
    #[serde(default)]
    pub unit: String,
    pub source: TorqueSource,
}

impl Torque {
    /// A torque attribute's text (`12 N·m`) split at its first space into
    /// value and unit; `None` for an empty one.
    pub fn from_attribute(text: &str) -> Option<Self> {
        let text = text.trim();
        if text.is_empty() {
            return None;
        }
        let (value, unit) = match text.split_once(char::is_whitespace) {
            Some((value, unit)) => (value.trim(), unit.trim()),
            None => (text, ""),
        };
        Some(Self { value: value.into(), unit: unit.into(), source: TorqueSource::Part })
    }

    /// `12 N·m`.
    pub fn text(&self) -> String {
        if self.unit.is_empty() {
            self.value.clone()
        } else {
            format!("{} {}", self.value, self.unit)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CalloutKind {
    Caution,
    Note,
    Check,
}

impl CalloutKind {
    pub fn label(self) -> &'static str {
        match self {
            Self::Caution => "Caution",
            Self::Note => "Note",
            Self::Check => "Check",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Callout {
    pub id: String,
    pub kind: CalloutKind,
    pub text: String,
    /// The occurrence the callout points at, if any.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub occurrence: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Arrow {
    pub occurrence: String,
    /// The direction the part travels IN from (unit-free; normalised on use).
    pub direction: [f64; 3],
    /// How far out the motion starts, in model units.
    pub distance: f64,
}

impl Arrow {
    /// The start offset of the insertion: `-direction` scaled to `distance`,
    /// so the part travels along `direction` to home. Zero for a zero vector.
    pub fn start_offset(&self) -> [f64; 3] {
        let [x, y, z] = self.direction;
        let length = (x * x + y * y + z * z).sqrt();
        if !(length > 1e-12) || !self.distance.is_finite() {
            return [0.0; 3];
        }
        let k = -self.distance / length;
        [x * k, y * k, z * k]
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Signoff {
    pub required: bool,
}

impl Default for Signoff {
    fn default() -> Self {
        Self { required: true }
    }
}

/// One occurrence in a re-target diff.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OccurrenceChange {
    pub occurrence: String,
    pub part: String,
    /// For a replaced occurrence: the part it was.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub was: Option<String>,
}

/// A re-target waiting for approval: what changed between the two assembly
/// revisions and the steps it touches.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Retarget {
    pub from: Target,
    pub to: Target,
    #[serde(default)]
    pub added: Vec<OccurrenceChange>,
    #[serde(default)]
    pub removed: Vec<OccurrenceChange>,
    #[serde(default)]
    pub replaced: Vec<OccurrenceChange>,
    /// Steps an add was removed from or changed in, and steps made for the
    /// added parts.
    #[serde(default)]
    pub touched_steps: Vec<String>,
    /// Steps left with nothing to add.
    #[serde(default)]
    pub empty_steps: Vec<String>,
}

impl Instructions {
    /// A new, empty block for `target`.
    pub fn new(target: Target) -> Self {
        let title = target.name.clone();
        Self {
            schema: SCHEMA,
            target,
            template: Template::default(),
            front: FrontMatter { title, ..FrontMatter::default() },
            steps: Vec::new(),
            id_counter: 0,
            retarget: None,
        }
    }

    /// Read the block from its JSON, validated. A schema-1 block (chapters of
    /// steps) reads as the tree: the root chapter's steps at the top, a
    /// sub-assembly's chapter as the children of the step that places it, and
    /// is schema 2 from then on (written back on the next edit).
    pub fn from_value(value: &Value) -> Result<Self, String> {
        let schema = value.get("schema").and_then(Value::as_u64).unwrap_or(0);
        let value = if schema == 1 { migrate_chapters(value)? } else { value.clone() };
        let mut block: Self = serde_json::from_value(value).map_err(|e| format!("instructions block: {e}"))?;
        block.validate()?;
        block.coalesce_adds();
        Ok(block)
    }

    /// Every step's adds as one row per part ([`Step::coalesce_adds`]).
    pub fn coalesce_adds(&mut self) {
        for_each_step_mut(&mut self.steps, &mut |step| step.coalesce_adds());
    }

    /// Read the block out of a whole document, if it has one.
    pub fn from_document(document: &Value) -> Result<Option<Self>, String> {
        match document.get(BLOCK_KEY) {
            None | Some(Value::Null) => Ok(None),
            Some(block) => Self::from_value(block).map(Some),
        }
    }

    pub fn to_value(&self) -> Value {
        serde_json::to_value(self).expect("instructions block serializes")
    }

    /// The structural rules: a known schema, a template the layout can draw,
    /// unique step ids over the whole tree, and no occurrence added twice.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema == 0 || self.schema > SCHEMA {
            return Err(format!("instructions schema {} is not supported (this build reads up to {SCHEMA})", self.schema));
        }
        if !STEPS_PER_PAGE.contains(&self.template.steps_per_page) {
            return Err(format!("steps per page must be 1, 2 or 4, not {}", self.template.steps_per_page));
        }
        let mut ids = HashSet::new();
        let mut occurrences = HashSet::new();
        for (_, step) in self.steps() {
            if !ids.insert(step.id.as_str()) {
                return Err(format!("duplicate id '{}'", step.id));
            }
            for add in &step.adds {
                for occurrence in &add.occurrences {
                    if !occurrences.insert(occurrence.as_str()) {
                        return Err(format!("occurrence '{occurrence}' is added in more than one step"));
                    }
                }
            }
        }
        Ok(())
    }

    /// A fresh id with `prefix` (`MS` step, `MK` callout; `MC` was a chapter).
    pub fn next_id(&mut self, prefix: &str) -> String {
        self.id_counter += 1;
        format!("{prefix}{}", self.id_counter)
    }

    pub fn step_count(&self) -> usize {
        self.steps().count()
    }

    /// Every step in reading order (a step's children, depth first, before the
    /// step itself) with its depth (0 at the top).
    pub fn steps(&self) -> impl Iterator<Item = (usize, &Step)> {
        fn walk<'a>(steps: &'a [Step], depth: usize, out: &mut Vec<(usize, &'a Step)>) {
            for step in steps {
                walk(&step.steps, depth + 1, out);
                out.push((depth, step));
            }
        }
        let mut out = Vec::with_capacity(16);
        walk(&self.steps, 0, &mut out);
        out.into_iter()
    }

    /// The index path of a step: an index per level from the top.
    fn path_of(&self, step_id: &str) -> Option<Vec<usize>> {
        fn walk(steps: &[Step], step_id: &str, path: &mut Vec<usize>) -> bool {
            for (i, step) in steps.iter().enumerate() {
                path.push(i);
                if step.id == step_id || walk(&step.steps, step_id, path) {
                    return true;
                }
                path.pop();
            }
            false
        }
        let mut path = Vec::new();
        walk(&self.steps, step_id, &mut path).then_some(path)
    }

    pub fn step(&self, step_id: &str) -> Option<&Step> {
        self.steps().find(|(_, s)| s.id == step_id).map(|(_, s)| s)
    }

    pub fn step_mut(&mut self, step_id: &str) -> Option<&mut Step> {
        let path = self.path_of(step_id)?;
        let (last, parents) = path.split_last()?;
        Some(&mut list_at_mut(&mut self.steps, parents)[*last])
    }

    /// The list the step sits in (its siblings, itself included) and its index
    /// there.
    pub fn siblings(&self, step_id: &str) -> Option<(&[Step], usize)> {
        let path = self.path_of(step_id)?;
        let (last, parents) = path.split_last()?;
        let mut list = &self.steps;
        for &i in parents {
            list = &list[i].steps;
        }
        Some((list.as_slice(), *last))
    }

    pub fn siblings_mut(&mut self, step_id: &str) -> Option<(&mut Vec<Step>, usize)> {
        let path = self.path_of(step_id)?;
        let (last, parents) = path.split_last()?;
        Some((list_at_mut(&mut self.steps, parents), *last))
    }

    /// The parent step's id: `Some(None)` at the top level, `None` for an
    /// unknown step.
    pub fn parent_id(&self, step_id: &str) -> Option<Option<&str>> {
        let path = self.path_of(step_id)?;
        let (_, parents) = path.split_last()?;
        let mut list = &self.steps;
        let mut parent = None;
        for &i in parents {
            parent = Some(list[i].id.as_str());
            list = &list[i].steps;
        }
        Some(parent)
    }

    /// How deep a step sits: 0 at the top level.
    pub fn depth(&self, step_id: &str) -> Option<usize> {
        self.path_of(step_id).map(|path| path.len() - 1)
    }

    /// The steps that build one assembly: the top level for `""`, else the
    /// children of the step that places the sub-assembly at `occurrence`.
    pub fn group_steps_mut(&mut self, occurrence: &str) -> Option<&mut Vec<Step>> {
        if occurrence.is_empty() {
            return Some(&mut self.steps);
        }
        let placing = self.steps().find(|(_, s)| s.places(occurrence)).map(|(_, s)| s.id.clone())?;
        self.step_mut(&placing).map(|step| &mut step.steps)
    }

    /// The step number a reader sees (1-based, in reading order).
    pub fn step_number(&self, step_id: &str) -> Option<usize> {
        self.steps().position(|(_, s)| s.id == step_id).map(|n| n + 1)
    }

    /// Distinct tools in first-use order, with the step ids that use each.
    pub fn tools_list(&self) -> Vec<(ToolUse, Vec<String>)> {
        let mut list: Vec<(ToolUse, Vec<String>)> = Vec::new();
        for (_, step) in self.steps() {
            for tool in &step.tools {
                let same = |t: &ToolUse| match (&t.tool_id, &tool.tool_id) {
                    (Some(a), Some(b)) => a == b,
                    (None, None) => t.name == tool.name && t.spec == tool.spec,
                    _ => false,
                };
                match list.iter_mut().find(|(t, _)| same(t)) {
                    Some((_, steps)) => {
                        if !steps.contains(&step.id) {
                            steps.push(step.id.clone());
                        }
                    }
                    None => list.push((tool.clone(), vec![step.id.clone()])),
                }
            }
        }
        list
    }

    /// The parts list: one row per part (and per sub-assembly), with its
    /// find number and the quantity added over every step, in first-add order.
    pub fn parts_list(&self) -> Vec<PartsRow> {
        let mut rows: Vec<PartsRow> = Vec::new();
        for (_, step) in self.steps() {
            for add in &step.adds {
                match rows.iter_mut().find(|r| r.part == add.part && r.find_number == add.find_number) {
                    Some(row) => row.quantity += add.quantity(),
                    None => rows.push(PartsRow {
                        part: add.part.clone(),
                        name: add.display_name().to_string(),
                        find_number: add.find_number.clone(),
                        quantity: add.quantity(),
                        subassembly: add.subassembly,
                        first_step: step.id.clone(),
                    }),
                }
            }
        }
        rows
    }
}

/// The list at an index path (every index but the step's own).
fn list_at_mut<'a>(mut list: &'a mut Vec<Step>, parents: &[usize]) -> &'a mut Vec<Step> {
    for &i in parents {
        list = &mut list[i].steps;
    }
    list
}

/// `f` on every step of the tree, parents before children.
pub fn for_each_step_mut(steps: &mut [Step], f: &mut dyn FnMut(&mut Step)) {
    for step in steps {
        f(step);
        for_each_step_mut(&mut step.steps, f);
    }
}

/// Keep the steps `keep` accepts, at every level.
pub fn retain_steps(steps: &mut Vec<Step>, keep: &dyn Fn(&Step) -> bool) {
    steps.retain(keep);
    for step in steps {
        retain_steps(&mut step.steps, keep);
    }
}

/// A schema-1 block as schema 2: the root chapter's steps are the top level;
/// a sub-assembly's chapter becomes the children of the step that places that
/// sub-assembly (so an old document reads as the tree the wizard proposes); a
/// chapter no step places becomes a top-level step of its own, ahead of the
/// root's steps as it printed, titled after the chapter and keeping its id.
fn migrate_chapters(value: &Value) -> Result<Value, String> {
    let mut object = value.as_object().cloned().ok_or("instructions block: not an object")?;
    let chapters: Vec<ChapterV1> = match object.remove("chapters") {
        Some(chapters) => serde_json::from_value(chapters).map_err(|e| format!("instructions block: chapters: {e}"))?,
        None => Vec::new(),
    };
    // Writers of that era put an empty `steps` object beside the chapters.
    object.remove("steps");
    let (root, others): (Vec<ChapterV1>, Vec<ChapterV1>) = chapters.into_iter().partition(|c| c.assembly.is_empty());
    let mut steps: Vec<Step> = root.into_iter().flat_map(|c| c.steps).collect();
    // Deepest chapters come first in a schema-1 block, so walk them from the
    // last: a sub-assembly's placing step is in place before its own chapter
    // looks for it.
    for chapter in others.into_iter().rev() {
        let mut placing = None;
        for_each_step_mut(&mut steps, &mut |step| {
            if placing.is_none() && step.places(&chapter.assembly) {
                placing = Some(step.id.clone());
            }
        });
        let chapter_steps = chapter.steps;
        match placing {
            Some(id) => {
                for_each_step_mut(&mut steps, &mut |step| {
                    if step.id == id {
                        step.steps = chapter_steps.clone();
                    }
                });
            }
            None => steps.insert(0, Step { id: chapter.id, title: chapter.title, steps: chapter_steps, ..Step::default() }),
        }
    }
    object.insert("schema".into(), Value::from(SCHEMA));
    object.insert("steps".into(), serde_json::to_value(steps).map_err(|e| format!("instructions block: {e}"))?);
    Ok(Value::Object(object))
}

/// One row of the parts list.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PartsRow {
    pub part: String,
    /// The display name ([`Add::display_name`]).
    pub name: String,
    pub find_number: String,
    pub quantity: usize,
    pub subassembly: bool,
    pub first_step: String,
}

