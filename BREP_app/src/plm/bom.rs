//! BOM, where-used and part attributes on the PLM (plan S6, D11).
//!
//! - [`fetch_bom`]: `GET /api/parts/:id/revisions/:rev/bom`, indented or flat,
//!   with the server's own sourcing and costs. The app shows them as sent and
//!   recomputes nothing.
//! - [`document_bom`]: the SAME shape read off the assembly's document: its
//!   occurrences folded into lines as S5 publishes them ([`super::uses`]),
//!   descending into each sub-assembly's own document. [`compare`] holds the
//!   two side by side, line for line. They agree exactly when the published
//!   uses lists are the documents', so a difference is a list that was not
//!   republished, or a Replace everywhere the document has not caught up with.
//! - [`where_used`]: "what breaks if I change this", before a new revision.
//! - [`patch_attribute`]: a part's catalog value, through `PATCH
//!   /api/parts/:id` (D11). No checkout, and `lock_released_attributes`
//!   decides whether a released part takes it at all: the refusal is the
//!   server's sentence.
use super::client::{PlmClient, PlmError};
use crate::store::DocumentIdentity;
use serde::Deserialize;
use serde_json::{json, Value};

/// The server's BOM of one revision (`BREP_plm/src/bom.rs` `Bom`).
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct ServerBom {
    pub comparison_lines: Vec<ServerLine>,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub state: String,
    pub flat: bool,
    pub levels: usize,
    pub lines: Vec<ServerLine>,
    pub totals: Vec<CurrencyTotal>,
    pub unpriced: usize,
    pub warnings: Vec<String>,
}

/// One line of the server's BOM.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, serde::Serialize)]
#[serde(default)]
pub struct ServerLine {
    pub part_type: String,
    pub part_values: std::collections::BTreeMap<String,Value>,
    pub owner_part: String,
    pub owner_revision: String,
    pub occurrence_ids: Vec<String>,
    pub occurrence_attributes: std::collections::BTreeMap<String,Value>,
    pub notes: String,
    pub level: usize,
    pub position: String,
    pub find_number: String,
    pub reference: String,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub state: String,
    pub floating: bool,
    pub category_path: String,
    pub attributes: Vec<String>,
    pub quantity: f64,
    pub unit: String,
    pub total: f64,
    pub assembly: bool,
    pub mpn: String,
    pub manufacturer: String,
    pub supplier: String,
    pub spn: String,
    pub currency: String,
    pub unit_price: Option<f64>,
    pub extended: Option<f64>,
    /// Counted in the totals (a priced line with nothing beneath it).
    pub costed: bool,
    pub warnings: Vec<String>,
    /// One word per warning (`unreleased`, `superseded`, …), for a chip.
    pub flags: Vec<String>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct CurrencyTotal {
    pub currency: String,
    pub total: f64,
    pub lines: usize,
}

/// `GET /api/parts/:id/where-used`.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct WhereUsed {
    pub part_id: String,
    pub number: String,
    pub revision_label: String,
    pub levels: usize,
    pub lines: Vec<WhereUsedLine>,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct WhereUsedLine {
    pub level: usize,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub state: String,
    /// The parent revision is released or in work: one a change still reaches.
    pub current: bool,
    pub uses_revision: String,
    pub floating: bool,
    pub quantity: f64,
    pub unit: String,
    pub find_number: String,
    pub top: bool,
}

/// The server's BOM of `part`/`revision`: every level, indented or flat.
pub async fn fetch_bom(client: &PlmClient, part: &str, revision: &str, flat: bool) -> Result<ServerBom, PlmError> {
    let path = format!("{}/bom?levels=0{}", super::identity::revision_path(part, revision), if flat { "&flat=true" } else { "" });
    json(client.call("GET", &path, None).await?, &path)
}

pub async fn fetch_occurrence_bom(client:&PlmClient,part:&str,revision:&str,flat:bool)->Result<ServerBom,PlmError>{
    let path=format!("{}/bom?levels=0&occurrences=true&flat={flat}",super::identity::revision_path(part,revision));
    json(client.call("GET",&path,None).await?,&path)
}

/// Every assembly that uses `part` (at `revision`, or any), every level up.
pub async fn where_used(client: &PlmClient, part: &str, revision: Option<&str>) -> Result<WhereUsed, PlmError> {
    let path = match revision {
        Some(revision) => format!("{}/where-used?levels=0&revision={}", super::identity::part_path(part), super::identity::segment(revision)),
        None => format!("{}/where-used?levels=0", crate::plm::identity::part_path(&part)),
    };
    json(client.call("GET", &path, None).await?, &path)
}

/// Set one catalog value on `part` (`null` or empty clears it). The key is
/// the server's (attribute keys are lower-case); the refusal, a released
/// part's lock included, is the server's sentence.
pub async fn patch_attribute(client: &PlmClient, part: &str, key: &str, value: &Value) -> Result<(), PlmError> {
    let body = json!({ "attributes": { attribute_key(key): value } });
    client
        .call("PATCH", &crate::plm::identity::part_path(&part), Some(body.to_string().into_bytes()))
        .await
        .map(|_| ())
}

/// The server's key for a BOM column's field: catalog keys are lower-case.
pub fn attribute_key(field: &str) -> String {
    field.trim().to_ascii_lowercase()
}

fn json<T: for<'de> Deserialize<'de>>(response: super::PlmResponse, what: &str) -> Result<T, PlmError> {
    serde_json::from_slice(&response.body).map_err(|e| PlmError::Malformed(format!("{what}: {e}")))
}

/// The PLM revision an open document IS, from its name: the store key
/// `part/<id>/rev/<id>`, or the explorer's path form of it
/// (`/models/part/<id>/rev/<id>.nbrep`).
pub fn revision_of_document(name: &str) -> Option<(String, String)> {
    let key = name.trim_start_matches('/');
    let key = key.strip_prefix("models/").unwrap_or(key);
    let key = key.strip_suffix(".nbrep").unwrap_or(key);
    match DocumentIdentity::parse_revision_key(key)? {
        DocumentIdentity::Revision { part, revision } => Some((part, revision)),
        DocumentIdentity::Path(_) => None,
    }
}

/// One line of the document's BOM, in the server's terms.
#[derive(Clone, Debug, PartialEq)]
pub struct DocumentLine {
    pub level: usize,
    /// `1.2.3`, as the server numbers its indented view.
    pub position: String,
    pub part_id: String,
    pub revision_id: String,
    /// Per ONE parent.
    pub quantity: f64,
    /// For ONE of the top: the quantities along the path multiplied.
    pub total: f64,
}

/// The document's BOM: each level's occurrences folded into lines the way S5
/// publishes them ([`super::uses::uses_lines`]), then each line's
/// sub-assembly descended into through the `partsLibrary` entry's own
/// document. `Err` names every occurrence, at any depth, that is no PLM
/// revision: such a document has no server BOM to be compared with.
pub fn document_bom(document: &Value) -> Result<Vec<DocumentLine>, Vec<String>> {
    let mut lines = Vec::new();
    let mut refusals = Vec::new();
    descend(document, 1, "", 1.0, &mut lines, &mut refusals);
    if refusals.is_empty() {
        Ok(lines)
    } else {
        Err(refusals)
    }
}

fn descend(document: &Value, level: usize, prefix: &str, parent_total: f64, out: &mut Vec<DocumentLine>, refusals: &mut Vec<String>) {
    let occurrences = super::uses::occurrences(document);
    let list = super::uses::uses_lines(&occurrences, &super::uses::plm_identity);
    for refusal in &list.refusals {
        let at = if prefix.is_empty() { String::new() } else { format!(" (under {prefix})") };
        refusals.push(format!("{}{at}: {}", refusal.occurrence, refusal.reason));
    }
    for (index, placed) in list.lines.iter().enumerate() {
        let position = if prefix.is_empty() { format!("{}", index + 1) } else { format!("{prefix}.{}", index + 1) };
        let quantity = f64::from(placed.line.quantity);
        let total = parent_total * quantity;
        out.push(DocumentLine {
            level,
            position: position.clone(),
            part_id: placed.line.part.clone(),
            revision_id: placed.line.revision.clone(),
            quantity,
            total,
        });
        // The first occurrence of the line names the library entry; every
        // occurrence of one line places the same part and revision.
        let entry = placed
            .occurrences
            .first()
            .and_then(|id| occurrences.iter().find(|o| &o.id == id))
            .and_then(|o| document.get("partsLibrary")?.get(&o.part_name));
        if let Some(child) = entry.and_then(|entry| entry.get("document")).filter(|d| d.is_object()) {
            descend(child, level + 1, &position, total, out, refusals);
        }
    }
}

/// Line for line: the document's BOM against the server's indented one.
/// Empty when they agree on every line's level, position, part, revision,
/// quantity and total; otherwise one sentence per difference, naming the
/// position and the server's part number.
pub fn compare(document: &[DocumentLine], server: &[ServerLine]) -> Vec<String> {
    let mut differences = Vec::new();
    let named = |line: &ServerLine| {
        if line.number.is_empty() {
            line.part_id.clone()
        } else {
            format!("{} rev {}", line.number, line.revision_label)
        }
    };
    for index in 0..document.len().max(server.len()) {
        match (document.get(index), server.get(index)) {
            (Some(doc), Some(srv)) => {
                let mut says = Vec::new();
                if doc.position != srv.position || doc.level != srv.level {
                    says.push(format!("the document has position {} (level {}), the PLM {} (level {})", doc.position, doc.level, srv.position, srv.level));
                }
                if doc.part_id != srv.part_id || doc.revision_id != srv.revision_id {
                    says.push(format!("the document places part {} revision {}", doc.part_id, doc.revision_id));
                }
                if (doc.quantity - srv.quantity).abs() > 1e-9 {
                    says.push(format!("quantity {} in the document, {} on the PLM", doc.quantity, srv.quantity));
                }
                if (doc.total - srv.total).abs() > 1e-9 {
                    says.push(format!("total {} in the document, {} on the PLM", doc.total, srv.total));
                }
                if !says.is_empty() {
                    differences.push(format!("line {} ({}): {}", srv.position, named(srv), says.join("; ")));
                }
            }
            (Some(doc), None) => differences.push(format!(
                "line {}: the document places part {} revision {}, which the PLM's BOM does not list",
                doc.position, doc.part_id, doc.revision_id
            )),
            (None, Some(srv)) => differences.push(format!("line {} ({}): on the PLM's BOM, not in the document", srv.position, named(srv))),
            (None, None) => {}
        }
    }
    differences
}

/// Draw a where-used answer: one row per parent revision, indented by level,
/// current ones plain and history weak. For the BOM panel and for S3's New
/// revision, which shows it before a new revision starts.
pub fn show_where_used(ui: &mut eframe::egui::Ui, used: &WhereUsed) {
    use eframe::egui;
    if used.lines.is_empty() {
        ui.label(format!("{} is used by no assembly.", used.number));
        return;
    }
    let current = used.lines.iter().filter(|l| l.current).count();
    ui.label(format!(
        "{} is used in {} assembly revision{} ({} current, which a change still reaches):",
        used.number,
        used.lines.len(),
        if used.lines.len() == 1 { "" } else { "s" },
        current
    ));
    egui::Grid::new(("where-used", &used.part_id)).striped(true).show(ui, |ui| {
        for heading in ["Assembly", "Rev", "State", "Uses", "Qty"] {
            ui.strong(heading);
        }
        ui.end_row();
        for line in &used.lines {
            let text = format!("{}{} {}", "  ".repeat(line.level.saturating_sub(1)), line.number, line.name);
            if line.current {
                ui.label(text);
            } else {
                ui.label(egui::RichText::new(text).weak());
            }
            ui.label(&line.revision_label);
            ui.label(&line.state);
            ui.label(&line.uses_revision);
            ui.label(format!("{} {}", line.quantity, line.unit));
            ui.end_row();
        }
    });
}

