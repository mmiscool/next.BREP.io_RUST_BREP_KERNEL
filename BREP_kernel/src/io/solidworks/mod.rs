//! SolidWorks parts (`.sldprt`) and assemblies (`.sldasm`): import.
//!
//! A SolidWorks part keeps each configuration's body as a Parasolid partition
//! in neutral binary (`Contents/Config-N-Partition`): a section of zlib
//! chunks, then the partition's rollback deltas. The file around it is an OLE
//! compound document (SolidWorks before 2015) or SolidWorks' own container
//! (2015 and later: a ZIP-like archive with rotated stream names and raw
//! DEFLATE). Import finds the partition and reads it with the Parasolid
//! reader; nothing else in the file is needed.
//!
//! An assembly keeps no bodies: `swXmlContents/COMPINSTANCETREE` lists the
//! files it uses, their models (file and configuration) and each instance
//! (`swReference`) with its placement. `swTransform` is a row-vector matrix in
//! metres: p' = p R + t, the rotation's rows first and the translation last
//! (PSAS's canard machining assembly, exported by SolidWorks both as an
//! assembly and as a Parasolid assembly, agrees so to 1e-14; read as columns
//! it misses by 67 mm).
//!
//! Container notes: blussyya/sldprt-format-research (CC-BY-SA-4.0) and the
//! OLE compound file format ([MS-CFB]).

use crate::parasolid::ParasolidImportReport;

mod container;


pub use container::solidworks_streams;

/// The partition of each configuration of a SolidWorks part, `Config-0` first.
pub fn solidworks_part_partitions(bytes: &[u8]) -> Result<Vec<(String, Vec<u8>)>, String> {
    let streams = solidworks_streams(bytes, &|name: &str| name.starts_with("Contents/Config-") && name.ends_with("-Partition") && !name.ends_with("GhostPartition"))?;
    let mut out = Vec::new();
    for (name, data) in streams {
        let config = name.trim_start_matches("Contents/").trim_end_matches("-Partition").to_string();
        let partition = first_section(&data).map_err(|e| format!("{name}: {e}"))?;
        out.push((config, partition));
    }
    out.sort_by_key(|(config, _)| config.trim_start_matches("Config-").parse::<u32>().unwrap_or(u32::MAX));
    if out.is_empty() {
        return Err("the part holds no body (no Config-N-Partition stream)".into());
    }
    Ok(out)
}

/// The first section of a partition stream: a four-byte size, a 16-byte
/// identifier, zlib chunks (each a four-byte uncompressed and compressed
/// length, then the zlib data) and eight zero bytes. The partition's data is
/// the chunks' concatenation; the sections after it are its rollback deltas.
fn first_section(data: &[u8]) -> Result<Vec<u8>, String> {
    let u32_at = |at: usize| data.get(at..at + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).ok_or("the partition stream is cut short");
    let size = u32_at(0)?;
    let end = 4 + size;
    if end > data.len() {
        return Err(format!("its first section claims {size} bytes of {}", data.len()));
    }
    let mut at = 20;
    let mut out = Vec::new();
    while at + 8 <= end.saturating_sub(8) {
        let (full, packed) = (u32_at(at)?, u32_at(at + 4)?);
        if out.len() + full > MAX_PARTITION {
            return Err(format!("the partition would be larger than {MAX_PARTITION} bytes"));
        }
        let chunk = data.get(at + 8..at + 8 + packed).ok_or("a chunk runs past its section")?;
        out.extend(zlib(chunk, full)?);
        at += 8 + packed;
    }
    Ok(out)
}

/// The bound on a partition's size: far above any part's body, and what a
/// chunk's stated length may never take the total past.
const MAX_PARTITION: usize = 1 << 30;

/// A zlib stream (RFC 1950): a two-byte header, raw DEFLATE, then the
/// Adler-32 of the data, which is checked.
pub(crate) fn zlib(chunk: &[u8], expected: usize) -> Result<Vec<u8>, String> {
    if expected > MAX_PARTITION {
        return Err(format!("a chunk states {expected} bytes"));
    }
    if chunk.len() < 2 || chunk[0] & 0x0f != 8 || (u16::from(chunk[0]) << 8 | u16::from(chunk[1])) % 31 != 0 {
        return Err("a chunk is not zlib data".into());
    }
    let out = crate::three_mf::inflate::inflate(&chunk[2..], expected.max(1)).map_err(|e| format!("a zlib chunk: {e}"))?;
    if out.len() != expected {
        return Err(format!("a chunk inflates to {} bytes, not the {expected} it states", out.len()));
    }
    // The checksum follows the DEFLATE data at the stream's end.
    if let Some(tail) = chunk.len().checked_sub(4).and_then(|at| chunk.get(at..)) {
        let stated = u32::from_be_bytes([tail[0], tail[1], tail[2], tail[3]]);
        if adler32(&out) != stated {
            return Err("a zlib chunk's checksum does not match its data".into());
        }
    }
    Ok(out)
}

fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for chunk in data.chunks(5552) {
        for &x in chunk {
            a += u32::from(x);
            b += a;
        }
        a %= 65521;
        b %= 65521;
    }
    b << 16 | a
}

/// The configuration a sub-assembly's instance tree was recorded for (its
/// most recent one), when it says.
fn recorded_configuration(bytes: &[u8]) -> Result<Option<u32>, String> {
    let xml = instance_tree_xml(bytes)?;
    Ok(elements(&xml, "swConfiguration")
        .into_iter()
        .find(|c| c.get("swMostRecentConfiguration").as_deref() == Some("YES"))
        .and_then(|c| c.get("swID"))
        .and_then(|id| id.parse().ok()))
}

/// A SolidWorks part's body (its first configuration), as the Parasolid
/// import reads it. The other configurations are named in the error when
/// there is more than one, so a caller can say which was taken.
pub fn import_solidworks_part_report(bytes: &[u8]) -> Result<ParasolidImportReport, String> {
    let partitions = solidworks_part_partitions(bytes).map_err(|e| format!("solidworks_import: {e}"))?;
    crate::parasolid::import_parasolid_bytes_report(&partitions[0].1).map_err(|e| format!("solidworks_import: {}: {e}", partitions[0].0))
}

/// One instance an assembly places.
#[derive(Clone, Debug, PartialEq)]
pub struct SwComponent {
    /// The instance's name in the assembly.
    pub name: String,
    /// The file it uses, as the assembly stored it (often an absolute
    /// Windows path), and that file's base name.
    pub path: String,
    pub file_name: String,
    /// Whether the file is a sub-assembly.
    pub assembly: bool,
    /// The configuration the instance uses: its index (`Config-N`) and name.
    pub configuration: u32,
    pub configuration_name: String,
    /// Where the instance goes: row-major 4×4, mm.
    pub transform: crate::Mat4,
    pub suppressed: bool,
}

/// The instances a SolidWorks assembly places, from its instance tree.
pub fn solidworks_assembly_components(bytes: &[u8]) -> Result<Vec<SwComponent>, String> {
    parse_instance_tree(&instance_tree_xml(bytes)?)
}

/// The instance tree's XML.
fn instance_tree_xml(bytes: &[u8]) -> Result<String, String> {
    let streams = solidworks_streams(bytes, &|name: &str| name.starts_with("swXmlContents/COMPINSTANCETREE"))?;
    let (name, data) = streams.into_iter().next().ok_or("the assembly has no instance tree (swXmlContents/COMPINSTANCETREE)")?;
    let xml = if data.starts_with(b"<?xml") || data.starts_with(b"<sw") {
        data
    } else {
        // `__ZLB`: a 16-byte identifier, the uncompressed and compressed
        // lengths, then zlib.
        let u32_at = |at: usize| data.get(at..at + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).ok_or("the instance tree is cut short");
        let (full, packed) = (u32_at(16)?, u32_at(20)?);
        zlib(data.get(24..24 + packed).ok_or("the instance tree is cut short")?, full).map_err(|e| format!("{name}: {e}"))?
    };
    Ok(String::from_utf8_lossy(&xml).into_owned())
}

fn parse_instance_tree(xml: &str) -> Result<Vec<SwComponent>, String> {
    let files: Vec<Element> = elements(xml, "swFile");
    let models: Vec<Element> = elements(xml, "swModel");
    let mut out = Vec::new();
    for reference in elements(xml, "swReference") {
        let model = models
            .iter()
            .find(|m| m.get("id") == reference.get("swModelRef"))
            .ok_or_else(|| format!("instance {} names no model", reference.get("swName").unwrap_or_default()))?;
        let file = files
            .iter()
            .find(|f| f.get("id") == model.get("swFileRef"))
            .ok_or_else(|| format!("model {} names no file", model.get("swName").unwrap_or_default()))?;
        let path = file.get("swPath").unwrap_or_default();
        let numbers: Vec<f64> = reference
            .get("swTransform")
            .unwrap_or_default()
            .split_whitespace()
            .map(|t| t.parse::<f64>().map_err(|_| format!("a transform holds {t:?}")))
            .collect::<Result<_, _>>()?;
        if numbers.len() != 16 {
            return Err(format!("instance {} has a transform of {} numbers", reference.get("swName").unwrap_or_default(), numbers.len()));
        }
        let s = &numbers;
        if s[3] != 0.0 || s[7] != 0.0 || s[11] != 0.0 {
            return Err("an instance's transform has a perspective part".into());
        }
        let scale = if s[15] != 0.0 { 1.0 / s[15] } else { return Err("an instance's transform has a zero weight".into()) };
        let mm = 1000.0;
        // p' = p R + t (rows): the column form takes R transposed.
        let transform = [
            s[0], s[4], s[8], s[12] * mm * scale,
            s[1], s[5], s[9], s[13] * mm * scale,
            s[2], s[6], s[10], s[14] * mm * scale,
            0.0, 0.0, 0.0, 1.0,
        ];
        let file_name = path.rsplit(['\\', '/']).next().unwrap_or(&path).to_string();
        out.push(SwComponent {
            name: reference.get("swName").unwrap_or_default(),
            assembly: file.get("swDocType").as_deref() == Some("ASSEMBLY"),
            path,
            file_name,
            configuration: model.get("swConfigurationId").and_then(|c| c.parse().ok()).unwrap_or(0),
            configuration_name: model.get("swConfigurationName").unwrap_or_default(),
            transform,
            suppressed: reference.get("swSuppressed").as_deref() == Some("YES"),
        });
    }
    Ok(out)
}

/// An element's attributes, from its start tag.
struct Element(Vec<(String, String)>);

impl Element {
    fn get(&self, key: &str) -> Option<String> {
        self.0.iter().find(|(k, _)| k == key).map(|(_, v)| v.clone())
    }
}

/// The start tags `<tag ...>` of `xml`, attributes unescaped.
fn elements(xml: &str, tag: &str) -> Vec<Element> {
    let open = format!("<{tag} ");
    let mut out = Vec::new();
    let mut rest = xml;
    while let Some(at) = rest.find(&open) {
        let body = &rest[at + open.len()..];
        let end = body.find('>').unwrap_or(body.len());
        let attributes = &body[..end];
        let mut pairs = Vec::new();
        let mut a = attributes;
        while let Some(eq) = a.find("=\"") {
            let key = a[..eq].trim().to_string();
            let after = &a[eq + 2..];
            let close = after.find('"').unwrap_or(after.len());
            pairs.push((key, unescape(&after[..close])));
            a = &after[(close + 1).min(after.len())..];
        }
        out.push(Element(pairs));
        rest = &body[end..];
    }
    out
}

fn unescape(value: &str) -> String {
    value.replace("&lt;", "<").replace("&gt;", ">").replace("&quot;", "\"").replace("&apos;", "'").replace("&amp;", "&")
}

/// A SolidWorks assembly's bodies, each instance's part read through
/// `resolve` (given a file's base name, its bytes) and placed; sub-assemblies
/// recursively. Suppressed instances are left out.
pub fn import_solidworks_assembly_report(
    bytes: &[u8],
    resolve: &mut dyn FnMut(&str) -> Option<Vec<u8>>,
) -> Result<ParasolidImportReport, String> {
    let mut report: Option<ParasolidImportReport> = None;
    let mut parts: rustc_hash::FxHashMap<(String, u32), ParasolidImportReport> = rustc_hash::FxHashMap::default();
    place(bytes, crate::step_matrix::MAT4_IDENTITY, 0, resolve, &mut parts, &mut report)?;
    report.ok_or_else(|| "solidworks_import: the assembly places no part".into())
}

fn place(
    bytes: &[u8],
    parent: crate::Mat4,
    depth: usize,
    resolve: &mut dyn FnMut(&str) -> Option<Vec<u8>>,
    parts: &mut rustc_hash::FxHashMap<(String, u32), ParasolidImportReport>,
    report: &mut Option<ParasolidImportReport>,
) -> Result<(), String> {
    if depth > 32 {
        return Err("solidworks_import: assemblies nest more than 32 deep".into());
    }
    let components = solidworks_assembly_components(bytes).map_err(|e| format!("solidworks_import: {e}"))?;
    for component in components.iter().filter(|c| !c.suppressed) {
        let matrix = crate::step_matrix::mat4_mul(&parent, &component.transform);
        let file = resolve(&component.file_name)
            .ok_or_else(|| format!("solidworks_import: {} uses {}, which is not beside the assembly", component.name, component.file_name))?;
        if component.assembly {
            // A sub-assembly's file records the tree of one configuration;
            // an instance of another is not read.
            let recorded = recorded_configuration(&file).map_err(|e| format!("solidworks_import: {}: {e}", component.file_name))?;
            if recorded.is_some_and(|c| c != component.configuration) {
                return Err(format!(
                    "solidworks_import: {} uses configuration {:?} of sub-assembly {}, whose file records only configuration {}",
                    component.name,
                    component.configuration_name,
                    component.file_name,
                    recorded.unwrap_or(0)
                ));
            }
            place(&file, matrix, depth + 1, resolve, parts, report)?;
            continue;
        }
        let key = (component.file_name.to_ascii_lowercase(), component.configuration);
        if !parts.contains_key(&key) {
            let partitions = solidworks_part_partitions(&file).map_err(|e| format!("solidworks_import: {}: {e}", component.file_name))?;
            let wanted = format!("Config-{}", component.configuration);
            let partition = partitions.iter().find(|(c, _)| *c == wanted).map(|(_, p)| p).ok_or_else(|| {
                format!(
                    "solidworks_import: {} uses configuration {:?} ({wanted}) of {}, which holds only {}",
                    component.name,
                    component.configuration_name,
                    component.file_name,
                    partitions.iter().map(|(c, _)| c.as_str()).collect::<Vec<_>>().join(", ")
                )
            })?;
            let part = crate::parasolid::import_parasolid_bytes_report(partition).map_err(|e| format!("solidworks_import: {}: {e}", component.file_name))?;
            parts.insert(key.clone(), part);
        }
        let part = &parts[&key];
        let transform = crate::AffineTransform::new(matrix).map_err(|e| format!("solidworks_import: {}: {e}", component.name))?;
        let out = report.get_or_insert_with(|| ParasolidImportReport {
            solids: Vec::new(),
            names: Vec::new(),
            appearances: Vec::new(),
            application: Some("SolidWorks".into()),
            modeller_version: part.modeller_version.clone(),
            stated_resolution_mm: part.stated_resolution_mm,
            fitted_curves: 0,
            unverified_curves: 0,
            diagnostics: Default::default(),
        });
        for (k, solid) in part.solids.iter().enumerate() {
            let placed = crate::transform_brep(solid, transform, transform.determinant3() < 0.0).map_err(|e| format!("solidworks_import: {}: {e}", component.name))?;
            out.solids.push(placed);
            out.names.push(if part.solids.len() == 1 { component.name.clone() } else { format!("{} ({})", component.name, k + 1) });
            out.appearances.push(part.appearances.get(k).cloned().unwrap_or_default());
        }
        out.fitted_curves += part.fitted_curves;
        out.unverified_curves += part.unverified_curves;
    }
    Ok(())
}
