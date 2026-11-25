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
    import_configuration(bytes, &partitions[0].0, &partitions[0].1).map_err(|e| format!("solidworks_import: {}: {e}", partitions[0].0))
}

/// A configuration's bodies: its partition's, or, when the partition holds
/// none, the bodies its features keep for themselves
/// (`Config-N-FeatureBodies/LocalBodies`). A part made only of imported
/// bodies (an `Imported1` feature) is saved so: an empty partition beside
/// them.
fn import_configuration(bytes: &[u8], config: &str, partition: &[u8]) -> Result<ParasolidImportReport, String> {
    let first = crate::parasolid::import_parasolid_bytes_report(partition);
    match &first {
        Err(e) if e.ends_with(NO_BODIES) => {}
        _ => return first,
    }
    let stream = format!("{config}-FeatureBodies/LocalBodies");
    let streams = solidworks_streams(bytes, &|name: &str| name == stream)?;
    let Some((_, data)) = streams.first() else { return first };
    import_local_bodies(data).map_err(|e| format!("{stream}: {e}"))
}

/// Each body of a `LocalBodies` stream imported, in order, as one report.
pub(crate) fn import_local_bodies(data: &[u8]) -> Result<ParasolidImportReport, String> {
    let mut report: Option<ParasolidImportReport> = None;
    for (name, partition) in local_bodies(data)? {
        // A body kept empty adds nothing; the others still import (cross
        // review: one empty entry lost the bodies beside it).
        let mut part = match crate::parasolid::import_parasolid_bytes_report(&partition) {
            Err(e) if e.ends_with(NO_BODIES) => continue,
            read => read.map_err(|e| format!("{name}: {e}"))?,
        };
        // A body its file leaves unnamed takes the name its feature keeps it
        // by (cross review), numbered when the file holds more than one.
        let count = part.solids.len();
        part.names.resize(count, String::new());
        for (k, given) in part.names.iter_mut().enumerate() {
            if given.is_empty() {
                *given = if count == 1 { name.clone() } else { format!("{name} ({})", k + 1) };
            }
        }
        let Some(out) = report.as_mut() else {
            report = Some(part);
            continue;
        };
        out.solids.extend(part.solids);
        out.names.extend(part.names);
        out.appearances.extend(part.appearances);
        out.fitted_curves += part.fitted_curves;
        out.unverified_curves += part.unverified_curves;
        out.diagnostics.events.extend(part.diagnostics.events);
        for (code, n) in part.diagnostics.counters {
            *out.diagnostics.counters.entry(code).or_insert(0) += n;
        }
        for (code, value) in part.diagnostics.measurements {
            let at = out.diagnostics.measurements.entry(code).or_insert(value);
            *at = at.max(value);
        }
    }
    report.ok_or_else(|| NO_BODIES.into())
}

/// What the Parasolid import says of a partition with no body in it.
const NO_BODIES: &str = "the file holds no bodies";

/// The bodies a configuration's features keep (`LocalBodies`): a count,
/// then for each body its name (an MFC string: `FF FE FF`, a length in
/// UTF-16 units, the UTF-16 text), five bytes, the length of what follows
/// (a zero byte and a partition section), and that, a section as
/// [`first_section`] reads: a whole Parasolid transmit file.
pub(crate) fn local_bodies(data: &[u8]) -> Result<Vec<(String, Vec<u8>)>, String> {
    let u32_at = |at: usize| data.get(at..at + 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).ok_or("the stream is cut short");
    let count = u32_at(0)?;
    let mut at = 4;
    let mut out = Vec::new();
    for _ in 0..count {
        if data.get(at..at + 3) != Some(&[0xFF, 0xFE, 0xFF][..]) {
            return Err(format!("no body name at byte {at}"));
        }
        let mut units = usize::from(*data.get(at + 3).ok_or("the stream is cut short")?);
        at += 4;
        if units == 0xFF {
            units = data.get(at..at + 2).map(|b| usize::from(u16::from_le_bytes([b[0], b[1]]))).ok_or("the stream is cut short")?;
            at += 2;
        }
        let text: Vec<u16> = data
            .get(at..at + 2 * units)
            .ok_or("a body name runs past the stream")?
            .chunks(2)
            .map(|c| u16::from_le_bytes([c[0], c[1]]))
            .collect();
        let name = String::from_utf16_lossy(&text);
        at += 2 * units;
        let block = u32_at(at + 5)?;
        let section = data.get(at + 10..).ok_or("the stream is cut short")?;
        let size = section.get(..4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]) as usize).ok_or("the stream is cut short")?;
        if block != size + 5 || 4 + size > section.len() {
            return Err(format!("{name}: a block of {block} bytes around a section of {size}"));
        }
        out.push((name.clone(), first_section(&section[..4 + size]).map_err(|e| format!("{name}: {e}"))?));
        at += 10 + 4 + size;
    }
    Ok(out)
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

/// The part of the tree that holds the assembly's own instances. Each
/// model that places instances holds them inside its element, a
/// sub-assembly's as well as the assembly's; the assembly's own model is
/// the one no instance places. Read as the assembly's, a sub-assembly's
/// instances were placed with its frame for the assembly's, and again
/// through the sub-assembly. A tree whose instances stand outside every
/// model is read whole.
fn own_instances(xml: &str) -> &str {
    let placed: Vec<String> = elements(xml, "swReference").iter().filter_map(|r| r.get("swModelRef")).collect();
    let mut owners = Vec::new();
    let mut rest = 0;
    while let Some(at) = xml[rest..].find("<swModel ").map(|at| rest + at) {
        let Some(tag_end) = xml[at..].find('>').map(|end| at + end) else { break };
        rest = tag_end + 1;
        if xml[..tag_end].ends_with('/') {
            continue;
        }
        let Some(close) = xml[rest..].find("</swModel>").map(|end| rest + end) else { break };
        let body = &xml[rest..close];
        let id = elements(&xml[at..=tag_end], "swModel").into_iter().next().and_then(|m| m.get("id"));
        if body.contains("<swReference ") && id.is_some_and(|id| !placed.contains(&id)) {
            owners.push(body);
        }
        rest = close;
    }
    match owners.as_slice() {
        [own] => own,
        _ => xml,
    }
}

fn parse_instance_tree(xml: &str) -> Result<Vec<SwComponent>, String> {
    let files: Vec<Element> = elements(xml, "swFile");
    let models: Vec<Element> = elements(xml, "swModel");
    let mut out = Vec::new();
    for reference in elements(own_instances(xml), "swReference") {
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

/// The event an import report carries for a virtual part with no body.
pub const VIRTUAL_PART_WITHOUT_BODY: &str = "import.solidworks.virtual_part_without_body";

/// The diagnostic code for a component whose file was not found: its
/// instances are left out and the rest of the assembly is placed.
pub const MISSING_COMPONENT_FILE: &str = "import.solidworks.missing_component_file";

/// A SolidWorks assembly's bodies, each instance's part read through
/// `resolve` (given a file's base name, its bytes) and placed; sub-assemblies
/// recursively. Suppressed instances are left out.
///
/// A component whose file `resolve` does not find (a part or a
/// sub-assembly) is left out, as SolidWorks opens an assembly whose
/// references are missing; the report names each such file once
/// ([`MISSING_COMPONENT_FILE`]). An assembly none of whose components is
/// found is refused, naming every missing file.
///
/// A virtual component (an instance of a part kept inside the assembly that
/// made it, its file named `<part>^<assembly>`) is read from that assembly's
/// `VirtualComp/<part>` stream, a whole part file, as any part is; a virtual
/// part that holds no body contributes nothing, and the report names it
/// ([`VIRTUAL_PART_WITHOUT_BODY`]).
pub fn import_solidworks_assembly_report(
    bytes: &[u8],
    resolve: &mut dyn FnMut(&str) -> Option<Vec<u8>>,
) -> Result<ParasolidImportReport, String> {
    let mut report: Option<ParasolidImportReport> = None;
    let mut parts: rustc_hash::FxHashMap<(String, u32), ParasolidImportReport> = rustc_hash::FxHashMap::default();
    let mut empty = Vec::new();
    let mut missing = Missing::default();
    place(bytes, crate::step_matrix::MAT4_IDENTITY, 0, resolve, &mut parts, &mut report, &mut empty, &mut missing)?;
    let Some(mut report) = report else {
        return Err(if !missing.files.is_empty() {
            format!(
                "solidworks_import: none of the assembly's component files is beside it (missing: {}); put the parts and sub-assemblies it uses in its folder",
                missing.describe()
            )
        } else if empty.is_empty() {
            "solidworks_import: the assembly places no part".to_string()
        } else {
            format!("solidworks_import: the assembly places no part with a body (virtual parts without one: {})", empty.join(", "))
        });
    };
    if !missing.files.is_empty() {
        report.diagnostics.event(
            crate::DiagnosticSeverity::Warning,
            crate::KernelStage::Collect,
            MISSING_COMPONENT_FILE,
            format!(
                "solidworks_import: {} component file(s) not beside the assembly, their instances left out: {}",
                missing.files.len(),
                missing.describe()
            ),
        );
    }
    for name in empty {
        report.diagnostics.event(
            crate::DiagnosticSeverity::Warning,
            crate::KernelStage::Collect,
            VIRTUAL_PART_WITHOUT_BODY,
            format!("solidworks_import: virtual part {name} holds no body; it contributes nothing"),
        );
    }
    Ok(report)
}

/// The component files an assembly uses that were not found, each with the
/// number of instances (at any depth) left out for it, in the order met.
#[derive(Default)]
struct Missing {
    files: Vec<(String, usize)>,
}

impl Missing {
    fn add(&mut self, file_name: &str) {
        match self.files.iter_mut().find(|(f, _)| f.eq_ignore_ascii_case(file_name)) {
            Some((_, n)) => *n += 1,
            None => self.files.push((file_name.to_string(), 1)),
        }
    }

    fn describe(&self) -> String {
        self.files
            .iter()
            .map(|(f, n)| if *n == 1 { f.clone() } else { format!("{f} (x{n})") })
            .collect::<Vec<_>>()
            .join(", ")
    }
}

/// An assembly's own name, from its instance tree: the assembly model no
/// instance places (the tree's root). `None` when the tree does not say.
fn own_name(assembly: &[u8]) -> Result<Option<String>, String> {
    let xml = instance_tree_xml(assembly)?;
    let placed: Vec<String> = elements(&xml, "swReference").iter().filter_map(|r| r.get("swModelRef")).collect();
    let files = elements(&xml, "swFile");
    let roots: Vec<String> = elements(&xml, "swModel")
        .into_iter()
        .filter(|m| m.get("id").is_some_and(|id| !placed.contains(&id)))
        .filter(|m| files.iter().any(|f| f.get("id") == m.get("swFileRef") && f.get("swDocType").as_deref() == Some("ASSEMBLY")))
        .filter_map(|m| m.get("swName"))
        .collect();
    Ok(match roots.as_slice() {
        [one] => Some(one.clone()),
        _ => None,
    })
}

/// The part file of a virtual component kept in `assembly` (its file named
/// `<part>^<owner>`): the assembly's `VirtualComp/<part>` stream. `None` for
/// a component that is not virtual, or whose part the assembly does not
/// keep; an error when the assembly cannot be read for it (a damaged stream
/// is not replaced by a file of the same name, cross review).
fn virtual_part(assembly: &[u8], file_name: &str) -> Result<Option<Vec<u8>>, String> {
    let Some((part, owner)) = file_name.split_once('^') else { return Ok(None) };
    // Only the assembly that made it keeps it: another's part of the same
    // name is not this one (cross review).
    let (owner, extension) = owner.rsplit_once('.').unwrap_or((owner, ""));
    // A virtual component nested in a virtual sub-assembly names its owner
    // with the owner's own `^` written as `_`.
    let same = |a: &str, b: &str| a.replace('^', "_").eq_ignore_ascii_case(&b.replace('^', "_"));
    if !own_name(assembly)?.is_some_and(|name| same(&name, owner)) {
        return Ok(None);
    }
    // The OLE container keeps it as `VirtualComp/<part>`, the 2015+ archive
    // as `VirtualComp/<part>-<extension>` (`-sldprt`, `-sldasm`).
    let plain = format!("VirtualComp/{part}");
    let suffixed = format!("VirtualComp/{part}-{}", extension.to_ascii_lowercase());
    let mut found = solidworks_streams(assembly, &|name: &str| name == plain || name.eq_ignore_ascii_case(&suffixed))?;
    found.sort_by_key(|(name, _)| name != &plain);
    Ok(found.into_iter().next().map(|(_, data)| data))
}

#[allow(clippy::too_many_arguments)]
fn place(
    bytes: &[u8],
    parent: crate::Mat4,
    depth: usize,
    resolve: &mut dyn FnMut(&str) -> Option<Vec<u8>>,
    parts: &mut rustc_hash::FxHashMap<(String, u32), ParasolidImportReport>,
    report: &mut Option<ParasolidImportReport>,
    empty: &mut Vec<String>,
    missing: &mut Missing,
) -> Result<(), String> {
    if depth > 32 {
        return Err("solidworks_import: assemblies nest more than 32 deep".into());
    }
    let components = solidworks_assembly_components(bytes).map_err(|e| format!("solidworks_import: {e}"))?;
    for component in components.iter().filter(|c| !c.suppressed) {
        let matrix = crate::step_matrix::mat4_mul(&parent, &component.transform);
        let kept = virtual_part(bytes, &component.file_name).map_err(|e| format!("solidworks_import: {}: {e}", component.file_name))?;
        let is_virtual = kept.is_some();
        let Some(file) = kept.or_else(|| resolve(&component.file_name)) else {
            missing.add(&component.file_name);
            continue;
        };
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
            place(&file, matrix, depth + 1, resolve, parts, report, empty, missing)?;
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
            // Empty when neither its partition nor any body its features
            // keep holds one (any other error refuses it).
            let part = match import_configuration(&file, &wanted, partition) {
                Err(e) if is_virtual && e.ends_with(NO_BODIES) => {
                    if !empty.contains(&component.file_name) {
                        empty.push(component.file_name.clone());
                    }
                    continue;
                }
                read => read.map_err(|e| format!("solidworks_import: {}: {e}", component.file_name))?,
            };
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
