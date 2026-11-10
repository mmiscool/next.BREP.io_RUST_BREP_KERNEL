//! IGES product structure and presentation: **308** subfigure definitions
//! placed by **408** singular subfigure instances, body names (a **406** form 15
//! name property, else the directory-entry label), colours (a **314** colour
//! definition, else the directory-entry colour number) and levels.
//!
//! Placements are row-major 4x4 affine matrices with their translations in
//! millimetres, the convention `StepOccurrence::placement` uses, so the
//! structured read hands the app the same thing a STEP assembly does.

use crate::ImportedColor;

use super::reader::{IgesFile, ParsedEntity};

/// Row-major affine 4x4; translations in millimetres.
pub(super) type Matrix = [f64; 16];

pub(super) const IDENTITY: Matrix = [
    1.0, 0.0, 0.0, 0.0, //
    0.0, 1.0, 0.0, 0.0, //
    0.0, 0.0, 1.0, 0.0, //
    0.0, 0.0, 0.0, 1.0,
];

/// Nesting deeper than this is a cycle or a broken file.
const MAX_DEPTH: usize = 64;

/// `a` applied after `b`.
pub(super) fn compose(a: &Matrix, b: &Matrix) -> Matrix {
    let mut out = [0.0; 16];
    for row in 0..4 {
        for column in 0..4 {
            out[row * 4 + column] = (0..4).map(|k| a[row * 4 + k] * b[k * 4 + column]).sum();
        }
    }
    out
}

/// The 124 at `pointer`, composed with any 124 it is itself under, with its
/// translation converted to millimetres by `scale`.
fn transform_matrix(file: &IgesFile, pointer: i64, scale: f64, depth: usize) -> Result<Matrix, String> {
    if pointer == 0 {
        return Ok(IDENTITY);
    }
    if depth > MAX_DEPTH {
        return Err(format!("iges_import: the transform chain at DE {pointer} does not end"));
    }
    let entity = file.entity(pointer)?;
    if entity.de.entity_type != 124 {
        return Err(format!(
            "iges_import: DE {pointer} is used as a transform but is entity {}",
            entity.de.entity_type
        ));
    }
    let mut own = IDENTITY;
    for row in 0..3 {
        for column in 0..4 {
            let value = entity.real(row * 4 + column)?;
            own[row * 4 + column] = if column == 3 { value * scale } else { value };
        }
    }
    Ok(compose(&transform_matrix(file, entity.de.transform, scale, depth + 1)?, &own))
}

/// Where a 408 puts its subfigure: scaled by S, moved by (X, Y, Z), then
/// placed by the instance's own transform.
pub(super) fn instance_matrix(file: &IgesFile, instance: &ParsedEntity, scale: f64) -> Result<Matrix, String> {
    let factor = match instance.params.get(4).map(|token| token.trim()) {
        None | Some("") => 1.0,
        Some(_) => instance.real(4)?,
    };
    let mut own = IDENTITY;
    for axis in 0..3 {
        own[axis * 4 + axis] = factor;
        own[axis * 4 + 3] = match instance.params.get(1 + axis).map(|token| token.trim()) {
            None | Some("") => 0.0,
            Some(_) => instance.real(1 + axis)? * scale,
        };
    }
    Ok(compose(&transform_matrix(file, instance.de.transform, scale, 0)?, &own))
}

/// The DE pointers of the property entities an entity lists after its
/// `regular` parameters (the associativity count and list, then the property
/// count and list).
fn properties(entity: &ParsedEntity, regular: usize) -> Vec<i64> {
    let count = |index: usize| -> usize {
        entity
            .params
            .get(index)
            .and_then(|token| token.trim().parse::<i64>().ok())
            .filter(|&count| count > 0)
            .map_or(0, |count| count as usize)
    };
    let associativities = count(regular);
    let properties_at = regular + 1 + associativities;
    let listed = count(properties_at);
    (0..listed)
        .filter_map(|index| entity.params.get(properties_at + 1 + index)?.trim().parse::<i64>().ok())
        .collect()
}

/// How many parameters an entity type has before its optional associativity
/// and property lists.
fn regular_parameters(entity: &ParsedEntity) -> Option<usize> {
    let int = |index: usize| entity.int(index).ok().filter(|&value| value >= 0).map(|value| value as usize);
    match entity.de.entity_type {
        186 => Some(3 + 2 * int(2)?),
        308 => Some(3 + int(2)?),
        408 => Some(5),
        _ => None,
    }
}

/// An entity's name: its 406 form 15 name property, else (for a 308) the
/// subfigure's own name, else its directory-entry label.
pub(super) fn entity_name(file: &IgesFile, entity: &ParsedEntity) -> Option<String> {
    if let Some(regular) = regular_parameters(entity) {
        for pointer in properties(entity, regular) {
            let Ok(property) = file.entity(pointer) else { continue };
            if property.de.entity_type == 406 && property.de.form == 15 {
                if let Some(name) = property.params.get(1).map(|name| name.trim()).filter(|name| !name.is_empty()) {
                    return Some(name.to_string());
                }
            }
        }
    }
    if entity.de.entity_type == 308 {
        if let Some(name) = entity.params.get(1).map(|name| name.trim()).filter(|name| !name.is_empty()) {
            return Some(name.to_string());
        }
    }
    (!entity.de.label.is_empty()).then(|| entity.de.label.clone())
}

/// An entity's colour: a 314 its directory entry points to, or one of the
/// eight colour numbers IGES defines. `None` for no colour (0) or anything
/// unreadable; a colour never fails an import.
pub(super) fn entity_color(file: &IgesFile, entity: &ParsedEntity) -> Option<ImportedColor> {
    let number = entity.de.color;
    if number < 0 {
        let definition = file.entity(-number).ok()?;
        if definition.de.entity_type != 314 {
            return None;
        }
        let percent = |index: usize| definition.real(index).ok().map(|value| value / 100.0);
        return Some(ImportedColor::new(percent(0)?, percent(1)?, percent(2)?));
    }
    let [r, g, b] = match number {
        1 => [0.0, 0.0, 0.0],
        2 => [1.0, 0.0, 0.0],
        3 => [0.0, 1.0, 0.0],
        4 => [0.0, 0.0, 1.0],
        5 => [1.0, 1.0, 0.0],
        6 => [1.0, 0.0, 1.0],
        7 => [0.0, 1.0, 1.0],
        8 => [1.0, 1.0, 1.0],
        _ => return None,
    };
    Some(ImportedColor::new(r, g, b))
}

/// The members a 308 lists.
pub(super) fn subfigure_members(entity: &ParsedEntity) -> Result<Vec<i64>, String> {
    let count = entity.int(2)?.max(0) as usize;
    (0..count).map(|index| entity.int(3 + index)).collect()
}

/// Every DE pointer some 308 lists: definitions that only appear through an
/// instance, never on their own.
pub(super) fn subfigure_member_set(file: &IgesFile) -> Result<rustc_hash::FxHashSet<i64>, String> {
    let mut members = rustc_hash::FxHashSet::default();
    for (_, subfigure) in file.pointers_of_type(308) {
        members.extend(subfigure_members(subfigure)?);
    }
    Ok(members)
}

/// The 308 an instance places, refusing what this import cannot read.
pub(super) fn instanced_subfigure<'f>(file: &'f IgesFile, instance_pointer: i64) -> Result<(i64, &'f ParsedEntity), String> {
    let instance = file.entity(instance_pointer)?;
    let pointer = instance.int(0)?;
    let definition = file.entity(pointer)?;
    match definition.de.entity_type {
        308 => Ok((pointer, definition)),
        416 => Err(format!(
            "iges_import: instance DE {instance_pointer} places a part from another file (416 '{}'), which this import cannot read",
            definition.params.first().map(|name| name.trim()).unwrap_or("")
        )),
        other => Err(format!(
            "iges_import: instance DE {instance_pointer} places entity {other}, not a subfigure definition (308)"
        )),
    }
}

/// One solid occurrence of the flattened structure: a 186 and where it goes.
pub(super) struct Placed {
    pub solid: i64,
    pub placement: Matrix,
    pub name: Option<String>,
    pub color: Option<ImportedColor>,
    pub level: i64,
}

/// Every 186 the file shows, with its placement: the ones standing on their
/// own, then each instance's in file order, nested instances composed.
pub(super) fn placed_solids(file: &IgesFile, scale: f64) -> Result<Vec<Placed>, String> {
    let members = subfigure_member_set(file)?;
    let mut placed = Vec::new();
    for (pointer, solid) in file.pointers_of_type(186) {
        if !members.contains(&pointer) {
            placed.push(Placed {
                solid: pointer,
                placement: IDENTITY,
                name: entity_name(file, solid),
                color: entity_color(file, solid),
                level: solid.de.level,
            });
        }
    }
    for (pointer, _) in file.pointers_of_type(408) {
        if !members.contains(&pointer) {
            expand(file, scale, pointer, &IDENTITY, None, None, 0, &mut placed)?;
        }
    }
    Ok(placed)
}

#[allow(clippy::too_many_arguments)]
fn expand(
    file: &IgesFile,
    scale: f64,
    instance_pointer: i64,
    parent: &Matrix,
    inherited_name: Option<&str>,
    inherited_color: Option<ImportedColor>,
    depth: usize,
    placed: &mut Vec<Placed>,
) -> Result<(), String> {
    if depth > MAX_DEPTH {
        return Err(format!("iges_import: instance DE {instance_pointer} nests deeper than {MAX_DEPTH} levels"));
    }
    let instance = file.entity(instance_pointer)?;
    let (subfigure_pointer, subfigure) = instanced_subfigure(file, instance_pointer)?;
    let placement = compose(parent, &instance_matrix(file, instance, scale)?);
    let name = entity_name(file, instance).or_else(|| inherited_name.map(str::to_string));
    let color = entity_color(file, instance).or(inherited_color);
    for member in subfigure_members(subfigure)? {
        let entity = file.entity(member)?;
        match entity.de.entity_type {
            186 => placed.push(Placed {
                solid: member,
                placement,
                name: entity_name(file, entity).or_else(|| name.clone()),
                color: entity_color(file, entity).or(color),
                level: entity.de.level,
            }),
            408 => expand(file, scale, member, &placement, name.as_deref(), color, depth + 1, placed)?,
            514 | 144 | 143 | 128 => {
                return Err(format!(
                    "iges_import: subfigure DE {subfigure_pointer} holds entity {} outside a solid, which this import cannot build yet; importing its solids alone would leave it out",
                    entity.de.entity_type
                ))
            }
            // Curves, points, annotation: not part of a solid model.
            _ => {}
        }
    }
    Ok(())
}

/// Whether a placement is the identity, exactly.
pub(super) fn is_identity(matrix: &Matrix) -> bool {
    matrix == &IDENTITY
}
