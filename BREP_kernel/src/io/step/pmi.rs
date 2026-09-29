//! AP242 PMI in the STEP writer: SEMANTIC representation (dimensions with
//! their values and tolerances, datums, geometric tolerances with datum
//! systems, modifiers and zones — typed from the kernel's PMI store, never
//! from display strings), POLYLINE PRESENTATION (the same layout the
//! viewport draws, from [`crate::feature_pipeline::pmi::layout`]) and SAVED
//! VIEWS (one `DRAUGHTING_MODEL` + `CAMERA_MODEL_D3` per PMI view, related
//! to the global draughting model), laid out per the CAx-IF "Recommended
//! Practices for the Representation and Presentation of PMI (AP242)" v4.0:
//!
//! - every referenced face / edge / vertex gets ONE `SHAPE_ASPECT` +
//!   `GEOMETRIC_ITEM_SPECIFIC_USAGE` (shared by every annotation on it) with
//!   an `ID_ATTRIBUTE`;
//! - `DIMENSIONAL_LOCATION('linear distance')` / `ANGULAR_LOCATION` between
//!   two aspects, `DIMENSIONAL_SIZE('diameter' | 'radius' | 'spherical …' |
//!   'curve length')` on one; values through
//!   `DIMENSIONAL_CHARACTERISTIC_REPRESENTATION` →
//!   `SHAPE_DIMENSION_REPRESENTATION` with 'nominal value' (and 'upper limit'
//!   / 'lower limit' for a limits block) measure items, and a
//!   `PLUS_MINUS_TOLERANCE` / `TOLERANCE_VALUE` pair for ± / deviation blocks;
//! - `DATUM` + `DATUM_FEATURE` + `SHAPE_ASPECT_RELATIONSHIP`;
//! - the fourteen `*_TOLERANCE` entities, complex with
//!   `GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE` (a `DATUM_SYSTEM` of
//!   `DATUM_REFERENCE_COMPARTMENT`s) and `GEOMETRIC_TOLERANCE_WITH_MODIFIERS`,
//!   plus a `TOLERANCE_ZONE` of form 'cylindrical or circular' for a ⌀ zone;
//! - `DRAUGHTING_CALLOUT` → `ANNOTATION_CURVE_OCCURRENCE` →
//!   `GEOMETRIC_CURVE_SET` of `POLYLINE`s plus an
//!   `ANNOTATION_TEXT_OCCURRENCE` → `TEXT_LITERAL`, on the view's
//!   `ANNOTATION_PLANE`; `DRAUGHTING_MODEL_ITEM_ASSOCIATION('PMI
//!   representation to presentation link')` ties each semantic element to
//!   its callout through the global draughting model;
//! - a saved view = `DRAUGHTING_MODEL(name, (styled MAPPED_ITEM of the
//!   shape, CAMERA_MODEL_D3, its callouts))` related to the global model by
//!   `MECHANICAL_DESIGN_AND_DRAUGHTING_RELATIONSHIP(view, global)`; the
//!   camera is a `VIEW_VOLUME` (.PARALLEL. / .CENTRAL.) with the view
//!   reference system at the eye, z along the viewing direction, the view
//!   plane at the target and a `PLANAR_BOX` window centred on it.
//!
//! Non-ASCII text (⌀ ± ° and the GD&T symbols) is written with Part 21's
//! `\X2\…\X0\` control directives, per the CAx-IF Unicode recommendation.

use super::styles::{write_colour_rgb, write_style_assignment, write_styled_item};
use super::{
    id_list, real, write_direction, write_placement, write_point, StepItemOwner, StepWriter,
};
use crate::feature_pipeline::pmi::annotations::fcf::{characteristic, datum_references};
use crate::feature_pipeline::pmi::layout::{present, LayoutStyle};
use crate::feature_pipeline::pmi::{
    PmiAnnotation, PmiGeometry, PmiProjection, PmiReport, PmiState, PmiStatus,
    ToleranceBlock, ToleranceMode,
};
use crate::feature_pipeline::Env;
use crate::Vec3;
use rustc_hash::FxHashMap as HashMap;

/// The PMI to write: the document block and the tail's resolution of it.
pub struct StepPmi<'a> {
    pub state: &'a PmiState,
    pub report: &'a PmiReport,
}

/// What the geometry pass left for the PMI writer.
pub(crate) struct StepContext<'a> {
    pub product_shape: usize,
    /// The `ADVANCED_BREP_SHAPE_REPRESENTATION`.
    pub representation: usize,
    pub geometry_context: usize,
    pub length_unit: usize,
    pub angle_unit: usize,
    /// Face name → its `ADVANCED_FACE` and the product that OWNS it. In a
    /// structured export a component's face lives in the PART's product, so the
    /// aspect it carries must be attached there, not to the root document.
    pub faces: &'a HashMap<String, (usize, StepItemOwner)>,
    pub edges: &'a HashMap<String, (usize, StepItemOwner)>,
    /// Solid name → its owner and its `VERTEX_POINT`s (ROOT-space point, entity
    /// id) — root space because that is the frame a `{body}@x,y,z` reference is
    /// written in, whatever product the vertex ended up in.
    pub vertices: &'a HashMap<String, (StepItemOwner, Vec<(Vec3, usize)>)>,
}

/// Millimetres per point (label text sizes are in points).
const MM_PER_POINT: f64 = 25.4 / 72.0;
/// Presentation line width (model units).
const LINE_WIDTH: f64 = 0.13;

/// A Part 21 string: quotes doubled, non-ASCII in `\X2\…\X0\` (UTF-16 code
/// units, the encoding every AP242 consumer reads), newlines as ` / `.
pub(crate) fn step_text(value: &str) -> String {
    let mut out = String::new();
    let mut run: Vec<u16> = Vec::new();
    let flush = |run: &mut Vec<u16>, out: &mut String| {
        if !run.is_empty() {
            out.push_str("\\X2\\");
            for unit in run.iter() {
                out.push_str(&format!("{unit:04X}"));
            }
            out.push_str("\\X0\\");
            run.clear();
        }
    };
    for ch in value.replace('\n', " / ").chars() {
        if ch.is_ascii() && !ch.is_ascii_control() {
            flush(&mut run, &mut out);
            if ch == '\'' {
                out.push_str("''");
            } else if ch == '\\' {
                out.push_str("\\\\");
            } else {
                out.push(ch);
            }
        } else if !ch.is_ascii() {
            let mut units = [0u16; 2];
            for unit in ch.encode_utf16(&mut units) {
                run.push(*unit);
            }
        }
    }
    flush(&mut run, &mut out);
    out
}

/// The referenced geometry an aspect attaches to: the entity, and the product
/// whose shape it defines.
#[derive(Clone, Copy)]
struct Item {
    entity: usize,
    owner: StepItemOwner,
}

struct Emitter<'a, 'b> {
    writer: &'a mut StepWriter,
    context: &'a StepContext<'b>,
    /// Reference name → its shape aspect (a `SHAPE_ASPECT` or, for a datum
    /// feature, the `DATUM_FEATURE`), shared by every annotation on it.
    aspects: HashMap<String, usize>,
    /// Datum letter → `DATUM` entity.
    datums: HashMap<String, usize>,
    /// Reference names that resolved to no entity in this file. Counted, never
    /// dropped in silence — an annotation whose geometry moved out from under
    /// it (or was deleted) is a fact the export report has to carry.
    unresolved: std::collections::BTreeSet<String>,
    null_style: usize,
    curve_style: usize,
}

impl Emitter<'_, '_> {
    fn add(&mut self, body: impl Into<String>) -> usize {
        self.writer.add(body)
    }

    /// Resolve a PMI reference name to the geometry entity it names, counting
    /// the miss when it names none.
    fn item(&mut self, name: &str) -> Option<Item> {
        let found = self.lookup(name);
        if found.is_none() {
            self.unresolved.insert(name.to_string());
        }
        found
    }

    /// [`Self::item`] without the bookkeeping.
    fn lookup(&self, name: &str) -> Option<Item> {
        if let Some((solid, coords)) = name.split_once('@') {
            let mut parts = coords.split(',').map(|part| part.trim().parse::<f64>().ok());
            let (x, y, z) = (parts.next()??, parts.next()??, parts.next()??);
            let query = Vec3::new(x, y, z);
            let (owner, points) = self.context.vertices.get(solid)?;
            let mut best: Option<(f64, usize)> = None;
            for (point, id) in points {
                let distance = point.sub(query).length();
                if best.map(|(d, _)| distance < d).unwrap_or(true) {
                    best = Some((distance, *id));
                }
            }
            return best
                .filter(|(d, _)| *d < 1e-6 + 1e-9 * query.length())
                .map(|(_, entity)| Item {
                    entity,
                    owner: *owner,
                });
        }
        if let Some((entity, owner)) = self.context.faces.get(name) {
            return Some(Item {
                entity: *entity,
                owner: *owner,
            });
        }
        if let Some((entity, owner)) = self.context.edges.get(name) {
            return Some(Item {
                entity: *entity,
                owner: *owner,
            });
        }
        None
    }

    /// The shape aspect for `name` (created once, with its GISU + id).
    fn aspect(&mut self, name: &str) -> Option<usize> {
        if let Some(id) = self.aspects.get(name) {
            return Some(*id);
        }
        let item = self.item(name)?;
        let aspect = self.add(format!(
            "SHAPE_ASPECT('{}','',#{},.T.)",
            step_text(name),
            item.owner.product_shape
        ));
        self.link_aspect(aspect, item, "");
        self.add(format!("ID_ATTRIBUTE('{}',#{aspect})", step_text(name)));
        self.aspects.insert(name.to_string(), aspect);
        Some(aspect)
    }

    fn link_aspect(&mut self, aspect: usize, item: Item, description: &str) {
        let target = item.entity;
        self.add(format!(
            "GEOMETRIC_ITEM_SPECIFIC_USAGE('','{}',#{aspect},#{},#{target})",
            step_text(description),
            item.owner.representation
        ));
    }

    fn length_measure(&mut self, value: f64) -> Result<usize, String> {
        Ok(self.add(format!(
            "LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{})",
            real(value)?,
            self.context.length_unit
        )))
    }

    fn angle_measure(&mut self, degrees: f64) -> Result<usize, String> {
        Ok(self.add(format!(
            "PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE({}),#{})",
            real(degrees.to_radians())?,
            self.context.angle_unit
        )))
    }

    /// A named measure representation item (`'nominal value'` …).
    fn measure_item(&mut self, name: &str, value: f64, angle: bool) -> Result<usize, String> {
        Ok(if angle {
            self.add(format!(
                "(MEASURE_REPRESENTATION_ITEM()MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE({}),#{})PLANE_ANGLE_MEASURE_WITH_UNIT()REPRESENTATION_ITEM('{name}'))",
                real(value.to_radians())?,
                self.context.angle_unit
            ))
        } else {
            self.add(format!(
                "(LENGTH_MEASURE_WITH_UNIT()MEASURE_REPRESENTATION_ITEM()MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{})REPRESENTATION_ITEM('{name}'))",
                real(value)?,
                self.context.length_unit
            ))
        })
    }

    /// The value + tolerance block of a dimension entity `dimension`.
    fn dimension_values(
        &mut self,
        dimension: usize,
        value: f64,
        angle: bool,
        tolerance: &ToleranceBlock,
        is_reference: bool,
        extra_items: &[usize],
    ) -> Result<(), String> {
        let mut items = vec![self.measure_item("nominal value", value, angle)?];
        let toleranced = !is_reference;
        if toleranced && tolerance.mode == ToleranceMode::Limits {
            items.push(self.measure_item("upper limit", value + tolerance.upper, angle)?);
            items.push(self.measure_item("lower limit", value - tolerance.lower, angle)?);
        }
        items.extend_from_slice(extra_items);
        let representation = self.add(format!(
            "SHAPE_DIMENSION_REPRESENTATION('',{},#{})",
            id_list(&items),
            self.context.geometry_context
        ));
        self.add(format!(
            "DIMENSIONAL_CHARACTERISTIC_REPRESENTATION(#{dimension},#{representation})"
        ));
        if toleranced {
            if let (ToleranceMode::Symmetric | ToleranceMode::Deviation, Some((lower, upper))) =
                (tolerance.mode, tolerance.bounds())
            {
                let (lower, upper) = if angle {
                    (self.angle_measure(lower)?, self.angle_measure(upper)?)
                } else {
                    (self.length_measure(lower)?, self.length_measure(upper)?)
                };
                let range = self.add(format!("TOLERANCE_VALUE(#{lower},#{upper})"));
                self.add(format!("PLUS_MINUS_TOLERANCE(#{range},#{dimension})"));
            }
        }
        Ok(())
    }

    /// The semantic representation element of one annotation (`None` for
    /// presentation-only kinds, or when a reference has no exported geometry).
    fn semantic(
        &mut self,
        annotation: &PmiAnnotation,
        report: &crate::feature_pipeline::pmi::PmiAnnotationReport,
        env: &Env,
    ) -> Result<Option<usize>, String> {
        let id = report.id.as_str();
        let tolerance = ToleranceBlock::read(annotation, env).unwrap_or(ToleranceBlock {
            mode: ToleranceMode::None,
            upper: 0.0,
            lower: 0.0,
        });
        let is_reference = annotation.flag("isReference");
        let value = report.value.unwrap_or(0.0);
        match &report.geometry {
            PmiGeometry::Linear { a, b, component } => {
                let aspects: Vec<usize> = report
                    .references
                    .iter()
                    .filter_map(|name| self.aspect(name))
                    .collect();
                let dimension = match aspects.as_slice() {
                    [single] if report.references.len() == 1 => {
                        self.add(format!("DIMENSIONAL_SIZE(#{single},'curve length')"))
                    }
                    [first, second] => self.add(format!(
                        "DIMENSIONAL_LOCATION('linear distance','',#{first},#{second})"
                    )),
                    _ => return Ok(None),
                };
                self.add(format!("ID_ATTRIBUTE('{}',#{dimension})", step_text(id)));
                // An aligned component is an ORIENTED location: its axis is the
                // x direction of an 'orientation' placement in the items.
                let mut extra = Vec::new();
                if let Some(axis) = component {
                    let x = match axis {
                        'X' => Vec3::new(1.0, 0.0, 0.0),
                        'Y' => Vec3::new(0.0, 1.0, 0.0),
                        _ => Vec3::new(0.0, 0.0, 1.0),
                    };
                    let z = x.perpendicular().unwrap_or(Vec3::new(0.0, 0.0, 1.0));
                    let origin = write_point(self.writer, Vec3::new(a[0], a[1], a[2]))?;
                    let z_dir = write_direction(self.writer, z)?;
                    let x_dir = write_direction(self.writer, x)?;
                    extra.push(self.add(format!(
                        "AXIS2_PLACEMENT_3D('orientation',#{origin},#{z_dir},#{x_dir})"
                    )));
                }
                let _ = b;
                self.dimension_values(dimension, value, false, &tolerance, is_reference, &extra)?;
                Ok(Some(dimension))
            }
            PmiGeometry::Radial { diameter, sphere, .. } => {
                let Some(aspect) = report.references.first().and_then(|name| self.aspect(name)) else {
                    return Ok(None);
                };
                let name = match (diameter, sphere) {
                    (true, false) => "diameter",
                    (false, false) => "radius",
                    (true, true) => "spherical diameter",
                    (false, true) => "spherical radius",
                };
                let dimension = self.add(format!("DIMENSIONAL_SIZE(#{aspect},'{name}')"));
                self.add(format!("ID_ATTRIBUTE('{}',#{dimension})", step_text(id)));
                self.dimension_values(dimension, value, false, &tolerance, is_reference, &[])?;
                Ok(Some(dimension))
            }
            PmiGeometry::Angular { degrees, .. } => {
                let aspects: Vec<usize> = report
                    .references
                    .iter()
                    .filter_map(|name| self.aspect(name))
                    .collect();
                let [first, second] = aspects.as_slice() else {
                    return Ok(None);
                };
                let selection = if *degrees > 180.0 { ".LARGE." } else { ".EQUAL." };
                let dimension = self.add(format!(
                    "ANGULAR_LOCATION('angular location','',#{first},#{second},{selection})"
                ));
                self.add(format!("ID_ATTRIBUTE('{}',#{dimension})", step_text(id)));
                self.dimension_values(dimension, *degrees, true, &tolerance, is_reference, &[])?;
                Ok(Some(dimension))
            }
            PmiGeometry::Hole { .. } => {
                let Some(aspect) = report.references.first().and_then(|name| self.aspect(name)) else {
                    return Ok(None);
                };
                let dimension = self.add(format!("DIMENSIONAL_SIZE(#{aspect},'diameter')"));
                self.add(format!("ID_ATTRIBUTE('{}',#{dimension})", step_text(id)));
                let callout = self.add(format!(
                    "DESCRIPTIVE_REPRESENTATION_ITEM('hole callout','{}')",
                    step_text(&report.text)
                ));
                let none = ToleranceBlock {
                    mode: ToleranceMode::None,
                    upper: 0.0,
                    lower: 0.0,
                };
                self.dimension_values(dimension, value, false, &none, false, &[callout])?;
                Ok(Some(dimension))
            }
            PmiGeometry::Datum { letter, .. } => {
                // Written up front by `write_datums`; the DMIA definition is
                // the datum feature aspect.
                let _ = letter;
                Ok(report.references.first().and_then(|name| self.aspects.get(name).copied()))
            }
            PmiGeometry::Fcf { frame, .. } => {
                let Some(aspect) = report.references.first().and_then(|name| self.aspect(name)) else {
                    return Ok(None);
                };
                let Some(kind) = characteristic(&frame.characteristic) else {
                    return Ok(None);
                };
                let magnitude = self.length_measure(value)?;
                // Datum system.
                let datums = datum_references(annotation);
                let mut compartments = Vec::new();
                for (letter, modifier) in &datums {
                    let Some(datum) = self.datums.get(letter).copied() else {
                        return Ok(None);
                    };
                    let modifiers = match material_condition(modifier) {
                        Some(condition) => format!("(SIMPLE_DATUM_REFERENCE_MODIFIER({condition}))"),
                        None => "$".into(),
                    };
                    compartments.push(self.add(format!(
                        "DATUM_REFERENCE_COMPARTMENT('','',#{},.F.,#{datum},{modifiers})",
                        self.context.product_shape
                    )));
                }
                let system = if compartments.is_empty() {
                    None
                } else {
                    Some(self.add(format!(
                        "DATUM_SYSTEM('','',#{},.F.,{})",
                        self.context.product_shape,
                        id_list(&compartments)
                    )))
                };
                let condition = material_condition(annotation.text("materialCondition"));
                let base = format!(
                    "GEOMETRIC_TOLERANCE('{}','',#{magnitude},#{aspect})",
                    step_text(id)
                );
                let tolerance = if system.is_none() && condition.is_none() {
                    self.add(format!(
                        "{}('{}','',#{magnitude},#{aspect})",
                        kind.step_entity,
                        step_text(id)
                    ))
                } else {
                    // Complex instance: types in alphabetical order.
                    let mut parts: Vec<String> = vec![base];
                    if let Some(system) = system {
                        parts.push(format!("GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE((#{system}))"));
                    }
                    if let Some(condition) = condition {
                        parts.push(format!("GEOMETRIC_TOLERANCE_WITH_MODIFIERS(({condition}))"));
                    }
                    parts.push(format!("{}()", kind.step_entity));
                    parts.sort();
                    self.add(format!("({})", parts.join("")))
                };
                if annotation.flag("zoneDiameter") {
                    let form = self.add("TOLERANCE_ZONE_FORM('cylindrical or circular')");
                    self.add(format!(
                        "TOLERANCE_ZONE('','',#{},.F.,(#{tolerance}),#{form})",
                        self.context.product_shape
                    ));
                }
                Ok(Some(tolerance))
            }
            PmiGeometry::None
            | PmiGeometry::Leader { .. }
            | PmiGeometry::Note { .. }
            | PmiGeometry::Explode { .. } => Ok(None),
        }
    }

    /// The datums of every view, up front (frames reference them).
    fn write_datums(&mut self, pmi: &StepPmi<'_>) {
        for view in &pmi.report.views {
            for report in &view.annotations {
                let PmiGeometry::Datum { letter, .. } = &report.geometry else {
                    continue;
                };
                if report.status != PmiStatus::Ok || !report.enabled || self.datums.contains_key(letter) {
                    continue;
                }
                let Some(name) = report.references.first() else {
                    continue;
                };
                let Some(item) = self.item(name) else {
                    continue;
                };
                let datum = self.add(format!(
                    "DATUM('{}','',#{},.F.,'{}')",
                    step_text(&report.id),
                    item.owner.product_shape,
                    step_text(letter)
                ));
                let feature = self.add(format!(
                    "DATUM_FEATURE('{}','',#{},.T.)",
                    step_text(name),
                    item.owner.product_shape
                ));
                self.link_aspect(feature, item, "datum feature");
                self.add(format!("ID_ATTRIBUTE('{}',#{feature})", step_text(name)));
                self.add(format!("SHAPE_ASPECT_RELATIONSHIP('','',#{feature},#{datum})"));
                self.datums.insert(letter.clone(), datum);
                self.aspects.entry(name.clone()).or_insert(feature);
            }
        }
    }

    /// The presentation of one annotation: its callout (polylines + text).
    fn callout(
        &mut self,
        report: &crate::feature_pipeline::pmi::PmiAnnotationReport,
        style: &LayoutStyle,
    ) -> Result<CalloutOut, String> {
        let drawn = present(&report.geometry, report.label_world, &report.text, style);
        let name = step_text(&report.id);
        let set_name = step_text(&curve_set_name(report));
        let polyline = |emitter: &mut Self, points: &[[f64; 3]], stats: &mut PolylineStats| -> Result<Option<usize>, String> {
            if points.len() < 2 {
                return Ok(None);
            }
            stats.add(points);
            let mut ids = Vec::with_capacity(points.len());
            for point in points {
                ids.push(write_point(emitter.writer, Vec3::new(point[0], point[1], point[2]))?);
            }
            Ok(Some(emitter.add(format!("POLYLINE('{name}',{})", id_list(&ids)))))
        };
        // Geometry subset: dimension / extension / leader lines, arrowheads
        // (closed), frames.
        let mut geometry_stats = PolylineStats::default();
        let mut curves = Vec::new();
        for line in &drawn.polylines {
            if let Some(id) = polyline(self, line, &mut geometry_stats)? {
                curves.push(id);
            }
        }
        for arrow in &drawn.arrows {
            let closed = [arrow[0], arrow[1], arrow[2], arrow[0]];
            if let Some(id) = polyline(self, &closed, &mut geometry_stats)? {
                curves.push(id);
            }
        }
        for frame in &drawn.frames {
            if let Some(id) = polyline(self, frame, &mut geometry_stats)? {
                curves.push(id);
            }
        }
        // Text subset: the runs stroked with the PMI font (Graphic
        // Presentation — the practice's character-based TEXT_LITERAL route is
        // shelved, so the text IS polylines like everything else).
        let mut text_stats = PolylineStats::default();
        let mut text_curves = Vec::new();
        for run in &drawn.texts {
            for stroke in run.strokes() {
                if let Some(id) = polyline(self, &stroke, &mut text_stats)? {
                    text_curves.push(id);
                }
            }
        }
        let mut contents = Vec::new();
        let subset = |emitter: &mut Self, curves: &[usize]| -> Option<usize> {
            if curves.is_empty() {
                return None;
            }
            let set = emitter.add(format!("GEOMETRIC_CURVE_SET('{set_name}',{})", id_list(curves)));
            Some(emitter.add(format!(
                "ANNOTATION_CURVE_OCCURRENCE('{name}',(#{}),#{set})",
                emitter.curve_style
            )))
        };
        if let Some(id) = subset(self, &curves) {
            contents.push(id);
        }
        let text_subset = subset(self, &text_curves);
        contents.extend(text_subset);
        let id = self.add(format!("DRAUGHTING_CALLOUT('{name}',{})", id_list(&contents)));
        let text = drawn
            .texts
            .iter()
            .map(|run| run.text.as_str())
            .collect::<Vec<_>>()
            .join(" ");
        let mut total = geometry_stats;
        total.merge(&text_stats);
        Ok(CalloutOut {
            id,
            text,
            total,
            text_subset,
            text_stats,
        })
    }

    /// The PMI validation properties of a callout (practice §10.3, combined
    /// per §10.3.4): the polyline curve length, the polyline centre point
    /// and the equivalent unicode string, at the callout, plus the centre
    /// point and string of its text subset (what gives a reader the label
    /// position back).
    fn validation(&mut self, callout: &CalloutOut, global: usize) -> Result<(), String> {
        let unicode = step_text(&callout.text);
        let length_unit = self.context.length_unit;
        let geometry_context = self.context.geometry_context;
        let property = |emitter: &mut Self, item: usize, stats: &PolylineStats, with_length: bool| -> Result<(), String> {
            let within = emitter.add(format!(
                "CHARACTERIZED_ITEM_WITHIN_REPRESENTATION('','',#{item},#{global})"
            ));
            let definition = emitter.add(format!("PROPERTY_DEFINITION('pmi validation property','',#{within})"));
            let mut items = Vec::new();
            if with_length {
                items.push(emitter.add(format!(
                    "MEASURE_REPRESENTATION_ITEM('polyline curve length',POSITIVE_LENGTH_MEASURE({}),#{length_unit})",
                    real(stats.length)?
                )));
            }
            let centre = stats.centroid();
            items.push(emitter.add(format!(
                "CARTESIAN_POINT('polyline centre point',({},{},{}))",
                real(centre[0])?,
                real(centre[1])?,
                real(centre[2])?
            )));
            items.push(emitter.add(format!(
                "DESCRIPTIVE_REPRESENTATION_ITEM('equivalent unicode string','{unicode}')"
            )));
            let representation = emitter.add(format!(
                "REPRESENTATION('',{},#{geometry_context})",
                id_list(&items)
            ));
            emitter.add(format!("PROPERTY_DEFINITION_REPRESENTATION(#{definition},#{representation})"));
            Ok(())
        };
        property(self, callout.id, &callout.total, true)?;
        if let Some(text_subset) = callout.text_subset {
            property(self, text_subset, &callout.text_stats, false)?;
        }
        Ok(())
    }
}

/// A written callout with what its validation properties need.
struct CalloutOut {
    /// The `DRAUGHTING_CALLOUT`.
    id: usize,
    /// The equivalent unicode string (the runs joined).
    text: String,
    total: PolylineStats,
    /// The text subset's `ANNOTATION_CURVE_OCCURRENCE`, when there is text.
    text_subset: Option<usize>,
    text_stats: PolylineStats,
}

/// Length-weighted polyline statistics (practice §10.3.1): the total curve
/// length and the centroid of the segment midpoints weighted by length.
#[derive(Debug, Clone, Copy, Default)]
struct PolylineStats {
    length: f64,
    moment: [f64; 3],
    /// A fallback for zero-length sets: the first point.
    first: Option<[f64; 3]>,
}

impl PolylineStats {
    fn add(&mut self, points: &[[f64; 3]]) {
        if self.first.is_none() {
            self.first = points.first().copied();
        }
        for pair in points.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let length = ((b[0] - a[0]).powi(2) + (b[1] - a[1]).powi(2) + (b[2] - a[2]).powi(2)).sqrt();
            self.length += length;
            for axis in 0..3 {
                self.moment[axis] += (a[axis] + b[axis]) * 0.5 * length;
            }
        }
    }

    fn merge(&mut self, other: &PolylineStats) {
        self.length += other.length;
        for axis in 0..3 {
            self.moment[axis] += other.moment[axis];
        }
        if self.first.is_none() {
            self.first = other.first;
        }
    }

    fn centroid(&self) -> [f64; 3] {
        if self.length > 1e-12 {
            [self.moment[0] / self.length, self.moment[1] / self.length, self.moment[2] / self.length]
        } else {
            self.first.unwrap_or([0.0; 3])
        }
    }
}

/// The `GEOMETRIC_CURVE_SET` name — the presented PMI type from the
/// practice's Table 14 (not semantic; a tree label for the reader).
fn curve_set_name(report: &crate::feature_pipeline::pmi::PmiAnnotationReport) -> String {
    match report.kind.as_str() {
        "linear" => "linear dimension".into(),
        "radial" => match &report.geometry {
            PmiGeometry::Radial { diameter: false, .. } => "radial dimension".into(),
            _ => "diameter dimension".into(),
        },
        "angle" => "angular dimension".into(),
        "holeCallout" => "diameter dimension".into(),
        "datum" => "datum".into(),
        "fcf" => match &report.geometry {
            PmiGeometry::Fcf { frame, .. } => match frame.characteristic.as_str() {
                "profileLine" => "profile of line".into(),
                "profileSurface" => "profile of surface".into(),
                "circularRunout" => "circular runout".into(),
                "totalRunout" => "total runout".into(),
                other => other.to_ascii_lowercase(),
            },
            _ => "general tolerance".into(),
        },
        _ => "note".into(),
    }
}

/// The AP242 enumeration for a material condition modifier.
fn material_condition(modifier: &str) -> Option<&'static str> {
    match modifier.trim().to_ascii_uppercase().as_str() {
        "MMC" => Some(".MAXIMUM_MATERIAL_REQUIREMENT."),
        "LMC" => Some(".LEAST_MATERIAL_REQUIREMENT."),
        _ => None,
    }
}

/// The camera of a view as a `CAMERA_MODEL_D3` (+ its view volume).
fn write_camera(
    emitter: &mut Emitter<'_, '_>,
    name: &str,
    camera: &crate::feature_pipeline::pmi::PmiCamera,
) -> Result<usize, String> {
    let eye = Vec3::new(camera.eye[0], camera.eye[1], camera.eye[2]);
    let target = Vec3::new(camera.target[0], camera.target[1], camera.target[2]);
    let view = camera.view_direction();
    let view = Vec3::new(view[0], view[1], view[2]);
    let up_hint = Vec3::new(camera.up[0], camera.up[1], camera.up[2]);
    let up = crate::feature_pipeline::pmi::resolve::perpendicular_in_plane(view, up_hint);
    // Right-handed frame: x × y = z with z = the viewing direction.
    let right = up.cross(view).normalized().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
    let distance = target.sub(eye).length().max(1e-6);
    let aspect = if camera.viewport[1] > 0.0 {
        camera.viewport[0] / camera.viewport[1]
    } else {
        1.5
    };
    let (projection, height) = match camera.projection {
        PmiProjection::Orthographic { half_height } => (".PARALLEL.", 2.0 * half_height),
        PmiProjection::Perspective { fov_y_deg } => {
            (".CENTRAL.", 2.0 * distance * (fov_y_deg.to_radians() * 0.5).tan())
        }
    };
    let width = height * aspect;
    let corner = emitter.add(format!(
        "CARTESIAN_POINT('',({},{}))",
        real(-width * 0.5)?,
        real(-height * 0.5)?
    ));
    let window_placement = emitter.add(format!("AXIS2_PLACEMENT_2D('',#{corner},$)"));
    let window = emitter.add(format!(
        "PLANAR_BOX('',{},{},#{window_placement})",
        real(width)?,
        real(height)?
    ));
    let projection_point = write_point(emitter.writer, Vec3::new(0.0, 0.0, 0.0))?;
    let volume = emitter.add(format!(
        "VIEW_VOLUME({projection},#{projection_point},{},{},.F.,{},.F.,.T.,#{window})",
        real(distance)?,
        real(0.0)?,
        real(distance * 2.0)?
    ));
    let reference = write_placement(emitter.writer, eye, view, right)?;
    Ok(emitter.add(format!(
        "CAMERA_MODEL_D3('{}',#{reference},#{volume})",
        step_text(name)
    )))
}

/// Write the whole PMI block. Called after `SHAPE_DEFINITION_REPRESENTATION`.
/// Write the document's PMI, returning how many DISTINCT reference names named
/// no entity in the file (every annotation on such a name is skipped).
pub(crate) fn write_pmi(
    writer: &mut StepWriter,
    context: &StepContext<'_>,
    pmi: &StepPmi<'_>,
) -> Result<usize, String> {
    if pmi.state.views.is_empty() {
        return Ok(0);
    }
    let env = Env::build("", &serde_json::Value::Null).unwrap_or_else(Env::poisoned);
    // The presentation entities here are the SHARED ones (`step/styles.rs`), so
    // annotation style and geometry colour cannot drift into two spellings of
    // the same AP242 chain.
    let null_style = write_style_assignment(writer, "NULL_STYLE(.NULL.)");
    let colour = write_colour_rgb(writer, [0.0, 0.0, 0.0])?;
    let curve_font = writer.add("DRAUGHTING_PRE_DEFINED_CURVE_FONT('continuous')");
    let curve_style = writer.add(format!(
        "CURVE_STYLE('',#{curve_font},POSITIVE_LENGTH_MEASURE({}),#{colour})",
        real(LINE_WIDTH)?
    ));
    let curve_style = write_style_assignment(writer, &format!("#{curve_style}"));
    let mut emitter = Emitter {
        writer,
        context,
        aspects: HashMap::default(),
        datums: HashMap::default(),
        unresolved: std::collections::BTreeSet::new(),
        null_style,
        curve_style,
    };
    // The shape, mapped into the draughting models with a null style.
    let identity = write_placement(
        emitter.writer,
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
        Vec3::new(1.0, 0.0, 0.0),
    )?;
    let map = emitter.add(format!("REPRESENTATION_MAP(#{identity},#{})", context.representation));
    let mapped = emitter.add(format!("MAPPED_ITEM('',#{map},#{identity})"));
    let styled_shape = write_styled_item(emitter.writer, "", null_style, mapped);

    emitter.write_datums(pmi);

    // Per view: semantic elements, callouts, the annotation plane.
    struct ViewOut {
        name: String,
        /// The view-aligned annotation plane, then one per picked plane.
        planes: Vec<usize>,
        camera: Option<usize>,
        callouts: Vec<CalloutOut>,
        links: Vec<(usize, usize)>,
    }
    let mut views_out: Vec<ViewOut> = Vec::new();
    for view in &pmi.state.views {
        let Some(view_report) = pmi.report.view(&view.id) else {
            continue;
        };
        let camera = view.camera.as_ref();
        let (view_dir, view_up, target) = match camera {
            Some(camera) => (camera.view_direction(), camera.up, camera.target),
            None => ([0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]),
        };
        let text_height = view.display.text_size_pt * MM_PER_POINT;
        let style = LayoutStyle {
            arrow: text_height * 0.9,
            text_height,
            view_dir,
            view_up,
            plane: None,
        };
        let view_vec = Vec3::new(view_dir[0], view_dir[1], view_dir[2]);
        let up_vec = crate::feature_pipeline::pmi::resolve::perpendicular_in_plane(
            view_vec,
            Vec3::new(view_up[0], view_up[1], view_up[2]),
        );
        let right = up_vec.cross(view_vec.scale(-1.0)).normalized().unwrap_or(Vec3::new(1.0, 0.0, 0.0));
        // View-aligned callouts go on the view's annotation plane; callouts
        // in a picked plane group per plane reference (one ANNOTATION_PLANE
        // each, placed ON that plane).
        let mut callouts = Vec::new();
        let mut view_aligned = Vec::new();
        let mut plane_groups: Vec<(String, crate::feature_pipeline::pmi::PmiPlane, Vec<usize>)> = Vec::new();
        let mut links = Vec::new();
        for (annotation, report) in view.annotations.iter().zip(view_report.annotations.iter()) {
            if report.status != PmiStatus::Ok || !report.enabled {
                continue;
            }
            if matches!(report.geometry, PmiGeometry::Explode { .. } | PmiGeometry::None) {
                continue;
            }
            let semantic = emitter.semantic(annotation, report, &env)?;
            // A view-aligned row is drawn in the view-parallel plane through
            // its label (practice §9.1: polylines lie in a plane parallel to
            // their ANNOTATION_PLANE); a picked plane is used as is.
            let row_style = LayoutStyle {
                plane: report.plane.or(Some(crate::feature_pipeline::pmi::PmiPlane {
                    origin: report.label_world,
                    normal: [-view_dir[0], -view_dir[1], -view_dir[2]],
                    x_axis: [right.x, right.y, right.z],
                })),
                ..style
            };
            let callout = emitter.callout(report, &row_style)?;
            let callout_id = callout.id;
            callouts.push(callout);
            match report.plane {
                Some(plane) => {
                    let key = annotation.plane_ref().unwrap_or("").to_string();
                    match plane_groups.iter_mut().find(|(name, _, _)| *name == key) {
                        Some(group) => group.2.push(callout_id),
                        None => plane_groups.push((key, plane, vec![callout_id])),
                    }
                }
                None => view_aligned.push(callout_id),
            }
            if let Some(semantic) = semantic {
                links.push((semantic, callout_id));
            }
        }
        let placement = write_placement(
            emitter.writer,
            Vec3::new(target[0], target[1], target[2]),
            view_vec.scale(-1.0),
            right,
        )?;
        let plane_geometry = emitter.add(format!("PLANE('',#{placement})"));
        let mut planes = vec![emitter.add(format!(
            "ANNOTATION_PLANE('{}',(#{null_style}),#{plane_geometry},{})",
            step_text(&view.name),
            id_list(&view_aligned)
        ))];
        for (key, plane, members) in &plane_groups {
            let placement = write_placement(
                emitter.writer,
                Vec3::new(plane.origin[0], plane.origin[1], plane.origin[2]),
                Vec3::new(plane.normal[0], plane.normal[1], plane.normal[2]),
                Vec3::new(plane.x_axis[0], plane.x_axis[1], plane.x_axis[2]),
            )?;
            let geometry = emitter.add(format!("PLANE('',#{placement})"));
            planes.push(emitter.add(format!(
                "ANNOTATION_PLANE('{}',(#{null_style}),#{geometry},{})",
                step_text(&format!("{} / {key}", view.name)),
                id_list(members)
            )));
        }
        let camera_id = match camera {
            Some(camera) => Some(write_camera(&mut emitter, &view.name, camera)?),
            None => None,
        };
        views_out.push(ViewOut {
            name: view.name.clone(),
            planes,
            camera: camera_id,
            callouts,
            links,
        });
    }

    // The global draughting model: every annotation plane + the shape.
    let mut global_items = vec![styled_shape];
    global_items.extend(views_out.iter().flat_map(|view| view.planes.iter().copied()));
    let global = emitter.add(format!(
        "DRAUGHTING_MODEL('',{},#{})",
        id_list(&global_items),
        context.geometry_context
    ));
    for view in &views_out {
        let mut items = vec![styled_shape];
        if let Some(camera) = view.camera {
            items.push(camera);
        }
        items.extend(view.callouts.iter().map(|callout| callout.id));
        let model = emitter.add(format!(
            "DRAUGHTING_MODEL('{}',{},#{})",
            step_text(&view.name),
            id_list(&items),
            context.geometry_context
        ));
        emitter.add(format!(
            "MECHANICAL_DESIGN_AND_DRAUGHTING_RELATIONSHIP('','',#{model},#{global})"
        ));
        for (semantic, callout) in &view.links {
            emitter.add(format!(
                "DRAUGHTING_MODEL_ITEM_ASSOCIATION('PMI representation to presentation link','',#{semantic},#{global},#{callout})"
            ));
        }
    }
    // PMI validation properties (§10.3) hang off the global model.
    for view in &views_out {
        for callout in &view.callouts {
            emitter.validation(callout, global)?;
        }
    }
    Ok(emitter.unresolved.len())
}
