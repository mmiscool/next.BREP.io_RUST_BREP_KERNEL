use super::*;

/// An orthonormal placement frame (STEP AXIS2_PLACEMENT_3D): origin plus the
/// local x/y/z axes (z = `axis`, x = `ref_direction` projected orthogonal to z).
#[derive(Clone, Copy, Debug)]
pub(super) struct Frame {
    pub(super) origin: Vec3,
    pub(super) x: Vec3,
    pub(super) y: Vec3,
    pub(super) z: Vec3,
}

// ---------------------------------------------------------------------------
// Part-21 value model
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
// A complete Part-21 value model: the round-trip subset does not read every
// variant's payload (string labels, typed unit measures, `$`/`*`), but a
// faithful parser must still recognise and carry them.
#[allow(dead_code)]
pub(super) enum Value {
    Int(i64),
    Real(f64),
    Str(String),
    Enum(String),
    Ref(usize),
    Unset,
    Derived,
    List(Vec<Value>),
    /// A typed parameter such as `LENGTH_MEASURE(1.E-6)`.
    Typed(String, Vec<Value>),
}

impl Value {
    pub(super) fn as_ref_id(&self) -> Result<usize, String> {
        match self {
            Value::Ref(id) => Ok(*id),
            other => Err(format!(
                "step_import: expected an entity reference, got {other:?}"
            )),
        }
    }

    fn as_int(&self) -> Result<i64, String> {
        match self {
            Value::Int(value) => Ok(*value),
            Value::Real(value) if value.fract() == 0.0 => Ok(*value as i64),
            other => Err(format!("step_import: expected an integer, got {other:?}")),
        }
    }

    pub(super) fn as_real(&self) -> Result<f64, String> {
        match self {
            Value::Real(value) => Ok(*value),
            Value::Int(value) => Ok(*value as f64),
            other => Err(format!("step_import: expected a real, got {other:?}")),
        }
    }

    pub(super) fn as_list(&self) -> Result<&[Value], String> {
        match self {
            Value::List(items) => Ok(items),
            other => Err(format!("step_import: expected a list, got {other:?}")),
        }
    }

    pub(super) fn enum_is(&self, name: &str) -> bool {
        matches!(self, Value::Enum(value) if value == name)
    }
}

/// One entity record: either a simple `TYPE(args)` or a complex/subsuper
/// instance `(TYPE(args)TYPE(args)…)` (a list of simple records).
#[derive(Clone, Debug)]
pub(super) struct Entity {
    pub(super) records: Vec<(String, Vec<Value>)>,
}

impl Entity {
    pub(super) fn find(&self, keyword: &str) -> Option<&[Value]> {
        self.records
            .iter()
            .find(|(name, _)| name == keyword)
            .map(|(_, args)| args.as_slice())
    }

    pub(super) fn has(&self, keyword: &str) -> bool {
        self.records.iter().any(|(name, _)| name == keyword)
    }
}

pub(super) fn bspline_curve_declares_closed(entity: &Entity) -> bool {
    entity
        .find("B_SPLINE_CURVE")
        .and_then(|args| args.get(3))
        .is_some_and(|value| value.enum_is("T"))
        || entity
            .find("B_SPLINE_CURVE_WITH_KNOTS")
            .and_then(|args| args.get(4))
            .is_some_and(|value| value.enum_is("T"))
}

// ---------------------------------------------------------------------------
// Tokenizer / recursive-descent parser
// ---------------------------------------------------------------------------

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(text: &'a str) -> Self {
        Self {
            bytes: text.as_bytes(),
            pos: 0,
        }
    }

    fn skip_trivia(&mut self) {
        loop {
            while self.pos < self.bytes.len() && self.bytes[self.pos].is_ascii_whitespace() {
                self.pos += 1;
            }
            // Part-21 comments: /* ... */
            if self.pos + 1 < self.bytes.len()
                && self.bytes[self.pos] == b'/'
                && self.bytes[self.pos + 1] == b'*'
            {
                self.pos += 2;
                while self.pos + 1 < self.bytes.len()
                    && !(self.bytes[self.pos] == b'*' && self.bytes[self.pos + 1] == b'/')
                {
                    self.pos += 1;
                }
                self.pos = (self.pos + 2).min(self.bytes.len());
                continue;
            }
            break;
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.pos).copied()
    }

    fn parse_string(&mut self) -> Result<String, String> {
        // Opening quote already confirmed by caller.
        self.pos += 1;
        let mut out = String::new();
        while self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            if c == b'\'' {
                if self.bytes.get(self.pos + 1) == Some(&b'\'') {
                    out.push('\'');
                    self.pos += 2;
                    continue;
                }
                self.pos += 1;
                return Ok(out);
            }
            out.push(c as char);
            self.pos += 1;
        }
        Err("step_import: unterminated string".into())
    }

    fn parse_keyword(&mut self) -> String {
        let start = self.pos;
        while self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            if c.is_ascii_alphanumeric() || c == b'_' {
                self.pos += 1;
            } else {
                break;
            }
        }
        String::from_utf8_lossy(&self.bytes[start..self.pos]).into_owned()
    }

    fn parse_number(&mut self) -> Result<Value, String> {
        let start = self.pos;
        let mut is_real = false;
        if matches!(self.peek(), Some(b'+') | Some(b'-')) {
            self.pos += 1;
        }
        while self.pos < self.bytes.len() {
            let c = self.bytes[self.pos];
            match c {
                b'0'..=b'9' => self.pos += 1,
                b'.' => {
                    is_real = true;
                    self.pos += 1;
                }
                b'e' | b'E' => {
                    is_real = true;
                    self.pos += 1;
                    if matches!(self.peek(), Some(b'+') | Some(b'-')) {
                        self.pos += 1;
                    }
                }
                _ => break,
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|_| "step_import: invalid number".to_string())?;
        if is_real {
            text.parse::<f64>()
                .map(Value::Real)
                .map_err(|_| format!("step_import: bad real '{text}'"))
        } else {
            text.parse::<i64>()
                .map(Value::Int)
                .map_err(|_| format!("step_import: bad integer '{text}'"))
        }
    }

    /// Parse one parameter value.
    fn parse_value(&mut self) -> Result<Value, String> {
        self.skip_trivia();
        let c = self.peek().ok_or("step_import: unexpected end of value")?;
        match c {
            b'#' => {
                self.pos += 1;
                match self.parse_number()? {
                    Value::Int(id) if id >= 0 => Ok(Value::Ref(id as usize)),
                    other => Err(format!("step_import: bad entity reference {other:?}")),
                }
            }
            b'\'' => Ok(Value::Str(self.parse_string()?)),
            b'(' => Ok(Value::List(self.parse_arg_list()?)),
            b'$' => {
                self.pos += 1;
                Ok(Value::Unset)
            }
            b'*' => {
                self.pos += 1;
                Ok(Value::Derived)
            }
            b'.' => {
                // Enumeration `.XXX.` unless a real literal like `.5`.
                if self.bytes.get(self.pos + 1).is_some_and(u8::is_ascii_digit) {
                    self.parse_number()
                } else {
                    self.pos += 1;
                    let name = self.parse_keyword();
                    if self.peek() == Some(b'.') {
                        self.pos += 1;
                    }
                    Ok(Value::Enum(name))
                }
            }
            b'+' | b'-' | b'0'..=b'9' => self.parse_number(),
            _ if c.is_ascii_alphabetic() => {
                // Typed value such as LENGTH_MEASURE(...) or a bare keyword.
                let keyword = self.parse_keyword();
                self.skip_trivia();
                if self.peek() == Some(b'(') {
                    Ok(Value::Typed(keyword, self.parse_arg_list()?))
                } else {
                    Ok(Value::Enum(keyword))
                }
            }
            other => Err(format!(
                "step_import: unexpected character '{}'",
                other as char
            )),
        }
    }

    /// Parse `( value, value, … )`. The opening paren is consumed here.
    fn parse_arg_list(&mut self) -> Result<Vec<Value>, String> {
        self.skip_trivia();
        if self.peek() != Some(b'(') {
            return Err("step_import: expected '('".into());
        }
        self.pos += 1;
        let mut items = Vec::new();
        loop {
            self.skip_trivia();
            match self.peek() {
                Some(b')') => {
                    self.pos += 1;
                    return Ok(items);
                }
                Some(b',') => {
                    self.pos += 1;
                }
                None => return Err("step_import: unterminated list".into()),
                _ => items.push(self.parse_value()?),
            }
        }
    }

    /// Parse the right-hand side of `#N=` into an Entity (simple or complex).
    fn parse_entity_body(&mut self) -> Result<Entity, String> {
        self.skip_trivia();
        if self.peek() == Some(b'(') {
            // Complex/subsuper instance: (KEYWORD(args)KEYWORD(args)...)
            self.pos += 1;
            let mut records = Vec::new();
            loop {
                self.skip_trivia();
                match self.peek() {
                    Some(b')') => {
                        self.pos += 1;
                        break;
                    }
                    Some(c) if c.is_ascii_alphabetic() => {
                        let keyword = self.parse_keyword();
                        let args = self.parse_arg_list()?;
                        records.push((keyword, args));
                    }
                    other => {
                        return Err(format!(
                            "step_import: malformed complex record near '{}'",
                            other.map(|b| b as char).unwrap_or(' ')
                        ))
                    }
                }
            }
            Ok(Entity { records })
        } else {
            let keyword = self.parse_keyword();
            if keyword.is_empty() {
                return Err("step_import: expected an entity keyword".into());
            }
            let args = self.parse_arg_list()?;
            Ok(Entity {
                records: vec![(keyword, args)],
            })
        }
    }
}

/// Split the DATA section into `#N = body ;` statements and parse each.
pub(super) fn parse_data_section(text: &str) -> Result<HashMap<usize, Entity>, String> {
    let data_start = text
        .find("DATA;")
        .ok_or("step_import: no DATA section found")?;
    let data_body = &text[data_start + "DATA;".len()..];
    let data_end = data_body.find("ENDSEC").unwrap_or(data_body.len());
    let data_body = &data_body[..data_end];

    let mut entities = HashMap::default();
    let mut parser = Parser::new(data_body);
    loop {
        parser.skip_trivia();
        let Some(c) = parser.peek() else { break };
        if c != b'#' {
            // Skip to the next statement terminator defensively.
            while parser.peek().is_some_and(|b| b != b';') {
                parser.pos += 1;
            }
            if parser.peek() == Some(b';') {
                parser.pos += 1;
            }
            continue;
        }
        parser.pos += 1; // '#'
        let id = match parser.parse_number()? {
            Value::Int(id) if id >= 0 => id as usize,
            other => return Err(format!("step_import: bad entity id {other:?}")),
        };
        parser.skip_trivia();
        if parser.peek() != Some(b'=') {
            return Err(format!("step_import: expected '=' after #{id}"));
        }
        parser.pos += 1;
        let entity = parser.parse_entity_body()?;
        parser.skip_trivia();
        if parser.peek() == Some(b';') {
            parser.pos += 1;
        }
        entities.insert(id, entity);
    }
    Ok(entities)
}

// ---------------------------------------------------------------------------
// Geometry resolution
// ---------------------------------------------------------------------------

pub(super) struct Resolver<'a> {
    pub(super) entities: &'a HashMap<usize, Entity>,
    /// Native length unit → MILLIMETRES. The kernel works in mm at all times;
    /// every length read from the file (coordinates, radii, semi-axes, offset
    /// distances) is multiplied by this. Angles, parameters, knots, and
    /// weights are dimensionless and stay untouched. Derived from the file's
    /// `GLOBAL_UNIT_ASSIGNED_CONTEXT` length unit (`SI_UNIT` prefix or
    /// `CONVERSION_BASED_UNIT` measure); 1.0 (assume mm) when absent.
    pub(super) length_scale: f64,
    /// Radians per one of the file's `PLANE_ANGLE_UNIT`, or `None` when the
    /// file declares none this parser can resolve. Only supplied pcurves read
    /// it: every angle the importer reads from a geometric entity is either
    /// already dimensionless or range-disambiguated, while a supplied pcurve's
    /// angular coordinate is neither. See `supplied::derive_angle_scale`.
    ///
    /// LAZY, and that is load-bearing rather than tidy. Deriving it sweeps
    /// every entity in the file — twice, when the first sweep finds no
    /// `GLOBAL_UNIT_ASSIGNED_CONTEXT` — and `Resolver::new` is called inside a
    /// loop over assembly edges, so computing it eagerly made importing an
    /// assembly O(edges x entities) for a value the default importer never
    /// reads. `OnceLock` rather than `OnceCell` because `bodies.rs` hands a
    /// `&Resolver` to a rayon `par_iter`, which requires `Sync`.
    angle_scale: std::sync::OnceLock<Option<f64>>,
}

impl<'a> Resolver<'a> {
    /// The resolver for one representation context: its length unit in
    /// millimetres. The plane-angle unit is derived on first use.
    pub(super) fn new(entities: &'a HashMap<usize, Entity>, length_scale: f64) -> Self {
        Self {
            entities,
            length_scale,
            angle_scale: std::sync::OnceLock::new(),
        }
    }

    /// Radians per one of the file's `PLANE_ANGLE_UNIT`, derived once per
    /// resolver on the first call and never at all when nothing asks.
    pub(super) fn angle_scale(&self) -> Option<f64> {
        *self
            .angle_scale
            .get_or_init(|| derive_angle_scale(self.entities))
    }
}

/// Millimetres per one of the given unit entity (an entity carrying
/// `LENGTH_UNIT`): `SI_UNIT(prefix, .METRE.)` → 1000 × prefix;
/// `CONVERSION_BASED_UNIT(name, #measure)` → measure × its base unit's scale.
pub(super) fn unit_length_scale_mm(
    entities: &HashMap<usize, Entity>,
    id: usize,
    depth: usize,
) -> Option<f64> {
    if depth > 4 {
        return None;
    }
    let entity = entities.get(&id)?;
    if !entity.has("LENGTH_UNIT") {
        return None;
    }
    if let Some(args) = entity.find("SI_UNIT") {
        // SI_UNIT(prefix, name) — name .METRE. is the only SI length unit.
        if !args.get(1).is_some_and(|name| name.enum_is("METRE")) {
            return None;
        }
        let prefix = match args.first() {
            Some(Value::Enum(prefix)) => match prefix.as_str() {
                "MILLI" => 1e-3,
                "CENTI" => 1e-2,
                "DECI" => 1e-1,
                "DECA" => 1e1,
                "HECTO" => 1e2,
                "KILO" => 1e3,
                "MICRO" => 1e-6,
                "NANO" => 1e-9,
                _ => return None,
            },
            _ => 1.0,
        };
        return Some(1000.0 * prefix);
    }
    if let Some(args) = entity.find("CONVERSION_BASED_UNIT") {
        // CONVERSION_BASED_UNIT(name, #LENGTH_MEASURE_WITH_UNIT).
        let measure_entity = entities.get(&args.get(1)?.as_ref_id().ok()?)?;
        let measure_args = measure_entity
            .find("LENGTH_MEASURE_WITH_UNIT")
            .or_else(|| measure_entity.find("MEASURE_WITH_UNIT"))?;
        let value = match measure_args.first()? {
            Value::Typed(_, inner) => inner.first()?.as_real().ok()?,
            other => other.as_real().ok()?,
        };
        let base = unit_length_scale_mm(entities, measure_args.get(1)?.as_ref_id().ok()?, depth + 1)?;
        return Some(value * base);
    }
    None
}

/// Every precision the file states — each
/// `UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(x), #unit, name, description)`
/// — in MILLIMETRES, sorted ascending and deduplicated. Empty when the file
/// states none, or none of them resolves to a length.
///
/// The figure is in the unit ITS OWN second argument names, and it is scaled
/// through the same [`unit_length_scale_mm`] the coordinates go through: 31
/// of the 48 corpus fixtures that state a precision declare metres, so a raw
/// `1.E-6` there is 1e-3 mm, and comparing it unscaled against millimetre
/// measurements inverts the answer. The unit is read off the uncertainty
/// entity rather than off a context, so a file that states several figures
/// in several contexts (`BIMExample-Site.step`: five values over four
/// orders) yields them all; the consistency check compares against the
/// LOOSEST (`precision::stated_precision_diagnostics`).
///
/// A file states the figure once per `REPRESENTATION_CONTEXT`; entity ids
/// are visited in ascending order so the result is deterministic whatever
/// the map's iteration order.
pub(super) fn stated_precisions_mm(entities: &HashMap<usize, Entity>) -> Vec<f64> {
    let mut ids: Vec<usize> = entities
        .iter()
        .filter(|(_, entity)| entity.has("UNCERTAINTY_MEASURE_WITH_UNIT"))
        .map(|(&id, _)| id)
        .collect();
    ids.sort_unstable();
    let mut out = Vec::new();
    for id in ids {
        let Some(args) = entities.get(&id).and_then(|e| e.find("UNCERTAINTY_MEASURE_WITH_UNIT")) else {
            continue;
        };
        let value = match args.first() {
            Some(Value::Typed(_, inner)) => inner.first().and_then(|v| v.as_real().ok()),
            Some(other) => other.as_real().ok(),
            None => None,
        };
        let scale = args
            .get(1)
            .and_then(|unit| unit.as_ref_id().ok())
            .and_then(|unit| unit_length_scale_mm(entities, unit, 0));
        if let (Some(value), Some(scale)) = (value, scale) {
            let mm = value * scale;
            if mm.is_finite() && mm > 0.0 {
                out.push(mm);
            }
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

/// One REPRESENTATION_CONTEXT's length unit → mm factor: the first
/// `GLOBAL_UNIT_ASSIGNED_CONTEXT` unit of that context that resolves to a
/// LENGTH_UNIT. `None` when the entity is not a unit-assigned context, or
/// carries no resolvable length unit.
///
/// A mixed-unit assembly gives EACH product representation its own context, so
/// this — not the file-global [`derive_length_scale_mm`] — is the correct scale
/// for that product's geometry (`io/step_import/assembly.rs`).
pub(super) fn context_length_scale_mm(
    entities: &HashMap<usize, Entity>,
    context_id: usize,
) -> Option<f64> {
    let args = entities
        .get(&context_id)?
        .find("GLOBAL_UNIT_ASSIGNED_CONTEXT")?;
    let units = args.first()?.as_list().ok()?;
    units
        .iter()
        .filter_map(|unit| unit.as_ref_id().ok())
        .find_map(|unit_id| unit_length_scale_mm(entities, unit_id, 0))
}

/// The file's length unit → mm factor: prefer the length unit referenced from
/// a `GLOBAL_UNIT_ASSIGNED_CONTEXT`, fall back to any `LENGTH_UNIT` entity,
/// default 1.0 (assume the file is already in millimetres).
///
/// ONE scale for the whole file: on a multi-context (mixed-unit) file the
/// surviving context is whichever the `HashMap` iterates to first. The
/// structured assembly lane resolves units per product instead — see
/// [`context_length_scale_mm`] — and the flat lane keeps this behaviour so its
/// output stays byte-identical.
pub(super) fn derive_length_scale_mm(entities: &HashMap<usize, Entity>) -> f64 {
    for (&id, entity) in entities {
        if !entity.has("GLOBAL_UNIT_ASSIGNED_CONTEXT") {
            continue;
        }
        if let Some(scale) = context_length_scale_mm(entities, id) {
            return scale;
        }
    }
    for (&id, entity) in entities {
        if entity.has("LENGTH_UNIT") {
            if let Some(scale) = unit_length_scale_mm(entities, id, 0) {
                return scale;
            }
        }
    }
    1.0
}

impl<'a> Resolver<'a> {
    pub(super) fn get(&self, id: usize) -> Result<&Entity, String> {
        self.entities
            .get(&id)
            .ok_or_else(|| format!("step_import: dangling reference #{id}"))
    }

    /// Convert a native-unit length from the file into millimetres.
    pub(super) fn length(&self, value: f64) -> f64 {
        value * self.length_scale
    }

    pub(super) fn point(&self, id: usize) -> Result<Vec3, String> {
        let entity = self.get(id)?;
        let args = entity
            .find("CARTESIAN_POINT")
            .ok_or_else(|| format!("step_import: #{id} is not a CARTESIAN_POINT"))?;
        let coords = args
            .get(1)
            .ok_or("step_import: CARTESIAN_POINT missing coordinates")?
            .as_list()?;
        let x = coords
            .first()
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        let y = coords
            .get(1)
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        let z = coords
            .get(2)
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        Ok(Vec3::new(self.length(x), self.length(y), self.length(z)))
    }

    pub(super) fn step_vertex_point(&self, id: usize) -> Result<Vec3, String> {
        let point_ref = self
            .get(id)?
            .find("VERTEX_POINT")
            .and_then(|args| args.get(1))
            .ok_or("step_import: VERTEX_POINT missing geometry")?
            .as_ref_id()?;
        self.point(point_ref)
    }

    fn control_points(&self, refs: &[Value], weights: &[f64]) -> Result<Vec<Vec4>, String> {
        refs.iter()
            .enumerate()
            .map(|(index, value)| {
                let point = self.point(value.as_ref_id()?)?;
                let weight = weights.get(index).copied().unwrap_or(1.0);
                Ok(Vec4::from_point(point, weight))
            })
            .collect()
    }

    pub(super) fn direction(&self, id: usize) -> Result<Vec3, String> {
        let entity = self.get(id)?;
        let coords = entity
            .find("DIRECTION")
            .and_then(|args| args.get(1))
            .ok_or_else(|| format!("step_import: #{id} is not a DIRECTION"))?
            .as_list()?;
        let x = coords
            .first()
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        let y = coords
            .get(1)
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        let z = coords
            .get(2)
            .map(Value::as_real)
            .transpose()?
            .unwrap_or(0.0);
        Vec3::new(x, y, z).normalized()
    }

    /// Resolve an AXIS2_PLACEMENT_3D into an orthonormal `Frame`. `axis` and
    /// `ref_direction` are optional in STEP; fall back to sensible defaults and
    /// re-orthogonalise `ref_direction` against `axis` (ISO 10303-42 §7.5).
    pub(super) fn placement(&self, id: usize) -> Result<Frame, String> {
        let entity = self.get(id)?;
        let args = entity
            .find("AXIS2_PLACEMENT_3D")
            .ok_or_else(|| format!("step_import: #{id} is not an AXIS2_PLACEMENT_3D"))?;
        let origin = self.point(args[1].as_ref_id()?)?;
        let z = match args.get(2) {
            Some(Value::Ref(axis_ref)) => self.direction(*axis_ref)?,
            _ => Vec3::new(0.0, 0.0, 1.0),
        };
        let ref_dir = match args.get(3) {
            Some(Value::Ref(dir_ref)) => Some(self.direction(*dir_ref)?),
            _ => None,
        };
        let x = match ref_dir {
            Some(dir) => {
                let projected = dir.sub(z.scale(dir.dot(z)));
                projected.normalized().or_else(|_| z.perpendicular())?
            }
            None => z.perpendicular()?,
        };
        let y = z.cross(x).normalized()?;
        Ok(Frame { origin, x, y, z })
    }

    /// Resolve an analytic surface (PLANE / CYLINDRICAL / CONICAL / SPHERICAL /
    /// TOROIDAL) into the kernel's analytic NURBS carrier, sized in the axial /
    /// planar directions to cover the face (`samples` = points on the face's
    /// edges). Periodic directions build the full revolution; face bounds trim.
    fn analytic_surface(
        &self,
        entity: &Entity,
        samples: &[Vec3],
        sphere_seam: Option<Vec3>,
    ) -> Result<Option<NurbsSurface>, String> {
        if let Some(args) = entity.find("PLANE") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            return Ok(Some(build_plane_over(&frame, samples)?));
        }
        if let Some(args) = entity.find("CYLINDRICAL_SURFACE") {
            let frame = reseam(&self.placement(args[1].as_ref_id()?)?, samples);
            let radius = self.length(args[2].as_real()?);
            let (base, height, _) = axial_extent(&frame, samples);
            return Ok(Some(build_cylinder(&frame, radius, base, height)?));
        }
        if let Some(args) = entity.find("CONICAL_SURFACE") {
            let frame = reseam_cone(&self.placement(args[1].as_ref_id()?)?, samples);
            let radius = self.length(args[2].as_real()?);
            let mut semi_angle = args[3].as_real()?;
            // Some AP214 producers write angular entity attributes in the
            // file's conversion-based degree unit (rather than normalizing
            // them to radians).  A cone semi-angle cannot reach pi/2, so this
            // range check is unambiguous and avoids constructing wildly sized
            // carrier surfaces from values such as `59.` degrees.
            if semi_angle.abs() >= std::f64::consts::FRAC_PI_2 {
                semi_angle = semi_angle.to_radians();
            }
            let (base, height, c_start) = axial_extent(&frame, samples);
            let tan = semi_angle.tan();
            let mut radius_bottom = radius + c_start * tan;
            let mut radius_top = radius + (c_start + height) * tan;
            // The apex band must scale with the geometry, not sit at an absolute
            // millimetre floor: metre-unit files carry healthy sub-millimetre
            // cones (radii ~3e-4 here) that a fixed 1e-3 band swallowed whole —
            // the same absolute-floor bug already corrected inside
            // `make_revolution` (see NEAR_AXIS_RELATIVE_TOLERANCE there).
            //
            // Two effects set the band. `axial_extent` pads the sampled axial
            // range outward by `margin`, so a valid pointed cone (apex exactly
            // at a sampled endpoint) has a padded radius up to `tan * margin`
            // below zero; that overshoot must NOT read as a genuine crossing.
            // On top of that, allow a relative slice of the cone's own radial
            // reach for numeric noise. A radius more negative than this band is
            // a true through-apex crossing (the generatrix crosses the axis and
            // the revolution would self-intersect); a face wholly inside it has
            // no reliable radial extent to build.
            let radial_extent = radius_bottom
                .abs()
                .max(radius_top.abs())
                .max(radius.abs());
            let margin = 1e-4 * height.abs().max(1.0) + 1e-9;
            let radius_tol = tan.abs() * margin + 1e-4 * radial_extent;
            if radius_bottom < -radius_tol
                || radius_top < -radius_tol
                || (radius_bottom <= radius_tol && radius_top <= radius_tol)
            {
                return Err(format!(
                    "step_import: conical surface face crosses or precedes the apex (unsupported; radii {radius_bottom:.9}, {radius_top:.9})"
                ));
            }
            // Axial extents derived from the sampled vertex-loop can put an
            // exact apex a few ulps past zero. Preserve the singular endpoint
            // instead of rejecting a valid pointed cone.
            radius_bottom = radius_bottom.max(0.0);
            radius_top = radius_top.max(0.0);
            return Ok(Some(build_cone(
                &frame,
                base,
                radius_bottom,
                radius_top,
                height,
            )?));
        }
        if let Some(args) = entity.find("SPHERICAL_SURFACE") {
            let placement = self.placement(args[1].as_ref_id()?)?;
            let frame = if let Some(x) = sphere_seam {
                let x = x.sub(placement.z.scale(x.dot(placement.z))).normalized()?;
                Frame {
                    origin: placement.origin,
                    x,
                    y: placement.z.cross(x).normalized()?,
                    z: placement.z,
                }
            } else {
                reseam(&placement, samples)
            };
            let radius = self.length(args[2].as_real()?);
            return Ok(Some(build_sphere(&frame, radius)?));
        }
        if let Some(args) = entity.find("TOROIDAL_SURFACE") {
            let frame = reseam(&self.placement(args[1].as_ref_id()?)?, samples);
            let major = self.length(args[2].as_real()?);
            let minor = self.length(args[3].as_real()?);
            return Ok(Some(build_torus(&frame, major, minor)?));
        }
        if let Some(args) = entity.find("SURFACE_OF_LINEAR_EXTRUSION") {
            // SURFACE_OF_LINEAR_EXTRUSION(name, swept_curve, extrusion VECTOR).
            // The carrier is conceptually infinite along the sweep (OCC writes
            // a UNIT vector), so — like the cylinder/cone axial sizing — the
            // v-extent must come from the face's sampled boundary, not the
            // vector's magnitude.
            let profile_ref = args[1].as_ref_id()?;
            let vector_ref = args[2].as_ref_id()?;
            let vector_entity = self.get(vector_ref)?;
            let vector_args = vector_entity.find("VECTOR").ok_or_else(|| {
                format!("step_import: extrusion axis #{vector_ref} is not a VECTOR")
            })?;
            let direction = self.direction(vector_args[1].as_ref_id()?)?;
            let profile = self.curve(profile_ref).map_err(|error| {
                format!("step_import: extrusion profile #{profile_ref}: {error}")
            })?;
            return Ok(Some(build_linear_extrusion(&profile, direction, samples)?));
        }
        if let Some(args) = entity.find("SURFACE_OF_REVOLUTION") {
            // SURFACE_OF_REVOLUTION(name, swept_curve, AXIS1_PLACEMENT). The
            // profile (generatrix) is revolved a full 360° about the axis
            // location/direction; the face's sampled boundary trims the periodic
            // wall, exactly like the cylinder/cone/torus analytic carriers (all
            // of which are themselves revolutions under the hood).
            let profile_ref = args
                .get(1)
                .ok_or("step_import: SURFACE_OF_REVOLUTION missing swept_curve")?
                .as_ref_id()?;
            let axis_ref = args
                .get(2)
                .ok_or("step_import: SURFACE_OF_REVOLUTION missing axis")?
                .as_ref_id()?;
            let (axis_point, axis_dir) = self.axis1_placement(axis_ref)?;
            let profile = self.curve(profile_ref).map_err(|error| {
                format!("step_import: revolution profile #{profile_ref}: {error}")
            })?;
            return Ok(Some(build_revolution(
                &profile, axis_point, axis_dir, samples,
            )?));
        }
        Ok(None)
    }

    /// Resolve an AXIS1_PLACEMENT into (location, axis direction). Unlike an
    /// AXIS2_PLACEMENT_3D there is no ref_direction: only the point and the
    /// (optional) axis, which defaults to +Z per ISO 10303-42.
    fn axis1_placement(&self, id: usize) -> Result<(Vec3, Vec3), String> {
        let entity = self.get(id)?;
        let args = entity
            .find("AXIS1_PLACEMENT")
            .ok_or_else(|| format!("step_import: #{id} is not an AXIS1_PLACEMENT"))?;
        let origin = self.point(
            args.get(1)
                .ok_or("step_import: AXIS1_PLACEMENT missing location")?
                .as_ref_id()?,
        )?;
        let axis = match args.get(2) {
            Some(Value::Ref(axis_ref)) => self.direction(*axis_ref)?,
            _ => Vec3::new(0.0, 0.0, 1.0),
        };
        Ok((origin, axis))
    }

    /// Resolve the geometry of an EDGE_CURVE, oriented so `evaluate(t0)` is
    /// `p_start` and `evaluate(t1)` is `p_end`. `closed` preserves the raw STEP
    /// endpoint identity: coordinate-close but ref-distinct conic endpoints are
    /// a short arc, not a full circle/ellipse. B_SPLINE uses its stored control
    /// net (reversed when the edge disagrees with the curve).
    pub(super) fn resolve_edge_curve(
        &self,
        id: usize,
        p_start: Vec3,
        p_end: Vec3,
        same_sense: bool,
        closed: bool,
    ) -> Result<NurbsCurve, String> {
        let entity = self.get(id)?;
        // OCC writes edge geometry as a curve-on-surface bundle:
        // SURFACE_CURVE / SEAM_CURVE / INTERSECTION_CURVE(name, curve_3d,
        // (pcurve refs…), master). The 3D curve carries the geometry; the
        // pcurve associates are rebuilt by our own projection, so unwrap and
        // resolve the underlying curve.
        for wrapper in ["SURFACE_CURVE", "SEAM_CURVE", "INTERSECTION_CURVE"] {
            if let Some(args) = entity.find(wrapper) {
                let inner = args
                    .get(1)
                    .ok_or_else(|| format!("step_import: {wrapper} #{id} missing curve_3d"))?
                    .as_ref_id()?;
                return self.resolve_edge_curve(inner, p_start, p_end, same_sense, closed);
            }
        }
        if entity.has("B_SPLINE_CURVE") || entity.has("B_SPLINE_CURVE_WITH_KNOTS") {
            let curve = self.curve(id)?;
            let authored_closed = bspline_curve_declares_closed(entity);
            // OCC reuses ONE long spline across several EDGE_CURVEs, each
            // trimmed to a sub-range purely by its vertices (a SURFACE_CURVE
            // habit). When the edge's vertices project strictly inside the
            // spline's domain, extract that sub-curve; a full-extent edge
            // keeps the stored net exactly (the overwhelmingly common case).
            if p_start.sub(p_end).length() > 0.0 {
                let [dom0, dom1] = curve.domain()?;
                let t_start = crate::project_point_to_curve(&curve, p_start)?;
                let t_end = crate::project_point_to_curve(&curve, p_end)?;
                // Floor at the split guard's own epsilon: a relative-only
                // tolerance underflows KNOT_IDENTITY_TOL on tiny-domain curves
                // and the caller then requests a split the guard rejects
                // (ABC 8575 at mm scale: span 1.3e-4, request 3.9e-10 from the
                // end).
                let span_tol =
                    (1e-6 * (dom1 - dom0).abs().max(1e-12)).max(crate::curve::KNOT_IDENTITY_TOL);
                let periodic_tol = span_tol.max(crate::curve::KNOT_IDENTITY_TOL);
                let interior = |t: f64| t > dom0 + span_tol && t < dom1 - span_tol;
                // Only trust projections that actually land on the curve —
                // an edge whose vertices are off-curve keeps legacy behavior.
                let scale = 1.0 + p_start.length().max(p_end.length());
                let on_curve =
                    t_start.distance <= 1e-5 * scale && t_end.distance <= 1e-5 * scale;
                if on_curve
                    && (interior(t_start.u) || interior(t_end.u))
                    && (t_start.u - t_end.u).abs() > span_tol
                {
                    if authored_closed
                        && (t_start.u - t_end.u).abs() > periodic_tol
                        && curve_is_geometrically_closed(&curve)?
                    {
                        if let Ok(piece) = directed_periodic_curve_piece(
                            &curve,
                            t_start.u,
                            t_end.u,
                            same_sense,
                            periodic_tol,
                        ) {
                            return Ok(piece);
                        }
                    }
                    let (lo, hi, reversed) = if t_start.u <= t_end.u {
                        (t_start.u, t_end.u, false)
                    } else {
                        (t_end.u, t_start.u, true)
                    };
                    let mut piece = curve.clone();
                    if lo > dom0 + span_tol {
                        piece = piece.split(lo)?.1;
                    }
                    let [_, piece_end] = piece.domain()?;
                    if hi < piece_end - span_tol {
                        piece = piece.split(hi)?.0;
                    }
                    // The vertices are authoritative for direction here; the
                    // stored same_sense flag described the FULL curve.
                    return if reversed { piece.reversed() } else { Ok(piece) };
                }
            }
            return if same_sense {
                Ok(curve)
            } else {
                curve.reversed()
            };
        }
        if entity.has("LINE") {
            // The straight edge segment is exactly the chord between vertices.
            return make_line(p_start, p_end);
        }
        if let Some(args) = entity.find("POLYLINE") {
            // A POLYLINE is a chain of straight segments through CARTESIAN_POINTs
            // (some faceted exporters carry edges this way). Two points is just a
            // line; more become a degree-1 polyline curve.
            let point_refs = args
                .get(1)
                .ok_or("step_import: POLYLINE missing point list")?
                .as_list()?;
            let points = point_refs
                .iter()
                .map(|value| self.point(value.as_ref_id()?))
                .collect::<Result<Vec<_>, String>>()?;
            let curve = polyline_curve(&points)?;
            return if same_sense {
                Ok(curve)
            } else {
                curve.reversed()
            };
        }
        if let Some(args) = entity.find("CIRCLE") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            let radius = self.length(args[2].as_real()?);
            return build_conic_edge(&frame, radius, radius, p_start, p_end, same_sense, closed);
        }
        if let Some(args) = entity.find("ELLIPSE") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            let semi_major = self.length(args[2].as_real()?);
            let semi_minor = self.length(args[3].as_real()?);
            return build_conic_edge(
                &frame, semi_major, semi_minor, p_start, p_end, same_sense, closed,
            );
        }
        if let Some(args) = entity.find("HYPERBOLA") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            return build_hyperbola_edge(
                &frame,
                self.length(args[2].as_real()?),
                self.length(args[3].as_real()?),
                p_start,
                p_end,
                same_sense,
                closed,
            );
        }
        if let Some(args) = entity.find("PARABOLA") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            return build_parabola_edge(
                &frame,
                self.length(args[2].as_real()?),
                p_start,
                p_end,
                same_sense,
                closed,
            );
        }
        Err(format!(
            "step_import: unsupported curve entity #{id} ({})",
            entity_label(entity)
        ))
    }

    fn curve(&self, id: usize) -> Result<NurbsCurve, String> {
        let entity = self.get(id)?;
        // Rational complex form: (BOUNDED_CURVE()B_SPLINE_CURVE(...)
        //   B_SPLINE_CURVE_WITH_KNOTS(...)...RATIONAL_B_SPLINE_CURVE((weights))...)
        if let (Some(spline), Some(with_knots)) = (
            entity.find("B_SPLINE_CURVE"),
            entity.find("B_SPLINE_CURVE_WITH_KNOTS"),
        ) {
            let degree = spline[0].as_int()? as usize;
            let point_refs = spline[1].as_list()?;
            let (multiplicities, knot_values) = if with_knots.len() >= 2 {
                (with_knots[0].as_list()?, with_knots[1].as_list()?)
            } else {
                return Err("step_import: B_SPLINE_CURVE_WITH_KNOTS missing knots".into());
            };
            let weights = match entity.find("RATIONAL_B_SPLINE_CURVE") {
                Some(rational) => rational
                    .first()
                    .ok_or("step_import: RATIONAL_B_SPLINE_CURVE missing weights")?
                    .as_list()?
                    .iter()
                    .map(Value::as_real)
                    .collect::<Result<Vec<_>, _>>()?,
                None => vec![1.0; point_refs.len()],
            };
            return self.build_curve(degree, point_refs, multiplicities, knot_values, &weights);
        }
        // Simple non-rational form.
        if let Some(args) = entity.find("B_SPLINE_CURVE_WITH_KNOTS") {
            let degree = args[1].as_int()? as usize;
            let point_refs = args[2].as_list()?;
            let multiplicities = args[6].as_list()?;
            let knot_values = args[7].as_list()?;
            let weights = vec![1.0; point_refs.len()];
            return self.build_curve(degree, point_refs, multiplicities, knot_values, &weights);
        }
        // Analytic conic profiles used as SWEPT curves (a SURFACE_OF_LINEAR_
        // EXTRUSION / SURFACE_OF_REVOLUTION generatrix). Unlike an EDGE_CURVE
        // there are no bounding vertices to trim to, so build the FULL closed
        // conic over its whole period — exactly as `make_circle` builds a full
        // circle: the swept surface is periodic in the profile direction and
        // the owning face's boundary loops trim it via projected pcurves. This
        // mirrors the CIRCLE/ELLIPSE handling in `resolve_edge_curve`, sharing
        // the same `build_ellipse_arc` rational-NURBS conic construction (an
        // ellipse is the affine image of a unit circle; a == b degenerates to
        // a plain circle).
        if let Some(args) = entity.find("CIRCLE") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            let radius = self.length(args[2].as_real()?);
            return build_ellipse_arc(&frame, radius, radius, 0.0, std::f64::consts::TAU);
        }
        if let Some(args) = entity.find("ELLIPSE") {
            let frame = self.placement(args[1].as_ref_id()?)?;
            let semi_major = self.length(args[2].as_real()?);
            let semi_minor = self.length(args[3].as_real()?);
            return build_ellipse_arc(&frame, semi_major, semi_minor, 0.0, std::f64::consts::TAU);
        }
        Err(format!(
            "step_import: unsupported curve entity #{id} ({})",
            entity_label(entity)
        ))
    }

    fn build_curve(
        &self,
        degree: usize,
        point_refs: &[Value],
        multiplicities: &[Value],
        knot_values: &[Value],
        weights: &[f64],
    ) -> Result<NurbsCurve, String> {
        let mut knots = expand_knots(multiplicities, knot_values)?;
        let mut control_points = self.control_points(point_refs, weights)?;
        reparameterize_collapsed_bezier_domain(degree, &mut knots, &control_points)?;
        clamp_spline(degree, &mut knots, &mut control_points)?;
        NurbsCurve::new(degree, knots, control_points)
    }

    /// Resolve the carrier surface of an ADVANCED_FACE: analytic entities
    /// (PLANE/CYLINDRICAL/CONICAL/SPHERICAL/TOROIDAL) first — sized to cover the
    /// face via `samples` — otherwise the exporter's B_SPLINE forms.
    pub(super) fn surface_for_face(
        &self,
        id: usize,
        samples: &[Vec3],
        sphere_seam: Option<Vec3>,
    ) -> Result<NurbsSurface, String> {
        let entity = self.get(id)?;
        // OFFSET_SURFACE(name, basis_surface, distance, self_intersect): a copy
        // of the basis carrier shifted along its own normal by `distance`.
        // Resolve the basis with the existing surface machinery (recursing so an
        // analytic or B_SPLINE basis both work), then offset via the kernel's
        // offset_surface. The face's boundary `samples` lie on the OFFSET
        // surface; they still size the basis carrier correctly because the shift
        // is along the (locally perpendicular) surface normal, leaving the
        // sweep/axial extent used for sizing unchanged. `distance` is a
        // length — converted to mm like every other length read.
        if let Some(args) = entity.find("OFFSET_SURFACE") {
            let basis_ref = args
                .get(1)
                .ok_or("step_import: OFFSET_SURFACE missing basis surface")?
                .as_ref_id()?;
            let distance = self.length(
                args.get(2)
                    .ok_or("step_import: OFFSET_SURFACE missing distance")?
                    .as_real()?,
            );
            let basis = self.surface_for_face(basis_ref, samples, sphere_seam)?;
            // A temporary carrier face with same_sense = true, so
            // stable_face_normal returns the raw du×dv normal. offset_surface
            // shifts by -distance·N, so negate to follow STEP's convention of
            // offsetting +distance along that normal.
            let carrier = FaceRecord {
                id: 0,
                surface: basis,
                same_sense: true,
                loops: Vec::new(),
                name: None,
            };
            return offset_surface(&carrier, -distance, 0.0).map_err(String::from);
        }
        if let Some(surface) = self.analytic_surface(entity, samples, sphere_seam)? {
            return Ok(surface);
        }
        self.surface(id)
    }

    fn surface(&self, id: usize) -> Result<NurbsSurface, String> {
        let entity = self.get(id)?;
        // Rational complex form.
        if let (Some(spline), Some(with_knots)) = (
            entity.find("B_SPLINE_SURFACE"),
            entity.find("B_SPLINE_SURFACE_WITH_KNOTS"),
        ) {
            let degree_u = spline[0].as_int()? as usize;
            let degree_v = spline[1].as_int()? as usize;
            let grid = spline[2].as_list()?;
            let u_mult = with_knots[0].as_list()?;
            let v_mult = with_knots[1].as_list()?;
            let u_knots = with_knots[2].as_list()?;
            let v_knots = with_knots[3].as_list()?;
            let weights = entity.find("RATIONAL_B_SPLINE_SURFACE").map(|rational| {
                rational
                    .first()
                    .ok_or("step_import: RATIONAL_B_SPLINE_SURFACE missing weights".to_string())
            });
            let weight_grid = match weights {
                Some(rows) => Some(rows?.as_list()?),
                None => None,
            };
            return self.build_surface(
                degree_u,
                degree_v,
                grid,
                u_mult,
                v_mult,
                u_knots,
                v_knots,
                weight_grid,
            );
        }
        // Simple non-rational form.
        if let Some(args) = entity.find("B_SPLINE_SURFACE_WITH_KNOTS") {
            let degree_u = args[1].as_int()? as usize;
            let degree_v = args[2].as_int()? as usize;
            let grid = args[3].as_list()?;
            let u_mult = args[8].as_list()?;
            let v_mult = args[9].as_list()?;
            let u_knots = args[10].as_list()?;
            let v_knots = args[11].as_list()?;
            return self.build_surface(
                degree_u, degree_v, grid, u_mult, v_mult, u_knots, v_knots, None,
            );
        }
        Err(format!(
            "step_import: unsupported surface entity #{id} ({})",
            entity_label(entity)
        ))
    }

    #[allow(clippy::too_many_arguments)]
    fn build_surface(
        &self,
        degree_u: usize,
        degree_v: usize,
        grid: &[Value],
        u_mult: &[Value],
        v_mult: &[Value],
        u_knots: &[Value],
        v_knots: &[Value],
        weight_grid: Option<&[Value]>,
    ) -> Result<NurbsSurface, String> {
        let mut knots_u = expand_knots(u_mult, u_knots)?;
        let mut knots_v = expand_knots(v_mult, v_knots)?;
        let mut control_points = grid
            .iter()
            .enumerate()
            .map(|(row_index, row)| {
                let refs = row.as_list()?;
                let weights = match weight_grid {
                    Some(rows) => rows
                        .get(row_index)
                        .ok_or("step_import: weight grid shape mismatch")?
                        .as_list()?
                        .iter()
                        .map(Value::as_real)
                        .collect::<Result<Vec<_>, _>>()?,
                    None => vec![1.0; refs.len()],
                };
                self.control_points(refs, &weights)
            })
            .collect::<Result<Vec<_>, String>>()?;
        // Unclamped/periodic exporters (e.g. Onshape uniform B-splines):
        // clamp each direction shape-exactly before validation. u operates on
        // whole rows; v on columns via transpose.
        clamp_spline(degree_u, &mut knots_u, &mut control_points)?;
        let mut columns: Vec<Vec<Vec4>> = (0..control_points[0].len())
            .map(|j| control_points.iter().map(|row| row[j]).collect())
            .collect();
        clamp_spline(degree_v, &mut knots_v, &mut columns)?;
        let control_points: Vec<Vec<Vec4>> = (0..columns[0].len())
            .map(|i| columns.iter().map(|column| column[i]).collect())
            .collect();
        NurbsSurface::new(degree_u, degree_v, knots_u, knots_v, control_points)
    }
}

fn entity_label(entity: &Entity) -> String {
    entity
        .records
        .iter()
        .map(|(name, _)| name.as_str())
        .collect::<Vec<_>>()
        .join("|")
}

/// Expand a (multiplicities, distinct-knots) pair into a full clamped knot
/// vector, e.g. mults=[3,2,3] knots=[0,0.5,1] → [0,0,0,0.5,0.5,1,1,1].
/// Element of a B-spline control net that Boehm knot insertion can blend:
/// a homogeneous control point (curve) or a full row/column of them
/// (surface direction). Weights are premultiplied, so the plain linear
/// combination is exact for rationals.
pub(super) trait SplineElement: Clone {
    fn lerp(a: &Self, b: &Self, alpha: f64) -> Self;
}

impl SplineElement for Vec4 {
    fn lerp(a: &Self, b: &Self, alpha: f64) -> Self {
        Vec4 {
            x: a.x + (b.x - a.x) * alpha,
            y: a.y + (b.y - a.y) * alpha,
            z: a.z + (b.z - a.z) * alpha,
            w: a.w + (b.w - a.w) * alpha,
        }
    }
}

impl SplineElement for Vec<Vec4> {
    fn lerp(a: &Self, b: &Self, alpha: f64) -> Self {
        a.iter()
            .zip(b.iter())
            .map(|(left, right)| SplineElement::lerp(left, right, alpha))
            .collect()
    }
}

/// Boehm single-knot insertion (Piegl–Tiller A5.1, r=1): shape-exact.
/// `t` must lie inside the knot range with existing multiplicity `s < p`.
fn insert_knot<T: SplineElement>(degree: usize, knots: &mut Vec<f64>, points: &mut Vec<T>, t: f64) {
    let p = degree;
    // Span k: last index with knots[k] <= t (t is never past the last knot here).
    let mut k = 0;
    for (index, knot) in knots.iter().enumerate() {
        if *knot <= t + crate::curve::KNOT_IDENTITY_TOL {
            k = index;
        }
    }
    let s = knots
        .iter()
        .filter(|knot| (**knot - t).abs() <= crate::curve::KNOT_IDENTITY_TOL)
        .count();
    let mut fresh: Vec<T> = Vec::with_capacity(points.len() + 1);
    fresh.extend_from_slice(&points[..=k.saturating_sub(p)]);
    for i in (k - p + 1)..=(k - s) {
        let denom = knots[i + p] - knots[i];
        let alpha = if denom.abs() <= crate::curve::KNOT_IDENTITY_TOL {
            0.0
        } else {
            (t - knots[i]) / denom
        };
        fresh.push(SplineElement::lerp(&points[i - 1], &points[i], alpha));
    }
    fresh.extend_from_slice(&points[k - s..]);
    *points = fresh;
    knots.insert(k + 1, t);
}

/// Clamp the START of a possibly-unclamped B-spline: insert the domain-start
/// knot until its multiplicity reaches `degree`, then trim the knots and
/// control net to the visible domain and write the fully-clamped boundary.
/// No-op when the start is already clamped.
fn clamp_spline_start<T: SplineElement>(
    degree: usize,
    knots: &mut Vec<f64>,
    points: &mut Vec<T>,
) -> Result<(), String> {
    let p = degree;
    if knots.len() < 2 * (p + 1) || points.len() + p + 1 != knots.len() {
        return Ok(()); // let the constructor report the real shape error
    }
    let u0 = knots[p];
    let already = (0..=p).all(|i| (knots[i] - u0).abs() <= crate::curve::KNOT_IDENTITY_TOL);
    if already {
        return Ok(());
    }
    let mut s = knots
        .iter()
        .filter(|knot| (**knot - u0).abs() <= crate::curve::KNOT_IDENTITY_TOL)
        .count()
        .min(p + 1);
    while s < p {
        insert_knot(p, knots, points, u0);
        s += 1;
    }
    let f = knots
        .iter()
        .position(|knot| (*knot - u0).abs() <= crate::curve::KNOT_IDENTITY_TOL)
        .ok_or("step_import: clamp lost its domain knot")?;
    let keep_from = (f + s).checked_sub(p + 1).ok_or("step_import: clamp trim underflow")?;
    let mut fresh_knots = vec![u0; p + 1];
    fresh_knots.extend_from_slice(&knots[f + s..]);
    *knots = fresh_knots;
    *points = points[keep_from..].to_vec();
    Ok(())
}

/// Clamp BOTH ends of a B-spline knot vector (unclamped/periodic exporters:
/// e.g. Onshape emits uniform unclamped surfaces). Shape-exact on the visible
/// domain; already-clamped input is untouched.
pub(super) fn clamp_spline<T: SplineElement>(
    degree: usize,
    knots: &mut Vec<f64>,
    points: &mut Vec<T>,
) -> Result<(), String> {
    clamp_spline_start(degree, knots, points)?;
    // Clamp the end by symmetry: reverse the parameterization and re-clamp.
    knots.reverse();
    for knot in knots.iter_mut() {
        *knot = -*knot;
    }
    points.reverse();
    clamp_spline_start(degree, knots, points)?;
    knots.reverse();
    for knot in knots.iter_mut() {
        *knot = -*knot;
    }
    points.reverse();
    Ok(())
}

/// Join `second` after `first` with a C0 (multiplicity-degree) knot at the
/// junction; `second`'s parameterization is shifted to continue seamlessly.
/// Both inputs must be clamped, same degree, and share the junction point.
pub(super) fn concatenate_curves_c0(first: &NurbsCurve, second: &NurbsCurve) -> Result<NurbsCurve, String> {
    if first.degree != second.degree {
        return Err("concatenate_curves_c0: degree mismatch".into());
    }
    let p = first.degree;
    let [_, first_end] = first.domain()?;
    let [second_start, _] = second.domain()?;
    let junction_a = first.evaluate(first_end)?;
    let junction_b = second.evaluate(second_start)?;
    if junction_a.sub(junction_b).length() > 1e-6 {
        return Err("concatenate_curves_c0: junction points differ".into());
    }
    let shift = first_end - second_start;
    let mut knots = first.knots[..first.knots.len() - 1].to_vec();
    knots.extend(second.knots[p + 1..].iter().map(|knot| knot + shift));
    let mut control_points = first.control_points.clone();
    control_points.extend_from_slice(&second.control_points[1..]);
    NurbsCurve::new(p, knots, control_points)
}

pub(super) fn curve_is_geometrically_closed(curve: &NurbsCurve) -> Result<bool, String> {
    let [domain_start, domain_end] = curve.domain()?;
    let start = curve.evaluate(domain_start)?;
    let end = curve.evaluate(domain_end)?;
    let finite = |point: Vec3| point.x.is_finite() && point.y.is_finite() && point.z.is_finite();
    if !finite(start) || !finite(end) {
        return Ok(false);
    }
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for control in &curve.control_points {
        if !control.w.is_finite() || control.w.abs() <= 1e-15 {
            return Ok(false);
        }
        let point = Vec3::new(
            control.x / control.w,
            control.y / control.w,
            control.z / control.w,
        );
        if !finite(point) {
            return Ok(false);
        }
        lo.x = lo.x.min(point.x);
        lo.y = lo.y.min(point.y);
        lo.z = lo.z.min(point.z);
        hi.x = hi.x.max(point.x);
        hi.y = hi.y.max(point.y);
        hi.z = hi.z.max(point.z);
    }
    let extent = hi.sub(lo).length();
    if !extent.is_finite() {
        return Ok(false);
    }
    Ok(start.sub(end).length() <= 1e-9 * extent.max(1.0))
}

fn increasing_periodic_curve_piece(
    curve: &NurbsCurve,
    from: f64,
    to: f64,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let [domain_start, domain_end] = curve.domain()?;
    let parameter_tolerance = tolerance.max(crate::curve::KNOT_IDENTITY_TOL);
    let extract = |lo: f64, hi: f64| -> Result<NurbsCurve, String> {
        let mut piece = curve.clone();
        if lo > domain_start + parameter_tolerance {
            piece = piece.split(lo)?.1;
        }
        let [_, piece_end] = piece.domain()?;
        if hi < piece_end - parameter_tolerance {
            piece = piece.split(hi)?.0;
        }
        Ok(piece)
    };
    if from <= to {
        return extract(from, to);
    }
    let tail = (domain_end - from > parameter_tolerance)
        .then(|| extract(from, domain_end))
        .transpose()?;
    let head = (to - domain_start > parameter_tolerance)
        .then(|| extract(domain_start, to))
        .transpose()?;
    match (tail, head) {
        (Some(tail), Some(head)) => concatenate_curves_c0(&tail, &head),
        (Some(piece), None) | (None, Some(piece)) => Ok(piece),
        (None, None) => Err("step_import: periodic curve subrange is empty".into()),
    }
}

pub(super) fn directed_periodic_curve_piece(
    curve: &NurbsCurve,
    start: f64,
    end: f64,
    same_sense: bool,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    if same_sense {
        increasing_periodic_curve_piece(curve, start, end, tolerance)
    } else {
        increasing_periodic_curve_piece(curve, end, start, tolerance)?.reversed()
    }
}

pub(super) fn expand_knots(multiplicities: &[Value], knot_values: &[Value]) -> Result<Vec<f64>, String> {
    if multiplicities.len() != knot_values.len() {
        return Err("step_import: knot multiplicity/value length mismatch".into());
    }
    let mut knots = Vec::new();
    for (mult, value) in multiplicities.iter().zip(knot_values) {
        let count = mult.as_int()?;
        if count < 1 {
            return Err("step_import: non-positive knot multiplicity".into());
        }
        let knot = value.as_real()?;
        for _ in 0..count {
            knots.push(knot);
        }
    }
    Ok(knots)
}

/// Repair a vendor Bezier whose parameter interval was serialized with both
/// endpoint knots equal even though its control polygon has real extent.
///
/// A Bezier's knot magnitudes only parameterize the curve; changing its
/// clamped domain from `[base, base]` to `[base, base + 1]` preserves the exact
/// rational geometry. Restrict this to the unambiguous Bezier layout so a
/// collapsed span inside a general B-spline is never invented or removed.
pub(super) fn reparameterize_collapsed_bezier_domain(
    degree: usize,
    knots: &mut [f64],
    control_points: &[Vec4],
) -> Result<bool, String> {
    let order = degree + 1;
    if control_points.len() != order || knots.len() != 2 * order {
        return Ok(false);
    }
    let start = knots[degree];
    let end = knots[knots.len() - 1 - degree];
    if (end - start).abs() > crate::curve::KNOT_IDENTITY_TOL {
        return Ok(false);
    }
    let first = control_points[0].point()?;
    let mut has_extent = false;
    for control in &control_points[1..] {
        if control.point()?.sub(first).length_squared() > 0.0 {
            has_extent = true;
            break;
        }
    }
    if !has_extent {
        return Ok(false);
    }
    for knot in &mut knots[order..] {
        *knot = start + 1.0;
    }
    Ok(true)
}
