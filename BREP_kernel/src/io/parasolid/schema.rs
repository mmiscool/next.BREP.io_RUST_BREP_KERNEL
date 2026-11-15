//! The base schema, `SCH_13006`, that embedded-schema files describe their
//! layouts against (Parasolid XT Format Reference §2.1.2).
//!
//! The base schema file is not published. This table is the reference's
//! chapter 5 node structures with the base layout restored where a later
//! schema changed it: a later writer's delta (Copy, Delete, Insert, Append)
//! against the base gives the base's field count and the position of every
//! field it kept, and the reference's field tables give names and types. A
//! field the base had but every seen writer deletes is `Unknown`: its type is
//! not known, so a file that copies it is refused rather than misread.

/// The base schema number embedded deltas are written against.
pub const BASE_SCHEMA: &str = "13006";

/// How a field's values are written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    /// `d`, `u`, `n`, `w`, `p`, `t`: integers, bytes, shorts, unicode
    /// characters, pointer indices and tags.
    Int,
    /// `f`.
    Real,
    /// `v` (vector) and `h` (hvec; only its position is written).
    Vector,
    /// `i`: two reals.
    Interval,
    /// `b`: six reals.
    Box,
    /// `c` and `l`: one character; logicals are `T` or `F`.
    Char,
    /// A base field deleted by every writer seen; its type is not known.
    Unknown,
}

/// Scalar, variable-length (the node's last field) or a fixed array.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Count {
    Scalar,
    Variable,
    Fixed(usize),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FieldDef {
    pub name: String,
    pub kind: Kind,
    pub count: Count,
}

/// The kind a schema type code names (§2.1.4).
pub fn kind_of(code: u8) -> Option<Kind> {
    Some(match code {
        b'd' | b'u' | b'n' | b'w' | b'p' | b't' => Kind::Int,
        b'f' => Kind::Real,
        b'v' | b'h' => Kind::Vector,
        b'i' => Kind::Interval,
        b'b' => Kind::Box,
        b'c' | b'l' => Kind::Char,
        b'X' => Kind::Unknown,
        _ => return None,
    })
}

/// The schema number Parasolid V30.1 writes. Files in it may omit their
/// schema deltas (`SCH_3001000_30100`, as Onshape writes): then every node
/// type is laid out by [`V30_DELTAS`].
pub const V30_SCHEMA: &str = "30100";

/// The schema SolidWorks 2022 (Parasolid 33.1) writes its transmit body in,
/// with no deltas (`SCH_2500000_25001`).
pub const V25_SCHEMA: &str = "25001";

/// Parasolid V12's schema, older than the base and so never written with
/// deltas (`SCH_1200000_12006`).
pub const V12_SCHEMA: &str = "12006";

/// The schemas whose files may carry no layout description and are still
/// read, by the layouts stored here.
pub const STORED_SCHEMAS: [&str; 3] = [V12_SCHEMA, V25_SCHEMA, V30_SCHEMA];

/// A node type's layout in a stored schema, as an edit script against the
/// base in the form embedded deltas take (§2.1.2.2), or `None` when no file
/// in that schema has shown the type (it is then refused by name).
pub fn stored_delta(schema: &str, node_type: u16) -> Option<&'static str> {
    let find = |table: &[(u16, &'static str)]| table.iter().find(|(t, _)| *t == node_type).map(|(_, d)| *d);
    match schema {
        V30_SCHEMA => v30_delta(node_type),
        V25_SCHEMA => find(V25_DELTAS).or_else(|| {
            // A type V30.1 lays out as the base has the base layout in every
            // schema between them; the types V30.1 changed are V25's only as
            // far as the reported file showed them.
            v30_delta(node_type).filter(|d| *d == "255 " || V25_AS_V30.contains(&node_type))
        }),
        V12_SCHEMA => find(V12_DELTAS),
        _ => None,
    }
}

/// Schema 25001 where it is not V30.1's: the layouts the reported
/// SolidWorks file (18,837 nodes) reads end to end by. BODY has V30.1's
/// `owner` and the trailing index/schema-map fields but not the mesh and
/// polyline pointers (its six trailing fields are all 0 there, so their
/// names follow V30.1's order); INTERSECTION has no `intersection_data` and
/// LIMIT no `term_use` (both are the base's).
#[rustfmt::skip]
const V25_DELTAS: &[(u16, &str)] = &[
    (12, "29 CCCCCCCCCCCCCDI5 owner12 0 CCCCCCCCCA16 index_map_offset0 0 1 dA9 index_map82 0 A17 node_id_index_map82 0 A20 schema_embedding_map82 0 A5 child12 0 A14 lowest_node_id0 0 1 dZ"),
    (38, "255 "),
    (41, "255 "),
];

/// Types V30.1 changed from the base whose V30.1 layout the reported schema
/// 25001 file reads by: REGION, LIST and POINTER_LIS_BLOCK.
const V25_AS_V30: [u16; 3] = [19, 70, 74];

/// Schema 12006: the layouts the reported V12 file (369 nodes) reads end to
/// end by. Topology, points, lines, circles, planes, cylinders, attributes
/// and their values are the base's. ATTRIB_DEF has no `field_names` and 13
/// legal owners, and its field types run straight on to the next node as
/// characters; LIST has its type where the base's first deleted field sits,
/// a pointer where its second sits, then a finger block, finger index and
/// notransmit flag (the file's list: block 369 three times, finger index 1).
#[rustfmt::skip]
const V12_DELTAS: &[(u16, &str)] = &[
    (12, "255 "), (13, "255 "), (14, "255 "), (15, "255 "), (16, "255 "), (17, "255 "), (18, "255 "), (19, "255 "),
    (29, "255 "), (30, "255 "), (31, "255 "), (50, "255 "), (51, "255 "),
    (70, "12 CCCCDI9 list_type0 0 1 uCCDI10 last_block1012 0 CA12 finger_block1012 0 A12 finger_index0 0 1 dA10 notransmit0 0 1 lZ"),
    (74, "255 "), (79, "255 "),
    (80, "6 CCCCDDI12 legal_owners0 13 1 lDI6 fields0 1 1 cTZ"),
    (81, "255 "), (84, "255 "),
];

/// Schema 30100's layout of a node type, as the edit script (or, for a type
/// the base lacks, the whole description) that files written with embedded
/// deltas (`SCH_3001168_30100_13006`) carry at the type's first occurrence,
/// verbatim. Harvested from 2000 ABC corpus files: every file gave each type
/// the same script. A type no corpus file showed is absent, so a file that
/// holds one is refused rather than read with a guessed layout.
pub fn v30_delta(node_type: u16) -> Option<&'static str> {
    V30_DELTAS.iter().find(|(t, _)| *t == node_type).map(|(_, d)| *d)
}

#[rustfmt::skip]
const V30_DELTAS: &[(u16, &str)] = &[
    (12, "34 CCCCCCI4 mesh1006 0 I8 polyline1008 0 CCCCCCCDI5 owner1040 0 CCCCCCI13 boundary_mesh1006 0 I17 boundary_polyline1008 0 CCCA16 index_map_offset0 0 1 dA9 index_map82 0 A17 node_id_index_map82 0 A20 schema_embedding_map82 0 A5 child12 0 A14 lowest_node_id0 0 1 dA16 mesh_offset_data206 0 Z"),
    (13, "255 "),
    (14, "255 "),
    (15, "255 "),
    (16, "255 "),
    (17, "255 "),
    (18, "255 "),
    (19, "8 CCCCCCCA5 owner12 0 Z"),
    (29, "255 "),
    (30, "255 "),
    (31, "255 "),
    (32, "255 "),
    (38, "12 CCCCCCCCCCCA17 intersection_data204 0 Z"),
    (40, "255 "),
    (41, "3 CI8 term_use0 0 1 cCZ"),
    (45, "255 "),
    (50, "255 "),
    (51, "255 "),
    (52, "255 "),
    (53, "255 "),
    (54, "255 "),
    (56, "255 "),
    (59, "255 "),
    (60, "255 "),
    (67, "255 "),
    (68, "255 "),
    (70, "11 CI9 list_type0 0 1 uI10 notransmit0 0 1 lCCCDCCDI12 finger_index0 0 1 dI12 finger_block1012 0 CZ"),
    (74, "4 CI16 index_map_offset0 0 1 dCCZ"),
    (79, "255 "),
    (80, "255 "),
    (81, "255 "),
    (82, "255 "),
    (83, "255 "),
    (84, "255 "),
    (89, "255 "),
    (98, "255 "),
    (124, "255 "),
    (125, "255 "),
    (126, "255 "),
    (127, "255 "),
    (128, "255 "),
    (133, "255 "),
    (134, "255 "),
    (135, "255 "),
    (136, "255 "),
    (137, "255 "),
    (141, "255 "),
    (163, "255 "),
    (184, "255 "),
    (204, "2 17 INTERSECTION_DATA17 Intersection data7 uv_type0 0 1 u6 values0 1 1 fT"),
];

/// A base node type's name and fields.
pub fn base(node_type: u16) -> Option<(&'static str, Vec<FieldDef>)> {
    let (_, name, fields) = BASE.iter().find(|(t, _, _)| *t == node_type)?;
    let fields = fields
        .iter()
        .map(|&(name, code, n)| FieldDef {
            name: name.to_string(),
            kind: kind_of(code).expect("base schema type codes are known"),
            count: match n {
                0 => Count::Scalar,
                1 => Count::Variable,
                n => Count::Fixed(n),
            },
        })
        .collect();
    Some((name, fields))
}

/// (node type, name, fields as (name, type code, elements: 0 scalar, 1
/// variable, n fixed)).
type BaseNode = (u16, &'static str, &'static [(&'static str, u8, usize)]);

#[rustfmt::skip]
const BASE: &[BaseNode] = &[
    (10, "ASSEMBLY", &[("highest_node_id", b'd', 0), ("attributes_features", b'p', 0), ("attribute_chains", b'p', 0), ("list", b'p', 0), ("lattice", b'p', 0), ("surface", b'p', 0), ("curve", b'p', 0), ("point", b'p', 0), ("mesh", b'p', 0), ("polyline", b'p', 0), ("key", b'p', 0), ("res_size", b'f', 0), ("res_linear", b'f', 0), ("ref_instance", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("state", b'u', 0), ("owner", b'p', 0), ("type", b'u', 0), ("sub_instance", b'p', 0), ("mesh_offset_data", b'p', 0)]),
    (11, "INSTANCE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("type", b'u', 0), ("part", b'p', 0), ("transform", b'p', 0), ("assembly", b'p', 0), ("next_in_part", b'p', 0), ("prev_in_part", b'p', 0), ("next_of_part", b'p', 0), ("prev_of_part", b'p', 0)]),
    (12, "BODY", &[("highest_node_id", b'd', 0), ("attributes_features", b'p', 0), ("attribute_chains", b'p', 0), ("surface", b'p', 0), ("curve", b'p', 0), ("point", b'p', 0), ("key", b'p', 0), ("res_size", b'f', 0), ("res_linear", b'f', 0), ("ref_instance", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("state", b'u', 0), ("base_owner", b'p', 0), ("body_type", b'u', 0), ("nom_geom_state", b'u', 0), ("shell", b'p', 0), ("boundary_surface", b'p', 0), ("boundary_curve", b'p', 0), ("boundary_point", b'p', 0), ("region", b'p', 0), ("edge", b'p', 0), ("vertex", b'p', 0)]),
    (13, "SHELL", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("body", b'p', 0), ("next", b'p', 0), ("face", b'p', 0), ("edge", b'p', 0), ("vertex", b'p', 0), ("region", b'p', 0), ("front_face", b'p', 0)]),
    (14, "FACE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("tolerance", b'f', 0), ("next", b'p', 0), ("previous", b'p', 0), ("loop", b'p', 0), ("shell", b'p', 0), ("surface", b'p', 0), ("sense", b'c', 0), ("next_on_surface", b'p', 0), ("previous_on_surface", b'p', 0), ("next_front", b'p', 0), ("previous_front", b'p', 0), ("front_shell", b'p', 0)]),
    (15, "LOOP", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("halfedge", b'p', 0), ("face", b'p', 0), ("next", b'p', 0)]),
    (16, "EDGE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("tolerance", b'f', 0), ("halfedge", b'p', 0), ("previous", b'p', 0), ("next", b'p', 0), ("curve", b'p', 0), ("next_on_curve", b'p', 0), ("previous_on_curve", b'p', 0), ("owner", b'p', 0)]),
    (17, "HALFEDGE", &[("attributes_features", b'p', 0), ("loop", b'p', 0), ("forward", b'p', 0), ("backward", b'p', 0), ("vertex", b'p', 0), ("other", b'p', 0), ("edge", b'p', 0), ("curve", b'p', 0), ("next_at_vx", b'p', 0), ("sense", b'c', 0)]),
    (18, "VERTEX", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("halfedge", b'p', 0), ("previous", b'p', 0), ("next", b'p', 0), ("point", b'p', 0), ("tolerance", b'f', 0), ("owner", b'p', 0)]),
    (19, "REGION", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("body", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("shell", b'p', 0), ("type", b'c', 0)]),
    (29, "POINT", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("pvec", b'v', 0)]),
    (30, "LINE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("pvec", b'v', 0), ("direction", b'v', 0)]),
    (31, "CIRCLE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("centre", b'v', 0), ("normal", b'v', 0), ("x_axis", b'v', 0), ("radius", b'f', 0)]),
    (32, "ELLIPSE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("centre", b'v', 0), ("normal", b'v', 0), ("x_axis", b'v', 0), ("major_radius", b'f', 0), ("minor_radius", b'f', 0)]),
    (38, "INTERSECTION", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("surface", b'p', 2), ("chart", b'p', 0), ("start", b'p', 0), ("end", b'p', 0)]),
    (40, "CHART", &[("base_parameter", b'f', 0), ("base_scale", b'f', 0), ("chart_count", b'd', 0), ("chordal_error", b'f', 0), ("angular_error", b'f', 0), ("parameter_error", b'f', 2), ("hvec", b'h', 1)]),
    (41, "LIMIT", &[("type", b'c', 0), ("hvec", b'h', 1)]),
    (45, "BSPLINE_VERTICES", &[("vertices", b'f', 1)]),
    (50, "PLANE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("pvec", b'v', 0), ("normal", b'v', 0), ("x_axis", b'v', 0)]),
    (51, "CYLINDER", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("pvec", b'v', 0), ("axis", b'v', 0), ("radius", b'f', 0), ("x_axis", b'v', 0)]),
    (52, "CONE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("pvec", b'v', 0), ("axis", b'v', 0), ("radius", b'f', 0), ("sin_half_angle", b'f', 0), ("cos_half_angle", b'f', 0), ("x_axis", b'v', 0)]),
    (53, "SPHERE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("centre", b'v', 0), ("radius", b'f', 0), ("axis", b'v', 0), ("x_axis", b'v', 0)]),
    (54, "TORUS", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("centre", b'v', 0), ("axis", b'v', 0), ("major_radius", b'f', 0), ("minor_radius", b'f', 0), ("x_axis", b'v', 0)]),
    (56, "BLENDED_EDGE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("blend_type", b'c', 0), ("surface", b'p', 2), ("spine", b'p', 0), ("range", b'f', 2), ("thumb_weight", b'f', 2), ("boundary", b'p', 2), ("start", b'p', 0), ("end", b'p', 0)]),
    (59, "BLEND_BOUND", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("boundary", b'n', 0), ("blend", b'p', 0)]),
    (60, "OFFSET_SURF", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("check", b'c', 0), ("true_offset", b'l', 0), ("surface", b'p', 0), ("offset", b'f', 0), ("scale", b'f', 0)]),
    (67, "SWEPT_SURF", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("section", b'p', 0), ("sweep", b'v', 0), ("scale", b'f', 0)]),
    (68, "SPUN_SURF", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("profile", b'p', 0), ("base", b'v', 0), ("axis", b'v', 0), ("start", b'v', 0), ("end", b'v', 0), ("start_param", b'f', 0), ("end_param", b'f', 0), ("x_axis", b'v', 0), ("scale", b'f', 0)]),
    (70, "LIST", &[("node_id", b'd', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("base_del1", b'X', 0), ("list_length", b'd', 0), ("block_length", b'd', 0), ("base_del2", b'X', 0), ("list_block", b'p', 0)]),
    (74, "POINTER_LIS_BLOCK", &[("n_entries", b'd', 0), ("next_block", b'p', 0), ("entries", b'p', 1)]),
    (79, "ATT_DEF_ID", &[("string", b'c', 1)]),
    (80, "ATTRIB_DEF", &[("next", b'p', 0), ("identifier", b'p', 0), ("type_id", b'd', 0), ("actions", b'u', 8), ("field_names", b'p', 0), ("legal_owners", b'l', 14), ("fields", b'u', 1)]),
    (81, "ATTRIBUTE", &[("node_id", b'd', 0), ("definition", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("next_of_type", b'p', 0), ("previous_of_type", b'p', 0), ("fields", b'p', 1)]),
    (82, "INT_VALUES", &[("values", b'd', 1)]),
    (83, "REAL_VALUES", &[("values", b'f', 1)]),
    (84, "CHAR_VALUES", &[("values", b'c', 1)]),
    (85, "POINT_VALUES", &[("values", b'v', 1)]),
    (86, "VECTOR_VALUES", &[("values", b'v', 1)]),
    (87, "AXIS_VALUES", &[("values", b'v', 1)]),
    (88, "TAG_VALUES", &[("values", b't', 1)]),
    (89, "DIRECTION_VALUES", &[("values", b'v', 1)]),
    (90, "FEATURE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("type", b'u', 0), ("first_member", b'p', 0)]),
    (91, "MEMBER_OF_FEATURE", &[("dummy_node_id", b'd', 0), ("owning_feature", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("next_member", b'p', 0), ("previous_member", b'p', 0)]),
    (98, "UNICODE_VALUES", &[("values", b'w', 1)]),
    (99, "FIELD_NAMES", &[("names", b'p', 1)]),
    (100, "TRANSFORM", &[("node_id", b'd', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("rotation_matrix", b'f', 9), ("translation_vector", b'v', 0), ("scale", b'f', 0), ("flag", b'd', 0), ("perspective_vector", b'v', 0)]),
    (101, "WORLD", &[("assembly", b'p', 0), ("attribute", b'p', 0), ("body", b'p', 0), ("transform", b'p', 0), ("lattice", b'p', 0), ("surface", b'p', 0), ("curve", b'p', 0), ("point", b'p', 0), ("mesh", b'p', 0), ("polyline", b'p', 0), ("alive", b'l', 0), ("attrib_def", b'p', 0), ("attdef_list", b'p', 0), ("highest_id", b'd', 0), ("current_id", b'd', 0), ("index_map", b'p', 0), ("schema_embedding_map", b'p', 0), ("mesh_offset_data", b'p', 0)]),
    (120, "PE_SURF", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("type", b'c', 0), ("data", b'p', 0), ("tf", b'p', 0), ("internal_geom", b'p', 1)]),
    (121, "INT_PE_DATA", &[("geom_type", b'd', 0), ("real_array", b'p', 0), ("int_array", b'p', 0)]),
    (122, "EXT_PE_DATA", &[("key", b'p', 0), ("real_array", b'p', 0), ("int_array", b'p', 0)]),
    (124, "B_SURFACE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("nurbs", b'p', 0), ("data", b'p', 0)]),
    (125, "SURFACE_DATA", &[("original_uint", b'i', 0), ("original_vint", b'i', 0), ("extended_uint", b'i', 0), ("extended_vint", b'i', 0), ("self_int", b'u', 0), ("original_u_start", b'c', 0), ("original_u_end", b'c', 0), ("original_v_start", b'c', 0), ("original_v_end", b'c', 0), ("extended_u_start", b'c', 0), ("extended_u_end", b'c', 0), ("extended_v_start", b'c', 0), ("extended_v_end", b'c', 0), ("analytic_form_type", b'c', 0), ("swept_form_type", b'c', 0), ("spun_form_type", b'c', 0), ("blend_form_type", b'c', 0), ("analytic_form", b'p', 0), ("swept_form", b'p', 0), ("spun_form", b'p', 0), ("blend_form", b'p', 0)]),
    (126, "NURBS_SURF", &[("u_periodic", b'l', 0), ("v_periodic", b'l', 0), ("u_degree", b'n', 0), ("v_degree", b'n', 0), ("n_u_vertices", b'd', 0), ("n_v_vertices", b'd', 0), ("u_knot_type", b'u', 0), ("v_knot_type", b'u', 0), ("n_u_knots", b'd', 0), ("n_v_knots", b'd', 0), ("rational", b'l', 0), ("u_closed", b'l', 0), ("v_closed", b'l', 0), ("surface_form", b'u', 0), ("vertex_dim", b'n', 0), ("bspline_vertices", b'p', 0), ("u_knot_mult", b'p', 0), ("v_knot_mult", b'p', 0), ("u_knots", b'p', 0), ("v_knots", b'p', 0)]),
    (127, "KNOT_MULT", &[("mult", b'n', 1)]),
    (128, "KNOT_SET", &[("knots", b'f', 1)]),
    (130, "PE_CURVE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("type", b'c', 0), ("data", b'p', 0), ("tf", b'p', 0), ("internal_geom", b'p', 1)]),
    (133, "TRIMMED_CURVE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("basis_curve", b'p', 0), ("point_1", b'v', 0), ("point_2", b'v', 0), ("parm_1", b'f', 0), ("parm_2", b'f', 0)]),
    (134, "B_CURVE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("nurbs", b'p', 0), ("data", b'p', 0)]),
    (135, "CURVE_DATA", &[("self_int", b'u', 0), ("analytic_form", b'p', 0)]),
    (136, "NURBS_CURVE", &[("degree", b'n', 0), ("n_vertices", b'd', 0), ("vertex_dim", b'n', 0), ("n_knots", b'd', 0), ("knot_type", b'u', 0), ("periodic", b'l', 0), ("closed", b'l', 0), ("rational", b'l', 0), ("curve_form", b'u', 0), ("bspline_vertices", b'p', 0), ("knot_mult", b'p', 0), ("knots", b'p', 0)]),
    (137, "SP_CURVE", &[("node_id", b'd', 0), ("attributes_features", b'p', 0), ("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("geometric_owner", b'p', 0), ("sense", b'c', 0), ("surface", b'p', 0), ("b_curve", b'p', 0), ("original", b'p', 0), ("tolerance_to_original", b'f', 0)]),
    (141, "GEOMETRIC_OWNER", &[("owner", b'p', 0), ("next", b'p', 0), ("previous", b'p', 0), ("shared_geometry", b'p', 0)]),
    (163, "HELIX_SU_FORM", &[("axis_pt", b'v', 0), ("axis_dir", b'v', 0), ("hand", b'c', 0), ("turns", b'i', 0), ("pitch", b'f', 0), ("gap", b'f', 0), ("tol", b'f', 0)]),
    (176, "PART_XMT_BLOCK", &[("n_entries", b'd', 0), ("index_map_offset", b'd', 0), ("index_map", b'p', 0), ("schema_embedding_map", b'p', 0), ("mesh_offset_data", b'p', 0), ("entries", b'p', 1)]),
    (184, "HELIX_CU_FORM", &[("axis_pt", b'v', 0), ("axis_dir", b'v', 0), ("point", b'v', 0), ("hand", b'c', 0), ("turns", b'i', 0), ("pitch", b'f', 0), ("tol", b'f', 0)]),
    (185, "POLYLINE_DATA", &[("n_pvecs", b'd', 0), ("base_parm", b'f', 0), ("pvec", b'p', 0)]),
];
