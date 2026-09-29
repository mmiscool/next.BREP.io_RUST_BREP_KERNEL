use super::*;

#[derive(Deserialize)]
pub(crate) struct ExtrudeBrepRequest {
    pub(crate) profile: Vec<NurbsCurve>,
    pub(crate) direction: Vec3,
    pub(crate) distance: f64,
}

#[derive(Deserialize)]
pub(crate) struct ExtrudeDraftRequest {
    /// The closed planar profile loop (LINE and circular-ARC segments).
    pub(crate) profile: Vec<NurbsCurve>,
    /// Extrude direction (must be (anti)parallel to the profile normal).
    pub(crate) direction: Vec3,
    pub(crate) distance: f64,
    /// Draft/taper angle in radians (positive tapers inward → smaller top).
    pub(crate) draft_angle: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[derive(Deserialize)]
pub(crate) struct RibRequest {
    /// The part the rib fuses into.
    pub(crate) solid: BrepSolid,
    /// The OPEN planar profile chain (V1: polygonal / straight-line segments).
    pub(crate) profile: Vec<NurbsCurve>,
    /// Total rib wall thickness (offset ±thickness/2 about the profile).
    pub(crate) thickness: f64,
    /// Extrude direction (typically −profileNormal, down into the part).
    pub(crate) extrude_dir: Vec3,
    /// The profile's own plane normal, when the caller knows it (a sketch
    /// publishes the plane it was drawn on). Absent → the plane is derived from
    /// the chain's bends, which a single straight segment cannot supply.
    #[serde(default)]
    pub(crate) plane_normal: Option<Vec3>,
    /// SolidWorks' Extrusion Direction. Absent → `PARALLEL_TO_SKETCH`, the same
    /// default the feature uses. There is no depth: the rib grows Up To Next.
    #[serde(default)]
    pub(crate) extrusion: RibExtrusion,
    /// The feature id every face name starts with (`RIB` when absent).
    #[serde(default)]
    pub(crate) name: Option<String>,
    /// The source name of each profile curve, index-aligned with `profile`
    /// (`SEG{i}` when absent). See [`crate::RibNames`].
    #[serde(default)]
    pub(crate) segment_names: Option<Vec<String>>,
}

impl RibRequest {
    pub(crate) fn names(&self) -> crate::RibNames {
        let feature = self.name.as_deref().unwrap_or("RIB");
        match &self.segment_names {
            Some(segments) => crate::RibNames {
                feature: feature.to_string(),
                segments: segments.clone(),
            },
            None => crate::RibNames::positional(feature, self.profile.len()),
        }
    }
}

#[derive(Deserialize)]
pub(crate) struct RevolveBrepRequest {
    pub(crate) profile: Vec<NurbsCurve>,
    pub(crate) axis_point: Vec3,
    pub(crate) axis_direction: Vec3,
    pub(crate) angle: f64,
    /// Aligned with `profile`; the kernel carries names through winding
    /// normalization and axis-curve skips, which the caller cannot
    /// reconstruct from the emitted face order.
    #[serde(default)]
    pub(crate) side_names: Vec<Option<String>>,
    /// `[start_cap, end_cap]` for partial revolutions.
    #[serde(default)]
    pub(crate) cap_names: Vec<Option<String>>,
}

#[derive(Deserialize)]
pub(crate) struct LoftBrepRequest {
    pub(crate) sections: Vec<Vec<NurbsCurve>>,
}

#[derive(Deserialize)]
pub(crate) struct GuidedLoftRequest {
    /// The loft-compatible cross-sections, each a closed loop of >= 2 curves.
    pub(crate) sections: Vec<Vec<NurbsCurve>>,
    /// The guide curve whose spine the loft follows.
    pub(crate) guide: NurbsCurve,
    #[serde(default)]
    pub(crate) name: Option<String>,
    /// Rotate sections into the guide's rotation-minimizing moving frame
    /// (§5.8 rotation-to-frame) instead of translation-only placement.
    #[serde(default)]
    pub(crate) rotate_to_frame: bool,
}

pub(crate) fn guided_loft_dispatch(
    request: &GuidedLoftRequest,
) -> Result<BrepSolid, crate::KernelRefusal> {
    if request.rotate_to_frame {
        loft_profile_brep_guided_frame(&request.sections, &request.guide, request.name.as_deref())
    } else {
        loft_profile_brep_guided(&request.sections, &request.guide, request.name.as_deref())
    }
}

#[derive(Deserialize)]
pub(crate) struct SweepRequest {
    /// The closed planar profile loop, as >= 2 curves (a circle is two arcs).
    pub(crate) profile: Vec<NurbsCurve>,
    /// The path curve to sweep the profile along.
    pub(crate) path: NurbsCurve,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[derive(Deserialize)]
pub(crate) struct HelixSweepRequest {
    /// The closed planar profile loop, as >= 2 curves (a circle is two arcs).
    pub(crate) profile: Vec<NurbsCurve>,
    /// A point on the helix axis, and the axis direction.
    pub(crate) axis_origin: Vec3,
    pub(crate) axis_direction: Vec3,
    /// Distance from the axis to the swept profile's centroid.
    pub(crate) helix_radius: f64,
    /// Axial rise per revolution (> 0).
    pub(crate) pitch: f64,
    /// Revolution count (> 0, <= 64; fractional turns allowed).
    pub(crate) turns: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

#[derive(Deserialize)]
pub(crate) struct EdgeBlendRequest {
    pub(crate) solid: BrepSolid,
    /// Either the edge id, or a 3D point ON the edge (ids do not survive
    /// the app-side decode, so callers identify edges geometrically).
    #[serde(default)]
    pub(crate) edge_id: Option<u64>,
    #[serde(default)]
    pub(crate) edge_point: Option<Vec3>,
    /// Fillet radius or chamfer leg distance.
    pub(crate) radius: f64,
    #[serde(default)]
    pub(crate) name: Option<String>,
}

pub(crate) fn resolve_blend_edge(request: &EdgeBlendRequest) -> Result<u64, String> {
    if let Some(edge_id) = request.edge_id {
        return Ok(edge_id);
    }
    let point = request
        .edge_point
        .ok_or("edge blend: edge_id or edge_point is required")?;
    match crate::topology::nearest_edge(&request.solid, point) {
        Some((edge_id, distance)) if distance <= 1e-3 => Ok(edge_id),
        Some((_, distance)) => Err(format!(
            "edge blend: no edge within tolerance of the given point              (nearest sample {distance:.6} away)"
        )),
        None => Err("edge blend: solid has no edges".into()),
    }
}

#[derive(Deserialize)]
pub(crate) struct TransformBrepRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) matrix: [f64; 16],
    #[serde(default)]
    pub(crate) reverse_orientation: bool,
}

#[derive(Deserialize)]
pub(crate) struct MirrorBrepRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) plane_point: Vec3,
    pub(crate) plane_normal: Vec3,
}

#[derive(Deserialize)]
pub(crate) struct SplitByPlaneRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) plane_point: Vec3,
    pub(crate) plane_normal: Vec3,
}

#[derive(Deserialize)]
pub(crate) struct SplitBySurfaceRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) tool: SplitSurface,
}

#[derive(Deserialize)]
pub(crate) struct SplitByFaceSurfaceRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) surface: NurbsSurface,
}

#[derive(Deserialize)]
pub(crate) struct TessellateBrepRequest {
    pub(crate) solid: BrepSolid,
    #[serde(default = "default_tessellation_slabs")]
    pub(crate) slabs_per_span_u: usize,
    #[serde(default = "default_tessellation_steps")]
    pub(crate) steps_per_span_v: usize,
}

#[derive(Deserialize)]
pub(crate) struct TessellateFaceRequest {
    pub(crate) face: FaceRecord,
    #[serde(default = "default_tessellation_slabs")]
    pub(crate) slabs_per_span_u: usize,
    #[serde(default = "default_tessellation_steps")]
    pub(crate) steps_per_span_v: usize,
}

#[derive(Serialize)]
pub(crate) struct FaceMetrics {
    pub(crate) parameter_space_area: f64,
    pub(crate) area: f64,
    pub(crate) volume_contribution: f64,
    pub(crate) trim_polygons: Vec<Vec<[f64; 2]>>,
}

#[derive(Deserialize)]
pub(crate) struct ClassifyPointsRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) points: Vec<Vec3>,
    #[serde(default = "default_intersection_tolerance")]
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct ArrangeSegmentsRequest {
    pub(crate) segments: Vec<Segment2>,
    pub(crate) tolerance: f64,
}

#[derive(Deserialize)]
pub(crate) struct BuildPcurveRequest {
    pub(crate) surface: NurbsSurface,
    pub(crate) curve: NurbsCurve,
}

#[derive(Deserialize)]
pub(crate) struct BuildImprintsRequest {
    pub(crate) solid_a: BrepSolid,
    pub(crate) solid_b: BrepSolid,
    #[serde(default)]
    pub(crate) options: ImprintOptions,
}

#[derive(Deserialize)]
pub(crate) struct ApplyEdgeSplitsRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) operand: u8,
    pub(crate) imprint: ImprintResultRecord,
}

#[derive(Deserialize)]
pub(crate) struct FragmentFacesRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) operand: u8,
    pub(crate) imprint: ImprintResultRecord,
}

#[derive(Deserialize)]
pub(crate) struct BooleanRequest {
    pub(crate) first: BrepSolid,
    pub(crate) second: BrepSolid,
    pub(crate) operation: BooleanOperation,
    #[serde(default)]
    pub(crate) options: BooleanOptions,
}

#[derive(Deserialize)]
pub(crate) struct OffsetSurfaceRequest {
    pub(crate) face: topology::FaceRecord,
    pub(crate) distance: f64,
    #[serde(default)]
    pub(crate) planar_extension: f64,
}

#[derive(Deserialize)]
pub(crate) struct OffsetFaceCarrierRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) face_id: u64,
    pub(crate) distance: f64,
    #[serde(default)]
    pub(crate) planar_extension: f64,
}

#[derive(Deserialize)]
pub(crate) struct OffsetShellRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) opening_face_ids: Vec<u64>,
    pub(crate) distance: f64,
}

#[derive(Deserialize)]
pub(crate) struct OffsetShellDiagnosticsRequest {
    pub(crate) solid: BrepSolid,
    pub(crate) opening_face_ids: Vec<u64>,
    pub(crate) distance: f64,
    #[serde(default)]
    pub(crate) tolerances: Option<KernelTolerances>,
}

#[derive(Deserialize)]
pub(crate) struct ExportStepRequest {
    pub(crate) solids: Vec<BrepSolid>,
    pub(crate) name: String,
    #[serde(default = "default_step_unit")]
    pub(crate) unit: String,
    #[serde(default)]
    pub(crate) timestamp: String,
}

#[derive(Deserialize)]
pub(crate) struct CoalesceRequest {
    pub(crate) solid: BrepSolid,
    #[serde(default = "default_intersection_tolerance")]
    pub(crate) tolerance: f64,
}

pub(crate) fn default_tessellation_slabs() -> usize {
    8
}

pub(crate) fn default_tessellation_steps() -> usize {
    8
}

pub(crate) fn default_step_unit() -> String {
    "millimeter".into()
}
