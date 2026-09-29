use crate::json_support::vec3_or;
use super::*;

// Components are derived from authored ACOMP features and namespaced display
// solids. Kernel records stay on the runner; namespace parsing uses the kernel's
// shared parser so both sides agree on component ownership.

/// One assembly component instance as the APP sees it: the owning ACOMP feature
/// id, the parts-library part name, the authored rigid pose (`inputParams.
/// transform` — `{translate, rotateEulerDeg}`, intrinsic-XYZ degrees), the
/// grounded flag, and the member solid names currently in the display scene.
#[derive(Debug, Clone, PartialEq)]
pub struct ComponentInfo {
    /// The owning ACOMP feature id (`ACOMP<digits>`) — also the namespace prefix.
    pub id: String,
    /// The parts-library entry this instance places (`inputParams.partName`).
    pub part_name: String,
    /// `inputParams.transform.translate` (numbers-or-zero: an expression-valued
    /// slot reads 0 here, matching the transform gizmo's numeric read).
    pub translate: [f64; 3],
    /// `inputParams.transform.rotateEulerDeg` (degrees, intrinsic XYZ).
    pub rotate_deg: [f64; 3],
    /// Grounded: an explicit `isFixed` wins; ABSENT mirrors the kernel's
    /// auto-ground rule (grounded iff no ACOMP precedes it in the history).
    pub fixed: bool,
    /// Member solid names in display-scene order (`{id}:{part solid name}`).
    /// Empty when the instance is rolled back / failed to build.
    pub members: Vec<String>,
}

/// The ACOMP dispatch predicate, mirroring the kernel's `is_acomp_type` (the
/// two literals `execute_feature` matches on).
pub(crate) fn is_acomp_feature_type(feature_type: &str) -> bool {
    matches!(feature_type, "ACOMP" | "ASSEMBLY COMPONENT")
}


impl EngineState {
    /// The OWNING component feature id of a scene solid — the OUTERMOST
    /// `ACOMP<digits>:` namespace segment of its name, verified against the
    /// history (the segment must be an ACOMP feature of THIS document; a nested
    /// chain's inner segments belong to the sub-assembly's own document).
    /// `None` for ordinary modeling solids (no prefix) and for sketch-child
    /// names (`S1:G20` — `S1` is not an ACOMP segment).
    pub fn component_of_solid(&self, solid_name: &str) -> Option<String> {
        let (chain, _local) = brep_kernel::split_component_namespace(solid_name);
        let head = *chain.first()?;
        let index = self.history.index_of(head)?;
        self.history
            .feature_type(index)
            .filter(|ty| is_acomp_feature_type(ty))
            .map(|_| head.to_string())
    }

    /// Every ACOMP feature id in history order (the structure tree's row order).
    pub fn component_ids(&self) -> Vec<String> {
        (0..self.history.len())
            .filter(|&i| {
                self.history
                    .feature_type(i)
                    .is_some_and(|ty| is_acomp_feature_type(&ty))
            })
            .filter_map(|i| self.history.feature_id(i))
            .collect()
    }

    /// The derived [`ComponentInfo`] for an ACOMP feature id (`None` when the id
    /// is missing or not an ACOMP feature).
    pub fn component_info(&self, feature_id: &str) -> Option<ComponentInfo> {
        let index = self.history.index_of(feature_id)?;
        self.history
            .feature_type(index)
            .filter(|ty| is_acomp_feature_type(ty))?;
        let params = self.history.feature_params(index).unwrap_or_default();
        let transform = params.get("transform");
        let translate = vec3_or(transform.and_then(|t| t.get("translate")), [0.0; 3]);
        let rotate_deg = vec3_or(transform.and_then(|t| t.get("rotateEulerDeg")), [0.0; 3]);
        // Grounded: explicit boolean wins; ABSENT (or null) mirrors the kernel's
        // auto-ground — the FIRST component of the document is grounded.
        let fixed = match params.get("isFixed") {
            Some(serde_json::Value::Bool(flag)) => *flag,
            _ => self
                .component_ids()
                .first()
                .is_some_and(|first| first == feature_id),
        };
        let members = self.member_solid_names(feature_id);
        Some(ComponentInfo {
            id: feature_id.to_string(),
            part_name: params
                .get("partName")
                .and_then(|v| v.as_str())
                .unwrap_or_default()
                .to_string(),
            translate,
            rotate_deg,
            fixed,
            members,
        })
    }

    /// The world bbox CENTER of a component's member solids — the Move gizmo's
    /// attach point. `None` when the component has no resident members (rolled
    /// back / failed build).
    pub fn component_bbox_center(&self, feature_id: &str) -> Option<[f64; 3]> {
        let mut bbox = crate::camera::Aabb::empty();
        for solid in self.scene.solids() {
            if owned_by(&solid.name, feature_id) {
                bbox.union(&solid.bbox);
            }
        }
        (!bbox.is_empty()).then(|| bbox.center())
    }

    /// The member solid names of ACOMP `feature_id`, in display-scene order.
    /// The caller has already resolved `feature_id` to an ACOMP feature of
    /// THIS document, so ownership is the outermost namespace segment alone —
    /// no per-solid history lookup. Routing every solid through
    /// [`Self::component_of_solid`] (a linear `index_of` scan each) made this
    /// O(solids × features); the BOM paid it once per row per frame.
    fn member_solid_names(&self, feature_id: &str) -> Vec<String> {
        self.scene
            .solids()
            .iter()
            .filter(|solid| owned_by(&solid.name, feature_id))
            .map(|solid| solid.name.clone())
            .collect()
    }
}

/// Whether `solid_name`'s OUTERMOST `ACOMP<digits>:` segment is `feature_id`.
/// A nested chain's inner segments belong to the sub-assembly's own document,
/// exactly as [`EngineState::component_of_solid`] reads them.
fn owned_by(solid_name: &str, feature_id: &str) -> bool {
    let (chain, _local) = brep_kernel::split_component_namespace(solid_name);
    chain.first().copied() == Some(feature_id)
}


