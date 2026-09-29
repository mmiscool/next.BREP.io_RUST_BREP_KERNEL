use super::*;

#[wasm_bindgen]
pub struct WasmNurbsCurve {
    pub(crate) curve: NurbsCurve,
}

#[wasm_bindgen]
pub struct WasmNurbsSurface {
    pub(crate) surface: NurbsSurface,
}

/// A solid encoded as one flat `Float64Array` plus a tiny JSON side channel
/// for face/edge names — the zero-JSON result path for topology-heavy ops.
/// Getters consume the buffers (each may be read once) to avoid double copies.
#[wasm_bindgen]
pub struct WasmSolidBuffer {
    pub(crate) data: Vec<f64>,
    pub(crate) names_json: String,
    pub(crate) metadata_json: String,
}

#[wasm_bindgen]
impl WasmSolidBuffer {
    pub fn take_data(&mut self) -> Vec<f64> {
        std::mem::take(&mut self.data)
    }

    pub fn names_json(&self) -> String {
        self.names_json.clone()
    }

    /// Operation-specific extras (e.g. offset-shell face images); "{}" when
    /// the operation has none.
    pub fn metadata_json(&self) -> String {
        self.metadata_json.clone()
    }
}

pub(crate) fn solid_buffer(solid: &BrepSolid, metadata_json: String) -> Result<WasmSolidBuffer, JsValue> {
    let (data, names) = encode_solid(solid).map_err(javascript_error)?;
    Ok(WasmSolidBuffer {
        data,
        names_json: serde_json::to_string(&names)
            .map_err(|error| javascript_error(error.to_string()))?,
        metadata_json,
    })
}

/// Tessellation output as typed arrays; getters consume the buffers.
#[wasm_bindgen]
pub struct WasmMeshBuffers {
    pub(crate) positions: Vec<f64>,
    pub(crate) normals: Vec<f64>,
    pub(crate) indices: Vec<u32>,
    pub(crate) face_ids: Vec<u32>,
}

#[wasm_bindgen]
impl WasmMeshBuffers {
    pub fn take_positions(&mut self) -> Vec<f64> {
        std::mem::take(&mut self.positions)
    }

    pub fn take_normals(&mut self) -> Vec<f64> {
        std::mem::take(&mut self.normals)
    }

    pub fn take_indices(&mut self) -> Vec<u32> {
        std::mem::take(&mut self.indices)
    }

    pub fn take_face_ids(&mut self) -> Vec<u32> {
        std::mem::take(&mut self.face_ids)
    }
}

// ---------------------------------------------------------------------------
// Persistent solid handle registry (Rust-pipeline migration, Stage 1).
//
// Keeps resident BrepSolids in the (single-threaded) main-thread wasm instance,
// keyed by an opaque u32 handle, so ops consume/produce handles instead of
// re-serializing the full topology across the wasm boundary on every call. The host pulls only
// tessellation / mass-props / names across the boundary, and only when it needs
// them — this is the foundation that eliminates the per-op round-trip (see
//   the Rust pipeline-migration design, Stage 1).
//
// Handle lifetime is EXPLICIT: register_* mints a handle, free_solid drops it.
// The wasm32 heap has a 4 GB ceiling and heavy solids are MB-scale resident, so
// callers MUST free handles they no longer need. register_* keeps full arena +
// validate (external / first-crossing ingest); kernel-produced results from the
// *_handle ops are trusted (no double-validate — the kernel just built them).
// ---------------------------------------------------------------------------
thread_local! {
    static SOLID_REGISTRY: std::cell::RefCell<SolidRegistry> =
        std::cell::RefCell::new(SolidRegistry::new());
}

pub(crate) struct SolidRegistry {
    pub(crate) next: u32,
    pub(crate) solids: std::collections::HashMap<u32, BrepSolid>,
}

impl SolidRegistry {
    fn new() -> Self {
        Self {
            next: 1,
            solids: std::collections::HashMap::new(),
        }
    }
    fn insert(&mut self, solid: BrepSolid) -> u32 {
        let handle = self.next;
        self.next = self.next.wrapping_add(1).max(1);
        self.solids.insert(handle, solid);
        handle
    }
}

pub(crate) fn register_solid_value(solid: BrepSolid) -> u32 {
    SOLID_REGISTRY.with(|registry| registry.borrow_mut().insert(solid))
}

/// Run `f` against a resident solid by handle (immutable borrow).
pub(crate) fn with_registered_solid<T>(
    handle: u32,
    f: impl FnOnce(&BrepSolid) -> Result<T, JsValue>,
) -> Result<T, JsValue> {
    SOLID_REGISTRY.with(|registry| {
        let registry = registry.borrow();
        let solid = registry
            .solids
            .get(&handle)
            .ok_or_else(|| javascript_error(format!("unknown solid handle {handle}")))?;
        f(solid)
    })
}

/// `String`-error sibling of [`with_registered_solid`] for the Rust feature
/// pipeline, which stays JsValue-free internally (JsValue only at the wasm
/// boundary; the host test target cannot run JsValue error paths). Short borrow:
/// never hold the registry across a feature execution.
/// [`with_registered_solid_str`] for a typed body: the closure returns
/// `KernelRefusal`, an unknown handle is an `InvalidInput`, and the result
/// keeps the class so a feature can carry it to `FeatureResult.refusal`.
pub(crate) fn with_registered_solid_typed<T>(
    handle: u32,
    f: impl FnOnce(&BrepSolid) -> Result<T, crate::KernelRefusal>,
) -> Result<T, crate::KernelRefusal> {
    SOLID_REGISTRY.with(|registry| {
        let registry = registry.borrow();
        let solid = registry.solids.get(&handle).ok_or_else(|| {
            crate::KernelRefusal::input(
                crate::KernelStage::Collect,
                "handle",
                format!("unknown solid handle {handle}"),
            )
        })?;
        f(solid)
    })
}

pub(crate) fn with_registered_solid_str<T>(
    handle: u32,
    f: impl FnOnce(&BrepSolid) -> Result<T, String>,
) -> Result<T, String> {
    SOLID_REGISTRY.with(|registry| {
        let registry = registry.borrow();
        let solid = registry
            .solids
            .get(&handle)
            .ok_or_else(|| format!("unknown solid handle {handle}"))?;
        f(solid)
    })
}

/// Run `f` against two resident solids in a single short borrow (the boolean /
/// two-operand op shape). The error type follows `f` (a typed `KernelRefusal`
/// from the boolean, or the `String` a stringly caller still uses) so the
/// feature pipeline stays JsValue-free; an unknown handle is an `InvalidInput`
/// refusal converted into that type. Never held across a feature execution (the RefCell would fight
/// a re-entrant borrow otherwise).
pub(crate) fn with_two_registered_solids<T, E: From<crate::KernelRefusal>>(
    a: u32,
    b: u32,
    f: impl FnOnce(&BrepSolid, &BrepSolid) -> Result<T, E>,
) -> Result<T, E> {
    SOLID_REGISTRY.with(|registry| {
        let registry = registry.borrow();
        let handle = |id: u32| -> Result<&BrepSolid, E> {
            registry.solids.get(&id).ok_or_else(|| {
                crate::KernelRefusal::input(
                    crate::KernelStage::Collect,
                    "solid_handle",
                    format!("unknown solid handle {id}"),
                )
                .into()
            })
        };
        let sa = handle(a)?;
        let sb = handle(b)?;
        f(sa, sb)
    })
}

/// Replace a resident solid's CONTENT under its EXISTING handle. The component
/// re-pose lane (`feature_pipeline::component::update_component_transform`)
/// mutates geometry in place so the handle — which the SceneMap, the history
/// cache, and the display layer all hold — stays valid; a rigid re-pose keeps
/// every topology id and name, so nothing keyed by them moves. Errors on an
/// unknown handle (never silently mints one).
pub(crate) fn replace_registered_solid(handle: u32, solid: BrepSolid) -> Result<(), String> {
    SOLID_REGISTRY.with(|registry| {
        let mut registry = registry.borrow_mut();
        match registry.solids.get_mut(&handle) {
            Some(slot) => {
                *slot = solid;
                Ok(())
            }
            None => Err(format!("unknown solid handle {handle}")),
        }
    })
}

/// Drop a resident solid by handle (feature-pipeline-internal, `pub(crate)`
/// sibling of the wasm `free_solid`). No-op if the handle is unknown. Callers
/// MUST free handles they no longer need — the wasm32 heap has a 4 GB ceiling.
pub(crate) fn free_registered_solid(handle: u32) {
    SOLID_REGISTRY.with(|registry| {
        registry.borrow_mut().solids.remove(&handle);
    });
}

/// Ingest a solid from the flat f64 codec into the resident registry (arena +
/// validate kept — external / first-crossing ingest). Returns an opaque handle.
#[wasm_bindgen]
pub fn register_solid_buffer(data: &[f64], names_json: &str) -> Result<u32, JsValue> {
    let solid = decode_solid_buffer(data, names_json)?;
    let solid = validate_imported_solid(solid, "register_solid_buffer")?;
    Ok(register_solid_value(solid))
}

/// Ingest a solid from JSON into the resident registry (arena + validate kept).
#[wasm_bindgen]
pub fn register_solid_json(solid_json: &str) -> Result<u32, JsValue> {
    let solid: BrepSolid =
        serde_json::from_str(solid_json).map_err(|error| javascript_error(error.to_string()))?;
    let solid = validate_imported_solid(solid, "register_solid_json")?;
    Ok(register_solid_value(solid))
}

/// Drop a resident solid. No-op if the handle is unknown.
#[wasm_bindgen]
pub fn free_solid(handle: u32) {
    SOLID_REGISTRY.with(|registry| {
        registry.borrow_mut().solids.remove(&handle);
    });
}

/// A clone of a resident solid, for Rust callers that need the topology itself:
/// the case-replay diagnostics and probes, and the history runner answering a
/// drawing sheet's topology request (native thread or browser worker alike),
/// which serializes the clone back to the main side. The feature pipeline's
/// results only ever hand out handles, and this is no `#[wasm_bindgen]` export:
/// JavaScript never calls it. Short borrow; the clone is the caller's.
pub fn registered_solid_clone(handle: u32) -> Result<BrepSolid, String> {
    with_registered_solid_str(handle, |solid| Ok(solid.clone()))
}

/// Number of resident solids (diagnostics / handle-leak detection).
#[wasm_bindgen]
pub fn registered_solid_count() -> usize {
    SOLID_REGISTRY.with(|registry| registry.borrow().solids.len())
}

/// Boolean of two resident solids -> a NEW resident handle. The result never
/// crosses the boundary as topology; only its handle is returned. Kernel-produced
/// result is trusted (no double-validate).
#[wasm_bindgen]
pub fn boolean_handle(
    a: u32,
    b: u32,
    operation: &str,
    tolerance: f64,
    merge_coplanar_faces: bool,
) -> Result<u32, JsValue> {
    let operation = match operation {
        "union" => BooleanOperation::Union,
        "intersect" => BooleanOperation::Intersect,
        "subtract" => BooleanOperation::Subtract,
        _ => return Err(javascript_error("unknown boolean operation".into())),
    };
    let result = SOLID_REGISTRY.with(|registry| {
        let registry = registry.borrow();
        let sa = registry
            .solids
            .get(&a)
            .ok_or_else(|| javascript_error(format!("unknown solid handle {a}")))?;
        let sb = registry
            .solids
            .get(&b)
            .ok_or_else(|| javascript_error(format!("unknown solid handle {b}")))?;
        boolean_operation(
            sa,
            sb,
            operation,
            &BooleanOptions {
                tolerance,
                merge_coplanar_faces,
                ..BooleanOptions::default()
            },
        )
        .map_err(crate::abi::geometry::javascript_refusal)
    })?;
    Ok(register_solid_value(result))
}

/// Tessellate a resident solid to watertight mesh buffers (the buffers cross the
/// boundary; the solid does not).
#[wasm_bindgen]
pub fn tessellate_handle(handle: u32, chord_tolerance: f64) -> Result<WasmMeshBuffers, JsValue> {
    with_registered_solid(handle, |solid| {
        let mesh = tessellate_brep_watertight(solid, chord_tolerance).map_err(javascript_error)?;
        Ok(WasmMeshBuffers {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
            face_ids: mesh.face_ids,
        })
    })
}

/// Analytic mass properties of a resident solid (a handful of scalars).
#[wasm_bindgen]
pub fn mass_properties_handle(handle: u32) -> Result<String, JsValue> {
    with_registered_solid(handle, |solid| {
        let _caller = crate::mass_caller("abi.mass_properties");
        let properties = solid_mass_properties(solid).map_err(javascript_error)?;
        serde_json::to_string(&properties).map_err(|error| javascript_error(error.to_string()))
    })
}

/// Rigid/affine transform of a resident solid -> a NEW resident handle (Stage 1b;
/// unblocks Transform/Pattern/bakeTransform from the legacy serialize lane). The
/// result never crosses the boundary as topology. Kernel-produced, so trusted.
#[wasm_bindgen]
pub fn transform_handle(
    handle: u32,
    matrix: &[f64],
    reverse_orientation: bool,
) -> Result<u32, JsValue> {
    let matrix: [f64; 16] = matrix
        .try_into()
        .map_err(|_| javascript_error("transform matrix must contain 16 values".into()))?;
    let transform = AffineTransform::new(matrix).map_err(javascript_error)?;
    let result = with_registered_solid(handle, |solid| {
        transform_brep(solid, transform, reverse_orientation).map_err(javascript_error)
    })?;
    Ok(register_solid_value(result))
}

/// The resident solid's face id -> name map (Stage 1b; lets the host rebuild its
/// selection/name index from a handle without materializing the full graph).
#[wasm_bindgen]
pub fn face_names_handle(handle: u32) -> Result<String, JsValue> {
    with_registered_solid(handle, |solid| {
        let mut map: Vec<(u64, Option<String>)> = Vec::new();
        for shell in &solid.shells {
            for face in &shell.faces {
                map.push((face.id, face.name.clone()));
            }
        }
        serde_json::to_string(&map).map_err(|error| javascript_error(error.to_string()))
    })
}

// ===========================================================================
// Native in-process display accessors (brep-render). Plain pub fns — NOT
// #[wasm_bindgen] — so they add nothing to the wasm export surface; the native
// renderer links the kernel as an rlib and reads display data with no JSON or
// typed-array boundary.
// ===========================================================================

/// Native sibling of [`boolean_handle`] (String errors — constructing a
/// `JsValue` error panics off-wasm): boolean of two RESIDENT solids → a NEW
/// resident handle, operands untouched. The assembly interference check's
/// non-destructive INTERSECT lane; callers free the result handle when done.
pub fn boolean_handle_native(
    a: u32,
    b: u32,
    operation: BooleanOperation,
    options: &BooleanOptions,
) -> Result<u32, String> {
    let result =
        with_two_registered_solids(a, b, |sa, sb| boolean_operation(sa, sb, operation, options))?;
    Ok(register_solid_value(result))
}
