use super::*;

#[wasm_bindgen]
pub struct WasmBrepSolid {
    pub(crate) solid: BrepSolid,
}

#[wasm_bindgen]
impl WasmBrepSolid {
    #[wasm_bindgen(constructor)]
    pub fn new(solid_json: &str) -> Result<WasmBrepSolid, JsValue> {
        let solid: BrepSolid = serde_json::from_str(solid_json)
            .map_err(|error| javascript_error(error.to_string()))?;
        let solid = validate_imported_solid(solid, "WasmBrepSolid")?;
        Ok(Self { solid })
    }

    /// Construct from the flat f64 codec instead of JSON — same arena
    /// validation as the JSON constructor.
    pub fn from_buffer(data: &[f64], names_json: &str) -> Result<WasmBrepSolid, JsValue> {
        let solid = decode_solid_buffer(data, names_json)?;
        let solid = validate_imported_solid(solid, "WasmBrepSolid")?;
        Ok(Self { solid })
    }

    pub fn to_buffer(&self) -> Result<WasmSolidBuffer, JsValue> {
        solid_buffer(&self.solid, "{}".into())
    }

    pub fn to_json(&self) -> Result<String, JsValue> {
        serde_json::to_string(&self.solid).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn validate_json(&self) -> Result<String, JsValue> {
        serde_json::to_string(&self.solid.validate())
            .map_err(|error| javascript_error(error.to_string()))
    }

    pub fn mass_properties_json(&self) -> Result<String, JsValue> {
        let properties = solid_mass_properties(&self.solid).map_err(javascript_error)?;
        serde_json::to_string(&properties).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn full_mass_properties_json(&self) -> Result<String, JsValue> {
        let properties = solid_mass_properties_full(&self.solid).map_err(javascript_error)?;
        serde_json::to_string(&properties).map_err(|error| javascript_error(error.to_string()))
    }

    /// Full mass properties scaled to a physical `density` (Golovanov §8.11):
    /// `mass = density * volume`, inertia and the principal moments scale by
    /// `density`, while volume, surface area, centroid, and the principal axes
    /// are density-independent. Returns the `DensityMassProperties` JSON.
    pub fn full_mass_properties_with_density_json(&self, density: f64) -> Result<String, JsValue> {
        let properties = solid_mass_properties_full(&self.solid).map_err(javascript_error)?;
        serde_json::to_string(&properties.with_density(density))
            .map_err(|error| javascript_error(error.to_string()))
    }

    pub fn tessellate_json(
        &self,
        slabs_per_span_u: usize,
        steps_per_span_v: usize,
    ) -> Result<String, JsValue> {
        let mesh = tessellate_brep(
            &self.solid,
            TessellationOptions {
                slabs_per_span_u: slabs_per_span_u.max(1),
                steps_per_span_v: steps_per_span_v.max(1),
            },
        )
        .map_err(javascript_error)?;
        serde_json::to_string(&mesh).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn transform_json(
        &self,
        matrix: &[f64],
        reverse_orientation: bool,
    ) -> Result<String, JsValue> {
        let matrix: [f64; 16] = matrix
            .try_into()
            .map_err(|_| javascript_error("transform matrix must contain 16 values".into()))?;
        let transform = AffineTransform::new(matrix).map_err(javascript_error)?;
        let solid = transform_brep(&self.solid, transform, reverse_orientation)
            .map_err(javascript_error)?;
        serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn transform_buffer(
        &self,
        matrix: &[f64],
        reverse_orientation: bool,
    ) -> Result<WasmSolidBuffer, JsValue> {
        let matrix: [f64; 16] = matrix
            .try_into()
            .map_err(|_| javascript_error("transform matrix must contain 16 values".into()))?;
        let transform = AffineTransform::new(matrix).map_err(javascript_error)?;
        let solid = transform_brep(&self.solid, transform, reverse_orientation)
            .map_err(javascript_error)?;
        solid_buffer(&solid, "{}".into())
    }

    pub fn boolean_buffer(
        &self,
        other: &WasmBrepSolid,
        operation: &str,
        tolerance: f64,
        merge_coplanar_faces: bool,
    ) -> Result<WasmSolidBuffer, JsValue> {
        let operation = match operation {
            "union" => BooleanOperation::Union,
            "intersect" => BooleanOperation::Intersect,
            "subtract" => BooleanOperation::Subtract,
            _ => return Err(javascript_error("unknown boolean operation".into())),
        };
        let solid = boolean_operation(
            &self.solid,
            &other.solid,
            operation,
            &BooleanOptions {
                tolerance,
                merge_coplanar_faces,
                ..BooleanOptions::default()
            },
        )
        .map_err(javascript_refusal)?;
        solid_buffer(&solid, "{}".into())
    }

    pub fn offset_shell_buffer(
        &self,
        opening_face_ids: &[f64],
        distance: f64,
    ) -> Result<WasmSolidBuffer, JsValue> {
        crate::panic_hook::set_once();
        let face_ids = opening_face_ids
            .iter()
            .map(|id| {
                if !id.is_finite() || *id < 0.0 || id.fract() != 0.0 {
                    Err(javascript_error("opening face IDs must be integers".into()))
                } else {
                    Ok(*id as u64)
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let result = offset_shell(&self.solid, &face_ids, distance).map_err(javascript_refusal)?;
        let metadata = serde_json::json!({ "face_images": result.face_images });
        solid_buffer(&result.solid, metadata.to_string())
    }

    pub fn tessellate_watertight_buffers(
        &self,
        chord_tolerance: f64,
    ) -> Result<WasmMeshBuffers, JsValue> {
        let mesh =
            tessellate_brep_watertight(&self.solid, chord_tolerance).map_err(javascript_error)?;
        Ok(WasmMeshBuffers {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
            face_ids: mesh.face_ids,
        })
    }

    /// Tessellate only the faces whose global sequential index `i` satisfies
    /// `i % stride == offset` (a face subset), so a heavy solid can be meshed
    /// in parallel across worker instances (dispatch `stride` calls, offsets
    /// 0..stride-1, concatenate the fragments on the caller side). No
    /// closed-shell validation — a fragment is not a closed mesh; the caller
    /// validates the concatenated whole. See
    /// `tessellate_brep_watertight_face_stride`.
    pub fn tessellate_watertight_face_stride_buffers(
        &self,
        chord_tolerance: f64,
        stride: usize,
        offset: usize,
    ) -> Result<WasmMeshBuffers, JsValue> {
        let mesh =
            tessellate_brep_watertight_face_stride(&self.solid, chord_tolerance, stride, offset)
                .map_err(javascript_error)?;
        Ok(WasmMeshBuffers {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
            face_ids: mesh.face_ids,
        })
    }

    /// Compute + serialize every edge's shared samples once, so the face-range
    /// split can hand the same buffer to each worker instead of recomputing
    /// `sample_all_edges` in all of them. See `sample_edges_encoded`.
    pub fn sample_edges_buffer(&self, chord_tolerance: f64) -> Result<Vec<f64>, JsValue> {
        sample_edges_encoded(&self.solid, chord_tolerance).map_err(javascript_error)
    }

    /// Face-stride tessellation using pre-computed edge samples (from
    /// `sample_edges_buffer`), skipping the redundant per-worker resampling.
    pub fn tessellate_watertight_face_stride_with_samples_buffers(
        &self,
        chord_tolerance: f64,
        stride: usize,
        offset: usize,
        samples: &[f64],
    ) -> Result<WasmMeshBuffers, JsValue> {
        let mesh = tessellate_brep_watertight_face_stride_with_samples(
            &self.solid,
            chord_tolerance,
            stride,
            offset,
            samples,
        )
        .map_err(javascript_error)?;
        Ok(WasmMeshBuffers {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
            face_ids: mesh.face_ids,
        })
    }

    pub fn tessellate_buffers(
        &self,
        slabs_per_span_u: usize,
        steps_per_span_v: usize,
    ) -> Result<WasmMeshBuffers, JsValue> {
        let mesh = tessellate_brep(
            &self.solid,
            TessellationOptions {
                slabs_per_span_u: slabs_per_span_u.max(1),
                steps_per_span_v: steps_per_span_v.max(1),
            },
        )
        .map_err(javascript_error)?;
        Ok(WasmMeshBuffers {
            positions: mesh.positions,
            normals: mesh.normals,
            indices: mesh.indices,
            face_ids: mesh.face_ids,
        })
    }

    pub fn boolean_json(
        &self,
        other: &WasmBrepSolid,
        operation: &str,
        tolerance: f64,
        merge_coplanar_faces: bool,
    ) -> Result<String, JsValue> {
        let operation = match operation {
            "union" => BooleanOperation::Union,
            "intersect" => BooleanOperation::Intersect,
            "subtract" => BooleanOperation::Subtract,
            _ => return Err(javascript_error("unknown boolean operation".into())),
        };
        let solid = boolean_operation(
            &self.solid,
            &other.solid,
            operation,
            &BooleanOptions {
                tolerance,
                merge_coplanar_faces,
                ..BooleanOptions::default()
            },
        )
        .map_err(javascript_refusal)?;
        serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn offset_shell_json(
        &self,
        opening_face_ids: &[f64],
        distance: f64,
    ) -> Result<String, JsValue> {
        crate::panic_hook::set_once();
        let face_ids = opening_face_ids
            .iter()
            .map(|id| {
                if !id.is_finite() || *id < 0.0 || id.fract() != 0.0 {
                    Err(javascript_error("opening face IDs must be integers".into()))
                } else {
                    Ok(*id as u64)
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let result = offset_shell(&self.solid, &face_ids, distance).map_err(javascript_refusal)?;
        serde_json::to_string(&result).map_err(|error| javascript_error(error.to_string()))
    }

    /// Golovanov §6.12 direct editing — delete a transition face (chamfer /
    /// fillet / simple 4-sided face) and heal the hole by extending and
    /// re-intersecting its neighbours. `face_id` is the wire id of the face to
    /// remove.
    pub fn delete_face_and_heal_buffer(&self, face_id: f64) -> Result<WasmSolidBuffer, JsValue> {
        crate::panic_hook::set_once();
        if !face_id.is_finite() || face_id < 0.0 || face_id.fract() != 0.0 {
            return Err(javascript_error(
                "delete_face_and_heal: face id must be a non-negative integer".into(),
            ));
        }
        let solid = delete_face_and_heal(&self.solid, face_id as u64).map_err(javascript_refusal)?;
        solid_buffer(&solid, "{}".into())
    }

    pub fn delete_face_and_heal_json(&self, face_id: f64) -> Result<String, JsValue> {
        crate::panic_hook::set_once();
        if !face_id.is_finite() || face_id < 0.0 || face_id.fract() != 0.0 {
            return Err(javascript_error(
                "delete_face_and_heal: face id must be a non-negative integer".into(),
            ));
        }
        let solid = delete_face_and_heal(&self.solid, face_id as u64).map_err(javascript_refusal)?;
        serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
    }

    /// Golovanov §6.12 direct editing — translate a face group rigidly by
    /// (dx, dy, dz) and heal the adjacency by re-intersecting the planar
    /// neighbours. Mirrors `delete_face_and_heal_json`; `face_ids` follows
    /// `offset_shell_json`'s integer-id validation.
    pub fn move_faces_json(
        &self,
        face_ids: &[f64],
        dx: f64,
        dy: f64,
        dz: f64,
    ) -> Result<String, JsValue> {
        crate::panic_hook::set_once();
        let face_ids = face_ids
            .iter()
            .map(|id| {
                if !id.is_finite() || *id < 0.0 || id.fract() != 0.0 {
                    Err(javascript_error(
                        "move_faces: face ids must be non-negative integers".into(),
                    ))
                } else {
                    Ok(*id as u64)
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        let solid =
            move_faces(&self.solid, &face_ids, Vec3::new(dx, dy, dz)).map_err(javascript_refusal)?;
        serde_json::to_string(&solid).map_err(|error| javascript_error(error.to_string()))
    }

    pub fn export_step(&self, name: &str, unit: &str, timestamp: &str) -> Result<String, JsValue> {
        export_step(std::slice::from_ref(&self.solid), name, unit, timestamp)
            .map_err(javascript_error)
    }

    /// Standalone heal/sew: pair coincident one-use boundary edges (open
    /// chains and closed rims), merge the shells they join, and re-orient the
    /// result outward. Best-effort — unsewable gaps stay open and are counted
    /// in the returned report. Returns `{ "solid": …, "report": … }`.
    pub fn sew_json(&self, tolerance: f64) -> Result<String, JsValue> {
        crate::panic_hook::set_once();
        let tolerance = if tolerance.is_finite() && tolerance > 0.0 {
            tolerance
        } else {
            1e-4
        };
        let (solid, report) = sew_solid(&self.solid, tolerance).map_err(javascript_refusal)?;
        serde_json::to_string(&serde_json::json!({ "solid": solid, "report": report }))
            .map_err(|error| javascript_error(error.to_string()))
    }
}

#[wasm_bindgen]
impl WasmNurbsSurface {
    #[wasm_bindgen(constructor)]
    pub fn new(surface_json: &str) -> Result<WasmNurbsSurface, JsValue> {
        let serialized: NurbsSurface = serde_json::from_str(surface_json)
            .map_err(|error| javascript_error(error.to_string()))?;
        let surface = NurbsSurface::new(
            serialized.degree_u,
            serialized.degree_v,
            serialized.knots_u,
            serialized.knots_v,
            serialized.control_points,
        )
        .map_err(javascript_error)?;
        Ok(Self { surface })
    }

    pub fn evaluate_many(&self, parameters_uv: &[f64]) -> Result<Vec<f64>, JsValue> {
        if parameters_uv.len() % 2 != 0 {
            return Err(javascript_error(
                "surface parameter buffer must contain u,v pairs".into(),
            ));
        }
        let mut points = Vec::with_capacity(parameters_uv.len() / 2 * 3);
        for pair in parameters_uv.chunks_exact(2) {
            let point = self
                .surface
                .evaluate(pair[0], pair[1])
                .map_err(javascript_error)?;
            points.extend([point.x, point.y, point.z]);
        }
        Ok(points)
    }

    pub fn normals_many(&self, parameters_uv: &[f64]) -> Result<Vec<f64>, JsValue> {
        if parameters_uv.len() % 2 != 0 {
            return Err(javascript_error(
                "surface parameter buffer must contain u,v pairs".into(),
            ));
        }
        let mut normals = Vec::with_capacity(parameters_uv.len() / 2 * 3);
        for pair in parameters_uv.chunks_exact(2) {
            let normal = self
                .surface
                .normal(pair[0], pair[1])
                .map_err(javascript_error)?;
            normals.extend([normal.x, normal.y, normal.z]);
        }
        Ok(normals)
    }

    pub fn derivatives_flat(
        &self,
        u: f64,
        v: f64,
        derivative_count: usize,
    ) -> Result<Vec<f64>, JsValue> {
        let derivatives = self
            .surface
            .derivatives(u, v, derivative_count)
            .map_err(javascript_error)?;
        let side = derivative_count + 1;
        let mut flat = vec![0.0; side * side * 3];
        for k in 0..side {
            for l in 0..side {
                let offset = (k * side + l) * 3;
                flat[offset] = derivatives[k][l].x;
                flat[offset + 1] = derivatives[k][l].y;
                flat[offset + 2] = derivatives[k][l].z;
            }
        }
        Ok(flat)
    }
}

#[wasm_bindgen]
impl WasmNurbsCurve {
    #[wasm_bindgen(constructor)]
    pub fn new(curve_json: &str) -> Result<WasmNurbsCurve, JsValue> {
        let serialized: NurbsCurve = serde_json::from_str(curve_json)
            .map_err(|error| javascript_error(error.to_string()))?;
        let curve = NurbsCurve::new(
            serialized.degree,
            serialized.knots,
            serialized.control_points,
        )
        .map_err(javascript_error)?;
        Ok(Self { curve })
    }

    pub fn evaluate_many(&self, parameters: &[f64]) -> Result<Vec<f64>, JsValue> {
        let mut points = Vec::with_capacity(parameters.len() * 3);
        for parameter in parameters {
            let point = self.curve.evaluate(*parameter).map_err(javascript_error)?;
            points.extend([point.x, point.y, point.z]);
        }
        Ok(points)
    }

    pub fn derivatives_flat(
        &self,
        parameter: f64,
        derivative_count: usize,
    ) -> Result<Vec<f64>, JsValue> {
        let derivatives = self
            .curve
            .derivatives(parameter, derivative_count)
            .map_err(javascript_error)?;
        let mut result = Vec::with_capacity(derivatives.len() * 3);
        for derivative in derivatives {
            result.extend([derivative.x, derivative.y, derivative.z]);
        }
        Ok(result)
    }
}
