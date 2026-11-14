//! Export imported mesh instances without reconstruction or appearance loss.
use super::{ThreeMfAppearance, ThreeMfReadResult, CORE_NAMESPACE, MATERIAL_NAMESPACE, BEAM_NAMESPACE};
use super::write::ZipWriter;
use std::fmt::Write;

fn escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// Write the mesh instances and appearance recovered by [`super::read_3mf`].
/// Uses object-local meshes and placements; derived flattened buffers are not
/// used. Unlike solid export, this preserves corner colours and packaged PNGs.
pub fn write_3mf_meshes(model: &ThreeMfReadResult) -> Result<Vec<u8>, String> {
    let mut resources = String::new();
    let mut objects = String::new();
    let mut build = String::new();
    let mut id = 1u32;
    let mut next = || -> Result<u32, String> {
        let value = id;
        id = id.checked_add(1).ok_or("3MF resource id overflow")?;
        Ok(value)
    };
    let mut textures = std::collections::BTreeMap::new();
    let mut zip = ZipWriter::new();
    let mut paths = std::collections::BTreeSet::new();
    let mut relationships = String::from("<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">");
    for texture in &model.textures {
        if textures.contains_key(&texture.id) { return Err("duplicate texture id".into()); }
        let resource = next()?;
        textures.insert(texture.id, resource);
        // Canonical package paths keep resources separate from model/control parts.
        let path = format!("3D/Textures/texture-{resource}.png");
        if !paths.insert(path.clone()) { return Err("duplicate texture path".into()); }
        zip.add(&path, &texture.png)?;
        write!(relationships, "<Relationship Id=\"tex{resource}\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dtexture\" Target=\"/{path}\"/>").unwrap();
        write!(resources, "<m:texture2d id=\"{resource}\" path=\"/{path}\" contenttype=\"image/png\" tilestyleu=\"{}\" tilestylev=\"{}\" filter=\"{}\"/>", escape(&texture.tile_u), escape(&texture.tile_v), escape(&texture.filter)).unwrap();
    }
    for instance in &model.instances {
        let mesh = model.meshes.get(instance.mesh).ok_or("invalid instance mesh index")?;
        if mesh.positions.len() % 3 != 0 || mesh.indices.len() % 3 != 0
            || (!mesh.triangle_appearance.is_empty() && mesh.triangle_appearance.len() != mesh.triangle_count()) {
            return Err("invalid mesh buffer lengths".into());
        }
        let object = next()?;
        write!(objects, "<object id=\"{object}\" type=\"model\" name=\"{}\"><mesh><vertices>", escape(instance.name.as_deref().unwrap_or(""))).unwrap();
        let object_attributes_end = objects.len() - "<mesh><vertices>".len() - 1;
        let mut default_property = None;
        for p in mesh.positions.chunks_exact(3) {
            write!(objects, "<vertex x=\"{}\" y=\"{}\" z=\"{}\"/>",p[0],p[1],p[2]).unwrap();
        }
        objects.push_str("</vertices><triangles>");
        for (index, t) in mesh.indices.chunks_exact(3).enumerate() {
            write!(objects, "<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"",t[0],t[1],t[2]).unwrap();
            if let Some(appearance) = mesh.triangle_appearance.get(index).and_then(Option::as_ref) {
                let property = next()?;
                default_property.get_or_insert(property);
                match appearance {
                    ThreeMfAppearance::Colors(colors) => {
                        write!(resources, "<m:colorgroup id=\"{property}\">").unwrap();
                        for c in colors { write!(resources, "<m:color color=\"#{:02X}{:02X}{:02X}{:02X}\"/>",c[0],c[1],c[2],c[3]).unwrap(); }
                        resources.push_str("</m:colorgroup>");
                        write!(objects, " pid=\"{property}\" p1=\"0\" p2=\"1\" p3=\"2\"").unwrap();
                    }
                    ThreeMfAppearance::Texture { texture_id, uv } => {
                        let texid = textures.get(texture_id).ok_or("missing texture resource")?;
                        write!(resources, "<m:texture2dgroup id=\"{property}\" texid=\"{texid}\">").unwrap();
                        for p in uv { write!(resources, "<m:tex2coord u=\"{}\" v=\"{}\"/>",p[0],p[1]).unwrap(); }
                        resources.push_str("</m:texture2dgroup>");
                        write!(objects, " pid=\"{property}\" p1=\"0\" p2=\"1\" p3=\"2\"").unwrap();
                    }
                    ThreeMfAppearance::Composite { colors, weights } => {
                        let base = next()?;
                        write!(resources, "<basematerials id=\"{base}\">").unwrap();
                        for (i,c) in colors.iter().enumerate() { write!(resources, "<base name=\"material-{i}\" displaycolor=\"#{:02X}{:02X}{:02X}\"/>",c[0],c[1],c[2]).unwrap(); }
                        resources.push_str("</basematerials>");
                        let indices = (0..colors.len()).map(|i| i.to_string()).collect::<Vec<_>>().join(" ");
                        write!(resources, "<m:compositematerials id=\"{property}\" matid=\"{base}\" matindices=\"{indices}\">").unwrap();
                        for corner in weights {
                            let values = corner.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ");
                            write!(resources, "<m:composite values=\"{values}\"/>").unwrap();
                        }
                        resources.push_str("</m:compositematerials>");
                        write!(objects, " pid=\"{property}\" p1=\"0\" p2=\"1\" p3=\"2\"").unwrap();
                    }
                }
            }
            objects.push_str("/>");
        }
        objects.push_str("</triangles>");
        if let Some(lattice) = &mesh.beam_lattice {
            write!(objects, "<b:beamlattice radius=\"1\" minlength=\"{}\"><b:beams>", lattice.min_length).unwrap();
            for beam in &lattice.beams {
                write!(objects, "<b:beam v1=\"{}\" v2=\"{}\" r1=\"{}\" r2=\"{}\" cap1=\"{}\" cap2=\"{}\"/>", beam.vertices[0], beam.vertices[1], beam.radii[0], beam.radii[1], escape(&beam.caps[0]), escape(&beam.caps[1])).unwrap();
            }
            objects.push_str("</b:beams></b:beamlattice>");
        }
        objects.push_str("</mesh></object>");
        if mesh.beam_lattice.is_none() && mesh.triangle_appearance.iter().all(Option::is_some) {
            if let Some(property) = default_property {
                objects.insert_str(object_attributes_end, &format!(" pid=\"{property}\" pindex=\"0\""));
            }
        }
        let transform = instance.transform.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ");
        write!(build, "<item objectid=\"{object}\" transform=\"{transform}\"/>").unwrap();
    }
    let has_beams = model.instances.iter().any(|i| model.meshes[i.mesh].beam_lattice.is_some());
    let has_materials = !model.textures.is_empty() || model.instances.iter().any(|i| !model.meshes[i.mesh].triangle_appearance.is_empty());
    let required = match (has_materials, has_beams) { (true, true) => "m b", (true, false) => "m", (false, true) => "b", _ => "" };
    let xml = format!("<model unit=\"millimeter\" xmlns=\"{CORE_NAMESPACE}\" xmlns:m=\"{MATERIAL_NAMESPACE}\" xmlns:b=\"{BEAM_NAMESPACE}\" requiredextensions=\"{required}\"><resources>{resources}{objects}</resources><build>{build}</build></model>");
    zip.add("3D/3dmodel.model", xml.as_bytes())?;
    relationships.push_str("</Relationships>");
    zip.add("3D/_rels/3dmodel.model.rels", relationships.as_bytes())?;
    zip.add("_rels/.rels", b"<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\"><Relationship Id=\"model\" Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\" Target=\"/3D/3dmodel.model\"/></Relationships>")?;
    zip.add("[Content_Types].xml", b"<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\"><Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/><Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/><Default Extension=\"png\" ContentType=\"image/png\"/></Types>")?;
    let bytes = zip.finish()?;
    super::read_3mf(&bytes).map_err(|e| e.to_string())?;
    Ok(bytes)
}
