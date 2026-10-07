//! Optional starter content. All writes use the normal PLM part/document APIs.
use super::{blocking, require_admin, Shared};
use crate::{
    db::PartSpec,
    model::{DocumentClass, Origin, User},
    Error,
};
use axum::{extract::State, http::HeaderMap, Json};
use serde::Deserialize;
use serde_json::{json, Value};
mod kicad;

pub async fn kicad_part_type(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    let part_type = db.mutate(|state| {
        if let Some(existing) = state.part_type("kicad") {
            if !matches!(existing.mode, crate::model::NumberMode::Counter) { return Err(Error::bad_request("The KiCad part type must use counter numbering.")); }
            return Ok(existing.clone());
        }
        let part_type = crate::model::PartType { id: "kicad".into(), name: "KiCad electrical parts".into(), prefix: "ELEC".into(), digits: 11, next: 1, created_at: crate::db::now(), mode: crate::model::NumberMode::Counter };
        state.part_types.push(part_type.clone());
        Ok(part_type)
    })?;
    Ok(Json(serde_json::to_value(part_type).map_err(Error::internal)?))
}

pub async fn kicad_libraries(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(blocking(move || kicad::libraries(&db)).await?))
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TaxonomyRequest { pub category: String }

pub async fn kicad_taxonomy(State(db): State<Shared>, headers: HeaderMap, Json(request): Json<TaxonomyRequest>) -> Result<Json<Value>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(blocking(move || kicad::repair_taxonomy(&db, &request.category)).await?))
}

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImportRequest {
    pub part_type: String,
    pub category: String,
    pub pack: String,
    #[serde(default)]
    pub library: String,
}

pub async fn import(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(request): Json<ImportRequest>,
) -> Result<Json<Value>, Error> {
    let user = require_admin(&db, &headers)?;
    Ok(Json(blocking(move || {
        db.read(|s| {
            let kind = s.part_type(&request.part_type).ok_or_else(|| Error::bad_request("choose a part type"))?;
            if !matches!(kind.mode, crate::model::NumberMode::Counter) {
                return Err(Error::bad_request("starter imports require a counter part type"));
            }
            if !request.category.is_empty() && crate::catalog::find(&s.categories, &request.category).is_none() {
                return Err(Error::bad_request("choose an existing category"));
            }
            Ok(())
        })?;
        if request.pack != "kicad" {
            // Validate the selection before provisioning anything.
            starter_entries(&request.pack)?;
            db.mutate(|state| {
                if let Some(kind) = state.part_type("starter-member") {
                    if kind.mode != crate::model::NumberMode::Free {
                        return Err(Error::conflict("starter-member must use free text numbering"));
                    }
                } else {
                    state.part_types.push(crate::model::PartType {
                        id: "starter-member".into(), name: "Starter family member".into(),
                        prefix: "".into(), digits: 9, next: 1, created_at: crate::db::now(),
                        mode: crate::model::NumberMode::Free,
                    });
                }
                Ok(())
            })?;
        }
        if request.pack == "kicad" {
            return kicad::import(&db, &user, &request);
        }
        let entries = starter_entries(&request.pack)?;
        let warnings: Vec<String> = Vec::new();
        let mut created = 0;
        let mut skipped = 0;
        let mut failed = Vec::new();
        for (reference, name, document) in entries {
            match install(&db, &user, &request, &reference, &name, document) {
                Ok(true) => created += 1,
                Ok(false) => skipped += 1,
                Err(error) => failed.push(json!({"name":name,"error":error.to_string()})),
            }
        }
        Ok(json!({"created":created,"skipped":skipped,"failed":failed,"warnings":warnings}))
    }).await?))
}

// A retry resumes a document-less first revision after a failed write. Existing
// completed parts are never replaced, including parts the user has edited.
fn install(
    db: &crate::db::Db,
    user: &User,
    request: &ImportRequest,
    reference: &str,
    name: &str,
    mut document: Value,
) -> Result<bool, Error> {
    let existing = db.read(|s| {
        s.parts
            .iter()
            .find(|p| p.part_type == request.part_type && p.external_ref == reference)
            .cloned()
    });
    let part = if let Some(part) = existing {
        if part.latest().is_some_and(|r| !r.content_hash.is_empty()) {
            return Ok(false);
        }
        part
    } else {
        db.create_part_with(user, &PartSpec {
            part_type: request.part_type.clone(), name: name.into(), category: request.category.clone(),
            description: if request.pack == "kicad" { "KiCad symbol library import".into() } else { "Parametric starter geometry in mm; simplified fasteners without threads. Verify dimensions against your supplier before release.".into() },
            document_class: if request.pack == "kicad" { DocumentClass::Normal } else { DocumentClass::Family },
            external_ref: reference.into(), origin: Origin::Imported, ..PartSpec::default()
        })?
    };
    if request.pack != "kicad" {
        db.update_part(&part.id, &json!({"member_part_type":"starter-member"}))?;
    }
    let revision = part
        .latest()
        .ok_or_else(|| Error::conflict("import part has no revision"))?;
    document["partAttributes"] = json!({"Part_Number":part.number,"Description":part.description});
    // Members use the dedicated free text type. The unique family number
    // prefixes their table numbers, avoiding collisions across packs.
    if let Some(rows) = document["familyTable"]["rows"].as_array_mut() {
        for (i, row) in rows.iter_mut().enumerate() {
            row["partNumber"] = json!(format!("{}-{:02}", part.number, i + 1));
        }
    }
    db.checkout(user, &part.id, &revision.id, "setup-wizard")?;
    let body = serde_json::to_string(&document).map_err(Error::internal)?;
    let written = db.write_import_document(
        user,
        &part.id,
        &revision.id,
        &revision.content_hash,
        &body,
        &json!({"uses":[]}),
    );
    let checked_in = db.checkin(user, &part.id, &revision.id, false);
    written?;
    checked_in?;
    Ok(true)
}

fn primitive(
    kind: &str,
    id: &str,
    dimensions: Value,
    position: Value,
    operation: &str,
    targets: &[&str],
) -> Value {
    let mut params = dimensions;
    params["id"] = json!(id);
    params["transform"] = json!({"position":position,"rotationEuler":[0,0,0],"scale":[1,1,1]});
    params["boolean"] = json!({"targets":targets,"operation":operation});
    json!({"type":kind,"inputParams":params,"persistentData":{}})
}
fn cube(
    id: &str,
    x: Value,
    y: Value,
    z: Value,
    position: Value,
    operation: &str,
    targets: &[&str],
) -> Value {
    primitive(
        "P.CU",
        id,
        json!({"sizeX":x,"sizeY":y,"sizeZ":z}),
        position,
        operation,
        targets,
    )
}
fn cylinder(
    id: &str,
    radius: Value,
    height: Value,
    position: Value,
    operation: &str,
    targets: &[&str],
) -> Value {
    primitive(
        "P.CY",
        id,
        json!({"radius":radius,"height":height}),
        position,
        operation,
        targets,
    )
}
fn family(expressions: &str, columns: &[&str], rows: Vec<Value>, features: Vec<Value>) -> Value {
    json!({"documentClass":"family","expressions":expressions,"configurator":{},"features":features,
        "familyTable":{"columns":columns.iter().map(|name|json!({"name":name,"label":name})).collect::<Vec<_>>(),"rows":rows}})
}
fn row(description: &str, values: Value) -> Value {
    json!({"partNumber":"","revision":"A","description":description,"values":values})
}

fn starter_entries(pack: &str) -> Result<Vec<(String, String, Value)>, Error> {
    let mut entries = Vec::new();
    let mut add = |id: &str, name: &str, doc: Value| {
        entries.push((format!("brep-starter:v1:{id}"), name.into(), doc))
    };
    match pack {
        "fasteners" => {
            add("socket-screws", "Metric socket head screws", family("diameter = 6; length = 20; headDiameter = 10; headHeight = 6;", &["diameter","length","headDiameter","headHeight"],
                [(3,10,5.5,3),(4,16,7.0,4),(5,20,8.5,5),(6,20,10.0,6),(8,30,13.0,8)].into_iter().map(|(d,l,h,t)|row(&format!("M{d} x {l} socket head screw"),json!({"diameter":d.to_string(),"length":l.to_string(),"headDiameter":h.to_string(),"headHeight":t.to_string()}))).collect(),
                vec![cylinder("Shank",json!("diameter / 2"),json!("length"),json!([0,0,0]),"NONE",&[]),cylinder("Head",json!("headDiameter / 2"),json!("headHeight"),json!([0,"length",0]),"UNION",&["Shank"])]));
            add("washers", "Metric plain washers", family("bore = 6.4; outer = 12; thickness = 1.6;", &["bore","outer","thickness"],
                [(3.2,7.0,0.5),(4.3,9.0,0.8),(5.3,10.0,1.0),(6.4,12.0,1.6),(8.4,16.0,1.6)].into_iter().map(|(b,o,t)|row(&format!("Washer {b} mm bore"),json!({"bore":b.to_string(),"outer":o.to_string(),"thickness":t.to_string()}))).collect(),
                vec![cylinder("Washer",json!("outer / 2"),json!("thickness"),json!([0,0,0]),"NONE",&[]),cylinder("Bore",json!("bore / 2"),json!("thickness + 2"),json!([0,-1,0]),"SUBTRACT",&["Washer"])]));
        }
        "bar-stock" => {
            add("rectangular-bar", "Rectangular bar stock", family("width = 20; height = 10; length = 100;", &["width","height","length"],
                [(10,5),(20,10),(25,25),(50,10)].into_iter().map(|(w,h)|row(&format!("{w} x {h} mm bar"),json!({"width":w.to_string(),"height":h.to_string(),"length":"100"}))).collect(),vec![cube("Bar",json!("width"),json!("height"),json!("length"),json!([0,0,0]),"NONE",&[])]));
            add(
                "round-bar",
                "Round bar stock",
                family(
                    "diameter = 20; length = 100;",
                    &["diameter", "length"],
                    [6, 10, 12, 20, 25, 50]
                        .into_iter()
                        .map(|d| {
                            row(
                                &format!("Round bar diameter {d} mm"),
                                json!({"diameter":d.to_string(),"length":"100"}),
                            )
                        })
                        .collect(),
                    vec![cylinder(
                        "Bar",
                        json!("diameter / 2"),
                        json!("length"),
                        json!([0, 0, 0]),
                        "NONE",
                        &[],
                    )],
                ),
            );
        }
        "i-beams" => add(
            "i-beam",
            "I beam cross sections",
            family(
                "width = 100; height = 100; web = 6; flange = 8; length = 100;",
                &["width", "height", "web", "flange", "length"],
                vec![
                    row(
                        "IPE 100 nominal section, sharp corners",
                        json!({"width":"55","height":"100","web":"4.1","flange":"5.7","length":"100"}),
                    ),
                    row(
                        "IPE 200 nominal section, sharp corners",
                        json!({"width":"100","height":"200","web":"5.6","flange":"8.5","length":"100"}),
                    ),
                ],
                vec![
                    cube(
                        "Web",
                        json!("web"),
                        json!("height - 2 * flange"),
                        json!("length"),
                        json!(["(width - web) / 2", "flange", 0]),
                        "NONE",
                        &[],
                    ),
                    cube(
                        "Top",
                        json!("width"),
                        json!("flange"),
                        json!("length"),
                        json!([0, "height - flange", 0]),
                        "UNION",
                        &["Web"],
                    ),
                    cube(
                        "Bottom",
                        json!("width"),
                        json!("flange"),
                        json!("length"),
                        json!([0, 0, 0]),
                        "UNION",
                        &["Top"],
                    ),
                ],
            ),
        ),
        "aluminum" => {
            add(
                "aluminum-tube",
                "Aluminum square tube cross sections",
                family(
                    "size = 20; wall = 2; length = 100;",
                    &["size", "wall", "length"],
                    [20, 25, 30, 40, 50]
                        .into_iter()
                        .map(|size| {
                            row(
                                &format!("{size} x {size} x 2 mm square tube"),
                                json!({"size":size.to_string(),"wall":"2","length":"100"}),
                            )
                        })
                        .collect(),
                    vec![
                        cube(
                            "Tube",
                            json!("size"),
                            json!("size"),
                            json!("length"),
                            json!([0, 0, 0]),
                            "NONE",
                            &[],
                        ),
                        cube(
                            "Void",
                            json!("size - 2 * wall"),
                            json!("size - 2 * wall"),
                            json!("length + 2"),
                            json!(["wall", "wall", -1]),
                            "SUBTRACT",
                            &["Tube"],
                        ),
                    ],
                ),
            );
            let mut profile = vec![cube(
                "Profile",
                json!("size"),
                json!("size"),
                json!("length"),
                json!([0, 0, 0]),
                "NONE",
                &[],
            )];
            for (id, x, y, w, h, target) in [
                (
                    "SlotBottom",
                    "(size - slot) / 2",
                    "-1",
                    "slot",
                    "size / 4 + 1",
                    "Profile",
                ),
                (
                    "SlotTop",
                    "(size - slot) / 2",
                    "3 * size / 4",
                    "slot",
                    "size / 4 + 1",
                    "SlotBottom",
                ),
                (
                    "SlotLeft",
                    "-1",
                    "(size - slot) / 2",
                    "size / 4 + 1",
                    "slot",
                    "SlotTop",
                ),
                (
                    "SlotRight",
                    "3 * size / 4",
                    "(size - slot) / 2",
                    "size / 4 + 1",
                    "slot",
                    "SlotLeft",
                ),
            ] {
                profile.push(cube(
                    id,
                    json!(w),
                    json!(h),
                    json!("length + 2"),
                    json!([x, y, -1]),
                    "SUBTRACT",
                    &[target],
                ));
            }
            // Undercuts give each slot a T cross section.
            for (id, x, y, w, h, target) in [
                (
                    "TBottom",
                    "(size - cavity) / 2",
                    "size / 8",
                    "cavity",
                    "size / 8",
                    "SlotRight",
                ),
                (
                    "TTop",
                    "(size - cavity) / 2",
                    "3 * size / 4",
                    "cavity",
                    "size / 8",
                    "TBottom",
                ),
                (
                    "TLeft",
                    "size / 8",
                    "(size - cavity) / 2",
                    "size / 8",
                    "cavity",
                    "TTop",
                ),
                (
                    "TRight",
                    "3 * size / 4",
                    "(size - cavity) / 2",
                    "size / 8",
                    "cavity",
                    "TLeft",
                ),
            ] {
                profile.push(cube(
                    id,
                    json!(w),
                    json!(h),
                    json!("length + 2"),
                    json!([x, y, -1]),
                    "SUBTRACT",
                    &[target],
                ));
            }
            add("aluminum-t-slot", "Aluminum T slot cross sections", family("size = 20; slot = 6; cavity = 10; length = 100;", &["size","slot","cavity","length"],
                [(20,6,10),(30,8,14),(40,8,16)].into_iter().map(|(size,slot,cavity)|row(&format!("{size} x {size} mm generic T slot"),json!({"size":size.to_string(),"slot":slot.to_string(),"cavity":cavity.to_string(),"length":"100"}))).collect(), profile));
            add("aluminum-angle", "Aluminum angle cross sections", family("width = 25; height = 25; wall = 3; length = 100;", &["width","height","wall","length"], [20,25,30,40,50].into_iter().map(|size|row(&format!("{size} x {size} x 3 mm angle"),json!({"width":size.to_string(),"height":size.to_string(),"wall":"3","length":"100"}))).collect(),
                vec![cube("Angle",json!("width"),json!("height"),json!("length"),json!([0,0,0]),"NONE",&[]),cube("Void",json!("width"),json!("height"),json!("length + 2"),json!(["wall","wall",-1]),"SUBTRACT",&["Angle"])]));
        }
        _ => return Err(Error::bad_request("unknown starter pack")),
    }
    Ok(entries)
}
