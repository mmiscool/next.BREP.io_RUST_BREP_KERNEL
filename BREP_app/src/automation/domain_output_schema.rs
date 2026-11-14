//! Remaining domain receipts, matched to the command producers and harness DTOs.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, nullable};

pub(super) fn ecad_action() -> Value {
    closed_object_optional(json!({"id":{"type":"string"},"workbench":{"enum":["diagram","pcb","symbol","pads"]}}), &[])
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn plm_follow_now() -> Value {
    closed_object_optional(json!({"requests":{"type":"integer","minimum":0},"moves":{"type":"integer","minimum":0}}), &[])
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn plm_bake_pass() -> Value {
    let text=json!({"type":"string"});
    let mut variants=Vec::new();
    for (status, extra) in [("done",None),("failed",Some("error")),("dropped",Some("why"))] {
        let mut fields=json!({"id":text,"number":text,"revision":text,"source":text,"status":{"const":status}});
        if let Some(key)=extra { fields[key]=text.clone(); }
        variants.push(closed_object_optional(fields,&[]));
    }
    closed_object_optional(json!({"jobs":array(json!({"oneOf":variants})),"ms":{"type":"integer","minimum":0}}), &[])
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn kicad_part_add() -> Value {
    let text=json!({"type":"string"});let maybe=nullable(text.clone());
    closed_object_optional(json!({"id":text,"document":text,"summary":text,"symbol":maybe,"footprint":maybe,
        "pads":{"type":"integer","minimum":0},"model":maybe,"notes":array(text)}), &[])
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn kicad_library_search() -> Value {
    let text=json!({"type":"string"});let count=json!({"type":"integer","minimum":0});
    let folder=closed_object_optional(json!({"path":text,"from":text,"exists":{"type":"boolean"}}), &[]);
    let folders=closed_object_optional(json!({"symbols":folder,"footprints":folder,"models":folder}), &[]);
    let row=json!({"id":text,"library":text,"name":text});
    let mut detailed=row.clone();
    detailed.as_object_mut().unwrap().extend(json!({"description":text,"reference":text,"pins":count,
        "footprint":nullable(text.clone()),"footprintFilters":array(text)}).as_object().unwrap().clone());
    let symbols=array(json!({"oneOf":[closed_object_optional(row.clone(),&[]),closed_object_optional(detailed,&[])]}));
    let footprints=array(closed_object_optional(row,&[]));
    let mut variants=Vec::new();
    for (s,f) in [(true,false),(false,true),(true,true)] {
        let mut fields=json!({"folders":folders});
        if s { fields["symbols"]=symbols.clone();fields["symbolMatches"]=count.clone();fields["symbolLibraries"]=count.clone(); }
        if f { fields["footprints"]=footprints.clone();fields["footprintMatches"]=count.clone();fields["footprintLibraries"]=count.clone(); }
        variants.push(closed_object_optional(fields,&[]));
    }
    json!({"type":"object","oneOf":variants})
}
pub(super) fn bom_wire_lines() -> Value {
    let text=json!({"type":"string"});let number=json!({"type":"number"});
    closed_object_optional(json!({"lines":array(closed_object_optional(json!({"id":text,"connectionId":text,"stockPartNumber":text,
        "length":nullable(number.clone()),"margin":number,"mfQty":nullable(number),
        "status":{"enum":["current","routing","cancelled","no-run","rolled-back","routed","missing-endpoint","waypoint-endpoint","same-endpoint","no-segments","no-route","port-reuse"]},"reason":text}),&[]))}), &[])
}
pub(super) fn wire_harness_state() -> Value {
    let text=json!({"type":"string"});let number=json!({"type":"number"});let boolean=json!({"type":"boolean"});
    let strings=array(text.clone());let count=json!({"type":"integer","minimum":0});let side=json!({"enum":["A","B"]});
    let connection=closed_object_optional(json!({"id":text,"name":text,"from":text,"to":text,"diameter":number}),&[]);
    let endpoint=closed_object_optional(json!({"id":text,"port":text,"point":text,"purpose":text,"kind":{"enum":["termination","waypoint"]},"component":text}),&["component"]);
    let segment=closed_object_optional(json!({"id":text,"first_port":text,"first_side":side,"second_port":text,"second_side":side,"length":number}),&[]);
    let route=closed_object_optional(json!({"connection_id":text,"feasible":boolean,
        "status":{"enum":["routed","missing-endpoint","waypoint-endpoint","same-endpoint","no-segments","no-route","port-reuse"]},
        "message":text,"length":nullable(number.clone()),"segment_ids":strings,"node_path":strings,"port_ids":strings}),&[]);
    let bundle=closed_object_optional(json!({"segment_id":text,"solid_name":text,"wire_count":count,"diameter":number,"length":number,
        "connection_ids":strings,"status":{"enum":["built","bundles-off","tight-bend","build-failed"]},"error":text}),&[]);
    let report=closed_object_optional(json!({"endpoints":array(endpoint),"segments":array(segment),"routes":array(route),"bundles":array(bundle),"segment_problems":strings}),&[]);
    closed_object_optional(json!({"connections":array(connection),"buildBundles":boolean,"cutMargin":number,"report":nullable(report)}),&[])
}

/// The active editor variants in viewport::ecad::state_json, plus command-only fields.
pub(super) fn ecad_state() -> Value {
    let text=json!({"type":"string"});let number=json!({"type":"number"});let boolean=json!({"type":"boolean"});
    let count=json!({"type":"integer","minimum":0});let integer=json!({"type":"integer"});
    let maybe=nullable(text.clone());let strings=array(text.clone());
    let point=json!({"type":"array","items":integer,"minItems":2,"maxItems":2});
    let pad=closed_object_optional(json!({"number":text,"at":point}),&[]);
    let common=json!({"target":text,"inStep":boolean,"revision":count,
        "pan":{"type":"array","items":number,"minItems":2,"maxItems":2},"zoom":number,
        "unreadable":maybe,"offender":maybe,"repair":maybe,"tool":text,
        "status":closed_object_optional(json!({"text":text,"at":nullable(point.clone()),"under":maybe}),&[]),
        "outdated":array(closed_object_optional(json!({"reference":text,"part":text}),&[])),
        "popupOpen":boolean,"modalOpen":boolean,"keysReachEditor":boolean});
    let mut symbol=common.clone();symbol["target"]=json!({"const":"symbol"});
    symbol["unitCount"]=count.clone();symbol["gateCount"]=count.clone();
    symbol["pins"]=array(closed_object_optional(json!({"number":text,"name":text,"at":point,"end":point,"unit":count,"gate":count}),&[]));
    let mut pads=common.clone();pads["target"]=json!({"const":"pads"});pads["pads"]=array(pad.clone());
    pads["pinsWithoutPads"]=strings.clone();pads["silk"]=array(array(point.clone()));
    pads["selection"]=nullable(json!({"oneOf":[
        closed_object_optional(json!({"kind":{"const":"pad"},"number":text}),&[]),closed_object_optional(json!({"kind":{"const":"silk"},"index":count}),&[])]}));
    let mut sheet=common;sheet["target"]=json!({"enum":["diagram","pcb"]});
    for key in ["view","kind","labelText","powerNetText","stockPartNumber"] { sheet[key]=text.clone(); }
    sheet["selection"]=nullable(json!({"oneOf":[
        closed_object_optional(json!({"kind":{"const":"component"},"id":text,"reference":maybe}),&[]),
        closed_object_optional(json!({"kind":{"enum":["wire","label"]},"id":text}),&[]),
        closed_object_optional(json!({"kind":{"const":"junction"},"at":point}),&[])]}));
    sheet["components"]=array(closed_object_optional(json!({"id":text,"reference":text,"value":text,"symbol":text,"at":point,"rotation":integer,"pins":count,
        "gates":array(closed_object_optional(json!({"gate":count,"name":text,"at":point,"rotation":integer}),&[]))}),&[]));
    sheet["wires"]=array(closed_object_optional(json!({"id":text,"from":maybe,"to":maybe,"connectionId":text,"stockPartNumber":text}),&[]));
    sheet["labels"]=array(closed_object_optional(json!({"id":text,"name":text,"at":point,"rotation":integer}),&[]));
    sheet["junctions"]=count.clone();sheet["nets"]=array(closed_object_optional(json!({"name":text,"pins":strings}),&[]));
    sheet["erc"]=closed_object_optional(json!({"run":boolean,"findings":array(closed_object_optional(json!({"severity":text,"kind":text,"message":text,"at":point,"pins":strings,"fix":maybe}),&[])),
        "noConnects":strings,"powerFlags":strings}),&[]);
    let dictionary=json!({"type":"object","additionalProperties":text});
    sheet["board"]=closed_object_optional(json!({"layers":count,"placements":array(closed_object_optional(json!({"reference":maybe,"at":point,"bottom":boolean,"pads":array(pad)}),&[])),
        "tracks":count,"trackPaths":array(closed_object_optional(json!({"net":maybe,"layer":count,"width":integer,"points":array(point.clone())}),&[])),"vias":count,
        "zones":array(closed_object_optional(json!({"net":text,"layer":count,"outline":array(point.clone()),"pieces":count,"holes":count,"vertices":count,"area":number,
            "spokes":nullable(array(closed_object_optional(json!({"pad":text,"spokes":count}),&[])))}),&[])),"zonesStale":boolean,
        "netClasses":array(closed_object_optional(json!({"name":text,"clearance":integer,"trackWidth":integer,"viaDiameter":integer,"viaDrill":integer,"patterns":strings,"nets":strings}),&[])),
        "netClassOf":dictionary,"netClassAssigned":dictionary,"netClassNet":maybe,
        "findings":nullable(array(closed_object_optional(json!({"kind":text,"message":text}),&[]))),"selectedZone":nullable(count.clone()),
        "selectedPad":nullable(closed_object_optional(json!({"reference":maybe,"number":text}),&[])),
        "selectedSilk":nullable(closed_object_optional(json!({"reference":maybe,"index":count}),&[])),"message":text}),&[]);
    json!({"type":"object","oneOf":[closed_object_optional(json!({"target":{"type":"null"}}),&[]),closed_object_optional(symbol,&[]),closed_object_optional(pads,&[]),closed_object_optional(sheet,&[])]})
}
