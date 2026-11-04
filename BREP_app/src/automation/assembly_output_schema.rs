//! Assembly discovery/inference result shapes, following the serialized kernel
//! reports. No constraint roster or solver behavior is reimplemented here.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_required, array, vector};

fn text() -> Value { json!({"type":"string"}) }
fn count() -> Value { json!({"type":"integer","minimum":0}) }
// Null FIRST, unlike `schema_dsl::nullable`: kept so these schemas stay byte-identical.
fn null_or(schema: Value) -> Value { json!({"anyOf":[{"type":"null"},schema]}) }
fn refusal() -> Value { closed_object_required(json!({"ok":{"const":false},"error":text()}), &["ok","error"]) }

// Outer branches admit the MCP adapter's frame/notices/after fields. Nested
// report objects remain closed. Keeping the branches inline also makes their
// discriminated union available to the generated TypeScript bindings.
fn envelope(mut success: Value) -> Value {
    success["additionalProperties"] = json!(true);
    let mut failure = refusal(); failure["additionalProperties"] = json!(true);
    json!({"type":"object","oneOf":[success,failure]})
}

pub(super) fn catalogue() -> Value {
    let row = closed_object_required(json!({"type":text(),"shortName":text(),"longName":text(),"label":text(),"icon":text(),
        "inputParamsSchema":{"type":"object","additionalProperties":{"type":"object"},"description":"Per-parameter form descriptors from the live constraint catalogue; descriptor fields vary by parameter kind."}
    }), &["type","shortName","longName","label","icon","inputParamsSchema"]);
    closed_object_required(json!({"constraints":array(row)}), &["constraints"])
}

pub(super) fn parts() -> Value { closed_object_required(json!({"parts":array(text())}), &["parts"]) }

pub(super) fn inferable_types() -> Value {
    let row = closed_object_required(json!({"type":text(),"label":text(),"icon":text(),"longName":text(),"detects":text(),"defaultOn":{"type":"boolean"}}), &["type","label","icon","longName","detects","defaultOn"]);
    closed_object_required(json!({"types":array(row)}), &["types"])
}

fn candidate(suppressed: bool) -> Value {
    let pair = json!({"type":"array","items":{"type":"string"},"minItems":2,"maxItems":2});
    let mut row = closed_object_required(json!({"type":text(),"elements":pair,"components":pair,"detail":text()}), &["type","elements","components","detail"]);
    if suppressed {
        row["properties"]["reason"] = json!({"enum":["redundant","pair-fully-constrained"]});
        row["properties"]["why"] = text();
        row["required"].as_array_mut().unwrap().extend([json!("reason"),json!("why")]);
    }
    row
}

fn scan_report() -> Value {
    let gates = closed_object_required(json!({"planesApart":count(),"nearestPlaneGap":{"type":["number","null"]},"sameFacing":count(),"noOverlap":count(),"axesApart":count(),"nearestAxisGap":{"type":["number","null"]},"coaxialButApart":count(),"noExtent":count(),"alsoOnCarrier":count()}), &["planesApart","nearestPlaneGap","sameFacing","noOverlap","axesApart","nearestAxisGap","coaxialButApart","noExtent","alsoOnCarrier"]);
    closed_object_required(json!({"ok":{"const":true},"componentCount":count(),"pairsConsidered":count(),"pairsApart":count(),"pairsAlreadyConstrained":count(),"pairsOverBudget":count(),"facesScanned":count(),
        "byType":{"type":"object","additionalProperties":count()},"candidates":array(candidate(false)),"suppressed":array(candidate(true)),"gates":gates,"warnings":array(text())
    }), &["ok","componentCount","pairsConsidered","pairsApart","pairsAlreadyConstrained","pairsOverBudget","facesScanned","byType","candidates","suppressed","gates","warnings"])
}

pub(super) fn scan() -> Value { envelope(scan_report()) }

fn solve_report() -> Value {
    let axis = closed_object_required(json!({"point":vector(3),"direction":vector(3)}), &["point","direction"]);
    let freedom = closed_object_required(json!({"rotations":count(),"translations":count(),"axes":array(axis),"slides":array(vector(3))}), &["rotations","translations","axes","slides"]);
    let solved = closed_object_required(json!({"ok":{"const":true},"mates":count(),"strategy":text(),"iterations":count(),"maxResidual":{"type":"number"},"dof":count(),"rank":count(),"redundant":count(),"status":text(),"bodies":array(text()),"bodyFreedom":{"type":"object","additionalProperties":freedom},"movedComponents":array(text()),"movedSolids":array(closed_object_required(json!({"name":text(),"handle":count()}), &["name","handle"]))}), &["ok","mates","strategy","iterations","maxResidual","dof","rank","redundant","status","bodies","bodyFreedom","movedComponents","movedSolids"]);
    let empty = closed_object_required(json!({"ok":{"const":true},"mates":{"const":0}}), &["ok","mates"]);
    let mut failed = refusal();
    failed["properties"]["nonConvergent"] = text();
    failed["properties"]["conflict"] = array(text());
    failed["properties"]["suggestSuppress"] = text();
    json!({"oneOf":[{"type":"null"},empty,solved,failed]})
}

pub(super) fn apply() -> Value {
    let mut report = scan_report();
    report["description"] = json!("Constraint inference scan used to create this batch; the settled model evaluation report is under after.report.");
    envelope(closed_object_required(json!({"ok":{"const":true},"created":array(text()),"report":report,"solve":solve_report()}), &["ok","created","report","solve"]))
}

pub(super) fn mutation_empty() -> Value {
    let mut result = closed_object_required(json!({}), &[]);
    result["description"] = json!("No application payload; the MCP adapter adds frame and optional notices/after. Query assembly_state to inspect the result.");
    result
}

fn held_ref() -> Value {
    let suggestion = closed_object_required(json!({"ref":text(),"basis":text(),"preTicked":{"type":"boolean"}}), &["ref","basis","preTicked"]);
    closed_object_required(json!({"ref":text(),"reason":text(),"suggestion":{"oneOf":[{"type":"null"},suggestion]}}), &["ref","reason","suggestion"])
}

fn held_mate(persistent: bool) -> Value {
    let mut row = closed_object_required(json!({"id":text(),"type":text(),"refs":array(held_ref())}), &["id","type","refs"]);
    if persistent {
        for field in ["origin","from","to"] {
            row["properties"][field] = text();
            row["required"].as_array_mut().unwrap().push(json!(field));
        }
    }
    row
}

fn replacement_report() -> Value {
    let strings = json!({"type":"object","additionalProperties":{"type":"string"}});
    let report = closed_object_required(json!({
        "ok":{"const":true},"origin":text(),
        "reattached":array(closed_object_required(json!({"id":text(),"from":text(),"to":text()}), &["id","from","to"])),
        "components":array(text()),"from":strings,"to":text(),"lineage":strings,
        "kept":array(closed_object_required(json!({"id":text(),"type":text()}), &["id","type"])),
        "unresolved":array(held_mate(false)),"notes":array(text())
    }), &["ok","origin","reattached","components","from","to","lineage","kept","unresolved","notes"]);
    json!({"oneOf":[{"type":"null"},report]})
}

pub(super) fn replace_component() -> Value {
    let mut report = replacement_report();
    report["description"] = json!("Replacement receipt, or null while pending. Read assembly_replace_report after evaluation; after.report describes model evaluation, not replacement.");
    closed_object_required(json!({"partName":text(),"components":array(text()),"report":report}), &["partName","components","report"])
}

pub(super) fn replace_report() -> Value {
    closed_object_required(json!({"report":replacement_report(),"unresolved":array(held_mate(true))}), &["report","unresolved"])
}

pub(super) fn replace_unresolved() -> Value {
    closed_object_required(json!({"unresolved":array(held_mate(true))}), &["unresolved"])
}

pub(super) fn replace_accept() -> Value {
    closed_object_required(json!({"accepted":count(),"unresolved":array(held_mate(true))}), &["accepted","unresolved"])
}

// State/inspection contracts follow cmd_assembly, assembly exports/lifecycle,
// component placement and the revision-bound interference page producer.
fn fields(properties: Value) -> Value {
    let required: Vec<String> = properties.as_object().unwrap().keys().cloned().collect();
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}
fn boolean() -> Value { json!({"type":"boolean"}) }
fn number() -> Value { json!({"type":"number"}) }
fn strings() -> Value { array(text()) }
fn arbitrary_attributes() -> Value {
    json!({"type":"object","additionalProperties":true,"description":"User-authored attribute keys and JSON values are preserved by the document store."})
}
fn statuses() -> Value {
    array(fields(json!({"id":text(),"type":text(),"enabled":boolean(),"open":boolean(),
        "status":text(),"message":text(),"satisfied":boolean(),
        "error":{"description":"Persisted per-constraint error value, or null when absent."},
        "conflictWith":{"description":"Persisted conflict metadata; defaults to an empty array."},
        "suppressSuggested":boolean()})))
}
fn dof() -> Value {
    let nested = array(fields(json!({"id":text(),"type":text(),"enabled":boolean(),"status":text(),"message":text(),"satisfied":boolean()})));
    let freedom = fields(json!({"rotations":count(),"translations":count(),
        "axes":array(fields(json!({"point":vector(3),"direction":vector(3)}))),"slides":array(vector(3))}));
    let mut success = closed_object_required(json!({"ok":{"const":true},"mates":count(),"strategy":text(),"iterations":count(),
        "maxResidual":number(),"dof":count(),"rank":count(),"redundant":count(),"status":text(),
        "bodies":strings(),"bodyFreedom":{"type":"object","additionalProperties":freedom},
        "movedComponents":strings(),"movedSolids":array(fields(json!({"name":text(),"handle":count()}))),
        "nestedConstraints":nested}), &["ok","mates"]);
    // With no mapped mates, only ok/mates is emitted. Solver diagnostics are
    // present after a solve; nested constraints are independently optional.
    success["description"] = json!("Empty-mate or solved report; diagnostic fields are absent before a mapped solve.");
    let failure = closed_object_required(json!({"ok":{"const":false},"error":text(),"nonConvergent":text(),
        "conflict":strings(),"suggestSuppress":text(),"nestedConstraints":nested}), &["ok","error"]);
    json!({"oneOf":[{"type":"null"},success,failure]})
}

pub(super) fn state() -> Value {
    let entry = fields(json!({"type":text(),"inputParams":{},"persistentData":{},"enabled":boolean(),"open":boolean()}));
    fields(json!({"hasAssembly":boolean(),"modelRevision":count(),"evaluationStatus":{"enum":["pending","evaluated"]},
        "state":fields(json!({"constraints":array(entry),"idCounter":count()})),"statuses":statuses(),"dof":dof()}))
}
pub(super) fn solve() -> Value { fields(json!({"statuses":statuses(),"dof":dof()})) }
pub(super) fn selected() -> Value { fields(json!({"selected":strings()})) }
pub(super) fn attributes() -> Value { fields(json!({"attributes":arbitrary_attributes()})) }
pub(super) fn updated() -> Value { fields(json!({"outdated":count(),"missing":strings(),"refreshed":count()})) }
pub(super) fn component() -> Value {
    let matrix = json!({"type":"array","items":{"type":"number"},"minItems":16,"maxItems":16});
    let placement = fields(json!({"status":{"enum":["pending","evaluated","not_evaluated"]},"modelRevision":count(),
        "authored":{"description":"Original transform JSON, including expressions, or null when absent."},
        "requestedMatrix":null_or(matrix.clone()),"evaluatedMatrix":null_or(matrix),"solverAdjusted":null_or(boolean()),
        "frame":{"const":"assembly"},"lengthUnit":{"const":"mm"},"angleUnit":{"const":"degree"},"matrixLayout":{"const":"row_major"},"errors":strings()}));
    fields(json!({"id":text(),"partName":text(),"translate":null_or(vector(3)),"rotateEulerDeg":null_or(vector(3)),
        "placement":placement,"fixed":boolean(),"members":strings(),"attributes":arbitrary_attributes(),
        "displayName":{"description":"Name attribute value if present, otherwise the part name string."}}))
}
pub(super) fn interference() -> Value {
    let diagnostic = closed_object_required(json!({"operation":text(),"category":text(),"operands":strings(),"message":text(),"detailId":text()}), &["operation","category","operands"]);
    let pair = fields(json!({"index":count(),"a":text(),"b":text(),
        "status":{"enum":["unverified","verified_interference","verified_touching","verified_clearance"]},
        "volume":number(),"aHidden":boolean(),"bHidden":boolean(),"minDistance":null_or(number()),
        "nearMiss":boolean(),"diagnostics":array(diagnostic),"reason":null_or(text())}));
    fields(json!({"modelRevision":count(),"scope":strings(),"componentCount":count(),"budget":count(),
        "pairTotal":count(),"checkedPairs":count(),"pageStart":count(),"pairs":array(pair.clone()),
        "clearanceMm":number(),"nearMisses":array(pair.clone()),"outcomes":array(pair),"skipped":strings(),"unverified":strings(),
        "booleansRun":count(),"booleanOperations":count(),
        "notChecked":fields(json!({"count":count(),"fromPairIndex":count(),"status":{"const":"not_checked_budget_exhausted"},"reason":null_or(json!({"const":"budget_exhausted"}))})),
        "unresolvedCount":count(),"interferenceCount":count(),"complete":boolean(),"allClear":boolean(),
        "continuationToken":null_or(text()),"volumeToleranceMm3":number(),"contactToleranceMm":number(),"contactVerification":text()}))
}
