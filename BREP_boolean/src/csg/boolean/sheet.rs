//! Non-manifold plan 1.6: trim a declared Sheet by the inside of a Solid.
//! The solid alone supplies inside/outside; the sheet is never a parity tool.
use super::*;

pub(super) fn trim(
    first: &BrepSolid,
    second: &BrepSolid,
    options: &BooleanOptions,
) -> Result<KernelOutcome<BrepSolid>, KernelRefusal> {
    let started = Instant::now();
    let (sheet, solid, operand) = match (first.kind, second.kind) {
        (crate::BodyKind::Sheet, crate::BodyKind::Solid) => (first, second, 0),
        (crate::BodyKind::Solid, crate::BodyKind::Sheet) => (second, first, 1),
        _ => return Err(KernelRefusal::input(KernelStage::Collect, "sheet_operand",
            "sheet trim requires one declared Sheet and one declared Solid")),
    };
    let open = one_use_edges(solid);
    if open > 0 {
        return Err(KernelRefusal::input(KernelStage::Collect, "operand_closure",
            format!("sheet trim: Solid operand is not a closed shell ({open} one-use edges)")));
    }
    for (name, body) in [("Sheet", sheet), ("Solid", solid)] {
        let issues = body.validate();
        if !issues.is_empty() {
            return Err(KernelRefusal::input(KernelStage::Collect, "sheet_trim_operand",
                format!("sheet trim: invalid {name} operand: {issues:?}")));
        }
    }
    let policy = boolean_policy(first, second, options)?;
    let imprint = build_imprints(first, second, &boolean_imprint_options(options, policy.model))?;
    let split = apply_edge_splits(sheet, operand, &imprint)?;
    let fragments = fragment_solid(&split, operand, &imprint)?;
    let classifier = SolidClassifier::new(solid, policy.model)
        .or_refuse(KernelStage::Select, "sheet.trim.classifier")?;
    let mut selected = Vec::new();
    for fragment in fragments {
        let class = classifier.classify(fragment.test_point)
            .or_refuse(KernelStage::Select, "sheet.trim.classify")?;
        if matches!(class.class, PointClass::In | PointClass::On) {
            selected.push(fragment);
        }
    }
    let operands = [(operand, &split), (1-operand, solid)].into_iter().collect();
    let mut value = assemble::assemble_open_fragments(selected, &operands, &imprint, policy.model)?;
    value.kind = crate::BodyKind::Sheet;
    // The open assembler keeps an unpaired closed edge face-local by marking
    // it degenerate. A Sheet's circular boundary has no second face to clear
    // that historical flag. Its measured nonzero length distinguishes a real
    // closed edge from a collapsed pole; no existing Solid assembly changes.
    for edge in &mut value.edges {
        if edge.degenerate && edge.start_vertex_id == edge.end_vertex_id
            && crate::curve_arc_length(&edge.curve, edge.t0, edge.t1)
                .or_refuse(KernelStage::Validate, "sheet.trim.edge_length")? > policy.model
        {
            edge.degenerate = false;
        }
    }
    let issues = value.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::new(
            RefusalClass::InvalidResultTopology { issues: issues.len() as u32 },
            KernelStage::Validate, format!("sheet trim: invalid result: {issues:?}")));
    }
    let value = crate::accept_sound_against(&[first, second], value, "sheet_trim")?;
    let mut diagnostics = KernelDiagnostics::default();
    diagnostics.count_n("sheet.trim", 1);
    diagnostics.measure_max("timing.total_ms", started.elapsed().as_secs_f64()*1000.0);
    Ok(KernelOutcome { value, diagnostics })
}
