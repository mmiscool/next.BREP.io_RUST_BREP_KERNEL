use super::*;

// Expression edits rerun the current history prefix so parameters that reference
// variables rebuild against the updated environment.
impl EngineState {
    /// The history's `expressions` source string (the variable sheet the panel's
    /// editor binds to). Raw source text — despite the `_json` suffix it mirrors
    /// the other engine readouts' naming; the verifier reads it verbatim.
    pub fn expressions_json(&self) -> String {
        self.history.expressions()
    }

    /// Replace the `expressions` source and re-run the rolled-to prefix so every
    /// feature param referencing a variable (e.g. `sizeX = "boxW"`) rebuilds with
    /// the new value — the panel's live-update path. Returns the build-report JSON.
    pub fn set_expressions(&mut self, expressions: &str) -> String {
        self.history.set_expressions(expressions);
        self.rerun_history()
    }

    /// Show the model evaluated with `preview`'s expressions source instead of
    /// the document's own, or (`None`) with the document's own again. Runs the
    /// history when what the runs evaluate changes; re-arming the same source
    /// runs nothing. Returns the build report.
    ///
    /// TRANSIENT by construction: the source is substituted into the run
    /// REQUEST only ([`Self::rerun_history`]), never into the history, so the
    /// document, its undo stacks, its dirty state and what Save writes are
    /// untouched.
    ///
    /// The preview is armed against the history's current
    /// [`edit_serial`](crate::history::History::edit_serial). The first run
    /// after the user edits the model (or undoes, or redoes) drops it, so the
    /// edit rebuilds against the document's own values and nothing previewed
    /// is shown under it. A host that edits the document ITSELF while
    /// previewing (the family table writing a cell) re-arms after the write.
    pub fn set_expression_preview(&mut self, preview: Option<ExpressionPreview>) -> String {
        let serial = self.history.edit_serial();
        // What the display was last BUILT with: the stored preview even when
        // an edit has made it stale, so ending a preview that a block write
        // (which runs nothing) left on screen still rebuilds.
        let before = self.expression_preview.as_ref().map(|p| p.expressions.clone());
        let after = preview.as_ref().map(|p| p.expressions.clone());
        self.expression_preview = preview.map(|p| ExpressionPreview { serial, ..p });
        if before == after {
            return self.history_report.clone();
        }
        self.rerun_history()
    }

    /// Re-arm the stored preview at the history's current edit serial,
    /// running nothing: for a host that has just edited the document itself
    /// (the family table committing a cell) and keeps previewing.
    pub fn rearm_expression_preview(&mut self) {
        let serial = self.history.edit_serial();
        if let Some(preview) = self.expression_preview.as_mut() {
            preview.serial = serial;
        }
    }

    /// The preview in force: `None` when there is none, or when the user has
    /// edited the model since it was armed (the next run drops it).
    pub fn expression_preview(&self) -> Option<&ExpressionPreview> {
        self.live_expression_preview()
    }

    fn live_expression_preview(&self) -> Option<&ExpressionPreview> {
        self.expression_preview
            .as_ref()
            .filter(|p| p.serial == self.history.edit_serial())
    }

    /// Drop a preview the user has edited the model under.
    pub(super) fn drop_stale_expression_preview(&mut self) {
        if self.live_expression_preview().is_none() {
            self.expression_preview = None;
        }
    }

    /// The rolled-to prefix as a run REQUEST: [`History::prefix_request`]
    /// with the live preview's expressions in place of the document's. Every
    /// execution of the model — the display run and the main-side re-runs
    /// that measure, export or pick against it — builds from this, so what is
    /// measured is what is shown.
    ///
    /// [`History::prefix_request`]: crate::history::History::prefix_request
    pub(super) fn run_request_value(&self) -> serde_json::Value {
        let mut request = self.history.prefix_request();
        if let (Some(preview), Some(object)) = (self.live_expression_preview(), request.as_object_mut()) {
            object.insert("expressions".into(), serde_json::Value::String(preview.expressions.clone()));
        }
        request
    }

    /// The history's `configurator` object (typed named inputs) as JSON — a
    /// read/display surface for the panel; deeper configurator editing is deferred.
    pub fn configurator_json(&self) -> String {
        self.history.configurator().to_string()
    }

    /// The parsed variable list for the sheet's name/value view:
    /// `[{ "name": "...", "expr": "..." }, …]` — one entry per `name = rhs;`
    /// assignment in the expressions source, in source order. The RHS is shown
    /// verbatim (its DEFINING expression); evaluating it to a live scalar needs the
    /// kernel's private `Env`, so a computed value column is deferred — the text
    /// editor + re-run is the source of truth for applied values.
    pub fn expression_variables_json(&self) -> String {
        let vars = parse_expression_variables(&self.history.expressions());
        serde_json::to_string(&vars).unwrap_or_else(|_| "[]".to_string())
    }
}

/// Parse the `name = rhs;` assignments from an expressions source into an ordered
/// `[{name, expr}]` list for the sheet's variable view. Mirrors the kernel
/// evaluator's statement grammar (`IDENT '=' expr ';'`) without reaching into its
/// private `Env`: `//` line comments are stripped, statements split on `;`, and
/// each `IDENT = rhs` yields one entry (a non-identifier LHS or empty RHS is
/// skipped, matching what the evaluator would reject).
fn parse_expression_variables(source: &str) -> Vec<serde_json::Value> {
    // Strip `//` line comments line-by-line (the DSL has no strings, so a bare
    // `//` always starts a comment), then split statements on ';'.
    let mut cleaned = String::with_capacity(source.len());
    for line in source.lines() {
        let code = match line.find("//") {
            Some(idx) => &line[..idx],
            None => line,
        };
        cleaned.push_str(code);
        cleaned.push('\n');
    }
    let mut out = Vec::new();
    for stmt in cleaned.split(';') {
        let stmt = stmt.trim();
        if stmt.is_empty() {
            continue;
        }
        let Some(eq) = stmt.find('=') else {
            continue;
        };
        let name = stmt[..eq].trim();
        let expr = stmt[eq + 1..].trim();
        if name.is_empty() || expr.is_empty() || !is_identifier(name) {
            continue;
        }
        out.push(serde_json::json!({ "name": name, "expr": expr }));
    }
    out
}

/// Whether `s` is a single expression-DSL identifier (`[A-Za-z_$][A-Za-z0-9_$]*`)
/// — the valid LHS of an assignment statement.
fn is_identifier(s: &str) -> bool {
    let mut chars = s.chars();
    match chars.next() {
        Some(c) if c.is_ascii_alphabetic() || c == '_' || c == '$' => {}
        _ => return false,
    }
    chars.all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$')
}



/// A transient expressions source for [`EngineState::set_expression_preview`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExpressionPreview {
    /// The whole expressions source the runs evaluate while it is in force.
    pub expressions: String,
    /// What is being previewed, for the working indicator (a part number).
    pub label: String,
    /// The history's edit serial it was armed at (set by the engine).
    pub serial: u64,
}

impl ExpressionPreview {
    pub fn new(expressions: impl Into<String>, label: impl Into<String>) -> Self {
        Self { expressions: expressions.into(), label: label.into(), serial: 0 }
    }
}
