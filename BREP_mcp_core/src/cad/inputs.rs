//! Export-option form state. Transport/execution integration owns the waiter.
//! This module does not export a document or send a client request.
use serde_json::{json, Map, Value};
use std::time::{Duration, Instant};

const MAX_PATH_CHARS: usize = 4096;
const INPUT_WAIT: Duration = Duration::from_secs(120);

tokio::task_local! {
    // Scoped to the originating tools/call future, never a shared last peer.
    static TASK_FORM_SUPPORTED: bool;
}

pub(crate) async fn with_task_form_support<F: std::future::Future>(supported: bool, future: F) -> F::Output {
    TASK_FORM_SUPPORTED.scope(supported, future).await
}

pub(crate) fn task_form_supported() -> bool {
    TASK_FORM_SUPPORTED.try_with(|supported| *supported).unwrap_or(false)
}

pub(crate) fn supports_task_form(context: &rmcp::service::RequestContext<rmcp::RoleServer>) -> bool {
    context.protocol_version().is_some_and(|version| version >= rmcp::model::ProtocolVersion::V_2026_07_28)
        && context.client_capabilities().is_some_and(|capabilities|
            capabilities.supports_tasks() && capabilities.elicitation.is_some_and(|elicitation| elicitation.form.is_some()))
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ExportOptions {
    pub format: String,
    pub path: String,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum Resolution {
    Accepted(ExportOptions),
    Declined,
    Cancelled,
    Expired,
}

#[derive(Debug)]
pub(crate) struct ExportForm {
    formats: Vec<String>,
    supplied: Map<String, Value>,
    missing: Vec<&'static str>,
}

impl ExportForm {
    /// `formats` comes from the live document_export schema, not client input.
    pub fn new(input: &Value, formats: Vec<String>) -> Result<Self, String> {
        if formats.is_empty() || formats.iter().any(String::is_empty) {
            return Err("export format catalogue is unavailable".into());
        }
        let supplied = input.as_object().ok_or("export options must be an object")?.clone();
        if supplied.keys().any(|key| !matches!(key.as_str(), "format" | "path")) {
            return Err("export options accept only format and path".into());
        }
        let form = Self { missing: ["format", "path"].into_iter()
            .filter(|field| !supplied.contains_key(*field)).collect(), supplied, formats };
        for (field, value) in &form.supplied { form.validate_field(field, value)?; }
        Ok(form)
    }

    fn validate_field(&self, field: &str, value: &Value) -> Result<(), String> {
        let text = value.as_str().ok_or_else(|| format!("{field} must be a string"))?;
        match field {
            "format" if !self.formats.iter().any(|format| format == text) =>
                Err("format is not offered by document_export".into()),
            "path" if text.trim().is_empty() || text.chars().count() > MAX_PATH_CHARS =>
                Err(format!("path must contain non-whitespace text and at most {MAX_PATH_CHARS} characters")),
            "format" | "path" => Ok(()),
            _ => Err(format!("unexpected export field {field}")),
        }
    }

    pub fn complete(&self) -> Option<ExportOptions> {
        self.missing.is_empty().then(|| ExportOptions {
            format: self.supplied["format"].as_str().unwrap().into(),
            path: self.supplied["path"].as_str().unwrap().into(),
        })
    }

    pub fn schema(&self) -> Value {
        let mut properties = Map::new();
        for field in &self.missing {
            let schema = match *field {
                "format" => json!({"type":"string","enum":self.formats}),
                _ => json!({"type":"string","minLength":1,"maxLength":MAX_PATH_CHARS,
                    "description":"Export filename or path; must contain non-whitespace text. No file is written by this form."}),
            };
            properties.insert((*field).into(), schema);
        }
        json!({"type":"object","properties":properties,"required":self.missing,"additionalProperties":false})
    }

    fn accept(&self, content: &Value) -> Result<ExportOptions, String> {
        let content = content.as_object().ok_or("accepted export input must be an object")?;
        if content.len() != self.missing.len() || content.keys().any(|key| !self.missing.contains(&key.as_str())) {
            return Err("response must contain exactly the requested missing fields".into());
        }
        let mut merged = self.supplied.clone();
        for (field, value) in content {
            self.validate_field(field, value)?;
            merged.insert(field.clone(), value.clone());
        }
        Ok(ExportOptions { format: merged["format"].as_str().unwrap().into(), path: merged["path"].as_str().unwrap().into() })
    }
}

pub(crate) struct PendingExport {
    id: String,
    form: ExportForm,
    deadline: Instant,
    resolution: Option<Resolution>,
}

impl PendingExport {
    pub fn new(id: String, form: ExportForm, now: Instant, execution_deadline: Instant) -> Result<Self, String> {
        if id.is_empty() || form.complete().is_some() { return Err("pending input needs an id and missing fields".into()); }
        if execution_deadline <= now { return Err("execution wall budget is exhausted".into()); }
        Ok(Self { id, form, deadline: execution_deadline.min(now + INPUT_WAIT), resolution: None })
    }

    pub fn schema(&self) -> Value { self.form.schema() }
    pub fn deadline(&self) -> Instant { self.deadline }

    pub fn input_requests(&mut self, now: Instant) -> Result<rmcp::model::InputRequests, String> {
        use rmcp::model::{ElicitRequest, ElicitRequestParams, ElicitationSchema, InputRequest};
        if self.resolution(now).is_some() { return Ok(Default::default()); }
        // The SDK's primitive form model omits additionalProperties. Exact
        // response keys are still enforced by ExportForm::accept on the server.
        let requested_schema: ElicitationSchema = serde_json::from_value(self.schema())
            .map_err(|error| format!("export input schema: {error}"))?;
        let request = InputRequest::Elicitation(ElicitRequest::new(ElicitRequestParams::FormElicitationParams {
            meta: None,
            message: "Choose the missing export options. This collects choices only; it does not write a file.".into(),
            requested_schema,
        }));
        Ok([(self.id.clone(), request)].into_iter().collect())
    }

    /// Polling expiry and cancellation use the same terminal state as updates.
    pub fn resolution(&mut self, now: Instant) -> Option<&Resolution> {
        if self.resolution.is_none() && now >= self.deadline { self.resolution = Some(Resolution::Expired); }
        self.resolution.as_ref()
    }

    pub fn cancel(&mut self, now: Instant) -> &Resolution {
        self.resolution(now);
        self.resolution.get_or_insert(Resolution::Cancelled)
    }

    /// Validation happens before consumption; callers can correct invalid input.
    pub fn respond(&mut self, id: &str, response: &Value, now: Instant) -> Result<&Resolution, String> {
        if id != self.id { return Err("input request is not outstanding for this execution".into()); }
        if self.resolution(now).is_some() { return Err("input request has already resolved or expired".into()); }
        let response = response.as_object().ok_or("elicitation response must be an object")?;
        if response.keys().any(|key| !matches!(key.as_str(), "action" | "content" | "_meta")) {
            return Err("unknown elicitation response field".into());
        }
        if response.get("_meta").is_some_and(|value| !value.is_object()) {
            return Err("elicitation response _meta must be an object".into());
        }
        let resolved = match response.get("action").and_then(Value::as_str) {
            Some("accept") => self.form.accept(response.get("content").ok_or("accepted input needs content")?).map(Resolution::Accepted)?,
            Some("decline") => Resolution::Declined,
            Some("cancel") => Resolution::Cancelled,
            _ => return Err("elicitation action must be accept, decline or cancel".into()),
        };
        self.resolution = Some(resolved);
        Ok(self.resolution.as_ref().unwrap())
    }
}

