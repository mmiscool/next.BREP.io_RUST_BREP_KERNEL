//! User-selected workflow prompts share the embedded skill text.
use rmcp::{ErrorData, model::{GetPromptRequestParams, GetPromptResult, ListPromptsResult, Prompt, PromptArgument, PromptMessage, Role}};

fn argument_names(name: &str) -> &'static [&'static str] {
    match name {
        "inspect-model" => &["document"],
        "safe-parametric-edit" | "repair-feature" => &["document", "feature"],
        "import-step" => &["document", "artifact"],
        "recover-execution" => &["executionId", "key"],
        "edit-assembly" => &["document", "component"],
        "export-model" => &["document", "format", "destination"],
        _ => &[],
    }
}

pub fn list(cursor: Option<&str>) -> Result<ListPromptsResult, ErrorData> {
    if cursor.is_some() {
        return Err(ErrorData::invalid_params("no cursor is valid for the complete workflow prompt catalogue", None));
    }
    let prompts = crate::skills::SKILLS.iter().map(|skill| {
        let mut arguments = vec![PromptArgument::new("request")
            .with_description("The user's requested outcome and constraints.").with_required(true)];
        arguments.extend(argument_names(skill.name).iter().map(|name| PromptArgument::new(*name)
            .with_description("Known workflow context; verify identifiers and destinations against the live document and user request.")
            .with_required(false)));
        Prompt::new(skill.name, Some(skill.description), Some(arguments))
    }).collect();
    Ok(ListPromptsResult::with_all_items(prompts).with_ttl_ms(300_000).with_cache_scope(rmcp::model::CacheScope::Public))
}

pub fn get(request: GetPromptRequestParams) -> Result<GetPromptResult, ErrorData> {
    let skill = crate::skills::SKILLS.iter().find(|skill| skill.name == request.name)
        .ok_or_else(|| ErrorData::invalid_params("unknown workflow prompt", None))?;
    let arguments = request.arguments.unwrap_or_default();
    if !arguments.get("request").and_then(|v| v.as_str()).is_some_and(|s| !s.trim().is_empty()) {
        return Err(ErrorData::invalid_params("request must be a nonempty string", None));
    }
    for (name, value) in &arguments {
        if name != "request" && !argument_names(skill.name).contains(&name.as_str()) {
            return Err(ErrorData::invalid_params(format!("unknown prompt argument: {name}"), None));
        }
        if !value.is_string() {
            return Err(ErrorData::invalid_params(format!("prompt argument {name} must be a string"), None));
        }
    }
    // Separate workflow text from quoted context. No system role, execution,
    // or additional authorization is created by retrieving a prompt.
    let mut result = GetPromptResult::new(vec![
        PromptMessage::new_text(Role::User, format!("Use this workflow for the requested task within the user's existing authorization. Retrieving it does not execute an operation or grant additional permissions.\n\n{}", skill.text)),
        PromptMessage::new_text(Role::User, format!("User-supplied workflow arguments (JSON data):\n{}", serde_json::to_string_pretty(&arguments).unwrap())),
    ]);
    result.description = Some(skill.description.into());
    Ok(result)
}

