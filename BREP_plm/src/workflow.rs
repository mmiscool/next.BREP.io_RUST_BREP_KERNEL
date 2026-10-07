//! Versioned workflow definitions and durable DAG executions. Scripts run outside
//! the database lock; a persisted claim prevents concurrent duplicate dispatch.
use crate::{
    auth,
    db::{self, Db, State},
    model::{Lifecycle, User},
    scripting, Error,
};
use serde::{Deserialize, Serialize};
use serde_json::{json, Map, Value};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Field {
    pub key: String,
    pub label: String,
    pub kind: String,
    pub choices: Vec<String>,
    pub required_at: Vec<String>,
    pub readers: Vec<String>,
    pub writers: Vec<String>,
}
impl Default for Field {
    fn default() -> Self {
        Self {
            key: String::new(),
            label: String::new(),
            kind: "text".into(),
            choices: vec![],
            required_at: vec![],
            readers: vec!["*".into()],
            writers: vec!["initiator".into()],
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Node {
    pub id: String,
    pub label: String,
    pub kind: String,
    pub x: f64,
    pub y: f64,
    pub assignees: Vec<String>,
    /// 0 means every resolved reviewer; form steps need one completion.
    pub approvals: usize,
    pub allow_self_review: bool,
    pub code: String,
    pub status: Option<Lifecycle>,
}
impl Default for Node {
    fn default() -> Self {
        Self {
            id: String::new(),
            label: String::new(),
            kind: "form".into(),
            x: 0.,
            y: 0.,
            assignees: vec![],
            approvals: 0,
            allow_self_review: false,
            code: String::new(),
            status: None,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Edge {
    pub from: String,
    pub to: String,
    #[serde(default = "always")]
    pub on: String,
}
fn always() -> String {
    "always".into()
}
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Definition {
    pub id: String,
    pub name: String,
    pub description: String,
    pub kind: String,
    pub active: bool,
    pub require_for_release: bool,
    pub part_types: Vec<String>,
    pub start_groups: Vec<String>,
    pub fields: Vec<Field>,
    pub nodes: Vec<Node>,
    pub edges: Vec<Edge>,
    pub version: u64,
    pub updated_by: String,
    pub updated_at: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Decision {
    pub user_id: String,
    pub username: String,
    pub outcome: String,
    pub comment: String,
    pub at: u64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Step {
    pub state: String,
    pub outcome: String,
    pub assignees: Vec<String>,
    pub decisions: Vec<Decision>,
    pub attempt: u64,
}
impl Default for Step {
    fn default() -> Self {
        Self {
            state: "pending".into(),
            outcome: String::new(),
            assignees: vec![],
            decisions: vec![],
            attempt: 0,
        }
    }
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Event {
    pub at: u64,
    pub node: String,
    pub action: String,
    pub user_id: String,
    pub username: String,
    pub message: String,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Run {
    pub id: String,
    pub title: String,
    pub definition: Definition,
    pub state: String,
    pub part: String,
    pub revision: String,
    pub model_hash: String,
    pub initiator: String,
    pub initiator_name: String,
    pub created_at: u64,
    pub fields: Map<String, Value>,
    pub steps: BTreeMap<String, Step>,
    pub log: Vec<Event>,
}
fn event(run: &mut Run, node: &str, action: &str, user: &User, message: impl Into<String>) {
    run.log.push(Event {
        at: db::now(),
        node: node.into(),
        action: action.into(),
        user_id: user.id.clone(),
        username: user.username.clone(),
        message: message.into(),
    });
}
fn terminal(step: &Step) -> bool {
    matches!(step.state.as_str(), "done" | "skipped")
}
fn selected(edge: &Edge, step: &Step) -> bool {
    step.state == "done" && (edge.on == "always" || edge.on == step.outcome)
}
fn matches_user(selectors: &[String], user: &User, initiator: &str, assignees: &[String]) -> bool {
    selectors.iter().any(|s| {
        s == "*"
            || s == &format!("user:{}", user.id)
            || s == &format!("user:{}", user.username)
            || (s == "initiator" && user.id == initiator)
            || (s == "assignee" && assignees.contains(&user.id))
            || s.strip_prefix("group:")
                .is_some_and(|g| user.groups.iter().any(|s| s == g))
    })
}
fn visible(run: &Run, user: &User) -> bool {
    user.is_admin()
        || run.initiator == user.id
        || run.steps.values().any(|s| s.assignees.contains(&user.id))
}
fn field_allowed(field: &Field, run: &Run, user: &User, write: bool, assignees: &[String]) -> bool {
    user.is_admin()
        || matches_user(
            if write {
                &field.writers
            } else {
                &field.readers
            },
            user,
            &run.initiator,
            assignees,
        )
}
fn check_value(field: &Field, value: &Value) -> Result<(), Error> {
    if value.is_null() {
        return Ok(());
    }
    let valid = match field.kind.as_str() {
        "text" | "textarea" => value.as_str().is_some_and(|s| s.len() <= 20000),
        "number" => value.as_f64().is_some_and(f64::is_finite),
        "boolean" => value.is_boolean(),
        "choice" => value
            .as_str()
            .is_some_and(|s| field.choices.iter().any(|c| c == s)),
        _ => false,
    };
    if valid {
        Ok(())
    } else {
        Err(Error::bad_request(format!(
            "invalid value for {}",
            field.label
        )))
    }
}
fn fields_patch(
    run: &mut Run,
    user: &User,
    node: &str,
    patch: &Map<String, Value>,
    trusted: bool,
) -> Result<(), Error> {
    let assignees = run
        .steps
        .get(node)
        .map(|s| s.assignees.clone())
        .unwrap_or_default();
    for (key, value) in patch {
        let field = run
            .definition
            .fields
            .iter()
            .find(|f| &f.key == key)
            .ok_or_else(|| Error::bad_request(format!("unknown workflow field {key}")))?;
        if !trusted
            && (!field_allowed(field, run, user, true, &assignees)
                || !field_allowed(field, run, user, false, &assignees))
        {
            return Err(Error::forbidden(format!(
                "you cannot edit workflow field {key}"
            )));
        }
        check_value(field, value)?;
    }
    for (key, value) in patch {
        run.fields.insert(key.clone(), value.clone());
    }
    Ok(())
}
fn required(run: &Run, node: &str) -> Result<(), Error> {
    for field in run
        .definition
        .fields
        .iter()
        .filter(|f| f.required_at.iter().any(|n| n == node))
    {
        if run
            .fields
            .get(&field.key)
            .is_none_or(|v| v.is_null() || v.as_str().is_some_and(|s| s.trim().is_empty()))
        {
            return Err(Error::bad_request(format!(
                "{} is required at this step",
                field.label
            )));
        }
    }
    Ok(())
}
fn identifier(id: &str) -> bool {
    !id.is_empty()
        && id.len() <= 80
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
}
fn check_selectors(state: &State, selectors: &[String], special: bool) -> Result<(), Error> {
    for selector in selectors {
        if special && matches!(selector.as_str(), "*" | "initiator" | "assignee") {
            continue;
        }
        if let Some(id) = selector.strip_prefix("user:") {
            if state.users.iter().any(|u| u.id == id || u.username == id) {
                continue;
            }
        }
        if selector
            .strip_prefix("group:")
            .is_some_and(|g| !g.trim().is_empty())
        {
            continue;
        }
        if selector == "initiator" {
            continue;
        }
        return Err(Error::bad_request(format!(
            "invalid workflow selector {selector}"
        )));
    }
    Ok(())
}
fn validate(state: &State, definition: &Definition) -> Result<(), Error> {
    if definition.name.trim().is_empty()
        || definition.name.len() > 200
        || definition.nodes.len() > 100
        || definition.edges.len() > 300
        || definition.fields.len() > 100
    {
        return Err(Error::bad_request(
            "workflow needs a name; maximum 100 nodes, 300 paths and 100 fields",
        ));
    }
    if definition.require_for_release && definition.kind != "release" {
        return Err(Error::bad_request(
            "only release workflows can be required for release",
        ));
    }
    if !matches!(
        definition.kind.as_str(),
        "general" | "release" | "change_request"
    ) {
        return Err(Error::bad_request(
            "choose general, release or change_request workflow",
        ));
    }
    for kind in &definition.part_types {
        if state.part_type(kind).is_none() {
            return Err(Error::bad_request("unknown workflow part type"));
        }
    }
    check_selectors(state, &definition.start_groups, true)?;
    let mut ids = BTreeSet::new();
    for node in &definition.nodes {
        if !identifier(&node.id)
            || !ids.insert(node.id.clone())
            || node.label.trim().is_empty()
            || !node.x.is_finite()
            || !node.y.is_finite()
            || node.x.abs() > 100000.
            || node.y.abs() > 100000.
        {
            return Err(Error::bad_request(
                "nodes need unique IDs, labels and finite positions",
            ));
        }
        if !matches!(
            node.kind.as_str(),
            "start" | "end" | "form" | "review" | "script" | "check" | "status" | "fork" | "join"
        ) {
            return Err(Error::bad_request("unknown workflow node kind"));
        }
        if matches!(node.kind.as_str(), "form" | "review") && node.assignees.is_empty() {
            return Err(Error::bad_request("human steps need assignees"));
        }
        check_selectors(state, &node.assignees, false)?;
        if matches!(node.kind.as_str(), "script" | "check")
            && (node.code.trim().is_empty() || node.code.len() > 64000)
        {
            return Err(Error::bad_request(
                "script steps need JavaScript (maximum 64 KB)",
            ));
        }
        if node.kind == "status"
            && (node.status.is_none() || node.status == Some(Lifecycle::Superseded))
        {
            return Err(Error::bad_request(
                "choose a lifecycle status for the status step",
            ));
        }
    }
    let starts: Vec<_> = definition
        .nodes
        .iter()
        .filter(|n| n.kind == "start")
        .collect();
    if starts.len() != 1 || !definition.nodes.iter().any(|n| n.kind == "end") {
        return Err(Error::bad_request(
            "workflow needs one Start and at least one End",
        ));
    }
    let mut links = BTreeSet::new();
    for edge in &definition.edges {
        if !ids.contains(&edge.from)
            || !ids.contains(&edge.to)
            || edge.from == edge.to
            || edge.on.trim().is_empty()
            || edge.on.len() > 80
            || !links.insert((&edge.from, &edge.to, &edge.on))
        {
            return Err(Error::bad_request(
                "paths need valid, distinct nodes and outcome names",
            ));
        }
    }
    for node in &definition.nodes {
        if node.kind == "start" && definition.edges.iter().any(|e| e.to == node.id) {
            return Err(Error::bad_request("Start cannot have incoming paths"));
        }
        if node.kind == "end" && definition.edges.iter().any(|e| e.from == node.id) {
            return Err(Error::bad_request("End cannot have outgoing paths"));
        }
        if node.kind != "end" && !definition.edges.iter().any(|e| e.from == node.id) {
            return Err(Error::bad_request(
                "each non-End step needs an outgoing path",
            ));
        }
    }
    // Topological sort also rejects loops. Every node must be reachable.
    let mut seen = BTreeSet::new();
    for _ in 0..definition.nodes.len() {
        for node in &definition.nodes {
            if (node.id == starts[0].id || definition.edges.iter().any(|e| e.to == node.id))
                && definition
                    .edges
                    .iter()
                    .filter(|e| e.to == node.id)
                    .all(|e| seen.contains(&e.from))
            {
                seen.insert(node.id.clone());
            }
        }
    }
    if seen.len() != ids.len() {
        return Err(Error::bad_request(
            "all steps must connect to Start, without cycles",
        ));
    }
    let mut keys = BTreeSet::new();
    for field in &definition.fields {
        if !identifier(&field.key)
            || !keys.insert(&field.key)
            || field.label.trim().is_empty()
            || !matches!(
                field.kind.as_str(),
                "text" | "textarea" | "number" | "boolean" | "choice"
            )
        {
            return Err(Error::bad_request(
                "fields need unique keys, labels and a supported type",
            ));
        }
        if field.kind == "choice" && field.choices.is_empty() {
            return Err(Error::bad_request("choice fields need options"));
        }
        for node in &field.required_at {
            if node != "start" && !ids.contains(node) {
                return Err(Error::bad_request(
                    "required fields must name a workflow step or start",
                ));
            }
        }
        check_selectors(state, &field.readers, true)?;
        check_selectors(state, &field.writers, true)?;
    }
    Ok(())
}
impl Db {
    pub fn save_workflow(
        &self,
        user: &User,
        mut definition: Definition,
    ) -> Result<Definition, Error> {
        if !user.is_admin() {
            return Err(Error::forbidden("workflow editing needs admin"));
        }
        if definition.id.is_empty() {
            definition.id = auth::new_id();
        }
        definition.name = definition.name.trim().into();
        self.mutate(|state| {
            validate(state,&definition)?;
            if self.security().config.lock_script_editor || !state.settings.script_editor_enabled {
                let previous=state.workflows.iter().find(|d|d.id==definition.id);
                for node in definition.nodes.iter().filter(|n|matches!(n.kind.as_str(),"script"|"check")) {
                    if !previous.is_some_and(|d|d.nodes.iter().any(|n|n.id==node.id && n.code==node.code && matches!(n.kind.as_str(),"script"|"check"))) {
                        return Err(Error::forbidden("workflow JavaScript editing is disabled by server script-editor policy"));
                    }
                }
            }
            if let Some(old)=state.workflows.iter().find(|d|d.id==definition.id) {
                if definition.version!=old.version{return Err(Error::conflict("workflow changed; reload before saving"));}
            } else if definition.version!=0 {return Err(Error::conflict("workflow version does not exist"));}
            definition.version+=1;definition.updated_by=user.id.clone();definition.updated_at=db::now();
            state.workflows.retain(|d|d.id!=definition.id);state.workflows.push(definition.clone());Ok(definition)
        })
    }
    pub fn start_workflow(
        &self,
        user: &User,
        key: &str,
        title: &str,
        part: &str,
        revision: &str,
        fields: Map<String, Value>,
    ) -> Result<String, Error> {
        if title.len() > 200 {
            return Err(Error::bad_request("workflow title is too long"));
        }
        let id = auth::new_id();
        self.mutate(|state| {
            let definition = state
                .workflows
                .iter()
                .find(|d| d.id == key && d.active)
                .cloned()
                .ok_or_else(|| Error::not_found("active workflow"))?;
            if !user.is_admin()
                && if definition.start_groups.is_empty() {
                    !user.can_author()
                } else {
                    !matches_user(&definition.start_groups, user, &user.id, &[])
                }
            {
                return Err(Error::forbidden("you cannot start this workflow"));
            }
            let mut part_id = String::new();
            let mut rev_id = String::new();
            let mut model_hash = String::new();
            if !part.is_empty() {
                let part = state
                    .part_by_id_or_number(part)
                    .ok_or_else(|| Error::not_found("workflow part"))?;
                if !definition.part_types.is_empty()
                    && !definition.part_types.contains(&part.part_type)
                {
                    return Err(Error::forbidden(
                        "workflow is not available for this part type",
                    ));
                }
                let revision = if revision.is_empty() {
                    part.latest()
                } else {
                    part.revision(revision)
                }
                .ok_or_else(|| Error::not_found("workflow revision"))?;
                part_id = part.id.clone();
                rev_id = revision.id.clone();
                model_hash = revision.content_hash.clone();
            } else if !definition.part_types.is_empty()
                || definition.nodes.iter().any(|n| n.kind == "status")
            {
                return Err(Error::bad_request(
                    "this workflow requires a part and revision",
                ));
            }
            let mut run = Run {
                id: id.clone(),
                title: if title.trim().is_empty() {
                    definition.name.clone()
                } else {
                    title.trim().into()
                },
                definition,
                state: "active".into(),
                part: part_id,
                revision: rev_id,
                model_hash,
                initiator: user.id.clone(),
                initiator_name: user.username.clone(),
                created_at: db::now(),
                fields: Map::new(),
                steps: BTreeMap::new(),
                log: vec![],
            };
            for node in &run.definition.nodes {
                let assignees: Vec<_> = state
                    .users
                    .iter()
                    .filter(|u| {
                        u.active
                            && matches_user(&node.assignees, u, &run.initiator, &[])
                            && (node.kind != "review"
                                || node.allow_self_review
                                || u.id != run.initiator)
                    })
                    .map(|u| u.id.clone())
                    .collect();
                if matches!(node.kind.as_str(), "form" | "review")
                    && (assignees.is_empty() || node.approvals > assignees.len())
                {
                    return Err(Error::bad_request(format!(
                        "{} needs eligible assignees (self-review is disabled unless configured)",
                        node.label
                    )));
                }
                run.steps.insert(
                    node.id.clone(),
                    Step {
                        assignees,
                        ..Step::default()
                    },
                );
            }
            fields_patch(&mut run, user, "start", &fields, false)?;
            required(&run, "start")?;
            let version = run.definition.version;
            event(
                &mut run,
                "",
                "started",
                user,
                format!("Workflow version {version}"),
            );
            state.workflow_runs.push(run);
            Ok(())
        })?;
        self.drive_workflow(&id)?;
        Ok(id)
    }
    /// Return a redacted smart form and execution graph, never trusted code.
    pub fn workflow_view(&self, user: &User, id: &str) -> Result<Value, Error> {
        self.read(|state| {
            let run = state
                .workflow_runs
                .iter()
                .find(|r| r.id == id)
                .ok_or_else(|| Error::not_found("workflow run"))?;
            if !visible(run, user) {
                return Err(Error::forbidden("this workflow is not assigned to you"));
            }
            let assignees: Vec<_> = run
                .steps
                .values()
                .flat_map(|s| s.assignees.clone())
                .collect();
            let mut output = serde_json::to_value(run).map_err(Error::internal)?;
            let readable: BTreeSet<_> = run
                .definition
                .fields
                .iter()
                .filter(|f| field_allowed(f, run, user, false, &assignees))
                .map(|f| f.key.clone())
                .collect();
            output["fields"]
                .as_object_mut()
                .unwrap()
                .retain(|k, _| readable.contains(k));
            output["definition"]["fields"]
                .as_array_mut()
                .unwrap()
                .retain(|f| readable.contains(f["key"].as_str().unwrap_or("")));
            for field in output["definition"]["fields"].as_array_mut().unwrap() {
                let f = run
                    .definition
                    .fields
                    .iter()
                    .find(|f| f.key == field["key"])
                    .unwrap();
                field["can_edit"] = json!(field_allowed(f, run, user, true, &assignees));
            }
            for node in output["definition"]["nodes"].as_array_mut().unwrap() {
                node.as_object_mut().unwrap().remove("code");
            }
            if !user.is_admin() {
                for entry in output["log"].as_array_mut().unwrap() {
                    if matches!(entry["action"].as_str(), Some("failed" | "script_log")) {
                        entry["message"] =
                            json!("Automation detail is available to administrators.");
                    }
                }
            }
            Ok(output)
        })
    }
    pub fn workflow_action(
        &self,
        user: &User,
        id: &str,
        node_id: &str,
        outcome: &str,
        comment: &str,
        fields: Map<String, Value>,
    ) -> Result<(), Error> {
        if !matches!(outcome, "success" | "reject") {
            return Err(Error::bad_request("choose success or reject"));
        }
        if comment.len() > 20000 {
            return Err(Error::bad_request("comment is too long"));
        }
        self.mutate(|state| {
            let current = state
                .workflow_runs
                .iter()
                .find(|r| r.id == id)
                .ok_or_else(|| Error::not_found("workflow run"))?;
            if current
                .definition
                .nodes
                .iter()
                .any(|n| n.id == node_id && n.kind == "review")
                && !current.part.is_empty()
                && state
                    .part(&current.part)
                    .and_then(|p| p.revision(&current.revision))
                    .is_none_or(|r| r.content_hash != current.model_hash)
            {
                return Err(Error::conflict(
                    "the model changed; start a new workflow to review its current content",
                ));
            }
            let run = state.workflow_runs.iter_mut().find(|r| r.id == id).unwrap();
            if run.state != "active" {
                return Err(Error::conflict("workflow is not active"));
            }
            let node = run
                .definition
                .nodes
                .iter()
                .find(|n| n.id == node_id)
                .cloned()
                .ok_or_else(|| Error::not_found("workflow step"))?;
            let step = run.steps.get(node_id).unwrap();
            if step.state != "waiting" || !step.assignees.contains(&user.id) {
                return Err(Error::forbidden("this step is not waiting on you"));
            }
            if step.decisions.iter().any(|d| d.user_id == user.id) {
                return Err(Error::conflict("you already completed this step"));
            }
            fields_patch(run, user, node_id, &fields, false)?;
            if outcome == "success" {
                required(run, node_id)?;
            }
            let step = run.steps.get_mut(node_id).unwrap();
            step.decisions.push(Decision {
                user_id: user.id.clone(),
                username: user.username.clone(),
                outcome: outcome.into(),
                comment: comment.into(),
                at: db::now(),
            });
            let threshold = if node.kind == "review" {
                if node.approvals == 0 {
                    step.assignees.len()
                } else {
                    node.approvals
                }
            } else {
                1
            };
            if outcome == "reject"
                || step
                    .decisions
                    .iter()
                    .filter(|d| d.outcome == "success")
                    .count()
                    >= threshold
            {
                step.state = "done".into();
                step.outcome = outcome.into();
            }
            event(
                run,
                node_id,
                if outcome == "success" {
                    "completed_by_user"
                } else {
                    "rejected_by_user"
                },
                user,
                comment,
            );
            if !fields.is_empty() {
                event(
                    run,
                    node_id,
                    "fields_updated",
                    user,
                    fields.keys().cloned().collect::<Vec<_>>().join(", "),
                );
            }
            Ok(())
        })?;
        self.drive_workflow(id)
    }
    pub fn control_workflow(
        &self,
        user: &User,
        id: &str,
        action: &str,
        node: &str,
    ) -> Result<(), Error> {
        self.mutate(|state| {
            let run=state.workflow_runs.iter_mut().find(|r|r.id==id).ok_or_else(||Error::not_found("workflow run"))?;
            if action=="cancel" {
                if !user.is_admin() && user.id!=run.initiator{return Err(Error::forbidden("only the initiator or admin can cancel"));}
                if matches!(run.state.as_str(),"completed"|"cancelled"|"rejected"){return Err(Error::conflict("workflow has ended"));}
                if run.steps.values().any(|s|s.state=="running"){return Err(Error::conflict("an automatic step is executing; wait for its result before cancelling"));}
                run.state="cancelled".into();event(run,"","cancelled",user,"");
            } else if action=="retry" {
                if !user.is_admin(){return Err(Error::forbidden("retry needs admin"));}
                if matches!(run.state.as_str(),"completed"|"cancelled"|"rejected"){return Err(Error::conflict("workflow has ended"));}
                let step=run.steps.get_mut(node).ok_or_else(||Error::not_found("workflow step"))?;
                if step.state!="failed"{return Err(Error::conflict("only a failed step can be retried"));}
                step.state="pending".into();run.state="active".into();event(run,node,"retry_requested",user,"Review any external side effects before retrying; scripts receive a stable idempotencyKey.");
            } else {return Err(Error::bad_request("unknown workflow control action"));}
            Ok(())
        })?;
        if action == "retry" {
            self.drive_workflow(id)?;
        }
        Ok(())
    }
    pub fn drive_workflow(&self, id: &str) -> Result<(), Error> {
        // A DAG runs each node once per attempt. Human work pauses dispatch;
        // all parallel human nodes are activated in the same pass.
        for _ in 0..101 {
            let job=self.mutate(|state| {
                let definition=state.workflow_runs.iter().find(|r|r.id==id).ok_or_else(||Error::not_found("workflow run"))?.definition.clone();
                let executor=state.user(&definition.updated_by).filter(|u|u.is_admin()).cloned().ok_or_else(||Error::conflict("workflow owner must be an active administrator; update the definition for new runs"))?;
                let names:BTreeMap<_,_>=state.users.iter().map(|u|(u.id.clone(),u.username.clone())).collect();
                let run=state.workflow_runs.iter_mut().find(|r|r.id==id).unwrap();
                if run.state!="active" {return Ok(None);}
                for _ in 0..definition.nodes.len()+1 {
                    let mut changed=false;
                    for node in &definition.nodes {
                        if run.steps[&node.id].state!="pending"{continue;}
                        let incoming:Vec<_>=definition.edges.iter().filter(|e|e.to==node.id).collect();
                        if !incoming.iter().all(|e|terminal(&run.steps[&e.from])){continue;}
                        let reached=node.kind=="start" || if node.kind=="join" { !incoming.is_empty() && incoming.iter().all(|e|selected(e,&run.steps[&e.from])) } else { incoming.iter().any(|e|selected(e,&run.steps[&e.from])) };
                        if !reached {
                            run.steps.get_mut(&node.id).unwrap().state="skipped".into();
                            event(run,&node.id,"skipped",&executor,"No selected incoming path");changed=true;continue;
                        }
                        if matches!(node.kind.as_str(),"form"|"review") {
                            run.steps.get_mut(&node.id).unwrap().state="waiting".into();
                            let assigned=run.steps[&node.id].assignees.iter().filter_map(|id|names.get(id)).cloned().collect::<Vec<_>>().join(", ");
                            event(run,&node.id,"assigned",&executor,assigned);changed=true;continue;
                        }
                        if let Err(error)=if matches!(node.kind.as_str(),"script"|"check") {Ok(())}else{required(run,&node.id)} {
                            run.steps.get_mut(&node.id).unwrap().state="failed".into();run.state="blocked".into();
                            event(run,&node.id,"failed",&executor,error.to_string());return Ok(None);
                        }
                        if matches!(node.kind.as_str(),"start"|"end"|"fork"|"join") {
                            let outcome=if node.kind=="end" && incoming.iter().any(|e|selected(e,&run.steps[&e.from]) && run.steps[&e.from].outcome=="reject"){"reject"}else{"success"};
                            let step=run.steps.get_mut(&node.id).unwrap();step.state="done".into();step.outcome=outcome.into();
                            event(run,&node.id,"completed",&executor,outcome);changed=true;continue;
                        }
                        let step=run.steps.get_mut(&node.id).unwrap();step.state="running".into();step.attempt+=1;
                        event(run,&node.id,"dispatched",&executor,format!("Idempotency key {}:{}",run.id,node.id));
                        return Ok(Some((run.clone(),node.clone(),executor)));
                    }
                    if !changed{break;}
                }
                if run.steps.values().all(terminal) {
                    if !definition.nodes.iter().any(|n|n.kind=="end" && run.steps[&n.id].state=="done") {
                        run.state="blocked".into();event(run,"","failed",&executor,"No End was reached; check outcome paths.");
                    } else {
                        run.state=if definition.nodes.iter().any(|n|n.kind=="end" && run.steps[&n.id].outcome=="reject"){"rejected"}else{"completed"}.into();
                        event(run,"","finished",&executor,run.state.clone());
                    }
                }
                Ok(None)
            })?;
            let Some((snapshot, node, executor)) = job else {
                return Ok(());
            };
            let result = self.execute_workflow_node(&snapshot, &node, &executor);
            self.mutate(|state| {
                let run = state
                    .workflow_runs
                    .iter_mut()
                    .find(|r| r.id == id)
                    .ok_or_else(|| Error::not_found("workflow run"))?;
                if run.steps[&node.id].state != "running"
                    || run.steps[&node.id].attempt != snapshot.steps[&node.id].attempt
                {
                    return Err(Error::conflict("workflow claim changed"));
                }
                match result {
                    Ok((outcome, fields, logs)) => {
                        let applied = fields_patch(run, &executor, &node.id, &fields, true)
                            .and_then(|_| required(run, &node.id));
                        if let Err(error) = applied {
                            run.steps.get_mut(&node.id).unwrap().state = "failed".into();
                            run.state = "blocked".into();
                            event(run, &node.id, "failed", &executor, error.to_string());
                        } else {
                            let step = run.steps.get_mut(&node.id).unwrap();
                            step.state = "done".into();
                            step.outcome = outcome.clone();
                            event(run, &node.id, "completed", &executor, outcome);
                            for line in logs {
                                event(run, &node.id, "script_log", &executor, line);
                            }
                            if !fields.is_empty() {
                                event(
                                    run,
                                    &node.id,
                                    "fields_updated",
                                    &executor,
                                    fields.keys().cloned().collect::<Vec<_>>().join(", "),
                                );
                            }
                        }
                    }
                    Err(error) => {
                        run.steps.get_mut(&node.id).unwrap().state = "failed".into();
                        run.state = "blocked".into();
                        event(run, &node.id, "failed", &executor, error.to_string());
                    }
                }
                Ok(())
            })?;
        }
        Err(Error::conflict("workflow dispatch limit reached"))
    }
    fn execute_workflow_node(
        &self,
        run: &Run,
        node: &Node,
        executor: &User,
    ) -> Result<(String, Map<String, Value>, Vec<String>), Error> {
        if node.kind == "status" {
            let warnings = self.transition_workflow(
                executor,
                &run.part,
                &run.revision,
                node.status.unwrap(),
                &run.model_hash,
                &run.id,
            )?;
            return Ok(("success".into(), Map::new(), warnings));
        }
        let input = json!({"runId":run.id,"nodeId":node.id,"idempotencyKey":format!("{}:{}",run.id,node.id),"fields":run.fields,"partId":run.part,"revisionId":run.revision,"modelHash":run.model_hash,"initiator":run.initiator_name,"attempt":run.steps[&node.id].attempt});
        match scripting::run_source(self, executor, &node.code, "workflow.js", "step", &input) {
            scripting::HookResult::Returned(success) => {
                if success.value == Value::Bool(false) {
                    return Ok(("reject".into(), Map::new(), success.logs));
                }
                let outcome = success
                    .value
                    .get("outcome")
                    .and_then(Value::as_str)
                    .unwrap_or("success")
                    .to_string();
                if outcome.trim().is_empty() || outcome.len() > 80 {
                    return Err(Error::bad_request(
                        "script outcome needs a nonempty name up to 80 bytes",
                    ));
                }
                let fields = match success.value.get("fields") {
                    Some(Value::Object(map)) => map.clone(),
                    Some(_) => return Err(Error::bad_request("script fields must be an object")),
                    None => Map::new(),
                };
                Ok((outcome, fields, success.logs))
            }
            scripting::HookResult::Refused(failure) => Err(Error::conflict(failure.message)),
            scripting::HookResult::Absent => {
                Err(Error::conflict("workflow step function is missing"))
            }
        }
    }
    /// Called once at boot. Never silently replay uncertain external effects.
    pub fn recover_workflows(&self) -> Result<(), Error> {
        let interrupted = self.read(|s| {
            s.workflow_runs
                .iter()
                .any(|r| r.steps.values().any(|step| step.state == "running"))
        });
        if interrupted {
            self.mutate(|state| {
                for run in &mut state.workflow_runs {
                    let nodes:Vec<_>=run.steps.iter().filter(|(_,s)|s.state=="running").map(|(id,_)|id.clone()).collect();
                    for id in nodes {
                        run.steps.get_mut(&id).unwrap().state="failed".into();run.state="blocked".into();
                        run.log.push(Event {at:db::now(),node:id,action:"interrupted".into(),user_id:String::new(),username:"server".into(),message:"Server stopped during dispatch; administrator must verify external effects before retrying.".into()});
                    }
                }
                Ok(())
            })?;
        }
        let active = self.read(|s| {
            s.workflow_runs
                .iter()
                .filter(|r| r.state == "active")
                .map(|r| r.id.clone())
                .collect::<Vec<_>>()
        });
        for id in active {
            if let Err(error) = self.drive_workflow(&id) {
                self.mutate(|state| {
                    let run = state.workflow_runs.iter_mut().find(|r| r.id == id).unwrap();
                    run.state = "blocked".into();
                    run.log.push(Event {
                        at: db::now(),
                        node: String::new(),
                        action: "failed".into(),
                        user_id: String::new(),
                        username: "server".into(),
                        message: error.to_string(),
                    });
                    Ok(())
                })?;
            }
        }
        Ok(())
    }
}

pub fn inbox(state: &State, user: &User) -> Vec<Value> {
    state.workflow_runs.iter().filter(|r|visible(r,user) && matches!(r.state.as_str(),"active"|"blocked")).flat_map(|run| {
        run.definition.nodes.iter().filter_map(|node| {
            let step=&run.steps[&node.id];
            let waiting=step.state=="waiting" && step.assignees.contains(&user.id) && !step.decisions.iter().any(|d|d.user_id==user.id);
            let failed=user.is_admin() && step.state=="failed";
            if !waiting && !failed {return None;}
            Some(json!({"kind":"workflow","target":run.id,"node":node.id,"title":run.title,"name":node.label,"state":step.state,"approvals":step.decisions.iter().filter(|d|d.outcome=="success").count(),"required_approvals":if node.approvals==0{step.assignees.len()}else{node.approvals},"initiator":run.initiator,"due":0,"overdue":false}))
        }).collect::<Vec<_>>()
    }).collect()
}
pub fn definition_view(definition: &Definition, user: &User) -> Value {
    let mut output = serde_json::to_value(definition).unwrap();
    if !user.is_admin() {
        output["can_start"] = json!(if definition.start_groups.is_empty() {
            user.can_author()
        } else {
            matches_user(&definition.start_groups, user, &user.id, &[])
        });
        for node in output["nodes"].as_array_mut().unwrap() {
            node.as_object_mut().unwrap().remove("code");
        }
        output["fields"].as_array_mut().unwrap().retain(|f| {
            let field = definition
                .fields
                .iter()
                .find(|v| v.key == f["key"])
                .unwrap();
            matches_user(&field.readers, user, &user.id, &[])
                && matches_user(&field.writers, user, &user.id, &[])
        });
    }
    output
}
pub fn run_list(state: &State, user: &User) -> Vec<Value> {
    state.workflow_runs.iter().filter(|r|visible(r,user)).rev().map(|r|json!({"id":r.id,"title":r.title,"state":r.state,"part":r.part,"revision":r.revision,"workflow":r.definition.name,"version":r.definition.version,"initiator":r.initiator_name,"created_at":r.created_at})).collect()
}

/// Optional mandatory release process, checked again under the transition lock.
pub fn release_gate(
    state: &State,
    part_id: &str,
    revision_id: &str,
    run_id: Option<&str>,
) -> Result<(), Error> {
    let part = state
        .part(part_id)
        .ok_or_else(|| Error::not_found("part"))?;
    let applicable: Vec<_> = state
        .workflows
        .iter()
        .filter(|d| {
            d.active
                && d.kind == "release"
                && d.require_for_release
                && (d.part_types.is_empty() || d.part_types.contains(&part.part_type))
        })
        .collect();
    if applicable.is_empty() {
        return Ok(());
    }
    let authorized = run_id
        .and_then(|id| state.workflow_runs.iter().find(|r| r.id == id))
        .is_some_and(|r| {
            r.state == "active"
                && r.part == part_id
                && r.revision == revision_id
                && applicable.iter().any(|d| d.id == r.definition.id)
                && r.definition.nodes.iter().any(|n| {
                    n.kind == "status"
                        && n.status == Some(Lifecycle::Released)
                        && r.steps[&n.id].state == "running"
                })
        });
    if authorized {
        Ok(())
    } else {
        Err(Error::conflict(
            "this part type requires release through an approved workflow",
        ))
    }
}
