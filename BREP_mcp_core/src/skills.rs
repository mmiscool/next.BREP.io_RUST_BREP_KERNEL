//! Allowlisted, versioned workflow instructions served by the Skills extension.
use serde_json::{json, Value};
pub const EXTENSION: &str = "io.modelcontextprotocol/skills";
pub struct Skill { pub name: &'static str, pub description: &'static str, pub text: &'static str }
pub const SKILLS: &[Skill] = &[
    Skill { name: "inspect-model", description: "Inspect BREP documents, units, solid validity, bounds and volume before choosing an edit.", text: include_str!("skills/inspect-model/SKILL.md") },
    Skill { name: "safe-parametric-edit", description: "Edit a BREP feature using document identity, revision checks and post-edit verification.", text: include_str!("skills/safe-parametric-edit/SKILL.md") },
    Skill { name: "import-step", description: "Import a STEP artifact into BREP and qualify the resulting solids and operation outcome.", text: include_str!("skills/import-step/SKILL.md") },
    Skill { name: "recover-execution", description: "Recover a lost BREP script response or reconcile failed and cancelled CAD work without duplicating changes.", text: include_str!("skills/recover-execution/SKILL.md") },
    Skill { name: "edit-assembly", description: "Edit BREP assembly components and constraints while preserving work-part context and component identity.", text: include_str!("skills/edit-assembly/SKILL.md") },
    Skill { name: "repair-feature", description: "Diagnose and repair a refused BREP feature using its actual schema, geometry references and refusal details.", text: include_str!("skills/repair-feature/SKILL.md") },
    Skill { name: "export-model", description: "Validate a BREP model and export it to an authorized format and destination.", text: include_str!("skills/export-model/SKILL.md") },
];
impl Skill {
    pub fn uri(&self) -> String { format!("skill://{}/SKILL.md", self.name) }
    pub fn entry(&self) -> Value {
        json!({"uri":self.uri(), "frontmatter":{"name":self.name,"description":self.description},
            "resources":[{"uri":self.uri(),"size":self.text.len(),"digest":format!("sha256:{}",crate::cad::canon::sha256_hex(self.text.as_bytes()))}]})
    }
}
pub fn get(uri: &str) -> Option<&'static Skill> { SKILLS.iter().find(|s| s.uri() == uri) }
pub fn request(method: &str, params: &Value) -> Result<Value, String> {
    let mut result = json!({"resultType":"complete","ttlMs":300000,"cacheScope":"public"});
    match method {
        "skills/list" => {
            if params.get("cursor").is_some() { return Err("no cursor is valid for this complete skill catalogue".into()); }
            result["skills"] = SKILLS.iter().map(Skill::entry).collect();
        }
        "skills/get" => result["skill"] = get(params["uri"].as_str().unwrap_or("")).ok_or("unknown skill URI")?.entry(),
        _ => return Err("unknown skills method".into()),
    }
    Ok(result)
}
