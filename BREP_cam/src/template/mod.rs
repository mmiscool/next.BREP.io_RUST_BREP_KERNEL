//! Saved shared-model operations with explicit setup/tool/target rebinding.
use crate::*;
use crate::library::{LibraryUnits,validate_program_tools};
use serde::{Serialize,Deserialize};
use std::collections::{BTreeMap,BTreeSet};
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase",deny_unknown_fields)]
pub struct OperationTemplate {pub format:String,pub version:u32,pub name:String,pub units:LibraryUnits,pub operations:Vec<Operation>,pub tools:Vec<Tool>}
impl OperationTemplate {
    pub fn capture(name:impl Into<String>,units:LibraryUnits,operations:&[Operation],tools:&[Tool])->Result<Self,String> {
        validate_program_tools(tools)?;
        let referenced:BTreeSet<_>=operations.iter().map(|o|&o.tool).collect();
        let out=Self {format:"brep-operation-template".into(),version:1,name:name.into(),units,operations:operations.to_vec(),tools:tools.iter().filter(|t|referenced.contains(&t.id)).cloned().collect()};out.validate()?;Ok(out)
    }
    pub fn validate(&self)->Result<(),String> {
        if self.format!="brep-operation-template"||self.version!=1 {return Err("unsupported operation template format/version".into());}
        validate_program_tools(&self.tools)?;let mut ids=BTreeSet::new();
        if self.operations.is_empty() {return Err("operation template is empty".into());}
        for op in &self.operations {
            op.validate()?;if op.id.as_str().is_empty()||!ids.insert(&op.id) {return Err("empty/duplicate template operation id".into());}
            if !self.tools.iter().any(|t|t.id==op.tool) {return Err("template references a missing tool".into());}
        }Ok(())
    }
    pub fn to_json(&self)->Result<String,String> {self.validate()?;serde_json::to_string_pretty(self).map(|s|s+"\n").map_err(|e|e.to_string())}
    pub fn from_json(s:&str)->Result<Self,String> {let v:Self=serde_json::from_str(s).map_err(|e|e.to_string())?;v.validate()?;Ok(v)}
    /// Matching first prefers the saved number with identical kind/geometry,
    /// then a unique geometric match. Every body/face reference must be supplied;
    /// old document names never silently survive into the new setup. Coordinate
    /// parameters remain setup-local and units must match (no implicit scaling).
    pub fn apply(&self,setup:SetupId,id_prefix:&str,units:LibraryUnits,tools:&[Tool],targets:&BTreeMap<TargetRef,TargetRef>)->Result<Vec<Operation>,String> {
        self.validate()?;validate_program_tools(tools)?;
        if self.units!=units {return Err("template and destination length units differ".into());}
        if setup.as_str().trim().is_empty()||id_prefix.trim().is_empty() {return Err("setup and operation id prefix must be nonempty".into());}
        let mut mapping=BTreeMap::new();
        for saved in &self.tools {
            let matches:Vec<_>=tools.iter().filter(|t|t.kind==saved.kind&&t.geometry==saved.geometry).collect();
            let numbered=matches.iter().find(|t|t.number==saved.number).copied();
            let chosen=numbered.or_else(||if matches.len()==1 {Some(matches[0])}else{None}).ok_or_else(||format!("tool {} has no unique number/geometry match",saved.id))?;
            mapping.insert(saved.id.clone(),chosen.id.clone());
        }
        let rebind=|r:&TargetRef|->Result<TargetRef,String> {let n=targets.get(r).ok_or_else(||format!("missing target binding for {r:?}"))?;if n.solid.trim().is_empty()||n.face.as_ref().is_some_and(|f|f.trim().is_empty())||r.face.is_some()!=n.face.is_some() {return Err("invalid target binding".into());}Ok(n.clone())};
        let mut out=Vec::new();
        for (i,saved) in self.operations.iter().enumerate() {
            let mut op=saved.clone();op.id=format!("{id_prefix}-{}",i+1).into();op.setup=setup.clone();op.tool=mapping[&saved.tool].clone();
            op.targets.target_solids=saved.targets.target_solids.iter().map(|s|rebind(&TargetRef::solid(s)).map(|r|r.solid)).collect::<Result<_,_>>()?;
            op.targets.protected_solids=saved.targets.protected_solids.iter().map(|s|rebind(&TargetRef::solid(s)).map(|r|r.solid)).collect::<Result<_,_>>()?;
            op.targets.drive_faces=saved.targets.drive_faces.iter().map(&rebind).collect::<Result<_,_>>()?;op.targets.avoid_faces=saved.targets.avoid_faces.iter().map(&rebind).collect::<Result<_,_>>()?;
            if op.targets.drive_faces.iter().any(|r|!op.targets.target_solids.contains(&r.solid)) {return Err("rebound drive face lies outside the rebound target bodies".into());}
            op.validate()?;out.push(op);
        }Ok(out)
    }
}
