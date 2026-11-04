//! Native routing for the shared files operation engine. Document roots are
//! owned by the app; configured roots and execution areas are owned by the host.
use super::{ops::Caller, CadService};
use crate::{
    files::{self, Request},
    tools::ToolOutput,
};
use serde_json::Value;
use std::{collections::BTreeMap, path::PathBuf};

pub struct Files {
    configured: Result<BTreeMap<String, PathBuf>, String>,
    gate: tokio::sync::Mutex<()>,
}
impl Files {
    pub fn new() -> Self {
        let configured = match std::env::var("BREP_MCP_FILE_ROOTS") {
            Ok(value) => serde_json::from_str::<BTreeMap<String, PathBuf>>(&value).map_err(|e| {
                format!("BREP_MCP_FILE_ROOTS must be a JSON object of name:path pairs: {e}")
            }),
            Err(std::env::VarError::NotPresent) => Ok(BTreeMap::new()),
            Err(e) => Err(format!("BREP_MCP_FILE_ROOTS: {e}")),
        }
        .and_then(|roots| {
            if roots
                .keys()
                .any(|k| matches!(k.as_str(), "documents" | "workspace" | "artifacts"))
            {
                return Err(
                    "configured file roots cannot replace documents/workspace/artifacts".into(),
                );
            }
            Ok(roots)
        });
        Self {
            configured,
            gate: tokio::sync::Mutex::new(()),
        }
    }
    pub async fn call(
        &self,
        service: &CadService,
        caller: &Caller,
        args: Value,
    ) -> Result<ToolOutput, String> {
        let request: Request = serde_json::from_value(args.clone()).map_err(|e| e.to_string())?;
        let readonly = request.operation.annotations().read_only;
        caller
            .require(&[if readonly {
                "host.files.read"
            } else {
                "host.files.write"
            }])
            .map_err(|e| e.message)?;
        let state = service
            .server
            .get()
            .and_then(std::sync::Weak::upgrade)
            .ok_or("the server is stopping")?;
        let _gate = self.gate.lock().await;
        if matches!(request.root.as_str(), "documents" | "workspace") {
            super::tools::ensure_session(service, &state.slot).await?;
            let session = crate::tools::app::current(&state.slot).await?;
            let reply = session.host.call_ok("files", args).await?;
            return Ok(ToolOutput::json(reply.result.unwrap_or(Value::Null)));
        }
        let mut roots = self.configured.clone()?;
        let base = state.session_root.join("file-artifacts");
        crate::host::private_dir(&base)?;
        let artifacts = if let Some(execution) = &caller.execution {
            files::relative(execution)?;
            if execution.contains('/') {
                return Err("invalid host execution identity".into());
            }
            let dir = base.join(execution);
            crate::host::private_dir(&dir)?;
            dir
        } else {
            base
        };
        roots.insert("artifacts".into(), artifacts);
        let backend = files::native::NativeFiles::new(roots)?;
        let reply = files::execute(&backend, &request)?;
        Ok(ToolOutput::json(
            serde_json::to_value(reply).map_err(|e| e.to_string())?,
        ))
    }
}
