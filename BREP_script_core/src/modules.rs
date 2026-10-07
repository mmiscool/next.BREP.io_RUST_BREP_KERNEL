//! In-memory ESM execution, separate from the legacy global-function hook API.
use crate::{Failure, Logs};
use boa_engine::builtins::promise::PromiseState;
use boa_engine::module::{ModuleLoader, ModuleRequest, Referrer};
use boa_engine::{Context, JsNativeError, JsResult, JsValue, Module, Source};
use std::{cell::RefCell, collections::BTreeMap, path::Path, rc::Rc};

struct Loader {
    sources: BTreeMap<String, String>,
    parsed: RefCell<BTreeMap<String, Module>>,
}
impl ModuleLoader for Loader {
    async fn load_imported_module(
        self: Rc<Self>,
        referrer: Referrer,
        request: ModuleRequest,
        context: &RefCell<&mut Context>,
    ) -> JsResult<Module> {
        let spec = request.specifier().to_std_string_escaped();
        if !(spec.starts_with("./") || spec.starts_with("../"))
            || spec.contains('\\')
            || !request.attributes().is_empty()
        {
            return Err(JsNativeError::typ()
                .with_message(format!("unsupported package import {spec}"))
                .into());
        }
        // These are virtual package keys, not filesystem paths. In particular,
        // wasm32-unknown-unknown has no absolute filesystem path semantics.
        let importer = referrer
            .path()
            .and_then(Path::to_str)
            .ok_or_else(|| JsNativeError::typ().with_message("missing package referrer"))?;
        let mut parts: Vec<&str> = importer.split('/').collect();
        parts.pop();
        for part in spec.split('/') {
            match part {
                "" | "." => {}
                ".." => {
                    if parts.pop().is_none() {
                        return Err(JsNativeError::typ()
                            .with_message("module outside package")
                            .into());
                    }
                }
                part => parts.push(part),
            }
        }
        let mut cx = context.borrow_mut();
        self.parse(&parts.join("/"), &mut cx)
    }
}
impl Loader {
    fn parse(&self, key: &str, cx: &mut Context) -> JsResult<Module> {
        if key.is_empty()
            || key
                .split('/')
                .any(|p| p.is_empty() || p == "." || p == "..")
            || key.contains('\\')
            || key.contains(':')
        {
            return Err(JsNativeError::typ()
                .with_message("invalid package module path")
                .into());
        }
        if let Some(m) = self.parsed.borrow().get(key) {
            return Ok(m.clone());
        }
        let source = self.sources.get(key).ok_or_else(|| {
            JsNativeError::typ().with_message(format!("missing package module {key}"))
        })?;
        let module = Module::parse(
            Source::from_bytes(source).with_path(Path::new(key)),
            None,
            cx,
        )
        .map_err(|e| JsNativeError::syntax().with_message(format!("{key}: {e}")))?;
        self.parsed
            .borrow_mut()
            .insert(key.to_owned(), module.clone());
        Ok(module)
    }
}
/// Reject asynchronous callbacks rather than serializing a Promise as `{}`.
pub fn synchronous(value: JsValue) -> JsResult<JsValue> {
    if value
        .as_object()
        .is_some_and(|o| o.is::<boa_engine::builtins::promise::Promise>())
    {
        Err(JsNativeError::typ()
            .with_message("Promise results are unsupported in synchronous plugin callbacks")
            .into())
    } else {
        Ok(value)
    }
}
/// Load and evaluate a package-local ESM graph in a fresh, bounded context.
/// Host setup and callback invocation occur within the same owning context.
pub fn with_module<T>(
    sources: &BTreeMap<String, String>,
    entry: &str,
    setup: impl FnOnce(&mut Context, &Logs) -> JsResult<()>,
    invoke: impl FnOnce(&Module, &mut Context) -> JsResult<T>,
) -> Result<T, Failure> {
    let loader = Rc::new(Loader {
        sources: sources.clone(),
        parsed: RefCell::new(BTreeMap::new()),
    });
    let mut cx = Context::builder()
        .module_loader(loader.clone())
        .build()
        .map_err(|e| Failure {
            message: e.to_string(),
            logs: vec![],
        })?;
    cx.runtime_limits_mut()
        .set_loop_iteration_limit(crate::LOOP_ITERATION_LIMIT);
    cx.runtime_limits_mut()
        .set_recursion_limit(crate::RECURSION_LIMIT);
    let logs = Rc::new(RefCell::new(Vec::new()));
    let result = (|| {
        crate::install_console(&mut cx, &logs)?;
        setup(&mut cx, &logs)?;
        let module = loader.parse(entry, &mut cx)?;
        let promise = module.load_link_evaluate(&mut cx);
        cx.run_jobs()?;
        match promise.state() {
            PromiseState::Fulfilled(_) => {}
            PromiseState::Rejected(value) => return Err(boa_engine::JsError::from_opaque(value)),
            PromiseState::Pending => {
                return Err(JsNativeError::typ()
                    .with_message("pending module evaluation is unsupported")
                    .into())
            }
        }
        invoke(&module, &mut cx)
    })();
    result.map_err(|e| {
        let trace = e.to_string();
        let stack = trace.find("\n    at ").map(|i| &trace[i..]).unwrap_or("");
        Failure {
            message: format!("{entry}: {}{stack}", crate::describe(e, &mut cx)),
            logs: logs.borrow().clone(),
        }
    })
}
