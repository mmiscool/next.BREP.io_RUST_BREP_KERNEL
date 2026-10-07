//! Scoped, interpreter-independent history extension provider.
use super::{FeatureContext, FeatureResult};
use serde::{Deserialize, Serialize};
use std::{cell::RefCell, rc::Rc};
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PluginPin {
    pub id: String,
    pub version: String,
    pub api_version: u32,
    pub digest: String,
}
pub trait ExtensionProvider {
    fn identity(&self, pins: &[PluginPin], feature_type: &str) -> Result<String, String>;
    fn execute(&self, pins: &[PluginPin], ctx: &FeatureContext<'_>) -> FeatureResult;
    /// Separate typed annotation domain; never dispatched as a solid feature.
    fn execute_annotation(
        &self,
        _pins: &[PluginPin],
        annotation: &super::pmi::PmiAnnotation,
        _context: &super::pmi::PmiContext<'_>,
    ) -> Result<super::pmi::PluginAnnotationOutput, String> {
        Err(format!("unavailable plugin annotation provider for '{}'", annotation.kind))
    }
}
thread_local! {
    static PROVIDER:RefCell<Option<Rc<dyn ExtensionProvider>>>=RefCell::new(None);
    static PINS:RefCell<Vec<PluginPin>>=const {RefCell::new(Vec::new())};
}
pub fn with_provider<R>(provider: Rc<dyn ExtensionProvider>, f: impl FnOnce() -> R) -> R {
    struct Restore(Option<Rc<dyn ExtensionProvider>>);
    impl Drop for Restore {
        fn drop(&mut self) {
            PROVIDER.with(|p| *p.borrow_mut() = self.0.take());
        }
    }
    let _restore = Restore(PROVIDER.with(|p| p.replace(Some(provider))));
    f()
}
pub(crate) struct PinScope(Vec<PluginPin>);
impl Drop for PinScope {
    fn drop(&mut self) {
        PINS.with(|p| *p.borrow_mut() = std::mem::take(&mut self.0));
    }
}
pub(crate) fn enter(pins: &[PluginPin]) -> PinScope {
    PinScope(PINS.with(|p| p.replace(pins.to_vec())))
}
pub(crate) fn identity(kind: &str) -> String {
    if !kind.contains('/') {
        return String::new();
    }
    let provider = PROVIDER.with(|p| p.borrow().clone());
    let pins = PINS.with(|p| p.borrow().clone());
    match provider {
        Some(p) => p
            .identity(&pins, kind)
            .unwrap_or_else(|e| format!("unavailable:{e}")),
        None => "unavailable plugin provider".into(),
    }
}
pub(crate) fn execute(ctx: &FeatureContext<'_>) -> FeatureResult {
    let provider = PROVIDER.with(|p| p.borrow().clone());
    let pins = PINS.with(|p| p.borrow().clone());
    match provider {
        Some(p) => p.execute(&pins, ctx),
        None => ctx.fail(format!(
            "unavailable plugin provider for '{}'",
            ctx.feature_type
        )),
    }
}
/// Clone an upstream solid after the enclosing feature's reference fence.
pub fn clone_reference(ctx: &FeatureContext<'_>, name: &str) -> Result<crate::BrepSolid, String> {
    let handle = ctx
        .scene
        .resolve_solid(name)
        .ok_or_else(|| format!("unresolved plugin reference {name}"))?;
    crate::with_registered_solid_str(handle, |s| Ok(s.clone()))
}
/// Publish only after every output has been validated by the extension owner.
pub fn publish(mut solid: crate::BrepSolid, name: &str) -> super::AddedSolid {
    for (i, face) in solid
        .shells
        .iter_mut()
        .flat_map(|s| s.faces.iter_mut())
        .enumerate()
    {
        face.name = Some(format!("{name}:face{i}"));
    }
    super::features::common::register_added(solid, name)
}
pub fn transform(
    solid: &crate::BrepSolid,
    position: [f64; 3],
    rotation_degrees: [f64; 3],
    scale: [f64; 3],
) -> Result<crate::BrepSolid, String> {
    let m = super::features::common::compose_trs_matrix(
        position,
        rotation_degrees.map(f64::to_radians),
        scale,
        [0.; 3],
    );
    crate::transform_brep(solid, crate::AffineTransform::new(m)?, false)
}
/// Extension-backed component snapshots require replay against the active trusted
/// provider. A serialized display snapshot cannot establish package availability.
pub(crate) fn document_uses_extensions(value: &serde_json::Value) -> bool {
    match value {
        serde_json::Value::Object(o) => {
            o.get("type")
                .and_then(|v| v.as_str())
                .is_some_and(|t| t.contains('/'))
                || o.values().any(document_uses_extensions)
        }
        serde_json::Value::Array(a) => a.iter().any(document_uses_extensions),
        _ => false,
    }
}

/// Resolve using explicit document pins even when invoked outside history replay.
pub(crate) fn execute_annotation(
    annotation: &super::pmi::PmiAnnotation,
    context: &super::pmi::PmiContext<'_>,
) -> Result<super::pmi::PluginAnnotationOutput, String> {
    let provider = PROVIDER.with(|p| p.borrow().clone());
    provider.ok_or_else(|| format!("unavailable plugin annotation provider for '{}'", annotation.kind))?
        .execute_annotation(&context.request.plugins, annotation, context)
}
