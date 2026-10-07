# BREP_plugins

Headless, native/WASM JavaScript CAD packages, backed by Boa and exact kernel
solids. Provision trusted `PackageBundle` objects explicitly with `Runtime::new`.
Saved documents contain only `plugins: [PluginPin]`; loading a document never
installs its code. Preserve old bundles by digest when installing updates.

`Runtime::execute_history` scopes the kernel extension provider. For observed
history, exports and nested-library operations, run the ordinary kernel call
inside `Runtime::with_provider`. Scopes restore on return and unwinding. Each
history document selects its own exact pins; component snapshots containing
plugins conservatively replay instead of trusting cached display geometry.

`Registry` owns serializable immutable feature/action/workbench/panel/annotation descriptors.
`for_document` selects exact pins and rejects missing/duplicate dependencies.
`Runtime::from_validated` accepts only metadata produced by a trusted worker;
it checks package ownership without executing code on the receiving UI thread.

ESM supports package-local relative imports only. Each install/callback creates
a fresh interpreter, evaluates the real module graph and invokes its default or
named `install(app)` export. Installation is atomic. Callback registrations are
validated again against installed metadata before invocation. Synchronous
callbacks reject Promise/thenable returns. No DOM, filesystem, network, ambient
time or random API is supplied. Loop/recursion limits and host operation caps
are not hard memory or wall-clock isolation; native geometry cannot be preempted.

Feature context exposes `params`, `persistentData`, `featureId`, `units` (`mm`),
and `geometry`: `sphere({radius})`, `cube({sizeX,sizeY,sizeZ})`,
`cylinder({radius,height})` (+Y), `transform(handle,{position,rotationEuler,scale})`
(angles in degrees), `boolean('UNION'|'SUBTRACT'|'INTERSECT',a,b)`,
`reference(name)` and `query(handle)` (`{volume}`). References must appear in
saved parameters/persistent inputs and remain subject to the component fence.
Solids are owned locally until all returned outputs validate; exceptions discard
the arena without touching upstream solids. Outputs publish as `featureId:key`.
Opaque handles cannot be saved in persistent state or reused by another call.

Actions receive host input (`params`, `selection`, `document`) and return an
`ActionPlan`. `ctx.document.addFeature(type,params,persistentData)`,
`updateFeature(id,params)` and `deleteFeature(id)` only stage commands;
`ctx.notify(text)` stages notifications. The host must validate the whole plan,
recompute, check document/selection revision, commit pins and commands atomically,
and create a single undo checkpoint. The runtime does not mutate a document.

V1 schemas support number (finite min/max), string, boolean and
reference_selection. Unknown parameter keys except reserved string `id` fail.
Arbitrary callback predicates, async callbacks and geometry adapters beyond the
methods above are explicitly unsupported. Module evaluation
must settle in the job drain; asynchronous host work belongs in host transport.

The examples below show declarative panels and typed annotations.


## Declarative panels

`app.registerPanel({id,label,controls})` registers static host-rendered content.
IDs use the package namespace. Workbenches claim panel IDs through `panels`.
Controls are tagged by `type`:

```js
app.registerPanel({
  id: "org.example.shapes/toolsPanel", label: "Tools",
  controls: [
    {type: "text", text: "Create a parametric ball."},
    {type: "table", columns: ["Property", "Value"], rows: [["Units", "mm"]]},
    {type: "action", id: "quick", label: "Insert", action: "org.example.shapes/insertBall", params: {radius: 5}},
    {type: "form", id: "custom", label: "Custom ball", action: "org.example.shapes/insertBall", params: {radius: 3}}
  ]
});
```

Action/form control IDs are unique local keys. Both link a named same-package
registered action; forms derive fields from that action's schema. `params`
defaults to `{}`. A form can leave required fields for the user to fill; a static
action must supply valid parameters or schema defaults. No panel callback runs
during a UI frame. `addSidePanel` aliases `registerPanel`; `addToolbarButton`
aliases `registerAction`, including its required named ID/run callback. Declare
workbench membership explicitly. Aliases use the same registry and lifecycle.

Panels allow at most 128 controls; tables allow 32 columns, 256 rows, and 4096
bytes per cell. Text controls allow 16384 bytes. Unknown controls/properties,
foreign action IDs and unresolved workbench panels reject the whole package.
Registry deserialization defaults absent panel/annotation lists to empty, so
older validated metadata and existing v1 bundles remain supported.

## Typed annotations

`app.registerAnnotation({id,label,inputParamsSchema,execute(ctx)})` contributes a
PMI annotation provider, separate from solid features. The same pinned callback
runs synchronously during native/WASM worker replay; the Rust PMI adapter draws
the result. The public `Runtime::execute_annotation(pins, annotation, context)`
implements the kernel `ExtensionProvider` method of the same signature.

```js
app.registerAnnotation({
  id: "org.example.shapes/distance", label: "Distance",
  inputParamsSchema: {length: {type: "number", default_value: 5, min: 0.01}},
  execute(ctx) {
    const result = ctx.annotation.linear({
      text: "Length", a: [0,0,0], b: [ctx.params.length,0,0],
      defaultLabel: [0,2,0]
    });
    result.persistentData = {lastLength: result.value};
    return result;
  }
});
```

Context contains evaluated/defaulted `params`, `persistentData`, `annotationId`,
`units` (`mm`), and `dependencies`. Only schema-declared `reference_selection`
parameters populate `dependencies[referenceName].point` with a Rust-resolved
world point. Unresolved references fail before the callback. This context has
no solid handles or document mutation API.

Rust-backed constructors return validated `PluginAnnotationOutput` values:

- `ctx.annotation.note({text,position})`: world-positioned text.
- `ctx.annotation.leader({text,targets,defaultLabel,dot?})`: leader targets;
  `dot` defaults to false.
- `ctx.annotation.linear({text,a,b,defaultLabel,component?})`: dimension between
  two points, optional `component` `X`/`Y`/`Z`. Rust projects the endpoint onto
  that axis and computes `value` and `unit:mm`, matching the rendered span.

Callbacks may instead return the typed object directly:
`{text,geometry,defaultLabel,references:[],value:null,unit:"",persistentData:null}`.
The last four fields are optional. `geometry` is one of
`{kind:"note",position}`, `{kind:"leader",targets,dot}`, or
`{kind:"linear",a,b,component?}`. `defaultLabel` and every point are finite
three-number arrays. Coordinates are bounded to ±1e12; text is nonempty and at
most 16384 bytes; targets/references are capped at 256; unit is `""`, `"mm"` or
`"deg"`. Zero-length linear geometry, unprojected raw component geometry, other geometry
kinds, unknown result fields,
nonfinite JSON and Promise/thenable results fail explicitly. Output references
must name declared resolved dependencies; the runtime records all declared
references even when omitted by the callback. Input/output JSON is capped at
1 MiB, as is registration metadata. Primitive calls share the 256 host-operation
limit. Persistent data remains JSON, and changes are returned for the enclosing
host transaction to apply only after successful replay.
