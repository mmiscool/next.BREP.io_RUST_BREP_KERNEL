---
name: design-part
description: Design a BREP part or assembly well: modelling order, expressions, stable references, checkpoints, and what the kernel refuses with the workaround for each.
---

How to build parts and assemblies that are correct, editable and survive later changes. Every function below is a
script call (`await brep.<group>.<tool>(args)` inside `run_script`); read a feature's parameters with
`brep.features.feature_schema({type})` (or `brep://schema/features/{type}`) before you set them, and the sketch
format at `brep://schema/sketch`. Lengths are millimetres, angles degrees.

## Rules

1. **One document per part; join parts in an assembly.** Build each part in its own document
   (`brep.document.doc_new`, save it with `document_save`), then place it with `brep.assembly.component_insert` in an
   assembly document. Do not model several parts as bodies in one part document.
2. **Constrain assemblies to the design intent.** The first component is grounded. Add mates from
   `assembly_constraint_catalogue` with `assembly_add_constraint` so only the motion the design allows remains;
   `assembly_solve` reports the remaining degrees of freedom; read it and explain any freedom left on purpose. Check
   `interference_check` before you report the assembly done.
3. **Drive editable sizes from expressions.** Set `brep.history.expressions_set({script: "len = 60;\nthk = 6;"})`
   and pass the variable names as string parameters (`distance: "thk"`). Any size a user may want to change later
   (length, width, thickness, hole size, count, spacing) should be a named variable, so the change is one
   `expressions_set` call, not a new request. Sketch coordinates are numbers: a sketch that must follow a variable
   is rebuilt from the variable's value with `feature_set_persistent`.
4. **Fastener holes are Hole features** (`H`, with `holeType` SIMPLE, COUNTERSINK, COUNTERBORE or THREADED), never subtracted
   cylinders: the hole carries its diameter, depth and type for drawings, instructions and manufacturing.
5. **Repeat with patterns, not copies.** `PATTERN` (linear or circular) and `M` repeat solids such as bosses and
   lugs; repeated holes are one Hole feature over a sketch holding every centre point. Never add the same feature
   again by hand for each copy.
6. **Sketch the true geometry.** Use `arc`, `circle`, `ellipse` and `bezier` sketch geometry when the design is round
   or smooth; never approximate a curve with many short lines (the result is faceted, cannot be filleted cleanly and
   is not dimensioned as a radius).
7. **Drawings, PDFs, assembly guides and HTML instruction viewers come only from the application**
   (`sheet_*`, `pmi_*`, `instructions_*`, `document_export`, `instructions_export`). Do not write them yourself with
   scripts, HTML templates, browsers or PDF tools. When the application cannot produce what was asked, say what is
   missing instead of building a substitute, unless the user asked for that.
8. **Lay out balloons and callouts deliberately.** Place balloons in clear space outside the part outlines, evenly
   spaced and aligned, with short leaders that do not cross and do not overlap parts, labels or page content. Never
   accept the automatic layout unchecked: export each page or step (`instructions_export` `png`/`svg`, or a sheet
   export), look at it, move each crowded balloon by dragging its label in the shown step's view
   (`brep.pointer.drag` from the label to clear space; `pmi_update_annotation` changes what it points at, not where
   it sits), and export again before delivering.

## Modelling order

1. **Plan the base profile** as the largest constant section of the part. Pick the plane that makes it a
   2D sketch: a part document starts with datum `D1` and its planes `D1:XY`, `D1:XZ`, `D1:YZ`; a face of the
   existing solid is also a sketch plane (`sketchPlane: "<face name>"`).
2. **Sketch** (`S`) with closed loops: segments name POINT IDS, and a loop is closed only when consecutive segments
   share point ids. An open chain is a path (a sweep spine), not a profile.
3. **Extrude** (`E`) or **Revolve** (`R`) the sketch to the base solid. Extrude goes along the sketch normal by
   `distance` and the opposite way by `distanceBack` (set it to 0 for a one-sided extrude). Revolve's `axis` is a
   sketch line (`<sketch id>:G<geometry id>`) or an edge that does not cross the profile.
4. **Secondary features**: bosses and cuts as further sketches extruded with `boolean: {operation: "UNION" |
   "SUBTRACT", targets: [<solid>]}`, ribs (`RIB`), holes (`H`).
5. **Patterns and mirrors** of the bosses, lugs and ribs (they copy solids; holes repeat through their sketch's
   points).
6. **Shell** (`O.S`), when the part is hollow, once the outside form is final and before any fillet on an opening's
   rim; the open faces are the `faces` you list.
7. **Fillets and chamfers last** (`F`, `CH`): every later feature renames and splits edges, and a blend is the
   feature most sensitive to what lies next to it.

Prefer a feature to a primitive plus a boolean: a sketch-and-extrude keeps its intent editable (one sketch, one
distance), where stacked primitives need several transforms to change one size. Use `B` booleans to combine
solids the features already made, or tool bodies that have no feature equivalent.

## Stable references

- Read names from the live model, never guess them: `after.report.featureOutputs[<feature id>]` names the solid a
  feature produced; `brep.scene.scene_entities({})` lists each solid's faces and edges by name.
- Names are rebuilt when an upstream feature changes. After editing an earlier feature, read
  `scene_entities` again and re-pick edges and faces before editing anything downstream; a Fillet or Chamfer whose
  names are all gone is refused, and a boolean whose targets are gone adds its body without the cut.
- Give features meaningful ids (`id: "Plate"`), so references and reports read as design intent.

## Checkpoints after every feature

- `r.after.report.featureErrors` is empty (a refused feature leaves the earlier geometry as it was).
- The solid count is what you intended (`scene_entities`), the solid is valid (`model_inspect`), and its volume and
  bounds move the way the feature should move them (`brep.scene.mass_properties({name})`).
- A changed expression reruns the history: check the same values again after `expressions_set`.
- `screenshot` when shape matters more than numbers.

## What the kernel refuses or finds fragile

A refusal leaves the earlier geometry unchanged and names itself in `featureErrors`. Quote it, change the one
thing it names, and rerun; repeated blind radius or distance changes are not a repair.

| Feature | The refusal says (start of the text) | What to do |
|---|---|---|
| Extrude, Revolve | `profile '…' not found (no sketch profile or resident face)` | The sketch has no CLOSED loop: consecutive segments must share point ids. |
| Extrude | `total extrusion length (distance + distanceBack) is zero` | Give a non-zero span. Remember `distanceBack` defaults to 10: set it (usually 0) on every extrude. |
| Sketch | `loop containment is inconsistent (crossing loops?)` | Loops may nest but never cross; split crossing shapes into separate sketches or features. |
| Sketch, Extrude | `sketch/plane on a non-planar face is not yet migrated` | Sketch on a planar face or a datum plane (`D1:XY`, `P` offset planes). |
| Revolve | `profile crosses the revolve axis` / `profile touches the axis` | Keep the profile on one side of the axis; an edge lying ON the axis is allowed. |
| Fillet | `… is too large for the faces it must lie on` | Smaller radius, or also select the neighbouring edges so the blend can run on. |
| Fillet | `the corner setbacks consume the whole of …` | The radius reaches the edge's length at its corners: reduce it. |
| Fillet | `no wall of radius … fits this edge … turns tighter than the ball` | The radius exceeds a neighbouring curved face's (or earlier round's) radius: stay below it. |
| Fillet | `… (fillet the concave edge before the rim)` | Split into two Fillet features: the concave edge first, the rim after. |
| Fillet, Chamfer | `none of the N selected reference(s) resolves on this model` | The edge names are stale or guessed: read `scene_entities` again and pick from it. |
| Chamfer | `chamfer requires the selected edges to belong to a single solid` | One Chamfer feature per solid. |
| Offset Shell | `could not close a watertight shell around the selected opening … Fix: run the offset shell EARLIER` | Move the shell before the curved cut or rim fillet that made the opening. |
| Offset Shell | `the solid is nowhere thicker than twice the shell distance` | The wall thickness is too large for the part: reduce `distance`. |
| Offset Shell | `opening face … is curved and not a ruled surface; an outward wall is built only on a planar or ruled opening` | Shell inward (positive distance) or open a planar face. |
| Boolean | `operation produced no boundary faces` | The tool only touches or grazes the target: make it overlap clearly or move it clear. |
| Boolean | `internal tangency … pinches to zero thickness` | A SUBTRACT tool is tangent inside the target: overlap it by a real amount or leave a real wall. |
| Hole | `no placement points (each hole needs a sketch center)` | `face` names a SKETCH whose points are the hole centres; give it points. |
| Hole | `countersink has no recess (countersinkDiameter must exceed diameter …)` | Make the countersink (or counterbore) diameter larger than the hole. |
| Loft | `sections must have the same curve count` | Give every section the same number of segments and the same loops. |
| Sweep | `… orientationMode: translate …` (closed or curved path) | Set `orientationMode: "pathAlign"`, or use Path Sweep (`SWP`). |
| Pattern | `Pattern union failed: …` | Use `booleanMode: "NONE"`, then union the result with a Boolean feature, or space the copies to overlap clearly. |

These succeed but do nothing, so check the checkpoints rather than the absence of an error:

- A fillet or chamfer of radius/distance 0 or less; an Offset Shell of distance 0; a Pattern with `count` 1.
- A Hole or any `boolean` whose `operation` is NONE or whose `targets` are empty or do not resolve: the body is
  added without the cut. Always give `boolean.targets` the solid named in `featureOutputs`.
- Pattern copies SOLIDS (`solids`), not features. To repeat holes, put every centre in one points-only sketch
  under one Hole feature; to repeat a boss, pattern its solid with `booleanMode: "UNION"` or union it after.
- Circular Pattern: in the default `countMode: "count and pitch"`, `totalAngleDeg` is the angle BETWEEN copies
  (default 360 puts every copy on the source); set it to the pitch or use `"count and span"`.
- Mirror keeps the source and adds a separate copy: union them with a Boolean feature when they are one part.
- A note `radius reduced to … locally near …` means the fillet did not keep the radius everywhere: check it.

Fragile, avoid when there is a choice: tiny overlaps (hundredths of a millimetre) between stacked bodies,
coincident or exactly tangent faces in a boolean, a fillet radius close to a neighbouring curvature radius, and a
shell thickness close to a face's radius of curvature. Prefer overlaps of a real fraction of the part's size.

## Worked example: a mounting plate

A 60 x 40 plate with true-arc corners, four counterbored M5 holes from one Hole feature, and its top edge rounded
last, every size an expression. It checks itself after each feature and returns what changed when the thickness
is edited. Run it in a new part document (`brep.document.doc_new({})` first if one is open).

```js
import { brep } from "@brep/cad";
export default async function main() {
  // Stop at the first refused feature, with the engine's own message.
  const ok = (r, what) => {
    const errors = r?.after?.report?.featureErrors ?? r?.report?.featureErrors ?? [];
    if (errors.length) throw new Error(`${what}: ${errors.join("; ")}`);
    return r;
  };
  // 1. Named sizes: the user edits these, not the features.
  const len = 60, wid = 40, cr = 6;
  ok(await brep.history.expressions_set({ script: `len = ${len};\nwid = ${wid};\nthk = 6;\nholeD = 5.5;\ncbD = 9.5;\ncbDepth = 3;` }), "expressions");
  // 2. Base sketch on the XY plane: four lines and four TRUE arcs (centre, start, end; counter-clockwise).
  const P = (id, x, y) => ({ id, x, y, fixed: true });
  const points = [
    P(1, cr, 0), P(2, len - cr, 0), P(3, len - cr, cr), P(4, len, cr),
    P(5, len, wid - cr), P(6, len - cr, wid - cr), P(7, len - cr, wid), P(8, cr, wid),
    P(9, cr, wid - cr), P(10, 0, wid - cr), P(11, 0, cr), P(12, cr, cr),
  ];
  const G = (id, type, pts) => ({ id, type, points: pts });
  const geometries = [
    G(20, "line", [1, 2]), G(21, "arc", [3, 2, 4]), G(22, "line", [4, 5]), G(23, "arc", [6, 5, 7]),
    G(24, "line", [7, 8]), G(25, "arc", [9, 8, 10]), G(26, "line", [10, 11]), G(27, "arc", [12, 11, 1]),
  ];
  ok(await brep.features.feature_add({ type: "S", id: "Base", params: { sketchPlane: "D1:XY" },
    persistent_data: { sketch: { points, geometries, constraints: [] } } }), "Base sketch");
  // 3. Extrude one way only: distanceBack defaults to 10, so set it.
  const plate = ok(await brep.features.feature_add({ type: "E", id: "Plate",
    params: { profile: "Base", distance: "thk", distanceBack: 0 } }), "Plate extrude");
  const solid = plate.after.report.featureOutputs.Plate[0];
  // 4. Hole centres: a points-only sketch on the top face (coordinates from the face centre).
  const top = "Plate:Base:PROFILE_END";
  const sx = len / 2 - 8, sy = wid / 2 - 8;
  ok(await brep.features.feature_add({ type: "S", id: "HolePts", params: { sketchPlane: top },
    persistent_data: { sketch: { points: [P(1, -sx, -sy), P(2, sx, -sy), P(3, sx, sy), P(4, -sx, sy)], geometries: [], constraints: [] } } }), "hole sketch");
  // 5. ONE Hole feature drills every sketch point.
  ok(await brep.features.feature_add({ type: "H", id: "Mount", params: { face: "HolePts", holeType: "COUNTERBORE",
    diameter: "holeD", throughAll: true, counterboreDiameter: "cbD", counterboreDepth: "cbDepth",
    boolean: { operation: "SUBTRACT", targets: [solid] } } }), "Mount holes");
  const drilled = await brep.scene.mass_properties({ name: solid });
  // 6. Fillet last, on edges read from the live model: the top face against the side walls.
  const scene = await brep.scene.scene_entities({});
  const body = scene.solids.find((s) => s.name === solid);
  const names = body.edges.map((e) => (typeof e === "string" ? e : e.name));
  const topEdges = names.filter((n) => n.includes("_END[") && n.split("|").every((f) => f.startsWith("Plate:Base:")));
  ok(await brep.features.feature_add({ type: "F", id: "TopRound", params: { edges: topEdges, radius: 1 } }), "TopRound fillet");
  const rounded = await brep.scene.mass_properties({ name: solid });
  // 7. Checkpoint, then prove the design is editable: one expression, whole history reruns.
  const inspect = await brep.scene.model_inspect({});
  if (inspect.solidCount !== 1 || !inspect.valid) throw new Error(`expected one valid solid: ${JSON.stringify(inspect.solids)}`);
  ok(await brep.history.expressions_set({ script: `len = ${len};\nwid = ${wid};\nthk = 8;\nholeD = 5.5;\ncbD = 9.5;\ncbDepth = 3;` }), "thicker plate");
  const thicker = await brep.scene.mass_properties({ name: solid });
  return { solid, topEdges: topEdges.length, drilled: drilled.volume, rounded: rounded.volume, thicker: thicker.volume, bounds: inspect.boundsMm };
}
```

Expected: one valid solid, eight top edges rounded, volume about 13 079 mm3 drilled, a little less once rounded,
and more after `thk = 8`. For an assembly, save this part (`document_save`), open an assembly document, insert it
with `component_insert`, and mate it to its neighbours with `assembly_add_constraint`.
