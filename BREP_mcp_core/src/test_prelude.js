// test-mcp's script-mode PRELUDE: what `brep-mcp test` puts in front of a
// `BREP_mcp/tests/scripts/*.js` program before it runs it through code mode.
//
// It makes every `await brep.<group>.<tool>(args)` call ONE STEP of the
// script, numbered in call order, exactly as one entry of the old JSON
// `steps` list was: the call's result (or error), the expectations made on
// it, its image, its elapsed time — reported to the runner by
// `brep.runner.step_record` when the next step starts, so the per-step
// artefacts (`NNNN-<tool>.json`, the images, `result.json`) are written as
// the script goes and survive a killed worker. A step that fails stops the
// script, as the JSON runner's stop-on-fail did.
//
// `test` is the assertion library, with the JSON format's semantics
// (BREP_mcp_core/src/script.rs `Expect::check`): JSON-pointer paths, `near`
// relative to max(|b|, 1), `exists` true for a null, `len` counting
// characters, the `*`/`?` glob of `matches` and `test.fails`.
//
// `__TEST_OUT__` is replaced by the runner with the run's output directory.
import { brep as __brep } from "@brep/cad";

const test = (() => {
  // The SDK's own functions, before they are wrapped: the runner's
  // bookkeeping calls must not count as steps.
  const raw = {};
  for (const group of Object.keys(__brep)) {
    raw[group] = Object.assign({}, __brep[group]);
  }
  const runner = raw.runner || {};

  class Stop extends Error {}

  let cur = null;
  let next = 0;
  let stopped = false;

  // ---- JSON values, as serde_json compares and prints them ----------------
  function pointer(value, path) {
    if (path === "") return { found: true, value };
    if (path[0] !== "/") return { found: false };
    let at = value;
    for (const token of path.slice(1).split("/")) {
      const key = token.replace(/~1/g, "/").replace(/~0/g, "~");
      if (Array.isArray(at)) {
        if (key.startsWith("+") || (key.startsWith("0") && key.length !== 1) || !/^[0-9]+$/.test(key)) return { found: false };
        const i = Number(key);
        if (i >= at.length) return { found: false };
        at = at[i];
      } else if (at !== null && typeof at === "object") {
        if (!Object.prototype.hasOwnProperty.call(at, key)) return { found: false };
        at = at[key];
      } else {
        return { found: false };
      }
    }
    return { found: true, value: at };
  }

  function same(a, b) {
    if (a === b) return true;
    if (typeof a !== typeof b || a === null || b === null || typeof a !== "object") return false;
    if (Array.isArray(a) !== Array.isArray(b)) return false;
    if (Array.isArray(a)) return a.length === b.length && a.every((x, i) => same(x, b[i]));
    const ka = Object.keys(a);
    if (ka.length !== Object.keys(b).length) return false;
    return ka.every((k) => Object.prototype.hasOwnProperty.call(b, k) && same(a[k], b[k]));
  }

  const show = (v) => (v === undefined ? "null" : JSON.stringify(v));

  function glob(pattern, text) {
    const p = [...pattern];
    const t = [...String(text)];
    // dp[j]: p[..i] matches t[..j]
    let dp = new Array(t.length + 1).fill(false);
    dp[0] = true;
    for (let i = 0; i < p.length; i++) {
      const nx = new Array(t.length + 1).fill(false);
      if (p[i] === "*") {
        let any = false;
        for (let j = 0; j <= t.length; j++) {
          any = any || dp[j];
          nx[j] = any;
        }
      } else {
        for (let j = 1; j <= t.length; j++) nx[j] = dp[j - 1] && (p[i] === "?" || p[i] === t[j - 1]);
      }
      dp = nx;
    }
    return dp[t.length];
  }

  // ---- the step lifecycle ---------------------------------------------------
  function current() {
    if (!cur) throw new Error("an expectation needs a step: call a brep.<group>.<tool> function first");
    return cur;
  }

  async function record(step) {
    let output = step.output;
    if (step.error !== undefined) output = { error: step.error };
    // Send large replies in bounded calls; the host assembles the exact JSON.
    const serialized = JSON.stringify(output === undefined ? null : output);
    const chunked = serialized.length > 128 * 1024;
    if (chunked) {
      for (let i = 0; i < serialized.length;) {
        let end = Math.min(i + 32 * 1024, serialized.length);
        // Never split a UTF-16 surrogate pair across JSON bridge calls.
        const last = serialized.charCodeAt(end - 1);
        if (last >= 0xd800 && last <= 0xdbff) end--;
        await runner.step_output_chunk({ chunk: serialized.slice(i, end) });
        i = end;
      }
    }
    const report = await runner.step_record({
      index: step.index,
      tool: step.tool,
      group: step.group,
      expected_error: Boolean(step.expectedError),
      chunked,
      outcome: step.outcome,
      failures: step.failures,
      elapsed_ms: step.elapsed_ms,
      output: chunked || output === undefined ? null : output,
      image: step.image || null,
      skip_image: Boolean(step.skipImage) || step.error !== undefined,
      save: step.save || null,
      compare: step.compare || null,
    });
    return report.failures || [];
  }

  // Close the open step: report it, and stop the script if it failed.
  async function close() {
    if (!cur) return;
    const step = cur;
    cur = null;
    if (step.outcome === undefined) step.outcome = step.failures.length ? "expectation_failed" : "pass";
    const extra = await record(step);
    if (extra.length && step.outcome === "pass") step.outcome = "expectation_failed";
    if (step.outcome !== "pass") {
      stopped = true;
      throw new Stop(`step ${step.index} (${step.tool}) failed`);
    }
  }

  function open(tool) {
    cur = { index: next++, tool, failures: [], started: Date.now() };
    return cur;
  }

  function needsSubstitution(args) {
    const text = JSON.stringify(args);
    return text !== undefined && (text.includes("${plm:") || text.includes("${out}"));
  }

  // Every SDK tool function becomes a step.
  for (const group of Object.keys(raw)) {
    for (const name of Object.keys(raw[group])) {
      const call = raw[group][name];
      if (typeof call !== "function") continue;
      __brep[group][name] = async (args) => {
        if (stopped) throw new Stop("the script already stopped");
        await close();
        const step = open(name);
        step.group = group;
        let a = args === undefined || args === null ? {} : args;
        try {
          // `${plm:NAME}` and `${out}` in the arguments, as the JSON runner
          // substituted every step's (inside the step: a name that is not
          // known fails THIS step).
          if (group !== "runner" && needsSubstitution(a)) {
            a = (await runner.substitute({ value: a })).value;
          }
          let out = await call(a);
          // Code mode hands a tool's pictures back as artifacts under
          // `images`; the step keeps the first for `save`/`compare`, and the
          // result the script reads is the tool's own JSON.
          if (out && Array.isArray(out.images) && out.images.length && out.images.every((i) => i && i.artifactId)) {
            step.image = out.images[0].artifactId;
            if (Object.keys(out).length === 2 && "value" in out) {
              out = out.value;
            } else {
              out = Object.assign({}, out);
              delete out.images;
            }
          }
          step.output = out;
          step.elapsed_ms = Date.now() - step.started;
          return out;
        } catch (e) {
          if (e instanceof Stop) throw e;
          step.error = e && e.message !== undefined ? e.message : String(e);
          step.elapsed_ms = Date.now() - step.started;
          throw e;
        }
      };
    }
  }

  class Expectation {
    constructor(value, path) {
      this.step = current();
      this.path = path;
      this.at = pointer(value, path);
    }
    fail(what) {
      let shown = show(this.at.value);
      if ([...shown].length > 160) shown = [...shown].slice(0, 160).join("") + "…";
      this.step.failures.push(`${this.path}: ${what}; actual ${shown}`);
    }
    present() {
      if (!this.at.found) {
        this.step.failures.push(`${this.path}: no value at that path`);
        return false;
      }
      return true;
    }
    num() {
      return typeof this.at.value === "number" ? this.at.value : null;
    }
    exists(expected = true) {
      if (this.at.found !== expected) this.step.failures.push(`${this.path}: exists == ${this.at.found} expected ${expected}`);
    }
    eq(e) {
      if (this.present() && !same(this.at.value, e)) this.fail(`expected ${show(e)}`);
    }
    ne(e) {
      if (this.present() && same(this.at.value, e)) this.fail(`expected not ${show(e)}`);
    }
    gt(b) {
      if (this.present() && !(this.num() !== null && this.num() > b)) this.fail(`expected > ${b}`);
    }
    gte(b) {
      if (this.present() && !(this.num() !== null && this.num() >= b)) this.fail(`expected >= ${b}`);
    }
    lt(b) {
      if (this.present() && !(this.num() !== null && this.num() < b)) this.fail(`expected < ${b}`);
    }
    lte(b) {
      if (this.present() && !(this.num() !== null && this.num() <= b)) this.fail(`expected <= ${b}`);
    }
    near(b, tol = 1e-6) {
      if (!this.present()) return;
      const a = this.num();
      if (!(a !== null && Math.abs(a - b) <= tol * Math.max(Math.abs(b), 1))) this.fail(`expected within ${tol} (relative) of ${b}`);
    }
    contains(needle) {
      if (!this.present()) return;
      const v = this.at.value;
      let ok = false;
      if (typeof v === "string" && typeof needle === "string") ok = v.includes(needle);
      else if (Array.isArray(v)) ok = v.some((x) => same(x, needle));
      else if (v !== null && typeof v === "object" && typeof needle === "string") ok = Object.prototype.hasOwnProperty.call(v, needle);
      if (!ok) this.fail(`expected to contain ${show(needle)}`);
    }
    matches(pattern) {
      if (!this.present()) return;
      if (typeof this.at.value !== "string") return this.fail("expected a string to match");
      if (!glob(pattern, this.at.value)) this.fail(`expected to match \`${pattern}\``);
    }
    len(n) {
      if (!this.present()) return;
      const v = this.at.value;
      let l = null;
      if (Array.isArray(v)) l = v.length;
      else if (typeof v === "string") l = [...v].length;
      else if (v !== null && typeof v === "object") l = Object.keys(v).length;
      if (l !== n) this.fail(`expected length ${n}`);
    }
  }

  return {
    /** The run's output directory (what `${out}` stands for). */
    out: "__TEST_OUT__",
    glob,
    pointer: (value, path) => pointer(value, path).value,
    /** An expectation on the current step's result at a JSON pointer. */
    expect: (value, path) => new Expectation(value, path),
    /** The current step must FAIL, with an error matching the glob. */
    async fails(pattern, promise) {
      try {
        await promise;
      } catch (e) {
        if (e instanceof Stop) throw e;
        const step = current();
        const message = step.error !== undefined ? step.error : e && e.message !== undefined ? e.message : String(e);
        step.skipImage = true;
        step.expectedError = true;
        if (!glob(pattern, message)) {
          step.failures.push(`tool failed as expected but with \`${message}\`, which does not match \`${pattern}\``);
        }
        return message;
      }
      const step = current();
      step.skipImage = true;
      step.failures.push(`expected the tool to fail with \`${pattern}\` but it succeeded`);
      return null;
    },
    /** Save the current step's image under this name in the run's directory. */
    save(name) {
      current().save = name;
    },
    /** Compare the current step's image with a baseline under BREP_mcp/tests/baselines/. */
    compare(baseline, maxDiffRatio = 0.002) {
      current().compare = { baseline, max_diff_ratio: maxDiffRatio };
    },
    /** Bind values of a result for `${plm:NAME}` in later steps ({NAME: "/pointer"}). */
    async bind(value, names) {
      const step = current();
      const r = await runner.bind({ value, bind: names });
      for (const f of r.failures || []) step.failures.push(f);
    },
    /** `${plm:NAME}` / `${out}` in a value, resolved now (for an expected value). */
    async sub(value) {
      return (await runner.substitute({ value })).value;
    },
    /** Run the script's body: every step reported, the first failure stops it. */
    async run(body) {
      try {
        await body();
        await close();
      } catch (e) {
        if (!(e instanceof Stop)) {
          const message = e && e.message !== undefined ? e.message : String(e);
          const step = cur || open("script");
          cur = null;
          if (step.error === undefined) step.error = message;
          if (step.elapsed_ms === undefined) step.elapsed_ms = Date.now() - step.started;
          step.outcome = "tool_error";
          step.failures.push(message);
          await record(step);
        }
      }
      return { steps: next, stopped };
    },
  };
})();
