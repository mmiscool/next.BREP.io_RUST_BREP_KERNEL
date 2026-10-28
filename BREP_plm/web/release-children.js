// Release, or check in, an assembly together with the parts it uses, in one
// prompt that defaults to proceed (owner rule: prompts, not refusals).
//
// Only the existing routes, deepest first: the revision's BOM names the
// children; each child's part says its lifecycle and lock; its review rule and
// the active release workflows say whether it may simply be released. A child
// that needs approval is SUBMITTED for review, never force-released; one that
// needs its release workflow, waits on a review, or is checked out by someone
// else is named and left alone. `run` stops at the first failure and reports
// exactly what was done and what was not, for a Retry of the rest: it never
// claims everything was done.
(() => {
  const enc = encodeURIComponent;
  const revisionPath = (part, revision) => `/api/parts/${enc(part)}/revisions/${enc(revision)}`;

  function workflowRequired(workflows, partType) {
    return (workflows || []).some((d) => d.active && d.require_for_release
      && (!d.part_types || !d.part_types.length || d.part_types.includes(partType)));
  }

  // The children of `part`/`revision` to act on, deepest first, each with the
  // steps it needs. `mode` is 'release' or 'checkin'.
  async function plan(api, part, revision, mode) {
    const bom = await api('GET', `${revisionPath(part, revision)}/bom?levels=0&flat=false`);
    // Keyed by part AND revision: revision ids are per part (every part's first is `A`).
    const deepest = new Map();
    for (const line of bom.lines || []) {
      if (!line.part_id || !line.revision_id) continue;
      const key = `${line.part_id}/${line.revision_id}`;
      const seen = deepest.get(key);
      if (!seen || line.level > seen.level) deepest.set(key, line);
    }
    const workflows = mode === 'release' ? await api('GET', '/api/workflows') : [];
    const parts = new Map();
    const items = [];
    for (const line of [...deepest.values()].sort((a, b) => b.level - a.level)) {
      if (!parts.has(line.part_id)) parts.set(line.part_id, await api('GET', `/api/parts/${enc(line.part_id)}`));
      const detail = parts.get(line.part_id);
      // The per-viewer views (`locked_by_me`, the lifecycle as a word), not the stored records.
      const view = (detail.revision_views || []).find((r) => r.id === line.revision_id) || {};
      const item = {
        part: line.part_id, revision: line.revision_id, level: line.level,
        number: line.number || detail.number || line.part_id,
        label: line.revision_label || view.label || '',
        lifecycle: view.lifecycle || line.state || '',
        lockedByMe: Boolean(view.locked_by_me), lockedBy: view.locked_by || '',
        steps: [], route: '', note: '',
      };
      if (mode === 'checkin') {
        if (item.lockedByMe) { item.steps = ['checkin']; item.route = 'checkin'; items.push(item); }
        continue;
      }
      if (item.lifecycle === 'released' || item.lifecycle === 'obsolete' || item.lifecycle === 'superseded') continue;
      if (item.lockedBy && !item.lockedByMe) {
        item.route = 'blocked'; item.note = `checked out by ${item.lockedBy}`;
        items.push(item); continue;
      }
      if (item.lockedByMe) item.steps.push('checkin');
      if (workflowRequired(workflows, detail.part_type)) {
        item.route = 'workflow'; item.note = 'its part type is released through a workflow';
      } else {
        const review = await api('GET', `${revisionPath(item.part, item.revision)}/review`);
        const needed = review.rule?.required_approvals || 0;
        const round = review.current;
        const approved = round && round.required_approvals > 0 && round.approvals >= round.required_approvals;
        if (!needed || approved) item.route = 'release';
        else if (item.lifecycle === 'inreview') { item.route = 'waiting'; item.note = 'waiting for its review'; }
        else { item.route = 'review'; item.note = `needs ${needed} approval${needed === 1 ? '' : 's'}`; }
      }
      if (item.route === 'release') item.steps.push('release');
      if (item.route === 'review') item.steps.push('submit');
      items.push(item);
    }
    return { items };
  }

  const verbs = { checkin: 'check in', release: 'release', submit: 'submit for review' };

  // One line for the prompt: what the child is and what will be done to it.
  function describe(item) {
    const name = `${item.number}${item.label ? ` rev ${item.label}` : ''}`;
    const state = [item.lifecycle, item.lockedByMe ? 'checked out by you' : ''].filter(Boolean).join(', ');
    const what = item.steps.length ? item.steps.map((s) => verbs[s]).join(', then ') : 'left as it is';
    return `${name}${state ? ` (${state})` : ''}: ${what}${item.note ? ` — ${item.note}` : ''}`;
  }

  // Run the steps in order; stop at the first failure. `done` lists the
  // finished steps, `failed` the step that failed and why, `remaining` the
  // items still to do (the failed one first, with its unfinished steps), for a
  // Retry. Items with no steps are `left` with their reason.
  async function run(api, items, onStep = () => {}) {
    const done = [];
    const left = [];
    for (let i = 0; i < items.length; i++) {
      const item = items[i];
      if (!item.steps.length) { left.push(item); continue; }
      for (let s = 0; s < item.steps.length; s++) {
        const step = item.steps[s];
        const base = revisionPath(item.part, item.revision);
        try {
          if (step === 'checkin') await api('POST', `${base}/checkin`, { force: false });
          else if (step === 'release') await api('POST', `${base}/state`, { to: 'released' });
          else if (step === 'submit') await api('POST', `${base}/submit`, {});
          done.push({ item, step });
          onStep({ item, step });
        } catch (error) {
          const rest = [{ ...item, steps: item.steps.slice(s) }, ...items.slice(i + 1)];
          return { done, left, failed: { item, step, error: String(error?.message || error) }, remaining: rest };
        }
      }
    }
    return { done, left, failed: null, remaining: [] };
  }

  globalThis.ReleaseChildren = { plan, run, describe, workflowRequired };
})();
