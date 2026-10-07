/* Native-file adoption: pure planning plus a resumable API runner. Kept free of
   DOM dependencies so the same assembly cases can be exercised in Node. */
'use strict';
globalThis.NativeImport = (() => {
  const clone = value => JSON.parse(JSON.stringify(value));
  const isComponent = feature => ['ACOMP', 'ASSEMBLY COMPONENT'].includes(feature.type);
  const canonical = value => JSON.stringify(value, function (key, item) {
    if (key === 'sourceSignature' || key === 'snapshot') return undefined;
    return item && typeof item === 'object' && !Array.isArray(item)
      ? Object.fromEntries(Object.keys(item).sort().map(k => [k, item[k]])) : item;
  });
  function path(name) {
    const out = [];
    for (const part of name.replaceAll('\\', '/').replace(/\.nbrep$/i, '').split('/')) {
      if (part === '..') out.pop();
      else if (part && part !== '.') out.push(part);
    }
    return out.join('/');
  }
  const dirname = name => name.includes('/') ? name.slice(0, name.lastIndexOf('/') + 1) : '';
  const revisionKey = key => /^part\/[^/]+\/rev\/[^/]+$/.test(key);
  function plan(files) {
    const rows = new Map(), uploaded = new Map(), errors = [];
    for (const file of files) {
      const id = path(file.name);
      if (!/\.nbrep$/i.test(file.name)) { errors.push(`${file.name}: choose a .nbrep file`); continue; }
      if (uploaded.has(id)) { errors.push(`Duplicate file path: ${file.name}`); continue; }
      uploaded.set(id, file.document);
    }
    function resolve(key, owner) {
      const normalized = path(key);
      const relative = path(dirname(owner) + key);
      if (uploaded.has(relative)) return relative;
      if (uploaded.has(normalized)) return normalized;
      const matches = [...uploaded.keys()].filter(id => normalized.endsWith('/' + id) || id.endsWith('/' + normalized));
      if (matches.length === 1) return matches[0];
      if (matches.length > 1) throw new Error(`Ambiguous source ${key}; upload files with their folder paths`);
      return key.startsWith('/') || /^[A-Za-z]:/.test(key) ? normalized : relative;
    }
    function visit(id, document, label, ancestors = []) {
      if (ancestors.length > 64) throw new Error('Assembly nesting exceeds 64 levels');
      if (ancestors.includes(id)) throw new Error(`Cyclic assembly: ${[...ancestors, id].join(' → ')}`);
      if (!document || !Array.isArray(document.features)) throw new Error(`${label}: missing native features array`);
      if (rows.has(id)) {
        if (canonical(rows.get(id).document) !== canonical(document)) throw new Error(`${label}: conflicting copies of ${id}; upload a consistent assembly`);
        return id;
      }
      const row = { id, name: label, document: clone(document), assembly: document.features.some(isComponent), dependencies: Object.create(null), root: uploaded.has(id) };
      rows.set(id, row);
      const library = document.partsLibrary || {};
      if (typeof library !== 'object' || Array.isArray(library)) throw new Error(`${label}: invalid partsLibrary`);
      for (const feature of document.features.filter(isComponent)) {
        if (!Object.hasOwn(library, feature.inputParams?.partName)) throw new Error(`${label}: missing component ${feature.inputParams?.partName}`);
      }
      for (const [name, entry] of Object.entries(library)) {
        if (!entry || typeof entry !== 'object') throw new Error(`${label}: invalid component ${name}`);
        let key = String(entry.sourceKey || '').replace(/^\/models\//, '').replace(/\.nbrep$/i, '');
        if (revisionKey(key)) { row.dependencies[name] = { key }; continue; }
        const childId = key ? resolve(key, id) : `${id}/@embedded/${name}`;
        const supplied = uploaded.get(childId);
        if (supplied && entry.document && canonical(supplied) !== canonical(entry.document)) throw new Error(`${name}: the uploaded file and embedded copy differ; update the assembly before importing`);
        const child = supplied || entry.document;
        if (!child) throw new Error(`${label}: upload the missing file ${key || name}`);
        row.dependencies[name] = { id: visit(childId, child, name, [...ancestors, id]) };
      }
      return id;
    }
    for (const [id, doc] of uploaded) {
      try { visit(id, doc, id.split('/').pop()); } catch (error) { errors.push(error.message); }
    }
    return { rows: [...rows.values()], errors: [...new Set(errors)] };
  }
  const externalRef = (namespace, id) => `native:${namespace.trim()}:${id}`;
  const revPath = target => `/api/parts/${encodeURIComponent(target.part)}/revisions/${encodeURIComponent(target.revision)}`;
  const idSegment = id => encodeURIComponent(id).replace(/[.!'()*~]/g, c => `%${c.charCodeAt(0).toString(16).toUpperCase()}`);
  const targetKey = target => `part/${idSegment(target.part)}/rev/${idSegment(target.revision)}`;
  function rewritten(row, rows, targets, stack = []) {
    if (stack.includes(row.id)) throw new Error('Cyclic import mapping');
    const doc = clone(row.document);
    for (const [name, dependency] of Object.entries(row.dependencies)) {
      const entry = doc.partsLibrary[name];
      if (dependency.key) { entry.sourceKey = dependency.key; continue; }
      const target = targets[dependency.id];
      if (!target) throw new Error(`No target for ${dependency.id}`);
      entry.sourceKey = targetKey(target);
      // A reuse choice must embed the server's document, not the uploaded copy.
      // Rewriting a descendant can change this dependency's geometry too.
      // Invalidate every rewritten imported dependency, including ancestors of
      // reused documents: a valid same-producer snapshot otherwise wins over
      // the new embedded document in the kernel's FAST path.
      entry.snapshot = '';
      entry.document = target.reusedDocument || rewritten(rows.find(r => r.id === dependency.id), rows, targets, [...stack, row.id]);
      // The server refreshes sourceSignature using the engine's sorted-key hash.
    }
    return doc;
  }
  async function inspect(rows, api) {
    const targets = new Map();
    for (const row of rows) {
      delete row.reviewed;
      if (row.action === 'create') continue;
      if (!row.part.trim()) throw new Error(`${row.name}: choose an existing part number`);
      const detail = await api('GET', `/api/parts/${encodeURIComponent(row.part.trim())}`);
      if (detail.document_class !== 'normal') throw new Error(`${row.name}: the target must be a normal part`);
      if (targets.has(detail.id) && (targets.get(detail.id) !== 'reuse' || row.action !== 'reuse')) throw new Error(`${detail.number}: two imported items cannot write the same part`);
      targets.set(detail.id, row.action);
      const revision = row.revision
        ? detail.revision_views.find(r => r.id === row.revision)
        : detail.revision_views.at(-1);
      if (row.revision && !revision) throw new Error(`${detail.number}: selected revision ${row.revision} no longer exists; look up the part and select a revision again`);
      if (!revision) throw new Error(`${detail.number}: no revision selected`);
      if (row.action === 'replace' && (!revision.editable || revision.locked_by)) throw new Error(`${detail.number} ${revision.label}: replacement requires an editable revision with no checkout`);
      if (row.action === 'revise' && !detail.multiple_open_drafts && detail.revision_views.some(r => r.editable)) throw new Error(`${detail.number}: an open draft already exists; select Replace editable draft or Reuse revision`);
      row.reviewed = { part: detail.id, number: detail.number, revision: revision.id, label: revision.label, hash: revision.content_hash, revisions: detail.revision_views.map(r => r.id) };
    }
    // Existing PLM references are retained and must actually exist on this server.
    const checked = new Set();
    for (const row of rows) for (const dependency of Object.values(row.dependencies)) {
      if (!dependency.key || checked.has(dependency.key)) continue;
      checked.add(dependency.key);
      const [,part,,rev] = dependency.key.split('/');
      const detail = await api('GET', `/api/parts/${encodeURIComponent(part)}`);
      if (!detail.revision_views.some(r => r.id === rev)) throw new Error(`Missing PLM reference ${dependency.key}`);
    }
  }
  // A receipt is persisted before/after each mutation. Ambiguous allocation
  // failures stop; the UI retains the receipt for reconciliation, never blindly
  // creates a second revision on retry.
  async function run(job, api, persist, progress, stopped = () => false) {
    Object.setPrototypeOf(job.targets, null);
    const checkpoint = async () => { await persist(job); progress(job); };
    const step = async (row, name, fn) => {
      if (stopped()) throw new Error('Import paused. Completed items are retained; resume to continue.');
      job.current = `${row.name}: ${name}`; await checkpoint();
      return fn();
    };
    for (const row of job.rows) {
      if (job.targets[row.id]) continue;
      if (row.action === 'create') {
        const existing = (await api('GET', `/api/parts?external_ref=${encodeURIComponent(row.external_ref)}&part_type=${encodeURIComponent(row.part_type)}&limit=2`)).parts;
        if (existing.length) {
          if (!row.allocating) throw new Error(`${row.name}: a part with this import identity already exists; review it as an existing part`);
          if (existing.length !== 1) throw new Error(`${row.name}: ambiguous import identity`);
          const p = await api('GET', `/api/parts/${existing[0].id}`);
          if (p.revisions.length !== 1 || p.revision_views[0].content_hash) throw new Error(`${row.name}: the recovered allocation has changed; inspect ${p.number}`);
          const rev = p.revision_views[0];
          job.targets[row.id] = { part: p.id, number: p.number, revision: rev.id, label: rev.label, hash: rev.content_hash };
        } else {
          row.allocating = true; await checkpoint();
          const p = await step(row, 'create part', () => api('POST', '/api/parts', { part_type: row.part_type, name: row.name, number: row.number, origin: 'imported', external_ref: row.external_ref, bulk: false }));
          job.targets[row.id] = { part: p.id, number: p.number, revision: p.revisions[0].id, label: p.revisions[0].label, hash: '' };
        }
      } else {
        const expected = row.reviewed;
        const p = await api('GET', `/api/parts/${expected.part}`);
        if (row.action === 'revise') {
          if (row.allocating) throw new Error(`${row.name}: a revision allocation may have completed; inspect ${p.number} before starting another import`);
          if (JSON.stringify(p.revision_views.map(r => r.id)) !== JSON.stringify(expected.revisions)) throw new Error(`${p.number}: revisions changed; review again`);
          const rev = await step(row, 'create revision', async () => {
            row.allocating = true; await checkpoint();
            return api('POST', `/api/parts/${p.id}/revisions`, { origin: 'imported' });
          });
          // Allocation carries the previous model after constructing its response;
          // read the resulting hash instead of assuming the returned draft is empty.
          const fresh = await api('GET', `/api/parts/${p.id}`);
          const created = fresh.revision_views.find(r => r.id === rev.id);
          if (!created) throw new Error(`${p.number}: the new revision disappeared`);
          job.targets[row.id] = { part: p.id, number: p.number, revision: rev.id, label: rev.label, hash: created.content_hash };
        } else {
          const rev = p.revision_views.find(r => r.id === expected.revision);
          if (!rev || rev.content_hash !== expected.hash || (row.action === 'replace' && (!rev.editable || rev.locked_by))) throw new Error(`${p.number}: target changed since review`);
          job.targets[row.id] = { ...expected };
          if (row.action === 'reuse') {
            const doc = await api('GET', `/api/store/doc/${targetKey(expected).replaceAll('%', '%25')}`);
            if (!doc) throw new Error(`${p.number}: the selected revision has no document`);
            job.targets[row.id].reusedDocument = doc;
            job.targets[row.id].done = true;
          }
        }
      }
      await checkpoint();
    }
    // Children first keeps progress useful if a large assembly is interrupted.
    const ordered = [], seen = new Set();
    function order(row) {
      if (seen.has(row.id)) return;
      seen.add(row.id);
      for (const dependency of Object.values(row.dependencies)) if (dependency.id) order(job.rows.find(r => r.id === dependency.id));
      ordered.push(row);
    }
    job.rows.forEach(order);
    for (const row of ordered) {
      const target = job.targets[row.id];
      if (target.done) continue;
      const route = revPath(target);
      if (!target.written) {
        const p = await api('GET', `/api/parts/${target.part}`);
        const rev = p.revision_views.find(r => r.id === target.revision);
        if (!rev || rev.content_hash !== target.hash || (rev.locked_by && !target.checkedOut)) throw new Error(`${target.number}: target changed or was checked out; inspect it before resuming`);
        await step(row, 'check out', () => api('POST', `${route}/checkout`, { client_id: 'native-import' }));
        target.checkedOut = true; await checkpoint();
        const result = await step(row, 'write model and BOM', () => api('PUT', `${route}/import`, { expected_hash: target.hash, document: rewritten(row, job.rows, job.targets) }));
        target.hash = result.content_hash;
        target.written = true; await checkpoint();
      }
      await step(row, 'check in', () => api('POST', `${route}/checkin`, {}));
      target.checkedOut = false; target.done = true; await checkpoint();
    }
    job.complete = true; job.current = 'Import complete'; await checkpoint();
    return job;
  }
  return { plan, canonical, path, externalRef, rewritten, inspect, run };
})();
