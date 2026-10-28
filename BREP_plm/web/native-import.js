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
  // A document's own content: every parts-library entry reduced to the item it
  // places, so a stale embedded copy of a child never makes its parent differ.
  function own(document, place) {
    const doc = clone(document);
    for (const [name, entry] of Object.entries(doc.partsLibrary || {})) doc.partsLibrary[name] = place(name, entry);
    return canonical(doc);
  }
  // `choices` answers the conflict prompts: { [id]: 'file' | 'embedded' | 'copy:<n>' }.
  function plan(files, choices = {}) {
    const uploaded = new Map(), errors = [], named = [];
    for (const file of files) {
      if (!/\.nbrep$/i.test(file.name)) { errors.push(`${file.name}: choose a .nbrep file`); continue; }
      named.push([path(file.name), file]);
    }
    // Identity is relative to the folder the files share, so the same tree
    // chosen as loose files or as a folder maps to the same parts.
    let common = null;
    for (const [id] of named) {
      const dirs = id.split('/').slice(0, -1);
      if (!common) { common = dirs; continue; }
      let i = 0; while (i < common.length && common[i] === dirs[i]) i++;
      common = common.slice(0, i);
    }
    for (const [full, file] of named) {
      const id = full.split('/').slice(common.length).join('/');
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
    // Every copy of every item, as found: the uploaded file and each embedded
    // copy. Copies that agree on their own content are one copy used more often.
    const copies = new Map(), variants = new Map();
    function collect(id, document, label, from, ancestors) {
      if (ancestors.length > 64) throw new Error('Assembly nesting exceeds 64 levels');
      if (id && ancestors.includes(id)) throw new Error(`Cyclic assembly: ${[...ancestors, id].join(' → ')}`);
      if (!document || !Array.isArray(document.features)) throw new Error(`${label}: missing native features array`);
      const library = document.partsLibrary || {};
      if (typeof library !== 'object' || Array.isArray(library)) throw new Error(`${label}: invalid partsLibrary`);
      for (const feature of document.features.filter(isComponent)) {
        if (!Object.hasOwn(library, feature.inputParams?.partName)) throw new Error(`${label}: missing component ${feature.inputParams?.partName}`);
      }
      const chain = id ? [...ancestors, id] : ancestors, owner = id || ancestors.at(-1) || '';
      const dependencies = Object.create(null);
      for (const [name, entry] of Object.entries(library)) {
        if (!entry || typeof entry !== 'object') throw new Error(`${label}: invalid component ${name}`);
        const key = String(entry.sourceKey || '').replace(/^\/models\//, '').replace(/\.nbrep$/i, '');
        if (revisionKey(key)) { dependencies[name] = { key }; continue; }
        if (!key) {
          // Embedded-only: promoted to its own item. The same name and content
          // anywhere in this import is one item; other content is name~2, ….
          if (!entry.document) throw new Error(`${label}: component ${name} has no document`);
          dependencies[name] = { id: collect(null, entry.document, name, label, chain) };
          continue;
        }
        const childId = resolve(key, owner);
        if (!entry.document && !uploaded.has(childId)) throw new Error(`${label}: upload the missing file ${key}`);
        dependencies[name] = { id: entry.document ? collect(childId, entry.document, name, label, chain) : childId };
      }
      const signature = own(document, name => ({ item: dependencies[name].key || dependencies[name].id }));
      if (!id) {
        const list = variants.get(label) || [];
        if (!list.includes(signature)) list.push(signature);
        variants.set(label, list);
        const n = list.indexOf(signature);
        id = `@embedded/${label}${n ? `~${n + 1}` : ''}`;
      }
      const item = copies.get(id) || { id, name: label, list: [] };
      copies.set(id, item);
      let copy = item.list.find(c => c.signature === signature);
      if (!copy) item.list.push(copy = { signature, document, dependencies, file: false, uses: 0, from: [] });
      if (from === null) copy.file = true; else { copy.uses++; if (!copy.from.includes(from)) copy.from.push(from); }
      return id;
    }
    for (const [id, doc] of uploaded) {
      try { collect(id, doc, id.split('/').pop(), null, []); } catch (error) { errors.push(error.message); }
    }
    // One copy per item. A disagreement is a prompt with a default, not a
    // refusal: the uploaded file wins, else the copy most parents embed.
    const conflicts = [], chosen = new Map();
    for (const item of copies.values()) {
      let pick = item.list[0];
      if (item.list.length > 1) {
        const file = item.list.find(c => c.file);
        const embedded = item.list.filter(c => !c.file).sort((a, b) => b.uses - a.uses)[0];
        const fallback = file ? 'file' : `copy:${item.list.indexOf(embedded)}`;
        let choice = choices[item.id] ?? fallback;
        if (choice === 'file' && file) pick = file;
        else if (choice === 'embedded') pick = embedded;
        else if (/^copy:\d+$/.test(choice) && item.list[Number(choice.slice(5))]) pick = item.list[Number(choice.slice(5))];
        else { choice = fallback; pick = file || embedded; }
        const others = item.list.filter(c => !c.file).flatMap(c => c.from);
        conflicts.push({
          id: item.id, name: item.name, choice,
          options: item.list.map((c, i) => ({ value: c.file ? 'file' : `copy:${i}`, label: c.file ? 'the uploaded file' : `the copy in ${c.from.join(', ')}` })),
          message: file
            ? `${item.name}: the copy embedded in ${[...new Set(others)].join(', ')} differs from the uploaded file (it may be older); importing the uploaded file`
            : `${item.name}: ${item.list.length} different embedded copies (in ${[...new Set(others)].join(', ')}); importing the copy in ${pick.from.join(', ')}`,
        });
      }
      chosen.set(item.id, pick);
    }
    // Rows are the items the chosen copies reach, parents before children.
    const rows = [], seen = new Set();
    function visit(id) {
      if (seen.has(id) || !chosen.has(id)) return;
      seen.add(id);
      const item = copies.get(id), copy = chosen.get(id);
      rows.push({ id, name: item.name, document: clone(copy.document), assembly: copy.document.features.some(isComponent), dependencies: copy.dependencies, root: uploaded.has(id) });
      for (const dependency of Object.values(copy.dependencies)) if (dependency.id) visit(dependency.id);
    }
    const used = new Set([...chosen.values()].flatMap(c => Object.values(c.dependencies).map(d => d.id)));
    const roots = [...uploaded.keys()].filter(id => chosen.has(id) && !used.has(id));
    for (const id of [...roots, ...uploaded.keys()]) visit(id);
    // The planned tree: each level's components with their counts at that level.
    const byId = new Map(rows.map(row => [row.id, row]));
    function branch(id, name, quantity, depth) {
      const row = byId.get(id), children = [], index = new Map();
      if (row && depth < 64) for (const feature of row.document.features.filter(isComponent)) {
        const part = feature.inputParams.partName, dependency = row.dependencies[part];
        if (!dependency) continue;
        const target = dependency.id || dependency.key;
        if (index.has(target)) { index.get(target).quantity++; continue; }
        const child = { id: target, name: part, quantity: 1, existing: Boolean(dependency.key), children: [] };
        index.set(target, child); children.push(child);
      }
      for (const child of children) if (!child.existing) child.children = branch(child.id, child.name, child.quantity, depth + 1).children;
      return { id, name: row?.name ?? name, quantity, children };
    }
    const tree = roots.map(id => branch(id, byId.get(id)?.name, 1, 0));
    return { rows, errors: [...new Set(errors)], conflicts, tree };
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
  // Propose an action for every row already imported under its external_ref,
  // children first. An item whose own content (its features, mates and the
  // revisions it places) equals the server's newest revision is reused, so an
  // unchanged tree writes nothing and a changed sub-assembly rewrites only
  // itself. A parent's embedded copy of a child is a cache that readers refresh
  // from the child's key, so it does not make the parent differ.
  async function propose(rows, api) {
    const byId = new Map(rows.map(row => [row.id, row])), targets = Object.create(null), done = new Set();
    async function visit(row) {
      if (done.has(row.id)) return;
      done.add(row.id);
      for (const dependency of Object.values(row.dependencies)) if (dependency.id && byId.has(dependency.id)) await visit(byId.get(dependency.id));
      row.unchanged = false; row.note = '';
      if (!row.external_ref) return;
      const matches = (await api('GET', `/api/parts?external_ref=${encodeURIComponent(row.external_ref)}&limit=2`)).parts;
      if (matches.length !== 1) return;
      const p = await api('GET', `/api/parts/${encodeURIComponent(matches[0].id)}`);
      const rev = p.revision_views.at(-1);
      row.part = p.number; row.part_type = p.part_type; row.available = p.revision_views; row.revision = rev?.id || '';
      if (!rev) return;
      const key = targetKey({ part: p.id, revision: rev.id });
      let unchanged = false;
      if (rev.content_hash) {
        const keys = Object.fromEntries(Object.entries(row.dependencies).map(([name, d]) => [name, d.key || (targets[d.id] ? targetKey(targets[d.id]) : null)]));
        if (Object.values(keys).every(Boolean)) {
          const server = await api('GET', `/api/store/doc/${key.replaceAll('%', '%25')}`);
          unchanged = Boolean(server) && own(server, (_, entry) => ({ sourceKey: entry.sourceKey })) === own(row.document, name => ({ sourceKey: keys[name] }));
        }
      }
      row.unchanged = unchanged;
      row.action = unchanged ? 'reuse' : rev.editable && !rev.locked_by ? 'replace' : p.revision_views.some(r => r.editable) ? 'reuse' : 'revise';
      // A change that cannot be written here is said, not hidden under Reuse.
      const held = !unchanged && row.action === 'reuse' && p.revision_views.find(r => r.editable && r.locked_by);
      row.note = held ? `changed, but draft ${held.label} is checked out by ${held.locked_by}: reusing it unless you choose another action` : '';
      // A revised child gets a new key, so its parents cannot be unchanged.
      if (row.action !== 'revise') targets[row.id] = { part: p.id, revision: rev.id };
    }
    for (const row of rows) await visit(row);
    return rows;
  }
  // After the user changes a row's action, revision or part by hand, the
  // proposals of the items that place it are stale: re-decide those ancestors
  // (children first) against what every row is now set to. A row whose action
  // the user chose by hand (`userAction`) keeps it.
  async function repropose(rows, api, changedId) {
    const byId = new Map(rows.map(row => [row.id, row]));
    const parents = new Map();
    for (const row of rows) for (const d of Object.values(row.dependencies)) if (d.id) (parents.get(d.id) || parents.set(d.id, []).get(d.id)).push(row);
    const order = [], seen = new Set();
    (function up(id) {
      for (const parent of parents.get(id) || []) {
        if (seen.has(parent.id)) continue;
        seen.add(parent.id); order.push(parent); up(parent.id);
      }
    })(changedId);
    const depth = new Map();
    const height = row => {
      if (depth.has(row.id)) return depth.get(row.id);
      depth.set(row.id, 0);
      const h = 1 + Math.max(0, ...Object.values(row.dependencies).filter(d => d.id && byId.has(d.id)).map(d => height(byId.get(d.id))));
      depth.set(row.id, h); return h;
    };
    order.sort((a, b) => height(a) - height(b));
    const target = row => ['reuse', 'replace'].includes(row.action) && row.part && row.revision ? { part: row.part.trim(), revision: row.revision } : null;
    for (const row of order) {
      if (row.userAction || !row.part || !row.revision) continue;
      const view = (row.available || []).find(r => r.id === row.revision);
      if (!view) continue;
      const keys = Object.fromEntries(Object.entries(row.dependencies).map(([name, d]) => [name, d.key || (target(byId.get(d.id)) ? targetKey(target(byId.get(d.id))) : null)]));
      let unchanged = false;
      if (view.content_hash && Object.values(keys).every(Boolean)) {
        const server = await api('GET', `/api/store/doc/${targetKey({ part: row.part.trim(), revision: row.revision }).replaceAll('%', '%25')}`);
        unchanged = Boolean(server) && own(server, (_, entry) => ({ sourceKey: entry.sourceKey })) === own(row.document, name => ({ sourceKey: keys[name] }));
      }
      if (unchanged === row.unchanged) continue;
      row.unchanged = unchanged;
      const views = row.available || [];
      row.action = unchanged ? 'reuse' : view.editable && !view.locked_by ? 'replace' : views.some(r => r.editable) ? 'reuse' : 'revise';
      const held = !unchanged && row.action === 'reuse' && views.find(r => r.editable && r.locked_by);
      row.note = held ? `changed, but draft ${held.label} is checked out by ${held.locked_by}: reusing it unless you choose another action` : '';
    }
    return rows;
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
  return { plan, canonical, path, externalRef, rewritten, inspect, propose, repropose, run };
})();
