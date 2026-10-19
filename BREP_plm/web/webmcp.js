// WebMCP for the PLM page: the server's API as the tools a browser's own
// agent may call (https://webmachinelearning.github.io/webmcp/).
//
// The CAD app derives its tools from a command registry; the PLM page has
// none, its API IS the registry, so the table below names each tool by hand
// against the handler it calls, with the argument schema the handler's
// request type takes (BREP_plm/src/api/*.rs). Every call goes through the
// page's own `api()` (app.js): the session cookie and the CSRF token are
// handled in ONE place, and a refusal is the server's own sentence — the
// page never decides what a user may do, and neither does a tool.
//
// The browser API is the part that moves: Chrome 155 exposes
// `document.modelContext` with `registerTool` / `getTools` / `executeTool`
// behind `--enable-features=WebMCP` (chrome://flags/#enable-webmcp-testing);
// the 2025 proposal had `navigator.modelContext` and `provideContext`, which
// is why the lookup tries both objects and never calls `provideContext`.
// Registration happens once, at load, signed in or not: Chrome 155 has no
// unregisterTool, and a tool called without a session answers the server's
// 401 as `{error}`. Signing in is the person's, through the form; there is no
// sign-in tool on purpose.
//
// Whatever the browser offers, the page gets `window.__plmWebMcp`:
//   api         'document.modelContext' | 'navigator.modelContext' | 'none'
//   tools       every tool name, in table order
//   registered  how many registerTool calls succeeded
//   errors      the registerTool failures, as `name: reason`
//   call(name, args)  the same path a registered tool's execute takes; a
//                     refusal rejects here and resolves as `{error}` there,
//                     because Chrome 155 reports a rejected execute to its
//                     agent as a bare "invocation failed" and the reason —
//                     the stray argument, the lock another user holds, the
//                     released revision — is what an agent needs to read.
// — which is how the verifier (BREP_plm/tests/web/webmcp-browser.mjs) and a
// devtools console drive the same tools whether or not the browser has an
// agent.

'use strict';

(() => {
  // ------------------------------------------------------------ schema bits
  const PART = { type: 'string', description: 'the part number, which is the part id (for example CPART000000001)' };
  const REV = { type: 'string', description: 'the revision label, which is its id within the part (for example A)' };
  const str = (description) => ({ type: 'string', description });
  const int = (description, extra = {}) => ({ type: 'integer', description, ...extra });
  const bool = (description, dflt) => ({ type: 'boolean', description, ...(dflt === undefined ? {} : { default: dflt }) });
  const obj = (properties, required = []) => ({ type: 'object', properties, required, additionalProperties: false });
  const NONE = obj({});
  const enc = encodeURIComponent;
  const rev = (args) => `/api/parts/${enc(args.part)}/revisions/${enc(args.revision)}`;
  /** `?k=v&…` for the args that are set, from `keys`. */
  const query = (args, keys) => {
    const p = new URLSearchParams();
    for (const k of keys) {
      const v = args[k];
      if (v !== undefined && v !== null && v !== '') p.set(k, String(v));
    }
    const s = p.toString();
    return s ? `?${s}` : '';
  };
  /** The body fields that are set, from `keys`. */
  const pick = (args, keys) => {
    const out = {};
    for (const k of keys) if (args[k] !== undefined) out[k] = args[k];
    return out;
  };
  const READ = { readOnlyHint: true, consequentialHint: false };
  const WRITE = { readOnlyHint: false, consequentialHint: false };
  const CONSEQUENTIAL = { readOnlyHint: false, consequentialHint: true };

  // ------------------------------------------------------------- the page
  // Where the page is: the hash route, the view it shows, who is signed in.
  const page = () => ({
    signed_in: !!me,
    username: me ? me.username : null,
    groups: me ? me.groups : [],
    hash: location.hash || '',
    view: typeof visiblePage === 'object' && visiblePage ? visiblePage.view : null,
    title: typeof visiblePage === 'object' && visiblePage ? visiblePage.title : null,
  });
  // Wait for the router to settle: a render, and the render a redirect
  // (an administrator's page asked for by someone else) queues after it.
  const settled = async () => {
    for (;;) {
      const q = routeQueue;
      await q.catch(() => {});
      // Let a hashchange the render caused reach its listener.
      await new Promise((resolve) => setTimeout(resolve, 0));
      if (q === routeQueue) return;
    }
  };
  const navigate = async (args) => {
    const hash = args.hash.startsWith('#') ? args.hash : `#${args.hash}`;
    if (!me) throw new Error('not signed in: the page shows the sign-in form');
    if (location.hash === hash) {
      await route();
    } else {
      const changed = new Promise((resolve) => {
        const done = () => { window.removeEventListener('hashchange', done); resolve(); };
        window.addEventListener('hashchange', done);
      });
      location.hash = hash;
      await changed;
    }
    await settled();
    return page();
  };

  const HASHES = 'Routes: #/workspace (Home), #/parts, #/parts/<category id>, #/part/<number>, #/part/<number>/<revision>/<summary|revisions|structure|attachments|review|history|sourcing>, #/reviews (inbox), #/catalog, #/catalog/<category id>, #/sourcing, #/bake, #/bom-columns, #/occurrence-fields, #/types, #/workflows, #/workflow-run/<run id>, #/workflow-editor/<id or new>, #/native-import, #/account; administrators also #/users, #/scripts, #/settings, #/security, #/audit, #/backups, #/setup. A page the signed-in user may not open redirects, and the reply says where the page went.';

  // ------------------------------------------------------------- the tools
  // {name, description, inputSchema, annotations, run(args)}; `run` answers
  // an object, or throws the refusal.
  const TOOLS = [
    // -- the page ----------------------------------------------------------
    { name: 'plm_page', description: 'Where the PLM page is: whether someone is signed in and who, the hash route, the view shown and its title.', inputSchema: NONE, annotations: READ, run: async () => page() },
    { name: 'plm_navigate', description: `Open a page of the PLM web app by its hash route and wait for it to render; answers like plm_page. ${HASHES}`, inputSchema: obj({ hash: str('the route, for example #/part/CPART000000001 or #/reviews') }, ['hash']), annotations: WRITE, run: navigate },
    { name: 'plm_whoami', description: 'The signed-in user as the server sees them: username, display name, groups, and what they may do (can_author, can_checkin, is_admin, can_bake), with the server version and features.', inputSchema: NONE, annotations: READ, run: async () => { const who = await api('GET', '/api/me'); delete who.csrf; return who; } },

    // -- parts and revisions ------------------------------------------------
    { name: 'plm_parts_search', description: 'Search parts. `q` matches number, name, category, tag or attribute value; the filters narrow it. One page at a time: `limit` rows, then `after` with the `next` cursor the reply carries. Each row names the part (its number is its id), its latest revision label and state, category, tags and MPN.', inputSchema: obj({ q: str('free text'), part_type: str('a part type id'), category: str('a category id'), tag: str('a tag'), manufacturer: str('a manufacturer id'), supplier: str('a supplier id'), external_ref: str('an external reference'), limit: int('rows per page (default 50)', { minimum: 1 }), after: str('the `next` cursor of the previous page'), include_attributes: bool('carry each row\'s attribute values', false) }), annotations: READ, run: async (a) => api('GET', `/api/parts${query({ ...a, include: a.include_attributes ? 'attributes' : '' }, ['q', 'part_type', 'category', 'tag', 'manufacturer', 'supplier', 'external_ref', 'limit', 'after', 'include'])}`) },
    { name: 'plm_part_get', description: 'One part with every revision: label, lifecycle state, who holds its checkout, document key, review summary, provenance, thumbnail; the part\'s category schema, sourcing and suggested next label.', inputSchema: obj({ part: PART }, ['part']), annotations: READ, run: async (a) => api('GET', `/api/parts/${enc(a.part)}`) },
    { name: 'plm_part_create', description: 'Create a part with its first revision (a Draft). `part_type` is a part type id (see plm_part_types); `number` only for a type that numbers by free text or pattern. Needs the author group.', inputSchema: obj({ part_type: str('the part type id'), name: str('the part name'), description: str(''), category: str('a category id'), number: str('the part number, for a type whose numbering allows it'), label: str('the first revision\'s label (default: the type\'s first)'), tags: { type: 'array', items: { type: 'string' } }, attributes: { type: 'object', description: 'attribute values by attribute id', additionalProperties: true }, external_ref: str('an external reference'), workspace_folder: str('a workspace folder id to link the part into') }, ['part_type', 'name']), annotations: WRITE, run: async (a) => api('POST', '/api/parts', pick(a, ['part_type', 'name', 'description', 'category', 'number', 'label', 'tags', 'attributes', 'external_ref', 'workspace_folder'])) },
    { name: 'plm_part_update', description: 'Change a part\'s name, description, category, tags, attributes or external_ref. Only the fields given change. Needs the author group.', inputSchema: obj({ part: PART, name: str(''), description: str(''), category: str('a category id'), tags: { type: 'array', items: { type: 'string' } }, attributes: { type: 'object', description: 'attribute values by attribute id; a null value clears one', additionalProperties: true }, external_ref: str('') }, ['part']), annotations: WRITE, run: async (a) => api('PATCH', `/api/parts/${enc(a.part)}`, pick(a, ['name', 'description', 'category', 'tags', 'attributes', 'external_ref'])) },
    { name: 'plm_revision_create', description: 'Add a revision to a part, as a Draft carrying the newest revision\'s document. Needs the author group.', inputSchema: obj({ part: PART, label: str('the new label (default: the part\'s suggested next label)') }, ['part']), annotations: WRITE, run: async (a) => api('POST', `/api/parts/${enc(a.part)}/revisions`, pick(a, ['label'])) },
    { name: 'plm_revision_checkout', description: 'Check a Draft revision out to the signed-in user: the lock a save, a uses-list change and a check-in need. Refused when another user holds it.', inputSchema: obj({ part: PART, revision: REV, client_id: str('a name for the client holding the lock') }, ['part', 'revision']), annotations: WRITE, run: async (a) => api('POST', `${rev(a)}/checkout`, pick(a, ['client_id'])) },
    { name: 'plm_revision_checkin', description: 'Release a revision\'s checkout. `force: true` breaks another user\'s lock, which needs the checkin group.', inputSchema: obj({ part: PART, revision: REV, force: bool('break a lock held by someone else', false) }, ['part', 'revision']), annotations: WRITE, run: async (a) => api('POST', `${rev(a)}/checkin`, pick(a, ['force'])) },
    { name: 'plm_revision_set_state', description: 'Move a revision through its lifecycle: draft, inreview, released, superseded or obsolete. Release is gated by the review rules; a released revision is immutable afterwards.', inputSchema: obj({ part: PART, revision: REV, to: { type: 'string', enum: ['draft', 'inreview', 'released', 'superseded', 'obsolete'], description: 'the state to move to' } }, ['part', 'revision', 'to']), annotations: CONSEQUENTIAL, run: async (a) => api('POST', `${rev(a)}/state`, { to: a.to }) },
    { name: 'plm_revision_delete', description: 'Delete a Draft revision that was never released. Cannot be undone.', inputSchema: obj({ part: PART, revision: REV }, ['part', 'revision']), annotations: CONSEQUENTIAL, run: async (a) => api('DELETE', rev(a)) },
    { name: 'plm_revision_attributes_get', description: 'A revision\'s typed field values (the part type\'s fields), with each field\'s definition.', inputSchema: obj({ part: PART, revision: REV }, ['part', 'revision']), annotations: READ, run: async (a) => api('GET', `${rev(a)}/attributes`) },
    { name: 'plm_revision_attributes_set', description: 'Set typed field values on a revision in work. Only the fields given change; a null clears one.', inputSchema: obj({ part: PART, revision: REV, attributes: { type: 'object', description: 'values by field id', additionalProperties: true } }, ['part', 'revision', 'attributes']), annotations: WRITE, run: async (a) => api('PATCH', `${rev(a)}/attributes`, { attributes: a.attributes }) },
    { name: 'plm_part_history', description: 'The audit trail of one part: every committed change, newest first, with who, when and what changed.', inputSchema: obj({ part: PART, user: str('only this user id'), action: str('only this action'), since: int('unix seconds'), until: int('unix seconds'), limit: int('rows (default 100)', { minimum: 1 }), before: int('the sequence number to page back from') }, ['part']), annotations: READ, run: async (a) => api('GET', `/api/parts/${enc(a.part)}/history${query(a, ['user', 'action', 'since', 'until', 'limit', 'before'])}`) },
    { name: 'plm_document_get', description: 'The CAD document stored for a revision (the .nbrep JSON, which can be large), or `document: null` for a revision with no model yet. The key is part/<number>/rev/<label>.', inputSchema: obj({ part: PART, revision: REV }, ['part', 'revision']), annotations: READ, run: async (a) => ({ key: `part/${a.part}/rev/${a.revision}`, document: await api('GET', `/api/store/doc/part/${enc(a.part)}/rev/${enc(a.revision)}`) }) },
    { name: 'plm_part_types', description: 'The part types: id, name, how each numbers (counter, free text, pattern or script) and its document class.', inputSchema: NONE, annotations: READ, run: async () => ({ part_types: await api('GET', '/api/part-types') }) },

    // -- review and approval ------------------------------------------------
    { name: 'plm_inbox', description: 'What waits on the signed-in user (reviews to decide, workflow tasks) and what they submitted, with due dates and approval counts.', inputSchema: NONE, annotations: READ, run: async () => api('GET', '/api/inbox') },
    { name: 'plm_review_get', description: 'A revision\'s review: its lifecycle, the current round (status, who opened it, approvals and which are stale, the required count, whether it is met), the reviewers, the discussion thread, and whether the signed-in user may decide.', inputSchema: obj({ part: PART, revision: REV }, ['part', 'revision']), annotations: READ, run: async (a) => api('GET', `${rev(a)}/review`) },
    { name: 'plm_review_submit', description: 'Submit a Draft revision for review (it moves to InReview), with extra reviewers, a due date and a note. Needs the author group.', inputSchema: obj({ part: PART, revision: REV, reviewers: { type: 'array', items: { type: 'string' }, description: 'reviewers beside the rule\'s: "user:<username>" or "group:<group>"' }, due: int('unix seconds'), note: str('a comment posted with the submission') }, ['part', 'revision']), annotations: WRITE, run: async (a) => api('POST', `${rev(a)}/submit`, pick(a, ['reviewers', 'due', 'note'])) },
    { name: 'plm_review_decide', description: 'Approve or reject a revision under review, as one of its reviewers, with a comment.', inputSchema: obj({ part: PART, revision: REV, verdict: { type: 'string', enum: ['approve', 'reject'] }, comment: str('') }, ['part', 'revision', 'verdict']), annotations: CONSEQUENTIAL, run: async (a) => api('POST', `${rev(a)}/review/decision`, pick(a, ['verdict', 'comment'])) },
    { name: 'plm_review_comment', description: 'Post a comment on a revision\'s discussion, optionally as a reply.', inputSchema: obj({ part: PART, revision: REV, body: str('the comment'), parent: str('the comment id replied to') }, ['part', 'revision', 'body']), annotations: WRITE, run: async (a) => api('POST', `${rev(a)}/comments`, pick(a, ['body', 'parent'])) },

    // -- assembly structure -------------------------------------------------
    { name: 'plm_uses_get', description: 'A revision\'s uses list: each child part, pinned revision (empty is floating: the child\'s released revision, else its newest), quantity, unit, find number and reference.', inputSchema: obj({ part: PART, revision: REV }, ['part', 'revision']), annotations: READ, run: async (a) => api('GET', `${rev(a)}/uses`) },
    { name: 'plm_uses_set', description: 'Replace a revision\'s uses list. The author group, holding the revision\'s checkout, on a revision in work.', inputSchema: obj({ part: PART, revision: REV, uses: { type: 'array', description: 'the whole list', items: obj({ part: PART, revision: str('the child revision label; empty or absent floats'), quantity: { type: 'number', description: 'per one parent (default 1)' }, unit: str('each unless stated (mm, g)'), find_number: str('the balloon number'), reference: str('reference designators or occurrence names'), notes: str('') }, ['part']) } }, ['part', 'revision', 'uses']), annotations: WRITE, run: async (a) => api('PUT', `${rev(a)}/uses`, { uses: a.uses }) },
    { name: 'plm_bom', description: 'A revision\'s bill of materials, as a tree or flat with quantities and costs rolled up.', inputSchema: obj({ part: PART, revision: REV, levels: str('how deep: a number, or all (default)'), flat: bool('roll the tree up into one line per part', false), occurrences: bool('expand occurrences with their typed fields', false) }, ['part', 'revision']), annotations: READ, run: async (a) => api('GET', `${rev(a)}/bom${query(a, ['levels', 'flat', 'occurrences'])}`) },
    { name: 'plm_bom_diff', description: 'What changed in a part\'s BOM between two of its revisions: lines added, removed and changed.', inputSchema: obj({ part: PART, from: str('the earlier revision label'), to: str('the later revision label') }, ['part', 'from', 'to']), annotations: READ, run: async (a) => api('GET', `/api/parts/${enc(a.part)}/bom/diff${query(a, ['from', 'to'])}`) },
    { name: 'plm_where_used', description: 'The assemblies that use a part, up the tree.', inputSchema: obj({ part: PART, revision: str('only this revision of the part'), levels: str('how far up: a number, or all (default)') }, ['part']), annotations: READ, run: async (a) => api('GET', `/api/parts/${enc(a.part)}/where-used${query(a, ['revision', 'levels'])}`) },
    { name: 'plm_attachments_list', description: 'The files attached to a part and, when a revision is named, to that revision: name, kind, size, who uploaded it and when.', inputSchema: obj({ part: PART, revision: str('a revision label, for its files too') }, ['part']), annotations: READ, run: async (a) => api('GET', `/api/parts/${enc(a.part)}/attachments${query(a, ['revision'])}`) },

    // -- catalog and sourcing -----------------------------------------------
    { name: 'plm_categories', description: 'The category tree: each category with its path, depth, part counts and typed attribute definitions.', inputSchema: NONE, annotations: READ, run: async () => ({ categories: await api('GET', '/api/categories') }) },
    { name: 'plm_category_schema', description: 'The attributes a part in a category carries, its own and inherited.', inputSchema: obj({ category: str('the category id') }, ['category']), annotations: READ, run: async (a) => api('GET', `/api/categories/${enc(a.category)}/schema`) },
    { name: 'plm_manufacturers', description: 'The manufacturers.', inputSchema: NONE, annotations: READ, run: async () => ({ manufacturers: await api('GET', '/api/manufacturers') }) },
    { name: 'plm_suppliers', description: 'The suppliers.', inputSchema: NONE, annotations: READ, run: async () => ({ suppliers: await api('GET', '/api/suppliers') }) },

    // -- workflows ---------------------------------------------------------
    { name: 'plm_workflows', description: 'The workflow definitions a run can start from.', inputSchema: NONE, annotations: READ, run: async () => api('GET', '/api/workflows') },
    { name: 'plm_workflow_runs', description: 'The workflow runs: title, definition, state, initiator, and the part and revision each is about.', inputSchema: NONE, annotations: READ, run: async () => api('GET', '/api/workflow-runs') },
    { name: 'plm_workflow_run', description: 'One workflow run with its nodes, their states, the actions open to the signed-in user and the form fields.', inputSchema: obj({ run: str('the run id') }, ['run']), annotations: READ, run: async (a) => api('GET', `/api/workflow-runs/${enc(a.run)}`) },
    { name: 'plm_workflow_start', description: 'Start a workflow run on a part revision.', inputSchema: obj({ workflow: str('the workflow definition id'), title: str(''), part: PART, revision: REV, fields: { type: 'object', description: 'the start form\'s values', additionalProperties: true } }, ['workflow', 'part', 'revision']), annotations: WRITE, run: async (a) => api('POST', '/api/workflow-runs', pick(a, ['workflow', 'title', 'part', 'revision', 'fields'])) },
    { name: 'plm_workflow_act', description: 'Take an action on a workflow run\'s node: choose an outcome, with a comment and the node\'s form values.', inputSchema: obj({ run: str('the run id'), node: str('the node id'), outcome: str('the outcome chosen'), comment: str(''), fields: { type: 'object', additionalProperties: true } }, ['run', 'node', 'outcome']), annotations: CONSEQUENTIAL, run: async (a) => api('POST', `/api/workflow-runs/${enc(a.run)}/actions`, pick(a, ['node', 'outcome', 'comment', 'fields'])) },

    // -- workspaces --------------------------------------------------------
    { name: 'plm_workspaces', description: 'The workspaces the signed-in user may browse: their own and the shared ones.', inputSchema: NONE, annotations: READ, run: async () => api('GET', '/api/workspaces') },
    { name: 'plm_workspace_folder', description: 'The entries in one workspace folder: subfolders, links to parts, and files with their versions.', inputSchema: obj({ owner: str('a user id or username; empty is the signed-in user'), parent: str('a folder id; empty is the top') }), annotations: READ, run: async (a) => api('GET', `/api/workspace/entries${query(a, ['owner', 'parent'])}`) },

    // -- the bake queue and the audit log ------------------------------------
    { name: 'plm_bake_jobs', description: 'The bake queue: each job\'s part, state, attempts and error.', inputSchema: NONE, annotations: READ, run: async () => api('GET', '/api/bake/jobs') },
    { name: 'plm_audit', description: 'The audit log across the server (administrators): every committed change, newest first.', inputSchema: obj({ kind: str('only this entity kind'), entity: str('only this entity id'), part: str('only this part'), user: str('only this user id'), action: str('only this action'), since: int('unix seconds'), until: int('unix seconds'), limit: int('rows', { minimum: 1 }), before: int('the sequence number to page back from') }), annotations: READ, run: async (a) => api('GET', `/api/audit${query(a, ['kind', 'entity', 'part', 'user', 'action', 'since', 'until', 'limit', 'before'])}`) },
  ];

  // ------------------------------------------------------------ the bridge
  const byName = new Map(TOOLS.map((t) => [t.name, t]));

  /** The arguments against the tool's schema: a stray or missing one is a refusal with its name. */
  const check = (tool, args) => {
    if (args === null || typeof args !== 'object' || Array.isArray(args)) throw new Error(`${tool.name}: arguments must be an object`);
    const schema = tool.inputSchema;
    for (const k of Object.keys(args)) {
      if (!(k in schema.properties)) throw new Error(`${tool.name}: unknown argument \`${k}\``);
    }
    for (const k of schema.required || []) {
      if (args[k] === undefined || args[k] === null || args[k] === '') throw new Error(`${tool.name}: \`${k}\` is required`);
    }
  };

  const call = async (name, args = {}) => {
    const tool = byName.get(name);
    if (!tool) throw new Error(`unknown tool \`${name}\``);
    check(tool, args ?? {});
    if (typeof api !== 'function') throw new Error('the page has not loaded');
    const result = await tool.run(args ?? {});
    // A reply is always an object: a bare value or an empty 204 is wrapped.
    if (result === null || result === undefined) return { ok: true };
    return typeof result === 'object' && !Array.isArray(result) ? result : { value: result };
  };

  const status = { api: 'none', tools: TOOLS.map((t) => t.name), registered: 0, errors: [], call };
  window.__plmWebMcp = status;

  const [which, mc] = document.modelContext
    ? ['document.modelContext', document.modelContext]
    : navigator.modelContext
      ? ['navigator.modelContext', navigator.modelContext]
      : ['none', null];
  if (!mc || typeof mc.registerTool !== 'function') {
    console.info('WebMCP: this browser exposes no modelContext; the tools are reachable as window.__plmWebMcp.call(name, args)');
    return;
  }
  status.api = which;
  (async () => {
    for (const t of TOOLS) {
      try {
        await mc.registerTool({
          name: t.name,
          title: t.name,
          description: t.description,
          inputSchema: t.inputSchema,
          // A refusal RESOLVES as `{error}` (see the top of the file).
          execute: (input) => call(t.name, input ?? {}).catch((e) => ({ error: String(e && e.message ? e.message : e) })),
          annotations: t.annotations,
        });
        status.registered += 1;
      } catch (e) {
        status.errors.push(`${t.name}: ${e}`);
      }
    }
    if (status.errors.length) {
      console.error(`WebMCP: ${status.errors.length} of ${TOOLS.length} tools failed to register`, status.errors);
    } else {
      console.info(`WebMCP: ${status.registered} tools registered on ${which}`);
    }
    status.ready = true;
  })();
})();
