// The PLM page. Vanilla, hash-routed, one fetch helper.
//
// The page never decides what a user may do — it asks the server (`/api/me`
// resolves the group rules into `can_author` / `can_checkin` / `is_admin`) and
// only DISABLES what the server would refuse. A button the page enables and
// the server rejects is a bug in one place; two copies of the rules would be a
// bug in two.

'use strict';

const $ = (id) => document.getElementById(id);
let me = null;
let currentPart = null;

// ---------------------------------------------------------------- transport

// The session's CSRF token, from /api/login and /api/me. Every request that
// changes anything sends it back; the server refuses one that does not.
let csrfToken = '';

async function api(method, path, body) {
  const options = { method, headers: {}, credentials: 'same-origin' };
  if (method !== 'GET' && csrfToken) options.headers['X-CSRF-Token'] = csrfToken;
  if (body !== undefined) {
    if (typeof body === 'string') {
      options.headers['Content-Type'] = 'application/json';
      options.body = body;
    } else {
      options.headers['Content-Type'] = 'application/json';
      options.body = JSON.stringify(body);
    }
  }
  const response = await fetch(path, options);
  if (response.status === 204) return null;
  const text = await response.text();
  let payload = null;
  if (text) {
    try { payload = JSON.parse(text); } catch { payload = text; }
  }
  if (!response.ok) {
    const message = payload && payload.error ? payload.error : `${response.status} ${response.statusText}`;
    throw new Error(message);
  }
  return payload;
}

function banner(message) {
  const el = $('banner');
  el.textContent = message || '';
  el.hidden = !message;
  // A modal dialog covers the page banner, so a refusal of what the dialog
  // just sent is also written inside the dialog, above its buttons.
  for (const old of document.querySelectorAll('p.dialog-error')) old.remove();
  const open = document.querySelector('dialog[open] form');
  if (message && open) {
    const line = document.createElement('p');
    line.className = 'error dialog-error';
    line.setAttribute('role', 'alert');
    line.textContent = message;
    open.insertBefore(line, open.querySelector('menu'));
  } else if (message) {
    window.scrollTo({ top: 0, behavior: 'smooth' });
  }
}

// A refusal belongs to the dialog that caused it: closing the dialog, by
// saving or by cancelling, clears it.
for (const dialog of document.querySelectorAll('dialog')) {
  dialog.addEventListener('close', () => banner(''));
}

// Run an action, show any refusal, and refresh. Every mutating button goes
// through this so a server message always reaches the user.
async function act(fn) {
  banner('');
  try {
    await fn();
  } catch (error) {
    banner(error.message);
  }
}

const escape = (value) => String(value ?? '').replace(/[&<>"']/g, (c) => (
  { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]
));

const CLASS_NAMES = { normal: 'normal', family: 'family', template: 'template' };
const classBadge = (cls) => `<span class="badge badge-${escape(cls)}">${escape(CLASS_NAMES[cls] || cls)}</span>`;

// How a part type numbers, in words.
function modeText(mode) {
  switch (mode && mode.kind) {
    case 'free': return 'free text';
    case 'pattern': return `pattern <code>${escape(mode.regex)}</code>`;
    case 'script': return `script <code>${escape(mode.script)}</code>`;
    default: return 'counter';
  }
}

// A part's or revision's thumbnail (P9), from the URL the server hands out
// (versioned by the picture, so the browser keeps it). No picture — none
// made yet, stale after a save, or one that fails to load — is a neutral
// placeholder box, never a broken image.
const thumb = (url, size = 'sm', title = '') => (url
  ? `<img class="thumb thumb-${size}" src="${escape(url)}" alt="" loading="lazy" decoding="async"${title ? ` title="${escape(title)}"` : ''}>`
  : `<span class="thumb thumb-${size} thumb-none"${title ? ` title="${escape(title)}"` : ''} aria-hidden="true"></span>`);
document.addEventListener('error', (event) => {
  const img = event.target;
  if (!(img instanceof HTMLImageElement) || !img.classList.contains('thumb')) return;
  const box = document.createElement('span');
  box.className = `${img.className} thumb-none`;
  box.setAttribute('aria-hidden', 'true');
  img.replaceWith(box);
}, true);

const when = (seconds) => (seconds ? new Date(seconds * 1000).toLocaleString() : '—');
const bytes = (n) => (n ? `${n.toLocaleString()} B` : '—');

// ------------------------------------------------------------------- routing

const VIEWS = ['parts', 'workspace', 'part', 'reviews', 'ecos', 'eco', 'catalog', 'sourcing', 'bake', 'types', 'users', 'scripts', 'settings', 'account', 'security', 'audit', 'backups'];

function show(view) {
  for (const name of VIEWS) $(`view-${name}`).hidden = name !== view;
  for (const link of document.querySelectorAll('header nav a')) {
    link.classList.toggle('current', link.getAttribute('href') === `#/${view}`);
  }
}

async function route() {
  if (!me) return;
  banner('');
  const hash = location.hash || '#/parts';
  const [, section, id] = hash.split('/');
  try {
    if (section === 'part' && id) {
      await renderPart(id);
      show('part');
    } else if (section === 'workspace') {
      await renderWorkspace();
      show('workspace');
    } else if (section === 'reviews') {
      await renderReviews();
      show('reviews');
    } else if (section === 'ecos') {
      await renderEcos();
      show('ecos');
    } else if (section === 'eco' && id) {
      await renderEco(decodeURIComponent(id));
      show('eco');
    } else if (section === 'catalog') {
      await renderCatalog(id ? decodeURIComponent(id) : null);
      show('catalog');
    } else if (section === 'sourcing') {
      await renderCompanies();
      show('sourcing');
    } else if (section === 'bake') {
      await renderBake();
      show('bake');
    } else if (section === 'types') {
      await renderTypes();
      show('types');
    } else if (section === 'scripts') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderScripts();
      show('scripts');
    } else if (section === 'settings') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderSettings();
      show('settings');
    } else if (section === 'audit') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderAudit();
      show('audit');
    } else if (section === 'account') {
      await renderAccount();
      show('account');
    } else if (section === 'backups') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderBackups();
      show('backups');
    } else if (section === 'security') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderSecurity();
      show('security');
    } else if (section === 'users') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      await renderUsers();
      show('users');
    } else {
      // `#/parts/<category id>` opens the list filtered to that category.
      if (section === 'parts' && id) {
        await loadCategories();
        fillCategoryFilter();
        $('filter-category').value = decodeURIComponent(id);
      }
      // Re-read on every visit: companies are added on another page, or by
      // another user.
      await loadCompanies();
      await renderParts();
      show('parts');
    }
  } catch (error) {
    banner(error.message);
  }
  // The badge follows every page change: a review may have arrived since.
  refreshInbox();
}

// -------------------------------------------------------------- open in CAD

// Where this server hosts the CAD app (`/cad/config`'s `app`), or null when it
// hosts none: then no "Open in CAD" button is shown anywhere.
let cadApp = null;

async function loadCadConfig() {
  try {
    const config = await api('GET', '/cad/config');
    cadApp = config && config.app ? config.app : null;
  } catch {
    cadApp = null;
  }
}

// An "Open in CAD" button for one revision, as HTML, or '' when no CAD app is
// hosted. Any view may drop it into its markup: the one listener below opens
// every such button in a new tab. Every document class (.nbrep, .fbrep, .tbrep)
// opens in the CAD app, and a revision with no document yet opens as a new one.
function openInCadButton(partId, revisionId, label = 'Open in CAD') {
  if (!cadApp || !partId || !revisionId) return '';
  const key = `part/${partId}/rev/${revisionId}`;
  return `<button class="link" data-open-cad="${escape(key)}" title="Open this revision in the CAD app, in a new tab">${escape(label)}</button>`;
}

document.addEventListener('click', (event) => {
  const button = event.target.closest && event.target.closest('[data-open-cad]');
  if (!button || !cadApp) return;
  event.preventDefault();
  // A new tab on this origin: the CAD app signs in with this page's session.
  window.open(`${cadApp}?open=${encodeURIComponent(button.dataset.openCad)}`, '_blank');
});

// -------------------------------------------------------------------- signin

async function boot() {
  try {
    me = await api('GET', '/api/me');
    csrfToken = me.csrf || '';
  } catch {
    me = null;
    csrfToken = '';
  }
  $('signin').hidden = !!me;
  $('shell').hidden = !me;
  if (!me) return;

  $('whoami').textContent = `${me.display_name || me.username} · ${me.groups.join(', ') || 'no groups'}`;
  for (const el of document.querySelectorAll('.admin-only')) el.hidden = !me.is_admin;
  for (const el of document.querySelectorAll('.author-only')) el.hidden = !me.can_author;
  await loadCadConfig();
  await route();
}

$('login-form').addEventListener('submit', async (event) => {
  event.preventDefault();
  $('login-error').textContent = '';
  try {
    const signedIn = await api('POST', '/api/login', {
      username: $('login-user').value,
      password: $('login-pass').value,
    });
    csrfToken = signedIn.csrf || '';
    $('login-pass').value = '';
    await boot();
  } catch (error) {
    $('login-error').textContent = error.message;
  }
});

$('signout').addEventListener('click', () => act(async () => {
  await api('POST', '/api/logout');
  me = null;
  location.hash = '';
  location.reload();
}));

// --------------------------------------------------------------- parts list

// How many parts one page of the list shows; "Load more" fetches the next.
const PARTS_PAGE = 100;
// Bumped by every fresh render, so a slow page for an older search cannot
// land in the list after a newer one.
let partsGeneration = 0;
let partsNext = null;
let partsQuery = '';

async function renderParts() {
  if (!categoriesLoaded) {
    await loadCategories();
    fillCategoryFilter();
  }
  if (!companiesLoaded) await loadCompanies();
  // `m:<id>` or `s:<id>`: a manufacturer or a supplier.
  const [kind, company] = ($('filter-company').value || ':').split(':');
  const params = new URLSearchParams({
    q: $('search').value.trim(),
    category: $('filter-category').value,
    tag: $('filter-tag').value.trim(),
    manufacturer: kind === 'm' ? company : '',
    supplier: kind === 's' ? company : '',
  });
  const filtered = params.get('q') || params.get('category') || params.get('tag') || company;
  partsQuery = params.toString();
  const generation = ++partsGeneration;
  const page = await api('GET', `/api/parts?${partsQuery}&limit=${PARTS_PAGE}`);
  if (generation !== partsGeneration) return;
  $('parts-empty').hidden = page.parts.length > 0;
  $('parts-empty').textContent = filtered ? 'No parts match.' : 'No parts yet.';
  $('parts-rows').innerHTML = page.parts.map(partRow).join('');
  partsNext = page.next;
  $('parts-more').hidden = !partsNext;
}

async function moreParts() {
  if (!partsNext) return;
  const generation = partsGeneration;
  const page = await api('GET', `/api/parts?${partsQuery}&limit=${PARTS_PAGE}&after=${encodeURIComponent(partsNext)}`);
  if (generation !== partsGeneration) return;
  $('parts-rows').insertAdjacentHTML('beforeend', page.parts.map(partRow).join(''));
  partsNext = page.next;
  $('parts-more').hidden = !partsNext;
}

$('parts-more').addEventListener('click', () => act(moreParts));

function partRow(part) {
  return `
    <tr>
      <td class="thumb-cell"><a href="#/part/${escape(part.id)}" tabindex="-1">${thumb(part.thumbnail_url)}</a></td>
      <td class="number"><a href="#/part/${escape(part.id)}">${escape(part.number)}</a></td>
      <td>${escape(part.name)}${part.tags.length ? `<br>${tagChips(part.tags)}` : ''}</td>
      <td>${escape(part.part_type)}</td>
      <td>${classBadge(part.document_class)}</td>
      <td>${categoryText(part)}</td>
      <td class="mono">${escape(part.mpn) || '<span class="muted">—</span>'}</td>
      <td>${escape(part.latest_label)}</td>
      <td><span class="state state-${escape(part.latest_state)}">${escape(part.latest_state)}</span></td>
      <td class="actions">${part.locked ? '🔒' : ''}</td>
    </tr>`;
}

let searchTimer = null;
for (const id of ['search', 'filter-tag']) {
  $(id).addEventListener('input', () => {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(() => act(renderParts), 150);
  });
}
$('filter-category').addEventListener('change', () => act(renderParts));
$('filter-company').addEventListener('change', () => act(renderParts));

// A tag chip anywhere filters the parts list to that tag.
document.addEventListener('click', (event) => {
  const chip = event.target.closest('button.tag');
  if (!chip) return;
  event.preventDefault();
  $('filter-tag').value = chip.dataset.tag;
  if (location.hash.startsWith('#/parts')) act(renderParts);
  else location.hash = '#/parts';
});

// ----------------------------------------------------------------- one part

async function renderPart(id) {
  const detail = await api('GET', `/api/parts/${encodeURIComponent(id)}`);
  // The editor belongs to one part's document; opening another part closes it
  // rather than leaving the last part's text under this part's heading.
  if (editorKey && !editorKey.startsWith(`part/${detail.id}/`)) {
    $('editor').hidden = true;
    editorKey = null;
  }
  currentPart = detail;
  $('part-thumb').innerHTML = thumb(detail.thumbnail_url, 'lg');
  $('part-number').textContent = detail.number;
  $('part-name').textContent = detail.name;
  $('part-meta').innerHTML =
    `${escape(detail.part_type)} · ${classBadge(detail.document_class)} · created ${escape(when(detail.created_at))}`
    + (detail.description ? ` · ${escape(detail.description)}` : '');
  renderDetails(detail);
  renderSourcing(detail);
  await renderSeed(detail);
  await renderStructure(detail);
  await renderAttachments(detail);
  await renderReview(detail);
  await renderHistory(detail);

  const openDraft = detail.revision_views.some((r) => r.editable);
  const blocked = openDraft && !detail.multiple_open_drafts;
  $('new-rev').disabled = blocked;
  $('new-rev-label').disabled = blocked;
  $('new-rev-label').placeholder = detail.suggested_label;
  $('new-rev-note').textContent = blocked
    ? 'A revision is still open, and this server allows one at a time. Release or delete it first.'
    : `Labels are free text. Leave it empty to use ${detail.suggested_label}.`
      + (openDraft ? ' The new revision starts from the released document, not from the open one.' : '');

  $('rev-rows').innerHTML = detail.revision_views.map((rev) => {
    const held = rev.locked_by
      ? `${escape(rev.locked_by)}${rev.locked_by_me ? ' (you)' : ''}<br><span class="muted">${when(rev.locked_at)}</span>`
      : '—';
    return `
      <tr>
        <td><span class="rev-label">${revThumb(rev)}<strong>${escape(rev.label)}</strong></span></td>
        <td><span class="state state-${escape(rev.lifecycle)}">${escape(rev.lifecycle)}</span></td>
        <td>${reviewCell(rev)}</td>
        <td class="source">${sourceText(rev)}</td>
        <td>${held}</td>
        <td>${bytes(rev.size)}</td>
        <td class="mono muted">${escape(rev.content_hash.slice(0, 12)) || '—'}</td>
        <td class="actions">${actions(rev)}</td>
      </tr>`;
  }).join('');

  for (const button of $('rev-rows').querySelectorAll('button[data-do]')) {
    button.addEventListener('click', () => onRevisionAction(button.dataset.do, button.dataset.rev));
  }
}

function revThumb(rev) {
  const t = rev.thumbnail;
  if (!t) return thumb('', 'xs', 'No picture yet');
  if (!t.current) return thumb('', 'xs', 'The picture is of an older save; the next save from CAD makes a new one');
  return thumb(t.url, 'xs', `Rendered by ${t.renderer}`);
}

function actions(rev) {
  const out = [];
  const rid = escape(rev.id);
  const cad = openInCadButton(currentPart.id, rev.id);
  if (cad) out.push(cad);
  if (rev.editable) {
    if (!rev.locked_by && me.can_author) out.push(`<button class="link" data-do="checkout" data-rev="${rid}">Check out</button>`);
    if (rev.locked_by_me) {
      out.push(`<button class="link" data-do="edit" data-rev="${rid}">Edit</button>`);
      out.push(`<button class="link" data-do="checkin" data-rev="${rid}">Check in</button>`);
    } else if (rev.locked_by && me.can_checkin) {
      out.push(`<button class="link danger" data-do="break" data-rev="${rid}">Break lock</button>`);
    }
    if (rev.lifecycle === 'draft' && me.can_author) out.push(`<button class="link" data-do="submit" data-rev="${rid}">Submit for review</button>`);
    if (rev.lifecycle === 'inreview' && me.can_author) out.push(`<button class="link" data-do="withdraw" data-rev="${rid}">Withdraw</button>`);
    if (!rev.eco && me.can_author) out.push(`<button class="link" data-do="to-eco" data-rev="${rid}">Add to change order</button>`);
    if (!rev.locked_by && me.can_checkin) out.push(`<button class="link" data-do="release" data-rev="${rid}">Release</button>`);
    if (me.can_author) out.push(`<button class="link" data-do="delete" data-rev="${rid}">Delete</button>`);
  } else {
    out.push(`<button class="link" data-do="view" data-rev="${rid}">View</button>`);
    if (rev.lifecycle === 'released' && me.can_checkin) {
      out.push(`<button class="link" data-do="obsolete" data-rev="${rid}">Obsolete</button>`);
    }
    if (['released', 'superseded'].includes(rev.lifecycle) && !rev.eco && me.can_author) {
      out.push(`<button class="link" data-do="to-eco" data-rev="${rid}">Add to change order</button>`);
    }
  }
  return out.join(' ');
}

function onRevisionAction(action, revisionId) {
  const part = currentPart.id;
  const base = `/api/parts/${encodeURIComponent(part)}/revisions/${encodeURIComponent(revisionId)}`;
  act(async () => {
    switch (action) {
      case 'checkout':
        await api('POST', `${base}/checkout`, { client_id: 'web' });
        break;
      case 'checkin':
        await api('POST', `${base}/checkin`, { force: false });
        break;
      case 'break':
        if (!confirm('Break this lock? The holder may have unsaved work — their edit stays in their app and is NOT written here.')) return;
        await api('POST', `${base}/checkin`, { force: true });
        break;
      case 'release': {
        if (!confirm('Release this revision? A released revision is immutable.')) return;
        const result = await api('POST', `${base}/state`, { to: 'released' });
        reviewRevision = revisionId;
        await renderPart(part);
        // The release stands; an after-release script that failed says so.
        if (result.warnings && result.warnings.length) banner(result.warnings.join('\n'));
        return;
      }
      case 'submit':
        await openSubmit(revisionId);
        return;
      case 'to-eco':
        await openAddToEco(revisionId);
        return;
      case 'withdraw':
        if (!confirm('Withdraw this revision from review? Its review ends, and it goes back to Draft.')) return;
        showWarnings(await api('POST', `${base}/state`, { to: 'draft' }));
        break;
      case 'obsolete':
        if (!confirm('Mark this revision obsolete?')) return;
        await api('POST', `${base}/state`, { to: 'obsolete' });
        break;
      case 'delete':
        if (!confirm('Delete this draft revision and its document?')) return;
        await api('DELETE', base);
        break;
      case 'edit':
      case 'view':
        await openEditor(revisionId, action === 'edit');
        return;
    }
    await renderPart(part);
  });
}

// ------------------------------------------------------------------ editor

let editorKey = null;

async function openEditor(revisionId, writable) {
  const rev = currentPart.revision_views.find((r) => r.id === revisionId);
  editorKey = rev.document_key;
  const body = await api('GET', `/api/store/doc/${rev.document_key}`);
  $('editor').hidden = false;
  $('editor-key').textContent = rev.document_key;
  $('editor-body').value = body === null ? '' : (typeof body === 'string' ? body : JSON.stringify(body, null, 2));
  $('editor-body').readOnly = !writable;
  $('editor-save').hidden = !writable;
  $('editor-note').textContent = writable
    ? 'This is the CAD document. It is stored verbatim; the CAD app will read and write this same key.'
    : `Revision ${rev.label} is ${rev.lifecycle} and immutable — shown read-only.`;
  $('editor-status').textContent = '';
  $('editor').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

$('editor-save').addEventListener('click', () => act(async () => {
  const result = await api('PUT', `/api/store/doc/${editorKey}`, $('editor-body').value);
  $('editor-status').textContent = `saved · seq ${result.seq} · ${result.content_hash.slice(0, 12)}`;
  await renderPart(currentPart.id);
}));

// Load a CAD file (.fbrep, .tbrep, .nbrep) from disk into the editor: how a
// family or a template made in the file-based CAD app comes into the PLM
// before the CAD app is wired to it.
$('editor-file').addEventListener('change', () => act(async () => {
  const file = $('editor-file').files[0];
  if (!file) return;
  const text = await file.text();
  JSON.parse(text); // refuse a file that is not a document before it reaches the box
  $('editor-body').value = text;
  $('editor-status').textContent = `loaded ${file.name} — not saved yet`;
  $('editor-file').value = '';
}));

// The import rewrote a document the editor may be showing. Saving the stale
// text would silently undo the import, so the editor re-reads it.
async function reloadEditor(key) {
  if ($('editor').hidden || editorKey !== key) return;
  const body = await api('GET', `/api/store/doc/${key}`);
  $('editor-body').value = body === null ? '' : (typeof body === 'string' ? body : JSON.stringify(body, null, 2));
  $('editor-status').textContent = 'reloaded: the import changed this document';
}

$('editor-close').addEventListener('click', () => {
  $('editor').hidden = true;
  editorKey = null;
});

// --------------------------------------------------------------- new part

let partTypes = [];

// The number field follows the chosen type's mode: a counter owns its numbers,
// free text and pattern types need one typed, a script may take one or not.
function syncNumberField() {
  const kind = partTypes.find((t) => t.id === $('np-type').value);
  const mode = kind ? kind.mode : { kind: 'counter' };
  const input = $('np-number');
  input.disabled = mode.kind === 'counter';
  input.required = mode.kind === 'free' || mode.kind === 'pattern';
  if (mode.kind === 'counter') input.value = '';
  input.placeholder = mode.kind === 'counter' ? kind.next_number
    : mode.kind === 'pattern' ? mode.regex
    : mode.kind === 'script' ? 'optional — the script decides'
    : 'e.g. the ERP number';
  $('np-number-note').innerHTML = {
    counter: `Allocated by the counter. The next is <span class="mono">${escape(kind ? kind.next_number : '')}</span>.`,
    free: 'Type any number. It must not already be in use.',
    pattern: `Must match <code>${escape(mode.regex)}</code> in full.`,
    script: `<code>${escape(mode.script)}</code> decides. It sees what you type here, if anything.`,
  }[mode.kind] || '';
}

$('new-part').addEventListener('click', () => act(async () => {
  partTypes = await api('GET', '/api/part-types');
  $('np-type').innerHTML = partTypes
    .map((t) => `<option value="${escape(t.id)}">${escape(t.name)} (${escape(modeText(t.mode).replace(/<[^>]+>/g, ''))})</option>`)
    .join('');
  syncNumberField();
  await loadCategories();
  // Start in the category the list is filtered to, if it is one.
  const filtered = $('filter-category').value;
  categoryOptions($('np-category'), { blank: '— none —' });
  $('np-category').value = categories.some((c) => c.id === filtered) ? filtered : '';
  await syncAttrForm($('np-category').value, $('np-attrs'), {});
  $('new-part-dialog').showModal();
}));

$('np-category').addEventListener('change', () => act(() => syncAttrForm($('np-category').value, $('np-attrs'), readAttrForm($('np-attrs')))));

$('np-type').addEventListener('change', syncNumberField);

$('new-part-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'create') return;
  event.preventDefault();
  act(async () => {
    const part = await api('POST', '/api/parts', {
      part_type: $('np-type').value,
      number: $('np-number').value,
      name: $('np-name').value,
      document_class: $('np-class').value,
      category: $('np-category').value,
      description: $('np-description').value,
      label: $('np-label').value,
      tags: splitTags($('np-tags').value),
      attributes: readAttrForm($('np-attrs')),
    });
    $('new-part-dialog').close();
    for (const id of ['np-number', 'np-name', 'np-category', 'np-tags', 'np-description', 'np-label']) $(id).value = '';
    $('np-attrs').innerHTML = '';
    $('np-class').value = 'normal';
    location.hash = `#/part/${part.id}`;
  });
});

$('new-rev-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    await api('POST', `/api/parts/${encodeURIComponent(currentPart.id)}/revisions`, {
      label: $('new-rev-label').value,
    });
    $('new-rev-label').value = '';
    await renderPart(currentPart.id);
  });
});

// ------------------------------------------------------------------ catalog

let categories = [];
let categoriesLoaded = false;

async function loadCategories() {
  categories = await api('GET', '/api/categories');
  categoriesLoaded = true;
  return categories;
}

const indent = (depth) => '\u00a0\u00a0\u00a0'.repeat(depth);

// Fill a <select> with the tree, indented. `blank` adds a first option with
// value '' and that text; `exclude` drops those ids.
function categoryOptions(select, { blank, exclude } = {}) {
  const skip = exclude || new Set();
  select.innerHTML = (blank !== undefined ? `<option value="">${escape(blank)}</option>` : '')
    + categories
      .filter((c) => !skip.has(c.id))
      .map((c) => `<option value="${escape(c.id)}">${indent(c.depth)}${escape(c.name)}</option>`)
      .join('');
}

function fillCategoryFilter() {
  const select = $('filter-category');
  const current = select.value;
  categoryOptions(select, { blank: 'All categories' });
  select.insertAdjacentHTML('beforeend', '<option value="_none">Uncategorized</option>');
  select.value = [...select.options].some((o) => o.value === current) ? current : '';
}

// A category and every category beneath it, by id.
function subtree(id) {
  const out = new Set([id]);
  let grew = true;
  while (grew) {
    grew = false;
    for (const c of categories) {
      if (c.parent && out.has(c.parent) && !out.has(c.id)) { out.add(c.id); grew = true; }
    }
  }
  return out;
}

// How a part's category reads in a list: its path, or the old free text that
// names no category, marked as such.
function categoryText(part) {
  if (part.category_path) return escape(part.category_path);
  if (part.category) return `<span class="muted" title="Not a catalog category">${escape(part.category)} ?</span>`;
  return '<span class="muted">—</span>';
}

const tagChips = (tags) => tags.map((t) => `<button class="tag" data-tag="${escape(t)}" title="Show parts tagged ${escape(t)}">${escape(t)}</button>`).join('');
const splitTags = (text) => text.split(',').map((t) => t.trim()).filter(Boolean);

function typeText(def) {
  switch (def.type) {
    case 'number': return def.unit ? `number (${escape(def.unit)})` : 'number';
    case 'bool': return 'yes / no';
    case 'enum': return `one of ${def.values.map(escape).join(', ')}`;
    default: return 'text';
  }
}

function valueText(def, value) {
  if (value === undefined || value === null || value === '') return '';
  if (def && def.type === 'bool') return value ? 'yes' : 'no';
  if (def && def.type === 'number' && def.unit) return `${escape(value)} ${escape(def.unit)}`;
  return escape(value);
}

// The input for one attribute of a schema, holding `value`.
function attrInput(def, value) {
  const key = escape(def.key);
  const v = value === undefined || value === null ? '' : value;
  const req = def.required ? ' <span class="req" title="Needed before a release">*</span>' : '';
  let control;
  if (def.type === 'number') {
    control = `<span class="unit"><input type="number" step="any" data-attr="${key}" value="${escape(v)}">${def.unit ? `<span>${escape(def.unit)}</span>` : ''}</span>`;
  } else if (def.type === 'bool') {
    control = `<select data-attr="${key}">
      <option value="">—</option>
      <option value="true" ${v === true ? 'selected' : ''}>yes</option>
      <option value="false" ${v === false ? 'selected' : ''}>no</option></select>`;
  } else if (def.type === 'enum') {
    control = `<select data-attr="${key}"><option value="">—</option>${def.values
      .map((option) => `<option ${option === v ? 'selected' : ''}>${escape(option)}</option>`).join('')}</select>`;
  } else {
    control = `<input data-attr="${key}" value="${escape(v)}">`;
  }
  return `<label><span>${escape(def.name)}${req}</span>${control}</label>`;
}

// Draw the attribute inputs for `categoryId`'s schema into `container`.
async function syncAttrForm(categoryId, container, values) {
  if (!categoryId || !categories.some((c) => c.id === categoryId)) {
    container.innerHTML = '';
    return [];
  }
  const schema = await api('GET', `/api/categories/${encodeURIComponent(categoryId)}/schema`);
  container.innerHTML = schema.attributes.map((def) => attrInput(def, values[def.key])).join('');
  return schema.attributes;
}

// The form's values: `{ key: text }`. An empty field is `null` when
// `clearEmpty` (an edit clears it) and left out otherwise (a new part).
function readAttrForm(container, clearEmpty = false) {
  const out = {};
  for (const el of container.querySelectorAll('[data-attr]')) {
    const value = el.value.trim();
    if (value !== '') out[el.dataset.attr] = value;
    else if (clearEmpty) out[el.dataset.attr] = null;
  }
  return out;
}

// -- the part's catalog panel and its edit dialog

function renderDetails(detail) {
  $('part-locked').hidden = !detail.catalog_locked;
  $('part-category').innerHTML = detail.category_path
    ? `Category: <a href="#/catalog/${encodeURIComponent(detail.category)}">${escape(detail.category_path)}</a>`
    : detail.category
      ? `Category text <strong>${escape(detail.category)}</strong> is not a catalog category, so the part is uncategorized.`
      : 'Uncategorized.';
  $('part-tags').innerHTML = detail.tags.length ? tagChips(detail.tags) : '';
  const rows = detail.schema.map((def) => {
    const value = detail.attributes[def.key];
    const shown = valueText(def, value);
    const cell = shown
      || (def.required ? '<span class="missing">missing — needed before release</span>' : '<span class="muted">—</span>');
    return `<dt>${escape(def.name)}${def.required ? ' <span class="req">*</span>' : ''}</dt><dd>${cell}</dd>`;
  });
  for (const key of detail.inert) {
    rows.push(`<dt class="muted">${escape(key)}</dt><dd><span class="inert-value">${escape(detail.attributes[key])}</span>
      <span class="muted">not an attribute of this category; kept, no effect</span></dd>`);
  }
  $('part-attrs').innerHTML = rows.join('');
  $('part-attrs').hidden = rows.length === 0;
}

$('edit-part').addEventListener('click', () => act(async () => {
  const part = currentPart;
  await loadCategories();
  $('pe-number').textContent = part.number;
  $('pe-name').value = part.name;
  $('pe-description').value = part.description;
  $('pe-tags').value = part.tags.join(', ');
  categoryOptions($('pe-category'), { blank: '— none —' });
  // Old free text that names no category stays selectable, so saving the
  // other fields does not silently change it.
  if (part.category && !categories.some((c) => c.id === part.category)) {
    $('pe-category').insertAdjacentHTML('beforeend',
      `<option value="${escape(part.category)}">${escape(part.category)} (not in the catalog)</option>`);
  }
  $('pe-category').value = part.category;
  $('pe-attrs').innerHTML = ''; // not the last part's inputs
  await syncEditAttrs();
  $('part-dialog').showModal();
}));

// The edit dialog's attribute inputs, and the values the chosen category
// does not define — which can be cleared, not changed.
async function syncEditAttrs() {
  const part = currentPart;
  const typed = $('pe-attrs').children.length ? readAttrForm($('pe-attrs')) : {};
  const values = { ...part.attributes, ...typed };
  const schema = await syncAttrForm($('pe-category').value, $('pe-attrs'), values);
  const keys = new Set(schema.map((d) => d.key));
  const inert = Object.keys(part.attributes).filter((k) => !keys.has(k));
  $('pe-inert').hidden = inert.length === 0;
  $('pe-inert').innerHTML = inert.length
    ? `<p class="muted">Kept from another category; they have no effect here:</p>${inert.map((k) => `
      <label class="inline"><input type="checkbox" data-clear="${escape(k)}"><span>clear <strong>${escape(k)}</strong> = ${escape(part.attributes[k])}</span></label>`).join('')}`
    : '';
}

$('pe-category').addEventListener('change', () => act(syncEditAttrs));

$('part-edit-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    const attributes = readAttrForm($('pe-attrs'), true);
    for (const box of $('pe-inert').querySelectorAll('input[data-clear]:checked')) attributes[box.dataset.clear] = null;
    await api('PATCH', `/api/parts/${encodeURIComponent(currentPart.id)}`, {
      name: $('pe-name').value,
      description: $('pe-description').value,
      category: $('pe-category').value,
      tags: splitTags($('pe-tags').value),
      attributes,
    });
    $('part-dialog').close();
    await renderPart(currentPart.id);
  });
});

// -- the catalog view

let selectedCategory = null;

async function renderCatalog(id) {
  await loadCategories();
  fillCategoryFilter();
  selectedCategory = categories.find((c) => c.id === id) || null;
  $('category-tree').innerHTML = categories.length
    ? categories.map((c) => `
      <li><button class="${selectedCategory && c.id === selectedCategory.id ? 'current' : ''}" data-cat="${escape(c.id)}"
        data-depth="${c.depth}">${escape(c.name)}<span class="count" title="${c.parts} here, ${c.parts_within} including sub-categories">${c.parts_within}</span></button></li>`).join('')
    : '<li class="empty">No categories yet.</li>';
  for (const button of $('category-tree').querySelectorAll('button[data-cat]')) {
    // Set through the CSSOM, not an inline style attribute: the page's CSP allows
    // no inline style.
    button.style.paddingLeft = `${8 + Number(button.dataset.depth) * 16}px`;
    button.addEventListener('click', () => { location.hash = `#/catalog/${encodeURIComponent(button.dataset.cat)}`; });
  }
  categoryOptions($('cn-parent'), { blank: '(top level)' });
  if (selectedCategory) $('cn-parent').value = selectedCategory.id;

  $('category-none').hidden = !!selectedCategory;
  $('category-body').hidden = !selectedCategory;
  if (!selectedCategory) return;
  const c = selectedCategory;
  const schema = await api('GET', `/api/categories/${encodeURIComponent(c.id)}/schema`);
  $('cat-title').textContent = c.path;
  $('cat-parts').href = `#/parts/${encodeURIComponent(c.id)}`;
  $('cat-parts').textContent = `Show parts (${c.parts_within})`;
  $('cat-meta').innerHTML = `<span class="mono">${escape(c.id)}</span> · ${c.parts} part${c.parts === 1 ? '' : 's'} filed here, ${c.parts_within} including sub-categories`;

  const inherited = schema.attributes.filter((a) => a.from !== c.id);
  const nameOf = (id) => (categories.find((x) => x.id === id) || { name: id }).name;
  $('cat-inherited-none').hidden = inherited.length > 0;
  $('cat-inherited-table').hidden = inherited.length === 0;
  $('cat-inherited').innerHTML = inherited.map((a) => `
    <tr><td class="mono">${escape(a.key)}</td><td>${escape(a.name)}</td><td>${typeText(a)}</td>
    <td>${a.required ? 'yes' : ''}</td><td><a href="#/catalog/${encodeURIComponent(a.from)}">${escape(nameOf(a.from))}</a></td></tr>`).join('');

  $('cat-name').value = c.name;
  categoryOptions($('cat-parent'), { blank: '(top level)', exclude: subtree(c.id) });
  $('cat-parent').value = c.parent || '';
  $('cat-own').innerHTML = '';
  for (const def of c.attributes) addAttrRow(def);
  syncOwnEmpty();
  for (const el of $('cat-form').querySelectorAll('input, select, button')) el.disabled = !me.is_admin;
}

function syncOwnEmpty() {
  $('cat-own-none').hidden = $('cat-own').children.length > 0;
}

// One editable row of the category's own attributes.
function addAttrRow(def = { key: '', name: '', type: 'text', required: false }) {
  const row = document.createElement('tr');
  const extra = def.type === 'number' ? (def.unit || '') : def.type === 'enum' ? def.values.join(', ') : '';
  row.innerHTML = `
    <td><input class="mono" data-f="key" value="${escape(def.key)}" placeholder="length" aria-label="Key"></td>
    <td><input data-f="name" value="${escape(def.name)}" placeholder="Length" aria-label="Name"></td>
    <td><select data-f="type" aria-label="Type">
      ${['text', 'number', 'bool', 'enum'].map((t) => `<option value="${t}" ${t === def.type ? 'selected' : ''}>${{ text: 'text', number: 'number', bool: 'yes / no', enum: 'list' }[t]}</option>`).join('')}
    </select></td>
    <td><input data-f="extra" value="${escape(extra)}" aria-label="Unit or values"></td>
    <td><input type="checkbox" data-f="required" ${def.required ? 'checked' : ''} aria-label="Required"></td>
    <td class="actions"><button type="button" class="link danger" data-f="remove">Remove</button></td>`;
  const sync = () => {
    const type = row.querySelector('[data-f=type]').value;
    const extraInput = row.querySelector('[data-f=extra]');
    extraInput.disabled = !me.is_admin || (type !== 'number' && type !== 'enum');
    extraInput.placeholder = type === 'number' ? 'unit, e.g. mm' : type === 'enum' ? 'M3, M4, M5' : '';
    if (type !== 'number' && type !== 'enum') extraInput.value = '';
  };
  row.querySelector('[data-f=type]').addEventListener('change', sync);
  row.querySelector('[data-f=remove]').addEventListener('click', () => { row.remove(); syncOwnEmpty(); });
  $('cat-own').appendChild(row);
  sync();
  syncOwnEmpty();
}

function readAttrRows() {
  return [...$('cat-own').children].map((row) => {
    const f = (name) => row.querySelector(`[data-f=${name}]`);
    const type = f('type').value;
    const def = { key: f('key').value.trim(), name: f('name').value.trim(), type, required: f('required').checked };
    if (type === 'number') def.unit = f('extra').value.trim();
    if (type === 'enum') def.values = splitTags(f('extra').value);
    return def;
  });
}

$('cat-add-attr').addEventListener('click', () => addAttrRow());

$('cat-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const id = selectedCategory.id;
    await api('PATCH', `/api/categories/${encodeURIComponent(id)}`, {
      name: $('cat-name').value,
      parent: $('cat-parent').value,
      attributes: readAttrRows(),
    });
    await renderCatalog(id);
  });
});

$('cat-delete').addEventListener('click', () => act(async () => {
  if (!confirm(`Delete the category ${selectedCategory.path}?`)) return;
  await api('DELETE', `/api/categories/${encodeURIComponent(selectedCategory.id)}`);
  location.hash = '#/catalog';
}));

$('category-new').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const created = await api('POST', '/api/categories', {
      id: $('cn-id').value,
      name: $('cn-name').value,
      parent: $('cn-parent').value,
    });
    $('cn-id').value = '';
    $('cn-name').value = '';
    location.hash = `#/catalog/${encodeURIComponent(created.id)}`;
  });
});

// -------------------------------------------------------------- part types

async function renderTypes() {
  partTypes = await api('GET', '/api/part-types');
  $('type-rows').innerHTML = partTypes.map((t) => `
    <tr>
      <td class="mono">${escape(t.id)}</td>
      <td>${escape(t.name)}</td>
      <td>${t.mode.kind === 'counter'
        ? `counter <span class="mono">${escape(t.prefix)}</span> + ${t.digits} digits`
        : modeText(t.mode)}</td>
      <td class="mono">${t.mode.kind === 'counter' ? escape(t.next_number) : '<span class="muted">typed or scripted</span>'}</td>
      <td class="actions">${me.is_admin ? `<button class="link" data-type="${escape(t.id)}">Edit</button>` : ''}</td>
    </tr>`).join('');
  for (const button of $('type-rows').querySelectorAll('button[data-type]')) {
    button.addEventListener('click', () => openTypeEditor(button.dataset.type));
  }
}

function previewNumber() {
  const prefix = $('type-prefix').value;
  const digits = Math.min(Math.max(parseInt($('type-digits').value, 10) || 1, 1), 18);
  const start = Math.max(parseInt($('type-start').value, 10) || 1, 1);
  $('type-preview').textContent = prefix + String(start).padStart(digits, '0');
}
for (const id of ['type-prefix', 'type-digits', 'type-start']) {
  $(id).addEventListener('input', previewNumber);
}
previewNumber();

function syncModeFields() {
  const kind = $('type-mode').value;
  for (const el of document.querySelectorAll('#type-form .mode-fields')) el.hidden = el.dataset.mode !== kind;
}
$('type-mode').addEventListener('change', syncModeFields);
syncModeFields();

// The mode object the API takes, from a select and its two fields.
function modeFrom(kind, regex, script) {
  if (kind === 'pattern') return { kind, regex };
  if (kind === 'script') return { kind, script: script || 'part-number.js' };
  return { kind };
}

$('type-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const kind = $('type-mode').value;
    await api('POST', '/api/part-types', {
      id: $('type-id').value,
      name: $('type-name').value,
      prefix: $('type-prefix').value,
      digits: parseInt($('type-digits').value, 10) || 9,
      start: parseInt($('type-start').value, 10) || 1,
      mode: modeFrom(kind, $('type-regex').value, $('type-script').value),
    });
    for (const id of ['type-id', 'type-name', 'type-prefix', 'type-regex', 'type-script']) $(id).value = '';
    await renderTypes();
  });
});

let editingType = null;

function syncEditFields() {
  for (const el of document.querySelectorAll('#type-edit-form .te-field')) el.hidden = el.dataset.mode !== $('te-mode').value;
}
$('te-mode').addEventListener('change', syncEditFields);

function openTypeEditor(id) {
  editingType = partTypes.find((t) => t.id === id);
  $('te-id').textContent = editingType.id;
  $('te-name').value = editingType.name;
  $('te-mode').value = editingType.mode.kind;
  $('te-regex').value = editingType.mode.regex || '';
  $('te-script').value = editingType.mode.script || '';
  syncEditFields();
  $('type-dialog').showModal();
}

$('type-edit-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    await api('PATCH', `/api/part-types/${encodeURIComponent(editingType.id)}`, {
      name: $('te-name').value,
      mode: modeFrom($('te-mode').value, $('te-regex').value, $('te-script').value),
    });
    $('type-dialog').close();
    await renderTypes();
  });
});

// ------------------------------------------------------------------- users

async function renderUsers() {
  const users = await api('GET', '/api/users');
  $('user-rows').innerHTML = users.map((u) => `
    <tr>
      <td class="mono">${escape(u.username)}</td>
      <td>${escape(u.display_name)}</td>
      <td class="muted">${escape(u.email)}</td>
      <td>${escape(u.groups.join(', '))}</td>
      <td>${u.active ? 'yes' : '<span class="muted">disabled</span>'}</td>
      <td class="actions">
        <button class="link" data-user="${escape(u.id)}" data-do="groups">Groups</button>
        <button class="link" data-user="${escape(u.id)}" data-do="password">Password</button>
        <button class="link" data-user="${escape(u.id)}" data-do="toggle">${u.active ? 'Disable' : 'Enable'}</button>
        <button class="link" data-user="${escape(u.id)}" data-do="sign-out" title="End every browser session of this user">Sign out</button>
      </td>
    </tr>`).join('');

  for (const button of $('user-rows').querySelectorAll('button[data-user]')) {
    button.addEventListener('click', () => onUserAction(button.dataset.do, button.dataset.user, users));
  }
}

function onUserAction(action, id, users) {
  const user = users.find((u) => u.id === id);
  act(async () => {
    if (action === 'groups') {
      const entered = prompt(`Groups for ${user.username} (comma separated)`, user.groups.join(', '));
      if (entered === null) return;
      await api('PATCH', `/api/users/${id}`, {
        groups: entered.split(',').map((g) => g.trim()).filter(Boolean),
      });
    } else if (action === 'password') {
      const entered = prompt(`New password for ${user.username} (at least 8 characters)`);
      if (!entered) return;
      await api('PATCH', `/api/users/${id}`, { password: entered });
    } else if (action === 'toggle') {
      await api('PATCH', `/api/users/${id}`, { active: !user.active });
    } else if (action === 'sign-out') {
      if (!confirm(`Sign ${user.username} out of every browser?`)) return;
      const done = await api('POST', `/api/users/${id}/sign-out`);
      banner('');
      alert(`${user.username}: ${done.sessions_ended} session${done.sessions_ended === 1 ? '' : 's'} ended.`);
    }
    await renderUsers();
  });
}

$('user-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const groups = [...$('user-form').querySelectorAll('input[type=checkbox]:checked')].map((c) => c.value);
    await api('POST', '/api/users', {
      username: $('user-name').value,
      display_name: $('user-display').value,
      email: $('user-email').value,
      password: $('user-pass').value,
      groups,
    });
    $('user-name').value = '';
    $('user-display').value = '';
    $('user-email').value = '';
    $('user-pass').value = '';
    await renderUsers();
  });
});

// ------------------------------------------------------------------ sourcing

let companies = { manufacturers: [], suppliers: [] };
let companiesLoaded = false;

async function loadCompanies() {
  const [manufacturers, suppliers] = await Promise.all([
    api('GET', '/api/manufacturers'),
    api('GET', '/api/suppliers'),
  ]);
  companies = { manufacturers, suppliers };
  companiesLoaded = true;
  // Whoever loads the lists keeps the parts filter current with them.
  fillCompanyFilter();
}

function fillCompanyFilter() {
  const select = $('filter-company');
  const keep = select.value;
  const group = (label, prefix, list) => list.length
    ? `<optgroup label="${label}">${list.map((c) => `<option value="${prefix}:${escape(c.id)}">${escape(c.name)}</option>`).join('')}</optgroup>`
    : '';
  select.innerHTML = '<option value="">Any maker or supplier</option>'
    + group('Manufacturers', 'm', companies.manufacturers)
    + group('Suppliers', 's', companies.suppliers);
  select.value = [...select.options].some((o) => o.value === keep) ? keep : '';
}

const STATUS_NAMES = { active: 'active', nrnd: 'NRND', obsolete: 'obsolete' };
const link = (url, text) => (url
  ? `<a href="${escape(url)}" target="_blank" rel="noopener noreferrer">${escape(text || url)}</a>`
  : escape(text || ''));
const price = (n) => Number(n).toLocaleString(undefined, { minimumFractionDigits: 2, maximumFractionDigits: 6 });

function breaksText(offer) {
  if (!offer.price_breaks.length) return '<span class="muted">no price</span>';
  const currency = offer.currency ? ` ${escape(offer.currency)}` : '';
  return offer.price_breaks
    .map((b) => `<span class="break">${b.qty.toLocaleString()}+ <strong>${price(b.unit_price)}</strong></span>`)
    .join(' ') + currency;
}

function renderSourcing(detail) {
  const list = detail.sourcing_view;
  $('sourcing-empty').hidden = list.length > 0;
  const author = me.can_author;
  $('sourcing-list').innerHTML = list.map((mp) => {
    const offers = mp.offers.map((o) => `
      <tr>
        <td>${escape(o.supplier_name)}</td>
        <td class="mono">${link(o.url, o.spn || (o.url ? 'page' : '—'))}</td>
        <td>${breaksText(o)}</td>
        <td>${o.lead_time_days == null ? '—' : `${o.lead_time_days} d`}</td>
        <td>${o.moq == null ? '—' : o.moq.toLocaleString()}</td>
        <td>${o.stock == null ? '—' : o.stock.toLocaleString()}</td>
        <td class="muted">${escape(when(o.updated_at))}</td>
        <td class="actions">${author ? `
          <button class="link" data-src="edit-offer" data-mp="${escape(mp.id)}" data-offer="${escape(o.id)}">Edit</button>
          <button class="link" data-src="delete-offer" data-mp="${escape(mp.id)}" data-offer="${escape(o.id)}">Delete</button>` : ''}</td>
      </tr>`).join('');
    return `
      <div class="mp${mp.preferred ? ' preferred' : ''}">
        <div class="toolbar">
          <strong>${escape(mp.manufacturer_name)}</strong>
          <span class="mono">${escape(mp.mpn)}</span>
          ${mp.preferred ? '<span class="badge badge-preferred">preferred</span>' : ''}
          <span class="badge badge-${escape(mp.status)}">${escape(STATUS_NAMES[mp.status] || mp.status)}</span>
          ${mp.datasheet ? link(mp.datasheet, 'datasheet') : ''}
          <span class="spacer"></span>
          ${author ? `
            <button class="link" data-src="add-offer" data-mp="${escape(mp.id)}">Add offer</button>
            <button class="link" data-src="edit-mp" data-mp="${escape(mp.id)}">Edit</button>
            <button class="link" data-src="delete-mp" data-mp="${escape(mp.id)}">Delete</button>` : ''}
        </div>
        ${mp.notes ? `<p class="muted">${escape(mp.notes)}</p>` : ''}
        ${mp.offers.length ? `
          <div class="table-wrap">
            <table class="offers">
              <thead><tr><th>Supplier</th><th>SPN</th><th>Price each</th><th>Lead</th><th>MOQ</th><th>Stock</th><th>Updated</th><th></th></tr></thead>
              <tbody>${offers}</tbody>
            </table>
          </div>` : '<p class="muted">No supplier offers.</p>'}
      </div>`;
  }).join('');
}

const partBase = () => `/api/parts/${encodeURIComponent(currentPart.id)}/sourcing`;
const findMp = (id) => currentPart.sourcing.find((m) => m.id === id);

function companyOptions(select, list, value) {
  select.innerHTML = list.map((c) => `<option value="${escape(c.id)}">${escape(c.name)}</option>`).join('');
  if (value) select.value = value;
}

let editingMp = null;

async function openMpDialog(mp) {
  await loadCompanies();
  editingMp = mp ? mp.id : null;
  $('mp-title').textContent = mp ? `Edit ${mp.mpn}` : 'Add manufacturer part';
  companyOptions($('mp-manufacturer'), companies.manufacturers, mp && mp.manufacturer);
  $('mp-no-makers').hidden = companies.manufacturers.length > 0;
  $('mp-mpn').value = mp ? mp.mpn : '';
  $('mp-status').value = mp ? mp.status : 'active';
  $('mp-preferred').checked = mp ? mp.preferred : currentPart.sourcing.length === 0;
  $('mp-datasheet').value = mp ? mp.datasheet : '';
  $('mp-notes').value = mp ? mp.notes : '';
  $('mp-dialog').showModal();
}

$('add-mp').addEventListener('click', () => act(() => openMpDialog(null)));

$('mp-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    const body = {
      manufacturer: $('mp-manufacturer').value,
      mpn: $('mp-mpn').value,
      status: $('mp-status').value,
      preferred: $('mp-preferred').checked,
      datasheet: $('mp-datasheet').value,
      notes: $('mp-notes').value,
    };
    if (editingMp) await api('PATCH', `${partBase()}/${encodeURIComponent(editingMp)}`, body);
    else await api('POST', partBase(), body);
    $('mp-dialog').close();
    await renderPart(currentPart.id);
  });
});

let editingOffer = null;

function addBreakRow(qty = '', unitPrice = '') {
  const row = document.createElement('div');
  row.className = 'break-row';
  row.innerHTML = `
    <label>Quantity <input type="number" min="1" step="1" data-f="qty" value="${escape(qty)}"></label>
    <label>Unit price <input type="number" min="0" step="any" data-f="price" value="${escape(unitPrice)}"></label>
    <button type="button" class="link" data-f="remove" aria-label="Remove this price break">Remove</button>`;
  row.querySelector('[data-f=remove]').addEventListener('click', () => row.remove());
  $('of-breaks').appendChild(row);
}

$('of-add-break').addEventListener('click', () => addBreakRow());

async function openOfferDialog(mpId, offer) {
  await loadCompanies();
  editingOffer = { mp: mpId, id: offer ? offer.id : null };
  const mp = findMp(mpId);
  $('of-title').textContent = offer ? `Edit offer for ${mp.mpn}` : `Add an offer for ${mp.mpn}`;
  companyOptions($('of-supplier'), companies.suppliers, offer && offer.supplier);
  $('of-no-suppliers').hidden = companies.suppliers.length > 0;
  $('of-spn').value = offer ? offer.spn : '';
  $('of-url').value = offer ? offer.url : '';
  $('of-currency').value = offer ? offer.currency : 'USD';
  $('of-breaks').innerHTML = '';
  for (const b of (offer ? offer.price_breaks : [])) addBreakRow(b.qty, b.unit_price);
  if (!offer || !offer.price_breaks.length) addBreakRow(1, '');
  $('of-lead').value = offer && offer.lead_time_days != null ? offer.lead_time_days : '';
  $('of-moq').value = offer && offer.moq != null ? offer.moq : '';
  $('of-stock').value = offer && offer.stock != null ? offer.stock : '';
  $('of-notes').value = offer ? offer.notes : '';
  $('offer-dialog').showModal();
}

$('offer-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    // A row with no price is a row nobody filled in, not a free part.
    const price_breaks = [...$('of-breaks').querySelectorAll('.break-row')]
      .map((row) => ({ qty: row.querySelector('[data-f=qty]').value, unit_price: row.querySelector('[data-f=price]').value }))
      .filter((b) => String(b.unit_price).trim() !== '');
    const body = {
      supplier: $('of-supplier').value,
      spn: $('of-spn').value,
      url: $('of-url').value,
      currency: $('of-currency').value,
      price_breaks,
      lead_time_days: $('of-lead').value,
      moq: $('of-moq').value,
      stock: $('of-stock').value,
      notes: $('of-notes').value,
    };
    const base = `${partBase()}/${encodeURIComponent(editingOffer.mp)}/offers`;
    if (editingOffer.id) await api('PATCH', `${base}/${encodeURIComponent(editingOffer.id)}`, body);
    else await api('POST', base, body);
    $('offer-dialog').close();
    await renderPart(currentPart.id);
  });
});

$('sourcing-list').addEventListener('click', (event) => {
  const button = event.target.closest('button[data-src]');
  if (!button) return;
  const { src, mp, offer } = button.dataset;
  act(async () => {
    const found = findMp(mp);
    switch (src) {
      case 'edit-mp':
        return openMpDialog(found);
      case 'add-offer':
        return openOfferDialog(mp, null);
      case 'edit-offer':
        return openOfferDialog(mp, found.offers.find((o) => o.id === offer));
      case 'delete-mp':
        if (!confirm(`Remove ${found.mpn} and its ${found.offers.length} offer(s) from this part?`)) return;
        await api('DELETE', `${partBase()}/${encodeURIComponent(mp)}`);
        break;
      case 'delete-offer':
        if (!confirm('Remove this offer?')) return;
        await api('DELETE', `${partBase()}/${encodeURIComponent(mp)}/offers/${encodeURIComponent(offer)}`);
        break;
    }
    await renderPart(currentPart.id);
  });
});

// -- the manufacturers and suppliers page

function companyRows(list, key) {
  const prefix = key === 'manufacturers' ? 'm' : 's';
  return list.map((c) => `
    <tr>
      <td><strong>${escape(c.name)}</strong>${c.notes ? `<br><span class="muted">${escape(c.notes)}</span>` : ''}</td>
      <td>${c.website ? link(c.website, c.website.replace(/^https?:\/\//, '')) : '<span class="muted">—</span>'}</td>
      <td>${c.parts ? `<button class="link" data-show="${prefix}:${escape(c.id)}" title="Show the parts sourced from ${escape(c.name)}">${c.parts}</button>` : '0'}</td>
      <td class="actions">
        ${me.can_author ? `<button class="link" data-co="edit" data-list="${key}" data-id="${escape(c.id)}">Edit</button>` : ''}
        ${me.is_admin ? `<button class="link" data-co="delete" data-list="${key}" data-id="${escape(c.id)}"
          ${c.uses ? `disabled title="Named by ${c.uses} ${key === 'manufacturers' ? 'manufacturer part' : 'offer'}${c.uses === 1 ? '' : 's'}"` : ''}>Delete</button>` : ''}
      </td>
    </tr>`).join('');
}

async function renderCompanies() {
  await loadCompanies();
  $('manufacturer-rows').innerHTML = companyRows(companies.manufacturers, 'manufacturers');
  $('supplier-rows').innerHTML = companyRows(companies.suppliers, 'suppliers');
  $('manufacturers-empty').hidden = companies.manufacturers.length > 0;
  $('suppliers-empty').hidden = companies.suppliers.length > 0;
}

for (const form of document.querySelectorAll('form.company-add')) {
  form.addEventListener('submit', (event) => {
    event.preventDefault();
    act(async () => {
      await api('POST', `/api/${form.dataset.list}`, { name: form.elements.name.value });
      form.reset();
      await renderCompanies();
    });
  });
}

let editingCompany = null;

$('view-sourcing').addEventListener('click', (event) => {
  const show = event.target.closest('button[data-show]');
  if (show) {
    $('filter-company').value = show.dataset.show;
    location.hash = '#/parts';
    return;
  }
  const button = event.target.closest('button[data-co]');
  if (!button) return;
  const list = button.dataset.list;
  const company = companies[list].find((c) => c.id === button.dataset.id);
  act(async () => {
    if (button.dataset.co === 'edit') {
      editingCompany = { list, id: company.id };
      $('co-title').textContent = `Edit ${company.name}`;
      $('co-name').value = company.name;
      $('co-website').value = company.website;
      $('co-notes').value = company.notes;
      $('company-dialog').showModal();
    } else if (button.dataset.co === 'delete') {
      if (!confirm(`Delete ${company.name}?`)) return;
      await api('DELETE', `/api/${list}/${encodeURIComponent(company.id)}`);
      await renderCompanies();
    }
  });
});

$('company-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    await api('PATCH', `/api/${editingCompany.list}/${encodeURIComponent(editingCompany.id)}`, {
      name: $('co-name').value,
      website: $('co-website').value,
      notes: $('co-notes').value,
    });
    $('company-dialog').close();
    await renderCompanies();
  });
});

// ----------------------------------------------------------------- settings

async function renderSettings() {
  const settings = await api('GET', '/api/settings');
  $('set-multiple-drafts').checked = settings.allow_multiple_open_drafts;
  $('set-lock-attributes').checked = settings.lock_released_attributes;
  $('set-released-children').checked = settings.require_released_children;
  $('set-idle').value = settings.session_idle_minutes;
  $('set-max-hours').value = settings.session_max_hours;
  $('set-script-editor').checked = settings.script_editor_enabled;
  $('set-workspaces-browsable').checked = settings.workspaces_browsable;
  $('set-script-locked').hidden = !settings.script_editor_locked;
  await renderReviewSettings(settings);
  await renderEcoSettings(settings);
  $('settings-status').textContent = '';
}

$('settings-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    await api('PATCH', '/api/settings', {
      allow_multiple_open_drafts: $('set-multiple-drafts').checked,
      lock_released_attributes: $('set-lock-attributes').checked,
      require_released_children: $('set-released-children').checked,
      session_idle_minutes: Number($('set-idle').value),
      session_max_hours: Number($('set-max-hours').value),
      script_editor_enabled: $('set-script-editor').checked,
      workspaces_browsable: $('set-workspaces-browsable').checked,
      review_rule: readRule({
        required: $('set-rv-required'), due: $('set-rv-due'), reviewers: $('set-rv-reviewers'), self: $('set-rv-self'),
      }),
      review_overrides: readOverrides(),
      eco_review_rule: readRule({
        required: $('set-eco-required'), due: $('set-eco-due'), reviewers: $('set-eco-reviewers'), self: $('set-eco-self'),
      }),
      eco_holds_revisions: $('set-eco-holds').checked,
    });
    await api('PATCH', '/api/ecos/numbering', readEcoNumbering());
    await renderSettings();
    $('settings-status').textContent = 'Saved.';
  });
});

// ------------------------------------------------------------------ scripts

// Example inputs for the test run: the shape each hook really receives, so an
// admin can edit rather than guess.
const SAMPLE_INPUT = {
  partNumber: {
    requested: '',
    partType: { id: 'made', name: 'Made', mode: { kind: 'script', script: 'part-number.js' } },
    part: { name: 'Motor mount bracket', description: '', category: 'brackets', document_class: 'normal' },
    user: { username: 'admin', groups: ['admin'] },
  },
  revisionLabel: {
    requested: '', suggested: 'B', existing: ['A'],
    part: { number: 'CPART000000001', name: 'Motor mount bracket' },
    user: { username: 'admin', groups: ['admin'] },
  },
  beforeRelease: {
    part: { number: 'CPART000000001', name: 'Motor mount bracket', description: '' },
    revision: { label: 'A', lifecycle: 'draft' },
    user: { username: 'admin', groups: ['admin'] },
  },
  afterRelease: {
    part: { number: 'CPART000000001', name: 'Motor mount bracket' },
    revision: { label: 'A', lifecycle: 'released' },
    user: { username: 'admin', groups: ['admin'] },
  },
};

// A starting point for each hook file, so "New" opens something that runs.
const STARTER = {
  'revision-label.js': `// Decide the label of every new revision.
// Return the label, return null to accept what was typed (or the suggestion),
// or throw to refuse.
function revisionLabel(input) {
  return input.requested || input.suggested;
}
`,
  'before-release.js': `// Throw to refuse a release. The message is shown to the user.
function beforeRelease(input) {
  if (!input.part.description) throw new Error('A released part needs a description.');
}
`,
  'after-release.js': `// Runs after a release is committed. It cannot undo it; a throw is shown as a warning.
function afterRelease(input) {
  console.log('released', input.part.number, input.revision.label);
}
`,
};
const PART_NUMBER_STARTER = `// Validate or allocate the number of a part of a Script-mode part type.
// input.requested is what the user typed ('' if nothing). Return the number,
// or throw to refuse.
function partNumber(input) {
  if (input.requested) return input.requested;
  throw new Error('Type a number for this part type.');
}
`;

let scriptPath = null;
let scriptSaved = '';

function scriptDirty() {
  return scriptPath !== null && $('script-text').value !== scriptSaved;
}

// Whether the server lets this admin edit and test-run scripts here.
let scriptEditorOn = true;

function syncScriptButtons() {
  const open = scriptPath !== null;
  $('script-text').disabled = !open;
  $('script-text').readOnly = !scriptEditorOn;
  $('script-save').disabled = !scriptEditorOn || !open || !scriptDirty();
  $('script-delete').disabled = !scriptEditorOn || !open;
  $('run-go').disabled = !scriptEditorOn || !open;
  for (const el of $('script-new').elements) el.disabled = !scriptEditorOn;
  $('script-dirty').textContent = scriptDirty() ? '· unsaved' : '';
}

async function renderScripts() {
  const [listing, settings] = await Promise.all([api('GET', '/api/scripts'), api('GET', '/api/settings')]);
  scriptEditorOn = settings.script_editor_enabled && !settings.script_editor_locked;
  $('scripts-readonly').hidden = scriptEditorOn;
  $('scripts-readonly').textContent = settings.script_editor_locked
    ? 'Read-only: this server was started with --lock-script-editor. Edit the scripts directory on the host.'
    : 'Read-only: editing scripts in the browser is turned off in Settings. Edit the scripts directory on the host, or turn it back on.';
  $('scripts-dir').textContent = listing.dir;
  $('hook-rows').innerHTML = listing.hooks.map((h) => `
    <tr><td class="mono">${escape(h.file)}</td><td class="mono">${escape(h.function)}()</td><td>${escape(h.when)}</td></tr>`).join('');
  $('script-list').innerHTML = listing.files.length
    ? listing.files.map((f) => `
      <li><button class="${f.path === scriptPath ? 'current' : ''}" data-path="${escape(f.path)}" title="${f.size} bytes · ${escape(when(f.modified))}">${escape(f.path)}</button></li>`).join('')
    : '<li class="empty">No scripts yet.</li>';
  for (const button of $('script-list').querySelectorAll('button[data-path]')) {
    button.addEventListener('click', () => act(() => openScript(button.dataset.path)));
  }
  if (!$('run-input').value) setSampleInput();
  syncScriptButtons();
}

function setSampleInput() {
  $('run-input').value = JSON.stringify(SAMPLE_INPUT[$('run-function').value] || {}, null, 2);
}

// Pick the function a file most likely defines, from its name or its text.
function guessFunction(path, text) {
  for (const name of Object.keys(SAMPLE_INPUT)) {
    if (new RegExp(`function\\s+${name}\\b`).test(text)) return name;
  }
  if (path.includes('label')) return 'revisionLabel';
  if (path.includes('before')) return 'beforeRelease';
  if (path.includes('after')) return 'afterRelease';
  return 'partNumber';
}

async function openScript(path, starter) {
  if (scriptDirty() && !confirm(`Discard unsaved changes to ${scriptPath}?`)) return;
  let text = starter;
  if (text === undefined) {
    text = (await api('GET', `/api/scripts/file/${path}`)).text;
    scriptSaved = text;
  } else {
    scriptSaved = '';
  }
  scriptPath = path;
  $('script-path').textContent = path;
  $('script-text').value = text;
  const guess = guessFunction(path, text);
  if ($('run-function').value !== guess) {
    $('run-function').value = guess;
    setSampleInput();
  }
  $('run-output').hidden = true;
  $('run-status').textContent = '';
  await renderScripts();
}

$('script-text').addEventListener('input', syncScriptButtons);
$('run-function').addEventListener('change', setSampleInput);

$('script-new').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    let path = $('script-new-path').value.trim();
    if (!path) return;
    if (!path.endsWith('.js')) path += '.js';
    const base = path.split('/').pop();
    await openScript(path, STARTER[base] || PART_NUMBER_STARTER);
    $('script-new-path').value = '';
  });
});

$('script-save').addEventListener('click', () => act(async () => {
  await api('PUT', `/api/scripts/file/${scriptPath}`, $('script-text').value);
  scriptSaved = $('script-text').value;
  await renderScripts();
}));

$('script-delete').addEventListener('click', () => act(async () => {
  if (!confirm(`Delete ${scriptPath} from the scripts directory?`)) return;
  await api('DELETE', `/api/scripts/file/${scriptPath}`);
  scriptPath = null;
  scriptSaved = '';
  $('script-path').textContent = 'No file open';
  $('script-text').value = '';
  await renderScripts();
}));

$('run-go').addEventListener('click', () => act(async () => {
  let input;
  try {
    input = JSON.parse($('run-input').value || 'null');
  } catch (error) {
    throw new Error(`The input is not JSON: ${error.message}`);
  }
  $('run-status').textContent = 'running…';
  const result = await api('POST', '/api/scripts/run', {
    path: scriptPath,
    source: $('script-text').value,
    function: $('run-function').value,
    input,
  });
  $('run-status').textContent = `${result.ok ? 'returned' : 'refused'} in ${result.millis} ms`;
  const out = $('run-output');
  out.hidden = false;
  out.classList.toggle('bad', !result.ok);
  const lines = [];
  if (result.ok) lines.push(`returned: ${JSON.stringify(result.value, null, 2)}`);
  else lines.push(`refused: ${result.error}`);
  if (result.logs.length) lines.push('', 'console:', ...result.logs);
  out.textContent = lines.join('\n');
}));

window.addEventListener('beforeunload', (event) => {
  if (scriptDirty()) event.preventDefault();
});

// ---------------------------------------------------------------------- go



// ------------------------------------------------- families and templates

// Where a revision came from: the family row or the template that made it,
// its bake, and whether someone has edited it since.
function sourceText(rev) {
  const out = [];
  const link = (p) => `<a href="#/part/${encodeURIComponent(p.part_id)}">${escape(p.number)}</a> rev ${escape(p.revision_label)}`;
  if (rev.family) out.push(`family ${link(rev.family)}`);
  if (rev.template) out.push(`template ${link(rev.template)}`);
  if (rev.hand_edited) out.push('<span class="st st-stale">edited since</span>');
  if (rev.uses) out.push(`assembly: uses ${rev.uses} part${rev.uses === 1 ? '' : 's'}`);
  if (rev.bake && rev.bake !== 'done') {
    out.push(`<span class="st st-${escape(rev.bake)}" title="${escape(rev.bake_error)}">bake ${escape(rev.bake)}</span>`);
  }
  return out.join(' · ') || '<span class="muted">—</span>';
}

const STATUS_WORDS = {
  new: 'new', current: 'current', stale: 'stale', unstamped: 'not from family', blocked: 'blocked',
};

let familyRevision = '';

async function renderSeed(detail) {
  $('family-panel').hidden = detail.document_class !== 'family';
  $('template-panel').hidden = detail.document_class !== 'template';
  if (detail.document_class === 'family') await renderFamily(detail);
  if (detail.document_class === 'template') await renderTemplate(detail);
}

async function memberTypeOptions(select, current) {
  partTypes = await api('GET', '/api/part-types');
  select.innerHTML = partTypes
    .map((t) => `<option value="${escape(t.id)}">${escape(t.name)} (${escape(modeText(t.mode).replace(/<[^>]+>/g, ''))})</option>`)
    .join('');
  select.value = current;
}

async function renderFamily(detail) {
  if (familyRevision && !detail.revisions.some((r) => r.id === familyRevision)) familyRevision = '';
  const q = familyRevision ? `?revision=${encodeURIComponent(familyRevision)}` : '';
  const view = await api('GET', `/api/parts/${encodeURIComponent(detail.id)}/family${q}`);
  $('fam-revision').innerHTML = view.revisions
    .map((r) => `<option value="${escape(r.id)}">${escape(r.label)} (${escape(r.lifecycle)})</option>`)
    .join('');
  $('fam-revision').value = view.revision_id;
  await memberTypeOptions($('fam-member-type'), view.member_part_type);
  $('fam-member-type').disabled = !me.can_author;
  $('fam-member-note').textContent = view.member_part_type_mode === 'counter'
    ? 'A counter type allocates its own numbers, so rows naming new parts will fail. Choose a free-text, pattern or script type.'
    : '';
  $('fam-nodoc').hidden = view.has_document;
  $('fam-generate').disabled = !view.has_document || !view.rows.length;
  $('fam-import').disabled = !view.has_document;

  const columns = view.columns.map((c) => c.label || c.name);
  $('fam-head').innerHTML = `<tr><th>Part number</th><th>Rev</th><th>Description</th>${
    columns.map((c) => `<th class="mono">${escape(c)}</th>`).join('')}<th>Member</th><th>Status</th><th></th></tr>`;
  $('fam-empty').hidden = view.rows.length > 0;
  $('fam-rows').innerHTML = view.rows.map((row) => {
    const m = row.member;
    const member = m
      ? `<a href="#/part/${encodeURIComponent(m.part_id)}">${escape(m.number)}</a>`
        + (m.revision_label ? ` rev ${escape(m.revision_label)} <span class="state state-${escape(m.lifecycle)}">${escape(m.lifecycle)}</span>` : '')
        + (m.bake && m.bake !== 'done' ? ` <span class="st st-${escape(m.bake)}" title="${escape(m.bake_error)}">bake ${escape(m.bake)}</span>` : '')
      : '<span class="muted">not yet</span>';
    const cells = view.columns.map((c) => `<td class="mono">${escape(row.values[c.name] ?? '')}</td>`).join('');
    const one = me.can_author && view.has_document && row.status !== 'current' && row.status !== 'blocked'
      ? `<button class="link" data-one="${escape(row.part_number)}">Generate</button>` : '';
    return `<tr>
      <td class="mono">${escape(row.part_number)}</td>
      <td>${escape(row.revision) || '<span class="muted">newest</span>'}</td>
      <td>${escape(row.description)}</td>
      ${cells}
      <td>${member}</td>
      <td><span class="st st-${escape(row.status)}">${escape(STATUS_WORDS[row.status] || row.status)}</span></td>
      <td class="note-cell">${escape(row.note)} ${one}</td>
    </tr>`;
  }).join('');
  for (const button of $('fam-rows').querySelectorAll('button[data-one]')) {
    button.addEventListener('click', () => generateFamily([button.dataset.one]));
  }
  $('fam-orphans').hidden = !view.orphans.length;
  $('fam-orphan-list').innerHTML = view.orphans.map((o) =>
    `<li><a href="#/part/${encodeURIComponent(o.part_id)}">${escape(o.number)}</a> ${escape(o.name)}
      <span class="muted">rev ${escape(o.revision_label)}, generated from family rev ${escape(o.generated_from || '?')}</span></li>`).join('');
  currentFamily = view;
}

let currentFamily = null;

function showReport(report) {
  const box = $('fam-report');
  const failed = report.rows.filter((r) => r.status === 'failed');
  const baking = report.rows.filter((r) => r.bake).length;
  box.className = failed.length ? 'report bad' : 'report';
  box.innerHTML = `<strong>Generated from revision ${escape(report.family_revision)}:</strong>
    ${report.written} written, ${report.skipped} unchanged, ${report.failed} failed`
    + (baking ? ` · ${baking} waiting for a bake` : '')
    + (failed.length ? `<ul>${failed.map((r) => `<li><span class="mono">${escape(r.number)}</span>${
      r.revision ? ` rev ${escape(r.revision)}` : ''}: ${escape(r.reason)}</li>`).join('')}</ul>` : '');
  box.hidden = false;
}

function generateFamily(only) {
  act(async () => {
    const report = await api('POST', `/api/parts/${encodeURIComponent(currentPart.id)}/generate`, {
      family_revision: $('fam-revision').value,
      only: only || [],
    });
    await renderPart(currentPart.id);
    showReport(report);
  });
}

$('fam-generate').addEventListener('click', () => generateFamily());

$('fam-revision').addEventListener('change', () => act(async () => {
  familyRevision = $('fam-revision').value;
  $('fam-report').hidden = true;
  await renderFamily(currentPart);
}));

$('fam-member-type').addEventListener('change', () => act(async () => {
  await api('PATCH', `/api/parts/${encodeURIComponent(currentPart.id)}`, { member_part_type: $('fam-member-type').value });
  await renderPart(currentPart.id);
}));

$('fam-import').addEventListener('click', () => {
  const draft = currentFamily && currentFamily.revision_writeable;
  $('import-target').textContent = draft
    ? `Rows go into the table of revision ${currentFamily.revision_label}, which is ${currentFamily.revision_state}.`
    : `Revision ${currentFamily ? currentFamily.revision_label : ''} cannot be changed, so rows go into the newest family revision still in work, if there is one.`;
  $('import-dialog').showModal();
});

$('import-file').addEventListener('change', () => act(async () => {
  const file = $('import-file').files[0];
  if (file) $('import-csv').value = await file.text();
  $('import-file').value = '';
}));

$('import-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'import') return;
  event.preventDefault();
  act(async () => {
    const result = await api('POST', `/api/parts/${encodeURIComponent(currentPart.id)}/family/import`, {
      csv: $('import-csv').value,
      revision: currentFamily && currentFamily.revision_writeable ? currentFamily.revision_id : '',
      generate: $('import-generate').checked,
    });
    $('import-dialog').close();
    $('import-csv').value = '';
    familyRevision = result.import.revision_id;
    await renderPart(currentPart.id);
    await reloadEditor(`part/${currentPart.id}/rev/${result.import.revision_id}`);
    const i = result.import;
    const box = $('fam-report');
    if (result.generate) {
      showReport(result.generate);
      box.insertAdjacentHTML('afterbegin', `Imported ${i.added} new and ${i.updated} updated rows into revision ${escape(i.revision_label)}. `);
    } else {
      box.className = 'report';
      box.innerHTML = `Imported ${i.added} new and ${i.updated} updated rows into revision ${escape(i.revision_label)}`
        + (i.columns_added.length ? `, adding columns ${i.columns_added.map((c) => `<span class="mono">${escape(c)}</span>`).join(', ')}` : '')
        + '. Press Generate to write the members.';
      box.hidden = false;
    }
  });
});

let currentTemplate = null;

function syncSpinNumber() {
  const kind = partTypes.find((t) => t.id === $('spin-type').value);
  const mode = kind ? kind.mode : { kind: 'counter' };
  const input = $('spin-number');
  input.disabled = mode.kind === 'counter';
  input.required = mode.kind === 'free' || mode.kind === 'pattern';
  if (mode.kind === 'counter') input.value = '';
  input.placeholder = mode.kind === 'counter' ? (kind ? kind.next_number : '') : mode.kind === 'pattern' ? mode.regex : '';
  $('spin-number-note').textContent = mode.kind === 'counter'
    ? `The ${kind ? kind.name : ''} counter numbers it.` : mode.kind === 'script' ? 'The script decides; it sees what you type.' : '';
}

async function renderTemplate(detail) {
  const view = await api('GET', `/api/parts/${encodeURIComponent(detail.id)}/template`);
  currentTemplate = view;
  $('tpl-revision').textContent = `inputs from revision ${view.revision_label}`;
  $('tpl-nodoc').hidden = view.has_document;
  $('spin-form').hidden = !me.can_author || !view.has_document;
  await memberTypeOptions($('spin-type'), view.member_part_type);
  syncSpinNumber();
  $('spin-none').hidden = view.inputs.length > 0;
  $('spin-inputs').innerHTML = view.inputs.map((input) => {
    const label = escape(input.label || input.name);
    const limits = [input.min != null ? `min ${input.min}` : '', input.max != null ? `max ${input.max}` : ''].filter(Boolean).join(', ');
    const field = input.choices && input.choices.length
      ? `<select data-input="${escape(input.name)}">${input.choices.map((c) =>
        `<option${c.trim() === input.default ? ' selected' : ''}>${escape(c)}</option>`).join('')}</select>`
      : `<input class="mono" data-input="${escape(input.name)}" value="${escape(input.default)}">`;
    return `<label><span>${label} <span class="mono muted">${escape(input.name)}</span>${limits ? ` <span class="muted">(${escape(limits)})</span>` : ''}</span>${field}</label>`;
  }).join('');
  $('tpl-copies-none').hidden = view.copies.length > 0;
  $('tpl-copies').innerHTML = view.copies.map((c) =>
    `<li><a href="#/part/${encodeURIComponent(c.part_id)}">${escape(c.number)}</a> ${escape(c.name)}
      <span class="muted mono">${escape(Object.entries(c.values).map(([k, v]) => `${k} = ${v}`).join(', '))}</span>
      ${c.bake && c.bake !== 'done' ? `<span class="st st-${escape(c.bake)}">bake ${escape(c.bake)}</span>` : ''}</li>`).join('');
}

$('spin-type').addEventListener('change', syncSpinNumber);

$('spin-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const values = {};
    for (const field of $('spin-inputs').querySelectorAll('[data-input]')) values[field.dataset.input] = field.value;
    const part = await api('POST', `/api/parts/${encodeURIComponent(currentPart.id)}/spin-out`, {
      name: $('spin-name').value,
      number: $('spin-number').value,
      part_type: $('spin-type').value,
      template_revision: currentTemplate ? currentTemplate.revision_id : '',
      values,
    });
    $('spin-name').value = '';
    $('spin-number').value = '';
    location.hash = `#/part/${part.id}`;
  });
});

// ----------------------------------------------------------- the bake queue

async function renderBake() {
  const filter = $('bake-filter').value;
  const jobs = await api('GET', `/api/bake/jobs${filter ? `?status=${encodeURIComponent(filter)}` : ''}`);
  $('bake-empty').hidden = jobs.length > 0;
  $('bake-empty').textContent = filter ? 'No jobs.' : 'Nothing waiting.';
  $('bake-rows').innerHTML = jobs.map((job) => {
    const worker = job.claimed_by
      ? `${escape(job.claimed_by)}${job.status === 'claimed' && job.lease_expires_at
        ? `<br><span class="muted">until ${escape(when(job.lease_expires_at))}</span>` : ''}`
      : '—';
    const retry = me.can_author && (job.status === 'failed' || job.status === 'claimed')
      ? `<button class="link" data-retry="${escape(job.id)}">${job.status === 'failed' ? 'Retry' : 'Take back'}</button>` : '';
    return `<tr>
      <td><a href="#/part/${encodeURIComponent(job.part_id)}">${escape(job.number)}</a><br><span class="muted">${escape(job.name)}</span></td>
      <td>${escape(job.revision_label)}</td>
      <td><span class="st st-${escape(job.status)}">${escape(job.status)}</span>${
        job.error ? `<br><span class="muted">${escape(job.error)}</span>` : ''}</td>
      <td>${escape(job.reason)}</td>
      <td>${escape(when(job.requested_at))}<br><span class="muted">${escape(job.requested_by)}</span></td>
      <td>${worker}${job.locked_by && job.status === 'pending'
        ? `<br><span class="muted">waits: checked out by ${escape(job.locked_by)}</span>` : ''}</td>
      <td>${job.attempts}</td>
      <td class="actions">${retry}</td>
    </tr>`;
  }).join('');
  for (const button of $('bake-rows').querySelectorAll('button[data-retry]')) {
    button.addEventListener('click', () => act(async () => {
      await api('POST', `/api/bake/jobs/${encodeURIComponent(button.dataset.retry)}/retry`);
      await renderBake();
    }));
  }
}

$('bake-filter').addEventListener('change', () => act(renderBake));
$('bake-refresh').addEventListener('click', () => act(renderBake));

window.addEventListener('hashchange', route);
boot();

// ---------------------------------------------------------------- structure
//
// A revision's uses list is the assembly structure; the BOM, the diff and
// where-used are all read from the server, which computes them. The page
// only edits the list, and only while the viewer holds the revision's lock —
// the server refuses otherwise, and the page says why before anyone tries.

const structure = { part: null, revision: '', tab: 'bom', lines: [], saved: '' };
const partUrl = () => `/api/parts/${encodeURIComponent(currentPart.id)}`;
const revUrl = (rev) => `${partUrl()}/revisions/${encodeURIComponent(rev)}`;
const qtyText = (n) => (Number.isInteger(n) ? String(n) : String(Number(n.toFixed(6))));
const money = (n) => Number(n).toLocaleString(undefined, { minimumFractionDigits: 2, maximumFractionDigits: 4 });
const stateChip = (state) => (state ? `<span class="state state-${escape(state)}">${escape(state)}</span>` : '—');

// The revision a structure view opens on: the current release, else the
// newest.
function defaultRevision(detail) {
  const views = detail.revision_views;
  const released = [...views].reverse().find((r) => r.lifecycle === 'released');
  return (released || views[views.length - 1]).id;
}

function revisionOptions(select, detail, value, { any = false } = {}) {
  const rows = [...detail.revision_views].reverse()
    .map((r) => `<option value="${escape(r.id)}">${escape(r.label)} · ${escape(r.lifecycle)}</option>`);
  select.innerHTML = (any ? '<option value="">any revision</option>' : '') + rows.join('');
  select.value = value;
}

async function renderStructure(detail) {
  const views = detail.revision_views;
  if (structure.part !== detail.id || !views.some((r) => r.id === structure.revision)) {
    // Another part opens on its BOM, not on whichever tab the last one was left.
    if (structure.part !== detail.id) structure.tab = 'bom';
    structure.part = detail.id;
    structure.revision = defaultRevision(detail);
    const newest = views[views.length - 1].id;
    const before = views.length > 1 ? views[views.length - 2].id : newest;
    revisionOptions($('diff-from'), detail, before);
    revisionOptions($('diff-to'), detail, newest);
    revisionOptions($('wu-revision'), detail, '', { any: true });
  } else {
    revisionOptions($('diff-from'), detail, $('diff-from').value || views[0].id);
    revisionOptions($('diff-to'), detail, $('diff-to').value || views[views.length - 1].id);
    revisionOptions($('wu-revision'), detail, $('wu-revision').value, { any: true });
  }
  revisionOptions($('bom-revision'), detail, structure.revision);
  showTab(structure.tab);
  await Promise.all([loadBom(), loadUses(), loadDiff(), loadWhereUsed()]);
}

function showTab(tab) {
  structure.tab = tab;
  for (const button of document.querySelectorAll('#bom-panel .tab')) {
    button.setAttribute('aria-selected', String(button.dataset.tab === tab));
  }
  for (const name of ['bom', 'uses', 'diff']) $(`tab-${name}`).hidden = name !== tab;
}

for (const button of document.querySelectorAll('#bom-panel .tab')) {
  button.addEventListener('click', () => showTab(button.dataset.tab));
}

$('bom-revision').addEventListener('change', () => act(async () => {
  structure.revision = $('bom-revision').value;
  await Promise.all([loadBom(), loadUses()]);
}));
$('bom-view').addEventListener('change', () => act(loadBom));
$('bom-levels').addEventListener('change', () => act(loadBom));

async function loadBom() {
  const flat = $('bom-view').value === 'flat';
  $('bom-levels-label').hidden = flat;
  const query = `levels=${encodeURIComponent($('bom-levels').value)}&flat=${flat}`;
  const base = `${revUrl(structure.revision)}/bom`;
  const bom = await api('GET', `${base}?${query}`);
  $('bom-csv').href = `${base}?${query}&format=csv`;
  const empty = bom.lines.length === 0;
  $('bom-empty').hidden = !empty;
  $('bom-wrap').hidden = empty;
  $('bom-csv').hidden = empty;
  $('bom-totals').hidden = empty;

  const warnings = bom.warnings;
  $('bom-warnings').hidden = warnings.length === 0;
  $('bom-warnings').innerHTML = warnings.length
    ? `<strong>${warnings.length} thing${warnings.length === 1 ? '' : 's'} to check before release</strong>
       <ul>${warnings.map((w) => `<li>${escape(w)}</li>`).join('')}</ul>`
    : '';

  $('bom-head').innerHTML = `<tr>${flat ? '' : '<th>Pos</th>'}<th>Find</th><th>Number</th><th>Name</th><th>Rev</th><th>State</th>
    ${flat ? '' : '<th class="num">Qty</th>'}<th class="num">Total</th><th>MPN</th><th>Supplier</th>
    <th class="num">Each</th><th class="num">Extended</th></tr>`;
  $('bom-rows').innerHTML = bom.lines.map((line) => {
    const unit = line.unit === 'each' ? '' : ` ${escape(line.unit)}`;
    const pos = `${'\u00a0\u00a0'.repeat(Math.max(0, line.level - 1))}${escape(line.position)}`;
    const rev = `${escape(line.revision_label || '?')}${line.floating ? ' <span class="floating" title="Follows the current release">floating</span>' : ''}`;
    const flags = line.flags.map((f, i) => `<span class="flag-chip" title="${escape(line.warnings[i])}">${escape(f)}</span>`).join('');
    const attrs = line.attributes.length ? `<span class="attrs-line">${escape(line.attributes.join(' · '))}</span>` : '';
    const each = line.unit_price == null ? '—' : `${escape(line.currency)} ${money(line.unit_price)}`;
    const extended = line.extended == null ? '—'
      : line.costed ? `${escape(line.currency)} ${money(line.extended)}`
        : `<span class="muted" title="An assembly: its parts are counted, not its own price">(${money(line.extended)})</span>`;
    return `<tr class="${line.assembly ? 'assembly' : ''}">
      ${flat ? '' : `<td class="pos">${pos}</td>`}
      <td>${escape(line.find_number)}</td>
      <td class="number"><a href="#/part/${encodeURIComponent(line.part_id)}">${escape(line.number)}</a></td>
      <td class="name">${escape(line.name)}${flags ? ` ${flags}` : ''}${attrs}</td>
      <td>${rev}</td>
      <td>${stateChip(line.state)}</td>
      ${flat ? '' : `<td class="num">${qtyText(line.quantity)}${unit}</td>`}
      <td class="num">${qtyText(line.total)}${unit}</td>
      <td class="mono">${escape(line.mpn) || '—'}</td>
      <td>${escape(line.supplier) || '—'}</td>
      <td class="num">${each}</td>
      <td class="num">${extended}</td>
    </tr>`;
  }).join('');
  const totals = bom.totals.map((t) => `${escape(t.currency || '(no currency)')} ${money(t.total)}`).join(' · ');
  $('bom-totals').innerHTML = `Total: ${totals || '—'}`
    + (bom.unpriced ? ` <span class="muted">· ${bom.unpriced} line${bom.unpriced === 1 ? '' : 's'} with no price</span>` : '');
}

// -- the uses list editor

function usesRevision() {
  return currentPart.revision_views.find((r) => r.id === structure.revision);
}

const usesEditable = () => {
  const rev = usesRevision();
  return Boolean(rev && rev.editable && rev.locked_by_me);
};

const usesBody = () => structure.lines.map((l) => ({
  part: l.part,
  revision: l.revision.trim(),
  quantity: String(l.quantity).trim(),
  unit: l.unit.trim(),
  find_number: l.find_number.trim(),
  reference: l.reference.trim(),
}));

async function loadUses() {
  const answer = await api('GET', `${revUrl(structure.revision)}/uses`);
  structure.lines = answer.uses.map((u) => ({
    part: u.part,
    number: u.number,
    name: u.name,
    revision: u.floating ? '' : u.revision_label,
    resolved: u.revision_label,
    state: u.state,
    quantity: qtyText(u.quantity),
    unit: u.unit,
    find_number: u.find_number,
    reference: u.reference,
  }));
  structure.saved = JSON.stringify(usesBody());
  $('uses-status').textContent = '';
  $('uses-search').value = '';
  $('uses-results').innerHTML = '';
  renderUses();
}

function renderUses() {
  const rev = usesRevision();
  const editable = usesEditable();
  let note = '';
  if (rev && !rev.editable) {
    note = `Rev ${rev.label} is ${rev.lifecycle}: its uses list is frozen with it. Start a new revision to change it.`;
  } else if (rev && !rev.locked_by) {
    note = me.can_author ? `Check out rev ${rev.label} to edit its uses list.` : '';
  } else if (rev && !rev.locked_by_me) {
    note = `Rev ${rev.label} is checked out by ${rev.locked_by}.`;
  }
  $('uses-locked').hidden = !note;
  $('uses-locked').textContent = note;
  $('uses-add').hidden = !editable;
  $('uses-save').hidden = !editable;
  $('uses-revert').hidden = !editable;
  $('uses-empty').hidden = structure.lines.length > 0;
  const off = editable ? '' : ' disabled';
  $('uses-rows').innerHTML = structure.lines.map((l, i) => `
    <tr>
      <td><a href="#/part/${encodeURIComponent(l.part)}">${escape(l.number)}</a> <span class="muted">${escape(l.name)}</span></td>
      <td><input class="rev" data-i="${i}" data-f="revision" value="${escape(l.revision)}" placeholder="${escape(l.resolved ? `(${l.resolved})` : 'current')}"
        aria-label="Revision of ${escape(l.number)}, empty follows the current release"${off}></td>
      <td><input class="qty" data-i="${i}" data-f="quantity" value="${escape(l.quantity)}" inputmode="decimal" aria-label="Quantity of ${escape(l.number)}"${off}></td>
      <td><input class="unit" data-i="${i}" data-f="unit" value="${escape(l.unit)}" aria-label="Unit"${off}></td>
      <td><input class="find" data-i="${i}" data-f="find_number" value="${escape(l.find_number)}" aria-label="Find number"${off}></td>
      <td><input class="ref" data-i="${i}" data-f="reference" value="${escape(l.reference)}" aria-label="Reference designators"${off}></td>
      <td class="actions">${editable ? `<button type="button" class="link" data-remove="${i}">Remove</button>` : ''}</td>
    </tr>`).join('');
  syncUsesStatus();
}

function syncUsesStatus() {
  if (!usesEditable()) return;
  const changed = JSON.stringify(usesBody()) !== structure.saved;
  $('uses-save').disabled = !changed;
  $('uses-revert').disabled = !changed;
  $('uses-status').textContent = changed ? 'Unsaved changes.' : '';
}

$('uses-rows').addEventListener('input', (event) => {
  const input = event.target.closest('input[data-f]');
  if (!input) return;
  structure.lines[Number(input.dataset.i)][input.dataset.f] = input.value;
  syncUsesStatus();
});

$('uses-rows').addEventListener('click', (event) => {
  const button = event.target.closest('button[data-remove]');
  if (!button) return;
  structure.lines.splice(Number(button.dataset.remove), 1);
  renderUses();
});

let usesTimer = null;
$('uses-search').addEventListener('input', () => {
  clearTimeout(usesTimer);
  usesTimer = setTimeout(() => act(searchUses), 200);
});

async function searchUses() {
  const q = $('uses-search').value.trim();
  if (!q) { $('uses-results').innerHTML = ''; return; }
  const found = (await api('GET', `/api/parts?q=${encodeURIComponent(q)}&limit=9`)).parts
    .filter((p) => p.id !== currentPart.id)
    .slice(0, 8);
  $('uses-results').innerHTML = found.length
    ? found.map((p) => `<li><button type="button" data-add="${escape(p.id)}" data-number="${escape(p.number)}" data-name="${escape(p.name)}">
        ${thumb(p.thumbnail_url, 'xs')}<strong class="mono">${escape(p.number)}</strong> ${escape(p.name)}
        <span class="muted">· rev ${escape(p.latest_label)} ${escape(p.latest_state)}</span></button></li>`).join('')
    : '<li class="muted">No part matches.</li>';
}

$('uses-results').addEventListener('click', (event) => {
  const button = event.target.closest('button[data-add]');
  if (!button) return;
  structure.lines.push({
    part: button.dataset.add,
    number: button.dataset.number,
    name: button.dataset.name,
    revision: '',
    resolved: '',
    state: '',
    quantity: '1',
    unit: 'each',
    find_number: String(structure.lines.length + 1),
    reference: '',
  });
  $('uses-search').value = '';
  $('uses-results').innerHTML = '';
  renderUses();
});

$('uses-save').addEventListener('click', () => act(async () => {
  await api('PUT', `${revUrl(structure.revision)}/uses`, { uses: usesBody() });
  await renderPart(currentPart.id);
  $('uses-status').textContent = 'Saved.';
}));

$('uses-revert').addEventListener('click', () => act(loadUses));

// -- comparing two revisions

$('diff-from').addEventListener('change', () => act(loadDiff));
$('diff-to').addEventListener('change', () => act(loadDiff));

async function loadDiff() {
  const from = $('diff-from').value;
  const to = $('diff-to').value;
  $('diff-wrap').hidden = !from || !to || from === to;
  if (!from || !to || from === to) {
    $('diff-rows').innerHTML = '';
    $('diff-summary').textContent = currentPart.revision_views.length < 2
      ? 'This part has one revision; there is nothing to compare yet.'
      : 'Pick two different revisions.';
    return;
  }
  const diff = await api('GET', `${partUrl()}/bom/diff?from=${encodeURIComponent(from)}&to=${encodeURIComponent(to)}`);
  $('diff-rows').innerHTML = diff.lines.map((l) => `
    <tr>
      <td class="number"><a href="#/part/${encodeURIComponent(l.part_id)}">${escape(l.number)}</a></td>
      <td>${escape(l.name)}</td>
      <td class="change-${escape(l.change)}">${escape(l.change)}</td>
      <td>${l.details.map(escape).join('<br>')}</td>
    </tr>`).join('');
  $('diff-wrap').hidden = diff.lines.length === 0;
  $('diff-summary').textContent = `From rev ${diff.from_revision} to rev ${diff.to_revision}: `
    + `${diff.lines.length} change${diff.lines.length === 1 ? '' : 's'}, ${diff.unchanged} part${diff.unchanged === 1 ? '' : 's'} unchanged.`;
}

// -- where-used

$('wu-revision').addEventListener('change', () => act(loadWhereUsed));
$('wu-levels').addEventListener('change', () => act(loadWhereUsed));

async function loadWhereUsed() {
  const revision = $('wu-revision').value;
  const levels = $('wu-levels').value;
  const used = await api('GET', `${partUrl()}/where-used?revision=${encodeURIComponent(revision)}&levels=${encodeURIComponent(levels)}`);
  $('wu-empty').hidden = used.lines.length > 0;
  $('wu-empty').textContent = revision
    ? `Nothing uses rev ${used.revision_label} of this part.`
    : 'Nothing uses this part.';
  $('wu-wrap').hidden = used.lines.length === 0;
  $('wu-rows').innerHTML = used.lines.map((l) => `
    <tr class="${l.current ? '' : 'muted'}">
      <td>${'\u00a0\u00a0\u00a0'.repeat(l.level - 1)}<a href="#/part/${encodeURIComponent(l.part_id)}">${escape(l.number)}</a>
        <span class="muted">${escape(l.name)}</span>${l.top ? ' <span class="badge">top</span>' : ''}</td>
      <td>${escape(l.revision_label)}</td>
      <td>${stateChip(l.state)}</td>
      <td>${escape(l.uses_revision)}</td>
      <td class="num">${qtyText(l.quantity)}${l.unit === 'each' ? '' : ` ${escape(l.unit)}`}</td>
      <td>${escape(l.find_number)}</td>
    </tr>`).join('');
}


// -- replace everywhere
//
// The server plans and writes (crate::replace); this dialog asks it for a dry
// run, lets the user leave parents out, and applies what is ticked.

const replacing = { to: null, report: null };

$('replace-open').addEventListener('click', () => act(openReplace));

async function openReplace() {
  replacing.to = null;
  replacing.report = null;
  $('rp-title').textContent = currentPart.number;
  $('rp-from').innerHTML = '<option value="">Any revision (the whole part)</option>'
    + currentPart.revision_views.map((r) => `<option value="${escape(r.id)}">Rev ${escape(r.label)} (${escape(r.lifecycle)})</option>`).join('');
  $('rp-from').value = $('wu-revision').value || '';
  $('rp-search').value = '';
  $('rp-results').innerHTML = '';
  const drafts = await api('GET', '/api/ecos?state=draft');
  $('rp-eco').innerHTML = '<option value="-">None</option>'
    + drafts.map((e) => `<option value="${escape(e.id)}">${escape(e.number)} — ${escape(e.title)}</option>`).join('')
    + '<option value="">A new change order…</option>';
  $('rp-eco-title').value = `Replace ${currentPart.number}`;
  chooseReplacement(currentPart.id, currentPart.number, currentPart.name, currentPart.revision_views);
  syncReplaceEco();
  $('replace-dialog').showModal();
}

function syncReplaceEco() {
  $('rp-eco-new-label').hidden = $('rp-eco').value !== '';
}
$('rp-eco').addEventListener('change', syncReplaceEco);

// Any change to the question makes the previous preview stale.
function staleReplace() {
  replacing.report = null;
  $('rp-apply').disabled = true;
  $('rp-wrap').hidden = true;
  $('rp-summary').textContent = '';
}
for (const id of ['rp-from', 'rp-to-rev']) $(id).addEventListener('change', staleReplace);

function chooseReplacement(id, number, name, revisions) {
  replacing.to = { id, number };
  $('rp-chosen').textContent = `Replacement: ${number} ${name}`;
  $('rp-to-rev').innerHTML = '<option value="">Follow its current release (floating)</option>'
    + revisions.map((r) => `<option value="${escape(r.id)}">Rev ${escape(r.label)} (${escape(r.lifecycle)})</option>`).join('');
  const released = revisions.find((r) => r.lifecycle === 'released');
  if (id === currentPart.id) {
    // A revision swap needs a revision; default to the current release.
    $('rp-to-rev').value = released ? released.id : (revisions[revisions.length - 1] || {}).id || '';
  } else {
    $('rp-to-rev').value = released ? released.id : '';
  }
  staleReplace();
}

let replaceTimer = null;
$('rp-search').addEventListener('input', () => {
  clearTimeout(replaceTimer);
  replaceTimer = setTimeout(() => act(searchReplacement), 200);
});

async function searchReplacement() {
  const q = $('rp-search').value.trim();
  if (!q) { $('rp-results').innerHTML = ''; return; }
  const found = (await api('GET', `/api/parts?q=${encodeURIComponent(q)}&limit=8`)).parts;
  $('rp-results').innerHTML = found.length
    ? found.map((p) => `<li><button type="button" data-pick="${escape(p.id)}">
        ${thumb(p.thumbnail_url, 'xs')}<strong class="mono">${escape(p.number)}</strong> ${escape(p.name)}
        <span class="muted">· rev ${escape(p.latest_label)} ${escape(p.latest_state)}</span></button></li>`).join('')
    : '<li class="muted">No part matches.</li>';
}

$('rp-results').addEventListener('click', (event) => {
  const button = event.target.closest('button[data-pick]');
  if (!button) return;
  act(async () => {
    const detail = await api('GET', `/api/parts/${encodeURIComponent(button.dataset.pick)}`);
    chooseReplacement(detail.id, detail.number, detail.name, detail.revision_views);
    $('rp-results').innerHTML = '';
    $('rp-search').value = '';
  });
});

function replaceRequest(dryRun) {
  const body = {
    from_revision: $('rp-from').value,
    to_part: replacing.to ? replacing.to.id : '',
    to_revision: $('rp-to-rev').value,
    dry_run: dryRun,
  };
  if (!dryRun) {
    body.parents = [...$('rp-rows').querySelectorAll('input[data-parent]:checked')].map((c) => c.dataset.parent);
    const eco = $('rp-eco').value;
    if (eco === '') body.eco = { new: { title: $('rp-eco-title').value || `Replace ${currentPart.number}` } };
    else if (eco !== '-') body.eco = eco;
  }
  return body;
}

const REPLACE_STATUS = {
  'would-write': 'will edit', 'would-create': 'will start a new revision',
  written: 'edited', created: 'new revision', skipped: 'skipped', refused: 'refused',
};

function renderReplace(report, preview) {
  $('rp-wrap').hidden = report.rows.length === 0;
  $('rp-rows').innerHTML = report.rows.map((r) => {
    const doable = r.status === 'would-write' || r.status === 'would-create';
    const writes = r.plan === 'edit' ? `rev ${escape(r.revision)} in place`
      : r.plan === 'new-revision' ? `new rev ${escape(r.revision)}` : '—';
    const lines = r.lines.map((l) => `${escape(l.from)} → ${escape(l.to)} <span class="muted">×${qtyText(l.quantity)}${l.find_number ? ` · find ${escape(l.find_number)}` : ''}</span>`).join('<br>');
    const result = `${escape(REPLACE_STATUS[r.status] || r.status)}${r.reason ? `: ${escape(r.reason)}` : ''}${r.eco ? `<br><span class="muted">change order: ${escape(r.eco)}</span>` : ''}`;
    return `
      <tr>
        <td>${preview && doable ? `<input type="checkbox" data-parent="${escape(r.parent_revision_id)}" checked aria-label="Include ${escape(r.number)} rev ${escape(r.parent_revision)}">` : ''}</td>
        <td><a href="#/part/${encodeURIComponent(r.part_id)}">${escape(r.number)}</a> <span class="muted">${escape(r.name)}</span></td>
        <td>${escape(r.parent_revision)} ${stateChip(r.parent_state)}</td>
        <td>${writes}</td>
        <td>${lines || '<span class="muted">nothing</span>'}</td>
        <td class="result-${escape(r.status)}">${result}</td>
      </tr>`;
  }).join('');
  const doable = report.rows.filter((r) => r.status === 'would-write' || r.status === 'would-create').length;
  $('rp-summary').textContent = preview
    ? (report.rows.length ? `${doable} of ${report.rows.length} assemblies can be changed. Untick any to leave out.` : 'No current assembly uses it.')
    : `${report.written} changed, ${report.skipped} skipped, ${report.refused} refused.${report.eco_number ? ` Change order ${report.eco_number}.` : ''}`;
  $('rp-apply').disabled = !preview || doable === 0;
}

$('rp-rows').addEventListener('change', () => {
  const ticked = $('rp-rows').querySelectorAll('input[data-parent]:checked').length;
  $('rp-apply').disabled = !replacing.report || ticked === 0;
});

$('rp-preview').addEventListener('click', () => act(async () => {
  const report = await api('POST', `${partUrl()}/replace`, replaceRequest(true));
  replacing.report = report;
  renderReplace(report, true);
}));

$('rp-apply').addEventListener('click', () => act(async () => {
  const body = replaceRequest(false);
  if (!body.parents.length) return;
  const report = await api('POST', `${partUrl()}/replace`, body);
  replacing.report = null;
  renderReplace(report, false);
  await loadWhereUsed();
}));


// -- attachments
//
// The bytes go up as the request body (crate::api::attachments), so a
// 100 MB file is never read into the page or the server's memory whole.

const attaching = { part: null, revision: null };

async function renderAttachments(detail) {
  if (attaching.part !== detail.id || !detail.revision_views.some((r) => r.id === attaching.revision)) {
    attaching.part = detail.id;
    attaching.revision = defaultRevision(detail);
    $('att-preview').hidden = true;
  }
  revisionOptions($('att-revision'), detail, attaching.revision);
  await loadAttachments();
}

$('att-revision').addEventListener('change', () => {
  attaching.revision = $('att-revision').value;
  act(loadAttachments);
});

const fileSize = (n) => (n >= 1048576 ? `${(n / 1048576).toFixed(1)} MB` : n >= 1024 ? `${Math.round(n / 1024)} KB` : `${n} B`);

async function loadAttachments() {
  const list = await api('GET', `${partUrl()}/attachments?revision=${encodeURIComponent(attaching.revision)}`);
  const rev = currentPart.revision_views.find((r) => r.id === attaching.revision);
  const revisionOpen = list.revision_editable;
  $('att-target').querySelector('option[value=revision]').textContent = `rev ${list.revision_label}`
    + (revisionOpen ? '' : ` (${rev ? rev.lifecycle : 'frozen'} — frozen)`);
  $('att-target').querySelector('option[value=revision]').disabled = !revisionOpen;
  if (!revisionOpen) $('att-target').value = 'part';
  const rows = [
    ...list.part.map((a) => ({ ...a, on: 'part', removable: true })),
    ...list.revision.map((a) => ({ ...a, on: `rev ${list.revision_label}`, removable: revisionOpen })),
  ];
  $('att-empty').hidden = rows.length > 0;
  $('att-wrap').hidden = rows.length === 0;
  $('att-rows').innerHTML = rows.map((a) => {
    const href = `/api/attachments/${encodeURIComponent(a.id)}`;
    const thumb = a.inline && a.media_type.startsWith('image/')
      ? `<img class="thumb" src="${href}?inline=true" alt="">` : '';
    const view = a.inline
      ? (a.media_type.startsWith('image/')
        ? `<button type="button" class="link" data-preview="${escape(a.id)}">Preview</button>`
        : `<a href="${href}?inline=true" target="_blank" rel="noopener">Open</a>`)
      : '';
    return `
      <tr>
        <td>${thumb}<a href="${href}" download="${escape(a.name)}">${escape(a.name)}</a>
          <span class="muted mono">${escape(a.media_type)}</span></td>
        <td>${escape(a.on)}</td>
        <td>${escape(a.kind)}</td>
        <td class="num">${fileSize(a.size)}</td>
        <td>${escape(a.uploaded_by_name || '')}<br><span class="muted">${escape(when(a.uploaded_at))}</span></td>
        <td class="actions">${view}
          ${a.removable && me.can_author ? `<button type="button" class="link danger" data-remove-file="${escape(a.id)}" data-name="${escape(a.name)}">Remove</button>` : ''}</td>
      </tr>`;
  }).join('');
}

$('att-rows').addEventListener('click', (event) => {
  const preview = event.target.closest('button[data-preview]');
  if (preview) {
    const box = $('att-preview');
    const same = !box.hidden && box.dataset.id === preview.dataset.preview;
    box.hidden = same;
    box.dataset.id = preview.dataset.preview;
    box.innerHTML = same ? '' : `<img src="/api/attachments/${encodeURIComponent(preview.dataset.preview)}?inline=true" alt="Preview">`;
    return;
  }
  const remove = event.target.closest('button[data-remove-file]');
  if (!remove) return;
  if (!confirm(`Remove ${remove.dataset.name}?`)) return;
  act(async () => {
    await api('DELETE', `/api/attachments/${encodeURIComponent(remove.dataset.removeFile)}`);
    if ($('att-preview').dataset.id === remove.dataset.removeFile) $('att-preview').hidden = true;
    await loadAttachments();
  });
});

async function uploadFiles(files) {
  const target = $('att-target').value === 'revision'
    ? `${partUrl()}/revisions/${encodeURIComponent(attaching.revision)}/attachments`
    : `${partUrl()}/attachments`;
  let done = 0;
  for (const file of files) {
    $('att-status').textContent = `Uploading ${file.name} (${fileSize(file.size)})…`;
    const query = new URLSearchParams({ name: file.name, kind: $('att-kind').value });
    const response = await fetch(`${target}?${query}`, {
      method: 'POST',
      credentials: 'same-origin',
      headers: { 'Content-Type': file.type || 'application/octet-stream', 'X-CSRF-Token': csrfToken || '' },
      body: file,
    });
    if (!response.ok) {
      let message = `${response.status} ${response.statusText}`;
      try { message = (await response.json()).error || message; } catch { /* not JSON */ }
      $('att-status').textContent = '';
      await loadAttachments();
      throw new Error(`${file.name}: ${message}`);
    }
    done += 1;
  }
  $('att-status').textContent = done ? `Added ${done} file${done === 1 ? '' : 's'}.` : '';
  await loadAttachments();
}

$('att-file').addEventListener('change', () => {
  const files = [...$('att-file').files];
  $('att-file').value = '';
  if (files.length) act(() => uploadFiles(files));
});

for (const name of ['dragenter', 'dragover']) {
  $('att-drop').addEventListener(name, (event) => {
    event.preventDefault();
    $('att-drop').classList.add('over');
  });
}
for (const name of ['dragleave', 'drop']) {
  $('att-drop').addEventListener(name, () => $('att-drop').classList.remove('over'));
}
$('att-drop').addEventListener('drop', (event) => {
  event.preventDefault();
  const files = [...event.dataTransfer.files];
  if (files.length) act(() => uploadFiles(files));
});


// ----------------------------------------------------------------- backups

async function renderBackups() {
  const b = await api('GET', '/api/admin/backups');
  $('bk-config').innerHTML = b.dir
    ? `Saved to <code>${escape(b.dir)}</code>, the newest ${b.keep} kept`
      + (b.every_minutes ? `, one every ${b.every_minutes} minute${b.every_minutes === 1 ? '' : 's'}.` : '. No schedule (<code>--backup-every</code>).')
    : 'No backup directory is set (<code>--backup-dir</code>), so backups are downloaded, not saved on the server.';
  $('bk-now').hidden = !b.dir;
  $('bk-now').disabled = b.running;
  $('bk-status').textContent = b.running ? 'A backup is running…' : '';
  const last = b.last;
  $('bk-last').innerHTML = !last ? 'None since the server started.'
    : last.ok
      ? `${escape(when(last.at))} · ${escape(last.trigger)} · ${last.files} files, ${fileSize(last.size)}`
        + `${last.file ? ` · <code>${escape(last.file)}</code>` : ''}`
        + `${last.attempts > 1 ? ` · ${last.attempts} tries` : ''}`
      : `<span class="error">${escape(when(last.at))} · ${escape(last.trigger)} failed: ${escape(last.error)}</span>`;
  $('bk-empty').hidden = b.saved.length > 0;
  $('bk-wrap').hidden = b.saved.length === 0;
  $('bk-rows').innerHTML = b.saved.map((f) => `
    <tr>
      <td><a href="/api/admin/backups/file/${encodeURIComponent(f.name)}" download>${escape(f.name)}</a></td>
      <td class="num">${fileSize(f.size)}</td>
      <td>${escape(when(f.modified))}</td>
    </tr>`).join('');
}

$('bk-now').addEventListener('click', () => act(async () => {
  $('bk-now').disabled = true;
  $('bk-status').textContent = 'Backing up…';
  try {
    await api('POST', '/api/admin/backups');
  } finally {
    await renderBackups();
  }
}));

$('bk-download').addEventListener('click', () => {
  $('bk-status').textContent = 'Preparing the download…';
  setTimeout(() => act(renderBackups), 1500);
});


// ----------------------------------------------------------------- account

const SCOPE_NAMES = { full: 'Full', read: 'Read only', worker: 'Worker' };

function tokenRow(t, showOwner) {
  const expires = t.expires_at ? (t.expired ? `<span class="error">expired ${escape(when(t.expires_at))}</span>` : escape(when(t.expires_at))) : 'never';
  return `
    <tr>
      ${showOwner ? `<td class="mono">${escape(t.username)}</td>` : ''}
      <td>${escape(t.name)}</td>
      <td>${escape(SCOPE_NAMES[t.scope] || t.scope)}</td>
      <td class="mono">${escape(t.prefix)}…</td>
      <td>${escape(when(t.created_at))}</td>
      <td>${escape(when(t.last_used_at))}</td>
      <td>${expires}</td>
      <td class="actions"><button class="link danger" data-token="${escape(t.id)}" data-name="${escape(t.name)}">Revoke</button></td>
    </tr>`;
}

function wireRevoke(container, after) {
  for (const button of container.querySelectorAll('button[data-token]')) {
    button.addEventListener('click', () => act(async () => {
      if (!confirm(`Revoke the token "${button.dataset.name}"? Anything using it stops working at once.`)) return;
      await api('DELETE', `/api/tokens/${button.dataset.token}`);
      await after();
    }));
  }
}

async function renderAccount() {
  $('account-who').textContent = `Signed in as ${me.username}${me.display_name ? ` (${me.display_name})` : ''}.`;
  const tokens = await api('GET', '/api/tokens');
  $('token-rows').innerHTML = tokens.map((t) => tokenRow(t, false)).join('');
  $('tokens-empty').hidden = tokens.length > 0;
  wireRevoke($('token-rows'), renderAccount);
  if (me.is_admin) {
    const users = await api('GET', '/api/users');
    const current = $('tk-user').value || me.id;
    $('tk-user').innerHTML = users
      .map((u) => `<option value="${escape(u.id)}">${escape(u.username)}${u.id === me.id ? ' (you)' : ''}</option>`)
      .join('');
    $('tk-user').value = current;
  }
}

$('password-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    if ($('pw-new').value !== $('pw-again').value) throw new Error('The two new passwords are not the same.');
    const done = await api('POST', '/api/me/password', { current: $('pw-current').value, new: $('pw-new').value });
    for (const id of ['pw-current', 'pw-new', 'pw-again']) $(id).value = '';
    $('pw-status').textContent = `Changed. ${done.other_sessions_ended} other session${done.other_sessions_ended === 1 ? '' : 's'} signed out.`;
  });
});

$('logout-all').addEventListener('click', () => act(async () => {
  if (!confirm('Sign out of every browser, this one included?')) return;
  await api('POST', '/api/logout-all');
  me = null;
  location.hash = '';
  location.reload();
}));

$('token-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const body = { name: $('tk-name').value.trim(), scope: $('tk-scope').value };
    const days = Number($('tk-days').value);
    if (days > 0) body.expires_days = days;
    if (me.is_admin && $('tk-user').value) body.user_id = $('tk-user').value;
    const created = await api('POST', '/api/tokens', body);
    $('tk-name').value = '';
    $('tk-days').value = '';
    $('token-secret-text').textContent = created.token;
    $('token-secret').hidden = false;
    await renderAccount();
  });
});

$('token-copy').addEventListener('click', () => act(async () => {
  await navigator.clipboard.writeText($('token-secret-text').textContent);
  $('token-copy').textContent = 'Copied';
}));

$('token-done').addEventListener('click', () => {
  $('token-secret-text').textContent = '';
  $('token-secret').hidden = true;
  $('token-copy').textContent = 'Copy';
});

// ---------------------------------------------------------------- security

async function renderSecurity() {
  const [lockouts, tokens] = await Promise.all([
    api('GET', '/api/security/lockouts'),
    api('GET', '/api/tokens?all=true'),
  ]);
  $('lockout-rows').innerHTML = lockouts.map((l) => `
    <tr>
      <td>${l.kind === 'user' ? 'Username' : 'Address'}</td>
      <td class="mono">${escape(l.key)}</td>
      <td>${l.failures}</td>
      <td>${escape(when(l.last_failure))}</td>
      <td>${l.locked ? `<span class="error">${escape(when(l.locked_until))}</span>` : '<span class="muted">not locked</span>'}</td>
      <td class="actions"><button class="link" data-kind="${escape(l.kind)}" data-key="${escape(l.key)}">Clear</button></td>
    </tr>`).join('');
  $('lockouts-empty').hidden = lockouts.length > 0;
  $('lockouts-clear').disabled = lockouts.length === 0;
  for (const button of $('lockout-rows').querySelectorAll('button[data-key]')) {
    button.addEventListener('click', () => act(async () => {
      const params = new URLSearchParams({ kind: button.dataset.kind, key: button.dataset.key });
      await api('DELETE', `/api/security/lockouts?${params}`);
      await renderSecurity();
    }));
  }
  $('all-token-rows').innerHTML = tokens.map((t) => tokenRow(t, true)).join('');
  $('all-tokens-empty').hidden = tokens.length > 0;
  wireRevoke($('all-token-rows'), renderSecurity);
}

$('lockouts-clear').addEventListener('click', () => act(async () => {
  if (!confirm('Clear every sign-in lockout?')) return;
  await api('DELETE', '/api/security/lockouts');
  await renderSecurity();
}));

// ---------------------------------------------------------------- audit log

const AUDIT_PAGE = 50;

// One value, short enough for a table cell; the full text is in the title.
function auditValue(value) {
  if (value === null || value === undefined) return '—';
  const text = typeof value === 'string' ? value : JSON.stringify(value);
  return text.length > 80 ? `${text.slice(0, 77)}…` : text;
}

// A review's rounds and a discussion are whole lists in the log; a cell says
// what they amount to — the last round's status, the newest comment.
function auditSummary(field, value) {
  if (!Array.isArray(value)) return value;
  const last = value[value.length - 1];
  if (!last) return value.length ? value : 'none';
  if (field === 'reviews') {
    const d = last.decisions || [];
    const said = d.length ? `, last: ${d[d.length - 1].verdict}` : '';
    return `${value.length} round${value.length === 1 ? '' : 's'}; current ${last.status}${said}`;
  }
  if (field === 'comments') return `${value.length} comment${value.length === 1 ? '' : 's'}; newest "${last.body}"`;
  if (field === 'items') return `${value.length} item${value.length === 1 ? '' : 's'}`;
  return value;
}

function auditChanges(event) {
  const lines = Object.entries(event.changes || {}).map(([field, raw]) => {
    const change = { before: raw.before === null ? null : auditSummary(field, raw.before), after: auditSummary(field, raw.after) };
    const before = change.before === null ? '' : `${escape(auditValue(change.before))}<span class="arrow">→</span>`;
    const full = `${field}: ${JSON.stringify(raw.before)} → ${JSON.stringify(raw.after)}`;
    return `<div title="${escape(full)}"><span class="field">${escape(field)}</span> ${before}${escape(auditValue(change.after))}</div>`;
  });
  if (event.detail) lines.push(`<div>${escape(event.detail)}</div>`);
  return lines.join('');
}

function auditTarget(event) {
  const e = event.entity;
  const label = escape(e.label || e.id);
  if (e.kind === 'part') return `<a href="#/part/${encodeURIComponent(e.id)}">${label}</a>`;
  if (e.kind === 'revision' && e.part_id) return `<a href="#/part/${encodeURIComponent(e.part_id)}">${label}</a>`;
  return `<span class="muted">${escape(e.kind)}</span> ${label}`;
}

function auditRow(event, showTarget) {
  const who = event.actor.username || '(server)';
  const via = [event.actor.via, event.actor.ip].filter(Boolean).join(' · ');
  return `
    <tr>
      <td class="when">${escape(when(event.at))}</td>
      <td>${escape(who)}<div class="via">${escape(via)}</div></td>
      <td class="action">${escape(event.action)}</td>
      <td>${showTarget ? auditTarget(event) : escape(event.entity.kind === 'revision' ? `rev ${event.entity.label.split(' rev ').pop()}` : 'part')}</td>
      <td class="changes">${auditChanges(event)}</td>
    </tr>`;
}

// A list that pages backwards with "Older": the rows so far, the query, and
// the oldest id shown.
async function pageAudit({ base, params, rows, empty, more, showTarget, append }) {
  const query = new URLSearchParams(params);
  query.set('limit', String(AUDIT_PAGE));
  if (append && rows.dataset.oldest) query.set('before', rows.dataset.oldest);
  const events = await api('GET', `${base}?${query}`);
  const html = events.map((e) => auditRow(e, showTarget)).join('');
  rows.innerHTML = append ? rows.innerHTML + html : html;
  if (events.length) rows.dataset.oldest = String(events[events.length - 1].id);
  else if (!append) delete rows.dataset.oldest;
  empty.hidden = rows.children.length > 0;
  more.hidden = events.length < AUDIT_PAGE;
}

// The part page's History panel.
let historyPart = null;

async function renderHistory(detail, append = false) {
  historyPart = detail.id;
  const base = `/api/parts/${encodeURIComponent(detail.id)}/history`;
  $('history-csv').href = `${base}?format=csv&limit=5000`;
  await pageAudit({
    base, params: {}, rows: $('history-rows'), empty: $('history-empty'), more: $('history-more'),
    showTarget: false, append,
  });
}

$('history-more').addEventListener('click', () => act(async () => {
  if (currentPart && currentPart.id === historyPart) await renderHistory(currentPart, true);
}));

// The admin's Audit page.
function auditParams() {
  const params = {};
  const user = $('af-user').value.trim();
  if (user) params.user = user;
  if ($('af-action').value) params.action = $('af-action').value;
  if ($('af-kind').value) params.kind = $('af-kind').value;
  // Dates are the viewer's local days: from the start of one to the end of the other.
  if ($('af-since').value) params.since = String(Math.floor(new Date(`${$('af-since').value}T00:00:00`).getTime() / 1000));
  if ($('af-until').value) params.until = String(Math.floor(new Date(`${$('af-until').value}T23:59:59`).getTime() / 1000));
  return params;
}

async function renderAudit(append = false) {
  const params = auditParams();
  $('audit-csv').href = `/api/audit?${new URLSearchParams({ ...params, format: 'csv', limit: '5000' })}`;
  await pageAudit({
    base: '/api/audit', params, rows: $('audit-rows'), empty: $('audit-empty'), more: $('audit-more'),
    showTarget: true, append,
  });
}

$('audit-filter').addEventListener('submit', (event) => {
  event.preventDefault();
  act(() => renderAudit());
});

$('af-clear').addEventListener('click', () => {
  for (const id of ['af-user', 'af-since', 'af-until']) $(id).value = '';
  $('af-action').value = '';
  $('af-kind').value = '';
  act(() => renderAudit());
});

$('audit-more').addEventListener('click', () => act(() => renderAudit(true)));

// ------------------------------------------------------------------ reviews

// A due date typed as a local day means the end of that day.
const dayToSeconds = (day) => (day ? Math.floor(new Date(`${day}T23:59:59`).getTime() / 1000) : 0);
const secondsToDay = (seconds) => {
  if (!seconds) return '';
  const d = new Date(seconds * 1000);
  return `${d.getFullYear()}-${String(d.getMonth() + 1).padStart(2, '0')}-${String(d.getDate()).padStart(2, '0')}`;
};
const splitList = (text) => text.split(',').map((t) => t.trim()).filter(Boolean);
const reviewChip = (status) => `<span class="state review-${escape(status)}">${escape(status)}</span>`;

// Show a `review-event` hook's failure: the event itself stood.
function showWarnings(result) {
  if (result && result.warnings && result.warnings.length) banner(result.warnings.join('\n'));
}

// The badge in the header: how many reviews wait on the person signed in.
async function refreshInbox() {
  if (!me) return;
  try {
    const inbox = await api('GET', '/api/inbox');
    $('inbox-badge').textContent = inbox.count;
    $('inbox-badge').hidden = inbox.count === 0;
  } catch {
    // The badge is a convenience; the page it links to says what is wrong.
  }
}

function reviewCell(rev) {
  const parts = [];
  if (rev.review) {
    const r = rev.review;
    parts.push(reviewChip(r.status));
    if (r.live && r.required_approvals) parts.push(`<span class="muted">${r.approvals}/${r.required_approvals}</span>`);
  }
  if (rev.comments) parts.push(`<span class="muted" title="Comments">💬 ${rev.comments}</span>`);
  if (rev.eco) parts.push(`<a class="badge" href="#/eco/${escape(rev.eco.id)}" title="${escape(rev.eco.action)} in ${escape(rev.eco.number)} (${escape(rev.eco.state)})">${escape(rev.eco.number)}</a>`);
  return parts.length ? parts.join(' ') : '<span class="muted">—</span>';
}

// -- the reviews page

function inboxRow(item) {
  const href = item.kind === 'eco' ? `#/eco/${escape(item.target)}` : `#/part/${escape(item.target)}`;
  const due = item.due ? `${escape(when(item.due))}${item.overdue ? ' <span class="state review-rejected">overdue</span>' : ''}` : '—';
  return `
    <tr>
      <td class="number"><a href="${href}">${escape(item.title)}</a></td>
      <td>${escape(item.name)}</td>
      <td>${escape(item.opened_by)}</td>
      <td>${escape(when(item.opened_at))}</td>
      <td>${due}</td>
      <td>${item.approvals}/${item.required_approvals}</td>
      <td>${reviewChip(item.status)}</td>
    </tr>`;
}

async function renderReviews() {
  const inbox = await api('GET', '/api/inbox');
  $('inbox-waiting').innerHTML = inbox.waiting_on_me.map(inboxRow).join('');
  $('inbox-waiting-empty').hidden = inbox.waiting_on_me.length > 0;
  $('inbox-submitted').innerHTML = inbox.submitted.map(inboxRow).join('');
  $('inbox-submitted-empty').hidden = inbox.submitted.length > 0;
}

// -- one revision's review, on the part page

let reviewRevision = '';
let replyTo = '';

// The revision the panel opens on: one in review if any, else the newest.
function defaultReviewRevision(detail) {
  const views = detail.revision_views;
  const inReview = [...views].reverse().find((r) => r.lifecycle === 'inreview');
  return (inReview || views[views.length - 1] || {}).id || '';
}

function roundHtml(round, current, prefix = 'rv') {
  const due = round.due
    ? ` · due ${escape(when(round.due))}${round.overdue ? ' <span class="state review-rejected">overdue</span>' : ''}`
    : '';
  const reviewers = round.reviewers.length
    ? round.reviewers.map((r) => {
      if (r.kind === 'group') {
        const who = r.approved_by.length ? ` — approved by ${escape(r.approved_by.join(', '))}` : '';
        return `<li><span class="badge">group</span> ${escape(r.name)}${who}</li>`;
      }
      const said = r.verdict ? ` ${reviewChip(r.verdict === 'approve' ? 'approved' : 'rejected')}` : ' <span class="muted">waiting</span>';
      return `<li>${escape(r.name)}${said}</li>`;
    }).join('')
    : '<li class="muted">Anyone in the check-in group may decide.</li>';
  const decisions = round.decisions.map((d) => `
    <li>${reviewChip(d.verdict === 'approve' ? 'approved' : 'rejected')} <strong>${escape(d.name)}</strong>
      <span class="muted">${escape(when(d.at))}</span>${d.comment ? `<div class="comment-body">${escape(d.comment)}</div>` : ''}</li>`).join('');
  const closed = round.closed_at
    ? `<p class="muted">Ended ${escape(when(round.closed_at))}${round.closed_by ? ` by ${escape(round.closed_by)}` : ''}.</p>`
    : '';
  const decide = current && round.can_decide ? `
    <div class="decide">
      <textarea id="${prefix}-decision-comment" rows="2" placeholder="Comment — required to reject" aria-label="Decision comment"></textarea>
      <div class="toolbar">
        <button type="button" data-verdict="approve">Approve</button>
        <button type="button" class="danger" data-verdict="reject">Reject</button>
      </div>
    </div>` : '';
  const change = current && round.can_change ? `<button type="button" class="ghost" id="${prefix}-change">Change</button>` : '';
  return `
    <div class="round">
      <div class="toolbar">
        ${reviewChip(round.status)}
        <strong>${round.approvals} of ${round.required_approvals} approval${round.required_approvals === 1 ? '' : 's'}</strong>
        <span class="muted">submitted by ${escape(round.opened_by)} ${escape(when(round.opened_at))}${due}
          · rule from ${escape(round.rule_source)}</span>
        <span class="spacer"></span>
        ${change}
      </div>
      <ul class="plain reviewers">${reviewers}</ul>
      ${decisions ? `<ul class="plain decisions">${decisions}</ul>` : ''}
      ${closed}
      ${decide}
    </div>`;
}

function threadHtml(comments, canComment) {
  const children = new Map();
  for (const c of comments) {
    const key = c.parent || '';
    if (!children.has(key)) children.set(key, []);
    children.get(key).push(c);
  }
  const known = new Set(comments.map((c) => c.id));
  const render = (parent) => (children.get(parent) || []).map((c) => `
    <li>
      <div class="comment-head"><strong>${escape(c.author)}</strong> <span class="muted">${escape(when(c.at))}</span>
        ${canComment ? `<button type="button" class="link" data-reply="${escape(c.id)}" data-author="${escape(c.author)}">Reply</button>` : ''}</div>
      <div class="comment-body">${escape(c.body)}</div>
      ${children.has(c.id) ? `<ul class="plain thread">${render(c.id)}</ul>` : ''}
    </li>`).join('');
  // A reply whose parent is unknown still shows, at the top level.
  const orphans = comments.filter((c) => c.parent && !known.has(c.parent)).map((c) => c.id);
  for (const id of orphans) {
    const c = comments.find((x) => x.id === id);
    children.set('', [...(children.get('') || []), { ...c, parent: '' }]);
  }
  return `<ul class="plain thread">${render('')}</ul>`;
}

async function renderReview(detail) {
  const views = detail.revision_views;
  if (!views.some((r) => r.id === reviewRevision)) reviewRevision = defaultReviewRevision(detail);
  $('rv-revision').innerHTML = views.map((r) => `<option value="${escape(r.id)}">${escape(r.label)} — ${escape(r.lifecycle)}</option>`).join('');
  $('rv-revision').value = reviewRevision;
  if (!reviewRevision) return;
  const panel = await api('GET', `${revUrl(reviewRevision)}/review`);
  $('rv-current').innerHTML = panel.current ? roundHtml(panel.current, true) : '';
  $('rv-none').hidden = !!panel.current;
  const rule = panel.rule;
  $('rv-none').textContent = rule.required_approvals
    ? `Not submitted for review yet. It needs ${rule.required_approvals} approval${rule.required_approvals === 1 ? '' : 's'} before it can release (rule from ${panel.rule_source}).`
    : `Not submitted for review. No approval is needed to release it (rule from ${panel.rule_source}); submitting asks for one anyway.`;
  $('rv-history').hidden = panel.history.length === 0;
  $('rv-history-list').innerHTML = panel.history.map((r) => roundHtml(r, false)).join('');
  $('rv-comments').innerHTML = panel.comments.length ? threadHtml(panel.comments, panel.can_comment) : '';
  $('rv-comments-empty').hidden = panel.comments.length > 0;
  $('rv-comment-form').hidden = !panel.can_comment;
  setReplyTo('', '');

  for (const button of $('rv-current').querySelectorAll('button[data-verdict]')) {
    button.addEventListener('click', () => act(async () => {
      const verdict = button.dataset.verdict;
      const comment = $('rv-decision-comment').value;
      if (verdict === 'reject' && !confirm('Reject this revision? It goes back to Draft, and the review ends.')) return;
      showWarnings(await api('POST', `${revUrl(reviewRevision)}/review/decision`, { verdict, comment }));
      await renderPart(currentPart.id);
    }));
  }
  const change = $('rv-change');
  if (change) change.addEventListener('click', () => { roundTarget = null; openRoundDialog(panel.current); });
  for (const button of $('rv-comments').querySelectorAll('button[data-reply]')) {
    button.addEventListener('click', () => {
      setReplyTo(button.dataset.reply, button.dataset.author);
      $('rv-comment').focus();
    });
  }
}

function setReplyTo(id, author) {
  replyTo = id;
  $('rv-reply-to').hidden = !id;
  $('rv-reply-to').textContent = id ? `Replying to ${author}` : '';
  $('rv-reply-cancel').hidden = !id;
}

$('rv-revision').addEventListener('change', () => act(async () => {
  reviewRevision = $('rv-revision').value;
  await renderReview(currentPart);
}));
$('rv-reply-cancel').addEventListener('click', () => setReplyTo('', ''));
$('rv-comment-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    const body = $('rv-comment').value;
    showWarnings(await api('POST', `${revUrl(reviewRevision)}/comments`, { body, parent: replyTo }));
    $('rv-comment').value = '';
    await renderPart(currentPart.id);
  });
});

// Reviewers as the server reports them, as the text the inputs take.
function reviewersText(refs, names = {}) {
  return refs.map((r) => `${r.kind}:${r.kind === 'user' ? (names[r.id] || r.id) : r.id}`).join(', ');
}

// -- submitting

let submitting = null;

async function openSubmit(revisionId) {
  submitting = revisionId;
  const rev = currentPart.revision_views.find((r) => r.id === revisionId);
  const panel = await api('GET', `${revUrl(revisionId)}/review`);
  $('sb-title').textContent = `${currentPart.number} rev ${rev ? rev.label : ''}`;
  const n = panel.rule.required_approvals;
  $('sb-rule').textContent = `${n ? `It needs ${n} approval${n === 1 ? '' : 's'}` : 'No approval is needed'} by the rule from ${panel.rule_source}.`
    + (panel.rule.reviewers.length ? ' The rule names its reviewers; you can add more.' : ' Anyone in the check-in group may decide, unless you name reviewers.');
  $('sb-reviewers').value = '';
  $('sb-due').value = '';
  $('sb-note').value = '';
  $('submit-dialog').showModal();
}

$('submit-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'submit') return;
  event.preventDefault();
  act(async () => {
    const body = { note: $('sb-note').value };
    const extra = splitList($('sb-reviewers').value);
    if (extra.length) body.reviewers = extra;
    if ($('sb-due').value) body.due = dayToSeconds($('sb-due').value);
    const result = await api('POST', `${revUrl(submitting)}/submit`, body);
    $('submit-dialog').close();
    reviewRevision = submitting;
    await renderPart(currentPart.id);
    showWarnings(result);
  });
});

// -- changing a round

function openRoundDialog(round) {
  $('rd-reviewers').value = round.reviewers.map((r) => `${r.kind}:${r.kind === 'user' ? r.name : r.id}`).join(', ');
  $('rd-due').value = secondsToDay(round.due);
  $('rd-required').value = round.required_approvals;
  $('rd-required-label').hidden = !me.can_checkin;
  $('round-dialog').showModal();
}

// Where the round dialog saves, and what it refreshes: a revision's round or
// a change order's.
let roundTarget = null;

$('round-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    const body = { reviewers: splitList($('rd-reviewers').value), due: dayToSeconds($('rd-due').value) };
    if (me.can_checkin) body.required_approvals = Number($('rd-required').value);
    const target = roundTarget || { url: `${revUrl(reviewRevision)}/review`, refresh: () => renderPart(currentPart.id) };
    const result = await api('PATCH', target.url, body);
    $('round-dialog').close();
    await target.refresh();
    showWarnings(result);
  });
});

// -- the rule, on the settings page

let reviewNames = {};

function readRule({ required, due, reviewers, self }) {
  return {
    required_approvals: Number(required.value) || 0,
    due_days: Number(due.value) || 0,
    reviewers: splitList(reviewers.value),
    allow_self_approval: self.checked,
  };
}

async function overrideOptions(select, scope, value) {
  if (scope === 'category') {
    await loadCategories();
    categoryOptions(select);
  } else {
    const types = await api('GET', '/api/part-types');
    select.innerHTML = types.map((t) => `<option value="${escape(t.id)}">${escape(t.name)} (${escape(t.id)})</option>`).join('');
  }
  if (value) select.value = value;
}

async function addOverrideRow(o = { scope: 'part_type', id: '', rule: { required_approvals: 1, due_days: 0, reviewers: [], allow_self_approval: false } }) {
  const row = document.createElement('tr');
  row.innerHTML = `
    <td><select data-f="scope" aria-label="Override for">
      <option value="part_type">Part type</option><option value="category">Category</option></select></td>
    <td><select data-f="id" aria-label="Which"></select></td>
    <td><input data-f="required" type="number" min="0" max="50" class="tiny" aria-label="Approvals" value="${o.rule.required_approvals}"></td>
    <td><input data-f="due" type="number" min="0" class="tiny" aria-label="Due days" value="${o.rule.due_days}"></td>
    <td><input data-f="reviewers" aria-label="Reviewers" placeholder="user:grace, group:quality" value="${escape(reviewersText(o.rule.reviewers, reviewNames))}"></td>
    <td><input data-f="self" type="checkbox" aria-label="The submitter's own approval counts" ${o.rule.allow_self_approval ? 'checked' : ''}></td>
    <td><button type="button" class="link danger" data-f="remove">Remove</button></td>`;
  const scope = row.querySelector('[data-f=scope]');
  scope.value = o.scope;
  await overrideOptions(row.querySelector('[data-f=id]'), o.scope, o.id);
  scope.addEventListener('change', () => act(() => overrideOptions(row.querySelector('[data-f=id]'), scope.value, '')));
  row.querySelector('[data-f=remove]').addEventListener('click', () => row.remove());
  $('set-rv-overrides').appendChild(row);
}

function readOverrides() {
  return [...$('set-rv-overrides').querySelectorAll('tr')].map((row) => {
    const f = (name) => row.querySelector(`[data-f=${name}]`);
    return {
      scope: f('scope').value,
      id: f('id').value,
      rule: readRule({ required: f('required'), due: f('due'), reviewers: f('reviewers'), self: f('self') }),
    };
  });
}

async function renderReviewSettings(settings) {
  try {
    const users = await api('GET', '/api/users');
    reviewNames = Object.fromEntries(users.map((u) => [u.id, u.username]));
  } catch {
    reviewNames = {};
  }
  const rule = settings.review_rule;
  $('set-rv-required').value = rule.required_approvals;
  $('set-rv-due').value = rule.due_days;
  $('set-rv-reviewers').value = reviewersText(rule.reviewers, reviewNames);
  $('set-rv-self').checked = rule.allow_self_approval;
  $('set-rv-overrides').innerHTML = '';
  for (const o of settings.review_overrides) await addOverrideRow(o);
}

$('set-rv-add').addEventListener('click', () => act(() => addOverrideRow()));

// ----------------------------------------------------------- change orders

const ECO_STATES = { draft: 'draft', inreview: 'in review', approved: 'approved', released: 'released', cancelled: 'cancelled' };
const ecoChip = (state) => `<span class="state eco-${escape(state)}">${escape(ECO_STATES[state] || state)}</span>`;
const priorityChip = (p) => (p === 'normal' ? '<span class="muted">normal</span>' : `<span class="badge priority-${escape(p)}">${escape(p)}</span>`);

async function renderEcos() {
  const rows = await api('GET', `/api/ecos?state=${encodeURIComponent($('eco-filter').value)}`);
  $('eco-rows').innerHTML = rows.map((e) => `
    <tr>
      <td class="number"><a href="#/eco/${escape(e.id)}">${escape(e.number)}</a></td>
      <td>${escape(e.title)}</td>
      <td>${ecoChip(e.state)}</td>
      <td>${priorityChip(e.priority)}</td>
      <td>${e.item_count}</td>
      <td>${e.review ? reviewChip(e.review) : '—'}</td>
      <td>${escape(e.created_by)}<br><span class="muted">${escape(when(e.created_at))}</span></td>
    </tr>`).join('');
  $('eco-empty').hidden = rows.length > 0;
}

$('eco-filter').addEventListener('change', () => act(renderEcos));

// -- creating and editing

let ecoNumbering = null;
let editingEco = null;

async function openEcoDialog(eco) {
  editingEco = eco;
  ecoNumbering = await api('GET', '/api/ecos/numbering');
  const typed = ecoNumbering.mode.kind !== 'counter';
  $('eco-dialog-title').textContent = eco ? `Edit ${eco.number}` : 'New change order';
  $('ed-number-label').hidden = !!eco || !typed;
  $('ed-number').value = '';
  $('ed-number').placeholder = ecoNumbering.mode.kind === 'pattern' ? ecoNumbering.mode.regex : '';
  $('ed-title').value = eco ? eco.title : '';
  $('ed-reason').value = eco ? eco.reason : '';
  $('ed-description').value = eco ? eco.description : '';
  $('ed-priority').value = eco ? eco.priority : 'normal';
  $('eco-dialog').showModal();
}

$('eco-new').addEventListener('click', () => act(() => openEcoDialog(null)));

$('eco-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    const body = {
      title: $('ed-title').value,
      reason: $('ed-reason').value,
      description: $('ed-description').value,
      priority: $('ed-priority').value,
    };
    let id;
    if (editingEco) {
      id = (await api('PATCH', `/api/ecos/${encodeURIComponent(editingEco.id)}`, body)).id;
    } else {
      body.number = $('ed-number').value;
      id = (await api('POST', '/api/ecos', body)).id;
    }
    $('eco-dialog').close();
    if (location.hash === `#/eco/${id}`) await renderEco(id);
    else location.hash = `#/eco/${id}`;
  });
});

// -- one change order

let currentEco = null;
let ecoReplyTo = '';
const ecoUrl = () => `/api/ecos/${encodeURIComponent(currentEco.id)}`;

function itemChanges(item) {
  const out = [];
  if (item.document_changed === true) out.push('document changed');
  if (item.document_changed === false) out.push('<span class="muted">same document</span>');
  if (item.bom_diff && item.bom_diff.lines.length) out.push(`${item.bom_diff.lines.length} BOM change${item.bom_diff.lines.length === 1 ? '' : 's'}`);
  if (item.attribute_changes.length) out.push(`${item.attribute_changes.length} catalog change${item.attribute_changes.length === 1 ? '' : 's'}`);
  return out.join(' · ') || '<span class="muted">—</span>';
}

function itemDetail(item) {
  const parts = [];
  if (item.document_changed !== null && item.document_changed !== undefined) {
    parts.push(`<div><span class="field">document</span> <span class="mono">${escape(item.previous_hash.slice(0, 12) || '—')}</span>
      <span class="arrow">→</span> <span class="mono">${escape(item.content_hash.slice(0, 12) || '—')}</span></div>`);
  }
  if (item.bom_diff && item.bom_diff.lines.length) {
    parts.push(`<div><span class="field">BOM</span><ul class="plain">${item.bom_diff.lines.map((l) => `
      <li><strong>${escape(l.number)}</strong> ${escape(l.change)}${l.details.length ? ` — ${escape(l.details.join(", "))}` : ''}</li>`).join('')}</ul></div>`);
  }
  for (const c of item.attribute_changes) {
    parts.push(`<div><span class="field">${escape(c.field)}</span> ${escape(auditValue(c.before))}<span class="arrow">→</span>${escape(auditValue(c.after))}
      <span class="muted">${escape(c.by)} ${escape(when(c.at))}</span></div>`);
  }
  return parts.join('');
}

async function renderEco(id) {
  const eco = await api('GET', `/api/ecos/${encodeURIComponent(id)}`);
  currentEco = eco;
  $('eco-number').textContent = eco.number;
  $('eco-title').textContent = eco.title;
  $('eco-meta').innerHTML = `${ecoChip(eco.state)} · priority ${priorityChip(eco.priority)} · created by ${escape(eco.created_by)} ${escape(when(eco.created_at))}`
    + (eco.released_at ? ` · released by ${escape(eco.released_by)} ${escape(when(eco.released_at))}` : '')
    + (eco.cancelled_at ? ` · cancelled by ${escape(eco.cancelled_by)} ${escape(when(eco.cancelled_at))}` : '');
  $('eco-reason').textContent = eco.reason || '—';
  $('eco-description').textContent = eco.description || '—';
  const open = ['draft', 'inreview', 'approved'].includes(eco.state);
  $('eco-submit').hidden = !(eco.state === 'draft' && me.can_author);
  $('eco-withdraw').hidden = !(['inreview', 'approved'].includes(eco.state) && me.can_author);
  $('eco-release').hidden = !eco.can_release;
  $('eco-edit').hidden = !(open && me.can_author);
  $('eco-cancel').hidden = !(open && me.can_author);
  $('eco-problems').hidden = !(open && eco.problems.length);
  $('eco-problems').innerHTML = eco.problems.length
    ? `<strong>It cannot release yet:</strong><ul>${eco.problems.map((p) => `<li>${escape(p)}</li>`).join('')}</ul>`
    : '';

  $('eco-items').innerHTML = eco.items.map((item) => `
    <tr>
      <td><span class="badge action-${escape(item.action)}">${escape(item.action)}</span></td>
      <td class="number"><a href="#/part/${escape(item.part_id)}">${escape(item.number)}</a><br><span class="muted">${escape(item.name)}</span></td>
      <td><strong>${escape(item.label)}</strong> <span class="state state-${escape(item.lifecycle)}">${escape(item.lifecycle)}</span></td>
      <td>${escape(item.replaces || '—')}</td>
      <td>${itemChanges(item)}${item.problem ? `<div class="error">${escape(item.problem)}</div>` : ''}
        ${itemDetail(item) ? `<details><summary>Details</summary><div class="item-detail">${itemDetail(item)}</div></details>` : ''}</td>
      <td class="actions">${eco.can_edit ? `<button class="link danger" data-remove="${escape(item.revision_id)}">Remove</button>` : ''}</td>
    </tr>`).join('');
  $('eco-items-empty').hidden = eco.items.length > 0;
  for (const button of $('eco-items').querySelectorAll('button[data-remove]')) {
    button.addEventListener('click', () => act(async () => {
      await api('DELETE', `${ecoUrl()}/items/${encodeURIComponent(button.dataset.remove)}`);
      await renderEco(eco.id);
    }));
  }
  $('eco-add').hidden = !eco.can_edit;
  $('eco-add-pick').hidden = true;
  $('eco-add-results').innerHTML = '';
  $('eco-add-search').value = '';

  $('eco-round').innerHTML = eco.current ? roundHtml(eco.current, true, 'eco') : '';
  $('eco-round-none').hidden = !!eco.current;
  const n = eco.rule.required_approvals;
  $('eco-round-none').textContent = `Not submitted yet. ${n ? `It needs ${n} approval${n === 1 ? '' : 's'}` : 'No approval is needed'} before it can release.`;
  $('eco-history').hidden = eco.history.length === 0;
  $('eco-history-list').innerHTML = eco.history.map((r) => roundHtml(r, false, 'eco-old')).join('');
  $('eco-comments').innerHTML = eco.comments.length ? threadHtml(eco.comments, eco.can_comment) : '';
  $('eco-comments-empty').hidden = eco.comments.length > 0;
  $('eco-comment-form').hidden = !eco.can_comment;
  setEcoReplyTo('', '');

  for (const button of $('eco-round').querySelectorAll('button[data-verdict]')) {
    button.addEventListener('click', () => act(async () => {
      const verdict = button.dataset.verdict;
      if (verdict === 'reject' && !confirm('Reject this change order? It goes back to Draft, and the review ends.')) return;
      showWarnings(await api('POST', `${ecoUrl()}/review/decision`, { verdict, comment: $('eco-decision-comment').value }));
      await renderEco(eco.id);
      refreshInbox();
    }));
  }
  const change = $('eco-change');
  if (change) {
    change.addEventListener('click', () => {
      roundTarget = { url: `${ecoUrl()}/review`, refresh: () => renderEco(eco.id) };
      openRoundDialog(eco.current);
    });
  }
  for (const button of $('eco-comments').querySelectorAll('button[data-reply]')) {
    button.addEventListener('click', () => {
      setEcoReplyTo(button.dataset.reply, button.dataset.author);
      $('eco-comment').focus();
    });
  }
}

function setEcoReplyTo(id, author) {
  ecoReplyTo = id;
  $('eco-reply-to').hidden = !id;
  $('eco-reply-to').textContent = id ? `Replying to ${author}` : '';
  $('eco-reply-cancel').hidden = !id;
}

$('eco-reply-cancel').addEventListener('click', () => setEcoReplyTo('', ''));
$('eco-comment-form').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    showWarnings(await api('POST', `${ecoUrl()}/comments`, { body: $('eco-comment').value, parent: ecoReplyTo }));
    $('eco-comment').value = '';
    await renderEco(currentEco.id);
  });
});

$('eco-edit').addEventListener('click', () => act(() => openEcoDialog(currentEco)));

$('eco-submit').addEventListener('click', () => {
  const n = currentEco.rule.required_approvals;
  $('es-title').textContent = currentEco.number;
  $('es-rule').textContent = n ? `It needs ${n} approval${n === 1 ? '' : 's'}.` : 'No approval is needed: it is approved as soon as it is submitted.';
  $('es-reviewers').value = '';
  $('es-note').value = '';
  $('eco-submit-dialog').showModal();
});

$('eco-submit-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'submit') return;
  event.preventDefault();
  act(async () => {
    const body = { note: $('es-note').value };
    const extra = splitList($('es-reviewers').value);
    if (extra.length) body.reviewers = extra;
    const result = await api('POST', `${ecoUrl()}/submit`, body);
    $('eco-submit-dialog').close();
    await renderEco(currentEco.id);
    showWarnings(result);
  });
});

for (const [button, path, question] of [
  ['eco-withdraw', 'withdraw', 'Withdraw this change order? Its review ends, and it goes back to Draft.'],
  ['eco-cancel', 'cancel', 'Cancel this change order? Its revisions are left as they are. This cannot be undone.'],
  ['eco-release', 'release', 'Release this change order? Every item is applied together, and released revisions are immutable.'],
]) {
  $(button).addEventListener('click', () => act(async () => {
    if (!confirm(question)) return;
    const result = await api('POST', `${ecoUrl()}/${path}`);
    await renderEco(currentEco.id);
    showWarnings(result);
  }));
}

// -- adding an item from the change order's page

let ecoAddTimer = null;
let ecoAddPart = null;

$('eco-add-search').addEventListener('input', () => {
  clearTimeout(ecoAddTimer);
  ecoAddTimer = setTimeout(() => act(async () => {
    const q = $('eco-add-search').value.trim();
    if (!q) { $('eco-add-results').innerHTML = ''; return; }
    const page = await api('GET', `/api/parts?q=${encodeURIComponent(q)}&limit=10`);
    $('eco-add-results').innerHTML = page.parts.map((p) => `
      <li><button type="button" class="link" data-pick="${escape(p.id)}">${escape(p.number)}</button> ${escape(p.name)}</li>`).join('')
      || '<li class="muted">No parts match.</li>';
    for (const button of $('eco-add-results').querySelectorAll('button[data-pick]')) {
      button.addEventListener('click', () => act(() => pickEcoPart(button.dataset.pick)));
    }
  }), 150);
});

async function pickEcoPart(id) {
  ecoAddPart = await api('GET', `/api/parts/${encodeURIComponent(id)}`);
  $('eco-add-part').textContent = `${ecoAddPart.number} ${ecoAddPart.name}`;
  $('eco-add-rev').innerHTML = ecoAddPart.revision_views.map((r) => `<option value="${escape(r.id)}">${escape(r.label)} — ${escape(r.lifecycle)}</option>`).join('');
  const views = ecoAddPart.revision_views;
  const open = [...views].reverse().find((r) => r.editable);
  $('eco-add-rev').value = (open || views[views.length - 1]).id;
  syncEcoAddAction();
  $('eco-add-results').innerHTML = '';
  $('eco-add-pick').hidden = false;
}

function syncEcoAddAction() {
  const rev = ecoAddPart && ecoAddPart.revision_views.find((r) => r.id === $('eco-add-rev').value);
  if (rev) $('eco-add-action').value = rev.editable ? 'release' : 'obsolete';
}

$('eco-add-rev').addEventListener('change', syncEcoAddAction);

$('eco-add').addEventListener('submit', (event) => {
  event.preventDefault();
  act(async () => {
    await api('POST', `${ecoUrl()}/items`, {
      part: ecoAddPart.id,
      revision: $('eco-add-rev').value,
      action: $('eco-add-action').value,
      note: $('eco-add-note').value,
    });
    $('eco-add-note').value = '';
    await renderEco(currentEco.id);
  });
});

// -- adding an item from a part's page

let addingRevision = null;

async function openAddToEco(revisionId) {
  addingRevision = currentPart.revision_views.find((r) => r.id === revisionId);
  const drafts = await api('GET', '/api/ecos?state=draft');
  $('ae-title').textContent = `${currentPart.number} rev ${addingRevision.label}`;
  $('ae-eco').innerHTML = drafts.map((e) => `<option value="${escape(e.id)}">${escape(e.number)} — ${escape(e.title)}</option>`).join('')
    + '<option value="">A new change order…</option>';
  $('ae-new-title').value = '';
  $('ae-action').value = addingRevision.editable ? 'release' : 'obsolete';
  syncAddToEco();
  $('add-to-eco-dialog').showModal();
}

function syncAddToEco() {
  $('ae-new-label').hidden = $('ae-eco').value !== '';
}

$('ae-eco').addEventListener('change', syncAddToEco);

$('add-to-eco-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'save') return;
  event.preventDefault();
  act(async () => {
    let id = $('ae-eco').value;
    if (!id) {
      id = (await api('POST', '/api/ecos', { title: $('ae-new-title').value || `Change ${currentPart.number}` })).id;
    }
    await api('POST', `/api/ecos/${encodeURIComponent(id)}/items`, {
      part: currentPart.id,
      revision: addingRevision.id,
      action: $('ae-action').value,
    });
    $('add-to-eco-dialog').close();
    await renderPart(currentPart.id);
  });
});

// -- settings

function syncEcoModeFields() {
  const mode = $('set-eco-mode').value;
  $('set-eco-regex-label').hidden = mode !== 'pattern';
  $('set-eco-script-label').hidden = mode !== 'script';
  $('set-eco-counter').hidden = mode !== 'counter';
  const digits = Math.max(1, Number($('set-eco-digits').value) || 1);
  $('set-eco-preview').textContent = mode === 'counter'
    ? `${$('set-eco-prefix').value}${String(Math.max(1, Number($('set-eco-next').value) || 1)).padStart(digits, '0')}`
    : 'typed or decided by the script';
}

for (const id of ['set-eco-mode', 'set-eco-prefix', 'set-eco-digits', 'set-eco-next']) {
  $(id).addEventListener('input', syncEcoModeFields);
}

async function renderEcoSettings(settings) {
  const rule = settings.eco_review_rule;
  $('set-eco-required').value = rule.required_approvals;
  $('set-eco-due').value = rule.due_days;
  $('set-eco-reviewers').value = reviewersText(rule.reviewers, reviewNames);
  $('set-eco-self').checked = rule.allow_self_approval;
  $('set-eco-holds').checked = settings.eco_holds_revisions;
  const numbering = await api('GET', '/api/ecos/numbering');
  $('set-eco-mode').value = numbering.mode.kind;
  $('set-eco-regex').value = numbering.mode.regex || '';
  $('set-eco-script').value = numbering.mode.script || '';
  $('set-eco-prefix').value = numbering.prefix;
  $('set-eco-digits').value = numbering.digits;
  $('set-eco-next').value = numbering.next;
  syncEcoModeFields();
}

function readEcoNumbering() {
  const kind = $('set-eco-mode').value;
  const mode = { kind };
  if (kind === 'pattern') mode.regex = $('set-eco-regex').value;
  if (kind === 'script') mode.script = $('set-eco-script').value;
  return {
    mode,
    prefix: $('set-eco-prefix').value,
    digits: Number($('set-eco-digits').value),
    next: Number($('set-eco-next').value),
  };
}

// --------------------------------------------------------------- workspace
//
// A user's folders of links and files (D12, the server's P6). The CAD app's File > Open
// shows the same workspace. Delete removes a link or a file, never a part (D9).

const wsState = { owner: '', ownerLabel: 'My workspace', path: [], entries: [], cut: null, versionsOf: null, list: null };

const wsReadOnly = () => wsState.owner !== '';
const wsFolder = () => (wsState.path.length ? wsState.path[wsState.path.length - 1].id : '');
const wsEntry = (id) => wsState.entries.find((e) => e.id === id);

async function renderWorkspace() {
  wsState.list = await api('GET', '/api/workspaces');
  $('ws-owner').innerHTML = wsState.list.workspaces.map((w) => {
    const value = w.mine ? '' : w.user_id;
    const label = w.mine ? 'My workspace' : `${w.display_name || w.username}'s workspace`;
    return `<option value="${escape(value)}"${value === wsState.owner ? ' selected' : ''}>${escape(label)}</option>`;
  }).join('');
  $('ws-private').hidden = wsState.list.browsable;
  await wsLoad();
}

async function wsLoad() {
  $('ws-status').textContent = '';
  const query = new URLSearchParams({ owner: wsState.owner, parent: wsFolder() });
  const folder = await api('GET', `/api/workspace/entries?${query}`);
  wsState.entries = folder.entries;
  const writable = !wsReadOnly();
  $('ws-readonly').hidden = writable;
  for (const el of document.querySelectorAll('#view-workspace .ws-write')) el.hidden = !writable;
  $('ws-crumbs').innerHTML = [`<button class="link" data-ws-crumb="0">${escape(wsState.ownerLabel)}</button>`]
    .concat(wsState.path.map((p, i) => `<button class="link" data-ws-crumb="${i + 1}">${escape(p.name)}</button>`))
    .join(' / ');
  $('ws-cut').hidden = !wsState.cut || !writable;
  $('ws-cut-name').textContent = wsState.cut ? wsState.cut.name : '';
  $('ws-empty').hidden = folder.entries.length > 0;
  $('ws-wrap').hidden = folder.entries.length === 0;
  $('ws-rows').innerHTML = folder.entries.map(wsRow).join('');
  if (wsState.versionsOf && !wsEntry(wsState.versionsOf)) {
    wsState.versionsOf = null;
    $('ws-versions').hidden = true;
  }
}

function wsWhat(e) {
  if (e.kind === 'folder') return `folder · ${e.children ?? 0} item${e.children === 1 ? '' : 's'}`;
  if (e.kind === 'file' && e.file) return `file · v${e.file.version} of ${e.file.versions} · ${fileSize(e.file.size)}`;
  const l = e.link || {};
  if (l.part_missing) return '<span class="error">the linked part is gone</span>';
  const part = `<a href="#/part/${encodeURIComponent(l.part_id)}">${escape(l.number)}</a> ${escape(l.name)}`;
  if (l.revision_missing) return `${part} · <span class="error">the pinned revision was deleted</span>`;
  return `${part} · rev ${escape(l.revision_label)} (${escape(l.lifecycle)}, ${l.pinned ? 'pinned' : 'follows the newest'})`;
}

function wsRow(e) {
  const writable = !wsReadOnly();
  const id = escape(e.id);
  const actions = [];
  if (e.kind === 'folder') actions.push(`<button class="link" data-ws-open="${id}">Open</button>`);
  if (e.kind === 'link' && e.link && !e.link.part_missing && !e.link.revision_missing) {
    actions.push(`<a href="#/part/${encodeURIComponent(e.link.part_id)}">Part</a>`);
    if (typeof openInCadButton === 'function') actions.push(openInCadButton(e.link.part_id, e.link.revision_id));
    if (writable) {
      actions.push(e.link.pinned
        ? `<button class="link" data-ws-follow="${id}">Follow newest</button>`
        : `<button class="link" data-ws-pin="${id}" data-rev="${escape(e.link.revision_id)}">Pin to ${escape(e.link.revision_label)}</button>`);
    }
  }
  if (e.kind === 'file') {
    actions.push(`<a href="/api/workspace/entries/${encodeURIComponent(e.id)}/content" download="${escape(e.name)}">Download</a>`);
    actions.push(`<button class="link" data-ws-versions="${id}">Versions</button>`);
    if (writable && me.can_author) actions.push(`<button class="link" data-ws-promote="${id}">Promote…</button>`);
  }
  if (writable) {
    actions.push(`<button class="link" data-ws-rename="${id}">Rename</button>`);
    actions.push(`<button class="link" data-ws-move="${id}">Move…</button>`);
    const verb = e.kind === 'link' ? 'Remove link' : 'Delete';
    actions.push(`<button class="link danger" data-ws-delete="${id}">${verb}</button>`);
  }
  const name = e.kind === 'folder'
    ? `<button class="link" data-ws-open="${id}">${escape(e.name)}/</button>`
    : escape(e.name);
  return `
    <tr data-ws-row="${id}">
      <td class="thumb-cell">${e.kind === 'link' ? thumb(e.link?.thumbnail_url) : ''}</td>
      <td>${name}</td>
      <td>${wsWhat(e)}</td>
      <td class="muted">${escape(when(e.modified_at))}</td>
      <td class="actions">${actions.join(' ')}</td>
    </tr>`;
}

async function wsUpdate(id, fields) {
  await api('PATCH', `/api/workspace/entries/${encodeURIComponent(id)}`, fields);
  await wsLoad();
}

$('ws-owner').addEventListener('change', () => act(async () => {
  wsState.owner = $('ws-owner').value;
  wsState.ownerLabel = $('ws-owner').selectedOptions[0]?.textContent || 'My workspace';
  wsState.path = [];
  wsState.versionsOf = null;
  $('ws-versions').hidden = true;
  await wsLoad();
}));

$('ws-reload').addEventListener('click', () => act(renderWorkspace));

$('ws-crumbs').addEventListener('click', (event) => {
  const crumb = event.target.closest('button[data-ws-crumb]');
  if (!crumb) return;
  wsState.path = wsState.path.slice(0, Number(crumb.dataset.wsCrumb));
  act(wsLoad);
});

$('ws-rows').addEventListener('click', (event) => {
  const button = event.target.closest('button');
  if (!button) return;
  const d = button.dataset;
  if (d.wsOpen) {
    const folder = wsEntry(d.wsOpen);
    wsState.path.push({ id: folder.id, name: folder.name });
    act(wsLoad);
  } else if (d.wsRename) {
    const entry = wsEntry(d.wsRename);
    const name = prompt(`Rename ${entry.name} to`, entry.name);
    if (name && name.trim() && name !== entry.name) act(() => wsUpdate(entry.id, { name: name.trim() }));
  } else if (d.wsMove) {
    wsState.cut = wsEntry(d.wsMove);
    act(wsLoad);
  } else if (d.wsDelete) {
    const entry = wsEntry(d.wsDelete);
    const question = entry.kind === 'link'
      ? `Remove the link ${entry.name}? The part and its revisions are not touched.`
      : entry.kind === 'folder'
        ? `Delete the folder ${entry.name} and everything in it? Parts are not touched.`
        : `Delete ${entry.name} and every version of it?`;
    if (!confirm(question)) return;
    act(async () => {
      await api('DELETE', `/api/workspace/entries/${encodeURIComponent(entry.id)}?recursive=${entry.kind === 'folder'}`);
      await wsLoad();
    });
  } else if (d.wsPin) {
    act(() => wsUpdate(d.wsPin, { revision: d.rev }));
  } else if (d.wsFollow) {
    act(() => wsUpdate(d.wsFollow, { revision: '' }));
  } else if (d.wsVersions) {
    wsState.versionsOf = d.wsVersions;
    act(wsVersions);
  } else if (d.wsPromote) {
    const entry = wsEntry(d.wsPromote);
    $('ws-promote-form').dataset.id = entry.id;
    $('ws-promote-name').textContent = entry.name;
    $('ws-promote-dialog').showModal();
  }
});

$('ws-paste').addEventListener('click', () => act(async () => {
  const cut = wsState.cut;
  wsState.cut = null;
  if (cut) await wsUpdate(cut.id, { parent: wsFolder() });
}));

$('ws-cut-cancel').addEventListener('click', () => {
  wsState.cut = null;
  act(wsLoad);
});

$('ws-new-folder').addEventListener('click', () => act(async () => {
  const name = $('ws-folder-name').value.trim();
  if (!name) return;
  await api('POST', '/api/workspace/folders', { parent: wsFolder(), name });
  $('ws-folder-name').value = '';
  await wsLoad();
}));

$('ws-link').addEventListener('click', () => act(async () => {
  const part = $('ws-link-part').value.trim();
  if (!part) return;
  await api('POST', '/api/workspace/links', { parent: wsFolder(), part, revision: $('ws-link-rev').value.trim() });
  $('ws-link-part').value = '';
  $('ws-link-rev').value = '';
  await wsLoad();
}));

async function wsVersions() {
  const entry = wsEntry(wsState.versionsOf);
  if (!entry) return;
  const { versions } = await api('GET', `/api/workspace/entries/${encodeURIComponent(entry.id)}/versions`);
  $('ws-versions-name').textContent = entry.name;
  $('ws-versions').hidden = false;
  const base = `/api/workspace/entries/${encodeURIComponent(entry.id)}`;
  $('ws-version-rows').innerHTML = versions.slice().reverse().map((v) => `
    <tr>
      <td>v${v.version}${v.current ? ' (current)' : ''}${v.restored_from ? ` <span class="muted">restored from v${v.restored_from}</span>` : ''}</td>
      <td class="num">${fileSize(v.size)}</td>
      <td>${escape(v.uploaded_by_name)}</td>
      <td class="muted">${escape(when(v.uploaded_at))}</td>
      <td class="actions"><a href="${base}/content?version=${v.version}" download="${escape(entry.name)}">Download</a>
        ${!v.current && !wsReadOnly() ? `<button class="link" data-ws-restore="${v.version}">Restore</button>` : ''}</td>
    </tr>`).join('');
}

$('ws-version-rows').addEventListener('click', (event) => {
  const restore = event.target.closest('button[data-ws-restore]');
  if (!restore) return;
  act(async () => {
    await api('POST', `/api/workspace/entries/${encodeURIComponent(wsState.versionsOf)}/versions/${restore.dataset.wsRestore}/restore`);
    await wsLoad();
    await wsVersions();
  });
});

async function wsUpload(files) {
  let done = 0;
  for (const file of files) {
    $('ws-status').textContent = `Uploading ${file.name} (${fileSize(file.size)})…`;
    // A file whose name is already here becomes its next version.
    const same = wsState.entries.find((e) => e.kind === 'file' && e.name.toLowerCase() === file.name.toLowerCase());
    const url = same
      ? `/api/workspace/entries/${encodeURIComponent(same.id)}/content`
      : `/api/workspace/files?${new URLSearchParams({ parent: wsFolder(), name: file.name })}`;
    const response = await fetch(url, {
      method: same ? 'PUT' : 'POST',
      credentials: 'same-origin',
      headers: { 'Content-Type': file.type || 'application/octet-stream', 'X-CSRF-Token': csrfToken || '' },
      body: file,
    });
    if (!response.ok) {
      let message = `${response.status} ${response.statusText}`;
      try { message = (await response.json()).error || message; } catch { /* not JSON */ }
      $('ws-status').textContent = '';
      await wsLoad();
      throw new Error(`${file.name}: ${message}`);
    }
    done += 1;
  }
  await wsLoad();
  $('ws-status').textContent = done ? `Added ${done} file${done === 1 ? '' : 's'}.` : '';
  if (wsState.versionsOf) await wsVersions();
}

$('ws-file').addEventListener('change', () => {
  const files = [...$('ws-file').files];
  $('ws-file').value = '';
  if (files.length) act(() => wsUpload(files));
});

for (const name of ['dragenter', 'dragover']) {
  $('ws-drop').addEventListener(name, (event) => {
    if (wsReadOnly()) return;
    event.preventDefault();
    $('ws-drop').classList.add('over');
  });
}
for (const name of ['dragleave', 'drop']) {
  $('ws-drop').addEventListener(name, () => $('ws-drop').classList.remove('over'));
}
$('ws-drop').addEventListener('drop', (event) => {
  if (wsReadOnly()) return;
  event.preventDefault();
  const files = [...event.dataTransfer.files];
  if (files.length) act(() => wsUpload(files));
});

$('ws-promote-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'promote') return;
  event.preventDefault();
  act(async () => {
    const id = $('ws-promote-form').dataset.id;
    const done = await api('POST', `/api/workspace/entries/${encodeURIComponent(id)}/promote`, {
      part: $('ws-promote-part').value.trim(),
      revision: $('ws-promote-rev').value.trim(),
      kind: $('ws-promote-kind').value,
    });
    $('ws-promote-dialog').close();
    $('ws-status').textContent = `Promoted to an attachment: ${done.attachment.name} (${done.attachment.kind}).`;
  });
});

// "Add to workspace…" on a part's page: pick one of your folders, and follow the newest
// revision or pin one.
const wsPick = { path: [], purpose: null };

async function wsPickLoad() {
  const parent = wsPick.path.length ? wsPick.path[wsPick.path.length - 1].id : '';
  const folder = await api('GET', `/api/workspace/entries?${new URLSearchParams({ parent })}`);
  $('ws-pick-crumbs').innerHTML = [`<button type="button" class="link" data-ws-pick-crumb="0">My workspace</button>`]
    .concat(wsPick.path.map((p, i) => `<button type="button" class="link" data-ws-pick-crumb="${i + 1}">${escape(p.name)}</button>`))
    .join(' / ');
  const folders = folder.entries.filter((e) => e.kind === 'folder');
  $('ws-pick-list').innerHTML = folders.length
    ? folders.map((f) => `<li><button type="button" class="link" data-ws-pick="${escape(f.id)}" data-name="${escape(f.name)}">${escape(f.name)}/</button></li>`).join('')
    : '<li class="muted">No folders here.</li>';
}

$('ws-pick-crumbs').addEventListener('click', (event) => {
  const crumb = event.target.closest('button[data-ws-pick-crumb]');
  if (!crumb) return;
  wsPick.path = wsPick.path.slice(0, Number(crumb.dataset.wsPickCrumb));
  act(wsPickLoad);
});

$('ws-pick-list').addEventListener('click', (event) => {
  const folder = event.target.closest('button[data-ws-pick]');
  if (!folder) return;
  wsPick.path.push({ id: folder.dataset.wsPick, name: folder.dataset.name });
  act(wsPickLoad);
});

$('part-add-ws').addEventListener('click', () => act(async () => {
  if (!currentPart) return;
  wsPick.path = [];
  $('part-ws-status').textContent = '';
  $('ws-pick-title').textContent = `Add ${currentPart.number} to your workspace`;
  $('ws-pick-rev').innerHTML = ['<option value="">follow the newest revision</option>']
    .concat(currentPart.revision_views.slice().reverse().map((r) => `<option value="${escape(r.id)}">pin to ${escape(r.label)} (${escape(r.lifecycle)})</option>`))
    .join('');
  await wsPickLoad();
  $('ws-pick-dialog').showModal();
}));

$('ws-pick-form').addEventListener('submit', (event) => {
  if (event.submitter && event.submitter.value !== 'choose') return;
  event.preventDefault();
  act(async () => {
    const parent = wsPick.path.length ? wsPick.path[wsPick.path.length - 1].id : '';
    await api('POST', '/api/workspace/links', { parent, part: currentPart.id, revision: $('ws-pick-rev').value, name: $('ws-pick-name').value.trim() });
    $('ws-pick-dialog').close();
    $('ws-pick-name').value = '';
    const where = wsPick.path.length ? wsPick.path.map((p) => p.name).join(' / ') : 'the top of your workspace';
    $('part-ws-status').textContent = `Added to ${where}.`;
  });
});
