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
let lifecycleOptions = [];
const statusName = state => lifecycleOptions.find(s => s.state === state)?.name || state;
const statusEnabled = state => lifecycleOptions.find(s => s.state === state)?.enabled !== false;

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

const VIEWS = ['workflows', 'workflow-editor', 'workflow-run', 'setup', 'file', 'empty', 'native-import', 'parts', 'workspace', 'part', 'reviews', 'ecos', 'eco', 'catalog', 'sourcing', 'bake', 'bom-columns', 'occurrence-fields', 'types', 'users', 'scripts', 'settings', 'account', 'security', 'audit', 'backups'];

let visiblePage = null;
function show(view) {
  const names = { workflows: 'Workflows', 'workflow-editor': 'Workflow editor', 'workflow-run': 'Workflow process', setup: 'Server setup', 'native-import': 'Import native files', parts: 'Parts', part: currentPart?.number || 'Part', workspace: 'Home', reviews: 'Review inbox', ecos: 'Change orders', eco: 'Change order', catalog: 'Classification', sourcing: 'Sourcing', bake: 'Bake queue', 'bom-columns':'BOM column configurations', 'occurrence-fields':'Occurrence fields', types: 'Part types', users: 'Users', scripts: 'Scripts', settings: 'Settings', account: 'My account', security: 'Security', audit: 'Audit', backups: 'Backups' };
  visiblePage = {view, title:names[view] || view};
  for(const menu of document.querySelectorAll('.top-nav details[open]'))menu.open=false;
  $('shell-status').textContent = names[view] || 'Ready';
  for (const name of VIEWS) $(`view-${name}`).hidden = name !== view;
  for (const link of document.querySelectorAll('.top-nav a')) {
    const current = link.getAttribute('href') === `#/${view === 'part' ? 'parts' : view === 'eco' ? 'ecos' : view}`;
    link.classList.toggle('current', current);
    if (current) link.setAttribute('aria-current', 'page');
    else link.removeAttribute('aria-current');
  }
}

let routeQueue = Promise.resolve();
function route() {
  for (const menu of document.querySelectorAll('.command-menu[open]')) menu.open = false;
  const hash=location.hash || '#/workspace';
  routeQueue=routeQueue.catch(()=>{}).then(async()=>{
    if(!me)return;
    OpenPages.capture();
    document.querySelector('.work-area').inert=true;
    try { await renderRoute(hash); }
    finally { document.querySelector('.work-area').inert=false; }
  });
  return routeQueue;
}
async function renderRoute(hash) {
  if (!me) return;
  banner('');
  const [, section, rawId, rawRevision, objectView] = hash.split('/');
  const id = rawId && decodeURIComponent(rawId);
  const revision = rawRevision && decodeURIComponent(rawRevision);
  try {
    await OpenPages.beforeRender(hash);
    const cached=OpenPages.resume(hash);
    if(cached){show(cached.view);}
    else if(section==='attachment'||section==='file'){const title=await FileDocuments.open(hash);show('file');visiblePage.title=title;}
    else if (section === 'empty') { show('empty');
    } else if (section === 'part' && id) {
      await renderPart(id, revision);
      if (OBJECT_TABS.some(([tab]) => tab === objectView)) selectObjectTab(objectView);
      show('part');
    } else if (section === 'native-import') {
      if (!me.can_author) { location.hash = '#/parts'; return; }
      await openNativeImport();
      show('native-import');
    } else if (section === 'workspace') {
      Workbench.restore(OpenPages.workspace(hash));
      await Workbench.home(id);
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
    } else if(section==='bom-columns'){await renderBomConfigurations();show('bom-columns');
    } else if(section==='occurrence-fields'){await renderOccurrenceFields();show('occurrence-fields');
    } else if (section === 'types') {
      await renderTypes();
      show('types');
    } else if (section === 'scripts') {
      if (!me.is_admin) { location.hash = '#/parts'; return; }
      if(id) {
        const path=[rawId,rawRevision,objectView].filter(Boolean).map(decodeURIComponent).join('/');
        const draft=OpenPages.scriptDraft(hash);
        scriptPath=null;scriptSaved='';
        await openScript(path, draft?.text, true);
        if(draft){scriptSaved=draft.saved;syncScriptButtons();}
      } else {scriptPath=null;scriptSaved='';$('script-text').value='';$('script-path').textContent='No file open';await renderScripts();}
      show('scripts');
    } else if (section === 'workflows') {
      await WorkflowUI.list(); show('workflows');
    } else if (section === 'workflow-editor') {
      if (!me.is_admin) { location.hash = '#/workflows'; return; }
      await WorkflowUI.editor(id || 'new'); show('workflow-editor');
    } else if (section === 'workflow-run' && id) {
      await WorkflowUI.detail(id); show('workflow-run');
    } else if (section === 'setup') {
      if (!me.is_admin) { location.hash = '#/workspace'; return; }
      await SetupWizard.open();
      show('setup');
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
      if (id === 'password') {
        $('password-form').scrollIntoView({ block: 'start' });
        $('pw-current').focus({ preventScroll: true });
      }
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
    let title=visiblePage.title;
    if(visiblePage.view==='part')title=currentPart.number+(revision?'/'+(currentPart.revisions.find(r=>r.id===revision||r.label===revision)?.label || revision):'');
    if(visiblePage.view==='workspace'&&id)title=$('home-detail').querySelector('h2')?.textContent || 'Folder';
    if(visiblePage.view==='scripts'&&id)title=scriptPath;
    if(visiblePage.view==='eco')title=$('eco-number').textContent;
    if(visiblePage.view==='catalog'&&id)title=$('cat-title').textContent || title;
    await OpenPages.afterRender(hash);
    OpenPages.activate(hash,visiblePage.view,title);
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
const idSegment = value => encodeURIComponent(value).replace(/[.!'()*~]/g, c => '%' + c.charCodeAt(0).toString(16).toUpperCase());
const documentKey = (part, revision) => `part/${idSegment(part)}/rev/${idSegment(revision)}`;

function openInCadButton(partId, revisionId, label = 'Open in CAD') {
  if (!cadApp || !partId || !revisionId) return '';
  const key = documentKey(partId, revisionId);
  return `<button type="button" data-open-cad="${escape(key)}" title="Open this revision in the CAD app, in a new tab">${escape(label)}</button>`;
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

  const cadReturn = new URLSearchParams(location.search).get('cad');
  if (cadReturn) {
    let target;
    try { target = new URL(cadReturn, location.origin); } catch { /* Ignore malformed return paths. */ }
    if (target && target.origin === location.origin && target.pathname.startsWith('/cad/app/')) {
      location.replace(target.href);
      return;
    }
  }

  $('user-display-name').textContent = me.display_name || me.username;
  $('whoami').textContent = `${me.username} · ${me.groups.join(', ') || 'no groups'}`;
  $('user-menu-trigger').setAttribute('aria-label', `User account menu for ${me.display_name || me.username}`);
  for (const el of document.querySelectorAll('.admin-only')) el.hidden = !me.is_admin;
  for (const el of document.querySelectorAll('.author-only')) el.hidden = !me.can_author;
  const setupSettings = await api('GET', '/api/settings');
  lifecycleOptions = setupSettings.status_options;
  if (me.is_admin && !setupSettings.setup_completed && !location.hash) location.hash = '#/setup';
  await loadCadConfig();
  OpenPages.restoreSession();
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

// A page replaces the previous rows, keeping DOM size bounded.
const PARTS_PAGE = 100; // Chunk size for explicitly selecting all matching parts.
let partsCursors = [null], partsPageIndex = 0, partsPageBusy = false, partsRestorePage = null;
// Bumped by every fresh render, so a slow page for an older search cannot
// land in the list after a newer one.
let partsGeneration = 0;
let partsNext = null;
let partsQuery = '';
let listedParts = [];
let selectedPartId = null;
const selectedParts = new Map();
let partsSelectionAnchor = null, partsBatchRunning = false, partsSelecting = false;
let partsBatchReport = null, partsBatchUser = null, partsBatchPolling = false, partsBatchOffset = 0;

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
  if ($('filter-geometry').value) params.set('attr.has_geometry', $('filter-geometry').value);
  if ($('filter-thumbnail').value) params.set('attr.has_thumbnail', $('filter-thumbnail').value);
  partsQuery = params.toString();
  partsCursors = partsRestorePage?.cursors || [null];
  partsPageIndex = partsRestorePage?.index || 0;
  partsRestorePage = null;
  partsSelectionAnchor = null;
  await loadPartsPage(partsPageIndex);
}

function updatePartsPager() {
  $('parts-prev').disabled = partsPageBusy || partsPageIndex === 0;
  $('parts-more').disabled = partsPageBusy || !partsNext;
  $('parts-page-size').disabled = partsPageBusy;
  $('parts-page-status').textContent = `Page ${partsPageIndex + 1}`;
}

async function loadPartsPage(index) {
  const generation = ++partsGeneration;
  const cursor = partsCursors[index];
  const size = Number($('parts-page-size').value);
  partsPageBusy = true; updatePartsPager();
  try {
  const page = await api('GET', `/api/parts?${partsQuery}&limit=${size}${cursor ? `&after=${encodeURIComponent(cursor)}` : ''}`);
  if (generation !== partsGeneration) return;
  $('parts-empty').hidden = page.parts.length > 0;
  $('parts-empty').textContent = 'No parts match.';
  partsPageIndex = index; partsSelectionAnchor = null;
  listedParts = page.parts;
  $('parts-rows').innerHTML = page.parts.map(partRow).join('');
  partsNext = page.next;
  partsCursors[index + 1] = page.next;
  updatePartsSummary();
  } finally { if (generation === partsGeneration) { partsPageBusy = false; updatePartsPager(); updatePartSelection(); } }
}

async function moreParts() {
  if (!partsNext || partsPageBusy) return;
  await loadPartsPage(partsPageIndex + 1);
}

$('parts-more').addEventListener('click', () => act(moreParts));
$('parts-prev').addEventListener('click', () => act(() => loadPartsPage(partsPageIndex - 1)));
$('parts-page-size').addEventListener('change', () => act(renderParts));

function partRow(part) {
  return `
    <tr data-part-id="${escape(part.id)}">
      <td><input type="checkbox" class="part-select" aria-label="Select ${escape(part.number)}"${selectedParts.has(part.id)?' checked':''}></td>
      <td class="thumb-cell"><span class="part-tile-preview"><a href="#/part/${encodeURIComponent(part.id)}" tabindex="-1">${thumb(part.thumbnail_url)}</a>${part.locked ? `<span class="part-tile-lock" role="img" aria-label="Checked out" title="Checked out">${PART_ATTRIBUTE_LOCK_ICON}</span>` : ''}</span></td>
      <td class="number"><div class="part-tile-identity"><a href="#/part/${encodeURIComponent(part.id)}">${escape(part.number)}/${escape(part.latest_label)}</a><span class="state state-${escape(part.latest_state)}">${escape(statusName(part.latest_state))}</span></div></td>
      <td><span class="part-tile-name">${escape(part.name)}</span>${part.document_class && part.document_class !== 'normal' ? classBadge(part.document_class) : ''}${part.tags.length ? tagChips(part.tags) : ''}</td>
      <td>${escape(part.part_type)}</td>
      <td>${categoryText(part)}</td>
      <td class="mono">${escape(part.mpn) || '<span class="muted">—</span>'}</td>
      <td>${escape(part.latest_label)}</td>
      <td><span class="state state-${escape(part.latest_state)}">${escape(statusName(part.latest_state))}</span></td>
      <td class="actions"><button class="link inspect-part" aria-label="Inspect ${escape(part.number)}" aria-pressed="false">Properties</button></td>
    </tr>`;
}

function updatePartsSummary() {
  const first = partsPageIndex * Number($('parts-page-size').value) + 1;
  $('parts-count').textContent = listedParts.length ? `Showing ${first}–${first + listedParts.length - 1}` : '0 parts';
  refreshPartsBatch();
  updatePartSelection();
  selectPartSummary(selectedPartId);
}

function selectPartSummary(id, reveal = false) {
  const part = listedParts.find(p => p.id === id);
  selectedPartId = part ? id : null;
  for (const row of $('parts-rows').rows) {
    const selected = row.dataset.partId === selectedPartId;
    row.classList.toggle('selected', selectedParts.has(row.dataset.partId));
    row.querySelector('.inspect-part').setAttribute('aria-pressed', String(selected));
  }
  if (!part) {
    $('selection-content').innerHTML = '<p class="selection-empty">Select a row to inspect its properties. Open a part number to work with its revisions.</p>';
    return;
  }
  return Workbench.catalog(part.id, reveal);
}

$('parts-rows').addEventListener('click', event => {
  if (event.target.closest('button.tag')) return;
  const row = event.target.closest('tr[data-part-id]');
  if(!row)return;
  const id=row.dataset.partId,index=listedParts.findIndex(part=>part.id===id);
  const checkbox=event.target.closest('.part-select');
  if(partsBatchRunning||partsSelecting){if(checkbox)event.preventDefault();return;}
  if(event.shiftKey&&partsSelectionAnchor!==null) {
    for(const part of listedParts.slice(Math.min(index,partsSelectionAnchor),Math.max(index,partsSelectionAnchor)+1))selectedParts.set(part.id,part.number);
  } else if(checkbox||event.ctrlKey||event.metaKey) {
    if(selectedParts.has(id))selectedParts.delete(id);else selectedParts.set(id,listedParts[index].number);
  }
  partsSelectionAnchor=index;updatePartSelection();
  if(!checkbox){event.preventDefault();selectPartSummary(id,true);}
});


$('parts-rows').addEventListener('dblclick', event => {
  if (partsBatchRunning || partsSelecting || event.ctrlKey || event.metaKey || event.shiftKey
      || event.target.closest('input, select, textarea, button, label')) return;
  const row = event.target.closest('tr[data-part-id]');
  const part = listedParts.find(part => part.id === row?.dataset.partId);
  if (!part) return;
  event.preventDefault();
  location.hash = `#/part/${encodeURIComponent(part.id)}/${encodeURIComponent(part.latest_revision_id)}/summary`;
});

function updatePartSelection() {
  const busy=partsBatchRunning||partsSelecting||partsPageBusy;
  $('parts-selection-count').textContent=`${selectedParts.size} selected`;
  let loadedSelected=0;
  for(const row of $('parts-rows').rows) {
    const selected=selectedParts.has(row.dataset.partId);
    row.classList.toggle('selected',selected);
    const checkbox=row.querySelector('.part-select');checkbox.checked=selected;checkbox.disabled=busy;
    if(selected)loadedSelected++;
  }
  $('parts-select-all').checked=listedParts.length>0&&loadedSelected===listedParts.length;
  $('parts-select-all').indeterminate=loadedSelected>0&&loadedSelected<listedParts.length;
  $('parts-select-all').disabled=busy||!listedParts.length;
  $('parts-select-matching').disabled=busy;
  $('parts-clear-selection').disabled=busy||!selectedParts.size;
  $('parts-force-resave').disabled=busy||!selectedParts.size||!me?.can_author;
}
$('parts-select-all').addEventListener('change',()=>{
  for(const part of listedParts)if($('parts-select-all').checked)selectedParts.set(part.id,part.number);else selectedParts.delete(part.id);
  updatePartSelection();
});
$('parts-clear-selection').addEventListener('click',()=>{selectedParts.clear();partsSelectionAnchor=null;updatePartSelection();});
$('parts-select-matching').addEventListener('click',()=>act(async()=>{
  partsSelecting=true;updatePartSelection();
  const query=partsQuery,generation=partsGeneration;let next=null;
  try {
    selectedParts.clear();
    do {
      const page=await api('GET',`/api/parts?${query}&limit=${PARTS_PAGE}${next?`&after=${encodeURIComponent(next)}`:''}`);
      if(generation!==partsGeneration)return;
      for(const part of page.parts)selectedParts.set(part.id,part.number);
      next=page.next;updatePartSelection();
    } while(next);
  } finally {partsSelecting=false;updatePartSelection();}
}));
$('parts-force-resave').addEventListener('click',()=>act(forceResaveSelectedParts));
function restorePartBatch() {
  if(partsBatchUser===me.id)return;
  partsBatchUser=me.id;partsBatchReport=null;partsBatchOffset=0;
  try {partsBatchReport=JSON.parse(localStorage.getItem(`plm.parts.resave:${me.id}`));}catch {}
}
async function forceResaveSelectedParts() {
  if(partsBatchRunning||!selectedParts.size)return;
  partsBatchRunning=true;updatePartSelection();
  $('parts-batch-status').textContent='Adding selected parts to the bake queue…';
  const labels=new Map(selectedParts);
  try {
    const report=await api('POST','/api/bake/resave',{parts:[...labels.keys()]});
    partsBatchOffset=0;
    partsBatchReport={...report,jobs:report.jobs.map(job=>({id:job.id})),failures:report.failures.map(f=>({...f,number:labels.get(f.part_id)||f.part_id}))};
    partsBatchUser=me.id;
    try {localStorage.setItem(`plm.parts.resave:${me.id}`,JSON.stringify(partsBatchReport));}catch {}
    const counts={pending:0,claimed:0,done:0,failed:0,waiting:0};
    for(const job of report.jobs){counts[job.status]++;if(job.status==='pending'&&job.waiting_on_checkout)counts.waiting++;}
    drawPartsBatch({jobs:report.jobs.slice(0,50),total:report.jobs.length,counts});
  } finally {partsBatchRunning=false;updatePartSelection();}
}
function drawPartsBatch(page) {
  if(!partsBatchReport)return;
  const {counts}=page,failures=partsBatchReport.failures;
  const total=page.total+failures.length;
  partsBatchOffset=Math.min(partsBatchOffset,Math.floor(Math.max(0,total-1)/50)*50);
  const tracked=partsBatchOffset<page.total?page.jobs:[];
  const shownFailures=failures.slice(Math.max(0,partsBatchOffset-page.total),Math.max(0,partsBatchOffset+50-page.total));
  $('parts-batch-status').textContent=`${counts.pending} queued, ${counts.claimed} running, ${counts.done} completed, ${counts.failed+failures.length} failed${counts.waiting?`, ${counts.waiting} waiting for checkout`:''}.`;
  $('parts-batch-results').hidden=false;
  $('parts-batch-result-list').innerHTML=tracked.map(job=>`<li${job.status==='failed'?' class="error"':''}>${escape(job.number)}: ${escape(job.status==='done'?(job.error||'Resaved model and thumbnail'):job.status==='claimed'?'Baking…':job.status==='failed'?job.error:job.waiting_on_checkout?`Waiting for ${job.locked_by} to check in`:'Queued for bake worker')}</li>`).join('')
    +shownFailures.map(f=>`<li class="error">${escape(f.number)}: ${escape(f.error)}</li>`).join('');
  $('parts-batch-page-status').textContent=`Page ${Math.floor(partsBatchOffset/50)+1} of ${Math.max(1,Math.ceil(total/50))} · ${total} results`;
  $('parts-batch-prev').disabled=partsBatchPolling||partsBatchOffset===0;
  $('parts-batch-next').disabled=partsBatchPolling||partsBatchOffset+50>=total;
}
async function refreshPartsBatch() {
  if(!me||document.hidden||partsBatchRunning||partsBatchPolling||$('view-parts').hidden)return;
  restorePartBatch();if(!partsBatchReport)return;
  partsBatchPolling=true;
  $('parts-batch-prev').disabled=true;$('parts-batch-next').disabled=true;
  const batch=partsBatchReport;
  try {
    const page=await api('POST','/api/bake/jobs/progress',{ids:batch.jobs.map(job=>job.id),offset:partsBatchOffset,limit:50});
    if(batch!==partsBatchReport)return;
    partsBatchPolling=false;drawPartsBatch(page);
  }catch {} finally {partsBatchPolling=false;}
}
for(const [id,step] of [['parts-batch-prev',-50],['parts-batch-next',50]]) {
  $(id).addEventListener('click',()=>{if(partsBatchPolling)return;partsBatchOffset+=step;refreshPartsBatch();});
}
setInterval(refreshPartsBatch,5000);

const OBJECT_TABS = [['summary', 'Summary'], ['revisions', 'Revisions'], ['structure', 'Structure'], ['review', 'Review'], ['attachments', 'Attachments'], ['sourcing', 'Sourcing'], ['history', 'History']];
let objectTab = 'summary';
$('object-tabs').innerHTML = OBJECT_TABS.map(([id, label]) => `<button type="button" role="tab" id="object-tab-${id}" data-object-tab="${id}" aria-controls="object-panel-${id}" aria-selected="false">${label}</button>`).join('');
for (const [id] of OBJECT_TABS) {
  const panel = document.querySelector(`[data-object-panel="${id}"]`);
  panel.id = panel.id || `object-panel-${id}`;
  $(`object-tab-${id}`).setAttribute('aria-controls', panel.id);
  panel.setAttribute('role', 'tabpanel');
  panel.setAttribute('aria-labelledby', `object-tab-${id}`);
}
function selectObjectTab(id) {
  objectTab = id;
  for (const button of $('object-tabs').querySelectorAll('button')) {
    const active = button.dataset.objectTab === id;
    button.setAttribute('aria-selected', String(active));
    button.tabIndex = active ? 0 : -1;
  }
  for (const panel of document.querySelectorAll('[data-object-panel]')) panel.classList.toggle('active', panel.dataset.objectPanel === id);
}
$('object-tabs').addEventListener('click', event => {
  const tab = event.target.closest('[data-object-tab]');
  if (tab) selectObjectTab(tab.dataset.objectTab);
});
$('object-tabs').addEventListener('keydown', event => {
  if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return;
  event.preventDefault();
  const index = OBJECT_TABS.findIndex(([id]) => id === objectTab);
  const next = event.key === 'Home' ? 0 : event.key === 'End' ? OBJECT_TABS.length - 1 : (index + (event.key === 'ArrowRight' ? 1 : -1) + OBJECT_TABS.length) % OBJECT_TABS.length;
  selectObjectTab(OBJECT_TABS[next][0]);
  $(`object-tab-${objectTab}`).focus();
});
selectObjectTab('summary');

$('menu-change-password').addEventListener('click', () => {
  if (location.hash === '#/account/password') act(route);
});
$('shell-back').addEventListener('click', () => history.back());
$('shell-refresh').addEventListener('click', () => act(route));
$('menu-new-part').addEventListener('click', () => $('new-part').click());
for (const menu of document.querySelectorAll('.command-menu')) {
  menu.addEventListener('toggle', () => {
    if (menu.open) for (const other of document.querySelectorAll('.command-menu')) if (other !== menu) other.open = false;
  });
  menu.addEventListener('click', event => { if (event.target.closest('a, button')) menu.open = false; });
}
document.addEventListener('click', event => {
  if (!event.target.closest('.command-menu')) for (const menu of document.querySelectorAll('.command-menu')) menu.open = false;
});
document.addEventListener('keydown', event => {
  if (event.key === 'Escape') for (const menu of document.querySelectorAll('.command-menu[open]')) { menu.open = false; menu.querySelector('summary').focus(); }
});

let searchTimer = null;
for (const id of ['search', 'filter-tag']) {
  $(id).addEventListener('input', () => {
    clearTimeout(searchTimer);
    searchTimer = setTimeout(() => act(renderParts), 150);
  });
}
$('filter-geometry').addEventListener('change', () => act(renderParts));
$('filter-thumbnail').addEventListener('change', () => act(renderParts));
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

async function renderPart(id, selectedRevision) {
  const detail = await api('GET', `/api/parts/${encodeURIComponent(id)}`);
  // The editor belongs to one part's document; opening another part closes it
  // rather than leaving the last part's text under this part's heading.
  if (editorKey && !editorKey.startsWith(`part/${idSegment(detail.id)}/`)) {
    $('editor').hidden = true;
    editorKey = null;
  }
  if (currentPart?.id !== detail.id) selectObjectTab('summary');
  currentPart = detail;
  if (selectedRevision && detail.revision_views.some(r => r.id === selectedRevision)) {
    structure.part = detail.id; structure.revision = selectedRevision;
    attaching.part = detail.id; attaching.revision = selectedRevision; reviewRevision = selectedRevision;
  }
  const cadRevision = detail.revision_views.find(r => r.id === selectedRevision) || detail.revision_views.at(-1);
  $('part-open-cad').innerHTML = cadRevision ? openInCadButton(detail.id, cadRevision.id, cadRevision.editable ? 'Edit in CAD' : 'Open in CAD') : '';
  $('part-thumb').innerHTML = thumb(detail.thumbnail_url, 'lg');
  $('part-number').textContent = detail.number;
  $('part-name').textContent = detail.name;
  $('part-meta').innerHTML =
    `${escape(detail.part_type)}${detail.document_class && detail.document_class !== 'normal' ? ` · ${classBadge(detail.document_class)}` : ''} · created ${escape(when(detail.created_at))}`
    + (detail.description ? ` · ${escape(detail.description)}` : '');
  await renderDetails(detail);
  renderSourcing(detail);
  await renderSeed(detail);
  await renderStructure(detail);
  await loadSummaryFields(detail, selectedRevision || structure.revision);
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
      <tr data-revision-id="${escape(rev.id)}">
        <td><span class="rev-label">${revThumb(rev)}<strong>${escape(rev.label)}</strong></span>${revisionContext(rev)}</td>
        <td><span class="state state-${escape(rev.lifecycle)}">${escape(statusName(rev.lifecycle))}</span></td>
        <td>${reviewCell(rev)}</td>
        <td>${held}</td>
        <td class="actions">${actions(rev)}</td>
      </tr>`;
  }).join('');

  for (const button of $('rev-rows').querySelectorAll('button[data-do]')) {
    button.addEventListener('click', () => onRevisionAction(button.dataset.do, button.dataset.rev));
  }
  Workbench.partChanged(detail);
}

function revThumb(rev) {
  const t = rev.thumbnail;
  if (!t) return thumb('', 'xs', 'No picture yet');
  if (!t.current) return thumb('', 'xs', 'The picture is of an older save; the next save from CAD makes a new one');
  return thumb(t.url, 'xs', `Revision ${rev.label} preview`);
}

function actions(rev) {
  const out = [];
  const advanced = [];
  const rid = escape(rev.id);
  const cad = openInCadButton(currentPart.id, rev.id);
  if (cad) out.push(cad);
  if (rev.editable) {
    if (!rev.locked_by && me.can_author) out.push(`<button class="link" data-do="checkout" data-rev="${rid}">Check out</button>`);
    if (rev.locked_by_me) {
      advanced.push(`<button class="link" data-do="edit" data-rev="${rid}">Edit model JSON</button>`);
      out.push(`<button class="link" data-do="checkin" data-rev="${rid}">Check in</button>`);
    } else if (rev.locked_by && me.can_checkin) {
      out.push(`<button class="link danger" data-do="break" data-rev="${rid}">Break lock</button>`);
    }
    if (rev.lifecycle === 'draft' && me.can_author && statusEnabled('inreview')) out.push(`<button class="link" data-do="submit" data-rev="${rid}">Submit for review</button>`);
    if (rev.lifecycle === 'inreview' && me.can_author) out.push(`<button class="link" data-do="withdraw" data-rev="${rid}">Withdraw</button>`);
    if (!rev.eco && me.can_author) out.push(`<button class="link" data-do="to-eco" data-rev="${rid}">Add to change order</button>`);
    if (!rev.locked_by && me.can_checkin && statusEnabled('released')) out.push(`<button class="link" data-do="release" data-rev="${rid}">Release</button>`);
    if (me.can_author) out.push(`<button class="link" data-do="delete" data-rev="${rid}">Delete</button>`);
  } else {
    advanced.push(`<button class="link" data-do="view" data-rev="${rid}">View model JSON</button>`);
    if (rev.lifecycle === 'released' && me.can_checkin && statusEnabled('obsolete')) {
      out.push(`<button class="link" data-do="obsolete" data-rev="${rid}">Obsolete</button>`);
    }
    if (['released', 'superseded'].includes(rev.lifecycle) && !rev.eco && me.can_author) {
      out.push(`<button class="link" data-do="to-eco" data-rev="${rid}">Add to change order</button>`);
    }
  }
  if (advanced.length) out.push(`<details class="revision-advanced"><summary>Advanced</summary>${advanced.join(' ')}</details>`);
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
  const body = await api('GET', `/api/store/doc/${rev.document_key.replaceAll('%', '%25')}`);
  $('editor').hidden = false;
  $('editor-key').textContent = `${currentPart.number}/${rev.label}`;
  $('editor-body').value = body === null ? '' : (typeof body === 'string' ? body : JSON.stringify(body, null, 2));
  $('editor-body').readOnly = !writable;
  $('editor-save').hidden = !writable;
  $('editor-note').textContent = writable
    ? 'Advanced model editing. Use Edit in CAD for normal design changes.'
    : `Revision ${rev.label} is shown read-only${rev.locked_by ? `; checked out by ${rev.locked_by}` : ''}. Saved changes remain visible.`;
  $('editor-status').textContent = '';
  $('editor').scrollIntoView({ behavior: 'smooth', block: 'start' });
}

$('editor-save').addEventListener('click', () => act(async () => {
  await api('PUT', `/api/store/doc/${editorKey.replaceAll('%', '%25')}`, $('editor-body').value);
  $('editor-status').textContent = 'Saved.';
  OpenPages.saved();
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
  const body = await api('GET', `/api/store/doc/${key.replaceAll('%', '%25')}`);
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
    location.hash = `#/part/${encodeURIComponent(part.id)}`;
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

const PART_ATTRIBUTE_LOCK_ICON = '<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8"><rect x="5" y="10" width="14" height="11" rx="2"/><path d="M8 10V7a4 4 0 0 1 8 0v3"/></svg>';
function partAttributeEditor(field, value, editable, attributes = '') {
  const lock = editable ? '' : PART_ATTRIBUTE_LOCK_ICON;
  return `<div class="part-attribute-input${editable ? '' : ' is-read-only'}"><span class="attribute-lock" aria-hidden="true"${editable ? '' : ' title="Read-only"'}>${lock}</span>${fieldValueEditor(field, value, !editable, attributes)}</div>`;
}

async function renderDetails(detail) {
  await loadCategories();
  const catalogEditable = me.can_author && !detail.catalog_locked;
  $('part-locked').hidden = !detail.catalog_locked;
  const options = categories.map(c => ({value: c.id, label: c.path || c.name}));
  if (detail.category && !options.some(o => o.value === detail.category)) {
    options.push({value: detail.category, label: `${detail.category} (not in the catalog)`});
  }
  const metadata = [
    {key: 'has_geometry', name: 'Contains 3D geometry', type: 'bool', editable: false},
    {key: 'has_thumbnail', name: 'Has thumbnail', type: 'bool', editable: false},
    {key: 'category', name: 'Category', type: 'enum', options, empty_label: 'Uncategorized', editable: catalogEditable},
    {key: 'tags', name: 'Tags', type: 'text', encoding: 'comma-list', editable: me.can_author},
  ];
  const rows = metadata.map(f => `<dt>${escape(f.name)}</dt><dd>${partAttributeEditor(f, detail[f.key], f.editable, `data-summary-metadata="${escape(f.key)}"`)}</dd>`);
  rows.push(...detail.schema.map(def => {
    const value = detail.attributes[def.key];
    const cell = partAttributeEditor(def, value, catalogEditable, `data-summary-catalog="${escape(def.key)}"`);
    return `<dt>${escape(def.name)}${def.required ? ' <span class="req">*</span>' : ''}</dt><dd>${cell}${def.required && value == null ? '<span class="missing">missing — needed before release</span>' : ''}</dd>`;
  }));
  for (const key of detail.inert) {
    rows.push(`<dt class="muted">${escape(key)}</dt><dd>${partAttributeEditor({name: key, type: 'text'}, detail.attributes[key], false)}
      <span class="muted">Not an attribute of this category; kept, no effect.</span>
      <button type="button" class="ghost" data-summary-clear="${escape(key)}"${catalogEditable ? '' : ' disabled'}>Clear value</button></dd>`);
  }
  $('part-attrs').innerHTML = rows.join('');
  $('part-attrs').hidden = false;
  for (const input of $('part-attrs').querySelectorAll('[data-summary-metadata]')) input.onchange = () => act(async () => {
    const field = metadata.find(f => f.key === input.dataset.summaryMetadata);
    input.disabled = true;
    try { await api('PATCH', `/api/parts/${encodeURIComponent(detail.id)}`, {[field.key]: editedFieldValue(field, input)}); }
    finally { await renderPart(detail.id, $('summary-revision').value); }
  });
  for (const input of $('part-attrs').querySelectorAll('[data-summary-catalog]')) input.onchange = () => act(async () => {
    const def = detail.schema.find(f => f.key === input.dataset.summaryCatalog);
    input.disabled = true;
    try { await api('PATCH', `/api/parts/${encodeURIComponent(detail.id)}`, {attributes: {[def.key]: editedFieldValue(def, input)}}); }
    finally { await renderPart(detail.id, $('summary-revision').value); }
  });
  for (const button of $('part-attrs').querySelectorAll('[data-summary-clear]')) button.onclick = () => act(async () => {
    button.disabled = true;
    try { await api('PATCH', `/api/parts/${encodeURIComponent(detail.id)}`, {attributes: {[button.dataset.summaryClear]: null}}); }
    finally { await renderPart(detail.id, $('summary-revision').value); }
  });
}

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
  $('cat-meta').innerHTML = `${c.parts} part${c.parts === 1 ? '' : 's'} filed here, ${c.parts_within} including sub-categories`;

  const inherited = schema.attributes.filter((a) => a.from !== c.id);
  const nameOf = (id) => (categories.find((x) => x.id === id) || { name: id }).name;
  $('cat-inherited-none').hidden = inherited.length > 0;
  $('cat-inherited-none').textContent = c.parent ? 'No inherited fields are defined by this category’s ancestors.' : 'None. This is a top-level category.';
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
      <td class="actions">${me.is_admin ? `<button class="link" data-type="${escape(t.id)}">Edit</button> <button class="link" data-type-fields="${escape(t.id)}">Fields…</button>` : ''}</td>
    </tr>`).join('');
  for(const button of $('type-rows').querySelectorAll('[data-type-fields]'))button.addEventListener('click',()=>act(()=>openFieldDefinitions(button.dataset.typeFields)));
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
    OpenPages.saved();
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

async function openScript(path, starter, routed = false) {
  if(!routed && (starter===undefined || OpenPages.scriptDraft(`#/scripts/${encodeURIComponent(path)}`))){location.hash=`#/scripts/${encodeURIComponent(path)}`;return;}
  if(!routed)OpenPages.capture();
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
  if(!routed){const hash=`#/scripts/${encodeURIComponent(path)}`;history.replaceState(null,'',hash);OpenPages.activate(hash,'scripts',path);OpenPages.capture();}
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
  OpenPages.saved();
  await renderScripts();
}));

$('script-delete').addEventListener('click', () => act(async () => {
  if (!confirm(`Delete ${scriptPath} from the scripts directory?`)) return;
  await api('DELETE', `/api/scripts/file/${scriptPath}`);
  OpenPages.closeTab(OpenPages.keyOf(location.hash),true);
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

// OpenPages owns unload warnings for active and background script drafts.

// ---------------------------------------------------------------------- go



// ------------------------------------------------- families and templates

// Surface build failures and regeneration risks; ancestry is useful on demand.
function revisionContext(rev) {
  const notices = [];
  if (rev.hand_edited) notices.push('<span class="st st-stale">Edited after generation</span>');
  if (rev.bake && rev.bake !== 'done') {
    const label = { pending: 'Build queued', claimed: 'Building', failed: 'Build failed' }[rev.bake] || `Build ${rev.bake}`;
    notices.push(`<span class="st st-${escape(rev.bake)}">${escape(label)}</span>${rev.bake_error ? `<div class="error">${escape(rev.bake_error)}</div>` : ''}`);
  }
  const parents = [];
  const link = p => `<a href="#/part/${encodeURIComponent(p.part_id)}/${encodeURIComponent(p.revision_label)}/summary">${escape(p.number)}/${escape(p.revision_label)}</a>`;
  if (rev.family) parents.push(`Family ${link(rev.family)}`);
  if (rev.template) parents.push(`Template ${link(rev.template)}`);
  if (parents.length) notices.push(`<details><summary>Generated from</summary>${parents.join('<br>')}</details>`);
  return notices.length ? `<div class="revision-context">${notices.join(' ')}</div>` : '';
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
    .map((r) => `<option value="${escape(r.id)}">${escape(r.label)} (${escape(statusName(r.lifecycle))})</option>`)
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
        + (m.revision_label ? ` rev ${escape(m.revision_label)} <span class="state state-${escape(m.lifecycle)}">${escape(statusName(m.lifecycle))}</span>` : '')
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
    await reloadEditor(documentKey(currentPart.id, result.import.revision_id));
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
    return `<label><span title="${escape(input.name)}">${label}${limits ? ` <span class="muted">(${escape(limits)})</span>` : ''}</span>${field}</label>`;
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
    location.hash = `#/part/${encodeURIComponent(part.id)}`;
  });
});

// ----------------------------------------------------------- the bake queue

let bakeWorkerBusy = false, bakeOffset = 0, bakeGeneration = 0, bakeLoading = false, bakeNext = null;
async function renderBakeWorker() {
  $('bake-worker-controls').hidden = !me.is_admin;
  if (!me.is_admin || bakeWorkerBusy) return;
  const worker = await api('GET', '/api/bake/worker');
  if (bakeWorkerBusy) return;
  $('bake-worker-status').textContent = worker.running ? `Running (PID ${worker.pid})` : 'Stopped';
  $('bake-worker-message').textContent = worker.error || (worker.configured
    ? (worker.running ? 'The worker is processing queued jobs.' : 'Start the worker to process queued jobs.')
    : 'The server operator must configure the native worker executable, token file and server URL before it can be started here.');
  $('bake-worker-start').disabled = !worker.configured || worker.running;
  $('bake-worker-stop').disabled = !worker.running;
}
for (const action of ['start', 'stop']) {
  $(`bake-worker-${action}`).addEventListener('click', () => act(async () => {
    if (bakeWorkerBusy) return;
    bakeWorkerBusy = true;
    $('bake-worker-start').disabled = true;
    $('bake-worker-stop').disabled = true;
    try {
      await api('POST', `/api/bake/worker/${action}`);
    } finally {
      bakeWorkerBusy = false;
      await renderBakeWorker();
    }
  }));
}
async function renderBake() {
  const generation=++bakeGeneration;
  bakeLoading=true;$('bake-prev').disabled=true;$('bake-next').disabled=true;
  try {
  await renderBakeWorker();
  const filter = $('bake-filter').value;
  const size=Number($('bake-page-size').value);
  const page = await api('GET', `/api/bake/jobs?${new URLSearchParams({status:filter,limit:size,offset:bakeOffset})}`);
  if(generation!==bakeGeneration)return;
  const jobs=page.jobs;
  bakeOffset=page.offset;bakeNext=page.next;
  $('bake-page-status').textContent=`Page ${Math.floor(bakeOffset/size)+1} of ${Math.max(1,Math.ceil(page.total/size))} · ${page.total} jobs`;
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
      <td>${worker}${job.waiting_on_checkout && job.status === 'pending'
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
  } finally {
    if(generation===bakeGeneration){bakeLoading=false;$('bake-prev').disabled=bakeOffset===0;$('bake-next').disabled=bakeNext===null;}
  }
}

$('bake-filter').addEventListener('change', () => act(async()=>{bakeOffset=0;await renderBake();}));
$('bake-page-size').addEventListener('change', () => act(async()=>{bakeOffset=0;await renderBake();}));
$('bake-prev').addEventListener('click', () => act(async()=>{bakeOffset=Math.max(0,bakeOffset-Number($('bake-page-size').value));await renderBake();}));
$('bake-next').addEventListener('click', () => act(async()=>{if(bakeNext===null)return;bakeOffset=bakeNext;await renderBake();}));
$('bake-refresh').addEventListener('click', () => act(renderBake));
setInterval(()=>{if(me&&!bakeLoading&&!document.hidden&&!$('view-bake').hidden)renderBake().catch(()=>{});},5000);

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
    .map((r) => `<option value="${escape(r.id)}">${escape(r.label)} · ${escape(statusName(r.lifecycle))}</option>`);
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
  await Promise.all([loadBom(), loadUses(), loadDiff(), loadWhereUsed(), loadRevisionFields(detail)]);
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

let bomReadSerial=0;
async function loadBom() {
  const serial=++bomReadSerial;
  const flat = $('bom-view').value === 'flat';
  const query = `levels=0&flat=${flat}&occurrences=true`;
  const base = `${revUrl(structure.revision)}/bom`;
  const bom = await api('GET', `${base}?${query}`);
  if(serial!==bomReadSerial)return;
  $('bom-csv').href = `${base}?${query}&format=csv`;
  const empty = bom.lines.length === 0;
  $('bom-empty').hidden = !empty;
  $('bom-wrap').hidden = empty;
  $('bom-csv').hidden = empty;
  $('bom-totals').hidden = empty;

  await drawConfiguredBom(bom,serial);
  if(serial!==bomReadSerial)return;
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
        <span class="muted">· rev ${escape(p.latest_label)} ${escape(statusName(p.latest_state))}</span></button></li>`).join('')
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
    + currentPart.revision_views.map((r) => `<option value="${escape(r.id)}">Rev ${escape(r.label)} (${escape(statusName(r.lifecycle))})</option>`).join('');
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
    + revisions.map((r) => `<option value="${escape(r.id)}">Rev ${escape(r.label)} (${escape(statusName(r.lifecycle))})</option>`).join('');
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
        <span class="muted">· rev ${escape(p.latest_label)} ${escape(statusName(p.latest_state))}</span></button></li>`).join('')
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
        <td>${preview && doable ? `<input type="checkbox" data-parent="${escape(documentKey(r.part_id, r.parent_revision_id))}" checked aria-label="Include ${escape(r.number)} rev ${escape(r.parent_revision)}">` : ''}</td>
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
        : `<a href="${FileDocuments.route('attachment',a.id,currentPart.id,a.on==='part'?'':attaching.revision)}">Open</a>`)
      : '';
    return `
      <tr>
        <td>${thumb}${FileDocuments.documentLink(a,currentPart.id,a.on==='part'?'':attaching.revision)}
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
  const lines = Object.entries(event.changes || {}).flatMap(([field, raw]) => {
    if (field === 'content_hash') return '<div>Model document updated</div>';
    if (field === 'thumbnail') return '<div>Preview updated</div>';
    if (field === 'size' && event.changes.content_hash) return [];
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

async function renderReviews() { await Workbench.inbox(); }

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
  $('rv-revision').innerHTML = views.map((r) => `<option value="${escape(r.id)}">${escape(r.label)} — ${escape(statusName(r.lifecycle))}</option>`).join('');
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
    parts.push(`<div>${item.document_changed ? 'Model changed from the previous revision' : 'Model unchanged from the previous revision'}</div>`);
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
      <td class="number"><a href="#/part/${encodeURIComponent(item.part_id)}">${escape(item.number)}</a><br><span class="muted">${escape(item.name)}</span></td>
      <td><strong>${escape(item.label)}</strong> <span class="state state-${escape(item.lifecycle)}">${escape(statusName(item.lifecycle))}</span></td>
      <td>${escape(item.replaces || '—')}</td>
      <td>${itemChanges(item)}${item.problem ? `<div class="error">${escape(item.problem)}</div>` : ''}
        ${itemDetail(item) ? `<details><summary>Details</summary><div class="item-detail">${itemDetail(item)}</div></details>` : ''}</td>
      <td class="actions">${eco.can_edit ? `<button class="link danger" data-part="${escape(item.part_id)}" data-remove="${escape(item.revision_id)}">Remove</button>` : ''}</td>
    </tr>`).join('');
  $('eco-items-empty').hidden = eco.items.length > 0;
  for (const button of $('eco-items').querySelectorAll('button[data-remove]')) {
    button.addEventListener('click', () => act(async () => {
      await api('DELETE', `${ecoUrl()}/items/${encodeURIComponent(button.dataset.remove)}?part=${encodeURIComponent(button.dataset.part)}`);
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
  $('eco-add-rev').innerHTML = ecoAddPart.revision_views.map((r) => `<option value="${escape(r.id)}">${escape(r.label)} — ${escape(statusName(r.lifecycle))}</option>`).join('');
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

async function renderWorkspace() { await Workbench.home(); }

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
    .concat(currentPart.revision_views.slice().reverse().map((r) => `<option value="${escape(r.id)}">pin to ${escape(r.label)} (${escape(statusName(r.lifecycle))})</option>`))
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

// ------------------------------------------------ native model import wizard
let nativeWizard = { rows: [], types: [], job: null, running: false, stop: false, owner: null, busy: false };
const niError = error => { $('ni-error').textContent = error?.message || String(error || ''); $('ni-error').hidden = !error; };
async function niAct(fn) {
  if (nativeWizard.busy || nativeWizard.running) return;
  nativeWizard.busy = true; niError(null);
  $('ni-files-step').disabled = true; $('ni-mapping-fields').disabled = true;
  try { await fn(); } catch (error) { niError(error); }
  finally {
    nativeWizard.busy = false;
    const frozen = Boolean(nativeWizard.job) || !$('ni-review-step').hidden;
    $('ni-files-step').disabled = frozen; $('ni-mapping-fields').disabled = frozen;
  }
}
function importDatabase() {
  return new Promise((resolve, reject) => {
    const request = indexedDB.open('brep-plm-native-import', 1);
    request.onupgradeneeded = () => request.result.createObjectStore('jobs');
    request.onsuccess = () => resolve(request.result);
    request.onerror = () => reject(new Error('Cannot save import progress in this browser. Enable browser storage before importing.'));
  });
}
async function importStorage(operation, value) {
  const db = await importDatabase();
  try {
    return await new Promise((resolve, reject) => {
      const transaction = db.transaction('jobs', operation === 'get' ? 'readonly' : 'readwrite');
      const store = transaction.objectStore('jobs');
      const key = nativeWizard.owner;
      const request = operation === 'get' ? store.get(key) : operation === 'delete' ? store.delete(key) : store.put(value, key);
      transaction.oncomplete = () => resolve(request.result);
      transaction.onerror = transaction.onabort = () => reject(new Error('Import progress could not be saved. Free browser storage before resuming.'));
    });
  } finally { db.close(); }
}
async function openNativeImport() {
  if (nativeWizard.owner === me.id) return;
  nativeWizard = { rows: [], types: await api('GET', '/api/part-types'), job: null, running: false, stop: false, owner: me.id, busy: false };
  $('ni-default-type').innerHTML = nativeWizard.types.map(t => `<option value="${escape(t.id)}">${escape(t.name)}</option>`).join('');
  nativeWizard.job = await importStorage('get');
  if (nativeWizard.job) niProgress(nativeWizard.job);
}
function niType(row) { return nativeWizard.types.find(t => t.id === row.part_type); }
function niNumbering(row) {
  const type = niType(row);
  if (!type) return 'Choose a part type';
  if (type.mode.kind === 'counter') {
    const before = nativeWizard.rows.slice(0, nativeWizard.rows.indexOf(row)).filter(r => r.action === 'create' && r.part_type === row.part_type).length;
    return `Estimated ${type.prefix}${String(type.next + before).padStart(type.digits, '0')}`;
  }
  return type.mode.kind === 'pattern' ? `Pattern: ${type.mode.regex}` : type.mode.kind === 'script' ? `Server script: ${type.mode.script}` : 'Enter a unique part number';
}
function niRenderRows() {
  $('ni-mapping-step').hidden = false;
  $('ni-rows').innerHTML = nativeWizard.rows.map((row, i) => {
    const creating = row.action === 'create', counter = niType(row)?.mode.kind === 'counter';
    return `<tr data-import-row="${i}"><td><input data-field="name" aria-label="Name for ${escape(row.name)}" value="${escape(row.name)}"><small>${row.assembly ? 'Assembly' : 'Part'} · ${escape(row.id)}${row.root ? ' · uploaded file' : ' · embedded'}</small></td>
      <td><select data-field="action" aria-label="Action for ${escape(row.name)}">${[['create','Create part'],['revise','Create new revision'],['replace','Replace editable draft'],['reuse','Reuse existing revision']].map(([v,t]) => `<option value="${v}" ${row.action === v ? 'selected' : ''}>${t}</option>`).join('')}</select></td>
      <td><select data-field="part_type" aria-label="Part type for ${escape(row.name)}" ${creating ? '' : 'disabled'}>${nativeWizard.types.map(t => `<option value="${escape(t.id)}" ${row.part_type === t.id ? 'selected' : ''}>${escape(t.name)}</option>`).join('')}</select><small>${creating ? escape(niNumbering(row)) : 'Existing part keeps its type and number'}</small></td>
      <td><input data-field="${creating ? 'number' : 'part'}" aria-label="${creating ? 'Number' : 'Existing part'} for ${escape(row.name)}" value="${escape(creating ? row.number : row.part)}" ${creating && counter ? 'disabled placeholder="Assigned on import"' : ''}>${creating ? '' : '<button type="button" data-target-lookup class="ghost">Load revisions</button>'}</td>
      <td>${creating ? 'First revision' : row.action === 'revise' ? 'Next revision' : `<select data-field="revision" aria-label="Revision for ${escape(row.name)}"><option value="">Newest (load to choose)</option>${(row.available || []).map(r => `<option value="${escape(r.id)}" ${r.id === row.revision ? 'selected' : ''}>${escape(r.label)} · ${escape(statusName(r.lifecycle))}${r.locked_by ? ' · checked out' : ''}</option>`).join('')}</select>`}</td></tr>`;
  }).join('');
}
async function niReadFiles(list) {
  if (nativeWizard.job) throw new Error('Finish the current import or choose Start another import before choosing files.');
  const files = [...list].filter(f => /\.nbrep$/i.test(f.name));
  if (!files.length) throw new Error('Choose at least one .nbrep file.');
  $('ni-review-step').hidden = true;
  $('ni-mapping-step').hidden = true;
  nativeWizard.rows = [];
  $('ni-rows').replaceChildren();
  $('ni-file-summary').textContent = 'Reading native models…';
  const documents = [];
  for (const file of files) {
    try { documents.push({ name: file.webkitRelativePath || file.name, document: JSON.parse(await file.text()) }); }
    catch { throw new Error(`${file.name}: this is not valid native JSON`); }
  }
  if (!$('ni-namespace').value.trim()) $('ni-namespace').value = NativeImport.path(documents[0].name).split('/')[0];
  const plan = NativeImport.plan(documents);
  if (plan.errors.length) { $('ni-file-summary').textContent = 'Files need attention.'; throw new Error(plan.errors.join('\n')); }
  const types = nativeWizard.types;
  if (!types.length) throw new Error('Create a part type before importing models.');
  for (const row of plan.rows) {
    row.action = 'create'; row.number = ''; row.part = ''; row.revision = '';
    row.part_type = (types.find(t => t.id === (row.assembly ? 'assembly' : 'component')) || types[0]).id;
    const matches = (await api('GET', `/api/parts?external_ref=${encodeURIComponent(NativeImport.externalRef($('ni-namespace').value, row.id))}&limit=2`)).parts;
    if (matches.length === 1) {
      const p = await api('GET', `/api/parts/${encodeURIComponent(matches[0].id)}`);
      row.part = p.number; row.part_type = p.part_type; row.available = p.revision_views;
      const rev = p.revision_views.at(-1);
      row.revision = rev?.id || '';
      row.action = rev?.editable && !rev.locked_by ? 'replace' : p.revision_views.some(r => r.editable) ? 'reuse' : 'revise';
    }
  }
  nativeWizard.rows = plan.rows;
  $('ni-file-summary').textContent = `${files.length} file(s), ${plan.rows.length} distinct parts and assemblies, ${plan.rows.reduce((n,r) => n + Object.keys(r.dependencies).length, 0)} component links.`;
  niRenderRows();
}
for (const id of ['ni-files', 'ni-folder']) $(id).addEventListener('change', event => niAct(() => niReadFiles(event.target.files)));
$('ni-rows').addEventListener('change', event => {
  const tr = event.target.closest('[data-import-row]'), field = event.target.dataset.field;
  if (!tr || !field || nativeWizard.running || nativeWizard.busy) return;
  const row = nativeWizard.rows[Number(tr.dataset.importRow)];
  row[field] = event.target.value;
  if (field === 'part') { row.available = []; row.revision = ''; }
  if (field === 'part_type') row.number = '';
  if (['action','part_type'].includes(field)) niRenderRows();
});
$('ni-rows').addEventListener('click', event => {
  if (!event.target.closest('[data-target-lookup]')) return;
  const row = nativeWizard.rows[Number(event.target.closest('[data-import-row]').dataset.importRow)];
  niAct(async () => {
    const p = await api('GET', `/api/parts/${encodeURIComponent(row.part.trim())}`);
    row.part = p.number; row.part_type = p.part_type; row.available = p.revision_views; row.revision = p.revision_views.at(-1)?.id || '';
    niRenderRows();
  });
});
$('ni-apply-type').addEventListener('click', () => {
  for (const row of nativeWizard.rows.filter(r => r.action === 'create')) { row.part_type = $('ni-default-type').value; row.number = ''; }
  niRenderRows();
});
$('ni-review').addEventListener('click', () => niAct(async () => {
  const namespace = $('ni-namespace').value.trim();
  if (!namespace) throw new Error('Give this import set a name.');
  if (!nativeWizard.rows.length) throw new Error('Choose native files first.');
  for (const row of nativeWizard.rows) {
    if (!row.name.trim()) throw new Error('Every part needs a name.');
    delete row.reviewed;
    row.external_ref = NativeImport.externalRef(namespace, row.id);
    if (row.action !== 'create') continue;
    const type = niType(row);
    if (!type) throw new Error(`${row.name}: select a part type`);
    if (['free','pattern'].includes(type.mode.kind) && !row.number.trim()) throw new Error(`${row.name}: enter a number for ${type.name}`);
    if (row.number.trim()) {
      const matches = (await api('GET', `/api/parts?q=${encodeURIComponent(row.number.trim())}&limit=50`)).parts;
      if (matches.some(p => p.number.toLowerCase() === row.number.trim().toLowerCase())) throw new Error(`${row.number}: already exists; choose an update action for this row`);
    }
    const matches = (await api('GET', `/api/parts?external_ref=${encodeURIComponent(row.external_ref)}&part_type=${encodeURIComponent(row.part_type)}&limit=2`)).parts;
    if (matches.length) throw new Error(`${row.name}: already imported as ${matches[0].number}; choose an update action`);
  }
  const numbers = nativeWizard.rows.filter(r => r.action === 'create' && r.number.trim()).map(r => r.number.trim().toLowerCase());
  if (new Set(numbers).size !== numbers.length) throw new Error('Two new parts have the same number.');
  await api('POST', '/api/import/validate', nativeWizard.rows.filter(r => r.action === 'create').map(r => ({part_type: r.part_type, number: r.number})));
  await NativeImport.inspect(nativeWizard.rows, api);
  $('ni-review-rows').innerHTML = nativeWizard.rows.map(row => `<tr><td>${escape(row.name)}<br><small>${escape(row.id)}</small></td><td>${escape({create:'Create part',revise:'Create new revision',replace:'REPLACE draft document and BOM',reuse:'Reuse without changes'}[row.action])}</td><td>${escape(row.reviewed?.number || row.number || niNumbering(row))}</td><td>${escape(row.action === 'create' ? 'First revision' : row.action === 'revise' ? 'New revision' : row.reviewed.label)}</td></tr>`).join('');
  $('ni-review-summary').textContent = `${nativeWizard.rows.length} items: ${nativeWizard.rows.filter(r=>r.action==='create').length} new parts, ${nativeWizard.rows.filter(r=>r.action==='revise').length} new revisions, ${nativeWizard.rows.filter(r=>r.action==='replace').length} draft replacements, ${nativeWizard.rows.filter(r=>r.action==='reuse').length} reused revisions.`;
  $('ni-review-step').hidden = false; $('ni-files-step').disabled = true; $('ni-mapping-fields').disabled = true;
  $('ni-confirm').checked = false; $('ni-run').disabled = true;
  $('ni-review-step').scrollIntoView({ block: 'start' });
}));
$('ni-confirm').addEventListener('change', () => { $('ni-run').disabled = !$('ni-confirm').checked; });
$('ni-back').addEventListener('click', () => {
  $('ni-review-step').hidden = true; $('ni-files-step').disabled = false; $('ni-mapping-fields').disabled = false;
});
function niProgress(job) {
  $('ni-results').hidden = false; $('ni-status').textContent = job.current || 'Ready to resume';
  $('ni-files-step').disabled = true; $('ni-mapping-fields').disabled = true;
  $('ni-result-rows').innerHTML = job.rows.map(row => {
    const target = job.targets[row.id];
    return `<li>${escape(row.name)} — ${target ? `<a href="#/part/${encodeURIComponent(target.part)}">${escape(target.number)} / ${escape(target.label)}</a> · ${target.done ? row.action === 'reuse' ? 'reused' : 'imported' : target.written ? 'saved, awaiting check-in' : 'allocated, awaiting document'}` : row.allocating ? 'allocation outcome unconfirmed — inspect the server before retrying' : 'not started'}</li>`;
  }).join('');
  $('ni-resume').hidden = Boolean(job.complete); $('ni-resume').disabled = nativeWizard.running;
  $('ni-reset').disabled = nativeWizard.running; $('ni-stop').hidden = !nativeWizard.running;
}
async function niExecute() {
  nativeWizard.running = true; nativeWizard.stop = false; niError(null);
  $('ni-review-step').hidden = true;
  $('ni-run').disabled = true;
  try { await NativeImport.run(nativeWizard.job, api, job => importStorage('put', job), niProgress, () => nativeWizard.stop); }
  catch (error) { niError(error); }
  finally { nativeWizard.running = false; niProgress(nativeWizard.job); }
}
$('ni-run').addEventListener('click', () => niAct(async () => {
  if (!$('ni-confirm').checked) return;
  nativeWizard.job = { rows: JSON.parse(JSON.stringify(nativeWizard.rows)), targets: {}, complete: false, current: 'Ready to import' };
  await importStorage('put', nativeWizard.job);
  await niExecute();
}));
$('ni-resume').addEventListener('click', () => niAct(niExecute));
$('ni-stop').addEventListener('click', () => { nativeWizard.stop = true; });
$('ni-receipt').addEventListener('click', () => {
  const job = nativeWizard.job;
  const receipt = { complete: job.complete, current: job.current, items: job.rows.map(row => ({ source: row.id, action: row.action, allocationPending: Boolean(row.allocating && !job.targets[row.id]), reviewed: row.reviewed, target: job.targets[row.id] && { ...job.targets[row.id], reusedDocument: undefined } })) };
  const url = URL.createObjectURL(new Blob([JSON.stringify(receipt,null,2)], {type:'application/json'}));
  const a = document.createElement('a'); a.href = url; a.download = 'native-import-receipt.json'; a.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
});
$('ni-reset').addEventListener('click', () => niAct(async () => {
  if (!nativeWizard.job?.complete && !confirm('Keep existing parts and discard this import’s saved progress? Download its receipt first if you need to reconcile it.')) return;
  await importStorage('delete'); nativeWizard.job = null; nativeWizard.rows = [];
  for (const id of ['ni-results','ni-mapping-step','ni-review-step']) $(id).hidden = true;
  $('ni-files-step').disabled = false; $('ni-mapping-fields').disabled = false;
  $('ni-rows').replaceChildren();
  $('ni-files').value = ''; $('ni-folder').value = ''; $('ni-file-summary').textContent = 'No files selected.'; $('ni-status').textContent = '';
}));
window.addEventListener('beforeunload', event => { if (nativeWizard.running) { event.preventDefault(); event.returnValue = ''; } });

Workbench.init();

// Checkout protects writes. Keep readers up to date with every accepted save
// and its later thumbnail upload, without replacing forms or local drafts.
let modelChangeSeq = 0, modelChangeBusy = false;
async function refreshSavedModels() {
  if(!me || document.hidden || modelChangeBusy || document.querySelector('.work-area').inert)return;
  modelChangeBusy=true;
  const hash=location.hash,user=me.id;
  try {
    const changes=await api('GET',`/api/store/changes?since=${modelChangeSeq}`);
    if(me?.id!==user || location.hash!==hash)return;
    const changed=new Set(changes.keys.map(key=>decodeURIComponent(key.split('/')[1])));
    if(!changes.stale&&!changed.size){modelChangeSeq=changes.seq;return;}
    if(!$('view-parts').hidden) {
      const generation=partsGeneration;
      for(const part of listedParts) {
        if(!changes.stale&&!changed.has(part.id))continue;
        const detail=await api('GET',`/api/parts/${encodeURIComponent(part.id)}`);
        if(location.hash!==hash || generation!==partsGeneration)return;
        part.thumbnail_url=detail.thumbnail_url;
        const row=[...$('parts-rows').rows].find(row=>row.dataset.partId===part.id);
        const preview=row?.querySelector('.thumb-cell a');
        if(preview)preview.innerHTML=thumb(detail.thumbnail_url);
      }
    }
    if(!$('view-part').hidden&&currentPart&&(changes.stale||changed.has(currentPart.id))) {
      const id=currentPart.id;
      const detail=await api('GET',`/api/parts/${encodeURIComponent(id)}`);
      if(location.hash!==hash || currentPart?.id!==id)return;
      currentPart=detail;
      $('part-thumb').innerHTML=thumb(detail.thumbnail_url,'lg');
      for(const row of $('rev-rows').rows) {
        const revision=detail.revision_views.find(revision=>revision.id===row.dataset.revisionId);
        const preview=row.querySelector('.rev-label .thumb');
        if(revision&&preview)preview.outerHTML=revThumb(revision);
      }
      if(editorKey&&!$('editor').hidden&&$('editor-body').readOnly) {
        const key=editorKey;
        const body=await api('GET',`/api/store/doc/${key.replaceAll('%','%25')}`);
        if(location.hash===hash&&editorKey===key&&$('editor-body').readOnly)$('editor-body').value=body===null?'':typeof body==='string'?body:JSON.stringify(body,null,2);
      }
    }
    if(location.hash!==hash)return;
    await Workbench.savedModels(changed,changes.stale);
    modelChangeSeq=changes.seq;
  } catch {
    // Retry after a temporary outage; background refresh must not erase work.
  } finally { modelChangeBusy=false; }
}
setInterval(refreshSavedModels,2000);
window.addEventListener('focus',refreshSavedModels);
document.addEventListener('visibilitychange',refreshSavedModels);

// Field definitions and the BOM layouts are shared with the CAD application.
let fieldDefinitionTarget=null, fieldDefinitions=[];
function drawFieldDefinitions(){
  $('fields-editor').innerHTML=fieldDefinitions.map((d,i)=>`<fieldset data-index="${i}"><label>Key <input data-def="key" value="${escape(d.key)}" required></label><label>Name <input data-def="name" value="${escape(d.name)}" required></label><label>Type <select data-def="type">${['text','number','bool','enum'].map(t=>`<option${d.type===t?' selected':''}>${t}</option>`).join('')}</select></label><label>Units <input data-def="unit" value="${escape(d.unit||'')}" ${d.type==='number'?'':'disabled'}></label><label>Choices (one per line) <textarea data-def="values" ${d.type==='enum'?'':'disabled'}>${escape((d.values||[]).join('\n'))}</textarea></label><label><input type="checkbox" data-def="required" ${d.required?'checked':''}> Required</label><button type="button" data-remove="${i}" class="ghost">Remove field</button></fieldset>`).join('');
  for(const row of $('fields-editor').querySelectorAll('fieldset')){
    for(const input of row.querySelectorAll('[data-def]'))input.addEventListener(input.matches('select,input[type=checkbox]')?'change':'input',()=>{
      const d=fieldDefinitions[Number(row.dataset.index)],key=input.dataset.def;
      d[key]=key==='required'?input.checked:key==='values'?input.value.split('\n').map(v=>v.trim()).filter(Boolean):input.value;
      if(key==='type')drawFieldDefinitions();
    });
  }
  for(const button of $('fields-editor').querySelectorAll('[data-remove]'))button.addEventListener('click',()=>{fieldDefinitions.splice(Number(button.dataset.remove),1);drawFieldDefinitions();});
}
async function openFieldDefinitions(type){
  fieldDefinitionTarget=type;
  const data=type?await api('GET',`/api/part-types/${encodeURIComponent(type)}/fields`):await api('GET','/api/bom/configuration');
  fieldDefinitions=structuredClone(type?data.fields:data.occurrence_fields);
  $('fields-title').textContent=type?`Fields for ${partTypes.find(t=>t.id===type)?.name||type}`:'Occurrence fields';
  drawFieldDefinitions();$('fields-dialog').showModal();
}
$('fields-add').addEventListener('click',()=>{fieldDefinitions.push({key:'',name:'',type:'text',required:false});drawFieldDefinitions();});
$('fields-cancel').addEventListener('click',()=>$('fields-dialog').close());
$('fields-form').addEventListener('submit',e=>{e.preventDefault();act(async()=>{
  const fields=fieldDefinitions.map(d=>({key:d.key.trim(),name:d.name.trim(),type:d.type,required:d.required,...(d.type==='number'?{unit:d.unit||''}:{}),...(d.type==='enum'?{values:d.values||[]}: {})}));
  await api('PUT',fieldDefinitionTarget?`/api/part-types/${encodeURIComponent(fieldDefinitionTarget)}/fields`:'/api/bom/occurrence-fields',{fields});
  $('fields-dialog').close();if(!fieldDefinitionTarget)await renderOccurrenceFields();
});});
$('occurrence-fields-edit').addEventListener('click',()=>act(()=>openFieldDefinitions(null)));
async function renderOccurrenceFields(){
  const config=await api('GET','/api/bom/configuration');
  $('occurrence-fields-list').innerHTML='<p>Callout / find number, reference designator, and installation notes are built-in occurrence fields.</p>'+config.occurrence_fields.map(d=>`<p><strong>${escape(d.name)}</strong> — ${escape(d.type)}${d.unit?` (${escape(d.unit)})`:''}</p>`).join('');
}
let bomConfiguration=null, bomLayoutEditing='', bomLayoutColumns=[];
const DEFAULT_BOM_COLUMNS=['builtin.number','builtin.name','builtin.revision_label','builtin.quantity','builtin.total'];
async function renderBomConfigurations(){
  bomConfiguration=await api('GET','/api/bom/configuration');
  $('bc-layout').innerHTML='<option value="">New configuration</option>'+bomConfiguration.layouts.map(l=>`<option value="${escape(l.id)}">${escape(l.name)} (${l.owner?'Personal':'Shared'})</option>`).join('');
  const target=bomConfiguration.layouts.find(l=>l.id===bomLayoutEditing)||bomConfiguration.layouts.find(l=>l.id===bomConfiguration.selected);
  chooseBomConfiguration(target?.id||'');
}
function chooseBomConfiguration(id){
  bomLayoutEditing=id;const l=bomConfiguration.layouts.find(l=>l.id===id);
  bomLayoutColumns=[...(l?.columns||DEFAULT_BOM_COLUMNS)];
  $('bc-layout').value=id;$('bc-name').value=l?.name||'';$('bc-shared').checked=Boolean(l&&!l.owner);
  const editable=!l||Boolean(l.owner)||me.is_admin;
  $('bc-name').disabled=!editable;$('bc-shared').disabled=!editable;$('bc-delete').disabled=!l||!editable;
  $('bc-form').querySelector('[type=submit]').disabled=!editable;
  $('bc-use').disabled=!l;$('bc-status').textContent='';drawBomConfigurationFields(editable);
}
let bomLayoutCanEdit=true;
const selectedColumns=element=>new Set([...element.selectedOptions].map(option=>option.value));
function drawBomConfigurationFields(editable=true,selectedRight=new Set(),selectedLeft=new Set()){
  bomLayoutCanEdit=editable;
  const label=field=>`${field.name} — ${field.scope==='part'?'Part revision':field.scope==='occurrence'?'Occurrence':field.edit?.resource==='part'?'Part':'Built-in'}${field.unit?` (${field.unit})`:''}`;
  const available=bomConfiguration.fields.filter(field=>!bomLayoutColumns.includes(field.id)).sort((a,b)=>a.name.localeCompare(b.name));
  $('bc-available').innerHTML=available.map(field=>`<option value="${escape(field.id)}" ${selectedLeft.has(field.id)?'selected':''}>${escape(label(field))}</option>`).join('');
  $('bc-configured').innerHTML=bomLayoutColumns.map(id=>{const field=bomConfiguration.fields.find(field=>field.id===id);return `<option value="${escape(id)}" ${selectedRight.has(id)?'selected':''}>${escape(field?label(field):`${id} (Unavailable field)`)}</option>`;}).join('');
  $('bc-available').disabled=!editable;$('bc-configured').disabled=!editable;refreshBomColumnButtons();
}
function refreshBomColumnButtons(){
  const selected=selectedColumns($('bc-configured'));
  $('bc-add').disabled=!bomLayoutCanEdit||!$('bc-available').selectedOptions.length;
  $('bc-remove').disabled=!bomLayoutCanEdit||!selected.size;
  $('bc-up').disabled=!bomLayoutCanEdit||!bomLayoutColumns.some((id,i)=>selected.has(id)&&i>0&&!selected.has(bomLayoutColumns[i-1]));
  $('bc-down').disabled=!bomLayoutCanEdit||!bomLayoutColumns.some((id,i)=>selected.has(id)&&i+1<bomLayoutColumns.length&&!selected.has(bomLayoutColumns[i+1]));
}
$('bc-available').addEventListener('change',refreshBomColumnButtons);
$('bc-configured').addEventListener('change',refreshBomColumnButtons);
$('bc-add').addEventListener('click',()=>{
  if(!bomLayoutCanEdit)return;
  const selected=selectedColumns($('bc-available'));
  for(const id of selected)if(!bomLayoutColumns.includes(id))bomLayoutColumns.push(id);
  drawBomConfigurationFields(true,selected);
});
$('bc-remove').addEventListener('click',()=>{
  if(!bomLayoutCanEdit)return;
  const selected=selectedColumns($('bc-configured'));
  bomLayoutColumns=bomLayoutColumns.filter(id=>!selected.has(id));drawBomConfigurationFields(true,new Set(),selected);
});
function moveBomColumns(direction){
  if(!bomLayoutCanEdit)return;
  const selected=selectedColumns($('bc-configured'));
  const indices=Array.from({length:bomLayoutColumns.length},(_,i)=>i);if(direction>0)indices.reverse();
  for(const i of indices){const to=i+direction;if(to>=0&&to<bomLayoutColumns.length&&selected.has(bomLayoutColumns[i])&&!selected.has(bomLayoutColumns[to]))[bomLayoutColumns[i],bomLayoutColumns[to]]=[bomLayoutColumns[to],bomLayoutColumns[i]];}
  drawBomConfigurationFields(true,selected);
}
$('bc-up').addEventListener('click',()=>moveBomColumns(-1));
$('bc-down').addEventListener('click',()=>moveBomColumns(1));
$('bc-layout').addEventListener('change',()=>chooseBomConfiguration($('bc-layout').value));
$('bc-new').addEventListener('click',()=>chooseBomConfiguration(''));
$('bc-copy').addEventListener('click',()=>{const columns=[...bomLayoutColumns],name=$('bc-name').value;chooseBomConfiguration('');bomLayoutColumns=columns;$('bc-name').value=name?`${name} copy`:'';drawBomConfigurationFields();});
$('bc-form').addEventListener('submit',e=>{e.preventDefault();act(async()=>{const saved=await api('POST','/api/bom/layouts',{id:bomLayoutEditing,name:$('bc-name').value,columns:bomLayoutColumns,shared:me.is_admin&&$('bc-shared').checked});bomLayoutEditing=saved.id;await renderBomConfigurations();$('bc-status').textContent='Saved';});});
$('bc-use').addEventListener('click',()=>act(async()=>{await api('PUT','/api/bom/selection',{id:bomLayoutEditing});$('bc-status').textContent='Selected for CAD and PLM';}));
$('bc-delete').addEventListener('click',()=>act(async()=>{await api('DELETE',`/api/bom/layouts/${encodeURIComponent(bomLayoutEditing)}`);bomLayoutEditing='';await renderBomConfigurations();}));

function fieldValueEditor(field,value,disabled,attributes=''){
  if(field.encoding==='comma-list'&&Array.isArray(value))value=value.join(', ');
  const common=`${attributes} ${disabled?'disabled':''} aria-label="${escape(field.name)}"`;
  if(field.type==='bool')return `<input type="checkbox" ${common} ${value===true?'checked':''}>`;
  if(field.type==='enum')return `<select ${common}><option value="">${escape(field.empty_label||'')}</option>${(field.options||(field.values||[]).map(v=>({value:v,label:v}))).map(o=>`<option value="${escape(o.value)}" ${o.value===value?'selected':''}>${escape(o.label)}</option>`).join('')}</select>`;
  return `<input type="${field.type==='number'?'number':'text'}" ${field.type==='number'?'step="any"':''} ${common} value="${escape(value==null?'':String(value))}">`;
}
function editedFieldValue(field,input){if(field.encoding==='comma-list')return splitTags(input.value);if(field.type==='bool')return input.checked;if(input.value==='')return null;return field.type==='number'?Number(input.value):input.value;}
// Summary always shows the complete part schema, independent of BOM layouts.
let summaryReadSerial = 0;
async function loadSummaryFields(detail=currentPart, selectedRevision=$('summary-revision').value, {background=false}={}) {
  if (!detail || !detail.revision_views.length) return;
  const serial = ++summaryReadSerial;
  const revision = detail.revision_views.some(r => r.id === selectedRevision) ? selectedRevision : defaultRevision(detail);
  const revisionSelect = $('summary-revision');
  const optionsSignature = JSON.stringify(detail.revision_views.map(r => [r.id, r.label, r.lifecycle]));
  if (revisionSelect._summaryOptions !== optionsSignature) {
    revisionOptions(revisionSelect, detail, revision);
    revisionSelect._summaryOptions = optionsSignature;
  } else if (revisionSelect.value !== revision) revisionSelect.value = revision;
  const path = `/api/parts/${encodeURIComponent(detail.id)}`;
  const data = await api('GET', `${path}/revisions/${encodeURIComponent(revision)}/attributes`);
  if (serial !== summaryReadSerial || currentPart?.id !== detail.id) return;
  if (background && document.activeElement?.closest('.summary-attributes')) return;
  const draw = (container, fields, values, resource) => {
    // Keep input nodes stable during background reads and after value saves.
    // Rebuild only when the configured field definitions actually change.
    const schema = JSON.stringify(fields.map(({value, editable, ...definition}) => definition));
    if (container._partAttributeSchema !== schema) {
      container.innerHTML = fields.map(f => `<dt>${escape(f.name)}${f.unit ? ` (${escape(f.unit)})` : ''}</dt><dd>${partAttributeEditor(f, values[f.key], f.editable, `data-summary-field="${escape(f.key)}"`)}</dd>`).join('');
      container._partAttributeSchema = schema;
    }
    for (const input of container.querySelectorAll('[data-summary-field]')) {
      const field = fields.find(f => f.key === input.dataset.summaryField);
      const value = values[field.key];
      if (input.disabled !== !field.editable) input.disabled = !field.editable;
      const wrapper = input.closest('.part-attribute-input');
      if (wrapper.classList.contains('is-read-only') !== !field.editable) {
        wrapper.classList.toggle('is-read-only', !field.editable);
        const lock = wrapper.querySelector('.attribute-lock');
        lock.innerHTML = field.editable ? '' : PART_ATTRIBUTE_LOCK_ICON;
        if (field.editable) lock.removeAttribute('title'); else lock.title = 'Read-only';
      }
      if (field.type === 'bool') {
        if (input.checked !== (value === true)) input.checked = value === true;
      } else {
        const shown = field.encoding === 'comma-list' && Array.isArray(value) ? value.join(', ') : value == null ? '' : String(value);
        if (input.value !== shown) input.value = shown;
      }
      input.onchange = () => act(async () => {
        const value = editedFieldValue(field, input);
        input.disabled = true;
        try {
          await api('PATCH', resource === 'part' ? path : `${path}/revisions/${encodeURIComponent(revision)}/attributes`, resource === 'part' ? {[field.key]: value ?? ''} : {attributes: {[field.key]: value}});
          $('summary-field-status').textContent = 'Saved';
          if (resource === 'part') {
            const fresh = await api('GET', path);
            if (currentPart?.id === detail.id) { currentPart = fresh; $('part-name').textContent = fresh.name; }
          }
        } finally { await loadSummaryFields(currentPart, revision); }
      });
    }
  };
  draw($('summary-record-fields'), data.record_fields, Object.fromEntries(data.record_fields.map(f => [f.key, f.value])), 'part');
  draw($('summary-revision-fields'), data.fields.map(f => ({...f, editable: data.editable})), data.attributes, 'revision');
  const status = data.editable ? 'Revision fields are editable' : 'Revision fields are read-only';
  if ($('summary-field-status').textContent !== status) $('summary-field-status').textContent = status;
}
$('summary-revision').addEventListener('change', () => act(() => loadSummaryFields()));
async function loadRevisionFields(detail=currentPart){
  if(!detail)return;
  const revision=$('revision-field-revision').value;
  revisionOptions($('revision-field-revision'),detail,detail.revision_views.some(r=>r.id===revision)?revision:defaultRevision(detail));
  const rev=$('revision-field-revision').value;
  const data=await api('GET',`/api/parts/${encodeURIComponent(detail.id)}/revisions/${encodeURIComponent(rev)}/attributes`);
  const editable=data.editable;
  $('revision-field-values').innerHTML=data.fields.length?data.fields.map(f=>`<label>${escape(f.name)}${f.unit?` (${escape(f.unit)})`:''}${fieldValueEditor(f,data.attributes[f.key],!editable,`data-revision-field="${escape(f.key)}"`)}</label>`).join(''):'<p class="muted">No revision fields configured for this part type.</p>';
  for(const input of $('revision-field-values').querySelectorAll('[data-revision-field]'))input.addEventListener('change',()=>act(async()=>{
    const field=data.fields.find(f=>f.key===input.dataset.revisionField);input.disabled=true;
    try{await api('PATCH',`/api/parts/${encodeURIComponent(detail.id)}/revisions/${encodeURIComponent(rev)}/attributes`,{attributes:{[field.key]:editedFieldValue(field,input)}});}finally{await loadRevisionFields(detail);}
  }));
}
$('revision-field-revision').addEventListener('change',()=>act(()=>loadRevisionFields()));
const bomAttributeView={show:false};
async function drawConfiguredBom(bom,serial){
  await drawBomAttributes({bom,head:$('bom-head'),rows:$('bom-rows'),selection:$('bom-layout'),editButton:$('bom-edit-attributes'),status:$('bom-edit-status'),treeControls:$('bom-tree-controls'),view:bomAttributeView,reload:loadBom,isCurrent:()=>serial===bomReadSerial});
}
// Every PLM BOM uses this editor, including Home, Parts and review detail panes.
async function drawBomAttributes({bom,head,rows,selection,editButton,status,treeControls,view,reload,isCurrent}){
  const config=await api('GET','/api/bom/configuration');
  if(!isCurrent())return;
  selection.innerHTML='<option value="">Default</option>'+config.layouts.map(l=>`<option value="${escape(l.id)}">${escape(l.name)} (${l.owner?'Personal':'Shared'})</option>`).join('');
  selection.value=config.selected||'';
  selection.onchange=()=>act(async()=>{await api('PUT','/api/bom/selection',{id:selection.value});view.show=false;await reload();});
  editButton.hidden=!me.can_author;
  editButton.textContent=view.show?'Hide extra attribute columns':'Edit attributes';
  editButton.setAttribute('aria-pressed',String(view.show));
  editButton.onclick=()=>act(async()=>{view.show=!view.show;await reload();});
  const layout=config.layouts.find(l=>l.id===config.selected),ids=[...(layout?.columns||DEFAULT_BOM_COLUMNS)];
  if(view.show)for(const field of config.fields){
    if(field.editable&&!ids.includes(field.id)&&(field.scope!=='part'||bom.lines.some(line=>line.part_type===field.part_type)))ids.push(field.id);
  }
  const fields=ids.map(id=>config.fields.find(f=>f.id===id)).filter(f=>f&&(bom.flat||f.id!=='builtin.number'));
  const treeKey=`${bom.part_id}/${bom.revision_id}`;
  if(view.treeKey!==treeKey){view.treeKey=treeKey;view.collapsed=new Set();}
  const branches=new Set(),lastSibling=new Set(),siblings=new Map();
  for(const line of bom.lines){
    const parent=line.position.split('.').slice(0,-1).join('.');
    if(parent)branches.add(parent);
    if(!siblings.has(parent))siblings.set(parent,[]);siblings.get(parent).push(line.position);
  }
  for(const children of siblings.values())lastSibling.add(children.at(-1));
  rows.closest('table').setAttribute('role',bom.flat?'table':'treegrid');
  treeControls.hidden=bom.flat;
  const maxDepth=bom.lines.reduce((depth,line)=>Math.max(depth,line.level),1);
  treeControls.innerHTML=`<button type="button" class="ghost" data-bom-expand-all>Expand all</button><button type="button" class="ghost" data-bom-collapse-all>Collapse all</button><label class="inline-select">Depth <select data-bom-expand-depth>${Array.from({length:maxDepth},(_,i)=>`<option value="${i+1}" ${Number(view.expandDepth||1)===i+1?'selected':''}>${i+1}</option>`).join('')}</select></label><button type="button" class="ghost" data-bom-expand-to>Expand to depth</button>`;
  treeControls.querySelector('[data-bom-expand-all]').onclick=()=>{view.collapsed.clear();paint();};
  treeControls.querySelector('[data-bom-collapse-all]').onclick=()=>{view.collapsed=new Set(branches);paint();};
  treeControls.querySelector('[data-bom-expand-depth]').onchange=event=>{view.expandDepth=Number(event.target.value);};
  treeControls.querySelector('[data-bom-expand-to]').onclick=()=>{
    const depth=Number(treeControls.querySelector('[data-bom-expand-depth]').value);view.expandDepth=depth;
    view.collapsed=new Set(bom.lines.filter(line=>branches.has(line.position)&&line.level>=depth).map(line=>line.position));paint();
  };
  const treeCell=line=>{
    const segments=line.position.split('.');
    const guides=segments.slice(0,-1).map((_,i)=>`<span class="bom-tree-guide ${lastSibling.has(segments.slice(0,i+1).join('.'))?'':'continues'}" data-bom-guide-index="${i}"></span>`).join('');
    const expanded=!view.collapsed.has(line.position),branch=branches.has(line.position);
    return `<td class="bom-tree-cell"><div class="bom-tree-entry">${guides}<span class="bom-tree-guide elbow ${lastSibling.has(line.position)?'last':''}" data-bom-guide-index="${segments.length-1}"></span>${branch?`<button type="button" class="bom-tree-toggle" data-bom-toggle="${escape(line.position)}" aria-expanded="${expanded}" aria-label="${expanded?'Collapse':'Expand'} ${escape(line.number)} at ${escape(line.position)}">${expanded?'▾':'▸'}</button>`:'<span class="bom-tree-spacer"></span>'}<svg class="bom-tree-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" aria-hidden="true">${line.assembly?'<path d="M3 6h7l2 3h9v12H3z"/>':'<path d="M5 3h9l5 5v13H5zM14 3v6h5"/>'}</svg><a href="#/part/${encodeURIComponent(line.part_id)}" title="Position ${escape(line.position)}">${escape(line.number)}</a></div></td>`;
  };
  const hasWarnings=bom.lines.some(line=>line.warnings?.length);
  head.innerHTML=`<tr>${hasWarnings?'<th class="bom-warning-column" aria-label="Warnings"></th>':''}${bom.flat?'':'<th>Part</th>'}${fields.map(f=>`<th>${escape(f.name)}${f.unit?` (${escape(f.unit)})`:''}</th>`).join('')}</tr>`;
  const valueOf=(line,f)=>f.scope==='part'?(f.part_type===line.part_type?line.part_values[f.key]:null):f.scope==='occurrence'?(f.id.startsWith('builtin.')?line[f.key]:line.occurrence_attributes[f.key]):line[f.key];
  function paint(){
  rows.innerHTML=bom.lines.map((line,i)=>{
    const ancestors=line.position.split('.').slice(0,-1).map((_,j)=>line.position.split('.').slice(0,j+1).join('.'));
    if(!bom.flat&&ancestors.some(parent=>view.collapsed.has(parent)))return '';
    return `<tr data-bom-position="${escape(line.position)}" ${bom.flat?'':`aria-level="${line.level}" ${branches.has(line.position)?`aria-expanded="${!view.collapsed.has(line.position)}"`:''}`}>${hasWarnings?`<td class="bom-warning-column">${line.warnings?.length?`<button type="button" class="bom-warning-icon" data-bom-warning="${i}" title="${escape(line.warnings.join('\n'))}" aria-label="Warnings for ${escape(line.number)} rev ${escape(line.revision_label)}">⚠</button>`:''}</td>`:''}${bom.flat?'':treeCell(line)}${fields.map(f=>{
    const value=valueOf(line,f),applicable=f.edit?.resource==='part'||(f.scope==='part'?f.part_type===line.part_type:line.occurrence_ids.length>0);
    if(f.editable&&applicable&&me.can_author)return `<td>${fieldValueEditor(f,value,false,`data-bom-row="${i}" data-bom-field="${escape(f.id)}"`)}</td>`;
    if(f.id==='builtin.number')return `<td><a href="#/part/${encodeURIComponent(line.part_id)}">${escape(line.number)}</a></td>`;
    return `<td>${escape(value==null?'—':String(value))}</td>`;
  }).join('')}</tr>`;}).join('');
  // The guide column offset is a custom property; set on the element, not written inline (the CSP allows no style attribute).
  for(const guide of rows.querySelectorAll('.bom-tree-guide[data-bom-guide-index]'))guide.style.setProperty('--bom-guide-index',guide.dataset.bomGuideIndex);
  for(const button of rows.querySelectorAll('[data-bom-toggle]')){
    const toggle=()=>{const position=button.dataset.bomToggle;if(view.collapsed.has(position))view.collapsed.delete(position);else view.collapsed.add(position);paint();rows.querySelector(`[data-bom-toggle="${CSS.escape(position)}"]`)?.focus();};
    button.onclick=toggle;button.onkeydown=event=>{if((event.key==='ArrowRight'&&view.collapsed.has(button.dataset.bomToggle))||(event.key==='ArrowLeft'&&!view.collapsed.has(button.dataset.bomToggle))){event.preventDefault();toggle();}};
  }
  for(const button of rows.querySelectorAll('[data-bom-warning]'))button.onclick=()=>{
    const line=bom.lines[Number(button.dataset.bomWarning)];
    let dialog=$('bom-warning-dialog');
    if(!dialog){dialog=document.createElement('dialog');dialog.id='bom-warning-dialog';document.body.append(dialog);}
    dialog.innerHTML=`<form method="dialog"><h2>${escape(line.number)} rev ${escape(line.revision_label)}</h2><ul>${line.warnings.map(w=>`<li>${escape(w)}</li>`).join('')}</ul><button type="submit">Close</button></form>`;
    dialog.showModal();
  };
  status.textContent=me.can_author?'Edit attribute values in the table. Changes save automatically.':'';
  for(const input of rows.querySelectorAll('[data-bom-field]'))input.addEventListener('change',()=>act(async()=>{
    const line=bom.lines[Number(input.dataset.bomRow)],field=fields.find(f=>f.id===input.dataset.bomField);input.disabled=true;
    const value=editedFieldValue(field,input);
    const target=field.edit;
    const part=target.resource==='occurrence'?line.owner_part:line.part_id,revision=target.resource==='occurrence'?line.owner_revision:line.revision_id;
    const base=`/api/parts/${encodeURIComponent(part)}`;
    const path=target.resource==='part'?base:`${base}/revisions/${encodeURIComponent(revision)}/${target.resource==='revision'?'attributes':'occurrences'}`;
    const payload=target.resource==='part'?{[target.key]:value??''}:{...(target.resource==='occurrence'?{ids:line.occurrence_ids}:{}),attributes:{[target.key]:value}};
    try{
      await api('PATCH',path,payload);
      if(isCurrent()){await reload();status.textContent='Attributes saved.';}
    }catch(error){
      if(isCurrent()){await reload();status.textContent=error.message;}
      throw error;
    }
  }));
  }
  paint();
}

let bomMetadataPolling=false;
setInterval(async()=>{
  if(!me||!currentPart||document.hidden||$('view-part').hidden||bomMetadataPolling||document.querySelector('.work-area').inert)return;
  if(document.activeElement?.closest('#bom-rows,#revision-field-values,[data-object-panel="summary"]'))return;
  bomMetadataPolling=true;
  try{if(objectTab==='structure'&&structure.tab==='bom')await loadBom();else if(objectTab==='revisions')await loadRevisionFields();else if(objectTab==='summary')await loadSummaryFields(currentPart,$('summary-revision').value,{background:true});}catch{}finally{bomMetadataPolling=false;}
},5000);
