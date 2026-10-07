// Administrator onboarding. Each step saves through the normal authorized APIs;
// optional imports report per-part failures and can safely be retried.
'use strict';
const SetupWizard = (() => {
  const steps = ['Part numbers', 'Catalog taxonomy', 'User permissions', 'Lifecycle statuses', 'KiCad library', 'Mechanical library', 'Finish'];
  const taxonomy = ['Mechanical/Fasteners/Screws', 'Mechanical/Fasteners/Nuts', 'Mechanical/Fasteners/Washers', 'Mechanical/Stock/Bar stock', 'Mechanical/Stock/I beams', 'Mechanical/Stock/Aluminum extrusions', 'Mechanical/Machined parts', 'Mechanical/Assemblies', 'Electrical/Components', 'Electrical/Connectors', 'Electrical/PCBs', 'Documents'];
  let step = 0, settings, types, categories, users, busy = false, loaded = false;
  let stopLibraries = false;
  const checkpointKey = 'plm-kicad-library-import-v1';
  let activeCheckpoint = null, storageProblem = '';
  const reports = new Map();
  const validCheckpoint = state => state && state.schema === 1 && state.release === '9.0.9.1'
    && typeof state.id === 'string' && state.id.length < 100
    && typeof state.part_type === 'string' && typeof state.category === 'string'
    && state.context && typeof state.context.prefix === 'string' && Number.isInteger(state.context.digits) && typeof state.context.category_path === 'string'
    && Array.isArray(state.libraries) && state.libraries.length > 0 && state.libraries.length <= 10000
    && state.libraries.every(name => typeof name === 'string' && /^[A-Za-z0-9_]{1,100}$/.test(name))
    && new Set(state.libraries).size === state.libraries.length
    && Number.isInteger(state.index) && state.index >= 0 && state.index <= state.libraries.length
    && state.counts && typeof state.counts === 'object' && !Array.isArray(state.counts)
    && Object.values(state.counts).every(row => row && Object.values(row).every(n=>Number.isSafeInteger(n)&&n>=0))
    && Array.isArray(state.failures) && state.failures.every(f=>f && state.libraries.includes(f.library) && typeof f.error === 'string')
    && (!state.retry_libraries || (Array.isArray(state.retry_libraries) && state.retry_libraries.every(name=>state.libraries.includes(name)) && Number.isInteger(state.retry_index) && state.retry_index>=0 && state.retry_index<=state.retry_libraries.length));
  const hasPending = state => state && (state.index<state.libraries.length || (state.retry_libraries && state.retry_index<state.retry_libraries.length));
  const checkpoint = () => {
    if(validCheckpoint(activeCheckpoint))return activeCheckpoint;
    try { const state=JSON.parse(localStorage.getItem(checkpointKey)||'null');return validCheckpoint(state)?state:null; } catch { return null; }
  };
  const saveCheckpoint = state => {
    activeCheckpoint=state;
    try { localStorage.setItem(checkpointKey,JSON.stringify(state));return true; }
    catch(error){ storageProblem=`Progress could not be saved (${error.message}). Paused after this library; export the report before closing this page.`;stopLibraries=true;return false; }
  };
  function reportStore() {
    return new Promise((resolve,reject)=>{
      const request=indexedDB.open('plm-kicad-import-reports',1);
      request.onupgradeneeded=()=>request.result.createObjectStore('reports',{keyPath:'key'});
      request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);
    });
  }
  async function saveReport(state,library,report) {
    const record={key:`${state.id}:${library}:${Date.now()}`,import_id:state.id,library,...report};reports.set(record.key,record);
    const db=await reportStore();
    try { await new Promise((resolve,reject)=>{const tx=db.transaction('reports','readwrite');tx.objectStore('reports').put(record);tx.oncomplete=resolve;tx.onerror=()=>reject(tx.error);tx.onabort=()=>reject(tx.error);}); }
    finally { db.close(); }
  }
  async function importReport() {
    const state=checkpoint();if(!state)throw new Error('No saved import report.');
    let stored=[];
    try { const db=await reportStore();try {stored=await new Promise((resolve,reject)=>{const request=db.transaction('reports').objectStore('reports').getAll();request.onsuccess=()=>resolve(request.result);request.onerror=()=>reject(request.error);});}finally{db.close();} } catch { /* In-memory details remain available after a storage failure. */ }
    const merged=new Map(stored.filter(r=>r.import_id===state.id).map(r=>[r.key,r]));
    for(const [key,record] of reports)if(record.import_id===state.id)merged.set(key,record);
    return {checkpoint:state,library_reports:[...merged.values()],storage_warning:storageProblem};
  }
  const actions = {};
  const root = () => $('view-setup');
  const input = (id, label, value = '', type = 'text') => `<label>${escape(label)}<input id="${id}" type="${type}" value="${escape(value)}"></label>`;
  const select = (id, label, rows) => `<label>${escape(label)}<select id="${id}">${rows}</select></label>`;
  const typeOptions = () => types.filter(t => t.mode.kind === 'counter').map(t => `<option value="${escape(t.id)}">${escape(t.name)}</option>`).join('');
  const kicadTypeOptions = () => (types.some(t=>t.id==='kicad'&&t.mode.kind==='counter') ? '' : '<option value="@kicad" selected>KiCad electrical parts (ELEC00000000001)</option>') + types.filter(t=>t.mode.kind==='counter').map(t=>`<option value="${escape(t.id)}" ${t.id==='kicad'?'selected':''}>${escape(t.name)}</option>`).join('');
  const categoryOptions = () => '<option value="@auto">Use starter taxonomy</option><option value="">Unclassified</option>' + categories.map(c => `<option value="${escape(c.id)}">${escape(c.path)}</option>`).join('');
  async function refresh() {
    [settings, types, categories, users] = await Promise.all([api('GET','/api/settings'), api('GET','/api/part-types'), api('GET','/api/categories'), api('GET','/api/users')]);
    lifecycleOptions = settings.status_options;
  }
  function draw() {
    const content = [
      () => `<p>Choose how each type assigns numbers. Counter types assign a prefix and padded sequence; free text accepts supplier numbers; patterns validate the entire number; scripts use an administrator script.</p>
        <ul>${types.map(t => `<li>${escape(t.name)}: ${modeText(t.mode)} ${t.mode.kind === 'counter' ? `· next ${escape(t.next_number)}` : ''} <a href="#/types">Edit</a></li>`).join('')}</ul>
        <h3>Add a part type</h3>${input('sw-type-id','Type ID')}${input('sw-type-name','Name')}${select('sw-mode','Number rule','<option value="counter">Counter</option><option value="free">Free text</option><option value="pattern">Pattern</option><option value="script">Script</option>')}
        ${input('sw-prefix','Counter prefix','PART')}${input('sw-digits','Counter digits',6,'number')}${input('sw-start','Counter starts at',1,'number')}${input('sw-rule','Pattern regex or script path')}
        <p id="sw-preview" class="muted"></p><button type="button" id="sw-add-type">Add type</button>`,
      () => `<p>Start with an engineering catalog. Edit these paths, one per line, to choose your taxonomy. Existing categories are kept. You can refine attributes and inheritance in <a href="#/catalog">Classification</a>.</p><label>Catalog paths<textarea id="sw-taxonomy" rows="14">${escape(taxonomy.join('\n'))}</textarea></label><button type="button" id="sw-add-tree">Add taxonomy</button><p>${categories.length} categories currently configured.</p>`,
      () => `<p>Every account can read the catalog. Author creates and edits work; Check-in can release and break checkout locks; Admin manages the server and includes those permissions; Worker runs CAD bake jobs. Viewer provides read access.</p>
        <table><thead><tr><th>User</th>${['viewer','author','checkin','worker','admin'].map(g => `<th>${g}</th>`).join('')}</tr></thead><tbody>${users.map((u,i) => `<tr><td>${escape(u.display_name || u.username)}</td>${['viewer','author','checkin','worker','admin'].map(g => `<td><input type="checkbox" data-user="${i}" data-group="${g}" aria-label="${escape(u.username)} ${g}" ${u.groups.includes(g)?'checked':''} ${u.id === me.id && g === 'admin'?'disabled':''}></td>`).join('')}</tr>`).join('')}</tbody></table><button type="button" id="sw-save-users">Save permissions</button>
        <h3>Add a user</h3>${input('sw-username','Username')}${input('sw-password','Password (at least 8 characters)','','password')}${select('sw-role','Permissions','<option value="viewer">Viewer</option><option value="author">Author</option><option value="checkin">Author and Check-in</option><option value="admin">Administrator</option><option value="worker">Bake worker</option>')}<button type="button" id="sw-add-user">Add user</button>`,
      () => `<p>Define the status names your team uses and choose which workflow options to offer. “none” is the initial editable state; “review” opens an approval round; “released” freezes the revision; “obsolete” retires it. Superseded is assigned automatically when a newer revision releases.</p>
        <table><thead><tr><th>Workflow</th><th>Status name</th><th>Available</th></tr></thead><tbody>${settings.status_options.map((s,i) => `<tr><td>${escape(s.state)}</td><td><input id="sw-status-${i}" aria-label="Name for ${s.state}" value="${escape(s.name)}" required maxlength="80"></td><td><input id="sw-enabled-${i}" type="checkbox" aria-label="Enable ${s.state}" ${s.enabled?'checked':''} ${['draft','superseded'].includes(s.state)?'disabled':''}></td></tr>`).join('')}</tbody></table><button type="button" id="sw-save-status">Save status options</button>`,
      () => `<p>Import all official KiCad symbol libraries at release 9.0.9.1, including ICs, microcontrollers and relays, or choose one library. Each symbol includes its assigned footprint and available linked STEP models. Unreferenced repository assets are not installed. Generic symbols need a footprint choice, and some footprints have no STEP model; these gaps are reported.</p><p>Classification follows KiCad’s function / manufacturer / series library names, for example KiCad / MCU / ST / STM32F4, KiCad / Amplifier / Operational, and KiCad / Relay. Existing imported KiCad parts can be reclassified without changing their documents, revisions, locks or status.</p>
        ${select('sw-import-scope','Import coverage','<option value="all">All symbol libraries</option><option value="single">One symbol library</option>')}${input('sw-library','Single library name','Device')}${select('sw-import-type','Counter part type',kicadTypeOptions())}${select('sw-import-category','Root for KiCad taxonomy','<option value="">Catalog root</option>' + categories.map(c => `<option value="${escape(c.id)}">${escape(c.path)}</option>`).join(''))}<button type="button" id="sw-import-kicad">Fetch and import KiCad libraries</button><button type="button" id="sw-resume-kicad" ${hasPending(checkpoint()) ? '' : 'disabled'}>Resume library import</button><button type="button" id="sw-retry-kicad" ${checkpoint()?.failures.length?'':'disabled'}>Retry libraries with failures</button><button type="button" id="sw-stop-kicad" disabled>Pause after current library</button><button type="button" id="sw-repair-kicad">Reclassify existing KiCad parts</button><button type="button" id="sw-show-kicad-report">View import details</button><button type="button" id="sw-download-kicad-report">Download import report</button><pre id="sw-kicad-report" class="script-output" hidden></pre><p class="muted">All libraries can take hours. Keep this page open while importing. Progress and detailed reports are saved in this browser after each library. Pause and resume safely; retry preserves edited documents and enriches unchanged imports. Failures and warnings remain available in the report; retry processes only libraries with failures.</p>`,
      () => `<p>Pre-populate parametric families with editable dimensions in millimeters. Cross sections include a 100 mm sample length. Fasteners use simplified geometry without threads; structural sections have sharp corners. Check supplier dimensions before releasing.</p>
        ${[['fasteners','Fasteners: socket head screws and washers'],['bar-stock','Bar stock: rectangular and round'],['i-beams','I beams: IPE 100 and IPE 200 nominal cross sections'],['aluminum','Aluminum extrusions: 20/30/40 mm generic T slot, square tube and equal angle']].map(([id,label]) => `<label><input type="checkbox" data-pack="${id}" checked> ${label}</label>`).join('')}
        ${select('sw-import-type','Counter part type for families',typeOptions())}${select('sw-import-category','Catalog category',categoryOptions())}<p>Imports also configure a Starter family member type with free text numbering. Member numbers derive from each family’s allocated number; you can change the member type and table later.</p><button type="button" id="sw-import-packs" ${typeOptions()?'':'disabled'}>Import selected families</button>`,
      () => `<p>Your server has ${types.length} part types, ${categories.length} catalog categories and ${users.length} users.</p><p>Status options: ${settings.status_options.filter(s=>s.enabled).map(s=>escape(s.name)).join(', ')}.</p><p>Finish marks server setup complete. You can reopen this wizard from the Administration menu at any time.</p>`
    ][step]();
    root().innerHTML = `<h1>PLM server setup</h1><p>Step ${step+1} of ${steps.length}: <strong>${steps[step]}</strong></p><ol class="setup-steps">${steps.map((name,i)=>`<li ${i===step?'aria-current="step"':''}>${escape(name)}</li>`).join('')}</ol><form id="sw-form"><div class="setup-content">${content}</div><p id="sw-result" role="status" aria-live="polite"></p><menu><button type="button" id="sw-back" ${step===0?'disabled':''}>Back</button><a href="#/workspace">Continue setup later</a><button type="submit">${step===steps.length-1?'Finish setup':'Next'}</button></menu></form>`;
    $('sw-form').onsubmit = e => { e.preventDefault(); run(async()=>{
      if(step===0 && ($("sw-type-id").value.trim() || $("sw-type-name").value.trim())) await actions["sw-add-type"]();
      if(step===1) await actions["sw-add-tree"]();
      if(step===2) await actions["sw-save-users"]();
      if(step===3) await saveStatuses();
      if(step===steps.length-1) { await api('PATCH','/api/settings',{setup_completed:true}); location.hash='#/workspace'; return; }
      step++; await refresh(); draw();
    }); };
    $('sw-back').onclick=()=>{step--;draw();};
    bind('sw-add-type', async()=>{
      const kind=$('sw-mode').value;
      const mode={kind}; if(kind==='pattern')mode.regex=$('sw-rule').value; if(kind==='script')mode.script=$('sw-rule').value;
      await api('POST','/api/part-types',{id:$('sw-type-id').value,name:$('sw-type-name').value,prefix:$('sw-prefix').value,digits:Number($('sw-digits').value),start:Number($('sw-start').value),mode});
      await refresh();draw();result('Part type added.');
    });
    if($('sw-mode')) {
      const preview=()=>{const kind=$('sw-mode').value; $('sw-preview').textContent=kind==='counter'?`Next number: ${$('sw-prefix').value}${$('sw-start').value.padStart(Math.min(18,Math.max(1,Number($('sw-digits').value)||1)),'0')}`:kind==='pattern'?'Numbers must match the full regex.':kind==='script'?'Install the script in the server scripts directory before creating parts.':'Users type each part number.';};
      ['sw-mode','sw-prefix','sw-digits','sw-start'].forEach(id=>$(id).oninput=preview);preview();
    }
    bind('sw-add-tree', async()=>{
      const paths=$('sw-taxonomy').value.split('\n').map(p=>p.split('/').map(s=>s.trim())).filter(p=>p.some(Boolean));
      if(paths.some(p=>p.some(s=>!s)))throw new Error('Each catalog path needs nonempty segments.');
      for(const path of paths){let parent='';for(const name of path){
        let category=categories.find(c=>c.parent===parent && c.name.toLowerCase()===name.toLowerCase());
        if(!category){const base='setup-'+[parent,name].join('-').toLowerCase().replace(/[^a-z0-9_-]+/g,'-');let id=base, suffix=1;while(categories.some(c=>c.id===id))id=`${base}-${suffix++}`;
          category=await api('POST','/api/categories',{id,name,parent});categories.push(category);
        }parent=category.id;
      }}await refresh();draw();result('Catalog taxonomy added.');
    });
    bind('sw-save-users',async()=>{for(let i=0;i<users.length;i++){const groups=[...root().querySelectorAll(`[data-user="${i}"]:checked`)].map(el=>el.dataset.group);const custom=users[i].groups.filter(g=>!['viewer','author','checkin','worker','admin'].includes(g));if(JSON.stringify([...groups,...custom].sort())!==JSON.stringify([...users[i].groups].sort()))await api('PATCH',`/api/users/${encodeURIComponent(users[i].id)}`,{groups:[...groups,...custom]});}await refresh();draw();result('Permissions saved.');});
    bind('sw-add-user',async()=>{const role=$('sw-role').value;await api('POST','/api/users',{username:$('sw-username').value,password:$('sw-password').value,groups:role==='checkin'?['author','checkin']:[role]});await refresh();draw();result('User added.');});
    bind('sw-save-status',async()=>{await saveStatuses();result('Status options saved.');});
    bind('sw-import-kicad',async()=>{
      if($('sw-import-scope').value==='single')await importPack('kicad',$('sw-library').value);
      else await importLibraries(false);
    });
    bind('sw-resume-kicad',async()=>{await importLibraries(true);});
    bind('sw-retry-kicad',async()=>{await importLibraries(true,true);});
    bind('sw-show-kicad-report',async()=>{const report=await importReport();$('sw-kicad-report').textContent=JSON.stringify(report,null,2);$('sw-kicad-report').hidden=false;});
    bind('sw-download-kicad-report',async()=>{const report=await importReport();const url=URL.createObjectURL(new Blob([JSON.stringify(report,null,2)],{type:'application/json'}));const link=document.createElement('a');link.href=url;link.download='kicad-import-report.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
    if($('sw-stop-kicad'))$('sw-stop-kicad').onclick=()=>{stopLibraries=true;$('sw-stop-kicad').disabled=true;};
    bind('sw-repair-kicad',async()=>{const report=await api('POST','/api/setup/kicad-taxonomy',{category:$('sw-import-category').value});result(`${report.reclassified} of ${report.parts} identified KiCad parts reclassified. Documents, revisions, locks and lifecycle status preserved.`);});
    bind('sw-import-packs',async()=>{const packs=[...root().querySelectorAll('[data-pack]:checked')].map(el=>el.dataset.pack);if(!packs.length)throw new Error('Select at least one starter pack.');for(const pack of packs)await importPack(pack);});
  }
  function result(message) { $('sw-result').textContent=message; }
  async function run(fn) {
    if(busy)return;
    busy=true;root().querySelectorAll('button').forEach(b=>b.disabled=true);
    try { await fn(); } catch(e) { result(e.message); }
    finally { busy=false;root().querySelectorAll('button').forEach(b=>b.disabled=false);if(step===0)$('sw-back').disabled=true;if($('sw-stop-kicad'))$('sw-stop-kicad').disabled=true;if($('sw-resume-kicad'))$('sw-resume-kicad').disabled=!(hasPending(checkpoint()));if($('sw-retry-kicad'))$('sw-retry-kicad').disabled=!checkpoint()?.failures.length;if(!typeOptions()&&$('sw-import-packs'))$('sw-import-packs').disabled=true; }
  }
  function bind(id,fn){actions[id]=fn;if($(id))$(id).onclick=()=>run(fn);}
  async function saveStatuses() { const status_options=settings.status_options.map((s,i)=>({...s,name:$(`sw-status-${i}`).value.trim(),enabled:$(`sw-enabled-${i}`).checked}));settings=await api('PATCH','/api/settings',{status_options});lifecycleOptions=settings.status_options; }
  async function kicadPartType() {
    const chosen=$('sw-import-type').value;
    if(chosen!=='@kicad')return chosen;
    const type=await api('POST','/api/setup/kicad-part-type');
    types.push({...type,mode:{kind:'counter'}});
    const option=$('sw-import-type').querySelector('option[value="@kicad"]');
    if(option){option.value=type.id;option.textContent=type.name;}
    return type.id;
  }
  function bulkProgress(state, library='') {
    const totals={created:0,updated:0,skipped:0,excluded:0,installed_footprints:0,installed_models:0,needs_footprint:0,without_model:0,failed_parts:0,model_reference_errors:0};
    for(const row of Object.values(state.counts))for(const key of Object.keys(totals))totals[key]+=row[key]||0;
    result(`${state.release}: ${state.index}/${state.libraries.length} libraries processed.${library?` Importing ${library}…`:''}\nLatest library results: ${totals.created} created, ${totals.updated} upgraded, ${totals.skipped} already imported; ${totals.excluded} excluded for missing footprints or usable 3D. ${totals.installed_footprints} footprints and ${totals.installed_models} parts with 3D installed. ${totals.needs_footprint} need a footprint choice; ${totals.without_model} source parts have no imported 3D. ${totals.failed_parts} part failures; ${totals.model_reference_errors} model reference failures. ${state.failures.length} libraries need retry.${state.failures.map(f=>`\n${f.library}: ${f.error}`).join('')}${storageProblem?`\n${storageProblem}`:''}`);
  }
  async function importLibraries(resume,retry=false) {
    let state=resume?checkpoint():null;
    if(resume&&!state)throw new Error('Saved progress is missing or invalid. Start a new import.');
    stopLibraries=false;storageProblem='';
    if(!state){
      result('Discovering all official KiCad symbol libraries…');
      const index=await api('GET','/api/setup/kicad-libraries');
      const part_type=await kicadPartType();
      const kind=types.find(t=>t.id===part_type),category=$('sw-import-category').value;
      state={schema:1,id:typeof crypto.randomUUID==='function'?crypto.randomUUID():`${Date.now()}-${Math.random().toString(36).slice(2)}`,release:index.release,libraries:index.libraries,index:0,part_type,category,context:{prefix:kind.prefix,digits:kind.digits,category_path:category?categories.find(c=>c.id===category)?.path:''},counts:{},failures:[]};
      if(!validCheckpoint(state))throw new Error('KiCad returned an invalid library inventory.');
      if(!saveCheckpoint(state)){result(storageProblem);return;}
    }
    const [currentTypes,currentCategories]=await Promise.all([api('GET','/api/part-types'),api('GET','/api/categories')]);
    const kind=currentTypes.find(t=>t.id===state.part_type&&t.mode.kind==='counter'),category=state.category?currentCategories.find(c=>c.id===state.category):null;
    if(!kind||(state.category&&!category))throw new Error('The saved part type or taxonomy root no longer exists. Start a new import with the current options.');
    if(kind.prefix!==state.context.prefix||kind.digits!==state.context.digits||(category?.path||'')!==state.context.category_path)throw new Error('The saved numbering rule or taxonomy root has changed. Start a new import with the current options.');
    $('sw-import-type').value=state.part_type;$('sw-import-category').value=state.category;
    if(retry){
      // Preserve the full run's reports/counts; this pass revisits only failures.
      state.retry_libraries=state.failures.map(f=>f.library);
      state.retry_index=0;
    }
    const retrying=Array.isArray(state.retry_libraries)&&state.retry_index<state.retry_libraries.length;
    const work=retrying?state.retry_libraries:state.libraries;
    $('sw-stop-kicad').disabled=false;
    while((retrying?state.retry_index:state.index)<work.length&&!stopLibraries){
      const library=work[retrying?state.retry_index:state.index];bulkProgress(state,library);
      let detail;
      try {
        const report=await api('POST','/api/setup/import',{pack:'kicad',library,part_type:state.part_type,category:state.category});
        state.counts[library]={};
        for(const key of ['created','updated','skipped','excluded','installed_footprints','installed_models','needs_footprint','without_model','model_reference_errors'])state.counts[library][key]=report[key]||0;
        state.counts[library].failed_parts=report.failed.length;
        state.failures=state.failures.filter(f=>f.library!==library);
        if(report.failed.length)state.failures.push({library,error:`${report.failed.length} part failures; ${report.model_reference_errors} model reference failures. View import details.`});
        detail=report;
      } catch(error) {
        state.failures=state.failures.filter(f=>f.library!==library);state.failures.push({library,error:error.message});detail={error:error.message};
      }
      try {await saveReport(state,library,detail);}catch(error){storageProblem=`Detailed report could not be saved (${error.message}). Paused after this library; export the report before closing.`;stopLibraries=true;}
      if(retrying)state.retry_index++;else state.index++;
      saveCheckpoint(state);bulkProgress(state);
    }
    const complete=(retrying?state.retry_index:state.index)===work.length;
    result(`${$('sw-result').textContent}\n${complete?'Selected libraries processed. View details or retry libraries with failures.':'Paused; use Resume library import to continue.'}`);
  }
  async function importPack(pack,library='') {
    const previous=$('sw-result').textContent;
    result(`${previous}\nImporting ${pack}${library?` / ${library}`:''}…`);
    const paths={kicad:'Electrical / Components',fasteners:'Mechanical / Fasteners','bar-stock':'Mechanical / Stock / Bar stock','i-beams':'Mechanical / Stock / I beams',aluminum:'Mechanical / Stock / Aluminum extrusions'};
    const chosen=$('sw-import-category').value;const category=chosen==='@auto'?(categories.find(c=>c.path.toLowerCase()===paths[pack]?.toLowerCase())?.id || ''):chosen;
    const part_type=pack==='kicad'?await kicadPartType():$('sw-import-type').value;
    const report=await api('POST','/api/setup/import',{pack,library,part_type,category});
    result(`${previous}\n${pack}: ${report.created} created, ${report.skipped} already imported.${pack==='kicad'?` ${report.updated} upgraded; ${report.excluded||0} excluded for missing footprints or usable 3D; saved documents include ${report.installed_footprints} footprints and ${report.installed_models} parts with 3D models. Source coverage: ${report.footprints}/${report.symbols} footprints, ${report.models}/${report.symbols} parts with 3D models. ${report.needs_footprint} need a footprint choice; ${report.footprints_without_model_reference} footprints have no model reference; ${report.model_reference_errors} model references could not be imported. ${report.without_model} source parts have no imported 3D model.`:''}${report.failed.map(f=>`\n${f.name}: ${f.error}`).join('')}${report.warnings.map(w=>`\n${w}`).join('')}`);
  }
  return { async open(){if(busy)return;if(!loaded){await refresh();loaded=true;}else await refresh();draw();} };
})();
