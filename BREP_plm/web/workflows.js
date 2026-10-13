// Visual workflow authoring and smart forms. Authorization and graph execution
// belong to the server; node positions only control the presentation here.
'use strict';
const WorkflowUI = (() => {
  let scriptLocked=false;
  let definitions=[],types=[],people=[],statuses=[],definition=null,selected=null,connectFrom=null,run=null;
  let connectionDrag=null,stopConnectionDrag=null,templateChoice='general',connectOutcome='always';
  const outputDrafts=new Map();
  const drafts=new Map();
  let selectedPath=null;
  const kinds=['start','form','decision','decision-script','review','check','script','status','fork','join','end'];
  const flow=WorkflowFlowchart;
  const stepInfo={decision:['Human decision','Choose one path'], 'decision-script':['Programmatic decision','Choose a path with JavaScript'],start:['Start','Entry point'],form:['Form','Collect information'],review:['Review','Request approvals'],check:['Check','Validate with a script'],script:['Script','Run an automation'],status:['Status','Change part lifecycle'],fork:['Fork','Branch into parallel paths'],join:['Join','Wait for every incoming path'],end:['End','Finish this branch']};
  const emptyProperties=()=>`<h2>Step details</h2><div class="wf-empty"><strong>${definition.nodes.length?'Select a step or path to get started':'No steps yet'}</strong><p>${definition.nodes.length?'Click a step to edit its behavior, or select a line to edit or delete that path.':'Add a Start and an End, then add the steps your process needs and connect them.'}</p><p>Add more steps using the palette on the left.</p><a href="/workflow-help.html#steps" target="_blank" rel="noopener">Learn about step types ↗</a></div>`;
  const split=text=>text.split(',').map(s=>s.trim()).filter(Boolean);
  const root=()=>$('view-workflow-editor');
  function mark(){drafts.set(definition.id || 'new',structuredClone(definition));$('wf-saved').textContent='Unsaved changes';}
  async function load(){[definitions,types,statuses]=await Promise.all([api('GET','/api/workflows'),api('GET','/api/part-types'),api('GET','/api/settings')]);scriptLocked=statuses.script_editor_locked || !statuses.script_editor_enabled;statuses=statuses.status_options;}
  const options=(values,selected)=>values.map(v=>`<option value="${escape(v)}" ${v===selected?'selected':''}>${escape(v)}</option>`).join('');
  function fieldInput(field,value,editable,required=false,scope='') {
    const id=`wf-field-${scope}-${field.key}`,common=`data-field="${escape(field.key)}" id="${escape(id)}" ${editable?'':'disabled'} ${required && field.kind!=='boolean'?'required':''}`;
    let control;
    if(field.kind==='boolean')control=`<input type="checkbox" ${common} ${value===true?'checked':''}>`;
    else if(field.kind==='textarea')control=`<textarea ${common}>${escape(value || '')}</textarea>`;
    else if(field.kind==='choice')control=`<select ${common}><option value="">Choose…</option>${options(field.choices,value)}</select>`;
    else control=`<input type="${field.kind==='number'?'number':'text'}" ${field.kind==='number'?'step="any"':''} ${common} value="${escape(value ?? '')}">`;
    return `<label for="${escape(id)}">${escape(field.label)}${required?' *':''}${control}</label>`;
  }
  function readFields(container) {
    const values={};container.querySelectorAll('[data-field]:not(:disabled)').forEach(el=>{
      if(el.type==='checkbox')values[el.dataset.field]=el.checked;
      else values[el.dataset.field]=el.value===''?null:el.type==='number'?Number(el.value):el.value;
    });return values;
  }
  async function list(partId, revisionId) {
    await load();const runs=partId?[]:await api('GET','/api/workflow-runs');
    const target=partId?await api('GET',`/api/parts/${encodeURIComponent(partId)}`):null;
    const revision=target?(revisionId?target.revision_views.find(r=>r.id===revisionId):target.revision_views.at(-1)):null;
    if(target&&!revision)throw new Error('This revision no longer exists. Open the part and choose another revision.');
    const available=definitions.filter(d=>d.active && (!target || ((me.is_admin || d.can_start) && (!d.part_types.length || d.part_types.includes(target.part_type)))));
    const back=target?`<a href="#/part/${encodeURIComponent(target.id)}/${encodeURIComponent(revision.id)}">← Back to part</a>`:'';
    $('view-workflows').innerHTML=`${back}<div class="toolbar"><h1>${target?'Start a workflow':'Workflows'}</h1>${me.is_admin?'<a class="button" href="#/workflow-editor/new">Create workflow</a>':''}</div><p>${target?`Choose a workflow for <strong>${escape(target.number)} · ${escape(target.name)}</strong>, revision <strong>${escape(revision.label)}</strong>.`:'Start a process using an available workflow. Your tasks appear in the Inbox.'}</p><div class="workflow-list">${available.map(d=>`<article><h2>${escape(d.name)}</h2><p>${escape(d.description)}</p><small>${d.part_types.length?d.part_types.map(id=>escape(types.find(t=>t.id===id)?.name || id)).join(', '):'All part types'}</small><div>${me.is_admin || d.can_start?`<button data-start="${escape(d.id)}">Start</button>`:''}${me.is_admin?`<a href="#/workflow-editor/${encodeURIComponent(d.id)}">Edit</a>`:''}</div></article>`).join('') || '<p>No workflows are available to start'+(target?' for this part':'')+'. '+(me.is_admin?'Create a workflow or check its availability, allowed part types and who can start it.':'Ask an administrator to make a workflow available to you.')+'</p>'}</div>${!target&&me.is_admin?`<p>Inactive definitions: ${definitions.filter(d=>!d.active).map(d=>`<a href="#/workflow-editor/${encodeURIComponent(d.id)}">${escape(d.name)}</a>`).join(', ') || 'None'}</p>`:''}${!target?`<h2>Process monitor</h2><table><thead><tr><th>Process</th><th>Workflow</th><th>Status</th><th>Initiator</th><th>Started</th></tr></thead><tbody>${runs.map(r=>`<tr><td><a href="#/workflow-run/${encodeURIComponent(r.id)}">${escape(r.title)}</a></td><td>${escape(r.workflow)} v${r.version}</td><td>${escape(r.state)}</td><td>${escape(r.initiator)}</td><td>${escape(when(r.created_at))}</td></tr>`).join('')}</tbody></table>`:''}<div id="wf-start-panel"></div>`;
    $('view-workflows').querySelectorAll('[data-start]').forEach(b=>b.onclick=()=>act(()=>startForm(b.dataset.start,target,revision)));
  }
  async function startForm(id,target,revision) {
    const d=definitions.find(d=>d.id===id);
    // Launches from a part resolve that exact record, independently of list pagination.
    const parts=target?[]:(await api('GET','/api/parts?limit=1000')).parts;
    const eligible=parts.filter(p=>!d.part_types.length || d.part_types.includes(p.part_type));
    const context=target?`<p>Part <strong>${escape(target.number)} · ${escape(target.name)}</strong> · Revision <strong>${escape(revision.label)}</strong></p><input id="wf-part" type="hidden" value="${escape(target.id)}"><input id="wf-revision" type="hidden" value="${escape(revision.id)}">`:`<label>Part<select id="wf-part"><option value="">No part</option>${eligible.map(p=>`<option value="${escape(p.id)}">${escape(p.number)} · ${escape(p.name)}</option>`).join('')}</select></label><label>Revision<select id="wf-revision"><option value="">Newest revision</option></select></label>`;
    $('wf-start-panel').innerHTML=`<form id="wf-start-form"><h2>Start ${escape(d.name)}</h2>${context}<label>Title<input id="wf-title" value="${escape(d.name)}" required maxlength="200"></label>${d.fields.map(f=>fieldInput(f,'',true,f.required_at.includes('start'))).join('')}<button type="submit">Start process</button></form>`;
    if(!target)$('wf-part').onchange=()=>act(async()=>{const id=$('wf-part').value;if(!id){$('wf-revision').innerHTML='<option value="">Newest revision</option>';return;}const part=await api('GET',`/api/parts/${encodeURIComponent(id)}`);$('wf-revision').innerHTML=part.revision_views.map(r=>`<option value="${escape(r.id)}">${escape(r.label)} · ${escape(statusName(r.lifecycle))}</option>`).join('');});
    $('wf-start-form').onsubmit=e=>{e.preventDefault();const button=e.submitter;button.disabled=true;act(async()=>{try{const result=await api('POST','/api/workflow-runs',{workflow:id,title:$('wf-title').value,part:$('wf-part').value,revision:$('wf-revision').value,fields:readFields($('wf-start-form'))});location.hash=`#/workflow-run/${encodeURIComponent(result.id)}`;await refreshInbox();}finally{button.disabled=false;}});};
    $('wf-start-panel').scrollIntoView();$('wf-title').focus();
  }
  function starter(template='general') {
    const node=(id,label,kind,x,y,extra={})=>({id,label,kind,x,y,assignees:[],approvals:0,allow_self_review:false,code:'',status:null,...extra});
    const d={id:'',name:template==='release'?'Part release':'New workflow',description:'',active:true,require_for_release:false,part_types:[],start_groups:['group:author'],version:0,updated_by:'',updated_at:0,fields:[{key:'reason',label:'Reason',kind:'textarea',choices:[],required_at:['start'],readers:['*'],writers:['initiator']}],nodes:[node('start','Start','start',40,160),node('request','Complete request','form',280,160,{assignees:['initiator']}),node('end','Complete','end',560,160)],edges:[{from:'start',to:'request',on:'always'},{from:'request',to:'end',on:'always'}]};
    if(template==='release') {
      d.nodes=[node('start','Start','start',30,180),node('review','Parallel model review','review',270,70,{assignees:['group:checkin']}),node('check','Automated check','check',270,300,{code:'function step(input) {\n  const part = plm.part(input.partId);\n  if (!part.description) throw new Error("Add a part description before release.");\n  return { outcome: "success" };\n}'}),node('join','Wait for all checks','join',540,180),node('release','Release model','status',780,180,{status:'released'}),node('end','Released','end',1020,180),node('reject','Rejected','end',540,420)];
      d.edges=[{from:'start',to:'review',on:'always'},{from:'start',to:'check',on:'always'},{from:'review',to:'join',on:'success'},{from:'check',to:'join',on:'success'},{from:'join',to:'release',on:'success'},{from:'release',to:'end',on:'success'},{from:'review',to:'reject',on:'reject'},{from:'check',to:'reject',on:'reject'}];
    }
    flow.layout(d.nodes,d.edges);return d;
  }
  async function editor(id) {
    await load();people=await api('GET','/api/users');
    if(id==='new' && !drafts.has('new')){templateChoice='general';for(const key of outputDrafts.keys())if(key.startsWith('new/'))outputDrafts.delete(key);}
    definition=drafts.get(id) || (id==='new'?starter():structuredClone(definitions.find(d=>d.id===id)));
    if(!definition)throw new Error('Workflow does not exist');if(stopConnectionDrag)stopConnectionDrag();selected=null;selectedPath=null;connectFrom=null;connectionDrag=null;
    root().innerHTML=`<div class="toolbar wf-toolbar"><div><a href="#/workflows">← Workflows</a><h1>Workflow editor</h1></div><span class="spacer"></span><span id="wf-saved" role="status">${definition.version?`Version ${definition.version}`:'New draft'}</span><a href="/workflow-help.html" target="_blank" rel="noopener">Workflow guide ↗</a><button id="wf-save">Save workflow</button></div><p class="muted">Design the steps, decisions and information your process needs.${id==='new'?' Start with a template, customize it, then save to make it available.':''}</p><p id="wf-save-error" class="wf-save-error" role="alert" hidden></p><section class="wf-section"><h2>Workflow settings</h2><div class="workflow-meta"><label>Name<input id="wf-name" value="${escape(definition.name)}" maxlength="200"></label><label>Description<input id="wf-description" value="${escape(definition.description)}"></label><label class="wf-checkbox"><input id="wf-active" type="checkbox" ${definition.active?'checked':''}>Available to start</label><label class="wf-checkbox" id="wf-required-release-label"><input id="wf-required-release" type="checkbox" ${definition.require_for_release?'checked':''}>Require this workflow for part release</label><label>Allowed part types<select id="wf-types" multiple>${types.map(t=>`<option value="${escape(t.id)}" ${definition.part_types.includes(t.id)?'selected':''}>${escape(t.name)}</option>`).join('')}</select><small>Leave empty for all types. Hold Ctrl or ⌘ to select several.</small></label><label>Who can start<input id="wf-starters" value="${escape(definition.start_groups.join(', '))}"><small>user:username, group:name, or *; empty allows authors.</small></label>${id==='new'?`<label>Template<select id="wf-template">${[['general','Smart form'],['release','Part release with parallel review and check']].map(([value,label])=>`<option value="${value}" ${value===templateChoice?'selected':''}>${label}</option>`).join('')}</select><small>Changing the template replaces this draft.</small></label>`:''}</div>
      </section><section class="wf-section wf-designer"><div class="wf-section-heading"><div><h2>Process designer</h2><p class="muted">1. Add steps &nbsp; → &nbsp; 2. Connect paths &nbsp; → &nbsp; 3. Configure each step</p></div><button id="wf-arrange" class="ghost" type="button">Arrange top down</button><button id="wf-reset-view" class="ghost" type="button">Back to start</button></div><div class="wf-canvas-tip"><span id="wf-connect-status" role="status">Drag OUT or a named decision output to another step’s top IN connector. Or click an output, then IN.</span><button id="wf-cancel-connect" class="ghost" type="button" hidden>Cancel connection</button></div><div class="workflow-editor-layout"><aside id="wf-palette"><h3>Add a step</h3>${kinds.map(kind=>`<button type="button" draggable="true" data-kind="${kind}" title="Click to add or drag onto the canvas"><strong>${stepInfo[kind][0]}</strong><small>${stepInfo[kind][1]}</small></button>`).join('')}</aside><div class="workflow-canvas-scroll"><div id="wf-canvas" tabindex="0" aria-label="Workflow process canvas"><svg id="wf-paths" aria-label="Workflow paths"></svg><div id="wf-nodes"></div></div></div><aside id="wf-properties">${emptyProperties()}</aside><datalist id="wf-outcomes"><option value="always"><option value="success"><option value="reject"></datalist></div><p class="wf-keyboard-hint muted">Drag steps to arrange them. Focus a step and use arrow keys to move it, or Enter to edit. Select a line to edit its outcome; Delete removes a selected path. Press Escape to cancel a connection.</p></section>
      <section class="wf-section"><div class="wf-section-heading"><h2>Form fields</h2><button id="wf-add-field" class="ghost">+ Add field</button></div><p class="muted">Collect information at the start or during a form step. Required at uses “start” or step IDs. <a href="/workflow-help.html#fields" target="_blank" rel="noopener">Field permissions and requirements ↗</a></p><div id="wf-fields"></div></section>`;
    ['name','description','active','types','starters','required-release'].forEach(key=>$(`wf-${key}`).onchange=()=>{definition.name=$('wf-name').value;definition.description=$('wf-description').value;definition.active=$('wf-active').checked;definition.require_for_release=$('wf-required-release').checked;definition.part_types=[...$('wf-types').selectedOptions].map(o=>o.value);definition.start_groups=split($('wf-starters').value);mark();});
    if($('wf-template'))$('wf-template').onchange=()=>{templateChoice=$('wf-template').value;for(const key of outputDrafts.keys())if(key.startsWith('new/'))outputDrafts.delete(key);definition=starter(templateChoice);drafts.set('new',definition);act(()=>editor('new'));};
    $('wf-save').onclick=()=>act(async()=>{const button=$('wf-save');button.disabled=true;button.textContent='Saving…';$('wf-save-error').hidden=true;try{for(const node of definition.nodes.filter(n=>flow.decision(n.kind)))if(outputNames(node).some(name=>!definition.edges.some(e=>e.from===node.id&&e.on===name)))throw new Error(`Connect every output of ${node.label} before saving`);definition=await api('POST','/api/workflows',definition);drafts.delete(id);drafts.delete('new');$('wf-saved').textContent=`Saved version ${definition.version}`;OpenPages.saved();if(id==='new')location.hash=`#/workflow-editor/${encodeURIComponent(definition.id)}`;}catch(error){$('wf-save-error').textContent=`Could not save: ${error.message}. Your draft is still here; fix the settings and try again.`;$('wf-save-error').hidden=false;throw error;}finally{button.disabled=false;button.textContent='Save workflow';}});
    $('wf-palette').querySelectorAll('[data-kind]').forEach(button=>{button.ondragstart=e=>e.dataTransfer.setData('application/x-plm-workflow-node',button.dataset.kind);button.onclick=()=>{const viewport=$('wf-canvas').parentElement;let x=viewport.scrollLeft+60,y=viewport.scrollTop+60;while(definition.nodes.some(n=>Math.abs(n.x-x)<224 && Math.abs(n.y-y)<140))y+=160;addNode(button.dataset.kind,x,y);$('wf-nodes').querySelector(`[data-node="${selected}"]`).scrollIntoView({block:'nearest',inline:'nearest'});};});
    $('wf-arrange').onclick=()=>{flow.layout(definition.nodes,definition.edges);mark();drawGraph();$('wf-canvas').scrollIntoView({block:'start',inline:'nearest'});};
    $('wf-reset-view').onclick=()=>$('wf-canvas').scrollIntoView({block:'start',inline:'nearest',behavior:'smooth'});
    $('wf-cancel-connect').onclick=()=>cancelConnection();
    root().onkeydown=e=>{if(e.key==='Escape')cancelConnection();if(selectedPath&&['Delete','Backspace'].includes(e.key)&&!e.target.closest('input,select,textarea,[contenteditable]')){e.preventDefault();deleteSelectedPath();}};
    const canvas=$('wf-canvas');canvas.ondragover=e=>e.preventDefault();canvas.ondrop=e=>{e.preventDefault();const kind=e.dataTransfer.getData('application/x-plm-workflow-node');if(!kinds.includes(kind))return;const rect=canvas.getBoundingClientRect();addNode(kind,e.clientX-rect.left,e.clientY-rect.top);};
    $('wf-add-field').onclick=()=>{definition.fields.push({key:`field${definition.fields.length+1}`,label:'New field',kind:'text',choices:[],required_at:[],readers:['*'],writers:['initiator']});mark();drawFields();};
    drawGraph();drawFields();
  }
  function addNode(kind,x,y) {
    let index=definition.nodes.length+1;while(definition.nodes.some(n=>n.id===`step${index}`))index++;
    const node={id:`step${index}`,label:stepInfo[kind][0],kind,x:Math.max(10,x),y:Math.max(10,y),assignees:['initiator'],approvals:0,allow_self_review:false,code:kind==='decision-script'?'function step(input) {\n  return { outcome: "yes" };\n}':'function step(input) {\n  return { outcome: "success", fields: {} };\n}',status:kind==='status'?'released':null};
    definition.nodes.push(node);selectedPath=null;selected=node.id;mark();drawGraph();properties();
  }
  const outputKey=node=>`${definition.id||'new'}/${node.id}`;
  function outputNames(node) {
    if(!flow.decision(node.kind))return ['always'];
    const key=outputKey(node);
    if(!outputDrafts.has(key)) {
      const names=[...new Set(definition.edges.filter(e=>e.from===node.id&&e.on!=='always').map(e=>e.on))];
      if(!names.length)names.push('yes','no');else if(names.length===1)names.push(names[0]==='yes'?'no':'yes');
      outputDrafts.set(key,names);
    }
    return outputDrafts.get(key);
  }
  function outputPorts(node) {
    return outputNames(node).map(outcome=>{const branch=flow.decision(node.kind);return `<button class="wf-port output ${branch?'wf-choice-port':''}" data-output="${escape(node.id)}" data-outcome="${escape(outcome)}" ${node.kind==='end'?'disabled':''}  title="${node.kind==='end'?'End finishes a branch':`Drag ${branch?outcome:'OUT'} to another step’s input`}" aria-label="Connect ${branch?escape(outcome)+' from ': 'from '}${escape(node.label)}"><span aria-hidden="true">${branch?escape(outcome):'OUT'}</span></button>`;}).join('');
  }
  function resizeCanvas() {
    const width=Math.max(620,...definition.nodes.map(n=>n.x+flow.width+100)),height=Math.max(320,...definition.nodes.map(n=>n.y+flow.height+80));
    $('wf-canvas').style.width=`${width}px`;$('wf-canvas').style.height=`${height}px`;
    root().querySelector('.workflow-editor-layout').style.setProperty('--wf-canvas-width',`${width}px`);
  }
  function drawGraph() {
    resizeCanvas();
    $('wf-nodes').innerHTML=definition.nodes.map(n=>`<div class="workflow-node ${flow.decision(n.kind)?'wf-decision':''} ${n.id===selected?'selected':''}" data-node="${escape(n.id)}" tabindex="0" role="group" aria-label="${escape(n.label)}"><svg class="wf-node-shape" viewBox="0 0 200 112" aria-hidden="true">${flow.shape(n.kind)}</svg><button class="wf-port input" data-input="${escape(n.id)}" ${n.kind==='start'?'disabled':''} title="${n.kind==='start'?'Start is the entry point and has no incoming paths':'Input: drop a connection here, or click after choosing an output'}" aria-label="Connect into ${escape(n.label)}"><span aria-hidden="true">IN</span></button><div class="wf-node-handle"><small>${escape(n.kind==='decision-script'?'Decision · script':n.kind==='decision'?'Decision · person':stepInfo[n.kind]?.[0] || n.kind)}</small><strong title="${escape(n.label)}">${escape(n.label)}</strong></div>${outputPorts(n)}</div>`).join('');
    $('wf-nodes').querySelectorAll('[data-node]').forEach(el=>{
      const position=definition.nodes.find(n=>n.id===el.dataset.node);el.style.left=`${position.x}px`;el.style.top=`${position.y}px`;
      el.querySelectorAll('.wf-choice-port').forEach(port=>{const p=flow.output(position,outputNames(position),port.dataset.outcome);port.style.left=`${p.x-position.x}px`;port.style.top=`${p.y-position.y}px`;port.style.bottom='auto';});
      el.onclick=e=>{if(e.target.closest('.wf-port'))return;selectedPath=null;selected=el.dataset.node;root().querySelectorAll('[data-node]').forEach(n=>n.classList.toggle('selected',n===el));paths();properties();};
      el.onkeydown=e=>{const node=definition.nodes.find(n=>n.id===el.dataset.node);if(['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key)){e.preventDefault();node.x=Math.max(10,node.x+({ArrowLeft:-10,ArrowRight:10}[e.key]||0));node.y=Math.max(10,node.y+({ArrowUp:-10,ArrowDown:10}[e.key]||0));el.style.left=`${node.x}px`;el.style.top=`${node.y}px`;mark();resizeCanvas();paths();}if(e.key==='Enter'){selectedPath=null;selected=node.id;root().querySelectorAll('[data-node]').forEach(n=>n.classList.toggle('selected',n===el));paths();properties();}};
      el.querySelector('.wf-node-handle').onpointerdown=e=>{if(e.button!==0)return;const node=definition.nodes.find(n=>n.id===el.dataset.node),start={x:e.clientX,y:e.clientY,nx:node.x,ny:node.y};e.target.setPointerCapture(e.pointerId);const move=ev=>{node.x=Math.max(10,start.nx+ev.clientX-start.x);node.y=Math.max(10,start.ny+ev.clientY-start.y);el.style.left=`${node.x}px`;el.style.top=`${node.y}px`;resizeCanvas();paths();};const end=()=>{e.target.removeEventListener('pointermove',move);e.target.removeEventListener('pointerup',end);e.target.removeEventListener('pointercancel',end);if(node.x!==start.nx || node.y!==start.ny)mark();};e.target.addEventListener('pointermove',move);e.target.addEventListener('pointerup',end);e.target.addEventListener('pointercancel',end);};
    });
    $('wf-nodes').querySelectorAll('[data-output]').forEach(button=>{
      let suppressClick=false;
      button.onclick=e=>{if(suppressClick && e.detail!==0){suppressClick=false;return;}connectFrom=button.dataset.output;connectOutcome=button.dataset.outcome;connectionFeedback();};
      button.onpointerdown=e=>{
        if(e.button!==0 || button.disabled)return;
        e.preventDefault();e.stopPropagation();button.focus({preventScroll:true});suppressClick=false;cancelConnection();connectFrom=button.dataset.output;connectOutcome=button.dataset.outcome;connectionFeedback();
        const start={x:e.clientX,y:e.clientY};let moved=false;
        button.setPointerCapture(e.pointerId);
        const targetAt=ev=>{
          const element=document.elementFromPoint(ev.clientX,ev.clientY);
          const input=element?.closest('[data-input]') || element?.closest('[data-node]')?.querySelector('[data-input]');
          return input && !input.disabled && input.dataset.input!==connectFrom?input:null;
        };
        const move=ev=>{
          if(ev.pointerId!==e.pointerId)return;
          if(Math.hypot(ev.clientX-start.x,ev.clientY-start.y)>4)moved=true;
          if(!moved)return;
          const rect=$('wf-canvas').getBoundingClientRect(),target=targetAt(ev);
          connectionDrag={x:ev.clientX-rect.left,y:ev.clientY-rect.top};
          root().querySelectorAll('[data-node]').forEach(el=>el.classList.toggle('connection-hover',el.dataset.node===target?.dataset.input));
          $('wf-connect-status').textContent=target?`Release to connect to “${definition.nodes.find(n=>n.id===target.dataset.input).label}”.`:'Drag to another step’s IN connector or box, then release. Escape cancels.';
          paths();
        };
        const cleanup=()=>{
          button.removeEventListener('pointermove',move);button.removeEventListener('pointerup',finish);button.removeEventListener('pointercancel',abort);
          if(button.hasPointerCapture(e.pointerId))button.releasePointerCapture(e.pointerId);
          root().querySelectorAll('.connection-hover').forEach(el=>el.classList.remove('connection-hover'));
          connectionDrag=null;stopConnectionDrag=null;
        };
        const finish=ev=>{
          if(ev.pointerId!==e.pointerId)return;
          const from=connectFrom,outcome=connectOutcome,target=targetAt(ev);suppressClick=moved;cleanup();
          if(moved){connectFrom=null;if(target)addPath(from,target.dataset.input,outcome);else{connectionFeedback();$('wf-connect-status').textContent='No path added. Drag OUT to another step’s IN connector or box.';}}
          else connectionFeedback();
          paths();
        };
        const abort=()=>{suppressClick=true;cleanup();connectFrom=null;connectionFeedback();paths();};
        stopConnectionDrag=abort;
        button.addEventListener('pointermove',move);button.addEventListener('pointerup',finish);button.addEventListener('pointercancel',abort);
      };
    });
    $('wf-nodes').querySelectorAll('[data-input]').forEach(b=>b.onclick=()=>{if(!connectFrom || connectFrom===b.dataset.input)return;addPath(connectFrom,b.dataset.input,connectOutcome);});paths();connectionFeedback();
  }
  function addPath(from,to,outcome) {
    if(!from || !to || from===to)return;
    const node=definition.nodes.find(n=>n.id===from),branch=flow.decision(node.kind);
    outcome??=branch?(outputNames(node).find(name=>!definition.edges.some(e=>e.from===from&&e.on===name))||outputNames(node)[0]):'always';
    const existing=definition.edges.find(e=>e.from===from&&e.on===outcome&&(branch||e.to===to));
    const duplicate=existing?.to===to;
    if(!duplicate){if(existing&&branch)existing.to=to;else definition.edges.push({from,to,on:outcome});mark();}
    selected=null;selectedPath=existing||definition.edges.at(-1);connectFrom=null;connectionDrag=null;drawGraph();pathProperties();
    $('wf-connect-status').textContent=duplicate?`These steps already have an “${outcome}” path.`:`Path added (${outcome}): ${node.label} → ${definition.nodes.find(n=>n.id===to).label}.`;
  }
  function cancelConnection() {
    if(stopConnectionDrag)stopConnectionDrag();connectFrom=null;connectionDrag=null;connectionFeedback();paths();
  }
  function connectionFeedback() {
    $('wf-connect-status').textContent=connectFrom?`Connecting from “${definition.nodes.find(n=>n.id===connectFrom)?.label}”, output ${connectOutcome}. Click another step’s IN connector, or drag OUT to it.`:'Drag OUT or a named decision output to another step’s top IN connector. Or click an output, then IN.';
    $('wf-cancel-connect').hidden=!connectFrom;
    root().querySelectorAll('[data-output]').forEach(b=>b.classList.toggle('connecting',b.dataset.output===connectFrom&&b.dataset.outcome===connectOutcome));
    root().querySelectorAll('[data-input]').forEach(b=>b.classList.toggle('connect-target',!!connectFrom && !b.disabled && b.dataset.input!==connectFrom));
  }
  function paths() {
    $('wf-paths').innerHTML='<defs><marker id="wf-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" class="wf-arrowhead"/></marker></defs>'+definition.edges.map((e,i)=>{const from=definition.nodes.find(n=>n.id===e.from),to=definition.nodes.find(n=>n.id===e.to);if(!from || !to)return '';const {d,label}=flow.route(from,to,definition.nodes,outputNames(from),e.on),caption=e.on;return `<g class="wf-edit-path ${selectedPath===e?'selected':''}" data-path-index="${i}" tabindex="0" role="button" aria-label="Edit path ${escape(from.label)} to ${escape(to.label)}, outcome ${escape(e.on)}"><title>Select to edit or delete this path</title><path class="wf-path-hit" d="${d}"/><path marker-end="url(#wf-arrow)" d="${d}"/><text text-anchor="middle" x="${label.x+8}" y="${label.y-8}">${escape(caption)}</text></g>`;}).join('');
    $('wf-paths').querySelectorAll('[data-path-index]').forEach(el=>{const select=focus=>{selected=null;selectedPath=definition.edges[Number(el.dataset.pathIndex)];root().querySelectorAll('[data-node]').forEach(n=>n.classList.remove('selected'));paths();pathProperties();$('wf-paths').querySelector(`[data-path-index="${definition.edges.indexOf(selectedPath)}"]`)?.focus({preventScroll:true});if(focus)$('wf-path-outcome').focus({preventScroll:true});};el.onclick=()=>select(false);el.onkeydown=e=>{if(e.key==='Enter' || e.key===' '){e.preventDefault();select(true);}if(['Delete','Backspace'].includes(e.key)){e.preventDefault();e.stopPropagation();selectedPath=definition.edges[Number(el.dataset.pathIndex)];deleteSelectedPath();}};});
    if(connectionDrag && connectFrom){const from=definition.nodes.find(n=>n.id===connectFrom),p=flow.output(from,outputNames(from),connectOutcome),x=p.x,y=p.y,middle=p.dy?(y+connectionDrag.y)/2:(x+connectionDrag.x)/2,d=p.dy?`M ${x} ${y} V ${middle} H ${connectionDrag.x} V ${connectionDrag.y}`:`M ${x} ${y} H ${middle} V ${connectionDrag.y} H ${connectionDrag.x}`;$('wf-paths').insertAdjacentHTML('beforeend',`<path class="wf-connection-preview" marker-end="url(#wf-arrow)" d="${d}"/>`);}

  }
  function properties() {
    const node=definition.nodes.find(n=>n.id===selected);if(!node)return;
    $('wf-properties').innerHTML=`<div class="wf-section-heading"><h2>Step details</h2><a href="/workflow-help.html#steps" target="_blank" rel="noopener">Step help ↗</a></div><p class="muted">${escape(stepInfo[node.kind][1])} · ID: <code>${escape(node.id)}</code></p><label>Name<input id="wf-node-name" value="${escape(node.label)}"></label><label>Kind<select id="wf-node-kind">${kinds.map(kind=>`<option value="${kind}" ${kind===node.kind?'selected':''}>${escape(stepInfo[kind][0])}</option>`).join('')}</select></label>${['form','review','decision'].includes(node.kind)?`<label>Assignees<input id="wf-node-assignees" value="${escape(node.assignees.join(', '))}"><small>user:username, group:name or initiator. Available users: ${people.map(u=>escape(u.username)).join(', ')}</small></label>`:''}${node.kind==='review'?`<label>Required approvals<input id="wf-node-approvals" type="number" min="0" value="${node.approvals}"><small>0 requires all assigned reviewers, in parallel.</small></label><label class="wf-checkbox"><input id="wf-node-self" type="checkbox" ${node.allow_self_review?'checked':''}>Allow initiator to review</label>`:''}${node.kind==='status'?`<label>Lifecycle status<select id="wf-node-status">${statuses.filter(s=>s.enabled && s.state!=='superseded').map(s=>`<option value="${escape(s.state)}" ${s.state===node.status?'selected':''}>${escape(s.name)}</option>`).join('')}</select></label>`:''}${['script','check','decision-script'].includes(node.kind)?`<label>JavaScript<textarea id="wf-node-code" rows="14" spellcheck="false" ${scriptLocked?'readonly':''}>${escape(node.code)}</textarea></label><small>Define step(input). Return { outcome, fields }${node.kind==='decision-script'?' with an explicit outgoing path name':', or false for reject'}; throw to stop with an error. input includes fields, partId, revisionId and idempotencyKey. Trusted scripts can use http.request, plm, fs and process.</small><label>Example<select id="wf-script-example" ${scriptLocked?'disabled':''}><option value="">Choose an example</option>${node.kind==='decision-script'?'<option value="decision">Choose based on a field</option>':'<option value="check">Model check</option><option value="erp">ERP order</option><option value="pdf">Publish vendor PDF</option>'}</select></label>`:''}${flow.decision(node.kind)?`<h3>Decision outputs</h3><div>${outputNames(node).map((name,index)=>`<label>Output name<input data-decision-output="${index}" value="${escape(name)}" maxlength="80"></label><button type="button" class="ghost" data-remove-output="${index}" ${outputNames(node).length<=2?'disabled':''}>Remove output</button>`).join('')}</div><label>New output name<input id="wf-new-output" maxlength="80" placeholder="For example: needs changes"></label><button id="wf-add-output" type="button" class="ghost">Add output</button><p>Each outgoing path is a choice. Name at least two paths (for example yes and no), with one destination per choice. ${node.kind==='decision'?'The first assigned person to choose completes the decision.':'Return { outcome: "yes" } from step(input) to select the matching path.'}</p>`:''}<button id="wf-delete-node" class="danger">Delete step</button>`;
    if($('wf-add-output'))$('wf-add-output').onclick=()=>{const name=$('wf-new-output').value.trim();if(!name||name==='always'||outputNames(node).includes(name)){banner('Choose a unique output name other than always.');return;}outputNames(node).push(name);mark();drawGraph();properties();};
    const update=()=>{node.label=$('wf-node-name').value;node.kind=$('wf-node-kind').value;if($('wf-node-assignees'))node.assignees=split($('wf-node-assignees').value);if($('wf-node-approvals'))node.approvals=Number($('wf-node-approvals').value);if($('wf-node-self'))node.allow_self_review=$('wf-node-self').checked;if($('wf-node-status'))node.status=$('wf-node-status').value;if($('wf-node-code'))node.code=$('wf-node-code').value;if(node.kind==='status'&&!node.status)node.status='released';mark();drawGraph();};
    $('wf-properties').querySelectorAll('input,select,textarea').forEach(el=>el.onchange=()=>{update();if(el.id==='wf-node-kind')properties();});
    $('wf-properties').querySelectorAll('[data-decision-output]').forEach(el=>el.onchange=()=>{const names=outputNames(node),index=Number(el.dataset.decisionOutput),old=names[index],name=el.value.trim();if(!name||name==='always'||names.some((n,i)=>n===name&&i!==index)){el.value=old;banner('Choose a unique output name other than always.');return;}names[index]=name;for(const edge of definition.edges.filter(e=>e.from===node.id&&e.on===old))edge.on=name;mark();drawGraph();});
    $('wf-properties').querySelectorAll('[data-remove-output]').forEach(el=>el.onclick=()=>{const names=outputNames(node),name=names[Number(el.dataset.removeOutput)];names.splice(Number(el.dataset.removeOutput),1);definition.edges=definition.edges.filter(e=>e.from!==node.id||e.on!==name);mark();drawGraph();properties();});
    if($('wf-script-example'))$('wf-script-example').onchange=()=>{
      const examples={decision:'function step(input) {\n  return { outcome: input.fields.reason ? "yes" : "no" };\n}',check:'function step(input) {\n  const document = plm.readDocument(input.partId, input.revisionId);\n  if (!document) throw new Error("Model document is missing");\n  return { outcome: "success" };\n}',erp:'function step(input) {\n  // Configure your ERP endpoint and authentication on this server.\n  const url = "";\n  if (!url) throw new Error("Configure the ERP endpoint");\n  const response = http.request({ url, method: "POST", headers: { "Idempotency-Key": input.idempotencyKey }, body: JSON.stringify({ part: input.partId, quantity: input.fields.quantity }) });\n  if (response.status >= 400) throw new Error("ERP order failed");\n  return { outcome: "success" };\n}',pdf:'function step(input) {\n  // Generate/export the PDF in an earlier step, then publish its bytes.\n  const url = "";\n  if (!url) throw new Error("Configure the vendor catalog endpoint");\n  const pdf = fs.readBase64(input.fields.pdfPath);\n  const response = http.request({ url, method: "PUT", headers: { "Content-Type": "application/pdf", "Idempotency-Key": input.idempotencyKey }, body_base64: pdf });\n  if (response.status >= 400) throw new Error("Catalog publication failed");\n  return { outcome: "success" };\n}'};
      node.code=examples[$('wf-script-example').value] || node.code;$('wf-node-code').value=node.code;mark();
    };
    $('wf-delete-node').onclick=()=>{outputDrafts.delete(outputKey(node));definition.nodes=definition.nodes.filter(n=>n.id!==node.id);definition.edges=definition.edges.filter(e=>e.from!==node.id && e.to!==node.id);selected=null;connectFrom=null;mark();drawGraph();$('wf-properties').innerHTML=emptyProperties();};
  }
  function deleteSelectedPath() {
    if(!selectedPath)return;
    definition.edges=definition.edges.filter(edge=>edge!==selectedPath);selectedPath=null;mark();drawGraph();$('wf-properties').innerHTML=emptyProperties();$('wf-canvas').focus({preventScroll:true});
  }
  function pathProperties() {
    if(!selectedPath||!definition.edges.includes(selectedPath)){selectedPath=null;return;}
    const edge=selectedPath,node=definition.nodes.find(n=>n.id===edge.from),to=definition.nodes.find(n=>n.id===edge.to);
    $('wf-properties').innerHTML=`<div class="wf-section-heading"><h2>Path details</h2><a href="/workflow-help.html#paths" target="_blank" rel="noopener">Path help ↗</a></div><p><strong>${escape(node.label)}</strong> → <strong>${escape(to.label)}</strong></p><label>${flow.decision(node.kind)?'Decision output':'Outcome'}<input id="wf-path-outcome" value="${escape(edge.on)}" list="wf-outcomes" maxlength="80"></label><p class="muted">${flow.decision(node.kind)?'This path belongs to a named decision output. Drag that output to another step to change its destination.':'Use always for any completion, success for approval, reject for rejection, or a script’s outcome name.'}</p><button id="wf-delete-path" type="button" class="danger">Delete path</button><p class="muted">You can also select a line and press Delete or Backspace.</p>`;
    $('wf-path-outcome').onchange=()=>{const name=$('wf-path-outcome').value.trim();if(!name||flow.decision(node.kind)&&(name==='always'||outputNames(node).some(n=>n===name&&n!==edge.on))){$('wf-path-outcome').setCustomValidity('Choose a unique, nonempty outcome name.');$('wf-path-outcome').reportValidity();return;}const old=edge.on;if(flow.decision(node.kind)){const names=outputNames(node),index=names.indexOf(old);if(index>=0)names[index]=name;}edge.on=name;mark();drawGraph();};
    $('wf-path-outcome').oninput=()=>$('wf-path-outcome').setCustomValidity('');
    $('wf-delete-path').onclick=deleteSelectedPath;
  }
  function drawFields() {
    $('wf-fields').innerHTML=definition.fields.map((f,i)=>`<fieldset data-form-field="${i}"><legend>${escape(f.label)}</legend>${[['key','Key'],['label','Label'],['choices','Choice options'],['required_at','Required at'],['readers','Can read'],['writers','Can edit']].map(([key,label])=>`<label>${label}<input data-property="${key}" value="${escape(Array.isArray(f[key])?f[key].join(', '):f[key])}"></label>`).join('')}<label>Type<select data-property="kind">${options(['text','textarea','number','boolean','choice'],f.kind)}</select></label><button data-remove-field="${i}">Remove field</button></fieldset>`).join('');
    $('wf-fields').querySelectorAll('[data-property]').forEach(el=>el.onchange=()=>{const field=definition.fields[Number(el.closest('[data-form-field]').dataset.formField)],key=el.dataset.property;field[key]=['choices','required_at','readers','writers'].includes(key)?split(el.value):el.value;mark();});
    $('wf-fields').querySelectorAll('[data-remove-field]').forEach(el=>el.onclick=()=>{definition.fields.splice(Number(el.dataset.removeField),1);mark();drawFields();});
  }
  function runChart() {
    const graph=structuredClone(run.definition);flow.layout(graph.nodes,graph.edges);
    const width=Math.max(620,...graph.nodes.map(n=>n.x+flow.width+60)),height=Math.max(300,...graph.nodes.map(n=>n.y+flow.height+60));
    const lines=graph.edges.map(e=>{const from=graph.nodes.find(n=>n.id===e.from),to=graph.nodes.find(n=>n.id===e.to),route=flow.route(from,to,graph.nodes,[...new Set(graph.edges.filter(edge=>edge.from===from.id).map(edge=>edge.on))],e.on),step=run.steps[e.from],chosen=step.state==='done'&&(e.on==='always'||step.outcome===e.on);return `<g class="${chosen?'wf-taken':'wf-untaken'}"><path d="${route.d}" marker-end="url(#wf-run-arrow)"/><text x="${route.label.x+8}" y="${route.label.y-8}">${escape(e.on)}</text></g>`;}).join('');
    return `<div class="workflow-canvas-scroll wf-run-chart" aria-label="Workflow progress flowchart"><div class="wf-run-canvas" data-chart-width="${width}" data-chart-height="${height}"><svg class="wf-run-paths" width="${width}" height="${height}"><defs><marker id="wf-run-arrow" markerWidth="8" markerHeight="8" refX="7" refY="4" orient="auto"><path d="M0 0 L8 4 L0 8" class="wf-arrowhead"/></marker></defs>${lines}</svg>${graph.nodes.map(n=>{const step=run.steps[n.id];return `<div class="workflow-node ${flow.decision(n.kind)?'wf-decision':''} wf-step-${escape(step.state)}" data-run-node="${escape(n.id)}" data-node-x="${n.x}" data-node-y="${n.y}"><svg class="wf-node-shape" viewBox="0 0 200 112" aria-hidden="true">${flow.shape(n.kind)}</svg><div class="wf-node-handle"><small>${escape(step.state)}${step.outcome?' · '+escape(step.outcome):''}</small><strong title="${escape(n.label)}">${escape(n.label)}</strong></div></div>`;}).join('')}</div></div>`;
  }
  async function detail(id) {
    run=await api('GET',`/api/workflow-runs/${encodeURIComponent(id)}`);
    const waiting=run.definition.nodes.filter(n=>run.steps[n.id].state==='waiting' && run.steps[n.id].assignees.includes(me.id) && !run.steps[n.id].decisions.some(d=>d.user_id===me.id));
    $('view-workflow-run').innerHTML=`<div class="toolbar"><h1>${escape(run.title)}</h1><a href="#/workflows">Process monitor</a><button id="wf-refresh-run">Refresh</button></div><p>${escape(run.definition.name)} v${run.definition.version} · ${escape(run.state)} · started by ${escape(run.initiator_name)}</p>${run.part?`<a href="#/part/${encodeURIComponent(run.part)}/${encodeURIComponent(run.revision)}">Open target model</a>`:''}${runChart()}
      <div id="wf-task-forms">${waiting.map(n=>`<form data-task="${escape(n.id)}"><h2>${escape(n.label)}</h2><p>${n.kind==='review'?'Review the target model and record your decision. Each assigned reviewer responds independently.':'Complete the fields required for this step.'}</p>${run.definition.fields.map(f=>fieldInput(f,run.fields[f.key],f.can_edit,f.required_at.includes(n.id),n.id)).join('')}<label>Comment<textarea data-comment></textarea></label>${n.kind==='decision'?run.definition.edges.filter(e=>e.from===n.id).map(e=>`<button type="submit" data-outcome="${escape(e.on)}">${escape(e.on)}</button>`).join(' '):`<button type="submit" data-outcome="success">${n.kind==='review'?'Approve':'Complete'}</button><button type="submit" data-outcome="reject" formnovalidate>Reject</button>`}</form>`).join('')}</div>${waiting.length?'':`<h2>Form values</h2>${run.definition.fields.map(f=>fieldInput(f,run.fields[f.key],false)).join('')}`}
      ${['active','blocked'].includes(run.state)&&(me.is_admin||run.initiator===me.id)?'<button id="wf-cancel-run" class="danger">Cancel process</button>':''}<h2>Workflow history</h2><table><thead><tr><th>Time</th><th>Step</th><th>Event</th><th>User</th><th>Detail</th></tr></thead><tbody>${run.log.map(e=>`<tr><td>${escape(when(e.at))}</td><td>${escape(run.definition.nodes.find(n=>n.id===e.node)?.label || e.node)}</td><td>${escape(e.action)}</td><td>${escape(e.username)}</td><td>${escape(e.message)}</td></tr>`).join('')}</tbody></table>`;
    const chart=$('view-workflow-run').querySelector('.wf-run-canvas');chart.style.width=`${chart.dataset.chartWidth}px`;chart.style.height=`${chart.dataset.chartHeight}px`;
    chart.querySelectorAll('[data-run-node]').forEach(el=>{el.style.left=`${el.dataset.nodeX}px`;el.style.top=`${el.dataset.nodeY}px`;});
    $('wf-refresh-run').onclick=()=>act(()=>detail(id));
    $('wf-task-forms').querySelectorAll('form').forEach(form=>form.onsubmit=e=>{e.preventDefault();act(async()=>{await api('POST',`/api/workflow-runs/${encodeURIComponent(id)}/actions`,{node:form.dataset.task,outcome:e.submitter.dataset.outcome,comment:form.querySelector('[data-comment]').value,fields:readFields(form)});await detail(id);await refreshInbox();});});
    $('view-workflow-run').querySelectorAll('[data-retry]').forEach(b=>b.onclick=()=>act(async()=>{await api('POST',`/api/workflow-runs/${encodeURIComponent(id)}/control`,{action:'retry',node:b.dataset.retry});await detail(id);await refreshInbox();}));
    if($('wf-cancel-run'))$('wf-cancel-run').onclick=()=>act(async()=>{await api('POST',`/api/workflow-runs/${encodeURIComponent(id)}/control`,{action:'cancel'});await detail(id);await refreshInbox();});
  }
  return {list,editor,detail};
})();
