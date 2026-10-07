/* Open pages belong to the signed-in browser session. Tabs and drafts survive refresh in session storage. */
'use strict';
globalThis.OpenPages = (() => {
  const tabs = new Map();
  let active = null, user = null, restoring = false, dragging = null, persistTimer;
  const storageKey=()=>`plm.open-pages.v1:${user}`;
  function persist() {
    if(!user)return;
    try {sessionStorage.setItem(storageKey(),JSON.stringify({version:1,activeHash:tabs.get(active)?.hash||'#/empty',barScroll:bar().scrollLeft,tabs:[...tabs.values()].map(tab=>({...tab,loaded:false,fields:[...tab.fields],baseline:[],edits:[...tab.edits]}))}));}catch(error){console.warn('Could not preserve PLM tabs in this browser session.',error);}
  }
  function schedule(){clearTimeout(persistTimer);persistTimer=setTimeout(()=>{if(document.querySelector('.work-area').inert){schedule();return;}capture();persist();},200);}
  function restoreSession() {
    tabs.clear();active=null;user=me.id;
    try {
      const state=JSON.parse(sessionStorage.getItem(storageKey())||'null');if(state?.version!==1||!Array.isArray(state.tabs))return;
      for(const entry of state.tabs){if(typeof entry.hash!=='string'||!entry.hash.startsWith('#/')||typeof entry.view!=='string')continue;const key=keyOf(entry.hash);tabs.set(key,{...entry,key,fields:new Map(entry.fields||[]),baseline:new Map(),edits:new Set(entry.edits||[]),scroll:entry.scroll||[],loaded:false});}
      if(!location.hash&&state.activeHash)history.replaceState(null,'',state.activeHash);
      draw();bar().scrollLeft=state.barScroll||0;
    }catch(error){console.warn('Could not restore PLM tabs.',error);}
  }
  function applyFields(tab) {
    controls(tab.view).forEach((n,i)=>{const key=fieldKey(n,i);if(tab.fields.has(key)&&!n.disabled&&!n.readOnly){if(['checkbox','radio'].includes(n.type))n.checked=tab.fields.get(key);else n.value=tab.fields.get(key);}});
  }
  async function beforeRender(hash) {
    const tab=tabs.get(keyOf(hash));if(!tab)return;
    if(tab.view==='parts'){await loadCategories();fillCategoryFilter();await loadCompanies();}
    if(!tab.loaded||tab.key!==active)applyFields(tab);
    if(tab.view==='reviews')Workbench.restoreInbox(tab.inboxSelection||null);
    if(tab.partState){Object.assign(structure,tab.partState.structure);Object.assign(bomAttributeView,tab.partState.bom,{collapsed:new Set(tab.partState.bom.collapsed||[])});}
    if(tab.partsState){selectedParts.clear();for(const [id,number] of tab.partsState.selected)selectedParts.set(id,number);selectedPartId=tab.partsState.inspected;partsSelectionAnchor=null;partsRestorePage=tab.partsState.page||null;}
  }
  async function afterRender(hash) {
    const tab=tabs.get(keyOf(hash));if(!tab||tab.loaded)return;
    if(tab.document)FileDocuments.restore(hash,tab.document);
    if(tab.partsState){await selectPartSummary(tab.partsState.inspected);}
    for(const [documentHash,state] of tab.documents||[])FileDocuments.restore(documentHash,state);
  }
  const bar = () => document.getElementById('open-pages');
  const keyOf = hash => {
    const bits=hash.split('/');
    // A part's Summary/BOM/History views share its revision tab.
    return bits[1]==='part' ? bits.slice(0,4).join('/') : hash;
  };
  const controls = view => view==='file' ? [] : [...document.querySelectorAll(`#view-${view} input, #view-${view} select, #view-${view} textarea`)].filter(n=>!n.closest('.file-document')&&n.id!=='script-text'&&!['password','file','hidden','submit','button'].includes(n.type));
  const fieldKey = (n,i) => n.id ? `id:${n.id}` : `field:${n.name || n.dataset.input || ''}:${i}`;
  const value = n => ['checkbox','radio'].includes(n.type) ? n.checked : n.value;
  function draw() {
    const focused=bar().contains(document.activeElement) ? document.activeElement.dataset.pageKey : null;
    bar().replaceChildren();
    for(const tab of tabs.values()) {
      const item=document.createElement('div');item.dataset.pageKey=tab.key;item.className='open-page';item.classList.toggle('active',tab.key===active);
      const button=document.createElement('button');button.type='button';button.className='open-page-select';button.dataset.pageKey=tab.key;
      button.setAttribute('role','tab');button.setAttribute('aria-selected',String(tab.key===active));button.setAttribute('aria-controls',`view-${tab.view}`);button.tabIndex=tab.key===active?0:-1;
      const icon=document.createElement('span');icon.className=`open-page-icon ${tab.view==='workspace'?'folder':tab.view==='part'?'part':'page'}`;icon.setAttribute('aria-hidden','true');
      if(tab.view==='part')icon.innerHTML='<svg viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.4"><path d="M10 1 18 5.5v9L10 19 2 14.5v-9L10 1Z M2 5.5l8 4.5 8-4.5 M10 10v9 M6 3.25l8 4.5"/></svg>';
      const label=document.createElement('span');label.textContent=tab.title;button.append(icon,label);button.title=`${tab.title} — Drag to reorder; Alt+Shift+Left/Right to move`;button.draggable=true;
      button.addEventListener('click',()=>{location.hash=tab.hash;});
      const close=document.createElement('button');close.type='button';close.className='open-page-close';close.textContent='×';close.setAttribute('aria-label',`Close ${tab.title}`);close.addEventListener('click',()=>closeTab(tab.key));
      item.append(button,close);bar().append(item);
      if(focused===tab.key)button.focus({preventScroll:true});
    }
  }
  function reorder(key,target,after) {
    if(key===target || !tabs.has(key) || !tabs.has(target))return;
    const entries=[...tabs.entries()],moving=entries.find(([k])=>k===key);
    const ordered=entries.filter(([k])=>k!==key);
    ordered.splice(ordered.findIndex(([k])=>k===target)+(after?1:0),0,moving);
    tabs.clear();for(const [k,tab] of ordered)tabs.set(k,tab);
    draw();persist();
    bar().querySelector(`[data-page-key="${CSS.escape(key)}"] [role=tab]`)?.scrollIntoView({block:'nearest',inline:'nearest'});
  }
  function clearDrop() { for(const item of bar().querySelectorAll('.open-page'))item.classList.remove('drop-before','drop-after','dragging'); }
  bar().addEventListener('dragstart',event=>{
    const tab=event.target.closest('[role=tab]');if(!tab)return;
    dragging=tab.dataset.pageKey;event.dataTransfer.effectAllowed='move';event.dataTransfer.setData('application/x-brep-page-tab',dragging);
    tab.closest('.open-page').classList.add('dragging');
  });
  bar().addEventListener('dragover',event=>{
    if(!dragging)return;
    const item=event.target.closest('.open-page');if(!item)return;
    event.preventDefault();event.dataTransfer.dropEffect='move';clearDrop();
    const bounds=item.getBoundingClientRect();item.classList.add(event.clientX>bounds.x+bounds.width/2?'drop-after':'drop-before');
    const edge=bar().getBoundingClientRect();if(event.clientX<edge.left+35)bar().scrollLeft-=24;else if(event.clientX>edge.right-35)bar().scrollLeft+=24;
  });
  bar().addEventListener('drop',event=>{
    if(!dragging)return;
    event.preventDefault();const item=event.target.closest('.open-page');
    if(item){const bounds=item.getBoundingClientRect();reorder(dragging,item.dataset.pageKey,event.clientX>bounds.x+bounds.width/2);}
    dragging=null;clearDrop();
  });
  bar().addEventListener('dragend',()=>{dragging=null;clearDrop();});
  function capture() {
    const tab=tabs.get(active);if(!tab||!tab.loaded)return;
    tab.scroll=[document.querySelector('.work-area'),...document.querySelectorAll(`#view-${tab.view}, #view-${tab.view} *`)].filter(n=>n.scrollTop||n.scrollLeft).map(n=>({id:n.id,selector:n.id?null:[...document.querySelectorAll(`#view-${tab.view} *`)].indexOf(n),top:n.scrollTop,left:n.scrollLeft,outer:n.classList.contains('work-area')}));
    tab.documents=FileDocuments.snapshots(document.getElementById(`view-${tab.view}`));
    tab.layouts=[...document.querySelectorAll(`#view-${tab.view} .master-detail`)].map(n=>({id:n.id,open:n.classList.contains('detail-open')}));
    if(tab.view==='reviews')tab.inboxSelection=Workbench.inboxSelection();
    tab.details=[...document.querySelectorAll(`#view-${tab.view} details`)].map(n=>n.open);
    if(tab.view==='workspace')tab.workspace=Workbench.snapshot();
    if(tab.view==='file')tab.document=FileDocuments.snapshot(tab.hash)||tab.document;
    if(tab.view==='parts')tab.partsState={selected:[...selectedParts],inspected:selectedPartId,page:{cursors:partsCursors.slice(0,partsPageIndex+1),index:partsPageIndex}};
    if(tab.view==='part')tab.partState={structure:{part:structure.part,revision:structure.revision,tab:structure.tab},bom:{show:bomAttributeView.show,treeKey:bomAttributeView.treeKey,collapsed:[...(bomAttributeView.collapsed||[])]}};
    if(tab.view==='part'){tab.objectView=objectTab;tab.editor={key:editorKey,text:document.getElementById('editor-body').value,title:document.getElementById('editor-key').textContent,hidden:document.getElementById('editor').hidden,note:document.getElementById('editor-note').textContent,readonly:document.getElementById('editor-body').readOnly,saveHidden:document.getElementById('editor-save').hidden};}
    if(tab.view==='scripts'&&scriptPath)tab.script={path:scriptPath,saved:scriptSaved,text:document.getElementById('script-text').value};
  }
  function remember(event) {
    schedule();
    if(restoring || document.querySelector('.work-area').inert)return;
    const tab=tabs.get(active);if(!tab||!event.target.closest(`#view-${tab.view}`))return;
    const fields=controls(tab.view),i=fields.indexOf(event.target);if(i<0)return;
    const key=fieldKey(event.target,i),current=value(event.target);
    if(current===tab.baseline.get(key)){tab.fields.delete(key);tab.edits.delete(key);}
    else {
      tab.fields.set(key,current);
      if(event.target.closest('form') || event.target.tagName==='TEXTAREA')tab.edits.add(key);
    }
  }
  function activate(hash,view,title) {
    if(user!==me.id){tabs.clear();active=null;user=me.id;}
    if(view==='empty'){active=null;draw();persist();return;}
    const key=keyOf(hash),prior=tabs.get(key),priorHash=prior?.hash,switching=active!==key;
    const tab=prior || {key,fields:new Map(),baseline:new Map(),edits:new Set(),scroll:[]};
    Object.assign(tab,{hash,view,title});tabs.set(key,tab);active=key;
    tab.baseline=new Map(controls(view).map((n,i)=>[fieldKey(n,i),value(n)]));
    restoring=true;
    if(switching&&prior) {
      applyFields(tab);
      if(view==='part'&&tab.objectView&&hash===priorHash)selectObjectTab(tab.objectView);
      if(view==='part'&&tab.editor){editorKey=tab.editor.key;document.getElementById('editor-body').value=tab.editor.text;document.getElementById('editor-key').textContent=tab.editor.title;document.getElementById('editor').hidden=tab.editor.hidden;document.getElementById('editor-note').textContent=tab.editor.note;document.getElementById('editor-body').readOnly=tab.editor.readonly;document.getElementById('editor-save').hidden=tab.editor.saveHidden;}
      if(view==='scripts')syncScriptButtons();
      if(view==='settings')syncEcoModeFields();
      for(const layout of tab.layouts||[])document.getElementById(layout.id)?.classList.toggle('detail-open',layout.open);
      document.querySelectorAll(`#view-${view} details`).forEach((n,i)=>{if(tab.details?.[i]!==undefined)n.open=tab.details[i];});
    }
    restoring=false;
    if(switching) {
      for(const n of [document.querySelector('.work-area'),document.getElementById(`view-${view}`),...document.querySelectorAll(`#view-${view} *`)]){if(n.scrollTop)n.scrollTop=0;if(n.scrollLeft)n.scrollLeft=0;}
      for(const s of tab.scroll){const n=s.outer?document.querySelector('.work-area'):s.id?document.getElementById(s.id):document.querySelectorAll(`#view-${view} *`)[s.selector];if(n){n.scrollTop=s.top;n.scrollLeft=s.left;}}
    }
    tab.loaded=true;draw();persist();bar().querySelector('[aria-selected=true]')?.scrollIntoView({block:'nearest',inline:'nearest'});
  }
  function dirty(tab) {
    if(tab.view==='file')return tab.loaded?FileDocuments.isDirty(tab.hash):tab.document?.dirty;
    if(tab.view==='scripts')return tab.script && tab.script.text!==tab.script.saved;
    // Keep a conservative close warning for edited form fields.
    return tab.edits.size>0;
  }
  function closeTab(key,force=false) {
    capture();const tab=tabs.get(key);if(!tab)return;
    if(tab.view==='file'&&FileDocuments.isSaving(tab.hash)){banner('This document is still saving. Close it after the save finishes.');return;}
    if(!force&&dirty(tab)&&!confirm(`Close ${tab.title} and discard its unsaved form entries?`))return;
    const keys=[...tabs.keys()],index=keys.indexOf(key);tabs.delete(key);if(tab.view==='file')FileDocuments.close(tab.hash);
    if(active===key){active=null;const next=tabs.get(keys[index+1]) || tabs.get(keys[index-1]);location.hash=next?.hash || '#/empty';}
    draw();persist();
  }
  function replaceRoute(oldHash,newHash) {
    const oldKey=keyOf(oldHash),newKey=keyOf(newHash),tab=tabs.get(oldKey);if(!tab)return;
    const order=[...tabs.entries()];tabs.clear();
    tab.hash=newHash;tab.key=newKey;for(const [key,value] of order)tabs.set(key===oldKey?newKey:key,value);
    if(active===oldKey){active=newKey;history.replaceState(null,'',newHash);}draw();persist();
  }
  function scriptDraft(hash){return tabs.get(keyOf(hash))?.script;}
  function saved(){const tab=tabs.get(active);if(tab){tab.fields.clear();tab.edits.clear();tab.baseline=new Map(controls(tab.view).map((n,i)=>[fieldKey(n,i),value(n)]));capture();persist();}}
  new ResizeObserver(()=>bar().querySelector('[aria-selected=true]')?.scrollIntoView({block:'nearest',inline:'nearest'})).observe(bar());
  document.addEventListener('input',remember);document.addEventListener('change',remember);
  document.addEventListener('scroll',schedule,true);document.addEventListener('click',schedule);document.addEventListener('toggle',schedule,true);
  window.addEventListener('pagehide',()=>{capture();persist();});
  window.addEventListener('beforeunload',event=>{capture();persist();if([...tabs.values()].some(dirty)||FileDocuments.hasDirty())event.preventDefault();});
  document.getElementById('open-pages').addEventListener('keydown',event=>{
    const buttons=[...bar().querySelectorAll('[role=tab]')],i=buttons.indexOf(event.target);if(i<0)return;
    if(event.altKey&&event.shiftKey&&['ArrowLeft','ArrowRight'].includes(event.key)){
      event.preventDefault();const target=buttons[i+(event.key==='ArrowLeft'?-1:1)];
      if(target)reorder(event.target.dataset.pageKey,target.dataset.pageKey,event.key==='ArrowRight');return;
    }
    if(event.key==='Delete'){event.preventDefault();closeTab(event.target.dataset.pageKey);return;}
    const next=event.key==='ArrowRight'?(i+1)%buttons.length:event.key==='ArrowLeft'?(i+buttons.length-1)%buttons.length:event.key==='Home'?0:event.key==='End'?buttons.length-1:-1;
    if(next>=0){event.preventDefault();buttons[next].focus();buttons[next].click();}
  });
  return {restoreSession,beforeRender,afterRender,replaceRoute,capture,activate,scriptDraft,saved,closeTab,keyOf,workspace:hash=>tabs.get(keyOf(hash))?.workspace,
    resume:hash=>{const tab=tabs.get(keyOf(hash));return tab&&tab.loaded&&tab.key!==active&&!['workflows','workflow-editor','workflow-run','setup','file','part','workspace','scripts','eco','catalog','parts'].includes(tab.view)?tab:null;}};
})();
