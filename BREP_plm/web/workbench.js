/* Home and inbox share a selection/detail interaction. Workspace drag/drop only
   moves workspace entries or creates links; it never reparents PLM revisions. */
'use strict';
globalThis.Workbench = (() => {
  const channels = Object.fromEntries(['home','inbox','catalog'].map(k => [k, { serial: 0, node: null, detail: null }]));
  const homeState = { user: null, nodes: new Map(), expanded: new Set(['root']), focus: 'root', moving: null, epoch: 0 };
  let inboxData = null, inboxSerial = 0;
  const box = mode => document.getElementById(mode === 'catalog' ? 'selection-content' : `${mode}-detail`);
  const layout = mode => document.getElementById(`${mode}-layout`);
  const enc = encodeURIComponent;
  const partLink = (part, revision = '', tab = 'summary') => `#/part/${enc(part)}${revision ? `/${enc(revision)}/${tab}` : ''}`;
  const stateChip = state => `<span class="state state-${escape(state)}">${escape(statusName(state))}</span>`;
  const pairs = items => `<dl class="detail-properties">${items.filter(([,v]) => v !== undefined && v !== null && v !== '').map(([k,v])=>`<dt>${escape(k)}</dt><dd>${escape(typeof v === 'object' ? JSON.stringify(v) : v)}</dd>`).join('')}</dl>`;
  const action = (name, label, extra = '') => `<button type="button" class="ghost" data-detail-action="${name}" ${extra}>${label}</button>`;
  async function safe(fn) { try { await fn(); } catch (error) { banner(error.message); } }
  function reveal(mode, focus) {
    if (!focus) return;
    layout(mode).classList.add('detail-open');
    if (matchMedia('(max-width: 720px)').matches) {
      const heading = box(mode).querySelector('h2');
      if (heading) { heading.tabIndex = -1; heading.focus({preventScroll:true}); }
      layout(mode).scrollIntoView({block:'start'});
    }
  }
  function register(node, parent) {
    const old = homeState.nodes.get(node.key);
    const value = {...old,...node,parent};
    homeState.nodes.set(node.key,value); return value;
  }
  function entryNode(entry, parent) {
    return register({key:`e:${entry.id}`,label:entry.name,kind:entry.kind === 'link' ? 'part' : entry.kind,entry,
      part:entry.link?.part_id,revision:entry.link?.pinned ? entry.link.revision_id : '',children:null},parent);
  }
  const canExpand = node => ['folder','part','revision','catalog'].includes(node.kind);
  const movable = node => (Boolean(node.entry) || ['part','revision'].includes(node.kind));
  async function load(node) {
    if (node.children !== null && node.children !== undefined) return;
    const epoch=homeState.epoch;
    let children=[];
    if (node.kind === 'folder') {
      const response=await api('GET',`/api/workspace/entries?${new URLSearchParams({parent:node.entry?.id || ''})}`);
      if(epoch!==homeState.epoch)return;
      children=response.entries.map(e=>entryNode(e,node.key));
      if(node.key==='root')children.push(register({key:'catalog',label:'All parts',kind:'catalog',children:null},'root'));
    } else if(node.kind === 'catalog') {
      const page=await api('GET','/api/parts?limit=100');
      if(epoch!==homeState.epoch)return;
      children=page.parts.map(p=>register({key:`p:${p.id}`,label:`${p.number} · ${p.name}`,part:p.id,kind:'part',children:null},node.key));
      if(page.next)children.push(register({key:'more',kind:'more',label:'Load more parts',cursor:page.next},node.key));
    } else if(node.kind === 'part') {
      const detail=await api('GET',`/api/parts/${enc(node.part)}`);
      if(epoch!==homeState.epoch)return;
      node.partDetail=detail;
      children=(detail.attachments || []).map(a=>attachmentNode(a,node,'Part attachment'));
      children.push(...detail.revision_views.slice().reverse().map(r=>register({key:`${node.key}:r:${r.id}`,kind:'revision',label:`Revision ${r.label}`,part:detail.id,revision:r.id,state:r.lifecycle,children:null,partDetail:detail},node.key)));
    } else if(node.kind === 'revision') {
      const detail=await api('GET',`/api/parts/${enc(node.part)}`);
      if(epoch!==homeState.epoch)return;
      const revision=detail.revisions.find(r=>r.id===node.revision);
      if(!revision)throw new Error('This revision was deleted. Refresh Home.');
      children=(revision.attachments || []).map(a=>attachmentNode(a,node,'Revision attachment'));
      if(revision.content_hash)children.unshift(register({key:`${node.key}:model`,kind:'model',label:'CAD model',part:node.part,revision:node.revision},node.key));
    }
    node.children=children;
  }
  function attachmentNode(attachment,parent,scope) {
    return register({key:`${parent.key}:a:${attachment.id}`,kind:'attachment',label:attachment.name,attachment,scope,part:parent.part,revision:parent.kind==='revision'?parent.revision:''},parent.key);
  }
  const treeIcons = {
    folder: '<path d="M3 7V5h6l2 2h10v13H3Z" fill="currentColor" fill-opacity=".16"/><path d="M3 7V5h6l2 2h10v13H3Z"/><path d="M3 10h18"/>',
    part: '<path d="m12 3 9 5v9l-9 5-9-5V8Z" fill="currentColor" fill-opacity=".12"/><path d="m12 3 9 5v9l-9 5-9-5V8Z"/><path d="m3 8 9 5 9-5M12 13v9M7.5 5.5l9 5"/>',
    file: '<path d="M5 3h9l5 5v13H5Z"/><path d="M14 3v6h5M8 13h8M8 17h6"/>',
    catalog: '<rect x="3" y="3" width="18" height="18" rx="2"/><path d="M3 9h18M3 15h18M9 3v18M15 3v18"/>',
    revision: '<path d="M6 3v10a4 4 0 0 0 4 4h10m-5-5 5 5-5 5"/>',
    more: '<circle cx="5" cy="12" r="1"/><circle cx="12" cy="12" r="1"/><circle cx="19" cy="12" r="1"/>',
  };
  function treeIcon(kind) {
    const shape = kind === 'model' ? 'part' : kind === 'attachment' ? 'file' : kind;
    return `<svg class="tree-icon tree-icon-${shape}" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${treeIcons[shape] || treeIcons.file}</svg>`;
  }
  function treeMarkup(node) {
    const expandable=canExpand(node), expanded=homeState.expanded.has(node.key);
    const selected=channels.home.node?.key===node.key;
    return `<div role="treeitem" tabindex="${homeState.focus===node.key?'0':'-1'}" data-tree-key="${escape(node.key)}" aria-label="${escape(node.label)}" aria-selected="${selected}"${expandable?` aria-expanded="${expanded}"`:''}>
      <div class="tree-row" ${movable(node)?'draggable="true"':''} ${node.kind==='folder'?'data-drop-folder="true"':''}>
        ${expandable?`<button type="button" class="tree-toggle" tabindex="-1" aria-label="${expanded?'Collapse':'Expand'} ${escape(node.label)}">${expanded?'▾':'▸'}</button>`:'<span class="tree-spacer"></span>'}
        ${expanded&&expandable?'<span class="tree-stem" aria-hidden="true"></span>':''}${treeIcon(node.kind)}<span class="tree-name">${escape(node.label)}</span>${node.state?stateChip(node.state):''}
      </div>
      ${expandable&&expanded?`<div role="group">${node.children ? node.children.length ? node.children.map(treeMarkup).join('') : '<p class="tree-empty">No items</p>' : '<p class="tree-empty">Loading…</p>'}</div>`:''}
    </div>`;
  }
  function drawTree() {
    const root=homeState.nodes.get('root');if(!root)return;
    const active=document.activeElement?.closest('#home-tree [role=treeitem]')?.dataset.treeKey;
    $('home-tree').innerHTML=treeMarkup(root);
    if(active)$('home-tree').querySelector(`[data-tree-key="${CSS.escape(active)}"]`)?.focus({preventScroll:true});
  }
  async function hydrate(node) {
    if(!homeState.expanded.has(node.key))return;
    await load(node);
    for(const child of node.children || [])if(canExpand(child))await hydrate(child);
  }
  async function home(folderId) {
    if(homeState.user!==me.id) {
      homeState.user=me.id;homeState.expanded=new Set(['root']);homeState.moving=null;channels.home.node=null;
    }
    const epoch=++homeState.epoch;
    ++channels.home.serial;
    homeState.nodes=new Map();
    const root=register({key:'root',label:'Home',kind:'folder',children:null},null);
    if(folderId) {
      let id=folderId;const visited=new Set();
      while(id&&!visited.has(id)) {
        visited.add(id);const result=await api('GET',`/api/workspace/entries/${enc(id)}`);
        if(epoch!==homeState.epoch)return;
        const entry=result.entry;
        if(entry.kind!=='folder')throw new Error('This workspace item is not a folder.');
        homeState.expanded.add(`e:${id}`);id=entry.parent;
      }
    }
    await hydrate(root);if(epoch!==homeState.epoch)return;
    const previous=channels.home.node;
    const selected=homeState.nodes.get(previous?.key)||(folderId?homeState.nodes.get(`e:${folderId}`):null)||root;
    homeState.focus=selected.key;drawTree();
    await select('home',selected,false);
  }
  async function toggle(node) {
    if(homeState.expanded.has(node.key))homeState.expanded.delete(node.key);
    else {homeState.expanded.add(node.key);drawTree();await load(node);}
    drawTree();
  }
  async function more(node) {
    const epoch=homeState.epoch;
    const page=await api('GET',`/api/parts?limit=100&after=${enc(node.cursor)}`);
    if(epoch!==homeState.epoch)return;
    const parent=homeState.nodes.get(node.parent);
    parent.children=parent.children.filter(n=>n.kind!=='more');
    parent.children.push(...page.parts.map(p=>register({key:`p:${p.id}`,label:`${p.number} · ${p.name}`,part:p.id,kind:'part',children:null},parent.key)));
    if(page.next){node.cursor=page.next;parent.children.push(node);}
    drawTree();
  }
  async function select(mode,node,focus=true) {
    if(node.kind==='more'){await more(node);return;}
    const channel=channels[mode],serial=++channel.serial;
    channel.node=node;channel.detail=null;
    FileDocuments.park(box(mode));
    box(mode).innerHTML='<p class="empty">Loading details…</p>';
    if(mode==='home'){
      homeState.focus=node.key;
      // Keep rows mounted so the browser can recognize a double-click.
      $('home-tree').querySelectorAll('[data-tree-key]').forEach(element=>{
        const selected=element.dataset.treeKey===node.key;
        element.setAttribute('aria-selected',String(selected));
        element.tabIndex=selected?0:-1;
      });
    }
    try {
      if(node.kind==='folder') {
        await load(node);if(serial!==channel.serial)return;
        box(mode).innerHTML=folderHtml(node);
      } else if(node.kind==='catalog') {
        box(mode).innerHTML='<h2>All parts</h2><p>Expand this collection to browse shared parts. Drag a part onto a Home folder to add a link, or select it and choose Add to folder.</p><a href="#/parts">Search all parts</a>';
      } else if(['part','revision','model'].includes(node.kind)) {
        const detail=await api('GET',`/api/parts/${enc(node.part)}`);if(serial!==channel.serial)return;
        channel.detail=detail;
        const revision=detail.revision_views.find(r=>r.id===node.revision) || (!node.revision?detail.revision_views.at(-1):null);
        if(!revision)throw new Error('The selected revision is no longer available. Refresh this view.');
        channel.revision=revision;
        box(mode).innerHTML=partHtml(mode,node,detail,revision);
        const review=await api('GET',`/api/parts/${enc(detail.id)}/revisions/${enc(revision.id)}/review`);
        if(serial!==channel.serial)return;
        channel.review=review;
        const area=box(mode).querySelector('[data-review-content]');
        area.innerHTML=reviewHtml(review.current,review.comments);
      } else if(node.kind==='attachment') {
        const a=node.attachment;
        box(mode).innerHTML=`<h2>${escape(a.name)}</h2><p class="muted">${escape(node.scope)}</p>${pairs([['Kind',a.kind],['Size',fileSize(a.size)],['Type',a.media_type],['Note',a.note]])}<a class="button-link" href="${FileDocuments.route('attachment',a.id,node.part,node.revision)}">Open document</a> <a class="button-link" href="/api/attachments/${enc(a.id)}" download>Download attachment</a><p><a href="${partLink(node.part,node.revision,'attachments')}">Manage attachments</a></p>${['image/png','image/jpeg','image/webp','image/gif'].includes(a.media_type)?`<img class="detail-preview" src="/api/attachments/${enc(a.id)}?inline=true" alt="${escape(a.name)}">`:''}`;
      } else if(node.kind==='file') {
        const answer=await api('GET',`/api/workspace/entries/${enc(node.entry.id)}/versions`);if(serial!==channel.serial)return;
        const versions=answer.versions || answer;
        box(mode).innerHTML=`<h2>${escape(node.label)}</h2>${pairs([['Version',node.entry.file?.version],['Size',fileSize(node.entry.file?.size || 0)]])}<div class="toolbar"><a class="button-link" href="${FileDocuments.route('workspace',node.entry.id)}">Open document</a> <a class="button-link" href="/api/workspace/entries/${enc(node.entry.id)}/content" download>Download</a>${entryActions(node)}</div><details><summary>File versions</summary><ul class="detail-files">${versions.slice().reverse().map(v=>`<li>Version ${v.version} · ${fileSize(v.size)} <a href="/api/workspace/entries/${enc(node.entry.id)}/content?version=${v.version}" download>Download</a>${!v.current?action('restore','Restore',`data-version="${v.version}"`):''}</li>`).join('')}</ul></details>${me.can_author?`<details><summary>Promote to attachment</summary><label>Part number<input data-promote-part></label><label>Revision (optional)<input data-promote-revision></label>${action('promote','Promote')}</details>`:''}`;
      } else if(node.kind==='eco') {
        const eco=await api('GET',`/api/ecos/${enc(node.part)}`);if(serial!==channel.serial)return;
        channel.eco=eco;
        box(mode).innerHTML=`<h2>${escape(eco.number)} · ${escape(eco.title)}</h2>${stateChip(eco.state)}<p>${escape(eco.reason)}</p><p>${escape(eco.description)}</p><section class="selection-review">${reviewHtml(eco.current,eco.comments)}</section><details><summary>Affected revisions (${eco.items.length})</summary><ul>${eco.items.map(i=>`<li><a href="${partLink(i.part_id,i.revision_id)}">${escape(i.number)} · ${escape(i.revision)}</a> · ${escape(i.action)}</li>`).join('')}</ul></details><p><a href="#/eco/${enc(eco.id)}">Open change order</a></p>`;
      }
      if(['file','attachment'].includes(node.kind)) {
        const hash=node.kind==='file'?FileDocuments.route('workspace',node.entry.id):FileDocuments.route('attachment',node.attachment.id,node.part,node.revision);
        const host=document.createElement('div');host.className='selection-document';box(mode).append(host);
        await FileDocuments.prepare(hash);
        if(serial!==channel.serial || !host.isConnected)return;
        // A double-click can navigate away while the editor is loading.
        if(mode==='home'&&!location.hash.startsWith('#/workspace'))return;
        const openLink=box(mode).querySelector('a.button-link');
        FileDocuments.mount(hash,host,(file,newHash)=>{
          if(node.kind==='attachment')node.attachment=file;
          else node.entry.file=file;
          if(openLink?.isConnected)openLink.href=newHash;
        });
      }
      if(serial===channel.serial)reveal(mode,focus);
    } catch(error) {
      if(serial===channel.serial){box(mode).innerHTML=`<p class="error">${escape(error.message)}</p>`;reveal(mode,focus);}
    }
  }
  function entryActions(node) {
    if(!node.entry)return '';
    return `${action('move','Move…')}${action('rename','Rename')}${action('delete',node.entry.kind==='link'?'Remove link':'Delete')}`;
  }
  function folderHtml(node) {
    const items=(node.children || []).filter(n=>n.kind!=='catalog');
    return `<h2>${escape(node.label)}</h2><p>${items.length} items</p><div class="toolbar">${node.entry?`<a class="button-link" href="#/workspace/${enc(node.entry.id)}">Open folder tab</a>`:''}${homeState.moving?action('place',`${homeState.moving.entry?'Move':'Link'} ${escape(homeState.moving.label)} here`)+action('cancel-move','Cancel move'):''}${entryActions(node)}</div><details class="folder-tools"><summary>Add to this folder</summary><label>Folder name<input data-folder-name autocomplete="off"></label>${action('new-folder','Create folder')}<label>Part number<input data-link-part autocomplete="off"></label><label>Revision (optional)<input data-link-revision></label>${action('link','Add part link')}<label>Files<input data-folder-files type="file" multiple></label></details><p class="muted">Expand folders to explore their contents. Drop files or drag a workspace item onto a folder to organize it.</p>${items.length?`<ul class="detail-files">${items.map(n=>`<li><button class="link" data-select-tree="${escape(n.key)}">${escape(n.label)}</button><span class="muted">${escape(n.kind)}</span></li>`).join('')}</ul>`:'<p class="empty">This folder is empty.</p>'}`;
  }
  function partHtml(mode,node,detail,rev) {
    const raw=detail.revisions.find(r=>r.id===rev.id);
    const attachments=node.kind==='revision' || node.kind==='model' ? raw?.attachments || [] : detail.attachments || [];
    const attributes=Object.entries(detail.attributes || {}).map(([key,value])=>[detail.schema?.find(s=>s.key===key)?.name || key,value]);
    const extra=mode==='home' ? node.entry?entryActions(node):action('move','Add to folder…') : '';
    let primary='';
    if(me.can_author&&rev.editable&&!rev.locked_by)primary+=action('checkout','Check out');
    if(rev.locked_by_me)primary+=action('checkin','Check in');
    if(me.can_author&&rev.lifecycle==='draft')primary+=action('submit','Request approval');
    if(me.can_checkin&&rev.editable&&!rev.locked_by)primary+=action('release','Release');
    return `<div class="detail-heading"><span data-model-preview>${thumb(['revision','model'].includes(node.kind)?rev.thumbnail?.url:detail.thumbnail_url,'lg')}</span><div><h2>${escape(detail.number)}</h2><p>${escape(detail.name)}</p></div></div><p>${escape(detail.description || '')}</p>
      <div class="revision-heading"><strong>Revision ${escape(rev.label)}</strong>${stateChip(rev.lifecycle)}</div>
      ${pairs([['Type',detail.part_type],['Category',detail.category_path || detail.category],['Contains 3D geometry',(raw?.has_geometry ?? detail.has_geometry) ? 'Yes' : 'No'],['Has thumbnail',(raw?.has_thumbnail ?? detail.has_thumbnail) ? 'Yes' : 'No'],['Checked out by',rev.locked_by || 'Not checked out'],...attributes])}
      <div class="toolbar">${openInCadButton(detail.id,rev.id,rev.editable?'Edit in CAD':'Open in CAD') || ''}${primary}</div>
      <section class="selection-review"><h3>Approval</h3><div data-review-content>Loading review…</div></section>
      <section><h3>${['revision','model'].includes(node.kind)?'Revision':'Part'} attachments</h3>${attachments.length?`<ul class="detail-files">${attachments.map(a=>`<li>${FileDocuments.documentLink(a,detail.id,['revision','model'].includes(node.kind)?rev.id:'')}<small>${escape(a.kind)} · ${fileSize(a.size)}</small></li>`).join('')}</ul>`:'<p class="muted">No attachments.</p>'}<details><summary>Add an attachment</summary><label>Attach to<select data-attachment-scope><option value="part">Part</option>${rev.editable?`<option value="revision"${['revision','model'].includes(node.kind)?' selected':''}>Revision ${escape(rev.label)}</option>`:''}</select></label><label>File<input type="file" data-attachment-file${me.can_author?'':' disabled'}></label></details></section>
      <div class="toolbar">${action('bom','BOM structure')}${extra}${mode==='home'&&node.entry?.kind==='link'?action(node.entry.link.pinned?'follow':'pin',node.entry.link.pinned?'Follow newest':'Pin this revision'):''}<a href="${partLink(detail.id,rev.id,'summary')}">Full part details</a></div><div data-bom-content></div>
      <details><summary>More information</summary>${pairs([['Created',when(detail.created_at)]])}<p><a href="${partLink(detail.id,rev.id,'revisions')}">Manage revisions</a> · <a href="${partLink(detail.id,rev.id,'sourcing')}">Sourcing</a> · <a href="${partLink(detail.id,rev.id,'history')}">History</a></p></details>`;
  }
  function reviewHtml(round,comments=[]) {
    if(!round)return '<p class="muted">No active approval request.</p>';
    return `<p>${reviewChip(round.status)} <strong>${round.approvals} / ${round.required_approvals} approvals</strong></p><p class="muted">Requested by ${escape(round.opened_by)}${round.due?` · Due ${escape(when(round.due))}`:''}</p>${round.can_decide?`<label>Decision comment<textarea class="short-area" data-review-comment rows="3" placeholder="Required when rejecting"></textarea></label><div class="toolbar">${action('approve','Approve')}${action('reject','Reject')}</div>`:''}<details><summary>Reviewers and discussion</summary><ul>${round.reviewers.map(r=>`<li>${escape(r.name)}${r.verdict?` · ${escape(r.verdict)}`:''}</li>`).join('')}</ul>${comments.map(c=>`<p><strong>${escape(c.author)}</strong> ${escape(c.body)}</p>`).join('')}</details>`;
  }
  async function catalog(part,revealDetail=false) {await select('catalog',{kind:'part',part},revealDetail);}
  async function inbox(keep=true) {
    const serial=++inboxSerial;
    const data=await api('GET','/api/inbox');if(serial!==inboxSerial)return;
    inboxData=data;drawInbox();
    const selected=channels.inbox.node;
    if(keep&&selected)await select('inbox',selected,false);
  }
  function drawInbox() {
    const list=inboxData?.[$('inbox-filter').value] || [];
    $('inbox-empty').hidden=Boolean(list.length);
    $('inbox-items').innerHTML=list.map((item,index)=>`<button type="button" class="inbox-item" data-inbox-index="${index}" aria-pressed="${channels.inbox.node?.part===item.target&&channels.inbox.node?.revision===item.revision_id}"><strong>${escape(item.title)}</strong><span>${escape(item.name)}</span><small>${item.kind==='workflow'?escape(item.state):`${item.approvals}/${item.required_approvals} approvals`}${item.due?` · ${item.overdue?'Overdue':'Due'} ${escape(when(item.due))}`:''}</small></button>`).join('');
  }
  async function refresh(mode) {
    if(mode==='home')await home();
    else if(mode==='inbox')await inbox();
    else if(channels[mode].node)await select(mode,channels[mode].node,false);
    await refreshInbox();
  }
  async function moveTo(node,folder) {
    const parent=folder.entry?.id || '';
    if(node.entry) {
      if(node.entry.id===parent)throw new Error('A folder cannot be moved inside itself.');
      await api('PATCH',`/api/workspace/entries/${enc(node.entry.id)}`,{parent});
    } else if(node.part)await api('POST','/api/workspace/links',{parent,part:node.part,revision:node.kind==='revision'?node.revision:''});
    else throw new Error('This item cannot be moved.');
    homeState.moving=null;homeState.expanded.add(folder.key);
    await home();$('home-status').textContent=`${node.label} ${node.entry?'moved':'linked'} to ${folder.label}.`;
  }
  async function upload(url,method,file) {
    const response=await fetch(url,{method,credentials:'same-origin',headers:{'Content-Type':file.type||'application/octet-stream','X-CSRF-Token':csrfToken},body:file});
    if(!response.ok){let message=response.statusText;try{message=(await response.json()).error||message;}catch{}throw new Error(message);}
  }
  async function uploadFolder(node,files) {
    const parent=node.entry?.id || '';
    const folder=await api('GET',`/api/workspace/entries?${new URLSearchParams({parent})}`);
    for(const file of files) {
      const same=folder.entries.find(e=>e.kind==='file'&&e.name.toLowerCase()===file.name.toLowerCase());
      await upload(same?`/api/workspace/entries/${enc(same.id)}/content`:`/api/workspace/files?${new URLSearchParams({parent,name:file.name})}`,same?'PUT':'POST',file);
      // Refresh between files so duplicate names in one drop become versions too.
      folder.entries=(await api('GET',`/api/workspace/entries?${new URLSearchParams({parent})}`)).entries;
    }
    homeState.expanded.add(node.key);await home();
  }
  async function detailAction(mode,button) {
    const channel=channels[mode],node=channel.node,detail=channel.detail,rev=channel.revision;
    if(!node)return;
    const area=box(mode),name=button.dataset.detailAction;
    const base=detail&&rev?`/api/parts/${enc(detail.id)}/revisions/${enc(rev.id)}`:'';
    if(name==='move'){homeState.moving=node;$('home-status').textContent=`Choose a destination folder, then select ${node.entry?'Move':'Link'} here.`;layout('home').classList.remove('detail-open');return;}
    if(name==='cancel-move'){homeState.moving=null;await select(mode,node,false);return;}
    if(name==='place'){if(homeState.moving)await moveTo(homeState.moving,node);return;}
    if(name==='rename') {const value=prompt('New name',node.entry.name);if(!value?.trim())return;await api('PATCH',`/api/workspace/entries/${enc(node.entry.id)}`,{name:value.trim()});}
    else if(name==='pin'||name==='follow')await api('PATCH',`/api/workspace/entries/${enc(node.entry.id)}`,{revision:name==='pin'?rev.id:''});
    else if(name==='delete'){if(!confirm(node.entry.kind==='link'?'Remove this workspace link? The part and revisions stay.':'Delete this workspace item and its contents?'))return;await api('DELETE',`/api/workspace/entries/${enc(node.entry.id)}?recursive=${node.entry.kind==='folder'}`);}
    else if(name==='new-folder'){const value=area.querySelector('[data-folder-name]').value.trim();if(!value)throw new Error('Enter a folder name.');await api('POST','/api/workspace/folders',{parent:node.entry?.id||'',name:value});homeState.expanded.add(node.key);}
    else if(name==='link'){const part=area.querySelector('[data-link-part]').value.trim();if(!part)throw new Error('Enter a part number.');await api('POST','/api/workspace/links',{parent:node.entry?.id||'',part,revision:area.querySelector('[data-link-revision]').value.trim()});homeState.expanded.add(node.key);}
    else if(name==='restore')await api('POST',`/api/workspace/entries/${enc(node.entry.id)}/versions/${button.dataset.version}/restore`);
    else if(name==='promote')await api('POST',`/api/workspace/entries/${enc(node.entry.id)}/promote`,{part:area.querySelector('[data-promote-part]').value.trim(),revision:area.querySelector('[data-promote-revision]').value.trim(),kind:'other'});
    else if(name==='checkout'||name==='checkin')await api('POST',`${base}/${name}`,name==='checkout'?{client_id:'web'}:{force:false});
    else if(name==='release'){if(!confirm(`Release revision ${rev.label}? Its document becomes immutable.`))return;showWarnings(await api('POST',`${base}/state`,{to:'released'}));}
    else if(name==='submit'){currentPart=detail;await openSubmit(rev.id);return;}
    else if(name==='approve'||name==='reject') {
      const comment=area.querySelector('[data-review-comment]')?.value || '';
      if(name==='reject'&&!comment.trim())throw new Error('Add a comment explaining the rejection.');
      const url=node.kind==='eco'?`/api/ecos/${enc(node.part)}`:base;
      showWarnings(await api('POST',`${url}/review/decision`,{verdict:name,comment}));
    } else if(name==='bom') {
      const serial=channel.serial;
      const target=area.querySelector('[data-bom-content]');
      target.innerHTML=bomHtml();
      const view={show:false};let read=0;
      const reload=async()=>{
        const request=++read;
        const isCurrent=()=>channel.serial===serial&&request===read&&target.isConnected;
        const bom=await api('GET',`${base}/bom?levels=0&flat=false&occurrences=true`);
        if(!isCurrent())return;
        await drawBomAttributes({bom,head:target.querySelector('thead'),rows:target.querySelector('tbody'),selection:target.querySelector('[data-bom-layout]'),editButton:target.querySelector('[data-bom-edit-attributes]'),status:target.querySelector('[data-bom-status]'),treeControls:target.querySelector('[data-bom-tree-controls]'),view,reload,isCurrent});
        if(isCurrent())target.querySelector('[data-bom-empty]').hidden=bom.lines.length>0;
      };
      await reload();return;
    }
    await refresh(mode);
  }
  function bomHtml() {
    return `<section class="selection-bom"><h3>BOM structure</h3><div class="toolbar"><label class="inline-select">Columns <select data-bom-layout></select></label><button type="button" class="ghost author-only" data-bom-edit-attributes aria-pressed="false">Edit attributes</button><a class="button-link" href="#/bom-columns">Configure columns…</a></div><div class="toolbar bom-tree-controls" data-bom-tree-controls></div><p class="muted" data-bom-status role="status"></p><div class="bom-scroll"><table class="bom"><thead></thead><tbody></tbody></table></div><p class="muted" data-bom-empty hidden>No components.</p></section>`;
  }

  function partChanged(detail) {
    for(const [mode,channel] of Object.entries(channels)) {
      const visible=mode==='catalog'?!$('view-parts').hidden:mode==='home'?!$('view-workspace').hidden:!$('view-reviews').hidden;
      if(visible&&channel.node?.part===detail.id)safe(()=>refresh(mode));
    }
  }
  function init() {
    $('home-tree').addEventListener('click',event=>{
      const element=event.target.closest('[data-tree-key]');if(!element)return;
      const node=homeState.nodes.get(element.dataset.treeKey);if(!node)return;
      safe(()=>event.target.closest('.tree-toggle')?toggle(node):select('home',node));
    });
    $('home-tree').addEventListener('dblclick',event=>{
      if(event.target.closest('.tree-toggle'))return;
      const item=event.target.closest('[data-tree-key]');
      const node=homeState.nodes.get(item?.dataset.treeKey);
      if(node?.kind==='attachment'){location.hash=FileDocuments.route('attachment',node.attachment.id,node.part,node.revision);}
      else if(node?.kind==='file'){location.hash=FileDocuments.route('workspace',node.entry.id);}
      else if(node?.kind==='folder'){location.hash=node.entry?`#/workspace/${enc(node.entry.id)}`:'#/workspace';}
      else if(node&&['part','revision','model'].includes(node.kind)){
        location.hash=partLink(node.part,node.revision);
      }
    });
    $('home-tree').addEventListener('keydown',event=>{
      const element=event.target.closest('[data-tree-key]');if(!element)return;
      const node=homeState.nodes.get(element.dataset.treeKey);
      if(!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','Home','End','Enter',' '].includes(event.key))return;
      event.preventDefault();
      const visible=[...$('home-tree').querySelectorAll('[role=treeitem]')];let target;
      if(event.key==='ArrowDown')target=visible[Math.min(visible.indexOf(element)+1,visible.length-1)];
      if(event.key==='ArrowUp')target=visible[Math.max(visible.indexOf(element)-1,0)];
      if(event.key==='Home')target=visible[0];if(event.key==='End')target=visible.at(-1);
      if(event.key==='ArrowRight'&&canExpand(node)) {
        if(!homeState.expanded.has(node.key))safe(()=>toggle(node));
        else target=element.querySelector('[role=group] [role=treeitem]');
      }
      if(event.key==='ArrowLeft') {
        if(canExpand(node)&&homeState.expanded.has(node.key))safe(()=>toggle(node));
        else if(node.parent)target=$('home-tree').querySelector(`[data-tree-key="${CSS.escape(node.parent)}"]`);
      }
      if(target){homeState.focus=target.dataset.treeKey;visible.forEach(n=>n.tabIndex=n===target?0:-1);target.focus();}
      if(event.key==='Enter'||event.key===' ')safe(()=>select('home',node));
    });
    let dragging=null;
    $('home-tree').addEventListener('dragstart',event=>{
      const row=event.target.closest('.tree-row');const node=homeState.nodes.get(row?.parentElement.dataset.treeKey);
      if(!node||!movable(node)){event.preventDefault();return;}
      dragging=node;event.dataTransfer.effectAllowed=node.entry?'move':'copy';event.dataTransfer.setData('application/x-brep-workspace',node.key);
    });
    $('home-tree').addEventListener('dragend',()=>{dragging=null;document.querySelectorAll('.drop-target').forEach(e=>e.classList.remove('drop-target'));});
    $('home-tree').addEventListener('dragover',event=>{
      const row=event.target.closest('[data-drop-folder]');if(!row)return;
      if(!dragging&&!event.dataTransfer.types.includes('Files'))return;
      event.preventDefault();event.dataTransfer.dropEffect=dragging?.entry?'move':'copy';row.classList.add('drop-target');
    });
    $('home-tree').addEventListener('dragleave',event=>event.target.closest('[data-drop-folder]')?.classList.remove('drop-target'));
    $('home-tree').addEventListener('drop',event=>{
      const row=event.target.closest('[data-drop-folder]');if(!row)return;
      event.preventDefault();row.classList.remove('drop-target');const folder=homeState.nodes.get(row.parentElement.dataset.treeKey);
      const source=dragging;dragging=null;
      safe(()=>source?moveTo(source,folder):uploadFolder(folder,[...event.dataTransfer.files]));
    });
    $('inbox-filter').addEventListener('change',()=>{channels.inbox.serial++;channels.inbox.node=null;box('inbox').innerHTML='<p class="empty">Select a request to review.</p>';layout('inbox').classList.remove('detail-open');drawInbox();});
    $('inbox-items').addEventListener('click',event=>{
      const button=event.target.closest('[data-inbox-index]');if(!button)return;
      const item=inboxData[$('inbox-filter').value][Number(button.dataset.inboxIndex)];
      if(item.kind==='workflow'){location.hash=`#/workflow-run/${encodeURIComponent(item.target)}`;return;}
      safe(async()=>{await select('inbox',{kind:item.kind==='eco'?'eco':'revision',part:item.target,revision:item.revision_id});drawInbox();});
    });
    document.querySelectorAll('[data-master-back]').forEach(button=>button.addEventListener('click',()=>{
      const mode=button.dataset.masterBack;layout(mode).classList.remove('detail-open');
      layout(mode).querySelector('[role=treeitem][tabindex="0"], .inbox-item[aria-pressed=true], .inspect-part[aria-pressed=true]')?.focus({preventScroll:true});
    }));
    for(const mode of Object.keys(channels)) {
      box(mode).addEventListener('click',event=>{
        const tree=event.target.closest('[data-select-tree]');if(tree){safe(()=>select('home',homeState.nodes.get(tree.dataset.selectTree)));return;}
        const button=event.target.closest('[data-detail-action]');if(!button)return;
        if(channels[mode].busy)return;
        channels[mode].busy=true;button.disabled=true;
        safe(async()=>{try{await detailAction(mode,button);}finally{channels[mode].busy=false;button.disabled=false;}});
      });
      box(mode).addEventListener('change',event=>{
        const input=event.target,files=[...(input.files || [])];if(!files.length)return;
        const channel=channels[mode],node=channel.node,detail=channel.detail,revision=channel.revision;
        if(input.matches('[data-folder-files]'))safe(()=>uploadFolder(node,files));
        if(input.matches('[data-attachment-file]')) {
          const scope=box(mode).querySelector('[data-attachment-scope]').value;
          const url=`/api/parts/${enc(detail.id)}${scope==='revision'?`/revisions/${enc(revision.id)}`:''}/attachments?${new URLSearchParams({name:files[0].name,kind:'other'})}`;
          safe(async()=>{await upload(url,'POST',files[0]);await refresh(mode);});
        }
      });
    }
    // Narrow tables become labelled cards. Only BOM/structure retains horizontal
    // scrolling; labels follow freshly rendered headers, including admin screens.
    let queued=false;
    const labelTables=()=>{
      queued=false;
      for(const table of document.querySelectorAll('table')) {
        if(table.closest('#bom-panel, .bom-scroll'))continue;
        const labels=[...(table.tHead?.rows[0]?.cells || [])].map(h=>h.textContent.trim() || h.getAttribute('aria-label') || '');
        for(const row of table.tBodies)for(const tr of row.rows)[...tr.cells].forEach((cell,i)=>{cell.dataset.label=labels[i] || '';});
      }
    };
    new MutationObserver(()=>{if(!queued){queued=true;queueMicrotask(labelTables);}}).observe($('shell'),{childList:true,subtree:true});
    labelTables();
  }
  async function savedModels(changed, stale) {
    for(const [mode,channel] of Object.entries(channels)) {
      const visible=mode==='catalog'?!$('view-parts').hidden:mode==='home'?!$('view-workspace').hidden:!$('view-reviews').hidden;
      const node=channel.node,serial=channel.serial;
      if(!visible || !channel.detail || (!stale&&!changed.has(node?.part)))continue;
      const detail=await api('GET',`/api/parts/${enc(node.part)}`);
      if(serial!==channel.serial)continue;
      const revision=detail.revision_views.find(r=>r.id===channel.revision?.id);
      if(!revision)continue;
      channel.detail=detail;channel.revision=revision;
      const preview=box(mode).querySelector('[data-model-preview]');
      if(preview)preview.innerHTML=thumb(['revision','model'].includes(node.kind)?revision.thumbnail?.url:detail.thumbnail_url,'lg');
      // First save can add a model to an already expanded revision.
      if(mode==='home') {
        for(const item of homeState.nodes.values())if(item.part===detail.id&&['part','revision'].includes(item.kind))item.children=null;
        await hydrate(homeState.nodes.get('root'));
        if(serial===channel.serial)drawTree();
      }
    }
  }
  return {init,home,inbox,catalog,partChanged,savedModels,
    inboxSelection:()=>channels.inbox.node,restoreInbox:node=>{channels.inbox.node=node;},
    snapshot:()=>({expanded:[...homeState.expanded],selected:channels.home.node?.key,focus:homeState.focus}),
    restore:state=>{homeState.user=me.id;homeState.expanded=new Set(state?.expanded || ['root']);channels.home.node=state?.selected?{key:state.selected}:null;homeState.focus=state?.focus || 'root';}
  };
})();
