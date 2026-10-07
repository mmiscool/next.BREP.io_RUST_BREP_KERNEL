/* MIME adapters own rendering; this host owns tabs, permissions, drafts and saves. */
'use strict';
globalThis.FileDocuments = (() => {
  const adapters=[],documents=new Map(),opening=new Map();let active=null,libraries;
  const element=(tag,className,text)=>{const n=document.createElement(tag);if(className)n.className=className;if(text!==undefined)n.textContent=text;return n;};
  function register(adapter) {
    if(!adapter.id || typeof adapter.create!=='function')throw new Error('A document adapter needs an id and create(context).');
    const old=adapters.findIndex(a=>a.id===adapter.id);if(old>=0)adapters.splice(old,1);
    adapters.push(adapter);
  }
  function renderer(file) {
    const mime=(file.media_type || '').split(';')[0].toLowerCase(),extension=file.name.toLowerCase().split('.').pop();
    return [...adapters].reverse().find(a=>a.mimeTypes?.some(type=>type===mime))
      || [...adapters].reverse().find(a=>a.extensions?.includes(extension)) || [...adapters].reverse().find(a=>a.mimeTypes?.some(type=>type.endsWith('/*')&&mime.startsWith(type.slice(0,-1)))) || adapters.find(a=>a.id==='download');
  }
  const route=(kind,id,part='',revision='')=>`#/${kind==='workspace'?'file':'attachment'}/${encodeURIComponent(id)}${part?`/${encodeURIComponent(part)}${revision?`/${encodeURIComponent(revision)}`:''}`:''}`;
  function documentLink(file,part='',revision='') { return `<a href="${escape(route('attachment',file.id,part,revision))}">${escape(file.name)}</a>`; }
  function loadScript(src) {return new Promise((resolve,reject)=>{const script=document.createElement('script');script.src=src;script.onload=resolve;script.onerror=()=>reject(new Error('Could not load the Markdown editor.'));document.head.append(script);});}
  function markdownLibraries() {
    if(!libraries)libraries=loadScript('/markdown-editor.js').catch(error=>{libraries=null;throw error;});
    return libraries;
  }
  async function requestText(url) {
    const r=await fetch(url,{credentials:'same-origin'});if(!r.ok)throw new Error(`Could not read this file (${r.status}).`);return r.text();
  }
  function update(doc) {
    const dirty=doc.controller?.isDirty?.() || false;
    doc.save.disabled=!doc.writable || !dirty || doc.saving;
    doc.status.textContent=doc.saving?'Saving…':dirty?'Unsaved changes':doc.savedMessage || '';
  }
  function mount(hash,container=document.getElementById('view-file'),onSaved) {
    const doc=documents.get(hash);if(!doc)return;
    container.append(doc.panel);if(onSaved)doc.onSaved=onSaved;
    active=hash;for(const [key,d] of documents)d.panel.hidden=key!==hash;
  }
  async function open(hash) {
    await prepare(hash);
    mount(hash);return documents.get(hash).name;
  }
  async function prepare(hash) {
    if(!documents.has(hash)) {
      if(!opening.has(hash))opening.set(hash,createDocument(hash).finally(()=>opening.delete(hash)));
      await opening.get(hash);
    }
    return documents.get(hash).name;
  }
  async function createDocument(hash) {
    const [,kind,rawId,rawPart,rawRev]=hash.split('/'),id=decodeURIComponent(rawId),part=rawPart?decodeURIComponent(rawPart):'',revision=rawRev?decodeURIComponent(rawRev):'';
    const scope=new URLSearchParams({part,revision});let meta,file,writable,reason,url;
    if(kind==='file') {
      meta=(await api('GET',`/api/workspace/entries/${encodeURIComponent(id)}`)).entry;
      if(meta.kind!=='file')throw new Error('This workspace item is not a file.');
      file={...meta.file,name:meta.name,id};writable=meta.owner===me.id;reason=writable?'':'Only the owner can edit this file.';
      url=`/api/workspace/entries/${encodeURIComponent(id)}/content`;
    } else {
      meta=await api('GET',`/api/attachments/${encodeURIComponent(id)}/info?${scope}`);
      file=meta.attachment;writable=meta.writable;reason=meta.read_only_reason;
      url=`/api/attachments/${encodeURIComponent(id)}?${scope}`;
    }
    const adapter=renderer(file),panel=element('section','file-document');panel.setAttribute('aria-label',file.name);
    const toolbar=element('div','toolbar'),save=element('button',null,'Save'),download=element('a','button-link','Download'),status=element('span','muted');
    save.type='button';download.href=url;download.download=file.name;toolbar.append(save,download,status);panel.append(toolbar);
    if(!writable&&reason)panel.append(element('p','muted',reason));
    const content=element('div','file-document-content');panel.append(content);panel.hidden=true;document.getElementById('view-file').append(panel);
    const doc={hash,kind,id,part,revision,file,name:file.name,writable,panel,content,save,status,url,scope};
    try {
      doc.controller=await adapter.create({container:content,file,url,writable,readText:()=>requestText(url),onChange:()=>update(doc)});
      save.hidden=typeof doc.controller.getContent!=='function';
      save.addEventListener('click',()=>act(()=>saveDocument(doc)));documents.set(hash,doc);
      update(doc);return file.name;
    } catch(error){doc.controller?.destroy?.();panel.remove();throw error;}
  }
  async function saveDocument(doc) {
    if(doc.saving || !doc.writable || !doc.controller.isDirty())return;
    doc.saving=true;update(doc);
    try {
      const original=doc.controller.getContent(),body=original;
      const url=doc.kind==='file'?doc.url:`/api/attachments/${encodeURIComponent(doc.id)}?${doc.scope}`;
      const r=await fetch(url,{method:'PUT',credentials:'same-origin',headers:{'Content-Type':doc.file.media_type || 'text/plain','X-CSRF-Token':csrfToken,'If-Match':`"${doc.file.sha256}"`},body});
      const answer=await r.json();if(!r.ok)throw new Error(r.status===404?'This file was replaced or removed. Reopen it before saving.':answer.error || `Save failed (${r.status}).`);
      if(doc.kind==='file')doc.file={...answer.entry.file,name:answer.entry.name,id:doc.id};
      else {
        doc.file=answer.attachment;doc.id=doc.file.id;
        const old=doc.hash,next=route('attachment',doc.id,doc.part,doc.revision);
        documents.delete(old);doc.hash=next;documents.set(next,doc);
        if(active===old)active=next;OpenPages.replaceRoute(old,next);
        doc.url=`/api/attachments/${encodeURIComponent(doc.id)}?${doc.scope}`;
        doc.panel.querySelector('a[download]').href=doc.url;
      }
      doc.controller.markSaved(original);doc.savedMessage='Saved.';
      doc.onSaved?.(doc.file,doc.hash);
    } finally {doc.saving=false;update(doc);}
  }
  register({id:'download',create:async({container})=>{container.append(element('p','muted','No viewer is available for this file type yet. Use Download to open it locally.'));return {};}});
  register({id:'pdf',mimeTypes:['application/pdf'],extensions:['pdf'],create:async({container,url,file})=>{
    const frame=element('iframe','document-pdf');frame.title=file.name;frame.src=url+(url.includes('?')?'&':'?')+'inline=true';container.append(frame);return {};
  }});
  register({id:'image',mimeTypes:['image/png','image/jpeg','image/gif','image/webp'],create:async({container,url,file})=>{
    const image=element('img','document-image');image.alt=file.name;image.src=url+(url.includes('?')?'&':'?')+'inline=true';container.append(image);return {};
  }});
  register({id:'markdown',mimeTypes:['text/markdown','text/x-markdown'],extensions:['md','markdown','mdown'],create:async({container,readText,writable,onChange})=>{
    await markdownLibraries();let text=await readText(),saved=text,visual=null,mode='visual';
    const modes=element('div','toolbar document-modes'),visualButton=element('button','ghost',writable?'Visual editor':'Rendered'),sourceButton=element('button','ghost','Markdown'),visualHost=element('div','markdown-visual'),source=element('textarea','markdown-source');
    source.setAttribute('aria-label','Markdown source');source.spellcheck=false;source.readOnly=!writable;source.value=text;source.hidden=true;
    for(const button of [visualButton,sourceButton])button.type='button';modes.append(visualButton,sourceButton);container.append(modes,visualHost,source);
    // Markdown is rendered through the editor's document schema, never raw HTML.
    visualHost.addEventListener('click',event=>{
      const link=event.target.closest('a[href]');if(!link)return;
      try{if(!['http:','https:','mailto:','tel:'].includes(new URL(link.href,location.href).protocol))event.preventDefault();}catch{event.preventDefault();}
    });
    visual=await PlmMarkdownEditor.create(visualHost,text,writable,markdown=>{if(mode==='visual'&&writable){text=markdown;onChange();}});
    function switchMode(next) {
      if(next!==mode) {
        if(next==='source')source.value=text;else visual.setContent(text);
        mode=next;
      }
      source.hidden=mode!=='source';visualHost.hidden=mode!=='visual';
      visualButton.setAttribute('aria-pressed',String(mode==='visual'));sourceButton.setAttribute('aria-pressed',String(mode==='source'));
      onChange();
    }
    visualButton.addEventListener('click',()=>switchMode('visual'));sourceButton.addEventListener('click',()=>switchMode('source'));
    source.addEventListener('input',()=>{text=source.value;onChange();});switchMode('visual');
    return {
      getContent:()=>text,isDirty:()=>text!==saved,markSaved:value=>{saved=value;},destroy:()=>visual.destroy(),
      snapshot:()=>({text,saved,mode,cursor:[source.selectionStart,source.selectionEnd],visualSelection:visual.getSelection()}),
      restoreSnapshot:state=>{text=state.text;saved=state.saved;source.value=text;visual.setContent(text);switchMode(state.mode==='source'?'source':'visual');if(state.cursor)source.setSelectionRange(...state.cursor);visual.setSelection(state.visualSelection);},
    };
  }});
  return {register,renderer,open,prepare,mount,
    snapshot:hash=>{const doc=documents.get(hash),state=doc?.controller?.snapshot?.();return state?{state,sha256:doc.file.sha256,dirty:doc.controller.isDirty()}:null;},
    restore:(hash,snapshot)=>{const doc=documents.get(hash);if(!doc?.controller?.restoreSnapshot||!snapshot)return;let state=snapshot.state;if(!snapshot.dirty&&snapshot.sha256!==doc.file.sha256){const current=doc.controller.getContent();state={...state,text:current,saved:current};}doc.controller.restoreSnapshot(state);if(snapshot.dirty)doc.file.sha256=snapshot.sha256;update(doc);},
    park:container=>{for(const doc of documents.values())if(container.contains(doc.panel)){doc.panel.hidden=true;document.getElementById('view-file').append(doc.panel);}},route,documentLink,isDirty:hash=>documents.get(hash)?.controller?.isDirty?.() || false,
    snapshots:container=>[...documents.values()].filter(doc=>container?.contains(doc.panel)).map(doc=>[doc.hash,FileDocuments.snapshot(doc.hash)]).filter(([,state])=>state),
    hasDirty:()=>[...documents.values()].some(doc=>doc.controller?.isDirty?.()),
    isSaving:hash=>documents.get(hash)?.saving || false,close:hash=>{const doc=documents.get(hash);if(doc){doc.controller.destroy?.();doc.panel.remove();documents.delete(hash);}}};
})();
