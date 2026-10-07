/* Native PLM Markdown editing: safe DOM rendering, Markdown storage, no dependencies. */
'use strict';
globalThis.PlmMarkdownEditor = (() => {
  const make = (tag, text) => { const node=document.createElement(tag);if(text!==undefined)node.textContent=text;return node; };
  const blockTags = new Set(['P','DIV','H1','H2','H3','H4','H5','H6','BLOCKQUOTE','PRE','UL','OL','TABLE','HR']);
  const escapeText = text => text.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/\\/g,'\\\\').replace(/([`*_~\[\]])/g,'\\$1').replace(/^( {0,3})([#>+-]|\d+[.)])(?=\s)/gm,'$1\\$2');
  function safeUrl(value, image=false) {
    const text=String(value || '').trim();if(!text)return '';
    if(image&&/^data:image\/(?:png|jpeg|gif|webp);base64,[a-z0-9+/=\s]+$/i.test(text))return text;
    try { const url=new URL(text,location.href);return (image?['http:','https:']:['http:','https:','mailto:','tel:']).includes(url.protocol)?text:''; } catch { return ''; }
  }
  function closing(text, marker, start) {
    let at=text.indexOf(marker,start);
    while(at>=0) {
      if(/^[*_~]+$/.test(marker)){let end=at;while(text[end]===marker[0])end++;const length=end-at;if(length>marker.length){if(marker.length===1){at=text.indexOf(marker,end);continue;}at=end-marker.length;}}
      let escapes=0;for(let i=at-1;i>=0&&text[i]==='\\';i--)escapes++;if(!(escapes%2))return at;at=text.indexOf(marker,at+marker.length);}
    return -1;
  }
  function bracketEnd(text, start, open, close) {
    let depth=1;for(let i=start+1;i<text.length;i++){if(text[i]==='\\'){i++;continue;}if(text[i]===open)depth++;if(text[i]===close&&!--depth)return i;}return -1;
  }
  function inline(parent, text, references=new Map(), depth=0, inTable=false) {
    if(depth>32){parent.append(document.createTextNode(text));return;}
    let plain='';const flush=()=>{if(plain){parent.append(document.createTextNode(plain));plain='';}};
    for(let i=0;i<text.length;) {
      if(text[i]==='\\'&&i+1<text.length&&/[!"#$%&'()*+,\-./:;<=>?@[\]\\^_`{|}~]/.test(text[i+1])){plain+=text[i+1];i+=2;continue;}
      if(text[i]==='`') {
        const marker=text.slice(i).match(/^`+/)[0],end=closing(text,marker,i+marker.length);
        if(end>=0){flush();let content=text.slice(i+marker.length,end).replace(/\n/g,' ');if(content.startsWith(' ')&&content.endsWith(' ')&&content.trim())content=content.slice(1,-1);if(inTable)content=content.replace(/\\\|/g,'|');parent.append(make('code',content));i=end+marker.length;continue;}
      }
      const image=text[i]==='!'&&text[i+1]==='[';
      if(text[i]==='['||image) {
        const start=i+(image?1:0),end=bracketEnd(text,start,'[',']');
        if(end>=0) {
          let destination=null,finish=end+1;
          if(text[finish]==='('){const close=bracketEnd(text,finish,'(',')');if(close>=0){destination=text.slice(finish+1,close).trim();finish=close+1;}}
          else if(text[finish]==='['){const close=bracketEnd(text,finish,'[',']');if(close>=0){destination=references.get((text.slice(finish+1,close)||text.slice(start+1,end)).toLowerCase());finish=close+1;}}
          else destination=references.get(text.slice(start+1,end).toLowerCase());
          if(destination!==null&&destination!==undefined) {
            const match=destination.match(/^(?:<([^>]+)>|(.*?))(?:\s+["'](.*)["'])?$/),url=safeUrl(match?.[1]||match?.[2]||destination,image);
            if(url){flush();const node=make(image?'img':'a');node.setAttribute(image?'src':'href',url);if(match?.[3])node.title=match[3];if(image)node.alt=text.slice(start+1,end).replace(/\\([\[\]\\])/g,'$1');else inline(node,text.slice(start+1,end),references,depth+1,inTable);parent.append(node);i=finish;continue;}
          }
        }
      }
      if(text[i]==='<') {
        const br=text.slice(i).match(/^<br\s*\/?>/i);if(br){flush();parent.append(make('br'));i+=br[0].length;continue;}
        const auto=text.slice(i).match(/^<((?:https?:\/\/|mailto:)[^<>\s]+)>/i);if(auto){flush();const link=make('a',auto[1].replace(/^mailto:/,''));link.href=auto[1];parent.append(link);i+=auto[0].length;continue;}
      }
      let matched=false;
      for(const [marker,tag] of [['***','strongem'],['___','strongem'],['**','strong'],['__','strong'],['~~','s'],['*','em'],['_','em']]) {
        if(!text.startsWith(marker,i)||/\s/.test(text[i+marker.length]||' '))continue;
        if(marker.includes('_')&&/[\p{L}\p{N}]/u.test(text[i-1]||''))continue;
        const end=closing(text,marker,i+marker.length);if(end<0||/\s/.test(text[end-1]))continue;
        flush();const node=make(tag==='strongem'?'strong':tag),target=tag==='strongem'?make('em'):node;if(target!==node)node.append(target);inline(target,text.slice(i+marker.length,end),references,depth+1,inTable);parent.append(node);i=end+marker.length;matched=true;break;
      }
      if(matched)continue;
      if(text[i]==='&') {
        const entity=text.slice(i).match(/^&(?:#(\d+)|#x([a-f\d]+)|(amp|lt|gt|quot|apos|nbsp));/i);
        if(entity){const code=entity[1]?Number(entity[1]):entity[2]?parseInt(entity[2],16):null;const named={amp:'&',lt:'<',gt:'>',quot:'"',apos:"'",nbsp:' '};if(code===null)plain+=named[entity[3].toLowerCase()];else if(code>0&&code<=0x10ffff)plain+=String.fromCodePoint(code);else plain+=entity[0];i+=entity[0].length;continue;}
      }
      if(text[i]==='\n'&&(/ {2}$/.test(plain)||/\\$/.test(plain))){plain=plain.replace(/ {2}$|\\$/,'');flush();parent.append(make('br'));i++;continue;}
      plain+=text[i++];
    }
    flush();
  }
  const listMatch=line=>line.match(/^( *)([-+*]|\d+[.)])\s+(.*)$/);
  const fenceMatch=line=>line.match(/^ {0,3}(`{3,}|~{3,})(.*)$/);
  const rule=line=>/^ {0,3}(?:(?:-\s*){3,}|(?:\*\s*){3,}|(?:_\s*){3,})$/.test(line);
  function cells(line) {
    const text=line.trim().replace(/^\|/,'').replace(/(?<!\\)\|$/,'');let item='',out=[];
    for(let i=0;i<text.length;i++){if(text[i]==='\\'&&i+1<text.length){item+=text[i]+text[++i];continue;}if(text[i]==='|'){out.push(item.trim());item='';}else item+=text[i];}out.push(item.trim());return out;
  }
  const tableRule=line=>line.includes('-')&&cells(line).every(cell=>/^:?-{3,}:?$/.test(cell));
  function parse(markdown, writable, references) {
    const fragment=document.createDocumentFragment();let lines=String(markdown).replace(/\r\n?/g,'\n').split('\n');
    if(!references){references=new Map();let fenced=null;lines=lines.filter(line=>{const fence=fenceMatch(line);if(fence){if(!fenced)fenced=fence[1][0];else if(fence[1][0]===fenced)fenced=null;return true;}if(fenced)return true;const match=line.match(/^ {0,3}\[([^\]]+)\]:\s*(.+)$/);if(match){references.set(match[1].toLowerCase(),match[2]);return false;}return true;});}
    const begins=(line,next)=>!!fenceMatch(line)||/^ {0,3}#{1,6}\s/.test(line)||rule(line)||/^ {0,3}>/.test(line)||!!listMatch(line)||(next&&tableRule(next));
    for(let i=0;i<lines.length;) {
      const line=lines[i];if(!line.trim()){i++;continue;}
      const fence=fenceMatch(line);
      if(fence){const body=[];i++;while(i<lines.length&&!new RegExp(`^ {0,3}${fence[1][0]==='`'?'`':'~'}{${fence[1].length},}\\s*$`).test(lines[i]))body.push(lines[i++]);if(i<lines.length)i++;const pre=make('pre'),code=make('code',body.join('\n'));pre.dataset.language=fence[2].trim();pre.append(code);fragment.append(pre);continue;}
      const heading=line.match(/^ {0,3}(#{1,6})\s+(.+?)\s*#*\s*$/);
      if(heading){const node=make(`h${heading[1].length}`);inline(node,heading[2],references);fragment.append(node);i++;continue;}
      if(lines[i+1]&&/^ {0,3}(?:=+|-+)\s*$/.test(lines[i+1])){const node=make(lines[i+1].trim()[0]==='='?'h1':'h2');inline(node,line.trim(),references);fragment.append(node);i+=2;continue;}
      if(rule(line)){fragment.append(make('hr'));i++;continue;}
      if(/^ {0,3}>/.test(line)){const body=[];while(i<lines.length&&/^ {0,3}>/.test(lines[i]))body.push(lines[i++].replace(/^ {0,3}> ?/,''));const quote=make('blockquote');quote.append(parse(body.join('\n'),writable,references));fragment.append(quote);continue;}
      const first=listMatch(line);
      if(first){const base=first[1].length,ordered=/\d/.test(first[2]),list=make(ordered?'ol':'ul');if(ordered)list.start=parseInt(first[2],10);
        while(i<lines.length){const match=listMatch(lines[i]);if(!match||match[1].length!==base||/\d/.test(match[2])!==ordered)break;const indent=base+match[2].length+1,body=[match[3]];i++;
          while(i<lines.length){const next=listMatch(lines[i]);if(next&&next[1].length===base)break;if(lines[i].trim()&&lines[i].match(/^ */)[0].length<=base)break;if(!lines[i].trim()){let at=i+1;while(at<lines.length&&!lines[at].trim())at++;if(at>=lines.length||(!listMatch(lines[at])&&lines[at].match(/^ */)[0].length<=base))break;}const continuation=lines[i++];body.push(continuation.slice(Math.min(indent,continuation.match(/^ */)[0].length)));}
          const li=make('li'),task=body[0].match(/^\[([ xX])\]\s+(.*)$/);if(task){li.dataset.task='true';const checkbox=make('input');checkbox.type='checkbox';checkbox.checked=task[1].toLowerCase()==='x';checkbox.disabled=!writable;checkbox.contentEditable='false';checkbox.setAttribute('aria-label','Task completed');li.append(checkbox);body[0]=task[2];}li.append(parse(body.join('\n'),writable,references));list.append(li);
          if(i<lines.length&&!lines[i].trim()){let at=i;while(at<lines.length&&!lines[at].trim())at++;const next=listMatch(lines[at]||'');if(next&&next[1].length===base&&/\d/.test(next[2])===ordered)i=at;else break;}
        }fragment.append(list);continue;}
      if(lines[i+1]&&tableRule(lines[i+1])){const headings=cells(line),align=cells(lines[i+1]),table=make('table'),head=make('thead'),row=make('tr'),body=make('tbody');for(let column=0;column<headings.length;column++){const cell=make('th');if(align[column]?.startsWith(':'))cell.dataset.align=align[column].endsWith(':')?'center':'left';else if(align[column]?.endsWith(':'))cell.dataset.align='right';inline(cell,headings[column],references,0,true);row.append(cell);}head.append(row);table.append(head,body);i+=2;while(i<lines.length&&lines[i].trim()&&lines[i].includes('|')){const values=cells(lines[i++]),row=make('tr');for(let column=0;column<headings.length;column++){const cell=make('td');inline(cell,values[column]||'',references,0,true);row.append(cell);}body.append(row);}fragment.append(table);continue;}
      if(/^(?: {4}|\t)/.test(line)){const body=[];while(i<lines.length&&(/^(?: {4}|\t)/.test(lines[i])||!lines[i].trim()))body.push(lines[i++].replace(/^(?: {4}|\t)/,''));while(!body.at(-1)&&body.length)body.pop();const pre=make('pre');pre.append(make('code',body.join('\n')));fragment.append(pre);continue;}
      const body=[line];i++;while(i<lines.length&&lines[i].trim()&&!begins(lines[i],lines[i+1]))body.push(lines[i++]);const paragraph=make('p');inline(paragraph,body.join('\n'),references);fragment.append(paragraph);
    }
    if(!fragment.childNodes.length){const paragraph=make('p');paragraph.append(make('br'));fragment.append(paragraph);}return fragment;
  }
  function inlineMarkdown(node, inTable=false) {
    if(node.nodeType===Node.TEXT_NODE)return escapeText(node.textContent.replace(/\u00a0/g,' '));
    if(node.nodeType!==Node.ELEMENT_NODE)return '';
    const text=()=>[...node.childNodes].map(child=>inlineMarkdown(child,inTable)).join('');
    switch(node.tagName){
      case 'BR':return inTable?'<br>':'  \n';
      case 'B':case 'STRONG':return `**${text()}**`;
      case 'I':case 'EM':return `*${text()}*`;
      case 'S':case 'DEL':case 'STRIKE':return `~~${text()}~~`;
      case 'CODE':{const raw=node.textContent,marker='`'.repeat(Math.max(1,...[...raw.matchAll(/`+/g)].map(m=>m[0].length+1))),pad=/^`|`$/.test(raw)||(/^ .* $/.test(raw)&&raw.trim())?' ':'';return `${marker}${pad}${raw}${pad}${marker}`;}
      case 'A':{const url=safeUrl(node.getAttribute('href'));return url?`[${text()}](<${url}>${node.title?` "${node.title.replace(/"/g,'\\"')}"`:''})`:text();}
      case 'IMG':{const url=safeUrl(node.getAttribute('src'),true);return url?`![${escapeText(node.alt||'')}](<${url}>${node.title?` "${node.title.replace(/"/g,'\\"')}"`:''})`:'';}
      case 'INPUT':return '';
      default:return text();
    }
  }
  function blocks(nodes) {
    const out=[];let pending='';const flush=()=>{if(pending.trim()){out.push(pending.trim());pending='';}};
    for(const node of nodes){if(node.nodeType===Node.ELEMENT_NODE&&blockTags.has(node.tagName)){flush();const content=blockMarkdown(node);if(content)out.push(content);}else pending+=inlineMarkdown(node);}flush();return out.join('\n\n');
  }
  function blockMarkdown(node) {
    const content=()=>[...node.childNodes].map(child=>inlineMarkdown(child)).join('').replace(/(?: {2}\n)+$/,'');
    if(/^H[1-6]$/.test(node.tagName))return `${'#'.repeat(Number(node.tagName[1]))} ${content()}`;
    if(node.tagName==='HR')return '---';
    if(node.tagName==='PRE'){const code=node.querySelector('code'),text=(code||node).textContent.replace(/\u00a0/g,' '),fence='`'.repeat(Math.max(3,...[...text.matchAll(/`+/g)].map(m=>m[0].length+1)));return `${fence}${node.dataset.language||''}\n${text}\n${fence}`;}
    if(node.tagName==='BLOCKQUOTE')return blocks(node.childNodes).split('\n').map(line=>line?`> ${line}`:'>').join('\n');
    if(node.tagName==='UL'||node.tagName==='OL'){let number=Number(node.getAttribute('start'))||1;return [...node.children].filter(li=>li.tagName==='LI').map(li=>{const marker=node.tagName==='OL'?`${number++}. `:'- ',checkbox=li.querySelector(':scope > input[type=checkbox]'),task=checkbox?`[${checkbox.checked?'x':' '}] `:'',text=blocks([...li.childNodes].filter(n=>n!==checkbox)),lines=text.split('\n');return marker+task+(lines[0]||'')+lines.slice(1).map(line=>'\n'+' '.repeat(marker.length)+line).join('');}).join('\n');}
    if(node.tagName==='TABLE'){const rows=[...node.rows];if(!rows.length)return '';const width=Math.max(...rows.map(row=>row.cells.length)),line=row=>'| '+Array.from({length:width},(_,i)=>row?.cells[i]?[...row.cells[i].childNodes].map(n=>inlineMarkdown(n,true)).join('').replace(/\|/g,'\\|'):'').join(' | ')+' |';const separators=Array.from({length:width},(_,i)=>{const align=rows[0].cells[i]?.dataset.align;return align==='center'?':---:':align==='right'?'---:':align==='left'?':---':'---';});return [line(rows[0]),'| '+separators.join(' | ')+' |',...rows.slice(1).map(line)].join('\n');}
    if([...node.children].some(child=>blockTags.has(child.tagName)))return blocks(node.childNodes);
    return content();
  }
  const icons={
    quote:'<path d="M4 10h5v8H3v-6c0-4 2-6 5-6M15 10h5v8h-6v-6c0-4 2-6 5-6"/>',
    ul:'<circle cx="4" cy="6" r="1"/><circle cx="4" cy="12" r="1"/><circle cx="4" cy="18" r="1"/><path d="M9 6h12M9 12h12M9 18h12"/>',
    ol:'<path d="M3 4h1v5M3 9h3M3 14c4-2 4 1 1 3l-1 2h3M10 6h11M10 12h11M10 18h11"/>',
    task:'<rect x="2" y="3" width="8" height="8" rx="1"/><path d="m4 7 2 2 3-4M14 7h8M3 16h19M3 21h19"/>',
    link:'<path d="m9 15 6-6M7 13l-2 2a4 4 0 0 0 6 6l3-3M17 11l2-2a4 4 0 0 0-6-6l-3 3"/>',
    code:'<path d="m8 5-6 7 6 7M16 5l6 7-6 7M14 3l-4 18"/>',
    block:'<rect x="2" y="3" width="20" height="18" rx="2"/><path d="m6 8 4 4-4 4M13 16h5"/>',
    hr:'<path d="M2 12h20"/>',
    table:'<rect x="2" y="3" width="20" height="18" rx="1"/><path d="M2 9h20M2 15h20M9 3v18M16 3v18"/>',
    image:'<rect x="2" y="3" width="20" height="18" rx="2"/><circle cx="7" cy="8" r="2"/><path d="m3 19 6-7 4 4 4-5 5 7"/>',
    undo:'<path d="m8 4-5 5 5 5M3 9h10a7 7 0 0 1 0 14"/>',
    redo:'<path d="m16 4 5 5-5 5M21 9H11a7 7 0 0 0 0 14"/>',
    row:'<rect x="2" y="2" width="20" height="20" rx="1"/><path d="M2 8h20M2 15h20M12 10v3M10.5 11.5h3"/>',
    column:'<rect x="2" y="2" width="20" height="20" rx="1"/><path d="M8 2v20M15 2v20M10 12h3M11.5 10.5v3"/>',
    remove:'<path d="M4 6h16M9 6V3h6v3M6 6l1 15h10l1-15M10 10v7M14 10v7"/>',
  };
  const icon=name=>`<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${icons[name]}</svg>`;
  async function create(host, markdown, writable, onChange) {
    const toolbar=make('div');toolbar.className='plm-markdown-toolbar';toolbar.setAttribute('role','toolbar');toolbar.setAttribute('aria-label','Markdown formatting');toolbar.hidden=!writable;
    const editor=make('div');editor.className='plm-markdown-content';editor.contentEditable=String(writable);editor.setAttribute('role',writable?'textbox':'document');editor.setAttribute('aria-label',writable?'Visual Markdown editor':'Markdown document');if(writable)editor.setAttribute('aria-multiline','true');
    editor.append(parse(markdown,writable));host.append(toolbar,editor);
    let baseline=blocks(editor.childNodes),range=null,dialog=null,destroyed=false;
    let history=[String(markdown)],historyIndex=0;
    const buttons=new Map();
    function changed(){if(destroyed)return;const next=blocks(editor.childNodes);if(next!==baseline){baseline=next;history.splice(historyIndex+1);history.push(next);historyIndex=history.length-1;onChange(next);}state();}
    function selection(){const current=getSelection();if(current?.rangeCount&&editor.contains(current.anchorNode)&&editor.contains(current.focusNode))range=current.getRangeAt(0).cloneRange();}
    function focus(){editor.focus({preventScroll:true});const current=getSelection();if(range&&editor.contains(range.commonAncestorContainer)){current.removeAllRanges();current.addRange(range);}else{range=document.createRange();range.selectNodeContents(editor);range.collapse(false);current.removeAllRanges();current.addRange(range);}}
    const anchor=()=>{const node=range?.startContainer;return node?.nodeType===Node.ELEMENT_NODE?node:node?.parentElement;};
    const inside=selector=>{const found=anchor()?.closest(selector);return found&&editor.contains(found)?found:null;};
    function command(name,value=null){if(!writable)return;
      if(name==='undo'||name==='redo'){const next=historyIndex+(name==='undo'?-1:1);if(next<0||next>=history.length)return;historyIndex=next;editor.replaceChildren(parse(history[next],writable));range=null;baseline=blocks(editor.childNodes);focus();onChange(history[next]);state();return;}
      focus();document.execCommand('styleWithCSS',false,false);document.execCommand(name,false,value);selection();changed();}
    function insert(node){const container=make('div');container.append(node);command('insertHTML',container.innerHTML);}
    function state(){
      for(const [name,button] of buttons){if(name==='undo')button.disabled=historyIndex===0;if(name==='redo')button.disabled=historyIndex===history.length-1;if(['bold','italic','strikeThrough'].includes(name))button.setAttribute('aria-pressed',String(document.queryCommandState(name)));if(['addRow','addColumn','removeRow','removeColumn'].includes(name))button.disabled=!inside('td,th');}
      const block=inside('h1,h2,h3,h4,h5,h6,p,pre');format.value=block&&/^H[1-6]$/.test(block.tagName)?block.tagName.toLowerCase():'p';
    }
    function button(name,label,action,graphic){const control=make('button');control.type='button';control.className='ghost';control.title=label;control.setAttribute('aria-label',label);control.dataset.command=name;if(graphic)control.innerHTML=icon(graphic);else{control.textContent=name==='bold'?'B':name==='italic'?'I':'S';control.classList.add(`format-${name}`);}control.addEventListener('pointerdown',event=>event.preventDefault());control.addEventListener('click',action);toolbar.append(control);buttons.set(name,control);return control;}
    const format=make('select');format.setAttribute('aria-label','Text style');for(const [value,label] of [['p','Paragraph'],...Array.from({length:6},(_,i)=>[`h${i+1}`,`Heading ${i+1}`])]){const option=make('option',label);option.value=value;format.append(option);}format.addEventListener('change',()=>command('formatBlock',format.value));toolbar.append(format);
    const gap=()=>{const separator=make('span');separator.className='plm-markdown-toolbar-separator';separator.setAttribute('aria-hidden','true');toolbar.append(separator);};
    button('bold','Bold',()=>command('bold'));button('italic','Italic',()=>command('italic'));button('strikeThrough','Strikethrough',()=>command('strikeThrough'));gap();
    button('quote','Block quote',()=>command('formatBlock',inside('blockquote')?'p':'blockquote'),'quote');button('ul','Bullet list',()=>command('insertUnorderedList'),'ul');button('ol','Numbered list',()=>command('insertOrderedList'),'ol');
    button('task','Task list',()=>{focus();if(!inside('li'))command('insertUnorderedList');const item=inside('li');if(!item)return;const checkbox=item.querySelector(':scope > input[type=checkbox]');if(checkbox){checkbox.remove();delete item.dataset.task;}else{const input=make('input');input.type='checkbox';input.contentEditable='false';input.setAttribute('aria-label','Task completed');item.dataset.task='true';item.prepend(input);}changed();},'task');gap();
    button('code','Inline code',()=>{focus();const code=inside('code');if(code&&!inside('pre')){code.replaceWith(...code.childNodes);changed();return;}const text=range?.toString()||'code';insert(make('code',text));},'code');
    button('block','Code block',()=>{focus();const pre=inside('pre');if(pre){const p=make('p',pre.textContent);pre.replaceWith(p);changed();return;}const node=make('pre');node.append(make('code',range?.toString()||''));insert(node);},'block');button('hr','Horizontal rule',()=>command('insertHorizontalRule'),'hr');gap();
    function showDialog(title,fields,submit,extra) {
      selection();dialog?.remove();dialog=make('dialog');dialog.className='plm-markdown-dialog';const form=make('form'),heading=make('h2',title),inputs={};form.append(heading);
      for(const field of fields){const label=make('label',field.label),input=make('input');input.type=field.type||'text';input.value=field.value??'';if(field.min)input.min=field.min;if(field.required)input.required=true;label.append(input);form.append(label);inputs[field.key]=input;}
      const menu=make('menu'),cancel=make('button','Cancel'),apply=make('button','Apply');cancel.type='button';cancel.className='ghost';cancel.onclick=()=>dialog.close();apply.type='submit';if(extra){const control=make('button',extra.label);control.type='button';control.className='ghost';control.onclick=()=>{dialog.close();extra.action();};menu.append(control);}menu.append(cancel,apply);form.append(menu);dialog.append(form);document.body.append(dialog);form.onsubmit=event=>{event.preventDefault();const values=Object.fromEntries(Object.entries(inputs).map(([key,input])=>[key,input.value]));dialog.close();submit(values);};dialog.onclose=()=>{dialog?.remove();dialog=null;};dialog.showModal();Object.values(inputs)[0]?.focus();
    }
    function linkDialog(){selection();const existing=inside('a');showDialog('Link',[{key:'url',label:'URL',value:existing?.getAttribute('href')||'',required:true},{key:'text',label:'Text',value:range?.toString()||existing?.textContent||''}],values=>{const url=safeUrl(values.url);if(!url)return;focus();if(existing){existing.href=url;if(values.text)existing.textContent=values.text;changed();}else if(range&&!range.collapsed&&values.text===range.toString())command('createLink',url);else{const link=make('a',values.text||url);link.href=url;insert(link);}},existing?{label:'Remove link',action:()=>{existing.replaceWith(...existing.childNodes);changed();}}:null);}
    button('link','Insert or edit link',linkDialog,'link');button('image','Insert image',()=>showDialog('Image',[{key:'url',label:'Image URL',required:true},{key:'alt',label:'Description'}],values=>{const url=safeUrl(values.url,true);if(!url)return;const image=make('img');image.src=url;image.alt=values.alt;insert(image);}), 'image');
    button('table','Insert table',()=>showDialog('Table',[{key:'rows',label:'Rows (including heading)',type:'number',min:'1',value:'3',required:true},{key:'columns',label:'Columns',type:'number',min:'1',value:'3',required:true}],values=>{const rows=Number(values.rows),columns=Number(values.columns);if(!Number.isInteger(rows)||!Number.isInteger(columns)||rows<1||columns<1)return;const table=make('table'),head=make('thead'),body=make('tbody');for(let r=0;r<rows;r++){const row=make('tr');for(let c=0;c<columns;c++){const cell=make(r?'td':'th');cell.append(r?make('br'):document.createTextNode(`Column ${c+1}`));row.append(cell);}(r?body:head).append(row);}table.append(head,body);const fragment=document.createDocumentFragment();fragment.append(table,make('p',''));insert(fragment);}), 'table');gap();
    button('addRow','Add table row',()=>{const cell=inside('td,th');if(!cell)return;const row=make('tr');for(let i=0;i<cell.parentElement.cells.length;i++){const next=make('td');next.append(make('br'));row.append(next);}const table=cell.closest('table');(table.tBodies[0]||table.appendChild(make('tbody'))).append(row);changed();},'row');
    button('addColumn','Add table column',()=>{const table=inside('table');if(!table)return;for(const row of table.rows){const cell=make(row.parentElement.tagName==='THEAD'?'th':'td');cell.append(make('br'));row.append(cell);}changed();},'column');
    button('removeRow','Remove table row',()=>{const row=inside('tr');if(!row)return;const table=row.closest('table');row.remove();if(!table.rows.length)table.remove();else if(!table.querySelector('th')){const head=make('thead'),next=table.rows[0];for(const cell of [...next.cells]){const th=make('th');th.append(...cell.childNodes);cell.replaceWith(th);}head.append(next);table.prepend(head);}changed();},'remove');
    button('removeColumn','Remove table column',()=>{const cell=inside('td,th');if(!cell)return;const index=cell.cellIndex,table=cell.closest('table');for(const row of table.rows)row.cells[index]?.remove();if(!table.rows[0]?.cells.length)table.remove();changed();},'remove');gap();button('undo','Undo',()=>command('undo'),'undo');button('redo','Redo',()=>command('redo'),'redo');
    function selectionChanged(){selection();if(editor.contains(getSelection()?.anchorNode))state();}
    document.addEventListener('selectionchange',selectionChanged);editor.addEventListener('input',changed);editor.addEventListener('change',changed);
    editor.addEventListener('click',event=>{if(event.target.closest('a')&&writable)event.preventDefault();});
    function pasted(html,text){
      if(!html)return parse(text,writable);
      const source=new DOMParser().parseFromString(html,'text/html');
      const allowed=new Set(['P','DIV','H1','H2','H3','H4','H5','H6','B','STRONG','I','EM','S','DEL','STRIKE','CODE','PRE','BLOCKQUOTE','UL','OL','LI','TABLE','THEAD','TBODY','TR','TH','TD','BR','HR','A','IMG']);
      function clean(node){if(node.nodeType===Node.TEXT_NODE)return document.createTextNode(node.textContent);if(node.nodeType!==Node.ELEMENT_NODE||['SCRIPT','STYLE','IFRAME','OBJECT','EMBED','FORM','SVG'].includes(node.tagName))return document.createDocumentFragment();const children=document.createDocumentFragment();for(const child of node.childNodes)children.append(clean(child));if(!allowed.has(node.tagName)){if(node.style?.fontWeight==='bold'||Number(node.style?.fontWeight)>=600){const strong=make('strong');strong.append(children);return strong;}if(node.style?.fontStyle==='italic'){const em=make('em');em.append(children);return em;}return children;}const copy=make(node.tagName.toLowerCase());if(node.tagName==='A'){const url=safeUrl(node.getAttribute('href'));if(!url)return children;copy.href=url;}if(node.tagName==='IMG'){const url=safeUrl(node.getAttribute('src'),true);if(!url)return children;copy.src=url;copy.alt=node.getAttribute('alt')||'';}if(node.tagName==='OL'&&node.start>1)copy.start=node.start;if(node.title)copy.title=node.title;copy.append(children);return copy;}
      const fragment=document.createDocumentFragment();for(const node of source.body.childNodes)fragment.append(clean(node));return fragment;
    }
    editor.addEventListener('paste',event=>{if(!writable)return;event.preventDefault();insert(pasted(event.clipboardData.getData('text/html'),event.clipboardData.getData('text/plain')));});
    editor.addEventListener('drop',event=>{if(!writable)return;event.preventDefault();const caret=document.caretRangeFromPoint?.(event.clientX,event.clientY);if(caret&&editor.contains(caret.commonAncestorContainer))range=caret;insert(pasted(event.dataTransfer.getData('text/html'),event.dataTransfer.getData('text/plain')));});
    editor.addEventListener('keydown',event=>{
      if((event.ctrlKey||event.metaKey)&&!event.altKey){const key=event.key.toLowerCase();if(key==='z'||key==='y'){event.preventDefault();command(key==='y'||event.shiftKey?'redo':'undo');return;}if(['b','i','k'].includes(key)){event.preventDefault();if(key==='k')linkDialog();else command(key==='b'?'bold':'italic');}}
      if(event.key==='Enter'&&inside('pre')){event.preventDefault();command('insertText','\n');}
      if(event.key==='Tab'&&inside('td,th')){event.preventDefault();const cell=inside('td,th'),table=cell.closest('table'),all=[...table.rows].flatMap(row=>[...row.cells]),next=all[all.indexOf(cell)+(event.shiftKey?-1:1)];if(next){range=document.createRange();range.selectNodeContents(next);range.collapse(true);focus();}}
    });
    state();
    function path(node){const result=[];while(node&&node!==editor){const parent=node.parentNode;if(!parent)return null;result.unshift([...parent.childNodes].indexOf(node));node=parent;}return node===editor?result:null;}
    return {
      getContent:()=>blocks(editor.childNodes),
      setContent(text){editor.replaceChildren(parse(text,writable));range=null;baseline=blocks(editor.childNodes);history=[String(text)];historyIndex=0;state();},
      getSelection:()=>{selection();return range?{start:path(range.startContainer),end:path(range.endContainer),startOffset:range.startOffset,endOffset:range.endOffset}:null;},
      setSelection(position){if(!position)return;const locate=steps=>steps?.reduce((node,index)=>node?.childNodes[index],editor);const start=locate(position.start),end=locate(position.end);if(!start||!end)return;try{range=document.createRange();range.setStart(start,Math.min(position.startOffset,start.nodeType===Node.TEXT_NODE?start.length:start.childNodes.length));range.setEnd(end,Math.min(position.endOffset,end.nodeType===Node.TEXT_NODE?end.length:end.childNodes.length));}catch{range=null;}},
      destroy(){destroyed=true;document.removeEventListener('selectionchange',selectionChanged);dialog?.close();dialog?.remove();host.replaceChildren();},
    };
  }
  return {create};
})();
