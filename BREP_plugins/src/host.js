'use strict';
// Closure-private callback registry and branded handles never serialize as resident IDs.
const __pluginHost = (() => {
 const nativeGeometry=globalThis.__geometry;delete globalThis.__geometry;
 const NativeWeakMap=WeakMap, hasHandle=Function.prototype.call.bind(WeakMap.prototype.has), getHandle=Function.prototype.call.bind(WeakMap.prototype.get), setHandle=Function.prototype.call.bind(WeakMap.prototype.set);
 const features=[], actions=[], workbenches=[], panels=[], annotations=[];
 const callbacks=new Map(); let sealed=false;
 const jsonSafe=(v,seen=new Set())=>{if(typeof v==='number'&&!Number.isFinite(v))throw Error('nonfinite JSON value');if(typeof v==='function'||typeof v==='symbol'||typeof v==='bigint'||v===undefined)throw Error('non-JSON value');if(v&&typeof v==='object'){if(seen.has(v))throw Error('cyclic JSON value');seen.add(v);for(const x of Object.values(v))jsonSafe(x,seen);seen.delete(v);}return v;};
 const sync=v=>{if(v && (typeof v.then==='function'))throw Error('Promise/thenable results unsupported');return v;};
 const register=(kind,list,d,fn)=>{
  if(sealed)throw Error('registration is closed');
  if(!d||typeof d!=='object')throw Error('registration must be an object');
  if(fn&&typeof d[fn]!=='function')throw Error('missing '+fn+' callback');
  if(callbacks.has(d.id))throw Error('duplicate registration '+d.id);
  callbacks.set(d.id,{kind,fn:d[fn]});
  const metadata={};for(const k of Object.keys(d)){if(k!==fn){if(typeof d[k]==='function')throw Error('unsupported callback '+k);metadata[k]=d[k];}}
  list.push(metadata);
 };
 const app=Object.freeze({registerFeature:d=>register('feature',features,d,'execute'),registerAction:d=>register('action',actions,d,'run'),registerWorkbench:d=>register('workbench',workbenches,d),registerPanel:d=>register('panel',panels,d),registerAnnotation:d=>register('annotation',annotations,d,'execute'),addSidePanel:d=>register('panel',panels,d),addToolbarButton:d=>register('action',actions,d,'run')});
 const metadata=()=>jsonSafe({features,actions,workbenches,panels,annotations});
 const invoke=(kind,id,input)=>{
  const def=callbacks.get(id);if(!def||def.kind!==kind)throw Error('registration unavailable '+id);
  if(kind==='annotation') {
   const primitive=(op,p)=>nativeGeometry('annotation.'+op,[jsonSafe(p)]);
   const annotation=Object.freeze({note:p=>primitive('note',p),leader:p=>primitive('leader',p),linear:p=>primitive('linear',p)});
   const ctx=Object.freeze({...input,annotation});
   return jsonSafe(sync(def.fn(ctx)));
  }
  const handles=new NativeWeakMap();
  const wrap=token=>{const h=Object.freeze({toJSON(){throw Error('geometry handles cannot be serialized');}});setHandle(handles,h,token);return h;};
  const token=h=>{if(!hasHandle(handles,h))throw Error('stale or foreign geometry handle');return getHandle(handles,h);};
  const call=(op,...args)=>nativeGeometry(op,args);
  const geometry=Object.freeze({sphere:p=>wrap(call('sphere',p)),cube:p=>wrap(call('cube',p)),cylinder:p=>wrap(call('cylinder',p)),reference:n=>wrap(call('reference',n)),transform:(h,p)=>wrap(call('transform',token(h),p)),boolean:(op,a,b)=>wrap(call('boolean',op,token(a),token(b))),query:h=>call('query',token(h))});
  const commands=[],notifications=[];
  const stage=c=>{if(commands.length>=256)throw Error('too many action commands');commands.push(c);};
  const document=Object.freeze({...input.document,addFeature:(featureType,inputParams={},persistentData={})=>stage({op:'addFeature',featureType,inputParams,persistentData}),updateFeature:(id,inputParams)=>stage({op:'updateFeature',id,inputParams}),deleteFeature:id=>stage({op:'deleteFeature',id})});
  const ctx=Object.freeze({...input,params:input.params||input,geometry,document,notify:message=>notifications.push(String(message))});
  const result=sync(def.fn(ctx));
  if(kind==='action')return jsonSafe({commands,notifications});
  if(!result||!result.outputs||typeof result.outputs!=='object')throw Error('feature requires outputs');
  const outputs={};for(const [key,h] of Object.entries(result.outputs))outputs[key]=token(h);
  // JSON stringify here rejects handles anywhere in persistentData.
  return {outputs,persistentData:JSON.parse(JSON.stringify(jsonSafe(result.persistentData??{})))};
 };
 return Object.freeze({app,metadata,seal(){sealed=true;},sync,invoke});
})();
// No ambient time or randomness in package initialization or callback execution.
globalThis.Date=undefined;globalThis.performance=undefined;Math.random=()=>{throw Error('ambient randomness unsupported');};
