// Flowchart geometry shared by layout and orthogonal connection rendering.
'use strict';
globalThis.WorkflowFlowchart = (() => {
  const width=200,height=112,gap=100;
  const decision=kind=>kind==='decision'||kind==='decision-script';
  function shape(kind) {
    if(kind==='start'||kind==='end')return '<rect x="2" y="2" width="196" height="108" rx="54"/>';
    if(decision(kind))return '<polygon points="100,2 198,56 100,110 2,56"/>';
    if(kind==='form')return '<polygon points="24,2 198,2 176,110 2,110"/>';
    return '<rect x="2" y="2" width="196" height="108"/>';
  }
  function layout(nodes,edges) {
    const levels=new Map();
    for(let pass=0;pass<nodes.length;pass++)for(const n of nodes) {
      if(levels.has(n.id))continue;
      const incoming=edges.filter(e=>e.to===n.id);
      if(incoming.every(e=>levels.has(e.from)))levels.set(n.id,incoming.length?1+Math.max(...incoming.map(e=>levels.get(e.from))):0);
    }
    // Unconnected/cyclic draft steps remain visible and editable for correction.
    const groups=new Map();for(const n of nodes){const level=levels.get(n.id)??levels.size;const row=groups.get(level)||[];row.push(n);groups.set(level,row);}
    const columns=Math.max(1,...[...groups.values()].map(row=>row.length));
    for(const [level,row] of groups)row.forEach((n,i)=>{n.x=60+(Math.max(500,columns*(width+gap)-gap)-(row.length*(width+gap)-gap))/2+i*(width+gap);n.y=60+level*(height+gap);});
  }
  // Visibility grid routing: every segment is horizontal or vertical and avoids
  // step interiors. A bend penalty keeps paths short and easy to follow.
  function output(node,choices=[],outcome='always') {
    if(!decision(node.kind)||!choices.includes(outcome))return {x:node.x+width/2,y:node.y+height,dx:0,dy:1};
    const index=choices.indexOf(outcome),x=choices.length===1?width/2:index*width/(choices.length-1);
    return {x:node.x+x,y:node.y+height/2+height/2*(1-Math.abs(x-width/2)/(width/2)),dx:x<width/2?-1:x>width/2?1:0,dy:x===width/2?1:0};
  }
  function route(from,to,nodes,choices=[],outcome='always') {
    const start=output(from,choices,outcome),end={x:to.x+width/2,y:to.y};
    const a={x:start.dx<0?from.x-24:start.dx>0?from.x+width+24:start.x,y:start.dy?start.y+24:start.y},b={x:end.x,y:end.y-24};
    const boxes=nodes.map(n=>({left:n.x-8,right:n.x+width+8,top:n.y-8,bottom:n.y+height+8}));
    const xs=[...new Set([a.x,b.x,...boxes.flatMap(r=>[r.left-16,r.right+16])])].sort((a,b)=>a-b);
    const ys=[...new Set([a.y,b.y,...boxes.flatMap(r=>[r.top-16,r.bottom+16])])].sort((a,b)=>a-b);
    const clear=(p,q)=>!boxes.some(r=>p.x===q.x?p.x>r.left&&p.x<r.right&&Math.max(p.y,q.y)>r.top&&Math.min(p.y,q.y)<r.bottom:p.y>r.top&&p.y<r.bottom&&Math.max(p.x,q.x)>r.left&&Math.min(p.x,q.x)<r.right);
    const heap=[];
    const push=item=>{heap.push(item);let i=heap.length-1;while(i){const p=(i-1)>>1;if(heap[p].score<=item.score)break;heap[i]=heap[p];i=p;}heap[i]=item;};
    const pop=()=>{const top=heap[0],tail=heap.pop();if(heap.length){let i=0;while(i*2+1<heap.length){let j=i*2+1;if(j+1<heap.length&&heap[j+1].score<heap[j].score)j++;if(heap[j].score>=tail.score)break;heap[i]=heap[j];i=j;}heap[i]=tail;}return top;};
    const key=(x,y,dir)=>`${x},${y},${dir}`,best=new Map();
    const initial={xi:xs.indexOf(a.x),yi:ys.indexOf(a.y),dir:start.dy?'v':'h',cost:0,points:[start,a]};initial.score=Math.abs(a.x-b.x)+Math.abs(a.y-b.y);push(initial);
    let points;
    while(heap.length){const c=pop(),p={x:xs[c.xi],y:ys[c.yi]},k=key(c.xi,c.yi,c.dir);if(best.has(k)&&best.get(k)<c.cost)continue;
      if(p.x===b.x&&p.y===b.y){points=[...c.points,end];break;}
      for(const [dx,dy,dir] of [[-1,0,'h'],[1,0,'h'],[0,-1,'v'],[0,1,'v']]){const xi=c.xi+dx,yi=c.yi+dy;if(xi<0||yi<0||xi>=xs.length||yi>=ys.length)continue;const q={x:xs[xi],y:ys[yi]};if(!clear(p,q))continue;const cost=c.cost+Math.abs(p.x-q.x)+Math.abs(p.y-q.y)+(dir===c.dir?0:32),k=key(xi,yi,dir);if(best.has(k)&&best.get(k)<=cost)continue;best.set(k,cost);push({xi,yi,dir,cost,score:cost+Math.abs(q.x-b.x)+Math.abs(q.y-b.y),points:[...c.points,q]});}
    }
    // Overlapping draft nodes cannot have a collision-free route; keep it orthogonal.
    points??=[start,a,{x:b.x,y:a.y},b,end];
    const simplified=[];for(const p of points){const last=simplified.at(-1);if(last&&last.x===p.x&&last.y===p.y)continue;while(simplified.length>1){const x=simplified.at(-2),y=simplified.at(-1);if((x.x===y.x&&y.x===p.x)||(x.y===y.y&&y.y===p.y))simplified.pop();else break;}simplified.push(p);}
    let label=simplified[0],length=-1;for(let i=1;i<simplified.length;i++){const p=simplified[i-1],q=simplified[i],l=Math.abs(p.x-q.x)+Math.abs(p.y-q.y);if(l>length){length=l;label={x:(p.x+q.x)/2,y:(p.y+q.y)/2};}}
    return {d:simplified.map((p,i)=>`${i?'L':'M'} ${p.x} ${p.y}`).join(' '),label,points:simplified};
  }
  return {width,height,decision,shape,layout,output,route};
})();
