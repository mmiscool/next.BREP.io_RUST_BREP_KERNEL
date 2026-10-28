//! One operation-to-linker adapter for all 3D finishing strategies.
//! Relinking preserves the certified cutting polyline. Filtering and arc
//! fitting remain explicit subsequent operations, since they change its points.
use crate::*;
use crate::link::{LinkContext,LinkOptions};

// The holder of the tool of the operation being generated, scoped to one
// `dispatch` call like the pre-operation stock (`link::with_pre_op_stock`),
// so the finishing strategies between dispatch and this adapter do not carry
// it. Unset outside dispatch: the stock certification below then checks the
// shank only and says so in the report. Thread-local for the same reason as
// the stock scope: generation links on the calling thread.
thread_local! {
    static TOOL_HOLDER: std::cell::Cell<Option<*const Holder>> = const { std::cell::Cell::new(None) };
}
const NO_HOLDER: *const Holder = std::ptr::null();

/// Run `f` with `holder` as the holder of the operation's tool (`None`: the
/// tool has no holder); the previous value is restored afterwards (also on unwind).
pub fn with_tool_holder<R>(holder: Option<&Holder>, f: impl FnOnce() -> R) -> R {
    struct Restore(Option<*const Holder>);
    impl Drop for Restore {fn drop(&mut self) {TOOL_HOLDER.with(|c| c.set(self.0));}}
    let previous=TOOL_HOLDER.with(|c| c.replace(Some(holder.map_or(NO_HOLDER,|h| h as *const _))));
    let _restore=Restore(previous);
    f()
}

/// The scoped holder: `None` outside a `with_tool_holder` scope, `Some(None)`
/// when the tool has no holder.
fn with_current_tool_holder<R>(f: impl for<'h> FnOnce(Option<Option<&'h Holder>>) -> R) -> R {
    let p=TOOL_HOLDER.with(|c| c.get());
    // SAFETY: a non-null pointer is set only while a `with_tool_holder` frame
    // on THIS thread's stack holds the `&Holder` it came from (restored when
    // that frame ends, also on unwind); `f` runs synchronously inside it and
    // the higher-ranked lifetime keeps the reference from escaping.
    f(p.map(|p| if p.is_null() {None} else {Some(unsafe {&*p})}))
}

/// A run of cut motions of the split path (`split_at_stock`) that the shank
/// or the holder may not take through the stock: the shape of lane C4's
/// `link::BlockedSpan` (motion range, reason, the highest material that may
/// stand there, the blocking radius).
#[derive(Clone,Debug,PartialEq)]
pub struct StockBlock {
    pub motions:std::ops::Range<usize>,
    /// The holder (true) or the shank (false) meets the material.
    pub holder:bool,
    pub material_top:f64,
    pub reach:f64,
    pub from:Point3,
    pub to:Point3,
}

/// The material of `stock` above the part. A column's top is the material
/// at its centre (where the replay sampled it), so it is compared with the
/// part's height at that same point: kept when it exceeds it by more than
/// `margin`, and only when it is above `floor` (lower material cannot reach
/// the non-cutting parts of the tool). Comparing with the part's lowest
/// height over the whole cell instead (lane C16's first rule) read a wall
/// column as stock wherever its cell also held floor, at any cell size: 94 of
/// the plate finish's 103 blocks (C7's measurement, 2026-10-20). Which cells a
/// shank or holder reaches stays the conservative capsule test.
fn stock_above_part(stock:&StockEnvelope,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,floor:f64,margin:f64,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<StockEnvelope,CamError> {
    let probe=Cutter::ball(2.0*tolerance.distance_eps.max(1e-4),1.0,1.0);
    let drop=crate::drop::DropCutter::new(mesh,&probe,index,tolerance)?;
    let below=mesh.bounds.min.z-1.0;
    let mut failure=None;
    let kept=stock.filtered(|top,c|{
        if failure.is_some() || top<=floor {return false;}
        let q=c[0].lerp(c[2],0.5);
        match drop.point(q.x,q.y,below,runner) {
            Ok(r) => top-r.point.z>margin,
            Err(e) => {failure=Some(e);false}
        }
    });
    match failure {Some(e)=>Err(e),None=>Ok(kept)}
}

/// Certify every cut chord's non-cutting parts (shank, and holder when the
/// tool has one) against the stock left above the part. Each blocked chord
/// is split into straight cuts: its longest clear prefix and suffix (found by
/// bisection to 0.01 along the chord) and the blocked middle; the blocked
/// cuts are returned as runs of motion indices of the split path. A
/// sub-chord of a part-certified straight chord is certified against the
/// part too, so the split changes no certificate.
pub fn split_at_stock(path:&Toolpath,cutter:&Cutter,holder:Option<&Holder>,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,stock:&StockEnvelope,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<(Toolpath,Vec<StockBlock>),CamError> {
    let profile=crate::cutter::ToolProfile::new(cutter,holder,crate::cutter::DEFAULT_CHORD);
    // Bands above the flutes: (height above the tip, radius, part). The
    // drop cutter already certified the shank against the part, so the shank
    // is checked against the stock ABOVE the part; nothing certified the
    // holder against the part, so it is checked against all the material.
    use crate::cutter::ToolPart;
    let zones:Vec<(f64,f64,ToolPart)>=profile.points.windows(2).filter(|w|w[0].part!=ToolPart::Cutting).map(|w|(w[0].height.min(w[1].height),w[0].radius.max(w[1].radius),w[0].part)).collect();
    let mut lowest_cut=f64::INFINITY;
    for m in &path.motions {if let MotionKind::Cut {to}=m.kind {lowest_cut=lowest_cut.min(to.z);}}
    if zones.is_empty() || !lowest_cut.is_finite() {return Ok((path.clone(),Vec::new()));}
    let eps=tolerance.distance_eps;
    let lowest_shank=zones.iter().filter(|z|z.2==ToolPart::Shank).map(|z|z.0).reduce(f64::min);
    let above=match lowest_shank {Some(h)=>Some(stock_above_part(stock,mesh,index,lowest_cut+h-eps,eps.max(1e-6),tolerance,runner)?),None=>None};
    // The worst band meeting material along a-b: (holder, material top, radius).
    let blocking=|a:Point3,b:Point3|->Option<(bool,f64,f64)> {
        let base=a.z.min(b.z);
        let mut worst:Option<(bool,f64,f64)>=None;
        for &(bottom,radius,part) in &zones {
            let material=if part==ToolPart::Holder {Some(stock)} else {above.as_ref()};
            if let Some(top)=material.and_then(|m|m.top_over_segment(a,b,radius)) {
                if top>base+bottom+eps && worst.is_none_or(|w|top>w.1 || (top==w.1 && radius>w.2)) {worst=Some((part==ToolPart::Holder,top,radius));}
            }
        }
        worst
    };
    let blocked=|a:Point3,b:Point3|blocking(a,b).is_some();
    let mut out=path.clone();out.motions.clear();
    let mut blocks:Vec<StockBlock>=Vec::new();
    let (mut at,mut cut_index)=(None::<Point3>,0u64);
    for m in &path.motions {
        let MotionKind::Cut {to}=m.kind else {out.motions.push(m.clone());if let Some(e)=m.end() {at=Some(e);}continue};
        let a=at.ok_or_else(||CamError::InvalidInput("finishing cut has no start position".into()))?;
        at=Some(to);
        runner.work(1,"finish-stock",cut_index,0)?;cut_index+=1;
        let Some((holder_hit,top,reach))=blocking(a,to) else {out.motions.push(m.clone());continue};
        let length=a.distance(to).max(1e-12);
        let steps=((length/0.01).log2().ceil().max(0.0) as usize).min(40);
        let p=|t:f64|a.lerp(to,t);
        let t0=if blocked(a,a) {0.0} else {let (mut lo,mut hi)=(0.0,1.0);for _ in 0..steps {let mid=0.5*(lo+hi);if blocked(a,p(mid)) {hi=mid;} else {lo=mid;}}lo};
        let t1=if blocked(to,to) {1.0} else {let (mut lo,mut hi)=(t0,1.0);for _ in 0..steps {let mid=0.5*(lo+hi);if blocked(p(mid),to) {lo=mid;} else {hi=mid;}}hi};
        let cut=|q:Point3|{let mut c=m.clone();c.kind=MotionKind::Cut {to:q};c};
        if t0>0.0 {out.motions.push(cut(p(t0)));}
        let k=out.motions.len();out.motions.push(cut(p(t1)));
        // A blocked cut right after another extends that run.
        match blocks.last_mut() {
            Some(last) if last.motions.end==k => {last.motions.end=k+1;last.to=p(t1);last.holder|=holder_hit;last.material_top=last.material_top.max(top);last.reach=last.reach.max(reach);}
            _ => blocks.push(StockBlock {motions:k..k+1,holder:holder_hit,material_top:top,reach:reach.max(cutter.radius()),from:p(t0),to:p(t1)}),
        }
        if t1<1.0 {out.motions.push(m.clone());}
    }
    Ok((out,blocks))
}


pub fn finish(path:&Toolpath,operation:&Operation,cutter:&Cutter,mesh:&MeshSnapshot,index:Option<&dyn TriangleIndex>,heights:&Heights,feeds:&FeedsSpeeds,tolerance:Tolerance,runner:&mut Runner<'_>)->Result<Toolpath,CamError> {
    let heights=operation.heights.as_ref().unwrap_or(heights);
    let feeds=operation.feeds.as_ref().unwrap_or(feeds);
    // Finishing cuts never take the shank or the holder through stock that
    // earlier operations left (integrator 2026-10-20, the plate's OP5): the
    // spans split_at_stock proves blocked are removed by link::trim_blocked,
    // which relinks the gaps above the material it leaves (lane C4; one call,
    // no second relink: its keep-outs live only inside it).
    let (mut out,kept,holder_known,known_stock)=crate::link::with_current_pre_op_stock(|stock|->Result<(Toolpath,Toolpath,bool,bool),CamError>{
        let context=LinkContext {mesh,cutter,index,tolerance,heights,feeds,params:&operation.linking,
            options:LinkOptions {radial_allowance:operation.stock_to_leave.radial,axial_allowance:operation.stock_to_leave.axial,chord_tolerance:operation.tolerance*0.25,reorder:false,allow_reverse:false,material_top:heights.top,..Default::default()},stock};
        let Some(envelope)=stock else {return Ok((crate::link::finish(path,&context,0.,None,runner)?,path.clone(),true,false))};
        let (split,blocks,holder_known)=with_current_tool_holder(|holder|->Result<_,CamError>{
            let (split,blocks)=split_at_stock(path,cutter,holder.flatten(),mesh,index,envelope,tolerance,runner)?;
            Ok((split,blocks,holder.is_some()))
        })?;
        if blocks.is_empty() {return Ok((crate::link::finish(path,&context,0.,None,runner)?,path.clone(),holder_known,true));}
        if blocks.iter().map(|b|b.motions.len()).sum::<usize>()==split.motions.iter().filter(|m|matches!(m.kind,MotionKind::Cut {..})).count() {
            return Err(CamError::InvalidInput(format!("finishing: every cut is blocked by stock left by earlier operations at the shank or holder ({} spans, first at {:?})",blocks.len(),blocks.first().map(|s|s.from))));
        }
        let spans:Vec<crate::link::BlockedSpan>=blocks.iter().map(|b|crate::link::BlockedSpan {motions:b.motions.clone(),reason:if b.holder {crate::link::TrimReason::Holder} else {crate::link::TrimReason::Unreachable},material_top:Some(b.material_top),reach:b.reach}).collect();
        let trimmed=crate::link::trim_blocked(&split,&spans,&context,runner)?;
        // The cuts the linker keeps are the split path's minus the trimmed ones.
        let mut kept=split.clone();
        let blocked:std::collections::BTreeSet<usize>=blocks.iter().flat_map(|b|b.motions.clone()).collect();
        kept.motions=split.motions.iter().enumerate().map(|(i,m)|{
            let mut m=m.clone();
            if blocked.contains(&i) {m.kind=MotionKind::Rapid {to:m.end().expect("a cut ends somewhere")};}
            m
        }).collect();
        Ok((trimmed.path,kept,holder_known,true))
    })?;
    let path=&kept;
    if !holder_known {
        out.report.warn(Diagnostic::new("holder-unknown","generated outside dispatch: the tool's holder is not known here, so only the shank was certified against the pre-operation stock"));
    }
    if heights.top.is_none() && !known_stock {
        out.report.warn(Diagnostic::new("link-stock-unknown", "the stock before this finishing operation is unknown: links are proven against the part only (set the operation's top height, or generate with the pre-operation stock)"));
    }
    // Preserve source cutting metadata too; the shared linker owns only the
    // newly introduced approach, entry, link, lead and retract motions.
    let mut previous=None;
    let mut source=path.motions.iter().filter_map(|m|{let start=previous;previous=m.end();if matches!(m.kind,MotionKind::Cut {..}) {Some((start,m))} else {None}});
    let mut previous=None;
    for m in &mut out.motions {let start=previous;previous=m.end();if matches!(m.kind,MotionKind::Cut {..}) {
        let (original_start,original)=source.next().ok_or_else(||CamError::InvalidInput("strategy linking changed the cut stream".into()))?;
        if start!=original_start || m.end()!=original.end() {return Err(CamError::InvalidInput("strategy linking changed a cut start or endpoint".into()));}*m=original.clone();
    }}
    if source.next().is_some() {return Err(CamError::InvalidInput("strategy linking omitted cut motions".into()));}
    Ok(out)
}

