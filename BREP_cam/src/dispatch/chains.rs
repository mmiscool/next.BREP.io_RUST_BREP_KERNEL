//! Identity-based chain extraction. Geometry is supplied by the worker; mesh
//! silhouettes cannot recover persistent edges or sketch geometry.
use super::*;
use crate::{ChainSource, ChainSide, RigidTransform};
#[cfg(feature="kernel")]
use crate::planar::{machining::PlanarChain, Vec2};

/// Resolve the exact requested source in machine coordinates. Implementations
/// must preserve ordered curve selection and reject disconnected/nonplanar data.
pub trait ChainGeometry {
    fn resolve(&self, source: &ChainSource, transform: &RigidTransform, chord: f64, tol: &Tolerance) -> Result<PlanarInput, CamError>;
}

pub(super) fn selected(input: &GenerateInput<'_>, context: &GenerateContext<'_>) -> Result<PlanarInput, CamError> {
    if input.operation.targets.chains.is_empty() { return planar_input(input); }
    let resolver=context.chain_geometry.ok_or_else(||refused(input,"selected chains require resolved edge/sketch geometry in GenerateContext"))?;
    let mut out=PlanarInput::default();
    for selection in &input.operation.targets.chains {
        selection.validate().map_err(invalid)?;
        let supported=match (&input.operation.strategy,selection.side) {
            (_,ChainSide::Auto)=>true,
            (Strategy::Contour2D(p),side)=>if selection.closed {side==if p.outside {ChainSide::Outside}else{ChainSide::Inside}} else {side==if p.outside {ChainSide::Right}else{ChainSide::Left}},
            (Strategy::Trace(p),side)=>side==match p.side {-1=>ChainSide::Left,1=>ChainSide::Right,_=>ChainSide::On},
            (Strategy::Slot(_)|Strategy::Engrave(_),ChainSide::On)=>true,
            (Strategy::Chamfer2D(_),side)=>side==if selection.closed {ChainSide::Outside}else{ChainSide::Right},
            _=>false,
        };
        if !supported {return Err(refused(input,&format!("chain side {:?} is unsupported by the strategy's compensation settings",selection.side)));}
        if selection.offset!=0. {return Err(refused(input,"chain offset is not supported"));}
        if selection.start.is_some() {return Err(refused(input,"chain start/lead placement is not supported"));}
        let mut resolved=resolver.resolve(&selection.source,&input.setup.model_to_machine,input.operation.tolerance,&input.tolerance)?;
        if resolved.chains.is_empty() {return Err(refused(input,"selected chain source contains no curves"));}
        if matches!(input.operation.strategy,Strategy::Contour2D(_)|Strategy::Chamfer2D(_)) && resolved.regions.iter().any(|r|r.region.loops.len()!=1) {return Err(refused(input,"profile islands require per-loop side compensation, which is not supported"));}
        for chain in &mut resolved.chains {
            if chain.chain.closed!=selection.closed {return Err(refused(input,"selected chain closure does not match geometry"));}
            if chain.chain.segments.is_empty() || !chain.z.is_finite() || chain.chain.segments.iter().any(|s|!s.start.is_finite() || !s.end.is_finite()) {return Err(refused(input,"selected chain is empty or non-finite"));}
            if selection.reversed {chain.chain=chain.chain.reversed();}
        }
        out.chains.extend(resolved.chains);
        out.regions.extend(resolved.regions);
    }
    let all:Vec<u32>=(0..input.mesh.triangle_count() as u32).collect();
    out.stock_top=if all.is_empty() {None}else{Some(input.mesh.bounds_of(&all).max.z)};
    Ok(out)
}

/// Kernel-backed geometry named by document identities, in model coordinates.
/// The caller supplies all selected objects; missing identities fail explicitly.
#[cfg(feature="kernel")]
pub struct KernelChainGeometry<'a> {
    pub solids: &'a [(&'a str, &'a brep_kernel::BrepSolid)],
    pub curves: &'a [(&'a str, &'a str, &'a brep_kernel::NurbsCurve)],
    pub profiles: &'a [(&'a str, &'a brep_kernel::SketchProfile)],
}

#[cfg(feature="kernel")]
impl ChainGeometry for KernelChainGeometry<'_> {
    fn resolve(&self, source:&ChainSource, transform:&RigidTransform, chord:f64, tol:&Tolerance)->Result<PlanarInput,CamError> {
        use crate::kernel_adapter::planar::{edge_chains,curve_segments,sketch_regions};
        source.validate().map_err(invalid)?;
        transform.validate().map_err(|e|invalid(e.to_string()))?;
        if !chord.is_finite() || chord<=0. {return Err(invalid("chain chord tolerance must be positive"));}
        let mut pieces=Vec::new();
        match source {
            ChainSource::Edges {edges}=>for edge in edges {
                let solid=self.solids.iter().find(|(id,_)|*id==edge.solid).ok_or_else(||invalid(format!("chain body {:?} is unresolved",edge.solid)))?.1;
                let mut extracted=edge_chains(solid,&[edge.edge.as_str()],transform,chord,tol)?;
                if extracted.len()!=1 {return Err(invalid("selected edge must resolve to one connected chain"));}
                pieces.push(extracted.remove(0));
            },
            ChainSource::SketchCurves {sketch,curves}=>for id in curves {
                let curve=self.curves.iter().find(|(s,c,_)|*s==sketch && *c==id).ok_or_else(||invalid(format!("sketch curve {sketch}:G{id} is unresolved")))?.2;
                let [a,b]=curve.domain().map_err(invalid)?;
                let (segments,z)=curve_segments(curve,a,b,true,transform,chord,tol)?;
                if z[1]-z[0]>tol.distance_eps {return Err(invalid("sketch curve is not horizontal planar geometry"));}
                pieces.push(PlanarChain {chain:crate::planar::Chain::open(segments),z:(z[0]+z[1])*0.5});
            },
            ChainSource::SketchProfile {sketch,regions}=>{
                let profile=self.profiles.iter().find(|(id,_)|*id==sketch).ok_or_else(||invalid(format!("sketch profile {sketch} is unresolved")))?.1;
                let extracted=sketch_regions(profile,transform,chord,tol)?;
                let indices:Vec<usize>=if regions.is_empty(){(0..extracted.len()).collect()}else{regions.iter().map(|i|*i as usize).collect()};
                let mut out=PlanarInput::default();
                for i in indices {
                    let region=extracted.get(i).ok_or_else(||invalid(format!("sketch profile {sketch} region {i} is unresolved")))?.clone();
                    out.chains.extend(region.region.loops.iter().map(|l|PlanarChain {chain:crate::planar::Chain::from_loop(l),z:region.z}));
                    out.regions.push(region);
                }
                return Ok(out);
            },
        }
        let chain=join_ordered(&pieces,tol)?;
        Ok(PlanarInput {chains:vec![chain],..Default::default()})
    }
}

#[cfg(feature="kernel")]
fn join_ordered(pieces:&[PlanarChain],tol:&Tolerance)->Result<PlanarChain,CamError> {
    let first=pieces.first().ok_or_else(||invalid("chain has no curves"))?;
    if pieces.iter().any(|p|(p.z-first.z).abs()>tol.distance_eps || p.chain.segments.is_empty()) {return Err(invalid("chain curves must share one horizontal plane"));}
    // Preserve selection order; the first edge may be intrinsically backwards.
    for reverse in [false,true] {
        let mut chain=if reverse {first.chain.reversed()}else{first.chain.clone()};
        let mut connected=true;
        for next in &pieces[1..] {
            let end=chain.end().unwrap();
            let next=if end.distance(next.chain.start().unwrap())<=tol.point_eps {next.chain.clone()}else if end.distance(next.chain.end().unwrap())<=tol.point_eps {next.chain.reversed()}else{connected=false;break};
            chain.segments.extend(next.segments);
        }
        if connected {
            chain.closed=chain.start().unwrap().distance(chain.end().unwrap())<=tol.point_eps;
            return Ok(PlanarChain {chain,z:first.z});
        }
    }
    Err(invalid("selected chain curves are disconnected or not in chain order"))
}

