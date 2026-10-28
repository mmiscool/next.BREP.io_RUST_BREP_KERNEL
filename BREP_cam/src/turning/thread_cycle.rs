use crate::{CamError,TurnThreadCycle,Motion,MotionKind,Feed,FeedsSpeeds,Runner,Point3};
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}
/// Execute an explicit synchronized schedule. Start/end are nominal tool-tip
/// reference positions, not diameter coordinates. Infeed offsets shift each
/// helix axially; the caller provides sufficient run-in/run-out clearance.
pub fn expand(c:&TurnThreadCycle,feeds:&FeedsSpeeds,runner:&mut Runner<'_>)->Result<Vec<Motion>,CamError> {
    runner.token().check()?;feeds.validate().map_err(CamError::InvalidInput)?;
    if !c.start.is_finite() || !c.end.is_finite() || c.start.y!=0. || c.end.y!=0. || c.start.x!=c.end.x || c.end.z>=c.start.z || !c.pitch.is_finite() || c.pitch<=0. || !c.clearance_x.is_finite() || c.clearance_x<0. || c.depths.is_empty() || c.depths.len()!=c.infeed_offsets.len() || c.depths.len()+c.spring_passes as usize>100_000 {return Err(invalid("invalid turning thread cycle"));}
    let sign=if c.internal{1.}else{-1.};
    let mut previous=0.;
    for (&d,&z) in c.depths.iter().zip(&c.infeed_offsets) {
        if !d.is_finite() || d<=previous || !z.is_finite() {return Err(invalid("thread depths must increase and flank offsets must be finite"));}
        let x=c.start.x+sign*d;
        if x<0. || if c.internal {c.clearance_x>=x} else {c.clearance_x<=x} {return Err(invalid("thread clearance is not outside the cut"));}
        previous=d;
    }
    let mut moves=Vec::new();
    let feed=c.pitch*feeds.spindle_rpm;
    if !feed.is_finite() || feed<=0. {return Err(invalid("invalid synchronized thread feed"));}
    let total=c.depths.len()+c.spring_passes as usize;
    moves.push(Motion::rapid(Point3::new(c.clearance_x,0.,c.start.z)));
    for i in 0..total {
        runner.work(1,"turn-thread",i as u64,total as u64)?;
        let index=i.min(c.depths.len()-1);let x=c.start.x+sign*c.depths[index];let shift=c.infeed_offsets[index];
        let start=Point3::new(x,0.,c.start.z+shift);let end=Point3::new(x,0.,c.end.z+shift);
        moves.push(Motion::rapid(Point3::new(c.clearance_x,0.,start.z)).with_path(i as u32));
        moves.push(Motion::plunge(start,feeds.plunge_feed).with_path(i as u32));
        moves.push(Motion::new(MotionKind::ThreadCut{to:end,pitch:c.pitch},Feed::Rate(feed)).with_path(i as u32));
        moves.push(Motion::retract(Point3::new(c.clearance_x,0.,end.z),Feed::RAPID).with_path(i as u32));
    }
    moves.push(Motion::rapid(Point3::new(c.clearance_x,0.,c.start.z)));
    if moves.iter().any(|m|!m.kind.is_finite()) {return Err(invalid("thread expansion overflow"));}
    runner.token().check()?;Ok(moves)
}
