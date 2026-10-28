//! Bounded scalar schedules; no generic planar offsets.
use crate::{CamError,Point3,Setup,StockCylinder};
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}
pub fn face_levels(front:f64,target:f64,step:f64,allowance:f64,finish:bool,budget:usize)->Result<Vec<f64>,CamError> {
    if ![front,target,step,allowance].iter().all(|v|v.is_finite()) || step<=0. || allowance<0. || target>front || target+allowance>front {return Err(invalid("invalid turning face depth schedule"));}
    let floor=target+allowance;let count=((front-floor)/step).ceil();
    if count>budget as f64 || budget==0 {return Err(invalid("turning face pass budget exceeded"));}
    let mut values:Vec<_>=(1..=count as usize).map(|i|(front-i as f64*step).max(floor)).collect();
    if finish && values.last().map_or(true,|v|*v!=target) {if values.len()>=budget{return Err(invalid("turning face pass budget exceeded"));}values.push(target);}
    Ok(values)
}
pub fn groove_centres(width:f64,blade:f64,step:f64,budget:usize)->Result<Vec<f64>,CamError> {
    if ![width,blade,step].iter().all(|v|v.is_finite()) || blade<=0. || width<blade || step<=0. || step>blade {return Err(invalid("groove must fit blade and stepover must overlap its width"));}
    let span=width-blade;let count=(span/step).ceil();
    if count+1.>budget as f64 {return Err(invalid("groove plunge budget exceeded"));}
    Ok((0..=count as usize).map(|i|-span/2.+(i as f64*step).min(span)).collect())
}
/// Transfer the SAME shared cylindrical stock through setup transforms. This
/// preserves its physical volume; it does not claim simulated rest-stock removal.
pub fn transfer_stock(stock:&StockCylinder,from:&Setup,to:&Setup)->Result<StockCylinder,CamError> {
    from.model_to_machine.validate()?;to.model_to_machine.validate()?;
    if !stock.base.is_finite() || !stock.axis.is_finite() || (stock.axis.length()-1.).abs()>1e-9 || !stock.radius.is_finite() || stock.radius<=0. || !stock.height.is_finite() || stock.height<=0. {return Err(invalid("invalid transferable bar stock"));}
    let model=from.machine_to_model();
    let mut base=to.model_to_machine.apply_point(model.apply_point(stock.base));
    let mut axis=to.model_to_machine.apply_vector(model.apply_vector(stock.axis));
    if (axis.z.abs()-1.).abs()>1e-9 || base.x.hypot(base.y)>1e-8 {return Err(invalid("transferred bar is not coaxial with new spindle"));}
    if axis.z<0. {base=base.add(axis.scale(stock.height));}
    // Eliminate transform round-off on the certified coaxial coordinates.
    base.x=0.;base.y=0.;axis=Point3::Z;
    Ok(StockCylinder{base,axis,radius:stock.radius,height:stock.height})
}
