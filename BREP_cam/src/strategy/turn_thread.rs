//! Constant-pitch threading schedules. The requested total radial depth is
//! explicit; these functions do not infer an ISO thread depth from pitch alone.
use crate::CamError;
fn invalid(s:&str)->CamError {CamError::InvalidInput(s.into())}

/// For a sharp V profile, removed area is proportional to squared radial
/// depth, hence equal-area passes have d_i = D sqrt(i/N). Constant-depth mode
/// has d_i = D i/N. Nose/truncation-specific area correction is not implied.
pub fn pass_depths(total:f64,passes:u32,constant_area:bool)->Result<Vec<f64>,CamError> {
    if !total.is_finite() || total<=0. || passes==0 || passes>100_000 {return Err(invalid("invalid turning thread depth/pass budget"));}
    let mut result=Vec::with_capacity(passes as usize);
    for i in 1..=passes {let fraction=i as f64/passes as f64;result.push(total*if constant_area{fraction.sqrt()}else{fraction});}
    *result.last_mut().unwrap()=total;Ok(result)
}

/// Absolute axial offsets from the nominal pass origin. Positive angle is
/// flank infeed; alternating switches flanks deterministically on each pass.
pub fn infeed_offsets(depths:&[f64],angle_deg:f64,alternate:bool)->Result<Vec<f64>,CamError> {
    if !angle_deg.is_finite() || !(0. ..60.).contains(&angle_deg) || depths.iter().any(|d|!d.is_finite() || *d<=0.) {return Err(invalid("invalid turning thread flank infeed"));}
    let factor=angle_deg.to_radians().tan();
    Ok(depths.iter().enumerate().map(|(i,d)|d*factor*if alternate && i%2==1{-1.}else{1.}).collect())
}
