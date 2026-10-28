//! Parameterized collision envelopes, not manufacturing drawings. ER nut
//! envelope sizes are documented in the design note; other proportions are
//! original editable starting shapes, not copied vendor catalogue geometry.
use super::*;
#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
#[serde(rename_all="kebab-case")]
pub enum HolderFamily {Er16,Er25,Er32,SideLock,ShrinkFit,Hydraulic}
#[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
#[serde(rename_all="camelCase")]
pub struct HolderDesign {
    pub family:HolderFamily,pub bore_diameter:f64,pub nose_diameter:f64,pub body_diameter:f64,
    pub nose_length:f64,pub transition_length:f64,pub body_length:f64,
}
impl HolderDesign {
    /// Select a common starting envelope in model units. All heights are
    /// editable and the requested height is the sum of the three segments.
    pub fn starting(family:HolderFamily,bore:f64,height:f64,units:LibraryUnits)->Result<Self,String> {
        if !bore.is_finite()||bore<=0.||!height.is_finite()||height<=0. {return Err("holder bore and height must be positive".into());}
        let scale=units.millimeters_per_unit();let b=bore*scale;
        let (nose,body)=match family {
            HolderFamily::Er16=>{if b>10.+1e-9 {return Err("ER16 bore exceeds 10 mm".into());}(28.,32.)},
            HolderFamily::Er25=>{if b>16.+1e-9 {return Err("ER25 bore exceeds 16 mm".into());}(42.,46.)},
            HolderFamily::Er32=>{if b>20.+1e-9 {return Err("ER32 bore exceeds 20 mm".into());}(50.,54.)},
            HolderFamily::SideLock=>(b+18.,b+24.),HolderFamily::ShrinkFit=>(b+8.,b+24.),HolderFamily::Hydraulic=>(b+20.,b+30.),
        };
        let d=Self {family,bore_diameter:bore,nose_diameter:nose/scale,body_diameter:body/scale,nose_length:height*0.25,transition_length:height*0.25,body_length:height*0.5};d.validate()?;Ok(d)
    }
    pub fn validate(&self)->Result<(),String> {
        for v in [self.bore_diameter,self.nose_diameter,self.body_diameter,self.nose_length,self.transition_length,self.body_length] {if !v.is_finite()||v<=0. {return Err("holder dimensions must be finite and positive".into());}}
        if self.bore_diameter>=self.nose_diameter.min(self.body_diameter) {return Err("holder bore must be smaller than its envelope".into());}
        if !(self.nose_length+self.transition_length+self.body_length).is_finite(){return Err("holder height overflows".into());}Ok(())
    }
    pub fn build(&self,id:impl Into<HolderId>,name:impl Into<String>)->Result<Holder,String> {
        self.validate()?;let h=Holder {id:id.into(),name:name.into(),segments:vec![HolderSegment {height:self.nose_length,lower_diameter:self.nose_diameter,upper_diameter:self.nose_diameter},HolderSegment {height:self.transition_length,lower_diameter:self.nose_diameter,upper_diameter:self.body_diameter},HolderSegment {height:self.body_length,lower_diameter:self.body_diameter,upper_diameter:self.body_diameter}]};
        if h.id.as_str().trim().is_empty() {return Err("holder id is empty".into());}h.validate()?;Ok(h)
    }
}
#[derive(Clone,Debug,Default,PartialEq,Serialize,Deserialize)]
pub struct HolderLibrary {pub holders:Vec<Holder>}
impl HolderLibrary {
    pub fn validate(&self)->Result<(),String> {let mut ids=BTreeSet::new();for h in &self.holders {h.validate()?;if h.id.as_str().trim().is_empty()||h.segments.is_empty()||!h.height().is_finite()||!ids.insert(&h.id) {return Err("invalid/duplicate holder library entry".into());}}Ok(())}
    pub fn insert(&mut self,h:Holder)->Result<(),String> {
        h.validate()?;if h.id.as_str().trim().is_empty()||h.segments.is_empty()||!h.height().is_finite() {return Err("holder requires id and finite nonempty profile".into());}
        if self.holders.iter().any(|other|other.id==h.id) {return Err("duplicate holder id".into());}self.holders.push(h);Ok(())
    }
}
