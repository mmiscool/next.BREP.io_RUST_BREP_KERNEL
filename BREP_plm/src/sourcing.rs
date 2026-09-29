//! Sourcing: who makes a part and who sells it.
//!
//! Two lists of companies.
//!
//! Sourcing is PART-level, like catalog attributes, and it is deliberately
//! not frozen by [`crate::model::Settings::lock_released_attributes`]: a
//! released design does not stop the market moving, and a buyer adding a
//! second source must not need an engineer to open a revision.
//!
//! Every write takes a JSON object and MERGES it, so the page, a script and a
//! future importer all speak the same shape. A company may be named by its id
//! or its name.

use serde_json::{json, Map, Value};

use crate::auth;
use crate::db::{now, Db, State};
use crate::model::{Company, ManufacturerPart, Part, PriceBreak, SourcingStatus, SupplierOffer};
use crate::Error;

/// Which list of companies a call is about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Companies {
    Manufacturers,
    Suppliers,
}

impl Companies {
    /// "manufacturer" / "supplier", for messages.
    pub fn one(self) -> &'static str {
        match self {
            Companies::Manufacturers => "manufacturer",
            Companies::Suppliers => "supplier",
        }
    }

    pub fn list(self, state: &State) -> &Vec<Company> {
        match self {
            Companies::Manufacturers => &state.manufacturers,
            Companies::Suppliers => &state.suppliers,
        }
    }

    fn list_mut(self, state: &mut State) -> &mut Vec<Company> {
        match self {
            Companies::Manufacturers => &mut state.manufacturers,
            Companies::Suppliers => &mut state.suppliers,
        }
    }
}

/// The company `key` names in `list` — by id, or else by name without regard
/// to ASCII case.
pub fn find_company<'a>(list: &'a [Company], key: &str) -> Option<&'a Company> {
    let key = key.trim();
    list.iter()
        .find(|c| c.id == key)
        .or_else(|| list.iter().find(|c| c.name.eq_ignore_ascii_case(key)))
}

/// How many manufacturer parts (for a manufacturer) or offers (for a
/// supplier) name `id`, across every part.
pub fn uses(state: &State, which: Companies, id: &str) -> usize {
    state
        .parts
        .iter()
        .flat_map(|p| p.sourcing.iter())
        .map(|mp| match which {
            Companies::Manufacturers => usize::from(mp.manufacturer == id),
            Companies::Suppliers => mp.offers.iter().filter(|o| o.supplier == id).count(),
        })
        .sum()
}

/// A part's sourcing with every company id resolved to its name — what a
/// person, and a script, actually want to read.
pub fn resolved(state: &State, part: &Part) -> Value {
    let name = |list: &[Company], id: &str| find_company(list, id).map(|c| c.name.clone()).unwrap_or_default();
    Value::Array(
        part.sourcing
            .iter()
            .map(|mp| {
                let mut out = serde_json::to_value(mp).unwrap_or(Value::Null);
                out["manufacturer_name"] = json!(name(&state.manufacturers, &mp.manufacturer));
                if let Some(offers) = out["offers"].as_array_mut() {
                    for offer in offers {
                        let id = offer["supplier"].as_str().unwrap_or_default().to_string();
                        offer["supplier_name"] = json!(name(&state.suppliers, &id));
                    }
                }
                out
            })
            .collect(),
    )
}

// ===========================================================================
// Field parsing — shared by every write
// ===========================================================================

fn object(fields: &Value) -> Result<&Map<String, Value>, Error> {
    fields
        .as_object()
        .ok_or_else(|| Error::bad_request("the fields to change must be an object"))
}

fn only(fields: &Map<String, Value>, allowed: &[&str], what: &str) -> Result<(), Error> {
    for key in fields.keys() {
        if !allowed.contains(&key.as_str()) {
            return Err(Error::bad_request(format!(
                "'{key}' is not a field of a {what} — {}",
                allowed.join(", ")
            )));
        }
    }
    Ok(())
}

fn text(fields: &Map<String, Value>, key: &str) -> Result<Option<String>, Error> {
    match fields.get(key) {
        None => Ok(None),
        Some(Value::Null) => Ok(Some(String::new())),
        Some(Value::String(s)) => Ok(Some(s.trim().to_string())),
        Some(Value::Number(n)) => Ok(Some(n.to_string())),
        Some(_) => Err(Error::bad_request(format!("'{key}' must be text"))),
    }
}

/// A whole number ≥ 0, from a number or from form text; `null` or `""` clears.
fn count(fields: &Map<String, Value>, key: &str) -> Result<Option<Option<u64>>, Error> {
    let bad = || Error::bad_request(format!("'{key}' must be a whole number of 0 or more"));
    match fields.get(key) {
        None => Ok(None),
        Some(Value::Null) => Ok(Some(None)),
        Some(Value::String(s)) if s.trim().is_empty() => Ok(Some(None)),
        Some(Value::String(s)) => s.trim().parse::<u64>().map(|n| Some(Some(n))).map_err(|_| bad()),
        Some(Value::Number(n)) => n.as_u64().map(|n| Some(Some(n))).ok_or_else(bad),
        Some(_) => Err(bad()),
    }
}

fn flag(fields: &Map<String, Value>, key: &str) -> Result<Option<bool>, Error> {
    match fields.get(key) {
        None => Ok(None),
        Some(Value::Bool(b)) => Ok(Some(*b)),
        Some(_) => Err(Error::bad_request(format!("'{key}' must be true or false"))),
    }
}

fn number(value: &Value) -> Option<f64> {
    match value {
        Value::Number(n) => n.as_f64(),
        Value::String(s) => s.trim().parse().ok(),
        _ => None,
    }
}

/// Price breaks from `[{qty, unit_price}]`: each quantity at least 1, each
/// price a finite number of 0 or more, no quantity twice; stored ascending.
fn price_breaks(value: &Value) -> Result<Vec<PriceBreak>, Error> {
    let items = match value {
        Value::Null => return Ok(Vec::new()),
        Value::Array(items) => items,
        _ => return Err(Error::bad_request("price_breaks must be a list of {qty, unit_price}")),
    };
    let mut out: Vec<PriceBreak> = Vec::with_capacity(items.len());
    for item in items {
        let qty = number(&item["qty"])
            .filter(|q| q.fract() == 0.0 && *q >= 1.0)
            .ok_or_else(|| Error::bad_request(format!("a price break quantity must be a whole number of 1 or more: {item}")))?
            as u64;
        let unit_price = number(&item["unit_price"])
            .filter(|p| p.is_finite() && *p >= 0.0)
            .ok_or_else(|| Error::bad_request(format!("a unit price must be a number of 0 or more: {item}")))?;
        if out.iter().any(|b| b.qty == qty) {
            return Err(Error::bad_request(format!("quantity {qty} has two prices")));
        }
        out.push(PriceBreak { qty, unit_price });
    }
    out.sort_by_key(|b| b.qty);
    Ok(out)
}

fn status(fields: &Map<String, Value>) -> Result<Option<SourcingStatus>, Error> {
    let Some(value) = text(fields, "status")? else { return Ok(None) };
    match value.to_ascii_lowercase().as_str() {
        "" | "active" => Ok(Some(SourcingStatus::Active)),
        "nrnd" => Ok(Some(SourcingStatus::Nrnd)),
        "obsolete" => Ok(Some(SourcingStatus::Obsolete)),
        other => Err(Error::bad_request(format!("'{other}' is not a status — active, nrnd or obsolete"))),
    }
}

/// The longest MPN or SPN accepted. Long enough for any real catalogue.
const MAX_PART_NUMBER: usize = 128;

fn part_number_text(value: &str, what: &str) -> Result<(), Error> {
    if value.chars().count() > MAX_PART_NUMBER || value.chars().any(char::is_control) {
        return Err(Error::bad_request(format!(
            "{what} must be at most {MAX_PART_NUMBER} characters with no control characters"
        )));
    }
    Ok(())
}

fn part_mut<'a>(state: &'a mut State, part_id: &str) -> Result<&'a mut Part, Error> {
    let id = state
        .part_by_id_or_number(part_id)
        .map(|p| p.id.clone())
        .ok_or_else(|| Error::not_found("part"))?;
    Ok(state.parts.get_mut(&id).expect("found above"))
}

fn company_id(state: &State, which: Companies, key: &str) -> Result<String, Error> {
    find_company(which.list(state), key)
        .map(|c| c.id.clone())
        .ok_or_else(|| Error::bad_request(format!("there is no {} '{}'", which.one(), key.trim())))
}

// ===========================================================================
// The store's sourcing operations
// ===========================================================================

impl Db {
    // -- companies ---------------------------------------------------------

    /// Add a manufacturer or supplier: `{name, website?, notes?}`.
    pub fn create_company(&self, which: Companies, fields: &Value) -> Result<Company, Error> {
        let fields = object(fields)?.clone();
        only(&fields, &["name", "website", "notes"], which.one())?;
        self.mutate(move |state| {
            let mut company = Company {
                id: auth::new_id(),
                name: String::new(),
                website: String::new(),
                notes: String::new(),
                created_at: now(),
            };
            apply_company(which.list(state), &mut company, &fields)?;
            which.list_mut(state).push(company.clone());
            Ok(company)
        })
    }

    pub fn update_company(&self, which: Companies, id: &str, fields: &Value) -> Result<Company, Error> {
        let fields = object(fields)?.clone();
        only(&fields, &["name", "website", "notes"], which.one())?;
        let id = id.to_string();
        self.mutate(move |state| {
            let list = which.list(state);
            let index = list
                .iter()
                .position(|c| c.id == id)
                .ok_or_else(|| Error::not_found(which.one()))?;
            let mut company = list[index].clone();
            apply_company(list, &mut company, &fields)?;
            which.list_mut(state)[index] = company.clone();
            Ok(company)
        })
    }

    /// Remove a company that nothing names. One still named by a manufacturer
    /// part or an offer is refused, with the count: deleting it would leave
    /// sourcing pointing at nobody.
    pub fn delete_company(&self, which: Companies, id: &str) -> Result<(), Error> {
        let id = id.to_string();
        self.mutate(move |state| {
            let company = which
                .list(state)
                .iter()
                .find(|c| c.id == id)
                .cloned()
                .ok_or_else(|| Error::not_found(which.one()))?;
            let used = uses(state, which, &id);
            if used > 0 {
                let what = match which {
                    Companies::Manufacturers => "manufacturer part",
                    Companies::Suppliers => "offer",
                };
                return Err(Error::conflict(format!(
                    "{} is named by {used} {what}{} — remove {} first",
                    company.name,
                    if used == 1 { "" } else { "s" },
                    if used == 1 { "it" } else { "them" },
                )));
            }
            which.list_mut(state).retain(|c| c.id != id);
            Ok(())
        })
    }

    // -- manufacturer parts -------------------------------------------------

    /// Add a manufacturer part to a part:
    /// `{manufacturer, mpn, status?, preferred?, datasheet?, notes?}`.
    pub fn add_manufacturer_part(&self, part_id: &str, fields: &Value) -> Result<ManufacturerPart, Error> {
        let fields = object(fields)?.clone();
        only(&fields, MP_FIELDS, "manufacturer part")?;
        let part_id = part_id.to_string();
        self.mutate(move |state| {
            let mut mp = ManufacturerPart {
                id: auth::new_id(),
                manufacturer: String::new(),
                mpn: String::new(),
                status: SourcingStatus::Active,
                preferred: false,
                datasheet: String::new(),
                notes: String::new(),
                offers: Vec::new(),
                created_at: now(),
            };
            apply_mp(state, &mut mp, &fields)?;
            let part = part_mut(state, &part_id)?;
            check_mp_unique(part, &mp)?;
            if mp.preferred {
                part.sourcing.iter_mut().for_each(|other| other.preferred = false);
            }
            part.sourcing.push(mp.clone());
            Ok(mp)
        })
    }

    pub fn update_manufacturer_part(&self, part_id: &str, mp_id: &str, fields: &Value) -> Result<ManufacturerPart, Error> {
        let fields = object(fields)?.clone();
        only(&fields, MP_FIELDS, "manufacturer part")?;
        let (part_id, mp_id) = (part_id.to_string(), mp_id.to_string());
        self.mutate(move |state| {
            let mut mp = find_mp(state, &part_id, &mp_id)?.clone();
            apply_mp(state, &mut mp, &fields)?;
            let part = part_mut(state, &part_id)?;
            check_mp_unique(part, &mp)?;
            if mp.preferred {
                part.sourcing.iter_mut().for_each(|other| other.preferred = false);
            }
            let slot = part.sourcing.iter_mut().find(|m| m.id == mp_id).expect("found above");
            *slot = mp.clone();
            Ok(mp)
        })
    }

    /// Remove a manufacturer part and every offer under it.
    pub fn delete_manufacturer_part(&self, part_id: &str, mp_id: &str) -> Result<(), Error> {
        let (part_id, mp_id) = (part_id.to_string(), mp_id.to_string());
        self.mutate(move |state| {
            find_mp(state, &part_id, &mp_id)?;
            part_mut(state, &part_id)?.sourcing.retain(|m| m.id != mp_id);
            Ok(())
        })
    }

    // -- supplier offers ----------------------------------------------------

    /// Add a supplier's offer to a manufacturer part:
    /// `{supplier, spn?, url?, currency?, price_breaks?, lead_time_days?, moq?, stock?, notes?}`.
    pub fn add_offer(&self, part_id: &str, mp_id: &str, fields: &Value) -> Result<SupplierOffer, Error> {
        let fields = object(fields)?.clone();
        only(&fields, OFFER_FIELDS, "supplier offer")?;
        let (part_id, mp_id) = (part_id.to_string(), mp_id.to_string());
        self.mutate(move |state| {
            find_mp(state, &part_id, &mp_id)?;
            let mut offer = SupplierOffer {
                id: auth::new_id(),
                supplier: String::new(),
                spn: String::new(),
                url: String::new(),
                currency: String::new(),
                price_breaks: Vec::new(),
                lead_time_days: None,
                moq: None,
                stock: None,
                notes: String::new(),
                updated_at: now(),
            };
            apply_offer(state, &mut offer, &fields)?;
            let part = part_mut(state, &part_id)?;
            let mp = part.sourcing.iter_mut().find(|m| m.id == mp_id).expect("found above");
            mp.offers.push(offer.clone());
            Ok(offer)
        })
    }

    pub fn update_offer(&self, part_id: &str, mp_id: &str, offer_id: &str, fields: &Value) -> Result<SupplierOffer, Error> {
        let fields = object(fields)?.clone();
        only(&fields, OFFER_FIELDS, "supplier offer")?;
        let (part_id, mp_id, offer_id) = (part_id.to_string(), mp_id.to_string(), offer_id.to_string());
        self.mutate(move |state| {
            let mut offer = find_mp(state, &part_id, &mp_id)?
                .offers
                .iter()
                .find(|o| o.id == offer_id)
                .cloned()
                .ok_or_else(|| Error::not_found("offer"))?;
            apply_offer(state, &mut offer, &fields)?;
            offer.updated_at = now();
            let part = part_mut(state, &part_id)?;
            let mp = part.sourcing.iter_mut().find(|m| m.id == mp_id).expect("found above");
            let slot = mp.offers.iter_mut().find(|o| o.id == offer_id).expect("found above");
            *slot = offer.clone();
            Ok(offer)
        })
    }

    pub fn delete_offer(&self, part_id: &str, mp_id: &str, offer_id: &str) -> Result<(), Error> {
        let (part_id, mp_id, offer_id) = (part_id.to_string(), mp_id.to_string(), offer_id.to_string());
        self.mutate(move |state| {
            if !find_mp(state, &part_id, &mp_id)?.offers.iter().any(|o| o.id == offer_id) {
                return Err(Error::not_found("offer"));
            }
            let part = part_mut(state, &part_id)?;
            let mp = part.sourcing.iter_mut().find(|m| m.id == mp_id).expect("found above");
            mp.offers.retain(|o| o.id != offer_id);
            Ok(())
        })
    }
}

const MP_FIELDS: &[&str] = &["manufacturer", "mpn", "status", "preferred", "datasheet", "notes"];
const OFFER_FIELDS: &[&str] =
    &["supplier", "spn", "url", "currency", "price_breaks", "lead_time_days", "moq", "stock", "notes"];

fn apply_company(list: &[Company], company: &mut Company, fields: &Map<String, Value>) -> Result<(), Error> {
    if let Some(name) = text(fields, "name")? {
        company.name = name;
    }
    if company.name.is_empty() {
        return Err(Error::bad_request("a company needs a name"));
    }
    if let Some(clash) = list
        .iter()
        .find(|c| c.id != company.id && c.name.eq_ignore_ascii_case(&company.name))
    {
        return Err(Error::conflict(format!("there is already a company named {}", clash.name)));
    }
    if let Some(website) = text(fields, "website")? {
        company.website = website;
    }
    if let Some(notes) = text(fields, "notes")? {
        company.notes = notes;
    }
    Ok(())
}

fn find_mp<'a>(state: &'a State, part_id: &str, mp_id: &str) -> Result<&'a ManufacturerPart, Error> {
    state
        .part_by_id_or_number(part_id)
        .ok_or_else(|| Error::not_found("part"))?
        .sourcing
        .iter()
        .find(|m| m.id == mp_id)
        .ok_or_else(|| Error::not_found("manufacturer part"))
}

fn apply_mp(state: &State, mp: &mut ManufacturerPart, fields: &Map<String, Value>) -> Result<(), Error> {
    if let Some(key) = text(fields, "manufacturer")? {
        mp.manufacturer = company_id(state, Companies::Manufacturers, &key)?;
    }
    if mp.manufacturer.is_empty() {
        return Err(Error::bad_request("a manufacturer part needs a manufacturer"));
    }
    if let Some(mpn) = text(fields, "mpn")? {
        mp.mpn = mpn;
    }
    if mp.mpn.is_empty() {
        return Err(Error::bad_request("a manufacturer part needs an MPN"));
    }
    part_number_text(&mp.mpn, "an MPN")?;
    if let Some(status) = status(fields)? {
        mp.status = status;
    }
    if let Some(preferred) = flag(fields, "preferred")? {
        mp.preferred = preferred;
    }
    if let Some(datasheet) = text(fields, "datasheet")? {
        mp.datasheet = datasheet;
    }
    if let Some(notes) = text(fields, "notes")? {
        mp.notes = notes;
    }
    Ok(())
}

/// One part never lists the same manufacturer's MPN twice. The same MPN on
/// two DIFFERENT parts is allowed: two internal parts may well approve one
/// catalogue item, and refusing it would block a legitimate setup.
fn check_mp_unique(part: &Part, mp: &ManufacturerPart) -> Result<(), Error> {
    if part
        .sourcing
        .iter()
        .any(|m| m.id != mp.id && m.manufacturer == mp.manufacturer && m.mpn.eq_ignore_ascii_case(&mp.mpn))
    {
        return Err(Error::conflict(format!("{} already lists {} from that manufacturer", part.number, mp.mpn)));
    }
    Ok(())
}

fn apply_offer(state: &State, offer: &mut SupplierOffer, fields: &Map<String, Value>) -> Result<(), Error> {
    if let Some(key) = text(fields, "supplier")? {
        offer.supplier = company_id(state, Companies::Suppliers, &key)?;
    }
    if offer.supplier.is_empty() {
        return Err(Error::bad_request("an offer needs a supplier"));
    }
    if let Some(spn) = text(fields, "spn")? {
        part_number_text(&spn, "an SPN")?;
        offer.spn = spn;
    }
    if let Some(url) = text(fields, "url")? {
        offer.url = url;
    }
    if let Some(currency) = text(fields, "currency")? {
        offer.currency = currency.to_ascii_uppercase();
    }
    if let Some(value) = fields.get("price_breaks") {
        offer.price_breaks = price_breaks(value)?;
    }
    if let Some(days) = count(fields, "lead_time_days")? {
        offer.lead_time_days = days
            .map(|d| u32::try_from(d).map_err(|_| Error::bad_request("lead_time_days is too large")))
            .transpose()?;
    }
    if let Some(moq) = count(fields, "moq")? {
        offer.moq = moq;
    }
    if let Some(stock) = count(fields, "stock")? {
        offer.stock = stock;
    }
    if let Some(notes) = text(fields, "notes")? {
        offer.notes = notes;
    }
    Ok(())
}
