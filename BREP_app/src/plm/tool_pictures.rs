//! The tools library's pictures, fetched for the offline instructions player
//! (`PlayerData::inline_tool_pictures`) and the in-app Player pane: one
//! `GET /api/tools/<id>/picture` per linked tool id, once per session, the
//! answer kept (`Some((media type, bytes))`, or `None` when the tool has no
//! picture or the request failed). The export door holds its export while
//! any is pending; the frame's idle contract waits on the requests like any
//! PLM request.
use std::cell::RefCell;
use std::collections::{BTreeMap, HashMap};
use std::rc::Rc;

use super::client::PlmClient;

/// A fetched picture: its media type and bytes.
pub type Fetched = Option<(String, Vec<u8>)>;

enum Entry {
    Pending,
    Done(Fetched),
}

thread_local! {
    static CACHE: RefCell<HashMap<String, Entry>> = RefCell::new(HashMap::new());
}

fn safe_id(id: &str) -> bool {
    !id.is_empty() && id.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
}

/// Ask for the pictures of `ids` not asked for yet.
pub fn request(client: &Rc<PlmClient>, ids: &[String]) {
    for id in ids {
        if !safe_id(id) {
            continue;
        }
        let fresh = CACHE.with(|cache| {
            let mut cache = cache.borrow_mut();
            if cache.contains_key(id) {
                return false;
            }
            cache.insert(id.clone(), Entry::Pending);
            true
        });
        if !fresh {
            continue;
        }
        let (client, id) = (client.clone(), id.clone());
        crate::store::spawn_waking(async move {
            let got = fetch(&client, &id).await;
            CACHE.with(|cache| cache.borrow_mut().insert(id, Entry::Done(got)));
        });
    }
}

async fn fetch(client: &PlmClient, id: &str) -> Fetched {
    let response = client.call("GET", &format!("/api/tools/{id}/picture"), None).await.ok()?;
    if response.status != 200 || response.body.is_empty() {
        return None;
    }
    let media = response
        .headers
        .iter()
        .find(|(name, _)| name.eq_ignore_ascii_case("content-type"))
        .map(|(_, value)| value.clone())
        .unwrap_or_default();
    Some((media, response.body))
}

/// How many of `ids` are still being fetched.
pub fn pending_of(ids: &[String]) -> usize {
    CACHE.with(|cache| {
        let cache = cache.borrow();
        ids.iter().filter(|id| matches!(cache.get(id.as_str()), Some(Entry::Pending))).count()
    })
}

/// How many pictures are being fetched, all told.
pub fn pending() -> usize {
    CACHE.with(|cache| cache.borrow().values().filter(|entry| matches!(entry, Entry::Pending)).count())
}

/// The answers held for `ids` (an id never asked for, or still pending, is
/// absent).
pub fn fetched(ids: &[String]) -> BTreeMap<String, Fetched> {
    CACHE.with(|cache| {
        let cache = cache.borrow();
        ids.iter()
            .filter_map(|id| match cache.get(id.as_str()) {
                Some(Entry::Done(got)) => Some((id.clone(), got.clone())),
                _ => None,
            })
            .collect()
    })
}

/// The PNG bytes held for `id`, for a pane that decodes them (the app decodes
/// PNG; another type is the page's to show).
pub fn png_of(id: &str) -> Option<Vec<u8>> {
    CACHE.with(|cache| match cache.borrow().get(id) {
        Some(Entry::Done(Some((media, bytes)))) if media.split(';').next().unwrap_or("").trim().eq_ignore_ascii_case("image/png") => Some(bytes.clone()),
        _ => None,
    })
}

/// Forget every picture (a session change: another server's library).
pub fn clear() {
    CACHE.with(|cache| cache.borrow_mut().clear());
}
