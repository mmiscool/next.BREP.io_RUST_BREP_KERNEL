//! [`Parts`]: the in-memory part list, which remembers what a change touched.
//!
//! The store keeps every part in memory for reads, and writes each change to
//! SQLite ([`crate::sql`]). Before SQLite, [`crate::db::Db::mutate`] copied
//! the WHOLE state before every change so that it could roll one back, and
//! rewrote the whole metadata file after it. Both were O(catalog) per write.
//!
//! This type makes a change O(what it touched). Every way to get a part
//! mutably goes through here, and while a change is open ([`Parts::begin`])
//! the first mutable touch of a part copies that part aside. At the end the
//! change either keeps its edits ([`Parts::commit`], which hands back each
//! touched part's before-copy so the store can write and audit exactly those)
//! or restores the copies ([`Parts::rollback`]).
//!
//! The field is private and there is no `DerefMut`, so the compiler finds any
//! write that tries to go around the journal.

use std::collections::{BTreeMap, HashMap};
use std::ops::Deref;

use serde::{Deserialize, Deserializer, Serialize, Serializer};

use crate::model::Part;

/// Every part, in the order they were created, with an id index.
#[derive(Debug, Default, Clone)]
pub struct Parts {
    items: Vec<Part>,
    index: HashMap<String, usize>,
    /// While a change is open: each touched position, and the part as it was
    /// before the change (`None`: the change created it).
    journal: Option<BTreeMap<usize, Option<Part>>>,
}

/// What one committed change did to one part.
#[derive(Debug)]
pub struct Touched {
    /// Where the part sits in creation order; the store's `ord`.
    pub position: usize,
    /// The part before the change, or `None` when the change created it.
    pub before: Option<Part>,
}

impl Parts {
    pub fn from_vec(items: Vec<Part>) -> Self {
        let index = items.iter().enumerate().map(|(i, p)| (p.id.clone(), i)).collect();
        Parts { items, index, journal: None }
    }

    /// The part with this id.
    pub fn get(&self, id: &str) -> Option<&Part> {
        self.index.get(id).map(|&i| &self.items[i])
    }

    /// Where the part with this id sits in creation order.
    pub fn position(&self, id: &str) -> Option<usize> {
        self.index.get(id).copied()
    }

    /// The part with this id, for changing. Journaled.
    pub fn get_mut(&mut self, id: &str) -> Option<&mut Part> {
        let position = self.position(id)?;
        Some(self.at_mut(position))
    }

    /// The part at `position`, for changing. Journaled.
    pub fn at_mut(&mut self, position: usize) -> &mut Part {
        if let Some(journal) = self.journal.as_mut() {
            journal.entry(position).or_insert_with(|| Some(self.items[position].clone()));
        }
        &mut self.items[position]
    }

    /// The newest part, for changing. Journaled.
    pub fn last_mut(&mut self) -> Option<&mut Part> {
        let last = self.items.len().checked_sub(1)?;
        Some(self.at_mut(last))
    }

    /// Every part, for changing. Journals EVERY part — O(catalog) — so it is
    /// for rare whole-store edits (a test, a data repair), never a request.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut Part> {
        if let Some(journal) = self.journal.as_mut() {
            for (position, part) in self.items.iter().enumerate() {
                journal.entry(position).or_insert_with(|| Some(part.clone()));
            }
        }
        self.items.iter_mut()
    }

    /// Add a part. Journaled. The caller has already refused a duplicate id.
    pub fn push(&mut self, part: Part) {
        let position = self.items.len();
        self.index.insert(part.id.clone(), position);
        self.items.push(part);
        if let Some(journal) = self.journal.as_mut() {
            journal.insert(position, None);
        }
    }

    /// Open a change. Nested changes are not a thing: the store holds one
    /// write lock.
    pub fn begin(&mut self) {
        debug_assert!(self.journal.is_none(), "a change is already open");
        self.journal = Some(BTreeMap::new());
    }

    /// Close the change keeping its edits, and say what it touched.
    pub fn commit(&mut self) -> Vec<Touched> {
        self.journal
            .take()
            .unwrap_or_default()
            .into_iter()
            .map(|(position, before)| Touched { position, before })
            .collect()
    }

    /// Close the change, putting every touched part back as it was and
    /// dropping any part the change created.
    pub fn rollback(&mut self) {
        let Some(journal) = self.journal.take() else { return };
        // Newest first, so created parts come off the end in reverse order.
        for (position, before) in journal.into_iter().rev() {
            match before {
                Some(part) => self.items[position] = part,
                None => {
                    let removed = self.items.remove(position);
                    self.index.remove(&removed.id);
                }
            }
        }
    }

    /// Put back the edits of a change that was committed in memory but whose
    /// write to disk then failed: `touched` is what [`Parts::commit`] said.
    pub fn undo(&mut self, touched: Vec<Touched>) {
        self.journal = Some(touched.into_iter().map(|t| (t.position, t.before)).collect());
        self.rollback();
    }
}

impl Deref for Parts {
    type Target = [Part];
    fn deref(&self) -> &[Part] {
        &self.items
    }
}

impl<'a> IntoIterator for &'a Parts {
    type Item = &'a Part;
    type IntoIter = std::slice::Iter<'a, Part>;
    fn into_iter(self) -> Self::IntoIter {
        self.items.iter()
    }
}

impl Serialize for Parts {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.items.serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Parts {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Parts::from_vec(Vec::deserialize(deserializer)?))
    }
}

