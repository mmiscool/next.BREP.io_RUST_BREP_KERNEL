//! [`Journaled`]: an in-memory list of keyed records that remembers what a
//! change touched, so the store writes and audits only those.
//!
//! [`crate::db::Db::mutate`] serializes and diffs the state's other fields on
//! every change — every write on the server, a sign-in's `last_seen` stamp
//! included — so a list kept there costs every write O(its length). Parts
//! have their own journal ([`crate::table::Parts`]); this is the same idea,
//! keyed by a string rather than a creation position, for the lists that
//! grow with use: workspace entries ([`crate::workspace::Workspace`]) and
//! browser sessions.
//!
//! The list is private and there is no `DerefMut`: every mutable touch goes
//! through [`Journaled::get_mut`], [`Journaled::iter_mut`],
//! [`Journaled::push`], [`Journaled::remove`] or [`Journaled::retain`], which
//! record the record's before-copy while a change is open.

use std::collections::{BTreeMap, BTreeSet, HashMap};

/// A record with a key that is unique in its list and never changes.
pub trait Keyed {
    fn key(&self) -> &str;
}

#[derive(Debug, Clone)]
pub struct Journaled<T> {
    items: Vec<T>,
    /// Key → position in `items`.
    index: HashMap<String, usize>,
    journal: Option<BTreeMap<String, Option<T>>>,
}

impl<T> Default for Journaled<T> {
    fn default() -> Self {
        Journaled { items: Vec::new(), index: HashMap::new(), journal: None }
    }
}

/// A record a change touched: its key and what it was before (`None`: it
/// was created). What it is after is read from the list (absent: removed).
#[derive(Debug, Clone)]
pub struct Touched<T> {
    pub key: String,
    pub before: Option<T>,
}

impl<T: Keyed + Clone> Journaled<T> {
    pub fn from_vec(items: Vec<T>) -> Self {
        let mut list = Journaled { items, index: HashMap::new(), journal: None };
        list.reindex();
        list
    }

    fn reindex(&mut self) {
        self.index = self.items.iter().enumerate().map(|(i, e)| (e.key().to_string(), i)).collect();
    }

    pub fn get(&self, key: &str) -> Option<&T> {
        self.index.get(key).map(|&i| &self.items[i])
    }

    fn note(&mut self, key: &str) {
        if self.journal.as_ref().is_none_or(|j| j.contains_key(key)) {
            return;
        }
        let before = self.get(key).cloned();
        if let Some(journal) = self.journal.as_mut() {
            journal.insert(key.to_string(), before);
        }
    }

    pub fn get_mut(&mut self, key: &str) -> Option<&mut T> {
        let position = *self.index.get(key)?;
        self.note(key);
        Some(&mut self.items[position])
    }

    /// Every record, mutably — each is recorded as touched. For a sweep
    /// that may change any of them; [`Journaled::get_mut`] for one.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut T> {
        if let Some(journal) = self.journal.as_mut() {
            for item in &self.items {
                journal.entry(item.key().to_string()).or_insert_with(|| Some(item.clone()));
            }
        }
        self.items.iter_mut()
    }

    /// Add a record. Its key must be new.
    pub fn push(&mut self, item: T) {
        debug_assert!(!self.index.contains_key(item.key()), "a key is never reused");
        let key = item.key().to_string();
        self.note(&key);
        self.index.insert(key, self.items.len());
        self.items.push(item);
    }

    /// Remove every record whose key is in `keys`.
    pub fn remove(&mut self, keys: &BTreeSet<String>) {
        self.retain(|item| !keys.contains(item.key()));
    }

    /// Keep the records `keep` accepts; the rest are removed (and recorded).
    pub fn retain(&mut self, mut keep: impl FnMut(&T) -> bool) {
        let gone: Vec<String> = self.items.iter().filter(|i| !keep(i)).map(|i| i.key().to_string()).collect();
        if gone.is_empty() {
            return;
        }
        for key in &gone {
            self.note(key);
        }
        let gone: BTreeSet<String> = gone.into_iter().collect();
        self.items.retain(|i| !gone.contains(i.key()));
        self.reindex();
    }

    /// Open a change: from here each first touch of a record is recorded.
    pub fn begin(&mut self) {
        self.journal = Some(BTreeMap::new());
    }

    /// Close the change, keeping its edits; what it touched.
    pub fn commit(&mut self) -> Vec<Touched<T>> {
        self.journal.take().unwrap_or_default().into_iter().map(|(key, before)| Touched { key, before }).collect()
    }

    /// Close the change, putting every touched record back as it was.
    pub fn rollback(&mut self) {
        let Some(journal) = self.journal.take() else { return };
        let created: BTreeSet<&String> = journal.iter().filter(|(_, b)| b.is_none()).map(|(k, _)| k).collect();
        self.items.retain(|e| !created.contains(&e.key().to_string()));
        self.reindex();
        for (_, before) in journal {
            let Some(item) = before else { continue };
            match self.index.get(item.key()) {
                Some(&i) => self.items[i] = item,
                None => {
                    self.index.insert(item.key().to_string(), self.items.len());
                    self.items.push(item);
                }
            }
        }
    }

    /// Put back a change that was committed in memory but failed to reach
    /// the disk.
    pub fn undo(&mut self, touched: Vec<Touched<T>>) {
        self.journal = Some(touched.into_iter().map(|t| (t.key, t.before)).collect());
        self.rollback();
    }

    /// The touched records that actually changed, each with what it is now
    /// (`None`: removed) — what a change writes and audits.
    pub fn changed<'a>(&'a self, touched: &'a [Touched<T>]) -> Vec<(&'a Touched<T>, Option<&'a T>)>
    where
        T: PartialEq,
    {
        touched
            .iter()
            .map(|t| (t, self.get(&t.key)))
            .filter(|(t, now)| t.before.as_ref() != *now)
            .collect()
    }
}

impl<T> std::ops::Deref for Journaled<T> {
    type Target = [T];
    fn deref(&self) -> &[T] {
        &self.items
    }
}

impl<T: serde::Serialize> serde::Serialize for Journaled<T> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        self.items.serialize(serializer)
    }
}

impl<'de, T: serde::Deserialize<'de> + Keyed + Clone> serde::Deserialize<'de> for Journaled<T> {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        Ok(Journaled::from_vec(Vec::deserialize(deserializer)?))
    }
}

impl Keyed for crate::model::Session {
    fn key(&self) -> &str {
        &self.token
    }
}

