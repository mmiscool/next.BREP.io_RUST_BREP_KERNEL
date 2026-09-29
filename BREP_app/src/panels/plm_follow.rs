//! The PLM pane follows other clients live (S3's recorded gap).
//!
//! Every mutation on the server moves the store's sequence number, including
//! the ones that write no document: a checkout, a check-in, a release, a
//! review decision. The change feed (`GET /api/store/changes?since=`) answers
//! that number, so a moved `seq` means "someone changed something". The keys
//! alone would miss lifecycle events, which touch no document.
//!
//! [`Follower`] asks the feed at most once every [`FOLLOW_SECS`], with at most
//! one request in flight. When the number moves, the pane re-reads each open
//! PLM document's part (one request per open panel, and a panel already
//! reading is not asked again) and the inbox badge asks again. So the cost of
//! following is:
//!
//! - one small request per interval while nothing happens;
//! - one feed request plus one part read per open PLM document, and one inbox
//!   read, per interval in which something happened.
//!
//! The first answer only records where the feed stands: nothing is re-read
//! for what happened before the session looked.
use crate::plm::client::{Changes, PlmClient};
use crate::plm::PlmFuture;
use std::future::Future;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

/// How often the pane asks whether anything changed, in seconds.
pub const FOLLOW_SECS: f64 = 5.0;

/// The change feed, as the follower asks it: the real client, or a test's
/// counting fake.
pub trait PlmFeed {
    fn changes(&self, since: u64) -> PlmFuture<Changes>;
}

impl PlmFeed for Rc<PlmClient> {
    fn changes(&self, since: u64) -> PlmFuture<Changes> {
        let client = self.clone();
        // The inherent method, by path: on an `Rc<PlmClient>`, `client.changes`
        // would resolve to THIS trait method again, forever.
        Box::pin(async move { PlmClient::changes(&client, since).await.map_err(|e| e.to_string()) })
    }
}

/// What the feed said moved: the keys it named (lock and lifecycle changes
/// name their revisions' keys, server `touch_part`), or `stale` when it could
/// not say.
#[derive(Clone, Debug, PartialEq)]
pub struct Moved {
    pub keys: Vec<String>,
    pub stale: bool,
}

/// Where the pane stands in the change feed.
#[derive(Default)]
pub struct Follower {
    /// The last `seq` the feed answered; `None` before the first answer.
    seen: Option<u64>,
    asked_at: Option<f64>,
    request: Option<PlmFuture<Changes>>,
    /// Feed requests made, for the budget test and the state blob.
    pub requests: u64,
    /// Times the feed said something moved.
    pub moves: u64,
}

impl Follower {
    /// A feed request is in flight.
    pub fn busy(&self) -> bool {
        self.request.is_some()
    }

    /// Ask at the next tick instead of at the interval.
    pub fn ask_now(&mut self) {
        self.asked_at = None;
    }

    /// Seconds until the next question (0 while one is in flight).
    pub fn next_in(&self, now: f64) -> f64 {
        if self.request.is_some() {
            return 0.0;
        }
        self.asked_at.map_or(0.0, |at| (at + FOLLOW_SECS - now).max(0.0))
    }

    /// Once a frame. `Some(keys)` when the feed moved since the last answer:
    /// the pane refreshes. The keys are the documents written (lifecycle
    /// events carry none), for whoever wants them.
    pub fn tick(&mut self, now: f64, feed: &dyn PlmFeed) -> Option<Moved> {
        let mut moved = None;
        if let Some(request) = self.request.as_mut() {
            let mut cx = Context::from_waker(Waker::noop());
            if let Poll::Ready(answer) = request.as_mut().poll(&mut cx) {
                self.request = None;
                if let Ok(changes) = answer {
                    // A stale answer means the log was trimmed past us: the
                    // keys are incomplete, but something certainly moved.
                    if self.seen.is_some_and(|seen| seen != changes.seq || changes.stale) {
                        self.moves += 1;
                        moved = Some(Moved { keys: changes.keys, stale: changes.stale });
                    }
                    self.seen = Some(changes.seq);
                }
                // A failed question is asked again at the next interval.
            }
        }
        let due = self.asked_at.is_none_or(|at| now - at >= FOLLOW_SECS);
        if moved.is_none() && due && self.request.is_none() {
            self.asked_at = Some(now);
            self.requests += 1;
            self.request = Some(feed.changes(self.seen.unwrap_or(0)));
            // An answer already in hand (a test's, a cache's) is taken now.
            return self.tick_answer();
        }
        moved
    }

    /// Take an answer that resolved as it was asked.
    fn tick_answer(&mut self) -> Option<Moved> {
        let request = self.request.as_mut()?;
        let mut cx = Context::from_waker(Waker::noop());
        let Poll::Ready(answer) = request.as_mut().poll(&mut cx) else { return None };
        self.request = None;
        let changes = answer.ok()?;
        let moved = self.seen.is_some_and(|seen| seen != changes.seq || changes.stale);
        self.seen = Some(changes.seq);
        if moved {
            self.moves += 1;
            Some(Moved { keys: changes.keys, stale: changes.stale })
        } else {
            None
        }
    }
}

