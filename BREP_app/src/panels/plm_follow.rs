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
use crate::plm::client::{Changes, PlmClient, PlmError};
use crate::plm::PlmFuture;
use std::future::Future;
use std::rc::Rc;
use std::task::{Context, Poll, Waker};

/// What a feed error starts with when the server refused the credential
/// (`401`: revoked, expired, signed out), not the network.
pub const SIGNED_OUT: &str = "signed out: ";

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
        Box::pin(async move {
            PlmClient::changes(&client, since).await.map_err(|e| match e {
                PlmError::SignIn(why) => format!("{SIGNED_OUT}{why}"),
                e => e.to_string(),
            })
        })
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
    /// Where the last feed question left the session: answered, unanswered
    /// (the server is down or the network is gone), or refused (the sign-in
    /// ended on the server). The pane says each change once.
    reach: Standing,
    /// A change of [`Self::reach`] the pane has not told yet.
    told: Option<Reach>,
    /// A Save's write found the server unreachable since the last answer
    /// ([`Self::note_unreachable`]).
    missed_write: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum Standing {
    #[default]
    Answered,
    Unanswered,
    Refused,
}

/// What the feed's answers say about reaching the server since last asked.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reach {
    /// The first unanswered question after answers.
    Lost,
    /// The first answer after unanswered questions.
    Back,
    /// The server refused the session's credential: revoked under Linked
    /// desktops, expired, or signed out elsewhere.
    SignedOut,
    /// The first answer after the credential was refused: signed in again.
    SignedIn,
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
                self.reached(answer.as_ref().map(|_| ()).map_err(String::as_str));
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

    /// Note one question's fate: a change of reachability is kept for
    /// [`Self::take_reach`].
    fn reached(&mut self, answer: Result<(), &str>) {
        let now = match answer {
            Ok(()) => Standing::Answered,
            Err(why) if why.starts_with(SIGNED_OUT) => Standing::Refused,
            Err(_) => Standing::Unanswered,
        };
        let was = std::mem::replace(&mut self.reach, now);
        // A write the server missed while every question of ours was
        // answered: the next answer is the news. A question missed meanwhile
        // takes over (lost, then back, as for any outage).
        if std::mem::take(&mut self.missed_write) && was == Standing::Answered && now == Standing::Answered {
            self.told = Some(Reach::Back);
            return;
        }
        let news = match (was, now) {
            (a, b) if a == b => return,
            (_, Standing::Unanswered) => Reach::Lost,
            (_, Standing::Refused) => Reach::SignedOut,
            (Standing::Refused, Standing::Answered) => Reach::SignedIn,
            (_, Standing::Answered) => Reach::Back,
        };
        // Lost then back (or signed out, then in) before the pane looked:
        // nothing to tell.
        self.told = match (self.told, news) {
            (Some(Reach::Lost), Reach::Back) | (Some(Reach::SignedOut), Reach::SignedIn) => None,
            _ => Some(news),
        };
    }

    /// A write found the server unreachable (and its failure was told): the
    /// next answer is news, 'reachable again', though no question of the
    /// follower's went unanswered. Says nothing itself.
    pub fn note_unreachable(&mut self) {
        self.missed_write = true;
    }

    /// The server stopped answering or answers again, or the sign-in ended
    /// or was made again, since the last call.
    pub fn take_reach(&mut self) -> Option<Reach> {
        self.told.take()
    }

    /// Take an answer that resolved as it was asked.
    fn tick_answer(&mut self) -> Option<Moved> {
        let request = self.request.as_mut()?;
        let mut cx = Context::from_waker(Waker::noop());
        let Poll::Ready(answer) = request.as_mut().poll(&mut cx) else { return None };
        self.request = None;
        self.reached(answer.as_ref().map(|_| ()).map_err(String::as_str));
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

