//! The command queue: the channel between a host and the app.
//!
//! A host calls [`AutomationQueue::submit`] with an [`Envelope`] and gets a
//! receiver for the [`Reply`]. The app drains the queue at three fixed points
//! in the frame (§4.4): [`drain_input`](AutomationQueue::drain_input) in
//! `raw_input_hook`, then [`drain_app`](AutomationQueue::drain_app) for
//! `Phase::Mutate` at the top of `ui` and for `Phase::Read` at the bottom. A
//! command waits in the queue until its phase comes round.
use crate::automation::command::{lookup, Ctx, Envelope, Handler, Notice, NoticeKind, Outcome, Phase, Reply};
use crate::automation::pointer::Pointer;
use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};

pub struct Pending {
    pub envelope: Envelope,
    pub reply: Sender<Reply>,
}

struct Awaiting {
    token: u64,
    region: crate::automation::cmd_capture::Region,
    pending: Pending,
}

#[derive(Default)]
struct Inner {
    pending: VecDeque<Pending>,
    awaiting: Vec<Awaiting>,
    notices: Vec<Notice>,
    /// How many of `notices` have already ridden out on a command REPLY.
    /// See [`Inner::reply_notices`].
    notices_replied: usize,
    pointer: Pointer,
    next_token: u64,
    frame: u64,
    poisoned: Option<String>,
}

/// Most notices kept. The app pushes one per toast whether or not a host is
/// listening, so an ordinary session (the `automation` feature is on by
/// default) must not grow a buffer for a reader that never arrives; the oldest
/// go first, like the toast overlay's own cap.
const MAX_NOTICES: usize = 256;

impl Inner {
    /// The notices a command REPLY carries: the ones that have not gone out on
    /// an earlier reply.
    ///
    /// Two readers share this one buffer, and each must see every notice once.
    /// A reply piggy-backs the new ones so an agent reads a refusal in the
    /// answer to the call that caused it; the `notices` COMMAND drains the whole
    /// buffer on its own cursor, so "every notice since the last drain" stays
    /// true no matter how many other calls went by in between. Taking the buffer
    /// for the replies — which is what this used to do — made `notices` report
    /// whatever happened to arrive after the last call, which for a script that
    /// waits before reading is nothing at all.
    fn reply_notices(&mut self) -> Vec<Notice> {
        let from = self.notices_replied.min(self.notices.len());
        self.notices_replied = self.notices.len();
        self.notices[from..].to_vec()
    }
}

#[derive(Default)]
pub struct AutomationQueue {
    inner: Mutex<Inner>,
    ctx: Mutex<Option<egui::Context>>,
}

impl AutomationQueue {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Give the queue the egui context so a submit can wake an idle frame loop.
    pub fn attach(&self, ctx: &egui::Context) {
        *self.ctx.lock().unwrap() = Some(ctx.clone());
    }

    /// Enqueue a command. The reply arrives on the returned receiver once the
    /// app has reached the command's phase (or at once, for an unknown command
    /// or a poisoned session).
    pub fn submit(&self, envelope: Envelope) -> Receiver<Reply> {
        let (tx, rx) = channel();
        let mut inner = self.inner.lock().unwrap();
        if let Some(why) = &inner.poisoned {
            let _ = tx.send(Reply::err(envelope.id, inner.frame, format!("session poisoned: {why}"), vec![]));
            return rx;
        }
        if lookup(&envelope.cmd).is_none() {
            let _ = tx.send(Reply::err(envelope.id, inner.frame, format!("unknown command `{}`", envelope.cmd), vec![]));
            return rx;
        }
        inner.pending.push_back(Pending { envelope, reply: tx });
        drop(inner);
        if let Some(ctx) = self.ctx.lock().unwrap().as_ref() {
            ctx.request_repaint();
        }
        rx
    }

    /// Record something the session should know about — a toast the user would
    /// have seen, a refusal, a panic. The app's own toast lane feeds this every
    /// frame (`BrepApp::ui`), so `notices` reports what was on screen.
    pub fn push_notice(&self, kind: NoticeKind, text: impl Into<String>) {
        let mut inner = self.inner.lock().unwrap();
        let frame = inner.frame;
        inner.notices.push(Notice { kind, frame, text: text.into() });
        let overflow = inner.notices.len().saturating_sub(MAX_NOTICES);
        if overflow > 0 {
            inner.notices.drain(0..overflow);
            inner.notices_replied = inner.notices_replied.saturating_sub(overflow);
        }
    }

    /// Drain for the `notices` COMMAND: everything since the last drain. The
    /// reply cursor goes back to the start with them — the buffer is empty, so
    /// the reply this drain rides out on carries nothing twice.
    pub fn take_notices(&self) -> Vec<Notice> {
        let mut inner = self.inner.lock().unwrap();
        inner.notices_replied = 0;
        std::mem::take(&mut inner.notices)
    }

    pub fn pointer(&self) -> Pointer {
        self.inner.lock().unwrap().pointer.clone()
    }

    pub fn frame(&self) -> u64 {
        self.inner.lock().unwrap().frame
    }

    pub fn has_pending(&self) -> bool {
        let inner = self.inner.lock().unwrap();
        !inner.pending.is_empty() || !inner.awaiting.is_empty()
    }

    /// Mark the session unusable (a panic unwound through the app). Every
    /// pending and future command is answered with the reason.
    pub fn poison(&self, why: impl Into<String>) {
        let why = why.into();
        let mut inner = self.inner.lock().unwrap();
        inner.poisoned = Some(why.clone());
        let frame = inner.frame;
        for p in inner.pending.drain(..) {
            let _ = p.reply.send(Reply::err(p.envelope.id, frame, format!("session poisoned: {why}"), vec![]));
        }
        for a in inner.awaiting.drain(..) {
            let _ = a.pending.reply.send(Reply::err(a.pending.envelope.id, frame, format!("session poisoned: {why}"), vec![]));
        }
    }

    pub fn is_poisoned(&self) -> bool {
        self.inner.lock().unwrap().poisoned.is_some()
    }

    /// Phase 1 of the frame: apply input commands as `egui::Event`s and
    /// complete captures whose `Event::Screenshot` arrived. `view` is the
    /// viewport rect of the last frame (for `region: viewport` crops).
    /// `ctx` is the app's: the pointer reads egui's clock from it (the
    /// headless host leaves `raw.time` unset, and egui adds `predicted_dt`),
    /// and a `click_gap` resets its click history.
    pub fn drain_input(&self, raw: &mut egui::RawInput, frame: u64, ppp: f32, view: Option<egui::Rect>, ctx: &egui::Context) {
        let mut inner = self.inner.lock().unwrap();
        inner.frame = frame;
        inner.pointer.now = raw.time.unwrap_or_else(|| ctx.input(|i| i.time) + f64::from(raw.predicted_dt));

        // Screenshots that landed in this frame's input.
        if !inner.awaiting.is_empty() {
            let mut done = Vec::new();
            for event in &raw.events {
                if let egui::Event::Screenshot { user_data, image, .. } = event {
                    let token = user_data.data.as_ref().and_then(|d| d.downcast_ref::<u64>()).copied();
                    if let Some(token) = token {
                        if let Some(pos) = inner.awaiting.iter().position(|a| a.token == token) {
                            let a = inner.awaiting.remove(pos);
                            done.push((a, image.clone()));
                        }
                    }
                }
            }
            for (a, image) in done {
                let notices = inner.reply_notices();
                let reply = match crate::automation::cmd_capture::encode_capture(&image, ppp, view, &a.region) {
                    Ok((json, png)) => {
                        let mut r = Reply::ok(a.pending.envelope.id, frame, json, notices);
                        r.blob = Some(png);
                        r
                    }
                    Err(e) => Reply::err(a.pending.envelope.id, frame, e, notices),
                };
                let _ = a.pending.reply.send(reply);
            }
        }

        // Input-phase commands, in order.
        let mut keep = VecDeque::new();
        let mut events = Vec::new();
        while let Some(p) = inner.pending.pop_front() {
            let Some(spec) = lookup(&p.envelope.cmd) else { continue };
            match (&spec.handler, spec.phase) {
                (Handler::Input(f), Phase::Input) => {
                    let result = f(&mut inner.pointer, p.envelope.args.clone(), &mut events);
                    let notices = inner.reply_notices();
                    let reply = match result {
                        Ok(v) => Reply::ok(p.envelope.id, frame, v, notices),
                        Err(e) => Reply::err(p.envelope.id, frame, e, notices),
                    };
                    let _ = p.reply.send(reply);
                }
                _ => keep.push_back(p),
            }
        }
        inner.pending = keep;
        // A `click_gap` this frame: a fresh `PointerState` before the pass
        // begins, so egui has no click to count the next one against. The
        // pass builds on the state this replaces (`InputState::begin_pass`).
        if std::mem::take(&mut inner.pointer.reset_clicks) {
            ctx.input_mut(|i| i.pointer = egui::PointerState::default());
        }
        raw.modifiers = inner.pointer.modifiers.egui();
        raw.events.extend(events);
    }

    /// Phases 2 and 3 of the frame. Runs every pending command registered for
    /// `phase`, in order.
    pub fn drain_app(&self, phase: Phase, ctx: &mut Ctx<'_>, frame: u64) {
        let batch: Vec<Pending> = {
            let mut inner = self.inner.lock().unwrap();
            inner.frame = frame;
            let mut keep = VecDeque::new();
            let mut batch = Vec::new();
            while let Some(p) = inner.pending.pop_front() {
                match lookup(&p.envelope.cmd) {
                    Some(spec) if spec.phase == phase => batch.push(p),
                    _ => keep.push_back(p),
                }
            }
            inner.pending = keep;
            batch
        };
        for p in batch {
            let spec = lookup(&p.envelope.cmd).expect("checked at submit");
            let Handler::App(f) = &spec.handler else { continue };
            let before = refused_counts(&ctx.app.docs);
            let result = f(ctx, p.envelope.args.clone());
            let result = refused_by_lock(&ctx.app.docs, &before, result);
            let mut inner = self.inner.lock().unwrap();
            match result {
                Ok(Outcome::Done(v)) => {
                    let notices = inner.reply_notices();
                    let _ = p.reply.send(Reply::ok(p.envelope.id, frame, v, notices));
                }
                Ok(Outcome::Blob { json, bytes }) => {
                    let notices = inner.reply_notices();
                    let mut r = Reply::ok(p.envelope.id, frame, json, notices);
                    r.blob = Some(bytes);
                    let _ = p.reply.send(r);
                }
                // No reply yet — the capture answers when its image arrives, and
                // takes the notices then (the cursor must not move here).
                Ok(Outcome::AwaitScreenshot { token, region }) => {
                    inner.awaiting.push(Awaiting { token, region, pending: p });
                }
                Err(e) => {
                    let notices = inner.reply_notices();
                    let _ = p.reply.send(Reply::err(p.envelope.id, frame, e, notices));
                }
            }
        }
    }

    /// A fresh token for a screenshot request.
    pub fn next_token(&self) -> u64 {
        let mut inner = self.inner.lock().unwrap();
        inner.next_token += 1;
        inner.next_token
    }
}

/// Each open document's refused-edit count ([`crate::document::Document::access`]).
fn refused_counts(docs: &crate::document::Documents) -> Vec<(u64, u64)> {
    docs.iter().map(|d| (d.id(), d.engine.history.refused_user_edits())).collect()
}

/// A command that tried to change a read-only document (a PLM revision this
/// user has not checked out, or a released one) did not change it — the
/// document's history refused the write — so its reply is that refusal, not
/// a success. A command that only reads, moves the camera, opens or closes a
/// document refuses nothing and answers as it did.
fn refused_by_lock(
    docs: &crate::document::Documents,
    before: &[(u64, u64)],
    result: Result<Outcome, String>,
) -> Result<Outcome, String> {
    if result.is_err() {
        return result;
    }
    for doc in docs.iter() {
        let was = before.iter().find(|(id, _)| *id == doc.id()).map(|(_, n)| *n);
        if was.is_some_and(|was| doc.engine.history.refused_user_edits() > was) {
            if let crate::document::Access::ReadOnly { reason } = doc.access() {
                return Err(format!("{} is read-only: {reason}", doc.title()));
            }
        }
    }
    result
}

