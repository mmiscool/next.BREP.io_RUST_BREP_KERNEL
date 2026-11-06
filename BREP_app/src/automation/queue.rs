//! The command queue: the channel between a host and the app.
//!
//! A host calls [`AutomationQueue::submit`] with an [`Envelope`] and gets a
//! receiver for the [`Reply`]. The app drains the queue at three fixed points
//! in the frame (§4.4): [`drain_input`](AutomationQueue::drain_input) in
//! `raw_input_hook`, then [`drain_app`](AutomationQueue::drain_app) for
//! `Phase::Mutate` at the top of `ui` and for `Phase::Read` at the bottom. A
//! command waits in the queue until its phase comes round.
//!
//! A desktop window is not always drawn: a Wayland compositor sends no frame
//! callbacks to a window on a hidden workspace, and winit then withholds
//! `RedrawRequested` indefinitely. So no command waits for one. A submit calls
//! the [waker](AutomationQueue::set_waker), which wakes the desktop event
//! loop independently of egui's repaint bookkeeping, and the loop dispatches
//! the queue's head with [`drain_frameless`](AutomationQueue::drain_frameless)
//! (`desktop_loop`). What a command needs first is its [`Need`]: app state
//! (dispatched at once), a current layout (dispatched at once while the last
//! pass is current, else after one), or a pass of its own (input egui must
//! apply, a capture of what a pass draws). For the last two the loop runs an
//! egui pass itself, on demand, without presenting it
//! ([`needs_pass`](AutomationQueue::needs_pass)); the pass drains the queue
//! exactly as a drawn frame does, so the queue's order is kept.
use crate::automation::command::{lookup, Ctx, Envelope, Handler, Notice, NoticeKind, Outcome, Phase, Reply};
use crate::automation::pointer::Pointer;
use std::collections::VecDeque;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex};

pub struct Pending {
    pub envelope: Envelope,
    pub reply: Sender<Reply>,
    deadline: Option<std::time::Instant>,
}

impl Pending {
    fn expired(&self, frame: u64) -> bool {
        if self.deadline.is_some_and(|d| std::time::Instant::now() >= d) {
            let _ = self.reply.send(Reply::err(self.envelope.id, frame,
                "host_deadline_expired: command expired before dispatch; no effects", vec![]));
            true
        } else { false }
    }
}

struct Awaiting {
    token: u64,
    region: crate::automation::cmd_capture::Region,
    document: Option<String>,
    pending: Pending,
}

struct Polling {
    pending: Pending,
    phase: Phase,
    poller: crate::automation::command::Poller,
    /// The document handle the command was bound to by its context: the
    /// eventual reply is tagged with that document's context, as an
    /// immediate one is.
    bound: Option<String>,
}

#[derive(Default)]
struct Inner {
    pending: VecDeque<Pending>,
    awaiting: Vec<Awaiting>,
    polling: Vec<Polling>,
    notices: Vec<Notice>,
    /// How many of `notices` have already ridden out on a command REPLY.
    /// See [`Inner::reply_notices`].
    notices_replied: usize,
    pointer: Pointer,
    next_token: u64,
    frame: u64,
    poisoned: Option<String>,
    /// The document the last drawn frame showed: `(handle, committed revision)`
    /// ([`AutomationQueue::publish_view`]), which a capture landing now is of.
    view: Option<(String, u64)>,
    /// When the last pass (drawn or on demand) published its layout.
    layout_at: Option<web_time::Instant>,
    /// Something may have changed the layout since that pass: a command that
    /// writes, work that finished, a resize.
    layout_dirty: bool,
    /// The pass now running applied input: what the input changed is laid
    /// out by the pass after it (egui draws once more after input), so this
    /// pass's layout is not the current one for long.
    input_this_pass: bool,
}

impl Inner {
    /// The last pass's layout still stands ([`Need::Layout`]).
    fn layout_current(&self) -> bool {
        !self.layout_dirty && self.layout_at.is_some_and(|at| at.elapsed() < LAYOUT_FRESH)
    }

    /// What the queue's head needs, if there is one.
    fn front_need(&self) -> Option<Need> {
        let p = self.pending.front()?;
        Some(lookup(&p.envelope.cmd).map_or(Need::State, |spec| need(spec, &p.envelope.args)))
    }

    /// The head can be dispatched now, without a pass.
    fn front_dispatchable(&self) -> bool {
        match self.front_need() {
            Some(Need::State) => true,
            Some(Need::Layout) => self.layout_current(),
            _ => false,
        }
    }
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

/// Wakes the thread that owns the app when work is queued.
pub type Waker = Arc<dyn Fn() + Send + Sync>;

/// What a command needs before it can be dispatched outside a drawn pass.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Need {
    /// Nothing: it reads or writes app state, which a pass has no part in.
    State,
    /// A current layout: what only a pass publishes (widget rects, the
    /// viewport rect, focus, the dock's rendered panes). Dispatched at once
    /// while the last pass is current ([`LAYOUT_FRESH`], nothing changed
    /// since), else after one.
    Layout,
    /// A pass of its own: input egui must apply in a pass, or a capture of
    /// what a pass draws.
    Pass,
}

/// App commands that need a pass of their own: the captures (the image IS a
/// pass), the presentation loan a capture series runs under, and `ping`, which
/// answers from the next pass (the composite tools step the app with it).
pub const PASS_BOUND: &[&str] = &["screenshot", "illustration_presentation", "ping"];

/// App commands that read the layout a pass publishes. `state_get` and
/// `state_schema` are here only for a name that is not a plain-state blob
/// (`automation::state_blobs`): those are built from the app at dispatch.
pub const LAYOUT_BOUND: &[&str] = &["hit_rects", "state_list", "state_get", "state_schema", "locate", "pick"];

/// How long the last pass's layout stands for a layout read when nothing has
/// changed since: one frame at 60 Hz, the age a drawn window's layout has.
/// Older than that, time alone may have moved it (a toast expiring, the
/// working indicator appearing), so the read gets a pass of its own.
pub const LAYOUT_FRESH: std::time::Duration = std::time::Duration::from_millis(16);

/// What `spec` called with `args` needs ([`Need`]).
pub fn need(spec: &crate::automation::command::CommandSpec, args: &serde_json::Value) -> Need {
    match spec.handler {
        // A read-only input command (`pointer_state`, which `session_info`
        // issues) emits no events and only reads the virtual pointer; every
        // input queued before it has already been applied to that pointer.
        Handler::Input(_) if spec.phase == Phase::Input && spec.annotations.read_only => Need::State,
        Handler::Input(_) => Need::Pass,
        Handler::App(_) if spec.phase == Phase::Input || PASS_BOUND.contains(&spec.name) => Need::Pass,
        Handler::App(_) if matches!(spec.name, "state_get" | "state_schema") => {
            let names = args.get("names").and_then(serde_json::Value::as_array);
            let state = names.is_some_and(|names| {
                names.iter().all(|n| n.as_str().is_some_and(crate::automation::state_blobs::is_state_blob))
            });
            if state { Need::State } else { Need::Layout }
        }
        Handler::App(_) if LAYOUT_BOUND.contains(&spec.name) => Need::Layout,
        Handler::App(_) => Need::State,
    }
}

/// What became of a command whose host stopped waiting for it
/// ([`AutomationQueue::withdraw`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Withdrawn {
    /// It was still queued and has been removed: it never ran.
    NotDispatched,
    /// A handler accepted it (it is capturing or polling), or it already
    /// answered: it may have had effects.
    Accepted,
}

#[derive(Default)]
pub struct AutomationQueue {
    inner: Mutex<Inner>,
    ctx: Mutex<Option<egui::Context>>,
    waker: Mutex<Option<Waker>>,
}

impl AutomationQueue {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Give the queue the egui context so a submit can wake an idle frame loop.
    pub fn attach(&self, ctx: &egui::Context) {
        *self.ctx.lock().unwrap() = Some(ctx.clone());
        if self.has_pending() { ctx.request_repaint(); }
    }

    /// Install the app thread's waker (the desktop event loop's proxy). Work
    /// queued before it was installed is not stranded: the waker fires at once.
    pub fn set_waker(&self, waker: Waker) {
        *self.waker.lock().unwrap() = Some(waker.clone());
        if self.has_pending() { waker(); }
    }

    fn wake(&self) {
        let waker = self.waker.lock().unwrap().clone();
        if let Some(waker) = waker { waker(); }
        if let Some(ctx) = self.ctx.lock().unwrap().as_ref() {
            ctx.request_repaint();
        }
    }

    /// Enqueue a command. The reply arrives on the returned receiver once the
    /// app has reached the command's phase (or at once, for an unknown command
    /// or a poisoned session).
    pub fn submit(&self, envelope: Envelope) -> Receiver<Reply> {
        self.submit_before(envelope, None)
    }

    /// A deadline applies before dispatch, never to work already accepted by a handler.
    pub fn submit_before(&self, envelope: Envelope, deadline: Option<std::time::Instant>) -> Receiver<Reply> {
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
        inner.pending.push_back(Pending { envelope, reply: tx, deadline });
        // Notify only after the command is visible to the app thread: a wake
        // that arrives first finds it queued.
        drop(inner);
        self.wake();
        rx
    }

    /// The host stopped waiting for command `id`. A command still queued is
    /// removed and answered as never dispatched; anything else was accepted.
    pub fn withdraw(&self, id: u64) -> Withdrawn {
        let mut inner = self.inner.lock().unwrap();
        let frame = inner.frame;
        if let Some(index) = inner.pending.iter().position(|p| p.envelope.id == id) {
            let p = inner.pending.remove(index).expect("index from position");
            let _ = p.reply.send(Reply::err(id, frame, "host_deadline_expired: command expired before dispatch; no effects", vec![]));
            return Withdrawn::NotDispatched;
        }
        Withdrawn::Accepted
    }

    /// Whether [`drain_frameless`](Self::drain_frameless) has anything to do:
    /// a head it can dispatch without a pass, or polled work.
    pub fn has_frameless_work(&self) -> bool {
        let inner = self.inner.lock().unwrap();
        !inner.polling.is_empty() || inner.front_dispatchable()
    }

    /// Whether the queue's head waits for a pass: input, a capture, or a
    /// layout read with no current layout. The desktop loop runs one on demand.
    pub fn needs_pass(&self) -> bool {
        let inner = self.inner.lock().unwrap();
        inner.front_need().is_some() && !inner.front_dispatchable()
    }

    /// The layout may have changed outside a command (a resize, finished
    /// background work): the next layout read gets a pass.
    pub fn mark_layout_dirty(&self) {
        self.inner.lock().unwrap().layout_dirty = true;
    }

    /// A pass (drawn or on demand) has just published its layout and answered
    /// its reads (`BrepApp::drain_reads`).
    pub fn note_pass(&self) {
        let mut inner = self.inner.lock().unwrap();
        inner.layout_at = Some(web_time::Instant::now());
        inner.layout_dirty = std::mem::take(&mut inner.input_this_pass);
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
        !inner.pending.is_empty() || !inner.awaiting.is_empty() || !inner.polling.is_empty()
    }

    /// Whether a command has started work the queue is polling (see
    /// `Outcome::Poll`): a host keeps stepping frames for it past its usual
    /// frame budget, since the work is advancing, not stuck.
    pub fn is_polling(&self, id: u64) -> bool {
        self.inner.lock().unwrap().polling.iter().any(|p| p.pending.envelope.id == id)
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
                let shown = inner.view.clone();
                // The image is of the frame drawn last: it shows `shown`. A
                // capture asked for while another document was visible is
                // refused rather than handed back as that document's picture.
                let switched = match (&a.document, &shown) {
                    (Some(asked), Some((drawn, _))) => asked != drawn,
                    _ => false,
                };
                let result = if switched {
                    Err("document_not_active: the visible document changed between the request and the capture".to_string())
                } else {
                    crate::automation::cmd_capture::encode_capture(&image, ppp, view, &a.region).map(|(mut json, png)| {
                        if let Some((handle, revision)) = &shown {
                            json["document"] = serde_json::json!({ "documentId": handle, "revision": revision.to_string() });
                        }
                        (json, png)
                    })
                };
                let reply = match result {
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
            if p.expired(frame) { continue; }
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
        inner.input_this_pass = !events.is_empty();
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
            self.dispatch(p, phase, ctx, frame);
        }
        if self.poll(phase, ctx, frame) {
            ctx.egui.request_repaint();
        }
    }

    /// Dispatch without a pass (the desktop loop's service step): the
    /// queue's head, in FIFO order, up to the first command that needs a pass
    /// ([`needs_pass`](Self::needs_pass), which the loop then runs); then one
    /// poll of the work they started. Returns whether polled work is still
    /// outstanding. Runs on the app's thread, so mutations stay sequential
    /// with the UI's own writers.
    pub fn drain_frameless(&self, ctx: &mut Ctx<'_>) -> bool {
        let frame = self.frame();
        loop {
            let next = {
                let mut inner = self.inner.lock().unwrap();
                let ready = inner.front_dispatchable();
                match inner.pending.front().and_then(|p| lookup(&p.envelope.cmd)) {
                    Some(spec) if ready => {
                        let p = inner.pending.pop_front().expect("front exists");
                        if let Handler::Input(f) = &spec.handler {
                            // A read-only input command: answered here, as the
                            // frame's input phase would answer it.
                            if !p.expired(frame) {
                                let mut events = Vec::new();
                                let result = f(&mut inner.pointer, p.envelope.args.clone(), &mut events);
                                debug_assert!(events.is_empty(), "{} is read-only but emitted input", spec.name);
                                let notices = inner.reply_notices();
                                let reply = match result {
                                    Ok(v) => Reply::ok(p.envelope.id, frame, v, notices),
                                    Err(e) => Reply::err(p.envelope.id, frame, e, notices),
                                };
                                let _ = p.reply.send(reply);
                            }
                            continue;
                        }
                        Some((p, spec.phase))
                    }
                    _ => None,
                }
            };
            let Some((p, phase)) = next else { break };
            self.dispatch(p, phase, ctx, frame);
        }
        let mutate = self.poll(Phase::Mutate, ctx, frame);
        let read = self.poll(Phase::Read, ctx, frame);
        mutate || read
    }

    /// Run one command's handler and answer (or park) it.
    fn dispatch(&self, p: Pending, phase: Phase, ctx: &mut Ctx<'_>, frame: u64) {
        if p.expired(frame) { return; }
        let spec = lookup(&p.envelope.cmd).expect("checked at submit");
        let Handler::App(f) = &spec.handler else { return };
        if !spec.annotations.read_only {
            // What it writes may move the layout: the next layout read waits
            // for a pass.
            self.mark_layout_dirty();
        }
        // A document context is checked HERE, on the frame thread, right
        // before the handler: the only point atomic with the UI's own
        // writers. It is stripped from the handler's arguments.
        let (args, bound) = match bind_context(&ctx.app.docs, phase, &p.envelope.cmd, p.envelope.args.clone()) {
            Ok(bound) => bound,
            Err(e) => {
                let mut inner = self.inner.lock().unwrap();
                let notices = inner.reply_notices();
                let _ = p.reply.send(Reply::err(p.envelope.id, frame, e, notices));
                return;
            }
        };
        let before = refused_counts(&ctx.app.docs);
        let route = bound.as_ref().and_then(|b| b.route).map(|index| ctx.app.docs.route_begin(index));
        let result = f(ctx, args);
        if let Some(visible) = route {
            ctx.app.docs.route_end(visible);
        }
        let result = refused_by_lock(&ctx.app.docs, &before, result);
        let handle = bound.map(|b| b.handle);
        let result = match &handle {
            Some(handle) => tag_context(&ctx.app.docs, handle, result),
            None => result,
        };
        let mut inner = self.inner.lock().unwrap();
        answer(&mut inner, p, phase, frame, result, handle);
    }

    /// Work started earlier: one poll each, in this phase. Returns whether
    /// any is still outstanding.
    fn poll(&self, phase: Phase, ctx: &mut Ctx<'_>, frame: u64) -> bool {
        let polling: Vec<Polling> = {
            let mut inner = self.inner.lock().unwrap();
            let (mine, rest) = std::mem::take(&mut inner.polling).into_iter().partition(|p| p.phase == phase);
            inner.polling = rest;
            mine
        };
        let mut still = false;
        for mut entry in polling {
            match (entry.poller)(ctx) {
                None => {
                    still = true;
                    self.inner.lock().unwrap().polling.push(entry);
                }
                Some(result) => {
                    // Finished work may have changed what a pass would show.
                    self.mark_layout_dirty();
                    let result = match &entry.bound {
                        Some(handle) => tag_context(&ctx.app.docs, handle, result),
                        None => result,
                    };
                    let mut inner = self.inner.lock().unwrap();
                    answer(&mut inner, entry.pending, phase, frame, result, entry.bound);
                    still |= !inner.polling.is_empty();
                }
            }
        }
        still
    }

    /// Record which document the frame just drawn shows (called at the end of
    /// every app frame), so a capture is bound to what it pictures.
    pub fn publish_view(&self, handle: &str, revision: u64) {
        let mut inner = self.inner.lock().unwrap();
        if inner.view.as_ref().is_none_or(|(h, r)| h != handle || *r != revision) {
            inner.view = Some((handle.to_string(), revision));
        }
    }

    /// Answer every capture still waiting for its image with `error` (an
    /// on-demand pass could not draw it).
    pub fn fail_captures(&self, error: &str) {
        let mut inner = self.inner.lock().unwrap();
        let frame = inner.frame;
        for a in std::mem::take(&mut inner.awaiting) {
            let notices = inner.reply_notices();
            let _ = a.pending.reply.send(Reply::err(a.pending.envelope.id, frame, error.to_string(), notices));
        }
    }

    /// A fresh token for a screenshot request.
    pub fn next_token(&self) -> u64 {
        let mut inner = self.inner.lock().unwrap();
        inner.next_token += 1;
        inner.next_token
    }
}

/// The root `context` key of a command's arguments: which document the
/// command is for and the committed revision its writer last saw
/// (contract §3 `DocumentContext`). Optional — a command without one
/// acts on the visible document as it always has.
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct DocumentContext {
    document_id: String,
    expected_revision: String,
}

/// Commands that read or write only a document's ENGINE — its history, run,
/// metadata, model and exports — and so may run on a background tab: the
/// queue routes them there for the one handler call (`Documents::route_begin`)
/// without touching what the user sees. Every other command acts on the
/// visible document and refuses a background context (`document_not_active`).
pub const ENGINE_ONLY: &[&str] = &[
    "feature_stage",
    "feature_stage_result",
    "run_cancel",
    "history_group_begin",
    "history_group_end",
    "feature_add_many",
    "doc_import",
    "doc_export",
    "model_inspect",
    "mass_properties",
    "history_listing",
    "frame_info",
    "metadata_set",
    "metadata_set_many",
    "metadata_remove",
];

/// A command bound to a document by its context: the handle it named and,
/// for an engine-only command addressed to a background tab, that tab.
#[derive(Debug)]
struct Bound {
    handle: String,
    route: Option<usize>,
}

/// Take `context` out of `args` and check it against the documents as they
/// are now. Errors begin with the contract's stable code (`revision_conflict:
/// …`), so a caller need not parse the prose.
///
/// An [`ENGINE_ONLY`] command may name a background tab and is routed there;
/// any other command must name the visible one (`document_not_active`).
/// While a Work part is edited in place every context refuses
/// (`document_busy`): the command would write the hosted part, not the tab
/// named. A Mutate-phase command requires the expected revision to be the
/// current one; a Read-phase command only reports the current revision.
fn bind_context(
    docs: &crate::document::Documents,
    phase: Phase,
    cmd: &str,
    mut args: serde_json::Value,
) -> Result<(serde_json::Value, Option<Bound>), String> {
    let Some(raw) = args.as_object_mut().and_then(|o| o.remove("context")) else {
        return Ok((args, None));
    };
    // A command with no arguments still parses `{}` after the strip.
    let context: DocumentContext = serde_json::from_value(raw)
        .map_err(|e| format!("invalid_arguments: context must be {{documentId, expectedRevision}}: {e}"))?;
    let expected = parse_revision(&context.expected_revision)?;
    match crate::handles::parse("doc", &context.document_id) {
        crate::handles::Parsed::Current => {}
        crate::handles::Parsed::OtherEpoch => {
            return Err("handle_epoch_mismatch: the document handle is from another run of the app".into())
        }
        crate::handles::Parsed::Malformed => return Err("invalid_arguments: documentId is not a document handle".into()),
    }
    let Some(index) = docs.index_of_handle(&context.document_id) else {
        return Err("document_unknown: no open document has that handle (closed, or never opened)".into());
    };
    if docs.work_part_in_place() {
        return Err("document_busy: a Work part is edited in place; a document command would write the part".into());
    }
    let route = if index == docs.active_index() {
        None
    } else if ENGINE_ONLY.contains(&cmd) {
        Some(index)
    } else {
        return Err("document_not_active: this command acts on the visible document; activate the named one first".into());
    };
    let current = docs.get(index).map_or(0, |d| d.committed_revision());
    if phase == Phase::Mutate && current != expected {
        return Err(format!(
            "revision_conflict: the document is at revision {current}, not {expected}; read it again before writing"
        ));
    }
    Ok((args, Some(Bound { handle: context.document_id, route })))
}

/// A decimal unsigned 64-bit revision, canonical (no sign, no leading zero).
fn parse_revision(text: &str) -> Result<u64, String> {
    let canonical = text == "0" || (!text.is_empty() && !text.starts_with('0') && text.bytes().all(|b| b.is_ascii_digit()));
    text.parse::<u64>()
        .ok()
        .filter(|_| canonical)
        .ok_or_else(|| "invalid_arguments: expectedRevision must be a canonical unsigned decimal string".into())
}

/// A command that was bound to a document answers with that document's
/// context as it stands after the handler: `{documentId, expectedRevision}` under the
/// result's `context` key (an object result), so the writer's next
/// expectation comes from the answer, not from a guess. A geometry run the
/// handler started moves the revision again when it lands.
fn tag_context(
    docs: &crate::document::Documents,
    handle: &str,
    result: Result<Outcome, String>,
) -> Result<Outcome, String> {
    let context = docs.index_of_handle(handle).and_then(|i| docs.get(i)).map(|doc| {
        serde_json::json!({ "documentId": handle, "expectedRevision": doc.committed_revision().to_string() })
    });
    let tag = |v: &mut serde_json::Value| {
        if let (Some(object), Some(context)) = (v.as_object_mut(), context.clone()) {
            object.insert("context".into(), context);
        }
    };
    match result {
        Ok(Outcome::Done(mut v)) => {
            tag(&mut v);
            Ok(Outcome::Done(v))
        }
        Ok(Outcome::Blob { mut json, bytes }) => {
            tag(&mut json);
            Ok(Outcome::Blob { json, bytes })
        }
        other => other,
    }
}

/// Send (or park) one command's reply.
fn answer(inner: &mut Inner, p: Pending, phase: Phase, frame: u64, result: Result<Outcome, String>, bound: Option<String>) {
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
        Ok(Outcome::AwaitScreenshot { token, region, document }) => {
            inner.awaiting.push(Awaiting { token, region, document, pending: p });
        }
        Ok(Outcome::Poll(poller)) => inner.polling.push(Polling { pending: p, phase, poller, bound }),
        Err(e) => {
            let notices = inner.reply_notices();
            let _ = p.reply.send(Reply::err(p.envelope.id, frame, e, notices));
        }
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

