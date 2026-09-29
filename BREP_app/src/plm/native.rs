//! The native executor that drives the non-`Send` PLM and store futures
//! (plan S2).
//!
//! Why not tokio: the store seam's `BackendFuture` and [`super::PlmFuture`] are
//! deliberately not `Send` — the mirror shares `Rc<RefCell<…>>` state with the
//! futures it spawns, exactly as it does in the browser — so a multi-thread
//! runtime cannot hold them, and a `LocalSet` would need a thread of its own to
//! drive it, which the mirror's `Rc`s cannot cross to. What wasm has instead is
//! the page's microtask queue on the one UI thread, with `fetch` doing the I/O
//! elsewhere and waking it. This module is that shape natively:
//!
//! * [`spawn`] polls a task at once (a future that is already resolved — the
//!   test backends' — finishes right there, as it always did) and otherwise
//!   parks it on this thread's queue.
//! * The I/O runs on another thread (ehttp's worker, over ureq). Its waker is
//!   `Send`: it marks the task ready and calls the wake hook ([`set_wake`], the
//!   frame loop's `request_repaint`), so a response wakes the UI rather than
//!   waiting for the mouse to move.
//! * [`run_pending`] polls every ready task; the frame loop calls it once per
//!   frame, as the browser drains microtasks between events.
//! * [`block_on`] waits for one future on this thread while still running the
//!   queue, for the boot hydrate that must finish before the first frame.
//!
//! Nothing here starts a thread or allocates until something is spawned, so a
//! session with no PLM pays nothing for it.

use std::cell::RefCell;
use std::future::Future;
use std::pin::Pin;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::task::{Context, Poll, Wake, Waker};

type LocalTask = Pin<Box<dyn Future<Output = ()>>>;
type WakeHook = Arc<dyn Fn() + Send + Sync>;

/// The UI's "something finished, draw a frame" hook, reachable from the I/O
/// thread. Process-wide, because a waker fires on whatever thread did the I/O.
static WAKE_HOOK: Mutex<Option<WakeHook>> = Mutex::new(None);

thread_local! {
    /// Tasks spawned on this thread and not finished yet.
    static QUEUE: RefCell<Vec<(LocalTask, Arc<TaskWaker>)>> = const { RefCell::new(Vec::new()) };
}

/// Install the hook a completing task calls — `egui::Context::request_repaint`
/// in the app. Replaces any earlier one.
pub fn set_wake(hook: impl Fn() + Send + Sync + 'static) {
    *WAKE_HOOK.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(hook));
}

/// Call the wake hook now, from the UI thread — for a store that has news of
/// its own (a failure recorded, an external change folded in) rather than an
/// I/O answer. No hook installed is a no-op.
pub fn request_wake() {
    let hook = WAKE_HOOK.lock().unwrap_or_else(|p| p.into_inner()).clone();
    if let Some(hook) = hook {
        hook();
    }
}

struct TaskWaker {
    ready: AtomicBool,
    /// The thread [`block_on`] parks, when a blocking wait owns this task.
    thread: Option<std::thread::Thread>,
}

impl Wake for TaskWaker {
    fn wake(self: Arc<Self>) {
        self.wake_by_ref();
    }

    fn wake_by_ref(self: &Arc<Self>) {
        self.ready.store(true, Ordering::Release);
        if let Some(thread) = &self.thread {
            thread.unpark();
        }
        request_wake();
    }
}

/// Run `task` on this thread: polled once now, then again each time its waker
/// fires and [`run_pending`] (or a [`block_on`]) runs.
pub fn spawn(task: impl Future<Output = ()> + 'static) {
    let mut task: LocalTask = Box::pin(task);
    let waker = Arc::new(TaskWaker { ready: AtomicBool::new(false), thread: None });
    if poll_once(&mut task, &waker).is_pending() {
        QUEUE.with(|queue| queue.borrow_mut().push((task, waker)));
    }
}

fn poll_once(task: &mut LocalTask, waker: &Arc<TaskWaker>) -> Poll<()> {
    let waker_handle = Waker::from(waker.clone());
    let mut cx = Context::from_waker(&waker_handle);
    task.as_mut().poll(&mut cx)
}

/// Poll every task whose waker has fired, until a pass wakes none. Returns how
/// many tasks are still unfinished. Safe to call from inside a task: tasks are
/// taken off the queue while they are polled, and anything spawned meanwhile
/// joins the queue afterwards.
pub fn run_pending() -> usize {
    loop {
        let tasks = QUEUE.with(|queue| std::mem::take(&mut *queue.borrow_mut()));
        let mut progressed = false;
        let mut still = Vec::with_capacity(tasks.len());
        for (mut task, waker) in tasks {
            if waker.ready.swap(false, Ordering::AcqRel) {
                progressed = true;
                if poll_once(&mut task, &waker).is_ready() {
                    continue;
                }
            }
            still.push((task, waker));
        }
        let remaining = QUEUE.with(|queue| {
            let mut queue = queue.borrow_mut();
            // Tasks spawned during this pass were pushed while we held the
            // taken list; keep the older ones first so FIFO order holds.
            still.append(&mut queue);
            *queue = still;
            queue.len()
        });
        if !progressed {
            return remaining;
        }
    }
}

/// How many spawned tasks have not finished — zero means every write-behind
/// push issued so far has settled.
pub fn pending() -> usize {
    QUEUE.with(|queue| queue.borrow().len())
}

/// Drive `future` to completion on this thread, running the spawned queue
/// while it waits and sleeping between wakes. For tests, and for callers with
/// no deadline. Not to be called from inside a spawned task.
pub fn block_on<T>(future: impl Future<Output = T>) -> T {
    match wait(future, None) {
        Some(value) => value,
        None => unreachable!("a wait with no deadline only returns with the value"),
    }
}

/// [`block_on`], giving up after `limit`: `None` when the future had not
/// finished by then (it is dropped). The boot hydrate waits through this, so a
/// server that never answers delays the first frame by `limit` and no more.
pub fn block_on_for<T>(future: impl Future<Output = T>, limit: std::time::Duration) -> Option<T> {
    wait(future, Some(std::time::Instant::now() + limit))
}

fn wait<T>(future: impl Future<Output = T>, deadline: Option<std::time::Instant>) -> Option<T> {
    let mut future = std::pin::pin!(future);
    let waker = Arc::new(TaskWaker { ready: AtomicBool::new(true), thread: Some(std::thread::current()) });
    let waker_handle = Waker::from(waker.clone());
    let mut cx = Context::from_waker(&waker_handle);
    loop {
        if waker.ready.swap(false, Ordering::AcqRel) {
            if let Poll::Ready(value) = future.as_mut().poll(&mut cx) {
                return Some(value);
            }
        }
        run_pending();
        if deadline.is_some_and(|deadline| std::time::Instant::now() >= deadline) {
            return None;
        }
        if !waker.ready.load(Ordering::Acquire) {
            // A spawned task's waker does not unpark this thread, so wake up
            // now and then to run the queue even when only it progressed.
            std::thread::park_timeout(std::time::Duration::from_millis(5));
        }
    }
}



