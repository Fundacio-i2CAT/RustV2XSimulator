//! [`Scheduler`] — the DES event loop.
//!
//! Mirrors `pyv2xsimulator/src/simulator/scheduler.py`.
//!
//! Events are stored in a sorted structure keyed by their fire time.
//! `start_simulation` pops the earliest event and runs it until the queue
//! is empty.

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use ordered_float::OrderedFloat;

#[allow(unused_imports)]
use log::{debug, info, trace, warn};

use super::event::{Event, EventHandle};

// ---------------------------------------------------------------------------
// SchedulerQueue — the inner mutable state shared with EventHandles
// ---------------------------------------------------------------------------

/// Internal event queue.  Wrapped in `Arc<Mutex<>>` so that [`EventHandle`]s
/// can insert follow-up events while the scheduler is running.
pub struct SchedulerQueue {
    /// Events keyed by fire time.  Multiple events may share the same time;
    /// they are stored in a `Vec` and processed in insertion order.
    map: BTreeMap<OrderedFloat<f64>, Vec<Event>>,
}

impl SchedulerQueue {
    pub fn new() -> Self {
        SchedulerQueue {
            map: BTreeMap::new(),
        }
    }

    /// Insert a new event into the queue.
    pub fn push(&mut self, event: Event) {
        self.map
            .entry(OrderedFloat(event.fire_time))
            .or_default()
            .push(event);
    }

    /// Pop the single earliest event, or `None` if the queue is empty.
    pub fn pop_earliest(&mut self) -> Option<Event> {
        if let Some((&key, _)) = self.map.iter().next() {
            let bucket = self.map.get_mut(&key).unwrap();
            let event = bucket.remove(0);
            if bucket.is_empty() {
                self.map.remove(&key);
            }
            Some(event)
        } else {
            None
        }
    }

    /// Returns `true` if there are no pending events.
    pub fn is_empty(&self) -> bool {
        self.map.is_empty()
    }

    /// Number of pending events across all time slots.
    pub fn len(&self) -> usize {
        self.map.values().map(|v| v.len()).sum()
    }
}

impl Default for SchedulerQueue {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Scheduler
// ---------------------------------------------------------------------------

/// Discrete-Event Simulation scheduler.
///
/// # Usage
/// ```no_run
/// use rustv2xsimulator::scheduler::{Scheduler, Event};
///
/// let mut scheduler = Scheduler::new();
///
/// // Schedule an event 1.0 s from the start.
/// scheduler.schedule_event(Event::new(1.0, |handle| {
///     println!("t = {:.3}", handle.get_current_simulation_time());
/// }));
///
/// scheduler.start_simulation(0.0);
/// ```
pub struct Scheduler {
    /// Shared event queue — also handed out inside [`EventHandle`]s.
    queue: Arc<Mutex<SchedulerQueue>>,
    /// Current simulation time (set by `start_simulation`).
    pub simulation_time: f64,
}

impl Scheduler {
    /// Create a new, empty scheduler.
    pub fn new() -> Self {
        Scheduler {
            queue: Arc::new(Mutex::new(SchedulerQueue::new())),
            simulation_time: 0.0,
        }
    }

    /// Add an event to the queue.
    ///
    /// Mirrors `Scheduler.schedule_event(event)` in Python.
    pub fn schedule_event(&mut self, event: Event) {
        self.queue.lock().unwrap().push(event);
    }

    /// Pop and run the earliest pending event.
    ///
    /// Mirrors `Scheduler.run_next_event()` in Python.
    pub fn run_next_event(&mut self) {
        let event = {
            let mut q = self.queue.lock().unwrap();
            q.pop_earliest()
        };
        if let Some(ev) = event {
            let t = ev.fire_time;
            trace!("[scheduler] running event t={:.4}", t);
            let wall = Instant::now();
            let handle = EventHandle::new(t, Arc::clone(&self.queue));
            ev.run(&handle);
            let elapsed_ms = wall.elapsed().as_secs_f64() * 1000.0;
            if elapsed_ms > 200.0 {
                warn!("[scheduler] event t={:.4} took {:.1} ms (wall-clock)", t, elapsed_ms);
            } else {
                debug!("[scheduler] event t={:.4} done in {:.1} ms", t, elapsed_ms);
            }
        }
    }

    /// Run all pending events in time order until the queue is empty.
    ///
    /// Mirrors `Scheduler.start_simulation(initial_simulation_time)`.
    pub fn start_simulation(&mut self, initial_simulation_time: f64) {
        self.simulation_time = initial_simulation_time;
        let total = self.queue.lock().unwrap().len();
        info!("[scheduler] starting DES loop — {} events queued", total);
        let wall_start = Instant::now();
        let mut n = 0usize;
        loop {
            let is_empty = self.queue.lock().unwrap().is_empty();
            if is_empty {
                break;
            }
            self.run_next_event();
            n += 1;
        }
        info!(
            "[scheduler] DES loop finished — {} events processed in {:.2} s wall-clock",
            n,
            wall_start.elapsed().as_secs_f64()
        );
        println!("Simulation finished.");
    }

    /// Returns the number of pending events.
    pub fn pending_events(&self) -> usize {
        self.queue.lock().unwrap().len()
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}
