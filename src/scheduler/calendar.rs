//! Calendar queue implementation for discrete-event simulation.
//!
//! Maintains strict temporal monotonicity with deterministic FIFO tie-breaking
//! for concurrent events at the same tick.

use crate::common::SimTime;
use crate::scheduler::event::{ActorId, Event, EventPayload};
use std::collections::BinaryHeap;

#[derive(Debug, Default)]
pub struct CalendarQueue {
    heap: BinaryHeap<Event>,
    next_event_id: u64,
    next_sequence: u64,
}

impl CalendarQueue {
    pub fn new() -> Self {
        Self {
            heap: BinaryHeap::new(),
            next_event_id: 1,
            next_sequence: 1,
        }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            heap: BinaryHeap::with_capacity(capacity),
            next_event_id: 1,
            next_sequence: 1,
        }
    }

    /// Schedule an event in the calendar queue.
    pub fn schedule(
        &mut self,
        timestamp: SimTime,
        priority: u8,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> u64 {
        let id = self.next_event_id;
        self.next_event_id += 1;
        let sequence = self.next_sequence;
        self.next_sequence += 1;

        let event = Event::new(id, timestamp, priority, sequence, actor_id, payload);
        self.heap.push(event);
        id
    }

    /// Push an already-constructed event.
    #[inline]
    pub fn push(&mut self, event: Event) {
        self.heap.push(event);
    }

    /// Pop the earliest event from the calendar queue.
    #[inline]
    pub fn pop(&mut self) -> Option<Event> {
        self.heap.pop()
    }

    /// Peek at the earliest event in the calendar queue.
    #[inline]
    pub fn peek(&self) -> Option<&Event> {
        self.heap.peek()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.heap.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    #[inline]
    pub fn clear(&mut self) {
        self.heap.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_calendar_ordering_and_fifo() {
        let mut cal = CalendarQueue::new();
        let t1 = SimTime::from_millis(10);
        let t2 = SimTime::from_millis(20);

        let id1 = cal.schedule(t2, 0, 1, EventPayload::Timer { token: 1 });
        let id2 = cal.schedule(t1, 0, 2, EventPayload::Timer { token: 2 });
        let id3 = cal.schedule(t1, 0, 3, EventPayload::Timer { token: 3 });

        // id2 and id3 have same SimTime t1, so id2 must pop before id3 (FIFO tie-break)
        let e1 = cal.pop().expect("event");
        assert_eq!(e1.id, id2);
        assert_eq!(e1.timestamp, t1);

        let e2 = cal.pop().expect("event");
        assert_eq!(e2.id, id3);
        assert_eq!(e2.timestamp, t1);

        let e3 = cal.pop().expect("event");
        assert_eq!(e3.id, id1);
        assert_eq!(e3.timestamp, t2);

        assert!(cal.is_empty());
    }
}
