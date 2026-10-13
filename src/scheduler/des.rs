//! Discrete-Event Simulation (DES) Core Scheduler.
//!
//! Provides strict causality verification (zero retrograde events),
//! event dispatch loop, and event execution context.

use crate::common::{SimError, SimResult, SimTime};
use crate::scheduler::calendar::CalendarQueue;
use crate::scheduler::event::{ActorId, Event, EventPayload};

/// Execution context provided to event handlers during step execution.
pub struct EventContext<'a> {
    pub current_time: SimTime,
    pub event_id: u64,
    pub actor_id: ActorId,
    scheduler: &'a mut DiscreteEventScheduler,
}

impl<'a> EventContext<'a> {
    /// Schedule a future event from within an event handler.
    pub fn schedule(
        &mut self,
        timestamp: SimTime,
        priority: u8,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> SimResult<u64> {
        self.scheduler
            .schedule(timestamp, priority, actor_id, payload)
    }

    /// Schedule an event with a relative delay from current simulation time.
    pub fn schedule_relative(
        &mut self,
        delay: SimTime,
        priority: u8,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> SimResult<u64> {
        let target_time = self.current_time + delay;
        self.scheduler
            .schedule(target_time, priority, actor_id, payload)
    }
}

/// The core Discrete-Event Scheduler engine.
#[derive(Debug)]
pub struct DiscreteEventScheduler {
    current_time: SimTime,
    calendar: CalendarQueue,
    events_processed: u64,
    max_duration: Option<SimTime>,
}

impl Default for DiscreteEventScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl DiscreteEventScheduler {
    pub fn new() -> Self {
        Self {
            current_time: SimTime::ZERO,
            calendar: CalendarQueue::new(),
            events_processed: 0,
            max_duration: None,
        }
    }

    pub fn with_duration(max_duration: SimTime) -> Self {
        Self {
            current_time: SimTime::ZERO,
            calendar: CalendarQueue::new(),
            events_processed: 0,
            max_duration: Some(max_duration),
        }
    }

    #[inline]
    pub fn current_time(&self) -> SimTime {
        self.current_time
    }

    #[inline]
    pub fn events_processed(&self) -> u64 {
        self.events_processed
    }

    #[inline]
    pub fn pending_events(&self) -> usize {
        self.calendar.len()
    }

    /// Schedule a new event with compile-time & runtime causality verification.
    pub fn schedule(
        &mut self,
        timestamp: SimTime,
        priority: u8,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> SimResult<u64> {
        if timestamp < self.current_time {
            return Err(SimError::CausalityViolation {
                event_time_ticks: timestamp.ticks(),
                current_time_ticks: self.current_time.ticks(),
                description: format!("Cannot schedule retroactive event for actor {}", actor_id),
            });
        }
        Ok(self
            .calendar
            .schedule(timestamp, priority, actor_id, payload))
    }

    /// Schedule an event with relative delay.
    pub fn schedule_relative(
        &mut self,
        delay: SimTime,
        priority: u8,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> SimResult<u64> {
        let target_time = self.current_time + delay;
        self.schedule(target_time, priority, actor_id, payload)
    }

    /// Advance simulation by popping the earliest event.
    ///
    /// Verifies strict causality: `event.timestamp >= self.current_time`.
    pub fn step(&mut self) -> SimResult<Option<Event>> {
        if let Some(event) = self.calendar.pop() {
            // Check causality invariant
            if event.timestamp < self.current_time {
                return Err(SimError::CausalityViolation {
                    event_time_ticks: event.timestamp.ticks(),
                    current_time_ticks: self.current_time.ticks(),
                    description: format!(
                        "Retrograde event #{} detected in calendar queue",
                        event.id
                    ),
                });
            }

            // Check max duration bound
            if let Some(max_dur) = self.max_duration {
                if event.timestamp > max_dur {
                    return Ok(None);
                }
            }

            self.current_time = event.timestamp;
            self.events_processed += 1;
            Ok(Some(event))
        } else {
            Ok(None)
        }
    }

    /// Reset the scheduler to initial state.
    pub fn reset(&mut self) {
        self.current_time = SimTime::ZERO;
        self.calendar.clear();
        self.events_processed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_causality_protection() {
        let mut des = DiscreteEventScheduler::new();
        des.schedule(
            SimTime::from_millis(100),
            0,
            1,
            EventPayload::Timer { token: 1 },
        )
        .unwrap();

        let ev = des.step().unwrap().expect("event");
        assert_eq!(ev.timestamp, SimTime::from_millis(100));
        assert_eq!(des.current_time(), SimTime::from_millis(100));

        // Attempting to schedule retroactive event (e.g. at 50 ms when current is 100 ms) must fail!
        let err = des.schedule(
            SimTime::from_millis(50),
            0,
            1,
            EventPayload::Timer { token: 2 },
        );
        assert!(err.is_err());
        match err.unwrap_err() {
            SimError::CausalityViolation { .. } => (),
            other => panic!("Expected CausalityViolation, got {:?}", other),
        }
    }
}
