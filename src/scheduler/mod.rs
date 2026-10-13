//! Discrete-event simulation scheduler module.

pub mod calendar;
pub mod des;
pub mod event;

pub use calendar::CalendarQueue;
pub use des::{DiscreteEventScheduler, EventContext};
pub use event::{ActorId, Event, EventPayload};
