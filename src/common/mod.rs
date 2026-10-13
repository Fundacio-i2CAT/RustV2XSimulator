//! Common utilities, profiling, geodetic transformations, and error handling.

pub mod error;
pub mod geo;
pub mod profiler;
pub mod time;

pub use error::{SimError, SimResult};
pub use geo::{euclidean_distance_2d, euclidean_distance_3d, GeodeticTransform, Wgs84Coordinate};
pub use profiler::{DecoupledWallClockProfiler, ProfileCategory, ProfilerSummary};
pub use time::{SimTime, TICKS_PER_MICROS, TICKS_PER_MILLIS, TICKS_PER_SECOND};
