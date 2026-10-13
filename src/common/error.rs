//! Common error definitions for `rustv2xsimulator`.

use std::fmt;

#[derive(Debug, Clone)]
pub enum SimError {
    CausalityViolation {
        event_time_ticks: u64,
        current_time_ticks: u64,
        description: String,
    },
    GeometryError(String),
    ConfigError(String),
    IoError(String),
    XmlParseError(String),
    MobilityError(String),
    ProtocolError(String),
}

impl fmt::Display for SimError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SimError::CausalityViolation {
                event_time_ticks,
                current_time_ticks,
                description,
            } => write!(
                f,
                "Causality violation: event at {} ticks is earlier than current time {} ticks ({})",
                event_time_ticks, current_time_ticks, description
            ),
            SimError::GeometryError(msg) => write!(f, "Geometry error: {}", msg),
            SimError::ConfigError(msg) => write!(f, "Config error: {}", msg),
            SimError::IoError(msg) => write!(f, "IO error: {}", msg),
            SimError::XmlParseError(msg) => write!(f, "XML parse error: {}", msg),
            SimError::MobilityError(msg) => write!(f, "Mobility error: {}", msg),
            SimError::ProtocolError(msg) => write!(f, "Protocol error: {}", msg),
        }
    }
}

impl std::error::Error for SimError {}

impl From<std::io::Error> for SimError {
    fn from(err: std::io::Error) -> Self {
        SimError::IoError(err.to_string())
    }
}

pub type SimResult<T> = Result<T, SimError>;
