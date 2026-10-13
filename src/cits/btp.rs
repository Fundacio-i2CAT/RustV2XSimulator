//! ETSI Basic Transport Protocol (BTP) Implementation via `rustflexstack`.
//!
//! Re-exports and integrates standard ETSI EN 302 636-5-1 components from `rustflexstack`.

pub use rustflexstack::btp::btp_header::{BTPAHeader, BTPBHeader};
pub use rustflexstack::btp::router::{BTPRouterHandle, BTPRouterInput, Router as BTPRouter};
pub use rustflexstack::btp::service_access_point::{BTPDataIndication, BTPDataRequest};

pub const BTP_PORT_CAM: u16 = 2001;
pub const BTP_PORT_DENM: u16 = 2002;
