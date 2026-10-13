//! IEEE 802.11bd Next Generation V2X (NGV) Physical Layer Suite.
//!
//! Standards Reference: IEEE Std 802.11bd-2022 (Amendment 5: Enhancements for Next Generation V2X)
//!
//! Provides:
//! - Physical layer timing constants, 10 MHz and 20 MHz channel modes (`consts_80211bd`)
//! - Modulation and Coding Schemes MCS 0 to MCS 9 (up to 256-QAM) and Dual Carrier Modulation (DCM)
//! - Vehicular Doppler channel aging and midamble tracking engine (`doppler_model`)
//! - Quasi-Cyclic LDPC forward error correction analytical error rates (`ldpc_error_rate`)
//! - Multi-stage physical decider (`decider_80211bd`)
//! - Simulator technology model implementing `WirelessTechnology` (`model`)

pub mod config;
pub mod consts_80211bd;
pub mod decider_80211bd;
pub mod doppler_model;
pub mod ldpc_error_rate;
pub mod model;

pub use config::Wifi80211bdConfig;
pub use consts_80211bd::{
    compute_duration_80211bd, compute_lsig_length_spoofed, Mcs80211bd, MidamblePeriod,
};
pub use decider_80211bd::{Decider80211bd, DeciderResult80211bd};
pub use doppler_model::DopplerChannelEngine;
pub use ldpc_error_rate::LdpcErrorRate;
pub use model::Wifi80211bdModel;
