//! 3GPP Release 16/17 NR-V2X Configuration.
//!
//! Standards Reference: 3GPP TS 38.331 (SL-BWP-Config / SL-ResourcePool) & TS 38.213 Section 16.

use serde::{Deserialize, Serialize};
use super::consts_nrv2x::NrNumerology;

/// Configuration parameters for 3GPP 5G NR-V2X Sidelink Mode 2.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NrV2xConfig {
    /// Carrier frequency in Hertz (default 5.890 GHz for 3GPP Band n47)
    pub carrier_freq_hz: f64,
    /// 5G NR Numerology mu in TS 38.211 (default Mu1_30kHz)
    pub numerology: NrNumerology,
    /// Channel bandwidth in MHz (10.0, 20.0, 40.0)
    pub bandwidth_mhz: f64,
    /// Subchannel size in Resource Blocks (allowed in TS 38.331: 10, 12, 15, 20, 25)
    pub subchannel_size_rbs: usize,
    /// Total number of active subchannels per slot in the Sidelink BWP
    pub num_subchannels: usize,
    /// Number of OFDM symbols allocated to PSCCH per slot (2 or 3)
    pub pscch_symbols: usize,
    /// Physical Sidelink Feedback Channel (PSFCH) period in slots: 0 (disabled), 1, 2, or 4
    pub psfch_period: usize,
    /// Sensing window length T0 in milliseconds (100 ms for short sensing, 1100 ms for full)
    pub sensing_window_ms: u64,
    /// Minimum selection window start T1 in slots (UE processing time T_proc_1)
    pub t1_min_slots: u64,
    /// Maximum selection window end T2 in slots (packet latency tolerance bound)
    pub t2_max_slots: u64,
    /// Re-evaluation processing margin T3 in slots prior to transmission slot (Rel-16)
    pub t3_reeval_slots: u64,
    /// Nominal resource reservation period P_rsvp in milliseconds (e.g. 100 ms for periodic CAMs)
    pub p_rsvp_ms: u64,
    /// Minimum reselection counter C_resel
    pub c_resel_min: u32,
    /// Maximum reselection counter C_resel
    pub c_resel_max: u32,
    /// Probability P_keep of retaining grant after C_resel reaches 0 (0.0 to 0.8)
    pub prob_resource_keep: f64,
    /// Reference Signal Received Power exclusion threshold in dBm
    pub rsrp_threshold_dbm: f64,
    /// Minimum percentage X% of candidate resources to retain (20%, 33%, 50%)
    pub min_candidate_pct: f64,
    /// Default PSSCH MCS index (0 to 28 per TS 38.214 Table 5.1.3.1-1)
    pub pssch_mcs: u8,
    /// Transmit power in dBm (default 23.0 dBm = 200 mW)
    pub tx_power_dbm: f64,
    /// Enable PSFCH closed-loop HARQ feedback
    pub enable_harq: bool,
    /// Maximum number of HARQ retransmissions (0 to 3)
    pub max_harq_retx: u8,
    /// Enable Release 16 Pre-emption mechanism
    pub enable_preemption: bool,
    /// Enable Release 16 Re-evaluation mechanism
    pub enable_reevaluation: bool,
}

impl Default for NrV2xConfig {
    fn default() -> Self {
        Self {
            carrier_freq_hz: 5.890e9,
            numerology: NrNumerology::Mu1_30kHz,
            bandwidth_mhz: 20.0,
            subchannel_size_rbs: 10,
            num_subchannels: 5,
            pscch_symbols: 3,
            psfch_period: 1,
            sensing_window_ms: 100,
            t1_min_slots: 2,
            t2_max_slots: 32,
            t3_reeval_slots: 2,
            p_rsvp_ms: 100,
            c_resel_min: 5,
            c_resel_max: 15,
            prob_resource_keep: 0.4,
            rsrp_threshold_dbm: -112.0,
            min_candidate_pct: 20.0,
            pssch_mcs: 10, // 16-QAM code rate 340/1024
            tx_power_dbm: 23.0,
            enable_harq: true,
            max_harq_retx: 2,
            enable_preemption: true,
            enable_reevaluation: true,
        }
    }
}

impl NrV2xConfig {
    /// Create high-speed highway configuration (mu = 2 @ 60 kHz SCS, 0.25 ms slot)
    pub fn high_speed_highway() -> Self {
        let mut cfg = Self::default();
        cfg.numerology = NrNumerology::Mu2_60kHz;
        cfg.t1_min_slots = 4;
        cfg.t2_max_slots = 64;
        cfg.t3_reeval_slots = 4;
        cfg
    }

    /// Create ultra-reliable low-latency (URLLC) platooning configuration (MCS 4 QPSK, HARQ enabled)
    pub fn urllc_platooning() -> Self {
        let mut cfg = Self::default();
        cfg.pssch_mcs = 4; // Robust QPSK
        cfg.enable_harq = true;
        cfg.max_harq_retx = 3;
        cfg.min_candidate_pct = 33.0;
        cfg
    }
}
