//! IEEE 802.11p and IEEE 1609.4 Physical and MAC Layer Constants.
//!
//! Direct translation of Veins / OMNeT++ `Consts80211p.h` and IEEE 802.11-2012 / IEEE 1609.4-2010:
//! - 10 MHz channel spacing timing parameters
//! - 8 standard OFDM Modulation and Coding Schemes (MCS)
//! - PLCP frame format constants (Preamble, Signal, Service, Tail)
//! - EDCA contention window limits and channel definitions

use serde::{Deserialize, Serialize};
use crate::common::SimTime;

/// Number of standard bitrates for IEEE 802.11p in a 10 MHz channel
pub const NUM_BITRATES_80211P: usize = 8;

/// Standard bitrates for 802.11p (in bits per second) per Table 17-14 IEEE 802.11-2007
pub const BITRATES_80211P: [u64; NUM_BITRATES_80211P] = [
    3_000_000, 4_500_000, 6_000_000, 9_000_000,
    12_000_000, 18_000_000, 24_000_000, 27_000_000,
];

/// Number of Data Bits Per Symbol (N_DBPS) for each bitrate per Table 17-3 IEEE 802.11-2007
pub const N_DBPS_80211P: [u32; NUM_BITRATES_80211P] = [
    24, 36, 48, 72, 96, 144, 192, 216,
];

/// OFDM symbol interval in seconds (8.0 microseconds for 10 MHz channels, Table 17-4)
pub const T_SYM_80211P_US: u64 = 8;

/// Length (in bits) of the SERVICE field in PHY header (17.3.2 PLCP frame format)
pub const PHY_HDR_SERVICE_BITS: usize = 16;

/// Length (in bits) of Tail field in PHY PPDU (17.3.2 PLCP frame format)
pub const PHY_TAIL_BITS: usize = 6;

/// Duration of the PLCP Preamble: 32 microseconds (Table 17-4)
pub const PHY_HDR_PREAMBLE_DURATION_US: u64 = 32;

/// Duration of the PLCP Signal: 8 microseconds (Table 17-4)
pub const PHY_HDR_PLCPSIGNAL_DURATION_US: u64 = 8;

/// Length of the PLCP Signal header: 24 bits (Figure 17-1 PPDU frame format)
pub const PHY_HDR_PLCPSIGNAL_BITS: usize = 24;

/// Bitrate of the PLCP Signal header: 3 Mbps (transmitted using BPSK 1/2)
pub const PHY_HDR_BITRATE_BPS: u64 = 3_000_000;

/// Slot Time for 10 MHz channel spacing: 13 microseconds (Table 17-15)
pub const SLOT_TIME_10MHZ_US: u64 = 13;

/// Short Interframe Space (SIFS) for 10 MHz channel spacing: 32 microseconds (Table 17-15)
pub const SIFS_10MHZ_US: u64 = 32;

/// Radio turnaround time (Rx to Tx): 1 microsecond (Table 17-15)
pub const RADIO_DELAY_US: u64 = 1;

/// Contention Window minimal size for 802.11p: 15
pub const CWMIN_11P: u32 = 15;

/// Contention Window maximal size for 802.11p: 1023
pub const CWMAX_11P: u32 = 1023;

/// IEEE 1609.4 slot interval: 50 milliseconds (Table H.1 IEEE 1609.4-2010)
pub const SWITCHING_INTERVAL_MS: u64 = 50;

/// IEEE 1609.4 guard interval: 4 milliseconds (SyncTolerance + MaxChSwitchTime)
pub const GUARD_INTERVAL_MS: u64 = 4;

/// IEEE 802.11p DSRC Channels reserved by FCC (center frequencies in GHz)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum DsrcChannel {
    /// Channel 172: Critical Safety (5.86 GHz)
    CritSol = 172,
    /// Channel 174: Service Channel 1 (5.87 GHz)
    Sch1 = 174,
    /// Channel 176: Service Channel 2 (5.88 GHz)
    Sch2 = 176,
    /// Channel 178: Control Channel CCH (5.89 GHz) - ETSI G5-CCH / US WAVE CCH
    Cch = 178,
    /// Channel 180: Service Channel 3 (5.90 GHz)
    Sch3 = 180,
    /// Channel 182: Service Channel 4 (5.91 GHz)
    Sch4 = 182,
    /// Channel 184: High Power Public Safety (5.92 GHz)
    Hpps = 184,
}

impl DsrcChannel {
    /// Return the carrier center frequency in Hertz.
    pub fn center_frequency_hz(&self) -> f64 {
        match self {
            Self::CritSol => 5.86e9,
            Self::Sch1 => 5.87e9,
            Self::Sch2 => 5.88e9,
            Self::Cch => 5.89e9,
            Self::Sch3 => 5.90e9,
            Self::Sch4 => 5.91e9,
            Self::Hpps => 5.92e9,
        }
    }

    /// Check if this is a control channel
    pub fn is_control_channel(&self) -> bool {
        matches!(self, Self::Cch)
    }
}

/// 802.11p Modulation and Coding Schemes (MCS)
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Mcs80211p {
    /// BPSK rate 1/2: 3 Mbps (N_DBPS = 24)
    BpskR1_2 = 0,
    /// BPSK rate 3/4: 4.5 Mbps (N_DBPS = 36)
    BpskR3_4 = 1,
    /// QPSK rate 1/2: 6 Mbps (N_DBPS = 48) - Standard C-ITS Default
    QpskR1_2 = 2,
    /// QPSK rate 3/4: 9 Mbps (N_DBPS = 72)
    QpskR3_4 = 3,
    /// 16-QAM rate 1/2: 12 Mbps (N_DBPS = 96)
    Qam16R1_2 = 4,
    /// 16-QAM rate 3/4: 18 Mbps (N_DBPS = 144)
    Qam16R3_4 = 5,
    /// 64-QAM rate 2/3: 24 Mbps (N_DBPS = 192)
    Qam64R2_3 = 6,
    /// 64-QAM rate 3/4: 27 Mbps (N_DBPS = 216)
    Qam64R3_4 = 7,
}

impl Default for Mcs80211p {
    fn default() -> Self {
        Self::QpskR1_2 // 6 Mbps default for ETSI C-ITS & IEEE 1609.4
    }
}

impl Mcs80211p {
    /// Return the transmission bitrate in bits per second.
    #[inline]
    pub fn bitrate_bps(&self) -> u64 {
        BITRATES_80211P[*self as usize]
    }

    /// Return the number of data bits per symbol (N_DBPS).
    #[inline]
    pub fn n_dbps(&self) -> u32 {
        N_DBPS_80211P[*self as usize]
    }

    /// Puncturing code rate b-value used in FEC error calculation (bValue in NistErrorRate):
    /// - 1 for rate 1/2
    /// - 2 for rate 2/3
    /// - 3 for rate 3/4
    #[inline]
    pub fn b_value(&self) -> u32 {
        match self {
            Self::BpskR1_2 | Self::QpskR1_2 | Self::Qam16R1_2 => 1,
            Self::Qam64R2_3 => 2,
            Self::BpskR3_4 | Self::QpskR3_4 | Self::Qam16R3_4 | Self::Qam64R3_4 => 3,
        }
    }

    /// Derive MCS from datarate in bits per second (defaults to QpskR1_2 if unknown).
    pub fn from_bitrate_bps(bitrate: u64) -> Self {
        match bitrate {
            3_000_000 => Self::BpskR1_2,
            4_500_000 => Self::BpskR3_4,
            6_000_000 => Self::QpskR1_2,
            9_000_000 => Self::QpskR3_4,
            12_000_000 => Self::Qam16R1_2,
            18_000_000 => Self::Qam16R3_4,
            24_000_000 => Self::Qam64R2_3,
            27_000_000 => Self::Qam64R3_4,
            _ => Self::QpskR1_2,
        }
    }

    /// Derive MCS from index 0..=7 (defaults to Qam64R3_4 if index > 7).
    pub fn from_index(idx: u8) -> Self {
        match idx {
            0 => Self::BpskR1_2,
            1 => Self::BpskR3_4,
            2 => Self::QpskR1_2,
            3 => Self::QpskR3_4,
            4 => Self::Qam16R1_2,
            5 => Self::Qam16R3_4,
            6 => Self::Qam64R2_3,
            _ => Self::Qam64R3_4,
        }
    }
}

/// Compute exact frame air transmission duration in IEEE 802.11p (10 MHz channel).
///
/// Parity with Veins `PhyLayer80211p::calculateDuration` and IEEE 802.11-2012 eq (18-18):
/// `Duration = T_preamble (32us) + T_signal (8us) + ceil((16 + 8*L + 6) / N_DBPS) * T_sym (8us)`
#[inline]
pub fn compute_duration_80211p(payload_bytes: usize, mcs: Mcs80211p) -> SimTime {
    let n_bits = PHY_HDR_SERVICE_BITS + (payload_bytes * 8) + PHY_TAIL_BITS;
    let n_dbps = mcs.n_dbps() as usize;
    let n_sym = (n_bits + n_dbps - 1) / n_dbps;
    let data_duration_us = (n_sym as u64) * T_SYM_80211P_US;
    let total_us = PHY_HDR_PREAMBLE_DURATION_US + PHY_HDR_PLCPSIGNAL_DURATION_US + data_duration_us;
    SimTime::from_micros(total_us)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_80211p_duration_calculation() {
        // For a 300 byte CAM packet at 6 Mbps (QPSK 1/2, N_DBPS = 48):
        // N_bits = 16 + 300*8 + 6 = 2422 bits.
        // N_sym = ceil(2422 / 48) = 51 symbols.
        // Data duration = 51 * 8us = 408 us.
        // Total duration = 32us + 8us + 408us = 448 us.
        let dur = compute_duration_80211p(300, Mcs80211p::QpskR1_2);
        assert_eq!(dur.to_micros(), 448);
    }

    #[test]
    fn test_80211p_mcs_from_index() {
        assert_eq!(Mcs80211p::from_index(0), Mcs80211p::BpskR1_2);
        assert_eq!(Mcs80211p::from_index(2), Mcs80211p::QpskR1_2);
        assert_eq!(Mcs80211p::from_index(7), Mcs80211p::Qam64R3_4);
        assert_eq!(Mcs80211p::from_index(99), Mcs80211p::Qam64R3_4);
    }
}
