//! IEEE 802.11bd Next Generation V2X (NGV) Physical Layer Constants.
//!
//! Standards Reference: IEEE Std 802.11bd-2022 (Amendment 5: Enhancements for Next Generation V2X)
//! - 10 MHz and 20 MHz channel bandwidth timings
//! - Mixed-format NGV PPDU preamble (L-STF, L-LTF, L-SIG, R-SIG, NGV-SIG, NGV-STF, NGV-LTF)
//! - Midamble configurations (None, 4, 8, 16 symbols) for Doppler mitigation
//! - 10 standard Modulation and Coding Schemes (MCS 0 to MCS 9, up to 256-QAM)
//! - Dual Carrier Modulation (DCM) frequency diversity flag
//! - L-SIG duration spoofing calculation for backward compatibility with IEEE 802.11p

use serde::{Deserialize, Serialize};
use crate::common::SimTime;

/// OFDM symbol interval in microseconds for 10 MHz channels (Table 27-4 IEEE 802.11bd)
pub const T_SYM_10MHZ_US: u64 = 8;

/// OFDM symbol interval in microseconds for 20 MHz channels (scaled)
pub const T_SYM_20MHZ_US: u64 = 4;

/// Legacy Preamble field durations (10 MHz):
/// L-STF: 16 microseconds
pub const DURATION_L_STF_10MHZ_US: u64 = 16;
/// L-LTF: 16 microseconds
pub const DURATION_L_LTF_10MHZ_US: u64 = 16;
/// L-SIG: 8 microseconds
pub const DURATION_L_SIG_10MHZ_US: u64 = 8;

/// NGV Preamble field durations (10 MHz):
/// R-SIG (Repeated L-SIG for auto-detection): 8 microseconds
pub const DURATION_R_SIG_10MHZ_US: u64 = 8;
/// NGV-SIG: 8 microseconds
pub const DURATION_NGV_SIG_10MHZ_US: u64 = 8;
/// NGV-STF: 8 microseconds
pub const DURATION_NGV_STF_10MHZ_US: u64 = 8;
/// NGV-LTF (1 spatial stream): 8 microseconds
pub const DURATION_NGV_LTF_10MHZ_US: u64 = 8;

/// Total Mixed-Format Preamble Duration for 10 MHz channel (72 microseconds)
pub const PREAMBLE_DURATION_10MHZ_US: u64 = DURATION_L_STF_10MHZ_US
    + DURATION_L_LTF_10MHZ_US
    + DURATION_L_SIG_10MHZ_US
    + DURATION_R_SIG_10MHZ_US
    + DURATION_NGV_SIG_10MHZ_US
    + DURATION_NGV_STF_10MHZ_US
    + DURATION_NGV_LTF_10MHZ_US;

/// Total Mixed-Format Preamble Duration for 20 MHz channel (36 microseconds)
pub const PREAMBLE_DURATION_20MHZ_US: u64 = PREAMBLE_DURATION_10MHZ_US / 2;

/// Length (in bits) of the SERVICE field in PHY header
pub const PHY_HDR_SERVICE_BITS: usize = 16;

/// Length (in bits) of Tail field in PHY PPDU
pub const PHY_TAIL_BITS: usize = 6;

/// Midamble insertion period per IEEE Std 802.11bd-2022 Clause 27.3.11.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MidamblePeriod {
    /// No midambles inserted (legacy-like, vulnerable to high Doppler spread)
    None = 0,
    /// Midamble inserted every 4 data symbols (maximum Doppler resilience, e.g. relative speeds > 200 km/h)
    Every4Symbols = 4,
    /// Midamble inserted every 8 data symbols (standard balance between overhead and Doppler tracking)
    Every8Symbols = 8,
    /// Midamble inserted every 16 data symbols (low overhead for moderate speeds)
    Every16Symbols = 16,
}

impl Default for MidamblePeriod {
    fn default() -> Self {
        Self::Every8Symbols
    }
}

impl MidamblePeriod {
    /// Numerical interval in symbols (0 if disabled)
    #[inline]
    pub fn interval_symbols(&self) -> usize {
        *self as usize
    }

    /// Calculate the number of midamble symbols inserted for a given number of data symbols.
    #[inline]
    pub fn calculate_midamble_symbols(&self, data_symbols: usize) -> usize {
        match self {
            Self::None => 0,
            Self::Every4Symbols => (data_symbols.saturating_sub(1)) / 4,
            Self::Every8Symbols => (data_symbols.saturating_sub(1)) / 8,
            Self::Every16Symbols => (data_symbols.saturating_sub(1)) / 16,
        }
    }
}

/// Modulation and Coding Schemes (MCS) for IEEE 802.11bd (Table 27-30 IEEE Std 802.11bd-2022).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Mcs80211bd {
    /// MCS 0: BPSK rate 1/2
    Mcs0Bpsk1_2 = 0,
    /// MCS 1: QPSK rate 1/2
    Mcs1Qpsk1_2 = 1,
    /// MCS 2: QPSK rate 3/4
    Mcs2Qpsk3_4 = 2,
    /// MCS 3: 16-QAM rate 1/2
    Mcs3Qam16_1_2 = 3,
    /// MCS 4: 16-QAM rate 3/4
    Mcs4Qam16_3_4 = 4,
    /// MCS 5: 64-QAM rate 2/3
    Mcs5Qam64_2_3 = 5,
    /// MCS 6: 64-QAM rate 3/4
    Mcs6Qam64_3_4 = 6,
    /// MCS 7: 64-QAM rate 5/6
    Mcs7Qam64_5_6 = 7,
    /// MCS 8: 256-QAM rate 3/4
    Mcs8Qam256_3_4 = 8,
    /// MCS 9: 256-QAM rate 5/6
    Mcs9Qam256_5_6 = 9,
}

impl Default for Mcs80211bd {
    fn default() -> Self {
        Self::Mcs2Qpsk3_4 // 9 Mbps in 10 MHz channel: excellent balance of throughput and range
    }
}

impl Mcs80211bd {
    /// Number of data bits per symbol (N_DBPS) in 10 MHz channel (52 subcarriers: 48 data + 4 pilots).
    #[inline]
    pub fn n_dbps_10mhz(&self, use_dcm: bool) -> usize {
        if use_dcm && *self == Self::Mcs0Bpsk1_2 {
            // DCM halves effective payload rate for 2x diversity
            12
        } else {
            match self {
                Self::Mcs0Bpsk1_2 => 24,
                Self::Mcs1Qpsk1_2 => 48,
                Self::Mcs2Qpsk3_4 => 72,
                Self::Mcs3Qam16_1_2 => 96,
                Self::Mcs4Qam16_3_4 => 144,
                Self::Mcs5Qam64_2_3 => 192,
                Self::Mcs6Qam64_3_4 => 216,
                Self::Mcs7Qam64_5_6 => 240,
                Self::Mcs8Qam256_3_4 => 288,
                Self::Mcs9Qam256_5_6 => 320,
            }
        }
    }

    /// Number of data bits per symbol (N_DBPS) in 20 MHz channel with 4.0 us symbol duration.
    #[inline]
    pub fn n_dbps_20mhz(&self, use_dcm: bool) -> usize {
        // With 4 us symbol duration (312.5 kHz subcarrier spacing), N_DBPS per symbol is equal to 10 MHz,
        // resulting in exactly 2x physical bitrate.
        self.n_dbps_10mhz(use_dcm)
    }

    /// Physical data bitrate in bits per second (bps) for 10 MHz channel.
    #[inline]
    pub fn bitrate_10mhz_bps(&self, use_dcm: bool) -> u64 {
        let n_dbps = self.n_dbps_10mhz(use_dcm) as u64;
        (n_dbps * 1_000_000) / T_SYM_10MHZ_US
    }

    /// Physical data bitrate in bits per second (bps) for 20 MHz channel.
    #[inline]
    pub fn bitrate_20mhz_bps(&self, use_dcm: bool) -> u64 {
        let n_dbps = self.n_dbps_20mhz(use_dcm) as u64;
        (n_dbps * 1_000_000) / T_SYM_20MHZ_US
    }

    /// Code rate as a float (e.g. 0.5, 0.75, 0.8333).
    #[inline]
    pub fn code_rate(&self) -> f64 {
        match self {
            Self::Mcs0Bpsk1_2 | Self::Mcs1Qpsk1_2 | Self::Mcs3Qam16_1_2 => 0.5,
            Self::Mcs5Qam64_2_3 => 2.0 / 3.0,
            Self::Mcs2Qpsk3_4 | Self::Mcs4Qam16_3_4 | Self::Mcs6Qam64_3_4 | Self::Mcs8Qam256_3_4 => 0.75,
            Self::Mcs7Qam64_5_6 | Self::Mcs9Qam256_5_6 => 5.0 / 6.0,
        }
    }

    /// Modulation constellation order (bits per subcarrier: 1, 2, 4, 6, 8).
    #[inline]
    pub fn modulation_order(&self) -> u32 {
        match self {
            Self::Mcs0Bpsk1_2 => 1,
            Self::Mcs1Qpsk1_2 | Self::Mcs2Qpsk3_4 => 2,
            Self::Mcs3Qam16_1_2 | Self::Mcs4Qam16_3_4 => 4,
            Self::Mcs5Qam64_2_3 | Self::Mcs6Qam64_3_4 | Self::Mcs7Qam64_5_6 => 6,
            Self::Mcs8Qam256_3_4 | Self::Mcs9Qam256_5_6 => 8,
        }
    }

    /// Derive MCS from index 0..=9 (defaults to Mcs9Qam256_5_6 if index > 9).
    pub fn from_index(idx: u8) -> Self {
        match idx {
            0 => Self::Mcs0Bpsk1_2,
            1 => Self::Mcs1Qpsk1_2,
            2 => Self::Mcs2Qpsk3_4,
            3 => Self::Mcs3Qam16_1_2,
            4 => Self::Mcs4Qam16_3_4,
            5 => Self::Mcs5Qam64_2_3,
            6 => Self::Mcs6Qam64_3_4,
            7 => Self::Mcs7Qam64_5_6,
            8 => Self::Mcs8Qam256_3_4,
            _ => Self::Mcs9Qam256_5_6,
        }
    }
}

/// Compute exact frame air transmission duration in IEEE 802.11bd.
///
/// Accounts for:
/// - Mixed-format preamble (72 microseconds at 10 MHz, 36 microseconds at 20 MHz)
/// - LDPC block padding and SERVICE/TAIL bits
/// - Periodic midamble symbols (each adding 1 OFDM symbol duration)
pub fn compute_duration_80211bd(
    payload_bytes: usize,
    mcs: Mcs80211bd,
    bandwidth_mhz: f64,
    midamble: MidamblePeriod,
    use_dcm: bool,
) -> SimTime {
    let is_20mhz = bandwidth_mhz >= 15.0;
    let t_sym_us = if is_20mhz { T_SYM_20MHZ_US } else { T_SYM_10MHZ_US };
    let preamble_us = if is_20mhz {
        PREAMBLE_DURATION_20MHZ_US
    } else {
        PREAMBLE_DURATION_10MHZ_US
    };

    let n_dbps = if is_20mhz {
        mcs.n_dbps_20mhz(use_dcm)
    } else {
        mcs.n_dbps_10mhz(use_dcm)
    };

    let n_bits = PHY_HDR_SERVICE_BITS + (payload_bytes * 8) + PHY_TAIL_BITS;
    let n_data_sym = (n_bits + n_dbps - 1) / n_dbps;

    // Calculate midamble symbols inserted in the Data field
    let n_midamble_sym = midamble.calculate_midamble_symbols(n_data_sym);
    let total_ofdm_sym = n_data_sym + n_midamble_sym;

    let total_us = preamble_us + (total_ofdm_sym as u64 * t_sym_us);
    SimTime::from_micros(total_us)
}

/// Compute spoofed L-SIG `LENGTH` field in bytes for backward compatibility with IEEE 802.11p.
///
/// When an 802.11p node receives an 802.11bd mixed-format frame, it reads the L-SIG field.
/// Per IEEE 802.11 standard:
/// Duration = ceil((16 + LENGTH*8 + 6) / 24) * 8 us + 40 us.
/// By spoofing `LENGTH`, legacy 802.11p stations compute the exact total duration and defer access,
/// preventing in-flight collisions.
pub fn compute_lsig_length_spoofed(total_duration: SimTime, bandwidth_mhz: f64) -> u16 {
    let dur_us = total_duration.to_micros();
    // Legacy 802.11p preamble is 40 us (32 us preamble + 8 us L-SIG)
    let legacy_preamble_us: u64 = if bandwidth_mhz >= 15.0 { 20 } else { 40 };
    if dur_us <= legacy_preamble_us {
        return 0;
    }
    let data_dur_us = dur_us - legacy_preamble_us;
    let legacy_t_sym = if bandwidth_mhz >= 15.0 { 4 } else { 8 };
    let num_legacy_sym = (data_dur_us + legacy_t_sym - 1) / legacy_t_sym;
    // In legacy 10 MHz, L-SIG transmits at 3 Mbps (24 bits per symbol)
    let legacy_n_dbps = if bandwidth_mhz >= 15.0 { 48 } else { 24 };
    let total_payload_bits = (num_legacy_sym * legacy_n_dbps).saturating_sub(22); // 16 service + 6 tail
    ((total_payload_bits / 8) as u16).min(4095)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_80211bd_bitrates_and_durations() {
        // MCS 0: 3 Mbps (10 MHz)
        assert_eq!(Mcs80211bd::Mcs0Bpsk1_2.bitrate_10mhz_bps(false), 3_000_000);
        // MCS 0 with DCM: 1.5 Mbps
        assert_eq!(Mcs80211bd::Mcs0Bpsk1_2.bitrate_10mhz_bps(true), 1_500_000);
        // MCS 1: 6 Mbps
        assert_eq!(Mcs80211bd::Mcs1Qpsk1_2.bitrate_10mhz_bps(false), 6_000_000);
        // MCS 8 (256-QAM 3/4): 36 Mbps
        assert_eq!(Mcs80211bd::Mcs8Qam256_3_4.bitrate_10mhz_bps(false), 36_000_000);
        // MCS 9 (256-QAM 5/6): 40 Mbps
        assert_eq!(Mcs80211bd::Mcs9Qam256_5_6.bitrate_10mhz_bps(false), 40_000_000);

        // In 20 MHz, MCS 9 reaches 80 Mbps
        assert_eq!(Mcs80211bd::Mcs9Qam256_5_6.bitrate_20mhz_bps(false), 80_000_000);
    }

    #[test]
    fn test_midamble_symbol_insertion() {
        // 10 data symbols with midamble every 4:
        // Midambles placed after symbol 4, symbol 8 -> 2 midambles
        let mid = MidamblePeriod::Every4Symbols;
        assert_eq!(mid.calculate_midamble_symbols(10), 2);

        // 17 data symbols with midamble every 8:
        // Midambles placed after symbol 8, symbol 16 -> 2 midambles
        let mid8 = MidamblePeriod::Every8Symbols;
        assert_eq!(mid8.calculate_midamble_symbols(17), 2);
    }

    #[test]
    fn test_duration_with_midambles() {
        // 300 byte CAM packet at MCS 2 (QPSK 3/4, N_DBPS = 72)
        // N_bits = 16 + 300*8 + 6 = 2422 bits.
        // N_data_sym = ceil(2422 / 72) = 34 symbols.
        // With Midamble Every 8 symbols: n_midamble = (34 - 1)/8 = 4 midambles.
        // Total OFDM symbols = 34 + 4 = 38 symbols.
        // Total duration = 72 us (preamble) + 38 * 8 us = 72 + 304 = 376 us.
        let dur = compute_duration_80211bd(
            300,
            Mcs80211bd::Mcs2Qpsk3_4,
            10.0,
            MidamblePeriod::Every8Symbols,
            false,
        );
        assert_eq!(dur.to_micros(), 376);
    }

    #[test]
    fn test_80211bd_mcs_from_index() {
        assert_eq!(Mcs80211bd::from_index(0), Mcs80211bd::Mcs0Bpsk1_2);
        assert_eq!(Mcs80211bd::from_index(2), Mcs80211bd::Mcs2Qpsk3_4);
        assert_eq!(Mcs80211bd::from_index(9), Mcs80211bd::Mcs9Qam256_5_6);
        assert_eq!(Mcs80211bd::from_index(99), Mcs80211bd::Mcs9Qam256_5_6);
    }
}
