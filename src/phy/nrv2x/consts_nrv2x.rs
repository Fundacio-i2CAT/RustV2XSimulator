//! 3GPP Release 16/17 5G NR-V2X Sidelink Physical Layer Constants and Numerologies.
//!
//! Standards References:
//! - 3GPP TS 38.211: Physical channels and modulation (Numerology, Frame Structure)
//! - 3GPP TS 38.214: Physical layer procedures for data (Table 5.1.3.1-1 MCS Table)
//! - 3GPP TS 38.331: Radio Resource Control (Sidelink pre-configuration)
//! - Parity References: 5G-LENA (`nr-sl-phy-mac-common.h`) & UWICORE (`getMCSinformation.m`)

use serde::{Deserialize, Serialize};
use crate::common::SimTime;

/// 3GPP 5G NR Numerology Index (mu in TS 38.211 Table 4.2-1)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NrNumerology {
    /// mu = 0: 15 kHz subcarrier spacing, 1.0 ms slot duration (14 OFDM symbols)
    Mu0_15kHz = 0,
    /// mu = 1: 30 kHz subcarrier spacing, 0.5 ms slot duration (Standard for Band n47 @ 5.9 GHz)
    Mu1_30kHz = 1,
    /// mu = 2: 60 kHz subcarrier spacing, 0.25 ms slot duration (High-speed highway / ultra-low latency)
    Mu2_60kHz = 2,
}

impl Default for NrNumerology {
    fn default() -> Self {
        Self::Mu1_30kHz // Standard European and FCC 5.9 GHz ITS band profile
    }
}

impl NrNumerology {
    /// Subcarrier spacing in Hertz: Delta f = 2^mu * 15 kHz
    pub fn scs_hz(&self) -> f64 {
        match self {
            Self::Mu0_15kHz => 15_000.0,
            Self::Mu1_30kHz => 30_000.0,
            Self::Mu2_60kHz => 60_000.0,
        }
    }

    /// Slot duration in SimTime
    pub fn slot_duration(&self) -> SimTime {
        match self {
            Self::Mu0_15kHz => SimTime::from_millis(1),
            Self::Mu1_30kHz => SimTime::from_micros(500),
            Self::Mu2_60kHz => SimTime::from_micros(250),
        }
    }

    /// Slots per subframe (1 ms)
    pub fn slots_per_subframe(&self) -> usize {
        match self {
            Self::Mu0_15kHz => 1,
            Self::Mu1_30kHz => 2,
            Self::Mu2_60kHz => 4,
        }
    }

    /// Slots per frame (10 ms)
    pub fn slots_per_frame(&self) -> usize {
        self.slots_per_subframe() * 10
    }

    /// Duration of a single OFDM symbol (14 symbols per slot)
    pub fn symbol_duration(&self) -> SimTime {
        SimTime::from_ticks(self.slot_duration().ticks() / 14)
    }
}

/// Standard 3GPP Modulation schemes for NR PSSCH (TS 38.211 Section 5.1)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NrModulation {
    Qpsk,
    Qam16,
    Qam64,
    Qam256,
}

impl NrModulation {
    /// Modulation order Q_m (bits per modulation symbol)
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Self::Qpsk => 2,
            Self::Qam16 => 4,
            Self::Qam64 => 6,
            Self::Qam256 => 8,
        }
    }
}

/// Modulation and Coding Scheme (MCS) for NR PSSCH per 3GPP TS 38.214 Table 5.1.3.1-1.
///
/// Parity with UWICORE `getMCSinformation.m` and 5G-LENA `nr-amc.cc`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct NrMcsEntry {
    pub mcs_index: u8,
    pub modulation: NrModulation,
    /// Target code rate R * 1024 (e.g. 120 means 120/1024)
    pub code_rate_x1024: u16,
    /// Spectral efficiency in bits per resource element
    pub spectral_efficiency: f64,
}

/// 3GPP TS 38.214 Table 5.1.3.1-1 standard MCS Table 1 (MCS 0 to MCS 28)
pub const NR_MCS_TABLE_1: [NrMcsEntry; 29] = [
    NrMcsEntry { mcs_index: 0, modulation: NrModulation::Qpsk, code_rate_x1024: 120, spectral_efficiency: 0.2344 },
    NrMcsEntry { mcs_index: 1, modulation: NrModulation::Qpsk, code_rate_x1024: 157, spectral_efficiency: 0.3066 },
    NrMcsEntry { mcs_index: 2, modulation: NrModulation::Qpsk, code_rate_x1024: 193, spectral_efficiency: 0.3770 },
    NrMcsEntry { mcs_index: 3, modulation: NrModulation::Qpsk, code_rate_x1024: 251, spectral_efficiency: 0.4902 },
    NrMcsEntry { mcs_index: 4, modulation: NrModulation::Qpsk, code_rate_x1024: 308, spectral_efficiency: 0.6016 },
    NrMcsEntry { mcs_index: 5, modulation: NrModulation::Qpsk, code_rate_x1024: 379, spectral_efficiency: 0.7402 },
    NrMcsEntry { mcs_index: 6, modulation: NrModulation::Qpsk, code_rate_x1024: 449, spectral_efficiency: 0.8770 },
    NrMcsEntry { mcs_index: 7, modulation: NrModulation::Qpsk, code_rate_x1024: 526, spectral_efficiency: 1.0273 },
    NrMcsEntry { mcs_index: 8, modulation: NrModulation::Qpsk, code_rate_x1024: 602, spectral_efficiency: 1.1758 },
    NrMcsEntry { mcs_index: 9, modulation: NrModulation::Qpsk, code_rate_x1024: 679, spectral_efficiency: 1.3262 },
    NrMcsEntry { mcs_index: 10, modulation: NrModulation::Qam16, code_rate_x1024: 340, spectral_efficiency: 1.3281 },
    NrMcsEntry { mcs_index: 11, modulation: NrModulation::Qam16, code_rate_x1024: 378, spectral_efficiency: 1.4766 },
    NrMcsEntry { mcs_index: 12, modulation: NrModulation::Qam16, code_rate_x1024: 434, spectral_efficiency: 1.6953 },
    NrMcsEntry { mcs_index: 13, modulation: NrModulation::Qam16, code_rate_x1024: 490, spectral_efficiency: 1.9141 },
    NrMcsEntry { mcs_index: 14, modulation: NrModulation::Qam16, code_rate_x1024: 553, spectral_efficiency: 2.1602 },
    NrMcsEntry { mcs_index: 15, modulation: NrModulation::Qam16, code_rate_x1024: 616, spectral_efficiency: 2.4063 },
    NrMcsEntry { mcs_index: 16, modulation: NrModulation::Qam16, code_rate_x1024: 658, spectral_efficiency: 2.5703 },
    NrMcsEntry { mcs_index: 17, modulation: NrModulation::Qam64, code_rate_x1024: 438, spectral_efficiency: 2.5664 },
    NrMcsEntry { mcs_index: 18, modulation: NrModulation::Qam64, code_rate_x1024: 466, spectral_efficiency: 2.7305 },
    NrMcsEntry { mcs_index: 19, modulation: NrModulation::Qam64, code_rate_x1024: 517, spectral_efficiency: 3.0293 },
    NrMcsEntry { mcs_index: 20, modulation: NrModulation::Qam64, code_rate_x1024: 567, spectral_efficiency: 3.3223 },
    NrMcsEntry { mcs_index: 21, modulation: NrModulation::Qam64, code_rate_x1024: 616, spectral_efficiency: 3.6094 },
    NrMcsEntry { mcs_index: 22, modulation: NrModulation::Qam64, code_rate_x1024: 666, spectral_efficiency: 3.9023 },
    NrMcsEntry { mcs_index: 23, modulation: NrModulation::Qam64, code_rate_x1024: 719, spectral_efficiency: 4.2129 },
    NrMcsEntry { mcs_index: 24, modulation: NrModulation::Qam64, code_rate_x1024: 772, spectral_efficiency: 4.5234 },
    NrMcsEntry { mcs_index: 25, modulation: NrModulation::Qam64, code_rate_x1024: 822, spectral_efficiency: 4.8164 },
    NrMcsEntry { mcs_index: 26, modulation: NrModulation::Qam64, code_rate_x1024: 873, spectral_efficiency: 5.1152 },
    NrMcsEntry { mcs_index: 27, modulation: NrModulation::Qam64, code_rate_x1024: 910, spectral_efficiency: 5.3320 },
    NrMcsEntry { mcs_index: 28, modulation: NrModulation::Qam64, code_rate_x1024: 948, spectral_efficiency: 5.5547 },
];

/// Helper to query MCS entry from Table 1
pub fn get_nr_mcs_entry(mcs_index: u8) -> NrMcsEntry {
    if (mcs_index as usize) < NR_MCS_TABLE_1.len() {
        NR_MCS_TABLE_1[mcs_index as usize]
    } else {
        NR_MCS_TABLE_1[NR_MCS_TABLE_1.len() - 1]
    }
}

/// Compute nominal Transport Block Size (TBS) in bytes for NR PSSCH per 3GPP TS 38.214 Section 5.1.3.2.
pub fn compute_nr_tbs_bytes(
    num_subchannels: usize,
    subchannel_size_rbs: usize,
    mcs_index: u8,
    num_symbols_pssch: usize,
) -> usize {
    let mcs = get_nr_mcs_entry(mcs_index);
    let total_rbs = num_subchannels * subchannel_size_rbs;
    // 12 subcarriers per RB; minus ~2 subcarriers per RB for DMRS overhead
    let res_per_rb = 12 * num_symbols_pssch.saturating_sub(2);
    let total_res = total_rbs * res_per_rb;
    
    // N_info = N_RE * R * Q_m
    let q_m = mcs.modulation.bits_per_symbol() as f64;
    let r = (mcs.code_rate_x1024 as f64) / 1024.0;
    let n_info = (total_res as f64) * q_m * r;

    // Byte align
    ((n_info / 8.0).floor() as usize).max(16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_numerology_properties() {
        assert_eq!(NrNumerology::Mu0_15kHz.scs_hz(), 15_000.0);
        assert_eq!(NrNumerology::Mu0_15kHz.slot_duration(), SimTime::from_millis(1));
        assert_eq!(NrNumerology::Mu1_30kHz.scs_hz(), 30_000.0);
        assert_eq!(NrNumerology::Mu1_30kHz.slot_duration(), SimTime::from_micros(500));
        assert_eq!(NrNumerology::Mu2_60kHz.scs_hz(), 60_000.0);
        assert_eq!(NrNumerology::Mu2_60kHz.slot_duration(), SimTime::from_micros(250));
    }

    #[test]
    fn test_mcs_table_parity_with_uwicore() {
        // Test MCS 0 (QPSK R=120/1024)
        let mcs0 = get_nr_mcs_entry(0);
        assert_eq!(mcs0.modulation, NrModulation::Qpsk);
        assert_eq!(mcs0.code_rate_x1024, 120);

        // Test MCS 10 (16-QAM R=340/1024)
        let mcs10 = get_nr_mcs_entry(10);
        assert_eq!(mcs10.modulation, NrModulation::Qam16);
        assert_eq!(mcs10.code_rate_x1024, 340);

        // Test MCS 28 (64-QAM R=948/1024)
        let mcs28 = get_nr_mcs_entry(28);
        assert_eq!(mcs28.modulation, NrModulation::Qam64);
        assert_eq!(mcs28.code_rate_x1024, 948);
    }
}
