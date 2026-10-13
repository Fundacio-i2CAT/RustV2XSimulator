//! 5G NR LDPC & Polar Coding Physical Layer Error Rate Model.
//!
//! Standards Reference: 3GPP TS 38.212 Section 5 & 6, TS 38.214 Table 5.1.3.1-1
//! Parity Reference: UWICORE Link-Level Dataset & 5G-LENA `nr-sl-error-model.cc`

use rand::Rng;
use serde::{Deserialize, Serialize};
use super::psfch_harq::NrHarqFeedback;

/// Detailed physical reception result for an NR-V2X sidelink transmission.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NrReceptionResult {
    /// True if both PSCCH (SCI-1A) and PSSCH (Data + SCI-2A) decoded successfully
    pub is_success: bool,
    /// True if Stage-1 SCI decoded error-free on PSCCH
    pub sci1_success: bool,
    /// True if Stage-2 SCI decoded error-free on PSSCH
    pub sci2_success: bool,
    /// True if Transport Block payload decoded error-free via NR LDPC on PSSCH
    pub tb_success: bool,
    /// Resulting HARQ feedback (ACK if successful, NACK if TB failed, DTX if SCI missed)
    pub harq_feedback: NrHarqFeedback,
    /// Effective SINR in dB
    pub sinr_db: f64,
}

/// Physical layer error rate evaluation engine for 5G NR Sidelink.
pub struct NrPhyBlerModel;

impl NrPhyBlerModel {
    /// 10% BLER threshold in dB for standard NR PSSCH MCS (Table 5.1.3.1-1) under TDL vehicular channels.
    /// Calibrated against UWICORE MATLAB link-level simulations.
    pub fn pssch_sinr_threshold_db(mcs_index: u8) -> f64 {
        match mcs_index {
            0 => -1.8,  // QPSK R=120
            1 => -0.5,  // QPSK R=157
            2 => 0.8,   // QPSK R=193
            3 => 2.0,   // QPSK R=251
            4 => 3.2,   // QPSK R=308
            5 => 4.8,   // QPSK R=379
            6 => 6.2,   // QPSK R=449
            7 => 7.8,   // QPSK R=526
            8 => 9.2,   // QPSK R=602
            9 => 10.6,  // QPSK R=679
            10 => 11.2, // 16-QAM R=340
            11 => 12.2, // 16-QAM R=378
            12 => 13.5, // 16-QAM R=434
            13 => 14.8, // 16-QAM R=490
            14 => 16.2, // 16-QAM R=553
            15 => 17.6, // 16-QAM R=616
            16 => 18.5, // 16-QAM R=658
            17 => 19.2, // 64-QAM R=438
            18 => 20.0, // 64-QAM R=466
            19 => 21.2, // 64-QAM R=517
            20 => 22.4, // 64-QAM R=567
            21 => 23.6, // 64-QAM R=616
            22 => 24.8, // 64-QAM R=666
            23 => 26.0, // 64-QAM R=719
            24 => 27.2, // 64-QAM R=772
            25 => 28.5, // 64-QAM R=822
            26 => 29.8, // 64-QAM R=873
            27 => 30.8, // 64-QAM R=910
            _ => 32.0,  // 64-QAM R=948
        }
    }

    /// Calculate Block Error Rate (BLER) for NR LDPC PSSCH.
    pub fn pssch_ldpc_bler(sinr_db: f64, mcs_index: u8, payload_len_bytes: usize) -> f64 {
        let th_db = Self::pssch_sinr_threshold_db(mcs_index);
        // LDPC steep waterfall slope
        let slope = 0.55;
        let delta = sinr_db - th_db;

        if delta > 4.0 {
            return 0.0;
        }
        if delta < -4.0 {
            return 1.0;
        }

        // Logistic waterfall curve with transport block length scaling
        let len_factor = (payload_len_bytes as f64 / 300.0).clamp(0.5, 3.0);
        let bler = (1.0 / (1.0 + (delta / slope).exp())) * len_factor;
        bler.clamp(0.0, 1.0)
    }

    /// Calculate Block Error Rate (BLER) for Polar coded PSCCH (SCI-1A).
    pub fn pscch_polar_bler(sinr_db: f64) -> f64 {
        // Polar codes with CRC-24C at QPSK rate ~0.2 have an error threshold around -4.0 dB
        let th_db = -4.0;
        let slope = 0.45;
        let delta = sinr_db - th_db;

        if delta > 3.0 {
            return 0.0;
        }
        if delta < -3.0 {
            return 1.0;
        }

        (1.0 / (1.0 + (delta / slope).exp())).clamp(0.0, 1.0)
    }

    /// Evaluate complete two-stage reception (PSCCH Polar + PSSCH LDPC).
    ///
    /// Direct compliance with 3GPP: packet is successfully decoded if and only if
    /// both the Stage-1 SCI, Stage-2 SCI, and Transport Block succeed.
    pub fn evaluate_reception<R: Rng>(
        sinr_db: f64,
        payload_len_bytes: usize,
        mcs_index: u8,
        rng: &mut R,
    ) -> NrReceptionResult {
        // 1. Evaluate PSCCH (Polar code, SCI Stage 1)
        let bler_sci1 = Self::pscch_polar_bler(sinr_db);
        let u1: f64 = rng.gen();
        let sci1_success = u1 > bler_sci1;

        if !sci1_success {
            // Cannot detect or locate PSSCH
            return NrReceptionResult {
                is_success: false,
                sci1_success: false,
                sci2_success: false,
                tb_success: false,
                harq_feedback: NrHarqFeedback::Dtx,
                sinr_db,
            };
        }

        // 2. Evaluate PSSCH SCI Stage 2
        let bler_sci2 = Self::pscch_polar_bler(sinr_db + 0.5); // Slightly smaller payload
        let u2: f64 = rng.gen();
        let sci2_success = u2 > bler_sci2;

        if !sci2_success {
            return NrReceptionResult {
                is_success: false,
                sci1_success: true,
                sci2_success: false,
                tb_success: false,
                harq_feedback: NrHarqFeedback::Dtx,
                sinr_db,
            };
        }

        // 3. Evaluate PSSCH Transport Block (NR LDPC)
        let bler_tb = Self::pssch_ldpc_bler(sinr_db, mcs_index, payload_len_bytes);
        let u3: f64 = rng.gen();
        let tb_success = u3 > bler_tb;

        let harq_feedback = if tb_success {
            NrHarqFeedback::Ack
        } else {
            NrHarqFeedback::Nack
        };

        NrReceptionResult {
            is_success: tb_success,
            sci1_success: true,
            sci2_success: true,
            tb_success,
            harq_feedback,
            sinr_db,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    #[test]
    fn test_high_sinr_dual_decoding() {
        let mut rng = StdRng::seed_from_u64(42);
        // At SINR = 20 dB, MCS 10 (th = 11.2 dB) should succeed easily
        let res = NrPhyBlerModel::evaluate_reception(20.0, 300, 10, &mut rng);
        assert!(res.is_success);
        assert!(res.sci1_success);
        assert!(res.sci2_success);
        assert!(res.tb_success);
        assert_eq!(res.harq_feedback, NrHarqFeedback::Ack);
    }

    #[test]
    fn test_intermediate_sinr_nack_feedback() {
        let mut rng = StdRng::seed_from_u64(42);
        // At SINR = 2.0 dB for MCS 10 (th = 11.2 dB):
        // SCI-1 succeeds (th = -4.0 dB), but TB fails -> Generates NACK!
        let res = NrPhyBlerModel::evaluate_reception(2.0, 300, 10, &mut rng);
        assert!(!res.is_success);
        assert!(res.sci1_success);
        assert!(!res.tb_success);
        assert_eq!(res.harq_feedback, NrHarqFeedback::Nack);
    }

    #[test]
    fn test_very_low_sinr_dtx_feedback() {
        let mut rng = StdRng::seed_from_u64(42);
        // At SINR = -10 dB: SCI-1 fails -> Generates DTX (receiver didn't hear packet)
        let res = NrPhyBlerModel::evaluate_reception(-10.0, 300, 10, &mut rng);
        assert!(!res.is_success);
        assert!(!res.sci1_success);
        assert_eq!(res.harq_feedback, NrHarqFeedback::Dtx);
    }
}
