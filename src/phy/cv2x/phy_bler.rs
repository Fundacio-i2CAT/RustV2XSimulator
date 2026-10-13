//! 3GPP Rel-14 C-V2X Sidelink Physical Layer BLER Model.
//!
//! Models block error rates (BLER) under AWGN and vehicular multipath fading:
//! - Dual physical channels: PSCCH (SCI Format 1) and PSSCH (Transport Block)
//! - Dual decoding condition: a transmission is received if and only if BOTH the
//!   PSCCH (SCI) and PSSCH (TB) decode without error
//! - Turbo coding BLER curves parameterized per 3GPP TR 36.885 vehicular calibration

use rand::Rng;
use serde::{Deserialize, Serialize};

/// Sidelink frame decoding result.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SidelinkReceptionResult {
    /// True if both PSCCH (SCI) and PSSCH (TB) decoded successfully
    pub is_success: bool,
    /// True if PSCCH (SCI) decoded successfully
    pub sci_success: bool,
    /// True if PSSCH (Transport Block) decoded successfully
    pub tb_success: bool,
    /// Received SINR in dB
    pub sinr_db: f64,
}

/// Sidelink Physical Layer BLER Evaluator.
pub struct SidelinkPhyModel;

impl SidelinkPhyModel {
    /// Calculate PSCCH (SCI) Block Error Rate (BLER) as a function of SINR in dB.
    ///
    /// Per 3GPP Rel-14 TR 36.885, PSCCH occupies 2 RBs and uses robust QPSK modulation
    /// with convolutional coding, transitioning around -2.5 dB SINR.
    #[inline]
    pub fn pscch_sci_bler(sinr_db: f64) -> f64 {
        if sinr_db < -6.0 {
            1.0
        } else if sinr_db > 1.0 {
            0.0
        } else {
            // Sigmoidal transition centered at -2.5 dB
            let exponent = 2.0 * (sinr_db - (-2.5));
            (1.0 / (1.0 + exponent.exp())).clamp(0.0, 1.0)
        }
    }

    /// Calculate PSSCH (Transport Block) Block Error Rate (BLER) as a function of SINR in dB,
    /// payload length, and MCS.
    ///
    /// Per 3GPP Rel-14 TR 36.885 calibration for MCS 7 (QPSK rate 0.58):
    /// Transition occurs around +3.0 dB to +5.0 dB depending on packet size.
    #[inline]
    pub fn pssch_tb_bler(sinr_db: f64, payload_len_bytes: usize, mcs: u8) -> f64 {
        // Shift threshold based on MCS (higher MCS requires higher SINR)
        let mcs_offset = (mcs as f64 - 7.0) * 1.2;
        let center_sinr = 3.5 + mcs_offset;

        if sinr_db < center_sinr - 4.0 {
            1.0
        } else if sinr_db > center_sinr + 6.0 {
            0.0
        } else {
            let size_factor = (payload_len_bytes as f64 / 300.0).clamp(0.8, 2.0);
            let exponent = 1.8 * (sinr_db - center_sinr);
            let bler = 1.0 / (1.0 + exponent.exp());
            (bler * size_factor).clamp(0.0, 1.0)
        }
    }

    /// Evaluate dual reception of PSCCH (SCI) and PSSCH (TB).
    pub fn evaluate_reception<R: Rng>(
        sinr_db: f64,
        payload_len_bytes: usize,
        mcs: u8,
        rng: &mut R,
    ) -> SidelinkReceptionResult {
        // Step 1: Decode PSCCH (SCI)
        let sci_bler = Self::pscch_sci_bler(sinr_db);
        let u_sci: f64 = rng.gen();
        let sci_success = u_sci > sci_bler;

        if !sci_success {
            // Without SCI, the receiver cannot identify PSSCH resource location or demodulate TB
            return SidelinkReceptionResult {
                is_success: false,
                sci_success: false,
                tb_success: false,
                sinr_db,
            };
        }

        // Step 2: Decode PSSCH (Transport Block)
        let tb_bler = Self::pssch_tb_bler(sinr_db, payload_len_bytes, mcs);
        let u_tb: f64 = rng.gen();
        let tb_success = u_tb > tb_bler;

        SidelinkReceptionResult {
            is_success: tb_success,
            sci_success: true,
            tb_success,
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
        let res = SidelinkPhyModel::evaluate_reception(15.0, 300, 7, &mut rng);
        assert!(res.is_success);
        assert!(res.sci_success);
        assert!(res.tb_success);
    }

    #[test]
    fn test_low_sinr_failure() {
        let mut rng = StdRng::seed_from_u64(42);
        let res = SidelinkPhyModel::evaluate_reception(-8.0, 300, 7, &mut rng);
        assert!(!res.is_success);
        assert!(!res.sci_success);
    }
}
