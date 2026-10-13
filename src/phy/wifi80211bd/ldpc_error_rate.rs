//! IEEE 802.11bd LDPC Forward Error Correction (FEC) and Error Rate Model.
//!
//! Standards Reference: IEEE Std 802.11bd-2022 Clause 27 & IEEE 802.11-19/0024r0
//!
//! Models:
//! 1. Quasi-Cyclic Low-Density Parity-Check (QC-LDPC) error waterfall performance for MCS 0 to MCS 9.
//! 2. ~2.5 - 3.0 dB LDPC coding gain over legacy IEEE 802.11p Binary Convolutional Codes (BCC).
//! 3. Dual Carrier Modulation (DCM) frequency diversity gain (~2.0 dB gain for MCS 0).
//! 4. Packet length error scaling: $\text{PER} = 1 - (1 - \text{BLER}_{\text{block}})^{N_{\text{blocks}}}$.

use crate::phy::nist_error_rate::erfc;
use crate::phy::wifi80211bd::consts_80211bd::Mcs80211bd;

/// Q-function: tail distribution function of the standard normal distribution $Q(x) = \frac{1}{2} \text{erfc}(x / \sqrt{2})$.
#[inline]
pub fn q_function(x: f64) -> f64 {
    0.5 * erfc(x / std::f64::consts::SQRT_2)
}

/// LDPC Error Rate Evaluator for IEEE 802.11bd.
pub struct LdpcErrorRate;

impl LdpcErrorRate {
    /// Return the 1% PER SNR threshold in dB for a given MCS (nominal 1000-byte frame).
    ///
    /// Values validated against IEEE 802.11-19/0024r0 and published 802.11bd link-level simulations:
    /// - MCS 0 (BPSK 1/2): 1.5 dB (or -0.5 dB with DCM)
    /// - MCS 1 (QPSK 1/2): 4.0 dB
    /// - MCS 2 (QPSK 3/4): 6.8 dB
    /// - MCS 3 (16-QAM 1/2): 10.2 dB
    /// - MCS 4 (16-QAM 3/4): 13.8 dB
    /// - MCS 5 (64-QAM 2/3): 17.5 dB
    /// - MCS 6 (64-QAM 3/4): 19.2 dB
    /// - MCS 7 (64-QAM 5/6): 20.8 dB
    /// - MCS 8 (256-QAM 3/4): 24.5 dB
    /// - MCS 9 (256-QAM 5/6): 26.2 dB
    pub fn snr_threshold_db(mcs: Mcs80211bd, use_dcm: bool) -> f64 {
        if use_dcm && mcs == Mcs80211bd::Mcs0Bpsk1_2 {
            // Dual Carrier Modulation provides 2.0 dB frequency diversity gain in fading channels
            -0.5
        } else {
            match mcs {
                Mcs80211bd::Mcs0Bpsk1_2 => 1.5,
                Mcs80211bd::Mcs1Qpsk1_2 => 4.0,
                Mcs80211bd::Mcs2Qpsk3_4 => 6.8,
                Mcs80211bd::Mcs3Qam16_1_2 => 10.2,
                Mcs80211bd::Mcs4Qam16_3_4 => 13.8,
                Mcs80211bd::Mcs5Qam64_2_3 => 17.5,
                Mcs80211bd::Mcs6Qam64_3_4 => 19.2,
                Mcs80211bd::Mcs7Qam64_5_6 => 20.8,
                Mcs80211bd::Mcs8Qam256_3_4 => 24.5,
                Mcs80211bd::Mcs9Qam256_5_6 => 26.2,
            }
        }
    }

    /// Steepness factor for LDPC waterfall transition in dB.
    /// LDPC codes exhibit a much steeper waterfall cliff than convolutional codes.
    #[inline]
    pub fn transition_width_db(mcs: Mcs80211bd) -> f64 {
        match mcs {
            Mcs80211bd::Mcs0Bpsk1_2 | Mcs80211bd::Mcs1Qpsk1_2 => 0.45,
            Mcs80211bd::Mcs2Qpsk3_4 | Mcs80211bd::Mcs3Qam16_1_2 => 0.50,
            Mcs80211bd::Mcs4Qam16_3_4 | Mcs80211bd::Mcs5Qam64_2_3 => 0.55,
            Mcs80211bd::Mcs6Qam64_3_4 | Mcs80211bd::Mcs7Qam64_5_6 => 0.60,
            Mcs80211bd::Mcs8Qam256_3_4 | Mcs80211bd::Mcs9Qam256_5_6 => 0.65,
        }
    }

    /// Calculate LDPC Block Error Rate (BLER) for a single codeword (up to 1944 bits / ~243 bytes).
    #[inline]
    pub fn ldpc_block_error_rate(sinr_db: f64, mcs: Mcs80211bd, use_dcm: bool) -> f64 {
        let gamma_th = Self::snr_threshold_db(mcs, use_dcm);
        let width = Self::transition_width_db(mcs);
        let arg = (sinr_db - gamma_th) / width;

        if arg > 6.0 {
            0.0 // Far above threshold: zero error
        } else if arg < -6.0 {
            1.0 // Far below threshold: total failure
        } else {
            // Shifted Q-function centered at 1% target (arg=0 -> BLER approx 0.01)
            let q = q_function(arg + 2.326);
            q.max(0.0).min(1.0)
        }
    }

    /// Calculate Packet Error Rate (PER) for an entire MPDU payload of `payload_bytes`.
    pub fn packet_error_rate(
        sinr_db: f64,
        payload_bytes: usize,
        mcs: Mcs80211bd,
        use_dcm: bool,
    ) -> f64 {
        if payload_bytes == 0 {
            return 0.0;
        }

        // LDPC codeword size in 802.11 is up to 1944 bits (approx 243 bytes payload)
        let block_payload_bytes = 243;
        let num_blocks = ((payload_bytes + block_payload_bytes - 1) / block_payload_bytes).max(1);

        let bler_block = Self::ldpc_block_error_rate(sinr_db, mcs, use_dcm);
        if bler_block <= 0.0 {
            0.0
        } else if bler_block >= 1.0 {
            1.0
        } else {
            1.0 - (1.0 - bler_block).powi(num_blocks as i32)
        }
    }

    /// Calculate Packet Success Rate (1.0 - PER).
    #[inline]
    pub fn packet_success_rate(
        sinr_db: f64,
        payload_bytes: usize,
        mcs: Mcs80211bd,
        use_dcm: bool,
    ) -> f64 {
        1.0 - Self::packet_error_rate(sinr_db, payload_bytes, mcs, use_dcm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ldpc_waterfall_thresholds() {
        // At threshold SNR + 3 dB, PER should be practically 0
        let per_high = LdpcErrorRate::packet_error_rate(10.0, 300, Mcs80211bd::Mcs1Qpsk1_2, false);
        assert!(per_high < 1e-4);

        // At threshold SNR - 3 dB, PER should be 1.0
        let per_low = LdpcErrorRate::packet_error_rate(0.0, 300, Mcs80211bd::Mcs1Qpsk1_2, false);
        assert!(per_low > 0.99);
    }

    #[test]
    fn test_dcm_gain_on_mcs0() {
        // At 0.5 dB:
        // Standard MCS 0 has threshold 1.5 dB (fails)
        let per_std = LdpcErrorRate::packet_error_rate(0.5, 300, Mcs80211bd::Mcs0Bpsk1_2, false);
        // DCM MCS 0 has threshold -0.5 dB (succeeds)
        let per_dcm = LdpcErrorRate::packet_error_rate(0.5, 300, Mcs80211bd::Mcs0Bpsk1_2, true);

        assert!(per_std > 0.1);
        assert!(per_dcm < 0.02);
    }
}
