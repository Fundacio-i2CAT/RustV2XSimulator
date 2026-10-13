//! IEEE 802.11p Physical Layer Decider.
//!
//! Direct translation of Veins / OMNeT++ `Decider80211p.cc` and `Decider80211p.h`:
//! - Receives frame power and evaluates against sensitivity threshold
//! - Decodes PLCP Signal header (24 bits at 3 Mbps BPSK 1/2)
//! - Decodes MPDU data payload (16-bit Service + payload + 6-bit Tail)
//! - Differentiates packet errors between thermal noise/attenuation and interference collisions
//! - Uses precomputed NIST WiFi error rate model (`get_global_nist_lut`) for ultra-fast performance

use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::phy::consts_80211p::{
    Mcs80211p, PHY_HDR_PLCPSIGNAL_BITS, PHY_HDR_SERVICE_BITS, PHY_TAIL_BITS,
};
use crate::phy::nist_error_rate::get_global_nist_lut;

/// Result of evaluating a packet reception in the decider.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PacketDecodeStatus {
    /// Packet was decoded with no bit errors
    Decoded,
    /// Packet had bit errors due to low signal power (noise attenuation)
    NotDecoded,
    /// Packet had bit errors due to interference from other concurrent signals
    Collision,
}

/// Decider evaluation output matching Veins `DeciderResult80211`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeciderResult80211 {
    /// True if the packet was successfully received and decoded without bit errors
    pub is_success: bool,
    /// True if the packet failure was caused specifically by a collision/interference
    pub is_collision: bool,
    /// Reception bitrate in bits per second
    pub bitrate_bps: u64,
    /// Minimum SINR observed over the packet duration in dB
    pub sinr_min_db: f64,
    /// Received signal power in dBm
    pub rx_power_dbm: f64,
}

/// IEEE 802.11p physical decider module.
#[derive(Debug, Clone)]
pub struct Decider80211p {
    /// Minimum receiving power level sensitivity in dBm (default -89.0 dBm)
    pub min_power_dbm: f64,
    /// Clear Channel Assessment (CCA) threshold in dBm (default -85.0 dBm)
    pub cca_threshold_dbm: f64,
    /// Thermal noise floor in dBm (default -99.0 dBm)
    pub noise_floor_dbm: f64,
    /// Track and differentiate collision statistics
    pub collect_collision_stats: bool,
}

impl Default for Decider80211p {
    fn default() -> Self {
        Self {
            min_power_dbm: -89.0,
            cca_threshold_dbm: -85.0,
            noise_floor_dbm: -99.0,
            collect_collision_stats: true,
        }
    }
}

impl Decider80211p {
    pub fn new(min_power_dbm: f64, cca_threshold_dbm: f64, noise_floor_dbm: f64) -> Self {
        Self {
            min_power_dbm,
            cca_threshold_dbm,
            noise_floor_dbm,
            collect_collision_stats: true,
        }
    }

    /// Evaluate reception of an incoming 802.11p packet.
    ///
    /// Parity with `Decider80211p::packetOk` in Veins:
    /// 1. Verifies receiving power exceeds sensitivity (`min_power_dbm`).
    /// 2. Evaluates PLCP header error probability (24 bits at 3 Mbps BPSK 1/2).
    /// 3. Evaluates MPDU payload error probability (SERVICE + Payload + TAIL) at active MCS.
    /// 4. Checks whether failure was due to noise (SNR) or interference collision (SINR).
    pub fn evaluate_packet<R: Rng>(
        &self,
        rx_power_dbm: f64,
        sinr_min_db: f64,
        payload_len_bytes: usize,
        mcs: Mcs80211p,
        rng: &mut R,
    ) -> DeciderResult80211 {
        // If received power is below minimum sensitivity, frame cannot be synchronized or detected
        if rx_power_dbm < self.min_power_dbm {
            return DeciderResult80211 {
                is_success: false,
                is_collision: false,
                bitrate_bps: mcs.bitrate_bps(),
                sinr_min_db,
                rx_power_dbm,
            };
        }

        let snr_min_db = rx_power_dbm - self.noise_floor_dbm;
        let lut = get_global_nist_lut();

        // 1. Evaluate PLCP Signal Header (24 bits transmitted at 3 Mbps BPSK 1/2)
        let header_mcs = Mcs80211p::BpskR1_2;
        let header_success_sinr = lut.fast_chunk_success_rate(header_mcs, sinr_min_db, PHY_HDR_PLCPSIGNAL_BITS);
        let header_success_snr = lut.fast_chunk_success_rate(header_mcs, snr_min_db, PHY_HDR_PLCPSIGNAL_BITS);

        let u_header: f64 = rng.gen();
        if u_header > header_success_sinr {
            // Header decoding failed
            let is_collision = self.collect_collision_stats && (u_header <= header_success_snr);
            return DeciderResult80211 {
                is_success: false,
                is_collision,
                bitrate_bps: mcs.bitrate_bps(),
                sinr_min_db,
                rx_power_dbm,
            };
        }

        // 2. Evaluate MPDU Data Payload
        let mpdu_bits = PHY_HDR_SERVICE_BITS + (payload_len_bytes * 8) + PHY_TAIL_BITS;
        let payload_success_sinr = lut.fast_chunk_success_rate(mcs, sinr_min_db, mpdu_bits);
        let payload_success_snr = lut.fast_chunk_success_rate(mcs, snr_min_db, mpdu_bits);

        let u_payload: f64 = rng.gen();
        if u_payload > payload_success_sinr {
            // Payload decoding failed
            let is_collision = self.collect_collision_stats && (u_payload <= payload_success_snr);
            DeciderResult80211 {
                is_success: false,
                is_collision,
                bitrate_bps: mcs.bitrate_bps(),
                sinr_min_db,
                rx_power_dbm,
            };
        } else {
            // Successfully decoded!
            DeciderResult80211 {
                is_success: true,
                is_collision: false,
                bitrate_bps: mcs.bitrate_bps(),
                sinr_min_db,
                rx_power_dbm,
            };
        }

        // Return result
        let is_ok = u_payload <= payload_success_sinr;
        let is_col = self.collect_collision_stats && !is_ok && (u_payload <= payload_success_snr);

        DeciderResult80211 {
            is_success: is_ok,
            is_collision: is_col,
            bitrate_bps: mcs.bitrate_bps(),
            sinr_min_db,
            rx_power_dbm,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    #[test]
    fn test_decider_high_snr_success() {
        let decider = Decider80211p::default();
        let mut rng = StdRng::seed_from_u64(42);

        // Good signal: -65 dBm, SINR = 25 dB
        let res = decider.evaluate_packet(-65.0, 25.0, 300, Mcs80211p::QpskR1_2, &mut rng);
        assert!(res.is_success);
        assert!(!res.is_collision);
    }

    #[test]
    fn test_decider_below_sensitivity() {
        let decider = Decider80211p::default();
        let mut rng = StdRng::seed_from_u64(42);

        // Signal below -89 dBm
        let res = decider.evaluate_packet(-95.0, 20.0, 300, Mcs80211p::QpskR1_2, &mut rng);
        assert!(!res.is_success);
    }

    #[test]
    fn test_decider_collision_detection() {
        let decider = Decider80211p::default();
        let mut rng = StdRng::seed_from_u64(42);

        // Good power (-70 dBm -> SNR ~ 29 dB), but low SINR (0 dB) due to strong interferer
        let res = decider.evaluate_packet(-70.0, 0.0, 300, Mcs80211p::QpskR1_2, &mut rng);
        assert!(!res.is_success);
        assert!(res.is_collision, "Should be categorized as collision");
    }
}
