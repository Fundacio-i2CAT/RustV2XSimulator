//! IEEE 802.11bd Physical Layer Decider.
//!
//! Standards Reference: IEEE Std 802.11bd-2022 Clause 27
//!
//! Implements multi-stage receiver decoding:
//! 1. Clear Channel Assessment (CCA) and sensitivity detection (-89 dBm at 10 MHz).
//! 2. Mixed-format preamble detection (L-SIG and NGV-SIG robust BPSK 1/2 decoding).
//! 3. Vehicular Doppler channel aging & midamble tracking (`DopplerChannelEngine`).
//! 4. Quasi-Cyclic LDPC data payload decoding (`LdpcErrorRate`).
//! 5. Collision differentiation (separating thermal noise drops from co-channel interference).

use rand::Rng;
use serde::{Deserialize, Serialize};

use crate::phy::wifi80211bd::consts_80211bd::{
    compute_duration_80211bd, Mcs80211bd, MidamblePeriod,
};
use crate::phy::wifi80211bd::doppler_model::{DEFAULT_CARRIER_FREQ_HZ, DopplerChannelEngine};
use crate::phy::wifi80211bd::ldpc_error_rate::LdpcErrorRate;

/// Result of evaluating an IEEE 802.11bd packet reception.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeciderResult80211bd {
    /// True if packet was decoded successfully without bit/block errors
    pub is_success: bool,
    /// True if failure was caused specifically by multi-user interference collision
    pub is_collision: bool,
    /// Effective physical bitrate in bits per second
    pub bitrate_bps: u64,
    /// Minimum raw SINR across the packet duration in dB
    pub sinr_min_db: f64,
    /// Effective SINR after Doppler channel aging and midamble tracking in dB
    pub sinr_effective_db: f64,
    /// Received signal power in dBm
    pub rx_power_dbm: f64,
    /// Maximum Doppler spread frequency in Hz
    pub doppler_spread_hz: f64,
}

/// Physical decider module for IEEE 802.11bd.
#[derive(Debug, Clone)]
pub struct Decider80211bd {
    /// Minimum receiver power sensitivity in dBm (-89.0 dBm for 10 MHz)
    pub min_power_dbm: f64,
    /// CCA energy detection threshold in dBm (-85.0 dBm)
    pub cca_threshold_dbm: f64,
    /// Thermal noise floor in dBm (-99.0 dBm for 10 MHz)
    pub noise_floor_dbm: f64,
    /// Carrier center frequency in Hz (default 5.890 GHz)
    pub carrier_freq_hz: f64,
    /// Channel bandwidth in MHz (10.0 or 20.0)
    pub bandwidth_mhz: f64,
    /// Track and differentiate collision statistics
    pub collect_collision_stats: bool,
}

impl Default for Decider80211bd {
    fn default() -> Self {
        Self {
            min_power_dbm: -89.0,
            cca_threshold_dbm: -85.0,
            noise_floor_dbm: -99.0,
            carrier_freq_hz: DEFAULT_CARRIER_FREQ_HZ,
            bandwidth_mhz: 10.0,
            collect_collision_stats: true,
        }
    }
}

impl Decider80211bd {
    pub fn new(bandwidth_mhz: f64, carrier_freq_hz: f64) -> Self {
        let (min_power, noise_floor) = if bandwidth_mhz >= 15.0 {
            (-86.0, -96.0) // 3 dB higher noise floor in 20 MHz
        } else {
            (-89.0, -99.0)
        };
        Self {
            min_power_dbm: min_power,
            cca_threshold_dbm: -85.0,
            noise_floor_dbm: noise_floor,
            carrier_freq_hz,
            bandwidth_mhz,
            collect_collision_stats: true,
        }
    }

    /// Evaluate reception of an incoming IEEE 802.11bd NGV packet.
    pub fn evaluate_packet<R: Rng>(
        &self,
        rx_power_dbm: f64,
        raw_sinr_db: f64,
        payload_bytes: usize,
        mcs: Mcs80211bd,
        midamble: MidamblePeriod,
        use_dcm: bool,
        relative_speed_mps: f64,
        rng: &mut R,
    ) -> DeciderResult80211bd {
        let bitrate_bps = if self.bandwidth_mhz >= 15.0 {
            mcs.bitrate_20mhz_bps(use_dcm)
        } else {
            mcs.bitrate_10mhz_bps(use_dcm)
        };

        let f_d = DopplerChannelEngine::compute_doppler_spread(
            relative_speed_mps,
            self.carrier_freq_hz,
        );

        // Stage 1: Sensitivity check
        if rx_power_dbm < self.min_power_dbm {
            return DeciderResult80211bd {
                is_success: false,
                is_collision: false,
                bitrate_bps,
                sinr_min_db: raw_sinr_db,
                sinr_effective_db: raw_sinr_db,
                rx_power_dbm,
                doppler_spread_hz: f_d,
            };
        }

        let raw_snr_db = rx_power_dbm - self.noise_floor_dbm;
        let pkt_duration = compute_duration_80211bd(
            payload_bytes,
            mcs,
            self.bandwidth_mhz,
            midamble,
            use_dcm,
        );

        // Stage 2: Mixed Preamble decoding (L-SIG + R-SIG repetition combining: +3 dB processing gain)
        let preamble_sinr = raw_sinr_db + 3.0;
        let preamble_snr = raw_snr_db + 3.0;
        let preamble_success_sinr = LdpcErrorRate::packet_success_rate(
            preamble_sinr,
            6, // 48 bits = 6 bytes
            Mcs80211bd::Mcs0Bpsk1_2,
            false,
        );
        let preamble_success_snr = LdpcErrorRate::packet_success_rate(
            preamble_snr,
            6,
            Mcs80211bd::Mcs0Bpsk1_2,
            false,
        );

        let u_preamble: f64 = rng.gen();
        if u_preamble > preamble_success_sinr {
            let is_col = self.collect_collision_stats && (u_preamble <= preamble_success_snr);
            return DeciderResult80211bd {
                is_success: false,
                is_collision: is_col,
                bitrate_bps,
                sinr_min_db: raw_sinr_db,
                sinr_effective_db: raw_sinr_db,
                rx_power_dbm,
                doppler_spread_hz: f_d,
            };
        }

        // Stage 3: Doppler Channel Aging & Midamble Tracking
        let sinr_effective_db = DopplerChannelEngine::compute_effective_sinr(
            raw_sinr_db,
            relative_speed_mps,
            self.carrier_freq_hz,
            pkt_duration,
            midamble,
        );

        let snr_effective_db = DopplerChannelEngine::compute_effective_sinr(
            raw_snr_db,
            relative_speed_mps,
            self.carrier_freq_hz,
            pkt_duration,
            midamble,
        );

        // Stage 4: LDPC Data Payload Decoding
        let payload_success_sinr = LdpcErrorRate::packet_success_rate(
            sinr_effective_db,
            payload_bytes,
            mcs,
            use_dcm,
        );
        let payload_success_snr = LdpcErrorRate::packet_success_rate(
            snr_effective_db,
            payload_bytes,
            mcs,
            use_dcm,
        );

        let u_payload: f64 = rng.gen();
        let is_success = u_payload <= payload_success_sinr;
        let is_collision = self.collect_collision_stats && !is_success && (u_payload <= payload_success_snr);

        DeciderResult80211bd {
            is_success,
            is_collision,
            bitrate_bps,
            sinr_min_db: raw_sinr_db,
            sinr_effective_db,
            rx_power_dbm,
            doppler_spread_hz: f_d,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn test_decider_sensitivity_cutoff() {
        let decider = Decider80211bd::default();
        let mut rng = StdRng::seed_from_u64(42);

        // Power below -89 dBm must fail
        let res = decider.evaluate_packet(
            -95.0,
            20.0,
            300,
            Mcs80211bd::Mcs1Qpsk1_2,
            MidamblePeriod::Every8Symbols,
            false,
            0.0,
            &mut rng,
        );
        assert!(!res.is_success);
    }

    #[test]
    fn test_decider_high_speed_midamble_effect() {
        let decider = Decider80211bd::default();
        let mut rng = StdRng::seed_from_u64(100);

        // Relative speed 250 km/h (approx 70 m/s)
        let v_rel = 70.0;
        let rx_power = -65.0; // High signal power
        let raw_sinr = 20.0; // High SINR

        // Run multiple trials with no midambles vs every 4 symbols
        let mut wins_no_mid = 0;
        let mut wins_with_mid = 0;

        for _ in 0..50 {
            let res_no_mid = decider.evaluate_packet(
                rx_power,
                raw_sinr,
                500,
                Mcs80211bd::Mcs2Qpsk3_4,
                MidamblePeriod::None,
                false,
                v_rel,
                &mut rng,
            );
            if res_no_mid.is_success {
                wins_no_mid += 1;
            }

            let res_with_mid = decider.evaluate_packet(
                rx_power,
                raw_sinr,
                500,
                Mcs80211bd::Mcs2Qpsk3_4,
                MidamblePeriod::Every4Symbols,
                false,
                v_rel,
                &mut rng,
            );
            if res_with_mid.is_success {
                wins_with_mid += 1;
            }
        }

        assert!(wins_with_mid > 40, "Midambles must preserve packet delivery at high speed");
        assert!(wins_no_mid < 15, "Without midambles, Doppler error floor must cause heavy packet loss");
    }
}
