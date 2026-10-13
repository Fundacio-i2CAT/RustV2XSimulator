//! Piecewise-constant Chunked Cumulative SINR Tracking (ns-3 v3.42 InterferenceHelper parity).
//!
//! Evaluates receiver capture effect (gamma_cap = 4.0 dB) and analytical PER mapping.

use crate::common::SimTime;
use rand::Rng;
use serde::{Deserialize, Serialize};

#[inline]
pub fn dbm_to_watts(dbm: f64) -> f64 {
    10.0f64.powf((dbm - 30.0) / 10.0)
}

#[inline]
pub fn watts_to_dbm(watts: f64) -> f64 {
    if watts <= 0.0 {
        -150.0
    } else {
        10.0 * watts.log10() + 30.0
    }
}

/// An active signal transmission observed at a specific receiver.
#[derive(Debug, Clone)]
pub struct SignalEvent {
    pub signal_id: u64,
    pub rx_power_dbm: f64,
    pub rx_power_watts: f64,
    pub start_time: SimTime,
    pub end_time: SimTime,
    pub is_desired: bool,
}

/// Piecewise constant interference chunk during a packet reception.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SinrChunk {
    pub duration_seconds: f64,
    pub desired_power_watts: f64,
    pub interference_power_watts: f64,
    pub noise_power_watts: f64,
    pub sinr_linear: f64,
    pub sinr_db: f64,
}

#[derive(Debug, Clone)]
pub struct CumulativeInterferenceTracker {
    pub noise_floor_dbm: f64,
    pub noise_floor_watts: f64,
    pub capture_threshold_db: f64,
}

impl Default for CumulativeInterferenceTracker {
    fn default() -> Self {
        Self::new(-99.0, 4.0)
    }
}

impl CumulativeInterferenceTracker {
    pub fn new(noise_floor_dbm: f64, capture_threshold_db: f64) -> Self {
        Self {
            noise_floor_dbm,
            noise_floor_watts: dbm_to_watts(noise_floor_dbm),
            capture_threshold_db,
        }
    }

    /// Track interference and evaluate packet decoding probability for a desired signal
    /// in the presence of concurrent interfering signals.
    pub fn evaluate_reception<R: Rng>(
        &self,
        desired: &SignalEvent,
        interferers: &[SignalEvent],
        payload_len_bytes: usize,
        rng: &mut R,
    ) -> (bool, f64, Vec<SinrChunk>) {
        if desired.rx_power_dbm < self.noise_floor_dbm {
            // Signal below thermal noise floor
            return (false, -100.0, Vec::new());
        }

        // Collect all critical boundary timestamps within [desired.start_time, desired.end_time]
        let mut time_points = vec![desired.start_time, desired.end_time];
        for intf in interferers {
            if intf.signal_id == desired.signal_id {
                continue;
            }
            if intf.end_time > desired.start_time && intf.start_time < desired.end_time {
                if intf.start_time > desired.start_time && intf.start_time < desired.end_time {
                    time_points.push(intf.start_time);
                }
                if intf.end_time > desired.start_time && intf.end_time < desired.end_time {
                    time_points.push(intf.end_time);
                }
            }
        }

        time_points.sort();
        time_points.dedup();

        let mut chunks = Vec::with_capacity(time_points.len().saturating_sub(1));
        let mut overall_psr = 1.0f64;
        let mut min_sinr_db = f64::MAX;
        let mut mean_sinr_db_acc = 0.0;
        let total_duration_s = (desired.end_time - desired.start_time)
            .to_secs_f64()
            .max(1e-9);

        for window in time_points.windows(2) {
            let t_start = window[0];
            let t_end = window[1];
            let dt_s = (t_end - t_start).to_secs_f64();
            if dt_s <= 0.0 {
                continue;
            }

            // Middle of the chunk
            let t_mid = SimTime::from_ticks((t_start.ticks() + t_end.ticks()) / 2);

            // Compute total interference power in this chunk (excluding the desired signal itself)
            let mut chunk_interf_watts = 0.0f64;
            for intf in interferers {
                if intf.signal_id != desired.signal_id && intf.is_active_at(t_mid) {
                    chunk_interf_watts += intf.rx_power_watts;
                }
            }

            let denom = chunk_interf_watts + self.noise_floor_watts;
            let sinr_linear = desired.rx_power_watts / denom.max(1e-15);
            let sinr_db = 10.0 * sinr_linear.log10();

            if sinr_db < min_sinr_db {
                min_sinr_db = sinr_db;
            }
            mean_sinr_db_acc += sinr_db * (dt_s / total_duration_s);

            chunks.push(SinrChunk {
                duration_seconds: dt_s,
                desired_power_watts: desired.rx_power_watts,
                interference_power_watts: chunk_interf_watts,
                noise_power_watts: self.noise_floor_watts,
                sinr_linear,
                sinr_db,
            });

            // Receiver capture check:
            // If interference is present, does desired signal overcome interference by capture margin?
            if chunk_interf_watts > 0.0 {
                let sir_db = 10.0 * (desired.rx_power_watts / chunk_interf_watts).log10();
                if sir_db < self.capture_threshold_db && sinr_db < 6.0 {
                    // Collision without capture
                    overall_psr *= 0.0;
                    break;
                }
            }

            // Analytical PER calculation for 802.11p QPSK 1/2 with convolutional coding (ns-3 parity)
            let chunk_per = compute_80211p_coded_per(sinr_db, payload_len_bytes);
            let chunk_psr = 1.0 - chunk_per;
            overall_psr *= chunk_psr.clamp(0.0, 1.0);
        }

        let is_success = if overall_psr <= 0.0 {
            false
        } else if overall_psr >= 0.9999 {
            true
        } else {
            let u: f64 = rng.gen();
            u < overall_psr
        };

        (is_success, mean_sinr_db_acc, chunks)
    }
}

/// Analytical Packet Error Rate (PER) for IEEE 802.11p OFDM 6 Mbps (QPSK 1/2 with convolutional coding).
/// Directly evaluates the NIST Error Rate model from Veins / ns-3.
#[inline]
pub fn compute_80211p_coded_per(sinr_db: f64, payload_len_bytes: usize) -> f64 {
    let mpdu_bits = 16 + (payload_len_bytes * 8) + 6;
    let psr = crate::phy::nist_error_rate::get_global_nist_lut().fast_chunk_success_rate(
        crate::phy::consts_80211p::Mcs80211p::QpskR1_2,
        sinr_db,
        mpdu_bits,
    );
    (1.0 - psr).clamp(0.0, 1.0)
}
#[inline]
#[allow(dead_code)]
fn compute_qpsk_ber(sinr_linear: f64) -> f64 {
    if sinr_linear <= 0.0 {
        0.5
    } else {
        0.5 * erfc((sinr_linear.sqrt()) / std::f64::consts::SQRT_2)
    }
}

/// Complementary error function approximation (Chebyshev fitting).
#[inline]
#[allow(dead_code)]
fn erfc(x: f64) -> f64 {
    let z = x.abs();
    let t = 1.0 / (1.0 + 0.5 * z);
    let ans = t
        * (-z * z - 1.26551223
            + t * (1.00002368
                + t * (0.37409196
                    + t * (0.09678418
                        + t * (-0.18628806
                            + t * (0.27886807
                                + t * (-1.13520398
                                    + t * (1.48851587 + t * (-0.82215223 + t * 0.17087277)))))))));
    let res = ans.exp();
    if x >= 0.0 {
        res
    } else {
        2.0 - res
    }
}

impl SignalEvent {
    #[inline]
    pub fn is_active_at(&self, t: SimTime) -> bool {
        t >= self.start_time && t < self.end_time
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn test_capture_effect() {
        let tracker = CumulativeInterferenceTracker::new(-99.0, 4.0);
        let mut rng = StdRng::seed_from_u64(42);

        let t_start = SimTime::from_millis(0);
        let t_end = SimTime::from_millis(1);

        let desired = SignalEvent {
            signal_id: 1,
            rx_power_dbm: -70.0,
            rx_power_watts: dbm_to_watts(-70.0),
            start_time: t_start,
            end_time: t_end,
            is_desired: true,
        };

        // Interferer with -80.0 dBm (10 dB weaker than desired -> capture should succeed!)
        let weak_intf = SignalEvent {
            signal_id: 2,
            rx_power_dbm: -80.0,
            rx_power_watts: dbm_to_watts(-80.0),
            start_time: t_start,
            end_time: t_end,
            is_desired: false,
        };

        let (success, sinr, _) = tracker.evaluate_reception(&desired, &[weak_intf], 300, &mut rng);
        assert!(success);
        assert!(sinr > 8.0);
    }
}
