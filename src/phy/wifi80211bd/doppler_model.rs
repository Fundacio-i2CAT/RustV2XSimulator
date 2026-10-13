//! Vehicular Doppler Channel Aging and Midamble Tracking Model for IEEE 802.11bd.
//!
//! Standards Reference: IEEE Std 802.11bd-2022 Clause 27 & IEEE 802.11-19/0024r0
//!
//! Models:
//! 1. Maximum Doppler frequency shift $f_d = \frac{v_{\text{rel}}}{\lambda}$ at 5.9 GHz ITS band.
//! 2. Jakes / Clarke temporal channel autocorrelation $R_H(\Delta t) = J_0(2\pi f_d \Delta t)$
//!    via high-precision Abramowitz & Stegun polynomial approximation.
//! 3. Channel estimation error variance $\sigma_e^2(\Delta t) = 2 \left(1 - J_0(2\pi f_d \Delta t)\right)$.
//! 4. Effective SINR degradation and Doppler error floor:
//!    $\text{SINR}_{\text{eff}} = \frac{\text{SINR}}{1 + \text{SINR} \cdot \sigma_e^2(\Delta t_{\text{eff}})}$.
//! 5. Periodic midamble channel re-estimation bounding $\Delta t_{\text{eff}} \le \frac{N_{\text{mid}} T_{\text{sym}}}{2}$.

use crate::common::SimTime;
use crate::phy::wifi80211bd::consts_80211bd::{MidamblePeriod, T_SYM_10MHZ_US};

/// Speed of light in vacuum (m/s)
pub const SPEED_OF_LIGHT_M_S: f64 = 299_792_458.0;

/// Default V2X carrier frequency in Hertz (5.890 GHz ITS Band)
pub const DEFAULT_CARRIER_FREQ_HZ: f64 = 5.890e9;

/// Compute zeroth-order Bessel function of the first kind $J_0(x)$
/// using Abramowitz and Stegun Section 9.4 polynomial approximation.
/// Maximum absolute error $|e(x)| < 1.0 \times 10^{-7}$.
pub fn bessel_j0(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 3.0 {
        let y = (x / 3.0) * (x / 3.0);
        1.0 + y * (-2.2499997 + y * (1.2656208 + y * (-0.3163866 + y * (0.0444479 + y * (-0.0039444 + y * 0.0002100)))))
    } else {
        let y = 3.0 / ax;
        let f0 = 0.79788456 + y * (-0.00000077 + y * (-0.00552740 + y * (0.00009512 + y * (0.00137237 + y * (-0.00072805 + y * 0.00014476)))));
        let g0 = -0.04166397 + y * (-0.00003954 + y * (0.00262573 + y * (-0.00003986 + y * (-0.00029386 + y * (0.00013558 + y * -0.00002035)))));
        let theta = ax - std::f64::consts::FRAC_PI_4;
        (1.0 / ax.sqrt()) * (f0 * theta.cos() - g0 * theta.sin())
    }
}

/// Vehicular Doppler and Midamble Tracking Engine.
pub struct DopplerChannelEngine;

impl DopplerChannelEngine {
    /// Calculate maximum Doppler spread frequency $f_d$ in Hertz.
    #[inline]
    pub fn compute_doppler_spread(relative_speed_mps: f64, carrier_freq_hz: f64) -> f64 {
        let wavelength_m = SPEED_OF_LIGHT_M_S / carrier_freq_hz;
        (relative_speed_mps / wavelength_m).abs()
    }

    /// Calculate channel temporal autocorrelation $R_H(\Delta t) = J_0(2\pi f_d \Delta t)$.
    #[inline]
    pub fn channel_correlation(doppler_spread_hz: f64, delta_t_s: f64) -> f64 {
        if doppler_spread_hz <= 1e-3 || delta_t_s <= 1e-9 {
            return 1.0;
        }
        let arg = 2.0 * std::f64::consts::PI * doppler_spread_hz * delta_t_s;
        bessel_j0(arg)
    }

    /// Calculate channel estimation error variance $\sigma_e^2(\Delta t)$.
    #[inline]
    pub fn estimation_error_variance(doppler_spread_hz: f64, delta_t_s: f64) -> f64 {
        let rho = Self::channel_correlation(doppler_spread_hz, delta_t_s);
        // Error variance bounded between [0.0, 2.0]
        (2.0 * (1.0 - rho)).max(0.0).min(2.0)
    }

    /// Compute effective elapsed channel aging time $\Delta t_{\text{eff}}$ given midamble configuration.
    ///
    /// - With `MidamblePeriod::None` (or legacy 802.11p), $\Delta t_{\text{eff}} = T_{\text{data}} / 2$.
    /// - With `MidamblePeriod::EveryN`, channel estimation is refreshed every $N_{\text{mid}}$ symbols:
    ///   $\Delta t_{\text{eff}} = \min(T_{\text{data}} / 2, \frac{N_{\text{mid}} \times T_{\text{sym}}}{2})$.
    pub fn compute_effective_aging_time(
        data_duration: SimTime,
        midamble: MidamblePeriod,
        t_sym_us: u64,
    ) -> f64 {
        let total_data_s = data_duration.to_secs_f64();
        match midamble {
            MidamblePeriod::None => (total_data_s / 2.0).max(0.0),
            MidamblePeriod::Every4Symbols
            | MidamblePeriod::Every8Symbols
            | MidamblePeriod::Every16Symbols => {
                let interval_us = (midamble.interval_symbols() as u64) * t_sym_us;
                let midamble_max_gap_s = (interval_us as f64) * 1e-6;
                // Average aging distance between successive midamble pilots is interval / 2
                (midamble_max_gap_s / 2.0).min(total_data_s / 2.0)
            }
        }
    }

    /// Compute effective SINR (dB) after Doppler channel aging and midamble tracking.
    ///
    /// $\text{SINR}_{\text{eff}} = \frac{\text{SINR}_{\text{linear}}}{1 + \text{SINR}_{\text{linear}} \cdot \sigma_e^2}$
    pub fn compute_effective_sinr(
        raw_sinr_db: f64,
        relative_speed_mps: f64,
        carrier_freq_hz: f64,
        data_duration: SimTime,
        midamble: MidamblePeriod,
    ) -> f64 {
        if relative_speed_mps <= 0.5 {
            // Stationary or negligible Doppler shift
            return raw_sinr_db;
        }

        let f_d = Self::compute_doppler_spread(relative_speed_mps, carrier_freq_hz);
        let delta_t_s = Self::compute_effective_aging_time(data_duration, midamble, T_SYM_10MHZ_US);
        let sigma_sq = Self::estimation_error_variance(f_d, delta_t_s);

        if sigma_sq <= 1e-6 {
            return raw_sinr_db;
        }

        let raw_sinr_linear = 10.0_f64.powf(raw_sinr_db / 10.0);
        let eff_sinr_linear = raw_sinr_linear / (1.0 + raw_sinr_linear * sigma_sq);
        10.0 * eff_sinr_linear.max(1e-9).log10()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bessel_j0_accuracy() {
        // Known values:
        // J0(0) = 1.0
        assert!((bessel_j0(0.0) - 1.0).abs() < 1e-6);
        // J0(2.4048255577) = 0.0 (first zero)
        assert!((bessel_j0(2.4048255577)).abs() < 1e-4);
        // J0(3.8317) approx -0.402759
        assert!((bessel_j0(3.8317) - (-0.402759)).abs() < 1e-3);
    }

    #[test]
    fn test_doppler_spread_computation() {
        // At 200 km/h = 55.556 m/s, carrier = 5.89 GHz:
        // Wavelength = 299792458 / 5.89e9 = 0.0509 m
        // f_d = 55.556 / 0.0509 = 1091.5 Hz
        let fd = DopplerChannelEngine::compute_doppler_spread(55.556, 5.89e9);
        assert!((fd - 1091.5).abs() < 5.0);
    }

    #[test]
    fn test_midamble_doppler_mitigation() {
        // High speed scenario: relative speed 216 km/h = 60 m/s
        let v_rel = 60.0;
        let carrier = 5.89e9;
        let data_dur = SimTime::from_micros(600); // 600 us long packet
        let raw_sinr = 25.0; // High raw SNR (25 dB)

        // Without midambles (None / 802.11p-like aging):
        let sinr_no_mid = DopplerChannelEngine::compute_effective_sinr(
            raw_sinr,
            v_rel,
            carrier,
            data_dur,
            MidamblePeriod::None,
        );

        // With midambles every 4 symbols:
        let sinr_with_mid = DopplerChannelEngine::compute_effective_sinr(
            raw_sinr,
            v_rel,
            carrier,
            data_dur,
            MidamblePeriod::Every4Symbols,
        );

        // Effective SINR without midambles should be degraded by at least 15 dB due to Doppler aging!
        assert!(sinr_no_mid < 10.0, "Without midambles, SINR must suffer severe Doppler error floor: got {}", sinr_no_mid);

        // With midambles, SINR is preserved (> 18 dB):
        assert!(sinr_with_mid > 18.0, "With midambles, SINR should remain high: got {}", sinr_with_mid);
        assert!(sinr_with_mid > sinr_no_mid + 10.0);
    }
}
