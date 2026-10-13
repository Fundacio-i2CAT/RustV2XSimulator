//! IEEE 1609.4 Multi-Channel Operation (CCH/SCH Alternating Switching).
//!
//! Direct translation of Veins / OMNeT++ `Mac1609_4.cc` multi-channel switching:
//! - 50 ms sync intervals alternating between Control Channel (CCH, ch 178) and Service Channel (SCH, ch 174..182)
//! - 4 ms guard intervals during which channel switching and transceiver retuning occur
//! - Prevents transmission during guard intervals
//! - Supports single-channel continuous CCH (standard European ETSI ITS-G5 default) and dual-channel switching (US WAVE)

use serde::{Deserialize, Serialize};
use crate::common::SimTime;
use crate::phy::consts_80211p::{DsrcChannel, GUARD_INTERVAL_MS, SWITCHING_INTERVAL_MS};

/// IEEE 1609.4 Channel Type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ChannelPhase {
    /// Control Channel (CCH) active phase
    Control,
    /// Service Channel (SCH) active phase
    Service,
    /// Guard interval (transceiver switching / synchronization)
    Guard,
}

/// Multi-Channel Operation Mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum MultiChannelMode {
    /// Continuous listening and sending on CCH (standard ETSI C-ITS / ITS-G5 single transceiver profile)
    ContinuousCch,
    /// IEEE 1609.4 alternating 50 ms CCH / SCH switching
    AlternatingSwitching,
}

impl Default for MultiChannelMode {
    fn default() -> Self {
        Self::ContinuousCch // Standard ETSI ITS-G5 default
    }
}

/// IEEE 1609.4 Channel Switch Coordinator.
#[derive(Debug, Clone)]
pub struct ChannelSwitchCoordinator {
    pub mode: MultiChannelMode,
    pub switching_interval: SimTime,
    pub guard_interval: SimTime,
    pub service_channel: DsrcChannel,
}

impl Default for ChannelSwitchCoordinator {
    fn default() -> Self {
        Self {
            mode: MultiChannelMode::ContinuousCch,
            switching_interval: SimTime::from_millis(SWITCHING_INTERVAL_MS),
            guard_interval: SimTime::from_millis(GUARD_INTERVAL_MS),
            service_channel: DsrcChannel::Sch1,
        }
    }
}

impl ChannelSwitchCoordinator {
    pub fn new(mode: MultiChannelMode, service_channel: DsrcChannel) -> Self {
        Self {
            mode,
            switching_interval: SimTime::from_millis(SWITCHING_INTERVAL_MS),
            guard_interval: SimTime::from_millis(GUARD_INTERVAL_MS),
            service_channel,
        }
    }

    /// Query active channel phase at given simulation time `now`.
    pub fn current_phase(&self, now: SimTime) -> ChannelPhase {
        if self.mode == MultiChannelMode::ContinuousCch {
            return ChannelPhase::Control;
        }

        let interval_ticks = self.switching_interval.ticks();
        let guard_ticks = self.guard_interval.ticks();
        let now_ticks = now.ticks();

        let cycle_index = now_ticks / interval_ticks;
        let time_in_cycle = now_ticks % interval_ticks;

        if time_in_cycle < guard_ticks {
            ChannelPhase::Guard
        } else if cycle_index % 2 == 0 {
            ChannelPhase::Control
        } else {
            ChannelPhase::Service
        }
    }

    /// Query the currently active radio channel (or None if in Guard interval).
    pub fn active_channel(&self, now: SimTime) -> Option<DsrcChannel> {
        match self.current_phase(now) {
            ChannelPhase::Control => Some(DsrcChannel::Cch),
            ChannelPhase::Service => Some(self.service_channel),
            ChannelPhase::Guard => None,
        }
    }

    /// Check if transmission of given packet type is permitted at `now`.
    pub fn can_transmit(&self, now: SimTime, is_control_packet: bool) -> bool {
        match self.current_phase(now) {
            ChannelPhase::Guard => false,
            ChannelPhase::Control => is_control_packet,
            ChannelPhase::Service => !is_control_packet,
        }
    }

    /// Compute time remaining until the next active phase for the designated channel type.
    pub fn time_to_next_phase(&self, now: SimTime, target_control: bool) -> SimTime {
        if self.mode == MultiChannelMode::ContinuousCch {
            return SimTime::ZERO;
        }

        let interval_ticks = self.switching_interval.ticks();
        let guard_ticks = self.guard_interval.ticks();
        let now_ticks = now.ticks();

        let cycle_index = now_ticks / interval_ticks;
        let time_in_cycle = now_ticks % interval_ticks;
        let is_control_cycle = cycle_index % 2 == 0;

        if is_control_cycle == target_control {
            if time_in_cycle < guard_ticks {
                // Currently in guard of the target cycle, ready when guard ends
                SimTime::from_ticks(guard_ticks - time_in_cycle)
            } else {
                // Currently active in target cycle!
                SimTime::ZERO
            }
        } else {
            // In the opposite cycle, wait until this cycle ends + next guard ends
            let remaining_this_cycle = interval_ticks - time_in_cycle;
            SimTime::from_ticks(remaining_this_cycle + guard_ticks)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_continuous_cch() {
        let coord = ChannelSwitchCoordinator::default();
        let now = SimTime::from_millis(1234);
        assert_eq!(coord.current_phase(now), ChannelPhase::Control);
        assert_eq!(coord.active_channel(now), Some(DsrcChannel::Cch));
        assert!(coord.can_transmit(now, true));
    }

    #[test]
    fn test_alternating_switching() {
        let coord = ChannelSwitchCoordinator::new(MultiChannelMode::AlternatingSwitching, DsrcChannel::Sch1);

        // t = 0 ms -> Guard interval of cycle 0 (CCH)
        assert_eq!(coord.current_phase(SimTime::from_millis(0)), ChannelPhase::Guard);
        assert!(!coord.can_transmit(SimTime::from_millis(2), true));

        // t = 5 ms -> Active CCH (cycle 0, 4ms..50ms)
        assert_eq!(coord.current_phase(SimTime::from_millis(5)), ChannelPhase::Control);
        assert!(coord.can_transmit(SimTime::from_millis(5), true));
        assert!(!coord.can_transmit(SimTime::from_millis(5), false));

        // t = 52 ms -> Guard interval of cycle 1 (SCH) (50ms..54ms)
        assert_eq!(coord.current_phase(SimTime::from_millis(52)), ChannelPhase::Guard);

        // t = 60 ms -> Active SCH (cycle 1, 54ms..100ms)
        assert_eq!(coord.current_phase(SimTime::from_millis(60)), ChannelPhase::Service);
        assert!(coord.can_transmit(SimTime::from_millis(60), false));
        assert!(!coord.can_transmit(SimTime::from_millis(60), true));
    }
}
