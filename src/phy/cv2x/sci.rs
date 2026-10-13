//! Sidelink Control Information (SCI Format 1).
//!
//! Standardized per 3GPP TS 36.212 Section 5.4.3.1.2 and TS 36.213 Section 14:
//! Transmitted over the PSCCH (Physical Sidelink Control Channel, 2 RBs) to inform
//! all nearby receivers of the upcoming transmission location, priority, and periodic reservations.

use serde::{Deserialize, Serialize};
use crate::scheduler::event::ActorId;

/// 3GPP Rel-14 Sidelink Control Information (SCI Format 1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SidelinkControlInfo {
    /// Transmitting station identifier
    pub sender_id: ActorId,
    /// ProSe Per-Packet Priority (PPPP: 0 = highest e.g. emergency braking, 7 = lowest)
    pub priority: u8,
    /// Resource Reservation Interval in milliseconds (P_rsvp, e.g. 100 ms for 10 Hz CAM)
    pub reservation_interval_ms: u64,
    /// Allocated sub-channel index for the accompanying PSSCH transport block
    pub subchannel_index: usize,
    /// Modulation and Coding Scheme (MCS) for the PSSCH transport block
    pub mcs: u8,
    /// Current resource reselection counter (C_resel)
    pub reselection_counter: u32,
    /// Absolute subframe timestamp when this SCI was broadcast
    pub tx_subframe: u64,
}

impl SidelinkControlInfo {
    pub fn new(
        sender_id: ActorId,
        priority: u8,
        reservation_interval_ms: u64,
        subchannel_index: usize,
        mcs: u8,
        reselection_counter: u32,
        tx_subframe: u64,
    ) -> Self {
        Self {
            sender_id,
            priority,
            reservation_interval_ms,
            subchannel_index,
            mcs,
            reselection_counter,
            tx_subframe,
        }
    }

    /// Check if this SCI announces a periodic reservation.
    #[inline]
    pub fn has_reservation(&self) -> bool {
        self.reservation_interval_ms > 0 && self.reselection_counter > 0
    }

    /// Predict the next reserved subframe for this vehicle.
    #[inline]
    pub fn next_reserved_subframe(&self) -> Option<u64> {
        if self.has_reservation() {
            Some(self.tx_subframe + self.reservation_interval_ms)
        } else {
            None
        }
    }
}
