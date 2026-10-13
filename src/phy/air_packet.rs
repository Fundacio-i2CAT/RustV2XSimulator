//! Wireless AirPacket representation for transmission in the physical medium.

use crate::common::SimTime;
use crate::scheduler::event::ActorId;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AirPacket {
    pub packet_id: u64,
    pub sender_id: ActorId,
    pub tx_power_dbm: f64,
    pub start_time: SimTime,
    pub duration: SimTime,
    pub end_time: SimTime,
    pub payload_bytes: Vec<u8>,
    pub access_category: u8,
}

impl AirPacket {
    pub fn new(
        packet_id: u64,
        sender_id: ActorId,
        tx_power_dbm: f64,
        start_time: SimTime,
        duration: SimTime,
        payload_bytes: Vec<u8>,
        access_category: u8,
    ) -> Self {
        let end_time = start_time + duration;
        Self {
            packet_id,
            sender_id,
            tx_power_dbm,
            start_time,
            duration,
            end_time,
            payload_bytes,
            access_category,
        }
    }

    #[inline]
    pub fn is_active_at(&self, t: SimTime) -> bool {
        t >= self.start_time && t < self.end_time
    }
}
