//! Event definitions and payloads for Discrete-Event Simulation.

use crate::common::SimTime;
use std::cmp::Ordering;

/// Monotonic identifier for actors/nodes in the simulation.
pub type ActorId = u32;

/// Typed payload representing discrete-event simulation actions.
#[derive(Debug, Clone)]
pub enum EventPayload {
    /// Periodic microscopic mobility update for all nodes or a specific node.
    MobilityStep { step_index: u64 },
    /// Outbound wireless transmission initiation at the MAC layer.
    MacTxStart {
        packet_id: u64,
        sender_id: ActorId,
        access_category: u8,
        payload_bytes: Vec<u8>,
    },
    /// Inbound wireless transmission completion/delivery at PHY layer.
    PhyRxDelivery {
        packet_id: u64,
        sender_id: ActorId,
        receiver_id: ActorId,
        payload_bytes: Vec<u8>,
        rx_power_dbm: f64,
        sinr_db: f64,
        is_success: bool,
    },
    /// Channel monitor periodic CBR window update.
    CbrUpdate { node_id: ActorId },
    /// Decentralized Congestion Control (DCC) periodic rate evaluation.
    DccEvaluate { node_id: ActorId },
    /// CAM generation timer event for a specific node.
    CamTrigger { node_id: ActorId },
    /// Krauß-safe sub-step emergency brake actuation upon EEBL DENM reception.
    KraussBrakeActuation {
        node_id: ActorId,
        leader_id: ActorId,
        eebl_delivery_time: SimTime,
        decel_req: f64,
    },
    /// General timer / callback.
    Timer { token: u64 },
}

/// A discrete simulation event ordered by timestamp, priority, and insertion sequence.
#[derive(Debug, Clone)]
pub struct Event {
    pub id: u64,
    pub timestamp: SimTime,
    pub priority: u8,
    pub sequence: u64,
    pub actor_id: ActorId,
    pub payload: EventPayload,
}

impl Event {
    pub fn new(
        id: u64,
        timestamp: SimTime,
        priority: u8,
        sequence: u64,
        actor_id: ActorId,
        payload: EventPayload,
    ) -> Self {
        Self {
            id,
            timestamp,
            priority,
            sequence,
            actor_id,
            payload,
        }
    }
}

// Strict ordering for Min-Heap Calendar:
// 1. Earliest SimTime first.
// 2. Lowest priority integer (0 = highest priority) first.
// 3. Lowest sequence counter (FIFO tie-breaking) first.
impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp == other.timestamp
            && self.priority == other.priority
            && self.sequence == other.sequence
    }
}

impl Eq for Event {}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse for Min-Heap: we want smallest (earliest) timestamp to be "greater" in BinaryHeap
        // so BinaryHeap::pop() returns the earliest event.
        other
            .timestamp
            .cmp(&self.timestamp)
            .then_with(|| other.priority.cmp(&self.priority))
            .then_with(|| other.sequence.cmp(&self.sequence))
    }
}
