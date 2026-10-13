//! Two-Stage Sidelink Control Information (2-Stage SCI).
//!
//! Standards Reference: 3GPP TS 38.212 Section 8.3 & Section 8.4
//! Parity Reference: 5G-LENA (`nr-sl-sci-f1a-header.h`, `nr-sl-sci-f2a-header.h`) & UWICORE (`nrSCI1Encode.m`, `nrSCI2Encode.m`)

use serde::{Deserialize, Serialize};

/// Sidelink Communication Cast Type per TS 38.212 Section 8.4.1.1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NrCastType {
    /// Broadcast communication (e.g. ETSI CAM, standard beaconing)
    Broadcast = 0,
    /// Groupcast with positive ACK and negative NACK (Option 1)
    Groupcast = 1,
    /// Unicast communication (e.g. Platooning coordination, sensor stream)
    Unicast = 2,
    /// Groupcast with NACK-only feedback on shared PSFCH (Option 2)
    GroupcastNackOnly = 3,
}

impl Default for NrCastType {
    fn default() -> Self {
        Self::Broadcast
    }
}

/// Second-stage SCI Format Indicator per TS 38.212 Table 8.3.1.1-1
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SciStage2Format {
    /// SCI Format 2-A: General unicast, groupcast, and broadcast transmission
    Format2A = 0,
    /// SCI Format 2-B: Sidelink channel state information (CSI) triggering
    Format2B = 1,
}

impl Default for SciStage2Format {
    fn default() -> Self {
        Self::Format2A
    }
}

/// SCI Format 1-A (Stage 1 SCI) carried over PSCCH.
///
/// Encoded using Polar codes with CRC-24C (TS 38.212 Section 8.3).
/// Demodulated by all listening UEs within radio range to perform autonomous channel sensing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SciFormat1A {
    /// Priority of transmission (0 to 7, where 0 is highest emergency priority)
    pub priority: u8,
    /// Resource reservation period P_rsvp in milliseconds (0 indicates no periodic reservation)
    pub reservation_period_ms: u16,
    /// Starting subchannel index
    pub start_subchannel: u8,
    /// Number of contiguous subchannels allocated
    pub num_subchannels: u8,
    /// Total number of subchannels in the resource pool
    pub total_subchannels: u8,
    /// Modulation and Coding Scheme (0 to 28)
    pub mcs: u8,
    /// Second-stage SCI format indicator
    pub stage2_format: SciStage2Format,
    /// Frequency resource assignment RIV (Resource Indication Value)
    pub freq_resource_riv: u16,
    /// Time resource assignment field
    pub time_resource_assignment: u16,
    /// Maximum number of reservations indicated (e.g. 2 or 3)
    pub max_num_per_reserve: u8,
    /// Gap in slots to first retransmission (if reserved)
    pub gap_retx1_slots: u8,
    /// Starting subchannel of first retransmission
    pub start_subchannel_retx1: u8,
    /// PSFCH overhead indicator (true if PSFCH slot is active)
    pub psfch_overhead: bool,
}

impl SciFormat1A {
    pub fn new(
        priority: u8,
        reservation_period_ms: u16,
        start_subchannel: u8,
        num_subchannels: u8,
        total_subchannels: u8,
        mcs: u8,
    ) -> Self {
        // Calculate standard RIV: TS 38.214 Section 8.1.2.2
        let n_sub = total_subchannels as u16;
        let l_sub = num_subchannels as u16;
        let s_sub = start_subchannel as u16;
        let riv = if (l_sub - 1) <= (n_sub / 2) {
            n_sub * (l_sub - 1) + s_sub
        } else {
            n_sub * (n_sub - l_sub + 1) + (n_sub - 1 - s_sub)
        };

        Self {
            priority,
            reservation_period_ms,
            start_subchannel,
            num_subchannels,
            total_subchannels,
            mcs,
            stage2_format: SciStage2Format::Format2A,
            freq_resource_riv: riv,
            time_resource_assignment: 0,
            max_num_per_reserve: 2,
            gap_retx1_slots: 0,
            start_subchannel_retx1: start_subchannel,
            psfch_overhead: false,
        }
    }

    /// True if an active future periodic reservation is announced
    pub fn has_periodic_reservation(&self) -> bool {
        self.reservation_period_ms > 0
    }
}

/// SCI Format 2-A (Stage 2 SCI) carried over PSSCH resource elements.
///
/// Encoded using Polar codes with CRC-24C (TS 38.212 Section 8.4).
/// Provides demultiplexing, HARQ process association, and Layer-2 addressing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct SciFormat2A {
    /// HARQ Process Identifier (0 to 15)
    pub harq_process_id: u8,
    /// New Data Indicator (toggled when a new TB is transmitted)
    pub ndi: bool,
    /// Redundancy Version (0, 2, 3, or 1)
    pub rv: u8,
    /// 16-bit Layer-2 Source Identifier
    pub source_id: u32,
    /// 16-bit Layer-2 Destination Identifier
    pub destination_id: u32,
    /// HARQ Feedback enabled indicator
    pub harq_feedback_enabled: bool,
    /// Communication Cast Type
    pub cast_type: NrCastType,
    /// Sidelink Channel State Information (CSI) request flag
    pub csi_req: bool,
}

impl SciFormat2A {
    pub fn new_broadcast(source_id: u32) -> Self {
        Self {
            harq_process_id: 0,
            ndi: true,
            rv: 0,
            source_id,
            destination_id: 0xFFFF, // Broadcast destination ID
            harq_feedback_enabled: false,
            cast_type: NrCastType::Broadcast,
            csi_req: false,
        }
    }

    pub fn new_unicast(
        source_id: u32,
        destination_id: u32,
        harq_process_id: u8,
        ndi: bool,
        rv: u8,
    ) -> Self {
        Self {
            harq_process_id,
            ndi,
            rv,
            source_id,
            destination_id,
            harq_feedback_enabled: true,
            cast_type: NrCastType::Unicast,
            csi_req: false,
        }
    }
}
