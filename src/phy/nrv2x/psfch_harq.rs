//! Physical Sidelink Feedback Channel (PSFCH) & Hybrid ARQ (HARQ) Entity.
//!
//! Standards Reference: 3GPP TS 38.213 Section 16.2 & TS 38.321 Section 5.14
//! Parity Reference: UWICORE (`HARQEntity.m`) & 5G-LENA (`nr-sl-ue-mac-harq.cc`)

use serde::{Deserialize, Serialize};

/// Sidelink HARQ Feedback status per TS 38.213 Section 16.2
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum NrHarqFeedback {
    /// Positive Acknowledgement (PSSCH decoded error-free)
    Ack,
    /// Negative Acknowledgement (PSSCH corrupted)
    Nack,
    /// Discontinuous Transmission (SCI not detected or not received)
    Dtx,
}

/// Standard Redundancy Version (RV) sequence per TS 38.214 Section 5.1.2.1
pub const NR_HARQ_RV_SEQUENCE: [u8; 4] = [0, 2, 3, 1];

/// Single Stop-and-Wait Sidelink HARQ Process.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NrHarqProcess {
    pub harq_id: u8,
    pub is_active: bool,
    pub ndi: bool,
    pub current_rv_index: usize,
    pub max_retransmissions: u8,
    pub transmission_count: u8,
    pub tbs_bytes: usize,
}

impl NrHarqProcess {
    pub fn new(harq_id: u8, max_retransmissions: u8) -> Self {
        Self {
            harq_id,
            is_active: false,
            ndi: false,
            current_rv_index: 0,
            max_retransmissions,
            transmission_count: 0,
            tbs_bytes: 0,
        }
    }

    /// Redundancy Version for current transmission attempt
    pub fn current_rv(&self) -> u8 {
        NR_HARQ_RV_SEQUENCE[self.current_rv_index % NR_HARQ_RV_SEQUENCE.len()]
    }

    /// Start a new Transport Block transmission
    pub fn start_new_transmission(&mut self, tbs_bytes: usize) {
        self.is_active = true;
        self.ndi = !self.ndi; // Toggle New Data Indicator
        self.current_rv_index = 0;
        self.transmission_count = 1;
        self.tbs_bytes = tbs_bytes;
    }

    /// Update state upon receiving HARQ feedback (or timeout).
    /// Returns true if a retransmission is needed, false if process completed.
    pub fn update_feedback(&mut self, feedback: NrHarqFeedback) -> bool {
        match feedback {
            NrHarqFeedback::Ack => {
                // Successful reception! Reset process
                self.is_active = false;
                self.current_rv_index = 0;
                false
            }
            NrHarqFeedback::Nack | NrHarqFeedback::Dtx => {
                if self.transmission_count <= self.max_retransmissions {
                    // Advance to next Redundancy Version in RV sequence
                    self.current_rv_index = (self.current_rv_index + 1) % NR_HARQ_RV_SEQUENCE.len();
                    self.transmission_count += 1;
                    true // Retransmission required
                } else {
                    // Exceeded max retransmissions: discard
                    self.is_active = false;
                    false
                }
            }
        }
    }
}

/// Sidelink HARQ Entity managing parallel stop-and-wait processes for a UE.
/// Direct parity with UWICORE `HARQEntity.m`.
#[derive(Debug, Clone)]
pub struct NrHarqEntity {
    pub num_processes: usize,
    pub processes: Vec<NrHarqProcess>,
    pub next_alloc_id: u8,
}

impl NrHarqEntity {
    pub fn new(num_processes: usize, max_retransmissions: u8) -> Self {
        let processes = (0..num_processes)
            .map(|id| NrHarqProcess::new(id as u8, max_retransmissions))
            .collect();
        Self {
            num_processes,
            processes,
            next_alloc_id: 0,
        }
    }

    /// Obtain an idle HARQ process ID for a new transmission
    pub fn allocate_process(&mut self) -> Option<u8> {
        for _ in 0..self.num_processes {
            let id = self.next_alloc_id as usize;
            self.next_alloc_id = ((self.next_alloc_id + 1) as usize % self.num_processes) as u8;
            if !self.processes[id].is_active {
                return Some(id as u8);
            }
        }
        None
    }

    /// Access HARQ process by ID
    pub fn get_process(&self, harq_id: u8) -> Option<&NrHarqProcess> {
        self.processes.get(harq_id as usize)
    }

    /// Mutably access HARQ process by ID
    pub fn get_process_mut(&mut self, harq_id: u8) -> Option<&mut NrHarqProcess> {
        self.processes.get_mut(harq_id as usize)
    }
}

/// PSFCH Sidelink Feedback Timing Calculations per 3GPP TS 38.213 Section 16.2.4.
#[derive(Debug, Clone, Copy)]
pub struct PsfchTimingCalculator {
    pub period_slots: usize,
    pub min_time_gap_slots: u64,
}

impl PsfchTimingCalculator {
    pub fn new(period_slots: usize) -> Self {
        Self {
            period_slots,
            min_time_gap_slots: 2, // 3GPP min processing gap between PSSCH and PSFCH
        }
    }

    /// True if `slot` contains PSFCH feedback resources
    pub fn is_psfch_slot(&self, slot: u64) -> bool {
        if self.period_slots == 0 {
            false
        } else {
            (slot as usize) % self.period_slots == 0
        }
    }

    /// Calculate designated PSFCH slot for a PSSCH transmission at `tx_slot`
    pub fn feedback_slot_for_tx(&self, tx_slot: u64) -> Option<u64> {
        if self.period_slots == 0 {
            return None;
        }

        let earliest_candidate = tx_slot + self.min_time_gap_slots;
        let rem = (earliest_candidate as usize) % self.period_slots;
        let delta = if rem == 0 {
            0
        } else {
            self.period_slots - rem
        };

        Some(earliest_candidate + delta as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_harq_rv_progression() {
        let mut proc = NrHarqProcess::new(0, 3);
        proc.start_new_transmission(300);
        assert_eq!(proc.current_rv(), 0);
        assert_eq!(proc.transmission_count, 1);

        // First NACK: moves to RV = 2
        assert!(proc.update_feedback(NrHarqFeedback::Nack));
        assert_eq!(proc.current_rv(), 2);
        assert_eq!(proc.transmission_count, 2);

        // Second NACK: moves to RV = 3
        assert!(proc.update_feedback(NrHarqFeedback::Nack));
        assert_eq!(proc.current_rv(), 3);

        // Third NACK: moves to RV = 1
        assert!(proc.update_feedback(NrHarqFeedback::Nack));
        assert_eq!(proc.current_rv(), 1);

        // Fourth NACK: exceeds max_retx (3), drops TB
        assert!(!proc.update_feedback(NrHarqFeedback::Nack));
        assert!(!proc.is_active);
    }

    #[test]
    fn test_psfch_timing() {
        let calc = PsfchTimingCalculator::new(4);
        assert!(calc.is_psfch_slot(0));
        assert!(calc.is_psfch_slot(4));
        assert!(calc.is_psfch_slot(8));
        assert!(!calc.is_psfch_slot(1));

        // tx at slot 1: earliest = 1 + 2 = 3. Next PSFCH slot multiple of 4 is 4!
        assert_eq!(calc.feedback_slot_for_tx(1), Some(4));
        // tx at slot 2: earliest = 2 + 2 = 4. Exactly 4!
        assert_eq!(calc.feedback_slot_for_tx(2), Some(4));
        // tx at slot 3: earliest = 3 + 2 = 5. Next multiple is 8!
        assert_eq!(calc.feedback_slot_for_tx(3), Some(8));
    }
}
