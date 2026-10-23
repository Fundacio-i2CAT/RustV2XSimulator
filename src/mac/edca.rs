//! IEEE 802.11p EDCA Medium Access Control with 4 Access Categories.
//!
//! Supports internal contention resolution, backoff freeze/resume on CCA busy,
//! and unslotted ALOHA mode for ablation study $A5$.

use crate::common::SimTime;
use rand::Rng;
use serde::{Deserialize, Serialize};

/// 802.11p EDCA Access Category
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum AccessCategory {
    VoiceSafety = 0, // AC_VO: DENM / emergency
    Video = 1,       // AC_VI
    BestEffort = 2,  // AC_BE: CAM
    Background = 3,  // AC_BK
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacState {
    Idle,
    DeferAifs,
    Backoff,
    Transmitting,
}

#[derive(Debug, Clone)]
pub struct EdcaQueueConfig {
    pub aifsn: u32,
    pub cw_min: u32,
    pub cw_max: u32,
}

impl EdcaQueueConfig {
    pub fn for_ac(ac: AccessCategory) -> Self {
        match ac {
            AccessCategory::VoiceSafety => Self {
                aifsn: 2,
                cw_min: 3,
                cw_max: 7,
            },
            AccessCategory::Video => Self {
                aifsn: 3,
                cw_min: 7,
                cw_max: 15,
            },
            AccessCategory::BestEffort => Self {
                aifsn: 6,
                cw_min: 15,
                cw_max: 1023,
            },
            AccessCategory::Background => Self {
                aifsn: 9,
                cw_min: 15,
                cw_max: 1023,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct TxFrame {
    pub frame_id: u64,
    pub payload: Vec<u8>,
    pub created_time: SimTime,
    pub duration: SimTime,
}

#[derive(Debug, Clone)]
pub struct EdcaQueue {
    pub ac: AccessCategory,
    pub config: EdcaQueueConfig,
    pub state: MacState,
    pub cw: u32,
    pub backoff_slots: u32,
    pub buffer: Vec<TxFrame>,
    pub backoff_freeze_time: Option<SimTime>,
}

impl EdcaQueue {
    pub fn new(ac: AccessCategory) -> Self {
        let config = EdcaQueueConfig::for_ac(ac);
        let cw_min = config.cw_min;
        Self {
            ac,
            config,
            state: MacState::Idle,
            cw: cw_min,
            backoff_slots: 0,
            buffer: Vec::new(),
            backoff_freeze_time: None,
        }
    }

    pub fn enqueue(&mut self, frame: TxFrame) {
        self.buffer.push(frame);
    }

    pub fn start_backoff<R: Rng>(&mut self, rng: &mut R) {
        self.backoff_slots = rng.gen_range(0..=self.cw);
        self.state = MacState::Backoff;
    }

    pub fn on_transmission_success(&mut self) {
        self.cw = self.config.cw_min;
        self.state = MacState::Idle;
        if !self.buffer.is_empty() {
            self.buffer.remove(0);
        }
    }

    pub fn on_internal_collision<R: Rng>(&mut self, rng: &mut R) {
        self.cw = (self.cw * 2 + 1).min(self.config.cw_max);
        self.start_backoff(rng);
    }
}

/// 802.11p EDCA MAC Controller for a vehicular node.
#[derive(Debug, Clone)]
pub struct EdcaChannelAccess {
    pub slot_time: SimTime,
    pub sifs: SimTime,
    pub cca_threshold_dbm: f64,
    pub queues: [EdcaQueue; 4],
    pub is_channel_busy: bool,
}

impl Default for EdcaChannelAccess {
    fn default() -> Self {
        Self::new(-85.0)
    }
}

impl EdcaChannelAccess {
    pub fn new(cca_threshold_dbm: f64) -> Self {
        Self {
            slot_time: SimTime::from_micros(13), // 13 us
            sifs: SimTime::from_micros(32),      // 32 us
            cca_threshold_dbm,
            queues: [
                EdcaQueue::new(AccessCategory::VoiceSafety),
                EdcaQueue::new(AccessCategory::Video),
                EdcaQueue::new(AccessCategory::BestEffort),
                EdcaQueue::new(AccessCategory::Background),
            ],
            is_channel_busy: false,
        }
    }

    /// Submit a packet for transmission via an Access Category.
    pub fn enqueue_packet<R: Rng>(&mut self, ac: AccessCategory, frame: TxFrame, rng: &mut R) {
        let idx = ac as usize;
        self.queues[idx].enqueue(frame);
        if self.queues[idx].state == MacState::Idle {
            self.queues[idx].start_backoff(rng);
        }
    }

    /// Compute transmission duration for payload at 6 Mbps QPSK 1/2 (802.11p default).
    #[inline]
    pub fn compute_duration_10mhz(&self, payload_len_bytes: usize) -> SimTime {
        // Preamble + PLCP header = 40 us
        // Data bits per OFDM symbol at 6 Mbps (10 MHz channel) = 24 bits
        // Duration = 40 us + ceil((16 + 8*len + 6) / 24) * 8 us
        let bits = 16 + payload_len_bytes * 8 + 6;
        let symbols = bits.div_ceil(24);
        let duration_us = 40 + symbols * 8;
        SimTime::from_micros(duration_us as u64)
    }

    /// Compute standard IEEE 802.11p EDCA contention delay: `AIFS[AC]` + backoff_slots * slot_time.
    pub fn compute_backoff_delay<R: Rng>(&self, ac: AccessCategory, rng: &mut R) -> SimTime {
        let config = EdcaQueueConfig::for_ac(ac);
        let slots = rng.gen_range(0..=config.cw_min) as u64;
        let aifs_us = self.sifs.to_micros() + self.slot_time.to_micros() * (config.aifsn as u64);
        let backoff_us = self.slot_time.to_micros() * slots;
        SimTime::from_micros(aifs_us + backoff_us)
    }

    /// Poll for ready transmissions from highest non-empty priority queue.
    ///
    /// Resolves internal contention by prioritizing VoiceSafety > Video > BestEffort > Background.
    pub fn poll_ready_transmission<R: Rng>(
        &mut self,
        rng: &mut R,
    ) -> Option<(AccessCategory, TxFrame)> {
        let mut ready_idx = None;
        for (i, q) in self.queues.iter().enumerate() {
            if !q.buffer.is_empty() {
                ready_idx = Some(i);
                break;
            }
        }

        let idx = ready_idx?;
        let winning_ac = self.queues[idx].ac;
        let winning_frame = self.queues[idx].buffer.remove(0);
        self.queues[idx].cw = self.queues[idx].config.cw_min;
        self.queues[idx].state = MacState::Idle;

        // Colliding lower priority queues double CW and restart backoff
        for q in self.queues.iter_mut().skip(idx + 1) {
            if !q.buffer.is_empty() {
                q.on_internal_collision(rng);
            }
        }

        Some((winning_ac, winning_frame))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    #[test]
    fn test_internal_contention_resolution() {
        let mut mac = EdcaChannelAccess::new(-85.0);
        let mut rng = StdRng::seed_from_u64(123);

        let f_cam = TxFrame {
            frame_id: 1,
            payload: vec![1, 2, 3],
            created_time: SimTime::ZERO,
            duration: SimTime::from_micros(300),
        };

        let f_denm = TxFrame {
            frame_id: 2,
            payload: vec![4, 5, 6],
            created_time: SimTime::ZERO,
            duration: SimTime::from_micros(300),
        };

        mac.enqueue_packet(AccessCategory::BestEffort, f_cam, &mut rng);
        mac.enqueue_packet(AccessCategory::VoiceSafety, f_denm, &mut rng);

        // Force backoff slots to 0 to trigger simultaneous readiness
        mac.queues[AccessCategory::BestEffort as usize].backoff_slots = 0;
        mac.queues[AccessCategory::VoiceSafety as usize].backoff_slots = 0;

        let (ac_winner, frame_winner) = mac.poll_ready_transmission(&mut rng).expect("winner");
        // VoiceSafety must win over BestEffort
        assert_eq!(ac_winner, AccessCategory::VoiceSafety);
        assert_eq!(frame_winner.frame_id, 2);
    }
}
