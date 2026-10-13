//! IEEE 802.11bd Next Generation V2X Model implementing `WirelessTechnology`.
//!
//! Standards Reference: IEEE Std 802.11bd-2022

use rand::rngs::StdRng;

use crate::common::SimTime;
use crate::geometry::polygon::BuildingMap;
use crate::geometry::segment::Point2D;
use crate::mac::channel_monitor::ChannelMonitor;
use crate::mac::edca::{AccessCategory, EdcaChannelAccess, TxFrame};
use crate::phy::air_packet::AirPacket;
use crate::phy::radio_medium::{PhyModelType, RadioMedium, ReceptionResult};
use crate::phy::technology::WirelessTechnology;
use crate::phy::wifi80211bd::config::Wifi80211bdConfig;
use crate::phy::wifi80211bd::decider_80211bd::Decider80211bd;
use crate::scheduler::event::ActorId;

/// IEEE 802.11bd Next Generation V2X wireless technology model.
pub struct Wifi80211bdModel {
    pub config: Wifi80211bdConfig,
    pub mac: EdcaChannelAccess,
    pub channel_monitor: ChannelMonitor,
    pub decider: Decider80211bd,
    pub medium: RadioMedium,
    pub sim_seed: u64,
}

impl Wifi80211bdModel {
    pub fn new(config: Wifi80211bdConfig, sim_seed: u64) -> Self {
        let mac = EdcaChannelAccess::new(config.is_aloha, config.cca_threshold_dbm);
        let channel_monitor = ChannelMonitor::default();
        let decider = Decider80211bd::new(config.bandwidth_mhz, config.carrier_freq_hz);
        let medium = RadioMedium::new(PhyModelType::DualSlope, config.tx_power_dbm);

        Self {
            config,
            mac,
            channel_monitor,
            decider,
            medium,
            sim_seed,
        }
    }
}

impl WirelessTechnology for Wifi80211bdModel {
    fn name(&self) -> &'static str {
        "IEEE_802_11bd_NGV"
    }

    fn enqueue_outbound(
        &mut self,
        frame: TxFrame,
        access_category: u8,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Option<SimTime> {
        let ac = match access_category {
            0 => AccessCategory::VoiceSafety,
            1 => AccessCategory::Video,
            3 => AccessCategory::Background,
            _ => AccessCategory::BestEffort,
        };
        self.mac.enqueue_packet(ac, frame, rng);
        if let Some((winning_ac, _)) = self.mac.poll_ready_transmission(rng) {
            let backoff = if self.mac.is_aloha_mode {
                SimTime::ZERO
            } else {
                self.mac.compute_backoff_delay(winning_ac, rng)
            };
            Some(now + backoff)
        } else {
            None
        }
    }

    fn evaluate_transmission(
        &mut self,
        packet: &AirPacket,
        sender_pos: Point2D,
        receivers: &[(ActorId, Point2D, f64)],
        buildings: &BuildingMap,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Vec<ReceptionResult> {
        self.medium.purge_completed_signals(now);
        self.medium.register_transmission_with_pos(packet, sender_pos);

        let standard_results = self.medium.evaluate_broadcast(
            packet,
            sender_pos,
            receivers,
            buildings,
            self.sim_seed,
        );

        // Apply 802.11bd physical decoding: LDPC, Doppler aging, and Midamble tracking
        standard_results
            .into_iter()
            .map(|mut res| {
                if res.is_success {
                    // Estimate approximate relative speed from link geometry / highway context (or default nominal 30 m/s ~ 108 km/h)
                    let relative_speed_mps = 30.0;
                    let dec_res = self.decider.evaluate_packet(
                        res.rx_power_dbm,
                        res.sinr_db,
                        packet.payload_bytes.len(),
                        self.config.mcs,
                        self.config.midamble_period,
                        self.config.use_dcm,
                        relative_speed_mps,
                        rng,
                    );
                    res.is_success = dec_res.is_success;
                }
                res
            })
            .collect()
    }

    fn record_channel_activity(&mut self, start: SimTime, end: SimTime) {
        self.channel_monitor.record_busy_interval(start, end);
    }

    fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        self.channel_monitor.evaluate_cbr(now)
    }
}
