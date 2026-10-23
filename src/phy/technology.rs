//! Pluggable Wireless Technology Trait Abstraction.
//!
//! Provides a unified, decoupled interface for vehicular wireless communications:
//! - IEEE 802.11p DSRC (`Dsrc80211pModel`)
//! - Parametric Empirical Model (`ParametricEmpiricalModel`, legacy commit 6885a2f3)
//! - Future extensions: C-V2X PC5 Mode 4, 5G NR-V2X Mode 2, IEEE 802.11bd, BLE ad-hoc

use rand::rngs::StdRng;
use serde::{Deserialize, Serialize};

use crate::common::SimTime;
use crate::geometry::polygon::BuildingMap;
use crate::geometry::segment::Point2D;
use crate::mac::channel_monitor::ChannelMonitor;
use crate::mac::edca::{AccessCategory, EdcaChannelAccess, TxFrame};
use crate::phy::air_packet::AirPacket;
use crate::phy::consts_80211p::Mcs80211p;
use crate::phy::cv2x::config::Cv2xConfig;
use crate::phy::cv2x::sb_sps::SensingBasedSpsScheduler;
use crate::phy::cv2x::sensing::SidelinkSensingManager;
use crate::phy::decider_80211p::Decider80211p;
use crate::phy::path_loss::LegacyPolynomialPathLoss;
use crate::phy::radio_medium::{PhyModelType, RadioMedium, ReceptionResult};
pub use crate::phy::nrv2x::model::NrV2xMode2Model;
pub use crate::phy::wifi80211bd::model::Wifi80211bdModel;
use crate::scheduler::event::ActorId;

/// Unified trait defining a pluggable vehicular wireless communication technology.
pub trait WirelessTechnology: Send + Sync {
    /// Human-readable technology identifier
    fn name(&self) -> &'static str;

    /// Enqueue an outbound frame for wireless medium access contention.
    /// Returns the scheduled transmission timestamp if channel access is granted.
    fn enqueue_outbound(
        &mut self,
        frame: TxFrame,
        access_category: u8,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Option<SimTime>;

    /// Evaluate reception across candidate receivers when an AirPacket transmission occurs.
    fn evaluate_transmission(
        &mut self,
        packet: &AirPacket,
        sender_pos: Point2D,
        receivers: &[(ActorId, Point2D, f64)],
        buildings: &BuildingMap,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Vec<ReceptionResult>;

    /// Record channel activity or update sensing window.
    fn record_channel_activity(&mut self, start: SimTime, end: SimTime);

    /// Periodic congestion monitor query (e.g. CBR for 802.11p or CBR_sidelink for C-V2X).
    fn evaluate_cbr(&mut self, now: SimTime) -> f64;
}

/// Standard IEEE 802.11p DSRC wireless technology implementation.
pub struct Dsrc80211pModel {
    pub mac: EdcaChannelAccess,
    pub channel_monitor: ChannelMonitor,
    pub medium: RadioMedium,
    pub sim_seed: u64,
    pub mcs: Mcs80211p,
    pub decider: Decider80211p,
}

impl Dsrc80211pModel {
    pub fn new(tx_power_dbm: f64, sim_seed: u64) -> Self {
        Self::with_mcs(tx_power_dbm, sim_seed, Mcs80211p::default())
    }

    pub fn with_mcs(tx_power_dbm: f64, sim_seed: u64, mcs: Mcs80211p) -> Self {
        Self {
            mac: EdcaChannelAccess::new(-85.0),
            channel_monitor: ChannelMonitor::default(),
            medium: RadioMedium::new(PhyModelType::DualSlope, tx_power_dbm),
            sim_seed,
            mcs,
            decider: Decider80211p::default(),
        }
    }
}

impl WirelessTechnology for Dsrc80211pModel {
    fn name(&self) -> &'static str {
        "IEEE_802_11p_DSRC"
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
            let backoff = self.mac.compute_backoff_delay(winning_ac, rng);
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

        if self.mcs == Mcs80211p::QpskR1_2 {
            standard_results
        } else {
            standard_results
                .into_iter()
                .map(|mut res| {
                    if res.is_success {
                        let dec_res = self.decider.evaluate_packet(
                            res.rx_power_dbm,
                            res.sinr_db,
                            packet.payload_bytes.len(),
                            self.mcs,
                            rng,
                        );
                        res.is_success = dec_res.is_success;
                    }
                    res
                })
                .collect()
        }
    }

    fn record_channel_activity(&mut self, start: SimTime, end: SimTime) {
        self.channel_monitor.record_busy_interval(start, end);
    }

    fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        self.channel_monitor.evaluate_cbr(now)
    }
}

/// Parametric empirical wireless model ported from commit 6885a2f3.
pub struct ParametricEmpiricalModel {
    pub config: LegacyPolynomialPathLoss,
    pub medium: RadioMedium,
    pub sim_seed: u64,
    pub channel_monitor: ChannelMonitor,
}

impl ParametricEmpiricalModel {
    pub fn new(config: LegacyPolynomialPathLoss, sim_seed: u64) -> Self {
        let mut medium = RadioMedium::new(PhyModelType::LegacyPoly, 23.0);
        medium.legacy_poly = config.clone();
        Self {
            config,
            medium,
            sim_seed,
            channel_monitor: ChannelMonitor::default(),
        }
    }
}

impl WirelessTechnology for ParametricEmpiricalModel {
    fn name(&self) -> &'static str {
        "Parametric_Empirical"
    }

    fn enqueue_outbound(
        &mut self,
        _frame: TxFrame,
        _access_category: u8,
        now: SimTime,
        _rng: &mut StdRng,
    ) -> Option<SimTime> {
        // Immediate medium access (unslotted)
        Some(now)
    }

    fn evaluate_transmission(
        &mut self,
        packet: &AirPacket,
        sender_pos: Point2D,
        receivers: &[(ActorId, Point2D, f64)],
        buildings: &BuildingMap,
        now: SimTime,
        _rng: &mut StdRng,
    ) -> Vec<ReceptionResult> {
        self.medium.purge_completed_signals(now);
        self.medium.register_transmission_with_pos(packet, sender_pos);
        self.medium.evaluate_broadcast(
            packet,
            sender_pos,
            receivers,
            buildings,
            self.sim_seed,
        )
    }

    fn record_channel_activity(&mut self, start: SimTime, end: SimTime) {
        self.channel_monitor.record_busy_interval(start, end);
    }

    fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        self.channel_monitor.evaluate_cbr(now)
    }
}

/// 3GPP Release 14 Cellular-V2X (C-V2X) PC5 Sidelink Mode 4 Model.
pub struct Cv2xMode4Model {
    pub config: Cv2xConfig,
    pub scheduler: SensingBasedSpsScheduler,
    pub sensing: SidelinkSensingManager,
    pub medium: RadioMedium,
    pub sim_seed: u64,
}

impl Cv2xMode4Model {
    pub fn new(config: Cv2xConfig, sim_seed: u64) -> Self {
        let scheduler = SensingBasedSpsScheduler::new(config.clone());
        let sensing = SidelinkSensingManager::new(config.clone());
        let medium = RadioMedium::new(PhyModelType::DualSlope, config.tx_power_dbm);
        Self {
            config,
            scheduler,
            sensing,
            medium,
            sim_seed,
        }
    }
}

impl WirelessTechnology for Cv2xMode4Model {
    fn name(&self) -> &'static str {
        "3GPP_C-V2X_PC5_Mode4"
    }

    fn enqueue_outbound(
        &mut self,
        _frame: TxFrame,
        _access_category: u8,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Option<SimTime> {
        let cur_sf = now.to_millis();
        let chosen_csr = self.scheduler.schedule_packet(cur_sf, &self.sensing, rng);
        self.sensing.record_local_tx(chosen_csr.subframe, chosen_csr.subchannel);
        Some(SimTime::from_millis(chosen_csr.subframe))
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
        self.scheduler.on_transmission_completed(rng);
        self.medium.purge_completed_signals(now);
        self.medium.register_transmission_with_pos(packet, sender_pos);

        let standard_results = self.medium.evaluate_broadcast(
            packet,
            sender_pos,
            receivers,
            buildings,
            self.sim_seed,
        );

        // Apply 3GPP Rel-14 Sidelink dual decoding (PSCCH SCI + PSSCH TB)
        standard_results
            .into_iter()
            .map(|mut res| {
                if res.is_success {
                    let dual_res = crate::phy::cv2x::phy_bler::SidelinkPhyModel::evaluate_reception(
                        res.sinr_db,
                        packet.payload_bytes.len(),
                        self.config.pssch_mcs,
                        rng,
                    );
                    res.is_success = dual_res.is_success;
                }
                res
            })
            .collect()
    }

    fn record_channel_activity(&mut self, start: SimTime, end: SimTime) {
        let start_sf = start.to_millis();
        let end_sf = end.to_millis();
        for sf in start_sf..=end_sf {
            self.sensing.record_subchannel_rssi(sf, 0, -85.0);
        }
    }

    fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        self.sensing.evaluate_cbr(now.to_millis())
    }
}

/// Configuration selector for wireless technology models.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WirelessTechnologyType {
    Dsrc80211p,
    ParametricEmpirical,
    Cv2xMode4,
    NrV2xMode2,
    Wifi80211bd,
    BleAdHoc,
}

impl Default for WirelessTechnologyType {
    fn default() -> Self {
        Self::Dsrc80211p
    }
}
