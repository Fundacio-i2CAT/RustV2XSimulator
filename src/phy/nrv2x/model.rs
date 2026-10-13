//! 3GPP 5G NR-V2X Sidelink Mode 2 Model implementing `WirelessTechnology`.
//!
//! Standards Reference: 3GPP Release 16/17 (TS 38.211, TS 38.212, TS 38.213, TS 38.214, TS 38.321)

use rand::rngs::StdRng;

use crate::common::SimTime;
use crate::geometry::polygon::BuildingMap;
use crate::geometry::segment::Point2D;
use crate::mac::edca::TxFrame;
use crate::phy::air_packet::AirPacket;
use crate::phy::radio_medium::{PhyModelType, RadioMedium, ReceptionResult};
use crate::phy::technology::WirelessTechnology;
use crate::scheduler::event::ActorId;

use super::config::NrV2xConfig;
use super::ldpc_bler::NrPhyBlerModel;
use super::mode2_scheduler::NrMode2Scheduler;
use super::psfch_harq::NrHarqEntity;
use super::resource_grid::NrResourceGrid;
use super::sensing::NrSensingManager;

/// 3GPP 5G NR-V2X Sidelink Mode 2 wireless technology model.
pub struct NrV2xMode2Model {
    pub config: NrV2xConfig,
    pub grid: NrResourceGrid,
    pub scheduler: NrMode2Scheduler,
    pub sensing: NrSensingManager,
    pub harq: NrHarqEntity,
    pub medium: RadioMedium,
    pub sim_seed: u64,
}

impl NrV2xMode2Model {
    pub fn new(config: NrV2xConfig, sim_seed: u64) -> Self {
        let grid = NrResourceGrid::new(
            config.numerology,
            config.num_subchannels,
            config.subchannel_size_rbs,
        );
        let scheduler = NrMode2Scheduler::new(config.clone());
        let sensing = NrSensingManager::new(config.clone());
        let harq = NrHarqEntity::new(8, config.max_harq_retx);
        let medium = RadioMedium::new(PhyModelType::DualSlope, config.tx_power_dbm);

        Self {
            config,
            grid,
            scheduler,
            sensing,
            harq,
            medium,
            sim_seed,
        }
    }
}

impl WirelessTechnology for NrV2xMode2Model {
    fn name(&self) -> &'static str {
        "3GPP_5G_NR_V2X_Sidelink_Mode2"
    }

    fn enqueue_outbound(
        &mut self,
        _frame: TxFrame,
        access_category: u8,
        now: SimTime,
        rng: &mut StdRng,
    ) -> Option<SimTime> {
        // Map EDCA access category to 3GPP NR Sidelink Priority (0 to 7)
        let priority = match access_category {
            0 => 1, // VoiceSafety / Emergency Braking (DENM)
            1 => 3, // Video / Advisory Safety
            3 => 6, // Background
            _ => 4, // BestEffort / CAM default
        };

        let cur_slot = self.grid.sim_time_to_slot(now);
        // Prune old sensing observations
        self.sensing.prune_old_sensing_data(cur_slot);

        // Run 3GPP TS 38.213 Section 16.6 Mode 2 Scheduler
        let chosen_res = self.scheduler.schedule_packet(cur_slot, priority, 1, &self.sensing, rng);
        self.sensing.record_local_tx(chosen_res.slot);

        // Schedule at slot boundary
        Some(self.grid.slot_to_sim_time(chosen_res.slot))
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

        // Apply 3GPP NR-V2X physical dual decoding (PSCCH Polar + PSSCH LDPC)
        standard_results
            .into_iter()
            .map(|mut res| {
                if res.is_success {
                    let nr_res = NrPhyBlerModel::evaluate_reception(
                        res.sinr_db,
                        packet.payload_bytes.len(),
                        self.config.pssch_mcs,
                        rng,
                    );
                    res.is_success = nr_res.is_success;
                }
                res
            })
            .collect()
    }

    fn record_channel_activity(&mut self, start: SimTime, end: SimTime) {
        let start_slot = self.grid.sim_time_to_slot(start);
        let end_slot = self.grid.sim_time_to_slot(end);
        for slot in start_slot..=end_slot {
            for sc in 0..self.config.num_subchannels {
                self.sensing.record_subchannel_rssi(slot, sc as u8, -85.0);
            }
        }
    }

    fn evaluate_cbr(&mut self, now: SimTime) -> f64 {
        let cur_slot = self.grid.sim_time_to_slot(now);
        self.sensing.evaluate_cbr(cur_slot)
    }
}
