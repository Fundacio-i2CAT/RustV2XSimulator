//! Simulator Link Layer Adapter bridging `rustflexstack` to the DES channel.
//!
//! Synchronously bridges GeoNetworking and BTP router actors to the discrete-event
//! simulation channel via explicit polling (`drain_bridges()` and `drain_outbound_frames()`).

use std::sync::mpsc::{Receiver, TryRecvError};

use rustflexstack::btp::router::BTPRouterHandle;
use crate::cits::geonet::RouterHandle;
use rustflexstack::geonet::service_access_point::{GNDataIndication, GNDataRequest};

use crate::common::SimTime;
use crate::mac::edca::{AccessCategory, TxFrame};
use crate::scheduler::event::ActorId;

pub struct SimulatorLinkLayer {
    pub station_id: ActorId,
    /// Raw GeoNetworking packets ready for wireless transmission (GN router -> MAC/PHY).
    gn_to_ll_rx: Receiver<Vec<u8>>,
    /// Handle to inject incoming packets or updates into the GN router.
    router_handle: RouterHandle,
    /// GN -> BTP: GN router sends GNDataIndication here; drained to btp_handle.
    gn_to_btp_rx: Receiver<GNDataIndication>,
    /// BTP router handle.
    btp_handle: BTPRouterHandle,
    /// BTP -> GN: BTP router sends GNDataRequest here; drained to router_handle.
    btp_to_gn_rx: Receiver<GNDataRequest>,
    next_frame_id: std::cell::Cell<u64>,
}

impl SimulatorLinkLayer {
    pub fn new(
        station_id: ActorId,
        gn_to_ll_rx: Receiver<Vec<u8>>,
        router_handle: RouterHandle,
        gn_to_btp_rx: Receiver<GNDataIndication>,
        btp_handle: BTPRouterHandle,
        btp_to_gn_rx: Receiver<GNDataRequest>,
    ) -> Self {
        Self {
            station_id,
            gn_to_ll_rx,
            router_handle,
            gn_to_btp_rx,
            btp_handle,
            btp_to_gn_rx,
            next_frame_id: std::cell::Cell::new(1),
        }
    }

    /// Drain GN <-> BTP cross-channels, forwarding messages to their respective actors.
    pub fn drain_bridges(&self) {
        const MAX_DRAIN: usize = 512;

        // 1. GN -> BTP indications
        for _ in 0..MAX_DRAIN {
            match self.gn_to_btp_rx.try_recv() {
                Ok(ind) => self.btp_handle.send_gn_data_indication(ind),
                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
            }
        }

        // 2. BTP -> GN requests
        for _ in 0..MAX_DRAIN {
            match self.btp_to_gn_rx.try_recv() {
                Ok(req) => self.router_handle.send_gn_data_request(req),
                Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
            }
        }
    }

    /// Drain all pending outbound GeoNetworking frames produced by the GN router
    /// and package them for EDCA MAC channel contention.
    pub fn drain_outbound_frames(&self, now: SimTime) -> Vec<(AccessCategory, TxFrame)> {
        let mut frames = Vec::new();

        while let Ok(packet) = self.gn_to_ll_rx.try_recv() {
            let fid = self.next_frame_id.get();
            self.next_frame_id.set(fid + 1);

            // Access category: default to VoiceSafety for safety packets or BestEffort
            let ac = AccessCategory::BestEffort;

            let frame = TxFrame {
                frame_id: fid,
                payload: packet,
                created_time: now,
                duration: SimTime::from_micros(300),
            };

            frames.push((ac, frame));
        }

        frames
    }

    /// Inject a received raw GeoNetworking packet from PHY into the GeoNetworking router.
    pub fn inject_rx(&self, packet: &[u8]) {
        self.router_handle.send_incoming_packet(packet.to_vec());
    }

    /// Access the underlying GN router handle.
    pub fn gn_handle(&self) -> &RouterHandle {
        &self.router_handle
    }

    /// Access the underlying BTP router handle.
    pub fn btp_handle(&self) -> &BTPRouterHandle {
        &self.btp_handle
    }
}
