//! ETSI GeoNetworking (GN) Protocol Implementation via `rustflexstack`.
//!
//! Re-exports and integrates standard ETSI EN 302 636-4-1 components from `rustflexstack`.

pub use rustflexstack::geonet::basic_header::{BasicHeader, BasicNH};
pub use rustflexstack::geonet::common_header::CommonHeader;
pub use rustflexstack::geonet::gbc_extended_header::GBCExtendedHeader;
pub use rustflexstack::geonet::gn_address::{GNAddress, M, MID, ST};
pub use rustflexstack::geonet::location_table::LocationTable;
pub use rustflexstack::geonet::mib::{GnSecurity, Mib};
pub use rustflexstack::geonet::position_vector::{LongPositionVector, ShortPositionVector, Tst};
pub use rustflexstack::geonet::router::{
    Router as GNRouter, RouterHandle, RouterInput, RouterOutput,
};
pub use rustflexstack::geonet::service_access_point::{
    Area, CommonNH, CommunicationProfile, GNDataIndication, GNDataRequest, HeaderSubType,
    HeaderType, PacketTransportType, TopoBroadcastHST, TrafficClass,
};
pub use rustflexstack::link_layer::packet_consts::ETH_P_GEONET as GN_ETHERTYPE;

use crate::common::{SimTime, Wgs84Coordinate};
use crate::scheduler::event::ActorId;

/// Helper to build a `LongPositionVector` from simulation state.
pub fn create_long_position_vector(
    _station_id: ActorId,
    mac: [u8; 6],
    now: SimTime,
    coord: &Wgs84Coordinate,
    speed_m_s: f64,
    heading_deg: f64,
) -> LongPositionVector {
    let lat_etsi = ((coord.latitude * 1e7).round() as i64).clamp(-900_000_000, 900_000_000) as u32;
    let lon_etsi =
        ((coord.longitude * 1e7).round() as i64).clamp(-1_800_000_000, 1_800_000_000) as u32;
    let speed_cm_s = ((speed_m_s * 100.0).round() as u64).min(16_382) as u16;
    let heading_tenths = ((heading_deg.rem_euclid(360.0) * 10.0).round() as u64).min(3600) as u16;

    LongPositionVector {
        gn_addr: GNAddress::new(M::GnMulticast, ST::PassengerCar, MID::new(mac)),
        tst: Tst::set_in_normal_timestamp_milliseconds(now.to_millis()),
        latitude: lat_etsi,
        longitude: lon_etsi,
        pai: true,
        s: speed_cm_s,
        h: heading_tenths,
    }
}
