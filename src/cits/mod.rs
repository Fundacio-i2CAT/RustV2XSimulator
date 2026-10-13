//! Standard ETSI C-ITS Protocol Stack using official `rustflexstack` implementations:
//! - CAM: [`CooperativeAwarenessBasicService`]
//! - DENM: [`DecentralizedEnvironmentalNotificationService`]
//! - GeoNetworking: [`GNRouter`]
//! - BTP: [`BTPRouter`]
//! - LDM: [`LdmFacility`]
//! - Location: [`LocationService`]
//! - Simulator Link Layer: [`SimulatorLinkLayer`]

pub mod btp;
pub mod cam;
pub mod denm;
pub mod geonet;
pub mod ldm;
pub mod link_layer;

pub use btp::{BTPRouter, BTPRouterHandle, BTP_PORT_CAM, BTP_PORT_DENM};
pub use cam::{
    summarize_cam, Cam, CamCoder, CamSummary, CamTriggerEvaluator,
    CooperativeAwarenessBasicService, GpsFix, LocationService, VehicleData,
};
pub use denm::{
    create_eebl_request, DENRequest, DecentralizedEnvironmentalNotificationService, Denm,
    DenmCoder, CAUSE_CODE_EMERGENCY_BRAKE, SUBCAUSE_CODE_EEBL,
};
pub use geonet::{
    create_long_position_vector, GNAddress, GNRouter, LongPositionVector, Mib, RouterHandle,
    GN_ETHERTYPE,
};
pub use ldm::{IfLdm3, IfLdm4, ItsDataObject, LdmFacility, LdmHandle};
pub use link_layer::SimulatorLinkLayer;
