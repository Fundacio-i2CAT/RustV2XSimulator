//! Physical layer, path loss models, knife-edge diffraction, chunked SINR, and parallel radio medium.

pub mod air_packet;
pub mod consts_80211p;
pub mod cv2x;
pub mod decider_80211p;
pub mod interference_helper;
pub mod knife_edge;
pub mod nist_error_rate;
pub mod path_loss;
pub mod radio_medium;
pub mod technology;
pub mod nrv2x;
pub mod wifi80211bd;

pub use air_packet::AirPacket;
pub use consts_80211p::{compute_duration_80211p, DsrcChannel, Mcs80211p};
pub use cv2x::{CandidateResource, Cv2xConfig, SensingBasedSpsScheduler, SidelinkControlInfo, SidelinkSensingManager};
pub use decider_80211p::{Decider80211p, DeciderResult80211, PacketDecodeStatus};
pub use interference_helper::{
    dbm_to_watts, watts_to_dbm, CumulativeInterferenceTracker, SignalEvent, SinrChunk,
};
pub use knife_edge::FresnelKnifeEdge;
pub use nist_error_rate::{get_global_nist_lut, NistErrorRate, NistLut};
pub use nrv2x::{
    NrHarqFeedback, NrMode2Grant, NrMode2Scheduler, NrNumerology, NrPhyBlerModel, NrReceptionResult,
    NrResource, NrResourceGrid, NrSciMeasurement, NrSensingManager, NrV2xConfig, NrV2xMode2Model,
    SciFormat1A, SciFormat2A,
};
pub use path_loss::{DualSlopePathLoss, LegacyPolynomialPathLoss, SPEED_OF_LIGHT};
pub use radio_medium::{PhyModelType, RadioMedium, ReceptionResult};
pub use technology::{
    Cv2xMode4Model, Dsrc80211pModel, NrV2xMode2Model as NrV2xTechModel, ParametricEmpiricalModel,
    Wifi80211bdModel, WirelessTechnology, WirelessTechnologyType,
};
pub use wifi80211bd::{
    compute_duration_80211bd, compute_lsig_length_spoofed, Decider80211bd, DeciderResult80211bd,
    DopplerChannelEngine, LdpcErrorRate, Mcs80211bd, MidamblePeriod, Wifi80211bdConfig,
};

