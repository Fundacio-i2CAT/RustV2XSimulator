//! 3GPP Release 16/17 5G NR-V2X Sidelink Mode 2 Suite.
//!
//! Standards Reference:
//! - 3GPP TS 38.211: Physical channels and modulation (Numerology, Frame Structure)
//! - 3GPP TS 38.212: Multiplexing and channel coding (Two-Stage SCI, Polar & LDPC)
//! - 3GPP TS 38.213: Physical layer procedures for control (Section 16: Sidelink Mode 2, PSFCH)
//! - 3GPP TS 38.214: Physical layer procedures for data (Table 5.1.3.1-1 MCS Table)
//! - 3GPP TS 38.321: Medium Access Control (MAC) protocol specification (Sidelink HARQ)
//!
//! Provenance & Validation References:
//! - CTTC 5G-LENA (`nr-sl-ue-mac.cc`, `nr-sl-sci-f1a-header.cc`, `nr-sl-error-model.cc`)
//! - UWICORE NR-V2X Link-Level Simulator (IEEE IoT-J 2024: `nrSCI1Encode.m`, `getMCSinformation.m`, `HARQEntity.m`)

pub mod config;
pub mod consts_nrv2x;
pub mod ldpc_bler;
pub mod mode2_scheduler;
pub mod model;
pub mod psfch_harq;
pub mod resource_grid;
pub mod sci;
pub mod sensing;

pub use config::NrV2xConfig;
pub use consts_nrv2x::{
    compute_nr_tbs_bytes, get_nr_mcs_entry, NrMcsEntry, NrModulation, NrNumerology, NR_MCS_TABLE_1,
};
pub use ldpc_bler::{NrPhyBlerModel, NrReceptionResult};
pub use mode2_scheduler::{NrMode2Grant, NrMode2Scheduler};
pub use model::NrV2xMode2Model;
pub use psfch_harq::{
    NrHarqEntity, NrHarqFeedback, NrHarqProcess, PsfchTimingCalculator, NR_HARQ_RV_SEQUENCE,
};
pub use resource_grid::{NrResource, NrResourceGrid};
pub use sci::{NrCastType, SciFormat1A, SciFormat2A, SciStage2Format};
pub use sensing::{NrSciMeasurement, NrSensingManager};
