//! ETSI Cooperative Awareness Basic Service (CAM) via `rustflexstack`.
//!
//! Re-exports and integrates standard ETSI TS 103 900 / EN 302 637-2 CAM from `rustflexstack`.

pub use rustflexstack::facilities::ca_basic_service::{
    cam_coder::*,
    cam_reception::CAMReceptionManagement,
    cam_transmission::{
        CAMTransmissionManagement, VehicleData, T_CHECK_CAM_GEN_MS, T_GEN_CAM_MAX_MS,
        T_GEN_CAM_MIN_MS,
    },
    Cam, CamCoder, CooperativeAwarenessBasicService,
};
pub use rustflexstack::facilities::location_service::{GpsFix, LocationService};

use rustflexstack::btp::service_access_point::BTPDataRequest;
use rustflexstack::geonet::gn_address::{GNAddress, M, MID, ST};
use rustflexstack::geonet::service_access_point::{
    Area, CommonNH, CommunicationProfile, HeaderSubType, HeaderType, PacketTransportType,
    TopoBroadcastHST, TrafficClass,
};
use rustflexstack::security::sn_sap::SecurityProfile;

use crate::common::SimTime;
use crate::geometry::segment::Point2D;
use crate::scheduler::event::ActorId;



/// Decoded summary of a CAM message for telemetry and LDM tracking.
#[derive(Debug, Clone)]
pub struct CamSummary {
    pub station_id: ActorId,
    pub generation_time_ms: u64,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_m: f64,
    pub speed_m_s: f64,
    pub heading_deg: f64,
    pub acceleration_m_s2: f64,
    pub has_low_frequency_container: bool,
}

/// Helper to parse key vehicular fields from a standard `Cam`.
pub fn summarize_cam(cam: &Cam) -> CamSummary {
    let station_id = cam.header.station_id.0;
    let basic = &cam.cam.cam_parameters.basic_container;
    let lat = basic.reference_position.latitude.0 as f64 * 1e-7;
    let lon = basic.reference_position.longitude.0 as f64 * 1e-7;
    let alt = basic.reference_position.altitude.altitude_value.0 as f64 * 0.01;
    let has_low_frequency_container =
        cam.cam.cam_parameters.low_frequency_container.is_some();

    let (speed_m_s, heading_deg, accel_m_s2) = match cam.cam.cam_parameters.high_frequency_container
    {
        HighFrequencyContainer::basicVehicleContainerHighFrequency(ref hf) => {
            let s = hf.speed.speed_value.0 as f64 * 0.01;
            let h = hf.heading.heading_value.0 as f64 * 0.1;
            let a = hf.longitudinal_acceleration.value.0 as f64 * 0.1;
            (s, h, a)
        }
        _ => (0.0, 0.0, 0.0),
    };

    CamSummary {
        station_id,
        generation_time_ms: cam.cam.generation_delta_time.0 as u64,
        latitude: lat,
        longitude: lon,
        altitude_m: alt,
        speed_m_s,
        heading_deg,
        acceleration_m_s2: accel_m_s2,
        has_low_frequency_container,
    }
}

/// Build a standard ASN.1 UPER encoded `Cam` and wrap into a `BTPDataRequest` on port 2001.
pub fn build_cam_btp_request(
    vd: &VehicleData,
    fix: &GpsFix,
    accel_m_s2: f64,
) -> Option<BTPDataRequest> {
    build_cam_btp_request_with_lf(vd, fix, accel_m_s2, false)
}

/// Build a standard ASN.1 UPER encoded `Cam` with optional LowFrequencyContainer.
pub fn build_cam_btp_request_with_lf(
    vd: &VehicleData,
    fix: &GpsFix,
    accel_m_s2: f64,
    include_lf: bool,
) -> Option<BTPDataRequest> {
    let gen_dt = generation_delta_time_now();

    let ref_pos = ReferencePositionWithConfidence::new(
        Latitude(((fix.latitude * 1e7).round() as i32).clamp(-900_000_000, 900_000_000)),
        Longitude(((fix.longitude * 1e7).round() as i32).clamp(-1_800_000_000, 1_800_000_000)),
        PositionConfidenceEllipse::new(
            SemiAxisLength(4095),
            SemiAxisLength(4095),
            Wgs84AngleValue(3601),
        ),
        Altitude::new(
            AltitudeValue(((fix.altitude_m * 100.0).round() as i32).clamp(-100_000, 800_000)),
            AltitudeConfidence::unavailable,
        ),
    );

    let basic_container = BasicContainer::new(TrafficParticipantType(vd.station_type), ref_pos);

    let heading_value = HeadingValue(((fix.heading_deg * 10.0).round() as u16).clamp(0, 3600));
    let heading = Heading::new(heading_value, HeadingConfidence(127));

    let speed_value = SpeedValue(((fix.speed_mps * 100.0).round() as u16).min(16_382));
    let speed = Speed::new(speed_value, SpeedConfidence(127));

    let vehicle_length = VehicleLength::new(
        VehicleLengthValue(vd.vehicle_length_value.clamp(1, 1023)),
        VehicleLengthConfidenceIndication::unavailable,
    );
    let vehicle_width = VehicleWidth(vd.vehicle_width.clamp(1, 62));

    let accel_val = ((accel_m_s2 * 10.0).round() as i16).clamp(-160, 160);
    let longitudinal_acceleration =
        AccelerationComponent::new(AccelerationValue(accel_val), AccelerationConfidence(102));
    let curvature = Curvature::new(CurvatureValue(1023), CurvatureConfidence::unavailable);
    let yaw_rate = YawRate::new(YawRateValue(32767), YawRateConfidence::unavailable);

    let hf = BasicVehicleContainerHighFrequency::new(
        heading,
        speed,
        vd.drive_direction,
        vehicle_length,
        vehicle_width,
        longitudinal_acceleration,
        curvature,
        CurvatureCalculationMode::unavailable,
        yaw_rate,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );

    let lf = if include_lf {
        let role = VehicleRole::default;
        let ext_lights = ExteriorLights(rasn::types::FixedBitString::<8>::default());
        let path = Path(vec![]);
        Some(LowFrequencyContainer::basicVehicleContainerLowFrequency(
            BasicVehicleContainerLowFrequency::new(role, ext_lights, path),
        ))
    } else {
        None
    };

    let cam = Cam::new(
        cam_header(vd.station_id),
        CamPayload::new(
            gen_dt,
            CamParameters::new(
                basic_container,
                HighFrequencyContainer::basicVehicleContainerHighFrequency(hf),
                lf,
                None,
                None,
            ),
        ),
    );

    let coder = CamCoder::new();
    let data = coder.encode(&cam).ok()?;

    Some(BTPDataRequest {
        btp_type: CommonNH::BtpB,
        source_port: 0,
        destination_port: 2001,
        destination_port_info: 0,
        gn_packet_transport_type: PacketTransportType {
            header_type: HeaderType::Tsb,
            header_sub_type: HeaderSubType::TopoBroadcast(TopoBroadcastHST::SingleHop),
        },
        gn_destination_address: GNAddress {
            m: M::GnMulticast,
            st: ST::Unknown,
            mid: MID::new([0xFF; 6]),
        },
        communication_profile: CommunicationProfile::Unspecified,
        gn_area: Area {
            latitude: 0,
            longitude: 0,
            a: 0,
            b: 0,
            angle: 0,
        },
        traffic_class: TrafficClass {
            scf: false,
            channel_offload: false,
            tc_id: 0,
        },
        security_profile: SecurityProfile::CooperativeAwarenessMessage,
        its_aid: 36,
        security_permissions: vec![],
        gn_max_hop_limit: 1,
        gn_max_packet_lifetime: Some(1.0),
        gn_repetition_interval: None,
        gn_max_repetition_time: None,
        destination: None,
        length: data.len() as u16,
        data,
    })
}

/// Dynamic kinematic CAM trigger evaluator per ETSI EN 302 637-2 §6.1.3:
/// - |Delta heading| >= 4.0 degrees
/// - Delta distance >= 4.0 m
/// - |Delta speed| >= 0.5 m/s
/// - Clamped between T_min = 100 ms and T_max = 1000 ms.
#[derive(Debug, Clone)]
pub struct CamTriggerEvaluator {
    pub station_id: ActorId,
    pub is_dynamic: bool,
    pub t_min: SimTime,
    pub t_max: SimTime,
    last_cam_time: Option<SimTime>,
    last_pos: Option<Point2D>,
    last_speed: Option<f64>,
    last_heading: Option<f64>,
    pub cam_count: u64,
}

impl CamTriggerEvaluator {
    pub fn new(station_id: ActorId, is_dynamic: bool) -> Self {
        Self {
            station_id,
            is_dynamic,
            t_min: SimTime::from_millis(T_GEN_CAM_MIN_MS),
            t_max: SimTime::from_millis(T_GEN_CAM_MAX_MS),
            last_cam_time: None,
            last_pos: None,
            last_speed: None,
            last_heading: None,
            cam_count: 0,
        }
    }

    pub fn should_generate(
        &mut self,
        now: SimTime,
        pos: Point2D,
        speed: f64,
        heading: f64,
    ) -> bool {
        if self.last_cam_time.is_none() {
            self.record(now, pos, speed, heading);
            return true;
        }

        let last_time = self.last_cam_time.unwrap();
        let elapsed = if now > last_time {
            now - last_time
        } else {
            SimTime::ZERO
        };

        if elapsed < self.t_min {
            return false;
        }

        if elapsed >= self.t_max {
            self.record(now, pos, speed, heading);
            return true;
        }

        if !self.is_dynamic {
            if elapsed >= SimTime::from_millis(100) {
                self.record(now, pos, speed, heading);
                return true;
            }
            return false;
        }

        let prev_pos = self.last_pos.unwrap_or(pos);
        let prev_speed = self.last_speed.unwrap_or(speed);
        let prev_heading = self.last_heading.unwrap_or(heading);

        let delta_dist = pos.distance_to(&prev_pos);
        let delta_speed = (speed - prev_speed).abs();
        let delta_heading = angle_diff_deg(heading, prev_heading).abs();

        // Standard ETSI TS 103 900 §6.1.3:
        // Moving vehicles on the road generate CAMs at 10 Hz (t_min = 100 ms)
        // when active/moving (speed >= 0.1 m/s) or dynamics exceed thresholds.
        let is_moving = speed >= 0.1;
        if is_moving || delta_dist >= 4.0 || delta_speed >= 0.5 || delta_heading >= 4.0 {
            self.record(now, pos, speed, heading);
            true
        } else {
            false
        }
    }

    fn record(&mut self, now: SimTime, pos: Point2D, speed: f64, heading: f64) {
        self.last_cam_time = Some(now);
        self.last_pos = Some(pos);
        self.last_speed = Some(speed);
        self.last_heading = Some(heading);
        self.cam_count += 1;
    }
}

fn angle_diff_deg(a: f64, b: f64) -> f64 {
    let mut diff = (a - b).rem_euclid(360.0);
    if diff > 180.0 {
        diff -= 360.0;
    }
    diff
}
