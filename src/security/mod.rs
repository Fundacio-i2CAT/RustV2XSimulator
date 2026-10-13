//! In-Memory ETSI TS 103 097 / IEEE 1609.2 Security & PKI Credentials Provider.
//!
//! Provides self-contained certificate generation (Root CA → AA → ATs) and
//! credential provisioning for simulated vehicles using `rustflexstack`.

use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

use rasn::prelude::*;

pub use rustflexstack::geonet::mib::GnSecurity;
pub use rustflexstack::security::certificate::{Certificate, OwnCertificate};
pub use rustflexstack::security::certificate_library::CertificateLibrary;
pub use rustflexstack::security::ecdsa_backend::EcdsaBackend;
pub use rustflexstack::security::security_asn::ieee1609_dot2::{
    CertificateId, EndEntityType, PsidGroupPermissions, SequenceOfPsidGroupPermissions,
    SubjectPermissions, ToBeSignedCertificate, VerificationKeyIndicator,
};
pub use rustflexstack::security::security_asn::ieee1609_dot2_base_types::{
    CrlSeries, Duration, EccP256CurvePoint, HashedId3, Hostname, Psid, PsidSsp, PsidSspRange,
    PublicVerificationKey, SequenceOfPsidSsp, SequenceOfPsidSspRange, Time32, Uint16, Uint32,
    ValidityPeriod,
};
pub use rustflexstack::security::sign_service::SignService;
pub use rustflexstack::security::sn_sap::{ReportVerify, SNSignRequest, SNVerifyRequest};
pub use rustflexstack::security::verify_service::verify_message;

// ETSI ITS-AID values
pub const ITS_AID_CAM: u64 = 36;
pub const ITS_AID_DENM: u64 = 37;
pub const ITS_AID_VAM: u64 = 638;

/// ITS epoch: 2004-01-01 00:00:00 UTC as a UNIX timestamp.
const ITS_EPOCH_UNIX: u64 = 1_072_915_200;

/// Return current time as seconds since ITS epoch.
pub fn its_time32_now() -> Time32 {
    let unix_secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_secs();
    let its_secs = unix_secs.saturating_sub(ITS_EPOCH_UNIX) as u32;
    Time32(Uint32(its_secs))
}

/// Default EndEntityType: app only (bit 0 = true, bit 1 = false).
fn default_ee_type() -> EndEntityType {
    let mut bits = FixedBitString::<8>::default();
    bits.set(0, true);
    EndEntityType(bits)
}

/// Helper: create a `SequenceOfPsidSsp` with the given PSIDs (no SSP).
pub fn app_permissions(psids: &[u64]) -> SequenceOfPsidSsp {
    let items: Vec<PsidSsp> = psids
        .iter()
        .map(|&p| PsidSsp::new(Psid(Integer::from(p as i128)), None))
        .collect();
    SequenceOfPsidSsp(items)
}

/// Helper: build a `SequenceOfPsidGroupPermissions` with `SubjectPermissions::explicit`
/// for the given PSIDs and minimum chain length.
pub fn cert_issue_permissions_explicit(
    psids: &[u64],
    min_chain_length: i128,
) -> SequenceOfPsidGroupPermissions {
    let ranges: Vec<PsidSspRange> = psids
        .iter()
        .map(|&p| PsidSspRange::new(Psid(Integer::from(p as i128)), None))
        .collect();
    let perm = PsidGroupPermissions::new(
        SubjectPermissions::explicit(SequenceOfPsidSspRange(ranges)),
        Integer::from(min_chain_length),
        Integer::from(0i128),
        default_ee_type(),
    );
    SequenceOfPsidGroupPermissions(vec![perm])
}

/// Helper: build a `SequenceOfPsidGroupPermissions` with `SubjectPermissions::all`.
pub fn cert_issue_permissions_all(min_chain_length: i128) -> SequenceOfPsidGroupPermissions {
    let perm = PsidGroupPermissions::new(
        SubjectPermissions::all(()),
        Integer::from(min_chain_length),
        Integer::from(0i128),
        default_ee_type(),
    );
    SequenceOfPsidGroupPermissions(vec![perm])
}

/// Cryptographic credentials for an individual simulated vehicle.
#[derive(Clone, Debug)]
pub struct VehicleCredentials {
    pub station_id: u32,
    pub cert: Certificate,
    pub key_bytes: Vec<u8>,
}

/// In-memory PKI hierarchy (Root CA → Authorization Authority → Authorization Tickets)
/// for local C-ITS security without an external PKI server.
pub struct SimPki {
    backend: EcdsaBackend,
    pub root_ca: OwnCertificate,
    pub aa: OwnCertificate,
    issued_ats: HashMap<u32, VehicleCredentials>,
}

impl SimPki {
    /// Generate a brand new certificate chain: Root CA (self-signed) and AA (issued by Root CA).
    pub fn new() -> Self {
        let mut backend = EcdsaBackend::new();

        // ─── 1. Root CA ───────────────────────────────────────────────────
        let root_tbs = ToBeSignedCertificate::new(
            CertificateId::name(Hostname("RootCA".to_string())),
            HashedId3(FixedOctetString::from([0u8; 3])),
            CrlSeries(Uint16(0)),
            ValidityPeriod::new(its_time32_now(), Duration::years(Uint16(20))),
            None,
            None,
            Some(app_permissions(&[ITS_AID_CAM, ITS_AID_DENM, ITS_AID_VAM])),
            Some(cert_issue_permissions_all(2)),
            None,
            None,
            None,
            VerificationKeyIndicator::verificationKey(PublicVerificationKey::ecdsaNistP256(
                EccP256CurvePoint::x_only(vec![0u8; 32].into()),
            )),
            None,
            None,
            None,
            None,
        );
        let root_ca = OwnCertificate::initialize_self_signed(&mut backend, root_tbs);

        // ─── 2. Authorization Authority (AA) ──────────────────────────────
        let aa_tbs = ToBeSignedCertificate::new(
            CertificateId::name(Hostname("AA".to_string())),
            HashedId3(FixedOctetString::from([0u8; 3])),
            CrlSeries(Uint16(0)),
            ValidityPeriod::new(its_time32_now(), Duration::years(Uint16(10))),
            None,
            None,
            Some(app_permissions(&[ITS_AID_CAM, ITS_AID_DENM, ITS_AID_VAM])),
            Some(cert_issue_permissions_explicit(
                &[ITS_AID_CAM, ITS_AID_DENM, ITS_AID_VAM],
                1,
            )),
            None,
            None,
            None,
            VerificationKeyIndicator::verificationKey(PublicVerificationKey::ecdsaNistP256(
                EccP256CurvePoint::x_only(vec![0u8; 32].into()),
            )),
            None,
            None,
            None,
            None,
        );
        let aa = OwnCertificate::initialize_issued(&mut backend, aa_tbs, &root_ca);

        Self {
            backend,
            root_ca,
            aa,
            issued_ats: HashMap::new(),
        }
    }

    /// Issue an Authorization Ticket (AT) for a given `station_id`.
    ///
    /// If an AT was already issued for this `station_id`, the cached credentials
    /// are returned.
    pub fn issue_at(&mut self, station_id: u32) -> VehicleCredentials {
        if let Some(existing) = self.issued_ats.get(&station_id) {
            return existing.clone();
        }

        let at_tbs = ToBeSignedCertificate::new(
            CertificateId::none(()),
            HashedId3(FixedOctetString::from([0u8; 3])),
            CrlSeries(Uint16(0)),
            ValidityPeriod::new(its_time32_now(), Duration::years(Uint16(5))),
            None,
            None,
            Some(app_permissions(&[ITS_AID_CAM, ITS_AID_DENM, ITS_AID_VAM])),
            None,
            None,
            None,
            None,
            VerificationKeyIndicator::verificationKey(PublicVerificationKey::ecdsaNistP256(
                EccP256CurvePoint::x_only(vec![0u8; 32].into()),
            )),
            None,
            None,
            None,
            None,
        );

        let own_at = OwnCertificate::initialize_issued(&mut self.backend, at_tbs, &self.aa);
        let key_bytes = self.backend.export_signing_key(own_at.key_id);

        let creds = VehicleCredentials {
            station_id,
            cert: own_at.cert.clone(),
            key_bytes,
        };

        self.issued_ats.insert(station_id, creds.clone());
        creds
    }

    /// Pre-generate ATs for a list of station IDs.
    pub fn pregenerate_ats(&mut self, station_ids: &[u32]) {
        for &id in station_ids {
            self.issue_at(id);
        }
    }

    /// Return all issued public AT certificates so far.
    pub fn get_all_at_certificates(&self) -> Vec<Certificate> {
        self.issued_ats.values().map(|c| c.cert.clone()).collect()
    }

    /// Setup the full security triple `(SignService, Option<EcdsaBackend>, Option<CertificateLibrary>)`
    /// for a vehicle to be passed into `GNRouter::spawn`.
    pub fn setup_vehicle_security(
        &self,
        creds: &VehicleCredentials,
        verify_packets: bool,
    ) -> (SignService, Option<EcdsaBackend>, Option<CertificateLibrary>) {
        let mut vehicle_backend = EcdsaBackend::new();
        let key_id = vehicle_backend.import_signing_key(&creds.key_bytes);
        let own_cert = OwnCertificate::new(creds.cert.clone(), key_id);

        let known_ats = self.get_all_at_certificates();

        let cert_library = CertificateLibrary::new(
            &vehicle_backend,
            vec![self.root_ca.cert.clone()],
            vec![self.aa.cert.clone()],
            known_ats.clone(),
        );

        let mut sign_service = SignService::new(vehicle_backend, cert_library);
        sign_service.add_own_certificate(own_cert);

        let (verify_backend, verify_cert_library) = if verify_packets {
            let backend = EcdsaBackend::new();
            let lib = CertificateLibrary::new(
                &backend,
                vec![self.root_ca.cert.clone()],
                vec![self.aa.cert.clone()],
                known_ats,
            );
            (Some(backend), Some(lib))
        } else {
            (None, None)
        };

        (sign_service, verify_backend, verify_cert_library)
    }

    /// Verify that the Root CA and AA signatures in the chain are valid.
    pub fn verify_chain(&self) -> bool {
        self.root_ca.verify(&self.backend) && self.aa.verify(&self.backend)
    }
}

impl Default for SimPki {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_certificate_chain_generation_and_verification() {
        let mut pki = SimPki::new();
        assert!(pki.verify_chain(), "Root CA and AA must verify");

        let creds1 = pki.issue_at(1);
        let creds2 = pki.issue_at(2);

        assert_eq!(creds1.station_id, 1);
        assert_eq!(creds2.station_id, 2);
        assert_ne!(creds1.key_bytes, creds2.key_bytes);

        // Verification of ATs against backend
        assert!(creds1.cert.verify(&pki.backend));
        assert!(creds2.cert.verify(&pki.backend));
    }

    #[test]
    fn test_sign_and_verify_roundtrip() {
        let mut pki = SimPki::new();
        pki.pregenerate_ats(&[1, 2]);

        let creds1 = pki.issue_at(1);
        let creds2 = pki.issue_at(2);

        let (mut sign_svc1, _, _) = pki.setup_vehicle_security(&creds1, true);
        let (_, verify_backend2, verify_cert_lib2) = pki.setup_vehicle_security(&creds2, true);

        let verify_backend2 = verify_backend2.unwrap();
        let mut verify_cert_lib2 = verify_cert_lib2.unwrap();

        // Sign a CAM request (ITS-AID 36)
        let tbs_payload = vec![0xCA, 0xFE, 0xBA, 0xBE];
        let sign_req = SNSignRequest {
            tbs_message: tbs_payload.clone(),
            its_aid: ITS_AID_CAM,
            permissions: vec![],
            generation_location: None,
        };
        let sign_confirm = sign_svc1.sign_request(&sign_req);
        assert!(!sign_confirm.sec_message.is_empty());

        // Verify with Vehicle 2
        let verify_req = SNVerifyRequest {
            message: sign_confirm.sec_message,
        };
        let (verify_confirm, _) =
            verify_message(&verify_req, &verify_backend2, &mut verify_cert_lib2);

        assert_eq!(verify_confirm.report, ReportVerify::Success);
        assert_eq!(verify_confirm.its_aid, ITS_AID_CAM);
        assert_eq!(verify_confirm.plain_message, tbs_payload);
    }
}
