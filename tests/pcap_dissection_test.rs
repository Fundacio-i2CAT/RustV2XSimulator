//! Integration Test 5: Standard PCAP Dissection & Protocol Framing.
//!
//! Validates generated .pcap traces for:
//! - Standard libpcap global header (magic 0xa1b2c3d4, version 2.4, LinkType 1 Ethernet)
//! - Valid Ethernet II framing with EtherType 0x8947 (ETSI GeoNetworking)
//! - GeoNetworking SHB header and BTP-B encapsulation.

use byteorder::{BigEndian, ByteOrder};
use std::fs::File;
use std::io::Read;
use tempfile::tempdir;

use rustv2xsimulator::cits::geonet::GN_ETHERTYPE;
use rustv2xsimulator::common::SimTime;
use rustv2xsimulator::pcap::writer::{
    PcapWriter, PCAP_MAGIC_NUMBER, PCAP_VERSION_MAJOR, PCAP_VERSION_MINOR,
};

#[test]
fn test_pcap_dissection_protocol_conformance() {
    let tmp_dir = tempdir().expect("tempdir");
    let pcap_path = tmp_dir.path().join("test_traffic.pcap");

    {
        let mut writer = PcapWriter::open(&pcap_path).expect("open pcap");

        // Construct standard GeoNetworking packet:
        // Basic Header (4 bytes) + Common Header (8 bytes) + Source LPV (24 bytes) + BTP (4 bytes) + CAM (25 bytes)
        let mut gn_pkt = vec![
            0x10, 0x00, 0x01, 0x01, // Basic Header
            0x20, 0x50, 0x02, 0x00, // Common Header
        ];
        gn_pkt.extend_from_slice(&(29u16).to_be_bytes()); // Payload len
        gn_pkt.push(0x01);
        gn_pkt.push(0x00);

        // Extended Header (LPV 24 bytes)
        gn_pkt.extend_from_slice(&[0u8; 24]);

        // Payload: BTP-B (Port 2001) + CAM
        gn_pkt.extend_from_slice(&(2001u16).to_be_bytes());
        gn_pkt.extend_from_slice(&(0u16).to_be_bytes());
        gn_pkt.extend_from_slice(&[0x02, 0x02]); // CAM PDU

        writer
            .write_frame(SimTime::from_millis(500), 101, &gn_pkt)
            .expect("write frame 1");
        writer
            .write_frame(SimTime::from_millis(600), 102, &gn_pkt)
            .expect("write frame 2");
        writer.close().expect("close pcap");
    }

    // Read back and dissect binary PCAP format
    let mut file = File::open(&pcap_path).expect("open pcap file");
    let mut buf = Vec::new();
    file.read_to_end(&mut buf).expect("read file");

    assert!(
        buf.len() >= 24,
        "PCAP must contain at least 24-byte global header"
    );

    // 1. Validate 24-byte Global Header
    let magic = BigEndian::read_u32(&buf[0..4]);
    let v_maj = BigEndian::read_u16(&buf[4..6]);
    let v_min = BigEndian::read_u16(&buf[6..8]);
    let snaplen = BigEndian::read_u32(&buf[16..20]);
    let linktype = BigEndian::read_u32(&buf[20..24]);

    assert_eq!(
        magic, PCAP_MAGIC_NUMBER,
        "Magic number must match standard libpcap"
    );
    assert_eq!(v_maj, PCAP_VERSION_MAJOR);
    assert_eq!(v_min, PCAP_VERSION_MINOR);
    assert_eq!(snaplen, 65535);
    assert_eq!(linktype, 1, "LinkType must be 1 (LINKTYPE_ETHERNET)");

    // 2. Validate First Packet Record
    let offset = 24;
    let ts_sec = BigEndian::read_u32(&buf[offset..offset + 4]);
    let ts_usec = BigEndian::read_u32(&buf[offset + 4..offset + 8]);
    let incl_len = BigEndian::read_u32(&buf[offset + 8..offset + 12]) as usize;

    assert_eq!(ts_sec, 0);
    assert_eq!(ts_usec, 500_000); // 500 ms = 500,000 us

    // 3. Validate Ethernet Frame Header
    let eth_offset = offset + 16;
    let ethertype = BigEndian::read_u16(&buf[eth_offset + 12..eth_offset + 14]);
    assert_eq!(
        ethertype, GN_ETHERTYPE,
        "EtherType must match standard ETSI GeoNetworking (0x8947)"
    );

    // 4. Validate GeoNetworking Basic & Common Header
    let gn_offset = eth_offset + 14;
    let basic_ver = buf[gn_offset] >> 4;
    assert_eq!(basic_ver, 1, "GeoNetworking version must be 1");

    let common_next_header = buf[gn_offset + 4] >> 4;
    assert_eq!(
        common_next_header, 2,
        "Common Header next_header must be 2 (BTP-B)"
    );

    println!(
        "PCAP frame dissection verified successfully. Frame length: {} bytes",
        incl_len
    );
}
