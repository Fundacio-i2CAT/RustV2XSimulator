//! High-performance binary PCAP file writer.
//!
//! Generates standard libpcap files directly parseable by Wireshark, tshark,
//! tcpdump, and capinfos without external C library dependencies.

use byteorder::{BigEndian, WriteBytesExt};
use std::fs::{create_dir_all, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::common::{SimError, SimResult, SimTime};
use crate::pcap::ethernet_frame::EthernetFrame;
use crate::scheduler::event::ActorId;

pub const PCAP_MAGIC_NUMBER: u32 = 0xa1b2c3d4;
pub const PCAP_VERSION_MAJOR: u16 = 2;
pub const PCAP_VERSION_MINOR: u16 = 4;
pub const LINKTYPE_ETHERNET: u32 = 1;

pub struct PcapWriter {
    writer: Option<BufWriter<File>>,
    packet_count: u64,
}

impl PcapWriter {
    /// Create a new PCAP writer and write standard 24-byte libpcap global header.
    pub fn open<P: AsRef<Path>>(path: P) -> SimResult<Self> {
        let p = path.as_ref();
        if let Some(parent) = p.parent() {
            create_dir_all(parent)?;
        }

        let file = File::create(p)
            .map_err(|e| SimError::IoError(format!("Failed to create PCAP file: {}", e)))?;
        let mut writer = BufWriter::with_capacity(64 * 1024, file);

        // Write 24-byte PCAP Global Header (Big-Endian)
        writer.write_u32::<BigEndian>(PCAP_MAGIC_NUMBER)?;
        writer.write_u16::<BigEndian>(PCAP_VERSION_MAJOR)?;
        writer.write_u16::<BigEndian>(PCAP_VERSION_MINOR)?;
        writer.write_i32::<BigEndian>(0)?; // thiszone (GMT)
        writer.write_u32::<BigEndian>(0)?; // sigfigs
        writer.write_u32::<BigEndian>(65535)?; // snaplen
        writer.write_u32::<BigEndian>(LINKTYPE_ETHERNET)?; // network linktype

        writer.flush()?;

        Ok(Self {
            writer: Some(writer),
            packet_count: 0,
        })
    }

    /// Write an Ethernet II frame carrying a GeoNetworking packet to PCAP file.
    pub fn write_frame(
        &mut self,
        timestamp: SimTime,
        sender_id: ActorId,
        gn_packet: &[u8],
    ) -> SimResult<()> {
        if let Some(ref mut w) = self.writer {
            let eth_frame = EthernetFrame::encapsulate(sender_id, gn_packet);

            // Compute seconds and microseconds from SimTime (100 ns ticks)
            let total_us = timestamp.to_micros();
            let ts_sec = (total_us / 1_000_000) as u32;
            let ts_usec = (total_us % 1_000_000) as u32;
            let frame_len = eth_frame.len() as u32;

            // 16-byte PCAP Record Header
            w.write_u32::<BigEndian>(ts_sec)?;
            w.write_u32::<BigEndian>(ts_usec)?;
            w.write_u32::<BigEndian>(frame_len)?; // incl_len
            w.write_u32::<BigEndian>(frame_len)?; // orig_len

            // Packet Data
            w.write_all(&eth_frame)?;
            self.packet_count += 1;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> SimResult<()> {
        if let Some(ref mut w) = self.writer {
            w.flush()?;
        }
        Ok(())
    }

    pub fn close(&mut self) -> SimResult<()> {
        self.flush()?;
        self.writer = None;
        Ok(())
    }

    #[inline]
    pub fn packet_count(&self) -> u64 {
        self.packet_count
    }
}

impl Drop for PcapWriter {
    fn drop(&mut self) {
        let _ = self.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_pcap_generation_and_reading() {
        let tmp = NamedTempFile::new().unwrap();
        let path = tmp.path();

        let mut writer = PcapWriter::open(path).expect("open writer");
        let t = SimTime::from_millis(1500);
        let dummy_payload = vec![0x10, 0x00, 0x01, 0x01, 0x20, 0x50, 0x02, 0x00];

        writer
            .write_frame(t, 42, &dummy_payload)
            .expect("write frame");
        writer.close().expect("close writer");

        // Verify file size: 24 (global header) + 16 (record header) + 14 (eth) + 8 (payload) = 62 bytes
        let meta = std::fs::metadata(path).unwrap();
        assert_eq!(meta.len(), 62);
    }
}
