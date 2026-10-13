//! Standard Ethernet II framing for ETSI C-ITS GeoNetworking packets.

use crate::cits::geonet::GN_ETHERTYPE;
use crate::scheduler::event::ActorId;
use byteorder::{BigEndian, ByteOrder};

pub const ETHERNET_HEADER_LEN: usize = 14;

/// Constructs an Ethernet II frame carrying an ETSI GeoNetworking packet.
#[derive(Debug, Clone)]
pub struct EthernetFrame;

impl EthernetFrame {
    /// Encapsulate GeoNetworking payload with standard Ethernet II header:
    /// - Destination MAC: FF:FF:FF:FF:FF:FF (Broadcast)
    /// - Source MAC: 02:00:xx:xx:xx:xx (Derived from sender ActorId)
    /// - EtherType: 0x8947 (ETSI GeoNetworking)
    pub fn encapsulate(sender_id: ActorId, gn_packet: &[u8]) -> Vec<u8> {
        let mut frame = Vec::with_capacity(ETHERNET_HEADER_LEN + gn_packet.len());

        // Destination MAC: Broadcast
        frame.extend_from_slice(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);

        // Source MAC: Locally Administered (02:00:station_id)
        let mut src_mac = [0x02, 0x00, 0x00, 0x00, 0x00, 0x00];
        BigEndian::write_u32(&mut src_mac[2..6], sender_id);
        frame.extend_from_slice(&src_mac);

        // EtherType: 0x8947
        frame.extend_from_slice(&GN_ETHERTYPE.to_be_bytes());

        // Payload
        frame.extend_from_slice(gn_packet);
        frame
    }

    /// Extract EtherType and inner payload from an Ethernet II frame.
    pub fn decapsulate(frame: &[u8]) -> Option<(u16, &[u8])> {
        if frame.len() < ETHERNET_HEADER_LEN {
            return None;
        }
        let ethertype = BigEndian::read_u16(&frame[12..14]);
        let payload = &frame[14..];
        Some((ethertype, payload))
    }
}
