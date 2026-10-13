//! Standard PCAP generation and Ethernet II frame encapsulation.

pub mod ethernet_frame;
pub mod writer;

pub use ethernet_frame::{EthernetFrame, ETHERNET_HEADER_LEN};
pub use writer::PcapWriter;
