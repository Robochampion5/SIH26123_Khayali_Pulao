use crate::network::payloads::Packet;
use std::collections::HashMap;
use std::net::{UdpSocket, SocketAddr};
use std::io;

/// Maximum UDP payload we will accept
const MAX_PACKET_SIZE: usize = 1472;

/// Rate limit: max INTENT broadcasts per 5 ticks per agent
const INTENT_RATE_LIMIT: u8 = 1;

/// Transport layer abstraction over UDP multicast.
///
/// In production, this would be replaced with Zenoh or similar.
/// For testing, this uses standard UDP sockets.
pub struct TransportLayer {
    socket: Option<UdpSocket>,
    multicast_addr: SocketAddr,
    /// Deduplication: (agent_id, seq_no) → received?
    seen_intents: HashMap<(u8, u16), bool>,
    /// Outbound rate limiter: agent_id → ticks since last INTENT broadcast
    rate_limiter: HashMap<u8, u8>,
    /// Inbound packet buffer
    recv_buf: Vec<u8>,
}

impl TransportLayer {
    /// Create a transport layer bound to a UDP multicast group.
    ///
    /// `bind_addr`: local address to bind (e.g., "0.0.0.0:5000")
    /// `multicast_addr`: multicast group address (e.g., "239.0.0.1:5000")
    pub fn new(bind_addr: &str, multicast_addr: &str) -> io::Result<Self> {
        let socket = UdpSocket::bind(bind_addr)?;
        socket.set_nonblocking(true)?;

        let mcast: SocketAddr = multicast_addr.parse()
            .map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bad multicast addr"))?;

        // Join multicast group
        if let std::net::IpAddr::V4(ipv4) = mcast.ip() {
            socket.join_multicast_v4(&ipv4, &std::net::Ipv4Addr::UNSPECIFIED)?;
        }

        Ok(Self {
            socket: Some(socket),
            multicast_addr: mcast,
            seen_intents: HashMap::new(),
            rate_limiter: HashMap::new(),
            recv_buf: vec![0u8; MAX_PACKET_SIZE],
        })
    }

    /// Create a mock transport layer for testing (no actual network)
    pub fn mock() -> Self {
        Self {
            socket: None,
            multicast_addr: "127.0.0.1:5000".parse().unwrap(),
            seen_intents: HashMap::new(),
            rate_limiter: HashMap::new(),
            recv_buf: vec![0u8; MAX_PACKET_SIZE],
        }
    }

    /// Send a packet to the multicast group
    pub fn send(&self, packet: &Packet) -> io::Result<()> {
        if let Some(ref socket) = self.socket {
            let data = packet.to_bytes();
            socket.send_to(&data, self.multicast_addr)?;
        }
        Ok(())
    }

    /// Non-blocking receive: drain all pending packets.
    /// Returns decoded packets. Deduplicates INTENT by (agent_id, seq_no).
    pub fn recv_all(&mut self) -> Vec<Packet> {
        let mut packets = Vec::new();

        if let Some(ref socket) = self.socket {
            loop {
                match socket.recv_from(&mut self.recv_buf) {
                    Ok((n, _addr)) => {
                        if n == 0 { break; }
                        if let Some(pkt) = Packet::from_bytes(&self.recv_buf[..n]) {
                            // Dedup INTENT by (agent_id, seq_no)
                            if let Packet::Intent(ref intent) = pkt {
                                let key = (intent.agent_id, intent.seq_no);
                                if self.seen_intents.contains_key(&key) {
                                    continue; // duplicate
                                }
                                self.seen_intents.insert(key, true);
                            }
                            packets.push(pkt);
                        }
                    }
                    Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(_) => break,
                }
            }
        }

        packets
    }


    /// Blind Triple-Transmit (§3.2 v3.0): send 3 rapid bursts (10ms spacing, 30ms window)
    /// to combat 15% UDP packet drop.
    pub fn send_triple(&self, packet: &Packet) -> io::Result<()> {
        for _ in 0..3 {
            self.send(packet)?;
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        Ok(())
    }

    /// Check if an INTENT broadcast is allowed (rate limiter).
    /// Call once per tick per agent — returns true if within limit.
    pub fn can_broadcast_intent(&mut self, agent_id: u8) -> bool {
        let counter = self.rate_limiter.entry(agent_id).or_insert(0);
        if *counter >= INTENT_RATE_LIMIT {
            false
        } else {
            *counter += 1;
            true
        }
    }

    /// Reset rate limiter counters (call every 5 ticks)
    pub fn reset_rate_limits(&mut self) {
        self.rate_limiter.clear();
    }

    /// Garbage-collect old dedup entries
    pub fn gc_dedup(&mut self, max_entries: usize) {
        if self.seen_intents.len() > max_entries {
            // Simple: clear half
            let keys: Vec<_> = self.seen_intents.keys().cloned().collect();
            for key in keys.iter().take(keys.len() / 2) {
                self.seen_intents.remove(key);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_transport() {
        let mut transport = TransportLayer::mock();
        let packets = transport.recv_all();
        assert!(packets.is_empty());
    }

    #[test]
    fn test_rate_limiter() {
        let mut transport = TransportLayer::mock();
        assert!(transport.can_broadcast_intent(1));
        assert!(!transport.can_broadcast_intent(1)); // rate limited
        transport.reset_rate_limits();
        assert!(transport.can_broadcast_intent(1)); // reset
    }
}
