#[cfg(target_os = "linux")]
pub use linux_uring::*;

#[cfg(not(target_os = "linux"))]
pub use mock_uring::*;

#[cfg(target_os = "linux")]
pub mod linux_uring {
    use std::net::UdpSocket;
    use std::os::unix::io::AsRawFd;
    use io_uring::{IoUring, opcode, types};
    use reed_solomon_erasure::galois_8::ReedSolomon;
    use std::io;
    use log::{error, info, warn};

    const RCVBUF_SIZE: usize = 1024 * 1024; // 1MB explicit buffer
    const CQ_SIZE: u32 = 256; // Ring size
    const DATA_SHARDS: usize = 4;
    const PARITY_SHARDS: usize = 2;

    pub struct UringNetwork {
        ring: IoUring,
        socket: UdpSocket,
        rs: ReedSolomon,
    }

    impl UringNetwork {
        /// Initialize `io_uring` with a UDP socket and Reed-Solomon FEC.
        pub fn new(bind_addr: &str, quadrant_id: u8) -> io::Result<Self> {
            let socket = UdpSocket::bind(bind_addr)?;
            
            // Explicit SO_RCVBUF allocation (1MB) to prevent kernel drops under heavy congestion
            let sock_fd = socket.as_raw_fd();
            unsafe {
                let buf_size: i32 = RCVBUF_SIZE as i32;
                let ret = libc::setsockopt(
                    sock_fd,
                    libc::SOL_SOCKET,
                    libc::SO_RCVBUF,
                    &buf_size as *const _ as *const libc::c_void,
                    std::mem::size_of_val(&buf_size) as libc::socklen_t,
                );
                if ret < 0 {
                    return Err(io::Error::last_os_error());
                }
            }

            // Initialize io_uring
            let ring = IoUring::new(CQ_SIZE)?;

            // Reed-Solomon initialization
            let rs = ReedSolomon::new(DATA_SHARDS, PARITY_SHARDS).map_err(|e| {
                io::Error::new(io::ErrorKind::Other, format!("ReedSolomon init failed: {:?}", e))
            })?;

            let mut network = Self { ring, socket, rs };

            // Subscribe to spatial multicast groups by quadrant
            network.subscribe_to_quadrants(quadrant_id)?;

            info!("UringNetwork initialized on {} with RCVBUF 1MB", bind_addr);
            Ok(network)
        }

        /// Spatial Scoping: Multicast groups partitioned by warehouse quadrant.
        fn subscribe_to_quadrants(&self, current_quadrant: u8) -> io::Result<()> {
            let local_addr = "0.0.0.0".parse().unwrap();
            // Subscribe to current quadrant
            let current_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant).parse().unwrap();
            self.socket.join_multicast_v4(&current_mcast, &local_addr)?;

            // Also subscribe to adjacent quadrants (example logic: +/- 1 for 1D mapping)
            if current_quadrant > 1 {
                let prev_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant - 1).parse().unwrap();
                self.socket.join_multicast_v4(&prev_mcast, &local_addr)?;
            }
            let next_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant + 1).parse().unwrap();
            self.socket.join_multicast_v4(&next_mcast, &local_addr)?;

            Ok(())
        }

        /// Encode payload with RS Parity (4 data, 2 parity)
        pub fn encode_payload_fec(&self, payload: &[u8]) -> Result<Vec<Vec<u8>>, String> {
            // Pad payload to multiple of DATA_SHARDS
            let shard_size = (payload.len() + DATA_SHARDS - 1) / DATA_SHARDS;
            let mut shards: Vec<Vec<u8>> = vec![vec![0; shard_size]; DATA_SHARDS + PARITY_SHARDS];
            
            for (i, chunk) in payload.chunks(shard_size).enumerate() {
                shards[i][..chunk.len()].copy_from_slice(chunk);
            }

            self.rs.encode(&mut shards).map_err(|e| format!("Encode error: {:?}", e))?;
            Ok(shards)
        }

        /// Decode payload with RS Parity (handles up to 2 missing shards)
        pub fn decode_payload_fec(&self, mut shards: Vec<Option<Vec<u8>>>) -> Result<Vec<u8>, String> {
            self.rs.reconstruct(&mut shards).map_err(|e| format!("Decode error: {:?}", e))?;
            
            let mut reconstructed = Vec::new();
            for i in 0..DATA_SHARDS {
                if let Some(ref shard) = shards[i] {
                    reconstructed.extend_from_slice(shard);
                }
            }
            Ok(reconstructed)
        }

        /// Polling loop using io_uring
        pub fn poll_incoming(&mut self, rx_buf: &mut [u8]) -> io::Result<usize> {
            let fd = types::Fd(self.socket.as_raw_fd());
            
            // Prepare recvmsg via io_uring
            let recv_e = opcode::Recv::new(fd, rx_buf.as_mut_ptr(), rx_buf.len() as u32)
                .build()
                .user_data(1);

            unsafe {
                self.ring.submission().push(&recv_e).expect("Submission queue full");
            }
            self.ring.submit_and_wait(1)?;

            let mut bytes_read = 0;
            let mut cqe = None;
            for e in self.ring.completion() {
                if e.user_data() == 1 {
                    bytes_read = e.result();
                    cqe = Some(e);
                }
            }

            if let Some(_) = cqe {
                if bytes_read < 0 {
                    return Err(io::Error::from_raw_os_error(-bytes_read));
                }
                Ok(bytes_read as usize)
            } else {
                Ok(0)
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
pub mod mock_uring {
    use std::io;
    use std::net::UdpSocket;
    use log::info;
    use reed_solomon_erasure::galois_8::ReedSolomon;

    const DATA_SHARDS: usize = 4;
    const PARITY_SHARDS: usize = 2;

    pub struct UringNetwork {
        socket: UdpSocket,
        rs: ReedSolomon,
    }

    impl UringNetwork {
        pub fn new(bind_addr: &str, quadrant_id: u8) -> io::Result<Self> {
            let socket = UdpSocket::bind(bind_addr)?;
            let rs = ReedSolomon::new(DATA_SHARDS, PARITY_SHARDS).map_err(|e| {
                io::Error::new(io::ErrorKind::Other, format!("ReedSolomon init failed: {:?}", e))
            })?;
            
            let mut network = Self { socket, rs };
            network.subscribe_to_quadrants(quadrant_id)?;

            info!("Mock UringNetwork initialized on {} (non-Linux OS).", bind_addr);
            Ok(network)
        }

        fn subscribe_to_quadrants(&self, current_quadrant: u8) -> io::Result<()> {
            let local_addr = "0.0.0.0".parse().unwrap();
            let current_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant).parse().unwrap();
            self.socket.join_multicast_v4(&current_mcast, &local_addr)?;

            if current_quadrant > 1 {
                let prev_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant - 1).parse().unwrap();
                self.socket.join_multicast_v4(&prev_mcast, &local_addr)?;
            }
            let next_mcast: std::net::Ipv4Addr = format!("239.255.1.{}", current_quadrant + 1).parse().unwrap();
            self.socket.join_multicast_v4(&next_mcast, &local_addr)?;
            Ok(())
        }

        pub fn encode_payload_fec(&self, payload: &[u8]) -> Result<Vec<Vec<u8>>, String> {
            let shard_size = (payload.len() + DATA_SHARDS - 1) / DATA_SHARDS;
            let mut shards = vec![vec![0; shard_size]; DATA_SHARDS + PARITY_SHARDS];
            for (i, chunk) in payload.chunks(shard_size).enumerate() {
                shards[i][..chunk.len()].copy_from_slice(chunk);
            }
            self.rs.encode(&mut shards).map_err(|e| format!("Encode error: {:?}", e))?;
            Ok(shards)
        }

        pub fn decode_payload_fec(&self, mut shards: Vec<Option<Vec<u8>>>) -> Result<Vec<u8>, String> {
            self.rs.reconstruct(&mut shards).map_err(|e| format!("Decode error: {:?}", e))?;
            let mut reconstructed = Vec::new();
            for i in 0..DATA_SHARDS {
                if let Some(ref shard) = shards[i] {
                    reconstructed.extend_from_slice(shard);
                }
            }
            Ok(reconstructed)
        }

        pub fn poll_incoming(&mut self, rx_buf: &mut [u8]) -> io::Result<usize> {
            self.socket.set_nonblocking(true)?;
            match self.socket.recv(rx_buf) {
                Ok(n) => Ok(n),
                Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
                Err(e) => Err(e),
            }
        }
    }
}
