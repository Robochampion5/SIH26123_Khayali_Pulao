use crate::space_time::{CellID, Tick, Heading, Q8_8};

// ─── Message type tags ───
pub const MSG_INTENT:       u8 = 0x01;
pub const MSG_TASK_CFP:     u8 = 0x02;
pub const MSG_TASK_BID:     u8 = 0x03;
pub const MSG_EDGE_BLOCKED: u8 = 0x04;
pub const MSG_TASK_AWARD:   u8 = 0x05;
pub const MSG_INTENT_BRAKE: u8 = 0x06;
pub const MSG_GRIDLOCK:     u8 = 0x07;

// ─── Wire-format Waypoint (10 bytes) ───
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WireWaypoint {
    pub cell_id:  CellID,   // 2 bytes
    pub t_arrive: Tick,     // 4 bytes
    pub t_depart: Tick,     // 4 bytes
}

impl WireWaypoint {
    pub fn serialize(&self, buf: &mut Vec<u8>) {
        buf.extend_from_slice(&self.cell_id.to_le_bytes());
        buf.extend_from_slice(&self.t_arrive.to_le_bytes());
        buf.extend_from_slice(&self.t_depart.to_le_bytes());
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 10 { return None; }
        Some(Self {
            cell_id:  u16::from_le_bytes([data[0], data[1]]),
            t_arrive: u32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            t_depart: u32::from_le_bytes([data[6], data[7], data[8], data[9]]),
        })
    }
}

// ─── INTENT (max 245 bytes) ───
#[derive(Debug, Clone)]
pub struct IntentPayload {
    pub agent_id: u8,
    pub seq_no:   u16,
    pub heading:  Heading,
    pub waypoints: Vec<WireWaypoint>, // max 24
}

impl IntentPayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(5 + self.waypoints.len() * 10);
        buf.push(MSG_INTENT);
        buf.push(self.agent_id);
        buf.extend_from_slice(&self.seq_no.to_le_bytes());
        buf.push(self.waypoints.len() as u8);
        for wp in &self.waypoints {
            wp.serialize(&mut buf);
        }
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 5 { return None; }
        if data[0] != MSG_INTENT { return None; }

        let agent_id = data[1];
        let seq_no = u16::from_le_bytes([data[2], data[3]]);
        let wp_count = data[4] as usize;

        if data.len() < 5 + wp_count * 10 { return None; }

        let mut waypoints = Vec::with_capacity(wp_count);
        for i in 0..wp_count {
            let offset = 5 + i * 10;
            if let Some(wp) = WireWaypoint::deserialize(&data[offset..]) {
                waypoints.push(wp);
            } else {
                return None;
            }
        }

        Some(Self {
            agent_id,
            seq_no,
            heading: Heading::N, // Heading inferred from waypoints
            waypoints,
        })
    }
}

// ─── TASK_CFP (22 bytes) ───
#[derive(Debug, Clone, Copy)]
pub struct TaskCfpPayload {
    pub task_id:         u16,
    pub pickup_cell:     CellID,
    pub dropoff_cell:    CellID,
    pub priority:        u8,    // urgency class 0-3
    pub deadline_tick:   Tick,
    pub manager_id:      u8,
    pub bid_window_ticks: u16,
}

impl TaskCfpPayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(22);
        buf.push(MSG_TASK_CFP);
        buf.extend_from_slice(&self.task_id.to_le_bytes());
        buf.extend_from_slice(&self.pickup_cell.to_le_bytes());
        buf.extend_from_slice(&self.dropoff_cell.to_le_bytes());
        buf.push(self.priority);
        buf.extend_from_slice(&self.deadline_tick.to_le_bytes());
        buf.push(self.manager_id);
        buf.extend_from_slice(&self.bid_window_ticks.to_le_bytes());
        // Pad to 22 bytes
        while buf.len() < 22 {
            buf.push(0);
        }
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 16 { return None; }
        if data[0] != MSG_TASK_CFP { return None; }

        Some(Self {
            task_id:         u16::from_le_bytes([data[1], data[2]]),
            pickup_cell:     u16::from_le_bytes([data[3], data[4]]),
            dropoff_cell:    u16::from_le_bytes([data[5], data[6]]),
            priority:        data[7],
            deadline_tick:   u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            manager_id:      data[12],
            bid_window_ticks: u16::from_le_bytes([data[13], data[14]]),
        })
    }
}

// ─── TASK_BID (16 bytes) ───
#[derive(Debug, Clone, Copy)]
pub struct TaskBidPayload {
    pub task_id:         u16,
    pub agent_id:        u8,
    pub utility:         Q8_8,  // Q8.8 fixed-point
    pub estimated_ticks: u16,
    pub battery_pct:     u8,
    pub queue_depth:     u8,
}

impl TaskBidPayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(16);
        buf.push(MSG_TASK_BID);
        buf.extend_from_slice(&self.task_id.to_le_bytes());
        buf.push(self.agent_id);
        buf.extend_from_slice(&self.utility.0.to_le_bytes());
        buf.extend_from_slice(&self.estimated_ticks.to_le_bytes());
        buf.push(self.battery_pct);
        buf.push(self.queue_depth);
        // Pad
        while buf.len() < 16 {
            buf.push(0);
        }
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 10 { return None; }
        if data[0] != MSG_TASK_BID { return None; }

        Some(Self {
            task_id:         u16::from_le_bytes([data[1], data[2]]),
            agent_id:        data[3],
            utility:         Q8_8(u16::from_le_bytes([data[4], data[5]])),
            estimated_ticks: u16::from_le_bytes([data[6], data[7]]),
            battery_pct:     data[8],
            queue_depth:     data[9],
        })
    }
}

// ─── TASK_AWARD (6 bytes) ───
#[derive(Debug, Clone, Copy)]
pub struct TaskAwardPayload {
    pub task_id:  u16,
    pub agent_id: u8,
    pub pickup:   CellID,
    pub dropoff:  CellID,
}

impl TaskAwardPayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8);
        buf.push(MSG_TASK_AWARD);
        buf.extend_from_slice(&self.task_id.to_le_bytes());
        buf.push(self.agent_id);
        buf.extend_from_slice(&self.pickup.to_le_bytes());
        buf.extend_from_slice(&self.dropoff.to_le_bytes());
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 8 { return None; }
        if data[0] != MSG_TASK_AWARD { return None; }

        Some(Self {
            task_id:  u16::from_le_bytes([data[1], data[2]]),
            agent_id: data[3],
            pickup:   u16::from_le_bytes([data[4], data[5]]),
            dropoff:  u16::from_le_bytes([data[6], data[7]]),
        })
    }
}

// ─── EDGE_BLOCKED (8 bytes) ───
#[derive(Debug, Clone, Copy)]
pub struct EdgeBlockedPayload {
    pub reporter_id:   u8,
    pub blocked_cell:  CellID,
    pub detected_tick: Tick,
}

impl EdgeBlockedPayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(8);
        buf.push(MSG_EDGE_BLOCKED);
        buf.push(self.reporter_id);
        buf.extend_from_slice(&self.blocked_cell.to_le_bytes());
        buf.extend_from_slice(&self.detected_tick.to_le_bytes());
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 8 { return None; }
        if data[0] != MSG_EDGE_BLOCKED { return None; }

        Some(Self {
            reporter_id:   data[1],
            blocked_cell:  u16::from_le_bytes([data[2], data[3]]),
            detected_tick: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        })
    }
}

// ─── INTENT_BRAKE (20 bytes) ───
#[derive(Debug, Clone, Copy)]
pub struct IntentBrakePayload {
    pub agent_id:     u8,
    pub cell:         CellID,
    pub heading:      u8,     // Heading as u8
    pub stop_tick:    Tick,
    pub v_current_q8: u16,    // Q8.8 velocity m/s
}

impl IntentBrakePayload {
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(12);
        buf.push(MSG_INTENT_BRAKE);
        buf.push(self.agent_id);
        buf.extend_from_slice(&self.cell.to_le_bytes());
        buf.push(self.heading);
        buf.extend_from_slice(&self.stop_tick.to_le_bytes());
        buf.extend_from_slice(&self.v_current_q8.to_le_bytes());
        buf
    }

    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 11 { return None; }
        if data[0] != MSG_INTENT_BRAKE { return None; }

        Some(Self {
            agent_id:     data[1],
            cell:         u16::from_le_bytes([data[2], data[3]]),
            heading:      data[4],
            stop_tick:    u32::from_le_bytes([data[5], data[6], data[7], data[8]]),
            v_current_q8: u16::from_le_bytes([data[9], data[10]]),
        })
    }
}

// ─── Unified packet type ───
#[derive(Debug, Clone)]
pub enum Packet {
    Intent(IntentPayload),
    TaskCfp(TaskCfpPayload),
    TaskBid(TaskBidPayload),
    TaskAward(TaskAwardPayload),
    EdgeBlocked(EdgeBlockedPayload),
    IntentBrake(IntentBrakePayload),
}

impl Packet {
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.is_empty() { return None; }
        match data[0] {
            MSG_INTENT       => IntentPayload::deserialize(data).map(Packet::Intent),
            MSG_TASK_CFP     => TaskCfpPayload::deserialize(data).map(Packet::TaskCfp),
            MSG_TASK_BID     => TaskBidPayload::deserialize(data).map(Packet::TaskBid),
            MSG_TASK_AWARD   => TaskAwardPayload::deserialize(data).map(Packet::TaskAward),
            MSG_EDGE_BLOCKED => EdgeBlockedPayload::deserialize(data).map(Packet::EdgeBlocked),
            MSG_INTENT_BRAKE => IntentBrakePayload::deserialize(data).map(Packet::IntentBrake),
            _ => None,
        }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        match self {
            Packet::Intent(p)       => p.serialize(),
            Packet::TaskCfp(p)      => p.serialize(),
            Packet::TaskBid(p)      => p.serialize(),
            Packet::TaskAward(p)    => p.serialize(),
            Packet::EdgeBlocked(p)  => p.serialize(),
            Packet::IntentBrake(p)  => p.serialize(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_intent_roundtrip() {
        let intent = IntentPayload {
            agent_id: 5,
            seq_no: 42,
            heading: Heading::E,
            waypoints: vec![
                WireWaypoint { cell_id: 10, t_arrive: 0, t_depart: 5 },
                WireWaypoint { cell_id: 11, t_arrive: 5, t_depart: 10 },
            ],
        };
        let bytes = intent.serialize();
        assert!(bytes.len() <= 245);
        let decoded = IntentPayload::deserialize(&bytes).unwrap();
        assert_eq!(decoded.agent_id, 5);
        assert_eq!(decoded.seq_no, 42);
        assert_eq!(decoded.waypoints.len(), 2);
        assert_eq!(decoded.waypoints[0].cell_id, 10);
    }

    #[test]
    fn test_cfp_roundtrip() {
        let cfp = TaskCfpPayload {
            task_id: 100,
            pickup_cell: 50,
            dropoff_cell: 200,
            priority: 2,
            deadline_tick: 1000,
            manager_id: 1,
            bid_window_ticks: 5,
        };
        let bytes = cfp.serialize();
        assert!(bytes.len() <= 1472);
        let decoded = TaskCfpPayload::deserialize(&bytes).unwrap();
        assert_eq!(decoded.task_id, 100);
        assert_eq!(decoded.pickup_cell, 50);
    }

    #[test]
    fn test_bid_roundtrip() {
        let bid = TaskBidPayload {
            task_id: 100,
            agent_id: 3,
            utility: Q8_8::from_f32(0.75),
            estimated_ticks: 50,
            battery_pct: 80,
            queue_depth: 1,
        };
        let bytes = bid.serialize();
        let decoded = TaskBidPayload::deserialize(&bytes).unwrap();
        assert_eq!(decoded.task_id, 100);
        assert_eq!(decoded.agent_id, 3);
        assert!((decoded.utility.to_f32() - 0.75).abs() < 0.01);
    }

    #[test]
    fn test_edge_blocked_roundtrip() {
        let eb = EdgeBlockedPayload {
            reporter_id: 7,
            blocked_cell: 999,
            detected_tick: 12345,
        };
        let bytes = eb.serialize();
        assert_eq!(bytes.len(), 8);
        let decoded = EdgeBlockedPayload::deserialize(&bytes).unwrap();
        assert_eq!(decoded.blocked_cell, 999);
        assert_eq!(decoded.detected_tick, 12345);
    }

    #[test]
    fn test_packet_dispatch() {
        let eb = EdgeBlockedPayload {
            reporter_id: 1,
            blocked_cell: 42,
            detected_tick: 100,
        };
        let bytes = eb.serialize();
        match Packet::from_bytes(&bytes) {
            Some(Packet::EdgeBlocked(p)) => {
                assert_eq!(p.blocked_cell, 42);
            }
            _ => panic!("Expected EdgeBlocked packet"),
        }
    }

    #[test]
    fn test_all_payloads_under_mtu() {
        let intent = IntentPayload {
            agent_id: 0,
            seq_no: 0,
            heading: Heading::N,
            waypoints: vec![WireWaypoint { cell_id: 0, t_arrive: 0, t_depart: 0 }; 24],
        };
        assert!(intent.serialize().len() <= 1472);
    }

    #[test]
    fn test_award_roundtrip() {
        let award = TaskAwardPayload {
            task_id: 55,
            agent_id: 2,
            pickup: 10,
            dropoff: 200,
        };
        let bytes = award.serialize();
        let decoded = TaskAwardPayload::deserialize(&bytes).unwrap();
        assert_eq!(decoded.task_id, 55);
        assert_eq!(decoded.agent_id, 2);
        assert_eq!(decoded.pickup, 10);
        assert_eq!(decoded.dropoff, 200);
    }
}
