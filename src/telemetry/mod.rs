use crate::space_time::{CellID, Tick, Heading};
use crate::fsm::AgentState;

/// Telemetry snapshot broadcast to visualization/monitoring tools.
///
/// Sent at 1Hz (every 10 ticks) or on state transition.
/// Kept small for UDP broadcast — no trajectory data, just current status.
#[derive(Debug, Clone)]
pub struct TelemetrySnapshot {
    pub agent_id: u8,
    pub tick: Tick,
    pub state: AgentState,
    pub cell: CellID,
    pub heading: Heading,
    pub x_m: f32,
    pub y_m: f32,
    pub theta_rad: f32,
    pub linear_vel: f32,
    pub angular_vel: f32,
    pub battery_pct: u8,
    pub yield_count: u8,
    pub task_id: Option<u16>,
    pub plan_attempts: u8,
}

impl TelemetrySnapshot {
    /// Serialize to bytes (fixed 32-byte payload)
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(32);
        buf.push(0x10); // msg_type: telemetry
        buf.push(self.agent_id);
        buf.extend_from_slice(&self.tick.to_le_bytes());
        buf.push(self.state.to_u8());
        buf.extend_from_slice(&self.cell.to_le_bytes());
        buf.push(self.heading.to_u8());
        buf.extend_from_slice(&self.x_m.to_le_bytes());
        buf.extend_from_slice(&self.y_m.to_le_bytes());
        buf.extend_from_slice(&self.theta_rad.to_le_bytes());
        buf.push(self.battery_pct);
        buf.push(self.yield_count);
        buf.extend_from_slice(&self.task_id.unwrap_or(0xFFFF).to_le_bytes());
        buf.push(self.plan_attempts);
        // Pad to 32 bytes
        while buf.len() < 32 {
            buf.push(0x00);
        }
        buf
    }

    /// Deserialize from bytes
    pub fn deserialize(data: &[u8]) -> Option<Self> {
        if data.len() < 32 || data[0] != 0x10 {
            return None;
        }
        Some(Self {
            agent_id: data[1],
            tick: u32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            state: AgentState::from_u8(data[6])?,
            cell: u16::from_le_bytes([data[7], data[8]]),
            heading: Heading::from_u8(data[9])?,
            x_m: f32::from_le_bytes([data[10], data[11], data[12], data[13]]),
            y_m: f32::from_le_bytes([data[14], data[15], data[16], data[17]]),
            theta_rad: f32::from_le_bytes([data[18], data[19], data[20], data[21]]),
            battery_pct: data[22],
            yield_count: data[23],
            task_id: {
                let tid = u16::from_le_bytes([data[24], data[25]]);
                if tid == 0xFFFF { None } else { Some(tid) }
            },
            plan_attempts: data[26],
            linear_vel: 0.0,
            angular_vel: 0.0,
        })
    }
}

/// Telemetry broadcaster — sends snapshots at configurable rate.
pub struct TelemetryBroadcaster {
    /// Broadcast interval in ticks (default: 10 = 1Hz)
    interval_ticks: u32,
    /// Last broadcast tick
    last_broadcast_tick: Tick,
    /// Accumulated snapshots for log/playback (optional)
    log: Option<Vec<TelemetrySnapshot>>,
    /// Whether broadcasting is active
    active: bool,
}

impl TelemetryBroadcaster {
    pub fn new(interval_ticks: u32) -> Self {
        Self {
            interval_ticks,
            last_broadcast_tick: 0,
            log: None,
            active: false,
        }
    }

    /// Create with logging enabled (for test/playback)
    pub fn with_logging(interval_ticks: u32) -> Self {
        Self {
            interval_ticks,
            last_broadcast_tick: 0,
            log: Some(Vec::new()),
            active: true,
        }
    }

    pub fn start(&mut self) {
        self.active = true;
    }

    pub fn stop(&mut self) {
        self.active = false;
    }

    /// Check if a broadcast is due at the given tick
    pub fn should_broadcast(&self, current_tick: Tick) -> bool {
        self.active && (current_tick >= self.last_broadcast_tick + self.interval_ticks)
    }

    /// Record a snapshot (and serialize for network send).
    /// Returns serialized bytes if broadcast was sent.
    pub fn broadcast(&mut self, snapshot: TelemetrySnapshot) -> Option<Vec<u8>> {
        if !self.active {
            return None;
        }

        let tick = snapshot.tick;
        let bytes = snapshot.serialize();

        if let Some(ref mut log) = self.log {
            log.push(snapshot);
        }

        self.last_broadcast_tick = tick;
        Some(bytes)
    }

    /// Force broadcast on state transition (regardless of interval)
    pub fn broadcast_state_change(&mut self, snapshot: TelemetrySnapshot) -> Option<Vec<u8>> {
        self.broadcast(snapshot)
    }

    /// Get logged snapshots (for test assertions)
    pub fn get_log(&self) -> Option<&[TelemetrySnapshot]> {
        self.log.as_deref()
    }

    pub fn clear_log(&mut self) {
        if let Some(ref mut log) = self.log {
            log.clear();
        }
    }
}

/// Fleet-level telemetry aggregator (for visualization dashboard).
///
/// Collects snapshots from all agents and provides fleet summary metrics.
pub struct FleetTelemetry {
    /// Latest snapshot per agent
    latest: std::collections::HashMap<u8, TelemetrySnapshot>,
}

impl FleetTelemetry {
    pub fn new() -> Self {
        Self {
            latest: std::collections::HashMap::new(),
        }
    }

    /// Update with a received snapshot
    pub fn update(&mut self, snapshot: TelemetrySnapshot) {
        self.latest.insert(snapshot.agent_id, snapshot);
    }

    /// Get latest snapshot for an agent
    pub fn get(&self, agent_id: u8) -> Option<&TelemetrySnapshot> {
        self.latest.get(&agent_id)
    }

    /// Get all latest snapshots
    pub fn all(&self) -> Vec<&TelemetrySnapshot> {
        self.latest.values().collect()
    }

    /// Count agents by state
    pub fn state_counts(&self) -> std::collections::HashMap<u8, usize> {
        let mut counts = std::collections::HashMap::new();
        for snap in self.latest.values() {
            *counts.entry(snap.state.to_u8()).or_insert(0) += 1;
        }
        counts
    }

    /// Average battery percentage across fleet
    pub fn avg_battery(&self) -> f32 {
        if self.latest.is_empty() {
            return 100.0;
        }
        let sum: u32 = self.latest.values().map(|s| s.battery_pct as u32).sum();
        sum as f32 / self.latest.len() as f32
    }

    /// Count agents currently in conflict (yielding states)
    pub fn yielding_count(&self) -> usize {
        self.latest.values().filter(|s| {
            matches!(s.state,
                AgentState::Yielding | AgentState::YieldingPullover | AgentState::YieldingParked
            )
        }).count()
    }

    /// Prune agents not heard from in N ticks
    pub fn prune_stale(&mut self, current_tick: Tick, max_age_ticks: u32) {
        self.latest.retain(|_id, snap| {
            current_tick.saturating_sub(snap.tick) < max_age_ticks
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_snapshot(agent_id: u8, tick: Tick, state: AgentState) -> TelemetrySnapshot {
        TelemetrySnapshot {
            agent_id,
            tick,
            state,
            cell: 42,
            heading: Heading::N,
            x_m: 1.25,
            y_m: 2.75,
            theta_rad: std::f32::consts::FRAC_PI_2,
            linear_vel: 0.5,
            angular_vel: 0.1,
            battery_pct: 85,
            yield_count: 2,
            task_id: Some(7),
            plan_attempts: 1,
        }
    }

    #[test]
    fn test_snapshot_roundtrip() {
        let snap = make_snapshot(3, 1000, AgentState::Executing);
        let bytes = snap.serialize();
        assert_eq!(bytes.len(), 32);
        let recover = TelemetrySnapshot::deserialize(&bytes).unwrap();
        assert_eq!(recover.agent_id, 3);
        assert_eq!(recover.tick, 1000);
        assert_eq!(recover.cell, 42);
        assert_eq!(recover.battery_pct, 85);
        assert_eq!(recover.task_id, Some(7));
    }

    #[test]
    fn test_snapshot_no_task() {
        let mut snap = make_snapshot(1, 500, AgentState::Idle);
        snap.task_id = None;
        let bytes = snap.serialize();
        let recover = TelemetrySnapshot::deserialize(&bytes).unwrap();
        assert_eq!(recover.task_id, None);
    }

    #[test]
    fn test_broadcaster_interval() {
        let broadcaster = TelemetryBroadcaster::with_logging(10);
        assert!(broadcaster.should_broadcast(0));
        assert!(broadcaster.should_broadcast(10));
    }

    #[test]
    fn test_broadcaster_log() {
        let mut broadcaster = TelemetryBroadcaster::with_logging(10);
        let snap = make_snapshot(1, 10, AgentState::Idle);
        broadcaster.broadcast(snap);
        assert_eq!(broadcaster.get_log().unwrap().len(), 1);
    }

    #[test]
    fn test_fleet_telemetry() {
        let mut fleet = FleetTelemetry::new();
        fleet.update(make_snapshot(1, 100, AgentState::Executing));
        fleet.update(make_snapshot(2, 100, AgentState::Yielding));
        fleet.update(make_snapshot(3, 100, AgentState::Idle));

        assert_eq!(fleet.all().len(), 3);
        assert_eq!(fleet.yielding_count(), 1);
    }

    #[test]
    fn test_fleet_prune() {
        let mut fleet = FleetTelemetry::new();
        fleet.update(make_snapshot(1, 10, AgentState::Idle));
        fleet.update(make_snapshot(2, 90, AgentState::Executing));

        fleet.prune_stale(100, 50);
        assert!(fleet.get(1).is_none());    // stale
        assert!(fleet.get(2).is_some());     // recent
    }
}
