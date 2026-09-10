//! AMR Agent Entry Point — Minimal working build
//!
//! Initializes tokio runtime, loads config, starts the 10Hz loop.

use amr_coordination::{
    fsm::{AgentContext, AgentState},
    space_time::{CellID, Tick, Heading},
    telemetry::{TelemetryBroadcaster, TelemetrySnapshot},
    sensor::{Localizer, LidarScan, OdometryReading},
    planner::{Trajectory, Waypoint},
    hal::{DiffDriveKinematics, MotorDriver, MotorCommand, WheelVelocity},
};
use amr_coordination::ReservationTable;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::Mutex;
use tokio::time::interval;
use tracing::{info, error};

#[derive(Debug, Clone)]
struct Config {
    agent_id: u8,
    tick_rate_hz: u32,
}

impl Default for Config {
    fn default() -> Self { Self { agent_id: 1, tick_rate_hz: 10 } }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt().init();
    info!("AMR Agent starting — agent {}", 1);

    // Minimal init to prove compilation
    let ctx = AgentContext::new(1, 0, 999);
    let _table = ReservationTable::new();
    let _kinematics = DiffDriveKinematics::default_amr();
    let _telemetry = TelemetryBroadcaster::new(10);
    let _localizer = Localizer::new(0, Heading::N, 32, 32, 0.5);
    let _snap = TelemetrySnapshot {
        agent_id: 1,
        tick: 100,
        state: AgentState::Idle,
        cell: 42,
        heading: Heading::N,
        x_m: 1.0, y_m: 2.0, theta_rad: 0.0,
        linear_vel: 0.0, angular_vel: 0.0,
        battery_pct: 100, yield_count: 0,
        task_id: None, plan_attempts: 0,
    };

    // Start the 10Hz control loop (simplified — real loop uses AgentRuntime)
    let mut ticker = interval(Duration::from_millis(100));
    let running = Arc::new(Mutex::new(true));

    loop {
        ticker.tick().await;
        if !*running.lock().await { break; }
        info!("Tick — state: {}", AgentState::Idle);
    }

    info!("AMR Agent shutdown complete");
    Ok(())
}
