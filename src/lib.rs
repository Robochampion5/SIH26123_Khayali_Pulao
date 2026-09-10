//! Decentralized AMR Coordination & Collision-Avoidance Framework
//!
//! SIH26123 — Edge-native, serverless fleet coordination for Autonomous Mobile Robots
//! in dynamic warehouse environments. Runs on Raspberry Pi 4 / Jetson Nano.
//!
//! Core components:
//! - `space_time`: CellID, Tick, SafeInterval, ReservationTable, Priority function
//! - `planner`: D-SIPP algorithm, SpatialGraph, kinematic heading constraints
//! - `fsm`: 9-state FSM (Idle, Planning, Executing, Yielding*, Replanning, Blocked, Charging)
//! - `network`: UDP multicast transport, 1-Round CNP, wire payload serialization
//! - `sensor`: Odometry, LiDAR, Localizer (Fused/DeadReckoning/Lost modes)
//! - `hal`: PID control, differential-drive kinematics, motor driver abstraction
//! - `telemetry`: 1Hz broadcast, fleet aggregation

pub mod space_time;
pub mod planner;
pub mod fsm;
pub mod network;
pub mod sensor;
pub mod hal;
pub mod telemetry;

/// Re-exports for convenience
pub use space_time::{CellID, Tick, Heading, SafeInterval, Q8_8, Q16_16};
pub use space_time::reservation::{ReservationTable};
pub use planner::{Trajectory, Waypoint, SpatialGraph};
pub use fsm::{AgentState, AgentContext};
pub use network::{TransportLayer, CnpManager, CnpBidder};
pub use sensor::{Pose, LidarScan, OdometryReading, Localizer};
pub use hal::{PidController, DiffDriveKinematics, WheelVelocity, MotorDriver, MotorCommand};
pub use telemetry::{TelemetrySnapshot, TelemetryBroadcaster, FleetTelemetry};