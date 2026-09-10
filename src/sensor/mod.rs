use crate::space_time::{CellID, Tick, Heading, Q16_16};
use std::collections::HashMap;

pub mod lidar;
pub mod odometry;

/// Maximum lidar range in cells
const MAX_LIDAR_RANGE_CELLS: u16 = 20;

/// Localization confidence threshold (0.0–1.0)
const LOCALIZATION_CONFIDENCE_THRESHOLD: f32 = 0.7;

/// Max dead-reckoning ticks without lidar correction before entering DEGRADED mode
const MAX_DEAD_RECKONING_TICKS: u16 = 3;

// ─── Pose: continuous position and heading ───

/// 2D pose in continuous coordinates (Q16.16 fixed-point meters)
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: Q16_16,       // meters from grid origin
    pub y: Q16_16,       // meters from grid origin
    pub theta: Q16_16,   // radians, [0, 2π)
}

impl Pose {
    pub fn from_cell(cell: CellID, heading: Heading, grid_width: u16, cell_size_m: f32) -> Self {
        let col = (cell % grid_width) as f32;
        let row = (cell / grid_width) as f32;
        Self {
            x: Q16_16::from_f32(col * cell_size_m + cell_size_m / 2.0),
            y: Q16_16::from_f32(row * cell_size_m + cell_size_m / 2.0),
            theta: Q16_16::from_f32(heading.to_rad()),
        }
    }

    /// Snap continuous pose to nearest discrete cell
    pub fn to_cell(&self, grid_width: u16, cell_size_m: f32) -> CellID {
        let col = (self.x.to_f32() / cell_size_m).floor().max(0.0) as u16;
        let row = (self.y.to_f32() / cell_size_m).floor().max(0.0) as u16;
        row * grid_width + col
    }

    /// Snap continuous heading to nearest discrete heading (Θ_4 quantization)
    pub fn to_heading(&self) -> Heading {
        let theta = self.theta.to_f32();
        // Normalize to [0, 2π)
        let theta = theta.rem_euclid(std::f32::consts::TAU);
        // Quantize to nearest 90° (π/4 boundaries)
        if theta < std::f32::consts::FRAC_PI_4 || theta >= 7.0 * std::f32::consts::FRAC_PI_4 {
            Heading::E
        } else if theta < 3.0 * std::f32::consts::FRAC_PI_4 {
            Heading::N
        } else if theta < 5.0 * std::f32::consts::FRAC_PI_4 {
            Heading::W
        } else {
            Heading::S
        }
    }
}

// ─── Sensor readings ───

/// Raw lidar scan (polar: angle_rad, distance_m, per ray)
#[derive(Debug, Clone)]
pub struct LidarScan {
    pub ranges: Vec<(f32, f32)>,  // (angle_rad, distance_m)
    pub timestamp_tick: Tick,
}

impl LidarScan {
    pub fn empty(tick: Tick) -> Self {
        Self { ranges: Vec::new(), timestamp_tick: tick }
    }

    /// Detect cells that appear blocked (have a lidar return within cell bounds)
    pub fn detect_blocked_cells(
        &self,
        pose: &Pose,
        grid_width: u16,
        grid_height: u16,
        cell_size_m: f32,
    ) -> Vec<CellID> {
        let mut blocked = Vec::new();
        let px = pose.x.to_f32();
        let py = pose.y.to_f32();

        for &(angle, dist) in &self.ranges {
            if dist <= 0.0 || dist > MAX_LIDAR_RANGE_CELLS as f32 * cell_size_m {
                continue;
            }
            let abs_angle = pose.theta.to_f32() + angle;
            let hit_x = px + dist * abs_angle.cos();
            let hit_y = py + dist * abs_angle.sin();

            let col = (hit_x / cell_size_m).floor() as i32;
            let row = (hit_y / cell_size_m).floor() as i32;

            if col >= 0 && col < grid_width as i32 && row >= 0 && row < grid_height as i32 {
                let cell = (row as u16) * grid_width + (col as u16);
                if !blocked.contains(&cell) {
                    blocked.push(cell);
                }
            }
        }
        blocked
    }
}

/// Odometry reading (wheel encoder deltas)
#[derive(Debug, Clone, Copy)]
pub struct OdometryReading {
    pub delta_left_m: f32,   // left wheel distance since last reading
    pub delta_right_m: f32,  // right wheel distance since last reading
    pub wheel_base_m: f32,   // distance between wheels
    pub timestamp_tick: Tick,
}

impl OdometryReading {
    /// Compute pose delta from differential drive odometry
    pub fn to_pose_delta(&self) -> (f32, f32, f32) {
        let d = (self.delta_left_m + self.delta_right_m) / 2.0;
        let dtheta = (self.delta_right_m - self.delta_left_m) / self.wheel_base_m;
        let dx = d * (dtheta / 2.0).cos(); // midpoint integration
        let dy = d * (dtheta / 2.0).sin();
        (dx, dy, dtheta)
    }
}

// ─── Localizer: fuses odometry + lidar for cell localization ───

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalizationMode {
    /// Lidar-corrected: high confidence
    Fused,
    /// Dead-reckoning only: degrading confidence
    DeadReckoning,
    /// No sensor data: lost
    Lost,
}

pub struct Localizer {
    /// Current estimated pose
    pub pose: Pose,
    /// Current mode
    pub mode: LocalizationMode,
    /// Ticks since last lidar correction
    dead_reckoning_ticks: u16,
    /// Grid dimensions for cell snapping
    grid_width: u16,
    grid_height: u16,
    cell_size_m: f32,
}

impl Localizer {
    pub fn new(
        initial_cell: CellID,
        initial_heading: Heading,
        grid_width: u16,
        grid_height: u16,
        cell_size_m: f32,
    ) -> Self {
        Self {
            pose: Pose::from_cell(initial_cell, initial_heading, grid_width, cell_size_m),
            mode: LocalizationMode::Fused,
            dead_reckoning_ticks: 0,
            grid_width,
            grid_height,
            cell_size_m,
        }
    }

    /// Update pose from odometry (dead-reckoning step)
    pub fn update_odometry(&mut self, odom: &OdometryReading) {
        let (dx, dy, dtheta) = odom.to_pose_delta();
        let cos_t = self.pose.theta.to_f32().cos();
        let sin_t = self.pose.theta.to_f32().sin();

        // Transform from robot frame to world frame
        let world_dx = dx * cos_t - dy * sin_t;
        let world_dy = dx * sin_t + dy * cos_t;

        self.pose.x = Q16_16::from_f32(self.pose.x.to_f32() + world_dx);
        self.pose.y = Q16_16::from_f32(self.pose.y.to_f32() + world_dy);
        self.pose.theta = Q16_16::from_f32(
            (self.pose.theta.to_f32() + dtheta).rem_euclid(std::f32::consts::TAU)
        );

        self.dead_reckoning_ticks += 1;
        if self.dead_reckoning_ticks > MAX_DEAD_RECKONING_TICKS {
            self.mode = LocalizationMode::DeadReckoning;
        }
    }

    /// Correct pose from lidar (simple cell-snap correction)
    /// In production, this would use scan matching or particle filter
    pub fn correct_from_lidar(&mut self, _scan: &LidarScan) {
        // Simplified: snap to cell center if within confidence threshold
        // A real implementation would do ICP or scan matching
        let cell = self.current_cell();
        let col = cell % self.grid_width;
        let row = cell / self.grid_width;

        // Soft-correct toward cell center (70% weight to lidar, 30% to odom)
        let center_x = (col as f32 + 0.5) * self.cell_size_m;
        let center_y = (row as f32 + 0.5) * self.cell_size_m;

        let alpha = 0.3; // correction strength
        self.pose.x = Q16_16::from_f32(
            self.pose.x.to_f32() * (1.0 - alpha) + center_x * alpha
        );
        self.pose.y = Q16_16::from_f32(
            self.pose.y.to_f32() * (1.0 - alpha) + center_y * alpha
        );

        self.dead_reckoning_ticks = 0;
        self.mode = LocalizationMode::Fused;
    }

    /// Get current discrete cell
    pub fn current_cell(&self) -> CellID {
        self.pose.to_cell(self.grid_width, self.cell_size_m)
    }

    /// Get current discrete heading
    pub fn current_heading(&self) -> Heading {
        self.pose.to_heading()
    }

    /// Reset localizer to a known position (e.g., after manual placement)
    pub fn reset(&mut self, cell: CellID, heading: Heading) {
        self.pose = Pose::from_cell(cell, heading, self.grid_width, self.cell_size_m);
        self.dead_reckoning_ticks = 0;
        self.mode = LocalizationMode::Fused;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pose_cell_roundtrip() {
        let cell: CellID = 12; // col=2, row=1 in 10-wide grid
        let pose = Pose::from_cell(cell, Heading::N, 10, 0.5);
        let recovered = pose.to_cell(10, 0.5);
        assert_eq!(recovered, cell);
    }

    #[test]
    fn test_heading_quantization() {
        // North = π/2
        let pose = Pose {
            x: Q16_16::from_f32(0.0),
            y: Q16_16::from_f32(0.0),
            theta: Q16_16::from_f32(std::f32::consts::FRAC_PI_2),
        };
        assert_eq!(pose.to_heading(), Heading::N);

        // East = 0
        let pose_e = Pose {
            x: Q16_16::from_f32(0.0),
            y: Q16_16::from_f32(0.0),
            theta: Q16_16::from_f32(0.0),
        };
        assert_eq!(pose_e.to_heading(), Heading::E);
    }

    #[test]
    fn test_odometry_straight() {
        let odom = OdometryReading {
            delta_left_m: 0.1,
            delta_right_m: 0.1,
            wheel_base_m: 0.3,
            timestamp_tick: 1,
        };
        let (dx, dy, dtheta) = odom.to_pose_delta();
        assert!((dx - 0.1).abs() < 0.001);
        assert!(dy.abs() < 0.001);
        assert!(dtheta.abs() < 0.001);
    }

    #[test]
    fn test_odometry_turn() {
        let odom = OdometryReading {
            delta_left_m: 0.0,
            delta_right_m: 0.1,
            wheel_base_m: 0.3,
            timestamp_tick: 1,
        };
        let (_dx, _dy, dtheta) = odom.to_pose_delta();
        // Right wheel moves more → turns left (positive dtheta)
        assert!(dtheta > 0.0);
    }

    #[test]
    fn test_localizer_dead_reckoning_mode() {
        let mut loc = Localizer::new(5, Heading::N, 10, 10, 0.5);
        assert_eq!(loc.mode, LocalizationMode::Fused);

        let odom = OdometryReading {
            delta_left_m: 0.01,
            delta_right_m: 0.01,
            wheel_base_m: 0.3,
            timestamp_tick: 1,
        };

        // After MAX_DEAD_RECKONING_TICKS+1 updates without lidar, should be DeadReckoning
        for _ in 0..=MAX_DEAD_RECKONING_TICKS {
            loc.update_odometry(&odom);
        }
        assert_eq!(loc.mode, LocalizationMode::DeadReckoning);

        // Lidar correction restores Fused mode
        loc.correct_from_lidar(&LidarScan::empty(10));
        assert_eq!(loc.mode, LocalizationMode::Fused);
    }

    #[test]
    fn test_lidar_blocked_detection() {
        let pose = Pose {
            x: Q16_16::from_f32(1.25),  // center of cell (2,0) in 10-wide, 0.5m cells
            y: Q16_16::from_f32(0.25),
            theta: Q16_16::from_f32(0.0),  // facing East
        };

        let scan = LidarScan {
            ranges: vec![
                (0.0, 0.5), // straight ahead → cell (3,0)
            ],
            timestamp_tick: 1,
        };

        let blocked = scan.detect_blocked_cells(&pose, 10, 10, 0.5);
        assert!(!blocked.is_empty());
        // Hit at (1.25 + 0.5, 0.25) = (1.75, 0.25) → cell col=3, row=0 → cell 3
        assert!(blocked.contains(&3));
    }
}
