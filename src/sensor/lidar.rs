use crate::space_time::{CellID, Tick, Heading};
use crate::sensor::LidarScan;

/// Lidar driver abstraction.
///
/// In production, this would interface with rplidar or similar via serial.
/// For simulation, returns mock scans.
pub struct LidarDriver {
    /// Whether the lidar is active
    active: bool,
    /// Max range in meters
    max_range_m: f32,
    /// Number of rays per scan
    num_rays: usize,
}

impl LidarDriver {
    pub fn new(max_range_m: f32, num_rays: usize) -> Self {
        Self {
            active: false,
            max_range_m,
            num_rays,
        }
    }

    /// Create a mock driver for testing
    pub fn mock() -> Self {
        Self {
            active: true,
            max_range_m: 10.0,
            num_rays: 360,
        }
    }

    pub fn start(&mut self) -> Result<(), &'static str> {
        self.active = true;
        Ok(())
    }

    pub fn stop(&mut self) {
        self.active = false;
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Read a scan. In production, blocks until scan complete (~100ms for 10Hz lidar).
    /// Returns empty scan if not active.
    pub fn read_scan(&self, current_tick: Tick) -> LidarScan {
        if !self.active {
            return LidarScan::empty(current_tick);
        }

        // In production: read from serial/USB
        // For now: return empty scan (test code injects mock data)
        LidarScan::empty(current_tick)
    }
}

/// Mock lidar that produces configurable scans for testing
pub struct MockLidar {
    /// Obstacles injected for testing: (relative_angle_rad, distance_m)
    obstacles: Vec<(f32, f32)>,
}

impl MockLidar {
    pub fn new() -> Self {
        Self {
            obstacles: Vec::new(),
        }
    }

    pub fn add_obstacle(&mut self, angle_rad: f32, distance_m: f32) {
        self.obstacles.push((angle_rad, distance_m));
    }

    pub fn clear(&mut self) {
        self.obstacles.clear();
    }

    pub fn scan(&self, tick: Tick) -> LidarScan {
        LidarScan {
            ranges: self.obstacles.clone(),
            timestamp_tick: tick,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_lidar() {
        let mut lidar = MockLidar::new();
        lidar.add_obstacle(0.0, 1.5);
        lidar.add_obstacle(std::f32::consts::FRAC_PI_2, 2.0);

        let scan = lidar.scan(10);
        assert_eq!(scan.ranges.len(), 2);
        assert_eq!(scan.timestamp_tick, 10);
    }

    #[test]
    fn test_driver_inactive() {
        let driver = LidarDriver::new(10.0, 360);
        assert!(!driver.is_active());
        let scan = driver.read_scan(5);
        assert!(scan.ranges.is_empty());
    }
}
