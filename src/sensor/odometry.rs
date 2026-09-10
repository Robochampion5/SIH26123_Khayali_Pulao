use crate::space_time::Tick;
use crate::sensor::OdometryReading;

/// Odometry driver abstraction.
///
/// In production, reads wheel encoders via GPIO/I2C.
/// For simulation, returns mock readings.
pub struct OdometryDriver {
    /// Wheel base in meters
    wheel_base_m: f32,
    /// Ticks per revolution for encoders
    ticks_per_rev: u32,
    /// Wheel radius in meters
    wheel_radius_m: f32,
    /// Last encoder counts (left, right)
    last_counts: (i64, i64),
    /// Whether driver is active
    active: bool,
}

impl OdometryDriver {
    pub fn new(wheel_base_m: f32, ticks_per_rev: u32, wheel_radius_m: f32) -> Self {
        Self {
            wheel_base_m,
            ticks_per_rev,
            wheel_radius_m,
            last_counts: (0, 0),
            active: false,
        }
    }

    /// Create a mock driver for testing
    pub fn mock() -> Self {
        Self {
            wheel_base_m: 0.3,
            ticks_per_rev: 1000,
            wheel_radius_m: 0.05,
            last_counts: (0, 0),
            active: true,
        }
    }

    pub fn start(&mut self) -> Result<(), &'static str> {
        self.active = true;
        Ok(())
    }

    pub fn stop(&mut self) {
        self.active = false;
    }

    /// Convert encoder ticks to meters
    fn ticks_to_meters(&self, ticks: i64) -> f32 {
        let revolutions = ticks as f32 / self.ticks_per_rev as f32;
        revolutions * 2.0 * std::f32::consts::PI * self.wheel_radius_m
    }

    /// Read odometry from encoders. Returns delta since last read.
    pub fn read(&mut self, current_tick: Tick) -> OdometryReading {
        if !self.active {
            return OdometryReading {
                delta_left_m: 0.0,
                delta_right_m: 0.0,
                wheel_base_m: self.wheel_base_m,
                timestamp_tick: current_tick,
            };
        }

        // In production: read GPIO encoder counters
        // For simulation: returns zero delta (test code injects via inject_counts)
        OdometryReading {
            delta_left_m: 0.0,
            delta_right_m: 0.0,
            wheel_base_m: self.wheel_base_m,
            timestamp_tick: current_tick,
        }
    }

    /// Inject encoder counts for testing
    pub fn inject_counts(&mut self, left: i64, right: i64, current_tick: Tick) -> OdometryReading {
        let delta_left = left - self.last_counts.0;
        let delta_right = right - self.last_counts.1;
        self.last_counts = (left, right);

        OdometryReading {
            delta_left_m: self.ticks_to_meters(delta_left),
            delta_right_m: self.ticks_to_meters(delta_right),
            wheel_base_m: self.wheel_base_m,
            timestamp_tick: current_tick,
        }
    }
}

/// Mock odometry source for testing
pub struct MockOdometry {
    wheel_base_m: f32,
}

impl MockOdometry {
    pub fn new(wheel_base_m: f32) -> Self {
        Self { wheel_base_m }
    }

    /// Generate a straight-line movement reading
    pub fn straight(&self, distance_m: f32, tick: Tick) -> OdometryReading {
        OdometryReading {
            delta_left_m: distance_m,
            delta_right_m: distance_m,
            wheel_base_m: self.wheel_base_m,
            timestamp_tick: tick,
        }
    }

    /// Generate a pure rotation reading (positive = left turn)
    pub fn rotate(&self, angle_rad: f32, tick: Tick) -> OdometryReading {
        let arc = angle_rad * self.wheel_base_m / 2.0;
        OdometryReading {
            delta_left_m: -arc,
            delta_right_m: arc,
            wheel_base_m: self.wheel_base_m,
            timestamp_tick: tick,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mock_odometry_straight() {
        let odom = MockOdometry::new(0.3);
        let reading = odom.straight(0.5, 10);
        assert_eq!(reading.delta_left_m, 0.5);
        assert_eq!(reading.delta_right_m, 0.5);
    }

    #[test]
    fn test_mock_odometry_rotate() {
        let odom = MockOdometry::new(0.3);
        let reading = odom.rotate(std::f32::consts::FRAC_PI_2, 10);
        // Left wheel moves backward, right forward for left turn
        assert!(reading.delta_left_m < 0.0);
        assert!(reading.delta_right_m > 0.0);
    }

    #[test]
    fn test_driver_inject_counts() {
        let mut driver = OdometryDriver::mock();
        let reading = driver.inject_counts(100, 100, 5);
        // 100 ticks / 1000 ticks_per_rev = 0.1 rev * 2π * 0.05m = ~0.0314m
        assert!(reading.delta_left_m > 0.0);
        assert!((reading.delta_left_m - reading.delta_right_m).abs() < 0.001);
    }
}
