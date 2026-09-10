use crate::space_time::{Heading, Q16_16};

/// Wheel velocity pair (rad/s per wheel)
#[derive(Debug, Clone, Copy)]
pub struct WheelVelocity {
    pub left: f32,   // rad/s
    pub right: f32,  // rad/s
}

impl WheelVelocity {
    pub fn zero() -> Self {
        Self { left: 0.0, right: 0.0 }
    }

    pub fn from_unicycle(v: f32, omega: f32, wheel_radius: f32, wheel_base: f32) -> Self {
        // Inverse differential-drive kinematics
        // v_left  = (v - omega * L/2) / r
        // v_right = (v + omega * L/2) / r
        let r = wheel_radius;
        let l = wheel_base;
        Self {
            left:  (v - omega * l / 2.0) / r,
            right: (v + omega * l / 2.0) / r,
        }
    }
}

/// Differential-drive kinematics.
///
/// Converts between:
///   - Unicycle model (linear velocity v, angular velocity ω)
///   - Wheel velocities (left/right rad/s)
///   - Pose deltas (dx, dy, dθ)
#[derive(Debug, Clone)]
pub struct DiffDriveKinematics {
    /// Wheel radius (meters)
    pub wheel_radius_m: f32,
    /// Distance between wheels (meters)
    pub wheel_base_m: f32,
    /// Maximum wheel angular velocity (rad/s)
    pub max_wheel_vel: f32,
}

impl DiffDriveKinematics {
    pub fn new(wheel_radius_m: f32, wheel_base_m: f32, max_wheel_vel: f32) -> Self {
        Self { wheel_radius_m, wheel_base_m, max_wheel_vel }
    }

    /// Standard warehouse AMR: 5cm radius wheels, 30cm base, up to 20 rad/s (1 m/s linear)
    pub fn default_amr() -> Self {
        Self::new(0.05, 0.30, 20.0)
    }

    /// Unicycle → wheel velocities (rad/s per wheel), clamped to max
    pub fn unicycle_to_wheels(&self, v: f32, omega: f32) -> WheelVelocity {
        let raw = WheelVelocity::from_unicycle(v, omega, self.wheel_radius_m, self.wheel_base_m);
        self.clamp_wheels(raw)
    }

    /// Wheel velocities (rad/s) → unicycle model (m/s, rad/s)
    pub fn wheels_to_unicycle(&self, wheels: WheelVelocity) -> (f32, f32) {
        let v_left  = wheels.left  * self.wheel_radius_m;
        let v_right = wheels.right * self.wheel_radius_m;
        let v     = (v_left + v_right) / 2.0;
        let omega = (v_right - v_left) / self.wheel_base_m;
        (v, omega)
    }

    /// Forward kinematics: wheel deltas (radians) → pose delta (dx m, dy m, dθ rad)
    /// Uses midpoint integration for accuracy.
    pub fn forward_kinematics(&self, delta_left_rad: f32, delta_right_rad: f32, theta: f32) -> (f32, f32, f32) {
        let dl = delta_left_rad  * self.wheel_radius_m;
        let dr = delta_right_rad * self.wheel_radius_m;
        let d      = (dl + dr) / 2.0;
        let dtheta = (dr - dl) / self.wheel_base_m;
        // Midpoint integration
        let mid_theta = theta + dtheta / 2.0;
        let dx = d * mid_theta.cos();
        let dy = d * mid_theta.sin();
        (dx, dy, dtheta)
    }

    /// Clamp wheel velocities to ±max_wheel_vel, scaling both proportionally if either exceeds
    pub fn clamp_wheels(&self, wheels: WheelVelocity) -> WheelVelocity {
        let max_abs = wheels.left.abs().max(wheels.right.abs());
        if max_abs <= self.max_wheel_vel {
            wheels
        } else {
            let scale = self.max_wheel_vel / max_abs;
            WheelVelocity {
                left:  wheels.left  * scale,
                right: wheels.right * scale,
            }
        }
    }

    /// Compute required angular velocity to face target heading from current heading.
    /// Returns ω (rad/s) — positive = counterclockwise.
    pub fn heading_angular_vel(&self, current_theta: f32, target_theta: f32, kp: f32, max_omega: f32) -> f32 {
        let mut err = target_theta - current_theta;
        // Wrap to [-π, π]
        while err > std::f32::consts::PI  { err -= std::f32::consts::TAU; }
        while err < -std::f32::consts::PI { err += std::f32::consts::TAU; }
        (kp * err).clamp(-max_omega, max_omega)
    }

    /// Maximum safe linear velocity given a stopping distance budget.
    /// v_max = sqrt(2 * a_max * d_stop), capped at hardware max.
    pub fn max_safe_velocity(&self, distance_to_stop_m: f32, a_max: f32) -> f32 {
        let v_hardware_max = self.max_wheel_vel * self.wheel_radius_m;
        (2.0 * a_max * distance_to_stop_m).sqrt().min(v_hardware_max)
    }

    /// Compute ticks needed to rotate by angle (radians) at max angular velocity.
    /// Useful for pre-computing rotation cost in D-SIPP.
    pub fn rotation_ticks(&self, angle_rad: f32, omega_max: f32, tick_dt: f32) -> u32 {
        if omega_max <= 0.0 || tick_dt <= 0.0 {
            return 0;
        }
        let t_seconds = angle_rad.abs() / omega_max;
        (t_seconds / tick_dt).ceil() as u32
    }

    /// Convert Θ_4 heading to radians (E=0, N=π/2, W=π, S=3π/2)
    pub fn heading_to_rad(heading: Heading) -> f32 {
        match heading {
            Heading::E => 0.0,
            Heading::N => std::f32::consts::FRAC_PI_2,
            Heading::W => std::f32::consts::PI,
            Heading::S => 3.0 * std::f32::consts::FRAC_PI_2,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unicycle_to_wheels_straight() {
        let kin = DiffDriveKinematics::default_amr();
        let wheels = kin.unicycle_to_wheels(0.5, 0.0);
        // Straight ahead: both wheels equal, v = ω_wheel * r → ω_wheel = 0.5 / 0.05 = 10 rad/s
        assert!((wheels.left - 10.0).abs() < 0.01);
        assert!((wheels.right - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_unicycle_to_wheels_turn() {
        let kin = DiffDriveKinematics::default_amr();
        // v=0, pure rotation left (positive omega)
        let wheels = kin.unicycle_to_wheels(0.0, 1.0);
        assert!(wheels.left < 0.0);
        assert!(wheels.right > 0.0);
        assert!((wheels.left + wheels.right).abs() < 0.01); // symmetric
    }

    #[test]
    fn test_wheels_to_unicycle_roundtrip() {
        let kin = DiffDriveKinematics::default_amr();
        let v_in = 0.3;
        let omega_in = 0.5;
        let wheels = kin.unicycle_to_wheels(v_in, omega_in);
        let (v_out, omega_out) = kin.wheels_to_unicycle(wheels);
        assert!((v_out - v_in).abs() < 0.001);
        assert!((omega_out - omega_in).abs() < 0.001);
    }

    #[test]
    fn test_clamp_proportional() {
        let kin = DiffDriveKinematics::default_amr(); // max_wheel_vel = 20.0
        let wheels = WheelVelocity { left: 30.0, right: 15.0 };
        let clamped = kin.clamp_wheels(wheels);
        // Scale = 20/30 = 0.667
        assert!((clamped.left - 20.0).abs() < 0.01);
        assert!((clamped.right - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_forward_kinematics_straight() {
        let kin = DiffDriveKinematics::default_amr();
        // 1 revolution each wheel → 2π * 0.05 = ~0.314m straight
        let (dx, dy, dtheta) = kin.forward_kinematics(
            2.0 * std::f32::consts::PI,
            2.0 * std::f32::consts::PI,
            0.0, // facing East
        );
        assert!((dx - 0.314).abs() < 0.01);
        assert!(dy.abs() < 0.01);
        assert!(dtheta.abs() < 0.001);
    }

    #[test]
    fn test_max_safe_velocity() {
        let kin = DiffDriveKinematics::default_amr();
        // d=0.5m, a=2.0 → v_max = sqrt(2*2*0.5) = sqrt(2) ≈ 1.414, but hardware max = 20*0.05 = 1.0
        let v = kin.max_safe_velocity(0.5, 2.0);
        assert!((v - 1.0).abs() < 0.01); // clamped to hardware max
    }

    #[test]
    fn test_heading_angular_vel_wraps() {
        let kin = DiffDriveKinematics::default_amr();
        // Turning from near 2π back to 0 should take shortest path
        let omega = kin.heading_angular_vel(
            6.0,  // just under 2π
            0.1,  // just past 0
            1.0,
            10.0,
        );
        // Error = 0.1 - 6.0 + 2π ≈ 0.38 rad; omega should be positive (CCW, small)
        assert!(omega > 0.0);
        assert!(omega < 1.0);
    }

    #[test]
    fn test_wheel_velocity_zero() {
        let w = WheelVelocity::zero();
        assert_eq!(w.left, 0.0);
        assert_eq!(w.right, 0.0);
    }
}
