use crate::space_time::Q16_16;

/// PID controller using Q16.16 fixed-point arithmetic.
///
/// Cascaded PID architecture:
///   Outer loop: heading PD controller → angular velocity setpoint
///   Inner loop: distance PID controller → linear velocity setpoint
///
/// All computations are deterministic fixed-point.
#[derive(Debug, Clone)]
pub struct PidController {
    /// Proportional gain (Q16.16)
    kp: Q16_16,
    /// Integral gain (Q16.16)
    ki: Q16_16,
    /// Derivative gain (Q16.16)
    kd: Q16_16,
    /// Integral accumulator (Q16.16)
    integral: Q16_16,
    /// Previous error for derivative term (Q16.16)
    prev_error: Q16_16,
    /// Output clamp (symmetric ±max)
    output_max: Q16_16,
    /// Integral windup limit
    integral_max: Q16_16,
}

impl PidController {
    pub fn new(kp: f32, ki: f32, kd: f32, output_max: f32, integral_max: f32) -> Self {
        Self {
            kp: Q16_16::from_f32(kp),
            ki: Q16_16::from_f32(ki),
            kd: Q16_16::from_f32(kd),
            integral: Q16_16::from_f32(0.0),
            prev_error: Q16_16::from_f32(0.0),
            output_max: Q16_16::from_f32(output_max),
            integral_max: Q16_16::from_f32(integral_max),
        }
    }

    /// Heading PD controller (no integral term — headings don't drift)
    pub fn heading_pd(kp: f32, kd: f32, max_angular_vel: f32) -> Self {
        Self::new(kp, 0.0, kd, max_angular_vel, 0.0)
    }

    /// Distance PID controller
    pub fn distance_pid(kp: f32, ki: f32, kd: f32, max_linear_vel: f32) -> Self {
        Self::new(kp, ki, kd, max_linear_vel, max_linear_vel * 2.0)
    }

    /// Compute PID output for a given error and dt (in seconds, Q16.16)
    pub fn update(&mut self, error: Q16_16, dt: Q16_16) -> Q16_16 {
        // P term
        let p_term = self.kp.mul(error);

        // I term with windup prevention
        self.integral = self.integral.add(error.mul(dt));
        // Clamp integral
        let int_f = self.integral.to_f32();
        let int_max_f = self.integral_max.to_f32();
        if int_f > int_max_f {
            self.integral = self.integral_max;
        } else if int_f < -int_max_f {
            self.integral = Q16_16::from_f32(-int_max_f);
        }
        let i_term = self.ki.mul(self.integral);

        // D term (on error, not measurement, for simplicity)
        let d_error = if dt.to_f32() > 0.0001 {
            Q16_16::from_f32((error.to_f32() - self.prev_error.to_f32()) / dt.to_f32())
        } else {
            Q16_16::from_f32(0.0)
        };
        let d_term = self.kd.mul(d_error);

        self.prev_error = error;

        // Sum and clamp
        let output = p_term.add(i_term).add(d_term);
        let out_f = output.to_f32();
        let max_f = self.output_max.to_f32();

        if out_f > max_f {
            Q16_16::from_f32(max_f)
        } else if out_f < -max_f {
            Q16_16::from_f32(-max_f)
        } else {
            output
        }
    }

    /// Reset the controller state (on setpoint change, mode transition)
    pub fn reset(&mut self) {
        self.integral = Q16_16::from_f32(0.0);
        self.prev_error = Q16_16::from_f32(0.0);
    }
}

/// Cascaded PID: heading PD → distance PID
pub struct CascadedController {
    pub heading_controller: PidController,
    pub distance_controller: PidController,
}

impl CascadedController {
    /// Default tuning for warehouse AMR
    pub fn default_amr() -> Self {
        Self {
            heading_controller: PidController::heading_pd(
                3.0,    // kp: aggressive heading correction
                0.1,    // kd: damping
                2.0,    // max angular velocity rad/s
            ),
            distance_controller: PidController::distance_pid(
                1.5,    // kp
                0.1,    // ki: steady-state error correction
                0.3,    // kd: damping
                1.0,    // max linear velocity m/s
            ),
        }
    }

    /// Compute (linear_velocity, angular_velocity) for a given position and heading error
    pub fn update(
        &mut self,
        heading_error: f32,
        distance_error: f32,
        dt: f32,
    ) -> (f32, f32) {
        let dt_q = Q16_16::from_f32(dt);

        // Outer loop: heading → angular velocity
        let angular_vel = self.heading_controller.update(
            Q16_16::from_f32(heading_error),
            dt_q,
        ).to_f32();

        // Inner loop: distance → linear velocity
        // Scale down linear velocity when turning hard (prevents overshoot)
        let heading_factor = 1.0_f32 - (heading_error.abs() / std::f32::consts::PI).min(0.8);
        // Angular velocity error must throttle linear velocity for turn constraints
        let linear_vel = (self.distance_controller.update(
            Q16_16::from_f32(distance_error * heading_factor),
            dt_q,
        ).to_f32()) * heading_factor;

        (linear_vel, angular_vel)
    }

    /// Reset both controllers
    pub fn reset(&mut self) {
        self.heading_controller.reset();
        self.distance_controller.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_proportional() {
        let mut pid = PidController::new(1.0, 0.0, 0.0, 10.0, 10.0);
        let dt = Q16_16::from_f32(0.1);
        let error = Q16_16::from_f32(5.0);
        let output = pid.update(error, dt);
        // P-only: output = kp * error = 1.0 * 5.0 = 5.0
        assert!((output.to_f32() - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_pid_clamping() {
        let mut pid = PidController::new(10.0, 0.0, 0.0, 3.0, 10.0);
        let dt = Q16_16::from_f32(0.1);
        let error = Q16_16::from_f32(5.0);
        let output = pid.update(error, dt);
        // Would be 50.0 but clamped to 3.0
        assert!((output.to_f32() - 3.0).abs() < 0.1);
    }

    #[test]
    fn test_pid_integral_windup() {
        let mut pid = PidController::new(0.0, 1.0, 0.0, 100.0, 2.0);
        let dt = Q16_16::from_f32(0.1);
        let error = Q16_16::from_f32(100.0);

        // Many updates to wind up integral
        for _ in 0..100 {
            pid.update(error, dt);
        }

        // Integral should be clamped to ±integral_max (2.0)
        assert!(pid.integral.to_f32() <= 2.0 + 0.1);
    }

    #[test]
    fn test_pid_reset() {
        let mut pid = PidController::new(1.0, 1.0, 1.0, 10.0, 10.0);
        let dt = Q16_16::from_f32(0.1);
        pid.update(Q16_16::from_f32(5.0), dt);
        pid.reset();
        assert!((pid.integral.to_f32()).abs() < 0.01);
        assert!((pid.prev_error.to_f32()).abs() < 0.01);
    }

    #[test]
    fn test_cascaded_straight() {
        let mut ctrl = CascadedController::default_amr();
        // No heading error, 1m distance → should get positive linear velocity
        let (v, w) = ctrl.update(0.0, 1.0, 0.1);
        assert!(v > 0.0);
        assert!(w.abs() < 0.1);
    }

    #[test]
    fn test_cascaded_turn() {
        let mut ctrl = CascadedController::default_amr();
        // Large heading error → should get angular velocity, reduced linear velocity
        let (v, w) = ctrl.update(1.0, 1.0, 0.1);
        assert!(w.abs() > 0.5);
        // Linear velocity should be reduced due to heading factor
        let (v_straight, _) = CascadedController::default_amr().update(0.0, 1.0, 0.1);
        assert!(v < v_straight);
    }
}
