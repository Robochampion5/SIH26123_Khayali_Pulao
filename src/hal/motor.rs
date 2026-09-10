use crate::hal::kinematics::WheelVelocity;

/// Motor command sent to the hardware driver.
#[derive(Debug, Clone, Copy)]
pub struct MotorCommand {
    /// Left wheel velocity (rad/s)
    pub left_vel: f32,
    /// Right wheel velocity (rad/s)
    pub right_vel: f32,
    /// Emergency stop flag (overrides velocities)
    pub e_stop: bool,
}

impl MotorCommand {
    pub fn from_wheel_velocity(wv: WheelVelocity) -> Self {
        Self {
            left_vel: wv.left,
            right_vel: wv.right,
            e_stop: false,
        }
    }

    pub fn stop() -> Self {
        Self {
            left_vel: 0.0,
            right_vel: 0.0,
            e_stop: false,
        }
    }

    pub fn emergency_stop() -> Self {
        Self {
            left_vel: 0.0,
            right_vel: 0.0,
            e_stop: true,
        }
    }

    /// Is the robot stationary?
    pub fn is_stopped(&self) -> bool {
        self.left_vel.abs() < 0.001 && self.right_vel.abs() < 0.001
    }
}

/// Motor driver abstraction.
///
/// In production: interfaces with GPIO/PWM to drive H-bridge or ESC.
/// For simulation: records last command for verification.
pub struct MotorDriver {
    /// Whether motors are enabled
    active: bool,
    /// Last command sent
    last_command: MotorCommand,
    /// Maximum wheel velocity (rad/s) — hardware limit
    max_wheel_vel: f32,
    /// Acceleration limit (rad/s²) — for ramp-rate limiting
    max_accel: f32,
    /// Command history for testing (if recording enabled)
    history: Option<Vec<MotorCommand>>,
}

impl MotorDriver {
    pub fn new(max_wheel_vel: f32, max_accel: f32) -> Self {
        Self {
            active: false,
            last_command: MotorCommand::stop(),
            max_wheel_vel,
            max_accel,
            history: None,
        }
    }

    /// Create a mock driver for testing, with command recording enabled.
    pub fn mock() -> Self {
        Self {
            active: true,
            last_command: MotorCommand::stop(),
            max_wheel_vel: 20.0,    // matches DiffDriveKinematics::default_amr()
            max_accel: 40.0,        // ~2 m/s² with 0.05m wheel radius
            history: Some(Vec::new()),
        }
    }

    pub fn start(&mut self) -> Result<(), &'static str> {
        self.active = true;
        Ok(())
    }

    pub fn stop_motors(&mut self) {
        self.send(MotorCommand::stop());
    }

    pub fn shutdown(&mut self) {
        self.send(MotorCommand::stop());
        self.active = false;
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn last_command(&self) -> &MotorCommand {
        &self.last_command
    }

    /// Send a motor command, applying ramp-rate limiting.
    ///
    /// In production, this writes PWM duty cycles to GPIO.
    /// Ramp-rate limits prevent mechanical shock and wheel slip.
    pub fn send(&mut self, mut cmd: MotorCommand) {
        if !self.active && !cmd.e_stop {
            return;
        }

        // Emergency stop bypasses rate limiting
        if cmd.e_stop {
            cmd.left_vel = 0.0;
            cmd.right_vel = 0.0;
            self.last_command = cmd;
            if let Some(ref mut hist) = self.history {
                hist.push(cmd);
            }
            return;
        }

        // Hardware velocity clamp
        cmd.left_vel  = cmd.left_vel.clamp(-self.max_wheel_vel, self.max_wheel_vel);
        cmd.right_vel = cmd.right_vel.clamp(-self.max_wheel_vel, self.max_wheel_vel);

        // Ramp-rate limiting (dt assumed 0.1s = 1 tick)
        let dt = 0.1;
        let max_delta = self.max_accel * dt;

        let delta_l = cmd.left_vel - self.last_command.left_vel;
        if delta_l.abs() > max_delta {
            cmd.left_vel = self.last_command.left_vel + delta_l.signum() * max_delta;
        }

        let delta_r = cmd.right_vel - self.last_command.right_vel;
        if delta_r.abs() > max_delta {
            cmd.right_vel = self.last_command.right_vel + delta_r.signum() * max_delta;
        }

        self.last_command = cmd;
        if let Some(ref mut hist) = self.history {
            hist.push(cmd);
        }
    }

    /// Get command history (for test assertions). Returns None if recording not enabled.
    pub fn command_history(&self) -> Option<&[MotorCommand]> {
        self.history.as_deref()
    }

    /// Clear command history
    pub fn clear_history(&mut self) {
        if let Some(ref mut hist) = self.history {
            hist.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::kinematics::WheelVelocity;

    #[test]
    fn test_motor_command_from_wheels() {
        let wv = WheelVelocity { left: 5.0, right: 8.0 };
        let cmd = MotorCommand::from_wheel_velocity(wv);
        assert!((cmd.left_vel - 5.0).abs() < 0.001);
        assert!((cmd.right_vel - 8.0).abs() < 0.001);
        assert!(!cmd.e_stop);
    }

    #[test]
    fn test_emergency_stop() {
        let cmd = MotorCommand::emergency_stop();
        assert!(cmd.e_stop);
        assert!(cmd.is_stopped());
    }

    #[test]
    fn test_driver_ramp_limiting() {
        let mut driver = MotorDriver::mock();
        // Try to jump from 0 to 20 rad/s in one tick
        // max_accel=40 → max_delta = 40*0.1 = 4.0 rad/s per tick
        let cmd = MotorCommand::from_wheel_velocity(WheelVelocity { left: 20.0, right: 20.0 });
        driver.send(cmd);
        // Should be clamped to 4.0 rad/s
        assert!((driver.last_command().left_vel - 4.0).abs() < 0.01);
        assert!((driver.last_command().right_vel - 4.0).abs() < 0.01);
    }

    #[test]
    fn test_driver_velocity_clamp() {
        let mut driver = MotorDriver::mock(); // max_wheel_vel = 20.0
        // Send many ramp steps to saturate
        for _ in 0..100 {
            let cmd = MotorCommand::from_wheel_velocity(WheelVelocity { left: 50.0, right: 50.0 });
            driver.send(cmd);
        }
        // Should never exceed max_wheel_vel
        assert!(driver.last_command().left_vel <= 20.0 + 0.01);
        assert!(driver.last_command().right_vel <= 20.0 + 0.01);
    }

    #[test]
    fn test_driver_e_stop_bypasses_ramp() {
        let mut driver = MotorDriver::mock();
        // Ramp up first
        for _ in 0..10 {
            let cmd = MotorCommand::from_wheel_velocity(WheelVelocity { left: 20.0, right: 20.0 });
            driver.send(cmd);
        }
        assert!(driver.last_command().left_vel > 0.0);

        // Emergency stop should bypass ramp limiting
        driver.send(MotorCommand::emergency_stop());
        assert!(driver.last_command().is_stopped());
        assert!(driver.last_command().e_stop);
    }

    #[test]
    fn test_driver_inactive_ignores() {
        let mut driver = MotorDriver::new(20.0, 40.0);
        // Not started yet
        driver.send(MotorCommand::from_wheel_velocity(WheelVelocity { left: 10.0, right: 10.0 }));
        // Should not have changed — driver inactive
        assert!(driver.last_command().is_stopped());
    }

    #[test]
    fn test_command_history() {
        let mut driver = MotorDriver::mock();
        driver.send(MotorCommand::stop());
        driver.send(MotorCommand::emergency_stop());
        let hist = driver.command_history().unwrap();
        assert_eq!(hist.len(), 2);
        assert!(!hist[0].e_stop);
        assert!(hist[1].e_stop);
    }
}
