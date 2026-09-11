#[cfg(target_os = "linux")]
pub use linux_gpio::*;

#[cfg(not(target_os = "linux"))]
pub use mock_gpio::*;

#[cfg(target_os = "linux")]
mod linux_gpio {
    use rppal::gpio::{Gpio, InputPin, OutputPin, Trigger};
    use std::time::Duration;
    use tokio::sync::mpsc;
    use log::{error, info, warn};

    // Physical pin mapping
    const LIDAR_SAFETY_PIN: u8 = 17; // BCM GPIO 17
    const MOTOR_ENABLE_PIN: u8 = 27; // BCM GPIO 27

    pub struct HardwareInterlock {
        lidar_pin: InputPin,
        motor_pin: OutputPin,
        alert_tx: mpsc::Sender<()>,
    }

    impl HardwareInterlock {
        pub fn new() -> Result<(Self, mpsc::Receiver<()>), Box<dyn std::error::Error>> {
            let gpio = Gpio::new()?;
            
            let mut lidar_pin = gpio.get(LIDAR_SAFETY_PIN)?.into_input_pullup();
            let mut motor_pin = gpio.get(MOTOR_ENABLE_PIN)?.into_output();
            
            motor_pin.set_high();

            let (alert_tx, alert_rx) = mpsc::channel(1);

            lidar_pin.set_interrupt(Trigger::FallingEdge)?;

            info!("HardwareInterlock initialized. LiDAR Pin: {}, Motor Pin: {}", LIDAR_SAFETY_PIN, MOTOR_ENABLE_PIN);

            let interlock = Self {
                lidar_pin,
                motor_pin,
                alert_tx,
            };

            Ok((interlock, alert_rx))
        }

        pub async fn listen_for_interrupt(mut self) {
            info!("Listening for sub-millisecond LiDAR hardware interrupts.");
            
            loop {
                let mut pin = self.lidar_pin.clone();
                let result = tokio::task::spawn_blocking(move || {
                    pin.poll_interrupt(true, Some(Duration::from_secs(1)))
                }).await;

                match result {
                    Ok(Ok(Some(_level))) => {
                        warn!("HARDWARE INTERLOCK TRIPPED! Disabling motor drivers immediately.");
                        self.motor_pin.set_low();
                        if let Err(e) = self.alert_tx.send(()).await {
                            error!("Failed to notify FSM of hardware interlock: {}", e);
                        }
                        break; 
                    }
                    Ok(Ok(None)) => {}
                    Ok(Err(e)) => error!("GPIO poll error: {}", e),
                    Err(e) => error!("Tokio spawn_blocking error: {}", e),
                }
            }
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod mock_gpio {
    use tokio::sync::mpsc;
    use log::{info, warn};

    pub struct HardwareInterlock {
        alert_tx: mpsc::Sender<()>,
    }

    impl HardwareInterlock {
        pub fn new() -> Result<(Self, mpsc::Receiver<()>), Box<dyn std::error::Error>> {
            info!("HardwareInterlock mocked (non-Linux OS).");
            let (alert_tx, alert_rx) = mpsc::channel(1);
            let interlock = Self { alert_tx };
            Ok((interlock, alert_rx))
        }

        pub async fn listen_for_interrupt(self) {
            info!("Mock listening for LiDAR interrupts.");
            // Just sleep forever in mock
            tokio::time::sleep(std::time::Duration::from_secs(86400)).await;
        }
    }
}
