use std::time::{Duration, Instant};
use log::{info, warn, error};

pub struct TaskCfpManager {
    burst_interval: Duration,
    burst_window: Duration,
}

impl TaskCfpManager {
    pub fn new() -> Self {
        Self {
            burst_interval: Duration::from_millis(100),
            burst_window: Duration::from_secs(1),
        }
    }

    /// Aggressive Linear Micro-Bursting for TASK_CFP.
    /// Broadcasts every 100ms for exactly 1.0s. If quorum fails, task is dequeued.
    pub async fn execute_cfp_burst<F, Fut>(&self, mut broadcast_fn: F) -> Result<(), &'static str>
    where
        F: FnMut() -> Fut,
        Fut: std::future::Future<Output = Result<bool, &'static str>>, // Returns true if quorum reached
    {
        let start = Instant::now();
        let mut attempts = 0;

        info!("Starting TASK_CFP micro-bursting window (1.0s)");

        while start.elapsed() < self.burst_window {
            attempts += 1;
            match broadcast_fn().await {
                Ok(true) => {
                    info!("CFP Quorum reached after {} attempts", attempts);
                    return Ok(());
                }
                Ok(false) => {
                    // Quorum not yet reached, wait for next interval
                }
                Err(e) => {
                    error!("Broadcast failed: {}", e);
                }
            }
            tokio::time::sleep(self.burst_interval).await;
        }

        warn!("CFP burst window expired. Quorum failed. Dequeuing task.");
        Err("Quorum failed within 1.0s window")
    }
}

/// Appends EDGE_BLOCKED coordinates to the 1Hz DIAGNOSTIC_HEARTBEAT
pub struct PiggybackTelemetry {
    blocked_cells: Vec<(u16, u16)>,
    block_timestamp: Option<Instant>,
}

impl PiggybackTelemetry {
    pub fn new() -> Self {
        Self {
            blocked_cells: Vec::new(),
            block_timestamp: None,
        }
    }

    pub fn report_edge_blocked(&mut self, x: u16, y: u16) {
        if !self.blocked_cells.contains(&(x, y)) {
            self.blocked_cells.push((x, y));
        }
        self.block_timestamp = Some(Instant::now());
    }

    pub fn get_piggyback_payload(&mut self) -> Option<Vec<(u16, u16)>> {
        if let Some(ts) = self.block_timestamp {
            // Broadcast for exactly 5 seconds
            if ts.elapsed() <= Duration::from_secs(5) {
                return Some(self.blocked_cells.clone());
            } else {
                // Expired
                self.blocked_cells.clear();
                self.block_timestamp = None;
            }
        }
        None
    }
}
