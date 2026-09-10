use crate::space_time::{CellID, Tick, Q8_8};
use crate::network::payloads::*;
use std::collections::HashMap;

/// Weights for bid utility computation (Q8.8 fixed-point)
const W1_DISTANCE: f32 = 0.60;
const W2_BATTERY:  f32 = 0.25;
const W3_QUEUE:    f32 = 0.15;

/// Bid window in ticks (500ms = 5 ticks at 10Hz)
const BID_WINDOW_TICKS: u16 = 5;

/// Retry interval for deferred tasks (2000ms = 20 ticks)
const RETRY_INTERVAL_TICKS: Tick = 20;

/// Maximum retry count before giving up
const MAX_RETRIES: u8 = 5;

/// Maximum battery percentage (for normalizing)
const B_MAX: f32 = 100.0;

// ─── CNP Manager: announces tasks and collects bids ───

#[derive(Debug)]
/// Blind Triple-Transmit (micro-burst): CFP/BID sent 3x in 30ms window (10ms spacing)
pub struct CnpManager {
    /// Active auctions: task_id → AuctionState
    auctions: HashMap<u16, AuctionState>,
    /// Deferred tasks awaiting retry
    deferred: Vec<DeferredTask>,
    /// Next task_id to assign
    next_task_id: u16,
}

#[derive(Debug)]
struct AuctionState {
    cfp: TaskCfpPayload,
    bids: Vec<TaskBidPayload>,
    start_tick: Tick,
    awarded: bool,
}

#[derive(Debug)]
struct DeferredTask {
    cfp: TaskCfpPayload,
    retry_count: u8,
    next_retry_tick: Tick,
}

impl CnpManager {
    pub fn new() -> Self {
        Self {
            auctions: HashMap::new(),
            deferred: Vec::new(),
            next_task_id: 1,
        }
    }

    /// Create a new task CFP and return the serialized payload to broadcast.
    pub fn create_task(
        &mut self,
        pickup: CellID,
        dropoff: CellID,
        priority: u8,
        deadline: Tick,
        manager_id: u8,
        current_tick: Tick,
    ) -> TaskCfpPayload {
        let task_id = self.next_task_id;
        self.next_task_id = self.next_task_id.wrapping_add(1);

        let cfp = TaskCfpPayload {
            task_id,
            pickup_cell: pickup,
            dropoff_cell: dropoff,
            priority,
            deadline_tick: deadline,
            manager_id,
            bid_window_ticks: BID_WINDOW_TICKS,
        };

        self.auctions.insert(task_id, AuctionState {
            cfp,
            bids: Vec::new(),
            start_tick: current_tick,
            awarded: false,
        });

        cfp
    }

    /// Process an incoming bid
    pub fn receive_bid(&mut self, bid: TaskBidPayload) {
        if let Some(auction) = self.auctions.get_mut(&bid.task_id) {
            if !auction.awarded {
                auction.bids.push(bid);
            }
        }
    }

    /// Check auctions whose bid window has closed and award to the best bidder.
    /// Returns a list of TASK_AWARD payloads to broadcast.
    pub fn check_awards(&mut self, current_tick: Tick) -> Vec<TaskAwardPayload> {
        let mut awards = Vec::new();
        let mut to_defer = Vec::new();

        for (task_id, auction) in self.auctions.iter_mut() {
            if auction.awarded {
                continue;
            }

            let elapsed = current_tick.saturating_sub(auction.start_tick);
            if elapsed < auction.cfp.bid_window_ticks as Tick {
                continue; // bid window still open
            }

            if auction.bids.is_empty() {
                // No bids → defer
                to_defer.push(auction.cfp);
                auction.awarded = true; // mark as handled
                continue;
            }

            // Award to highest utility; tiebreak by lowest agent_id
            let best = auction.bids.iter().max_by(|a, b| {
                a.utility.0.cmp(&b.utility.0)
                    .then_with(|| b.agent_id.cmp(&a.agent_id)) // lower ID wins tie
            });

            if let Some(winner) = best {
                let award = TaskAwardPayload {
                    task_id: *task_id,
                    agent_id: winner.agent_id,
                    pickup: auction.cfp.pickup_cell,
                    dropoff: auction.cfp.dropoff_cell,
                };
                awards.push(award);
                auction.awarded = true;
            }
        }

        // Move expired no-bid auctions to deferred queue
        for cfp in to_defer {
            self.deferred.push(DeferredTask {
                cfp,
                retry_count: 0,
                next_retry_tick: current_tick + RETRY_INTERVAL_TICKS,
            });
        }

        awards
    }

    /// Get CFPs ready for retry and return them for re-broadcast.
    pub fn check_retries(&mut self, current_tick: Tick) -> Vec<TaskCfpPayload> {
        let mut retries = Vec::new();
        self.deferred.retain_mut(|d| {
            if current_tick >= d.next_retry_tick {
                if d.retry_count >= MAX_RETRIES {
                    log::warn!("Task {} permanently deferred after {} retries", d.cfp.task_id, d.retry_count);
                    return false; // drop from queue
                }
                d.retry_count += 1;
                // Exponential backoff: retry_interval * 2^retry_count, capped at 100 ticks (10s)
                let backoff = (RETRY_INTERVAL_TICKS << d.retry_count.min(3)).min(100);
                d.next_retry_tick = current_tick + backoff;
                retries.push(d.cfp);
            }
            true
        });
        retries
    }

    /// Cleanup completed auctions older than a threshold
    pub fn gc(&mut self, current_tick: Tick) {
        self.auctions.retain(|_, a| {
            !a.awarded || current_tick.saturating_sub(a.start_tick) < 500
        });
    }
}

// ─── CNP Bidder: computes and sends bids for announced tasks ───

#[derive(Debug)]
pub struct CnpBidder {
    /// Agent's own ID
    agent_id: u8,
    /// Tasks we've already bid on (idempotency: task_id → seq_no)
    bid_history: HashMap<u16, u16>,
    /// Current monotonic sequence number for outgoing bids
    seq_no: u16,
}

impl CnpBidder {
    pub fn new(agent_id: u8) -> Self {
        Self {
            agent_id,
            bid_history: HashMap::new(),
            seq_no: 0,
        }
    }

    /// Compute a bid for a given CFP. Returns None if we've already bid on this task.
    ///
    /// Utility:
    ///   U = w1 · 1/(D_total+1) + w2 · B/B_max + w3 · 1/(Q+1)
    ///
    /// All arithmetic uses Q8.8 fixed-point for bit-exact cross-platform results.
    pub fn compute_bid(
        &mut self,
        cfp: &TaskCfpPayload,
        d_total: u16,       // estimated total travel ticks (D-SIPP estimate)
        battery_pct: u8,
        queue_depth: u8,
    ) -> Option<TaskBidPayload> {
        // Idempotency: don't bid twice on the same task
        if self.bid_history.contains_key(&cfp.task_id) {
            return None;
        }

        // Compute utility in Q8.8
        let term1 = Q8_8::from_f32(W1_DISTANCE / (d_total as f32 + 1.0));
        let term2 = Q8_8::from_f32(W2_BATTERY * (battery_pct as f32 / B_MAX));
        let term3 = Q8_8::from_f32(W3_QUEUE / (queue_depth as f32 + 1.0));
        let utility = term1.add(term2).add(term3);

        self.seq_no = self.seq_no.wrapping_add(1);
        self.bid_history.insert(cfp.task_id, self.seq_no);

        Some(TaskBidPayload {
            task_id: cfp.task_id,
            agent_id: self.agent_id,
            utility,
            estimated_ticks: d_total,
            battery_pct,
            queue_depth,
        })
    }

    /// Garbage-collect old bid history
    pub fn gc(&mut self, max_entries: usize) {
        if self.bid_history.len() > max_entries {
            // Keep only the most recent half (crude but effective)
            let cutoff = self.seq_no.wrapping_sub(max_entries as u16 / 2);
            self.bid_history.retain(|_, seq| *seq > cutoff);
        }
    }
}


/// Split-Brain Quorum (§3.2 v3.0): agent claims iff U_self = max(U_fleet) AND |heard_from| > N/2
/// Even-split tie (e.g. 2/2 in N=4): partition with min(global_ID) wins; other defers.
impl CnpBidder {
    pub fn should_claim_task(&self, self_utility: u16, max_fleet_utility: u16, heard_from: usize, total_fleet: usize, min_global_id_in_partition: u8) -> bool {
        if self_utility != max_fleet_utility { return false; }
        if heard_from > total_fleet / 2 { return true; }
        if heard_from == total_fleet / 2 {
            // Even-split: lowest global ID partition achieves quorum
            return min_global_id_in_partition == self.agent_id;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_manager_create_and_award() {
        let mut mgr = CnpManager::new();
        let cfp = mgr.create_task(10, 50, 1, 1000, 0, 0);

        let bid = TaskBidPayload {
            task_id: cfp.task_id,
            agent_id: 3,
            utility: Q8_8::from_f32(0.8),
            estimated_ticks: 20,
            battery_pct: 90,
            queue_depth: 0,
        };
        mgr.receive_bid(bid);

        // Window hasn't closed yet (tick 0, window = 5)
        let awards = mgr.check_awards(3);
        assert!(awards.is_empty());

        // Window closed
        let awards = mgr.check_awards(6);
        assert_eq!(awards.len(), 1);
        assert_eq!(awards[0].agent_id, 3);
        assert_eq!(awards[0].task_id, cfp.task_id);
    }

    #[test]
    fn test_manager_no_bids_defers() {
        let mut mgr = CnpManager::new();
        let cfp = mgr.create_task(10, 50, 0, 500, 0, 0);

        // Window closes with no bids
        let awards = mgr.check_awards(10);
        assert!(awards.is_empty());
        assert_eq!(mgr.deferred.len(), 1);
    }

    #[test]
    fn test_manager_best_bid_wins() {
        let mut mgr = CnpManager::new();
        let cfp = mgr.create_task(10, 50, 0, 500, 0, 0);

        mgr.receive_bid(TaskBidPayload {
            task_id: cfp.task_id, agent_id: 1,
            utility: Q8_8::from_f32(0.5), estimated_ticks: 30, battery_pct: 80, queue_depth: 0,
        });
        mgr.receive_bid(TaskBidPayload {
            task_id: cfp.task_id, agent_id: 2,
            utility: Q8_8::from_f32(0.9), estimated_ticks: 15, battery_pct: 90, queue_depth: 0,
        });

        let awards = mgr.check_awards(10);
        assert_eq!(awards[0].agent_id, 2); // higher utility wins
    }

    #[test]
    fn test_bidder_utility_range() {
        let mut bidder = CnpBidder::new(1);
        let cfp = TaskCfpPayload {
            task_id: 1, pickup_cell: 10, dropoff_cell: 50,
            priority: 0, deadline_tick: 500, manager_id: 0, bid_window_ticks: 5,
        };

        let bid = bidder.compute_bid(&cfp, 20, 80, 1).unwrap();
        let u = bid.utility.to_f32();
        // w1/(21) + w2*80/100 + w3/(2) ≈ 0.029 + 0.20 + 0.075 = 0.304
        assert!(u > 0.0 && u < 1.0, "Utility {} out of expected range", u);
    }

    #[test]
    fn test_bidder_idempotent() {
        let mut bidder = CnpBidder::new(1);
        let cfp = TaskCfpPayload {
            task_id: 1, pickup_cell: 10, dropoff_cell: 50,
            priority: 0, deadline_tick: 500, manager_id: 0, bid_window_ticks: 5,
        };

        let bid1 = bidder.compute_bid(&cfp, 20, 80, 0);
        assert!(bid1.is_some());
        let bid2 = bidder.compute_bid(&cfp, 20, 80, 0);
        assert!(bid2.is_none()); // idempotent
    }

    #[test]
    fn test_retry_backoff() {
        let mut mgr = CnpManager::new();
        let _cfp = mgr.create_task(10, 50, 0, 500, 0, 0);

        // Close window with no bids
        mgr.check_awards(10);
        assert_eq!(mgr.deferred.len(), 1);

        // Not ready yet
        let retries = mgr.check_retries(20);
        assert!(retries.is_empty());

        // Ready at tick 30 (0 + 20 retry interval)
        let retries = mgr.check_retries(30);
        assert_eq!(retries.len(), 1);

        // Next retry should be at 30 + 40 (backoff: 20 << 1)
        let retries2 = mgr.check_retries(60);
        assert!(retries2.is_empty());
        let retries3 = mgr.check_retries(70);
        assert_eq!(retries3.len(), 1);
    }
}
