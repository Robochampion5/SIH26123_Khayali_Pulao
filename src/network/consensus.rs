//! Two-Phase Commit (2PC) & Lamport Clocks — Version 4.0
//!
//! IntentLockManager: coordinate trajectory reservation across a 15-cell radius
//! before entering EXECUTING state. PREPARE → ACK_PROMISE collection → COMMIT/ABORT.
//!
//! LamportClock: causally ordered event timestamps for all network messages.
//! Every send increments; every receive merges max(local, remote) + 1.
//!
//! Zero-tolerance: no agent enters EXECUTING without holding committed locks
//! for every cell in its planned trajectory.

use crate::space_time::{CellID, Tick, SafeInterval};
use std::collections::HashMap;

// ─── Lamport Clock ─────────────────────────────────────────────────────────────

/// Causally ordered scalar clock for decentralized event ordering.
///
/// Invariant: if event A causally precedes B, then clock(A) < clock(B).
/// Tiebreak on agent_id for strict total ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LamportClock {
    /// Current logical timestamp
    counter: u64,
    /// Agent owning this clock (tiebreaker for concurrent events)
    agent_id: u8,
}

impl LamportClock {
    pub fn new(agent_id: u8) -> Self {
        Self { counter: 0, agent_id }
    }

    /// Increment on local event (planning, state transition)
    pub fn tick(&mut self) -> u64 {
        self.counter += 1;
        self.counter
    }

    /// Increment on message send. Returns timestamp to embed in wire payload.
    pub fn send_timestamp(&mut self) -> u64 {
        self.counter += 1;
        self.counter
    }

    /// Merge on message receive: max(local, remote) + 1
    pub fn receive(&mut self, remote_timestamp: u64) {
        self.counter = self.counter.max(remote_timestamp) + 1;
    }

    /// Current value (read-only, no increment)
    pub fn current(&self) -> u64 {
        self.counter
    }

    pub fn agent_id(&self) -> u8 {
        self.agent_id
    }

    /// Compare two (timestamp, agent_id) pairs for strict total ordering.
    /// Lower timestamp wins; ties broken by lower agent_id.
    pub fn order(ts_a: u64, id_a: u8, ts_b: u64, id_b: u8) -> std::cmp::Ordering {
        ts_a.cmp(&ts_b).then_with(|| id_a.cmp(&id_b))
    }
}

// ─── 2PC Protocol Types ────────────────────────────────────────────────────────

/// Radius in cells for lock scope (Manhattan distance from any waypoint)
const LOCK_RADIUS: u16 = 15;

/// Timeout for ACK_PROMISE collection (in ticks). After this, ABORT.
const PREPARE_TIMEOUT_TICKS: Tick = 10;

/// Maximum cells in a single lock request
const MAX_LOCK_CELLS: usize = 256;

/// Transaction ID: (agent_id, lamport_timestamp) is globally unique
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TransactionId {
    pub agent_id: u8,
    pub timestamp: u64,
}

impl TransactionId {
    pub fn new(agent_id: u8, timestamp: u64) -> Self {
        Self { agent_id, timestamp }
    }
}

/// Outcome of the 2PC transaction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwoPhaseOutcome {
    /// PREPARE sent, awaiting ACK_PROMISE responses
    Pending,
    /// Sufficient ACK_PROMISEs received — locks committed
    Committed,
    /// Timeout expired or NACK received — locks released
    Aborted,
}

/// A single cell lock within a transaction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellLock {
    pub cell: CellID,
    pub interval: SafeInterval,
    pub agent_id: u8,
    pub txn_id: TransactionId,
}

/// PREPARE message: broadcast to all agents within 15-cell radius
#[derive(Debug, Clone)]
pub struct PreparePayload {
    pub txn_id: TransactionId,
    pub lamport_ts: u64,
    /// Cells + intervals this agent wants to lock
    pub locks: Vec<CellLockRequest>,
    /// Tick when PREPARE was sent (for timeout)
    pub sent_tick: Tick,
}

#[derive(Debug, Clone, Copy)]
pub struct CellLockRequest {
    pub cell: CellID,
    pub t_start: Tick,
    pub t_end: Tick,
}

impl CellLockRequest {
    /// Serialize to bytes (8 bytes: cell u16 + t_start u32 + t_end u32... but we pack tighter)
    pub fn to_bytes(&self) -> [u8; 10] {
        let mut buf = [0u8; 10];
        buf[0..2].copy_from_slice(&self.cell.to_le_bytes());
        buf[2..6].copy_from_slice(&self.t_start.to_le_bytes());
        buf[6..10].copy_from_slice(&self.t_end.to_le_bytes());
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 10 { return None; }
        Some(Self {
            cell: u16::from_le_bytes([data[0], data[1]]),
            t_start: u32::from_le_bytes([data[2], data[3], data[4], data[5]]),
            t_end: u32::from_le_bytes([data[6], data[7], data[8], data[9]]),
        })
    }
}

/// ACK_PROMISE response from a peer: "I promise not to lock these cells"
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AckPromise {
    pub txn_id: TransactionId,
    pub responder_id: u8,
    pub lamport_ts: u64,
    pub accepted: bool,
}

/// COMMIT message: tells peers this transaction's locks are final
#[derive(Debug, Clone, Copy)]
pub struct CommitPayload {
    pub txn_id: TransactionId,
    pub lamport_ts: u64,
}

/// ABORT message: tells peers to release prepared locks
#[derive(Debug, Clone, Copy)]
pub struct AbortPayload {
    pub txn_id: TransactionId,
    pub lamport_ts: u64,
}

// ─── IntentLockManager ─────────────────────────────────────────────────────────

/// Active transaction state for the local agent
#[derive(Debug)]
struct PendingTransaction {
    txn_id: TransactionId,
    locks: Vec<CellLockRequest>,
    sent_tick: Tick,
    acks_received: Vec<AckPromise>,
    /// How many peers we expect to hear from
    expected_ack_count: usize,
    outcome: TwoPhaseOutcome,
}

/// Manages 2PC locking for trajectory reservations.
///
/// Flow:
/// 1. Agent plans trajectory via D-SIPP
/// 2. Agent calls `prepare()` → broadcasts PREPARE to all peers within 15-cell radius
/// 3. Peers respond with ACK_PROMISE (or NACK if conflicting lock held)
/// 4. If all ACKs received within timeout → `commit()` → EXECUTING
/// 5. If timeout or any NACK → `abort()` → stay in PLANNING
///
/// Invariant: agent NEVER enters EXECUTING without committed locks.
#[derive(Debug)]
pub struct IntentLockManager {
    /// Lamport clock for this agent
    pub clock: LamportClock,
    /// Active outbound transaction (at most one at a time)
    pending: Option<PendingTransaction>,
    /// Locks held by OTHER agents (received via COMMIT)
    foreign_locks: HashMap<TransactionId, Vec<CellLock>>,
    /// Locks committed by THIS agent
    own_committed: HashMap<TransactionId, Vec<CellLock>>,
    /// Prepared-but-not-committed foreign locks (received via PREPARE, responded ACK)
    foreign_prepared: HashMap<TransactionId, Vec<CellLockRequest>>,
    /// Grid dimensions (for radius calculation)
    grid_width: u16,
    grid_height: u16,
}

impl IntentLockManager {
    pub fn new(agent_id: u8, grid_width: u16, grid_height: u16) -> Self {
        Self {
            clock: LamportClock::new(agent_id),
            pending: None,
            foreign_locks: HashMap::new(),
            own_committed: HashMap::new(),
            foreign_prepared: HashMap::new(),
            grid_width,
            grid_height,
        }
    }

    /// Agent ID
    pub fn agent_id(&self) -> u8 {
        self.clock.agent_id()
    }

    // ── Phase 1: PREPARE ────────────────────────────────────────────────────

    /// Initiate 2PC for a planned trajectory.
    ///
    /// Extracts cells within 15-cell Manhattan radius of each waypoint,
    /// builds lock requests, and returns a PREPARE payload to broadcast.
    ///
    /// Returns None if a transaction is already pending.
    pub fn prepare(
        &mut self,
        waypoint_cells: &[(CellID, Tick, Tick)], // (cell, t_arrive, t_depart)
        expected_peers: usize,
        current_tick: Tick,
    ) -> Option<PreparePayload> {
        if self.pending.is_some() {
            return None; // one transaction at a time
        }

        let ts = self.clock.send_timestamp();
        let txn_id = TransactionId::new(self.agent_id(), ts);

        // Build lock requests for every waypoint cell+interval
        let mut locks = Vec::with_capacity(waypoint_cells.len());
        for &(cell, t_arrive, t_depart) in waypoint_cells {
            locks.push(CellLockRequest {
                cell,
                t_start: t_arrive,
                t_end: t_depart,
            });
        }

        // Cap at MAX_LOCK_CELLS
        if locks.len() > MAX_LOCK_CELLS {
            locks.truncate(MAX_LOCK_CELLS);
        }

        let prepare = PreparePayload {
            txn_id,
            lamport_ts: ts,
            locks: locks.clone(),
            sent_tick: current_tick,
        };

        self.pending = Some(PendingTransaction {
            txn_id,
            locks,
            sent_tick: current_tick,
            acks_received: Vec::with_capacity(expected_peers),
            expected_ack_count: expected_peers,
            outcome: TwoPhaseOutcome::Pending,
        });

        Some(prepare)
    }

    // ── Incoming ACK_PROMISE ────────────────────────────────────────────────

    /// Process an ACK_PROMISE from a peer. Returns the transaction outcome
    /// if this ACK completes the collection (all ACKs received → Committed).
    pub fn receive_ack(&mut self, ack: AckPromise) -> Option<TwoPhaseOutcome> {
        self.clock.receive(ack.lamport_ts);

        let pending = self.pending.as_mut()?;
        if ack.txn_id != pending.txn_id {
            return None; // stale ACK for old transaction
        }
        if pending.outcome != TwoPhaseOutcome::Pending {
            return None; // already resolved
        }

        pending.acks_received.push(ack);

        // NACK → immediate abort
        if !ack.accepted {
            pending.outcome = TwoPhaseOutcome::Aborted;
            return Some(TwoPhaseOutcome::Aborted);
        }

        // All ACKs received → ready to commit
        let ack_count = pending.acks_received.iter().filter(|a| a.accepted).count();
        if ack_count >= pending.expected_ack_count {
            // Don't set committed yet — caller must call commit() to finalize
            return Some(TwoPhaseOutcome::Committed);
        }

        None
    }

    // ── Phase 2: COMMIT / ABORT ─────────────────────────────────────────────

    /// Commit the pending transaction. Finalizes locks and returns COMMIT payload.
    ///
    /// Caller broadcasts this COMMIT to all peers.
    pub fn commit(&mut self) -> Option<CommitPayload> {
        let pending = self.pending.take()?;
        let ts = self.clock.send_timestamp();
        let txn_id = pending.txn_id;

        // Store committed locks
        let cell_locks: Vec<CellLock> = pending.locks.iter().map(|lr| {
            CellLock {
                cell: lr.cell,
                interval: SafeInterval::new(lr.t_start, lr.t_end),
                agent_id: self.agent_id(),
                txn_id,
            }
        }).collect();
        self.own_committed.insert(txn_id, cell_locks);

        Some(CommitPayload { txn_id, lamport_ts: ts })
    }

    /// Abort the pending transaction. Returns ABORT payload to broadcast.
    pub fn abort(&mut self) -> Option<AbortPayload> {
        let pending = self.pending.take()?;
        let ts = self.clock.send_timestamp();
        Some(AbortPayload {
            txn_id: pending.txn_id,
            lamport_ts: ts,
        })
    }

    /// Check if the pending transaction has timed out.
    pub fn check_timeout(&mut self, current_tick: Tick) -> Option<TwoPhaseOutcome> {
        let pending = self.pending.as_mut()?;
        if pending.outcome != TwoPhaseOutcome::Pending {
            return None;
        }
        if current_tick.saturating_sub(pending.sent_tick) >= PREPARE_TIMEOUT_TICKS {
            pending.outcome = TwoPhaseOutcome::Aborted;
            return Some(TwoPhaseOutcome::Aborted);
        }
        None
    }

    // ── Incoming PREPARE from other agents ──────────────────────────────────

    /// Handle a PREPARE from another agent. Checks if the requested locks
    /// conflict with our committed or pending locks.
    ///
    /// Returns ACK_PROMISE to send back.
    pub fn handle_prepare(&mut self, prepare: &PreparePayload) -> AckPromise {
        self.clock.receive(prepare.lamport_ts);

        // Check for conflicts against our committed locks
        let conflicts = self.has_conflict(&prepare.locks, prepare.txn_id.agent_id);

        let ts = self.clock.send_timestamp();

        if !conflicts {
            // Store as prepared (we promise not to lock these cells)
            self.foreign_prepared.insert(
                prepare.txn_id,
                prepare.locks.clone(),
            );
        }

        AckPromise {
            txn_id: prepare.txn_id,
            responder_id: self.agent_id(),
            lamport_ts: ts,
            accepted: !conflicts,
        }
    }

    /// Handle a COMMIT from another agent. Convert prepared locks to committed.
    pub fn handle_commit(&mut self, commit: &CommitPayload) {
        self.clock.receive(commit.lamport_ts);

        if let Some(prepared) = self.foreign_prepared.remove(&commit.txn_id) {
            let locks: Vec<CellLock> = prepared.iter().map(|lr| {
                CellLock {
                    cell: lr.cell,
                    interval: SafeInterval::new(lr.t_start, lr.t_end),
                    agent_id: commit.txn_id.agent_id,
                    txn_id: commit.txn_id,
                }
            }).collect();
            self.foreign_locks.insert(commit.txn_id, locks);
        }
    }

    /// Handle an ABORT from another agent. Release prepared locks.
    pub fn handle_abort(&mut self, abort_payload: &AbortPayload) {
        self.clock.receive(abort_payload.lamport_ts);
        self.foreign_prepared.remove(&abort_payload.txn_id);
    }

    // ── Conflict Detection ──────────────────────────────────────────────────

    /// Check if any requested lock conflicts with existing committed locks.
    fn has_conflict(&self, requests: &[CellLockRequest], requesting_agent: u8) -> bool {
        for req in requests {
            let req_interval = SafeInterval::new(req.t_start, req.t_end);

            // Check own committed locks
            for locks in self.own_committed.values() {
                for lock in locks {
                    if lock.cell == req.cell && lock.interval.overlaps(&req_interval) {
                        return true;
                    }
                }
            }

            // Check foreign committed locks (skip requesting agent's own)
            for (txn_id, locks) in &self.foreign_locks {
                if txn_id.agent_id == requesting_agent {
                    continue;
                }
                for lock in locks {
                    if lock.cell == req.cell && lock.interval.overlaps(&req_interval) {
                        return true;
                    }
                }
            }

            // Check foreign prepared locks (promises we made to others)
            for (txn_id, prepared) in &self.foreign_prepared {
                if txn_id.agent_id == requesting_agent {
                    continue;
                }
                for prep in prepared {
                    if prep.cell == req.cell {
                        let prep_interval = SafeInterval::new(prep.t_start, prep.t_end);
                        if prep_interval.overlaps(&req_interval) {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Release all locks for a completed trajectory (agent reached goal)
    pub fn release_own_locks(&mut self, txn_id: &TransactionId) {
        self.own_committed.remove(txn_id);
    }

    /// Release all locks for this agent (emergency: battery abort, replan)
    pub fn release_all_own_locks(&mut self) {
        self.own_committed.clear();
    }

    /// Garbage-collect expired foreign locks
    pub fn gc(&mut self, current_tick: Tick) {
        // Remove committed locks whose intervals have all expired
        self.foreign_locks.retain(|_, locks| {
            locks.retain(|l| l.interval.t_end > current_tick);
            !locks.is_empty()
        });

        // Remove stale prepared locks (older than 2× timeout)
        self.foreign_prepared.retain(|_, reqs| {
            reqs.retain(|r| r.t_end > current_tick);
            !reqs.is_empty()
        });
    }

    /// Compute cells within LOCK_RADIUS Manhattan distance of a given cell.
    /// Used to determine which peers to send PREPARE to.
    pub fn cells_in_radius(&self, center: CellID) -> Vec<CellID> {
        let cx = (center % self.grid_width) as i32;
        let cy = (center / self.grid_width) as i32;
        let r = LOCK_RADIUS as i32;
        let mut cells = Vec::new();

        for dy in -r..=r {
            let remain = r - dy.abs();
            for dx in -remain..=remain {
                let nx = cx + dx;
                let ny = cy + dy;
                if nx >= 0 && nx < self.grid_width as i32
                    && ny >= 0 && ny < self.grid_height as i32
                {
                    cells.push((ny * self.grid_width as i32 + nx) as CellID);
                }
            }
        }
        cells
    }

    /// Check if a peer agent is within LOCK_RADIUS of any cell in the trajectory.
    pub fn is_peer_in_radius(&self, peer_cell: CellID, trajectory_cells: &[CellID]) -> bool {
        let px = (peer_cell % self.grid_width) as i32;
        let py = (peer_cell / self.grid_width) as i32;

        for &cell in trajectory_cells {
            let cx = (cell % self.grid_width) as i32;
            let cy = (cell / self.grid_width) as i32;
            let dist = (px - cx).abs() + (py - cy).abs();
            if dist <= LOCK_RADIUS as i32 {
                return true;
            }
        }
        false
    }

    /// Is there a pending transaction?
    pub fn has_pending(&self) -> bool {
        self.pending.is_some()
    }

    /// Get pending transaction outcome
    pub fn pending_outcome(&self) -> Option<TwoPhaseOutcome> {
        self.pending.as_ref().map(|p| p.outcome)
    }

    /// Count committed own locks
    pub fn own_lock_count(&self) -> usize {
        self.own_committed.values().map(|v| v.len()).sum()
    }

    /// Count committed foreign locks
    pub fn foreign_lock_count(&self) -> usize {
        self.foreign_locks.values().map(|v| v.len()).sum()
    }
}

// ─── Wire Serialization ────────────────────────────────────────────────────────

/// PREPARE payload wire format:
/// [msg_type: u8 = 0x10] [agent_id: u8] [lamport_ts: u64] [lock_count: u8]
/// [locks: CellLockRequest × lock_count]
impl PreparePayload {
    pub fn to_bytes(&self) -> Vec<u8> {
        let lock_count = self.locks.len().min(255) as u8;
        let mut buf = Vec::with_capacity(12 + lock_count as usize * 10);
        buf.push(0x10); // msg_type
        buf.push(self.txn_id.agent_id);
        buf.extend_from_slice(&self.lamport_ts.to_le_bytes());
        buf.push(lock_count);
        buf.extend_from_slice(&self.sent_tick.to_le_bytes());
        for lock in self.locks.iter().take(lock_count as usize) {
            buf.extend_from_slice(&lock.to_bytes());
        }
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 16 || data[0] != 0x10 {
            return None;
        }
        let agent_id = data[1];
        let lamport_ts = u64::from_le_bytes([
            data[2], data[3], data[4], data[5],
            data[6], data[7], data[8], data[9],
        ]);
        let lock_count = data[10] as usize;
        let sent_tick = u32::from_le_bytes([data[11], data[12], data[13], data[14]]);

        let mut locks = Vec::with_capacity(lock_count);
        for i in 0..lock_count {
            let offset = 15 + i * 10;
            if offset + 10 > data.len() { break; }
            if let Some(lr) = CellLockRequest::from_bytes(&data[offset..offset + 10]) {
                locks.push(lr);
            }
        }

        Some(Self {
            txn_id: TransactionId::new(agent_id, lamport_ts),
            lamport_ts,
            locks,
            sent_tick,
        })
    }
}

/// ACK_PROMISE wire format:
/// [msg_type: u8 = 0x11] [txn_agent: u8] [txn_ts: u64] [responder: u8] [lamport: u64] [accepted: u8]
impl AckPromise {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(20);
        buf.push(0x11);
        buf.push(self.txn_id.agent_id);
        buf.extend_from_slice(&self.txn_id.timestamp.to_le_bytes());
        buf.push(self.responder_id);
        buf.extend_from_slice(&self.lamport_ts.to_le_bytes());
        buf.push(if self.accepted { 1 } else { 0 });
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 20 || data[0] != 0x11 {
            return None;
        }
        let txn_agent = data[1];
        let txn_ts = u64::from_le_bytes([
            data[2], data[3], data[4], data[5],
            data[6], data[7], data[8], data[9],
        ]);
        let responder_id = data[10];
        let lamport_ts = u64::from_le_bytes([
            data[11], data[12], data[13], data[14],
            data[15], data[16], data[17], data[18],
        ]);
        let accepted = data[19] != 0;

        Some(Self {
            txn_id: TransactionId::new(txn_agent, txn_ts),
            responder_id,
            lamport_ts,
            accepted,
        })
    }
}

/// COMMIT wire format:
/// [msg_type: u8 = 0x12] [txn_agent: u8] [txn_ts: u64] [lamport: u64]
impl CommitPayload {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(18);
        buf.push(0x12);
        buf.push(self.txn_id.agent_id);
        buf.extend_from_slice(&self.txn_id.timestamp.to_le_bytes());
        buf.extend_from_slice(&self.lamport_ts.to_le_bytes());
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 18 || data[0] != 0x12 {
            return None;
        }
        Some(Self {
            txn_id: TransactionId::new(
                data[1],
                u64::from_le_bytes([
                    data[2], data[3], data[4], data[5],
                    data[6], data[7], data[8], data[9],
                ]),
            ),
            lamport_ts: u64::from_le_bytes([
                data[10], data[11], data[12], data[13],
                data[14], data[15], data[16], data[17],
            ]),
        })
    }
}

/// ABORT wire format: identical structure to COMMIT with msg_type = 0x13
impl AbortPayload {
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut buf = Vec::with_capacity(18);
        buf.push(0x13);
        buf.push(self.txn_id.agent_id);
        buf.extend_from_slice(&self.txn_id.timestamp.to_le_bytes());
        buf.extend_from_slice(&self.lamport_ts.to_le_bytes());
        buf
    }

    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < 18 || data[0] != 0x13 {
            return None;
        }
        Some(Self {
            txn_id: TransactionId::new(
                data[1],
                u64::from_le_bytes([
                    data[2], data[3], data[4], data[5],
                    data[6], data[7], data[8], data[9],
                ]),
            ),
            lamport_ts: u64::from_le_bytes([
                data[10], data[11], data[12], data[13],
                data[14], data[15], data[16], data[17],
            ]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lamport_clock_tick() {
        let mut clock = LamportClock::new(1);
        assert_eq!(clock.current(), 0);
        let t1 = clock.tick();
        assert_eq!(t1, 1);
        let t2 = clock.tick();
        assert_eq!(t2, 2);
    }

    #[test]
    fn test_lamport_clock_send_receive() {
        let mut clock_a = LamportClock::new(1);
        let mut clock_b = LamportClock::new(2);

        let ts_a = clock_a.send_timestamp(); // clock_a = 1
        assert_eq!(ts_a, 1);

        clock_b.receive(ts_a); // clock_b = max(0, 1) + 1 = 2
        assert_eq!(clock_b.current(), 2);

        let ts_b = clock_b.send_timestamp(); // clock_b = 3
        assert_eq!(ts_b, 3);

        clock_a.receive(ts_b); // clock_a = max(1, 3) + 1 = 4
        assert_eq!(clock_a.current(), 4);
    }

    #[test]
    fn test_lamport_total_ordering() {
        use std::cmp::Ordering;
        // Same timestamp, different agents → tiebreak on agent_id
        assert_eq!(LamportClock::order(5, 1, 5, 2), Ordering::Less);
        assert_eq!(LamportClock::order(5, 2, 5, 1), Ordering::Greater);
        // Different timestamps
        assert_eq!(LamportClock::order(3, 1, 5, 1), Ordering::Less);
        assert_eq!(LamportClock::order(7, 2, 5, 1), Ordering::Greater);
    }

    #[test]
    fn test_transaction_id_uniqueness() {
        let a = TransactionId::new(1, 100);
        let b = TransactionId::new(1, 101);
        let c = TransactionId::new(2, 100);
        assert_ne!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn test_2pc_full_cycle_commit() {
        // Agent 1 wants to lock cells [5, 6, 7] from tick 10-20
        let mut mgr1 = IntentLockManager::new(1, 10, 10);
        let mut mgr2 = IntentLockManager::new(2, 10, 10);

        let waypoints = vec![
            (5u16, 10u32, 20u32),
            (6, 12, 22),
            (7, 14, 24),
        ];

        // Phase 1: PREPARE
        let prepare = mgr1.prepare(&waypoints, 1, 0).unwrap();
        assert_eq!(prepare.locks.len(), 3);
        assert!(mgr1.has_pending());

        // Agent 2 receives PREPARE and responds with ACK
        let ack = mgr2.handle_prepare(&prepare);
        assert!(ack.accepted);

        // Agent 1 receives ACK
        let outcome = mgr1.receive_ack(ack);
        assert_eq!(outcome, Some(TwoPhaseOutcome::Committed));

        // Phase 2: COMMIT
        let commit = mgr1.commit().unwrap();
        assert!(!mgr1.has_pending());

        // Agent 2 applies commit
        mgr2.handle_commit(&commit);
        assert_eq!(mgr2.foreign_lock_count(), 3);
    }

    #[test]
    fn test_2pc_conflict_abort() {
        let mut mgr1 = IntentLockManager::new(1, 10, 10);
        let mut mgr2 = IntentLockManager::new(2, 10, 10);

        // Agent 2 already has committed locks on cell 5
        let wp2 = vec![(5u16, 10u32, 20u32)];
        let prep2 = mgr2.prepare(&wp2, 0, 0).unwrap();
        mgr2.commit().unwrap();

        // Agent 1 tries to lock cell 5 (same interval)
        let wp1 = vec![(5u16, 12u32, 18u32)];
        let prepare = mgr1.prepare(&wp1, 1, 0).unwrap();

        // Agent 2 receives PREPARE — conflict!
        let ack = mgr2.handle_prepare(&prepare);
        assert!(!ack.accepted);

        // Agent 1 receives NACK → abort
        let outcome = mgr1.receive_ack(ack);
        assert_eq!(outcome, Some(TwoPhaseOutcome::Aborted));

        let abort = mgr1.abort().unwrap();
        mgr2.handle_abort(&abort);
    }

    #[test]
    fn test_2pc_timeout() {
        let mut mgr = IntentLockManager::new(1, 10, 10);
        let waypoints = vec![(5u16, 10u32, 20u32)];

        mgr.prepare(&waypoints, 1, 0).unwrap();

        // Not timed out yet
        assert!(mgr.check_timeout(5).is_none());

        // Timed out
        let outcome = mgr.check_timeout(PREPARE_TIMEOUT_TICKS + 1);
        assert_eq!(outcome, Some(TwoPhaseOutcome::Aborted));
    }

    #[test]
    fn test_cells_in_radius() {
        let mgr = IntentLockManager::new(1, 32, 32);
        let cells = mgr.cells_in_radius(16 * 32 + 16); // center of 32×32 grid
        // Diamond with radius 15: area = 2*15² + 2*15 + 1 = 481
        assert!(cells.len() > 400);
        assert!(cells.len() <= 481);
    }

    #[test]
    fn test_peer_in_radius() {
        let mgr = IntentLockManager::new(1, 32, 32);
        // Cell 0 and cell 10 : Manhattan distance = 10 < 15 → in radius
        assert!(mgr.is_peer_in_radius(10, &[0]));
        // Cell 0 and cell 500: distance > 15 → out of radius
        assert!(!mgr.is_peer_in_radius(500, &[0]));
    }

    #[test]
    fn test_prepare_roundtrip() {
        let payload = PreparePayload {
            txn_id: TransactionId::new(3, 42),
            lamport_ts: 42,
            locks: vec![
                CellLockRequest { cell: 100, t_start: 10, t_end: 20 },
                CellLockRequest { cell: 200, t_start: 15, t_end: 25 },
            ],
            sent_tick: 5,
        };
        let bytes = payload.to_bytes();
        let recovered = PreparePayload::from_bytes(&bytes).unwrap();
        assert_eq!(recovered.txn_id.agent_id, 3);
        assert_eq!(recovered.locks.len(), 2);
        assert_eq!(recovered.locks[0].cell, 100);
        assert_eq!(recovered.locks[1].t_end, 25);
    }

    #[test]
    fn test_ack_roundtrip() {
        let ack = AckPromise {
            txn_id: TransactionId::new(1, 10),
            responder_id: 2,
            lamport_ts: 15,
            accepted: true,
        };
        let bytes = ack.to_bytes();
        let recovered = AckPromise::from_bytes(&bytes).unwrap();
        assert_eq!(recovered.txn_id.agent_id, 1);
        assert_eq!(recovered.responder_id, 2);
        assert!(recovered.accepted);
    }

    #[test]
    fn test_commit_roundtrip() {
        let commit = CommitPayload {
            txn_id: TransactionId::new(5, 99),
            lamport_ts: 100,
        };
        let bytes = commit.to_bytes();
        let recovered = CommitPayload::from_bytes(&bytes).unwrap();
        assert_eq!(recovered.txn_id.agent_id, 5);
        assert_eq!(recovered.txn_id.timestamp, 99);
    }

    #[test]
    fn test_gc_expired_locks() {
        let mut mgr1 = IntentLockManager::new(1, 10, 10);
        let mut mgr2 = IntentLockManager::new(2, 10, 10);

        // Agent 2 commits locks at tick 10-20
        let wp = vec![(5u16, 10u32, 20u32)];
        let prep = mgr2.prepare(&wp, 0, 0).unwrap();
        let commit = mgr2.commit().unwrap();

        // Agent 1 learns about agent 2's locks
        mgr1.handle_prepare(&PreparePayload {
            txn_id: commit.txn_id,
            lamport_ts: 1,
            locks: wp.iter().map(|&(c, s, e)| CellLockRequest { cell: c, t_start: s, t_end: e }).collect(),
            sent_tick: 0,
        });
        mgr1.handle_commit(&commit);
        assert_eq!(mgr1.foreign_lock_count(), 1);

        // GC at tick 25 → expired
        mgr1.gc(25);
        assert_eq!(mgr1.foreign_lock_count(), 0);
    }

    #[test]
    fn test_release_own_locks() {
        let mut mgr = IntentLockManager::new(1, 10, 10);
        let wp = vec![(5u16, 10u32, 20u32)];
        let prep = mgr.prepare(&wp, 0, 0).unwrap();
        let commit = mgr.commit().unwrap();
        assert_eq!(mgr.own_lock_count(), 1);

        mgr.release_own_locks(&commit.txn_id);
        assert_eq!(mgr.own_lock_count(), 0);
    }

    #[test]
    fn test_no_double_prepare() {
        let mut mgr = IntentLockManager::new(1, 10, 10);
        let wp = vec![(5u16, 10u32, 20u32)];
        assert!(mgr.prepare(&wp, 1, 0).is_some());
        // Second prepare while first is pending → None
        assert!(mgr.prepare(&wp, 1, 0).is_none());
    }
}
