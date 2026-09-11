use std::ops::{BitAnd, BitOrAssign};
use log::debug;
use crate::space_time::{CellID, Tick, SafeInterval};

const GRID_SIZE: usize = 256 * 256;
const TIME_HORIZON_TICKS: usize = 64; // 6.4 seconds at 100ms per tick

/// Reservation entry for tracking which agent has reserved a cell-time slot
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reservation {
    pub agent_id: u8,
    pub t_start: Tick,
    pub t_end: Tick,
    pub trajectory_hash: u64,
}

/// 1D Bitset Reservation Table mapped linearly to 256x256 grid.
/// Each `u64` represents 64 ticks (e.g., 6.4s) into the future.
/// This contiguous block fits easily in memory and is highly L2-cache friendly.
#[derive(Debug, Clone)]
pub struct ReservationBitset {
    // 65536 elements of u64 = 524KB, fits perfectly in most L2 caches
    grid_blocks: Vec<u64>,
    current_tick: u64,
}

impl ReservationBitset {
    pub fn new() -> Self {
        Self {
            grid_blocks: vec![0; GRID_SIZE],
            current_tick: 0,
        }
    }

    /// Advance the simulation tick. This requires rolling the temporal bitset window.
    /// In a purely absolute implementation, we just use absolute time. To avoid O(N) shifts,
    /// we can just use absolute time % 64.
    pub fn advance_to_tick(&mut self, new_tick: u64) {
        let delta = new_tick.saturating_sub(self.current_tick);
        if delta == 0 {
            return;
        }

        if delta >= TIME_HORIZON_TICKS as u64 {
            // Cleared entirely
            self.grid_blocks.fill(0);
        } else {
            // Shift all bits down by `delta`
            // E.g. bit 1 represents +1 tick from current_tick.
            // If delta=1, the new current_tick is old_current_tick+1, so bit 1 becomes bit 0.
            for cell in self.grid_blocks.iter_mut() {
                *cell >>= delta;
            }
        }
        self.current_tick = new_tick;
    }

    #[inline(always)]
    fn cell_index(cell_id: CellID) -> usize {
        cell_id as usize
    }

    /// Convert a tick interval [t_start, t_end) into a bitmask
    #[inline(always)]
    fn interval_to_mask(&self, t_start: Tick, t_end: Tick) -> Option<u64> {
        let relative_start = t_start.saturating_sub(self.current_tick as Tick);
        let relative_end = t_end.saturating_sub(self.current_tick as Tick);

        if relative_start >= 64 {
            // Horizon exceeded
            return None;
        }

        let end = relative_end.min(64);
        if relative_start >= end {
            return None;
        }

        let len = end - relative_start;
        let mut mask = if len == 64 {
            u64::MAX
        } else {
            (1u64 << len) - 1
        };
        mask <<= relative_start;

        Some(mask)
    }

    /// O(1) Bitwise AND conflict checking
    #[inline(always)]
    pub fn is_free(&self, cell_id: CellID, interval: &SafeInterval) -> bool {
        let idx = Self::cell_index(cell_id);
        if let Some(mask) = self.interval_to_mask(interval.t_start, interval.t_end) {
            (self.grid_blocks[idx] & mask) == 0
        } else {
            // Beyond horizon or invalid interval, assume free or handled by fallback
            true
        }
    }

    /// Insert a reservation for an agent
    pub fn insert_interval(&mut self, cell_id: CellID, agent_id: u8, interval: SafeInterval, trajectory_hash: u64) -> Result<(), &'static str> {
        let idx = Self::cell_index(cell_id);
        if let Some(mask) = self.interval_to_mask(interval.t_start, interval.t_end) {
            if (self.grid_blocks[idx] & mask) != 0 {
                return Err("Collision detected");
            }
            self.grid_blocks[idx] |= mask;
            // In a full implementation, we'd also store the reservation metadata
            // For now, we just track occupancy in the bitset
        }
        Ok(())
    }

    /// Remove all reservations for an agent
    pub fn remove_agent_reservations(&mut self, agent_id: u8) {
        // In a full implementation, we would need to track which bits belong to which agent
        // For this bitset implementation, we don't track agent ownership per bit
        // A real implementation would need either:
        // 1. A separate structure to track agent->bits mappings for clean removal
        // 2. Or accept that we can't selectively remove agent reservations from the bitset
        //
        // For now, we'll note that this is a limitation and in practice we'd need
        // a hybrid approach or to accept that agent removal requires clearing and rebuilding
        // Since this is called primarily on yielding, we'll conservatively clear nothing
        // and rely on the GC mechanism to clear old reservations
        // TODO: Implement proper agent-based removal with auxiliary data structures
    }

    /// Query reservations for a cell at a specific time
    pub fn query(&self, cell_id: CellID, t_start: Tick, t_end: Tick) -> Vec<Reservation> {
        // This implementation returns empty vec as we don't track per-agent data in the bitset
        // A full implementation would need to store reservation metadata alongside the bitset
        vec![]
    }

    /// Garbage collect old reservations
    pub fn gc(&mut self, current_tick: Tick) {
        self.advance_to_tick(current_tick as u64);
    }
}