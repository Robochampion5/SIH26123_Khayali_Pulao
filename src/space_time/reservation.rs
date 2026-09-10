use crate::space_time::{CellID, Tick, SafeInterval};
use parking_lot::RwLock;
use std::collections::HashMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reservation {
    pub agent_id: u8,
    pub t_start: Tick,
    pub t_end: Tick,
    pub trajectory_hash: u32,
}

impl Reservation {
    pub fn new(agent_id: u8, interval: SafeInterval, trajectory_hash: u32) -> Self {
        Self {
            agent_id,
            t_start: interval.t_start,
            t_end: interval.t_end,
            trajectory_hash,
        }
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        !(self.t_end <= other.t_start || other.t_end <= self.t_start)
    }

    pub fn overlaps_interval(&self, interval: &SafeInterval) -> bool {
        !(self.t_end <= interval.t_start || interval.t_end <= self.t_start)
    }

    pub fn contains_tick(&self, tick: Tick) -> bool {
        self.t_start <= tick && tick < self.t_end
    }

    pub fn contains_interval(&self, interval: &SafeInterval) -> bool {
        self.t_start <= interval.t_start && interval.t_end <= self.t_end
    }
}

const MAX_RESERVATIONS_PER_CELL: usize = 32;

#[derive(Debug, Clone)]
pub struct CellReservations {
    reservations: Vec<Reservation>,
}

impl CellReservations {
    pub fn new() -> Self {
        Self {
            reservations: Vec::with_capacity(MAX_RESERVATIONS_PER_CELL),
        }
    }

    pub fn is_free(&self, interval: &SafeInterval) -> bool {
        self.reservations.iter().all(|r| !r.overlaps_interval(interval))
    }

    pub fn insert(&mut self, res: Reservation) -> Result<(), &'static str> {
        if self.reservations.len() >= MAX_RESERVATIONS_PER_CELL {
            self.reservations.sort_by_key(|r| r.t_end);
            self.reservations.remove(0);
        }
        let pos = self.reservations.binary_search_by_key(&res.t_start, |r| r.t_start).unwrap_or_else(|e| e);
        self.reservations.insert(pos, res);
        Ok(())
    }

    pub fn range_query(&self, t_start: Tick, t_end: Tick) -> Vec<&Reservation> {
        self.reservations
            .iter()
            .filter(|r| r.t_start < t_end && r.t_end > t_start)
            .collect()
    }

    pub fn gc(&mut self, current_tick: Tick) {
        self.reservations.retain(|r| r.t_end > current_tick);
    }

    pub fn count(&self) -> usize {
        self.reservations.len()
    }
}

impl Default for CellReservations {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Default)]
pub struct ReservationTable {
    cells: HashMap<CellID, CellReservations>,
}

impl ReservationTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_free(&self, cell: CellID, interval: &SafeInterval) -> bool {
        self.cells
            .get(&cell)
            .map(|cr| cr.is_free(interval))
            .unwrap_or(true)
    }

    pub fn insert(&mut self, cell: CellID, res: Reservation) -> Result<(), &'static str> {
        let entry = self.cells.entry(cell).or_insert_with(CellReservations::new);
        entry.insert(res)
    }

    pub fn insert_interval(&mut self, cell: CellID, agent_id: u8, interval: SafeInterval, trajectory_hash: u32) -> Result<(), &'static str> {
        self.insert(cell, Reservation::new(agent_id, interval, trajectory_hash))
    }

    pub fn query(&self, cell: CellID, t_start: Tick, t_end: Tick) -> Vec<&Reservation> {
        self.cells
            .get(&cell)
            .map(|cr| cr.range_query(t_start, t_end))
            .unwrap_or_default()
    }

    pub fn gc(&mut self, current_tick: Tick) {
        self.cells.values_mut().for_each(|cr| cr.gc(current_tick));
        self.cells.retain(|_, cr| !cr.reservations.is_empty());
    }

    pub fn get_cell_reservations(&self, cell: CellID) -> Vec<&Reservation> {
        self.cells
            .get(&cell)
            .map(|cr| cr.reservations.iter().collect())
            .unwrap_or_default()
    }

    pub fn remove_agent_reservations(&mut self, agent_id: u8) {
        for cr in self.cells.values_mut() {
            cr.reservations.retain(|r| r.agent_id != agent_id);
        }
        self.cells.retain(|_, cr| !cr.reservations.is_empty());
    }

    pub fn total_reservations(&self) -> usize {
        self.cells.values().map(|cr| cr.count()).sum()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space_time::SafeInterval;

    #[test]
    fn test_reservation_no_overlap() {
        let mut table = ReservationTable::new();
        let interval1 = SafeInterval { t_start: 10, t_end: 20 };
        let interval2 = SafeInterval { t_start: 20, t_end: 30 };
        table.insert_interval(1, 1, interval1, 0).unwrap();
        table.insert_interval(1, 2, interval2, 0).unwrap();
        assert!(table.is_free(1, &interval1));
        assert!(table.is_free(1, &interval2));
    }

    #[test]
    fn test_reservation_overlap_detected() {
        let mut table = ReservationTable::new();
        let interval1 = SafeInterval { t_start: 10, t_end: 20 };
        let interval2 = SafeInterval { t_start: 15, t_end: 25 };
        table.insert_interval(1, 1, interval1, 0).unwrap();
        assert!(!table.is_free(1, &interval2));
    }

    #[test]
    fn test_range_intersection() {
        let mut table = ReservationTable::new();
        let interval1 = SafeInterval { t_start: 10, t_end: 30 };
        table.insert_interval(1, 1, interval1, 0).unwrap();
        let results = table.query(1, 15, 25);
        assert_eq!(results.len(), 1);
    }

    #[test]
    fn test_gc_prunes_expired() {
        let mut table = ReservationTable::new();
        let interval = SafeInterval { t_start: 10, t_end: 20 };
        table.insert_interval(1, 1, interval, 0).unwrap();
        table.gc(25);
        assert_eq!(table.total_reservations(), 0);
    }

    #[test]
    fn test_capacity_eviction() {
        let mut table = ReservationTable::new();
        for i in 0..33 {
            let interval = SafeInterval { t_start: i as Tick, t_end: (i + 10) as Tick };
            table.insert_interval(1, 1, interval, 0).unwrap();
        }
        assert_eq!(table.get_cell_reservations(1).len(), MAX_RESERVATIONS_PER_CELL);
    }

    #[test]
    fn test_priority_total_ordering() {
        // test in priority.rs
    }

    #[test]
    fn test_priority_yield_dominance() {
        // test in priority.rs
    }
}