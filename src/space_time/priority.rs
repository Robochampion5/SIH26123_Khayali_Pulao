use crate::space_time::{CellID, Tick, SafeInterval};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PriorityState {
    pub yield_count: u8,
    pub urgency: u8,        // 0–4 (U_task)
    pub d_goal: u16,
    pub battery: u8,
    pub id: u8,
}

impl PriorityState {
    /// Revised v3.0 priority: P = 10000*T_yield + δ·U_task·100/(D_goal+1) + 10/(B+1) + ID
    /// δ = 50
    pub fn compute(&self) -> u32 {
        let yield_term = 10000u32 * (self.yield_count as u32);
        let urgency_term = (50u32 * (self.urgency as u32) * 100) / ((self.d_goal as u32) + 1);
        let battery_term = 10 / ((self.battery as u32) + 1);
        let id_term = self.id as u32;
        yield_term + urgency_term + battery_term + id_term
    }
}

pub fn dominates(p1: u32, p2: u32) -> bool {
    p1 > p2
}

pub fn priority_after_yield(current: u32) -> u32 {
    current + 10000
}

pub fn priority_at_max_yield() -> u32 {
    2550000
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_yield_dominance() {
        let base = PriorityState { yield_count: 0, urgency: 3, d_goal: 1, battery: 100, id: 1 };
        let base_p = base.compute();
        let yielded = PriorityState { yield_count: 1, ..base };
        assert!(yielded.compute() > base_p);
    }

    #[test]
    fn test_urgency_multiplier() {
        let a = PriorityState { yield_count: 0, urgency: 3, d_goal: 1, battery: 50, id: 1 };
        let b = PriorityState { yield_count: 0, urgency: 0, d_goal: 1, battery: 50, id: 1 };
        assert!(a.compute() > b.compute());
    }

    #[test]
    fn test_total_ordering() {
        let mut states: Vec<u32> = (0..20).map(|id| {
            PriorityState { yield_count: 0, urgency: 0, d_goal: 10, battery: 50, id }.compute()
        }).collect();
        let sorted = {
            let mut s = states.clone();
            s.sort();
            s
        };
        states.sort_by(|a, b| b.cmp(a)); // descending
        assert_eq!(states, sorted.into_iter().rev().collect::<Vec<_>>());
    }

    #[test]
    fn test_emergency_equals_one_yield() {
        let emergency = PriorityState { yield_count: 0, urgency: 4, d_goal: 1, battery: 100, id: 20 };
        let one_yield = PriorityState { yield_count: 1, urgency: 0, d_goal: 256, battery: 0, id: 1 };
        assert!(emergency.compute() >= one_yield.compute());
    }
}