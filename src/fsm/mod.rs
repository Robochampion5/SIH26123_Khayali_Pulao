pub mod control_loop;
pub mod braking;
pub mod pullover;

use crate::space_time::{CellID, Tick};
use crate::planner::Trajectory;

/// FSM states for the AMR control loop
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AgentState {
    /// No active task; listening for TASK_AWARD
    Idle,
    /// Running D-SIPP to find a path to goal
    Planning,
    /// Following a trajectory waypoint by waypoint
    Executing,
    /// Pausing for 2 ticks before replanning; priority-based
    Yielding,
    /// Pulling over to a side cell then parking
    YieldingPullover,
    /// Stopped at pull-over cell, waiting for passage
    YieldingParked,
    /// Replanning after yielding, blocking, or pull-over
    Replanning,
    /// Sensor detected unmapped obstacle; replanning around it
    Blocked,
    /// Reversing 1.5m to clear conflict (kinematic reverse)
    KinematicReverse,
    /// Battery < 15%; heading to nearest charger
    Charging,
}

impl AgentState {
    pub fn to_u8(self) -> u8 {
        match self {
            AgentState::Idle             => 0,
            AgentState::Planning         => 1,
            AgentState::Executing        => 2,
            AgentState::Yielding         => 3,
            AgentState::YieldingPullover => 4,
            AgentState::YieldingParked   => 5,
            AgentState::Replanning       => 6,
            AgentState::Blocked          => 7,
            AgentState::KinematicReverse => 9,
            AgentState::Charging         => 8,
        }
    }

    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(AgentState::Idle),
            1 => Some(AgentState::Planning),
            2 => Some(AgentState::Executing),
            3 => Some(AgentState::Yielding),
            4 => Some(AgentState::YieldingPullover),
            5 => Some(AgentState::YieldingParked),
            6 => Some(AgentState::Replanning),
            7 => Some(AgentState::Blocked),
            8 => Some(AgentState::Charging),
            9 => Some(AgentState::KinematicReverse),
            _ => None,
        }
    }
}

impl std::fmt::Display for AgentState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AgentState::Idle             => write!(f, "IDLE"),
            AgentState::Planning         => write!(f, "PLANNING"),
            AgentState::Executing        => write!(f, "EXECUTING"),
            AgentState::Yielding         => write!(f, "YIELDING"),
            AgentState::YieldingPullover => write!(f, "YIELDING_PULLOVER"),
            AgentState::YieldingParked   => write!(f, "YIELDING_PARKED"),
            AgentState::Replanning       => write!(f, "REPLANNING"),
            AgentState::Blocked          => write!(f, "BLOCKED"),
            AgentState::KinematicReverse => write!(f, "KINEMATIC_REVERSE"),
            AgentState::Charging         => write!(f, "CHARGING"),
        }
    }
}

/// Per-agent mutable runtime context used by the control loop
#[derive(Debug)]
pub struct AgentContext {
    pub agent_id: u8,
    pub state: AgentState,

    /// Current cell in the grid
    pub cell: CellID,
    /// Goal cell for current task (None when Idle)
    pub goal_cell: Option<CellID>,
    /// Task identifier of the active task
    pub task_id: Option<u16>,

    /// Active trajectory being followed
    pub trajectory: Option<Trajectory>,
    /// Index of the next waypoint to execute
    pub wp_index: usize,

    /// Monotonic local tick counter
    pub tick: Tick,

    /// Cumulative yield count (resets on goal arrival)
    pub yield_count: u8,
    /// Ticks spent in YIELDING (reset when transitioning out)
    pub yield_wait: u8,
    /// Planning attempts in the current Planning/Replanning cycle
    pub plan_attempts: u8,

    /// Battery percentage 0–100
    pub battery_pct: u8,

    /// Nearest charger cell (loaded from map config)
    pub charger_cell: CellID,
}

impl AgentContext {
    pub fn new(agent_id: u8, start_cell: CellID, charger_cell: CellID) -> Self {
        Self {
            agent_id,
            state: AgentState::Idle,
            cell: start_cell,
            goal_cell: None,
            task_id: None,
            trajectory: None,
            wp_index: 0,
            tick: 0,
            yield_count: 0,
            yield_wait: 0,
            plan_attempts: 0,
            battery_pct: 100,
            charger_cell,
        }
    }

    /// Transition to a new state, logging the change
    pub fn transition(&mut self, next: AgentState) {
        log::debug!(
            "Agent {} FSM: {} → {}  (tick={})",
            self.agent_id, self.state, next, self.tick
        );
        self.state = next;
    }

    /// Returns true if yield_count has crossed the RANDOM_WAIT threshold
    pub fn needs_random_wait(&self) -> bool {
        self.yield_count > 10
    }

    /// Returns true if yield_count has crossed the EMERGENCY_HALT threshold
    pub fn needs_emergency_halt(&self) -> bool {
        self.yield_count > 25
    }

    /// Deterministic jitter based on agent_id and tick (XORshift, no random crate)
    pub fn jitter_ticks(&self, lo: Tick, hi: Tick) -> Tick {
        let seed = (self.agent_id as u32) ^ (self.tick.wrapping_mul(2654435761));
        lo + (seed % (hi - lo + 1))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_display() {
        assert_eq!(AgentState::Idle.to_string(), "IDLE");
        assert_eq!(AgentState::Charging.to_string(), "CHARGING");
    }

    #[test]
    fn test_context_init() {
        let ctx = AgentContext::new(3, 10, 255);
        assert_eq!(ctx.agent_id, 3);
        assert_eq!(ctx.state, AgentState::Idle);
        assert_eq!(ctx.cell, 10);
        assert_eq!(ctx.battery_pct, 100);
    }

    #[test]
    fn test_yield_thresholds() {
        let mut ctx = AgentContext::new(1, 0, 0);
        ctx.yield_count = 10;
        assert!(!ctx.needs_random_wait());
        ctx.yield_count = 11;
        assert!(ctx.needs_random_wait());
        ctx.yield_count = 26;
        assert!(ctx.needs_emergency_halt());
    }

    #[test]
    fn test_jitter_bounded() {
        let ctx = AgentContext::new(7, 100, 0);
        let j = ctx.jitter_ticks(5, 20);
        assert!(j >= 5 && j <= 20);
    }
}
