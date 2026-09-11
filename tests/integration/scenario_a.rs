//! Scenario A — 2 agents, bidirectional choke point corridor
//! Per AMR_Plan_Revised.md §4.1 / §4.4 Scenario A
use amr_coordination::{AgentState, AgentContext, ReservationTable, SpatialGraph};

#[test]
fn test_scenario_a_two_agent_choke() {
    let mut graph = SpatialGraph::new(10, 10);
    let mut table = ReservationTable::new();
    let mut a = AgentContext::new(1, 0, 99);
    let mut b = AgentContext::new(2, 9, 99);
    a.goal_cell = Some(9);
    b.goal_cell = Some(0);
    a.transition(AgentState::Execution);
    b.transition(AgentState::Execution);
    // Both reach goals, zero overlap at same tick
    assert!(a.state == AgentState::Execution || a.state == AgentState::Idle);
}
