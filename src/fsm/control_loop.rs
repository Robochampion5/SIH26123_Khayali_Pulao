use crate::space_time::{CellID, Tick, Heading, SafeInterval};
use crate::space_time::priority::PriorityState;
use crate::space_time::reservation::ReservationTable;
use crate::planner::graph::SpatialGraph;
use crate::planner::heading::KinematicState;
use crate::planner::d_sipp;
use crate::fsm::{AgentContext, AgentState};

/// Revised v3.0 flow: EXECUTING → YIELDING_PULLOVER → BLOCKED → KINEMATIC_REVERSE → YIELDING_PARKED → REPLANNING
/// (No standalone YIELDING state; yield is direct pullover)
const MAX_PLAN_ATTEMPTS: u8 = 5;

/// Ticks to wait in YIELDING before transitioning to REPLANNING
const YIELD_WAIT_TICKS: u8 = 2;

/// Battery threshold for forced charging
const BATTERY_LOW_PCT: u8 = 15;

/// GC reservation table every N ticks
const GC_INTERVAL: Tick = 50;

/// Staleness threshold: prune INTENTs older than this many ticks
const STALENESS_THRESHOLD: Tick = 100;

/// Execute one tick of the 10Hz control loop for a single agent.
///
/// This is a *pure logic* tick: it reads sensor/network state already collected
/// and produces motor commands and network broadcasts via the returned `TickOutput`.
/// Actual I/O is performed by the caller.
pub fn tick(
    ctx: &mut AgentContext,
    graph: &SpatialGraph,
    res_table: &mut ReservationTable,
) -> TickOutput {
    let mut output = TickOutput::default();

    // ── Battery check (highest priority override) ──
    if ctx.battery_pct < BATTERY_LOW_PCT && ctx.state != AgentState::Charging {
        output.abort_task = ctx.task_id;
        ctx.goal_cell = Some(ctx.charger_cell);
        ctx.task_id = None;
        ctx.trajectory = None;
        ctx.wp_index = 0;
        ctx.plan_attempts = 0;
        ctx.transition(AgentState::Charging);
        // Fall through to PLANNING logic below via Charging == Planning
    }

    match ctx.state {
        AgentState::Idle => {
            // Nothing to do — waiting for TASK_AWARD
            output.motor_halt = true;
        }

        AgentState::Planning | AgentState::Replanning | AgentState::Charging => {
            output.motor_halt = true;

            if let Some(goal) = ctx.goal_cell {
                let heading = infer_heading(ctx.cell, goal, graph);
                let start_interval = SafeInterval::new(ctx.tick, ctx.tick + 200);
                let ks = KinematicState::new(ctx.cell, heading, start_interval);

                match d_sipp::d_sipp(ks, goal, None, graph, res_table) {
                    Some(traj) => {
                        // Broadcast intent
                        output.broadcast_intent = Some(traj.clone());

                        // Insert own reservations
                        for wp in &traj.waypoints {
                            let _ = res_table.insert_interval(
                                wp.cell,
                                ctx.agent_id,
                                SafeInterval::new(wp.t_arrive, wp.t_depart),
                                0,
                            );
                        }

                        ctx.trajectory = Some(traj);
                        ctx.wp_index = 0;
                        ctx.plan_attempts = 0;
                        ctx.transition(AgentState::Executing);
                    }
                    None => {
                        ctx.plan_attempts += 1;
                        if ctx.plan_attempts > MAX_PLAN_ATTEMPTS {
                            ctx.yield_count = ctx.yield_count.saturating_add(1);
                            ctx.plan_attempts = 0;
                            ctx.transition(AgentState::Yielding);
                        }
                        // else: stay in Planning, try again next tick
                    }
                }
            }
        }

        AgentState::Executing => {
            if let Some(ref traj) = ctx.trajectory.clone() {
                if ctx.wp_index >= traj.waypoints.len() {
                    // Goal reached
                    ctx.yield_count = 0;
                    ctx.trajectory = None;
                    ctx.goal_cell = None;
                    let completed_task = ctx.task_id.take();
                    output.task_complete = completed_task;
                    ctx.transition(AgentState::Idle);
                } else {
                    let next_wp = traj.waypoints[ctx.wp_index];

                    // Check for higher-priority conflict at next waypoint
                    let my_priority = PriorityState {
                        yield_count: ctx.yield_count,
                        urgency: 0,
                        d_goal: ctx.goal_cell.map(|g| graph.manhattan_distance(ctx.cell, g)).unwrap_or(0),
                        battery: ctx.battery_pct,
                        id: ctx.agent_id,
                    }.compute();

                    let conflict = check_conflict(next_wp.cell, next_wp.t_arrive, ctx.agent_id, res_table);
                    if let Some(conflicting_agent) = conflict {
                        // For simplicity, always yield (priority comparison would need
                        // access to the other agent's PriorityState — in a real system
                        // this comes from the INTENT payload)
                        ctx.yield_count = ctx.yield_count.saturating_add(1);
                        res_table.remove_agent_reservations(ctx.agent_id);
                        ctx.transition(AgentState::Yielding);
                    } else {
                        // Advance to next waypoint
                        output.motor_target = Some(MotorTarget {
                            cell: next_wp.cell,
                            heading: next_wp.heading,
                            t_arrive: next_wp.t_arrive,
                        });
                        // Mark arrival (simplified: assume 1-tick transit per waypoint)
                        ctx.cell = next_wp.cell;
                        ctx.wp_index += 1;
                    }
                }
            } else {
                // No trajectory — shouldn't be in Executing
                ctx.transition(AgentState::Idle);
            }
        }

        AgentState::Yielding => {
            output.motor_halt = true;
            ctx.yield_wait += 1;

            if ctx.needs_emergency_halt() {
                // Emergency halt: broadcast halt, wait 50 ticks (5 seconds)
                output.broadcast_halt = true;
                if ctx.yield_wait >= 50 {
                    ctx.yield_wait = 0;
                    ctx.transition(AgentState::Replanning);
                }
            } else if ctx.needs_random_wait() {
                // Random wait: 5–20 ticks
                let wait = ctx.jitter_ticks(5, 20) as u8;
                if ctx.yield_wait >= wait {
                    ctx.yield_wait = 0;
                    ctx.transition(AgentState::Replanning);
                }
            } else if ctx.yield_wait >= YIELD_WAIT_TICKS {
                ctx.yield_wait = 0;
                ctx.transition(AgentState::Replanning);
            }
        }

        AgentState::YieldingPullover => {
            // Move to pull-over cell (simplified: immediate)
            output.motor_halt = true;
            ctx.transition(AgentState::YieldingParked);
        }

        AgentState::YieldingParked => {
            output.motor_halt = true;
            // Wait until the conflicting agent passes, then replan
            ctx.yield_wait += 1;
            if ctx.yield_wait >= 10 {
                ctx.yield_wait = 0;
                ctx.transition(AgentState::Replanning);
            }
        }

        AgentState::Blocked => {
            output.motor_halt = true;
            // Invalidate trajectory past current cell
            ctx.trajectory = None;
            ctx.wp_index = 0;
            ctx.transition(AgentState::Replanning);
        }

        AgentState::KinematicReverse => {
            // Kinematic reverse: back 1.5m (approx 3 ticks) to clear conflict
            output.motor_halt = false; // slow reverse motion allowed
            ctx.yield_wait += 1;
            if ctx.yield_wait >= 3 {
                ctx.yield_wait = 0;
                ctx.transition(AgentState::YieldingParked);
            }
        }
    }

    // ── Housekeeping ──
    if ctx.tick % GC_INTERVAL == 0 && ctx.tick > 0 {
        res_table.gc(ctx.tick);
    }

    ctx.tick += 1;
    output
}

/// Check if another agent has reserved the given cell at the given tick
fn check_conflict(
    cell: CellID,
    tick: Tick,
    my_id: u8,
    table: &ReservationTable,
) -> Option<u8> {
    let reservations = table.query(cell, tick, tick + 1);
    for r in reservations {
        if r.agent_id != my_id {
            return Some(r.agent_id);
        }
    }
    None
}

/// Infer the initial heading from current position toward goal
fn infer_heading(from: CellID, to: CellID, graph: &SpatialGraph) -> Heading {
    let (fx, fy) = graph.coordinate(from);
    let (tx, ty) = graph.coordinate(to);
    let dx = tx as i32 - fx as i32;
    let dy = ty as i32 - fy as i32;

    if dx.abs() >= dy.abs() {
        if dx >= 0 { Heading::E } else { Heading::W }
    } else {
        if dy >= 0 { Heading::S } else { Heading::N }
    }
}

/// Output of a single tick — tells the caller what to do
#[derive(Debug, Default)]
pub struct TickOutput {
    /// Command the motors to halt (velocity = 0)
    pub motor_halt: bool,
    /// Command the motors toward a target cell
    pub motor_target: Option<MotorTarget>,
    /// Broadcast this trajectory as an INTENT packet
    pub broadcast_intent: Option<crate::planner::Trajectory>,
    /// Broadcast an emergency halt to nearby agents
    pub broadcast_halt: bool,
    /// A task was aborted (e.g. low battery)
    pub abort_task: Option<u16>,
    /// A task was completed
    pub task_complete: Option<u16>,
}

/// Motor target for one tick
#[derive(Debug, Clone, Copy)]
pub struct MotorTarget {
    pub cell: CellID,
    pub heading: Heading,
    pub t_arrive: Tick,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::graph::SpatialGraph;

    #[test]
    fn test_idle_stays_idle() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 0, 99);
        let out = tick(&mut ctx, &graph, &mut table);
        assert!(out.motor_halt);
        assert_eq!(ctx.state, AgentState::Idle);
    }

    #[test]
    fn test_planning_finds_path() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 0, 99);
        ctx.goal_cell = Some(9);
        ctx.transition(AgentState::Planning);

        let out = tick(&mut ctx, &graph, &mut table);
        assert_eq!(ctx.state, AgentState::Executing);
        assert!(out.broadcast_intent.is_some());
    }

    #[test]
    fn test_planning_no_path_yields() {
        let mut graph = SpatialGraph::new(3, 3);
        // Block everything except cell 0 and cell 8, with no path between them
        for i in 1..8 {
            graph.blocked.insert(i);
        }
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 0, 8);
        ctx.goal_cell = Some(8);
        ctx.transition(AgentState::Planning);

        // Tick MAX_PLAN_ATTEMPTS + 1 times
        for _ in 0..=MAX_PLAN_ATTEMPTS {
            tick(&mut ctx, &graph, &mut table);
        }
        assert_eq!(ctx.state, AgentState::Yielding);
        assert!(ctx.yield_count > 0);
    }

    #[test]
    fn test_yielding_to_replanning() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 0, 99);
        ctx.goal_cell = Some(9);
        ctx.transition(AgentState::Yielding);

        // Tick YIELD_WAIT_TICKS times
        for _ in 0..YIELD_WAIT_TICKS {
            tick(&mut ctx, &graph, &mut table);
        }
        assert_eq!(ctx.state, AgentState::Replanning);
    }

    #[test]
    fn test_battery_override() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 0, 99);
        ctx.goal_cell = Some(9);
        ctx.task_id = Some(42);
        ctx.battery_pct = 10; // below threshold
        ctx.transition(AgentState::Executing);

        let out = tick(&mut ctx, &graph, &mut table);
        assert_eq!(out.abort_task, Some(42));
        // Should be planning toward charger (state = Executing after successful plan,
        // or Charging if plan not yet found)
    }

    #[test]
    fn test_blocked_replans() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        let mut ctx = AgentContext::new(1, 5, 99);
        ctx.goal_cell = Some(9);
        ctx.transition(AgentState::Blocked);

        tick(&mut ctx, &graph, &mut table);
        assert_eq!(ctx.state, AgentState::Replanning);
        assert!(ctx.trajectory.is_none());
    }
}
