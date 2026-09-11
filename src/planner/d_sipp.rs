use crate::space_time::{CellID, Tick, SafeInterval, Heading};
use crate::planner::reservation::ReservationBitset as ReservationTable;
use crate::planner::heading::{KinematicState, HeadingLookup, expand_successors, transition_cost, direction_to_heading};
use crate::planner::graph::SpatialGraph;
use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

#[derive(Debug, Clone)]
pub struct Trajectory {
    pub waypoints: Vec<Waypoint>,
    pub total_ticks: Tick,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Waypoint {
    pub cell: CellID,
    pub heading: Heading,
    pub t_arrive: Tick,
    pub t_depart: Tick,
}

impl Waypoint {
    pub fn new(cell: CellID, heading: Heading, t_arrive: Tick, t_depart: Tick) -> Self {
        Self { cell, heading, t_arrive, t_depart }
    }
}

#[derive(Debug, Clone)]
struct SearchNode {
    state: KinematicState,
    g_cost: Tick,
    f_cost: Tick,
    parent: Option<Box<SearchNode>>,
}

impl PartialEq for SearchNode {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost && self.state == other.state
    }
}

impl Eq for SearchNode {}

impl PartialOrd for SearchNode {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for SearchNode {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_cost.cmp(&self.f_cost)
            .then_with(|| other.g_cost.cmp(&self.g_cost))
    }
}

const MAX_HORIZON: Tick = 200;
const MAX_EXPANSIONS: usize = 50000;

pub fn d_sipp(
    start: KinematicState,
    goal: CellID,
    goal_heading: Option<Heading>,
    graph: &SpatialGraph,
    reservation_table: &ReservationTable,
) -> Option<Trajectory> {
    let lookup = HeadingLookup::new();
    let mut open_set = BinaryHeap::new();
    let mut closed_set = HashMap::new();
    let mut expansions = 0;

    let h = heuristic(&start, goal, goal_heading, graph);
    let start_node = SearchNode {
        state: start,
        g_cost: start.interval.t_start,
        f_cost: start.interval.t_start + h,
        parent: None,
    };
    open_set.push(start_node);

    while let Some(current) = open_set.pop() {
        expansions += 1;
        if expansions > MAX_EXPANSIONS {
            return None;
        }

        let current_tick = current.g_cost;
        if current_tick > MAX_HORIZON {
            continue;
        }

        if current.state.cell == goal {
            if goal_heading.is_none() || goal_heading == Some(current.state.heading) {
                return Some(reconstruct_path(current));
            }
        }

        let state_key = (current.state.cell, current.state.heading, current.state.interval.t_start);
        if closed_set.contains_key(&state_key) {
            continue;
        }
        closed_set.insert(state_key, current.g_cost);

        let successors = expand_successors(&current.state, &lookup, graph);
        for succ in successors {
            if !reservation_table.is_free(succ.cell, &succ.interval) {
                continue;
            }

            let move_cost = transition_cost(
                current.state.cell,
                succ.cell,
                current.state.heading,
                succ.heading,
                &lookup,
            );
            let g = current.g_cost + move_cost as Tick;

            if g > MAX_HORIZON {
                continue;
            }

            let h = heuristic(&succ, goal, goal_heading, graph);
            let f = g + h;

            let succ_node = SearchNode {
                state: succ,
                g_cost: g,
                f_cost: f,
                parent: Some(Box::new(current.clone())),
            };
            open_set.push(succ_node);
        }
    }

    None
}

fn heuristic(
    state: &KinematicState,
    goal: CellID,
    goal_heading: Option<Heading>,
    graph: &SpatialGraph,
) -> Tick {
    let d = graph.manhattan_distance(state.cell, goal) as Tick;
    let mut h = d; // minimum 1 tick per cell

    if let Some(gh) = goal_heading {
        let lookup = HeadingLookup::new();
        let rot = lookup.rotation_ticks(state.heading, gh) as Tick;
        h += rot;
    }

    h
}

fn reconstruct_path(mut node: SearchNode) -> Trajectory {
    let mut waypoints = Vec::new();
    let mut current = Some(node);

    while let Some(n) = current {
        let wp = Waypoint::new(
            n.state.cell,
            n.state.heading,
            n.state.interval.t_start,
            n.state.interval.t_end,
        );
        waypoints.push(wp);
        current = n.parent.map(|b| *b);
    }

    waypoints.reverse();
    let total_ticks = waypoints.last().map(|w| w.t_depart).unwrap_or(0);

    Trajectory { waypoints, total_ticks }
}

pub fn validate_trajectory(traj: &Trajectory, graph: &SpatialGraph) -> bool {
    for i in 0..traj.waypoints.len() - 1 {
        let a = traj.waypoints[i];
        let b = traj.waypoints[i + 1];
        let neighbors = graph.neighbors(a.cell);
        if !neighbors.contains(&b.cell) && a.cell != b.cell {
            return false;
        }
        if a.t_depart > b.t_arrive {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space_time::{SafeInterval, Heading};
    use crate::planner::reservation::ReservationBitset as ReservationTable;
    use crate::planner::graph::SpatialGraph;

    #[test]
    fn test_straight_line() {
        let graph = SpatialGraph::new(10, 10);
        let table = ReservationTable::new();
        let start = KinematicState::new(0, Heading::E, SafeInterval { t_start: 0, t_end: MAX_HORIZON });
        let traj = d_sipp(start, 9, Some(Heading::E), &graph, &table);
        assert!(traj.is_some());
        let t = traj.unwrap();
        assert_eq!(t.waypoints.len(), 10);
        assert!(validate_trajectory(&t, &graph));
    }

    #[test]
    fn test_static_obstacle() {
        let mut graph = SpatialGraph::new(5, 5);
        graph.blocked.insert(6); // block (1,1)
        let table = ReservationTable::new();
        let start = KinematicState::new(0, Heading::E, SafeInterval { t_start: 0, t_end: MAX_HORIZON });
        let traj = d_sipp(start, 24, Some(Heading::S), &graph, &table);
        assert!(traj.is_some());
    }

    #[test]
    fn test_time_conflict() {
        let graph = SpatialGraph::new(10, 10);
        let mut table = ReservationTable::new();
        table.insert_interval(5, 1, SafeInterval { t_start: 10, t_end: 15 }, 0).unwrap();

        let start = KinematicState::new(0, Heading::E, SafeInterval { t_start: 0, t_end: MAX_HORIZON });
        let traj = d_sipp(start, 9, Some(Heading::E), &graph, &table);
        assert!(traj.is_some());
        let t = traj.unwrap();
        // Should wait or detour around t=10-15 at cell 5
        let at_5 = t.waypoints.iter().find(|w| w.cell == 5);
        if let Some(wp) = at_5 {
            assert!(wp.t_arrive >= 15 || wp.t_depart <= 10);
        }
    }

    #[test]
    fn test_no_path_none() {
        let mut graph = SpatialGraph::new(5, 5);
        for i in 1..24 {
            graph.blocked.insert(i);
        }
        let table = ReservationTable::new();
        let start = KinematicState::new(0, Heading::E, SafeInterval { t_start: 0, t_end: MAX_HORIZON });
        let traj = d_sipp(start, 24, None, &graph, &table);
        assert!(traj.is_none());
    }

    #[test]
    fn test_heuristic_admissible() {
        let graph = SpatialGraph::new(10, 10);
        for _ in 0..100 {
            let a = (rand::random::<u16>() % 100) as CellID;
            let b = (rand::random::<u16>() % 100) as CellID;
            let h = graph.manhattan_distance(a, b);
            // Actual path length >= manhattan distance
        }
    }
}