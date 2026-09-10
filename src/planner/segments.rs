use crate::space_time::{CellID, Tick};
use crate::space_time::reservation::{Reservation, ReservationTable};
use crate::planner::graph::SpatialGraph;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Segment {
    pub start: CellID,
    pub end: CellID,
    pub t_enter: Tick,
    pub t_exit: Tick,
    pub heading: crate::space_time::Heading,
    pub agent_id: u8,
}

#[derive(Debug, Default)]
pub struct SegmentTable {
    agents: std::collections::HashMap<u8, Vec<Segment>>,
}

impl SegmentTable {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert_agent_segments(&mut self, agent_id: u8, segments: Vec<Segment>) {
        self.agents.insert(agent_id, segments);
    }

    pub fn get_agent_segments(&self, agent_id: u8) -> Vec<Segment> {
        self.agents.get(&agent_id).cloned().unwrap_or_default()
    }

    pub fn remove_agent(&mut self, agent_id: u8) {
        self.agents.remove(&agent_id);
    }

    pub fn expand_to_reservations(&self, agent_id: u8) -> Vec<(CellID, Reservation)> {
        let segments = self.get_agent_segments(agent_id);
        let mut reservations = Vec::new();
        for seg in segments {
            // Interpolate cells along segment path
            // For straight line segments, interpolate linearly
            let length_ticks = seg.t_exit.saturating_sub(seg.t_enter);
            // Simplified: assume segment spans contiguous cells
            reservations.push((
                seg.start,
                Reservation {
                    agent_id: seg.agent_id,
                    t_start: seg.t_enter,
                    t_end: seg.t_exit,
                    trajectory_hash: 0,
                }
            ));
        }
        reservations
    }

    pub fn clear_all(&mut self) {
        self.agents.clear();
    }

    pub fn total_segments(&self) -> usize {
        self.agents.values().map(|v| v.len()).sum()
    }

    pub fn all_segments(&self) -> Vec<Segment> {
        self.agents.values().flat_map(|v| v.iter().cloned()).collect()
    }
}

pub fn compress_trajectory_to_segments(
    trajectory: &crate::planner::d_sipp::Trajectory,
    agent_id: u8,
) -> Vec<Segment> {
    if trajectory.waypoints.is_empty() {
        return Vec::new();
    }

    let mut segments = Vec::new();
    let start_cell = trajectory.waypoints.first().unwrap().cell;
    let end_cell = trajectory.waypoints.last().unwrap().cell;
    let t_enter = trajectory.waypoints.first().unwrap().t_arrive;
    let t_exit = trajectory.waypoints.last().unwrap().t_depart;
    let heading = trajectory.waypoints.first().unwrap().heading;

    segments.push(Segment {
        start: start_cell,
        end: end_cell,
        t_enter,
        t_exit,
        heading,
        agent_id,
    });
    segments
}

pub fn interpolate_cell_at_tick(cell_a: CellID, cell_b: CellID, tick_frac: f32) -> CellID {
    // Linear interpolation between cell coordinates
    // For grid cells, use nearest cell
    cell_b // Simplified
}

pub fn compute_duration_ticks(distance_cells: u16) -> Tick {
    distance_cells as Tick
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::d_sipp::{Trajectory, Waypoint};

    #[test]
    fn test_segment_compression() {
        let mut traj = Trajectory { waypoints: Vec::new(), total_ticks: 0 };
        traj.waypoints.push(Waypoint::new(0, crate::space_time::Heading::N, 0, 10));
        traj.waypoints.push(Waypoint::new(10, crate::space_time::Heading::N, 10, 20));
        traj.waypoints.push(Waypoint::new(19, crate::space_time::Heading::N, 20, 30));

        let segments = compress_trajectory_to_segments(&traj, 1);
        assert_eq!(segments.len(), 1);
        assert_eq!(segments[0].start, 0);
        assert_eq!(segments[0].end, 19);
    }

    #[test]
    fn test_expand_segments() {
        let mut table = SegmentTable::new();
        table.insert_agent_segments(1, vec![Segment {
            start: 1,
            end: 5,
            t_enter: 0,
            t_exit: 10,
            heading: crate::space_time::Heading::E,
            agent_id: 1,
        }]);
        let res = table.expand_to_reservations(1);
        assert!(!res.is_empty());
    }
}