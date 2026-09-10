use crate::space_time::{CellID, Tick, SafeInterval, Heading};

pub const OMEGA_MAX: f32 = std::f32::consts::PI; // rad/s
pub const DT: f32 = 0.1; // 100ms
pub const T_90: u8 = ((std::f32::consts::FRAC_PI_2) / (OMEGA_MAX * DT)).ceil() as u8; // 2 ticks

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct KinematicState {
    pub cell: CellID,
    pub heading: Heading,
    pub interval: SafeInterval,
}

impl KinematicState {
    pub fn new(cell: CellID, heading: Heading, interval: SafeInterval) -> Self {
        Self { cell, heading, interval }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct HeadingLookup {
    pub rot_cost: [[u8; 4]; 4], // rot_cost[from][to] in ticks
}

impl HeadingLookup {
    pub fn new() -> Self {
        let mut rot_cost = [[0u8; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let diff = (j as i8 - i as i8).abs();
                let diff = diff.min(4 - diff);
                rot_cost[i][j] = ((diff as f32 * std::f32::consts::FRAC_PI_2) / (OMEGA_MAX * DT)).ceil() as u8;
            }
        }
        Self { rot_cost }
    }

    pub fn rotation_ticks(&self, from: Heading, to: Heading) -> u8 {
        self.rot_cost[from as usize][to as usize]
    }

    pub fn heading_to_theta(&self, heading: Heading) -> f32 {
        match heading {
            Heading::N => 0.0,
            Heading::E => std::f32::consts::FRAC_PI_2,
            Heading::S => std::f32::consts::PI,
            Heading::W => -std::f32::consts::FRAC_PI_2,
        }
    }
}

impl Default for HeadingLookup {
    fn default() -> Self {
        Self::new()
    }
}

pub fn expand_successors(
    state: &KinematicState,
    lookup: &HeadingLookup,
    graph: &crate::planner::graph::SpatialGraph,
) -> Vec<KinematicState> {
    let mut successors = Vec::new();
    let cell = state.cell;
    let current_heading = state.heading;

    let neighbors = graph.neighbors(cell);
    for &next_cell in &neighbors {
        let required_heading = direction_to_heading(cell, next_cell);
        let rot_ticks = lookup.rotation_ticks(current_heading, required_heading);

        let t_depart = state.interval.t_start + rot_ticks as Tick;
        let t_arrive = t_depart + 1; // 1 tick traversal

        if t_arrive <= state.interval.t_end {
            successors.push(KinematicState::new(
                next_cell,
                required_heading,
                SafeInterval { t_start: t_arrive, t_end: t_arrive + 1 },
            ));
        }
    }

    // Wait action (stay in place)
    if state.interval.t_start + 1 <= state.interval.t_end {
        successors.push(KinematicState::new(
            cell,
            current_heading,
            SafeInterval { t_start: state.interval.t_start + 1, t_end: state.interval.t_end },
        ));
    }

    successors
}

pub fn transition_cost(
    from_cell: CellID,
    to_cell: CellID,
    current_heading: Heading,
    required_heading: Heading,
    lookup: &HeadingLookup,
) -> u8 {
    if from_cell == to_cell {
        1 // WAIT
    } else {
        lookup.rotation_ticks(current_heading, required_heading) + 1 // rotation + move
    }
}

pub fn direction_to_heading(from: CellID, to: CellID) -> Heading {
    let width = crate::planner::graph::GRID_WIDTH as i32;
    let from_x = from as i32 % width;
    let from_y = from as i32 / width;
    let to_x = to as i32 % width;
    let to_y = to as i32 / width;

    match (to_x - from_x, to_y - from_y) {
        (0, -1) => Heading::N,
        (1, 0) => Heading::E,
        (0, 1) => Heading::S,
        (-1, 0) => Heading::W,
        _ => Heading::N, // fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::space_time::SafeInterval;

    #[test]
    fn test_rotation_lookup() {
        let lookup = HeadingLookup::new();
        assert_eq!(lookup.rotation_ticks(Heading::N, Heading::N), 0);
        assert_eq!(lookup.rotation_ticks(Heading::N, Heading::E), T_90);
        assert_eq!(lookup.rotation_ticks(Heading::N, Heading::S), 2 * T_90);
        assert_eq!(lookup.rotation_ticks(Heading::N, Heading::W), T_90);
    }

    #[test]
    fn test_heading_to_theta() {
        let lookup = HeadingLookup::new();
        assert_eq!(lookup.heading_to_theta(Heading::N), 0.0);
        assert_eq!(lookup.heading_to_theta(Heading::E), std::f32::consts::FRAC_PI_2);
    }

    #[test]
    fn test_transition_cost() {
        let lookup = HeadingLookup::new();
        assert_eq!(transition_cost(1, 1, Heading::N, Heading::N, &lookup), 1);
        assert_eq!(transition_cost(1, 2, Heading::N, Heading::E, &lookup), T_90 + 1);
    }
}