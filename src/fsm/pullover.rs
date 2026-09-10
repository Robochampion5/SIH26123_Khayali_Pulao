use crate::space_time::{CellID, Heading};
use crate::planner::graph::SpatialGraph;

/// Find a pull-over cell: a side cell adjacent to `current` that is traversable
/// and NOT in the direction of travel (heading).
///
/// Pull-over cells allow a yielding agent to step aside so higher-priority
/// agents can pass without waiting.
///
/// Returns None if no suitable side cell exists (narrow corridor).
pub fn find_pullover_cell(
    current: CellID,
    heading: Heading,
    graph: &SpatialGraph,
) -> Option<CellID> {
    let (cx, cy) = graph.coordinate(current);
    let cx = cx as i32;
    let cy = cy as i32;

    // Side directions relative to heading
    let side_offsets: [(i32, i32); 2] = match heading {
        Heading::N | Heading::S => [(-1, 0), (1, 0)], // left/right
        Heading::E | Heading::W => [(0, -1), (0, 1)], // up/down
    };

    for &(dx, dy) in &side_offsets {
        let nx = cx + dx;
        let ny = cy + dy;
        if nx >= 0 && nx < graph.width as i32 && ny >= 0 && ny < graph.height as i32 {
            let side_cell = graph.cell_from_coord(nx as u16, ny as u16);
            if graph.is_traversable(side_cell) {
                return Some(side_cell);
            }
        }
    }

    None
}

/// Check if a cell is an intersection (3+ traversable neighbors).
/// Intersections are poor pull-over locations because they block cross-traffic.
pub fn is_intersection(cell: CellID, graph: &SpatialGraph) -> bool {
    graph.neighbors(cell).len() >= 3
}

/// Score a candidate pull-over cell. Lower is better.
/// Penalty for: intersection, distance from corridor center, blocked neighbors.
pub fn pullover_score(cell: CellID, graph: &SpatialGraph) -> u16 {
    let mut score: u16 = 0;
    let n = graph.neighbors(cell).len();

    if n >= 3 {
        score += 100; // intersection penalty
    }
    if n == 1 {
        score += 10; // dead-end: okay for pull-over but limits escape routes
    }
    // Prefer cells with exactly 2 neighbors (corridor side)

    score
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pullover_open_grid() {
        let graph = SpatialGraph::new(5, 5);
        // Cell 6 = (1,1), heading E → side cells are (1,0)=1 and (1,2)=11
        let cell = find_pullover_cell(6, Heading::E, &graph);
        assert!(cell.is_some());
        let c = cell.unwrap();
        assert!(c == 1 || c == 11);
    }

    #[test]
    fn test_pullover_narrow_corridor() {
        let mut graph = SpatialGraph::new(3, 1);
        // 3×1 grid, heading E at cell 1. No side cells.
        let cell = find_pullover_cell(1, Heading::E, &graph);
        assert!(cell.is_none());
    }

    #[test]
    fn test_intersection_detection() {
        let graph = SpatialGraph::new(5, 5);
        // Cell 6 = (1,1) has 4 neighbors in an open grid
        assert!(is_intersection(6, &graph));
        // Corner cell 0 = (0,0) has 2 neighbors
        assert!(!is_intersection(0, &graph));
    }

    #[test]
    fn test_pullover_score() {
        let graph = SpatialGraph::new(5, 5);
        let score_corner = pullover_score(0, &graph);     // 2 neighbors
        let score_center = pullover_score(6, &graph);      // 4 neighbors (intersection)
        assert!(score_corner < score_center);
    }
}
