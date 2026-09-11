use crate::space_time::CellID;
use std::collections::HashSet;
use std::fs;

pub const GRID_WIDTH: u16 = 256;
pub const GRID_HEIGHT: u16 = 256;
pub const MAX_CELLS: usize = (GRID_WIDTH as usize) * (GRID_HEIGHT as usize);

#[derive(Debug, Clone)]
pub struct SpatialGraph {
    pub width: u16,
    pub height: u16,
    pub blocked: HashSet<CellID>,
    pub connectivity: u8, // 4 or 8
}

impl SpatialGraph {
    pub fn new(width: u16, height: u16) -> Self {
        Self {
            width,
            height,
            blocked: HashSet::new(),
            connectivity: 4,
        }
    }

    pub fn from_json(path: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let content = fs::read_to_string(path)?;
        let map: WarehouseMap = serde_json::from_str(&content)?;

        let mut graph = Self::new(map.width, map.height);
        graph.connectivity = map.connectivity.unwrap_or(4);
        for cell in map.blocked_cells {
            graph.blocked.insert(cell);
        }
        Ok(graph)
    }

    pub fn is_traversable(&self, cell: CellID) -> bool {
        !self.blocked.contains(&cell) && self.cell_valid(cell)
    }

    fn cell_valid(&self, cell: CellID) -> bool {
        let x = cell as u32 % self.width as u32;
        let y = cell as u32 / self.width as u32;
        x < self.width as u32 && y < self.height as u32
    }

    pub fn neighbors(&self, cell: CellID) -> Vec<CellID> {
        let mut result = Vec::with_capacity(8);
        let x = cell as i32 % self.width as i32;
        let y = cell as i32 / self.width as i32;

        let dirs_4 = [(0, -1), (1, 0), (0, 1), (-1, 0)];
        let dirs_8 = [
            (0, -1), (1, -1), (1, 0), (1, 1),
            (0, 1), (-1, 1), (-1, 0), (-1, -1),
        ];

        let dirs = if self.connectivity == 8 { &dirs_8[..] } else { &dirs_4[..] };

        for &(dx, dy) in dirs {
            let nx = x + dx;
            let ny = y + dy;
            if nx >= 0 && nx < self.width as i32 && ny >= 0 && ny < self.height as i32 {
                let n_cell = (ny * self.width as i32 + nx) as CellID;
                if self.is_traversable(n_cell) {
                    result.push(n_cell);
                }
            }
        }
        result
    }

    pub fn manhattan_distance(&self, a: CellID, b: CellID) -> u16 {
        let (ax, ay) = self.coordinate(a);
        let (bx, by) = self.coordinate(b);
        ((ax as i32 - bx as i32).abs() + (ay as i32 - by as i32).abs()) as u16
    }

    pub fn coordinate(&self, cell: CellID) -> (u16, u16) {
        let x = (cell as u32 % self.width as u32) as u16;
        let y = (cell as u32 / self.width as u32) as u16;
        (x, y)
    }

    pub fn cell_from_coord(&self, x: u16, y: u16) -> CellID {
        (y as u32 * self.width as u32 + x as u32) as CellID
    }
}

#[derive(Debug, serde::Deserialize)]
struct WarehouseMap {
    width: u16,
    height: u16,
    blocked_cells: Vec<CellID>,
    connectivity: Option<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_graph_creation() {
        let graph = SpatialGraph::new(10, 10);
        assert_eq!(graph.width, 10);
        assert_eq!(graph.height, 10);
    }

    #[test]
    fn test_neighbors_4_connected() {
        let graph = SpatialGraph::new(5, 5);
        let neighbors = graph.neighbors(6); // cell (1,1)
        assert_eq!(neighbors.len(), 4);
        assert!(neighbors.contains(&1));  // (1,0)
        assert!(neighbors.contains(&11)); // (1,2)
        assert!(neighbors.contains(&7));  // (2,1)
        assert!(neighbors.contains(&5));  // (0,1)
    }

    #[test]
    fn test_blocked_cell() {
        let mut graph = SpatialGraph::new(5, 5);
        graph.blocked.insert(6);
        assert!(!graph.is_traversable(6));
        let neighbors = graph.neighbors(1); // (1,0)
        assert!(!neighbors.contains(&6));
    }

    #[test]
    fn test_manhattan_distance() {
        let graph = SpatialGraph::new(10, 10);
        assert_eq!(graph.manhattan_distance(0, 11), 2); // (0,0) to (1,1)
        assert_eq!(graph.manhattan_distance(5, 25), 3); // (5,0) to (5,2)
    }
}