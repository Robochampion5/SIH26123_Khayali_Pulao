use crate::space_time::{CellID, Tick, DT_SEC};

/// Maximum deceleration m/s²
pub const A_MAX: f32 = 2.0;

/// Maximum operational velocity m/s (warehouse AMR)
pub const V_MAX: f32 = 1.0;

/// Cell size in meters (warehouse grid)
pub const CELL_SIZE_M: f32 = 0.5;

/// Compute stopping distance in cells from current velocity
///
/// d_stop = v² / (2 · a_max)
/// Returns the number of cells needed to come to a full stop.
pub fn stopping_distance_cells(v_mps: f32) -> u16 {
    let d_meters = (v_mps * v_mps) / (2.0 * A_MAX);
    let cells = (d_meters / CELL_SIZE_M).ceil() as u16;
    cells
}

/// Compute stopping time in ticks from current velocity
///
/// t_stop = v / a_max
pub fn stopping_time_ticks(v_mps: f32) -> Tick {
    let t_sec = v_mps / A_MAX;
    let ticks = (t_sec / DT_SEC).ceil() as Tick;
    ticks
}

/// Compute the velocity profile for a braking maneuver.
///
/// Returns a Vec of (tick_offset, velocity_mps) pairs from now until stop.
/// Used by the HAL layer to send motor commands during emergency braking.
pub fn braking_profile(v_current: f32) -> Vec<(Tick, f32)> {
    let mut profile = Vec::new();
    let mut v = v_current;
    let mut t: Tick = 0;

    while v > 0.001 {
        profile.push((t, v));
        v -= A_MAX * DT_SEC;
        if v < 0.0 { v = 0.0; }
        t += 1;
    }
    profile.push((t, 0.0));
    profile
}

/// Check if the agent can stop before reaching a given cell distance
pub fn can_stop_before(v_mps: f32, cells_ahead: u16) -> bool {
    stopping_distance_cells(v_mps) < cells_ahead
}

/// Compute the "safe following distance" in cells at current velocity.
/// Adds a 1-cell safety margin.
pub fn safe_following_distance(v_mps: f32) -> u16 {
    stopping_distance_cells(v_mps) + 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_stopping_distance_zero_velocity() {
        assert_eq!(stopping_distance_cells(0.0), 0);
    }

    #[test]
    fn test_stopping_distance_max_velocity() {
        // v=1.0, a=2.0 → d = 0.25m → ceil(0.25/0.5) = 1 cell
        assert_eq!(stopping_distance_cells(V_MAX), 1);
    }

    #[test]
    fn test_stopping_time() {
        // v=1.0, a=2.0 → t = 0.5s → ceil(0.5/0.1) = 5 ticks
        assert_eq!(stopping_time_ticks(V_MAX), 5);
    }

    #[test]
    fn test_braking_profile_decreases() {
        let profile = braking_profile(1.0);
        assert!(profile.len() > 1);
        for i in 1..profile.len() {
            assert!(profile[i].1 <= profile[i - 1].1);
        }
        assert_eq!(profile.last().unwrap().1, 0.0);
    }

    #[test]
    fn test_can_stop_before() {
        assert!(can_stop_before(1.0, 5));
        assert!(!can_stop_before(1.0, 0));
    }

    #[test]
    fn test_safe_following() {
        let d = safe_following_distance(1.0);
        assert!(d > stopping_distance_cells(1.0));
    }
}
