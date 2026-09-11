pub mod priority;

/// CellID: compact cell address in warehouse grid (max 256×256 = 65,536)
pub type CellID = u16;

/// Tick: integer time step. Δt = 100ms. u32 → 49.7 days before overflow.
pub type Tick = u32;

/// Maximum tick value before overflow concern
pub const MAX_TICK: Tick = u32::MAX;

/// Ticks per second
pub const TICKS_PER_SEC: Tick = 10;

/// One tick duration in seconds
pub const DT_SEC: f32 = 0.1;

/// Maximum planning horizon in ticks (20 seconds)
pub const MAX_HORIZON: Tick = 200;

/// Quantized heading: Θ_4 = {N, E, S, W}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Heading {
    N = 0,
    E = 1,
    S = 2,
    W = 3,
}

impl Heading {
    pub fn opposite(self) -> Self {
        match self {
            Heading::N => Heading::S,
            Heading::E => Heading::W,
            Heading::S => Heading::N,
            Heading::W => Heading::E,
        }
    }

    pub fn to_rad(self) -> f32 {
        match self {
            Heading::N => 0.0,
            Heading::E => std::f32::consts::FRAC_PI_2,
            Heading::S => std::f32::consts::PI,
            Heading::W => -std::f32::consts::FRAC_PI_2,
        }
    }

    pub fn to_u8(self) -> u8 {
        self as u8
    }

    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Heading::N),
            1 => Some(Heading::E),
            2 => Some(Heading::S),
            3 => Some(Heading::W),
            _ => None,
        }
    }

    /// Infallible version: masks to 2 bits (always succeeds)
    pub fn from_u8_masked(v: u8) -> Self {
        match v & 0x03 {
            0 => Heading::N,
            1 => Heading::E,
            2 => Heading::S,
            3 => Heading::W,
            _ => unreachable!(),
        }
    }
}

/// Safe interval: maximal contiguous unreserved time window for a vertex
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SafeInterval {
    pub t_start: Tick,
    pub t_end: Tick,
}

impl SafeInterval {
    pub fn new(t_start: Tick, t_end: Tick) -> Self {
        debug_assert!(t_start <= t_end, "Invalid interval: start > end");
        Self { t_start, t_end }
    }

    pub fn duration(&self) -> Tick {
        self.t_end.saturating_sub(self.t_start)
    }

    pub fn contains(&self, tick: Tick) -> bool {
        self.t_start <= tick && tick < self.t_end
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        !(self.t_end <= other.t_start || other.t_end <= self.t_start)
    }

    pub fn intersection(&self, other: &Self) -> Option<Self> {
        let start = self.t_start.max(other.t_start);
        let end = self.t_end.min(other.t_end);
        if start < end {
            Some(Self { t_start: start, t_end: end })
        } else {
            None
        }
    }

    pub fn padded(&self, pad_ticks: Tick) -> Self {
        Self {
            t_start: self.t_start.saturating_sub(pad_ticks),
            t_end: self.t_end.saturating_add(pad_ticks),
        }
    }
}

/// Q8.8 fixed-point: u16 storing values 0.000 to 255.996
/// Used for bit-exact utility calculations across ARM/x86
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct Q8_8(pub u16);

impl Q8_8 {
    pub fn from_f32(v: f32) -> Self {
        let clamped = v.max(0.0).min(255.996);
        Self((clamped * 256.0) as u16)
    }

    pub fn to_f32(self) -> f32 {
        self.0 as f32 / 256.0
    }

    pub fn add(self, other: Self) -> Self {
        Self(self.0.saturating_add(other.0))
    }

    pub fn mul_frac(self, numerator: u16, denominator: u16) -> Self {
        Self(((self.0 as u32 * numerator as u32) / denominator as u32) as u16)
    }
}

/// Q16.16 fixed-point for PID calculations
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Q16_16(pub i32);

impl Q16_16 {
    pub fn from_f32(v: f32) -> Self {
        Self((v * 65536.0) as i32)
    }

    pub fn to_f32(self) -> f32 {
        self.0 as f32 / 65536.0
    }

    pub fn mul(self, other: Self) -> Self {
        Self(((self.0 as i64 * other.0 as i64) >> 16) as i32)
    }

    pub fn add(self, other: Self) -> Self {
        Self(self.0.saturating_add(other.0))
    }

    pub fn sub(self, other: Self) -> Self {
        Self(self.0.saturating_sub(other.0))
    }
}

/// Cells adjacent check (4-connected)
pub fn cells_adjacent(a: CellID, b: CellID, grid_width: u16) -> bool {
    let ax = a as i32 % grid_width as i32;
    let ay = a as i32 / grid_width as i32;
    let bx = b as i32 % grid_width as i32;
    let by = b as i32 / grid_width as i32;
    let dx = (ax - bx).abs();
    let dy = (ay - by).abs();
    (dx == 1 && dy == 0) || (dx == 0 && dy == 1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_safe_interval_contains() {
        let interval = SafeInterval::new(10, 20);
        assert!(interval.contains(10));
        assert!(interval.contains(15));
        assert!(!interval.contains(20));
    }

    #[test]
    fn test_safe_interval_overlaps() {
        let a = SafeInterval::new(10, 20);
        let b = SafeInterval::new(15, 25);
        assert!(a.overlaps(&b));
        let c = SafeInterval::new(20, 30);
        assert!(!a.overlaps(&c));
    }

    #[test]
    fn test_safe_interval_padded() {
        let interval = SafeInterval::new(10, 20);
        let padded = interval.padded(1);
        assert_eq!(padded.t_start, 9);
        assert_eq!(padded.t_end, 21);
    }

    #[test]
    fn test_q8_8_roundtrip() {
        let v = Q8_8::from_f32(1.5);
        assert!((v.to_f32() - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_q16_16_roundtrip() {
        let v = Q16_16::from_f32(3.14);
        assert!((v.to_f32() - 3.14).abs() < 0.001);
    }

    #[test]
    fn test_cells_adjacent() {
        assert!(cells_adjacent(5, 6, 10));  // horizontal
        assert!(cells_adjacent(5, 15, 10)); // vertical
        assert!(!cells_adjacent(5, 16, 10)); // diagonal
    }

    #[test]
    fn test_heading_opposite() {
        assert_eq!(Heading::N.opposite(), Heading::S);
        assert_eq!(Heading::E.opposite(), Heading::W);
    }

    #[test]
    fn test_tick_no_overflow() {
        let tick: Tick = MAX_TICK - 1;
        let next = tick.wrapping_add(1);
        assert_eq!(next, MAX_TICK);
    }
}