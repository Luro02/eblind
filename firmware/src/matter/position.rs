use bincode::{Decode, Encode};

/// Represents the position of a window covering as a percentage,
/// think of it like how much of the window is covered.
///
/// A 0% position means the window is fully open.
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Decode, Encode)]
pub struct Position {
    /// Internally the position is stored as a percentage from 0.0 to 1.0.
    position: f64,
}

impl Position {
    pub const fn new(position: f64) -> Self {
        assert!(
            position >= 0.0 && position <= 1.0,
            "Position must be between 0.0 and 1.0"
        );
        Self { position }
    }

    pub const fn min() -> Self {
        Self::open()
    }

    pub const fn max() -> Self {
        Self::closed()
    }

    pub const fn open() -> Self {
        Self::new(0.0)
    }

    pub const fn closed() -> Self {
        Self::new(1.0)
    }

    /// Returns true if the target position is closer to fully open than the
    /// current position.
    pub const fn is_opening(&self, target_position: Self) -> bool {
        // It is opening if (target - current) is negative, and closing if it is positive.
        //
        // target - current < 0
        // <=> target < current
        self.position > target_position.position
    }

    #[must_use]
    pub const fn map_to(&self, max: f64) -> f64 {
        self.position * max
    }
}
