/// How much to increment `ramp` when
/// signal is clocked correctly
pub const RAMP_STEP: u16 = 20;
/// How much to speed up or slow down the ramp
pub const RAMP_ADJUST: u16 = 9;
/// How much to increment `ramp` when
/// receiver is too slow
pub const RAMP_ADVANCE: u16 = RAMP_STEP + RAMP_ADJUST;
/// How much to increment `ramp` when
/// receiver is too slow
pub const RAMP_DELAY: u16 = RAMP_STEP - RAMP_ADJUST;

pub struct Pll<const TICKS_PER_BIT: u8> {
    /// Number of ticks representing
    intergrator: u8,
    /// Indicator for when bit is ready
    ramp: u16,
    /// Last tick's state
    last_state: bool,
}

impl<const TICKS_PER_BIT: u8> Pll<TICKS_PER_BIT> {
    /// When bit is ready
    const RAMP_LENGTH: u16 = TICKS_PER_BIT as u16 * RAMP_STEP;
    /// Ramp midpoint
    const RAMP_TRANSITION: u16 = Self::RAMP_LENGTH / 2;

    pub fn new() -> Self {
        Self {
            intergrator: 0,
            ramp: 0,
            last_state: false,
        }
    }

    /// Clears state
    pub fn clear(&mut self) {
        self.intergrator = 0;
        self.last_state = false;
        self.ramp = 0;
    }

    pub fn tick(&mut self, state: bool) -> Option<u8> {
        if state {
            self.intergrator += 1;
        }

        // Possible bit boundry when samples are different
        if state != self.last_state {
            self.last_state = state;

            // If ramp is delayed advance it
            if self.ramp < Self::RAMP_TRANSITION {
                self.ramp += RAMP_ADVANCE;
            } else {
                // When ramp is too fast delay it
                self.ramp += RAMP_DELAY;
            }
        } else {
            // Increment ramp by standard step
            self.ramp += RAMP_STEP;
        }

        // Ram idicates bit ended
        if self.ramp >= Self::RAMP_LENGTH {
            let bit = (self.intergrator > TICKS_PER_BIT / 2) as u8;

            self.intergrator = 0;
            // Propagate ramp offset
            self.ramp -= Self::RAMP_LENGTH;

            Some(bit)
        } else {
            None
        }
    }
}
