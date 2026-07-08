/// How much to increment `ramp` when
/// signal is clocked correctly
pub const RAMP_STEP: u16 = 20;

#[derive(Debug)]
pub struct Pll<const TICKS_PER_BIT: u8> {
    pub samples: u8,
    /// Number of ticks representing
    pub intergrator: u8,
    /// Indicator for when bit is ready
    pub ramp: u16,
    /// Last tick's state
    pub last_state: bool,
}

impl<const TICKS_PER_BIT: u8> Pll<TICKS_PER_BIT> {
    /// When bit is ready
    const RAMP_LENGTH: u16 = TICKS_PER_BIT as u16 * RAMP_STEP;
    /// Ramp midpoint
    const RAMP_TRANSITION: u16 = Self::RAMP_LENGTH / 2;
    /// How much to speed up or slow down the ramp
    const RAMP_ADJUST: u16 = 9;
    /// How much to increment `ramp` when
    /// receiver is too slow
    const RAMP_ADVANCE: u16 = RAMP_STEP + Self::RAMP_ADJUST;
    /// How much to increment `ramp` when
    /// receiver is too fast
    const RAMP_DELAY: u16 = RAMP_STEP - Self::RAMP_ADJUST;

    pub fn new() -> Self {
        Self {
            samples: 0,
            intergrator: 0,
            ramp: 0,
            last_state: false,
        }
    }

    /// Clears state
    pub fn clear(&mut self) {
        self.samples = 0;
        self.intergrator = 0;
        self.last_state = false;
        self.ramp = 0;
    }

    pub fn tick(&mut self, state: bool) -> Option<u8> {
        if state {
            self.intergrator += 1;
        }
        self.samples += 1;

        // Possible bit boundry when samples are different
        if state != self.last_state {
            self.last_state = state;

            // If ramp is delayed advance it
            if self.ramp < Self::RAMP_TRANSITION {
                self.ramp += Self::RAMP_DELAY;
            } else {
                self.ramp += Self::RAMP_ADVANCE;
            }
        } else {
            // Increment ramp by standard step
            self.ramp += RAMP_STEP;
        }

        // Ram idicates bit ended
        if self.ramp >= Self::RAMP_LENGTH {
            let bit = (self.intergrator > self.samples / 2) as u8;

            self.intergrator = 0;
            self.samples = 0;
            // Propagate ramp offset
            self.ramp -= Self::RAMP_LENGTH;

            Some(bit)
        } else {
            None
        }
    }
}
