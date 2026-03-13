use core::convert::Infallible;

use embedded_hal::digital::v2::{InputPin, OutputPin};

pub struct MockPin {
    state: bool,
}

impl MockPin {
    pub fn new() -> Self {
        Self { state: false }
    }

    pub fn with_state(state: bool) -> Self {
        Self { state }
    }

    pub fn sync_with(&mut self, other: &dyn InputPin<Error = Infallible>) {
        self.state = other.is_high().unwrap();
    }
}

impl OutputPin for MockPin {
    type Error = Infallible;

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        Ok(())
    }

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        Ok(())
    }
}

impl InputPin for MockPin {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
}
