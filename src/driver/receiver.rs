use embedded_hal::digital::v2::InputPin;

use crate::consts::MAX_BUFFER_SIZE;

#[derive(Debug)]
enum RxState {
    Idle,
    WaitingForOne,
    Syncing,
    WaitingForStartByte,
    ReadingSize,
    ReadingMessage,
}

pub struct Receiver<const TICKS_PER_BIT: u8, Pin: InputPin> {
    buffer: [u8; MAX_BUFFER_SIZE],
    bit_index: usize,
    state: RxState,
    pin: Pin,
    ticks: u8,
    /// How many ticks in a bit where 1
    n1s_in_bit: u8,
}

impl<Pin: InputPin, const TICKS_PER_BIT: u8> Receiver<TICKS_PER_BIT, Pin> {
    pub fn new(pin: Pin) -> Self {
        Self {
            buffer: [0; MAX_BUFFER_SIZE],
            bit_index: 0,
            state: RxState::Idle,
            ticks: 0,
            n1s_in_bit: 0,
            pin,
        }
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.ticks = 0;
        self.n1s_in_bit = 0;
        self.state = RxState::Idle;
    }

    fn get_pin_state(&self) -> bool {
        self.pin.is_high().unwrap_or(false)
    }

    /// Reads rx state, increments `ticks`
    /// and when `ticks` reaches `TICKS_PER_BIT`
    /// returns a bit, otherwise returns `None`
    fn get_bit(&mut self) -> Option<u8> {
        if self.get_pin_state() {
            self.n1s_in_bit += 1;
        }

        self.ticks += 1;

        if self.ticks >= TICKS_PER_BIT {
            self.ticks = 0;

            // Bit is one if more than half of ticks where one
            let is_bit_one = self.n1s_in_bit > (TICKS_PER_BIT / 2);
            self.n1s_in_bit = 0;

            Some(is_bit_one as u8)
        } else {
            None
        }
    }
}
