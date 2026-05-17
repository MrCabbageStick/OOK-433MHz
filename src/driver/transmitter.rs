use embedded_hal::digital::v2::OutputPin;

use crate::consts::{MAX_BUFFER_SIZE, MESSAGE_START_BYTE, SYNC_BYTE, SYNC_SEQUENCE_BIT_LENGTH};

enum TxState {
    Idle,
    Syncing,
    SendingStartByte,
    SendingData,
    DataSent,
}

pub struct Transmitter<const TICKS_PER_BIT: u8, Pin: OutputPin> {
    buffer: [u8; MAX_BUFFER_SIZE],
    message_bit_length: usize,
    bit_index: usize,
    state: TxState,
    pin: Pin,
    ticks: u8,
}

impl<Pin: OutputPin, const TICKS_PER_BIT: u8> Transmitter<TICKS_PER_BIT, Pin> {
    pub fn new(pin: Pin) -> Self {
        Self {
            buffer: [0; MAX_BUFFER_SIZE],
            message_bit_length: 0,
            bit_index: 0,
            state: TxState::Idle,
            ticks: 0,
            pin,
        }
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.message_bit_length = 0;
        self.state = TxState::Idle;
    }

    pub fn transmit(&mut self) {
        // Hold state for TICKS_PER_BIT
        if self.ticks != 0 {
            self.ticks += 1;

            if self.ticks >= TICKS_PER_BIT {
                self.ticks = 0;
            }

            return;
        }

        self.ticks += 1;

        match self.state {
            TxState::Idle => return,
            TxState::Syncing => {
                if self.send_sync() {
                    self.state = TxState::SendingStartByte;
                }
            }
            TxState::SendingStartByte => {
                if self.send_start_byte() {
                    self.state = TxState::SendingData
                }
            }
            TxState::SendingData => {
                if self.send_data() {
                    self.state = TxState::DataSent
                }
            }
            TxState::DataSent => {
                self.cleanup();
            }
        }
    }

    /// Sets pin high or low,
    /// abstracted if I were to add inverted 1 and 0 handling
    fn set_pin_state(&mut self, high: bool) {
        if high {
            let _ = self.pin.set_high();
        } else {
            let _ = self.pin.set_low();
        }
    }

    /// Sends bits of the synchronization bytes
    /// and, when all are sent, returns true
    /// Sends least significant bits first
    fn send_sync(&mut self) -> bool {
        // Extract 7 ls bits to not use % operator
        let bit = self.bit_index & 0x7;

        let state = (SYNC_BYTE >> bit) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        // If synced clean bit_index and move along
        if self.bit_index >= SYNC_SEQUENCE_BIT_LENGTH as usize {
            self.bit_index = 0;
            return true;
        }

        false
    }

    /// Sends bits of the message start byte
    /// and, when all are sent, returns true
    /// Sends least significant bits first
    fn send_start_byte(&mut self) -> bool {
        let state = (MESSAGE_START_BYTE >> self.bit_index) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        if self.bit_index >= 8 {
            self.bit_index = 0;
            return true;
        }

        false
    }

    /// Sends bits of the message
    /// and, when all are sent, returns true
    /// Sends least significant bits first
    fn send_data(&mut self) -> bool {
        let byte = self.bit_index >> 3;
        let bit = self.bit_index & 0x7;

        let state = (self.buffer[byte] >> bit) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        if self.bit_index >= self.message_bit_length {
            self.bit_index = 0;
            return true;
        }

        false
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::digital::v2::InputPin;

    use crate::{consts::MESSAGE_START_BYTE, mock_pin::MockPin};

    use super::*;

    #[test]
    fn sync_bits() {
        const TICKS_PER_BIT: u8 = 5;
        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::Syncing;

        let mut byte = 0u8;

        for _ in 0..(SYNC_SEQUENCE_BIT_LENGTH / 8) {
            for bi in 0..8 {
                for _ in 0..TICKS_PER_BIT {
                    driver.transmit();
                }

                byte |= (driver.pin.is_high().unwrap() as u8 & 0x1) << bi;
            }
            // Check for byte correctness
            assert!(
                byte == SYNC_BYTE,
                "Received sync byte (0x{:x}) does not match expected sync byte (0x{SYNC_BYTE:x})",
                byte
            );

            byte = 0;
        }

        // Check if driver moved to the next state
        assert!(
            matches!(driver.state, TxState::SendingStartByte),
            "Transmitter failed to move to the next state"
        )
    }

    #[test]
    fn message_start_byte() {
        const TICKS_PER_BIT: u8 = 5;
        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingStartByte;

        let mut byte = 0u8;

        for bi in 0..8 {
            for _ in 0..TICKS_PER_BIT {
                driver.transmit();
            }
            byte |= (driver.pin.is_high().unwrap() as u8 & 0x1) << bi;
        }

        assert!(
            byte == MESSAGE_START_BYTE,
            "Received mesage start byte byte (0x{:x}) does not match expected byte (0x{MESSAGE_START_BYTE:x})",
            byte
        );

        // Check if driver moved to the next state
        assert!(
            matches!(driver.state, TxState::SendingData),
            "Transmitter failed to move to the next state"
        )
    }

    #[test]
    fn message_data() {
        const TICKS_PER_BIT: u8 = 5;
        const DATA: [u8; 5] = [0x1, 0x10, 0xf8, 0xff, 0x00];
        let bit_length = DATA.len() * 8;

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingData;
        driver.message_bit_length = bit_length;
        driver.buffer[0..DATA.len()].copy_from_slice(&DATA);

        let mut received = [0u8; DATA.len()];

        for bi in 0..bit_length as usize {
            for _ in 0..TICKS_PER_BIT {
                driver.transmit();
            }

            let byte_i = bi / 8;
            let bit_i = bi % 8;

            let state = driver.pin.is_high().unwrap() as u8 & 0x1;

            received[byte_i] |= state << bit_i;
        }

        assert!(
            DATA == received,
            "Received data ({received:?}) does not match the source data ({DATA:?})"
        );

        assert!(
            matches!(driver.state, TxState::DataSent),
            "Transmitter failed to move to the next state"
        )
    }

    #[test]
    fn ticks_per_bit() {
        const TICKS_PER_BIT: u8 = 8;
        const DATA: [u8; 1] = [0xaa];
        let bit_length = DATA.len() * 8;

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingData;
        driver.message_bit_length = bit_length;
        driver.buffer[0..DATA.len()].copy_from_slice(&DATA);

        for bit_i in 0..bit_length {
            let data_bit = (DATA[0] >> bit_i) & 0x1;

            for tick_i in 0..TICKS_PER_BIT {
                driver.transmit();

                let state = driver.pin.is_high().unwrap() as u8 & 0x1;

                assert!(
                    state == data_bit,
                    "Expected {data_bit}, but got {state} on {tick_i} tick in {bit_i} bit of 0b{:b}",
                    DATA[0]
                );
            }
        }
    }
}
