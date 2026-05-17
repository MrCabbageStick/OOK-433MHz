use embedded_hal::digital::v2::OutputPin;

use crate::consts::{MAX_BUFFER_SIZE, MESSAGE_START_BYTE, SYNC_BYTE, SYNC_SEQUENCE_BIT_LENGTH};

enum TxState {
    Idle,
    Syncing,
    SendingStartByte,
    SendingData,
}

pub struct Transmitter<Pin: OutputPin> {
    buffer: [u8; MAX_BUFFER_SIZE],
    message_length: usize,
    bit_index: usize,
    state: TxState,
    pin: Pin,
}

impl<Pin: OutputPin> Transmitter<Pin> {
    pub fn new(pin: Pin) -> Self {
        Self {
            buffer: [0; MAX_BUFFER_SIZE],
            message_length: 0,
            bit_index: 0,
            state: TxState::Idle,
            pin,
        }
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.message_length = 0;
        self.state = TxState::Idle;
    }

    pub fn transmit(&mut self) {
        match self.state {
            TxState::Idle => return,

            TxState::Syncing => self.send_sync(),

            TxState::SendingStartByte => self.send_start_byte(),

            TxState::SendingData => {}
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
    /// and, when all are sent, moves the state to `SendingStartByte`\
    /// Sends least significant bits first
    pub fn send_sync(&mut self) {
        // Extract 7 ls bits to not use % operator
        let bit = self.bit_index & 0x7;

        let state = (SYNC_BYTE >> bit) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        // If synced clean bit_index and move along
        if self.bit_index >= SYNC_SEQUENCE_BIT_LENGTH as usize {
            self.bit_index = 0;
            self.state = TxState::SendingStartByte;
        }
    }

    /// Sends bits of the message start byte
    /// and, when all are sent, moves the state to `SendingData`\
    /// Sends least significant bits first
    pub fn send_start_byte(&mut self) {
        let state = (MESSAGE_START_BYTE >> self.bit_index) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        if self.bit_index >= 8 {
            self.bit_index = 0;
            self.state = TxState::SendingData;
        }
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::digital::v2::InputPin;

    use crate::{consts::MESSAGE_START_BYTE, mock_pin::MockPin};

    use super::*;

    #[test]
    fn sync_bits() {
        let mut driver = Transmitter::new(MockPin::new());
        driver.state = TxState::Syncing;

        let mut byte = 0u8;

        for _ in 0..(SYNC_SEQUENCE_BIT_LENGTH / 8) {
            for bi in 0..8 {
                driver.transmit();
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
        let mut driver = Transmitter::new(MockPin::new());
        driver.state = TxState::SendingStartByte;

        let mut byte = 0u8;

        for bi in 0..8 {
            driver.transmit();
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
}
