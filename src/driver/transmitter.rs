use embedded_hal::digital::v2::OutputPin;

use crate::{
    consts::{
        MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_START_BYTE, SYNC_BYTE,
        SYNC_SEQUENCE_BIT_LENGTH,
    },
    data_coding::radio_head_4b6b::encode_in_place,
};

#[derive(Debug)]
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

        // Send 6 bits out of every byte
        // as encoding uses only 6 bits
        if bit == 5 {
            self.bit_index += 2; // Skip 2 bits
        }

        if self.bit_index >= self.message_bit_length {
            self.bit_index = 0;
            return true;
        }

        false
    }

    pub fn send(&mut self, bytes: &[u8]) -> Option<usize> {
        // n_bytes = max(bytes.len(), MAX_MESSAGE_LENGTH)
        let n_bytes = if bytes.len() > MAX_MESSAGE_LENGTH {
            MAX_MESSAGE_LENGTH
        } else {
            bytes.len()
        };

        // Set message length as first byte
        self.buffer[0] = n_bytes as u8;
        // Populate buffer
        self.buffer[1..=n_bytes].copy_from_slice(&bytes[0..n_bytes]);

        // (n_bytes + message_size) * encoding overhead * 8 bits per byte
        // Even though encoding uses only 6 bits of every byte
        // set value to number of all bits in bytes
        self.message_bit_length = (n_bytes + 1) * 8 * 2;

        // Encode data
        match encode_in_place(&mut self.buffer, n_bytes + 1) {
            Ok(_) => {
                self.state = TxState::Syncing;
                Some(n_bytes)
            }
            Err(_) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use embedded_hal::digital::v2::InputPin;

    use crate::{
        consts::{MESSAGE_OFFSET, MESSAGE_START_BYTE},
        data_coding::radio_head_4b6b::decode_in_place,
        mock_pin::MockPin,
    };

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
        // Use only 6 ls bits
        const DATA: [u8; 5] = [0x1, 0x10, 0x38, 0x3f, 0x00];
        let bit_length = DATA.len() * 8;

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingData;
        driver.message_bit_length = bit_length;
        driver.buffer[0..DATA.len()].copy_from_slice(&DATA);

        let mut received = [0u8; DATA.len()];

        // After encoding only 6 bits of every byte will be sent
        let n_bits_to_send = DATA.len() * 6;

        for bi in 0..n_bits_to_send as usize {
            for _ in 0..TICKS_PER_BIT {
                driver.transmit();
            }

            let byte_i = bi / 6;
            let bit_i = bi % 6;

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

    #[test]
    fn send() {
        const TICKS_PER_BIT: u8 = 8;
        const DATA: &[u8; 13] = b"Hello, there!";

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());

        driver.send(DATA);

        const ENCODED_LENGTH: usize = (DATA.len() + 1) * 2;
        let mut decoded_buf = [0u8; ENCODED_LENGTH];
        decoded_buf.copy_from_slice(&driver.buffer[0..ENCODED_LENGTH]);

        assert!(
            decode_in_place(&mut decoded_buf).is_ok(),
            "Unable to decode buffer"
        );

        assert!(
            decoded_buf[0] == DATA.len() as u8,
            "First bytes does not code for message size, expected {}, but got {}",
            DATA.len(),
            decoded_buf[0]
        );

        assert!(
            &decoded_buf[MESSAGE_OFFSET..=DATA.len()] == DATA,
            "Decoded data does not equal source data, expected {:?}, but got {:?}",
            DATA,
            &decoded_buf[MESSAGE_OFFSET..=DATA.len()]
        )
    }
}
