use embedded_hal::digital::v2::OutputPin;
use ufmt::derive::uDebug;

use crate::{
    consts::{
        MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_START_BYTE, SYNC_BYTE,
        SYNC_SEQUENCE_BIT_LENGTH,
    },
    data_coding::radio_head_4b6b::encode_in_place,
};

#[derive(Debug, uDebug)]
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
    byte_index: usize,
    state: TxState,
    pub pin: Pin,
    ticks: u8,
}

impl<Pin: OutputPin, const TICKS_PER_BIT: u8> Transmitter<TICKS_PER_BIT, Pin> {
    pub fn new(pin: Pin) -> Self {
        Self {
            buffer: [0; MAX_BUFFER_SIZE],
            message_bit_length: 0,
            bit_index: 0,
            byte_index: 0,
            state: TxState::Idle,
            ticks: 0,
            pin,
        }
    }

    pub fn is_idle(&self) -> bool {
        matches!(self.state, TxState::Idle)
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.byte_index = 0;
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
            TxState::Idle => {
                // Dont tick on idle
                self.ticks = 0;
            }
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
        // let byte = self.bit_index >> 3;
        // let bit = self.bit_index & 0x7;

        let state = (self.buffer[self.byte_index] >> self.bit_index) & 0x1;
        self.set_pin_state(state != 0);

        self.bit_index += 1;

        // // Send 6 bits out of every byte
        // // as encoding uses only 6 bits
        // if bit == 5 {
        //     self.bit_index += 2; // Skip 2 bits
        // }
        if self.bit_index >= 6 {
            self.bit_index = 0;
            self.byte_index += 1;
        }

        if self.byte_index * 6 + self.bit_index >= self.message_bit_length {
            self.bit_index = 0;
            self.byte_index = 0;
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
        // (message size + byte for length) * encoded nibble size * nibbles per byte
        self.message_bit_length = (n_bytes + 1) * 6 * 2;

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
    use heapless::Vec;

    use crate::{
        consts::{MESSAGE_OFFSET, MESSAGE_START_BYTE, SYNC_BYTE, SYNC_SEQUENCE_BIT_LENGTH},
        data_coding::radio_head_4b6b::decode_in_place,
        mock_pin::MockPin,
    };

    use super::*;

    const TICKS_PER_BIT: u8 = 5;

    /// Advance the transmitter by exactly one bit period and return the
    /// pin state that was set at the *start* of that period.
    fn tick_one_bit(driver: &mut Transmitter<TICKS_PER_BIT, MockPin>) -> bool {
        // First tick: transmitter sets the pin and increments internal tick counter
        driver.transmit();
        let state = driver.pin.is_high().unwrap();

        // Remaining ticks: transmitter holds state (no-ops internally)
        for _ in 1..TICKS_PER_BIT {
            driver.transmit();
            // Pin must stay stable during the hold ticks
            assert_eq!(
                driver.pin.is_high().unwrap(),
                state,
                "Pin changed mid-bit-period"
            );
        }

        state
    }

    /// Collect `n_bits` worth of bit periods into a byte (LSB first).
    // fn collect_bits(
    //     driver: &mut Transmitter<TICKS_PER_BIT, MockPin>,
    //     n_bits: usize,
    // ) -> Vec<bool, 64> {
    //     let mut bits = Vec::new();
    //     for _ in 0..n_bits {
    //         bits.push(tick_one_bit(driver)).unwrap();
    //     }
    //     bits
    // }

    fn bits_to_byte_lsb(bits: &[bool]) -> u8 {
        assert!(bits.len() <= 8);
        bits.iter()
            .enumerate()
            .fold(0u8, |acc, (i, &b)| acc | ((b as u8) << i))
    }

    // ── sync ────────────────────────────────────────────────────────────────

    #[test]
    fn sync_bits() {
        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::Syncing;

        for _ in 0..(SYNC_SEQUENCE_BIT_LENGTH / 8) {
            let bits: Vec<bool, 8> = (0..8).map(|_| tick_one_bit(&mut driver)).collect();

            let byte = bits_to_byte_lsb(&bits);
            assert_eq!(
                byte, SYNC_BYTE,
                "Sync byte mismatch: got 0x{byte:02x}, expected 0x{SYNC_BYTE:02x}"
            );
        }

        assert!(
            matches!(driver.state, TxState::SendingStartByte),
            "Expected SendingStartByte after sync, got {:?}",
            driver.state
        );
    }

    // ── start byte ──────────────────────────────────────────────────────────

    #[test]
    fn message_start_byte() {
        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingStartByte;

        let bits: Vec<bool, 8> = (0..8).map(|_| tick_one_bit(&mut driver)).collect();
        let byte = bits_to_byte_lsb(&bits);

        assert_eq!(
            byte, MESSAGE_START_BYTE,
            "Start byte mismatch: got 0x{byte:02x}, expected 0x{MESSAGE_START_BYTE:02x}"
        );

        assert!(
            matches!(driver.state, TxState::SendingData),
            "Expected SendingData after start byte, got {:?}",
            driver.state
        );
    }

    // ── data ────────────────────────────────────────────────────────────────

    #[test]
    fn message_data() {
        // Values chosen to check all 6 useful bits (encoding uses bits 0–5 only)
        const DATA: [u8; 5] = [0x01, 0x10, 0x38, 0x3f, 0x00];

        let bit_length = DATA.len() * 6; // matches what `send()` would set

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::SendingData;
        driver.message_bit_length = bit_length;
        driver.buffer[..DATA.len()].copy_from_slice(&DATA);

        let mut received = [0u8; DATA.len()];

        // send_data sends bits 0–5 of each byte and skips bits 6–7
        for byte_i in 0..DATA.len() {
            for bit_i in 0..6usize {
                let state = tick_one_bit(&mut driver);
                received[byte_i] |= (state as u8) << bit_i;
            }
        }

        assert_eq!(
            DATA, received,
            "Received data {received:?} does not match source {DATA:?}"
        );

        assert!(
            matches!(driver.state, TxState::DataSent),
            "Expected DataSent after all data bits, got {:?}",
            driver.state
        );
    }

    #[test]
    fn data_sent_transitions_to_idle() {
        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.state = TxState::DataSent;

        // One full bit period in DataSent should trigger cleanup -> Idle
        tick_one_bit(&mut driver);

        assert!(
            matches!(driver.state, TxState::Idle),
            "Expected Idle after DataSent tick, got {:?}",
            driver.state
        );
    }

    // ── ticks_per_bit ───────────────────────────────────────────────────────

    #[test]
    fn ticks_per_bit() {
        const TICKS: u8 = 8;
        const DATA: [u8; 1] = [0xaa];
        let bit_length = DATA.len() * 6;

        let mut driver = Transmitter::<TICKS, _>::new(MockPin::new());
        driver.state = TxState::SendingData;
        driver.message_bit_length = bit_length;
        driver.buffer[..DATA.len()].copy_from_slice(&DATA);

        for bit_i in 0..6usize {
            let expected = (DATA[0] >> bit_i) & 0x1;

            for tick_i in 0..TICKS {
                driver.transmit();
                let actual = driver.pin.is_high().unwrap() as u8;
                assert_eq!(
                    actual, expected,
                    "tick {tick_i} of bit {bit_i}: expected {expected}, got {actual}"
                );
            }
        }
    }

    // ── send() integration ──────────────────────────────────────────────────

    #[test]
    fn send_encodes_and_sets_state() {
        const DATA: &[u8] = b"Hello, there!";

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        let n = driver.send(DATA).expect("send() returned None");

        assert_eq!(n, DATA.len(), "send() reported wrong byte count");
        assert!(
            matches!(driver.state, TxState::Syncing),
            "Expected Syncing after send(), got {:?}",
            driver.state
        );

        // Decode the buffer and verify contents
        const ENCODED_LEN: usize = (DATA.len() + 1) * 2;
        let mut decoded = [0u8; ENCODED_LEN];
        decoded.copy_from_slice(&driver.buffer[..ENCODED_LEN]);

        decode_in_place(&mut decoded).expect("decode_in_place failed");

        assert_eq!(
            decoded[0],
            DATA.len() as u8,
            "Length byte mismatch: expected {}, got {}",
            DATA.len(),
            decoded[0]
        );
        assert_eq!(
            &decoded[MESSAGE_OFFSET..MESSAGE_OFFSET + DATA.len()],
            DATA,
            "Decoded payload does not match source"
        );
    }

    #[test]
    fn send_truncates_oversized_message() {
        let oversized: Vec<u8, { MAX_MESSAGE_LENGTH + 1 }> =
            (0..MAX_MESSAGE_LENGTH + 1).map(|i| i as u8).collect();

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        let n = driver.send(&oversized).expect("send() returned None");

        assert_eq!(
            n, MAX_MESSAGE_LENGTH,
            "send() should truncate to MAX_MESSAGE_LENGTH"
        );
    }

    // ── full round-trip ─────────────────────────────────────────────────────

    /// Drive the transmitter through entire sending process and decode
    /// every bit off the wire, then verify the recovered payload.
    #[test]
    fn full_transmission_round_trip() {
        const DATA: &[u8] = b"round trip!";

        let mut driver = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        driver.send(DATA).unwrap();

        // Skip sync bits
        for _ in 0..SYNC_SEQUENCE_BIT_LENGTH {
            tick_one_bit(&mut driver);
        }

        assert!(
            matches!(driver.state, TxState::SendingStartByte),
            "Driver should be in a 'SendingStartByte' phase, but is in: '{:?}'",
            driver.state
        );

        // Skip start byte
        for _ in 0..8 {
            tick_one_bit(&mut driver);
        }

        assert!(
            matches!(driver.state, TxState::SendingData),
            "Driver should be in a 'SendingStartByte' phase, but is in: '{:?}'",
            driver.state
        );

        // Collect encoded payload bytes (6 bits each, LSB first)
        let n_encoded_bytes = (DATA.len() + 1) * 2;
        let mut encoded_bytes = Vec::<u8, 64>::new();

        for _ in 0..n_encoded_bytes {
            let mut byte = 0u8;

            for bit_i in 0..6usize {
                let state = tick_one_bit(&mut driver);
                byte |= (state as u8) << bit_i;
            }

            encoded_bytes.push(byte).unwrap();
        }

        assert!(
            matches!(driver.state, TxState::DataSent),
            "Transmitter state should be 'DataSent', but is {:?}",
            driver.state
        );

        // Next tick should be cleanup
        driver.transmit();

        assert!(
            matches!(driver.state, TxState::Idle),
            "Transmitter state should be 'Idle', but is {:?}",
            driver.state
        );

        let decoded_size = match decode_in_place(&mut encoded_bytes) {
            Ok(n) => n,
            Err(e) => panic!("Error occured during decoding: {e:?}"),
        };

        let data_and_length_byte_size = DATA.len() + 1;

        assert_eq!(
            decoded_size, data_and_length_byte_size,
            "Length byte mismatch"
        );

        assert_eq!(
            &encoded_bytes[1..decoded_size],
            DATA,
            "Payload mismatch after round-trip"
        );
    }
}
