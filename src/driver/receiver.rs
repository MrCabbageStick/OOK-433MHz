use core::{error::Error, fmt::Display};

use embedded_hal::digital::v2::InputPin;

use crate::{
    consts::{
        MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_OFFSET, MESSAGE_START_BYTE,
        SYNC_SEQUENCE_BIT_LENGTH,
    },
    data_coding::radio_head_4b6b::{RunningDecoder, RunningDecoderError},
    driver::receiver::ReceiverError::DecoderError,
};

#[derive(Debug)]
enum RxState {
    Idle,
    WaitingForOne,
    Syncing,
    WaitingForStartByte,
    ReadingSize,
    ReadingMessage { message_size: u8 },
    MessageReceived { message_size: u8 },
}

pub struct Receiver<const TICKS_PER_BIT: u8, Pin: InputPin> {
    buffer: [u8; MAX_BUFFER_SIZE],
    buffer_byte_index: usize,
    bit_index: usize,
    state: RxState,
    pin: Pin,
    ticks: u8,
    /// How many ticks in a bit where 1
    n1s_in_bit: u8,
    current_byte: u8,

    decoder: RunningDecoder,
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
            current_byte: 0,
            buffer_byte_index: 0,
            decoder: RunningDecoder::new(),
        }
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.ticks = 0;
        self.n1s_in_bit = 0;
        self.state = RxState::Idle;
        self.current_byte = 0;
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

    /// Returns read bytes if message is ready,
    /// or None if it's not. If receiver is idle
    /// starts receiving
    pub fn receive(&mut self) -> Result<&[u8], ReceiverError> {
        match self.state {
            RxState::Idle => self.state = RxState::WaitingForOne,
            RxState::WaitingForOne => {
                // If one detected move to the next state
                if self.get_pin_state() {
                    self.state = RxState::Syncing;
                    // Bit was already from the transmitter's sync sequence
                    self.current_byte = 1;
                }
            }
            RxState::Syncing => match self.sync() {
                // Move to the next state
                Ok(true) => self.state = RxState::WaitingForStartByte,
                // Wait for more bits
                Ok(false) => {}
                // Failed
                Err(err) => {
                    self.cleanup();
                    return Err(err);
                }
            },
            RxState::WaitingForStartByte => match self.wait_start_byte() {
                Ok(true) => self.state = RxState::ReadingSize,
                Ok(false) => {}
                Err(err) => {
                    self.cleanup();
                    return Err(err);
                }
            },
            RxState::ReadingSize => match self.read_message_size() {
                Ok((true, size)) => {
                    if size as usize > MAX_MESSAGE_LENGTH {
                        return Err(ReceiverError::MessageTooLong);
                    }
                    self.state = RxState::ReadingMessage { message_size: size };
                }
                Ok((false, _)) => {}
                Err(err) => {
                    self.cleanup();
                    return Err(err);
                }
            },
            RxState::ReadingMessage { message_size } => match self.read_message(message_size) {
                Ok(true) => self.state = RxState::MessageReceived { message_size },
                Ok(false) => {}
                Err(err) => {
                    self.cleanup();
                    return Err(err);
                }
            },
            RxState::MessageReceived { message_size } => {
                self.cleanup();
                return Ok(&self.buffer[MESSAGE_OFFSET..(message_size as usize) + MESSAGE_OFFSET]);
            }
        }

        Err(ReceiverError::MessageNotReady)
    }

    /// Wait for repeating sequence of 1s and 0s.
    /// If bits are not alternating return `SyncError`,
    /// if not enought bits are present to sync return `false`,
    /// otherwise return `true`
    fn sync(&mut self) -> Result<bool, ReceiverError> {
        // Get bit from ticks
        let Some(state) = self.get_bit() else {
            return Ok(false);
        };

        // If current state is the same as previuos one
        // return an error
        if self.current_byte == state {
            return Err(ReceiverError::SyncError);
        }

        self.current_byte = state;
        self.bit_index += 1;

        if self.bit_index as u8 >= SYNC_SEQUENCE_BIT_LENGTH {
            // Cleanup
            self.bit_index = 0;
            self.current_byte = 0;

            // Is synced
            Ok(true)
        } else {
            // Not synced yet
            Ok(false)
        }
    }

    /// Wait for a byte, if it's a `MESSAGE_START_BYTE`
    /// return `true`, if it's not return `Err(WrongStartByte)`.
    /// If byte is not complete return `false`
    fn wait_start_byte(&mut self) -> Result<bool, ReceiverError> {
        let Some(bit) = self.get_bit() else {
            return Ok(false);
        };

        self.current_byte |= bit << self.bit_index;
        self.bit_index += 1;

        if self.bit_index >= 8 {
            if self.current_byte != MESSAGE_START_BYTE {
                return Err(ReceiverError::WrongStartByte);
            }

            self.current_byte = 0;
            self.bit_index = 0;

            return Ok(true);
        }

        Ok(false)
    }

    /// Read message size from 2 incoming 6-bit nibbles.
    /// Returns `(true, size)`` when decoder finishes decoding the
    /// nibbles, when not finished returns `(false, _)`, and when
    /// decoder is unable to decode a nibble returns
    /// `ReceiverError::DecoderError`
    fn read_message_size(&mut self) -> Result<(bool, u8), ReceiverError> {
        let Some(bit) = self.get_bit() else {
            return Ok((false, 0));
        };

        self.current_byte |= bit << self.bit_index;
        self.bit_index += 1;

        if self.bit_index >= 6 {
            self.bit_index = 0;

            let decoder_res = self.decoder.next_nibble(self.current_byte);

            self.current_byte = 0;

            match decoder_res {
                Err(RunningDecoderError::ByteNotReady) => {
                    return Ok((false, 0));
                }
                Err(err) => {
                    return Err(ReceiverError::DecoderError(err));
                }
                Ok(byte) => {
                    return Ok((true, byte));
                }
            }
        }

        Ok((false, 0))
    }

    fn read_message(&mut self, message_size: u8) -> Result<bool, ReceiverError> {
        let Some(bit) = self.get_bit() else {
            return Ok(false);
        };

        self.current_byte |= bit << self.bit_index;
        self.bit_index += 1;

        // 6-bit encoded nibble
        if self.bit_index >= 6 {
            self.bit_index = 0;

            let decoder_res = self.decoder.next_nibble(self.current_byte);
            self.current_byte = 0;

            match decoder_res {
                Err(RunningDecoderError::ByteNotReady) => {
                    return Ok(false);
                }
                Err(err) => return Err(ReceiverError::DecoderError(err)),
                Ok(byte) => {
                    self.buffer[self.buffer_byte_index] = byte;
                    self.buffer_byte_index += 1;

                    if self.buffer_byte_index >= message_size as usize {
                        self.buffer_byte_index = 0;
                        return Ok(true);
                    }
                }
            }
        }

        Ok(false)
    }
}

#[derive(Debug)]
pub enum ReceiverError {
    /// Not an error per se, but an information
    MessageNotReady,
    /// Sync signal is not  a repeating sequence of 1s and 0s
    SyncError,
    WrongStartByte,
    MessageTooLong,
    DecoderError(RunningDecoderError),
}

#[cfg(test)]
mod tests {
    use embedded_hal::digital::v2::OutputPin;

    use crate::{
        consts::{MESSAGE_START_BYTE, SYNC_SEQUENCE_BIT_LENGTH},
        data_coding::radio_head_4b6b::encode_in_place,
        driver::receiver::{Receiver, ReceiverError, RxState},
        mock_pin::MockPin,
    };

    const TICKS_PER_BIT: u8 = 5;

    fn get_default_receiver() -> Receiver<TICKS_PER_BIT, MockPin> {
        Receiver::<TICKS_PER_BIT, _>::new(MockPin::new())
    }

    /// Advance the receiver by exactly one bit period,
    /// return `Err` if error was returned in the meantime
    fn tick_one_bit(driver: &mut Receiver<TICKS_PER_BIT, MockPin>) -> Result<(), ReceiverError> {
        for _ in 0..TICKS_PER_BIT {
            match driver.receive() {
                Err(ReceiverError::MessageNotReady) => {}
                Ok(_) => {}
                Err(err) => return Err(err),
            }
        }

        Ok(())
    }

    #[test]
    fn move_from_idle() {
        let mut driver = get_default_receiver();

        let message_status = driver.receive();

        assert!(
            matches!(message_status, Err(ReceiverError::MessageNotReady)),
            "Receiver::receive() returned wrong error type: {message_status:?}"
        );

        assert!(
            !matches!(driver.state, RxState::Idle),
            "Driver is still in Idle state"
        );
    }

    #[test]
    fn syncing_with_correct_signal() {
        let mut driver = get_default_receiver();
        driver.state = RxState::Syncing;

        for i in 0..SYNC_SEQUENCE_BIT_LENGTH {
            let value = i & 0x1 != 1;
            let _ = driver.pin.set_state(value.into());

            match tick_one_bit(&mut driver) {
                Err(err) => panic!("Error encoutered during syncing on bit {i}: {err:?}"),
                Ok(_) => {}
            }
        }

        assert!(
            !matches!(driver.state, RxState::Syncing),
            "Driver not moved to the next state after syncing"
        );
    }

    #[test]
    fn syncing_with_incorrect_signal() {
        let mut driver = get_default_receiver();
        driver.state = RxState::Syncing;

        let mut error_flag = false;

        for _ in 0..SYNC_SEQUENCE_BIT_LENGTH {
            let value = true;
            let _ = driver.pin.set_state(value.into());

            if matches!(tick_one_bit(&mut driver), Err(_)) {
                error_flag = true;
                break;
            }
        }

        assert!(error_flag, "There was no error during syncing");

        assert!(
            matches!(driver.state, RxState::Idle),
            "Driver not cleaned up after sync error"
        );
    }

    #[test]
    fn read_correct_message_start() {
        let mut driver = get_default_receiver();
        driver.state = RxState::WaitingForStartByte;

        for i in 0..8 {
            let bit = (MESSAGE_START_BYTE >> i) & 0x1;
            driver.pin.set_state((bit == 1).into());

            let res = tick_one_bit(&mut driver);

            assert!(matches!(res, Ok(_)), "Error occured on bit #{i}: {bit}");
        }

        assert!(
            !matches!(driver.state, RxState::WaitingForStartByte),
            "Driver not moved to the next state from `RxState::WaitingForStartByte`"
        )
    }

    #[test]
    fn read_incorrect_message_start() {
        let mut driver = get_default_receiver();
        driver.state = RxState::WaitingForStartByte;

        let mut error_flag = false;

        for i in 0..8 {
            let bit = (0xff >> i) & 0x1;
            driver.pin.set_state((bit == 1).into());

            match tick_one_bit(&mut driver) {
                Ok(_) => {}
                Err(_) => {
                    error_flag = true;
                    break;
                }
            }
        }

        assert!(error_flag, "No error on incorrect start byte");

        assert!(
            matches!(driver.state, RxState::Idle),
            "Driver not cleaned up after message start byte error"
        );
    }

    #[test]
    fn read_message_size() {
        let mut driver = get_default_receiver();
        driver.state = RxState::ReadingSize;

        let size = 25u8;
        let mut encoded_size = [size, 0];
        encode_in_place(&mut encoded_size, 1).expect("Size buffer is too short for some reason?");

        for bit_n in 0..encoded_size.len() * 6 {
            let bit_i = bit_n % 6;
            let byte_i = bit_n / 6;

            let bit = (encoded_size[byte_i] >> bit_i) & 0x1;

            driver.pin.set_state((bit == 1).into());

            match tick_one_bit(&mut driver) {
                Err(ReceiverError::DecoderError(err)) => {
                    panic!("Decoder error on byte: {byte_i} on bit: {bit_i}. {err:?}")
                }
                _ => {}
            }
        }

        match driver.state {
            RxState::ReadingMessage { message_size } => {
                assert!(
                    message_size == size,
                    "Decoded size ({message_size}) does not equal initial size ({size})"
                );
            }
            state => {
                panic!("Driver is in incorrect state: {state:?}")
            }
        }
    }

    #[test]
    fn read_message() {
        const MESSAGE: &[u8] = b"A test message";
        let mut encoded_message = [0; MESSAGE.len() * 2];
        encoded_message[0..MESSAGE.len()].copy_from_slice(MESSAGE);
        encode_in_place(&mut encoded_message, MESSAGE.len())
            .expect("Unable to encode message for some reason");

        let mut driver = get_default_receiver();

        driver.state = RxState::ReadingMessage {
            message_size: MESSAGE.len() as u8,
        };

        for bit_n in 0..encoded_message.len() * 6 {
            let bit_i = bit_n % 6;
            let byte_i = bit_n / 6;

            let bit = (encoded_message[byte_i] >> bit_i) & 0x1;

            driver.pin.set_state((bit == 1).into());

            match tick_one_bit(&mut driver) {
                Err(ReceiverError::DecoderError(err)) => {
                    panic!("Decoder error on byte: {byte_i} on bit: {bit_i}. {err:?}")
                }
                _ => {}
            }
        }

        assert!(
            !matches!(driver.state, RxState::ReadingMessage { .. }),
            "Driver didn't move to the next state"
        );

        assert!(
            &driver.buffer[0..MESSAGE.len()] == MESSAGE,
            "Initial message and decoded message are not the same
            Initial: {MESSAGE:?}
            Decoded: {:?}",
            &driver.buffer[0..MESSAGE.len()]
        )
    }
}
