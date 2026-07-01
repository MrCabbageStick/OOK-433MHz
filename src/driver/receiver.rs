use core::{error::Error, fmt::Display};

use embedded_hal::digital::v2::InputPin;
use ufmt::derive::uDebug;

use crate::{
    consts::{
        MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_OFFSET, MESSAGE_START_BYTE,
        SYNC_SEQUENCE_BIT_LENGTH,
    },
    data_coding::radio_head_4b6b::{RunningDecoder, RunningDecoderError},
    driver::{pll::Pll, receiver::ReceiverError::DecoderError},
};

#[derive(Debug, uDebug, Clone, Copy)]
pub enum RxState {
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
    pub pin: Pin,
    ticks: u8,
    current_byte: u8,

    decoder: RunningDecoder,
    pll: Pll<TICKS_PER_BIT>,
}

impl<Pin: InputPin, const TICKS_PER_BIT: u8> Receiver<TICKS_PER_BIT, Pin> {
    pub fn new(pin: Pin) -> Self {
        Self {
            buffer: [0; MAX_BUFFER_SIZE],
            bit_index: 0,
            state: RxState::Idle,
            ticks: 0,
            pin,
            current_byte: 0,
            buffer_byte_index: 0,
            decoder: RunningDecoder::new(),
            pll: Pll::new(),
        }
    }

    pub fn state(&self) -> RxState {
        self.state
    }

    pub fn cleanup(&mut self) {
        self.bit_index = 0;
        self.ticks = 0;
        self.state = RxState::Idle;
        self.current_byte = 0;
        self.decoder.reset();
        self.buffer_byte_index = 0;
        self.pll.clear();
    }

    fn get_pin_state(&self) -> bool {
        self.pin.is_high().unwrap_or(false)
    }

    /// Returns read bytes if message is ready,
    /// or None if it's not. If receiver is idle
    /// starts receiving
    pub fn receive(&mut self) -> Result<&[u8], ReceiverError> {
        let pin_state = self.get_pin_state();
        let bit = self.pll.tick(pin_state);

        match self.state {
            RxState::Idle => self.state = RxState::WaitingForOne,
            RxState::WaitingForOne => {
                // If one detected move to the next state
                if pin_state {
                    self.state = RxState::Syncing;
                    // Tick was already from the transmitter's sync sequence
                    self.ticks += 1;
                }
            }
            RxState::Syncing => match self.sync(bit) {
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
            RxState::WaitingForStartByte => match self.wait_start_byte(bit) {
                Ok(true) => self.state = RxState::ReadingSize,
                Ok(false) => {}
                Err(err) => {
                    self.cleanup();
                    return Err(err);
                }
            },
            RxState::ReadingSize => match self.read_message_size(bit) {
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
            RxState::ReadingMessage { message_size } => {
                match self.read_message(message_size, bit) {
                    Ok(true) => self.state = RxState::MessageReceived { message_size },
                    Ok(false) => {}
                    Err(err) => {
                        self.cleanup();
                        return Err(err);
                    }
                }
            }
            RxState::MessageReceived { message_size } => {
                self.cleanup();
                return Ok(&self.buffer[..message_size as usize]);
            }
        }

        Err(ReceiverError::MessageNotReady)
    }

    /// Wait for repeating sequence of 1s and 0s.
    /// If bits are not alternating return `SyncError`,
    /// if not enought bits are present to sync return `false`,
    /// otherwise return `true`
    fn sync(&mut self, bit: Option<u8>) -> Result<bool, ReceiverError> {
        // Get bit from ticks
        let Some(state) = bit else {
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
    fn wait_start_byte(&mut self, bit: Option<u8>) -> Result<bool, ReceiverError> {
        let Some(bit) = bit else {
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
    fn read_message_size(&mut self, bit: Option<u8>) -> Result<(bool, u8), ReceiverError> {
        let Some(bit) = bit else {
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

    fn read_message(&mut self, message_size: u8, bit: Option<u8>) -> Result<bool, ReceiverError> {
        let Some(bit) = bit else {
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

#[derive(Debug, uDebug, PartialEq, Eq)]
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
        consts::{
            MESSAGE_OFFSET, MESSAGE_START_BYTE, SYNC_BYTE, SYNC_SEQUENCE, SYNC_SEQUENCE_BIT_LENGTH,
        },
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
        );
    }

    #[test]
    fn full_receiver_test() {
        // === SETUP === //
        const DATA_MESSAGE_OFFSET: usize = 5;
        const MESSAGE: &[u8] = b"This is a test message";
        // Sync sequence size + message start byte + size byte (encoded on 2 bytes) + encoded message size
        const FULL_DATA_LENGTH: usize = SYNC_SEQUENCE.len() + 1 + 2 + MESSAGE.len() * 2;
        let mut full_data = [0u8; FULL_DATA_LENGTH];
        // Put epxtected byte sequence
        full_data[0] = 0b01010101;
        full_data[1] = 0b01010101;
        full_data[2] = MESSAGE_START_BYTE;
        // Put message size
        full_data[3] = MESSAGE.len() as u8;
        // Encode message size
        encode_in_place(&mut full_data[3..5], 1).expect("Unable to encode message size");
        // Put message
        full_data[DATA_MESSAGE_OFFSET..DATA_MESSAGE_OFFSET + MESSAGE.len()]
            .copy_from_slice(MESSAGE);
        // Encode message part
        encode_in_place(&mut full_data[DATA_MESSAGE_OFFSET..], MESSAGE.len())
            .expect("Unable to encode message");

        // === TEST === //
        let mut driver = get_default_receiver();
        // Prime the driver
        let _ = driver.receive();

        for byte_i in 0..full_data.len() {
            // Number of bits in a 'byte'
            // 8 for sync and start_byte,
            // 6 for encoded data
            let n_bits_from_byte = if byte_i < 3 { 8 } else { 6 };

            for bit_i in 0..n_bits_from_byte {
                let state = (full_data[byte_i] >> bit_i) & 0x1;
                driver.pin.set_state((state == 1).into());

                match tick_one_bit(&mut driver) {
                    Ok(_) => {}
                    Err(err) => {
                        panic!(
                            "Receiver ecountered an error on byte {byte_i} in bit {bit_i}: {err:?}"
                        );
                    }
                }
            }
        }

        let message = driver
            .receive()
            .expect("Error occured when message should be present");

        assert!(
            message == MESSAGE,
            "Messages are different
            Initial:  {MESSAGE:?}
            Received: {message:?}"
        );
    }
}
