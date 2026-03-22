use crate::{
    consts::{
        MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_START_BYTE, PREAMBLE, PREAMBLE_SIZE,
        SYNC_SEQUENCE_BIT_LENGTH,
    },
    data_coding::radio_head_4b6b,
};
use embedded_hal::digital::v2::{InputPin, OutputPin, PinState};
use heapless::Vec;

#[derive(PartialEq, Eq)]
pub enum OokMode {
    Idle,
    Transmit,
    Receive,
}

pub struct OokDriver<TX, RX>
where
    TX: OutputPin,
    RX: InputPin,
{
    // Transmitter fields
    pub tx: TX,
    /// Buffer for message to be sent
    tx_buf: Vec<u8, MAX_BUFFER_SIZE>,
    /// Index of byte in `tx_buf` or PREAMBLE
    tx_buf_index: usize,
    /// Index of bit in byte of `tx_buf`
    tx_bit_index: u8,
    tx_current_tick: u8,
    tx_preamble_sent: bool,

    // Receiver fields
    pub rx: RX,
    rx_buf: Vec<u8, MAX_BUFFER_SIZE>,
    rx_byte: u8,
    rx_bit_index: usize,
    rx_message_length: u8,
    rx_message_received: bool,
    rx_message_started: bool,
    rx_current_tick: u8,
    rx_n_ones_in_tick: u8,
    rx_detected_one: bool,

    rx_sync_n_correct_bits: u8,
    rx_sync_bits: u8,
    rx_synced: bool,
    rx_decoder: radio_head_4b6b::RunningDecoder,
    rx_error: Option<FatalReceiverError>,

    // Misc
    mode: OokMode,
    ticks_per_bit: u8,
}

impl<TX, RX> OokDriver<TX, RX>
where
    TX: OutputPin,
    RX: InputPin,
{
    pub fn new(tx: TX, rx: RX) -> Self {
        // Insert preamble into tx_buf, it should stay there forever
        // let tx_buf = Vec::from_slice(&PREAMBLE)
        //     .expect("Unable to fit preamble into tx_buf, MAX_BUFFER_SIZE might be smaller than PREAMBLE_SIZE");

        Self {
            tx,
            tx_buf: Vec::new(),
            tx_bit_index: 0,
            tx_buf_index: 0,
            tx_current_tick: 0,
            tx_preamble_sent: false,

            rx,
            rx_buf: Vec::new(),
            rx_byte: 0,
            rx_bit_index: 0,
            rx_message_length: 0,
            rx_message_received: false,
            rx_message_started: false,
            rx_current_tick: 0,
            rx_n_ones_in_tick: 0,
            rx_detected_one: false,

            rx_sync_bits: 0,
            rx_sync_n_correct_bits: 0,
            rx_synced: false,
            rx_decoder: radio_head_4b6b::RunningDecoder::new(),
            rx_error: None,

            mode: OokMode::Idle,
            ticks_per_bit: 8,
        }
    }

    /// Depending on the current mode receive, transmit data or do nothing
    pub fn tick(&mut self) {
        match self.mode {
            OokMode::Receive => {
                self.receive();
            }
            OokMode::Transmit => {
                self.transmit();
            }
            OokMode::Idle => {}
        }
    }

    // ===== TRANSMISSION =====

    /// Cleanup after transmission is finished
    fn end_transmission(&mut self) {
        self.tx_bit_index = 0;
        self.tx_buf_index = 0;
        self.mode = OokMode::Idle;
        self.tx_current_tick = 0;
        self.tx_preamble_sent = false;
        self.set_tx_state(false);
    }

    /// Transmits next bit\
    /// If end of buffer is reached finishes transmission
    fn transmit(&mut self) {
        let byte_source = if !self.tx_preamble_sent {
            // Retrieve bytes from preamble
            &PREAMBLE
        } else {
            // Retrive bytes from buffer
            self.tx_buf.as_slice()
        };

        // State bit as bool, most significant first
        // Use only 6 bits of a byte
        let state = (byte_source[self.tx_buf_index] >> (5 - self.tx_bit_index)) & 1 == 1;

        self.set_tx_state(state);

        self.tx_current_tick += 1;
        // Skip if not enough ticks to send a bit
        if self.tx_current_tick < self.ticks_per_bit {
            return;
        }

        self.tx_current_tick = 0;

        self.tx_bit_index += 1;
        // If byte ends
        if self.tx_bit_index >= 6 {
            self.tx_bit_index = 0;

            // Increment buf index
            self.tx_buf_index += 1;

            // If preamble sent
            if !self.tx_preamble_sent && self.tx_buf_index == PREAMBLE_SIZE {
                self.tx_preamble_sent = true;
                self.tx_buf_index = 0;
                return;
            }

            // If end of buffer reached end transmission
            if self.tx_buf_index >= self.tx_buf.len() {
                self.end_transmission();
            }
        }
    }

    /// Put bytes into `tx_buffer` and set mode to Transmit\
    /// If bytes length is larger than MAX_MESSAGE_LENGTH it will be truncated\
    /// Returns number of bytes put into the buffer
    pub fn send(&mut self, bytes: &[u8]) -> usize {
        // Clear buffer (maybe move to end_transittion)
        self.tx_buf.clear();

        if bytes.len() == 0 {
            return 0;
        }

        // Truncate bytes if needed
        let safe_bytes = if bytes.len() > MAX_MESSAGE_LENGTH {
            &bytes[..MAX_MESSAGE_LENGTH]
        } else {
            bytes
        };

        // Push message length to buf
        self.tx_buf
            .push(safe_bytes.len() as u8)
            .expect("Unable to push message length into tx_buf");

        // Push data to buf
        self.tx_buf.extend(safe_bytes.iter().copied());
        // Make space for encoding
        self.tx_buf.extend(safe_bytes.iter().copied());

        self.tx_buf
            .push(0)
            .expect("Unable to make space for encoded size");

        radio_head_4b6b::encode_in_place(&mut self.tx_buf, safe_bytes.len() + 1)
            .expect("tx_buf cannot be encoded");

        self.mode = OokMode::Transmit;

        safe_bytes.len()
    }

    // ===== RECEIVING =====

    /// Cleanup before reading into the `rx_buffer`
    fn setup_receiver(&mut self) {
        self.mode = OokMode::Receive;

        self.rx_message_received = false;
        self.rx_message_length = 0;
        self.rx_current_tick = 0;
        self.rx_n_ones_in_tick = 0;
        self.rx_detected_one = false;
        self.rx_byte = 0;
        self.rx_message_started = false;

        self.rx_sync_bits = 0;
        self.rx_sync_n_correct_bits = 0;
        self.rx_synced = false;
        self.rx_decoder.reset();
        self.rx_error = None;

        self.rx_buf.clear();
    }

    /// Check if receiver is available, if so, prime it for receiving data\
    /// This methods clears contents of `rx_buf` so any message stored in it
    /// will be removed
    pub fn start_receiving(&mut self) -> bool {
        if self.mode == OokMode::Idle {
            self.setup_receiver();
        }

        self.mode == OokMode::Receive
    }

    /// Checks if message is ready to be read.\
    /// Returns reference to `rx_buf`
    pub fn get_message(&mut self) -> Result<&[u8], ReceiverError> {
        if self.rx_message_received {
            return Ok(&self.rx_buf);
        }

        Err(ReceiverError::MessageNotReady)
    }

    /// Handle tick to bit conversion\
    /// Return None if not enought ticks read to make a byte
    fn get_bit(&mut self, state: bool) -> Option<u8> {
        self.rx_n_ones_in_tick += state as u8;

        self.rx_current_tick += 1;

        // Skip if not enough ticks to read a bit
        if self.rx_current_tick < self.ticks_per_bit {
            return None;
        }

        self.rx_current_tick = 0;

        // At least half of ticks should be 1 for bit to be 1
        let state_from_ticks = (self.rx_n_ones_in_tick >= (self.ticks_per_bit / 2)) as u8;
        self.rx_n_ones_in_tick = 0;

        Some(state_from_ticks)
    }

    /// Take bit and check if receiver is synchronized
    fn get_synced(&mut self, bit: u8) -> bool {
        if self.rx_synced {
            return true;
        }

        // Push bit
        self.rx_sync_bits <<= 1;
        self.rx_sync_bits |= bit;

        // Check if ls bits are 0b10 or 0b101,
        // as this matches a sequence of repeating 1s and 0s
        if self.rx_sync_bits & 0b11 == 0b10 || self.rx_sync_bits & 0b111 == 0b101 {
            self.rx_sync_n_correct_bits += 1;

            // SYNC_SEQUENCE_BIT_LENGTH - 1, because first bit isn't counted
            if self.rx_sync_n_correct_bits >= SYNC_SEQUENCE_BIT_LENGTH - 1 {
                // Don't return true from here, because this is the last bit
                // of sync sequnce
                self.rx_synced = true;
            }
        } else {
            // It wasn't sync sequence after all
            self.rx_sync_n_correct_bits = 0;
        }

        return false;
    }

    /// Take bit and return a byte\
    /// Return None when not enough bits to make a byte
    fn get_byte(&mut self, bit: u8) -> Option<u8> {
        // Append bit to byte
        // Use onnly 6 ls bits
        self.rx_byte |= bit << (5 - self.rx_bit_index);
        self.rx_bit_index += 1;

        // Full 6 bits
        if self.rx_bit_index >= 6 {
            self.rx_bit_index = 0;
            return Some(self.rx_byte);
        }

        None
    }

    fn message_started(&mut self, bit: u8) -> bool {
        if self.rx_message_started {
            return true;
        }

        self.rx_byte |= bit << (5 - self.rx_bit_index);
        self.rx_bit_index += 1;

        // Full byte
        if self.rx_bit_index >= 8 {
            if self.rx_byte == MESSAGE_START_BYTE {
                self.rx_message_started = true;
                self.rx_byte = 0;
                self.rx_bit_index = 0;
                // Return false so this bit ius not used in message data
                return false;
            }
            // TODO: I should discard message here probably
        }

        false
    }

    /// Receive bit from `rx` and put it into `rx_buf`
    fn receive(&mut self) {
        if self.rx_error.is_some() {
            return;
        }

        let rx_state = self.read_rx_state();

        // Wait for at least one 1 to be detected
        if !self.rx_detected_one {
            if !rx_state {
                return;
            } else {
                self.rx_detected_one = true;
            }
        }

        let Some(bit) = self.get_bit(rx_state) else {
            return;
        };

        // Syncing
        if !self.get_synced(bit) {
            return;
        }

        if !self.message_started(bit) {
            return;
        }

        let Some(_byte) = self.get_byte(bit) else {
            return;
        };

        // if !self.rx_message_started {
        //     // Check for start byte
        //     if self.rx_byte == MESSAGE_START_BYTE {
        //         self.rx_message_started = true;
        //         self.rx_byte = 0;
        //     }

        //     // I think I should discard message here
        //     return;
        // }

        match self.rx_decoder.next_nibble(self.rx_byte) {
            Ok(decoded_byte) => {
                self.rx_byte = decoded_byte;
            }
            Err(err) => match err {
                radio_head_4b6b::RunningDecoderError::ByteNotReady => {
                    // Wait for next nibble to form a byte
                    self.rx_byte = 0;
                    return;
                }
                radio_head_4b6b::RunningDecoderError::UnknownSymbol(s) => {
                    self.handle_fatal_receiver_error(FatalReceiverError::UnknownReceiverSymbol(s));
                    return;
                }
            },
        }

        // Message length not set up yet
        if self.rx_message_length == 0 {
            self.rx_message_length = self.rx_byte;
        }

        self.rx_buf.push(self.rx_byte).unwrap();
        self.rx_byte = 0;

        // Message end reached or message wouldn't fit and gets truncated
        if self.rx_buf.len() >= self.rx_message_length as usize + 1
            || self.rx_buf.len() >= MAX_BUFFER_SIZE
        {
            self.rx_message_received = true;
            self.mode = OokMode::Idle;
        }
    }

    fn handle_fatal_receiver_error(&mut self, error: FatalReceiverError) {
        self.rx_error = Some(error);
        self.mode = OokMode::Idle;
    }

    pub fn get_read_error(&self) -> &Option<FatalReceiverError> {
        &self.rx_error
    }

    // ===== MISC =====

    /// Helper function if I were to add support for swapped hight and low  
    fn set_tx_state(&mut self, state: bool) {
        let _ = self.tx.set_state(PinState::from(state));
    }

    /// Helper function if I were to add support for swapped hight and low  
    fn read_rx_state(&self) -> bool {
        self.rx.is_high().unwrap_or(false)
    }

    pub fn is_idle(&self) -> bool {
        return self.mode == OokMode::Idle;
    }
}

pub enum ReceiverError {
    MessageNotReady,
}

pub enum FatalReceiverError {
    UnknownReceiverSymbol(u8),
}

#[cfg(test)]
mod tests {
    use crate::{
        consts::{MESSAGE_OFFSET, SYNC_SEQUENCE},
        mock_pin::MockPin,
    };

    use super::*;

    #[test]
    fn tx_buf_correct_size_message() {
        let mut driver = OokDriver::new(MockPin::new(), MockPin::new());

        let data = b"Hello there";
        let n_sent_bytes = driver.send(data);

        assert!(n_sent_bytes == data.len());
        assert!(driver.mode == OokMode::Transmit);

        radio_head_4b6b::decode_in_place(&mut driver.tx_buf).expect("Unable to decode tx_buf");

        assert!(&driver.tx_buf[1..data.len() + 1] == data);

        let data_size = driver.tx_buf[0];
        assert!(data_size == data.len() as u8);
    }

    const TOO_LARGE_MESSAGE_SIZE: usize = MAX_MESSAGE_LENGTH + 1;

    #[test]
    fn tx_buf_message_too_large() {
        let mut driver = OokDriver::new(MockPin::new(), MockPin::new());

        let mut data = Vec::<u8, TOO_LARGE_MESSAGE_SIZE>::new();
        data.resize(TOO_LARGE_MESSAGE_SIZE, b'^').unwrap();

        let n_sent_bytes = driver.send(data.as_slice());

        radio_head_4b6b::decode_in_place(&mut driver.tx_buf).expect("Unable to decode tx_buf");

        assert!(n_sent_bytes == MAX_MESSAGE_LENGTH, "{}", n_sent_bytes);
        assert!(&driver.tx_buf[1..n_sent_bytes + 1] == &data[..MAX_MESSAGE_LENGTH]);
    }

    #[test]
    fn transmit_ticking() {
        let mut driver = OokDriver::new(MockPin::new(), MockPin::new());

        let data = b"Hello there!";

        let n_bytes_sent = driver.send(data);

        let mut transmitted_data = Vec::<u8, MAX_MESSAGE_LENGTH>::new();
        let mut current_byte = 0u8;
        let mut nth_bit = 0u8;
        let mut n_ticks = 0u8;

        let mut n_bytes = 0usize;

        let mut decoder = radio_head_4b6b::RunningDecoder::new();

        while driver.mode == OokMode::Transmit {
            driver.tick();

            n_ticks += 1;

            current_byte |= (driver.tx.is_high().unwrap() as u8) << (5 - nth_bit);

            if n_ticks < driver.ticks_per_bit {
                continue;
            }

            n_ticks = 0;

            nth_bit += 1;

            if nth_bit >= 6 {
                nth_bit = 0;

                // After sync bytes and message start byte
                if n_bytes >= PREAMBLE_SIZE {
                    match decoder.next_nibble(current_byte) {
                        Ok(byte) => transmitted_data.push(byte).unwrap(),
                        Err(err) => match err {
                            radio_head_4b6b::RunningDecoderError::ByteNotReady => {}
                            radio_head_4b6b::RunningDecoderError::UnknownSymbol(s) => {
                                panic!("Unable to decode incoming data, unknown symbol: 0x{:x}", s);
                            }
                        },
                    }
                }

                current_byte = 0;
                n_bytes += 1;
            }
        }

        assert!(
            &transmitted_data[MESSAGE_OFFSET..] == data,
            "{:?}",
            transmitted_data
        );
        assert!(&transmitted_data[MESSAGE_OFFSET - 1] == &(n_bytes_sent as u8));
    }

    const MESSAGES: [&[u8]; 3] = [
        b"Hello there!",
        b"It's been a while",
        b"abcdefghijklmnopqrstuwxyz",
    ];

    #[test]
    fn transmit_multiple_messages() {
        let mut transmitter = OokDriver::new(MockPin::new(), MockPin::new());

        for message in MESSAGES {
            let expected_tick_count = (
                // Encoded data and length byte, with 6 bits per byte
                (message.len() + 1) * 2 * 6 
                // Preamble with 6 bits per byte
                + PREAMBLE_SIZE * 6
            ) * transmitter.ticks_per_bit as usize;

            let mut n_ticks = 0usize;

            transmitter.send(message);

            while transmitter.mode == OokMode::Transmit {
                transmitter.tick();

                assert!(
                    n_ticks < expected_tick_count,
                    "Transmitter ticked too many times to send a message\n"
                );

                n_ticks += 1;
            }

            assert!(
                n_ticks == expected_tick_count,
                "Transmitter ticked incorrect number of times to send a message"
            );
        }
    }

    #[test]
    fn receiver_test() {
        let mut transmitter = OokDriver::new(MockPin::new(), MockPin::new());
        let mut receiver = OokDriver::new(MockPin::new(), MockPin::new());

        let data = b"^Hello there!";

        let n_bytes = transmitter.send(data);

        // Prime receiver
        receiver.start_receiving();

        let msg = loop {
            // Return if message ready
            if let Ok(msg) = receiver.get_message() {
                break msg;
            }

            transmitter.tick();

            if let Some(err) = receiver.get_read_error() {
                match err {
                    FatalReceiverError::UnknownReceiverSymbol(sym) => {
                        panic!(
                            "Error while reading data, unknown symbol: 0x{:x}\nrx: {:?}\ntx: {:?}",
                            sym, receiver.rx_buf, transmitter.tx_buf
                        )
                    }
                }
            }

            // Sync state on receiver's rx pin and transmitter's tx pin
            receiver.rx.sync_with(&transmitter.tx);
            receiver.tick();
        };

        // Check if length matches
        assert!(msg[0] as usize == n_bytes, "{:?}", msg);
        // Check if data was transmitted successfully
        assert!(&msg[1..] == data, "{:?}\n{:?}", &msg[1..], data);
    }

    #[test]
    fn syncing() {
        let mut possible_preceding_bytes: [u8; 256] = [0; 256];
        (0..=255)
            .enumerate()
            .for_each(|(i, byte)| possible_preceding_bytes[i] = byte);

        for possible_byte in possible_preceding_bytes {
            let signal = [possible_byte, SYNC_SEQUENCE[0], SYNC_SEQUENCE[1]];

            let mut receiver = OokDriver::new(MockPin::new(), MockPin::new());

            for byte in signal {
                let mut bit_index = 0;

                while bit_index < 8 {
                    receiver.get_synced((byte >> (7 - bit_index)) & 0x1);
                    bit_index += 1;
                }
            }

            assert!(
                receiver.rx_synced,
                "Receiver not synced with byte {:x} preceeding the sync signal",
                possible_byte
            );
        }
    }
}
