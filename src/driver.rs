use crate::consts::{
    MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, MESSAGE_START_BYTE, PREAMBLE, PREAMBLE_SIZE,
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
    tx_buf: Vec<u8, MAX_BUFFER_SIZE>,
    /// Index of byte in `tx_buf`
    tx_buf_index: usize,
    /// Index of bit in byte of `tx_buf`
    tx_bit_index: u8,
    tx_current_tick: u8,

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
        let tx_buf = Vec::from_slice(&PREAMBLE)
            .expect("Unable to fit preamble into tx_buf, MAX_BUFFER_SIZE might be smaller than PREAMBLE_SIZE");

        Self {
            tx,
            tx_buf,
            tx_bit_index: 0,
            tx_buf_index: 0,
            tx_current_tick: 0,

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
        self.set_tx_state(false);
    }

    /// Transmits next bit\
    /// If end of buffer is reached finishes transmission
    fn transmit(&mut self) {
        //! First sends then incerements

        let state = (self.tx_buf[self.tx_buf_index] >> self.tx_bit_index) & 1 == 1;
        self.set_tx_state(state);

        self.tx_current_tick += 1;
        // Skip if not enough ticks to read a bit
        if self.tx_current_tick < self.ticks_per_bit {
            return;
        }

        self.tx_current_tick = 0;

        self.tx_bit_index += 1;
        // If byte ends
        if self.tx_bit_index >= 8 {
            self.tx_bit_index = 0;

            // Increment buf index
            self.tx_buf_index += 1;
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
        // Leave preamble inside buffer
        self.tx_buf.truncate(PREAMBLE_SIZE);

        if bytes.len() == 0 {
            return 0;
        }

        // Truncate bytes if needed
        let safe_bytes = if bytes.len() > MAX_MESSAGE_LENGTH {
            &bytes[..MAX_MESSAGE_LENGTH]
        } else {
            bytes
        };

        self.tx_buf
            .push(safe_bytes.len() as u8)
            .expect("Unable to push message length into tx_buf");

        self.tx_buf.extend(safe_bytes.iter().copied());

        self.mode = OokMode::Transmit;

        safe_bytes.len()
    }

    // ===== RECEIVING =====

    /// Cleanup before reading into the `rx_buffer`
    fn start_receiving(&mut self) {
        self.mode = OokMode::Receive;
        self.rx_message_received = false;
        self.rx_message_length = 0;
        self.rx_current_tick = 0;
        self.rx_n_ones_in_tick = 0;
        self.rx_detected_one = false;
        self.rx_buf.clear();
    }

    /// Check if receiver is available, if so, prime it for receiving data
    pub fn receiver_available(&mut self) -> bool {
        if self.mode == OokMode::Idle {
            self.start_receiving();
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

    /// Receive bit from `rx` and put it into `rx_buf`
    fn receive(&mut self) {
        let rx_state = self.read_rx_state();
        self.rx_n_ones_in_tick += rx_state as u8;

        // Wait for at least one 1 to be detected
        // This might help with synchronization
        if !self.rx_detected_one {
            if !rx_state {
                return;
            } else {
                self.rx_detected_one = true;
            }
        }

        self.rx_current_tick += 1;

        // Skip if not enough ticks to read a bit
        if self.rx_current_tick < self.ticks_per_bit {
            return;
        }

        self.rx_current_tick = 0;

        // At least half of ticks should be 1 for bit to be 1
        let state_from_ticks = (self.rx_n_ones_in_tick >= (self.ticks_per_bit / 2)) as u8;
        self.rx_n_ones_in_tick = 0;

        if !self.rx_message_started {
            // Push bit onto byte
            // This way its possible to detect start byte
            self.rx_byte >>= 1;
            self.rx_byte |= state_from_ticks << 7;

            // Check if message start byte reached
            if self.rx_byte == MESSAGE_START_BYTE {
                self.rx_message_started = true;
                self.rx_buf.push(self.rx_byte).unwrap();
                self.rx_byte = 0;
            }

            return;
        }

        // Append bit to byte
        self.rx_byte |= state_from_ticks << self.rx_bit_index;
        self.rx_bit_index += 1;

        // Full byte
        if self.rx_bit_index >= 8 {
            self.rx_bit_index = 0;

            // Message length not set up yet
            if self.rx_message_length == 0 {
                self.rx_message_length = self.rx_byte;
            }

            self.rx_buf.push(self.rx_byte).unwrap();
            self.rx_byte = 0;

            // Message end reached or message wouldn't fit and gets truncated
            if self.rx_buf.len() >= self.rx_message_length as usize + PREAMBLE_SIZE
                || self.rx_buf.len() >= MAX_MESSAGE_LENGTH
            {
                self.rx_message_received = true;
                self.mode = OokMode::Idle
            }
        }
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

#[cfg(test)]
mod tests {
    use crate::mock_pin::MockPin;

    use super::*;

    const TOO_LARGE_MESSAGE_SIZE: usize = MAX_MESSAGE_LENGTH + 1;

    #[test]
    fn tx_buf_correct_size_message() {
        let mut driver = OokDriver::new(MockPin::new(), MockPin::new());

        let data = b"Hello there";
        let n_sent_bytes = driver.send(data);

        assert!(n_sent_bytes == data.len());
        assert!(driver.mode == OokMode::Transmit);
        assert!(&driver.tx_buf[PREAMBLE_SIZE..PREAMBLE_SIZE + data.len()] == data);

        let data_size = driver.tx_buf[PREAMBLE_SIZE - 1];
        assert!(data_size == data.len() as u8);
    }

    #[test]
    fn tx_buf_message_too_large() {
        let mut driver = OokDriver::new(MockPin::new(), MockPin::new());

        let mut data = Vec::<u8, TOO_LARGE_MESSAGE_SIZE>::new();
        data.resize(TOO_LARGE_MESSAGE_SIZE, b'^').unwrap();

        let n_sent_bytes = driver.send(data.as_slice());

        assert!(n_sent_bytes == MAX_MESSAGE_LENGTH, "{}", n_sent_bytes);
        assert!(
            &driver.tx_buf[PREAMBLE_SIZE..PREAMBLE_SIZE + n_sent_bytes]
                == &data[..MAX_MESSAGE_LENGTH]
        );
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

        while driver.mode == OokMode::Transmit {
            driver.tick();

            n_ticks += 1;

            if n_ticks < driver.ticks_per_bit {
                continue;
            }

            n_ticks = 0;

            current_byte |= (driver.tx.is_high().unwrap() as u8) << nth_bit;

            nth_bit += 1;

            if nth_bit >= 8 {
                nth_bit = 0;
                transmitted_data.push(current_byte).unwrap();
                current_byte = 0;
            }
        }

        assert!(&transmitted_data[PREAMBLE_SIZE..] == data);
        assert!(&transmitted_data[PREAMBLE_SIZE - 1] == &(n_bytes_sent as u8));
    }

    #[test]
    fn receiver_test() {
        let mut transmitter = OokDriver::new(MockPin::new(), MockPin::new());
        let mut receiver = OokDriver::new(MockPin::new(), MockPin::new());

        let data = b"Hello there!";

        let n_bytes = transmitter.send(data);

        // Prime receiver
        receiver.receiver_available();

        let msg = loop {
            // Return if message ready
            if let Ok(msg) = receiver.get_message() {
                break msg;
            }

            transmitter.tick();
            // Sync state on receiver's rx pin and transmitter's tx pin
            receiver.rx.sync_with(&transmitter.tx);
            receiver.tick();
        };

        // Check for preambule
        assert!(msg[0] == MESSAGE_START_BYTE);
        // Check if length matches
        assert!(msg[PREAMBLE_SIZE - 1] as usize == n_bytes);
        // Check if data was transmitted successfully
        assert!(&msg[PREAMBLE_SIZE..] == data);
    }
}

pub enum ReceiverError {
    MessageNotReady,
}
