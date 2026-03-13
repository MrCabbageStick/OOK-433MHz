use crate::consts::{MAX_BUFFER_SIZE, MAX_MESSAGE_LENGTH, PREAMBLE, PREAMBLE_SIZE};
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

    // Receiver fields
    pub rx: RX,
    rx_buf: Vec<u8, MAX_BUFFER_SIZE>,

    // Misc
    mode: OokMode,
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

            rx,
            rx_buf: Vec::new(),

            mode: OokMode::Idle,
        }
    }

    /// Depending on the current mode receive, transmit data or do nothing
    pub fn tick(&mut self) {
        match self.mode {
            OokMode::Receive => {}
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
        self.set_tx_state(false);
    }

    /// Transmits next bit\
    /// If end of buffer is reached finishes transmission
    fn transmit(&mut self) {
        //! First sends then incerements

        let state = (self.tx_buf[self.tx_buf_index] >> self.tx_bit_index) & 1 == 1;
        self.set_tx_state(state);

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

    pub fn receiver_available(&self) -> bool {
        self.mode == OokMode::Idle
    }

    // pub fn receive(&mut self) -> Result<&[u8], ReceiverError> {}

    // ===== MISC =====

    /// Helper function if I were to add support for swapped hight and low  
    fn set_tx_state(&mut self, state: bool) {
        let _ = self.tx.set_state(PinState::from(state));
    }

    /// Helper function if I were to add support for swapped hight and low  
    fn read_rx_state(&self) -> bool {
        self.rx.is_high().unwrap_or(false)
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

        driver.send(data);

        let mut transmitted_data = Vec::<u8, MAX_MESSAGE_LENGTH>::new();
        let mut current_byte = 0u8;
        let mut nth_bit = 0u8;

        while driver.mode == OokMode::Transmit {
            driver.tick();

            current_byte |= (driver.tx.is_high().unwrap() as u8) << nth_bit;

            nth_bit += 1;
            if nth_bit >= 8 {
                nth_bit = 0;
                transmitted_data.push(current_byte).unwrap();
                current_byte = 0;
            }
        }

        assert!(&transmitted_data[PREAMBLE_SIZE..] == data);
    }
}

pub enum ReceiverError {
    MessageNotReady,
}
