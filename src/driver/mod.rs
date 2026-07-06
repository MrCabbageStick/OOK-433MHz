pub mod pll;
pub mod receiver;
pub mod transmitter;

#[cfg(test)]
mod combined_tests {
    use embedded_hal::digital::v2::{InputPin, OutputPin};

    use crate::{
        driver::{
            receiver::{Receiver, ReceiverError::MessageNotReady},
            transmitter::Transmitter,
        },
        mock_pin::MockPin,
    };

    const TICKS_PER_BIT: u8 = 5;

    fn synchronize_pins(src: &MockPin, dst: &mut MockPin) {
        dst.set_state(src.is_high().expect("Unable to read mock pin state").into());
    }

    #[test]
    fn transmit_and_receive() {
        const MESSAGE: &[u8] = b"This is a test message, a long one, but could be longer!";

        let mut transmitter = Transmitter::<TICKS_PER_BIT, _>::new(MockPin::new());
        let mut receiver = Receiver::<TICKS_PER_BIT, _>::new(MockPin::new());

        // Prime the receiver
        let _ = receiver.receive();

        transmitter.send(MESSAGE);

        let message = loop {
            transmitter.transmit();

            if transmitter.is_idle() {
                break None;
            }

            synchronize_pins(&transmitter.pin, &mut receiver.pin);

            match receiver.receive() {
                Err(MessageNotReady) => {}
                Err(err) => panic!("Receiver error encoutered: {err:?}"),
                Ok(message) => break Some(message),
            }
        };

        let Some(message) = message else {
            panic!("Message not received")
        };

        assert!(
            message == MESSAGE,
            "Messages are different
            Initial:  {MESSAGE:?}
            Received: {message:?}"
        );
    }
}
