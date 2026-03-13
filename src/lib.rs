#![no_std]

pub mod consts;
pub mod driver;
pub mod mock_pin;

#[cfg(test)]
mod lib_test {
    use super::*;

    #[test]
    fn consts_bounds() {
        assert!(
            consts::MAX_MESSAGE_LENGTH <= 255,
            "Message length must fit into 1 byte. MAX_MESSAGE_LENGTH is too big"
        );

        assert!(
            consts::MAX_MESSAGE_LENGTH != 0,
            "Message length is 0. MAX_MESSAGE_LENGTH is too small"
        );
    }
}
