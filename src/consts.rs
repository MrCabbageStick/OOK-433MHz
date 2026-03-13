/// Max size of tx and rx buffers\
/// Including encoding and headers
pub const MAX_BUFFER_SIZE: usize = 96;

pub const MESSAGE_START_BYTE: u8 = 0xCC;

pub const PREAMBLE: [u8; 1] = [MESSAGE_START_BYTE];

/// Preamble + Message size byte
pub const PREAMBLE_SIZE: usize = PREAMBLE.len() + 1;

pub const MAX_MESSAGE_LENGTH: usize = MAX_BUFFER_SIZE - PREAMBLE_SIZE;
