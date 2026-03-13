/// Max size of tx and rx buffers\
/// Including encoding and headers
pub const MAX_BUFFER_SIZE: usize = 96;

pub const PREAMBLE_SIZE: usize = 0;

pub const MAX_MESSAGE_LENGTH: usize = MAX_BUFFER_SIZE - PREAMBLE_SIZE;
