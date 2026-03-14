/// Max size of tx and rx buffers\
/// Including encoding and headers
pub const MAX_BUFFER_SIZE: usize = 96;

pub const MESSAGE_START_BYTE: u8 = 0xCC;

/// Alternating 1s and 0s
pub const SYNC_SEQUENCE: [u8; 2] = [0xAA, 0xAA];

pub const SYNC_SEQUENCE_BIT_LENGTH: u8 = SYNC_SEQUENCE.len() as u8 * 8;

/// Sync bytes + Message start byte
pub const PREAMBLE: [u8; 3] = [SYNC_SEQUENCE[0], SYNC_SEQUENCE[1], MESSAGE_START_BYTE];

/// Sync bytes + Message start byte
pub const PREAMBLE_SIZE: usize = PREAMBLE.len();

/// Preamble + Message size byte
pub const MESSAGE_OFFSET: usize = PREAMBLE_SIZE + 1;

pub const MAX_MESSAGE_LENGTH: usize = MAX_BUFFER_SIZE - MESSAGE_OFFSET;
