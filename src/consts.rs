/// Max size of tx and rx buffers\
/// Including encoding and headers
pub const MAX_BUFFER_SIZE: usize = 96;

pub const MESSAGE_START_BYTE: u8 = 0xCC;

/// Alternating 1s and 0s
pub const SYNC_SEQUENCE: [u8; 2] = [0xAA, 0xAA];

/// Alternating 1s ond 0s
pub const SYNC_BYTE: u8 = 0xAA;

/// Number of bits in sync signal needed to successfully sync
pub const SYNC_SEQUENCE_BIT_LENGTH: u8 = 2 * 8;

// SYNC_BYTE SYNC_BYTE MESSAGE_START
// 10101010  10101010  11001100
// Split into 6 bit chunks
// 101010 101010 101011 001100

/// Sync bytes + Message start byte, split into 6 bit chunks
pub const PREAMBLE: [u8; 4] = [0b101010, 0b101010, 0b101011, 0b001100];
// pub const PREAMBLE: [u8; 3] = [SYNC_SEQUENCE[0], SYNC_SEQUENCE[1], MESSAGE_START_BYTE];

/// Sync bytes + Message start byte
pub const PREAMBLE_SIZE: usize = PREAMBLE.len();

/// Message offset from buffer start (message length byte)
pub const MESSAGE_OFFSET: usize = 1;

/// Leave space for encoding and message size byte
pub const MAX_MESSAGE_LENGTH: usize = MAX_BUFFER_SIZE / 2 - 1;
