pub mod radio_head_4b6b {
    /// 4 bit to 6 bit symbol converter table
    /// Used to convert the high and low nybbles of the transmitted data
    /// into 6 bit symbols for transmission. Each 6-bit symbol has 3 1s and 3 0s
    /// with at most 3 consecutive identical bits\
    /// source: https://github.com/PaulStoffregen/RadioHead/blob/master/RH_ASK.cpp
    pub const SYMBOLS: [u8; 16] = [
        0xd, 0xe, 0x13, 0x15, 0x16, 0x19, 0x1a, 0x1c, 0x23, 0x25, 0x26, 0x29, 0x2a, 0x2c, 0x32,
        0x34,
    ];

    pub fn symbol_to_half_byte(symbol: u8) -> Result<u8, RH4b6bDecodeError> {
        match SYMBOLS.iter().position(|s| *s == symbol) {
            Some(i) => Ok(i as u8),
            None => Err(RH4b6bDecodeError::UnknownSymbol(symbol)),
        }
    }

    pub fn encode_in_place(data: &mut [u8], data_length: usize) -> Result<(), RH4b6bEncodeError> {
        // Data will take double the amount of space it did before encoding

        if data.len() < data_length * 2 {
            return Err(RH4b6bEncodeError::DataBufferTooShort);
        }

        // Move data around starting from the end
        // every item on position n goes to position n*2
        for i in (0..data_length).rev() {
            data[i * 2] = data[i];
        }

        // Encode data on 2 consecutive bytes
        for chunk in data.chunks_mut(2) {
            chunk[1] = SYMBOLS[(chunk[0] >> 4) as usize];
            chunk[0] = SYMBOLS[(chunk[0] & 0xf) as usize];
        }

        Ok(())
    }

    pub fn decode_in_place(data: &mut [u8]) -> Result<(), RH4b6bDecodeError> {
        // Not even
        if data.len() & 0x1 == 1 {
            return Err(RH4b6bDecodeError::DataSizeNotEven);
        }

        for i in 0..data.len() / 2 {
            let ls_symbol = data[i * 2];
            let ms_symbol = data[i * 2 + 1];

            data[i] = symbols_to_byte(ls_symbol, ms_symbol)?;
        }

        Ok(())
    }

    pub fn symbols_to_byte(ls_symbol: u8, ms_symbol: u8) -> Result<u8, RH4b6bDecodeError> {
        let mut byte = symbol_to_half_byte(ls_symbol)?;
        byte |= (symbol_to_half_byte(ms_symbol)? << 4) as u8;

        Ok(byte)
    }

    pub struct RunningDecoder {
        byte: u8,
        ms_nibble_next: bool,
    }

    impl RunningDecoder {
        pub fn new() -> Self {
            Self {
                byte: 0,
                ms_nibble_next: false,
            }
        }

        pub fn next_nibble(&mut self, nibble: u8) -> Result<u8, RunningDecoderError> {
            if self.ms_nibble_next {
                self.byte |= symbol_to_half_byte(nibble)
                    .or_else(|_| Err(RunningDecoderError::UnknownSymbol(nibble)))?
                    << 4;
                self.ms_nibble_next = false;

                Ok(self.byte)
            } else {
                self.byte = symbol_to_half_byte(nibble)
                    .or_else(|_| Err(RunningDecoderError::UnknownSymbol(nibble)))?;

                self.ms_nibble_next = true;
                Err(RunningDecoderError::ByteNotReady)
            }
        }
    }

    pub enum RunningDecoderError {
        UnknownSymbol(u8),
        ByteNotReady,
    }

    #[derive(Debug)]
    pub enum RH4b6bEncodeError {
        DataBufferTooShort,
    }

    #[derive(Debug)]
    pub enum RH4b6bDecodeError {
        DataSizeNotEven,
        UnknownSymbol(u8),
    }

    #[cfg(test)]
    mod tests {
        use heapless::Vec;

        use super::*;

        const RAW_DATA: &[u8] = b"Mornin'";
        const CORRECT_ENCODED_DATA: &[u8] = &[
            0x2c, 0x16, 0x34, 0x1a, 0x13, 0x1c, 0x32, 0x1a, 0x25, 0x1a, 0x32, 0x1a, 0x1c, 0x13,
        ];

        #[test]
        fn encode_test() {
            let mut data: [u8; 14] = [0; 14];
            for (i, byte) in RAW_DATA.iter().enumerate() {
                data[i] = *byte;
            }

            encode_in_place(&mut data, RAW_DATA.len()).unwrap();

            assert!(data == CORRECT_ENCODED_DATA);
        }

        #[test]
        fn encode_not_enought_space() {
            let mut data = [0; 6];
            let length = data.len();

            assert!(matches!(encode_in_place(&mut data, length), Err(_)));

            assert!(matches!(encode_in_place(&mut data, length * 2 - 1), Err(_)));
        }

        #[test]
        fn decode_test() {
            let mut data = [0; CORRECT_ENCODED_DATA.len()];
            CORRECT_ENCODED_DATA
                .iter()
                .enumerate()
                .for_each(|(i, byte)| data[i] = *byte);

            let _ = decode_in_place(&mut data).inspect_err(|err| match err {
                RH4b6bDecodeError::UnknownSymbol(symbol) => panic!("Unknown symbol: {}", symbol),
                RH4b6bDecodeError::DataSizeNotEven => panic!("Data size not event"),
            });

            assert!(
                &data[..RAW_DATA.len()] == RAW_DATA,
                "Expected: {:?}\nReceived: {:?}",
                RAW_DATA,
                &data[..RAW_DATA.len()]
            );
        }

        #[test]
        fn running_decoder() {
            let mut decoder = RunningDecoder::new();

            let mut decoded_data = Vec::<_, 7>::new();

            for nibble in CORRECT_ENCODED_DATA {
                match decoder.next_nibble(*nibble) {
                    Ok(byte) => decoded_data.push(byte).unwrap(),
                    Err(err) => match err {
                        RunningDecoderError::ByteNotReady => {}
                        RunningDecoderError::UnknownSymbol(s) => {
                            panic!("Unknown symbol {}", s);
                        }
                    },
                }
            }

            assert!(
                decoded_data == RAW_DATA,
                "Data expected: {:?}\nData received: {:?}",
                RAW_DATA,
                decoded_data
            )
        }
    }
}
