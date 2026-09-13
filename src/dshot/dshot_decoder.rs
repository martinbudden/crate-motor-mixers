use super::{DshotBidirectionalFrame,Telemetry};


#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct DshotDecoder;

/// Dshot Decoder.
/// ```text
/// eRPM Telemetry Frame Structure.
///
/// The encoding of the eRPM data is not as straight forward as the one of the throttle frame.
/// The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
///
///     eeem mmmm mmmm cccc
///
/// where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
/// The resultant value is the mantissa shifted left by the exponent.
/// ```
#[allow(unused)]
impl DshotDecoder {
    // GCR lookup tables
    const GCR_BIT_LENGTHS: [u32; 17] = [0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5];
    const GCR_SET_BITS: [u32; 6] = [0b_00000, 0b_00001, 0b_00011, 0b_00111, 0b_01111, 0b_11111];
    const QUINTET_TO_NIBBLE: [u32; 32] = [
        255, 255, 255, 255, 255, 255, 255, 255, 255, 9, 10, 11, 255, 13, 14, 15, 255, 255, 2, 3, 255, 5, 6, 7, 255, 0,
        8, 1, 255, 4, 12, 255,
    ];

    /// Check if bidirectional checksum is valid.
    #[inline]
    #[must_use]
    pub fn checksum_bidirectional_is_ok(value: u16) -> bool {
        DshotBidirectionalFrame::calculate_checksum(value >> 4) == (value & 0x0F)
    }

    /// Decode `erpm`.
    /// # Errors `DecodeError`
    pub fn decode_erpm(value: u16) -> Result<u16, DecodeError> {
        if value == 0x0FFF {
            return Ok(0);
        }
        let mantissa: u16 = value & 0x01FF;
        let exponent: u16 = (value & 0xFE00) >> 9;
        let result = mantissa << exponent;
        if result == 0 {
            return Err(DecodeError::Erpm);
        }
        Ok(result)
    }

    /*fn decode_telemetry_frame(value: u16) -> Result<TelemetryFrame, DecodeError> {
        let type_val = (value & 0x0F00) >> 8;
        let is_erpm = (type_val & 0x01) != 0 || type_val == 0;
        if is_erpm {
            let result = Self::decode_erpm(value)?;
            return Ok(TelemetryFrame::Erpm(u32::from(result)));
        }
        let type_val = (value & 0x0F00) >> 8;
        Ok((value & 0x00FF, TelemetryType::from_u16(type_val >> 1)))
    }*/

    pub fn decode_telemetry_frame(raw_12: u16) -> Telemetry {
        let exponent = (raw_12 >> 9) & 0x07;
        let bit8 = (raw_12 >> 8) & 1;

        if exponent == 0 || bit8 == 1 {
            const ONE_MINUTE_IN_MICROSECONDS: u32 = 60_000_000;
            if raw_12 == 0 || raw_12 == 0x0FFF {
                return Telemetry::Erpm(0);
            }
            let mantissa = raw_12 & 0x1FF;
            let period_us = u32::from(mantissa) << u32::from(exponent);
            if period_us == 0 {
                return Telemetry::Erpm(0);
            }
            return Telemetry::Erpm(ONE_MINUTE_IN_MICROSECONDS / period_us);
        }

        let data = (raw_12 & 0xFF) as u8;
        match exponent {
            1 => Telemetry::Temperature(data),
            2 => Telemetry::Voltage(u32::from(data) * 250),
            3 => Telemetry::Current(u32::from(data) * 1000),
            4 => Telemetry::Debug1(data),
            5 => Telemetry::Debug2(data),
            6 => Telemetry::Debug3(data),
            7 => Telemetry::StateEvent(data),
            _ => Telemetry::Unknown { type_id: exponent, value: data },
        }
    }

    /// Decode samples returned by Raspberry Pi PIO implementation.
    ///
    /// Returns the value of the Extended Dshot Telemetry (EDT) frame (without the checksum).
    /// # Errors `DecodeError`
    pub fn decode_samples(value: u64) -> Result<Telemetry, DecodeError> {
        // telemetry data must start with a 0, so if the first bit is high, we don't have any data
        if (value & 0x8000_0000_0000_0000) != 0 {
            return Err(DecodeError::NoData);
        }

        let mut consecutive_bit_count: usize = 1; // we always start with the MSB
        let mut current_bit: u32 = 0;
        let mut bit_count: u32 = 0;
        let mut gcr_result: u32 = 0;

        // starting at 2nd bit since we know our data starts with a 0
        // 56 samples @ 0.917us sample rate = 51.33us sampled
        // loop the mask from 2nd MSB to  LSB
        let mut mask: u64 = 0x4000_0000_0000_0000;
        #[allow(clippy::if_not_else)] // TODO: fix this
        while mask != 0 {
            if ((value & mask) != 0) != (current_bit != 0) {
                // if the masked bit doesn't match the current string of bits then end the current string and flip current_bit
                // bitshift gcr_result by N, and
                gcr_result <<= Self::GCR_BIT_LENGTHS[consecutive_bit_count];
                // then set N bits in gcr_result, if current_bit is 1
                if current_bit != 0 {
                    gcr_result |= Self::GCR_SET_BITS[Self::GCR_BIT_LENGTHS[consecutive_bit_count] as usize];
                }
                bit_count += Self::GCR_BIT_LENGTHS[consecutive_bit_count];
                // invert current_bit, and reset consecutive_bit_count
                current_bit = !current_bit;
                consecutive_bit_count = 1; // first bit found in the string is the one we just processed
            } else {
                // otherwise increment consecutive_bit_count
                consecutive_bit_count += 1;
                if consecutive_bit_count > 16 {
                    // invalid run length at the current sample rate (outside of GCR_BIT_LENGTHS table)
                    return Err(DecodeError::InvalidRunLength);
                }
            }
            mask >>= 1;
        }

        // outside the loop, we still need to account for the final bits if the string ends with 1s
        // bitshift gcr_result by N, and
        gcr_result <<= Self::GCR_BIT_LENGTHS[consecutive_bit_count];
        // then set set N bits in gcr_result, if current_bit is 1
        if current_bit != 0 {
            gcr_result |= Self::GCR_SET_BITS[Self::GCR_BIT_LENGTHS[consecutive_bit_count] as usize];
        }
        // count bit_count (for debugging)
        bit_count += Self::GCR_BIT_LENGTHS[consecutive_bit_count];

        // GCR data should be 21 bits
        if bit_count < 21 {
            return Err(DecodeError::GcrData);
        }

        // chop the GCR data down to just the 21 most significant bits
        gcr_result >>= bit_count - 21;

        // convert 21-bit edge transition GCR to 20-bit binary GCR
        let gcr20: u32 = Self::gcr21_to_gcr20(gcr_result);

        let result: u16 = Self::gcr20_to_erpm(gcr20);

        if !Self::checksum_bidirectional_is_ok(result) {
            return Err(DecodeError::Crc);
        }

        Ok(Self::decode_telemetry_frame(result >> 4))
    }

    #[inline]
    pub fn decode_samples_slice(_samples: &[u32], _telemetry_type: &mut u16) -> u32 {
        0
    }
    pub fn gcr21_decode(gcr21: u32) -> Result<u16, DecodeError> {
        let gcr = gcr21 & 0x000F_FFFF;
        let gcr20 = gcr ^ (gcr >> 1);

        let mut ret: u32 = Self::QUINTET_TO_NIBBLE[(gcr20 & 0x1F) as usize];
        if ret == 0xFF {
            return Err(DecodeError::GcrData);
        }

        let nibble = Self::QUINTET_TO_NIBBLE[((gcr20 >> 5) & 0x1F) as usize];
        if nibble == 0xFF {
            return Err(DecodeError::GcrData);
        }
        ret |= nibble << 4;

        let nibble = Self::QUINTET_TO_NIBBLE[((gcr20 >> 10) & 0x1F) as usize];
        if nibble == 0xFF {
            return Err(DecodeError::GcrData);
        }
        ret |= nibble << 8;

        let nibble = Self::QUINTET_TO_NIBBLE[((gcr20 >> 15) & 0x1F) as usize];
        if nibble == 0xFF {
            return Err(DecodeError::GcrData);
        }
        ret |= nibble << 12;

        #[allow(clippy::cast_possible_truncation)]
        Ok(ret as u16)
    }

    #[must_use]
    pub fn gcr20_to_erpm(value: u32) -> u16 {
        let mut ret: u32 = Self::QUINTET_TO_NIBBLE[(value & 0x1F) as usize];
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 5) & 0x1F) as usize] << 4;
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 10) & 0x1F) as usize] << 8;
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 15) & 0x1F) as usize] << 12;
        #[allow(clippy::cast_possible_truncation)]
        {
            ret as u16
        }
    }
    #[inline]
    #[must_use]
    pub fn gcr21_to_gcr20(value: u32) -> u32 {
        value ^ (value >> 1)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotDecoder>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_quintets() {
        assert_eq!(0, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[0] as usize]);
        assert_eq!(1, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[1] as usize]);
        assert_eq!(2, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[2] as usize]);
        assert_eq!(3, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[3] as usize]);
        assert_eq!(4, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[4] as usize]);
        assert_eq!(5, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[5] as usize]);
        assert_eq!(6, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[6] as usize]);
        assert_eq!(7, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[7] as usize]);
        assert_eq!(8, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[8] as usize]);
        assert_eq!(9, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[9] as usize]);
        assert_eq!(10, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[10] as usize]);
        assert_eq!(11, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[11] as usize]);
        assert_eq!(12, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[12] as usize]);
        assert_eq!(13, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[13] as usize]);
        assert_eq!(14, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[14] as usize]);
        assert_eq!(15, DshotDecoder::QUINTET_TO_NIBBLE[DshotBidirectionalFrame::NIBBLE_TO_QUINTET[15] as usize]);
    }

    /*#[test]
    fn dshot_codec_mappings() {
        assert_eq!(0b_1101_0100_1011_1101_0110, DshotFrame::erpm_to_gcr20(0b_1000_0010_1100_0110));
        assert_eq!(0b_1000_0010_1100_0110, DshotDecoder::gcr20_to_erpm(0b_1101_0100_1011_1101_0110));

        assert_eq!(0b0_1010_1010_1010_1010_1010, DshotDecoder::gcr21_to_gcr20(0b0_1100_1100_1100_1100_1100));
        // TODO: check dshot_codec_mappings
        //assert_eq!(0b_011001100110011001100, DshotCodec::gr20_to_gcr21(0b_10101010101010101010));
    }*/

    #[test]
    fn gcr_decode_rejects_invalid_input() {
        // All zeros and all ones should fail
        assert_eq!(Err(DecodeError::GcrData), DshotDecoder::gcr21_decode(0));
        assert_eq!(Err(DecodeError::GcrData), DshotDecoder::gcr21_decode(0x1FFFF));
    }

    /*#[test]
    fn dshot_codec_checksum() {
        assert!(DshotDecoder::checksum_is_ok(DshotFrame::encode_raw_bidirectional(0b_0100_0001_0110)));
    }
    #[test]
    fn gcr_decode_valid_checksum() {
        // Test values with valid checksum (XOR of nibbles = 0xF)
        // 0xF000: nibbles 0,0,0,F -> XOR = F ✓
        let gcr20 = DshotFrame::erpm_to_gcr20(0xF000);
        assert_eq!(0x7E739, gcr20);
        let encoded = DshotFrame::gcr20_to_gcr21(gcr20);
        assert_eq!(Ok(0xF000), DshotDecoder::gcr21_decode(encoded));

        let encoded = DshotFrame::gcr_encode(0xF000);
        assert_eq!(Ok(0xF000), DshotDecoder::gcr21_decode(encoded));

        // 0x1E00: nibbles 0,0,E,1 -> XOR = F ✓
        let encoded = DshotFrame::gcr_encode(0x1E00);
        assert_eq!(Ok(0x1E00), DshotDecoder::gcr21_decode(encoded));

        // 0x2D00: nibbles 0,0,D,2 -> XOR = F ✓
        let encoded = DshotFrame::gcr_encode(0x2D00);
        assert_eq!(Ok(0x2D00), DshotDecoder::gcr21_decode(encoded));

        // 0x1234: nibbles 4,3,2,1 -> XOR = 4^3^2^1 = 4 (not F, invalid)
        // Need a value where nibbles XOR to F
        // 0x8421: nibbles 1,2,4,8 -> XOR = 1^2^4^8 = F ✓
        let encoded = DshotFrame::gcr_encode(0x8421);
        assert_eq!(Ok(0x8421), DshotDecoder::gcr21_decode(encoded));
    }*/
}
