/// Dshot Encoder/Decoder.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DshotCodec;

impl DshotCodec {
    /// ```text
    /// DShot Frame Structure
    /// The DShot Frame defines which information is at which position in the data stream:
    ///
    ///     11 bit throttle(S): 2048 possible values.
    ///         0 is reserved for disarmed.
    ///         1 to 47 are reserved for special commands.
    ///         48 to 2047 (2000 steps) are for the actual throttle value
    ///     1 bit telemetry request(T) - if this is set, telemetry data is sent back via a separate channel
    ///     4 bit checksum(C) aka CRC (Cyclic Redundancy Check) to validate the frame
    ///
    /// This results in a 16 bit (2 byte) frame with the following structure:
    ///
    ///    SSSSSSSSSSSTCCCC
    ///
    /// eRPM Telemetry Frame Structure
    ///
    /// The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
    /// The encoding of the eRPM data is not as straight forward as the one of the throttle frame:
    ///
    ///     eeemmmmmmmmmcccc
    ///
    /// where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
    /// The resultant value is the mantissa shifted left by the exponent.
    /// ```text
    pub const TELEMETRY_TYPE_ERPM: u16 = 0;
    pub const TELEMETRY_TYPE_TEMPERATURE: u16 = 1;
    pub const TELEMETRY_TYPE_VOLTAGE: u16 = 2;
    pub const TELEMETRY_TYPE_CURRENT: u16 = 3;
    pub const TELEMETRY_TYPE_DEBUG1: u16 = 4;
    pub const TELEMETRY_TYPE_DEBUG2: u16 = 5;
    pub const TELEMETRY_TYPE_STRESS_LEVEL: u16 = 6;
    pub const TELEMETRY_TYPE_STATE_EVENTS: u16 = 7;
    pub const TELEMETRY_TYPE_COUNT: u16 = 8;
    pub const TELEMETRY_INVALID: u16 = 0xFFFF;

    /// Convert PWM (1000-2000) to Dshot value.
    #[inline]
    #[must_use]
    pub fn pwm_to_dshot(value: u16) -> u16 {
        ((value - 1000) * 2) + 47
    }

    /// Convert PWM to Dshot with clipping.
    #[inline]
    #[must_use]
    pub fn pwm_to_dshot_clamped(value: u16) -> u16 {
        if value > 2000 {
            Self::pwm_to_dshot(2000)
        } else if value > 1000 {
            Self::pwm_to_dshot(value)
        } else {
            0
        }
    }

    /// Unidirectional (non-inverted) checksum.
    #[inline]
    #[must_use]
    pub fn checksum_unidirectional(value: u16) -> u16 {
        (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
    }

    /// Check if unidirectional checksum is valid.
    #[inline]
    #[must_use]
    pub fn checksum_unidirectional_is_ok(value: u16) -> bool {
        Self::checksum_unidirectional(value >> 4) == (value & 0x0F)
    }

    /// Bidirectional (inverted) checksum.
    #[inline]
    #[must_use]
    pub fn checksum_bidirectional(value: u16) -> u16 {
        (!(value ^ (value >> 4) ^ (value >> 8))) & 0x0F
    }

    /// Check if bidirectional checksum is valid.
    #[inline]
    #[must_use]
    pub fn checksum_bidirectional_is_ok(value: u16) -> bool {
        Self::checksum_bidirectional(value >> 4) == (value & 0x0F)
    }

    /// Create unidirectional Dshot frame.
    #[inline]
    #[must_use]
    pub fn frame_unidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_unidirectional(value)
    }

    /// Create bidirectional Dshot frame.
    #[inline]
    #[must_use]
    pub fn frame_bidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_bidirectional(value)
    }

    // GCR lookup tables
    pub const GCR_BIT_LENGTHS: [u32; 17] = [0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5];

    pub const GCR_SET_BITS: [u32; 6] = [0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111];

    pub const QUINTET_TO_NIBBLE: [u32; 32] =
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 10, 11, 0, 13, 14, 15, 0, 0, 2, 3, 0, 5, 6, 7, 0, 0, 8, 1, 0, 4, 12, 0];

    pub const NIBBLE_TO_QUINTET: [u8; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    /// Decode `erpm`.
    /// # Errors `TELEMETRY_INVALID`
    pub fn decode_erpm(value: u16) -> Result<u16, u16> {
        let mut value = value;
        // eRPM range
        if value == 0x0FFF {
            return Ok(0);
        }
        let m: u16 = value & 0x01FF;
        let e: u16 = (value & 0xFE00) >> 9;
        value = m << e;
        if value == 0 {
            return Err(Self::TELEMETRY_INVALID);
        }
        Ok(value)
    }

    fn decode_telemetry_frame(value: u16) -> Result<(u16, u16), u16> {
        let type_val = (value & 0x0F00) >> 8;
        let is_erpm = (type_val & 0x01) != 0 || type_val == 0;
        if is_erpm {
            let m = value & 0x01FF;
            let e = (value & 0xFE00) >> 9;
            let result = m << e;
            if result == 0 {
                return Err(Self::TELEMETRY_INVALID);
            }
            return Ok((result, Self::TELEMETRY_TYPE_ERPM));
        }
        let type_val = (value & 0x0F00) >> 8;
        Ok((value & 0x00FF, type_val >> 1))
    }

    /// Decode samples returned by Raspberry Pi PIO implementation.
    ///
    /// Returns the value of the Extended Dshot Telemetry (EDT) frame (without the  checksum).
    /// # Errors `TELEMETRY_INVALID`
    pub fn decode_samples(value: u64) -> Result<(u32, u16), u16> {
        // telemetry data must start with a 0, so if the first bit is high, we don't have any data
        if (value & 0x8000_0000_0000_0000) != 0 {
            return Err(Self::TELEMETRY_INVALID);
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
                    return Err(Self::TELEMETRY_INVALID);
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
            return Err(Self::TELEMETRY_INVALID);
        }

        // chop the GCR data down to just the 21 most significant bits
        gcr_result >>= bit_count - 21;

        // convert 21-bit edge transition GCR to 20-bit binary GCR
        let gcr20: u32 = Self::gcr21_to_gcr20(gcr_result);

        let result: u16 = Self::gcr20_to_erpm(gcr20);

        if !Self::checksum_bidirectional_is_ok(result) {
            return Err(Self::TELEMETRY_INVALID);
        }
        match Self::decode_telemetry_frame(result >> 4) {
            Ok((result, telemetry_type)) => Ok((u32::from(result), telemetry_type)),
            Err(_) => Err(Self::TELEMETRY_INVALID),
        }
    }

    #[inline]
    pub fn decode_samples_slice(_samples: &[u32], _telemetry_type: &mut u16) -> u32 {
        0
    }

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    #[inline]
    #[must_use]
    pub fn erpm_to_gcr20(value: u16) -> u32 {
        let mut ret = u32::from(Self::NIBBLE_TO_QUINTET[(value & 0x1F) as usize]);
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 4) & 0x1F) as usize]) << 5;
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 8) & 0x1F) as usize]) << 10;
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 12) & 0x1F) as usize]) << 15;
        ret
    }

    /// Map the GCR to a 21 bit value, this new value starts with a 0 and the rest of the bits are set by the following two rules:
    ///    1. If the current input bit in GCR data is a 1 then the output bit is the inverse of the previous output bit
    ///    2. If the current input bit in GCR data is a 0 then the output bit is the same as the previous output
    #[must_use]
    pub fn gr20_to_gcr21(value: u32) -> u32 {
        let mut ret = 0;
        let mut previous_output_bit = 0;

        let mut mask = 1 << 20;
        while mask != 0 {
            ret <<= 1;
            let input_bit = value & mask;
            let output_bit = if input_bit != 0 { !previous_output_bit } else { previous_output_bit };
            previous_output_bit = output_bit;
            ret |= output_bit;
            mask >>= 1;
        }
        ret
    }

    #[inline]
    #[must_use]
    pub fn gcr21_to_gcr20(value: u32) -> u32 {
        value ^ (value >> 1)
    }

    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    pub fn gcr20_to_erpm(value: u32) -> u16 {
        let mut ret: u32 = Self::QUINTET_TO_NIBBLE[(value & 0x1F) as usize];
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 5) & 0x1F) as usize] << 4;
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 10) & 0x1F) as usize] << 8;
        ret |= Self::QUINTET_TO_NIBBLE[((value >> 15) & 0x1F) as usize] << 12;
        ret as u16
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotCodec>();
    }
    #[test]
    fn dshot_quintets() {
        assert_eq!(0, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[0] as usize]);
        assert_eq!(1, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[1] as usize]);
        assert_eq!(2, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[2] as usize]);
        assert_eq!(3, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[3] as usize]);
        assert_eq!(4, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[4] as usize]);
        assert_eq!(5, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[5] as usize]);
        assert_eq!(6, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[6] as usize]);
        assert_eq!(7, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[7] as usize]);
        assert_eq!(8, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[8] as usize]);
        assert_eq!(9, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[9] as usize]);
        assert_eq!(10, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[10] as usize]);
        assert_eq!(11, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[11] as usize]);
        assert_eq!(12, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[12] as usize]);
        assert_eq!(13, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[13] as usize]);
        assert_eq!(14, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[14] as usize]);
        assert_eq!(15, DshotCodec::QUINTET_TO_NIBBLE[DshotCodec::NIBBLE_TO_QUINTET[15] as usize]);
    }

    #[test]
    fn dshot_codec_checksum() {
        assert_eq!(0b0000_0000_0110, DshotCodec::checksum_unidirectional(0b1000_0010_1100));
        assert_eq!(0b0000_0000_1001, DshotCodec::checksum_bidirectional(0b1000_0010_1100));

        assert_eq!(0b1000_0010_1100_0110, DshotCodec::frame_unidirectional(0b0100_0001_0110));
        assert_eq!(0b1000_0010_1100_1001, DshotCodec::frame_bidirectional(0b0100_0001_0110));

        assert!(DshotCodec::checksum_unidirectional_is_ok(DshotCodec::frame_unidirectional(0b0100_0001_0110)));
        assert!(DshotCodec::checksum_bidirectional_is_ok(DshotCodec::frame_bidirectional(0b0100_0001_0110)));
    }
    #[test]
    fn dshot_codec_mappings() {
        assert_eq!(0b1101_0100_1011_1101_0110, DshotCodec::erpm_to_gcr20(0b1000_0010_1100_0110));
        assert_eq!(0b1000_0010_1100_0110, DshotCodec::gcr20_to_erpm(0b1101_0100_1011_1101_0110));

        assert_eq!(0b0_1010_1010_1010_1010_1010, DshotCodec::gcr21_to_gcr20(0b0_1100_1100_1100_1100_1100));
        // TODO: check dshot_codec_mappings
        //assert_eq!(0b011001100110011001100, DshotCodec::gr20_to_gcr21(0b10101010101010101010));
    }
    #[test]
    fn dshot_codec() {
        assert_eq!(47, DshotCodec::pwm_to_dshot(1000));
        assert_eq!(2047, DshotCodec::pwm_to_dshot(2000));

        assert_eq!(0, DshotCodec::pwm_to_dshot_clamped(0));
        assert_eq!(0, DshotCodec::pwm_to_dshot_clamped(10));
        assert_eq!(0, DshotCodec::pwm_to_dshot_clamped(999));

        assert_eq!(0, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        //assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        assert_eq!(49, DshotCodec::pwm_to_dshot_clamped(1001));
        assert_eq!(51, DshotCodec::pwm_to_dshot_clamped(1002));
        assert_eq!(53, DshotCodec::pwm_to_dshot_clamped(1003));
        assert_eq!(1047, DshotCodec::pwm_to_dshot_clamped(1500));
        assert_eq!(2045, DshotCodec::pwm_to_dshot_clamped(1999));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2000));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2001));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2002));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(4000));

        assert_eq!(1542, DshotCodec::frame_unidirectional(48)); //0x606
        assert_eq!(1572, DshotCodec::frame_unidirectional(49)); // 0x624
        assert_eq!(33547, DshotCodec::frame_unidirectional(1048)); // 0x830B
        assert_eq!(65484, DshotCodec::frame_unidirectional(2046)); // 0xFFCC
        assert_eq!(65518, DshotCodec::frame_unidirectional(2047)); // 0xFFEB, 0xFFFF=65535

        // testing out of range values
        assert_eq!(0, DshotCodec::frame_unidirectional(0));
        assert_eq!(34, DshotCodec::frame_unidirectional(1));
        assert_eq!(68, DshotCodec::frame_unidirectional(2));
        assert_eq!(325, DshotCodec::frame_unidirectional(10));

        //assert_eq!(1, DshotCodec::frame_unidirectional(2048));
        //assert_eq!(35, DshotCodec::frame_unidirectional(2049));
        //assert_eq!(69, DshotCodec::frame_unidirectional(2050));
    }
}
