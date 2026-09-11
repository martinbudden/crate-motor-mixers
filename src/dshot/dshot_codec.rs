use super::Command;

/// Dshot Encoder/Decoder.
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
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DshotCodec;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    NoData,
    InvalidRunLength,
    GcrData,
    Crc,
    Erpm,
    _TelemetryType,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TelemetryFrame {
    Erpm(u32),
    /// 1°C per unit.
    Temperature(u8),
    /// 250mV per unit.
    Voltage(u32),
    /// 1A (1000mA) per unit.
    Current(u32),
    Debug1(u8),
    Debug2(u8),
    Debug3(u8),
    StateEvent(u8),
    Unknown {
        type_id: u16,
        value: u8,
    },
}

#[allow(unused)]
impl DshotCodec {
    const THROTTLE_OFFSET: u16 = 48;
    const THROTTLE_MIN: u16 = 48;
    const THROTTLE_MAX: u16 = 2047;

    // GCR lookup tables
    const GCR_BIT_LENGTHS: [u32; 17] = [0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5];
    const GCR_SET_BITS: [u32; 6] = [0b_00000, 0b_00001, 0b_00011, 0b_00111, 0b_01111, 0b_11111];
    const QUINTET_TO_NIBBLE: [u32; 32] = [
        255, 255, 255, 255, 255, 255, 255, 255, 255, 9, 10, 11, 255, 13, 14, 15, 255, 255, 2, 3, 255, 5, 6, 7, 255, 0,
        8, 1, 255, 4, 12, 255,
    ];
    const NIBBLE_TO_QUINTET: [u8; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    /// Convert PWM value (1000-2000) to Dshot value (48-2047).
    #[inline]
    #[must_use]
    pub fn pwm_to_dshot(value: u16) -> u16 {
        ((value - 1000) * 2) + Self::THROTTLE_OFFSET
    }

    /// Convert PWM to Dshot with clipping.
    #[inline]
    #[must_use]
    pub fn pwm_to_dshot_clamped(value: u16) -> u16 {
        if value >= 2000 {
            Self::THROTTLE_MAX
        } else if value >= 1000 {
            Self::pwm_to_dshot(value)
        } else {
            Self::THROTTLE_MIN
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

    #[inline]
    #[must_use]
    pub fn encode_raw_value_unidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_unidirectional(value)
    }

    #[inline]
    #[must_use]
    pub fn encode_raw_value_bidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_bidirectional(value)
    }

    #[inline]
    #[must_use]
    pub fn encode_command_unidirectional(command: Command) -> u16 {
        Self::encode_raw_value_unidirectional(command as u16)
    }

    #[inline]
    #[must_use]
    pub fn encode_command_bidirectional(command: Command) -> u16 {
        Self::encode_raw_value_bidirectional(command as u16)
    }

    /// Decode `erpm`.
    /// # Errors `DecodeError`
    pub fn decode_erpm(value: u16) -> Result<u16, DecodeError> {
        // eRPM range
        if value == 0x0FFF {
            return Ok(0);
        }
        let m: u16 = value & 0x01FF;
        let e: u16 = (value & 0xFE00) >> 9;
        let result = m << e;
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

    pub fn decode_telemetry_frame(raw_12: u16) -> TelemetryFrame {
        let exponent = (raw_12 >> 9) & 0x07;
        let bit8 = (raw_12 >> 8) & 1;

        if exponent == 0 || bit8 == 1 {
            if raw_12 == 0 || raw_12 == 0x0FFF {
                return TelemetryFrame::Erpm(0);
            }
            let mantissa = raw_12 & 0x1FF;
            let period_us = u32::from(mantissa) << u32::from(exponent);
            if period_us == 0 {
                return TelemetryFrame::Erpm(0);
            }
            return TelemetryFrame::Erpm(60_000_000 / period_us);
        }

        let data = (raw_12 & 0xFF) as u8;
        match exponent {
            1 => TelemetryFrame::Temperature(data),
            2 => TelemetryFrame::Voltage(u32::from(data) * 250),
            3 => TelemetryFrame::Current(u32::from(data) * 1000),
            4 => TelemetryFrame::Debug1(data),
            5 => TelemetryFrame::Debug2(data),
            6 => TelemetryFrame::Debug3(data),
            7 => TelemetryFrame::StateEvent(data),
            _ => TelemetryFrame::Unknown { type_id: exponent, value: data },
        }
    }

    /// Decode samples returned by Raspberry Pi PIO implementation.
    ///
    /// Returns the value of the Extended Dshot Telemetry (EDT) frame (without the checksum).
    /// # Errors `DecodeError`
    pub fn decode_samples(value: u64) -> Result<TelemetryFrame, DecodeError> {
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

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    #[inline]
    #[must_use]
    pub fn erpm_to_gcr20(value: u16) -> u32 {
        let mut ret = u32::from(Self::NIBBLE_TO_QUINTET[(value & 0x0F) as usize]);
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 4) & 0x0F) as usize]) << 5;
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 8) & 0x0F) as usize]) << 10;
        ret |= u32::from(Self::NIBBLE_TO_QUINTET[((value >> 12) & 0x0F) as usize]) << 15;
        ret
    }

    /// Map the GCR to a 21 bit value, this new value starts with a 0 and the rest of the bits are set by the following two rules:
    ///    1. If the current input bit in GCR data is a 1 then the output bit is the inverse of the previous output bit
    ///    2. If the current input bit in GCR data is a 0 then the output bit is the same as the previous output
    #[must_use]
    pub fn gcr20_to_gcr21(input: u32) -> u32 {
        let mut ret = 0;
        let mut prev_gcr_bit = 0;
        let mut mask = 1 << 19;

        while mask != 0 {
            ret <<= 1;
            let input_bit = u32::from((input & mask) != 0);
            let gcr_bit = input_bit ^ prev_gcr_bit;
            prev_gcr_bit = gcr_bit;
            ret |= gcr_bit;
            mask >>= 1;
        }
        ret
    }

    pub fn gcr_encode(value: u16) -> u32 {
        let gcr20 = Self::erpm_to_gcr20(value);
        Self::gcr20_to_gcr21(gcr20)
    }

    #[inline]
    #[must_use]
    pub fn gcr21_to_gcr20(value: u32) -> u32 {
        value ^ (value >> 1)
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
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotCodec>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
        assert_eq!(0b_0000_0000_0110, DshotCodec::checksum_unidirectional(0b_1000_0010_1100));
        assert_eq!(0b_0000_0000_1001, DshotCodec::checksum_bidirectional(0b_1000_0010_1100));

        assert_eq!(0b_1000_0010_1100_0110, DshotCodec::encode_raw_value_unidirectional(0b_0100_0001_0110));
        assert_eq!(0b_1000_0010_1100_1001, DshotCodec::encode_raw_value_bidirectional(0b_0100_0001_0110));

        assert!(DshotCodec::checksum_unidirectional_is_ok(DshotCodec::encode_raw_value_unidirectional(
            0b_0100_0001_0110
        )));
        assert!(DshotCodec::checksum_bidirectional_is_ok(DshotCodec::encode_raw_value_bidirectional(
            0b_0100_0001_0110
        )));
    }
    #[test]
    fn gcr_decode_rejects_invalid_input() {
        // All zeros and all ones should fail
        assert_eq!(Err(DecodeError::GcrData), DshotCodec::gcr21_decode(0));
        assert_eq!(Err(DecodeError::GcrData), DshotCodec::gcr21_decode(0x1FFFF));
    }

    #[test]
    fn gcr_decode_valid_checksum() {
        // Test values with valid checksum (XOR of nibbles = 0xF)
        // 0xF000: nibbles 0,0,0,F -> XOR = F ✓
        let gcr20 = DshotCodec::erpm_to_gcr20(0xF000);
        assert_eq!(0x7E739, gcr20);
        let encoded = DshotCodec::gcr20_to_gcr21(gcr20);
        assert_eq!(Ok(0xF000), DshotCodec::gcr21_decode(encoded));

        let encoded = DshotCodec::gcr_encode(0xF000);
        assert_eq!(Ok(0xF000), DshotCodec::gcr21_decode(encoded));

        // 0x1E00: nibbles 0,0,E,1 -> XOR = F ✓
        let encoded = DshotCodec::gcr_encode(0x1E00);
        assert_eq!(Ok(0x1E00), DshotCodec::gcr21_decode(encoded));

        // 0x2D00: nibbles 0,0,D,2 -> XOR = F ✓
        let encoded = DshotCodec::gcr_encode(0x2D00);
        assert_eq!(Ok(0x2D00), DshotCodec::gcr21_decode(encoded));

        // 0x1234: nibbles 4,3,2,1 -> XOR = 4^3^2^1 = 4 (not F, invalid)
        // Need a value where nibbles XOR to F
        // 0x8421: nibbles 1,2,4,8 -> XOR = 1^2^4^8 = F ✓
        let encoded = DshotCodec::gcr_encode(0x8421);
        assert_eq!(Ok(0x8421), DshotCodec::gcr21_decode(encoded));
    }

    #[test]
    fn dshot_codec_mappings() {
        assert_eq!(0b_1101_0100_1011_1101_0110, DshotCodec::erpm_to_gcr20(0b_1000_0010_1100_0110));
        assert_eq!(0b_1000_0010_1100_0110, DshotCodec::gcr20_to_erpm(0b_1101_0100_1011_1101_0110));

        assert_eq!(0b0_1010_1010_1010_1010_1010, DshotCodec::gcr21_to_gcr20(0b0_1100_1100_1100_1100_1100));
        // TODO: check dshot_codec_mappings
        //assert_eq!(0b_011001100110011001100, DshotCodec::gr20_to_gcr21(0b_10101010101010101010));
    }
    #[test]
    fn dshot_codec() {
        assert_eq!(48, DshotCodec::pwm_to_dshot(1000));
        assert_eq!(2048, DshotCodec::pwm_to_dshot(2000));

        assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(0));
        assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(10));
        assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(999));

        assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        //assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        assert_eq!(50, DshotCodec::pwm_to_dshot_clamped(1001));
        assert_eq!(52, DshotCodec::pwm_to_dshot_clamped(1002));
        assert_eq!(54, DshotCodec::pwm_to_dshot_clamped(1003));
        assert_eq!(1048, DshotCodec::pwm_to_dshot_clamped(1500));
        assert_eq!(2046, DshotCodec::pwm_to_dshot_clamped(1999));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2000));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2001));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(2002));
        assert_eq!(2047, DshotCodec::pwm_to_dshot_clamped(4000));

        assert_eq!(1542, DshotCodec::encode_raw_value_unidirectional(48)); //0x606
        assert_eq!(1572, DshotCodec::encode_raw_value_unidirectional(49)); // 0x624
        assert_eq!(33547, DshotCodec::encode_raw_value_unidirectional(1048)); // 0x830B
        assert_eq!(65484, DshotCodec::encode_raw_value_unidirectional(2046)); // 0xFFCC
        assert_eq!(65518, DshotCodec::encode_raw_value_unidirectional(2047)); // 0xFFEB, 0xFFFF=65535

        // testing out of range values
        assert_eq!(0, DshotCodec::encode_raw_value_unidirectional(0));
        assert_eq!(34, DshotCodec::encode_raw_value_unidirectional(1));
        assert_eq!(68, DshotCodec::encode_raw_value_unidirectional(2));
        assert_eq!(325, DshotCodec::encode_raw_value_unidirectional(10));

        //assert_eq!(1, DshotCodec::frame_unidirectional(2048));
        //assert_eq!(35, DshotCodec::frame_unidirectional(2049));
        //assert_eq!(69, DshotCodec::frame_unidirectional(2050));
    }
    #[test]
    fn commands() {
        assert_eq!(0, DshotCodec::encode_command_unidirectional(Command::MotorStop));
        //            SSSS_SSSS_SSST_CCCC
        assert_eq!(0b_0000_0000_0010_0010, DshotCodec::encode_command_unidirectional(Command::Beep1));
        assert_eq!(0b_0000_0101_1100_1001, DshotCodec::encode_command_unidirectional(Command::SignalLineERPMTelemetry));
        assert_eq!(
            0b_0000_0101_1110_1011,
            DshotCodec::encode_command_unidirectional(Command::SignalLineERPMPeriodTelemetry)
        );
        // bidirectional form is the same with the checksum bits inverted
        assert_eq!(0b_0000_0000_0010_1101, DshotCodec::encode_command_bidirectional(Command::Beep1));
        assert_eq!(0b_0000_0101_1100_0110, DshotCodec::encode_command_bidirectional(Command::SignalLineERPMTelemetry));
        assert_eq!(
            0b_0000_0101_1110_0100,
            DshotCodec::encode_command_bidirectional(Command::SignalLineERPMPeriodTelemetry)
        );
    }
}
