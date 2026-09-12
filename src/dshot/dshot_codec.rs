use super::Command;

/// Dshot Encoder.
/// ```text
/// Dshot Frame Structure
/// The Dshot Frame defines which information is at which position in the data stream:
///
///     S: 11 bit throttle/command: 2048 possible values.
///         0 is reserved for disarmed.
///         1 to 47 are reserved for special commands.
///         48 to 2047 (2000 steps) are for the actual throttle value
///     T: 1 bit telemetry request - if this is set, telemetry data is sent back
///     C: 4 bit checksum to validate the frame
///
/// This results in a 16 bit (2 byte) frame with the following structure:
///
///    SSSS SSSS SSST CCCC
/// ```
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DshotEncoder;

#[allow(unused)]
impl DshotEncoder {
    const THROTTLE_OFFSET: u16 = 48;
    const THROTTLE_MIN: u16 = 48;
    const THROTTLE_MAX: u16 = 2047;

    pub(crate) const NIBBLE_TO_QUINTET: [u8; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    /// Convert PWM value (1000-2000) to Dshot value (48-2047).
    #[inline]
    #[must_use]
    pub fn pwm_to_frame(pwm: u16) -> u16 {
        ((pwm - 1000) * 2) + Self::THROTTLE_OFFSET
    }

    /// Convert PWM value (1000-2000) to Dshot value (48-2047),
    /// clamping PWM value to (1000-2000).
    #[inline]
    #[must_use]
    pub fn pwm_to_frame_clamped(pwm: u16) -> u16 {
        if pwm >= 2000 {
            Self::THROTTLE_MAX
        } else if pwm >= 1000 {
            Self::pwm_to_frame(pwm)
        } else {
            Self::THROTTLE_MIN
        }
    }

    /// Convert throttle value [0.0,1.0] to Dshot frame value [48,2047],
    /// clamping PWM value to (1000-2000).
    #[inline]
    #[must_use]
    pub fn throttle_to_frame(throttle: f32) -> u16 {
        #[allow(clippy::cast_possible_truncation,clippy::cast_sign_loss)]
        let pwm = ((throttle.abs() + 1.0) * 1000.0) as u16;
        if pwm >= 2000 {
            Self::THROTTLE_MAX
        } else if pwm >= 1000 {
            Self::pwm_to_frame(pwm)
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

    /// Bidirectional (inverted) checksum.
    #[inline]
    #[must_use]
    pub fn checksum_bidirectional(value: u16) -> u16 {
        (!(value ^ (value >> 4) ^ (value >> 8))) & 0x0F
    }

    #[inline]
    #[must_use]
    pub fn encode_raw_unidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_unidirectional(value)
    }

    #[inline]
    #[must_use]
    pub fn encode_raw_bidirectional(value: u16) -> u16 {
        let value = value << 1;
        (value << 4) | Self::checksum_bidirectional(value)
    }

    #[inline]
    #[must_use]
    pub fn encode_command_unidirectional(command: Command) -> u16 {
        Self::encode_raw_unidirectional(command as u16)
    }

    #[inline]
    #[must_use]
    pub fn encode_command_bidirectional(command: Command) -> u16 {
        Self::encode_raw_bidirectional(command as u16)
    }

    #[inline]
    #[must_use]
    pub fn encode_pwm_unidirectional(pwm: u16) -> u16 {
        Self::encode_raw_unidirectional(Self::pwm_to_frame_clamped(pwm))
    }

    #[inline]
    #[must_use]
    pub fn encode_pwm_bidirectional(pwm: u16) -> u16 {
        Self::encode_raw_bidirectional(Self::pwm_to_frame_clamped(pwm))
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
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotEncoder>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_codec_checksum() {
        assert_eq!(0b_0000_0000_0110, DshotEncoder::checksum_unidirectional(0b_1000_0010_1100));
        assert_eq!(0b_0000_0000_1001, DshotEncoder::checksum_bidirectional(0b_1000_0010_1100));

        assert_eq!(0b_1000_0010_1100_0110, DshotEncoder::encode_raw_unidirectional(0b_0100_0001_0110));
        assert_eq!(0b_1000_0010_1100_1001, DshotEncoder::encode_raw_bidirectional(0b_0100_0001_0110));
    }
    #[test]
    fn dshot_codec() {
        assert_eq!(48, DshotEncoder::pwm_to_frame(1000));
        assert_eq!(2048, DshotEncoder::pwm_to_frame(2000));

        assert_eq!(48, DshotEncoder::pwm_to_frame_clamped(0));
        assert_eq!(48, DshotEncoder::pwm_to_frame_clamped(10));
        assert_eq!(48, DshotEncoder::pwm_to_frame_clamped(999));

        assert_eq!(48, DshotEncoder::pwm_to_frame_clamped(1000)); // should this be 0 or 48 ?
        //assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        assert_eq!(50, DshotEncoder::pwm_to_frame_clamped(1001));
        assert_eq!(52, DshotEncoder::pwm_to_frame_clamped(1002));
        assert_eq!(54, DshotEncoder::pwm_to_frame_clamped(1003));
        assert_eq!(1048, DshotEncoder::pwm_to_frame_clamped(1500));
        assert_eq!(2046, DshotEncoder::pwm_to_frame_clamped(1999));
        assert_eq!(2047, DshotEncoder::pwm_to_frame_clamped(2000));
        assert_eq!(2047, DshotEncoder::pwm_to_frame_clamped(2001));
        assert_eq!(2047, DshotEncoder::pwm_to_frame_clamped(2002));
        assert_eq!(2047, DshotEncoder::pwm_to_frame_clamped(4000));

        assert_eq!(1542, DshotEncoder::encode_raw_unidirectional(48)); //0x606
        assert_eq!(1572, DshotEncoder::encode_raw_unidirectional(49)); // 0x624
        assert_eq!(33547, DshotEncoder::encode_raw_unidirectional(1048)); // 0x830B
        assert_eq!(65484, DshotEncoder::encode_raw_unidirectional(2046)); // 0xFFCC
        assert_eq!(65518, DshotEncoder::encode_raw_unidirectional(2047)); // 0xFFEB, 0xFFFF=65535

        // testing out of range values
        assert_eq!(0, DshotEncoder::encode_raw_unidirectional(0));
        assert_eq!(34, DshotEncoder::encode_raw_unidirectional(1));
        assert_eq!(68, DshotEncoder::encode_raw_unidirectional(2));
        assert_eq!(325, DshotEncoder::encode_raw_unidirectional(10));

        //assert_eq!(1, DshotCodec::frame_unidirectional(2048));
        //assert_eq!(35, DshotCodec::frame_unidirectional(2049));
        //assert_eq!(69, DshotCodec::frame_unidirectional(2050));
    }
    #[test]
    fn commands() {
        assert_eq!(0, DshotEncoder::encode_command_unidirectional(Command::MotorStop));
        //            SSSS_SSSS_SSST_CCCC
        assert_eq!(0b_0000_0000_0010_0010, DshotEncoder::encode_command_unidirectional(Command::Beep1));
        assert_eq!(
            0b_0000_0101_1100_1001,
            DshotEncoder::encode_command_unidirectional(Command::SignalLineERPMTelemetry)
        );
        assert_eq!(
            0b_0000_0101_1110_1011,
            DshotEncoder::encode_command_unidirectional(Command::SignalLineERPMPeriodTelemetry)
        );
        // bidirectional form is the same with the checksum bits inverted
        assert_eq!(0b_0000_0000_0010_1101, DshotEncoder::encode_command_bidirectional(Command::Beep1));
        assert_eq!(
            0b_0000_0101_1100_0110,
            DshotEncoder::encode_command_bidirectional(Command::SignalLineERPMTelemetry)
        );
        assert_eq!(
            0b_0000_0101_1110_0100,
            DshotEncoder::encode_command_bidirectional(Command::SignalLineERPMPeriodTelemetry)
        );
    }
}
