use core::ops::Deref;

use crate::dshot::Command;

#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, PartialOrd, Ord)]
pub struct DshotBidirectionalFrame(u16);

impl TryFrom<u16> for DshotBidirectionalFrame {
    type Error = u16;

    #[inline]
    fn try_from(value: u16) -> Result<Self, u16> {
        if value <= Self::MAX_RAW_VALUE {
            Ok(DshotBidirectionalFrame::encode_raw(value, DshotBidirectionalFrame::NO_TELEMETRY))
        } else {
            Err(value)
        }
    }
}

impl From<DshotBidirectionalFrame> for u16 {
    #[inline]
    fn from(frame: DshotBidirectionalFrame) -> Self {
        frame.value()
    }
}

impl Deref for DshotBidirectionalFrame {
    type Target = u16;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[allow(unused)]
impl DshotBidirectionalFrame {
    pub const NO_TELEMETRY: bool = false;
    pub const WITH_TELEMETRY: bool = true;
    // DShot throttle is strictly 11 bits (0 to 2047)
    pub const MAX_RAW_VALUE: u16 = 2047;
    pub const THROTTLE_OFFSET: u16 = 48;
    const THROTTLE_MIN: u16 = 48;
    const THROTTLE_MAX: u16 = 2047;

    const TELEMETRY_BIT: u16 = 0x10;
    const CHECKSUM_BITS: u16 = 0x0F;

    pub(crate) const NIBBLE_TO_QUINTET: [u8; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    pub const fn new(value: u16) -> Self {
        Self::encode_raw(value, Self::NO_TELEMETRY)
    }

    pub const fn from_raw(value: u16) -> Self {
        Self(value)
    }

    pub const fn from_command(command: Command) -> Self {
        Self::encode_raw(command as u16, Self::NO_TELEMETRY)
    }

    pub const fn from_command_telemetry(command: Command, with_telemetry: bool) -> Self {
        Self::encode_raw(command as u16, with_telemetry)
    }

    pub const fn raw(self) -> u16 {
        self.0
    }

    pub const fn value(self) -> u16 {
        self.0 >> 5
    }

    /// Returns whether telemetry is enabled.
    pub const fn is_telemetry_enabled(self) -> bool {
        self.0 & Self::TELEMETRY_BIT != 0
    }

    pub const fn checksum(self) -> u16 {
        self.0 & Self::CHECKSUM_BITS
    }

    pub const fn calculate_checksum(frame_raw: u16) -> u16 {
        (!(frame_raw ^ (frame_raw >> 4) ^ (frame_raw >> 8))) & 0x0F
    }

    pub const fn encode_raw(frame_raw: u16, with_telemetry: bool) -> Self {
        let frame_raw = if with_telemetry { frame_raw << 1 | 0x01 } else { frame_raw << 1 };
        Self((frame_raw << 4) | Self::calculate_checksum(frame_raw))
    }

    const fn pwm_to_dshot_raw(pwm: u16) -> u16 {
        ((pwm - 1000) * 2) + Self::THROTTLE_OFFSET
    }

    /// Convert PWM value (1000-2000) to Dshot value (48-2047),
    /// clamping PWM value to (1000-2000).
    pub const fn pwm_clamped_to_dshot_raw(pwm: u16) -> u16 {
        if pwm >= 2000 {
            Self::THROTTLE_MAX
        } else if pwm >= 1000 {
            Self::pwm_to_dshot_raw(pwm)
        } else {
            Self::THROTTLE_MIN
        }
    }

    /// Convert throttle value [0.0,1.0] to Dshot frame value [48,2047],
    /// clamping PWM value to (1000-2000).
    pub const fn throttle_to_frame(throttle: f32) -> Self {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let pwm = ((throttle.abs() + 1.0) * 1000.0) as u16;
        Self::encode_raw(Self::pwm_clamped_to_dshot_raw(pwm), Self::WITH_TELEMETRY)
    }

    // see [DSHOT - the missing Handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)
    // for a good description of these conversions
    #[inline]
    #[must_use]
    pub fn to_gcr20(self) -> u32 {
        let value = self.0;
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

    pub fn gcr_encode(self) -> u32 {
        let gcr20 = self.to_gcr20();
        Self::gcr20_to_gcr21(gcr20)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotBidirectionalFrame>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_codec_checksum() {
        assert_eq!(0b_0000_0000_1001, DshotBidirectionalFrame::calculate_checksum(0b_1000_0010_1100));
    }
    #[test]
    fn dshot_codec() {
        assert_eq!(48, DshotBidirectionalFrame::pwm_to_dshot_raw(1000));
        assert_eq!(2048, DshotBidirectionalFrame::pwm_to_dshot_raw(2000));

        assert_eq!(48, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(0));
        assert_eq!(48, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(10));
        assert_eq!(48, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(999));

        assert_eq!(48, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1000)); // should this be 0 or 48 ?
        //assert_eq!(48, DshotCodec::pwm_to_dshot_clamped(1000)); // should this be 0 or 48 ?
        assert_eq!(50, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1001));
        assert_eq!(52, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1002));
        assert_eq!(54, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1003));
        assert_eq!(548, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1250));
        assert_eq!(1048, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1500));
        assert_eq!(1548, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1750));
        assert_eq!(2046, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(1999));
        assert_eq!(2047, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(2000));
        assert_eq!(2047, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(2001));
        assert_eq!(2047, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(2002));
        assert_eq!(2047, DshotBidirectionalFrame::pwm_clamped_to_dshot_raw(4000));

        assert_eq!(48, DshotBidirectionalFrame::throttle_to_frame(0.0).value());
        assert_eq!(548, DshotBidirectionalFrame::throttle_to_frame(0.25).value());
        assert_eq!(1048, DshotBidirectionalFrame::throttle_to_frame(0.5).value());
        assert_eq!(1548, DshotBidirectionalFrame::throttle_to_frame(0.75).value());
        assert_eq!(2047, DshotBidirectionalFrame::throttle_to_frame(1.0).value());

        //assert_eq!(1542, DshotFrame::encode_raw(48).as_u16()); //0x606
        /*assert_eq!(1572, DshotFrame::encode_raw_unidirectional(49)); // 0x624
        assert_eq!(33547, DshotFrame::encode_raw_unidirectional(1048)); // 0x830B
        assert_eq!(65484, DshotFrame::encode_raw_unidirectional(2046)); // 0xFFCC
        assert_eq!(65518, DshotFrame::encode_raw_unidirectional(2047)); // 0xFFEB, 0xFFFF=65535

        // testing out of range values
        assert_eq!(0, DshotFrame::encode_raw_unidirectional(0));
        assert_eq!(34, DshotFrame::encode_raw_unidirectional(1));
        assert_eq!(68, DshotFrame::encode_raw_unidirectional(2));
        assert_eq!(325, DshotFrame::encode_raw_unidirectional(10));*/

        //assert_eq!(1, DshotCodec::frame_unidirectional(2048));
        //assert_eq!(35, DshotCodec::frame_unidirectional(2049));
        //assert_eq!(69, DshotCodec::frame_unidirectional(2050));
    }
    #[rustfmt::skip]
    #[test]
    fn commands() {
        assert_eq!(1, DshotBidirectionalFrame::from_command_telemetry(Command::Beep1, DshotBidirectionalFrame::NO_TELEMETRY).value());
        assert_eq!(0b_0000_0000_0010_1101, DshotBidirectionalFrame::from_command_telemetry(Command::Beep1, DshotBidirectionalFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0000_0011_1100, DshotBidirectionalFrame::from_command_telemetry(Command::Beep1, DshotBidirectionalFrame::WITH_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1100_0110, DshotBidirectionalFrame::from_command_telemetry(Command::SignalLineERPMTelemetry, DshotBidirectionalFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1101_0111, DshotBidirectionalFrame::from_command_telemetry(Command::SignalLineERPMTelemetry, DshotBidirectionalFrame::WITH_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1110_0100, DshotBidirectionalFrame::from_command_telemetry(Command::SignalLineERPMPeriodTelemetry, DshotBidirectionalFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1111_0101, DshotBidirectionalFrame::from_command_telemetry(Command::SignalLineERPMPeriodTelemetry, DshotBidirectionalFrame::WITH_TELEMETRY).raw());
    }
    #[test]
    fn test_dshot_checksum_values() {
        // --- Case 1: Pure Zero (e.g., Disarmed / Throttle 0, No Telemetry) ---
        // Inner value = 0. Shifted value for checksum calculation = 0 << 1 = 0
        // Checksum formula: (!(0 ^ 0 ^ 0)) & 0x0F = 0x0F (15)
        let frame_zero = DshotBidirectionalFrame::new(0);
        assert_eq!(15, frame_zero.checksum());
        // Encoded: (0 << 5) | 15 = 15 (0x000F)
        assert_eq!(frame_zero.raw(), 0x000F);

        // --- Case 2: Minimum Throttle Command (DShot value 48) ---
        // Inner value = 48 (0x30 or 0b0011_0000)
        // Shifted value = 48 << 1 = 96 (0x60 or 0b0110_0000)
        // Nibble 0 (bits 0-3):   0x0 (0b0000)
        // Nibble 1 (bits 4-7):   0x6 (0b0110)
        // Nibble 2 (bits 8-11):  0x0 (0b0000)
        // XOR: 0x0 ^ 0x6 ^ 0x0 = 0x6
        // NOT & Mask: (!0x6) & 0x0F = 0x9 (9)
        let frame = DshotBidirectionalFrame::new(48);
        //assert_eq!(frame.checksum(), 9);
        // Encoded: (96 << 4) | 9 = 1536 | 9 = 1545 (0x0609)
        assert_eq!(frame.raw(), 0x0609);

        // --- Case 3: High Throttle (DShot value 1000) ---
        // Inner value = 1000 (0x3E8)
        // Shifted value = 1000 << 1 = 2000 (0x7D0 or 0b0111_1101_0000)
        // Nibble 0 (bits 0-3):   0x0 (0b0000)
        // Nibble 1 (bits 4-7):   0xD (0b1101)
        // Nibble 2 (bits 8-11):  0x7 (0b0111)
        // XOR: 0x0 ^ 0xD ^ 0x7 = 0xA (0b1010)
        // NOT & Mask: (!0xA) & 0x0F = 0x5 (5)
        let frame = DshotBidirectionalFrame::new(1000);
        assert_eq!(frame.checksum(), 5);
        // Encoded: (2000 << 4) | 5 = 32000 | 5 = 32005 (0x7D05)
        assert_eq!(frame.raw(), 32005);

        // --- Case 4: Maximum Possible Value (DShot value 2047) ---
        // Inner value = 2047 (0x7FF)
        // Shifted value = 2047 << 1 = 4094 (0xFFE or 0b1111_1111_1110)
        // Nibble 0 (bits 0-3):   0xE (0b1110)
        // Nibble 1 (bits 4-7):   0xF (0b1111)
        // Nibble 2 (bits 8-11):  0xF (0b1111)
        // XOR: 0xE ^ 0xF ^ 0xF = 0xE (0b1110)
        // NOT & Mask: (!0xE) & 0x0F = 0x1 (1)
        let frame = DshotBidirectionalFrame::new(2047);
        assert_eq!(frame.checksum(), 1);
        // Encoded: (4094 << 4) | 1 = 65504 | 1 = 65505 (0xFFE1)
        assert_eq!(frame.raw(), 65505);
    }
}
