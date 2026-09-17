use core::ops::Deref;

use crate::dshot::DshotCommand;

/// `DshotCommandFrame`: transmitted Flight Controller to ESC.
///
/// Whenever the FC wants the motor to spin, beep, or change direction, it transmits a 16-bit `DshotCommandFrame`.
/// Bits 0–10 (11 bits): Throttle value or Command id.
///     Values 48 to 2047 represent motor speed (throttle).
///     Values 1 to 47 are reserved for commands.
/// Bit 11     (1 bit):      Telemetry Request Flag.
/// Bits 12–15 (4 bits): Checksum (technically a 4-bit Longitudinal Redundancy Check (LRC)).
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, PartialOrd, Ord)]
pub struct DshotCommandFrame(u16);

impl TryFrom<u16> for DshotCommandFrame {
    type Error = u16;

    #[inline]
    fn try_from(value: u16) -> Result<Self, u16> {
        if value <= Self::MAX_RAW_VALUE {
            Ok(DshotCommandFrame::encode_raw(value, DshotCommandFrame::NO_TELEMETRY))
        } else {
            Err(value)
        }
    }
}

impl From<DshotCommandFrame> for u16 {
    #[inline]
    fn from(frame: DshotCommandFrame) -> Self {
        frame.raw()
    }
}

impl Deref for DshotCommandFrame {
    type Target = u16;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DshotCommandFrame {
    pub const NO_TELEMETRY: bool = false;
    pub const WITH_TELEMETRY: bool = true;

    // `Dshot` command/throttle payload is 11 bits (0 to 2047)
    pub const MAX_RAW_VALUE: u16 = 2047;
    pub const THROTTLE_OFFSET: u16 = 48;
    pub const THROTTLE_MIN: u16 = 48;
    pub const THROTTLE_MAX: u16 = 2047;

    const TELEMETRY_BIT: u16 = 0x10;
    const CHECKSUM_BITS: u16 = 0x0F;

    // 4-bit to 5-bit GCR translation table.
    pub(crate) const NIBBLE_TO_QUINTET: [u8; 16] =
        [0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F];

    #[inline]
    #[must_use]
    pub const fn new(value: u16) -> Self {
        Self::encode_raw(value, Self::NO_TELEMETRY)
    }

    #[inline]
    #[must_use]
    pub const fn from_raw(value: u16) -> Self {
        Self(value)
    }

    #[inline]
    #[must_use]
    pub const fn from_command(command: DshotCommand) -> Self {
        Self::encode_raw(command as u16, Self::WITH_TELEMETRY)
    }

    #[inline]
    #[must_use]
    pub const fn from_command_telemetry(command: DshotCommand, with_telemetry: bool) -> Self {
        Self::encode_raw(command as u16, with_telemetry)
    }

    #[inline]
    #[must_use]
    pub const fn raw(self) -> u16 {
        self.0
    }

    /// Extracts the original 11-bit command/throttle value from the processed 16-bit frame.
    #[inline]
    #[must_use]
    pub const fn value(self) -> u16 {
        self.0 >> 5
    }

    #[inline]
    #[must_use]
    pub const fn is_telemetry_enabled(self) -> bool {
        (self.0 & Self::TELEMETRY_BIT) != 0
    }

    #[inline]
    #[must_use]
    pub const fn checksum(self) -> u16 {
        self.0 & Self::CHECKSUM_BITS
    }

    /// Calculates the standard 4-bit XOR outbound checksum.
    /// Note: Unlike telemetry, outbound frames do NOT invert the result.
    #[inline]
    #[must_use]
    pub const fn calculate_checksum(frame_raw: u16) -> u16 {
        (frame_raw ^ (frame_raw >> 4) ^ (frame_raw >> 8)) & 0x0F
    }

    /// Assembles an 11-bit command and a telemetry flag into a complete 16-bit `Dshot` transmission word.
    #[must_use]
    pub const fn encode_raw(value: u16, with_telemetry: bool) -> Self {
        // Clamp input value to prevent register overflow corruption
        let value = if value > Self::MAX_RAW_VALUE { Self::MAX_RAW_VALUE } else { value };

        // Shift left by 1 and inject the telemetry selection bit
        let frame_raw = if with_telemetry { (value << 1) | 0x01 } else { value << 1 };

        Self((frame_raw << 4) | Self::calculate_checksum(frame_raw))
    }

    /// Converts a throttle scale `[0.0, 1.0]` directly to the `Dshot` frame range `[48, 2047]`.
    #[must_use]
    pub fn from_throttle(throttle: f32) -> Self {
        #[allow(unused)]
        use num_traits::float::FloatCore;

        // Clamp throttle to prevent out-of-bounds calculations
        let throttle = throttle.clamp(0.0, 1.0);

        // Scale linearly across the available 1999 active throttle steps
        let range = f32::from(Self::THROTTLE_MAX - Self::THROTTLE_MIN);
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        let dshot_value = (throttle * range).round() as u16 + Self::THROTTLE_MIN;

        // Note: For standard flight controller loops, from_throttle commands typically
        // default to NO_TELEMETRY unless explicitly handling a BDShot scheduling block.
        Self::encode_raw(dshot_value, Self::NO_TELEMETRY)
    }
}

impl DshotCommandFrame {
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

    #[must_use]
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
        is_full::<DshotCommandFrame>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_codec_checksum() {
        assert_eq!(DshotCommandFrame::calculate_checksum(0b_1000_0010_1100), 0b_0000_0000_0110,);
    }
    #[test]
    fn dshot_codec() {
        assert_eq!(48, DshotCommandFrame::from_throttle(0.0).value());
        assert_eq!(548, DshotCommandFrame::from_throttle(0.25).value());
        assert_eq!(1048, DshotCommandFrame::from_throttle(0.5).value());
        assert_eq!(1547, DshotCommandFrame::from_throttle(0.75).value());
        assert_eq!(2047, DshotCommandFrame::from_throttle(1.0).value());

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
        assert_eq!(1, DshotCommandFrame::from_command_telemetry(DshotCommand::Beep1, DshotCommandFrame::NO_TELEMETRY).value());
        assert_eq!(0b_0000_0000_0010_0010, DshotCommandFrame::from_command_telemetry(DshotCommand::Beep1, DshotCommandFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0000_0011_0011, DshotCommandFrame::from_command_telemetry(DshotCommand::Beep1, DshotCommandFrame::WITH_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1100_1001, DshotCommandFrame::from_command_telemetry(DshotCommand::SignalLineErpmTelemetry, DshotCommandFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1101_1000, DshotCommandFrame::from_command_telemetry(DshotCommand::SignalLineErpmTelemetry, DshotCommandFrame::WITH_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1110_1011, DshotCommandFrame::from_command_telemetry(DshotCommand::SignalLineErpmPeriodTelemetry, DshotCommandFrame::NO_TELEMETRY).raw());
        assert_eq!(0b_0000_0101_1111_1010, DshotCommandFrame::from_command_telemetry(DshotCommand::SignalLineErpmPeriodTelemetry, DshotCommandFrame::WITH_TELEMETRY).raw());
    }
    /*#[test]
    fn test_dshot_checksum_values() {
        // --- Case 1: Pure Zero (e.g., Disarmed / Throttle 0, No Telemetry) ---
        // Inner value = 0. Shifted value for checksum calculation = 0 << 1 = 0
        // Checksum formula: (!(0 ^ 0 ^ 0)) & 0x0F = 0x0F (15)
        let frame_zero = DshotCommandFrame::new(0);
        assert_eq!(frame_zero.checksum(), 0);
        // Encoded: (0 << 5) | 15 = 15 (0x000F)
        assert_eq!(frame_zero.raw(), 0);

        // --- Case 2: Minimum Throttle Command (`Dshot` value 48) ---
        // Inner value = 48 (0x30 or 0b0011_0000)
        // Shifted value = 48 << 1 = 96 (0x60 or 0b0110_0000)
        // Nibble 0 (bits 0-3):   0x0 (0b0000)
        // Nibble 1 (bits 4-7):   0x6 (0b0110)
        // Nibble 2 (bits 8-11):  0x0 (0b0000)
        // XOR: 0x0 ^ 0x6 ^ 0x0 = 0x6
        // NOT & Mask: (!0x6) & 0x0F = 0x9 (9)
        let frame = DshotCommandFrame::new(48);
        //assert_eq!(frame.checksum(), 9);
        // Encoded: (96 << 4) | 9 = 1536 | 9 = 1545 (0x0609)
        assert_eq!(frame.raw(), 0x0609);

        // --- Case 3: High Throttle (`Dshot` value 1000) ---
        // Inner value = 1000 (0x3E8)
        // Shifted value = 1000 << 1 = 2000 (0x7D0 or 0b0111_1101_0000)
        // Nibble 0 (bits 0-3):   0x0 (0b0000)
        // Nibble 1 (bits 4-7):   0xD (0b1101)
        // Nibble 2 (bits 8-11):  0x7 (0b0111)
        // XOR: 0x0 ^ 0xD ^ 0x7 = 0xA (0b1010)
        // NOT & Mask: (!0xA) & 0x0F = 0x5 (5)
        let frame = DshotCommandFrame::new(1000);
        assert_eq!(frame.checksum(), 5);
        // Encoded: (2000 << 4) | 5 = 32000 | 5 = 32005 (0x7D05)
        assert_eq!(frame.raw(), 32005);

        // --- Case 4: Maximum Possible Value (`Dshot` value 2047) ---
        // Inner value = 2047 (0x7FF)
        // Shifted value = 2047 << 1 = 4094 (0xFFE or 0b1111_1111_1110)
        // Nibble 0 (bits 0-3):   0xE (0b1110)
        // Nibble 1 (bits 4-7):   0xF (0b1111)
        // Nibble 2 (bits 8-11):  0xF (0b1111)
        // XOR: 0xE ^ 0xF ^ 0xF = 0xE (0b1110)
        // NOT & Mask: (!0xE) & 0x0F = 0x1 (1)
        let frame = DshotCommandFrame::new(2047);
        assert_eq!(frame.checksum(), 1);
        // Encoded: (4094 << 4) | 1 = 65504 | 1 = 65505 (0xFFE1)
        assert_eq!(frame.raw(), 65505);
    }*/
}
#[cfg(test)]
mod command_frame_tests {
    #![allow(clippy::unwrap_used)]
    use super::*;

    #[test]
    fn test_encode_raw_no_telemetry() {
        // Let's pick a standard throttle value: 1000
        // 1. Shift left by 1 for telemetry bit (0): 1000 << 1 = 2000 (0x7D0)
        // 2. Calculate checksum:
        //    nibble0 = 0x0, nibble1 = 0xD, nibble2 = 0x7
        //    0x0 ^ 0xD ^ 0x7 = 0xA
        // 3. Shift payload left by 4 and add checksum: (2000 << 4) | 0xA = 32010 (0x7D0A)
        let frame = DshotCommandFrame::encode_raw(1000, DshotCommandFrame::NO_TELEMETRY);

        assert_eq!(frame.raw(), 0x7D0A);
        assert_eq!(frame.value(), 1000);
        assert!(!frame.is_telemetry_enabled());
        assert_eq!(frame.checksum(), 0x0A);
    }

    #[test]
    fn test_encode_raw_with_telemetry() {
        // Let's pick the same throttle value (1000) but enable bidirectional telemetry
        // 1. Shift left by 1 and inject telemetry bit (1): (1000 << 1) | 1 = 2001 (0x7D1)
        // 2. Calculate checksum:
        //    nibble0 = 0x1, nibble1 = 0xD, nibble2 = 0x7
        //    0x1 ^ 0xD ^ 0x7 = 0xB
        // 3. Shift payload left by 4 and add checksum: (2001 << 4) | 0xB = 32027 (0x7D1B)
        let frame = DshotCommandFrame::encode_raw(1000, DshotCommandFrame::WITH_TELEMETRY);

        assert_eq!(frame.raw(), 0x7D1B);
        assert_eq!(frame.value(), 1000);
        assert!(frame.is_telemetry_enabled());
        assert_eq!(frame.checksum(), 0x0B);
    }

    #[test]
    fn test_try_from_valid_and_invalid() {
        // Max valid raw value is 2047
        let valid_res = DshotCommandFrame::try_from(2047);
        assert!(valid_res.is_ok());
        assert_eq!(valid_res.unwrap().value(), 2047);

        // Over the limit should return the error value
        let invalid_res = DshotCommandFrame::try_from(2048);
        assert!(invalid_res.is_err());
        assert_eq!(invalid_res.unwrap_err(), 2048);
    }

    #[test]
    fn test_from_command_mapping() {
        // Using MotorStop (value 0)
        let frame = DshotCommandFrame::from_command(DshotCommand::MotorStop);
        assert_eq!(frame.value(), 0);
        assert!(frame.is_telemetry_enabled());

        // Using BeepTone1 (value 1) with telemetry explicitly enabled
        let frame_telemetry = DshotCommandFrame::from_command_telemetry(DshotCommand::Beep1, true);
        assert_eq!(frame_telemetry.value(), 1);
        assert!(frame_telemetry.is_telemetry_enabled());
    }

    #[test]
    fn test_from_throttle_scaling() {
        // 1. Minimum Active Throttle (0.0) -> Should map to THROTTLE_MIN (48)
        let frame_min = DshotCommandFrame::from_throttle(0.0);
        assert_eq!(frame_min.value(), 48);

        // 2. Maximum Active Throttle (1.0) -> Should map to THROTTLE_MAX (2047)
        let frame_max = DshotCommandFrame::from_throttle(1.0);
        assert_eq!(frame_max.value(), 2047);

        // 3. Midpoint Throttle (0.5) -> (2047 - 48) * 0.5 = 999.5 -> rounded to 1000 -> + 48 = 1048
        let frame_midpoint = DshotCommandFrame::from_throttle(0.5);
        assert_eq!(frame_midpoint.value(), 1048);
    }

    #[test]
    fn test_from_throttle_clamping() {
        // Negative inputs should be clamped safely to 0.0 -> evaluating to 48
        let frame_neg = DshotCommandFrame::from_throttle(-0.25);
        assert_eq!(frame_neg.value(), 48);

        // Over-unity inputs should be clamped safely to 1.0 -> evaluating to 2047
        let frame_over = DshotCommandFrame::from_throttle(1.5);
        assert_eq!(frame_over.value(), 2047);
    }

    #[test]
    fn test_input_value_clamping_protection() {
        // If an absolute rogue value over 2047 bypasses validation into encode_raw directly,
        // the constructor must clamp it to 2047 instead of shifting out-of-bounds junk bits
        let frame = DshotCommandFrame::encode_raw(9999, DshotCommandFrame::NO_TELEMETRY);
        assert_eq!(frame.value(), 2047);
    }
}
