use core::ops::Deref;

use super::{DshotError, Telemetry};

/// `ErpmTelemetryFrame` : returned from ESC after setting the output in bidirectional mode.
/// ```text
/// eRPM Telemetry Frame Structure.
///
/// The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
///
///     eeem mmmm mmmm cccc
///
/// where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
/// The resultant value is the mantissa shifted left by the exponent.
/// ```
#[derive(Debug, Copy, Clone, Default, Eq, PartialEq, PartialOrd, Ord)]
pub struct ErpmTelemetryFrame(u16);

impl TryFrom<u16> for ErpmTelemetryFrame {
    type Error = u16;

    #[inline]
    fn try_from(value: u16) -> Result<Self, u16> {
        if ErpmTelemetryFrame::is_checksum_ok(value) { Ok(ErpmTelemetryFrame::from_raw(value)) } else { Err(value) }
    }
}

impl From<ErpmTelemetryFrame> for u16 {
    #[inline]
    fn from(frame: ErpmTelemetryFrame) -> Self {
        frame.raw()
    }
}

impl Deref for ErpmTelemetryFrame {
    type Target = u16;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

#[allow(unused)]
impl ErpmTelemetryFrame {
    // eeem mmmm mmmm cccc
    const CHECKSUM_BITS: u16 = 0x000F;
    const MANTISSA_BITS: u16 = 0x1FF0;
    const EXPONENT_BITS: u16 = 0xF000;
    const ONE_MINUTE_IN_MICROSECONDS: u32 = 60_000_000;

    #[inline]
    #[must_use]
    pub const fn from_raw(value: u16) -> Self {
        Self(value)
    }

    #[inline]
    #[must_use]
    pub const fn from_exponent_mantissa(exponent: u16, mantissa: u16) -> Self {
        let raw_12 = (exponent << 9) | (mantissa & 0x1FFF);
        Self((raw_12 << 4) | Self::calculate_checksum(raw_12))
    }

    #[inline]
    #[must_use]
    pub const fn raw(self) -> u16 {
        self.0
    }

    #[must_use]
    pub const fn calculate_checksum(raw_12: u16) -> u16 {
        (!(raw_12 ^ (raw_12 >> 4) ^ (raw_12 >> 8))) & 0x0F
    }

    #[inline]
    #[must_use]
    pub const fn checksum(self) -> u16 {
        self.0 & Self::CHECKSUM_BITS
    }

    /// Check if checksum is ok (XOR of all 4 nibbles must equal 0x0F).
    #[inline]
    #[must_use]
    pub const fn is_checksum_ok(value: u16) -> bool {
        let checksum = (value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F;
        checksum == 0x0F
    }

    /// Check if checksum is ok (XOR of all 4 nibbles must equal 0x0F).
    #[inline]
    #[must_use]
    pub const fn checksum_is_ok(self) -> bool {
        let checksum = (self.0 ^ (self.0 >> 4) ^ (self.0 >> 8) ^ (self.0 >> 12)) & 0x0F;
        checksum == 0x0F
    }

    #[inline]
    #[must_use]
    pub const fn mantissa(self) -> u16 {
        self.0 & Self::MANTISSA_BITS >> 4
    }

    #[inline]
    #[must_use]
    pub const fn exponent(self) -> u16 {
        self.0 & Self::EXPONENT_BITS >> 13
    }

    #[inline]
    #[must_use]
    fn period_us(self) -> u32 {
        u32::from(self.mantissa()) << self.exponent()
    }

    #[inline]
    #[must_use]
    pub fn erpm(self) -> u32 {
        Self::ONE_MINUTE_IN_MICROSECONDS.checked_div(self.period_us()).unwrap_or_default()
    }

    /// Decode `erpm`.
    /// # Errors `DecodeError`
    pub const fn decode_erpm(self) -> Result<u16, DshotError> {
        let value = self.0 >> 4;
        if value == 0x0FFF {
            return Ok(0);
        }
        let mantissa: u16 = value & 0x01FF;
        let exponent: u16 = (value & 0xFE00) >> 9;
        let result = mantissa << exponent;
        if result == 0 {
            return Err(DshotError::InvalidErpm);
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

    #[must_use]
    pub fn decode_telemetry(self) -> Telemetry {
        let raw_12 = self.0 >> 4;
        let exponent = (raw_12 >> 9) & 0x07;
        let bit8 = (raw_12 >> 8) & 1;

        if exponent == 0 || bit8 == 1 {
            if raw_12 == 0 || raw_12 == 0x0FFF {
                return Telemetry::Erpm(0);
            }
            let mantissa = raw_12 & 0x1FF;
            let period_us = u32::from(mantissa) << u32::from(exponent);
            if period_us == 0 {
                return Telemetry::Erpm(0);
            }
            return Telemetry::Erpm(Self::ONE_MINUTE_IN_MICROSECONDS / period_us);
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
}
#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<ErpmTelemetryFrame>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn temperature() {
        let frame = ErpmTelemetryFrame::from_exponent_mantissa(1, 25);
        assert_eq!(frame.decode_telemetry(), Telemetry::Temperature(25));
        let frame = ErpmTelemetryFrame::from_exponent_mantissa(1, 100);
        assert_eq!(frame.decode_telemetry(), Telemetry::Temperature(100));
        let frame = ErpmTelemetryFrame::from_exponent_mantissa(1, 255);
        assert_eq!(frame.decode_telemetry(), Telemetry::Temperature(255));
    }
}
