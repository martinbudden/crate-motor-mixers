use core::ops::Deref;

use super::{DshotError, Telemetry};

/// In bidirectional Dshot, the ESC sends a GCR21 frame to the Flight Controller.
/// The is then decoded into an `ErpmTelemetryFrame`
/// ```text
/// eRPM Telemetry Frame Structure.
///
/// The eRPM telemetry frame sent by the ESC in bidirectional DSHOT mode is a 16 bit value, in the format:
///
///     eeem mmmm mmmm cccc
///
/// where m is the 9-bit mantissa and e is the 3 bit exponent and cccc the checksum.
/// The 9 bit value M is shifted left E times to get the period in micro seconds.
/// This gives a range of 1 us to 65408 us.
/// Which translates to a minimum e-frequency of 15.29 hz (for 14 pole motors that is 3.82 hz).
/// ```
#[derive(Debug, Copy, Clone, Eq, PartialEq, PartialOrd, Ord)]
pub struct ErpmTelemetryFrame(u16);

impl Default for ErpmTelemetryFrame {
    fn default() -> Self {
        Self::from_raw_12(0)
    }
}

impl TryFrom<u16> for ErpmTelemetryFrame {
    type Error = DshotError;

    #[inline]
    fn try_from(raw_16: u16) -> Result<Self, DshotError> {
        ErpmTelemetryFrame::try_from_raw_16(raw_16)
    }
}

impl From<ErpmTelemetryFrame> for u16 {
    #[inline]
    fn from(frame: ErpmTelemetryFrame) -> Self {
        frame.raw_16()
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
    pub const fn from_raw_12(raw_12: u16) -> Self {
        Self((raw_12 << 4) | Self::calculate_checksum(raw_12))
    }

    /// # Errors
    pub fn try_from_raw_16(raw_16: u16) -> Result<Self, DshotError> {
        if Self::is_checksum_ok(raw_16) { Ok(Self(raw_16)) } else { Err(DshotError::InvalidChecksum) }
    }

    #[inline]
    #[must_use]
    pub const fn raw_16(self) -> u16 {
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
    pub const fn from_exponent_mantissa(exponent: u16, mantissa: u16) -> Self {
        let raw_12 = (exponent << 9) | (mantissa & 0x1FFF);
        Self::from_raw_12(raw_12)
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
    #[inline]
    #[must_use]
    pub fn erpm_f32(self) -> f32 {
        #[allow(clippy::cast_precision_loss)]
        {
            self.erpm() as f32
        }
    }

    /// Decode `erpm`.
    /// # Errors `DecodeError`
    pub const fn decode_erpm(self) -> Result<u16, DshotError> {
        let e3m9 = self.0 >> 4;
        if e3m9 == 0x0FFF {
            return Ok(0);
        }
        let mantissa: u16 = e3m9 & 0x01FF;
        let exponent: u16 = (e3m9 & 0xFE00) >> 9;
        let result = mantissa << exponent;
        if result == 0 {
            return Err(DshotError::InvalidErpm);
        }
        Ok(result)
    }

    /*fn decode_telemetry_frame(value: u16) -> Result<TelemetryFrame, DshotError> {
        let type_val = (value & 0x0F00) >> 8;
        let is_erpm = (type_val & 0x01) != 0 || type_val == 0;
        if is_erpm {
            let result = Self::decode_erpm(value)?;
            return Ok(TelemetryFrame::Erpm(u32::from(result)));
        }
        let type_val = (value & 0x0F00) >> 8;
        Ok((value & 0x00FF, TelemetryType::from_u16(type_val >> 1)))
    }*/

    /// # Errors
    pub fn try_decode_erpm(self) -> Result<u32, DshotError> {
        let raw_12 = self.0 >> 4;
        let exponent = (raw_12 >> 9) & 0x07;
        let bit8 = (raw_12 >> 8) & 1;

        if exponent == 0 || bit8 == 1 {
            if raw_12 == 0 || raw_12 == 0x0FFF {
                return Ok(0);
            }
            let mantissa = raw_12 & 0x1FF;
            let period_us = u32::from(mantissa) << u32::from(exponent);
            if period_us == 0 {
                return Ok(0);
            }
            return Ok(Self::ONE_MINUTE_IN_MICROSECONDS / period_us);
        }
        Err(DshotError::InvalidErpm)
    }

    /// # Errors
    pub fn try_decode_telemetry(self) -> Result<Telemetry, DshotError> {
        let raw_12 = self.0 >> 4;
        let exponent = (raw_12 >> 9) & 0x07;
        let bit8 = (raw_12 >> 8) & 1;

        if exponent == 0 || bit8 == 1 {
            if raw_12 == 0 || raw_12 == 0x0FFF {
                return Ok(Telemetry::Erpm(0));
            }
            let mantissa = raw_12 & 0x1FF;
            let period_us = u32::from(mantissa) << u32::from(exponent);
            if period_us == 0 {
                return Ok(Telemetry::Erpm(0));
            }
            return Ok(Telemetry::Erpm(Self::ONE_MINUTE_IN_MICROSECONDS / period_us));
        }

        let data = (raw_12 & 0xFF) as u8;
        match exponent {
            1 => Ok(Telemetry::Temperature(data)),
            2 => Ok(Telemetry::Voltage(u32::from(data) * 250)),
            3 => Ok(Telemetry::Current(u32::from(data) * 1000)),
            4 => Ok(Telemetry::Debug1(data)),
            5 => Ok(Telemetry::Debug2(data)),
            6 => Ok(Telemetry::Debug3(data)),
            7 => Ok(Telemetry::StateEvent(data)),
            _ => Err(DshotError::InvalidTelemetry),
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
        assert_eq!(frame.try_decode_telemetry(), Ok(Telemetry::Temperature(25)));
        let frame = ErpmTelemetryFrame::from_exponent_mantissa(1, 100);
        assert_eq!(frame.try_decode_telemetry(), Ok(Telemetry::Temperature(100)));
        let frame = ErpmTelemetryFrame::from_exponent_mantissa(1, 255);
        assert_eq!(frame.try_decode_telemetry(), Ok(Telemetry::Temperature(255)));
    }
}
