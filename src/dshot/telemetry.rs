#![allow(unused)]

#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum TelemetryType {
    #[default]
    Erpm = 0,
    Temperature = 1,
    Voltage = 2,
    Current = 3,
    Debug1 = 4,
    Debug2 = 5,
    StressLevel = 6,
    StateEvents = 7,
    Invalid = 0xFF,
}

impl TelemetryType {
    /// Forgiving conversion, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::Erpm,
            1 => Self::Temperature,
            2 => Self::Voltage,
            3 => Self::Current,
            5 => Self::Debug1,
            6 => Self::Debug2,
            7 => Self::StressLevel,
            8 => Self::StateEvents,
            0xFF => Self::Invalid,
            _ => Self::default(),
        }
    }

    #[must_use]
    pub fn from_u16(value: u16) -> Self {
        if value > 255 { TelemetryType::Invalid } else { Self::from_u8(value.to_le_bytes()[0]) }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Telemetry {
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

#[cfg(test)]
mod test_traits {
    use super::*;

    //fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    fn is_full_eq<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + Eq + PartialEq>() {}
    fn is_full_eq_no_default<T: Sized + Send + Sync + Unpin + Copy + Clone + Eq + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full_eq::<TelemetryType>();
        is_full_eq_no_default::<Telemetry>();
    }
}
