use dshot_codec::DshotSpeed;
#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

/// Motor protocol.
/// Betaflight compatible values.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[repr(u8)]
pub enum MotorProtocol {
    #[default]
    Pwm = 0,
    OneShot125 = 1,
    OneShot42 = 2,
    MultiShot = 3,
    Brushed = 4,
    Dshot150 = 5,
    Dshot300 = 6,
    Dshot600 = 7,
    Proshot1000 = 8,
    Disabled = 9,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for MotorProtocol {}

impl TryFrom<u8> for MotorProtocol {
    type Error = ();

    /// Validating conversion from `u8` to `MotorProtocol`. Invalid values return error.
    fn try_from(value: u8) -> Result<Self, ()> {
        let default = Self::default();
        if value == default as u8 {
            Ok(default)
        } else {
            let ret = Self::from_u8(value);
            if ret == default { Err(()) } else { Ok(ret) }
        }
    }
}

impl TryFrom<DshotSpeed> for MotorProtocol {
    type Error = ();

    /// Validating conversion from `DshotSpeed` to `MotorProtocol`. `Dshot1200` returns error.
    fn try_from(dshot_speed: DshotSpeed) -> Result<Self, ()> {
        match dshot_speed {
            DshotSpeed::Dshot150 => Ok(Self::Dshot150),
            DshotSpeed::Dshot300 => Ok(Self::Dshot300),
            DshotSpeed::Dshot600 => Ok(Self::Dshot600),
            DshotSpeed::Dshot1200 => Err(()),
        }
    }
}

impl TryFrom<MotorProtocol> for DshotSpeed {
    type Error = ();

    /// Validating conversion from `MotorProtocol` to `DshotSpeed`. Invalid values return error.
    fn try_from(motor_protocol: MotorProtocol) -> Result<Self, ()> {
        match motor_protocol {
            MotorProtocol::Dshot150 => Ok(Self::Dshot150),
            MotorProtocol::Dshot300 => Ok(Self::Dshot300),
            MotorProtocol::Dshot600 => Ok(Self::Dshot600),
            _ => Err(()),
        }
    }
}

impl MotorProtocol {
    /// Forgiving conversion from `u8` to `MotorProtocol`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            0 => Self::Pwm,
            1 => Self::OneShot125,
            2 => Self::OneShot42,
            3 => Self::MultiShot,
            4 => Self::Brushed,
            5 => Self::Dshot150,
            6 => Self::Dshot300,
            7 => Self::Dshot600,
            8 => Self::Proshot1000,
            9 => Self::Disabled,
            _ => Self::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct MotorDeviceConfig {
    /// The update rate of motor outputs (50-498Hz).
    pub motor_pwm_rate: u16,
    pub motor_protocol: MotorProtocol,
    /// Active-High vs Active-Low. Useful for brushed FCs converted for brushless operation.
    pub motor_inversion: u8,
    pub use_continuous_update: u8,
    pub use_burst_dshot: u8,
    pub use_dshot_telemetry: u8,
    pub use_dshot_edt: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for MotorDeviceConfig {}

impl Default for MotorDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorDeviceConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            motor_pwm_rate: 480, // 16000 for brushed
            motor_protocol: MotorProtocol::Dshot300,
            motor_inversion: 0,
            use_continuous_update: 1,
            use_burst_dshot: 0,
            use_dshot_telemetry: 0,
            use_dshot_edt: 0,
        }
    }
    pub fn set_motor_protocol(&mut self, motor_protocol: u8) {
        self.motor_protocol = MotorProtocol::from_u8(motor_protocol);
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorDeviceConfig>();
        is_full::<MotorProtocol>();
    }
    #[cfg(feature = "serde")]
    #[test]
    fn serde_types() {
        is_serde::<MotorDeviceConfig>();
        is_serde::<MotorProtocol>();
    }
    #[cfg(feature = "storage")]
    #[test]
    fn storage_types() {
        is_storage::<MotorDeviceConfig>();
        is_storage::<MotorProtocol>();
    }
}
