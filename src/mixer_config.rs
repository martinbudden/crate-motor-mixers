#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[repr(u8)]
#[allow(missing_docs)]
pub enum MixerType {
    Tricopter = 1,
    //QuadP = 2,
    #[default]
    QuadX = 3,
    Bicopter = 4,
    Gimbal = 5,
    Y6 = 6,
    HexP = 7,
    FlyingWingSinglePropeller = 8,
    Y4 = 9,
    HexX = 10,
    OctoQuadX = 11,
    OctoFlatP = 12,
    OctoFlatX = 13,
    AirplaneSinglePropeller = 14,
    Heli120Ccpm = 15,
    Heli90Deg = 16,
    Vtail4 = 17,
    HexH = 18,
    PpmToServo = 19, // PPM -> servo relay
    DualCopter = 20,
    SingleCopter = 21,
    Atail4 = 22,
    Custom = 23,
    CustomAirplane = 24,
    CustomTri = 25,
    QuadX1234 = 26,
    OctoXp = 27,
}

impl TryFrom<u8> for MixerType {
    type Error = ();

    /// Validating conversion from `u8` to `MixerType`. Invalid values return error.
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

impl MixerType {
    /// Forgiving conversion from `u8` to `MixerType`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::Tricopter,
            //2 => Self::QuadP,
            3 => Self::QuadX,
            4 => Self::Bicopter,
            //5 => Self::Gimbal,
            //6 => Self::Y6,
            //7 => Self::HexP,
            8 => Self::FlyingWingSinglePropeller,
            //9 => Self::Y4,
            #[cfg(feature = "eight_motors")]
            10 => Self::HexX,
            #[cfg(feature = "eight_motors")]
            11 => Self::OctoQuadX,
            //12 => Self::OctoFlatP,
            //13 => Self::OctoFlatX,
            14 => Self::AirplaneSinglePropeller,
            //15 => Self::Heli120Ccpm,
            //16 => Self::Heli90Deg,
            //17 => Self::Vtail4,
            //18 => Self::HexH,
            //19 => Self::PpmToServo,
            //20 => Self::DualCopter,
            //21 => Self::SingleCopter,
            //22 => Self::Atail4,
            //23 => Self::Custom,
            //24 => Self::CustomAirplane,
            //25 => Self::CustomTri,
            //26 => Self::QuadX1234,
            //27 => Self::OctoXp,
            _ => Self::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct MixerConfig {
    /// constants compatible with Betaflight `mixerMode_e` enums.
    pub mixer_type: MixerType,
    pub yaw_motors_reversed: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for MixerConfig {}

impl Default for MixerConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self { mixer_type: MixerType::QuadX, yaw_motors_reversed: 1 }
    }
    pub fn set_mixer_type(&mut self, mixer_type: u8) {
        self.mixer_type = MixerType::from_u8(mixer_type);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct MotorConfig {
    /// percentage of the motor range added to the disarmed value to give the idle value.
    pub motor_idle: u16,
    // value of throttle at full power, can be set up to 2000.
    pub max_throttle: u16,
    // value for ESCs when they are not armed. For some specific ESCs this value must be lowered to 900.
    pub min_command: u16,
    // Motor constant: estimated RPM under no load.
    pub kv: u16,
    // Number of motor poles, used to calculate actual RPM from eRPM.
    pub motor_pole_count: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for MotorConfig {}

impl Default for MotorConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorConfig {
    const DEFAULT_MOTOR_POLE_COUNT: u8 = 14;
    #[must_use]
    pub const fn new() -> Self {
        Self {
            motor_idle: 550, // 700 for brushed
            max_throttle: 2000,
            min_command: 1000,
            kv: 1960,
            motor_pole_count: Self::DEFAULT_MOTOR_POLE_COUNT,
        }
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
        is_full::<MixerConfig>();
        is_full::<MotorConfig>();
    }
    #[cfg(feature = "serde")]
    #[test]
    fn serde_types() {
        is_serde::<MixerConfig>();
        is_serde::<MotorConfig>();
    }
    #[cfg(feature = "storage")]
    #[test]
    fn storage_types() {
        is_storage::<MixerConfig>();
        is_storage::<MotorConfig>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new() {
        let config = MixerConfig::new();
        assert_eq!(MixerType::QuadX, config.mixer_type);
    }
}
