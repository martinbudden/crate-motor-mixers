#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

// parameters to mix function
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(missing_docs)]
pub struct MotorOutputRange {
    /// Minimum motor output, typically set to 5.5% to avoid ESC desynchronization,
    /// may be set to zero if using dynamic idle control or brushed motors.
    pub min: f32,
    /// Maximum motor output, typically set to 1.0.
    pub max: f32,
}

impl MotorOutputRange {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { min: 0.0, max: 1.0 }
    }
}

impl Default for MotorOutputRange {
    fn default() -> Self {
        Self::new()
    }
}

// parameters to mix function
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[allow(missing_docs)]
pub struct MotorMixerParameters {
    /// used by tricopter.
    pub max_servo_angle_radians: f32,
    /// possibly adjusted throttle value for recording by blackbox.
    pub throttle: f32,
    /// used by test code.
    pub undershoot: f32,
    /// used by test code.
    pub overshoot: f32,
}

impl MotorMixerParameters {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { max_servo_angle_radians: 0.0, throttle: 0.0, undershoot: 0.0, overshoot: 0.0 }
    }
}

impl Default for MotorMixerParameters {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    PpmToServo = 19,  // PPM -> servo relay
    DualCopter = 20,
    SingleCopter = 21,
    Atail4 = 22,
    Custom = 23,
    CustomAirplane = 24,
    CustomTri = 25,
    QuadX1234 = 26,
    OctoXp = 27,
    // Don't forget to update COUNT if you add any new mixer types.
}

impl MixerType {
    pub const COUNT: u8 = 28;

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
            //11 => Self::OctoQuadX,
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

impl TryFrom<u8> for MixerType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        if value < Self::COUNT { Ok(Self::from_u8(value)) } else { Err(()) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MixerConfig {
    /// constants compatible with Betaflight `mixerMode_e` enums.
    pub mixer_type: MixerType,
    pub yaw_motors_reversed: u8,
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

impl Default for MixerConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// PWM (analog) or Dshot (digital).
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
pub enum ProtocolFamily {
    Unknown = 0,
    Pwm = 1,
    Dshot = 2,
}

/// Motor protocol.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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
    // Don't forget to update COUNT if you add any new protocols.
}

impl MotorProtocol {
    pub const COUNT: u8 = 10;

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

impl TryFrom<u8> for MotorProtocol {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        if value < Self::COUNT { Ok(Self::from_u8(value)) } else { Err(()) }
    }
}

#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
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

impl Default for MotorDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct MotorConfig {
    pub device: MotorDeviceConfig,
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

impl MotorConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            device: MotorDeviceConfig::new(),
            motor_idle: 550, // 700 for brushed
            max_throttle: 2000,
            min_command: 1000,
            kv: 1960,
            motor_pole_count: 14,
        }
    }
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ServoDeviceConfig {
    /// PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms).
    /// This is the value for servos when they should be in the middle. e.g. 1500.
    pub servo_center_pulse: u16,
    // The update rate of servo outputs, typically 50-498Hz.
    pub servo_pwm_rate: u16,
}

impl ServoDeviceConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self { servo_center_pulse: 1500, servo_pwm_rate: 50 }
    }
}

impl Default for ServoDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ServoConfig {
    pub device: ServoDeviceConfig,
    /// lowpass servo filter frequency selection; 1/1000ths of loop freq.
    pub servo_lowpass_freq: u16,
    // send tail servo correction pulses even when unarmed.
    pub tri_unarmed_servo: u8,
    pub channel_forwarding_start_channel: u8,
}

impl ServoConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            device: ServoDeviceConfig::new(),
            servo_lowpass_freq: 0,
            tri_unarmed_servo: 0,
            channel_forwarding_start_channel: 0,
        }
    }
}

impl Default for ServoConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_config<
        T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq + Serialize + for<'a> Deserialize<'a>,
    >() {
    }

    #[test]
    fn normal_types() {
        is_full::<MotorMixerParameters>();
        is_full::<MixerConfig>();
        is_full::<MotorDeviceConfig>();
        is_full::<MotorConfig>();
        is_full::<ServoDeviceConfig>();
        is_full::<ServoConfig>();
    }
    #[cfg(feature = "serde")]
    #[test]
    fn config_types() {
        is_config::<MotorMixerParameters>();
        is_config::<MixerConfig>();
        is_config::<MotorDeviceConfig>();
        is_config::<MotorConfig>();
        is_config::<ServoDeviceConfig>();
        is_config::<ServoConfig>();
    }
    #[test]
    fn new() {
        let config = MixerConfig::new();
        assert_eq!(MixerType::QuadX, config.mixer_type);
    }
}
