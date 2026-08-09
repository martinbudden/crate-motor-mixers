use core::ops::{Deref, DerefMut};

use signal_filters::SlewRateLimiterf32;

use crate::{
    MixerConfig, MixerType, MotorConfig, MotorMixerParameters, MotorOutputRange,
};

#[cfg(feature = "eight_motors")]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 8;
#[cfg(not(feature = "eight_motors"))]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 4;


/// Common properties of all motor mixers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommon {
    pub outputs: MotorOutputs,
    pub output_filters: MotorOutputFilters,
    pub mixer_type: MixerType,
    pub motor_count: u8,
    pub output_denominator: u8,
    output_count: u8,
    pub mixer_config: MixerConfig,
    pub motor_config: MotorConfig,
    mixer_parameters: MotorMixerParameters,
    /// used for blackbox recording.
    throttle_command: f32,
    motors_is_on: bool,
    motors_is_armed: bool,
    /// reversed motors typically used to flip multi-rotor after a crash.
    motors_is_reversed: bool,
    pub mix_params: MotorMixerParameters,
    pub range: MotorOutputRange,
}

impl MotorMixerCommon {
    #[must_use]
    pub const fn new(mixer_config: MixerConfig, motor_config: MotorConfig) -> Self {
        let motor_count = match mixer_config.mixer_type {
            MixerType::Tricopter | MixerType::CustomTri => 3,
            MixerType::Bicopter | MixerType::DualCopter => 2,
            MixerType::FlyingWingSinglePropeller | MixerType::AirplaneSinglePropeller | MixerType::SingleCopter => 1,
            MixerType::Y6 | MixerType::HexP | MixerType::HexX | MixerType::HexH => 6,
            MixerType::OctoQuadX | MixerType::OctoFlatP | MixerType::OctoFlatX | MixerType::OctoXp => 8,
            _ => 4,
        };
        // output count includes servos.
        let output_count = match mixer_config.mixer_type {
            MixerType::FlyingWingSinglePropeller => 3,
            MixerType::Y6 | MixerType::HexP | MixerType::HexX | MixerType::HexH => 6,
            MixerType::OctoQuadX | MixerType::OctoFlatP | MixerType::OctoFlatX | MixerType::OctoXp => 8,
            _ => 4,
        };
        Self {
            outputs: MotorOutputs::new(),
            output_filters: MotorOutputFilters::new(),
            mixer_type: mixer_config.mixer_type,
            motor_count,
            output_denominator: 1,
            output_count,
            mixer_config,
            motor_config,
            mixer_parameters: MotorMixerParameters::new(),
            throttle_command: 0.0, // used for blackbox recording
            motors_is_on: false,
            motors_is_armed: false,
            motors_is_reversed: false, //reversed motors typically used to flip multi-rotor after a crash
            mix_params: MotorMixerParameters::new(),
            range: MotorOutputRange::new(),
        }
    }
}

impl Default for MotorMixerCommon {
    fn default() -> Self {
        Self::new(MixerConfig::new(), MotorConfig::new())
    }
}

#[allow(unused)]
impl MotorMixerCommon {
    #[inline]
    #[must_use]
    pub fn output_denominator(&self) -> usize {
        self.output_denominator as usize
    }

    pub fn set_output_denominator(&mut self, output_denominator: u8) {
        self.output_denominator = output_denominator;
    }

    #[must_use]
    pub fn output_count(&self) -> usize {
        usize::from(self.output_count)
    }

    #[must_use]
    pub fn motor_count(&self) -> usize {
        usize::from(self.motor_count)
    }

    #[must_use]
    pub fn motors_is_on(&self) -> bool {
        self.motors_is_on
    }

    pub fn motors_switch_off(&mut self) {
        self.motors_is_on = false;
    }

    pub fn motors_switch_on(&mut self) {
        self.motors_is_on = true;
    }

    #[must_use]
    pub fn motors_is_armed(&self) -> bool {
        self.motors_is_armed
    }

    /// Switch off motors and disarm.
    pub fn disarm_motors(&mut self) {
        self.motors_switch_off();
        self.motors_is_armed = false;
    }

    /// Arm motors, ensuring they are switched off first.
    pub fn arm_motors(&mut self) {
        self.motors_switch_off();
        self.motors_is_armed = true;
    }

    #[must_use]
    pub fn throttle_command(&self) -> f32 {
        self.throttle_command
    }

    #[inline]
    pub fn set_throttle_command(&mut self, throttle_command: f32) {
        self.throttle_command = throttle_command;
    }

    #[inline]
    pub fn output_this_cycle(&mut self) -> bool {
        // TODO: check the logic of this
        self.output_count += 1;
        if self.output_count < self.output_denominator {
            return false;
        }
        self.output_count = 0;
        true
    }
}

/// Struct containing array of motor outputs, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorOutputs {
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorOutputs {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputs {
    type Target = [f32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputs {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Array of motor rotation frequencies, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorFrequencies(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorFrequencies {
    #[must_use]
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorFrequencies {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorFrequencies {
    type Target = [f32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorFrequencies {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputFilters(pub [SlewRateLimiterf32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorOutputFilters {
    pub const fn new() -> Self {
        Self([SlewRateLimiterf32::new(); MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorOutputFilters {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputFilters {
    type Target = [SlewRateLimiterf32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputFilters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
#[cfg(test)]
mod tests {
    use crate::MixerType;

    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommon>();
        is_full::<MotorOutputs>();
        is_full::<MotorOutputFilters>();
    }
    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let mixer = MotorMixerCommon::new(mixer_config, motor_config);
        assert_eq!(MixerType::QuadX, mixer.mixer_type);
    }
}
