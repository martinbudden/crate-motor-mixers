use core::ops::{Deref, DerefMut};

use crate::{MixerConfig, MixerType, MotorConfig, MotorFrequencies, MotorMixerMessage, MotorMixerParameters};
use signal_filters::SlewRateLimiterf32;

pub const MAX_MOTOR_COUNT: usize = 8;

/// Array of motor rotation frequencies, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs(pub [f32; MAX_MOTOR_COUNT]);

impl MotorOutputs {
    pub const fn new() -> Self {
        Self([0.0; MAX_MOTOR_COUNT])
    }
}

impl Default for MotorOutputs {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputs {
    type Target = [f32; MAX_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputs {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

//pub type MotorOutputFilters = [SlewRateLimiterf32; MAX_MOTOR_COUNT];
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputFilters(pub [SlewRateLimiterf32; MAX_MOTOR_COUNT]);

impl MotorOutputFilters {
    pub const fn new() -> Self {
        Self([SlewRateLimiterf32::new(); MAX_MOTOR_COUNT])
    }
}

impl Default for MotorOutputFilters {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputFilters {
    type Target = [SlewRateLimiterf32; MAX_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputFilters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Common properties of all motor mixers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommon {
    pub outputs: MotorOutputs,
    pub output_filters: MotorOutputFilters,
    mixer_type: u8,
    output_denominator: u8,
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
}

impl MotorMixerCommon {
    #[must_use]
    pub const fn with_config(mixer_config: MixerConfig, motor_config: MotorConfig) -> Self {
        Self {
            outputs: MotorOutputs::new(),
            output_filters: MotorOutputFilters::new(),
            mixer_type: MixerType::QuadX as u8,
            output_denominator: 1,
            output_count: 0,
            mixer_config,
            motor_config,
            mixer_parameters: MotorMixerParameters::new(),
            throttle_command: 0.0, // used for blackbox recording
            motors_is_on: false,
            motors_is_armed: false,
            motors_is_reversed: false, //reversed motors typically used to flip multi-rotor after a crash
        }
    }

    #[must_use]
    pub const fn new() -> Self {
        Self::with_config(MixerConfig::new(), MotorConfig::new())
    }
}

impl Default for MotorMixerCommon {
    fn default() -> Self {
        Self::new()
    }
}

pub trait MotorMixerOutput {
    fn output_to_motors(&mut self, motor_mixer_message: MotorMixerMessage);
}

pub trait MotorMixer {
    fn common(&self) -> &MotorMixerCommon;
    fn common_mut(&mut self) -> &mut MotorMixerCommon;

    #[inline]
    fn output_denominator(&self) -> usize {
        self.common().output_denominator as usize
    }

    fn motors_is_on(&self) -> bool {
        self.common().motors_is_on
    }
    fn motors_switch_off(&mut self) {
        self.common_mut().motors_is_on = false;
    }
    fn motors_switch_on(&mut self) {
        self.common_mut().motors_is_on = true;
    }
    fn motors_is_armed(&self) -> bool {
        self.common().motors_is_armed
    }
    /// Switch off motors and disarm.
    fn disarm_motors(&mut self) {
        self.motors_switch_off();
        self.common_mut().motors_is_armed = false;
    }
    /// Arm motors, ensuring they are switched off first.
    fn arm_motors(&mut self) {
        self.motors_switch_off();
        self.common_mut().motors_is_armed = true;
    }
    fn throttle_command(&self) -> f32 {
        self.common().throttle_command
    }
    #[inline]
    fn set_throttle_command(&mut self, throttle_command: f32) {
        self.common_mut().throttle_command = throttle_command;
    }
    #[inline]
    fn output_this_cycle(&mut self) -> bool {
        // TODO: check the logic of this
        self.common_mut().output_count += 1;
        if self.common().output_count < self.common().output_denominator {
            return false;
        }
        self.common_mut().output_count = 0;
        true
    }
}

impl MotorMixer for MotorMixerCommon {
    #[inline]
    fn common(&self) -> &MotorMixerCommon {
        self
    }
    #[inline]
    fn common_mut(&mut self) -> &mut MotorMixerCommon {
        self
    }
}
pub trait MotorMixerDriver {
    fn write_to_motors(&mut self, motor_outputs: MotorOutputs);
    fn read_motor_frequencies_hz(&mut self) -> MotorFrequencies;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommon>();
    }
    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let mixer = MotorMixerCommon::with_config(mixer_config, motor_config);
        assert_eq!(MixerType::QuadX as u8, mixer.mixer_type);
    }
}
