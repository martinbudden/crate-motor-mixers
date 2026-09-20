use core::ops::{Deref, DerefMut};

use crate::{
    MotorMixerCommands,
    mixers::{
        MixerAirplane, MixerBicopter, MixerHexacopter, MixerOctocopter, MixerQuadcopter, MixerTricopter, MixerWing,
    },
};
use dshot_codec::DshotCommand;
use signal_filters::SlewRateLimiterf32;

use super::{MixerConfig, MixerType, MotorConfig};

#[cfg(feature = "eight_motors")]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 8;
#[cfg(not(feature = "eight_motors"))]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mixer {
    Wing(MixerWing),
    Airplane(MixerAirplane),
    Bicopter(MixerBicopter),
    Tricopter(MixerTricopter),
    Quadcopter(MixerQuadcopter),
    Hexacopter(MixerHexacopter),
    Octocopter(MixerOctocopter),
}

/// Common properties of all motor mixers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerCommon {
    pub outputs: MotorOutputs,
    pub output_filters: MotorOutputFilters,
    pub mixer: Mixer,
    pub motor_count: u8,
    pub output_denominator: u8,
    output_count: u8,
    //pub mixer_config: MixerConfig,
    //pub motor_config: MotorConfig,
    /// used for blackbox recording.
    throttle_command: f32,
    motors_is_on: bool,
    motors_is_armed: bool,
    /// reversed motors typically used to flip multi-rotor after a crash.
    motors_is_reversed: bool,
}

impl Default for MotorMixerCommon {
    fn default() -> Self {
        Self::new(MixerConfig::new(), MotorConfig::new())
    }
}

impl MotorMixerCommon {
    /// Constructor.
    #[must_use]
    pub const fn new(mixer_config: MixerConfig, _motor_config: MotorConfig) -> Self {
        let (mixer, motor_count, output_count) = match mixer_config.mixer_type {
            MixerType::FlyingWingSinglePropeller => {
                (Mixer::Wing(MixerWing::new()), MixerWing::MOTOR_COUNT_U8, MixerWing::OUTPUT_COUNT_U8)
            }
            MixerType::AirplaneSinglePropeller => {
                (Mixer::Airplane(MixerAirplane::new()), MixerAirplane::MOTOR_COUNT_U8, MixerAirplane::OUTPUT_COUNT_U8)
            }
            MixerType::Bicopter => {
                (Mixer::Bicopter(MixerBicopter::new()), MixerBicopter::MOTOR_COUNT_U8, MixerBicopter::OUTPUT_COUNT_U8)
            }
            MixerType::Tricopter => (
                Mixer::Tricopter(MixerTricopter::new()),
                MixerTricopter::MOTOR_COUNT_U8,
                MixerTricopter::OUTPUT_COUNT_U8,
            ),
            MixerType::HexX => (
                Mixer::Hexacopter(MixerHexacopter::new()),
                MixerHexacopter::MOTOR_COUNT_U8,
                MixerHexacopter::OUTPUT_COUNT_U8,
            ),
            MixerType::OctoQuadX => (
                Mixer::Octocopter(MixerOctocopter::new()),
                MixerOctocopter::MOTOR_COUNT_U8,
                MixerOctocopter::OUTPUT_COUNT_U8,
            ),
            _ => (
                Mixer::Quadcopter(MixerQuadcopter::new()),
                MixerQuadcopter::MOTOR_COUNT_U8,
                MixerQuadcopter::OUTPUT_COUNT_U8,
            ),
        };
        Self {
            outputs: MotorOutputs::new(),
            output_filters: MotorOutputFilters::new(),
            mixer,
            motor_count,
            output_denominator: 1,
            output_count,
            //mixer_config,
            //motor_config,
            throttle_command: 0.0,
            motors_is_on: false,
            motors_is_armed: false,
            motors_is_reversed: false,
        }
    }
}

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

impl MotorMixerCommon {
    pub fn mix(&mut self, commands: MotorMixerCommands) {
        self.set_throttle_command(commands.throttle);

        match &mut self.mixer {
            Mixer::Airplane(_mixer) => {
                let outputs = MixerAirplane::mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Wing(_mixer) => {
                let outputs = MixerWing::mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Bicopter(_mixer) => {
                let outputs = MixerBicopter::mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Tricopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Quadcopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Hexacopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Octocopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.output_count()) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
        }
    }
}

/// Struct containing array of motor outputs, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl Default for MotorOutputs {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorOutputs {
    #[must_use]
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
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

/// Struct containing array of motor commands, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DshotCommands(pub [DshotCommand; MAX_SUPPORTED_MOTOR_COUNT]);

impl Default for DshotCommands {
    fn default() -> Self {
        Self::new()
    }
}

impl DshotCommands {
    #[must_use]
    pub const fn new() -> Self {
        Self([DshotCommand::MotorStop; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Deref for DshotCommands {
    type Target = [DshotCommand; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for DshotCommands {
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
    #[must_use]
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
/// Parameters to mix function.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorSaturation {
    /// Possibly adjusted throttle value for recording by blackbox.
    pub throttle: f32,
    /// Used by test code.
    pub undershoot: f32,
    /// Used by test code.
    pub overshoot: f32,
}

impl Default for MotorSaturation {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorSaturation {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { throttle: 0.0, undershoot: 0.0, overshoot: 0.0 }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommon>();
        is_full::<MotorOutputs>();
        is_full::<DshotCommands>();
        is_full::<MotorFrequencies>();
        is_full::<MotorOutputFilters>();
        is_full::<MotorSaturation>();
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let mixer = MotorMixerCommon::new(mixer_config, motor_config);
        assert_eq!(1, mixer.output_denominator);
    }
}
