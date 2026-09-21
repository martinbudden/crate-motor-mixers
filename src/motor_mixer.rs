use dshot_codec::DshotCommand;

use crate::{
    MixerConfig, MixerType, MotorConfig, MotorDriver, MotorOutputFilters,
    drivers::{MotorFrequencies, MotorOutputs},
    mixers::{
        MixerAirplane, MixerBicopter, MixerHexacopter, MixerOctocopter, MixerQuadcopter, MixerTricopter, MixerWing,
        MotorMixerCommands, MotorMixerMessage,
    },
};

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

/*
            MotorMixer
                │
                │ MotorOutputs
                ▼
        MotorDriver
        /          \
MotorDriverPwm     MotorDriverDshot
        │                │
        │                ├── Dshot output
        │                └── telemetry
        │
        └── PWM output
*/

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorMixer {
    driver: MotorDriver,
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
    #[allow(unused)]
    motors_is_reversed: bool,
}

impl MotorMixer {
    /// Constructor.
    #[must_use]
    pub const fn new(mixer_config: MixerConfig, _motor_config: MotorConfig, driver: MotorDriver) -> Self {
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
            driver,
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

impl MotorMixer {
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

impl MotorMixer {
    pub fn mix(&mut self, commands: MotorMixerCommands) {
        self.set_throttle_command(commands.throttle);

        match &mut self.mixer {
            Mixer::Airplane(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerAirplane::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Wing(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerWing::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Bicopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerBicopter::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Tricopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerTricopter::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Quadcopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerQuadcopter::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Hexacopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerHexacopter::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
            Mixer::Octocopter(mixer) => {
                let outputs = mixer.mix(commands);
                for (ii, output) in outputs.iter().enumerate().take(MixerOctocopter::OUTPUT_COUNT) {
                    self.outputs[ii] = self.output_filters[ii].update(*output);
                }
            }
        }
    }
}

impl MotorMixer {
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        self.driver.motor_frequencies()
    }

    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        self.driver.write_command_to_all_motors(command).await;
    }

    /// Calculate motor mix and output to motors.
    /// It is typically called at frequency of between 500Hz and 1000Hz.
    pub async fn output_to_motors(&mut self, commands: MotorMixerMessage) {
        const DPS_TO_SIGNED_UNIT_INTERVAL: f32 = 0.001;

        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution.
        if !self.motors_is_on() || !self.motors_is_armed() {
            self.outputs = MotorOutputs::default();
            self.driver.write_to_motors(self.outputs).await;
            return;
        }

        let mixer_commands = MotorMixerCommands {
            throttle: (commands.throttle).clamp(0.0, 1.0),
            // scale roll, pitch, and yaw from DPS to the signed unit interval, [-1.0, 1.0].
            roll: (commands.roll_dps * DPS_TO_SIGNED_UNIT_INTERVAL).clamp(-1.0, 1.0),
            pitch: (commands.pitch_dps * DPS_TO_SIGNED_UNIT_INTERVAL).clamp(-1.0, 1.0),
            yaw: (commands.yaw_dps * DPS_TO_SIGNED_UNIT_INTERVAL).clamp(-1.0, 1.0),
        };
        self.mix(mixer_commands);

        self.driver.write_to_motors(self.outputs).await;
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    //fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorMixer>();
    }
}
