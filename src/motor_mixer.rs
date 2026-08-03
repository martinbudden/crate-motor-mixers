use crate::{
    MixerConfig, MixerType, MotorConfig, MotorFrequencies, MotorMixerCommands, MotorMixerMessage, MotorProtocol,
    mixer_common::{MotorMixerCommon, MotorOutputs},
    motor_drivers_quad_dshot::MotorDriverQuadDshot,
    motor_drivers_quad_pwm::MotorDriverQuadPwm,
};

#[derive(Clone, Copy, Debug, PartialEq)]
enum MotorDriver {
    QuadPwm(MotorDriverQuadPwm),
    QuadDshot(MotorDriverQuadDshot),
}

impl MotorDriver {
    fn write_to_motors(&mut self, outputs: MotorOutputs) {
        match self {
            MotorDriver::QuadPwm(driver) => driver.write_to_motors(outputs),
            MotorDriver::QuadDshot(driver) => driver.write_to_motors(outputs),
        }
    }

    fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        match self {
            MotorDriver::QuadPwm(_) => None,
            MotorDriver::QuadDshot(driver) => driver.motor_frequencies(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixer {
    common: MotorMixerCommon,
    driver: MotorDriver,
}

impl MotorMixer {
    #[must_use]
    pub const fn new(mixer_config: MixerConfig, motor_config: MotorConfig) -> Self {
        let driver = match motor_config.device.motor_protocol {
            MotorProtocol::Dshot150 | MotorProtocol::Dshot300 | MotorProtocol::Dshot600 => {
                MotorDriver::QuadDshot(MotorDriverQuadDshot::new())
            }
            _ => MotorDriver::QuadPwm(MotorDriverQuadPwm::new()),
        };
        Self { common: MotorMixerCommon::new(mixer_config, motor_config), driver }
    }
}

impl MotorMixer {
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        self.driver.motor_frequencies()
    }

    /// Calculate and output motor mix.
    /// It is typically called at frequency of between 500Hz and 1000Hz.
    pub fn output_to_motors(&mut self, commands_dps: MotorMixerMessage) {
        const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;

        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution
        if !self.common.motors_is_on() || !self.common.motors_is_armed() {
            self.common.outputs = MotorOutputs::default();
            self.driver.write_to_motors(self.common.outputs);
            return;
        }
        let commands = MotorMixerCommands {
            throttle: commands_dps.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
        };
        self.common.set_throttle_command(self.common.mix_params.throttle);
        match self.common.mixer_type {
            MixerType::Tricopter => {
                let outputs = &crate::mix_tricopter(commands, self.common.range, &mut self.common.mix_params);
                for (ii, output) in outputs.iter().enumerate().take(self.common.output_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }

            MixerType::Bicopter => {
                let outputs = &crate::mix_bicopter(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.common.output_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }
            MixerType::FlyingWingSinglePropeller => {
                let outputs = &crate::mix_wing(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.common.output_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }
            #[cfg(feature = "eight_motors")]
            MixerType::HexX => {
                let outputs = &crate::mix_tricopter(commands, self.common.range, &mut self.common.mix_params);
                for (ii, output) in outputs.iter().enumerate().take(self.common.output_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }
            MixerType::AirplaneSinglePropeller => {
                let outputs = &crate::mix_airplane(commands);
                for (ii, output) in outputs.iter().enumerate().take(self.common.output_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }

            _ => {
                let outputs = &crate::mix_quad_x(commands, self.common.range, &mut self.common.mix_params);
                for (ii, output) in outputs.iter().enumerate().take(self.common.motor_count()) {
                    self.common.outputs[ii] = self.common.output_filters[ii].update(*output);
                }
            }
        }

        self.driver.write_to_motors(self.common.outputs);
    }
}

#[cfg(test)]
mod tests {
    use crate::{MixerConfig, MotorConfig};

    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommon>();
    }
    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let _motor_mixer_common = MotorMixerCommon::new(mixer_config, motor_config);
    }
}
