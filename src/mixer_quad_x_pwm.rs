use crate::{
    MotorFrequencies, MotorMixer, MotorMixerCommands, MotorMixerCommon, MotorMixerDriver, MotorMixerMessage,
    MotorMixerOutput, MotorMixerParameters, mix_quad_x, mixer::MotorOutputs,
};

impl MotorMixerDriver for MotorMixerQuadXPwm {
    fn write_to_motors(&mut self, _motor_outputs: MotorOutputs) {
        // TODO: implement write_to_motor for MotorMixerQuadXPwm
    }
    // Null implementation for PWM, since we cannot obtain the motor frequencies.
    fn read_motor_frequencies_hz(&mut self) -> MotorFrequencies {
        MotorFrequencies::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerQuadXPwm {
    common: MotorMixerCommon,
    max_duty: u32,
}

impl Default for MotorMixerQuadXPwm {
    fn default() -> Self {
        Self::new(MotorMixerCommon::new())
    }
}

impl MotorMixerQuadXPwm {
    const MOTOR_COUNT: usize = 4;

    #[must_use]
    pub const fn new(common: MotorMixerCommon) -> Self {
        Self {
            common, // more idiomatic than calling new
            max_duty: 255,
        }
    }
}

impl MotorMixerOutput for MotorMixerQuadXPwm {
    // Calculate and output motor mix.
    // Called by the scheduler when the updateOutputsUsingPIDs function running in the AHRS task SIGNALs that output data is available.
    // It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
    fn output_to_motors(&mut self, commands_dps: MotorMixerMessage) {
        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution
        if !self.common.motors_is_on() || !self.common.motors_is_armed() {
            self.common.outputs = MotorOutputs::default();
            self.write_to_motors(self.common.outputs);
            return;
        }
        if self.common.output_this_cycle() {
            const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;
            let mut mix_params = MotorMixerParameters::default();
            let commands = MotorMixerCommands {
                throttle: commands_dps.throttle,
                // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
                roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
                pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
                yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
            };
            self.common.set_throttle_command(mix_params.throttle);

            self.common.outputs[..Self::MOTOR_COUNT].copy_from_slice(&mix_quad_x(commands, &mut mix_params));
            for ii in 0..Self::MOTOR_COUNT {
                self.common.outputs[ii] = self.common.output_filters[ii].update(self.common.outputs[ii]);
            }

            self.write_to_motors(self.common.outputs);
        }
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
        is_full::<MotorMixerQuadXPwm>();
    }
    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let motor_mixer_common = MotorMixerCommon::with_config(mixer_config, motor_config);
        let motor_mixer_quad_x_pwm = MotorMixerQuadXPwm::new(motor_mixer_common);
        assert_eq!(255, motor_mixer_quad_x_pwm.max_duty);
    }
}
