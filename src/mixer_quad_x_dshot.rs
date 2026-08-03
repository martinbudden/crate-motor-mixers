use crate::{
    MotorFrequencies, MotorMixer, MotorMixerCommands, MotorMixerCommon, MotorMixerDriver, MotorMixerMessage,
    MotorMixerOutput, MotorMixerParameters, RpmNotchFilterBank, RpmNotchFilterBankConfig, mix_quad_x,
    mixer::{MAX_SUPPORTED_MOTOR_COUNT, MotorOutputs},
};

impl MotorMixerDriver for MotorMixerQuadXDshot {
    fn write_to_motors(&mut self, _motor_outputs: MotorOutputs) {
        // TODO: implement write_to_motors for MotorMixerQuadXShot
    }
    fn read_motor_frequencies_hz(&mut self) -> MotorFrequencies {
        // TODO: implement read_motor_frequencies_hz for MotorMixerQuadDshot
        MotorFrequencies::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorMixerQuadXDshot {
    common: MotorMixerCommon,
    config: RpmNotchFilterBankConfig,
    rpm_notch_filters: RpmNotchFilterBank,
    motor_frequencies_hz: MotorFrequencies,
    rpm_filter_iteration_count: usize,
}

impl Default for MotorMixerQuadXDshot {
    fn default() -> Self {
        Self::new(
            MotorMixerCommon::new(),
            RpmNotchFilterBankConfig::new(),
            RpmNotchFilterBank::DEFAULT_LOOPTIME_SECONDS,
        )
    }
}

impl MotorMixerQuadXDshot {
    pub const MOTOR_COUNT: usize = 4;

    #[must_use]
    pub fn new(common: MotorMixerCommon, config: RpmNotchFilterBankConfig, looptime_seconds: f32) -> Self {
        let rpm_notch_filters = RpmNotchFilterBank::new(config, looptime_seconds);
        // rpm_filter_harmonics_count calculated in RpmNotchFilterBank::new()
        // We need to complete the rpm_filter iterations before the next time rpm_filter.start() is called.
        // So, for example, if there are 2 harmonics and 4 motors that gives 8 iterations in total.
        // So if output_denominator is 2, then we need to do 4 iterations.
        // If output denominator is 3, then we need to do 3 iterations.
        let rpm_filter_iteration_count =
            (rpm_notch_filters.rpm_filter_harmonics_count() * Self::MOTOR_COUNT).div_ceil(common.output_denominator());
        Self {
            common,
            config,
            rpm_notch_filters,
            motor_frequencies_hz: MotorFrequencies([0.0f32; MAX_SUPPORTED_MOTOR_COUNT]),
            rpm_filter_iteration_count,
        }
    }
}

impl MotorMixerOutput for MotorMixerQuadXDshot {
    // Calculate and output motor mix.
    // Called by the scheduler when the updateOutputsUsingPIDs function running in the AHRS task SIGNALs that output data is available.
    // It is typically called at frequency of between 1000Hz and 8000Hz, so it has to be FAST.
    fn output_to_motors(&mut self, commands_dps: MotorMixerMessage) {
        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution
        if !self.common.motors_is_on() || !self.common.motors_is_armed() {
            self.common.outputs = MotorOutputs::new();
            self.write_to_motors(self.common.outputs);
            return;
        }

        // TODO: read the motor frequencies from the Dshot driver.
        // We retain the values, so they can be displayed on an OSD or recorded in blackbox, if required.
        self.motor_frequencies_hz = self.read_motor_frequencies_hz();
        self.rpm_notch_filters.start_updating_filter_frequencies(self.motor_frequencies_hz);

        if self.common.output_this_cycle() {
            const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;
            let mut mix_params = MotorMixerParameters::new();
            let commands = MotorMixerCommands {
                throttle: commands_dps.throttle,
                // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
                roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
                pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
                yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
            };
            self.common.outputs[..Self::MOTOR_COUNT].copy_from_slice(&mix_quad_x(commands, &mut mix_params));
            self.common.set_throttle_command(mix_params.throttle);

            self.write_to_motors(self.common.outputs);
        }

        for _ in 0..self.rpm_filter_iteration_count {
            self.rpm_notch_filters.update_filter_frequencies_step();
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
        is_full::<MotorMixerQuadXDshot>();
    }
    #[test]
    fn test_new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let motor_mixer_common = MotorMixerCommon::with_config(mixer_config, motor_config);
        let rpm_notch_filter_bank_config = RpmNotchFilterBankConfig::new();
        let looptime_seconds = RpmNotchFilterBank::DEFAULT_LOOPTIME_SECONDS;

        let _motor_mixer_quad_x_pwm =
            MotorMixerQuadXDshot::new(motor_mixer_common, rpm_notch_filter_bank_config, looptime_seconds);
    }
}
