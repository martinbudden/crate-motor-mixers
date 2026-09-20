use super::{
    MixerConfig, MotorConfig, MotorMixerCommands, MotorMixerMessage,
    motor_driver::MotorDriver,
    {MotorFrequencies, MotorMixerCommon, MotorOutputs},
};

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
    common: MotorMixerCommon,
    driver: MotorDriver,
}

impl MotorMixer {
    #[must_use]
    pub const fn new(mixer_config: MixerConfig, motor_config: MotorConfig, driver: MotorDriver) -> Self {
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
    pub async fn output_to_motors(&mut self, commands_dps: MotorMixerMessage) {
        const MIXER_OUTPUT_SCALE_FACTOR: f32 = 1000.0;

        // ALWAYS write 0.0 to the motors if they are not switched on, as a safety precaution
        if !self.common.motors_is_on() || !self.common.motors_is_armed() {
            self.common.outputs = MotorOutputs::default();
            self.driver.write_to_motors(self.common.outputs).await;
            return;
        }
        let commands = MotorMixerCommands {
            throttle: commands_dps.throttle,
            // scale roll, pitch, and yaw from DPS range to [-1.0F, 1.0F]
            roll: commands_dps.roll_dps * MIXER_OUTPUT_SCALE_FACTOR,
            pitch: commands_dps.pitch_dps * MIXER_OUTPUT_SCALE_FACTOR,
            yaw: commands_dps.yaw_dps * MIXER_OUTPUT_SCALE_FACTOR,
        };
        self.common.mix(commands);
        self.driver.write_to_motors(self.common.outputs).await;
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorMixer>();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MixerConfig, MotorConfig};

    #[test]
    fn new() {
        let mixer_config = MixerConfig::new();
        let motor_config = MotorConfig::new();
        let _motor_mixer_common = MotorMixerCommon::new(mixer_config, motor_config);
    }
}
