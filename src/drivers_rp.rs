#![cfg(any(feature = "rp2350", feature = "rp2040"))]

use crate::{
    drivers::output_to_duty,
    mixer_common::{MotorFrequencies, MotorOutputs},
};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};

//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadPwm {
    pwm0: Pwm<'static>,
    pwm1: Pwm<'static>,
    config0: PwmConfig,
    config1: PwmConfig,
    _top: f32,
}

impl MotorDriverQuadPwm {
    #[must_use]
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>) -> Self {
        let config0 = PwmConfig::default();
        let config1 = PwmConfig::default();
        let _top = f32::from(config0.top);

        Self { pwm0, pwm1, config0, config1, _top }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    #[inline]
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        let max_duty = 1000.0_f32;
        self.config0.compare_a = output_to_duty(motor_outputs[0], max_duty) as u16;
        self.config0.compare_b = output_to_duty(motor_outputs[1], max_duty) as u16;
        self.config1.compare_a = output_to_duty(motor_outputs[2], max_duty) as u16;
        self.config1.compare_b = output_to_duty(motor_outputs[3], max_duty) as u16;

        self.pwm0.set_config(&self.config0);
        self.pwm1.set_config(&self.config1);
    }
}

/*
let pwm0 = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, Config::default());
let pwm1 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, Config::default());
*/

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
}

impl MotorDriverQuadDshot {
    pub const fn new() -> Self {
        Self { motor_frequencies: MotorFrequencies::new() }
    }
}

impl MotorDriverQuadDshot {
    pub fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }

    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
        Some(self.motor_frequencies)
    }
}
