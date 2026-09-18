#![cfg(rp)]

use crate::MotorOutputs;

use embassy_rp::pwm::{Pwm, PwmOutput, SetDutyCycle};

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverPwm {
    pwm0_a: PwmOutput<'static>,
    pwm0_b: PwmOutput<'static>,
    pwm1_a: PwmOutput<'static>,
    pwm1_b: PwmOutput<'static>,
    frequency_hz: f32,
}

impl MotorDriverPwm {
    #[allow(clippy::expect_used)]
    #[must_use]
    /// # Panics
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>, frequency_hz: f32) -> Self {
        let (pwm0_a, pwm0_b) = pwm0.split();
        let (pwm1_a, pwm1_b) = pwm1.split();

        let pwm0_a = pwm0_a.expect("PWM A must be configured");
        let pwm0_b = pwm0_b.expect("PWM B must be configured");
        let pwm1_a = pwm1_a.expect("PWM A must be configured");
        let pwm1_b = pwm1_b.expect("PWM B must be configured");

        Self { pwm0_a, pwm0_b, pwm1_a, pwm1_b, frequency_hz }
    }

    #[inline]
    fn output_to_duty(output: f32, top: u16, frequency_hz: f32) -> u16 {
        // Standard 50Hz PWM.
        const PWM_CENTER_US: f32 = 1_500.0;
        const PWM_RANGE_US: f32 = 500.0;

        let output = output.clamp(-1.0, 1.0);
        // -1.0 → 1000 µs
        //  0.0 → 1500 µs
        // +1.0 → 2000 µs
        let pulse_width_us = PWM_CENTER_US + output * PWM_RANGE_US;

        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        {
            (pulse_width_us * frequency_hz / 1_000_000.0 * f32::from(top)) as u16
        }
    }

    #[inline]
    fn set_motor_output(pwm: &mut PwmOutput<'static>, output: f32, top: u16, frequency_hz: f32) {
        let duty = Self::output_to_duty(output, top, frequency_hz);

        #[allow(clippy::expect_used)]
        pwm.set_duty_cycle(duty).expect("motor PWM duty cycle is within configured range");
    }

    #[inline]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        core::future::ready(()).await;

        let top = self.pwm0_a.max_duty_cycle();

        Self::set_motor_output(&mut self.pwm0_a, outputs[0], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm0_b, outputs[1], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm1_a, outputs[2], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm1_b, outputs[3], top, self.frequency_hz);
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorDriverPwm>();
    }
}
