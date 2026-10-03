#![cfg(feature = "esp32s3")]

use super::MotorOutputs;

use embedded_hal::pwm::SetDutyCycle;

use esp_hal::ledc::{LowSpeed, channel::Channel};

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverPwm {
    ch0: Channel<'static, LowSpeed>,
    ch1: Channel<'static, LowSpeed>,
    ch2: Channel<'static, LowSpeed>,
    ch3: Channel<'static, LowSpeed>,
    #[allow(unused)]
    frequency_hz: f32,
}

impl MotorDriverPwm {
    #[must_use]
    pub fn new(
        ch0: Channel<'static, LowSpeed>,
        ch1: Channel<'static, LowSpeed>,
        ch2: Channel<'static, LowSpeed>,
        ch3: Channel<'static, LowSpeed>,
        frequency_hz: f32,
    ) -> Self {
        Self { ch0, ch1, ch2, ch3, frequency_hz }
    }

    #[inline]
    pub async fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        core::future::ready(()).await;

        _ = self.ch0.set_duty_cycle(output_to_duty(motor_outputs[0]));
        _ = self.ch1.set_duty_cycle(output_to_duty(motor_outputs[1]));
        _ = self.ch2.set_duty_cycle(output_to_duty(motor_outputs[2]));
        _ = self.ch3.set_duty_cycle(output_to_duty(motor_outputs[3]));
    }
}

#[inline]
fn output_to_duty(output: f32) -> u16 {
    //const PWM_MAX_DUTY: f32 = f32::from(1 << 14) - 1.0;
    const PWM_CENTER_US: f32 = 1_500.0;
    const PWM_RANGE_US: f32 = 500.0;

    let output = output.clamp(-1.0, 1.0);

    // -1.0 → 1000 µs
    //  0.0 → 1500 µs
    // +1.0 → 2000 µs
    let pulse_width_us = PWM_CENTER_US + output * PWM_RANGE_US;

    // 50 Hz → 20,000 µs period.
    //
    // 14-bit LEDC:
    // 0     → 0%
    // 16383 → 100%
    //
    // Therefore:
    // duty = pulse / period * 16383
    (pulse_width_us / 20_000.0 * 16_383.0) as u16
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
