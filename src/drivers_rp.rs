#![cfg(feature = "rp")]

use super::{
    drivers::output_to_duty,
    mixer_common::{MotorFrequencies, MotorOutputs},
};

use crate::dshot::{DshotEncoder, Protocol};
use crate::dshot_rp::PioBidirectionalQuadDshot;

use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_rp::{
    Peri,
    interrupt::typelevel::Binding,
    peripherals::PIO0,
    pio::{InterruptHandler, PioPin},
};

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

/// Dshot driver using PIO for 4 motors.
/// Currently hardcoded to use PIO0.
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
    pio: PioBidirectionalQuadDshot<'static, PIO0>,
    erpm_to_hz: f32,
}

impl MotorDriverQuadDshot {
    pub const DEFAULT_MOTOR_POLE_COUNT: u16 = 14;
    const SECONDS_PER_MINUTE: f32 = 60.0;

    pub fn new(
        pio: Peri<'static, PIO0>,
        irq: impl Binding<<PIO0 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO0>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        protocol: Protocol,
        motor_pole_count: u16,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            pio: PioBidirectionalQuadDshot::new(pio, irq, pin0, pin1, pin2, pin3, protocol),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / (motor_pole_count as f32),
        }
    }
}

impl MotorDriverQuadDshot {
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        // outputs are in the range 0.0 - 1000.0
        let pwm = DshotEncoder::pwm_to_dshot_clamped((outputs[0] + 1000.0) as u16);
        let erpm = self.pio.send_and_receive_raw_sm0(pwm).await;
        let erpm = erpm.unwrap_or_default();
        self.motor_frequencies[0] = (erpm as f32) * self.erpm_to_hz;
    }

    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        Some(self.motor_frequencies)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorDriverQuadPwm>();
    }
}
