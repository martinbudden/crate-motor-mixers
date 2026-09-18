#![cfg(feature = "stm32")]

use crate::MotorOutputs;

use embassy_stm32::timer::{
    GeneralInstance4Channel,
    simple_pwm::{SimplePwm, SimplePwmChannel},
};

// TODO: sort out MotorDriverPwmGeneral for stm32 variant
#[cfg(feature = "motors_t3")]
pub type MotorDriverPwm = MotorDriverPwmGeneral<embassy_stm32::peripherals::TIM3>;

#[cfg(feature = "motors_t8")]
pub type MotorDriverPwm = MotorDriverPwmGeneral<embassy_stm32::peripherals::TIM8>;

#[cfg(feature = "motors_t3_t5")]
pub type MotorDriverPwm = MotorDriverPwmGeneral2<embassy_stm32::peripherals::TIM3, embassy_stm32::peripherals::TIM5>;

#[cfg(feature = "motors_t4_t3")]
pub type MotorDriverPwm = MotorDriverPwmGeneral2<embassy_stm32::peripherals::TIM4, embassy_stm32::peripherals::TIM3>;

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverPwmGeneral<T>
where
    T: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T>,
    ch1: SimplePwmChannel<'static, T>,
    ch2: SimplePwmChannel<'static, T>,
    ch3: SimplePwmChannel<'static, T>,
}

#[allow(unused)]
impl<T> MotorDriverPwmGeneral<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm1: SimplePwm<'static, T>) -> Self {
        let channels = pwm1.split();

        Self { ch0: channels.ch1, ch1: channels.ch2, ch2: channels.ch3, ch3: channels.ch4 }
    }

    #[inline]
    pub async fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        core::future::ready(()).await;

        let max_duty = 1000.0_f32;
        self.ch0.set_duty_cycle(output_to_duty(motor_outputs[0], max_duty));
        self.ch1.set_duty_cycle(output_to_duty(motor_outputs[1], max_duty));
        self.ch2.set_duty_cycle(output_to_duty(motor_outputs[2], max_duty));
        self.ch3.set_duty_cycle(output_to_duty(motor_outputs[3], max_duty));

        self.ch0.enable();
        self.ch1.enable();
        self.ch2.enable();
        self.ch3.enable();
    }
}

#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverPwmGeneral2<T1, T2>
where
    T1: GeneralInstance4Channel,
    T2: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T1>,
    ch1: SimplePwmChannel<'static, T1>,
    ch2: SimplePwmChannel<'static, T2>,
    ch3: SimplePwmChannel<'static, T2>,
}

#[allow(unused)]
impl<T1, T2> MotorDriverPwmGeneral2<T1, T2>
where
    T1: GeneralInstance4Channel,
    T2: GeneralInstance4Channel,
{
    pub fn new2(pwm1: SimplePwm<'static, T1>, pwm2: SimplePwm<'static, T2>) -> Self {
        let channels1 = pwm1.split();
        let channels2 = pwm2.split();

        Self { ch0: channels1.ch1, ch1: channels1.ch2, ch2: channels2.ch1, ch3: channels2.ch2 }
    }

    #[inline]
    pub async fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        core::future::ready(()).await;

        let max_duty = 1000.0_f32;
        self.ch0.set_duty_cycle(output_to_duty(motor_outputs[0], max_duty));
        self.ch1.set_duty_cycle(output_to_duty(motor_outputs[1], max_duty));
        self.ch2.set_duty_cycle(output_to_duty(motor_outputs[2], max_duty));
        self.ch3.set_duty_cycle(output_to_duty(motor_outputs[3], max_duty));

        self.ch0.enable();
        self.ch1.enable();
        self.ch2.enable();
        self.ch3.enable();
    }
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation, unused)]
#[inline]
fn output_to_duty(output: f32, max_duty: f32) -> u32 {
    let output = output.clamp(-1.0, 1.0);

    // -1.0 → 1000 µs
    //  0.0 → 1500 µs
    // +1.0 → 2000 µs
    let pulse_width_us = 1500.0 + output * 500.0;

    // 50 Hz → 20,000 µs period.
    (pulse_width_us / 20_000.0 * max_duty) as u32
}

/*
let p = embassy_stm32::init(Default::default());
let ch1 = PwmPin::new_ch1(p.PA8); // TIM1_CH1
let ch2 = PwmPin::new_ch2(p.PA9);
let pwm = SimplePwm::new(p.TIM1, Some(ch1), Some(ch2), None, None, khz(1));
let mut driver = MotorDriverPwm::new(pwm);
*/

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorDriverPwm>();
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_to_duty() {
        assert_eq!(1000, output_to_duty(-1.0, 20_000.0));
        assert_eq!(1500, output_to_duty(0.0, 20_000.0));
        assert_eq!(2000, output_to_duty(1.0, 20_000.0));
    }
}
