#![cfg(feature = "stm32")]
#![allow(unused)]

use embassy_stm32::timer::{
    GeneralInstance4Channel,
    simple_pwm::{SimplePwm, SimplePwmChannel},
};

use crate::{
    drivers::output_to_duty,
    mixer_common::{MotorFrequencies, MotorOutputs},
};

// TODO: sort out MotorDriverQuadPwmGeneral for stm32 variant
#[cfg(feature = "motors_t8")]
pub type MotorDriverQuadPwm = MotorDriverQuadPwmGeneral<embassy_stm32::peripherals::TIM8>;

#[cfg(feature = "motors_t3_t2")]
pub type MotorDriverQuadPwm =
    MotorDriverQuadPwmGeneral2<embassy_stm32::peripherals::TIM3, embassy_stm32::peripherals::TIM2>;

#[cfg(feature = "motors_t4_t3")]
pub type MotorDriverQuadPwm =
    MotorDriverQuadPwmGeneral2<embassy_stm32::peripherals::TIM4, embassy_stm32::peripherals::TIM3>;

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadPwmGeneral<T>
where
    T: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T>,
    ch1: SimplePwmChannel<'static, T>,
    ch2: SimplePwmChannel<'static, T>,
    ch3: SimplePwmChannel<'static, T>,
}

impl<T> MotorDriverQuadPwmGeneral<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm1: SimplePwm<'static, T>) -> Self {
        let channels = pwm1.split();

        Self { ch0: channels.ch1, ch1: channels.ch2, ch2: channels.ch3, ch3: channels.ch4 }
    }

    #[inline]
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
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

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadPwmGeneral2<T1, T2>
where
    T1: GeneralInstance4Channel,
    T2: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T1>,
    ch1: SimplePwmChannel<'static, T1>,
    ch2: SimplePwmChannel<'static, T2>,
    ch3: SimplePwmChannel<'static, T2>,
}

impl<T1, T2> MotorDriverQuadPwmGeneral2<T1, T2>
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
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
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

/*
let p = embassy_stm32::init(Default::default());
let ch1 = PwmPin::new_ch1(p.PA8); // TIM1_CH1
let ch2 = PwmPin::new_ch2(p.PA9);
let pwm = SimplePwm::new(p.TIM1, Some(ch1), Some(ch2), None, None, khz(1));
let mut driver = MotorDriverQuadPwm::new(pwm);
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

    #[allow(clippy::unnecessary_wraps)]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
        Some(self.motor_frequencies)
    }
}
