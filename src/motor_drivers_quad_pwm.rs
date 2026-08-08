#![allow(unused)]

use cfg_if::cfg_if;

use crate::motor_driver::MotorOutputs;

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
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

cfg_if! {
if #[cfg(feature = "rp2040")] {
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

#[allow(missing_debug_implementations,missing_copy_implementations)]
pub struct MotorDriverQuadPwm {
    pwm0: Pwm<'static>,
    pwm1: Pwm<'static>,
    config0: PwmConfig,
    config1: PwmConfig,
    top: f32,
}

impl MotorDriverQuadPwm {
    #[must_use]
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>) -> Self {
        let config0 = PwmConfig::default();
        let config1 = PwmConfig::default();
        let top = f32::from(config0.top);

        Self {
            pwm0,
            pwm1,
            config0,
            config1,
            top,
        }
    }

    #[allow(clippy::cast_sign_loss,clippy::cast_possible_truncation)]
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

} else if #[cfg(feature = "stm32")] {

use embassy_stm32::timer::{
    simple_pwm::{SimplePwm, SimplePwmChannel},
    GeneralInstance4Channel
};

//type PwmType = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;
pub type MotorDriverQuadPwm = MotorDriverQuadPwmGeneral<embassy_stm32::peripherals::TIM4,embassy_stm32::peripherals::TIM3>;

#[allow(missing_debug_implementations,missing_copy_implementations)]
pub struct MotorDriverQuadPwmGeneral<T1, T2>
where
    T1: GeneralInstance4Channel,
    T2: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T1>,
    ch1: SimplePwmChannel<'static, T1>,
    ch2: SimplePwmChannel<'static, T2>,
    ch3: SimplePwmChannel<'static, T2>,
}

impl<T1, T2> MotorDriverQuadPwmGeneral<T1, T2>
where
    T1: GeneralInstance4Channel,
    T2: GeneralInstance4Channel,
{
    pub fn new(
        pwm1: SimplePwm<'static, T1>,
        pwm2: SimplePwm<'static, T2>,
    ) -> Self {
        let channels1 = pwm1.split();
        let channels2 = pwm2.split();

        Self {
            ch0: channels1.ch1,
            ch1: channels1.ch2,
            ch2: channels2.ch3,
            ch3: channels2.ch4,
        }
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

} else if #[cfg(feature = "esp32")] {

/*use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, SpeedMode};
use esp_idf_hal::gpio::PinDriver;

let pin = PinDriver::output(p.GPIO0).unwrap();
let timer = LedcTimerDriver::new(&ledc, SpeedMode::Low, &TimerConfig::default()).unwrap();
let mut channel = LedcDriver::new(&ledc, SpeedMode::Low, &timer, pin).unwrap();

channel.set_duty(1023).unwrap(); // 10-bit duty   }
*/
//#[cfg(feature = "esp32")]
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;

use esp_idf_hal::ledc::{LedcDriver, LedcTimerDriver, SpeedMode, Channel};

pub struct MotorDriverQuadPwm {
    channels: [LedcDriver<'static>; 4],
}

impl MotorDriverQuadPwm {
    pub fn new(
        ch0: LedcDriver<'static>,
        ch1: LedcDriver<'static>,
        ch2: LedcDriver<'static>,
        ch3: LedcDriver<'static>,
    ) -> Self {
        Self {
            channels: [ch0, ch1, ch2, ch3],
        }
    }

    #[inline]
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        let max_duty = self.driver.get_max_duty() as f32;

        self.driver.set_duty(Channel::CH0, output_to_duty(motor_outputs[0]), max_duty);
        self.driver.set_duty(Channel::CH1, output_to_duty(motor_outputs[1]), max_duty);
        self.driver.set_duty(Channel::CH2, output_to_duty(motor_outputs[2]), max_duty);
        self.driver.set_duty(Channel::CH3, output_to_duty(motor_outputs[3]), max_duty);

        self.driver.update_duty().unwrap();
    }
}

} else {

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverQuadPwm;

impl MotorDriverQuadPwm {
    #[must_use]
    pub const fn new() -> Self {
        Self {}
    }

    pub fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }
}

}
//#[cfg(feature = "esp32")]
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;

}

#[cfg(test)]
mod tests {
    //    #![allow(clippy::float_cmp)]
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {}

    #[test]
    fn test_output_to_duty() {
        assert_eq!(1000, output_to_duty(-1.0, 20_000.0));
        assert_eq!(1500, output_to_duty(0.0, 20_000.0));
        assert_eq!(2000, output_to_duty(1.0, 20_000.0));
    }
}
