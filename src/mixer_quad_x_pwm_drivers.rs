#![allow(unused)]

use cfg_if::cfg_if;

cfg_if! {
if #[cfg(feature = "rp2040")] {
use embassy_rp::pwm::{Config, Pwm};
use crate::mixer::MotorOutputs;
//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

pub struct MotorDriver {
    pwm0: Pwm<'static>,
    pwm1: Pwm<'static>,
    config0: Config,
    config1: Config,
    top: f32,
}

impl MotorDriver {
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>) -> Self {
        let config0 = Config::default();
        let config1 = Config::default();
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
    pub fn write_motors(&mut self, motor_outputs: MotorOutputs) {
        self.config0.compare_a = (((1.0 + motor_outputs[0]) / 20.0) * self.top) as u16;
        self.config0.compare_b = (((1.0 + motor_outputs[1]) / 20.0) * self.top) as u16;
        self.config1.compare_a = (((1.0 + motor_outputs[2]) / 20.0) * self.top) as u16;
        self.config1.compare_b = (((1.0 + motor_outputs[3]) / 20.0) * self.top) as u16;

        self.pwm0.set_config(&self.config0);
        self.pwm1.set_config(&self.config1);
    }
}

/*
let pwm0 = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, Config::default());
let pwm1 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, Config::default());
*/

} else if #[cfg(feature = "stm32")] {

use embassy_stm32::timer::simple_pwm::{SimplePwm, SimplePwmChannel};
use embassy_stm32::timer::GeneralInstance4Channel;
use crate::mixer::MotorOutputs;

//type PwmType = SimplePwm<'static, embassy_stm32::peripherals::TIM1>;

pub struct MotorDriver<T>
where
    T: GeneralInstance4Channel,
{
    ch0: SimplePwmChannel<'static, T>,
    ch1: SimplePwmChannel<'static, T>,
    ch2: SimplePwmChannel<'static, T>,
    ch3: SimplePwmChannel<'static, T>,
}

impl<T> MotorDriver<T>
where
    T: GeneralInstance4Channel,
{
    pub fn new(pwm: SimplePwm<'static, T>) -> Self {
        let channels = pwm.split();
        Self {
            ch0: channels.ch1,
            ch1: channels.ch2,
            ch2: channels.ch3,
            ch3: channels.ch4,
        }
    }

    #[allow(clippy::cast_sign_loss,clippy::cast_possible_truncation)]
    #[inline]
    pub fn write_motors(&mut self, motor_outputs: MotorOutputs) {
        self.ch0.set_duty_cycle(((1.0 + motor_outputs[0]) * 1000.0 / 20.0) as u32);
        self.ch0.enable();
        self.ch1.set_duty_cycle(((1.0 + motor_outputs[1]) * 1000.0 / 20.0) as u32);
        self.ch1.enable();
        self.ch2.set_duty_cycle(((1.0 + motor_outputs[2]) * 1000.0 / 20.0) as u32);
        self.ch2.enable();
        self.ch3.set_duty_cycle(((1.0 + motor_outputs[3]) * 1000.0 / 20.0) as u32);
        self.ch3.enable();
    }
}
/*
let p = embassy_stm32::init(Default::default());
let ch1 = PwmPin::new_ch1(p.PA8); // TIM1_CH1
let ch2 = PwmPin::new_ch2(p.PA9);
let pwm = SimplePwm::new(p.TIM1, Some(ch1), Some(ch2), None, None, khz(1));
let mut driver = MotorDriver::new(pwm);
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
use crate::mixer::MotorOutputs;

pub struct MotorDriver {
    channels: [LedcDriver<'static>; 4],
}

impl MotorDriver {
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
    pub fn write_motor(&mut self, motor_index: u8, motor_output: f32) {
        // Convert [0.0, 1.0] to pulse width (1-2ms for ESCs)
        let pulse_ms = 1.0 + motor_output;
        // Scale to duty cycle based on timer resolution
        let max_duty = self.driver.get_max_duty();
        let duty = ((pulse_ms / 20.0) * max_duty as f32) as u32;

        // Set duty for the appropriate channel
        match motor_index {
            0 => self.driver.set_duty(Channel::CH0, duty).unwrap(),
            1 => self.driver.set_duty(Channel::CH1, duty).unwrap(),
            2 => self.driver.set_duty(Channel::CH2, duty).unwrap(),
            3 => self.driver.set_duty(Channel::CH3, duty).unwrap(),
            _ => return,
        }
        self.driver.update_duty().unwrap();
    }

    #[allow(clippy::cast_sign_loss,clippy::cast_possible_truncation)]
    #[inline]
    pub fn write_motors(&mut self, motor_outputs: MotorOutputs) {
        let max_duty = self.driver.get_max_duty();

        self.driver.set_duty(Channel::CH0, ((1.0 + motor_outputs[0]) * max_duty / 20.0) as u32);
        self.driver.set_duty(Channel::CH1, ((1.0 + motor_outputs[1]) * max_duty / 20.0) as u32);
        self.driver.set_duty(Channel::CH2, ((1.0 + motor_outputs[2]) * max_duty / 20.0) as u32);
        self.driver.set_duty(Channel::CH3, ((1.0 + motor_outputs[3]) * max_duty / 20.0) as u32);
    }
}

}
//#[cfg(feature = "esp32")]
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;

}

#[cfg(test)]
mod tests {
    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {}
}
