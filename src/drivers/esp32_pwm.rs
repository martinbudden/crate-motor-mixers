#![cfg(feature = "esp32")]

use super::MotorOutputs;

use esp_idf_hal::ledc::{Channel, LedcDriver, LedcTimerDriver, SpeedMode};

/*
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver, SpeedMode};
use esp_idf_hal::gpio::PinDriver;

let pin = PinDriver::output(p.GPIO0).unwrap();
let timer = LedcTimerDriver::new(&ledc, SpeedMode::Low, &TimerConfig::default()).unwrap();
let mut channel = LedcDriver::new(&ledc, SpeedMode::Low, &timer, pin).unwrap();

channel.set_duty(1023).unwrap(); // 10-bit duty   }
*/
//type PwmType = SimplePwm<'static, embassy_esp32::peripherals::LED_PWM>;

pub struct MotorDriverPwm {
    channels: [LedcDriver<'static>; 4],
}

impl MotorDriverPwm {
    pub fn new(
        ch0: LedcDriver<'static>,
        ch1: LedcDriver<'static>,
        ch2: LedcDriver<'static>,
        ch3: LedcDriver<'static>,
    ) -> Self {
        Self { channels: [ch0, ch1, ch2, ch3] }
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

    #[inline]
    pub async fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        core::future::ready(()).await;

        let max_duty = self.driver.get_max_duty() as f32;

        self.driver.set_duty(Channel::CH0, output_to_duty(motor_outputs[0]), max_duty);
        self.driver.set_duty(Channel::CH1, output_to_duty(motor_outputs[1]), max_duty);
        self.driver.set_duty(Channel::CH2, output_to_duty(motor_outputs[2]), max_duty);
        self.driver.set_duty(Channel::CH3, output_to_duty(motor_outputs[3]), max_duty);

        self.driver.update_duty().unwrap();
    }
}

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
        assert_eq!(1000, MotorDriverPwm::output_to_duty(-1.0, 20_000.0));
        assert_eq!(1500, MotorDriverPwm::output_to_duty(0.0, 20_000.0));
        assert_eq!(2000, MotorDriverPwm::output_to_duty(1.0, 20_000.0));
    }
}
