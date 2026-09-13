#![cfg(feature = "esp32")]

use super::{
    drivers::output_to_duty,
    {MotorFrequencies, MotorOutputs},
};
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
        Self { channels: [ch0, ch1, ch2, ch3] }
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

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
}

impl MotorDriverQuadDshot {
    #[must_use]
    pub const fn new() -> Self {
        Self { motor_frequencies: MotorFrequencies::new() }
    }
}

#[allow(clippy::unused_async)]
impl MotorDriverQuadDshot {
    pub async fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }

    pub async fn write_commands_to_motors(&mut self, _commands: MotorCommands) {
        _ = self;
    }
    pub async fn write_command_to_all_motors(&mut self, _command: Command) {
        _ = self;
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
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
