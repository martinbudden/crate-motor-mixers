#![cfg(feature = "esp32")]

use crate::{
    drivers::output_to_duty,
    mixer_common::{MotorFrequencies, MotorOutputs},
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
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
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
