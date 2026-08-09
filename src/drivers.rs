#[cfg(feature = "esp32")]
use crate::drivers_esp32::{MotorDriverQuadDshot, MotorDriverQuadPwm};
#[cfg(feature = "rp2350")]
use crate::drivers_rp::{MotorDriverQuadDshot, MotorDriverQuadPwm};
#[cfg(feature = "std")]
use crate::drivers_std::{MotorDriverQuadDshot, MotorDriverQuadPwm};
#[cfg(feature = "stm32")]
use crate::drivers_stm32::{MotorDriverQuadDshot, MotorDriverQuadPwm};


use crate::{
    mixer_common::{MotorFrequencies, MotorOutputs},
};

#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub enum MotorDriver {
    QuadPwm(MotorDriverQuadPwm),
    QuadDshot(MotorDriverQuadDshot),
}

impl MotorDriver {
    pub fn write_to_motors(&mut self, outputs: MotorOutputs) {
        match self {
            Self::QuadPwm(driver) => driver.write_to_motors(outputs),
            Self::QuadDshot(driver) => driver.write_to_motors(outputs),
        }
    }

    // Returns the motor frequencies (ie revolutions per second) of the motors from the driver.
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        match self {
            Self::QuadPwm(_) => None,
            Self::QuadDshot(driver) => driver.motor_frequencies(),
        }
    }
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation, unused)]
#[inline]
pub fn output_to_duty(output: f32, max_duty: f32) -> u32 {
    let output = output.clamp(-1.0, 1.0);

    // -1.0 → 1000 µs
    //  0.0 → 1500 µs
    // +1.0 → 2000 µs
    let pulse_width_us = 1500.0 + output * 500.0;

    // 50 Hz → 20,000 µs period.
    (pulse_width_us / 20_000.0 * max_duty) as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn _is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
    }
    #[test]
    fn test_output_to_duty() {
        assert_eq!(1000, output_to_duty(-1.0, 20_000.0));
        assert_eq!(1500, output_to_duty(0.0, 20_000.0));
        assert_eq!(2000, output_to_duty(1.0, 20_000.0));
    }
}
