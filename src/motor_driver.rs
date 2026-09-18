use dshot_codec::DshotCommand;
use embassy_time::{Duration, Timer};

use super::{MotorCommands, MotorFrequencies, MotorOutputs};

use super::{MotorDriverQuadDshot, MotorDriverQuadPwm};

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub enum MotorDriver {
    QuadPwm(MotorDriverQuadPwm),
    QuadDshot(MotorDriverQuadDshot),
}

impl MotorDriver {
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        match self {
            Self::QuadPwm(driver) => driver.write_to_motors(outputs).await,
            Self::QuadDshot(driver) => driver.write_to_motors(outputs).await,
        }
    }

    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        match self {
            Self::QuadPwm(_driver) => {}
            Self::QuadDshot(driver) => driver.write_commands_to_motors(commands).await,
        }
    }

    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        match self {
            Self::QuadPwm(_driver) => {}
            Self::QuadDshot(driver) => driver.write_command_to_all_motors(command).await,
        }
    }

    /// Arm all ESCs by sending `MotorStop` at 1kHz for the given duration.
    pub async fn arm_all_motors(&mut self, duration: Duration) {
        match self {
            Self::QuadPwm(_driver) => {}
            Self::QuadDshot(driver) => {
                let iterations = duration.as_millis();
                for _ in 0..iterations {
                    driver.write_command_to_all_motors(DshotCommand::MotorStop).await;
                    Timer::after(Duration::from_millis(1)).await;
                }
            }
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

    #[test]
    fn test_output_to_duty() {
        assert_eq!(1000, output_to_duty(-1.0, 20_000.0));
        assert_eq!(1500, output_to_duty(0.0, 20_000.0));
        assert_eq!(2000, output_to_duty(1.0, 20_000.0));
    }
}
