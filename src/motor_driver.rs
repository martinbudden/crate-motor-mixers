use dshot_codec::DshotCommand;
use embassy_time::{Duration, Timer};

use crate::{
    drivers::{MotorDriverDshot, MotorDriverPwm},
    mixers::{DshotCommands, MotorFrequencies, MotorOutputs},
};

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub enum MotorDriver {
    Pwm(MotorDriverPwm),
    Dshot(MotorDriverDshot),
}

impl MotorDriver {
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        match self {
            Self::Pwm(driver) => driver.write_to_motors(outputs).await,
            Self::Dshot(driver) => driver.write_to_motors(outputs).await,
        }
    }

    pub async fn write_commands_to_motors(&mut self, commands: DshotCommands) {
        match self {
            Self::Pwm(_driver) => {}
            Self::Dshot(driver) => driver.write_commands_to_motors(commands).await,
        }
    }

    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        match self {
            Self::Pwm(_driver) => {}
            Self::Dshot(driver) => driver.write_command_to_all_motors(command).await,
        }
    }

    /// Arm all ESCs by sending `MotorStop` at 1kHz for the given duration.
    pub async fn arm_all_motors(&mut self, duration: Duration) {
        match self {
            Self::Pwm(_driver) => {}
            Self::Dshot(driver) => {
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
            Self::Pwm(_) => None,
            Self::Dshot(driver) => driver.motor_frequencies(),
        }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorDriver>();
    }
}
