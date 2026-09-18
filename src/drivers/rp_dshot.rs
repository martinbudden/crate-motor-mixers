use dshot_codec::{DshotCommand, DshotSpeed};

use super::rp_dshot_pio::MotorDriverQuadDshotPio1;
use crate::{MotorCommands, MotorFrequencies, MotorOutputs};

#[cfg(rp)]
use embassy_rp::{
    Peri,
    interrupt::typelevel::Binding,
    peripherals::PIO1,
    pio::{InterruptHandler, PioPin},
};

#[cfg(all(rp, feature = "eight_motors"))]
use embassy_rp::peripherals::PIO1;
/// Bidirectional Dshot driver using `PIO` for 4 motors.
/// Hardcoded to use `PIO1`. PIO0 is reserved for UART and SPI.
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadDshot {
    driver: MotorDriverQuadDshotPio1,
    motor_frequencies: MotorFrequencies,
}

#[allow(unused)]
impl MotorDriverQuadDshot {
    pub const DEFAULT_MOTOR_POLE_COUNT: u16 = 14;
    const SECONDS_PER_MINUTE: f32 = 60.0;

    #[cfg(all(rp, not(feature = "eight_motors")))]
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn new(
        pio: Peri<'static, PIO1>,
        irq: impl Binding<<PIO1 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO1>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        dshot_speed: DshotSpeed,
        motor_pole_count: u16,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            driver: MotorDriverQuadDshotPio1::new(pio, irq, pin0, pin1, pin2, pin3, dshot_speed, motor_pole_count),
        }
    }
}

impl MotorDriverQuadDshot {
    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        #[cfg(rp)]
        self.driver.write_to_motors(outputs).await;
        #[cfg(all(rp, feature = "eight_motors"))]
        driver2.write_to_motors(outputs).await;

        #[cfg(not(rp))]
        core::future::ready(()).await;
    }

    /// # Errors
    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        self.driver.write_commands_to_motors(commands).await;
    }

    /// # Errors
    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        self.driver.write_command_to_all_motors(command).await;
    }

    #[allow(unused, clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        Some(self.motor_frequencies)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_normal<T: Sized + Send + Sync + Unpin>() {}

    #[test]
    fn normal_types() {
        is_normal::<MotorDriverQuadDshot>();
    }
}
