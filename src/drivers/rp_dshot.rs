use dshot_codec::DshotCommand;

use super::{DshotCommands, MotorFrequencies, MotorOutputs};

#[cfg(rp)]
use {
    crate::drivers::rp_dshot_pio::MotorDriverQuadDshotPio1,
    embassy_rp::{
        Peri,
        interrupt::typelevel::Binding,
        peripherals::PIO1,
        pio::{InterruptHandler, PioPin},
    },
    embassy_time::Timer,
};

#[cfg(all(rp, feature = "eight_motors"))]
use embassy_rp::peripherals::PIO1;
/// Bidirectional Dshot driver using `PIO` for 4 motors.
/// Hardcoded to use `PIO1`. PIO0 is reserved for UART and SPI.
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverDshot {
    #[cfg(rp)]
    driver: MotorDriverQuadDshotPio1,
    motor_frequencies: MotorFrequencies,
}

#[allow(unused)]
impl MotorDriverDshot {
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
        dshot_speed: dshot_codec::DshotSpeed,
        motor_pole_count: u8,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            driver: MotorDriverQuadDshotPio1::new(pio, irq, pin0, pin1, pin2, pin3, dshot_speed, motor_pole_count),
        }
    }
    #[cfg(all(rp, feature = "eight_motors"))]
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn new(
        pio_a: Peri<'static, PIO1>,
        irq_a: impl Binding<<PIO1 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO1>>,
        pio_b: Peri<'static, PIO2>,
        irq_b: impl Binding<<PIO2 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO2>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        pin4: Peri<'static, impl PioPin + 'static>,
        pin5: Peri<'static, impl PioPin + 'static>,
        pin6: Peri<'static, impl PioPin + 'static>,
        pin7: Peri<'static, impl PioPin + 'static>,
        dshot_speed: dshot_codec::DshotSpeed,
        motor_pole_count: u8,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            driver: MotorDriverQuadDshotPio1::new(pio_a, irq_a, pin0, pin1, pin2, pin3, dshot_speed, motor_pole_count),
            driver_b: MotorDriverQuadDshotPio1::new(
                pio_b,
                irq_b,
                pin4,
                pin5,
                pin6,
                pin7,
                dshot_speed,
                motor_pole_count,
            ),
        }
    }
}

impl MotorDriverDshot {
    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        #[cfg(rp)]
        self.driver.write_to_motors(outputs).await;
        #[cfg(all(rp, feature = "eight_motors"))]
        self.driver_b.write_to_motors(outputs[4..]).await;

        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = self;
            _ = outputs;
        }
    }

    /// # Errors
    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: DshotCommands) {
        #[cfg(rp)]
        self.driver.write_commands_to_motors(commands).await;
        #[cfg(all(rp, feature = "eight_motors"))]
        self.driver_b.write_commands_to_motors(commands[4..]).await;
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = self;
            _ = commands;
        }
    }

    /// # Errors
    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        #[cfg(rp)]
        for _ in 0..command.repetitions_required() {
            self.driver.write_command_to_all_motors(command).await;
            #[cfg(feature = "eight_motors")]
            self.driver_b.write_command_to_all_motors(command).await;
            Timer::after_micros(u64::from(command.delay_required_us())).await;
        }
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = self;
            _ = command;
        }
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
        is_normal::<MotorDriverDshot>();
    }
}
