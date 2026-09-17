use embassy_time::{Duration, Timer};

use crate::{
    MotorCommands, MotorFrequencies, MotorOutputs,
    dshot::{DshotCommand, DshotCommandFrame, DshotError, DshotTelemetryFrame, GcrFrame},
};
#[cfg(rp)]
use {
    crate::{dshot::DshotSpeed, dshot_rp::BidirectionalQuadDshotPio},
    embassy_rp::{
        Peri,
        interrupt::typelevel::Binding,
        peripherals::PIO0,
        pio::{InterruptHandler, PioPin},
    },
};

/// Bidirectional Dshot driver using `PIO` for 4 motors.
/// Currently hardcoded to use `PIO0`.
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
    #[cfg(rp)]
    pio: BidirectionalQuadDshotPio<'static, PIO0>,
    erpm_to_hz: f32,
}

#[allow(unused)]
impl MotorDriverQuadDshot {
    pub const DEFAULT_MOTOR_POLE_COUNT: u16 = 14;
    const SECONDS_PER_MINUTE: f32 = 60.0;

    #[cfg(rp)]
    #[allow(clippy::too_many_arguments)]
    #[must_use]
    pub fn new(
        pio: Peri<'static, PIO0>,
        irq: impl Binding<<PIO0 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO0>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        protocol: DshotSpeed,
        motor_pole_count: u16,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            pio: BidirectionalQuadDshotPio::new(pio, irq, pin0, pin1, pin2, pin3, protocol),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / f32::from(motor_pole_count),
        }
    }
}

impl MotorDriverQuadDshot {
    #[inline]
    pub async fn send_frame(&mut self, frame: DshotCommandFrame, index: usize) {
        #[cfg(rp)]
        self.pio.send_frame(frame, index).await;
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = index;
        }
    }

    /// # Errors
    #[inline]
    pub async fn send_frame_and_receive_gcr21(
        &mut self,
        frame: DshotCommandFrame,
        index: usize,
    ) -> Result<GcrFrame, DshotError> {
        #[cfg(rp)]
        {
            let gcr_frame = self.pio.send_frame_and_receive_gcr20(frame, index).await?;
            Ok(gcr_frame)
        }
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = index;
            let gcr_frame = GcrFrame::default();
            Ok(gcr_frame)
        }
    }

    /// # Errors
    #[inline]
    pub async fn write_to_motor(
        &mut self,
        frame: DshotCommandFrame,
        index: usize,
    ) -> Result<DshotTelemetryFrame, DshotError> {
        let gcr_frame = self.send_frame_and_receive_gcr21(frame, index).await?;
        let erpm_frame = gcr_frame.try_decode()?;
        Ok(erpm_frame)
    }

    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        for index in 0..4 {
            let frame = DshotCommandFrame::from_throttle(outputs[index]);
            let result = self.write_to_motor(frame, index).await;
            if let Ok(erpm_telemetry_frame) = result {
                self.motor_frequencies[index] = erpm_telemetry_frame.erpm_f32() * self.erpm_to_hz;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        for index in 0..4 {
            let command = commands[index];
            let frame = DshotCommandFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        for index in 0..4 {
            let frame = DshotCommandFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
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
        is_normal::<MotorDriverQuadDshot>();
    }
}
