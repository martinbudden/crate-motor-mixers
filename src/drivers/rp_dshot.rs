use embassy_time::{Duration, Timer};

use crate::{
    MotorCommands, MotorFrequencies, MotorOutputs,
    dshot::{DshotCommand, DshotCommandFrame, DshotError, DshotTelemetryFrame, GcrFrame},
};
#[cfg(rp)]
use {
    crate::{dshot::DshotSpeed, dshot_rp::BidirectionalDshotSm},
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
    sm0: BidirectionalDshotSm<'static, PIO0, 0>,
    #[cfg(rp)]
    sm1: BidirectionalDshotSm<'static, PIO0, 1>,
    #[cfg(rp)]
    sm2: BidirectionalDshotSm<'static, PIO0, 2>,
    #[cfg(rp)]
    sm3: BidirectionalDshotSm<'static, PIO0, 3>,
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
        dshot_speed: DshotSpeed,
        motor_pole_count: u16,
    ) -> Self {
        use embassy_rp::pio::Pio;

        let mut pio = Pio::new(pio, irq);

        Self {
            motor_frequencies: MotorFrequencies::new(),
            sm0: BidirectionalDshotSm::new(pio.sm0, pin0, &mut pio.common, dshot_speed),
            sm1: BidirectionalDshotSm::new(pio.sm1, pin1, &mut pio.common, dshot_speed),
            sm2: BidirectionalDshotSm::new(pio.sm2, pin2, &mut pio.common, dshot_speed),
            sm3: BidirectionalDshotSm::new(pio.sm3, pin3, &mut pio.common, dshot_speed),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / f32::from(motor_pole_count),
        }
    }
}

impl MotorDriverQuadDshot {
    /// Convenience function to send frame by `motor_index`.
    #[inline]
    pub async fn send_frame(&mut self, frame: DshotCommandFrame, motor_index: usize) {
        #[cfg(rp)]
        match motor_index {
            0 => self.sm0.send_frame(frame).await,
            1 => self.sm1.send_frame(frame).await,
            2 => self.sm2.send_frame(frame).await,
            _ => self.sm3.send_frame(frame).await,
        }
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = motor_index;
        }
    }

    /// Convenience function to send frame and receive gcr20 by `motor_index`.
    /// # Errors
    #[inline]
    pub async fn send_frame_and_receive_gcr20(
        &mut self,
        frame: DshotCommandFrame,
        motor_index: usize,
    ) -> Result<GcrFrame, DshotError> {
        #[cfg(rp)]
        {
            let gcr_frame = match motor_index {
                0 => self.sm0.send_frame_and_receive_gcr20(frame).await?,
                1 => self.sm1.send_frame_and_receive_gcr20(frame).await?,
                2 => self.sm2.send_frame_and_receive_gcr20(frame).await?,
                _ => self.sm3.send_frame_and_receive_gcr20(frame).await?,
            };
            Ok(gcr_frame)
        }
        #[cfg(not(rp))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = motor_index;
            let gcr_frame = GcrFrame::default();
            Ok(gcr_frame)
        }
    }

    /// # Errors
    #[inline]
    pub async fn write_to_motor(
        &mut self,
        frame: DshotCommandFrame,
        motor_index: usize,
    ) -> Result<DshotTelemetryFrame, DshotError> {
        let gcr_frame = self.send_frame_and_receive_gcr20(frame, motor_index).await?;
        let erpm_frame = gcr_frame.try_decode()?;
        Ok(erpm_frame)
    }

    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        for motor_index in 0..4 {
            let frame = DshotCommandFrame::from_throttle(outputs[motor_index]);
            let result = self.write_to_motor(frame, motor_index).await;
            if let Ok(erpm_telemetry_frame) = result {
                self.motor_frequencies[motor_index] = erpm_telemetry_frame.erpm_f32() * self.erpm_to_hz;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_to_motors_joined(&mut self, outputs: MotorOutputs) {
        let frame0 = DshotCommandFrame::from_throttle(outputs[0]);
        let frame1 = DshotCommandFrame::from_throttle(outputs[1]);
        let frame2 = DshotCommandFrame::from_throttle(outputs[2]);
        let frame3 = DshotCommandFrame::from_throttle(outputs[3]);

        #[cfg(rp)]
        let (res0, res1, res2, res3) = {
            // Execute all 4 transactions in parallel across the PIO state machines.
            // The join4 macro awaits until ALL 4 asynchronous futures resolve.
            embassy_futures::join::join4(
                self.sm0.send_frame_and_receive_gcr20(frame0),
                self.sm1.send_frame_and_receive_gcr20(frame1),
                self.sm2.send_frame_and_receive_gcr20(frame2),
                self.sm3.send_frame_and_receive_gcr20(frame3),
            )
            .await
        };
        #[cfg(not(rp))]
        let (res0, res1, res2, res3) = {
            (
                Err(DshotError::NotImplemented),
                Err(DshotError::NotImplemented),
                Err(DshotError::NotImplemented),
                Err(DshotError::NotImplemented),
            )
        };
        #[cfg(not(rp))]
        core::future::ready(()).await;

        // Process the individual results sequentially after they complete
        let results: [Result<GcrFrame, DshotError>; 4] = [res0, res1, res2, res3];
        for (motor_index, result) in results.into_iter().enumerate() {
            if let Ok(gcr_frame) = result
                && let Ok(erpm_telemetry_frame) = gcr_frame.try_decode()
            {
                self.motor_frequencies[motor_index] = erpm_telemetry_frame.erpm_f32() * self.erpm_to_hz;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        for motor_index in 0..4 {
            let command = commands[motor_index];
            let frame = DshotCommandFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        for motor_index in 0..4 {
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
