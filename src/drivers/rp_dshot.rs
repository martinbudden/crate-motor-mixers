use embassy_time::{Duration, Timer};

use crate::{
    MAX_SUPPORTED_MOTOR_COUNT, MotorCommands, MotorFrequencies, MotorOutputs,
    dshot::{DshotCommand, DshotCommandFrame, DshotError, DshotTelemetryFrame, GcrFrame},
};

#[cfg(all(rp, feature = "eight_motors"))]
use embassy_rp::peripherals::PIO1;
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
    #[cfg(all(rp, feature = "eight_motors"))]
    sm4: BidirectionalDshotSm<'static, PIO1, 0>,
    #[cfg(all(rp, feature = "eight_motors"))]
    sm5: BidirectionalDshotSm<'static, PIO1, 1>,
    #[cfg(all(rp, feature = "eight_motors"))]
    sm6: BidirectionalDshotSm<'static, PIO1, 2>,
    #[cfg(all(rp, feature = "eight_motors"))]
    sm7: BidirectionalDshotSm<'static, PIO1, 3>,
    erpm_to_hz: f32,
}

#[allow(unused)]
impl MotorDriverQuadDshot {
    pub const DEFAULT_MOTOR_POLE_COUNT: u16 = 14;
    const SECONDS_PER_MINUTE: f32 = 60.0;

    #[cfg(all(rp, feature = "eight_motors"))]
    #[allow(clippy::too_many_arguments, clippy::similar_names)]
    #[must_use]
    pub fn new(
        pio0: Peri<'static, PIO0>,
        pio1: Peri<'static, PIO1>,
        irq0: impl Binding<<PIO0 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO0>>,
        irq1: impl Binding<<PIO1 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO1>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        pin4: Peri<'static, impl PioPin + 'static>,
        pin5: Peri<'static, impl PioPin + 'static>,
        pin6: Peri<'static, impl PioPin + 'static>,
        pin7: Peri<'static, impl PioPin + 'static>,
        dshot_speed: DshotSpeed,
        motor_pole_count: u16,
    ) -> Self {
        use embassy_rp::pio::Pio;

        let mut pio0 = Pio::new(pio0, irq0);
        let mut pio1 = Pio::new(pio1, irq1);

        Self {
            motor_frequencies: MotorFrequencies::new(),
            sm0: BidirectionalDshotSm::new(pio0.sm0, pin0, &mut pio0.common, dshot_speed),
            sm1: BidirectionalDshotSm::new(pio0.sm1, pin1, &mut pio0.common, dshot_speed),
            sm2: BidirectionalDshotSm::new(pio0.sm2, pin2, &mut pio0.common, dshot_speed),
            sm3: BidirectionalDshotSm::new(pio0.sm3, pin3, &mut pio0.common, dshot_speed),
            sm4: BidirectionalDshotSm::new(pio1.sm0, pin4, &mut pio1.common, dshot_speed),
            sm5: BidirectionalDshotSm::new(pio1.sm1, pin5, &mut pio1.common, dshot_speed),
            sm6: BidirectionalDshotSm::new(pio1.sm2, pin6, &mut pio1.common, dshot_speed),
            sm7: BidirectionalDshotSm::new(pio1.sm3, pin7, &mut pio1.common, dshot_speed),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / f32::from(motor_pole_count),
        }
    }
    #[cfg(all(rp, not(feature = "eight_motors")))]
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
    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        #[cfg(rp)]
        let (res0, res1, res2, res3) = {
            let frame0 = DshotCommandFrame::from_throttle(outputs[0]);
            let frame1 = DshotCommandFrame::from_throttle(outputs[1]);
            let frame2 = DshotCommandFrame::from_throttle(outputs[2]);
            let frame3 = DshotCommandFrame::from_throttle(outputs[3]);

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
        #[cfg(all(rp, feature = "eight_motors"))]
        let (res4, res5, res6, res7) = {
            let frame4 = DshotCommandFrame::from_throttle(outputs[4]);
            let frame5 = DshotCommandFrame::from_throttle(outputs[5]);
            let frame6 = DshotCommandFrame::from_throttle(outputs[6]);
            let frame7 = DshotCommandFrame::from_throttle(outputs[7]);

            // Execute all 4 transactions in parallel across the PIO state machines.
            // The join4 macro awaits until ALL 4 asynchronous futures resolve.
            embassy_futures::join::join4(
                self.sm4.send_frame_and_receive_gcr20(frame4),
                self.sm5.send_frame_and_receive_gcr20(frame5),
                self.sm6.send_frame_and_receive_gcr20(frame6),
                self.sm7.send_frame_and_receive_gcr20(frame7),
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
        #[cfg(all(not(rp), feature = "eight_motors"))]
        let (res4, res5, res6, res7) = {
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
        #[cfg(feature = "eight_motors")]
        let results: [Result<GcrFrame, DshotError>; 8] = [res0, res1, res2, res3, res4, res5, res6, res7];
        #[cfg(not(feature = "eight_motors"))]
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
    pub async fn write_to_motors_unjoined(&mut self, outputs: MotorOutputs) {
        for motor_index in 0..MAX_SUPPORTED_MOTOR_COUNT {
            let frame = DshotCommandFrame::from_throttle(outputs[motor_index]);
            let result = self.write_to_motor(frame, motor_index).await;
            if let Ok(erpm_telemetry_frame) = result {
                self.motor_frequencies[motor_index] = erpm_telemetry_frame.erpm_f32() * self.erpm_to_hz;
            }
        }
    }

    /// Convenience function to write to motor by `motor_index`.
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

    /// Convenience function to send frame by `motor_index`.
    #[inline]
    pub async fn send_frame(&mut self, frame: DshotCommandFrame, motor_index: usize) {
        #[cfg(all(rp, not(feature = "eight_motors")))]
        match motor_index {
            0 => self.sm0.send_frame(frame).await,
            1 => self.sm1.send_frame(frame).await,
            2 => self.sm2.send_frame(frame).await,
            _ => self.sm3.send_frame(frame).await,
        }
        #[cfg(all(rp, feature = "eight_motors"))]
        match motor_index {
            0 => self.sm0.send_frame(frame).await,
            1 => self.sm1.send_frame(frame).await,
            2 => self.sm2.send_frame(frame).await,
            3 => self.sm3.send_frame(frame).await,
            4 => self.sm4.send_frame(frame).await,
            5 => self.sm5.send_frame(frame).await,
            6 => self.sm6.send_frame(frame).await,
            _ => self.sm7.send_frame(frame).await,
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
        #[cfg(all(rp, not(feature = "eight_motors")))]
        {
            let gcr_frame = match motor_index {
                0 => self.sm0.send_frame_and_receive_gcr20(frame).await?,
                1 => self.sm1.send_frame_and_receive_gcr20(frame).await?,
                2 => self.sm2.send_frame_and_receive_gcr20(frame).await?,
                _ => self.sm3.send_frame_and_receive_gcr20(frame).await?,
            };
            Ok(gcr_frame)
        }
        #[cfg(all(rp, feature = "eight_motors"))]
        {
            let gcr_frame = match motor_index {
                0 => self.sm0.send_frame_and_receive_gcr20(frame).await?,
                1 => self.sm1.send_frame_and_receive_gcr20(frame).await?,
                2 => self.sm2.send_frame_and_receive_gcr20(frame).await?,
                3 => self.sm3.send_frame_and_receive_gcr20(frame).await?,
                4 => self.sm4.send_frame_and_receive_gcr20(frame).await?,
                5 => self.sm5.send_frame_and_receive_gcr20(frame).await?,
                6 => self.sm6.send_frame_and_receive_gcr20(frame).await?,
                _ => self.sm7.send_frame_and_receive_gcr20(frame).await?,
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
    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        for motor_index in 0..MAX_SUPPORTED_MOTOR_COUNT {
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
        for motor_index in 0..MAX_SUPPORTED_MOTOR_COUNT {
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
