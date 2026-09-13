use embassy_time::{Duration, Timer};

use super::{MotorFrequencies, MotorOutputs};
use crate::{
    MotorCommands,
    dshot::{Command, DecodeError, DshotBidirectionalFrame, DshotError, ErpmTelemetryFrame, GcrFrame},
};

#[cfg(feature = "rp")]
use {
    crate::{dshot::Protocol, dshot_rp::PioBidirectionalQuadDshot},
    embassy_rp::{
        Peri,
        interrupt::typelevel::Binding,
        peripherals::PIO0,
        pio::{InterruptHandler, PioPin},
        pwm::{Config as PwmConfig, Pwm},
    },
};

//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

#[cfg(feature = "rp")]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadPwm {
    pwm0: Pwm<'static>,
    pwm1: Pwm<'static>,
    config0: PwmConfig,
    config1: PwmConfig,
    _top: f32,
}

#[cfg(feature = "rp")]
impl MotorDriverQuadPwm {
    #[must_use]
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>) -> Self {
        let config0 = PwmConfig::default();
        let config1 = PwmConfig::default();
        let _top = f32::from(config0.top);

        Self { pwm0, pwm1, config0, config1, _top }
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    #[inline]
    pub fn write_to_motors(&mut self, motor_outputs: MotorOutputs) {
        use super::drivers::output_to_duty;

        let max_duty = 1000.0_f32;
        self.config0.compare_a = output_to_duty(motor_outputs[0], max_duty) as u16;
        self.config0.compare_b = output_to_duty(motor_outputs[1], max_duty) as u16;
        self.config1.compare_a = output_to_duty(motor_outputs[2], max_duty) as u16;
        self.config1.compare_b = output_to_duty(motor_outputs[3], max_duty) as u16;

        self.pwm0.set_config(&self.config0);
        self.pwm1.set_config(&self.config1);
    }
}

/*
let pwm0 = Pwm::new_output_ab(p.PWM_SLICE0, p.PIN_0, p.PIN_1, Config::default());
let pwm1 = Pwm::new_output_ab(p.PWM_SLICE1, p.PIN_2, p.PIN_3, Config::default());
*/

/// Bidirectional Dshot driver using `PIO` for 4 motors.
/// Currently hardcoded to use `PIO0`.
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
    #[cfg(feature = "rp")]
    pio: PioBidirectionalQuadDshot<'static, PIO0>,
    erpm_to_hz: f32,
}

#[allow(unused)]
impl MotorDriverQuadDshot {
    pub const DEFAULT_MOTOR_POLE_COUNT: u16 = 14;
    const SECONDS_PER_MINUTE: f32 = 60.0;

    #[cfg(feature = "rp")]
    #[must_use]
    pub fn new(
        pio: Peri<'static, PIO0>,
        irq: impl Binding<<PIO0 as embassy_rp::pio::Instance>::Interrupt, InterruptHandler<PIO0>>,
        pin0: Peri<'static, impl PioPin + 'static>,
        pin1: Peri<'static, impl PioPin + 'static>,
        pin2: Peri<'static, impl PioPin + 'static>,
        pin3: Peri<'static, impl PioPin + 'static>,
        protocol: Protocol,
        motor_pole_count: u16,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            pio: PioBidirectionalQuadDshot::new(pio, irq, pin0, pin1, pin2, pin3, protocol),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / (motor_pole_count as f32),
        }
    }
}

impl MotorDriverQuadDshot {
    #[allow(unused)]
    fn decode_gcr_result(&self, result: Result<GcrFrame, DshotError>) -> Result<f32, DecodeError> {
        let gcr_frame = result.map_err(|_| DecodeError::GcrData)?;
        let erpm_raw = gcr_frame.decode()?;
        let erpm_telemetry_frame = ErpmTelemetryFrame::from_raw(erpm_raw);
        if erpm_telemetry_frame.checksum_is_ok() {
            #[allow(clippy::cast_precision_loss)]
            Ok((erpm_telemetry_frame.erpm() as f32) * self.erpm_to_hz)
        } else {
            Err(DecodeError::InvalidChecksum)
        }
    }

    async fn pio_send_frame_and_receive_gcr(
        &mut self,
        frame: DshotBidirectionalFrame,
        index: usize,
    ) -> Result<GcrFrame, DshotError> {
        #[cfg(feature = "rp")]
        match index {
            1 => self.pio.send_frame_and_receive_gcr_sm1(frame).await,
            2 => self.pio.send_frame_and_receive_gcr_sm2(frame).await,
            3 => self.pio.send_frame_and_receive_gcr_sm3(frame).await,
            _ => self.pio.send_frame_and_receive_gcr_sm0(frame).await,
        }
        #[cfg(not(feature = "rp"))]
        {
            core::future::ready(()).await;
            _ = index;
            Ok(GcrFrame::from_raw(u32::from(frame.raw())))
        }
    }

    async fn pio_send_frame(&mut self, frame: DshotBidirectionalFrame, index: usize) {
        #[cfg(feature = "rp")]
        match index {
            1 => self.pio.send_frame_sm1(frame).await,
            2 => self.pio.send_frame_sm2(frame).await,
            3 => self.pio.send_frame_sm3(frame).await,
            _ => self.pio.send_frame_sm0(frame).await,
        }
        #[cfg(not(feature = "rp"))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = index;
        }
    }

    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        for index in 0..4 {
            let frame = DshotBidirectionalFrame::throttle_to_frame(outputs[index]);
            let gcr_result = self.pio_send_frame_and_receive_gcr(frame, index).await;
            if let Ok(frequency) = self.decode_gcr_result(gcr_result) {
                self.motor_frequencies[index] = frequency;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_commands_to_motors(&mut self, commands: MotorCommands) {
        for index in 0..4 {
            let command = commands[index];
            let frame = DshotBidirectionalFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.pio_send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: Command) {
        for index in 0..4 {
            let frame = DshotBidirectionalFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.pio_send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
        }
    }

    #[allow(unused, clippy::unnecessary_wraps)]
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
        #[cfg(feature = "rp")]
        is_normal::<MotorDriverQuadPwm>();
        is_normal::<MotorDriverQuadDshot>();
    }
}
