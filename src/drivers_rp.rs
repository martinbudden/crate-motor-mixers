use embassy_time::{Duration, Timer};

use super::{MotorFrequencies, MotorOutputs};
use crate::{
    MotorCommands,
    dshot::{Command, DshotBidirectionalFrame, DshotError, ErpmTelemetryFrame, GcrFrame},
};

#[cfg(feature = "rp")]
use {
    crate::{dshot::DshotProtocol, dshot_rp::BidirectionalQuadDshotPio},
    embassy_rp::{
        Peri,
        interrupt::typelevel::Binding,
        peripherals::PIO0,
        pio::{InterruptHandler, PioPin},
        pwm::{Pwm, PwmOutput, SetDutyCycle},
    },
};

//type PwmType = SimplePwm<'static, embassy_rp::peripherals::PWM_SLICE0>;

#[cfg(feature = "rp")]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverQuadPwm {
    pwm0_a: PwmOutput<'static>,
    pwm0_b: PwmOutput<'static>,
    pwm1_a: PwmOutput<'static>,
    pwm1_b: PwmOutput<'static>,
    frequency_hz: f32,
}

#[cfg(feature = "rp")]
impl MotorDriverQuadPwm {
    #[must_use]
    pub fn new(pwm0: Pwm<'static>, pwm1: Pwm<'static>, frequency_hz: f32) -> Self {
        let (pwm0_a, pwm0_b) = pwm0.split();
        let (pwm1_a, pwm1_b) = pwm1.split();

        let pwm0_a = pwm0_a.expect("PWM A must be configured");
        let pwm0_b = pwm0_b.expect("PWM B must be configured");
        let pwm1_a = pwm1_a.expect("PWM A must be configured");
        let pwm1_b = pwm1_b.expect("PWM B must be configured");

        Self { pwm0_a, pwm0_b, pwm1_a, pwm1_b, frequency_hz }
    }
    fn output_to_duty(output: f32, top: u16, frequency_hz: f32) -> u16 {
        // Standard 50Hz PWM.
        const PWM_CENTER_US: f32 = 1_500.0;
        const PWM_RANGE_US: f32 = 500.0;

        let output = output.clamp(-1.0, 1.0);
        // -1.0 → 1000 µs
        //  0.0 → 1500 µs
        // +1.0 → 2000 µs
        let pulse_width_us = PWM_CENTER_US + output * PWM_RANGE_US;

        (pulse_width_us * frequency_hz / 1_000_000.0 * f32::from(top)) as u16
    }

    #[inline]
    fn set_motor_output(pwm: &mut PwmOutput<'static>, output: f32, top: u16, frequency_hz: f32) {
        let duty = Self::output_to_duty(output, top, frequency_hz);

        pwm.set_duty_cycle(duty).expect("motor PWM duty cycle is within configured range");
    }

    #[inline]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        core::future::ready(()).await;

        let top = self.pwm0_a.max_duty_cycle();

        Self::set_motor_output(&mut self.pwm0_a, outputs[0], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm0_b, outputs[1], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm1_a, outputs[2], top, self.frequency_hz);
        Self::set_motor_output(&mut self.pwm1_b, outputs[3], top, self.frequency_hz);
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
    pio: BidirectionalQuadDshotPio<'static, PIO0>,
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
        protocol: DshotProtocol,
        motor_pole_count: u16,
    ) -> Self {
        Self {
            motor_frequencies: MotorFrequencies::new(),
            pio: BidirectionalQuadDshotPio::new(pio, irq, pin0, pin1, pin2, pin3, protocol),
            erpm_to_hz: 2.0 * (100.0 / Self::SECONDS_PER_MINUTE) / (motor_pole_count as f32),
        }
    }
}

impl MotorDriverQuadDshot {
    #[inline]
    pub async fn send_frame(&mut self, frame: DshotBidirectionalFrame, index: usize) {
        #[cfg(feature = "rp")]
        self.pio.send_frame(frame, index).await;
        #[cfg(not(feature = "rp"))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = index;
        }
    }

    #[inline]
    pub async fn send_frame_and_receive_gcr21(
        &mut self,
        frame: DshotBidirectionalFrame,
        index: usize,
    ) -> Result<GcrFrame, DshotError> {
        #[cfg(feature = "rp")]
        {
            let gcr_frame = self.pio.send_frame_and_receive_gcr21(frame, index).await?;
            Ok(gcr_frame)
        }
        #[cfg(not(feature = "rp"))]
        {
            core::future::ready(()).await;
            _ = frame;
            _ = index;
            let gcr_frame = GcrFrame::default();
            Ok(gcr_frame)
        }
    }

    #[inline]
    pub async fn write_to_motor(
        &mut self,
        frame: DshotBidirectionalFrame,
        index: usize,
    ) -> Result<ErpmTelemetryFrame, DshotError> {
        let gcr_frame = self.send_frame_and_receive_gcr21(frame, index).await?;
        let erpm_frame = gcr_frame.try_decode()?;
        Ok(erpm_frame)
    }

    #[allow(unused)]
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        for index in 0..4 {
            let frame = DshotBidirectionalFrame::from_throttle(outputs[index]);
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
            let frame = DshotBidirectionalFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.send_frame(frame, 0).await;
                Timer::after(Duration::from_micros(300)).await;
            }
        }
    }

    #[allow(unused)]
    pub async fn write_command_to_all_motors(&mut self, command: Command) {
        for index in 0..4 {
            let frame = DshotBidirectionalFrame::from_command(command);
            for _ in 0..command.repetitions_required() {
                self.send_frame(frame, 0).await;
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
