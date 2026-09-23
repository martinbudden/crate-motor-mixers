#![allow(unused)]

use embassy_time::Timer;

use dshot_codec::{DshotCommand, DshotCommandFrame, DshotMotorMasks, DshotTiming, DshotWaveform};

use super::{DshotCommands, MotorFrequencies, MotorOutputs};

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverDshot<'d> {
    masks: DshotMotorMasks,
    timing: DshotTiming,
    waveform: &'d mut DshotWaveform,
    motor_frequencies: MotorFrequencies,
    erpm_to_hz: f32,
}

impl<'d> MotorDriverDshot<'d> {
    const MOTOR_COUNT: usize = 4;

    #[must_use]
    pub fn new(waveform: &'d mut DshotWaveform, dshot_speed: dshot_codec::DshotSpeed, motor_pole_count: u8) -> Self {
        const SECONDS_PER_MINUTE: f32 = 60.0;
        let masks = DshotMotorMasks::new(1 << 2, 1 << 2, 1 << 3, 1 << 4);
        let timing = DshotTiming::new(dshot_speed);
        Self {
            masks,
            timing,
            waveform,
            motor_frequencies: MotorFrequencies::new(),
            erpm_to_hz: 2.0 * (100.0 / SECONDS_PER_MINUTE) / f32::from(motor_pole_count),
        }
    }

    #[inline]
    pub async fn send_command_frames(&mut self, frames: [DshotCommandFrame; MotorDriverDshot::MOTOR_COUNT]) {
        _ = self;
        _ = frames;
        core::future::ready(()).await;
    }
}

impl MotorDriverDshot<'_> {
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        let commands_frames = [
            DshotCommandFrame::from_throttle_unidirectional(outputs[0]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[1]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[2]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[3]),
        ];
        self.send_command_frames(commands_frames).await;
    }

    pub async fn write_commands_to_motors(&mut self, commands: DshotCommands) {
        let commands_frames = [
            DshotCommandFrame::from_command(commands[0]),
            DshotCommandFrame::from_command(commands[1]),
            DshotCommandFrame::from_command(commands[2]),
            DshotCommandFrame::from_command(commands[3]),
        ];
        self.send_command_frames(commands_frames).await;
    }

    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        let commands_frames = [
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
        ];
        for _ in 0..command.repetitions_required() {
            self.send_command_frames(commands_frames).await;
            Timer::after_micros(u64::from(command.delay_required_us())).await;
        }
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self.erpm_to_hz;
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
