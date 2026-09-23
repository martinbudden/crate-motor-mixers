#![cfg(not(any(feature = "esp32", rp, feature = "stm32")))]
use dshot_codec::{DshotCommand, DshotCommandFrame};

use super::{DshotCommands, MotorFrequencies, MotorOutputs};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverPwm;

impl MotorDriverPwm {
    #[must_use]
    pub const fn new() -> Self {
        Self {}
    }

    #[inline]
    pub async fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        core::future::ready(()).await;
        _ = self;
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverDshot {
    motor_frequencies: MotorFrequencies,
}

impl MotorDriverDshot {
    const MOTOR_COUNT: usize = 4;

    #[must_use]
    pub const fn new() -> Self {
        Self { motor_frequencies: MotorFrequencies::new() }
    }

    #[inline]
    pub async fn send_command_frames(&mut self, frames: [DshotCommandFrame; MotorDriverDshot::MOTOR_COUNT]) {
        _ = self;
        _ = frames;
        core::future::ready(()).await;
    }
}

impl MotorDriverDshot {
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
        }
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        Some(self.motor_frequencies)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorDriverPwm>();
        is_full::<MotorDriverDshot>();
    }
}
