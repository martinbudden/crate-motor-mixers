#![cfg(not(any(feature = "esp32", rp, feature = "stm32")))]
use crate::dshot::DshotCommand;
use crate::{MotorCommands, MotorFrequencies, MotorOutputs};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverQuadPwm;

impl MotorDriverQuadPwm {
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
pub struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
}

impl MotorDriverQuadDshot {
    #[must_use]
    pub const fn new() -> Self {
        Self { motor_frequencies: MotorFrequencies::new() }
    }
}

#[allow(clippy::unused_async)]
impl MotorDriverQuadDshot {
    pub async fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }

    pub async fn write_commands_to_motors(&mut self, _commands: MotorCommands) {
        _ = self;
    }

    pub async fn write_command_to_all_motors(&mut self, _command: DshotCommand) {
        _ = self;
    }

    pub async fn reverse_all_motors(&mut self, _command: DshotCommand) {
        _ = self;
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
        Some(self.motor_frequencies)
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorDriverQuadPwm>();
    }
}
