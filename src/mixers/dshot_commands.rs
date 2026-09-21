use core::ops::{Deref, DerefMut};

use super::MAX_SUPPORTED_MOTOR_COUNT;
use dshot_codec::DshotCommand;

/// Struct containing array of motor commands, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DshotCommands(pub [DshotCommand; MAX_SUPPORTED_MOTOR_COUNT]);

impl Default for DshotCommands {
    fn default() -> Self {
        Self::new()
    }
}

impl DshotCommands {
    #[must_use]
    pub const fn new() -> Self {
        Self([DshotCommand::MotorStop; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Deref for DshotCommands {
    type Target = [DshotCommand; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for DshotCommands {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<DshotCommands>();
    }
}
