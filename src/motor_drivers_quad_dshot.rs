#![allow(unused)]
use crate::{MotorFrequencies, mixer_common::MotorOutputs};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct MotorDriverQuadDshot {
    motor_frequencies: MotorFrequencies,
}

impl MotorDriverQuadDshot {
    pub const fn new() -> Self {
        Self { motor_frequencies: MotorFrequencies::new() }
    }
}

impl MotorDriverQuadDshot {
    pub fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }

    #[allow(clippy::unnecessary_wraps)]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
        Some(self.motor_frequencies)
    }
}

