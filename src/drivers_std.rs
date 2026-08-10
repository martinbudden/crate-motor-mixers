#![cfg(feature = "std")]

use crate::mixer_common::{MotorFrequencies, MotorOutputs};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorDriverQuadPwm;

impl MotorDriverQuadPwm {
    #[must_use]
    pub const fn new() -> Self {
        Self {}
    }

    pub fn write_to_motors(&mut self, _outputs: MotorOutputs) {
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

impl MotorDriverQuadDshot {
    pub fn write_to_motors(&mut self, _outputs: MotorOutputs) {
        _ = self;
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self;
        Some(self.motor_frequencies)
    }
}
