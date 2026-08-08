use core::ops::{Deref, DerefMut};

use signal_filters::SlewRateLimiterf32;

use crate::{motor_drivers_quad_dshot::MotorDriverQuadDshot, motor_drivers_quad_pwm::MotorDriverQuadPwm};

#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub enum MotorDriver {
    QuadPwm(MotorDriverQuadPwm),
    QuadDshot(MotorDriverQuadDshot),
}

impl MotorDriver {
    pub fn write_to_motors(&mut self, outputs: MotorOutputs) {
        match self {
            Self::QuadPwm(driver) => driver.write_to_motors(outputs),
            Self::QuadDshot(driver) => driver.write_to_motors(outputs),
        }
    }

    // Returns the motor frequencies (ie revolutions per second) of the motors from the driver.
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        match self {
            Self::QuadPwm(_) => None,
            Self::QuadDshot(driver) => driver.motor_frequencies(),
        }
    }
}

#[cfg(feature = "eight_motors")]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 8;
#[cfg(not(feature = "eight_motors"))]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 4;

/// Struct containing array of motor outputs, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorOutputs {
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorOutputs {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputs {
    type Target = [f32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputs {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Array of motor rotation frequencies, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorFrequencies(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorFrequencies {
    #[must_use]
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorFrequencies {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorFrequencies {
    type Target = [f32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorFrequencies {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputFilters(pub [SlewRateLimiterf32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorOutputFilters {
    pub const fn new() -> Self {
        Self([SlewRateLimiterf32::new(); MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorOutputFilters {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorOutputFilters {
    type Target = [SlewRateLimiterf32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorOutputFilters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorOutputs>();
        is_full::<MotorOutputFilters>();
    }
}
