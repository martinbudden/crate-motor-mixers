use core::ops::{Deref, DerefMut};

#[cfg(feature = "eight_motors")]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 8;
#[cfg(not(feature = "eight_motors"))]
pub const MAX_SUPPORTED_MOTOR_COUNT: usize = 4;

/// Struct containing array of motor outputs, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputs(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl Default for MotorOutputs {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorOutputs {
    #[must_use]
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
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

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorOutputs>();
    }
}
