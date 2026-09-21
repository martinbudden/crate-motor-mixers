use core::ops::{Deref, DerefMut};

use signal_filters::SlewRateLimiterf32;

use super::MAX_SUPPORTED_MOTOR_COUNT;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorOutputFilters(pub [SlewRateLimiterf32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorOutputFilters {
    #[must_use]
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
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorOutputFilters>();
    }
}
