#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

// parameters to mix function
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
#[allow(missing_docs)]
pub struct MotorOutputRange {
    /// Minimum motor output, typically set to 5.5% to avoid ESC desynchronization,
    /// may be set to zero if using dynamic idle control or brushed motors.
    pub min: f32,
    /// Maximum motor output, typically set to 1.0.
    pub max: f32,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for MotorOutputRange {}

impl Default for MotorOutputRange {
    fn default() -> Self {
        Self::new()
    }
}

impl MotorOutputRange {
    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { min: 0.0, max: 1.0 }
    }
    /// Set the min of a newly constructed range.
    #[must_use]
    pub const fn with_min(mut self, min: f32) -> Self {
        self.min = min;
        self
    }
    /// Set the max of a newly constructed range.
    #[must_use]
    pub const fn with_max(mut self, max: f32) -> Self {
        self.max = max;
        self
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SaturationCompensation {
    /// Method 1: reduces yaw rate to preserve throttle and attitude stabilization.
    #[default]
    YawReduction,
    /// Method 2: adjusts throttle baseline up or down to maximize yaw authority.
    ThrottleAdjustment,
}

impl TryFrom<u8> for SaturationCompensation {
    type Error = ();

    /// Validating conversion from `u8` to `SaturationCompensation`. Invalid values return error.
    fn try_from(value: u8) -> Result<Self, ()> {
        let default = Self::default();
        if value == default as u8 {
            Ok(default)
        } else {
            let ret = Self::from_u8(value);
            if ret == default { Err(()) } else { Ok(ret) }
        }
    }
}

impl SaturationCompensation {
    /// Forgiving conversion from `u8` to `SaturationCompensation`, converts invalid values to default.
    #[must_use]
    pub fn from_u8(value: u8) -> Self {
        match value {
            1 => Self::ThrottleAdjustment,
            _ => Self::YawReduction,
        }
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_serde<T: Serialize + MaxSize + for<'a> Deserialize<'a>>() {}
    #[cfg(feature = "storage")]
    fn is_storage<T: for<'a> PostcardValue<'a>>() {}

    #[test]
    fn normal_types() {
        is_full::<SaturationCompensation>();
        is_full::<MotorOutputRange>();
    }
    #[cfg(feature = "serde")]
    #[test]
    fn serde_types() {
        is_serde::<MotorOutputRange>();
    }
    #[cfg(feature = "storage")]
    #[test]
    fn storage_types() {
        is_storage::<MotorOutputRange>();
    }
}
