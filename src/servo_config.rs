#[cfg(feature = "storage")]
use sequential_storage::map::PostcardValue;
#[cfg(feature = "serde")]
use {
    postcard::experimental::max_size::MaxSize,
    serde::{Deserialize, Serialize},
};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct ServoDeviceConfig {
    /// PWM values, in milliseconds, common range is 1000-2000 (1ms to 2ms).
    /// This is the value for servos when they should be in the middle. e.g. 1500.
    pub servo_center_pulse: u16,
    // The update rate of servo outputs, typically 50-498Hz.
    pub servo_pwm_rate: u16,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for ServoDeviceConfig {}

impl Default for ServoDeviceConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl ServoDeviceConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self { servo_center_pulse: 1500, servo_pwm_rate: 50 }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize, MaxSize))]
pub struct ServoConfig {
    /// lowpass servo filter frequency selection; 1/1000ths of loop freq.
    pub servo_lowpass_freq: u16,
    // send tail servo correction pulses even when unarmed.
    pub tri_unarmed_servo: u8,
    pub channel_forwarding_start_channel: u8,
}

#[cfg(feature = "storage")]
impl PostcardValue<'_> for ServoConfig {}

impl Default for ServoConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl ServoConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self { servo_lowpass_freq: 0, tri_unarmed_servo: 0, channel_forwarding_start_channel: 0 }
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
        is_full::<ServoDeviceConfig>();
        is_full::<ServoConfig>();
    }
    #[cfg(feature = "serde")]
    #[test]
    fn serde_types() {
        is_serde::<ServoDeviceConfig>();
        is_serde::<ServoConfig>();
    }
    #[cfg(feature = "storage")]
    #[test]
    fn storage_types() {
        is_storage::<ServoDeviceConfig>();
        is_storage::<ServoConfig>();
    }
}
