mod esp32_dshot;
mod esp32_pwm;
mod host;
mod rp_dshot;
mod rp_dshot_pio;
mod rp_pwm;
mod stm32_dshot;
mod stm32_pwm;

#[cfg(feature = "esp32")]
pub use {esp32_dshot::MotorDriverDshot, esp32_pwm::MotorDriverPwm};

#[cfg(rp)]
pub use {rp_dshot::MotorDriverDshot, rp_pwm::MotorDriverPwm};

#[cfg(feature = "stm32")]
pub use {stm32_dshot::MotorDriverDshot, stm32_pwm::MotorDriverPwm};

#[cfg(not(any(feature = "esp32", rp, feature = "stm32")))]
pub use host::{MotorDriverDshot, MotorDriverPwm};
