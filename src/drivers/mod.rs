mod esp32_dshot;
mod esp32_pwm;
mod host;
mod rp_dshot;
mod rp_dshot_pio;
mod rp_pwm;
mod stm32_dshot;
mod stm32_pwm;

#[cfg(feature = "esp32")]
pub use {esp32_dshot::MotorDriverQuadDshot, esp32_pwm::MotorDriverQuadPwm};

#[cfg(rp)]
pub use {rp_dshot::MotorDriverQuadDshot, rp_pwm::MotorDriverQuadPwm};

#[cfg(feature = "stm32")]
pub use {stm32_dshot::MotorDriverQuadDshot, stm32_pwm::MotorDriverQuadPwm};

#[cfg(not(any(feature = "esp32", rp, feature = "stm32")))]
pub use host::{MotorDriverQuadDshot, MotorDriverQuadPwm};
