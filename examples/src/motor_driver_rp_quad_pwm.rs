//! Minimal test for MotorDriverQuadDshot.
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PINs 11-14

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{
    clocks::clk_sys_freq,
    pwm::{Config as PwmConfig, Pwm},
};
use embassy_time::{Duration, Timer};
use fixed::traits::ToFixed;
use {defmt_rtt as _, panic_probe as _};

use motor_mixers::{MotorDriverQuadPwm, MotorOutputs};

#[allow(unused)]
fn pwm_config_400hz() -> PwmConfig {
    let mut config = PwmConfig::default();

    config.divider = 10.to_fixed();
    config.top = 37_499;
    config.phase_correct = false;
    config.enable = true;

    config
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting motor-mixers Basic test");

    // Initialize MotorDriverQuadDshot on pins 0-3
    let config0 = PwmConfig::default();
    let config1 = PwmConfig::default();
    let frequency_hz = 50.0;

    let pwm0 = Pwm::new_output_ab(p.PWM_SLICE5, p.PIN_10, p.PIN_11, config0);
    let pwm1 = Pwm::new_output_ab(p.PWM_SLICE6, p.PIN_12, p.PIN_13, config1);

    let mut driver = MotorDriverQuadPwm::new(pwm0, pwm1, frequency_hz);

    let motor_outputs = MotorOutputs::new();

    info!("Sending motor outputs for 2 seconds");
    for _ in 0..2000 {
        driver.write_to_motors(motor_outputs).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    info!("Sending motor outputs indefinitely");
    let mut count: u32 = 0;
    loop {
        driver.write_to_motors(motor_outputs).await;
        Timer::after(Duration::from_millis(1)).await;
        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
