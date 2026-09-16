//! Minimal test for motor mixer.
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PINs 11-14

#![allow(unused)]
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use motor_mixers::{
    MixerConfig, MotorConfig, MotorDriver, MotorDriverQuadDshot, MotorMixer, MotorMixerMessage, dshot::DshotSpeed,
};

#[cfg(feature = "rp")]
use embassy_rp::{bind_interrupts, clocks::clk_sys_freq, peripherals::PIO0, pio::InterruptHandler};
#[cfg(feature = "rp")]
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[cfg(feature = "rp")]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting motor-mixers Basic test");

    // Initialize MotorDriverQuadDshot on pins 11-14
    let driver_quad_dshot = MotorDriverQuadDshot::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        p.PIN_12,
        p.PIN_14,
        p.PIN_15,
        DshotSpeed::Dshot300,
        MotorDriverQuadDshot::DEFAULT_MOTOR_POLE_COUNT,
    );
    let driver = MotorDriver::QuadDshot(driver_quad_dshot);
    let mixer_config = MixerConfig::default();
    let motor_config = MotorConfig::default();
    let mut motor_mixer = MotorMixer::new(mixer_config, motor_config, driver);
    let mixer_message = MotorMixerMessage::new();

    info!("Sending mixer message for 2 seconds");
    for _ in 0..2000 {
        motor_mixer.output_to_motors(mixer_message).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    info!("Sending mixer message indefinitely");
    let mut count: u32 = 0;
    loop {
        motor_mixer.output_to_motors(mixer_message).await;
        Timer::after(Duration::from_millis(1)).await;
        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
