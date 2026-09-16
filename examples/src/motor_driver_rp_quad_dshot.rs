//! Minimal test for MotorDriverQuadDshot.
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PINs 11-14

#![no_std]
#![no_main]
#![cfg(feature = "rp")]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, clocks::clk_sys_freq, peripherals::PIO0, pio::InterruptHandler};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use motor_mixers::{
    MotorDriverQuadDshot,
    dshot::{DshotCommand, DshotCommandFrame, DshotSpeed},
};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Print system clock for verification
    let sys_freq = clk_sys_freq();
    info!("System clock: {} Hz", sys_freq);
    info!("Starting motor-mixers Basic test");

    // Initialize MotorDriverQuadDshot on pins 11-14
    let mut driver = MotorDriverQuadDshot::new(
        p.PIO0,
        Irqs,
        p.PIN_11,
        p.PIN_12,
        p.PIN_14,
        p.PIN_15,
        DshotProtocol::Dshot300,
        MotorDriverQuadDshot::DEFAULT_MOTOR_POLE_COUNT,
    );

    // Arm ESC with MotorStop (value 0) for 2 seconds
    info!("Sending MotorStop for 2 seconds");
    let frame = DshotBidirectionalFrame::from_command(DshotCommand::MotorStop);
    for _ in 0..2000 {
        driver.send_frame(frame, 0).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    // Continue with MotorStop to keep ESC armed
    info!("Sending MotorStop for indefinitely");
    let mut count: u32 = 0;
    loop {
        driver.send_frame(frame, 0).await;
        Timer::after(Duration::from_millis(1)).await;
        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
