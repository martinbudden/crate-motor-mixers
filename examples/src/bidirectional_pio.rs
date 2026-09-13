//! Minimal test for bidirectional PIO - continuous minimum throttle at 2kHz
//!
//! Use to verify:
//! 1. Signal timing with oscilloscope
//! 2. ESC startup tones (signal recognized)
//! 3. ESC stays armed (no timeout)
//!
//! Hardware: Raspberry Pi Pico / Pico 2
//! Connections:
//!   - ESC signal: PINs 11-14

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::{bind_interrupts, clocks::clk_sys_freq, peripherals::PIO0, pio::InterruptHandler};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

use motor_mixers::{
    dshot::{Command, DshotBidirectionalFrame, Protocol},
    dshot_rp::PioBidirectionalQuadDshot,
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

    // Initialize DShot150 on pins 11-14
    let mut dshot =
        PioBidirectionalQuadDshot::new(p.PIO0, Irqs, p.PIN_11, p.PIN_12, p.PIN_14, p.PIN_15, Protocol::Dshot300);

    info!("DShot300 initialized on PIN_11");
    info!("Expected DShot300 timing:");
    info!("  - Bit period: ~3.33us");
    info!("  - Bit 1 HIGH: ~2.5us (75%)");
    info!("  - Bit 0 HIGH: ~1.17us (35%)");

    // Arm ESC with MotorStop (value 0) for 2 seconds
    info!("Sending MotorStop for 2 seconds (arming sequence)...");
    info!("(ESC should produce startup tones)");
    let frame = DshotBidirectionalFrame::from_command(Command::MotorStop);
    for _ in 0..2000 {
        dshot.send_frame_sm0(frame).await;
        Timer::after(Duration::from_millis(1)).await;
    }

    // Continue with MotorStop to keep ESC armed
    info!("Arming complete. Continuing MotorStop at 1kHz...");
    info!("ESC should stay armed (no motor spin)");
    let mut count: u32 = 0;
    loop {
        dshot.send_frame_sm0(frame).await;
        Timer::after(Duration::from_millis(1)).await;
        count = count.wrapping_add(1);
        if count.is_multiple_of(1000) {
            info!("Running... {} frames sent", count);
        }
    }
}
