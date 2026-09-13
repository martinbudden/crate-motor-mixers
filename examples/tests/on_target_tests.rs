//! On-target unit tests for motor-mixers
//!
//! These tests run directly on the RP2040/RP2350 hardware using defmt-test.
//! They verify that library functions work correctly on the target MCU.
//!
//! Run with: `cargo test --test on_target_tests` (requires probe-rs)

#![no_std]
#![no_main]

use defmt_rtt as _;
use embassy_rp as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use super::*;
}
