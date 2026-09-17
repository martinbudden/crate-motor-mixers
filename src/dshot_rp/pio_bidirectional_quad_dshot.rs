use fixed::{FixedU32, types::extra::U8};

use crate::dshot::DshotSpeed;

#[cfg(rp)]
use {
    crate::dshot::{DshotCommandFrame, DshotError, GcrFrame},
    embassy_rp::{
        Peri, clocks,
        gpio::Pull,
        pio::{self, Common as PioCommon, Config as PioConfig, Instance, PioPin, StateMachine, program::pio_asm},
    },
    embassy_time::{Duration, with_timeout},
};

// Bidirectional Dshot PIO program based on pico-bidir-dshot reference.
//
// Program layout (offsets from program_origin):
//   program_origin + 0: push block     (pushes previous RX data)
//   program_origin + 1: set pindirs, 1 (pin as output)
//   program_origin + 2: pull block     (waits for TX frame — idle position)
//
// TX Phase (40 cycles per bit):
//   14 cycles LOW, 14 cycles data bit (inverted), 11 cycles HIGH, 1 jmp
//
// RX Phase (pulse-width measurement):
//   Wait for falling edge, measure pulse widths using counting loops.
//   20 GCR-encoded bits which are subsequently decoded to 16-bit telemetry + checksum.
//   Tight 2-cycle wait loop matches reference implementation.
#[cfg(rp)]
macro_rules! dshot_bidirectional_program {
() => { pio_asm!(
    ".wrap_target"
        "push block"                    // Push any pending RX data
        "set pindirs, 1"                // Pin as output
        "pull block"                    // Pull TX frame (inverted)

        // TX Phase: send 16-bit Dshot frame
        "out null, 16"                  // Discard upper 16 bits (zeros)
    "tx_bit:"
        "set pins, 0 [13]"              // 14 cycles LOW
        "out pins, 1 [13]"              // 14 cycles: output data bit
        "set pins, 1 [10]"              // 11 cycles HIGH
        "jmp !osre, tx_bit"             // Loop until OSR empty (1 cycle)

        // Prepare for RX
        "set x, 20"                     // 21 bits to receive
        "mov osr, ~null"                // OSR = 0xFFFFFFFF (source of 1s)
        "set pindirs, 0"                // Pin as input

        // Wait for falling edge (tight loop — 2 cycles per check)
    "wait_for_pin:"
        "jmp pin, wait_for_pin [1]"     // Loop while pin HIGH

        // RX Phase: pulse-width measurement
    "new_zero:"
        "set y, 6"                      // 7 iterations (first measurement)
        "jmp meas_zero"

    "another_zero:"
        "set y, 13 [1]"                 // 14 iterations (continuing)

    "meas_zero:"
        "jmp pin, new_one"              // If HIGH, transition to measuring HIGH
        "jmp y--, meas_zero"            // Keep measuring LOW
        "in null, 1"                    // Timeout: long LOW = shift in 0
        "jmp x--, another_zero"         // Next bit
        "jmp done"                      // All bits received

    "new_one:"
        "set y, 6 [1]"                  // 7 iterations
        "jmp meas_one"

    "another_one:"
        "set y, 13 [1]"                 // 14 iterations

    "meas_one:"
        "jmp pin, cont_one"             // Still HIGH, continue measuring
        "jmp new_zero"                  // Went LOW, short HIGH pulse (no shift)
    "cont_one:"
        "jmp y--, meas_one"             // Keep measuring HIGH
        "in osr, 1"                     // Timeout: long HIGH = shift in 1
        "jmp x--, another_one"          // Next bit

    "done:"
        ".wrap"
    )};
}
/// Bidirectional `Dshot` State Machine driver for 1 ESC with telemetry.
///
/// Supports `Dshot150`, `Dshot300`, `Dshot600`. `Dshot1200` is not supported.
#[allow(missing_debug_implementations, missing_copy_implementations)]
#[cfg(rp)]
pub struct BidirectionalDshotSm<'a, PIO: Instance, const SM: usize> {
    sm: StateMachine<'a, PIO, SM>,
    program_origin: u8,
}

#[cfg(rp)]
impl<'a, PIO: Instance, const SM: usize> BidirectionalDshotSm<'a, PIO, SM> {
    pub fn new(
        mut sm: StateMachine<'a, PIO, SM>,
        pin: Peri<'a, impl PioPin + 'a>,
        pio_common: &mut PioCommon<'a, PIO>,
        dshot_speed: DshotSpeed,
    ) -> Self {
        let mut pin = pio_common.make_pio_pin(pin);
        pin.set_pull(Pull::Up);

        let mut config = PioConfig::default();

        let prg = dshot_bidirectional_program!();
        let program = pio_common.load_program(&prg.program);

        config.use_program(&program, &[]);

        config.clock_divider = pio_clock_divider(dshot_speed, clocks::clk_sys_freq());

        config.shift_out = pio::ShiftConfig { auto_fill: false, direction: pio::ShiftDirection::Left, threshold: 32 };
        config.shift_in = pio::ShiftConfig { auto_fill: false, direction: pio::ShiftDirection::Left, threshold: 32 };

        config.fifo_join = pio::FifoJoin::Duplex;

        config.set_jmp_pin(&pin);
        config.set_set_pins(&[&pin]);
        config.set_out_pins(&[&pin]);
        config.set_in_pins(&[&pin]);

        sm.set_config(&config);
        sm.set_pin_dirs(pio::Direction::Out, &[&pin]);
        sm.restart();
        sm.set_enable(true);
        sm.set_clock_divider(config.clock_divider);

        Self { sm, program_origin: program.origin }
    }
}

#[cfg(rp)]
impl<PIO: Instance, const SM: usize> BidirectionalDshotSm<'_, PIO, SM> {
    /// Reset PIO program counter to the pull-block address.
    fn reset_program_counter(&mut self) {
        // program_origin + 0: push block     (pushes previous RX data)
        // program_origin + 1: set pindirs, 1 (pin as output)
        // program_origin + 2: pull block     (waits for TX frame — idle position)
        let pull_block_address = self.program_origin + 2;

        if self.sm.get_addr() != pull_block_address {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            #[allow(clippy::unusual_byte_groupings)]
            const MOV_ISR_NULL: u16 = 0b101_00000_110_00_011;
            unsafe { self.sm.exec_instr(MOV_ISR_NULL) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instruction = u16::from(self.program_origin + 1) & 0x1F;
            unsafe { self.sm.exec_instr(jmp_instruction) };
        }
    }

    /// Sends a `DshotCommandFrame` and returns a `GcrFrame`.
    /// It is the responsibility of the caller to check this frame is valid and decode it.
    ///
    /// # Errors
    /// Returns [`DshotError::TxTimeout`] if pushing to the TX FIFO times out,
    /// or [`DshotError::RxTimeout`] if the ESC fails to return a telemetry packet.
    pub async fn send_frame_and_receive_gcr20(&mut self, frame: DshotCommandFrame) -> Result<GcrFrame, DshotError> {
        // Clear any stale rx data out of the FIFO queue
        while self.sm.rx().try_pull().is_some() {}

        // Clear state variations by forcing the execution index back to the wrapper start
        self.reset_program_counter();

        // Bitwise invert the raw frame data.
        // Mask explicitly with 0xFFFF to ensure the upper 16 bits are strictly zeroed out,
        // preventing `out null, 16` inside the PIO from discarding live values.
        let frame_inverted = u32::from(!frame.raw()) & 0x0000_FFFF;

        // Push the data into the TX FIFO block
        with_timeout(Duration::from_millis(10), self.sm.tx().wait_push(frame_inverted))
            .await
            .map_err(|_| DshotError::TxTimeout)?;

        // Wait for the telemetry packet response from the ESC.
        // Timeout is 2ms to absorb the transmission window
        // and allow the ESC sufficient time to calculate the GCR reply.
        let gcr20_raw = with_timeout(Duration::from_millis(2), self.sm.rx().wait_pull())
            .await
            .map_err(|_| DshotError::RxTimeout)?;

        // 6. Map the raw 20-bit GCR value to our domain container
        let gcr_frame = GcrFrame::from_raw(gcr20_raw);
        Ok(gcr_frame)
    }

    /// Sends a `DshotCommandFrame`
    /// Does not return any response.
    pub async fn send_frame(&mut self, frame: DshotCommandFrame) {
        // Clear any stale rx data out of the FIFO queue
        while self.sm.rx().try_pull().is_some() {}
        self.reset_program_counter();

        // Bidirectional DShot inverts the frame payload.
        // Fix: Mask explicitly with 0xFFFF to ensure clean upper padding zeros.
        let frame_inverted = u32::from(!frame.raw()) & 0x0000_FFFF;
        self.sm.tx().wait_push(frame_inverted).await;
    }

    /// Synchronously sends a `DshotCommandFrame`
    /// Does not return any response.
    pub fn send_frame_blocking(&mut self, frame: DshotCommandFrame) {
        // Clear any stale rx data out of the FIFO queue
        while self.sm.rx().try_pull().is_some() {}
        self.reset_program_counter();

        // Bidirectional DShot inverts the frame payload.
        // Fix: Mask explicitly with 0xFFFF to ensure clean upper padding zeros.
        let frame_inverted = u32::from(!frame.raw()) & 0x0000_FFFF;
        self.sm.tx().push(frame_inverted);
    }
}

#[allow(unused)]
fn pio_clock_divider(dshot_speed: DshotSpeed, sys_clock_frequency: u32) -> FixedU32<U8> {
    // pio clock divider = system_clock / (40 × dshot_speed_baud_rate) encoded as FixedU32<U8>

    let sys_clock = u64::from(sys_clock_frequency);
    let target = 12_000_000u64 * u64::from(dshot_speed.baud_rate()) / 300_000;
    #[allow(clippy::cast_possible_truncation)]
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pio_clock_divider() {
        // Verify divider: target PIO clock = 12MHz * dshot_speed/300kHz
        // Dshot600: target = 12MHz * 600/300 = 24MHz
        // At 125MHz: divider = 125/24 = 5.2083...
        let sys_clock_125 = 125_000_000u32;
        let divider = pio_clock_divider(DshotSpeed::Dshot600, sys_clock_125);
        let bits = (125 << 8) / 24;
        assert_eq!(1333, bits);
        let expected: FixedU32<U8> = FixedU32::from_bits(bits);
        assert_eq!(expected, divider);

        let sys_clock_120 = 120_000_000u32;
        let divider = pio_clock_divider(DshotSpeed::Dshot300, sys_clock_120);
        let bits = (120 << 8) / 24;
        assert_eq!(1280, bits);
        let pio_clock = u64::from(sys_clock_120) * 256 / u64::from(divider.to_bits());
        assert_eq!(pio_clock, 40 * 300_000);
    }
}
