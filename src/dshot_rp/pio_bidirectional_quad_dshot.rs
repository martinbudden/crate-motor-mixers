use fixed::{FixedU32, types::extra::U8};

#[cfg(feature = "rp")]
use {
    crate::dshot::{DshotBidirectionalFrame, DshotError, GcrFrame},
    embassy_rp::{
        Peri, clocks,
        gpio::Pull,
        interrupt::typelevel::Binding,
        pio::{self, Common as PioCommon, Config as PioConfig, Instance, Pio, PioPin, StateMachine, program::pio_asm},
    },
    embassy_time::{Duration, with_timeout},
};

use crate::dshot::DshotProtocol;

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
//   21 GCR-encoded bits which are subsequently decoded to 16-bit telemetry + checksum.
//   Tight 2-cycle wait loop matches reference implementation.
#[cfg(feature = "rp")]
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
/// Bidirectional Dshot PIO driver for 4 ESCs with telemetry.
///
/// Uses 4 state machines, one for each ESC.
///
/// Supports `Dshot150`, `Dshot300`, `Dshot600`.
/// `Dshot1200` is not supported.
#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
#[cfg(feature = "rp")]
pub struct BidirectionalQuadDshotPio<'a, PIO: Instance> {
    sm0: BidirectionalDshotSm<'a, PIO, 0>,
    sm1: BidirectionalDshotSm<'a, PIO, 1>,
    sm2: BidirectionalDshotSm<'a, PIO, 2>,
    sm3: BidirectionalDshotSm<'a, PIO, 3>,
}

#[cfg(feature = "rp")]
impl<'a, PIO: Instance> BidirectionalQuadDshotPio<'a, PIO> {
    /// # Panics if `dshot_protocol` is `Dshot1200`.
    #[allow(unused)]
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, pio::InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        pin1: Peri<'a, impl PioPin + 'a>,
        pin2: Peri<'a, impl PioPin + 'a>,
        pin3: Peri<'a, impl PioPin + 'a>,
        dshot_protocol: DshotProtocol,
    ) -> Self {
        assert!(
            !matches!(dshot_protocol, DshotProtocol::Dshot1200),
            "Dshot1200 is not supported in bidirectional mode"
        );

        let mut pio = Pio::new(pio, irq);

        let sm0 = BidirectionalDshotSm::new(pio.sm0, pin0, &mut pio.common, dshot_protocol);
        let sm1 = BidirectionalDshotSm::new(pio.sm1, pin1, &mut pio.common, dshot_protocol);
        let sm2 = BidirectionalDshotSm::new(pio.sm2, pin2, &mut pio.common, dshot_protocol);
        let sm3 = BidirectionalDshotSm::new(pio.sm3, pin3, &mut pio.common, dshot_protocol);

        Self { sm0, sm1, sm2, sm3 }
    }
}

#[cfg(feature = "rp")]
impl<'a, PIO: Instance> BidirectionalQuadDshotPio<'a, PIO> {
    #[inline]
    pub async fn send_frame_and_receive_gcr(
        &mut self,
        frame: DshotBidirectionalFrame,
        sm_index: usize,
    ) -> Result<GcrFrame, DshotError> {
        match sm_index {
            1 => self.sm1.send_frame_and_receive_gcr(frame).await,
            2 => self.sm2.send_frame_and_receive_gcr(frame).await,
            3 => self.sm3.send_frame_and_receive_gcr(frame).await,
            _ => self.sm0.send_frame_and_receive_gcr(frame).await,
        }
    }

    /// Sends a `DshotBidirectionalFrame`
    /// Does not return any response.
    #[allow(unused)]
    #[inline]
    pub async fn send_frame(&mut self, frame: DshotBidirectionalFrame, sm_index: usize) {
        match sm_index {
            1 => self.sm1.send_frame(frame).await,
            2 => self.sm2.send_frame(frame).await,
            3 => self.sm3.send_frame(frame).await,
            _ => self.sm0.send_frame(frame).await,
        }
    }
    /// Synchronously sends a `DshotBidirectionalFrame`
    /// Does not return any response.
    #[allow(unused)]
    #[inline]
    pub fn send_frame_blocking(&mut self, frame: DshotBidirectionalFrame, sm_index: usize) {
        match sm_index {
            1 => self.sm1.send_frame_blocking(frame),
            2 => self.sm2.send_frame_blocking(frame),
            3 => self.sm3.send_frame_blocking(frame),
            _ => self.sm0.send_frame_blocking(frame),
        }
    }
}

/// Bidirectional `Dshot` State Machine driver for 1 ESC with telemetry.
///
/// Supports `Dshot150`, `Dshot300`, `Dshot600`. `Dshot1200` is not supported.
#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
#[cfg(feature = "rp")]
pub struct BidirectionalDshotSm<'a, PIO: Instance, const SM: usize> {
    sm: StateMachine<'a, PIO, SM>,
    program_origin: u8,
}

#[cfg(feature = "rp")]
impl<'a, PIO: Instance, const SM: usize> BidirectionalDshotSm<'a, PIO, SM> {
    pub fn new(
        mut sm: StateMachine<'a, PIO, SM>,
        pin: Peri<'a, impl PioPin + 'a>,
        pio_common: &mut PioCommon<'a, PIO>,
        dshot_protocol: DshotProtocol,
    ) -> Self {
        let mut pin = pio_common.make_pio_pin(pin);
        pin.set_pull(Pull::Up);

        let mut config = PioConfig::default();

        let prg = dshot_bidirectional_program!();
        let program = pio_common.load_program(&prg.program);

        config.use_program(&program, &[]);

        config.clock_divider = bidir_pio_clock_divider(dshot_protocol, clocks::clk_sys_freq());

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

#[cfg(feature = "rp")]
impl<'a, PIO: Instance, const SM: usize> BidirectionalDshotSm<'a, PIO, SM> {
    /// Reset PIO program counter to the pull-block address.
    fn reset_program_counter(&mut self) {
        // program_origin + 0: push block     (pushes previous RX data)
        // program_origin + 1: set pindirs, 1 (pin as output)
        // program_origin + 2: pull block     (waits for TX frame — idle position)
        let pull_block_address = self.program_origin + 2;

        if self.sm.get_addr() != pull_block_address {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            const MOV_ISR_NULL: u16 = 0b101_00000_110_00_011;
            unsafe { self.sm.exec_instr(MOV_ISR_NULL) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instruction = u16::from(self.program_origin + 1) & 0x1F;
            unsafe { self.sm.exec_instr(jmp_instruction) };
        }
    }

    /// Sends a `DshotBidirectionalFrame` and returns an unvalidated `GcrFrame`.
    /// It is the responsibility of the caller to check the `GcrFrame` is valid before using it.
    ///
    /// wait_push timeout  → PioTxTimeout
    /// wait_pull timeout  → PioRxTimeout
    ///
    /// # Errors ` DshotError::PioTxTimeout`, ` DshotError::PioRxTimeout`
    async fn send_frame_and_receive_gcr(&mut self, frame: DshotBidirectionalFrame) -> Result<GcrFrame, DshotError> {
        // Clear any existing rx data
        while self.sm.rx().try_pull().is_some() {}
        self.reset_program_counter();

        // bidirectional dshot inverts frame
        let frame_inverted = u32::from(!frame.raw());
        // TODO: check 10ms timeout ins PIO `send_and_receive`.
        with_timeout(Duration::from_millis(10), self.sm.tx().wait_push(frame_inverted))
            .await
            .map_err(|_| DshotError::TxTimeout)?;

        let rx_data = with_timeout(Duration::from_micros(500), self.sm.rx().wait_pull())
            .await
            .map_err(|_| DshotError::RxTimeout)?;

        Ok(GcrFrame::from_raw(rx_data))
    }

    /// Sends a `DshotBidirectionalFrame`
    /// Does not return any response.
    #[allow(unused)]
    pub async fn send_frame(&mut self, frame: DshotBidirectionalFrame) {
        while self.sm.rx().try_pull().is_some() {}
        self.reset_program_counter();

        // bidirectional dshot inverts frame
        let frame_inverted = u32::from(!frame.raw());
        self.sm.tx().wait_push(frame_inverted).await;
    }

    /// Synchronously sends a `DshotBidirectionalFrame`
    /// Does not return any response.
    #[allow(unused)]
    pub fn send_frame_blocking(&mut self, frame: DshotBidirectionalFrame) {
        while self.sm.rx().try_pull().is_some() {}
        self.reset_program_counter();

        // bidirectional dshot inverts frame
        let frame_inverted = u32::from(!frame.raw());
        self.sm.tx().push(frame_inverted);
    }
}

#[allow(unused)]
fn bidir_pio_clock_divider(protocol: DshotProtocol, sys_clock_frequency: u32) -> FixedU32<U8> {
    // pio clock divider = system_clock / (40 × protocol_baud_rate) encoded as FixedU32<U8>

    let sys_clock = u64::from(sys_clock_frequency);
    let target = 12_000_000u64 * u64::from(protocol.baud_rate()) / 300_000;
    #[allow(clippy::cast_possible_truncation)]
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_bidir_divider_at_125mhz() {
        // Verify bidir divider: target PIO clock = 12MHz * dshot_speed/300kHz
        // Dshot600: target = 12MHz * 600/300 = 24MHz
        // At 125MHz: divider = 125/24 = 5.2083...
        const SYS_CLOCK: u32 = 125_000_000;
        let divider = bidir_pio_clock_divider(DshotProtocol::Dshot600, SYS_CLOCK);
        let bits = (125 << 8) / 24;
        assert_eq!(1333, bits);
        let expected: FixedU32<U8> = FixedU32::from_bits(bits);

        assert_eq!(expected, divider);
    }
}
