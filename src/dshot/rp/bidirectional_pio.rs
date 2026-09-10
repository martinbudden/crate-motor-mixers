#![cfg(any(feature = "rp2040", feature = "rp235xa", feature = "rp235xb"))]

use embassy_rp::{
    Peri,
    clocks::clk_sys_freq,
    pio::{
        Config as PioConfig, Direction, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
    },
    {gpio::Pull, interrupt::typelevel::Binding, pio::program::pio_asm},
};
use embassy_time::{Duration, with_timeout};
use fixed::{FixedU32, types::extra::U8};

use crate::dshot::{dshot_error::DshotError, protocol::Protocol};

#[allow(unused)]
#[allow(clippy::cast_possible_truncation)]
fn tx_pio_clock_divider(protocol: Protocol) -> FixedU32<U8> {
    let sys_clock = u64::from(clk_sys_freq());
    FixedU32::<U8>::from_bits(((sys_clock << 8) / (8 * u64::from(protocol.baud_rate()))) as u32)
}

#[allow(clippy::cast_possible_truncation)]
fn bidir_pio_clock_divider(protocol: Protocol) -> FixedU32<U8> {
    let sys_clock = u64::from(clk_sys_freq());
    let target = 12_000_000u64 * u64::from(protocol.baud_rate()) / 300_000;
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}

/// Bidirectional `DShot` PIO driver for single ESC with telemetry.
///
/// Supports `DShot150`, `DShot300`, `DShot600`. `DShot1200` is not supported
/// (panics at construction).
pub struct BidirectionalDshotPio<'a, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
    // _pin: Pin<'a, PIO>,
    origin: u8,
}

impl<'a, PIO: Instance> BidirectionalDshotPio<'a, PIO> {
    /// # Panics
    ///
    /// Panics if `speed` is `Protocol::Dshot1200`.
    #[allow(unused)]
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        protocol: Protocol,
    ) -> Self {
        assert!(!matches!(protocol, Protocol::Dshot1200), "Dshot1200 is not supported for bidirectional mode");

        let mut pio = Pio::new(pio, irq);
        let mut pin = pio.common.make_pio_pin(pin0);

        pin.set_pull(Pull::Up);

        // Bidirectional DShot PIO program based on pico-bidir-dshot reference.
        //
        // Program layout (offsets from origin):
        //   origin + 0: push block     (pushes previous RX data)
        //   origin + 1: set pindirs, 1 (pin as output)
        //   origin + 2: pull block     (waits for TX frame — idle position)
        //
        // TX Phase (32 cycles per bit):
        //   14 cycles LOW, 14 cycles data bit (inverted), 11 cycles HIGH, 1 jmp
        //
        // RX Phase (pulse-width measurement):
        //   Wait for falling edge, measure pulse widths using counting loops.
        //   21 GCR-encoded bits decoded to 16-bit telemetry + CRC.
        //   Tight 2-cycle wait loop matches reference implementation.
        let prg = pio_asm!(
            ".wrap_target"
            "push block"                    // Push any pending RX data
            "set pindirs, 1"                // Pin as output
            "pull block"                    // Pull TX frame (inverted)

            // TX Phase: send 16-bit DShot frame
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
        );

        let loaded = pio.common.load_program(&prg.program);
        let origin = loaded.origin;
        let clock_divider = bidir_pio_clock_divider(protocol);

        let mut pio_config = PioConfig::default();
        pio_config.use_program(&loaded, &[]);

        pio_config.clock_divider = clock_divider;

        pio_config.shift_out = ShiftConfig { auto_fill: false, direction: ShiftDirection::Left, threshold: 32 };
        pio_config.shift_in = ShiftConfig { auto_fill: false, direction: ShiftDirection::Left, threshold: 32 };

        pio_config.fifo_join = FifoJoin::Duplex;

        pio_config.set_jmp_pin(&pin);
        pio_config.set_set_pins(&[&pin]);
        pio_config.set_out_pins(&[&pin]);
        pio_config.set_in_pins(&[&pin]);

        pio.sm0.set_config(&pio_config);
        pio.sm0.set_pin_dirs(Direction::Out, &[&pin]);
        pio.sm0.restart();
        pio.sm0.set_enable(true);
        pio.sm0.set_clock_divider(clock_divider);

        Self { pio_instance: pio, origin }
    }

    /// Reset PIO to the pull-block position if it drifted (e.g. telemetry timeout).
    #[allow(unused)]
    fn sync_pc(&mut self) {
        let expected_pc = self.origin + 2;
        let current_pc = self.pio_instance.sm0.get_addr();

        if current_pc != expected_pc {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            // MOV ISR, NULL = 0b101_00000_110_00_011 = 0xA0C3
            unsafe { self.pio_instance.sm0.exec_instr(0xA0C3) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instr = u16::from(self.origin + 1) & 0x1F;
            unsafe { self.pio_instance.sm0.exec_instr(jmp_instr) };
        }
    }

    /// Send a frame, read raw RX value, and decode telemetry
    #[allow(unused)]
    async fn send_and_receive_raw(&mut self, frame_raw: u16) -> Result<u32, DshotError> {
        // Clear stale RX data
        while self.pio_instance.sm0.rx().try_pull().is_some() {}
        self.sync_pc();

        let tx_data = u32::from(!frame_raw); // bidir DShot sends inverted

        if with_timeout(Duration::from_millis(10), self.pio_instance.sm0.tx().wait_push(tx_data)).await.is_err() {
            return Err(DshotError::TelemetryTimeout);
        }

        let rx_data = with_timeout(Duration::from_micros(500), self.pio_instance.sm0.rx().wait_pull())
            .await
            .map_err(|_| DshotError::TelemetryTimeout)?;

        if rx_data == 0 {
            return Err(DshotError::TelemetryTimeout);
        }

        Ok(rx_data)
    }
}
