#![cfg(feature = "rp")]

use embassy_rp::{
    Peri, clocks,
    gpio::Pull,
    interrupt::typelevel::Binding,
    pio::{
        Config as PioConfig, Direction, FifoJoin, Instance, InterruptHandler, Pio, PioPin, ShiftConfig, ShiftDirection,
        StateMachine, program::pio_asm,
    },
};
use embassy_time::{Duration, with_timeout};

use super::clock_divider::bidir_pio_clock_divider;
use crate::dshot::{DshotError, Protocol};

// Bidirectional Dshot PIO program based on pico-bidir-dshot reference.
//
// Program layout (offsets from program_origin):
//   program_origin + 0: push block     (pushes previous RX data)
//   program_origin + 1: set pindirs, 1 (pin as output)
//   program_origin + 2: pull block     (waits for TX frame — idle position)
//
// TX Phase (32 cycles per bit):
//   14 cycles LOW, 14 cycles data bit (inverted), 11 cycles HIGH, 1 jmp
//
// RX Phase (pulse-width measurement):
//   Wait for falling edge, measure pulse widths using counting loops.
//   21 GCR-encoded bits decoded to 16-bit telemetry + CRC.
//   Tight 2-cycle wait loop matches reference implementation.
macro_rules! dshot_bidirectional {
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
/// Bidirectional `Dshot` PIO driver for 4 ESCs with telemetry.
///
/// Supports `Dshot150`, `Dshot300`, `Dshot600`. `Dshot1200` is not supported
/// (panics at construction).
#[allow(unused)]
#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct PioBidirectionalQuadDshot<'a, PIO: Instance> {
    pio_instance: Pio<'a, PIO>,
    // _pin: Pin<'a, PIO>,
    program_origin: u8,
}

impl<'a, PIO: Instance> PioBidirectionalQuadDshot<'a, PIO> {
    /// # Panics
    ///
    /// Panics if `speed` is `Protocol::Dshot1200`.
    #[allow(unused)]
    pub fn new(
        pio: Peri<'a, PIO>,
        irq: impl Binding<PIO::Interrupt, InterruptHandler<PIO>>,
        pin0: Peri<'a, impl PioPin + 'a>,
        pin1: Peri<'a, impl PioPin + 'a>,
        pin2: Peri<'a, impl PioPin + 'a>,
        pin3: Peri<'a, impl PioPin + 'a>,
        protocol: Protocol,
    ) -> Self {
        assert!(!matches!(protocol, Protocol::Dshot1200), "Dshot1200 is not supported for bidirectional mode");

        let mut pio = Pio::new(pio, irq);

        let mut pin0 = pio.common.make_pio_pin(pin0);
        pin0.set_pull(Pull::Up);
        let mut pin1 = pio.common.make_pio_pin(pin1);
        pin1.set_pull(Pull::Up);
        let mut pin2 = pio.common.make_pio_pin(pin2);
        pin2.set_pull(Pull::Up);
        let mut pin3 = pio.common.make_pio_pin(pin3);
        pin3.set_pull(Pull::Up);

        let prg = dshot_bidirectional!();

        let program = pio.common.load_program(&prg.program);
        let program_origin = program.origin;
        let clock_divider = bidir_pio_clock_divider(protocol, clocks::clk_sys_freq());

        let mut pio_config = PioConfig::default();
        pio_config.use_program(&program, &[]);

        pio_config.clock_divider = clock_divider;

        pio_config.shift_out = ShiftConfig { auto_fill: false, direction: ShiftDirection::Left, threshold: 32 };
        pio_config.shift_in = ShiftConfig { auto_fill: false, direction: ShiftDirection::Left, threshold: 32 };

        pio_config.fifo_join = FifoJoin::Duplex;

        // PIN 0
        pio_config.set_jmp_pin(&pin0);
        pio_config.set_set_pins(&[&pin0]);
        pio_config.set_out_pins(&[&pin0]);
        pio_config.set_in_pins(&[&pin0]);

        pio.sm0.set_config(&pio_config);
        pio.sm0.set_pin_dirs(Direction::Out, &[&pin0]);
        pio.sm0.restart();
        pio.sm0.set_enable(true);
        pio.sm0.set_clock_divider(clock_divider);

        // PIN 1
        pio_config.set_jmp_pin(&pin1);
        pio_config.set_set_pins(&[&pin1]);
        pio_config.set_out_pins(&[&pin1]);
        pio_config.set_in_pins(&[&pin1]);

        pio.sm1.set_config(&pio_config);
        pio.sm1.set_pin_dirs(Direction::Out, &[&pin1]);
        pio.sm1.restart();
        pio.sm1.set_enable(true);
        pio.sm1.set_clock_divider(clock_divider);

        // PIN 2
        pio_config.set_jmp_pin(&pin2);
        pio_config.set_set_pins(&[&pin2]);
        pio_config.set_out_pins(&[&pin2]);
        pio_config.set_in_pins(&[&pin2]);

        pio.sm2.set_config(&pio_config);
        pio.sm2.set_pin_dirs(Direction::Out, &[&pin2]);
        pio.sm2.restart();
        pio.sm2.set_enable(true);
        pio.sm2.set_clock_divider(clock_divider);

        // PIN 3
        pio_config.set_jmp_pin(&pin3);
        pio_config.set_set_pins(&[&pin3]);
        pio_config.set_out_pins(&[&pin3]);
        pio_config.set_in_pins(&[&pin3]);

        pio.sm3.set_config(&pio_config);
        pio.sm3.set_pin_dirs(Direction::Out, &[&pin3]);
        pio.sm3.restart();
        pio.sm3.set_enable(true);
        pio.sm3.set_clock_divider(clock_divider);
        Self { pio_instance: pio, program_origin }
    }

    /// Reset PIO program counter to the pull-block address.
    fn reset_program_counter<const SM: usize>(sm: &mut StateMachine<'_, PIO, SM>, program_origin: u8) {
        // program_origin + 0: push block     (pushes previous RX data)
        // program_origin + 1: set pindirs, 1 (pin as output)
        // program_origin + 2: pull block     (waits for TX frame — idle position)
        let pull_block_address = program_origin + 2;

        if sm.get_addr() != pull_block_address {
            // Clear ISR to discard any partial RX data from an interrupted frame.
            const MOV_ISR_NULL: u16 = 0b101_00000_110_00_011;
            unsafe { sm.exec_instr(MOV_ISR_NULL) };

            // Construct unconditional JMP instruction: opcode 000, no delay, condition 000
            let jmp_instruction = u16::from(program_origin + 1) & 0x1F;
            unsafe { sm.exec_instr(jmp_instruction) };
        }
    }

    /// Send a frame and return the raw rx value.
    /// wait_push timeout  → TxTimeout
    /// wait_pull timeout  → TelemetryTimeout
    /// rx_data == 0       → InvalidTelemetry
    /// # Errors
    async fn send_frame_and_receive<const SM: usize>(
        sm: &mut StateMachine<'_, PIO, SM>,
        program_origin: u8,
        frame: u16,
    ) -> Result<u32, DshotError> {
        // Clear any existing rx data
        while sm.rx().try_pull().is_some() {}
        Self::reset_program_counter(sm, program_origin);

        let frame_inverted = u32::from(!frame);
        // TODO: check 10ms timeout ins PIO `send_and_receive`.
        with_timeout(Duration::from_millis(10), sm.tx().wait_push(frame_inverted)).await.map_err(|_| DshotError::TxTimeout)?;

        let rx_data = with_timeout(Duration::from_micros(500), sm.rx().wait_pull())
            .await
            .map_err(|_| DshotError::TelemetryTimeout)?;

        Ok(rx_data)
    }

    #[allow(unused)]
    pub async fn send_frame<const SM: usize>(sm: &mut StateMachine<'_, PIO, SM>, program_origin: u8, frame: u16) {
        while sm.rx().try_pull().is_some() {}
        Self::reset_program_counter(sm, program_origin);

        // bidirectional dshot inverts frame
        let frame_inverted = u32::from(!frame);
        sm.tx().wait_push(frame_inverted).await;
    }

    #[allow(unused)]
    pub fn send_frame_blocking<const SM: usize>(sm: &mut StateMachine<'_, PIO, SM>, program_origin: u8, frame: u16) {
        while sm.rx().try_pull().is_some() {}
        Self::reset_program_counter(sm, program_origin);

        // bidirectional dshot inverts frame
        let frame_inverted = u32::from(!frame);
        sm.tx().push(frame_inverted);
    }

    pub async fn send_frame_and_receive_sm0(&mut self, frame: u16) -> Result<u32, DshotError> {
        Self::send_frame_and_receive(&mut self.pio_instance.sm0, self.program_origin, frame).await
    }

    pub async fn send_frame_and_receive_sm1(&mut self, frame: u16) -> Result<u32, DshotError> {
        Self::send_frame_and_receive(&mut self.pio_instance.sm1, self.program_origin, frame).await
    }

    pub async fn send_frame_and_receive_sm2(&mut self, frame: u16) -> Result<u32, DshotError> {
        Self::send_frame_and_receive(&mut self.pio_instance.sm2, self.program_origin, frame).await
    }

    pub async fn send_frame_and_receive_sm3(&mut self, frame: u16) -> Result<u32, DshotError> {
        Self::send_frame_and_receive(&mut self.pio_instance.sm3, self.program_origin, frame).await
    }
}
