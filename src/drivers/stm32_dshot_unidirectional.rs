#![cfg(feature = "stm32")]

use embassy_stm32::{
    Peri, PeripheralType,
    dma::{Channel, ChannelInstance, TransferOptions},
    gpio::{AnyPin, Level, Output, Pin, Speed},
    interrupt::typelevel::Binding,
    timer::{
        BasicInstance, BasicNoCr2Instance, UpDma,
        low_level::{RoundTo, Timer},
    },
};
use embassy_time::Timer as EmbassyTimer;

use dshot_codec::{DshotCommand, DshotCommandFrame, DshotMotorMasks, DshotTiming, DshotWaveform};

use super::{DshotCommands, MotorFrequencies, MotorOutputs};

#[cfg(feature = "dshot_t1")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM1>;
#[cfg(feature = "dshot_t2")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM2>;
#[cfg(feature = "dshot_t3")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM3>;
#[cfg(feature = "dshot_t4")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM4>;
#[cfg(feature = "dshot_t5")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM5>;
#[cfg(feature = "dshot_t6")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM6>;
#[cfg(feature = "dshot_t7")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM7>;
#[cfg(feature = "dshot_t8")]
pub type MotorDriverDshot = MotorDriverDshotUnidirectional<'static, embassy_stm32::peripherals::TIM8>;

#[allow(missing_debug_implementations, missing_copy_implementations)]
pub struct MotorDriverDshotUnidirectional<'d, T: BasicNoCr2Instance> {
    timer: Timer<'d, T>,
    dma: Channel<'d>,

    dma_request: embassy_stm32::dma::Request,

    _motor1: Output<'d>,
    _motor2: Output<'d>,
    _motor3: Output<'d>,
    _motor4: Output<'d>,

    bsrr: *mut u32,

    masks: DshotMotorMasks,
    timing: DshotTiming,
    waveform: &'d mut DshotWaveform,
    motor_frequencies: MotorFrequencies,
    erpm_to_hz: f32,
}

// SAFETY:
// `bsrr` points to a memory-mapped GPIO BSRR register belonging to the
// GPIO port containing the four motor pins. The address is obtained from
// Embassy's GPIO peripheral metadata rather than being supplied by the
// caller.
//
// The driver owns the four `Output` values, so no other instance of the
// driver can access those pins through Embassy's GPIO ownership model.
//
// The BSRR register is a hardware peripheral register and does not contain
// Rust-managed memory.
//
// Therefore moving the driver between executor contexts/threads does not
// invalidate the pointer or violate Rust's ownership rules.
unsafe impl<T: BasicNoCr2Instance> Send for MotorDriverDshotUnidirectional<'_, T> {}

const MOTOR_COUNT: usize = 4;

impl<'d, T> MotorDriverDshotUnidirectional<'d, T>
where
    T: BasicNoCr2Instance + BasicInstance,
{
    pub fn new<D, P1, P2, P3, P4>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        irq: impl Binding<D::Interrupt, embassy_stm32::dma::InterruptHandler<D>> + 'd,
        motor1: Peri<'d, P1>,
        motor2: Peri<'d, P2>,
        motor3: Peri<'d, P3>,
        motor4: Peri<'d, P4>,
        waveform: &'d mut DshotWaveform,
        dshot_speed: dshot_codec::DshotSpeed,
        motor_pole_count: u8,
    ) -> Self
    where
        D: ChannelInstance + UpDma<T>,
        P1: PeripheralType,
        P2: PeripheralType,
        P3: PeripheralType,
        P4: PeripheralType,
        AnyPin: From<P1>,
        AnyPin: From<P2>,
        AnyPin: From<P3>,
        AnyPin: From<P4>,
    {
        const SECONDS_PER_MINUTE: f32 = 60.0;

        let motor1: Peri<'d, AnyPin> = motor1.into();
        let motor2: Peri<'d, AnyPin> = motor2.into();
        let motor3: Peri<'d, AnyPin> = motor3.into();
        let motor4: Peri<'d, AnyPin> = motor4.into();

        let port = motor1.port();

        assert_eq!(motor2.port(), port);
        assert_eq!(motor3.port(), port);
        assert_eq!(motor4.port(), port);

        let bsrr = motor1.block().bsrr().as_ptr() as *mut u32;
        let masks = DshotMotorMasks::new(1 << motor1.pin(), 1 << motor2.pin(), 1 << motor3.pin(), 1 << motor4.pin());

        // Obtain the timer's DMA request before erasing D into
        // Embassy's runtime DMA Channel type.
        let dma_request = <D as UpDma<T>>::request(&dma);

        let dma = Channel::new(dma, irq);
        let motor1 = Output::new(motor1, Level::Low, Speed::VeryHigh);
        let motor2 = Output::new(motor2, Level::Low, Speed::VeryHigh);
        let motor3 = Output::new(motor3, Level::Low, Speed::VeryHigh);
        let motor4 = Output::new(motor4, Level::Low, Speed::VeryHigh);

        let timer = Timer::new(timer);

        let timing = DshotTiming::new(dshot_speed);
        Self {
            timer,
            dma,
            dma_request,

            _motor1: motor1,
            _motor2: motor2,
            _motor3: motor3,
            _motor4: motor4,
            bsrr,
            masks,
            timing,
            waveform,
            motor_frequencies: MotorFrequencies::new(),
            erpm_to_hz: 2.0 * (100.0 / SECONDS_PER_MINUTE) / f32::from(motor_pole_count),
        }
    }

    #[inline]
    pub async fn send_command_frames(&mut self, frames: [DshotCommandFrame; MOTOR_COUNT]) {
        let packets = [frames[0].raw(), frames[1].raw(), frames[2].raw(), frames[3].raw()];
        self.send_packets(packets).await;
    }

    /// Send four already-encoded DShot packets.
    ///
    /// The packets should be the 16-bit DShot words produced by `dshot-codec`.
    #[inline]
    pub async fn send_packets(&mut self, packets: [u16; MOTOR_COUNT]) {
        self.waveform.encode(packets, self.masks, self.timing);
        self.send().await;
    }

    pub async fn send(&mut self) {
        let tick_rate = self.timing.tick_rate();

        // Configure the timer while DMA requests are disabled.
        // RoundTo::Slower means we never run the DShot timing faster than requested.
        self.timer.set_frequency(embassy_stm32::time::Hertz(tick_rate), RoundTo::Slower);

        self.timer.stop();
        self.timer.reset();
        // Applying the new PSC/ARR values requires an update event.
        // `generate_update_event()` temporarily changes URS so this
        // update does NOT produce a DMA request.
        self.timer.generate_update_event();

        // Arm the DMA first.
        // write() starts the DMA stream, but it cannot perform a
        // transfer until TIMx generates an update request.
        let transfer = unsafe {
            self.dma.write(self.dma_request, self.waveform.as_slice(), self.bsrr, TransferOptions::default())
        };

        //  Now allow timer update events to generate DMA requests.
        self.timer.enable_update_dma(true);
        // The first DMA transfer occurs on the first timer update.
        self.timer.start();

        // Wait until all 128 BSRR writes have completed.
        transfer.await;

        // Stop the timer before disabling UDE.
        // This guarantees that no additional update request can arrive
        // while we're tearing the transfer down.
        self.timer.stop();
        self.timer.enable_update_dma(false);

        // The final waveform entry should already have driven all active outputs low.
        // Explicitly force them low as a safety measure anyway.
        unsafe {
            core::ptr::write_volatile(self.bsrr, self.masks.all() << 16);
        }
    }
}

impl<'d, T> MotorDriverDshotUnidirectional<'d, T>
where
    T: BasicNoCr2Instance + BasicInstance,
{
    pub async fn write_to_motors(&mut self, outputs: MotorOutputs) {
        let commands_frames = [
            DshotCommandFrame::from_throttle_unidirectional(outputs[0]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[1]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[2]),
            DshotCommandFrame::from_throttle_unidirectional(outputs[3]),
        ];
        self.send_command_frames(commands_frames).await;
    }

    pub async fn write_commands_to_motors(&mut self, commands: DshotCommands) {
        let commands_frames = [
            DshotCommandFrame::from_command(commands[0]),
            DshotCommandFrame::from_command(commands[1]),
            DshotCommandFrame::from_command(commands[2]),
            DshotCommandFrame::from_command(commands[3]),
        ];
        self.send_command_frames(commands_frames).await;
    }

    pub async fn write_command_to_all_motors(&mut self, command: DshotCommand) {
        let commands_frames = [
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
            DshotCommandFrame::from_command(command),
        ];
        for _ in 0..command.repetitions_required() {
            self.send_command_frames(commands_frames).await;
            EmbassyTimer::after_micros(u64::from(command.delay_required_us())).await;
        }
    }

    #[allow(clippy::unnecessary_wraps)]
    #[must_use]
    pub fn motor_frequencies(&self) -> Option<MotorFrequencies> {
        _ = self.erpm_to_hz;
        Some(self.motor_frequencies)
    }
}
