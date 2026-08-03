use core::ops::{Deref, DerefMut};

use signal_filters::{BiquadFilterVector3f32, Pt1Filterf32};
use vqm::Vector3f32;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

//use defmt::debug;
//use embassy_time::{Instant, Timer};
use crate::{
    mixer::MAX_SUPPORTED_MOTOR_COUNT,
    rpm_notch_filters_state_machine::{
        FUNDAMENTAL, RPM_FILTER_HARMONICS_COUNT, RpmFilterMotorState, RpmFilterMotorStates, SECOND_HARMONIC, State,
        THIRD_HARMONIC,
    },
};

/// Array of motor rotation frequencies, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorFrequencies(pub [f32; MAX_SUPPORTED_MOTOR_COUNT]);

impl MotorFrequencies {
    #[must_use]
    pub const fn new() -> Self {
        Self([0.0; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Default for MotorFrequencies {
    fn default() -> Self {
        Self::new()
    }
}

impl Deref for MotorFrequencies {
    type Target = [f32; MAX_SUPPORTED_MOTOR_COUNT];
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MotorFrequencies {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct RpmNotchFilterBankConfig {
    /// range in which notch filters fade down to `min_hz`.
    pub rpm_filter_fade_range_hz: u16,
    /// Q of the notch filters * 100.
    pub rpm_filter_q_x100: u16,
    /// LPF cutoff (from motor rpm converted to Hz).
    pub rpm_filter_lpf_hz: u16,
    /// weight as a percentage for each harmonic.
    pub rpm_filter_weights_x100: [u16; RPM_FILTER_HARMONICS_COUNT],
    /// minimum notch frequency for fundamental harmonic.
    pub rpm_filter_min_hz: u8,
    pub motor_count: u8,
}

impl RpmNotchFilterBankConfig {
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rpm_filter_fade_range_hz: 50,
            rpm_filter_q_x100: 500,
            rpm_filter_lpf_hz: 150,
            rpm_filter_weights_x100: [100, 0, 100],
            #[allow(clippy::cast_possible_truncation)]
            rpm_filter_min_hz: 100,
            motor_count: 4, // default to using 4 motors
        }
    }
}

impl Default for RpmNotchFilterBankConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmNotchFilterFrequencies {
    pub motor_frequencies_hz: MotorFrequencies,
    pub min_hz: f32,
    pub max_hz: f32,
    pub fade_range_hz: f32,
}

impl RpmNotchFilterFrequencies {
    const DEFAULT_FADE_RANGE: f32 = 50.0;

    #[must_use]
    pub const fn with_fade_range_hz(fade_range_hz: f32) -> Self {
        Self { motor_frequencies_hz: MotorFrequencies::new(), min_hz: 100.0, max_hz: 0.0, fade_range_hz }
    }

    #[must_use]
    pub const fn new() -> Self {
        Self::with_fade_range_hz(Self::DEFAULT_FADE_RANGE)
    }
}

impl Default for RpmNotchFilterFrequencies {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NotchFilters(pub [[BiquadFilterVector3f32; RPM_FILTER_HARMONICS_COUNT]; MAX_SUPPORTED_MOTOR_COUNT]);

impl NotchFilters {
    pub const fn new() -> Self {
        Self([[BiquadFilterVector3f32::new(); RPM_FILTER_HARMONICS_COUNT]; MAX_SUPPORTED_MOTOR_COUNT])
    }
}

impl Deref for NotchFilters {
    type Target = [[BiquadFilterVector3f32; RPM_FILTER_HARMONICS_COUNT]; MAX_SUPPORTED_MOTOR_COUNT];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for NotchFilters {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmNotchFilterBankContext {
    pub motor_rpm_filters: [Pt1Filterf32; MAX_SUPPORTED_MOTOR_COUNT],
    pub notch_filters: NotchFilters,
    pub motor_states: RpmFilterMotorStates,
    pub weights: [f32; RPM_FILTER_HARMONICS_COUNT],
}

impl RpmNotchFilterBankContext {
    pub const fn new() -> Self {
        Self {
            motor_rpm_filters: [Pt1Filterf32::new(); MAX_SUPPORTED_MOTOR_COUNT],
            notch_filters: NotchFilters::new(),
            motor_states: [RpmFilterMotorState::new(); MAX_SUPPORTED_MOTOR_COUNT],
            weights: [0.0; RPM_FILTER_HARMONICS_COUNT],
        }
    }
}

impl Default for RpmNotchFilterBankContext {
    fn default() -> Self {
        Self::new()
    }
}

/// Bank of `RpmFilters`, one for each motor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmNotchFilterBank {
    config: RpmNotchFilterBankConfig,
    frequencies: RpmNotchFilterFrequencies,
    state: State,
    ctx: RpmNotchFilterBankContext,
    // all the notch filters have the same q and looptime
    looptime_seconds: f32,
    q: f32,
    rpm_filter_harmonics_count: usize,
}

impl RpmNotchFilterBank {
    pub const DEFAULT_LOOPTIME_SECONDS: f32 = 0.001;

    #[must_use]
    pub fn new(config: RpmNotchFilterBankConfig, looptime_seconds: f32) -> Self {
        let mut this = Self {
            config,
            frequencies: RpmNotchFilterFrequencies::new(),
            state: State::new(),
            ctx: RpmNotchFilterBankContext::new(),
            looptime_seconds,
            q: 0.0,
            rpm_filter_harmonics_count: 0,
        };
        this.set_config(config);
        this
    }
}

impl Default for RpmNotchFilterBank {
    fn default() -> Self {
        Self::new(RpmNotchFilterBankConfig::new(), Self::DEFAULT_LOOPTIME_SECONDS)
    }
}

impl RpmNotchFilterBank {
    #[must_use]
    pub fn rpm_filter_harmonics_count(&self) -> usize {
        self.rpm_filter_harmonics_count
    }

    pub fn set_config(&mut self, config: RpmNotchFilterBankConfig) {
        self.config = config;

        self.q = f32::from(config.rpm_filter_q_x100) * 0.01;

        self.state = State::Stopped;
        // just under  Nyquist frequency (ie just under half sampling rate)
        // for 8kHz loop this is 3840Hz
        self.frequencies.max_hz = 480_000.0 / self.looptime_seconds;

        // pre-calculate frequencies for speed in iteration steps
        self.frequencies.min_hz = f32::from(config.rpm_filter_min_hz);
        self.frequencies.fade_range_hz = f32::from(config.rpm_filter_fade_range_hz);

        self.rpm_filter_harmonics_count = 0;
        for harmonic in 0..RPM_FILTER_HARMONICS_COUNT {
            if config.rpm_filter_weights_x100[harmonic] != 0 {
                self.rpm_filter_harmonics_count += 1;
            }
            self.ctx.weights[harmonic] = f32::from(config.rpm_filter_weights_x100[harmonic]);
            #[allow(clippy::cast_precision_loss)]
            for motor in 0..(config.motor_count as usize).min(MAX_SUPPORTED_MOTOR_COUNT) {
                self.ctx.notch_filters[motor][harmonic].init_notch(
                    self.frequencies.min_hz * (harmonic + 1) as f32,
                    self.looptime_seconds,
                    self.q,
                );
            }
        }

        if config.rpm_filter_lpf_hz == 0 {
            for mut rpm_filter in self.ctx.motor_rpm_filters {
                rpm_filter.set_to_passthrough();
            }
        } else {
            for mut rpm_filter in self.ctx.motor_rpm_filters {
                rpm_filter.set_cutoff_frequency_and_reset(f32::from(config.rpm_filter_lpf_hz), self.looptime_seconds);
            }
        }
    }

    /// Start the filter state machine
    /// This is called from `MotorMixer::output_to_motors` and so needs to be FAST.
    #[inline]
    pub fn start_updating_filter_frequencies(&mut self, motor_frequencies_hz: MotorFrequencies) {
        if self.config.rpm_filter_lpf_hz == 0 {
            return;
        }
        self.frequencies.motor_frequencies_hz = motor_frequencies_hz;
        self.state.start();
    }

    #[inline]
    pub fn update_filter_frequencies_step(&mut self) {
        self.state.update_filter_frequencies_step(&self.config, self.frequencies, &mut self.ctx);
    }

    /// Apply the notch filters for all selected harmonics for the given motor.
    #[inline]
    pub fn update_notch_filters_for_motor(
        ctx: &mut RpmNotchFilterBankContext,
        input: Vector3f32,
        motor_index: usize,
    ) -> Vector3f32 {
        let mut ret = ctx.notch_filters[motor_index][FUNDAMENTAL].update_notch_weighted(input);

        if ctx.weights[SECOND_HARMONIC] != 0.0 {
            ret = ctx.notch_filters[motor_index][SECOND_HARMONIC].update_notch_weighted(ret);
        }
        if ctx.weights[THIRD_HARMONIC] != 0.0 {
            ret = ctx.notch_filters[motor_index][THIRD_HARMONIC].update_notch_weighted(ret);
        }
        ret
    }
}

pub trait RpmNotchFilters {
    fn common(&self) -> &RpmNotchFilterBank;
    fn common_mut(&mut self) -> &mut RpmNotchFilterBank;
    fn config(&self) -> &RpmNotchFilterBankConfig;

    fn update(&mut self, value: Vector3f32, motor_index: usize) -> Vector3f32;
}

impl RpmNotchFilters for RpmNotchFilterBank {
    fn common(&self) -> &RpmNotchFilterBank {
        self
    }
    fn common_mut(&mut self) -> &mut RpmNotchFilterBank {
        self
    }
    fn config(&self) -> &RpmNotchFilterBankConfig {
        &self.common().config
    }

    fn update(&mut self, value: Vector3f32, _motor_index: usize) -> Vector3f32 {
        value
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn _is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}
    #[cfg(feature = "serde")]
    fn is_config<
        T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq + Serialize + for<'a> Deserialize<'a>,
    >() {
    }

    #[test]
    fn normal_types() {
        is_full::<RpmNotchFilterBankConfig>();
        #[cfg(feature = "serde")]
        is_config::<RpmNotchFilterBankConfig>();
        is_full::<RpmNotchFilterFrequencies>();
        is_full::<RpmNotchFilterBankContext>();
        is_full::<RpmNotchFilterBank>();
    }
    #[test]
    fn test_new() {
        let config = RpmNotchFilterBankConfig::new();
        assert_eq!(50, config.rpm_filter_fade_range_hz);
    }
}
