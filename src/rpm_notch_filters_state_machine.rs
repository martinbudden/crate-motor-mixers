use signal_filters::SignalFilter;
#[allow(unused)]
use vqm::TrigonometricMethods; // Required for .sin_cos()

use crate::{
    mixer::MAX_MOTOR_COUNT,
    rpm_notch_filters::{RpmNotchFilterBankConfig, RpmNotchFilterBankContext, RpmNotchFilterFrequencies},
};

pub const RPM_FILTER_HARMONICS_COUNT: usize = 3;

pub const FUNDAMENTAL: usize = 0;
pub const SECOND_HARMONIC: usize = 1;
pub const THIRD_HARMONIC: usize = 2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RpmFilterMotorState {
    frequency_hz_unclamped: f32,
    weight_multiplier: f32,
    // no need to cache omega, since we are caching sin_omega and cos_omega instead
    sin_omega: f32,
    cos_omega: f32,
}

impl RpmFilterMotorState {
    pub const fn new() -> Self {
        Self { frequency_hz_unclamped: 0.0, weight_multiplier: 0.0, sin_omega: 0.0, cos_omega: 0.0 }
    }
}

impl Default for RpmFilterMotorState {
    fn default() -> Self {
        Self::new()
    }
}

pub type RpmFilterMotorStates = [RpmFilterMotorState; MAX_MOTOR_COUNT];

// NOTE: I have considered the typestate pattern for the state machine and have elected not to use it.
/// State enum to drive state machine.
#[repr(u8)]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum State {
    #[default]
    Stopped,
    Fundamental(usize),
    SecondHarmonic(usize),
    ThirdHarmonic(usize),
}

impl State {
    pub const fn new() -> Self {
        Self::Stopped
    }
}

/*impl From<u8> for State {
    fn from(value: u8) -> Self {
        match value {
            0 => State::Stopped,
            1 => State::Fundamental(0),
            2 => Self::SecondHarmonic(0),
            3 => Self::ThirdHarmonic(0),
            _ => State::Stopped,
        }
    }
}*/

/// State machine to set notch frequencies for the rpm filter bank.
///
/// On each iteration of the state machine, one harmonic is set for one motor.
///
/// For a tri-bladed quadcopter (which will typically filter the fundamental frequency and first harmonic)
/// 8 iterations of the state machine are required to set all the notch filters.
impl State {
    /// External trigger to start the state machine.
    pub fn start(&mut self) {
        // TODO: consider if we want to restart from the beginning if start is called before state machine has stopped
        if let State::Stopped = self {
            *self = State::Fundamental(0);
        }
    }

    /// Perform one step of the state machine.<br>
    /// The state machine sets notch filter for one harmonic of one motor on each iteration.
    /// This is called from `MotorMixer::rpm_filter_set_frequency_hz_iteration_step` and so needs to be FAST.
    pub fn update(
        &mut self,
        config: &RpmNotchFilterBankConfig,
        frequencies: RpmNotchFilterFrequencies,
        ctx: &mut RpmNotchFilterBankContext,
    ) {
        *self = match core::mem::take(self) {
            State::Stopped => {
                // If we are stopped, we stay stopped until start() is called
                // Explicitly setting *self = State::Stopped defends against a change in the default.
                State::Stopped
            }
            State::Fundamental(motor_index) => {
                let frequency_hz = frequencies.motor_frequencies_hz[motor_index];
                let frequency_hz_unclamped = ctx.motor_rpm_filters[motor_index].update(frequency_hz);
                let frequency_hz = frequency_hz_unclamped.clamp(frequencies.min_hz, frequencies.max_hz);
                ctx.motor_states[motor_index].frequency_hz_unclamped = frequency_hz_unclamped;

                let margin_frequency_hz = frequency_hz - frequencies.min_hz;
                let weight_multiplier = if margin_frequency_hz < frequencies.fade_range_hz {
                    margin_frequency_hz / frequencies.fade_range_hz
                } else {
                    1.0
                };
                ctx.motor_states[motor_index].weight_multiplier = weight_multiplier;

                // omega = frequency * _2PiLoopTimeSeconds
                // max_frequency < 0.5 / looptime_seconds
                // max_omega = (0.5 / looptime_seconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
                // so omega is in range [0, PI]
                let omega = ctx.notch_filters[motor_index][FUNDAMENTAL].calculate_omega(frequency_hz);

                // Calculate sin(omega) and cos(omega) and cache their values.
                // The second and third harmonics use trigonometric identities to calculate sin(2*omega), sin(3*omega) etc,
                // this is significantly faster than calling sin_cos() again.
                (ctx.motor_states[motor_index].sin_omega, ctx.motor_states[motor_index].cos_omega) = omega.sin_cos();

                ctx.notch_filters[motor_index][FUNDAMENTAL].set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    ctx.motor_states[motor_index].sin_omega,
                    ctx.motor_states[motor_index].cos_omega,
                    ctx.weights[FUNDAMENTAL] * weight_multiplier,
                );
                // move onto the next state
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    if config.rpm_filter_harmonics >= 2 {
                        if config.rpm_filter_weights_x100[SECOND_HARMONIC] != 0 {
                            State::SecondHarmonic(0)
                        } else if config.rpm_filter_harmonics >= 3
                            && config.rpm_filter_weights_x100[THIRD_HARMONIC] != 0
                        {
                            State::ThirdHarmonic(0)
                        } else {
                            State::Stopped
                        }
                    } else {
                        State::Stopped
                    }
                } else {
                    State::Fundamental(motor_index + 1)
                }
            }
            State::SecondHarmonic(motor_index) => {
                let motor_state = ctx.motor_states[motor_index];
                if motor_state.frequency_hz_unclamped > frequencies.half_of_max_hz {
                    // ie 2.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the second harmonic if it is above the Nyquist frequency
                    ctx.weights[SECOND_HARMONIC] = 0.0;
                } else {
                    ctx.weights[SECOND_HARMONIC] = f32::from(config.rpm_filter_weights_x100[SECOND_HARMONIC]) * 0.01;
                    // sin(2θ) = 2 * sin(θ) * cos(θ)
                    // cos(2θ) = 2 * cos^2(θ) - 1
                    let sin_2_omega = motor_state.sin_omega * motor_state.cos_omega * 2.0;
                    let cos_2_omega = motor_state.cos_omega * motor_state.cos_omega * 2.0 - 1.0;
                    ctx.notch_filters[motor_index][SECOND_HARMONIC]
                        .set_notch_frequency_weighted_from_sin_cos_assuming_q(
                            sin_2_omega,
                            cos_2_omega,
                            ctx.weights[SECOND_HARMONIC] * ctx.motor_states[motor_index].weight_multiplier,
                        );
                }
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    if config.rpm_filter_harmonics >= 3 && config.rpm_filter_weights_x100[THIRD_HARMONIC] != 0 {
                        State::ThirdHarmonic(0)
                    } else {
                        State::Stopped
                    }
                } else {
                    State::Fundamental(motor_index + 1)
                }
            }
            State::ThirdHarmonic(motor_index) => {
                let motor_state = ctx.motor_states[motor_index];
                if motor_state.frequency_hz_unclamped > frequencies.third_of_max_hz {
                    // ie 3.0 * frequency_hz_unclamped > _max_frequency_hz
                    // no point filtering the third harmonic if it is above the Nyquist frequency
                    ctx.weights[THIRD_HARMONIC] = 0.0;
                } else {
                    ctx.weights[THIRD_HARMONIC] = f32::from(config.rpm_filter_weights_x100[THIRD_HARMONIC]) * 0.01;
                    // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
                    //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
                    //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
                    //         = sin(θ) * ( 4 * cos^2(θ) - 1)
                    // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
                    //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
                    let four_cos_squared_omega = 4.0 * motor_state.cos_omega * motor_state.cos_omega;
                    let sin_3_omega = motor_state.sin_omega * (four_cos_squared_omega - 1.0);
                    let cos_3_omega = motor_state.cos_omega * (four_cos_squared_omega - 3.0);
                    ctx.notch_filters[motor_index][THIRD_HARMONIC]
                        .set_notch_frequency_weighted_from_sin_cos_assuming_q(
                            sin_3_omega,
                            cos_3_omega,
                            ctx.weights[THIRD_HARMONIC] * motor_state.weight_multiplier,
                        );
                }
                if motor_index == config.motor_count as usize {
                    // we have set the notch frequency for all motors, so we are finished
                    State::Stopped
                } else {
                    State::Fundamental(motor_index + 1)
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<State>();
        is_full::<RpmFilterMotorState>();
    }
    #[test]
    fn test_new() {
        let config = RpmNotchFilterBankConfig::new();
        assert_eq!(50, config.rpm_filter_fade_range_hz);
    }
}
