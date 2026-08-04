use signal_filters::SignalFilter;
#[allow(unused)]
use vqm::TrigonometricMethods; // Required for .sin_cos()

use crate::rpm_notch_filters::{RpmNotchFilterBankConfig, RpmNotchFilterBankContext, RpmNotchFilterFrequencies};

pub const RPM_FILTER_HARMONICS_COUNT: usize = 3;

pub const FUNDAMENTAL: usize = 0;
pub const SECOND_HARMONIC: usize = 1;
pub const THIRD_HARMONIC: usize = 2;

// NOTE: I have considered the typestate pattern for the state machine and have elected not to use it.
/// State enum to drive state machine.
#[repr(u8)]
#[derive(Clone, Copy, Default, Debug, PartialEq)]
pub enum State {
    #[default]
    Stopped,
    Fundamental {
        motor_index: usize,
    },
    SecondHarmonic {
        motor_index: usize,
    },
    ThirdHarmonic {
        motor_index: usize,
    },
}

impl State {
    pub const fn new() -> Self {
        Self::Stopped
    }
}

/// State machine to set notch frequencies for the rpm filter bank.
///
/// On each iteration of the state machine, one harmonic is set for one motor.
///
/// For a tri-bladed quadcopter (which will typically filter the fundamental frequency and third harmonic)
/// 8 iterations of the state machine are required to set all the notch filters.
impl State {
    /// External trigger to start the state machine.
    pub fn start(&mut self) {
        // TODO: consider if we want to restart from the beginning if start is called before state machine has stopped
        if let State::Stopped = self {
            *self = State::Fundamental { motor_index: 0 };
        }
    }

    /// Perform one step of the state machine.<br>
    /// The state machine sets notch filter for one harmonic of one motor on each iteration.
    /// This is called from `MotorMixer::rpm_filter_set_frequency_hz_iteration_step` and so needs to be FAST.
    pub fn update_filter_frequencies_step(
        &mut self,
        config: &RpmNotchFilterBankConfig,
        frequencies: RpmNotchFilterFrequencies,
        ctx: &mut RpmNotchFilterBankContext,
    ) {
        let motor_count = usize::from(config.motor_count);
        *self = match core::mem::take(self) {
            State::Stopped => {
                // If we are stopped, we stay stopped until start() is called
                // Explicitly setting *self = State::Stopped defends against a change in the default.
                State::Stopped
            }
            State::Fundamental { motor_index } => {
                let motor_state = &mut ctx.motor_states[motor_index];
                let notch_filter = &mut ctx.notch_filters[motor_index][FUNDAMENTAL];

                motor_state.frequency_hz =
                    ctx.motor_rpm_filters[motor_index].update(frequencies.motor_frequencies_hz[motor_index]);
                let frequency_hz_clamped = motor_state.frequency_hz.clamp(frequencies.min_hz, frequencies.max_hz);

                let margin_frequency_hz = frequency_hz_clamped - frequencies.min_hz;
                motor_state.weight_multiplier = if margin_frequency_hz < frequencies.fade_range_hz {
                    margin_frequency_hz / frequencies.fade_range_hz
                } else {
                    1.0
                };

                // omega = frequency * _2PiLoopTimeSeconds
                // max_frequency < 0.5 / looptime_seconds
                // max_omega = (0.5 / looptime_seconds) * 2PiLooptimeSeconds = 0.5 * 2PI = PI;
                // so omega is in range [0, PI]
                let omega = notch_filter.calculate_omega(frequency_hz_clamped);

                // Calculate sin(omega) and cos(omega) and cache their values.
                // The second and third harmonics use trigonometric identities to calculate sin(2*omega), sin(3*omega) etc,
                // this is significantly faster than calling sin_cos() again.
                (motor_state.sin_omega, motor_state.cos_omega) = omega.sin_cos();

                notch_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    motor_state.sin_omega,
                    motor_state.cos_omega,
                    ctx.weights[FUNDAMENTAL] * motor_state.weight_multiplier,
                );
                // move onto the next state
                let motor_index = motor_index + 1;
                if motor_index < motor_count {
                    State::Fundamental { motor_index }
                } else {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    // If the second harmonic is being filtered then move onto it.
                    if config.rpm_filter_weights_x100[SECOND_HARMONIC] != 0 {
                        State::SecondHarmonic { motor_index: 0 }
                    // Otherwise try and move onto the third
                    } else if config.rpm_filter_weights_x100[THIRD_HARMONIC] != 0 {
                        State::ThirdHarmonic { motor_index: 0 }
                    } else {
                        State::Stopped
                    }
                }
            }
            State::SecondHarmonic { motor_index } => {
                let motor_state = &ctx.motor_states[motor_index];
                let notch_filter = &mut ctx.notch_filters[motor_index][SECOND_HARMONIC];
                // sin(2θ) = 2 * sin(θ) * cos(θ)
                // cos(2θ) = 2 * cos^2(θ) - 1
                let sin_2_omega = 2.0 * motor_state.sin_omega * motor_state.cos_omega;
                let cos_2_omega = 2.0 * motor_state.cos_omega * motor_state.cos_omega - 1.0;
                notch_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    sin_2_omega,
                    cos_2_omega,
                    ctx.weights[SECOND_HARMONIC] * motor_state.weight_multiplier,
                );
                let motor_index = motor_index + 1;
                if motor_index < motor_count {
                    State::SecondHarmonic { motor_index }
                } else {
                    // we have set the notch frequency for all motors, so move onto the next harmonic if there is one, otherwise we are finished
                    if config.rpm_filter_weights_x100[THIRD_HARMONIC] != 0 {
                        State::ThirdHarmonic { motor_index: 0 }
                    } else {
                        State::Stopped
                    }
                }
            }
            State::ThirdHarmonic { motor_index } => {
                let motor_state = &ctx.motor_states[motor_index];
                let notch_filter = &mut ctx.notch_filters[motor_index][THIRD_HARMONIC];
                // sin(3θ) = 3 * sin(θ)   - 4 * sin^3(θ)
                //         = sin(θ) * ( 3 - 4 * sin^2(θ) )
                //         = sin(θ) * ( 3 - 4 * (1 - cos^2(θ)) )
                //         = sin(θ) * ( 4 * cos^2(θ) - 1)
                // cos(3θ) = 4 * cos^3(θ) - 3 * cos(θ)
                //         = cos(θ) * ( 4 * cos^2(θ) - 3 )
                let four_cos_squared_omega = 4.0 * motor_state.cos_omega * motor_state.cos_omega;
                let sin_3_omega = motor_state.sin_omega * (four_cos_squared_omega - 1.0);
                let cos_3_omega = motor_state.cos_omega * (four_cos_squared_omega - 3.0);
                notch_filter.set_notch_frequency_weighted_from_sin_cos_assuming_q(
                    sin_3_omega,
                    cos_3_omega,
                    ctx.weights[THIRD_HARMONIC] * motor_state.weight_multiplier,
                );
                let motor_index = motor_index + 1;
                if motor_index < motor_count {
                    State::ThirdHarmonic { motor_index }
                } else {
                    // we have set the notch frequency for all motors, so we are finished
                    State::Stopped
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
    }
}
