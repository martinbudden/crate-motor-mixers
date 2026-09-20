//! Mixer calculations.
//!
//! Note mixer calculations are traditionally done using a "mix" matrix.
//! This, however, is problematic: if a manoeuver causes the output
//! to a motor to be clipped, then this clipping can cause
//! an uncommanded change on another axis (eg the so-called "yaw jumps").
//!
//! The mixer calculations are done on each axis individually,
//! checking for overshoot and undershoot, and corrections
//! applied to avoid unwanted jumps.

use crate::{MotorMixerCommands, MotorOutputRange, MotorSaturation, SaturationCompensation};

/// Classic X-configuration quadcopter.
/// Includes overflow and yaw-jump compensation.
/// Motor numbering is the same as Betaflight.
/// Motor rotation is "propellers out" (ie Betaflight "yaw reversed").
///
/// ```text
/// CW = clockwise
/// CC = counter clockwise
///
///
///        front
///  vCC^ 4     2 ^CWv
///        \   /
///         |X|
///        /   \
///  ^CWv 3     1 vCC^
///
///
/// "Mix" calculation
///                                          m
/// Roll right              (left+  right-)  --++
/// Pitch up (stick back)   (front+ back-)   -+-+
/// Yaw clockwise           (CC+    CW-)     +--+
/// ```text
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerQuadcopter {
    range: MotorOutputRange,
    saturation: MotorSaturation,
    saturation_compensation: SaturationCompensation,
}

impl Default for MixerQuadcopter {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerQuadcopter {
    pub const MOTOR_COUNT_U8: u8 = 4;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 4;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            range: MotorOutputRange::new(),
            saturation: MotorSaturation::new(),
            saturation_compensation: SaturationCompensation::YawReduction,
        }
    }
    /// Set the range of a newly constructed tricopter.
    #[must_use]
    pub const fn with_range(mut self, range: MotorOutputRange) -> Self {
        self.set_range(range);
        self
    }
    /// Set the saturation compensation of a newly constructed tricopter.
    #[must_use]
    pub const fn with_saturation_compensation(mut self, saturation_compensation: SaturationCompensation) -> Self {
        self.set_saturation_compensation(saturation_compensation);
        self
    }
}

impl MixerQuadcopter {
    pub const fn set_range(&mut self, range: MotorOutputRange) {
        self.range = range;
    }
    #[must_use]
    pub const fn range(self) -> MotorOutputRange {
        self.range
    }

    pub const fn set_saturation_compensation(&mut self, saturation_compensation: SaturationCompensation) {
        self.saturation_compensation = saturation_compensation;
    }
    #[must_use]
    pub const fn saturation_compensation(self) -> SaturationCompensation {
        self.saturation_compensation
    }

    #[must_use]
    pub const fn saturation(self) -> MotorSaturation {
        self.saturation
    }
}

impl MixerQuadcopter {
    #[must_use]
    pub fn mix(&mut self, commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based.
        const BACK_RIGHT: usize = 0;
        const FRONT_RIGHT: usize = 1;
        const BACK_LEFT: usize = 2;
        const FRONT_LEFT: usize = 3;

        self.saturation.throttle = commands.throttle;
        self.saturation.overshoot = 0.0;
        self.saturation.undershoot = 0.0;

        // Calculate the motor outputs without yaw applied.
        let mut outputs: [f32; Self::OUTPUT_COUNT] = [
            commands.throttle - commands.roll - commands.pitch,
            commands.throttle - commands.roll + commands.pitch,
            commands.throttle + commands.roll - commands.pitch,
            commands.throttle + commands.roll + commands.pitch,
        ];

        // Roll & Pitch Overflow Management (Clamping preserves axis symmetry).
        for output in &mut outputs {
            *output = output.clamp(self.range.min, self.range.max);
        }

        // Add initial raw yaw demands to the calculated baseline.
        outputs[BACK_RIGHT] += commands.yaw;
        outputs[FRONT_RIGHT] -= commands.yaw;
        outputs[BACK_LEFT] -= commands.yaw;
        outputs[FRONT_LEFT] += commands.yaw;

        // Calculate tracking constraints based on yaw stick direction.
        if commands.yaw > 0.0 {
            self.saturation.undershoot = (self.range.min - outputs[FRONT_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[BACK_LEFT]).max(self.saturation.undershoot);

            self.saturation.overshoot = (outputs[BACK_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[FRONT_LEFT] - self.range.max).max(self.saturation.overshoot);
        } else if commands.yaw < 0.0 {
            self.saturation.undershoot = (self.range.min - outputs[BACK_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[FRONT_LEFT]).max(self.saturation.undershoot);

            self.saturation.overshoot = (outputs[FRONT_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[BACK_LEFT] - self.range.max).max(self.saturation.overshoot);
        }

        // Apply unified compensation block if a boundary constraint was triggered.
        if self.saturation.undershoot > 0.0 || self.saturation.overshoot > 0.0 {
            let compensation = match self.saturation_compensation {
                SaturationCompensation::ThrottleAdjustment => {
                    self.saturation.throttle += self.saturation.undershoot - self.saturation.overshoot;
                    self.saturation.undershoot + self.saturation.overshoot
                }
                SaturationCompensation::YawReduction => self.saturation.undershoot.max(self.saturation.overshoot),
            };

            // This boolean mapping avoids pipeline-stalling branches and heavy FPU multiplications,
            // allowing the compiler to use conditional register selection (like VMOVGE/VMOVLT).
            if commands.yaw >= 0.0 {
                outputs[BACK_RIGHT] -= compensation;
                outputs[FRONT_RIGHT] += compensation;
                outputs[BACK_LEFT] += compensation;
                outputs[FRONT_LEFT] -= compensation;
            } else {
                outputs[BACK_RIGHT] += compensation;
                outputs[FRONT_RIGHT] -= compensation;
                outputs[BACK_LEFT] -= compensation;
                outputs[FRONT_LEFT] += compensation;
            }
        }

        // Final safety guard to catch floating-point truncation/rounding leaks.
        for output in &mut outputs {
            *output = output.clamp(self.range.min, self.range.max);
        }

        outputs
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MixerQuadcopter>();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn test_mixer_quad_x_roll() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let range = MotorOutputRange::default();
        let mut mixer = MixerQuadcopter::new().with_range(range);

        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();

        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.0, mix_params.throttle);
        assert_eq!(0.0, outputs[0]);
        assert_eq!(0.0, outputs[1]);
        assert_eq!(0.0, outputs[2]);
        assert_eq!(0.0, outputs[3]);

        commands.throttle = 0.4;
        commands.roll = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.1, outputs[1], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.7, outputs[2], epsilon = EPSILON); // throttle + commands.roll
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.roll

        commands.throttle = 0.8;
        commands.roll = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.roll
        assert_eq!(0.5, outputs[1]); // throttle - commands.roll
        assert_eq!(1.0, outputs[2]); // throttle + commands.roll
        assert_eq!(1.0, outputs[3]); // throttle + commands.roll

        commands.throttle = 0.1;
        commands.roll = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.1, mix_params.throttle);
        assert_eq!(0.0, outputs[0]); // throttle - commands.roll
        assert_eq!(0.0, outputs[1]); // throttle - commands.roll
        assert_eq!(0.4, outputs[2]); // throttle + commands.roll
        assert_eq!(0.4, outputs[3]); // throttle + commands.roll
    }

    #[test]
    fn test_mixer_quad_x_pitch() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let range = MotorOutputRange::default();
        let mut mixer = MixerQuadcopter::new().with_range(range);

        commands.throttle = 0.4;
        commands.pitch = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[1], epsilon = EPSILON); // throttle + commands.pitch
        assert_abs_diff_eq!(0.1, outputs[2], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.pitch

        commands.throttle = 0.8;
        commands.pitch = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot); // pitch overshoot is ignored
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[1]); // throttle + commands.pitch
        assert_eq!(0.5, outputs[2]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[3]); // throttle + commands.pitch

        commands.throttle = 0.1;
        commands.pitch = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot); // pitch overshoot is ignored
        assert_eq!(0.1, mix_params.throttle);
        assert_eq!(0.0, outputs[0]); // throttle - commands.pitch
        assert_eq!(0.4, outputs[1]); // throttle + commands.pitch
        assert_eq!(0.0, outputs[2]); // throttle - commands.pitch
        assert_eq!(0.4, outputs[3]); // throttle + commands.pitch
    }
}
/*
    #[test]
    fn test_mixer_quad_x_yaw() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let mut range = MotorOutputRange::default();
        let mut mix_params = MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };

        commands.throttle = 0.4;
        commands.yaw = 0.3;
        let mut outputs = mixer.mix(commands);
        let mix_params = mixer.saturation();
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.7, outputs[0], epsilon = EPSILON); // throttle + commands.yaw
        assert_abs_diff_eq!(0.1, outputs[1], epsilon = EPSILON); // throttle - commands.yaw
        assert_abs_diff_eq!(0.1, outputs[2], epsilon = EPSILON); // throttle - commands.yaw
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.yaw

        // this will give an undershoot of -0.1, so commands.yaw should be adjusted to 0.2
        commands.throttle = 0.4;
        commands.yaw = 0.3;
        range.min = 0.2;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_abs_diff_eq!(0.1, mix_params.undershoot, epsilon = EPSILON);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.5, mix_params.throttle);
        assert_eq!(0.6, outputs[0]); // throttle + commands.yaw
        assert_eq!(0.2, outputs[1]); // throttle - commands.yaw
        assert_eq!(0.2, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[3]); // throttle + commands.yaw

        // this will give an undershoot of -0.1, so commands.yaw should be adjusted to -0.2
        commands.throttle = 0.4;
        commands.yaw = -0.3;
        range.min = 0.2;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_abs_diff_eq!(0.1, mix_params.undershoot, epsilon = EPSILON);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.5, mix_params.throttle);
        assert_eq!(0.2, outputs[0]); // throttle + commands.yaw
        assert_eq!(0.6, outputs[1]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.2, outputs[3]); // throttle + commands.yaw

        // this will give an overshoot of 0.1, so commands.yaw should be adjusted to 0.2
        commands.throttle = 0.8;
        commands.yaw = 0.3;
        range.min = 0.0;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_abs_diff_eq!(0.1, mix_params.overshoot, epsilon = EPSILON);
        assert_eq!(0.7, mix_params.throttle);
        assert_eq!(1.0, outputs[0]); // throttle + commands.yaw
        assert_eq!(0.6, outputs[1]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[2]); // throttle - commands.yaw
        assert_eq!(1.0, outputs[3]); // throttle + commands.yaw

        // this will give an overshoot of 0.1, so commands.yaw should be adjusted to -0.2
        commands.throttle = 0.8;
        commands.yaw = -0.3;
        range.min = 0.0;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_abs_diff_eq!(0.1, mix_params.overshoot, epsilon = EPSILON);
        assert_eq!(0.7, mix_params.throttle);
        assert_eq!(0.6, outputs[0]); // throttle + commands.yaw
        assert_eq!(1.0, outputs[1]); // throttle - commands.yaw
        assert_eq!(1.0, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[3]); // throttle + commands.yaw
    }
}

#[cfg(test)]
mod quadcopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_nominal_hover() {
        let commands = MotorMixerCommands { throttle: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters::default();

        let outputs = mix_quad_x(commands, range, &mut params);

        // In a perfect hover, all motors should match throttle value
        assert_eq!(outputs, [0.5, 0.5, 0.5, 0.5]);
        assert_eq!(params.overshoot, 0.0);
        assert_eq!(params.undershoot, 0.0);
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn test_pure_roll_symmetry() {
        let commands = MotorMixerCommands {
            throttle: 0.5,
            roll: 0.2, // Roll right means left side goes up, right side goes down
            pitch: 0.0,
            yaw: 0.0,
        };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters::default();

        let outputs = mix_quad_x(commands, range, &mut params);

        // Check right motors (0 and 1) decreased, left motors (2 and 3) increased
        assert_eq!(outputs[0], 0.3); // BACK_RIGHT: 0.5 - 0.2
        assert_eq!(outputs[1], 0.3); // FRONT_RIGHT: 0.5 - 0.2
        assert_eq!(outputs[2], 0.7); // BACK_LEFT: 0.5 + 0.2
        assert_eq!(outputs[3], 0.7); // FRONT_LEFT: 0.5 + 0.2
    }

    #[test]
    fn test_yaw_saturation_at_max_throttle() {
        // Force the mixer into top-end saturation
        let commands = MotorMixerCommands {
            throttle: 1.0, // Already at maximum limits
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.2, // Demanding positive (CW) yaw
        };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_quad_x(commands, range, &mut params);

        // Total vertical thrust verification to prove no yaw jump occurred:
        // Sum of baseline motors before yaw saturation compensation would have overflowed.
        // With your strategy, the total thrust must remain perfectly balanced.
        //let total_thrust: f32 = outputs.iter().sum();

        // At max throttle, total thrust cap should scale correctly without blowing past max array ceilings
        assert!(outputs[0] <= 1.0);
        assert!(outputs[1] <= 1.0);
        assert!(outputs[2] <= 1.0);
        assert!(outputs[3] <= 1.0);

        // Ensure the mixer detected the overshoot threat and logged it
        assert!(params.overshoot > 0.0, "Expected overshoot metrics to capture boundary collisions");
    }

    #[test]
    fn test_yaw_saturation_at_min_throttle() {
        // Force the mixer into bottom-end saturation
        let commands = MotorMixerCommands {
            throttle: 0.0, // At minimum limit
            roll: 0.0,
            pitch: 0.0,
            yaw: -0.3, // Demanding negative (CCW) yaw
        };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_quad_x(commands, range, &mut params);

        // Ensure no motor drops below the absolute minimum allowed range line
        assert!(outputs[0] >= 0.0);
        assert!(outputs[1] >= 0.0);
        assert!(outputs[2] >= 0.0);
        assert!(outputs[3] >= 0.0);

        // Ensure the mixer detected and tracked the undershoot limit condition
        assert!(params.undershoot > 0.0, "Expected undershoot parameters to catch floor collisions");
    }

    #[test]
    fn test_extreme_combined_saturation() {
        // Extreme scenario: Full throttle, full pitch up, full roll right, full yaw
        let commands = MotorMixerCommands { throttle: 1.0, roll: 0.5, pitch: 0.5, yaw: 0.5 };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters::default();

        let outputs = mix_quad_x(commands, range, &mut params);

        // Verify all outputs remain tightly constrained inside your hardware envelope
        for (i, &output) in outputs.iter().enumerate() {
            assert!(output >= range.min && output <= range.max, "Motor {i} broke boundaries with value {output}");
        }
    }
}
*/
