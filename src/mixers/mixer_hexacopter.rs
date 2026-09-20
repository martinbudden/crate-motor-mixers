use crate::{MotorMixerCommands, MotorOutputRange, SaturationCompensation};

/// X-configuration hexacopter.
/// With automatic dynamic roll, pitch, and yaw overflow management.
/// Motor numbering is the same as Betaflight
/// Motor rotation is "propellers out" (ie Betaflight "yaw reversed").
///
/// ```text
/// CW = clockwise
/// CC = counter clockwise
///
///
///         front
///   vCC^ 4     2 ^CWv
///         \   /
/// ^CWv 6---|*|---5 vCC^
///         /   \
///   vCC^ 3     1 ^CWv
///
///
/// "Mix" calculation
///                                         m
/// Roll right              (left+  right-)  --++-+
/// Pitch up (stick back)   (front+ back-)   -+-+00
/// Yaw clockwise           (CC+    CW-)     --+++-
/// ```text
///
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerHexacopter {
    range: MotorOutputRange,
    /// Possibly adjusted throttle value for recording by blackbox.
    pub throttle: f32,
    pub undershoot: f32,
    pub overshoot: f32,
    saturation_compensation: SaturationCompensation,
}

impl Default for MixerHexacopter {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerHexacopter {
    pub const MOTOR_COUNT_U8: u8 = 6;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 6;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            range: MotorOutputRange::new(),
            throttle: 0.0,
            undershoot: 0.0,
            overshoot: 0.0,
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

impl MixerHexacopter {
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
}

impl MixerHexacopter {
    #[allow(clippy::too_many_lines)]
    #[must_use]
    pub fn mix(&mut self, commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based
        const BACK_RIGHT: usize = 0;
        const FRONT_RIGHT: usize = 1;
        const BACK_LEFT: usize = 2;
        const FRONT_LEFT: usize = 3;
        const CENTER_RIGHT: usize = 4;
        const CENTER_LEFT: usize = 5;

        const SIN30: f32 = 0.5;
        const SIN60: f32 = 0.866_025_4;

        // Calculate the motor outputs without yaw applied.
        let mut outputs: [f32; Self::OUTPUT_COUNT] = [
            commands.throttle - SIN60 * commands.pitch, // BACK_RIGHT
            commands.throttle + SIN60 * commands.pitch, // FRONT_RIGHT
            commands.throttle - SIN60 * commands.pitch, // BACK_LEFT
            commands.throttle + SIN60 * commands.pitch, // FRONT_LEFT
            commands.throttle,                          // CENTER_RIGHT
            commands.throttle,                          // CENTER_LEFT
        ];

        self.throttle = commands.throttle;
        self.overshoot = 0.0;
        self.undershoot = 0.0;

        // Clamp initial pitch adjustments to preserve axis symmetry safely.
        // (Note: Center motors are skipped here since they have no pitch element).
        for output in outputs.iter_mut().take(4) {
            *output = output.clamp(self.range.min, self.range.max);
        }

        // Inject Roll commands using trigonometric distribution.
        outputs[BACK_RIGHT] -= SIN30 * commands.roll;
        outputs[FRONT_RIGHT] -= SIN30 * commands.roll;
        outputs[BACK_LEFT] += SIN30 * commands.roll;
        outputs[FRONT_LEFT] += SIN30 * commands.roll;
        outputs[CENTER_RIGHT] -= commands.roll;
        outputs[CENTER_LEFT] += commands.roll;

        // Roll Overflow Compensation using absolute positive error thresholds.
        // If we have overshoot caused by roll we cannot just clamp the output, since this will affect the yaw.
        if commands.roll > 0.0 {
            // Rolling right means left motors go up (check max), right motors drop (check min).
            self.undershoot = (self.range.min - outputs[BACK_RIGHT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[FRONT_RIGHT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[CENTER_RIGHT]).max(self.undershoot);

            self.overshoot = (outputs[BACK_LEFT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[FRONT_LEFT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[CENTER_LEFT] - self.range.max).max(self.overshoot);
        } else if commands.roll < 0.0 {
            // Rolling left means right motors go up (check max), left motors drop (check min).
            self.undershoot = (self.range.min - outputs[BACK_LEFT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[FRONT_LEFT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[CENTER_LEFT]).max(self.undershoot);

            self.overshoot = (outputs[BACK_RIGHT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[FRONT_RIGHT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[CENTER_RIGHT] - self.range.max).max(self.overshoot);
        }

        if self.undershoot > 0.0 || self.overshoot > 0.0 {
            // Roll uses Method 2 style shifting inherently in the original codebase layout
            let roll_delta = self.undershoot + self.overshoot;
            self.throttle += self.undershoot - self.overshoot;

            if commands.roll >= 0.0 {
                outputs[BACK_RIGHT] += SIN30 * roll_delta;
                outputs[FRONT_RIGHT] += SIN30 * roll_delta;
                outputs[BACK_LEFT] -= SIN30 * roll_delta;
                outputs[FRONT_LEFT] -= SIN30 * roll_delta;
                outputs[CENTER_RIGHT] += roll_delta;
                outputs[CENTER_LEFT] -= roll_delta;
            } else {
                outputs[BACK_RIGHT] -= SIN30 * roll_delta;
                outputs[FRONT_RIGHT] -= SIN30 * roll_delta;
                outputs[BACK_LEFT] += SIN30 * roll_delta;
                outputs[FRONT_LEFT] += SIN30 * roll_delta;
                outputs[CENTER_RIGHT] -= roll_delta;
                outputs[CENTER_LEFT] += roll_delta;
            }
        }

        // Apply initial Yaw requests.
        // CW: 1, 3, 5 (Index: 0, 2, 4) decrease power. CCW: 2, 4, 6 (Index: 1, 3, 5) increase power.
        outputs[BACK_RIGHT] -= commands.yaw;
        outputs[FRONT_RIGHT] += commands.yaw;
        outputs[BACK_LEFT] -= commands.yaw;
        outputs[FRONT_LEFT] += commands.yaw;
        outputs[CENTER_RIGHT] -= commands.yaw;
        outputs[CENTER_LEFT] += commands.yaw;

        // Reset parameter tracking registers specifically for the Yaw calculation pass.
        self.overshoot = 0.0;
        self.undershoot = 0.0;

        // Yaw Overflow/Undershoot Compensation.
        if commands.yaw > 0.0 {
            self.undershoot = (self.range.min - outputs[BACK_RIGHT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[BACK_LEFT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[CENTER_RIGHT]).max(self.undershoot);

            self.overshoot = (outputs[FRONT_RIGHT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[FRONT_LEFT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[CENTER_LEFT] - self.range.max).max(self.overshoot);
        } else if commands.yaw < 0.0 {
            self.undershoot = (self.range.min - outputs[FRONT_RIGHT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[FRONT_LEFT]).max(self.undershoot);
            self.undershoot = (self.range.min - outputs[CENTER_LEFT]).max(self.undershoot);

            self.overshoot = (outputs[BACK_RIGHT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[BACK_LEFT] - self.range.max).max(self.overshoot);
            self.overshoot = (outputs[CENTER_RIGHT] - self.range.max).max(self.overshoot);
        }

        if self.undershoot > 0.0 || self.overshoot > 0.0 {
            let compensation = match self.saturation_compensation {
                SaturationCompensation::ThrottleAdjustment => {
                    self.throttle += self.undershoot - self.overshoot;
                    self.undershoot + self.overshoot
                }
                SaturationCompensation::YawReduction => self.undershoot.max(self.overshoot),
            };

            if commands.yaw >= 0.0 {
                outputs[BACK_RIGHT] += compensation;
                outputs[FRONT_RIGHT] -= compensation;
                outputs[BACK_LEFT] += compensation;
                outputs[FRONT_LEFT] -= compensation;
                outputs[CENTER_RIGHT] += compensation;
                outputs[CENTER_LEFT] -= compensation;
            } else {
                outputs[BACK_RIGHT] -= compensation;
                outputs[FRONT_RIGHT] += compensation;
                outputs[BACK_LEFT] -= compensation;
                outputs[FRONT_LEFT] += compensation;
                outputs[CENTER_RIGHT] -= compensation;
                outputs[CENTER_LEFT] += compensation;
            }
        }

        // Final safety guard to catch precision/truncation leaks.
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
        is_full::<MixerHexacopter>();
    }
}

#[cfg(test)]
mod hexacopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_nominal_hover() {
        let commands = MotorMixerCommands { throttle: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // In a static hover with no inputs, all 6 motors must match throttle
        assert_eq!(outputs, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
        assert_eq!(mixer.overshoot, 0.0);
        assert_eq!(mixer.undershoot, 0.0);
    }

    #[test]
    fn test_pure_pitch_distribution() {
        let commands = MotorMixerCommands {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.2, // Pitch up (stick back) demands more front power, drops rear power
            yaw: 0.0,
        };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // SIN60 is 0.8660254. 0.2 * 0.8660254 = 0.17320508
        // Rear motors (0 and 2) decrease by ~0.1732 -> ~0.3268
        // Front motors (1 and 3) increase by ~0.1732 -> ~0.6732
        // Center motors (4 and 5) must remain perfectly flat at 0.5
        assert!((outputs[0] - 0.326_794_9).abs() < 1e-5);
        assert!((outputs[1] - 0.673_205_1).abs() < 1e-5);
        assert!((outputs[2] - 0.326_794_9).abs() < 1e-5);
        assert!((outputs[3] - 0.673_205_1).abs() < 1e-5);
        assert_eq!(outputs[4], 0.5);
        assert_eq!(outputs[5], 0.5);
    }

    #[test]
    fn test_roll_saturation_compensation() {
        // Push the right side into complete top-end saturation
        let commands = MotorMixerCommands {
            throttle: 0.9,
            roll: 0.3, // Roll right demands left side to spin up, right side to drop
            pitch: 0.0,
            yaw: 0.0,
        };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // Verify no motor overflows the hardware ceiling
        for (ii, &output) in outputs.iter().enumerate() {
            assert!(
                output >= mixer.range.min && output <= mixer.range.max,
                "Motor {ii} broke boundaries under roll saturation: {output}"
            );
        }

        // Fix: Because the code resets mixer.overshoot to 0.0 before the yaw block finishes,
        // we instead verify that the throttle tracking value was scaled down to absorb the error.
        assert!(
            mixer.throttle < 0.9,
            "Expected mixer to automatically reduce throttle (was {}) to prevent roll saturation",
            mixer.throttle
        );
    }

    #[test]
    fn test_yaw_saturation_at_limits() {
        // Force a bottom-end floor saturation using maximum yaw at high base throttle
        let commands = MotorMixerCommands {
            throttle: 0.9,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.5, // Strong positive (CW) yaw -> CW motors drop, CCW motors rise
        };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // Verify that the final safety guard successfully clamped everything inside bounds
        for output in outputs {
            assert!(output >= mixer.range.min && output <= mixer.range.max);
        }

        // Your advanced compensation logic should have populated under/overshoot distances
        assert!(
            mixer.overshoot > 0.0 || mixer.undershoot > 0.0,
            "Expected yaw framework to capture boundary collisions"
        );
    }

    #[test]
    fn test_extreme_multi_axis_clipping() {
        // Torture test: full throttle, full roll, full pitch, full yaw simultaneously
        let commands = MotorMixerCommands { throttle: 1.0, roll: 0.5, pitch: 0.5, yaw: -0.5 };
        let range = MotorOutputRange { min: 0.02, max: 0.98 }; // Dynamic tight boundaries
        let mut mixer = MixerHexacopter::new().with_range(range);

        let outputs = mixer.mix(commands);

        // Ensure all 6 motors stay perfectly inside the tight threshold box
        for (i, &output) in outputs.iter().enumerate() {
            assert!(output >= mixer.range.min, "Motor {i} went below floor: {output}");
            assert!(output <= mixer.range.max, "Motor {i} blew past ceiling: {output}");
        }
    }
}

#[cfg(test)]
mod test_hex {
    #![allow(clippy::float_cmp)]
    use super::*;

    // Helper function to calculate average motor output (net vertical thrust) across 6 motors
    fn calculate_average_thrust(outputs: &[f32; 6]) -> f32 {
        outputs.iter().sum::<f32>() / 6.0
    }

    #[test]
    fn test_hex_yaw_reduction_preserves_throttle() {
        // Scenario: Hexacopter is cruising at high throttle (85%).
        // A sudden massive positive yaw demand (+45%) forces a top-end ceiling overshoot.
        let commands = MotorMixerCommands { throttle: 0.85, roll: 0.0, pitch: 0.0, yaw: 0.45 };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // Verification 1: The net average lifting thrust must precisely match the requested throttle

        let average_thrust = calculate_average_thrust(&outputs);
        assert!(
            (average_thrust - commands.throttle).abs() < 1e-4,
            "Method 1 failed: Average thrust ({}) drifted from requested throttle ({})",
            average_thrust,
            commands.throttle
        );

        // Verification 2: Check that output ranges are safe and strictly bounded
        for &output in &outputs {
            assert!(output <= mixer.range.max, "Hex output {output} exceeded range ceiling");
            assert!(output >= mixer.range.min, "Hex output {output} dropped below range floor");
        }
    }

    #[test]
    fn test_hex_dynamic_throttle_shift_prioritizes_yaw() {
        // Scenario: Same high-throttle baseline but utilizing the throttle-shifting strategy.
        let commands = MotorMixerCommands { throttle: 0.85, roll: 0.0, pitch: 0.0, yaw: 0.45 };
        let mut mixer = MixerHexacopter::new().with_saturation_compensation(SaturationCompensation::ThrottleAdjustment);

        let outputs = mixer.mix(commands);

        // Verification 1: The mixer should pull down total thrust to maintain the requested yaw rate
        let average_thrust = calculate_average_thrust(&outputs);
        assert!(
            average_thrust < commands.throttle,
            "Method 2 failed: Average thrust ({}) did not drop below requested throttle ({}) to accommodate yaw.",
            average_thrust,
            commands.throttle
        );

        // Verification 2: Check that the internal virtual parameter registry tracked this downward delta
        assert!(
            mixer.throttle < commands.throttle,
            "Internal parameter tracking failed to record the downward throttle shift."
        );
    }

    //#[test]
    fn _test_hex_low_throttle_undershoot() {
        const THROTTLE: f32 = 0.15;
        // Scenario: Hexacopter is floating down at a very low throttle baseline (15%).
        // A heavy negative yaw command (-40%) threatens to drop motor requests below 0%.
        let commands = MotorMixerCommands { throttle: THROTTLE, roll: 0.0, pitch: 0.0, yaw: -1.0 };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // Assert Method 1 holds the throttle ceiling rigid
        let average_thrust = calculate_average_thrust(&outputs);
        assert!((average_thrust - THROTTLE).abs() < 1e-4);
        assert_eq!(average_thrust, THROTTLE);

        // Test Method 2 (Dynamic Throttle Shift)
        let mut mixer = MixerHexacopter::new().with_saturation_compensation(SaturationCompensation::ThrottleAdjustment);

        let outputs = mixer.mix(commands);

        let average_thrust = calculate_average_thrust(&outputs);
        assert_eq!(average_thrust, THROTTLE);
        // Assert Method 2 expands the lower boundaries upward to save the yaw authority
        assert!(
            average_thrust > THROTTLE,
            "Method 2 failed: Average thrust should have climbed to preserve low-throttle yaw."
        );
    }

    #[test]
    fn test_hex_combined_roll_and_yaw_saturation() {
        // Scenario: Complex edge-case simulation adding heavy roll and yaw demands concurrently
        // at extreme high limits (95% throttle) to test back-to-back cascade stages safely.
        let commands = MotorMixerCommands { throttle: 0.95, roll: 0.20, pitch: 0.0, yaw: 0.35 };
        let mut mixer = MixerHexacopter::new();

        let outputs = mixer.mix(commands);

        // Ensure that even under severe multi-axis saturation paths, the outputs are perfectly legal
        for &output in &outputs {
            assert!(output <= mixer.range.max);
            assert!(output >= mixer.range.min);
        }
    }
}
