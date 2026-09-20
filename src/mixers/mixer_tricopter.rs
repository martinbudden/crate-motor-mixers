use crate::{MotorMixerCommands, MotorOutputRange, SaturationCompensation};
#[allow(unused)]
use vqm::MathMethods; // Required for .cos()

/// Tricopter, 3 motors, one servo-tiltable.
/// Tracks geometric center-of-thrust and tilt-loss attenuation.
/// Motor numbering is the same as Betaflight.
///
/// ```text
/// CW = clockwise
/// CC = counter clockwise
///
///
///     front
///   vCC^   ^CWv
///     3     2
///      \   /
///       |Y|
///        |
///        1
///       vCW^
///
/// "Mix" calculation
///                                         m
/// Roll right              (left+  right-)  0-+
/// Pitch up (stick back)   (front+ back-)   -++
/// Yaw clockwise           (CC+    CW-)     --+
/// ```text
///
/// NOTE: For coordinated flight the aircraft's nose is aligned with the direction of the turn, ie we yaw right when we roll right,
/// so we want the front right motor to turn clockwise.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerTricopter {
    range: MotorOutputRange,
    /// Possibly adjusted throttle value for recording by blackbox.
    pub throttle: f32,
    pub undershoot: f32,
    pub overshoot: f32,
    saturation_compensation: SaturationCompensation,
    max_servo_angle_radians: f32,
}

impl Default for MixerTricopter {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerTricopter {
    pub const MOTOR_COUNT_U8: u8 = 3;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 4;
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
            max_servo_angle_radians: 30.0f32.to_radians(),
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
    /// Set the max servo angle of a newly constructed tricopter, in radians.
    #[must_use]
    pub const fn with_max_servo_angle_radians(mut self, max_servo_angle_radians: f32) -> Self {
        self.set_max_servo_angle_radians(max_servo_angle_radians);
        self
    }
    /// Set the max servo angle of a newly constructed tricopter, in radians.
    #[must_use]
    pub const fn with_max_servo_angle_degrees(mut self, max_servo_angle_degrees: f32) -> Self {
        self.set_max_servo_angle_radians(max_servo_angle_degrees.to_radians());
        self
    }
}

impl MixerTricopter {
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

    pub const fn set_max_servo_angle_radians(&mut self, max_servo_angle_radians: f32) {
        self.max_servo_angle_radians = max_servo_angle_radians;
    }
    #[must_use]
    pub const fn max_servo_angle_radians(self) -> f32 {
        self.max_servo_angle_radians
    }
}

impl MixerTricopter {
    #[must_use]
    pub fn mix(&mut self, commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based.
        const REAR: usize = 0;
        const FR: usize = 1;
        const FL: usize = 2;
        const _SERVO: usize = 3;

        const TWO_THIRDS: f32 = 2.0 / 3.0;
        const FOUR_THIRDS: f32 = 4.0 / 3.0;

        self.throttle = commands.throttle;
        self.overshoot = 0.0;
        self.undershoot = 0.0;

        // Calculate physical servo tilt angle based on current yaw demands.
        let pivot_angle_radians = commands.yaw * self.max_servo_angle_radians;

        // Safety Guard: clamp cos calculation to prevent division by zero or negative angles.
        let cos_tilt = pivot_angle_radians.cos().max(0.1); // cos(84 degrees) ~ 0.1

        // Base mixing distribution applying the 1/3 and 2/3 center-of-mass geometric rules.
        let mut outputs: [f32; Self::OUTPUT_COUNT] = [
            (commands.throttle - FOUR_THIRDS * commands.pitch) / cos_tilt,
            commands.throttle - commands.roll + TWO_THIRDS * commands.pitch,
            commands.throttle + commands.roll + TWO_THIRDS * commands.pitch,
            commands.yaw, // Maps straight to the tail servo tilt
        ];

        // Check for Rear Motor Top-End Saturation (Overshoot).
        // Front motors are unlikely to overshoot since there are two of them and there is no yaw-related attenuation.
        if outputs[REAR] > self.range.max {
            self.overshoot = outputs[REAR] - self.range.max;
            outputs[REAR] = self.range.max;
        }

        // Check for Front Motor Bottom-End Saturation (Undershoot).
        let min_front = outputs[FL].min(outputs[FR]);
        if min_front < self.range.min {
            self.undershoot = self.range.min - min_front;
        }

        // Centralized Saturated Thrust Compensation Block
        if self.overshoot > 0.0 || self.undershoot > 0.0 {
            match self.saturation_compensation {
                SaturationCompensation::ThrottleAdjustment => {
                    // Adjust tracked virtual throttle baseline and apply raw deltas
                    self.throttle += self.undershoot - self.overshoot;

                    outputs[FR] = outputs[FR] - self.overshoot + self.undershoot;
                    outputs[FL] = outputs[FL] - self.overshoot + self.undershoot;
                    outputs[REAR] += self.undershoot;
                }
                SaturationCompensation::YawReduction => {
                    // Symmetrically damp the overshoot or undershoot impact using max violation bounds.
                    // This acts to prioritize attitude hold without modifying the internal throttle parameter.
                    let max_violation = self.overshoot.max(self.undershoot);

                    outputs[FR] -= max_violation;
                    outputs[FL] -= max_violation;
                    // Rear motor stays clamped at self.range.max/min to prioritize steady attitude tracking over total yaw rate
                }
            }
        }

        // Now clamp the three motor outputs.
        for output in &mut outputs[0..Self::MOTOR_COUNT] {
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
        is_full::<MixerTricopter>();
    }
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn test_mixer_tricopter() {
        const REAR: usize = 0;
        const FR: usize = 1;
        const FL: usize = 2;
        const S0: usize = 3;
        const EPSILON: f32 = 0.000_000_1;

        let range = MotorOutputRange { min: 0.1, max: 1.0 };
        let mut mixer = MixerTricopter::new()
            .with_range(range)
            .with_saturation_compensation(SaturationCompensation::ThrottleAdjustment)
            .with_max_servo_angle_degrees(60.0);
        let mut commands = MotorMixerCommands::new().with_throttle(0.4);

        let outputs = mixer.mix(commands);
        assert_eq!(0.0, mixer.undershoot);
        assert_eq!(0.0, mixer.overshoot);
        assert_eq!(0.4, mixer.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.4, outputs[REAR]);
        assert_eq!(0.0, outputs[S0]);

        commands.yaw = 0.3;
        let outputs = mixer.mix(commands);
        assert_eq!(0.0, mixer.undershoot);
        assert_eq!(0.0, mixer.overshoot);
        assert_eq!(0.4, mixer.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.420_584_9, outputs[REAR]);
        assert_eq!(0.3, outputs[S0]);

        commands.yaw = 1.0;
        let outputs = mixer.mix(commands);
        assert_eq!(0.0, mixer.undershoot);
        assert_abs_diff_eq!(0.0, mixer.overshoot, epsilon = EPSILON);
        assert_eq!(0.4, mixer.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_abs_diff_eq!(0.8, outputs[REAR], epsilon = EPSILON);
        assert_eq!(1.0, outputs[S0]);
    }
}

/*#[cfg(test)]
mod tricopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;


    #[test]
    fn test_nominal_hover_no_yaw() {
        let commands = MotorMixerCommands {
            throttle: 0.6,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0, // Servo centered
        };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            max_servo_angle_radians: core::f32::consts::FRAC_PI_6, // 30 degrees max tilt
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // When yaw is 0.0, cos(0) = 1.0 All motors should receive the baseline throttle
        assert_eq!(outputs[0], 0.6); // REAR
        assert_eq!(outputs[1], 0.6); // FR
        assert_eq!(outputs[2], 0.6); // FL
        assert_eq!(outputs[3], 0.0); // SERVO centered
    }

    #[test]
    fn test_tilt_compensation_increases_rear_motor() {
        let commands = MotorMixerCommands {
            throttle: 0.6,
            roll: 0.0,
            pitch: 0.0,
            yaw: 1.0, // Hard yaw command, maximizing servo tilt angle
        };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            max_servo_angle_radians: core::f32::consts::FRAC_PI_6, // 30 degrees max tilt
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // Because the tail servo tilts, the rear motor loses vertical thrust component.
        // The mixer must increase the rear motor's raw value (> 0.6) to compensate.
        assert!(outputs[0] > 0.6, "Expected rear motor power ({}) to increase to compensate for tilt loss", outputs[0]);
        assert_eq!(outputs[3], 1.0); // Servo channel correctly maps yaw command
    }

    #[test]
    fn test_rear_motor_overshoot_saturation() {
        let commands = MotorMixerCommands {
            throttle: 0.9,
            roll: 0.0,
            pitch: -0.2, // Pitch down (stick forward) demands more rear motor output
            yaw: 1.0,    // Tilt demands even more rear power
        };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::YawReduction,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            max_servo_angle_radians: core::f32::consts::FRAC_PI_6,
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // Ensure the rear motor is safely clamped to the maximum allowed limit
        assert_eq!(outputs[0], range.max);

        // Ensure overshoot is caught and logged as a positive error distance
        assert!(params.overshoot > 0.0);
    }

    #[test]
    fn test_front_motor_undershoot_compensation() {
        let commands = MotorMixerCommands {
            throttle: 0.1, // Near the bottom floor
            roll: 0.3,     // Sharp roll right drops Front Right significantly
            pitch: 0.0,
            yaw: 0.0,
        };
        let range = MotorOutputRange { min: 0.05, max: 1.0 }; // Hardware floor set to 0.05
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            max_servo_angle_radians: core::f32::consts::FRAC_PI_6,
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // Verify that no ESC-driven motor outputs drop below the absolute hardware range floor
        assert!(outputs[0] >= range.min);
        assert!(outputs[1] >= range.min);
        assert!(outputs[2] >= range.min);

        // The system should detect that the front right motor threatened to clip the floor
        assert!(params.undershoot > 0.0);
    }
}

#[cfg(test)]
mod tricopter_tests2 {
    use super::*;

    /*  impl Default for MotorMixerParameters {
        fn default() -> Self {
            Self {
                throttle: 0.0,
                overshoot: 0.0,
                undershoot: 0.0,
                max_servo_angle_radians: 40.0f32.to_radians(), // Default 40 degree tilt authority
            }
        }
    }*/

    #[test]
    fn test_tricopter_servo_mapping() {
        let commands = MotorMixerCommands {
            throttle: 50.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.5, // 50% right yaw stick
        };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::YawReduction,
            max_servo_angle_radians: 60.0f32.to_radians(),
            ..Default::default()
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // Verification: The 4th array position (index 3) must map completely untouched to the tail servo
        assert_eq!(outputs[3], commands.yaw, "Servo channel failed to pass through raw yaw stick input.");
    }

    #[test]
    fn test_tricopter_rear_overshoot_under_yaw_tilt() {
        // Scenario: Operating at very high throttle (95%). A heavy yaw stick command is input.
        // As the tail motor tilts, the cosine compensation calculation multiplies the requested
        // rear motor load upward, forcing it past the maximum boundary limit.
        let commands = MotorMixerCommands { throttle: 0.95, roll: 0.0, pitch: 0.0, yaw: 0.4 };
        let range = MotorOutputRange::default();

        // Test Method 1 (Yaw Reduction)
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::YawReduction,
            max_servo_angle_radians: 60.0f32.to_radians(),
            ..Default::default()
        };
        let outputs = mix_tricopter(commands, range, &mut params);

        // Assert rear motor is safely bound to max limits
        assert!(outputs[0] <= range.max);

        // Test Method 2 (Dynamic Throttle Shift)
        let mut params = MotorMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            max_servo_angle_radians: 60.0f32.to_radians(),
            ..Default::default()
        };
        let outputs = mix_tricopter(commands, range, &mut params);

        // Method 2 will pull down front motor commands to offset the lifting discrepancy
        let average = (outputs[0] + outputs[1] + outputs[2]) / 3.0;
        assert_eq!(outputs[0], 1.0);
        assert!((outputs[1] - 0.91).abs() < 1e-4);
        assert!((outputs[2] - 0.91).abs() < 1e-4);
        assert!(
            average < commands.throttle,
            "Method 2 failed: Did not shift overall engine baseline downward during tail tilt overflow."
        );
    }

    #[test]
    fn test_tricopter_low_throttle_front_undershoot() {
        // Scenario: Floating descending at 10% throttle, with strong pitch inputs pushing
        // front motor configurations past the range floor limits.
        let commands = MotorMixerCommands {
            throttle: 10.0,
            roll: 0.0,
            pitch: -300.0, // Aggressive forward pitch
            yaw: 0.0,
        };
        let range = MotorOutputRange::default();

        let mut params =
            MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };
        let outputs = mix_tricopter(commands, range, &mut params);

        // Assert all physical motor tracks are perfectly legally constrained inside the FPU
        for &output in &outputs[0..3] {
            assert!(output >= range.min, "Tricopter motor dropped below structural range floor");
            assert!(output <= range.max, "Tricopter motor exceeded range ceiling");
        }
    }
}


*/
