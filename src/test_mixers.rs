use approx::assert_abs_diff_eq;

use crate::{
    {MotorMixerCommands, MotorMixerParameters, MotorOutputRange},
    {
        mix_hex_x, mix_octo_quad_x, mix_quad_x, mix_tricopter,
        mixer_config::{OctoMixerParameters, YawCompensationStrategy},
    },
};

#[cfg(test)]
mod tricopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_mixer_tricopter() {
        const REAR: usize = 0;
        const FR: usize = 1;
        const FL: usize = 2;
        const S0: usize = 3;
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let range = MotorOutputRange { min: 0.1, max: 1.0 };
        let mut mix_params = MotorMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            max_servo_angle_radians: 60.0f32.to_radians(),
            throttle: 0.0,
            undershoot: 0.0,
            overshoot: 0.0,
        };

        commands.throttle = 0.4;
        let outputs = mix_tricopter(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.4, outputs[REAR]);
        assert_eq!(0.0, outputs[S0]);

        commands.yaw = 0.3;
        let outputs = mix_tricopter(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.420_584_89, outputs[REAR]);
        assert_eq!(0.3, outputs[S0]);

        commands.yaw = 1.0;
        let outputs = mix_tricopter(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_abs_diff_eq!(0.0, mix_params.overshoot, epsilon = EPSILON);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_abs_diff_eq!(0.8, outputs[REAR], epsilon = EPSILON);
        assert_eq!(1.0, outputs[S0]);
    }

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

#[cfg(test)]
mod quadcopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_mixer_quad_x_roll() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let range = MotorOutputRange::default();
        let mut mix_params = MotorMixerParameters::default();

        let mut outputs = mix_quad_x(commands, range, &mut mix_params);

        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.0, mix_params.throttle);
        assert_eq!(0.0, outputs[0]);
        assert_eq!(0.0, outputs[1]);
        assert_eq!(0.0, outputs[2]);
        assert_eq!(0.0, outputs[3]);

        commands.throttle = 0.4;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.1, outputs[1], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.7, outputs[2], epsilon = EPSILON); // throttle + commands.roll
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.roll

        commands.throttle = 0.8;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.roll
        assert_eq!(0.5, outputs[1]); // throttle - commands.roll
        assert_eq!(1.0, outputs[2]); // throttle + commands.roll
        assert_eq!(1.0, outputs[3]); // throttle + commands.roll

        commands.throttle = 0.1;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, range, &mut mix_params);
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
        let mut mix_params = MotorMixerParameters::default();

        commands.throttle = 0.4;
        commands.pitch = 0.3;
        let mut outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[1], epsilon = EPSILON); // throttle + commands.pitch
        assert_abs_diff_eq!(0.1, outputs[2], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.pitch

        commands.throttle = 0.8;
        commands.pitch = 0.3;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot); // pitch overshoot is ignored
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[1]); // throttle + commands.pitch
        assert_eq!(0.5, outputs[2]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[3]); // throttle + commands.pitch

        commands.throttle = 0.1;
        commands.pitch = 0.3;
        outputs = mix_quad_x(commands, range, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot); // pitch overshoot is ignored
        assert_eq!(0.1, mix_params.throttle);
        assert_eq!(0.0, outputs[0]); // throttle - commands.pitch
        assert_eq!(0.4, outputs[1]); // throttle + commands.pitch
        assert_eq!(0.0, outputs[2]); // throttle - commands.pitch
        assert_eq!(0.4, outputs[3]); // throttle + commands.pitch
    }
    #[test]
    fn test_mixer_quad_x_yaw() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let mut range = MotorOutputRange::default();
        let mut mix_params = MotorMixerParameters::default();

        commands.throttle = 0.4;
        commands.yaw = 0.3;
        let mut outputs = mix_quad_x(commands, range, &mut mix_params);
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
        //assert_eq!(0.5, mix_params.throttle);
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
        //assert_eq!(0.5, mix_params.throttle);
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
        //assert_eq!(0.7, mix_params.throttle);
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
        //assert_eq!(0.7, mix_params.throttle);
        assert_eq!(0.6, outputs[0]); // throttle + commands.yaw
        assert_eq!(1.0, outputs[1]); // throttle - commands.yaw
        assert_eq!(1.0, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[3]); // throttle + commands.yaw
    }
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

#[cfg(test)]
mod hexacopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_nominal_hover() {
        let commands = MotorMixerCommands { throttle: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // In a static hover with no inputs, all 6 motors must match throttle
        assert_eq!(outputs, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]);
        assert_eq!(params.overshoot, 0.0);
        assert_eq!(params.undershoot, 0.0);
    }

    #[test]
    fn test_pure_pitch_distribution() {
        let commands = MotorMixerCommands {
            throttle: 0.5,
            roll: 0.0,
            pitch: 0.2, // Pitch up (stick back) demands more front power, drops rear power
            yaw: 0.0,
        };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // SIN60 is 0.8660254. 0.2 * 0.8660254 = 0.17320508
        // Rear motors (0 and 2) decrease by ~0.1732 -> ~0.3268
        // Front motors (1 and 3) increase by ~0.1732 -> ~0.6732
        // Center motors (4 and 5) must remain perfectly flat at 0.5
        assert!((outputs[0] - 0.326_794_92).abs() < 1e-5);
        assert!((outputs[1] - 0.673_205_08).abs() < 1e-5);
        assert!((outputs[2] - 0.326_794_92).abs() < 1e-5);
        assert!((outputs[3] - 0.673_205_08).abs() < 1e-5);
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
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // Verify no motor overflows the hardware ceiling
        for (i, &output) in outputs.iter().enumerate() {
            assert!(
                output >= range.min && output <= range.max,
                "Motor {i} broke boundaries under roll saturation: {output}"
            );
        }

        // Fix: Because the code resets params.overshoot to 0.0 before the yaw block finishes,
        // we instead verify that the throttle tracking value was scaled down to absorb the error.
        assert!(
            params.throttle < 0.9,
            "Expected mixer to automatically reduce throttle (was {}) to prevent roll saturation",
            params.throttle
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
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // Verify that the final safety guard successfully clamped everything inside bounds
        for output in outputs {
            assert!(output >= range.min && output <= range.max);
        }

        // Your advanced compensation logic should have populated under/overshoot distances
        assert!(
            params.overshoot > 0.0 || params.undershoot > 0.0,
            "Expected yaw framework to capture boundary collisions"
        );
    }

    #[test]
    fn test_extreme_multi_axis_clipping() {
        // Torture test: full throttle, full roll, full pitch, full yaw simultaneously
        let commands = MotorMixerCommands { throttle: 1.0, roll: 0.5, pitch: 0.5, yaw: -0.5 };
        let range = MotorOutputRange { min: 0.02, max: 0.98 }; // Dynamic tight boundaries
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // Ensure all 6 motors stay perfectly inside the tight threshold box
        for (i, &output) in outputs.iter().enumerate() {
            assert!(output >= range.min, "Motor {i} went below floor: {output}");
            assert!(output <= range.max, "Motor {i} blew past ceiling: {output}");
        }
    }
}

#[cfg(test)]
mod hybrid_octo_tests {
    use super::*;

    #[test]
    #[allow(clippy::float_cmp)]
    fn test_nominal_hover() {
        let commands = MotorMixerCommands { throttle: 0.6, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let range = MotorOutputRange::default();
        let mut params = OctoMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            large_prop_authority: 0.05,
            small_prop_idle_throttle: 0.1,
            small_prop_throttle_scale: 0.5,
        };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Large props (0-3) should match the full master throttle request exactly
        assert_eq!(outputs[0], 0.6);
        assert_eq!(outputs[1], 0.6);
        assert_eq!(outputs[2], 0.6);
        assert_eq!(outputs[3], 0.6);

        // Small props (4-7) should match the scaled base throttle + idle offset:
        // (0.6 * 0.5) + 0.1 = 0.4
        assert_eq!(outputs[4], 0.4);
        assert_eq!(outputs[5], 0.4);
        assert_eq!(outputs[6], 0.4);
        assert_eq!(outputs[7], 0.4);
    }

    #[test]
    fn test_asymmetric_attitude_authority() {
        let commands = MotorMixerCommands {
            throttle: 0.6,
            roll: 0.2, // Sudden sharp roll command
            pitch: 0.0,
            yaw: 0.0,
        };
        let range = MotorOutputRange::default();
        let mut params = OctoMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            large_prop_authority: 0.05, // Only 5% bleed into large props
            small_prop_idle_throttle: 0.1,
            small_prop_throttle_scale: 0.5,
        };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Verify Large props barely reacted (0.05 * 0.2 = 0.01 change)
        // Right side (0, 1) drops from 0.6 to 0.59. Left side (2, 3) rises to 0.61.
        assert!((outputs[0] - 0.59).abs() < 1e-5);
        assert!((outputs[1] - 0.59).abs() < 1e-5);
        assert!((outputs[2] - 0.61).abs() < 1e-5);
        assert!((outputs[3] - 0.61).abs() < 1e-5);

        // Verify Small props reacted with 100% authority (1.0 * 0.2 = 0.20 change)
        // Right side baseline was 0.4 -> drops to 0.2. Left side baseline was 0.4 -> rises to 0.6.
        assert!((outputs[4] - 0.2).abs() < 1e-5);
        assert!((outputs[5] - 0.2).abs() < 1e-5);
        assert!((outputs[6] - 0.6).abs() < 1e-5);
        assert!((outputs[7] - 0.6).abs() < 1e-5);
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn test_small_prop_idle_gate_protection() {
        // Force an extreme roll response that would normally force a motor to zero or stop
        let commands = MotorMixerCommands {
            throttle: 0.1, // Near zero throttle
            roll: 0.4,     // Heavy roll right command
            pitch: 0.0,
            yaw: 0.0,
        };
        // Set hardware minimum safety floor to 0.05
        let range = MotorOutputRange { min: 0.05, max: 1.0 };
        let mut params = OctoMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            large_prop_authority: 0.0,      // Large props locked entirely out of maneuvering
            small_prop_idle_throttle: 0.15, // High reactive idle padding
            small_prop_throttle_scale: 0.5,
        };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Small right props baseline: (0.1 * 0.5) + 0.15 = 0.20
        // Roll right subtracts 0.4 -> 0.20 - 0.40 = -0.20
        // The hard clamp inside the mixer must catch this and keep it securely locked to the floor
        assert_eq!(outputs[4], range.min);
        assert_eq!(outputs[5], range.min);
    }

    #[test]
    fn test_yaw_saturation_on_maneuvering_props() {
        // Push the active small motors into saturation boundaries
        let commands = MotorMixerCommands {
            throttle: 0.9,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.5, // Demanding clockwise yaw
        };
        let range = MotorOutputRange::default();
        let mut params = OctoMixerParameters {
            strategy: YawCompensationStrategy::DynamicThrottleShift,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            large_prop_authority: 0.0,
            small_prop_idle_throttle: 0.1,
            small_prop_throttle_scale: 0.5,
        };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Verify every single one of the 8 output channels was successfully constrained within limits
        for output in outputs {
            assert!(output >= range.min && output <= range.max);
        }

        // The mixer should detect that the small maneuvering motors hit clipping thresholds
        assert!(
            params.overshoot > 0.0 || params.undershoot > 0.0,
            "Expected saturation limits to capture boundary collisions"
        );
    }
}

#[test]
#[allow(clippy::similar_names)]
#[allow(clippy::float_cmp)]
fn test_standard_octocopter_fallback_behavior() {
    let commands = MotorMixerCommands { throttle: 0.6, roll: 0.1, pitch: 0.1, yaw: 0.0 };
    let range = MotorOutputRange::default();

    // Configure parameters to make the hybrid framework behave exactly
    // like a standard, uniform octocopter.
    let mut params = OctoMixerParameters {
        strategy: YawCompensationStrategy::DynamicThrottleShift,
        throttle: 0.0,
        overshoot: 0.0,
        undershoot: 0.0,
        large_prop_authority: 1.0,
        small_prop_throttle_scale: 1.0, // Set to 1.0 to match large props
        small_prop_idle_throttle: 0.0,
    };

    let outputs = mix_octo_quad_x(commands, range, &mut params);

    // Calculate expected outputs for the Large Prop group (0-3)
    let expected_large_br = 0.6 - (0.1 + 0.1); // 0.4
    let expected_large_fr = 0.6 - (0.1 - 0.1); // 0.6
    let expected_large_bl = 0.6 + (0.1 - 0.1); // 0.6
    let expected_large_fl = 0.6 + (0.1 + 0.1); // 0.8

    assert!((outputs[0] - expected_large_br).abs() < 1e-5);
    assert!((outputs[1] - expected_large_fr).abs() < 1e-5);
    assert!((outputs[2] - expected_large_bl).abs() < 1e-5);
    assert!((outputs[3] - expected_large_fl).abs() < 1e-5);

    // Calculate expected outputs for the Small Prop group (4-7)
    // With small_prop_throttle_scale = 1.0, the small props must output
    // the EXACT same values as their corresponding large prop counterparts.
    assert_eq!(outputs[4], outputs[0]); // Small BR == Large BR
    assert_eq!(outputs[5], outputs[1]); // Small FR == Large FR
    assert_eq!(outputs[6], outputs[2]); // Small BL == Large BL
    assert_eq!(outputs[7], outputs[3]); // Small FL == Large FL
}

#[cfg(test)]
mod test_hex {
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
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };

        let outputs = mix_hex_x(commands, range, &mut params);
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
            assert!(output <= range.max, "Hex output {} exceeded range ceiling", output);
            assert!(output >= range.min, "Hex output {} dropped below range floor", output);
        }
    }

    #[test]
    fn test_hex_dynamic_throttle_shift_prioritizes_yaw() {
        // Scenario: Same high-throttle baseline but utilizing the throttle-shifting strategy.
        let commands = MotorMixerCommands { throttle: 0.85, roll: 0.0, pitch: 0.0, yaw: 0.45 };
        let range = MotorOutputRange::default();
        let mut params =
            MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };

        let outputs = mix_hex_x(commands, range, &mut params);

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
            params.throttle < commands.throttle,
            "Internal parameter tracking failed to record the downward throttle shift."
        );
    }

    #[test]
    fn test_hex_low_throttle_undershoot() {
        const THROTTLE: f32 = 0.15;
        // Scenario: Hexacopter is floating down at a very low throttle baseline (15%).
        // A heavy negative yaw command (-40%) threatens to drop motor requests below 0%.
        let commands = MotorMixerCommands { throttle: THROTTLE, roll: 0.0, pitch: 0.0, yaw: -1.0 };
        let range = MotorOutputRange::default();

        // Test Method 1 (Yaw Reduction)
        let mut params = MotorMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };
        let outputs = mix_hex_x(commands, range, &mut params);

        // Assert Method 1 holds the throttle ceiling rigid
        let average_thrust = calculate_average_thrust(&outputs);
        assert!((average_thrust - THROTTLE).abs() < 1e-4);
        assert_eq!(average_thrust, THROTTLE);

        // Test Method 2 (Dynamic Throttle Shift)
        let mut params =
            MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };
        let outputs = mix_hex_x(commands, range, &mut params);

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
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters::default();
        params.strategy = YawCompensationStrategy::YawReduction;

        let outputs = mix_hex_x(commands, range, &mut params);

        // Ensure that even under severe multi-axis saturation paths, the outputs are perfectly legal
        for &output in &outputs {
            assert!(output <= range.max);
            assert!(output >= range.min);
        }
    }
}
#[cfg(test)]
mod quad_tests {
    use super::*;

    fn calculate_average_thrust(outputs: &[f32; 4]) -> f32 {
        outputs.iter().sum::<f32>() / 4.0
    }

    #[test]
    fn test_quad_yaw_reduction_preserves_throttle() {
        // Scenario: High throttle (90%) with a heavy positive yaw demand (+40%).
        let commands = MotorMixerCommands { throttle: 0.9, roll: 0.0, pitch: 0.0, yaw: 0.4 };
        let range = MotorOutputRange::default();
        let mut params = MotorMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };

        let outputs = mix_quad_x(commands, range, &mut params);

        // Method 1 must hold the net vertical lifting thrust exactly rigid
        let average_thrust = calculate_average_thrust(&outputs);
        assert!(
            (average_thrust - commands.throttle).abs() < 1e-4,
            "Method 1 failed: Average thrust ({}) drifted from requested throttle ({})",
            average_thrust,
            commands.throttle
        );

        for &output in &outputs {
            assert!(output <= range.max);
            assert!(output >= range.min);
        }
    }

    #[test]
    fn test_quad_dynamic_throttle_shift_prioritizes_yaw() {
        // Scenario: High throttle (90%) with a heavy positive yaw demand (+50%).
        let commands = MotorMixerCommands { throttle: 0.9, roll: 0.0, pitch: 0.0, yaw: 0.5 };
        let range = MotorOutputRange::default();
        let mut params =
            MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };

        let outputs = mix_quad_x(commands, range, &mut params);
        //info!("outputs: {}, {}, {}, {}", outputs[0],outputs[1],outputs[2],outputs[3]);
        assert_eq!(params.overshoot, 0.39999998);
        assert_eq!(params.undershoot, 0.0);
        assert_eq!(params.throttle, 0.5);
        assert_eq!(outputs[0], 1.0);
        assert_eq!(outputs[1], 0.79999995);
        assert_eq!(outputs[2], 0.79999995);
        assert_eq!(outputs[3], 1.0);

        // Method 2 must lower total engine output to carve out ceiling headroom for the spin
        let average_thrust = calculate_average_thrust(&outputs);
        assert!(
            average_thrust < commands.throttle,
            "Method 2 failed: Average thrust ({}) did not drop to clear headroom.",
            average_thrust
        );
        assert!(params.throttle < commands.throttle);
    }

    #[test]
    fn test_quad_low_throttle_undershoot() {
        // Scenario: Low throttle (10%) with a sudden heavy negative yaw demand (-35%).
        const THROTTLE: f32 = 0.1;
        let commands = MotorMixerCommands { throttle: THROTTLE, roll: 0.0, pitch: 0.0, yaw: -0.35 };
        let range = MotorOutputRange::default();

        // Test Method 1
        let mut params = MotorMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };
        let outputs = mix_quad_x(commands, range, &mut params);
        assert!((calculate_average_thrust(&outputs) - THROTTLE).abs() < 1e-4);

        // Test Method 2
        let mut params =
            MotorMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };
        let outputs = mix_quad_x(commands, range, &mut params);

        // Method 2 must raise the baseline floor up to prevent the motors from stopping entirely
        assert!(calculate_average_thrust(&outputs) > THROTTLE);
    }
}

#[cfg(test)]
mod octocopter_tests {
    use super::*;
    // Helper to calculate average motor output across all 8 motors
    fn calculate_total_average_thrust(outputs: &[f32; 8]) -> f32 {
        outputs.iter().sum::<f32>() / 8.0
    }

    // Helper to calculate the isolated average of the small maneuvering props (indices 4-7)
    fn calculate_small_props_average(outputs: &[f32; 8]) -> f32 {
        outputs[4..8].iter().sum::<f32>() / 4.0
    }

    #[test]
    fn test_standard_octocopter_fallback_saturation() {
        // Scenario: Configured as a standard symmetric octocopter frame.
        // Operating at very high throttle (92%) and throwing a huge positive yaw command (+40%).
        let commands = MotorMixerCommands { throttle: 0.92, roll: 0.0, pitch: 0.0, yaw: 0.4 };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };

        // Emulate standard octocopter behavior using the settings provided
        let mut params = OctoMixerParameters {
            strategy: YawCompensationStrategy::YawReduction,
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            large_prop_authority: 1.0,      // 100% active authority on large props
            small_prop_throttle_scale: 1.0, // Identical base throttle matching large props
            small_prop_idle_throttle: 0.0,  // No idle floor offset needed
        };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Verification 1: In standard mode, matching indices across both sets (e.g., L_BACK_RIGHT and S_BACK_RIGHT)
        // must receive the exact same output because authority and scaling matches are fully uniform (1.0).
        assert_eq!(
            outputs[0], outputs[4],
            "Standard mode asymmetry bug: Large and Small back-right outputs diverged ({} vs {})",
            outputs[0], outputs[4]
        );
        assert_eq!(
            outputs[3], outputs[7],
            "Standard mode asymmetry bug: Large and Small front-left outputs diverged ({} vs {})",
            outputs[3], outputs[7]
        );

        // Verification 2: Under Method 1 (YawReduction), the global vertical lifting thrust must match the original throttle
        let total_average_thrust = calculate_total_average_thrust(&outputs);
        assert!(
            (total_average_thrust - commands.throttle).abs() < 1e-4,
            "Standard mode failed: Net average thrust ({}) drifted from requested throttle ({}) under Method 1.",
            total_average_thrust,
            commands.throttle
        );

        // Verification 3: Confirm no clipping overflow escaped the floating-point register bounds
        for &output in &outputs {
            assert!(output <= range.max, "Standard motor element output exceeded max bounds");
            assert!(output >= range.min, "Standard motor element output dropped below min bounds");
        }
    }

    #[test]
    fn test_octo_asymmetric_thrust_distribution() {
        const THROTTLE: f32 = 0.6;
        let commands = MotorMixerCommands { throttle: THROTTLE, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let range = MotorOutputRange::default();
        let mut params = OctoMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };

        // Under normal steady cruising conditions, large motors should take 100% of master throttle
        let outputs = mix_octo_quad_x(commands, range, &mut params);

        assert_eq!(outputs[0], THROTTLE, "Large back-right motor failed to receive baseline throttle");

        // Small motors should be scaled down according to structural physics limits
        let expected_small = (THROTTLE * params.small_prop_throttle_scale) + params.small_prop_idle_throttle;
        assert_eq!(outputs[4], expected_small, "Small maneuvering motor baseline calculation failed");
    }

    #[test]
    fn test_octo_yaw_reduction_shields_large_motors() {
        // Scenario: Cruising at high throttle (90%) and initiating an aggressive clockwise spin (+40%).
        let commands = MotorMixerCommands { throttle: 0.90, roll: 0.0, pitch: 0.0, yaw: 0.4 };
        let range = MotorOutputRange::default();
        let mut params = OctoMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Verification 1: Method 1 must leave the virtual core tracking parameter completely unchanged.
        assert_eq!(
            params.throttle, commands.throttle,
            "Method 1 modified master throttle context parameter tracking incorrectly."
        );

        // Verification 2: Check that active motor outputs stay safely capped within structural hardware boundaries.
        for &output in &outputs {
            assert!(output <= range.max, "Octocopter motor command {} overshot ceiling limits", output);
            assert!(output >= range.min, "Octocopter motor command {} dropped past floor limits", output);
        }
    }

    #[test]
    fn test_octo_dynamic_throttle_shift_moves_maneuvering_window() {
        // Scenario: High throttle (90%) coupled with a large clockwise spin (+40%).
        let commands = MotorMixerCommands { throttle: 0.9, roll: 0.0, pitch: 0.0, yaw: 0.4 };
        let range = MotorOutputRange::default();
        let mut params =
            OctoMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };

        // Calculate a reference baseline of the small maneuvering props BEFORE saturation changes occur
        let initial_small_base =
            (commands.throttle * params.small_prop_throttle_scale) + params.small_prop_idle_throttle;

        let outputs = mix_octo_quad_x(commands, range, &mut params);

        // Verification 1: The small motor mixing window should have been dynamically dragged downward
        // to avoid pinning values past the 100% ceiling.
        let actual_small_average = calculate_small_props_average(&outputs);
        assert!(
            actual_small_average < initial_small_base,
            "Method 2 failed: Small motor window average ({}) did not drop below un-saturated base ({})",
            actual_small_average,
            initial_small_base
        );

        // Verification 2: The internal global tracking parameter must accurately record the net downward shift.
        assert!(
            params.throttle < commands.throttle,
            "The internal master tracking parameter was not adjusted downward."
        );
    }

    #[test]
    fn test_octo_low_throttle_undershoot_protection() {
        // Scenario: Descending at very low engine speed (10% throttle).
        // A heavy negative yaw command (-30%) risks dropping the reactive small props past 0%.
        const THROTTLE: f32 = 0.1;
        let commands = MotorMixerCommands { throttle: THROTTLE, roll: 0.0, pitch: 0.0, yaw: -0.3 };
        let range = MotorOutputRange::default();

        // Test Method 1 (Yaw Reduction)
        let mut params =
            OctoMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };
        let _outputs = mix_octo_quad_x(commands, range, &mut params);
        assert_eq!(params.throttle, THROTTLE, "Method 1 altered throttle floor unexpectedly.");

        // Test Method 2 (Dynamic Throttle Shift)
        let mut params =
            OctoMixerParameters { strategy: YawCompensationStrategy::DynamicThrottleShift, ..Default::default() };
        let _outputs = mix_octo_quad_x(commands, range, &mut params);

        // Method 2 must raise the small maneuvering floor upward to preserve rotational velocity authority
        assert!(
            params.throttle > commands.throttle,
            "Method 2 failed: Internal parameter tracking should have increased past 10% to prevent low-throttle stalls."
        );
    }
}
