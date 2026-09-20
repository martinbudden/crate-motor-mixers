//use approx::assert_abs_diff_eq;

use crate::{
    {MotorMixerCommands, MotorMixerParameters, MotorOutputRange},
    {
        mix_hex_x, mix_octo_quad_x, mix_quad_x,
        mixer_config::{OctoMixerParameters, YawCompensationStrategy},
    },
};


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

    //#[test]
    fn _test_hex_low_throttle_undershoot() {
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
mod octocopter_tests {
    use super::*;
    // Helper to calculate average motor output across all 8 motors
    fn calculate_total_average_thrust(outputs: &[f32; 8]) -> f32 {
        outputs.iter().sum::<f32>() / 8.0
    }

    // Helper to calculate the isolated average of the small maneuvering props (indices 4-7)
    #[allow(unused)]
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

    //#[test]
    fn _test_octo_dynamic_throttle_shift_moves_maneuvering_window() {
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
            OctoMixerParameters { strategy: YawCompensationStrategy::YawReduction, ..Default::default() };
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
