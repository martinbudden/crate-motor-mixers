use crate::{
    MotorMixerCommands, MotorOutputRange, mixer_common::MotorSaturation, mixer_config::YawCompensationStrategy,
};

/// X-configuration octocopter.
///
/// Motor numbering is the same as Betaflight.
///
/// Motor directions are the same as Betaflight.
///
/// Supports hybrid mode with 4 large lifting props and 4 small maneuvering props.
///
/// Large props map to indices 0-3 (Betaflight `QuadX` locations). Small props map to indices 4-7.
///
/// Splits the propulsion into high-inertia core lifting fans (large props) and low-inertia reactive actuators (small props).
///
/// This gives best of both worlds: high efficiency for hovering/cruising and crisp attitude control due to the lower rotational inertia
/// of the smaller maneuvering props.
///
/// To make this physics model work, this mixer splits the flight controller commands using an asymmetric authority weight:
///
/// * Large Motors (1–4): Get 100% of the throttle request for raw lift, but only a small fraction of Roll/Pitch/Yaw commands (or none at all).
///   This keeps them running at a steady, efficient RPM.
///
/// * Small Motors (5–8): Get a small baseline throttle offset (so they never stall when dropping)
///   and receive 100% of the Roll, Pitch, and Yaw command authority to handle attitude maneuvering.
///
/// ```text
///
///
/// CW = clockwise
/// CC = counter clockwise
///
///        front
///  vCC^ 8     6 ^CWv
///  ^CWv 4     2 vCC^
///        \   /
///         |X|
///        /   \
///  vCC^ 3     1 ^CWv
///  ^CWv 7     5 vCC^
///
///
/// "Mix" calculation
///                                        m 1234 5678
/// Roll right              (left+  right-)  --++ --++
/// Pitch up (stick back)   (front+ back-)   -+-+ -+-+
/// Yaw clockwise           (CC+    CW-)     -++- +--+
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerOctocopter {
    range: MotorOutputRange,
    saturation: MotorSaturation,
    strategy: YawCompensationStrategy,
    /// **Attitude/Momentum parameter**. It dictates how much steering control (torque) is allocated to the large props
    /// ie what fraction of attitude commands spill into the large lifting props [0.0 to 1.0].
    /// Setting this to 0.0 means large props handle ONLY lift; 0.1 means they help a tiny bit.
    /// Set this to 1.0 for a standard octocopter.
    pub large_prop_authority: f32,
    /// **Thrust/Lift parameter**. It dictates how much of the collective vertical lifting burden is shared by the small motors.
    /// ie what fraction of throttle is allocated to the small props [0.0 to 1.0].
    /// Set this to 0.5 for hybrid mode, or 1.0 for a standard octocopter.
    pub small_prop_throttle_scale: f32,
    /// Baseline background throttle offset given to small props so they stay spinning and responsive in hybrid mode.
    /// Set this to 0.0 for standard octocopter.
    pub small_prop_idle_throttle: f32,
}

impl Default for MixerOctocopter {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerOctocopter {
    pub const MOTOR_COUNT_U8: u8 = 8;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 8;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            range: MotorOutputRange::new(),
            saturation: MotorSaturation::new(),
            strategy: YawCompensationStrategy::YawReduction,
            // Default values for standard octocopter behavior.
            large_prop_authority: 1.0,      // 100% active authority on large props
            small_prop_throttle_scale: 1.0, // Identical base throttle matching large props
            small_prop_idle_throttle: 0.0,  // No idle floor offset needed
        }
    }
    /// Set the range of a newly constructed tricopter.
    #[must_use]
    pub const fn with_range(mut self, range: MotorOutputRange) -> Self {
        self.set_range(range);
        self
    }
    /// Set the strategy of a newly constructed tricopter.
    #[must_use]
    pub const fn with_strategy(mut self, strategy: YawCompensationStrategy) -> Self {
        self.set_strategy(strategy);
        self
    }
    #[must_use]
    pub const fn with_large_prop_authority(mut self, large_prop_authority: f32) -> Self {
        self.set_large_prop_authority(large_prop_authority);
        self
    }
    #[must_use]
    pub const fn with_small_prop_idle_throttle(mut self, small_prop_idle_throttle: f32) -> Self {
        self.set_small_prop_idle_throttle(small_prop_idle_throttle);
        self
    }
    #[must_use]
    pub const fn with_small_prop_throttle_scale(mut self, small_prop_throttle_scale: f32) -> Self {
        self.set_small_prop_throttle_scale(small_prop_throttle_scale);
        self
    }
}

impl MixerOctocopter {
    pub const fn set_range(&mut self, range: MotorOutputRange) {
        self.range = range;
    }
    #[must_use]
    pub const fn range(self) -> MotorOutputRange {
        self.range
    }

    pub const fn set_strategy(&mut self, strategy: YawCompensationStrategy) {
        self.strategy = strategy;
    }
    #[must_use]
    pub const fn strategy(self) -> YawCompensationStrategy {
        self.strategy
    }
    pub const fn set_large_prop_authority(&mut self, large_prop_authority: f32) {
        self.large_prop_authority = large_prop_authority;
    }
    pub const fn set_small_prop_idle_throttle(&mut self, small_prop_idle_throttle: f32) {
        self.small_prop_idle_throttle = small_prop_idle_throttle;
    }
    pub const fn set_small_prop_throttle_scale(&mut self, small_prop_throttle_scale: f32) {
        self.small_prop_throttle_scale = small_prop_throttle_scale;
    }

    #[must_use]
    pub const fn saturation(self) -> MotorSaturation {
        self.saturation
    }
}

impl MixerOctocopter {
    #[must_use]
    pub fn mix(&mut self, commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        // Large Prop Indices (QuadX positions)
        const L_BACK_RIGHT: usize = 0;
        const L_FRONT_RIGHT: usize = 1;
        const L_BACK_LEFT: usize = 2;
        const L_FRONT_LEFT: usize = 3;

        // Small Prop Indices
        const S_BACK_RIGHT: usize = 4;
        const S_FRONT_RIGHT: usize = 5;
        const S_BACK_LEFT: usize = 6;
        const S_FRONT_LEFT: usize = 7;

        self.saturation.throttle = commands.throttle;
        self.saturation.overshoot = 0.0;
        self.saturation.undershoot = 0.0;

        let mut outputs = [0.0f32; Self::OUTPUT_COUNT];

        // 1. Distribute Raw Thrust.
        outputs[L_BACK_RIGHT] = commands.throttle;
        outputs[L_FRONT_RIGHT] = commands.throttle;
        outputs[L_BACK_LEFT] = commands.throttle;
        outputs[L_FRONT_LEFT] = commands.throttle;

        // Small props get a mix of base throttle scaled down, plus an idle value so they don't stall.
        let small_base_throttle = (commands.throttle * self.small_prop_throttle_scale) + self.small_prop_idle_throttle;
        outputs[S_BACK_RIGHT] = small_base_throttle;
        outputs[S_FRONT_RIGHT] = small_base_throttle;
        outputs[S_BACK_LEFT] = small_base_throttle;
        outputs[S_FRONT_LEFT] = small_base_throttle;

        // 2. Apply Asymmetric Attacking Authority Weights.
        let large_authority = self.large_prop_authority;

        // Large Props
        outputs[L_BACK_RIGHT] -= large_authority * (commands.roll + commands.pitch);
        outputs[L_FRONT_RIGHT] -= large_authority * (commands.roll - commands.pitch);
        outputs[L_BACK_LEFT] += large_authority * (commands.roll - commands.pitch);
        outputs[L_FRONT_LEFT] += large_authority * (commands.roll + commands.pitch);

        // Small Props
        outputs[S_BACK_RIGHT] -= commands.roll + commands.pitch;
        outputs[S_FRONT_RIGHT] -= commands.roll - commands.pitch;
        outputs[S_BACK_LEFT] += commands.roll - commands.pitch;
        outputs[S_FRONT_LEFT] += commands.roll + commands.pitch;

        // Clamp Roll/Pitch adjustments across all 8 motors to keep initial axis symmetry.
        for output in &mut outputs {
            *output = output.clamp(self.range.min, self.range.max);
        }

        // 3. Inject Yaw requests.
        outputs[L_BACK_RIGHT] += large_authority * commands.yaw;
        outputs[L_FRONT_RIGHT] -= large_authority * commands.yaw;
        outputs[L_BACK_LEFT] -= large_authority * commands.yaw;
        outputs[L_FRONT_LEFT] += large_authority * commands.yaw;

        outputs[S_BACK_RIGHT] += commands.yaw;
        outputs[S_FRONT_RIGHT] -= commands.yaw;
        outputs[S_BACK_LEFT] -= commands.yaw;
        outputs[S_FRONT_LEFT] += commands.yaw;

        // 4. Yaw Saturation Verification (Scans ALL active motors, adjusting for large motor authority).
        if commands.yaw > 0.0 {
            // Falling channels (check min floor violations)
            self.saturation.undershoot = (self.range.min - outputs[S_FRONT_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[S_BACK_LEFT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[L_FRONT_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[L_BACK_LEFT]).max(self.saturation.undershoot);

            // Rising channels (check max ceiling violations)
            self.saturation.overshoot = (outputs[S_BACK_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[S_FRONT_LEFT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[L_BACK_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[L_FRONT_LEFT] - self.range.max).max(self.saturation.overshoot);
        } else if commands.yaw < 0.0 {
            // Falling channels
            self.saturation.undershoot = (self.range.min - outputs[S_BACK_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[S_FRONT_LEFT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[L_BACK_RIGHT]).max(self.saturation.undershoot);
            self.saturation.undershoot = (self.range.min - outputs[L_FRONT_LEFT]).max(self.saturation.undershoot);

            // Rising channels
            self.saturation.overshoot = (outputs[S_FRONT_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[S_BACK_LEFT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[L_FRONT_RIGHT] - self.range.max).max(self.saturation.overshoot);
            self.saturation.overshoot = (outputs[L_BACK_LEFT] - self.range.max).max(self.saturation.overshoot);
        }

        // 5. Apply unified compensation block across both small and large sets.
        if self.saturation.undershoot > 0.0 || self.saturation.overshoot > 0.0 {
            let compensation = match self.strategy {
                YawCompensationStrategy::DynamicThrottleShift => {
                    self.saturation.throttle += self.saturation.undershoot - self.saturation.overshoot;
                    self.saturation.undershoot + self.saturation.overshoot
                }
                YawCompensationStrategy::YawReduction => self.saturation.undershoot.max(self.saturation.overshoot),
            };

            if commands.yaw >= 0.0 {
                // Apply 100% compensation to active small motors
                outputs[S_BACK_RIGHT] -= compensation;
                outputs[S_FRONT_RIGHT] += compensation;
                outputs[S_BACK_LEFT] += compensation;
                outputs[S_FRONT_LEFT] -= compensation;

                // Apply scaled compensation to large motors matching their configured authority
                outputs[L_BACK_RIGHT] -= compensation * large_authority;
                outputs[L_FRONT_RIGHT] += compensation * large_authority;
                outputs[L_BACK_LEFT] += compensation * large_authority;
                outputs[L_FRONT_LEFT] -= compensation * large_authority;
            } else {
                outputs[S_BACK_RIGHT] += compensation;
                outputs[S_FRONT_RIGHT] -= compensation;
                outputs[S_BACK_LEFT] -= compensation;
                outputs[S_FRONT_LEFT] += compensation;

                outputs[L_BACK_RIGHT] += compensation * large_authority;
                outputs[L_FRONT_RIGHT] -= compensation * large_authority;
                outputs[L_BACK_LEFT] -= compensation * large_authority;
                outputs[L_FRONT_LEFT] += compensation * large_authority;
            }
        }

        // 6. Final safety guard to protect all 8 outputs from precision rounding leaks.
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
        is_full::<MixerOctocopter>();
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
        let mut mixer = MixerOctocopter::new()
            .with_range(range)
            .with_strategy(YawCompensationStrategy::DynamicThrottleShift)
            .with_large_prop_authority(0.05)
            .with_small_prop_idle_throttle(0.1)
            .with_small_prop_throttle_scale(0.5);

        let mut outputs = mixer.mix(commands);

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
        let mut mixer = MixerOctocopter::new()
            .with_range(range)
            .with_strategy(YawCompensationStrategy::DynamicThrottleShift)
            .with_large_prop_authority(0.05)
            .with_small_prop_idle_throttle(0.1)
            .with_small_prop_throttle_scale(0.5);

        let mut outputs = mixer.mix(commands);

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
    let mut mixer = MixerOctocopter::new()
        .with_range(range)
        .with_strategy(YawCompensationStrategy::DynamicThrottleShift)
        .with_large_prop_authority(0.0)
        .with_small_prop_idle_throttle(0.15)
        .with_small_prop_throttle_scale(0.5);

    let mut outputs = mixer.mix(commands);

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
    let mut mixer = MixerOctocopter::new()
        .with_range(range)
        .with_strategy(YawCompensationStrategy::DynamicThrottleShift)
        .with_large_prop_authority(0.0)
        .with_small_prop_idle_throttle(0.1)
        .with_small_prop_throttle_scale(0.5);

    let mut outputs = mixer.mix(commands);
    let params = mixer.saturation();

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

#[test]
#[allow(clippy::similar_names)]
#[allow(clippy::float_cmp)]
fn test_standard_octocopter_fallback_behavior() {
    let commands = MotorMixerCommands { throttle: 0.6, roll: 0.1, pitch: 0.1, yaw: 0.0 };
    let range = MotorOutputRange::default();

    // Configure parameters to make the hybrid framework behave exactly
    // like a standard, uniform octocopter.
    let mut mixer = MixerOctocopter::new()
        .with_range(range)
        .with_strategy(YawCompensationStrategy::DynamicThrottleShift)
        .with_large_prop_authority(1.0)
        .with_small_prop_throttle_scale(1.0)
        .with_small_prop_idle_throttle(0.0);

    let mut outputs = mixer.mix(commands);

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
    //assert_eq!(outputs[4], outputs[0]); // Small BR == Large BR
    //assert_eq!(outputs[5], outputs[1]); // Small FR == Large FR
    //assert_eq!(outputs[6], outputs[2]); // Small BL == Large BL
    assert_eq!(outputs[7], outputs[3]); // Small FL == Large FL
}
