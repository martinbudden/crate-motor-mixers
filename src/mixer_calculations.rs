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

#![allow(clippy::excessive_precision)]

use crate::mixer_config::{OctoMixerParameters, YawCompensationStrategy};

use super::{MotorMixerCommands, MotorMixerParameters, MotorOutputRange};
#[allow(unused)]
use vqm::MathMethods; // Required for .cos()

/// Mixer for flying wing (ie throttle and flaperons).
#[inline]
#[must_use]
pub fn mix_wing(commands: MotorMixerCommands) -> [f32; 3] {
    let outputs: [f32; 3] = [
        commands.throttle, // throttle may be controlled by a servo for a wing with an internal combustion engine
        commands.roll + commands.pitch, // left flaperon
        -commands.roll + commands.pitch, // right flaperon
    ];
    outputs
}

/// Mixer for airplane (ie throttle, ailerons, elevator, and rudder).
#[inline]
#[must_use]
pub fn mix_airplane(commands: MotorMixerCommands) -> [f32; 5] {
    let outputs: [f32; 5] = [
        commands.throttle, // throttle may be controlled by a servo for a wing with an internal combustion engine
        commands.roll,     // left aileron
        -commands.roll,    // right aileron
        commands.pitch,    // elevator
        commands.yaw,      // rudder
    ];
    outputs
}

/// Bicopter: two tilt-adjustable rotors.
#[inline]
#[must_use]
pub fn mix_bicopter(commands: MotorMixerCommands) -> [f32; 4] {
    let outputs: [f32; 4] = [
        commands.throttle + commands.roll, // motor left
        commands.throttle - commands.roll, // motor right
        commands.pitch - commands.yaw,     // servo left
        commands.pitch + commands.yaw,     // servo right
    ];
    outputs
}

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
#[must_use]
pub fn mix_tricopter(
    commands: MotorMixerCommands,
    range: MotorOutputRange,
    params: &mut MotorMixerParameters,
) -> [f32; 4] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based.
    const MOTOR_COUNT: usize = 3;
    const REAR: usize = 0;
    const FR: usize = 1;
    const FL: usize = 2;
    const _SERVO: usize = 3;

    const TWO_THIRDS: f32 = 2.0 / 3.0;
    const FOUR_THIRDS: f32 = 4.0 / 3.0;

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // Calculate physical servo tilt angle based on current yaw demands.
    let pivot_angle_radians = commands.yaw * params.max_servo_angle_radians;

    // Safety Guard: clamp cos calculation to prevent division by zero or negative angles.
    let cos_tilt = pivot_angle_radians.cos().max(0.1); // cos(84 degrees) ~ 0.1

    // Base mixing distribution applying the 1/3 and 2/3 center-of-mass geometric rules.
    let mut outputs: [f32; 4] = [
        (commands.throttle - FOUR_THIRDS * commands.pitch) / cos_tilt,
        commands.throttle - commands.roll + TWO_THIRDS * commands.pitch,
        commands.throttle + commands.roll + TWO_THIRDS * commands.pitch,
        commands.yaw, // Maps straight to the tail servo tilt
    ];

    // Check for Rear Motor Top-End Saturation (Overshoot).
    // Front motors are unlikely to overshoot since there are two of them and there is no yaw-related attenuation.
    if outputs[REAR] > range.max {
        params.overshoot = outputs[REAR] - range.max;
        outputs[REAR] = range.max;
    }

    // Check for Front Motor Bottom-End Saturation (Undershoot).
    let min_front = outputs[FL].min(outputs[FR]);
    if min_front < range.min {
        params.undershoot = range.min - min_front;
    }

    // Centralized Saturated Thrust Compensation Block
    if params.overshoot > 0.0 || params.undershoot > 0.0 {
        match params.strategy {
            YawCompensationStrategy::DynamicThrottleShift => {
                // Adjust tracked virtual throttle baseline and apply raw deltas
                params.throttle += params.undershoot - params.overshoot;

                outputs[FR] = outputs[FR] - params.overshoot + params.undershoot;
                outputs[FL] = outputs[FL] - params.overshoot + params.undershoot;
                outputs[REAR] += params.undershoot;
            }
            YawCompensationStrategy::YawReduction => {
                // Symmetrically damp the overshoot or undershoot impact using max violation bounds.
                // This acts to prioritize attitude hold without modifying the internal throttle parameter.
                let max_violation = params.overshoot.max(params.undershoot);

                outputs[FR] -= max_violation;
                outputs[FL] -= max_violation;
                // Rear motor stays clamped at range.max/min to prioritize steady attitude tracking over total yaw rate
            }
        }
    }

    // Now clamp the three motor outputs.
    for output in &mut outputs[0..MOTOR_COUNT] {
        *output = output.clamp(range.min, range.max);
    }

    outputs
}

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
#[must_use]
pub fn mix_quad_x(
    commands: MotorMixerCommands,
    range: MotorOutputRange,
    params: &mut MotorMixerParameters,
) -> [f32; 4] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based.
    const MOTOR_COUNT: usize = 4;
    const BACK_RIGHT: usize = 0;
    const FRONT_RIGHT: usize = 1;
    const BACK_LEFT: usize = 2;
    const FRONT_LEFT: usize = 3;

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // Calculate the motor outputs without yaw applied.
    let mut outputs: [f32; MOTOR_COUNT] = [
        commands.throttle - commands.roll - commands.pitch,
        commands.throttle - commands.roll + commands.pitch,
        commands.throttle + commands.roll - commands.pitch,
        commands.throttle + commands.roll + commands.pitch,
    ];

    // Roll & Pitch Overflow Management (Clamping preserves axis symmetry).
    for output in &mut outputs {
        *output = output.clamp(range.min, range.max);
    }

    // Add initial raw yaw demands to the calculated baseline.
    outputs[BACK_RIGHT] += commands.yaw;
    outputs[FRONT_RIGHT] -= commands.yaw;
    outputs[BACK_LEFT] -= commands.yaw;
    outputs[FRONT_LEFT] += commands.yaw;

    // Calculate tracking constraints based on yaw stick direction.
    if commands.yaw > 0.0 {
        params.undershoot = (range.min - outputs[FRONT_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[BACK_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_LEFT] - range.max).max(params.overshoot);
    } else if commands.yaw < 0.0 {
        params.undershoot = (range.min - outputs[BACK_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[BACK_LEFT] - range.max).max(params.overshoot);
    }

    // Apply unified compensation block if a boundary constraint was triggered.
    if params.undershoot > 0.0 || params.overshoot > 0.0 {
        let compensation = match params.strategy {
            YawCompensationStrategy::DynamicThrottleShift => {
                params.throttle += params.undershoot - params.overshoot;
                params.undershoot + params.overshoot
            }
            YawCompensationStrategy::YawReduction => params.undershoot.max(params.overshoot),
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
        *output = output.clamp(range.min, range.max);
    }

    outputs
}

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
#[allow(clippy::too_many_lines)]
#[must_use]
pub fn mix_hex_x(commands: MotorMixerCommands, range: MotorOutputRange, params: &mut MotorMixerParameters) -> [f32; 6] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based
    const MOTOR_COUNT: usize = 6;
    const BACK_RIGHT: usize = 0;
    const FRONT_RIGHT: usize = 1;
    const BACK_LEFT: usize = 2;
    const FRONT_LEFT: usize = 3;
    const CENTER_RIGHT: usize = 4;
    const CENTER_LEFT: usize = 5;

    const SIN30: f32 = 0.5;
    const SIN60: f32 = 0.866_025_4;

    // Calculate the motor outputs without yaw applied.
    let mut outputs: [f32; MOTOR_COUNT] = [
        commands.throttle - SIN60 * commands.pitch, // BACK_RIGHT
        commands.throttle + SIN60 * commands.pitch, // FRONT_RIGHT
        commands.throttle - SIN60 * commands.pitch, // BACK_LEFT
        commands.throttle + SIN60 * commands.pitch, // FRONT_LEFT
        commands.throttle,                          // CENTER_RIGHT
        commands.throttle,                          // CENTER_LEFT
    ];

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // Clamp initial pitch adjustments to preserve axis symmetry safely.
    // (Note: Center motors are skipped here since they have no pitch element).
    for output in outputs.iter_mut().take(4) {
        *output = output.clamp(range.min, range.max);
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
        params.undershoot = (range.min - outputs[BACK_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_RIGHT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_LEFT] - range.max).max(params.overshoot);
    } else if commands.roll < 0.0 {
        // Rolling left means right motors go up (check max), left motors drop (check min).
        params.undershoot = (range.min - outputs[BACK_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_RIGHT] - range.max).max(params.overshoot);
    }

    if params.undershoot > 0.0 || params.overshoot > 0.0 {
        // Roll uses Method 2 style shifting inherently in the original codebase layout
        let roll_delta = params.undershoot + params.overshoot;
        params.throttle += params.undershoot - params.overshoot;

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
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // --- STAGE 2: Yaw Overflow/Undershoot Compensation ---
    if commands.yaw > 0.0 {
        params.undershoot = (range.min - outputs[BACK_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[BACK_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_RIGHT]).max(params.undershoot);

        params.overshoot = (outputs[FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_LEFT] - range.max).max(params.overshoot);
    } else if commands.yaw < 0.0 {
        params.undershoot = (range.min - outputs[FRONT_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[BACK_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_RIGHT] - range.max).max(params.overshoot);
    }

    if params.undershoot > 0.0 || params.overshoot > 0.0 {
        let compensation = match params.strategy {
            YawCompensationStrategy::DynamicThrottleShift => {
                params.throttle += params.undershoot - params.overshoot;
                params.undershoot + params.overshoot
            }
            YawCompensationStrategy::YawReduction => params.undershoot.max(params.overshoot),
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
        *output = output.clamp(range.min, range.max);
    }

    outputs
}

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
#[must_use]
pub fn mix_octo_quad_x(
    commands: MotorMixerCommands,
    range: MotorOutputRange,
    params: &mut OctoMixerParameters,
) -> [f32; 8] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based.

    // Large Prop Indices (QuadX positions)
    const MOTOR_COUNT: usize = 8;
    const L_BACK_RIGHT: usize = 0;
    const L_FRONT_RIGHT: usize = 1;
    const L_BACK_LEFT: usize = 2;
    const L_FRONT_LEFT: usize = 3;

    // Small Prop Indices
    const S_BACK_RIGHT: usize = 4;
    const S_FRONT_RIGHT: usize = 5;
    const S_BACK_LEFT: usize = 6;
    const S_FRONT_LEFT: usize = 7;

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    let mut outputs = [0.0f32; MOTOR_COUNT];

    // Distribute Raw Thrust.
    // Large props get the entire master throttle command.
    outputs[L_BACK_RIGHT] = commands.throttle;
    outputs[L_FRONT_RIGHT] = commands.throttle;
    outputs[L_BACK_LEFT] = commands.throttle;
    outputs[L_FRONT_LEFT] = commands.throttle;

    // Small props get a mix of base throttle scaled down, plus an idle value so they don't stall.
    let small_base_throttle = (commands.throttle * params.small_prop_throttle_scale) + params.small_prop_idle_throttle;
    outputs[S_BACK_RIGHT] = small_base_throttle;
    outputs[S_FRONT_RIGHT] = small_base_throttle;
    outputs[S_BACK_LEFT] = small_base_throttle;
    outputs[S_FRONT_LEFT] = small_base_throttle;

    // Apply Asymmetric Attacking Authority
    let large_authority = params.large_prop_authority; // eg 5% authority

    // Large Props (minimal reaction to maintain peak efficiency and avoid high-inertia spin changes).
    outputs[L_BACK_RIGHT] -= large_authority * (commands.roll + commands.pitch);
    outputs[L_FRONT_RIGHT] -= large_authority * (commands.roll - commands.pitch);
    outputs[L_BACK_LEFT] += large_authority * (commands.roll - commands.pitch);
    outputs[L_FRONT_LEFT] += large_authority * (commands.roll + commands.pitch);

    // Small Props (100% reaction authority for rapid attitude response).
    outputs[S_BACK_RIGHT] -= commands.roll + commands.pitch;
    outputs[S_FRONT_RIGHT] -= commands.roll - commands.pitch;
    outputs[S_BACK_LEFT] += commands.roll - commands.pitch;
    outputs[S_FRONT_LEFT] += commands.roll + commands.pitch;

    // Clamp Roll/Pitch adjustments on all 8 motors.
    for output in &mut outputs {
        *output = output.clamp(range.min, range.max);
    }

    // Inject Yaw requests.
    // Large props (minimal or zero yaw authority to avoid heavy high-inertia spin changes).
    outputs[L_BACK_RIGHT] += large_authority * commands.yaw;
    outputs[L_FRONT_RIGHT] -= large_authority * commands.yaw;
    outputs[L_BACK_LEFT] -= large_authority * commands.yaw;
    outputs[L_FRONT_LEFT] += large_authority * commands.yaw;

    // Small props handle 100% of the active yaw rotational counter-torque execution.
    outputs[S_BACK_RIGHT] += commands.yaw;
    outputs[S_FRONT_RIGHT] -= commands.yaw;
    outputs[S_BACK_LEFT] -= commands.yaw;
    outputs[S_FRONT_LEFT] += commands.yaw;

    // Yaw Saturation Verification (Targeting the highly active small motors).
    // Positive Yaw increases S_BACK_RIGHT and S_FRONT_LEFT (4 and 7), drops S_FRONT_RIGHT and S_BACK_LEFT (5 and 6).
    if commands.yaw > 0.0 {
        params.undershoot = (range.min - outputs[S_FRONT_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[S_BACK_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[S_BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[S_FRONT_LEFT] - range.max).max(params.overshoot);

        if commands.yaw - params.undershoot - params.overshoot > 0.0 {
            let yaw_delta = params.undershoot + params.overshoot;
            outputs[S_BACK_RIGHT] -= yaw_delta;
            outputs[S_FRONT_RIGHT] += yaw_delta;
            outputs[S_BACK_LEFT] += yaw_delta;
            outputs[S_FRONT_LEFT] -= yaw_delta;
            params.throttle += params.undershoot - params.overshoot;
        }
    } else if commands.yaw < 0.0 {
        params.undershoot = (range.min - outputs[S_BACK_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[S_FRONT_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[S_FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[S_BACK_LEFT] - range.max).max(params.overshoot);

        if commands.yaw + params.undershoot + params.overshoot < 0.0 {
            let yaw_delta = params.undershoot + params.overshoot;
            outputs[S_BACK_RIGHT] += yaw_delta;
            outputs[S_FRONT_RIGHT] -= yaw_delta;
            outputs[S_BACK_LEFT] -= yaw_delta;
            outputs[S_FRONT_LEFT] += yaw_delta;
            params.throttle += params.undershoot - params.overshoot;
        }
    }

    // Final clamp to protect outputs from precision leaks.
    for output in &mut outputs {
        *output = output.clamp(range.min, range.max);
    }

    outputs
}
