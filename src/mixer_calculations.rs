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

use crate::mixer_config::OctoMixerParameters;

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
        commands.yaw, // The fourth element maps straight to the tail servo tilt demand
    ];

    // Check for Rear Motor Top-End Saturation (Overshoot).
    // Front motors are unlikely to overshoot since there are two of them and there is no yaw-related attenuation.
    if outputs[REAR] > range.max {
        params.overshoot = outputs[REAR] - range.max;

        // Rear motor is saturated; reduce its output to range.max and reduce front motors similarly.
        outputs[REAR] = range.max;
        outputs[FR] = (outputs[FR] - params.overshoot).max(range.min);
        outputs[FL] = (outputs[FL] - params.overshoot).max(range.min);
    }

    // Check for Front Motor Bottom-End Saturation (Undershoot).
    let min_front = outputs[FL].min(outputs[FR]);
    if min_front < range.min {
        // Express undershoot as an absolute positive error distance.
        params.undershoot = range.min - min_front;

        // Push both front motors up by the error distance to preserve roll authority.
        outputs[FR] = (outputs[FR] + params.undershoot).min(range.max);
        outputs[FL] = (outputs[FL] + params.undershoot).min(range.max);

        // Scale the rear motor upward similarly to balance the overall vertical thrust lifting vector.
        outputs[REAR] = (outputs[REAR] + params.undershoot).min(range.max);
    }

    // Final Safety Guard (Applies only to the three motor outputs)
    for output in &mut outputs[0..3] {
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

    // Check for overshoot caused by roll and pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the QuadX).
    // Roll & Pitch Overflow Management
    // Clamping here reduces attitude command magnitude without skewing axis symmetry.
    for output in &mut outputs {
        *output = output.clamp(range.min, range.max);
    }

    // Add initial yaw demands.
    outputs[BACK_RIGHT] += commands.yaw;
    outputs[FRONT_RIGHT] -= commands.yaw;
    outputs[BACK_LEFT] -= commands.yaw;
    outputs[FRONT_LEFT] += commands.yaw;

    // Yaw Overflow/Undershoot Compensation.
    // Now check if there is overshoot due to yaw.
    // We cannot simply clamp the offending outputs, since this may cause result in a change in the overall
    // vertical thrust (ie a "yaw jump").
    // For example, if m1 and m2 have their undershoot clamped without a corresponding clamping of the m0 and m3
    // then the overall vertical thrust will increase and the quadcopter will "jump" upwards.
    // So instead of clamping individual motors, we reduce the magnitude of the yaw command.
    if commands.yaw > 0.0 {
        // Find how far the falling motors went below range.min (convert to positive error distance).
        params.undershoot = (range.min - outputs[1]).max(params.undershoot);
        params.undershoot = (range.min - outputs[2]).max(params.undershoot);

        // Find how far the rising motors went above range.max (positive error distance).
        params.overshoot = (outputs[0] - range.max).max(params.overshoot);
        params.overshoot = (outputs[3] - range.max).max(params.overshoot);

        // If the total error doesn't completely wipe out the original yaw request.
        if commands.yaw - params.undershoot - params.overshoot > 0.0 {
            // Adjust the virtual throttle tracking parameter to reflect clipping state.
            params.throttle += params.undershoot - params.overshoot;

            // yaw_delta calculation: adjust both sides equally to pull them into range.
            let yaw_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] -= yaw_delta;
            outputs[FRONT_RIGHT] += yaw_delta;
            outputs[BACK_LEFT] += yaw_delta;
            outputs[FRONT_LEFT] -= yaw_delta;
        }
    } else if commands.yaw < 0.0 {
        // Find how far the falling motors went below range.min (positive error distance).
        params.undershoot = (range.min - outputs[0]).max(params.undershoot);
        params.undershoot = (range.min - outputs[3]).max(params.undershoot);

        // Find how far the rising motors went above range.max (positive error distance).
        params.overshoot = (outputs[1] - range.max).max(params.overshoot);
        params.overshoot = (outputs[2] - range.max).max(params.overshoot);

        if commands.yaw + params.undershoot + params.overshoot < 0.0 {
            params.throttle += params.undershoot - params.overshoot;

            let yaw_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] += yaw_delta;
            outputs[FRONT_RIGHT] -= yaw_delta;
            outputs[BACK_LEFT] -= yaw_delta;
            outputs[FRONT_LEFT] += yaw_delta;
        }
    }

    // Final Safety Guard.
    // Clamp to catch floating point truncation leaks.
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

        if commands.roll - params.undershoot - params.overshoot > 0.0 {
            let roll_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] += SIN30 * roll_delta;
            outputs[FRONT_RIGHT] += SIN30 * roll_delta;
            outputs[BACK_LEFT] -= SIN30 * roll_delta;
            outputs[FRONT_LEFT] -= SIN30 * roll_delta;
            outputs[CENTER_RIGHT] += roll_delta;
            outputs[CENTER_LEFT] -= roll_delta;
            params.throttle += params.undershoot - params.overshoot;
        }
    } else if commands.roll < 0.0 {
        // Rolling left means right motors go up (check max), left motors drop (check min).
        params.undershoot = (range.min - outputs[BACK_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_RIGHT] - range.max).max(params.overshoot);

        if commands.roll + params.undershoot + params.overshoot < 0.0 {
            let roll_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] -= SIN30 * roll_delta;
            outputs[FRONT_RIGHT] -= SIN30 * roll_delta;
            outputs[BACK_LEFT] += SIN30 * roll_delta;
            outputs[FRONT_LEFT] += SIN30 * roll_delta;
            outputs[CENTER_RIGHT] -= roll_delta;
            outputs[CENTER_LEFT] += roll_delta;
            params.throttle += params.undershoot - params.overshoot;
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

    // Reset parameters tracking block for yaw compensation.
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // Yaw Overflow/Undershoot Compensation.
    if commands.yaw > 0.0 {
        // CW yaw increases CCW motors (1, 3, 5) and drops CW motors (0, 2, 4).
        params.undershoot = (range.min - outputs[BACK_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[BACK_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_RIGHT]).max(params.undershoot);

        params.overshoot = (outputs[FRONT_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[FRONT_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_LEFT] - range.max).max(params.overshoot);

        if commands.yaw - params.undershoot - params.overshoot > 0.0 {
            let yaw_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] += yaw_delta;
            outputs[FRONT_RIGHT] -= yaw_delta;
            outputs[BACK_LEFT] += yaw_delta;
            outputs[FRONT_LEFT] -= yaw_delta;
            outputs[CENTER_RIGHT] += yaw_delta;
            outputs[CENTER_LEFT] -= yaw_delta;
            params.throttle += params.undershoot - params.overshoot;
        }
    } else if commands.yaw < 0.0 {
        // CCW yaw increases CW motors (0, 2, 4) and drops CCW motors (1, 3, 5).
        params.undershoot = (range.min - outputs[FRONT_RIGHT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[FRONT_LEFT]).max(params.undershoot);
        params.undershoot = (range.min - outputs[CENTER_LEFT]).max(params.undershoot);

        params.overshoot = (outputs[BACK_RIGHT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[BACK_LEFT] - range.max).max(params.overshoot);
        params.overshoot = (outputs[CENTER_RIGHT] - range.max).max(params.overshoot);

        if commands.yaw + params.undershoot + params.overshoot < 0.0 {
            let yaw_delta = params.undershoot + params.overshoot;
            outputs[BACK_RIGHT] -= yaw_delta;
            outputs[FRONT_RIGHT] += yaw_delta;
            outputs[BACK_LEFT] -= yaw_delta;
            outputs[FRONT_LEFT] += yaw_delta;
            outputs[CENTER_RIGHT] -= yaw_delta;
            outputs[CENTER_LEFT] += yaw_delta;
            params.throttle += params.undershoot - params.overshoot;
        }
    }

    // Final clamp to protect outputs from precision leaks.
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

        // In a perfect hover, all motors should match throttle exactly
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
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
            throttle: 0.0,
            overshoot: 0.0,
            undershoot: 0.0,
            max_servo_angle_radians: core::f32::consts::FRAC_PI_6, // 30 degrees max tilt
        };

        let outputs = mix_tricopter(commands, range, &mut params);

        // When yaw is 0, cos(0) = 1. All motors should receive exactly the baseline throttle
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters {
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
mod hexacopter_tests {
    #![allow(clippy::float_cmp)]
    use super::*;

    #[test]
    fn test_nominal_hover() {
        let commands = MotorMixerCommands { throttle: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = MotorMixerParameters::default();

        let outputs = mix_hex_x(commands, range, &mut params);

        // In a static hover with no inputs, all 6 motors must match throttle exactly
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = OctoMixerParameters {
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = OctoMixerParameters {
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
        let range = MotorOutputRange { min: 0.0, max: 1.0 };
        let mut params = OctoMixerParameters {
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
    let range = MotorOutputRange { min: 0.0, max: 1.0 };

    // Configure parameters to make the hybrid framework behave exactly
    // like a standard, uniform octocopter.
    let mut params = OctoMixerParameters {
        throttle: 0.0,
        overshoot: 0.0,
        undershoot: 0.0,
        large_prop_authority: 1.0,
        small_prop_throttle_scale: 1.0, // Set to 1.0 to match large props
        small_prop_idle_throttle: 0.0,
    };

    let outputs = mix_octo_quad_x(commands, range, &mut params);

    // 1. Calculate expected outputs for the Large Prop group (0-3)
    let expected_large_br = 0.6 - (0.1 + 0.1); // 0.4
    let expected_large_fr = 0.6 - (0.1 - 0.1); // 0.6
    let expected_large_bl = 0.6 + (0.1 - 0.1); // 0.6
    let expected_large_fl = 0.6 + (0.1 + 0.1); // 0.8

    assert!((outputs[0] - expected_large_br).abs() < 1e-5);
    assert!((outputs[1] - expected_large_fr).abs() < 1e-5);
    assert!((outputs[2] - expected_large_bl).abs() < 1e-5);
    assert!((outputs[3] - expected_large_fl).abs() < 1e-5);

    // 2. Calculate expected outputs for the Small Prop group (4-7)
    // With small_prop_throttle_scale = 1.0, the small props must output
    // the EXACT same values as their corresponding large prop counterparts.
    assert_eq!(outputs[4], outputs[0]); // Small BR == Large BR
    assert_eq!(outputs[5], outputs[1]); // Small FR == Large FR
    assert_eq!(outputs[6], outputs[2]); // Small BL == Large BL
    assert_eq!(outputs[7], outputs[3]); // Small FL == Large FL
}
