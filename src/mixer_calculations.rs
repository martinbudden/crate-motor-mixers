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

use crate::{MotorMixerCommands, MotorMixerParameters};
#[allow(unused)]
use vqm::TrigonometricMethods; // Required for .cos()

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
///
#[must_use]
pub fn mix_tricopter(commands: MotorMixerCommands, params: &mut MotorMixerParameters) -> [f32; 4] {
    const REAR: usize = 0;
    const FR: usize = 1;
    const FL: usize = 2;
    //const S0: usize = 3;
    const TWO_THIRDS: f32 = 2.0 / 3.0;
    const FOUR_THIRDS: f32 = 4.0 / 3.0;

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    let pivot_angle_radians = commands.yaw * params.max_servo_angle_radians;

    let mut outputs: [f32; 4] = [
        (commands.throttle - FOUR_THIRDS * commands.pitch) / pivot_angle_radians.cos(),
        commands.throttle - commands.roll + TWO_THIRDS * commands.pitch,
        commands.throttle + commands.roll + TWO_THIRDS * commands.pitch,
        commands.yaw,
    ];

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_TRICOPTER)
    // check for rear overshoot, front motors unlikely to overshoot since there are two of them and there is no yaw-related attenuation
    params.overshoot = outputs[REAR] - params.motor_output_max;
    if params.overshoot > 0.0 {
        // rear motor is saturated, so reduce its output to params.motor_output_max and reduce front motors similarly
        // TODO: should also increase yaw
        outputs[REAR] = params.motor_output_max;
        outputs[FR] = (outputs[FR] - params.overshoot).max(params.motor_output_min);
        outputs[FL] = (outputs[FL] - params.overshoot).max(params.motor_output_min);
    }

    // check for front undershoot
    params.undershoot = (outputs[FL]).min(outputs[FR]) - params.motor_output_min;
    if params.undershoot < 0.0 {
        outputs[REAR] = params.motor_output_min;
        outputs[FR] = (outputs[FR] - params.undershoot).min(params.motor_output_max);
        outputs[FL] = (outputs[FL] - params.undershoot).min(params.motor_output_max);
    }
    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_TRICOPTER)

    outputs
}

/// Classic X-configuration quadcopter.
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
pub fn mix_quad_x(commands: MotorMixerCommands, params: &mut MotorMixerParameters) -> [f32; 4] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based

    const BACK_RIGHT: usize = 0;
    const FRONT_RIGHT: usize = 1;
    const BACK_LEFT: usize = 2;
    const FRONT_LEFT: usize = 3;

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    // calculate the motor outputs without yaw applied
    let mut outputs: [f32; 4] = [
        commands.throttle - commands.roll - commands.pitch, // + commands.yaw;
        commands.throttle - commands.roll + commands.pitch, // - commands.yaw;
        commands.throttle + commands.roll - commands.pitch, // - commands.yaw;
        commands.throttle + commands.roll + commands.pitch, // + commands.yaw;
    ];

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)

    // Check for overshoot caused by roll and pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the QuadX).
    for output in &mut outputs[..] {
        *output = output.clamp(params.motor_output_min, params.motor_output_max);
    }

    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    outputs[BACK_RIGHT] += commands.yaw;
    outputs[FRONT_RIGHT] -= commands.yaw;
    outputs[BACK_LEFT] -= commands.yaw;
    outputs[FRONT_LEFT] += commands.yaw;

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)

    // Now check if there is overshoot due to yaw
    // We cannot simply clamp the offending outputs, since this may cause result in a change in the overall
    // vertical thrust (ie a "yaw jump").
    // For example, if m1 and m2 have their undershoot clamped without a corresponding clamping of the m0 and m3
    // then the overall vertical thrust will increase and the quadcopter will "jump" upwards.
    // So instead of clamping individual motors, we reduce the magnitude of the yaw command.
    if commands.yaw > 0.0 {
        // check if m1 or m2 will have output less than params.motor_output_min
        params.undershoot = (outputs[1] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[2] - params.motor_output_min).min(params.undershoot);
        // check if m0 or m3 will have output greater than params.motor_output_max
        params.overshoot = (outputs[0] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[3] - params.motor_output_max).max(params.overshoot);
        if commands.yaw + params.undershoot - params.overshoot > 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let yaw_delta = params.undershoot - params.overshoot;
            outputs[BACK_RIGHT] += yaw_delta;
            outputs[FRONT_RIGHT] -= yaw_delta;
            outputs[BACK_LEFT] -= yaw_delta;
            outputs[FRONT_LEFT] += yaw_delta;
        }
    } else if commands.yaw < 0.0 {
        // check if m0 or m3 will have output less than params.motor_output_min
        params.undershoot = (outputs[0] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[3] - params.motor_output_min).min(params.undershoot);
        // check if m1 or m2 will have output greater than params.motor_output_max
        params.overshoot = (outputs[1] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[2] - params.motor_output_max).max(params.overshoot);
        if commands.yaw - (params.undershoot - params.overshoot) < 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let yaw_delta = -(params.undershoot - params.overshoot);
            outputs[BACK_RIGHT] += yaw_delta;
            outputs[FRONT_RIGHT] -= yaw_delta;
            outputs[BACK_LEFT] -= yaw_delta;
            outputs[FRONT_LEFT] += yaw_delta;
        }
    }

    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)

    outputs
}

/// X-configuration hexacopter.
/// Motor numbering is the same as Betaflight
/// Motor rotation is "propellers out" (ie Betaflight "yaw reversed").
///
/// ```text
/// CW = clockwise
/// CCW = counter clockwise
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
#[must_use]
pub fn mix_hex_x(commands: MotorMixerCommands, params: &mut MotorMixerParameters) -> [f32; 6] {
    // NOTE: motor array indices are zero-based, whereas motor numbering in the diagram above is one-based

    // calculate the motor outputs without yaw applied
    const SIN30: f32 = 0.5;
    const SIN60: f32 = 0.866_025_4;

    let mut outputs: [f32; 6] = [
        commands.throttle - SIN60 * commands.pitch, // back right
        commands.throttle + SIN60 * commands.pitch, // front right
        commands.throttle - SIN60 * commands.pitch, // back left
        commands.throttle + SIN60 * commands.pitch, // front left
        commands.throttle,                          // center right
        commands.throttle,                          // center left
    ];

    params.throttle = commands.throttle;
    params.overshoot = 0.0;
    params.undershoot = 0.0;

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)

    // Check for overshoot caused by pitch.
    // If there is overshoot, we can just clamp the output, since this will just reduce the magnitude of the command
    // without without affecting the other axes (because of the symmetry of the HexX).
    // NOTE: motors outputs[4] and outputs[5] are not clamped, since they have no effect on pitch.
    outputs[0] = outputs[0].clamp(params.motor_output_min, params.motor_output_max);
    outputs[1] = outputs[1].clamp(params.motor_output_min, params.motor_output_max);
    outputs[2] = outputs[2].clamp(params.motor_output_min, params.motor_output_max);
    outputs[3] = outputs[4].clamp(params.motor_output_min, params.motor_output_max);

    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    outputs[0] -= SIN30 * commands.roll;
    outputs[1] -= SIN30 * commands.roll;
    outputs[2] += SIN30 * commands.roll;
    outputs[3] += SIN30 * commands.roll;
    outputs[4] -= commands.roll;
    outputs[5] += commands.roll;

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH)

    // If we have overshoot caused by roll we cannot just clamp the output, since this will affect the yaw
    if commands.roll > 0.0 {
        // check if m2, m3, or m5 will have output less than params.motor_output_min
        params.undershoot = (outputs[2] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[3] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[5] - params.motor_output_min).min(params.undershoot);
        // check if m0, m1, or m4 will have output greater than params.motor_output_max
        params.overshoot = (outputs[0] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[1] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[4] - params.motor_output_max).max(params.overshoot);
        if commands.roll + params.undershoot - params.overshoot > 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let roll_delta = params.undershoot - params.overshoot;
            outputs[0] -= roll_delta;
            outputs[1] -= roll_delta;
            outputs[2] += roll_delta;
            outputs[3] += roll_delta;
            outputs[4] -= roll_delta;
            outputs[5] += roll_delta;
        }
    } else {
        // TODO: check if the motor numbers are correct here
        // check if m2, m3, or m5 will have output less than params.motor_output_min
        params.undershoot = (outputs[2] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[3] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[5] - params.motor_output_min).min(params.undershoot);
        // check if m0, m1, or m4 will have output greater than params.motor_output_max
        params.overshoot = (outputs[0] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[1] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[4] - params.motor_output_max).max(params.overshoot);
        if commands.yaw - (params.undershoot - params.overshoot) < 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let roll_delta = -params.undershoot - params.overshoot;
            outputs[0] -= roll_delta;
            outputs[1] -= roll_delta;
            outputs[2] += roll_delta;
            outputs[3] += roll_delta;
            outputs[4] -= roll_delta;
            outputs[5] += roll_delta;
        }
    }

    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_ROLL_PITCH

    outputs[0] -= commands.yaw;
    outputs[1] -= commands.yaw;
    outputs[2] += commands.yaw;
    outputs[3] += commands.yaw;
    outputs[4] += commands.yaw;
    outputs[5] -= commands.yaw;

    //#if !defined(LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW)
    if commands.yaw > 0.0 {
        // check if m0, m1, or m5 will have output less than params.motor_output_min
        params.undershoot = (outputs[0] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[1] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[5] - params.motor_output_min).min(params.undershoot);
        // check if m2, m3, or m4 will have output greater than params.motor_output_max
        params.overshoot = (outputs[2] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[3] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[4] - params.motor_output_max).max(params.overshoot);
        if commands.yaw + (params.undershoot - params.overshoot) > 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let yaw_delta = params.undershoot - params.overshoot;
            outputs[0] -= yaw_delta;
            outputs[1] -= yaw_delta;
            outputs[2] += yaw_delta;
            outputs[3] += yaw_delta;
            outputs[4] += yaw_delta;
            outputs[5] -= yaw_delta;
        }
    } else {
        // check if m2, m3, or m4 will have output less than params.motor_output_min
        params.undershoot = (outputs[2] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[3] - params.motor_output_min).min(params.undershoot);
        params.undershoot = (outputs[4] - params.motor_output_min).min(params.undershoot);
        // check if m0, m1, or m5 will have output greater than params.motor_output_max
        params.overshoot = (outputs[1] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[3] - params.motor_output_max).max(params.overshoot);
        params.overshoot = (outputs[5] - params.motor_output_max).max(params.overshoot);
        if commands.yaw - (params.undershoot - params.overshoot) < 0.0 {
            params.throttle -= params.undershoot + params.overshoot;
            let yaw_delta = -params.undershoot - params.overshoot;
            outputs[0] -= yaw_delta;
            outputs[1] -= yaw_delta;
            outputs[2] += yaw_delta;
            outputs[3] += yaw_delta;
            outputs[4] += yaw_delta;
            outputs[5] -= yaw_delta;
        }
    }
    //#endif // LIBRARY_MOTOR_MIXERS_USE_NO_OVERFLOW_CHECKING_YAW

    outputs
}

#[cfg(test)]
mod tests {
    #![allow(clippy::float_cmp)]
    use approx::assert_abs_diff_eq;

    use super::*;

    #[allow(unused)]
    fn is_normal<T: Sized + Send + Sync + Unpin>() {}
    fn is_full<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq>() {}

    #[test]
    fn normal_types() {
        is_full::<MotorMixerCommands>();
        is_full::<MotorMixerParameters>();
    }
    #[test]
    fn test_mixer_quad_x_roll() {
        const EPSILON: f32 = 0.000_000_1;
        let mut commands = MotorMixerCommands::default();
        let mut mix_params = MotorMixerParameters::default();

        let mut outputs = mix_quad_x(commands, &mut mix_params);

        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.0, mix_params.throttle);
        assert_eq!(0.0, outputs[0]);
        assert_eq!(0.0, outputs[1]);
        assert_eq!(0.0, outputs[2]);
        assert_eq!(0.0, outputs[3]);

        commands.throttle = 0.4;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.1, outputs[1], epsilon = EPSILON); // throttle - commands.roll
        assert_abs_diff_eq!(0.7, outputs[2], epsilon = EPSILON); // throttle + commands.roll
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.roll

        commands.throttle = 0.8;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.roll
        assert_eq!(0.5, outputs[1]); // throttle - commands.roll
        assert_eq!(1.0, outputs[2]); // throttle + commands.roll
        assert_eq!(1.0, outputs[3]); // throttle + commands.roll

        commands.throttle = 0.1;
        commands.roll = 0.3;
        outputs = mix_quad_x(commands, &mut mix_params);
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
        let mut mix_params = MotorMixerParameters::default();

        commands.throttle = 0.4;
        commands.pitch = 0.3;
        let mut outputs = mix_quad_x(commands, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_abs_diff_eq!(0.1, outputs[0], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[1], epsilon = EPSILON); // throttle + commands.pitch
        assert_abs_diff_eq!(0.1, outputs[2], epsilon = EPSILON); // throttle - commands.pitch
        assert_abs_diff_eq!(0.7, outputs[3], epsilon = EPSILON); // throttle + commands.pitch

        commands.throttle = 0.8;
        commands.pitch = 0.3;
        outputs = mix_quad_x(commands, &mut mix_params);
        assert_eq!(0.0, mix_params.undershoot);
        assert_eq!(0.0, mix_params.overshoot); // pitch overshoot is ignored
        assert_eq!(0.8, mix_params.throttle);
        assert_eq!(0.5, outputs[0]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[1]); // throttle + commands.pitch
        assert_eq!(0.5, outputs[2]); // throttle - commands.pitch
        assert_eq!(1.0, outputs[3]); // throttle + commands.pitch

        commands.throttle = 0.1;
        commands.pitch = 0.3;
        outputs = mix_quad_x(commands, &mut mix_params);
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
        let mut mix_params = MotorMixerParameters::default();
        commands.throttle = 0.4;
        commands.yaw = 0.3;
        let mut outputs = mix_quad_x(commands, &mut mix_params);
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
        mix_params.motor_output_min = 0.2;
        outputs = mix_quad_x(commands, &mut mix_params);
        assert_abs_diff_eq!(-0.1, mix_params.undershoot, epsilon = EPSILON);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.5, mix_params.throttle);
        assert_eq!(0.6, outputs[0]); // throttle + commands.yaw
        assert_eq!(0.2, outputs[1]); // throttle - commands.yaw
        assert_eq!(0.2, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[3]); // throttle + commands.yaw

        // this will give an undershoot of -0.1, so commands.yaw should be adjusted to -0.2
        commands.throttle = 0.4;
        commands.yaw = -0.3;
        mix_params.motor_output_min = 0.2;
        outputs = mix_quad_x(commands, &mut mix_params);
        assert_abs_diff_eq!(-0.1, mix_params.undershoot, epsilon = EPSILON);
        assert_eq!(0.0, mix_params.overshoot);
        assert_eq!(0.5, mix_params.throttle);
        assert_eq!(0.2, outputs[0]); // throttle + commands.yaw
        assert_eq!(0.6, outputs[1]); // throttle - commands.yaw
        assert_eq!(0.6, outputs[2]); // throttle - commands.yaw
        assert_eq!(0.2, outputs[3]); // throttle + commands.yaw

        // this will give an overshoot of 0.1, so commands.yaw should be adjusted to 0.2
        commands.throttle = 0.8;
        commands.yaw = 0.3;
        mix_params.motor_output_min = 0.0;
        outputs = mix_quad_x(commands, &mut mix_params);
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
        mix_params.motor_output_min = 0.0;
        outputs = mix_quad_x(commands, &mut mix_params);
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
        let mut mix_params = MotorMixerParameters {
            motor_output_min: 0.1,
            motor_output_max: 1.0,
            max_servo_angle_radians: 60.0f32.to_radians(),
            throttle: 0.0,
            undershoot: 0.0,
            overshoot: 0.0,
        };

        commands.throttle = 0.4;
        let outputs = mix_tricopter(commands, &mut mix_params);
        assert_eq!(0.3, mix_params.undershoot);
        assert_eq!(-0.6, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.4, outputs[REAR]);
        assert_eq!(0.0, outputs[S0]);

        commands.yaw = 0.3;
        let outputs = mix_tricopter(commands, &mut mix_params);
        assert_eq!(0.3, mix_params.undershoot);
        assert_eq!(-0.579_415_1, mix_params.overshoot);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_eq!(0.420_584_89, outputs[REAR]);
        assert_eq!(0.3, outputs[S0]);

        commands.yaw = 1.0;
        let outputs = mix_tricopter(commands, &mut mix_params);
        assert_eq!(0.3, mix_params.undershoot);
        assert_abs_diff_eq!(-0.2, mix_params.overshoot, epsilon = EPSILON);
        assert_eq!(0.4, mix_params.throttle);
        assert_eq!(0.4, outputs[FL]);
        assert_eq!(0.4, outputs[FR]);
        assert_abs_diff_eq!(0.8, outputs[REAR], epsilon = EPSILON);
        assert_eq!(1.0, outputs[S0]);
    }
}
