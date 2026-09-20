use crate::MotorMixerCommands;

/// Mixer for airplane (ie throttle, ailerons, elevator, and rudder).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MixerAirplane {}

impl MixerAirplane {
    pub const MOTOR_COUNT_U8: u8 = 1;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 5;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {}
    }
}

impl MixerAirplane {
    #[inline]
    #[must_use]
    pub const fn mix(commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        let outputs: [f32; Self::OUTPUT_COUNT] = [
            commands.throttle, // throttle may be controlled by a servo for a wing with an internal combustion engine
            commands.roll,     // left aileron
            -commands.roll,    // right aileron
            commands.pitch,    // elevator
            commands.yaw,      // rudder
        ];
        outputs
    }
}

#[cfg(test)]
mod test_traits {
    use super::*;

    fn is_full_eq<T: Sized + Send + Sync + Unpin + Copy + Clone + Default + PartialEq + Eq>() {}

    #[test]
    fn normal_types() {
        is_full_eq::<MixerAirplane>();
    }
}
