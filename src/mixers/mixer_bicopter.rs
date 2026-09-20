use crate::MotorMixerCommands;

/// Bicopter: two tilt-adjustable rotors.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MixerBicopter {}

impl MixerBicopter {
    pub const MOTOR_COUNT_U8: u8 = 1;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 4;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self {}
    }
}

impl MixerBicopter {
    #[inline]
    #[must_use]
    pub const fn mix(commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        let outputs: [f32; Self::OUTPUT_COUNT] = [
            commands.throttle + commands.roll, // motor left
            commands.throttle - commands.roll, // motor right
            commands.pitch - commands.yaw,     // servo left
            commands.pitch + commands.yaw,     // servo right
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
        is_full_eq::<MixerBicopter>();
    }
}
