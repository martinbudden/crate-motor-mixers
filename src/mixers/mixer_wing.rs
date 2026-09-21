use super::{MotorMixerCommands, MotorOutputRange};

/// Mixer for flying wing (ie throttle and flaperons).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MixerWing {
    range: MotorOutputRange,
}

impl Default for MixerWing {
    fn default() -> Self {
        Self::new()
    }
}

impl MixerWing {
    pub const MOTOR_COUNT_U8: u8 = 1;
    pub const MOTOR_COUNT: usize = Self::MOTOR_COUNT_U8 as usize;
    pub const OUTPUT_COUNT_U8: u8 = 3;
    pub const OUTPUT_COUNT: usize = Self::OUTPUT_COUNT_U8 as usize;

    /// Constructor.
    #[must_use]
    pub const fn new() -> Self {
        Self { range: MotorOutputRange::new() }
    }
    /// Set the range of a newly constructed tricopter.
    #[must_use]
    pub const fn with_range(mut self, range: MotorOutputRange) -> Self {
        self.set_range(range);
        self
    }
}

impl MixerWing {
    #[inline]
    pub const fn set_range(&mut self, range: MotorOutputRange) {
        self.range = range;
    }
    #[must_use]
    pub const fn range(self) -> MotorOutputRange {
        self.range
    }
    #[must_use]
    pub fn mix(&mut self, commands: MotorMixerCommands) -> [f32; Self::OUTPUT_COUNT] {
        let mut outputs: [f32; Self::OUTPUT_COUNT] = [
            commands.throttle, // throttle may be controlled by a servo for a wing with an internal combustion engine
            commands.roll + commands.pitch, // left flaperon
            -commands.roll + commands.pitch, // right flaperon
        ];

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
        is_full::<MixerWing>();
    }
}
