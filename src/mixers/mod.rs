#![allow(unused)]

mod dshot_commands;
mod mixer_airplane;
mod mixer_bicopter;
mod mixer_commands;
pub(crate) mod mixer_config;
mod mixer_hexacopter;
mod mixer_octocopter;
mod mixer_quadcopter;
mod mixer_tricopter;
mod mixer_wing;
mod motor_frequencies;
mod motor_output_filters;
mod motor_output_range;
mod motor_outputs;

pub use mixer_airplane::MixerAirplane;
pub use mixer_bicopter::MixerBicopter;
pub use mixer_wing::MixerWing;

pub use mixer_tricopter::MixerTricopter;

pub use mixer_hexacopter::MixerHexacopter;
pub use mixer_octocopter::MixerOctocopter;
pub use mixer_quadcopter::MixerQuadcopter;

pub use dshot_commands::DshotCommands;
pub use mixer_commands::{MotorMixerCommands, MotorMixerMessage};
/*pub use mixer_config::{
    MixerConfig, MixerType, MotorConfig, MotorDeviceConfig, MotorProtocol, ProtocolFamily,
    ServoConfig, ServoDeviceConfig,
};*/
pub use motor_frequencies::MotorFrequencies;
pub use motor_output_filters::MotorOutputFilters;
pub use motor_output_range::{MotorOutputRange, SaturationCompensation};
pub use motor_outputs::{MAX_SUPPORTED_MOTOR_COUNT, MotorOutputs};
