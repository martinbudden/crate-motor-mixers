#![doc = include_str!("../README.md")]
#![no_std]
#![deny(clippy::unwrap_used)]
#![deny(clippy::expect_used)]
#![deny(clippy::panic)]
//#![deny(missing_docs)]
#![deny(
    missing_copy_implementations,
    missing_debug_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unused_must_use,
    unused_extern_crates,
    unused_import_braces,
    unused_qualifications,
    unused_results
)]
#![warn(unused_results)]
#![warn(clippy::pedantic)]
#![warn(clippy::doc_paragraphs_missing_punctuation)]

mod drivers;
pub mod dshot_rp;
mod mixers;

mod dynamic_idle_controller;

mod mixer_commands;

mod motor_driver;

mod mixer_config;
mod motor_mixer;

mod rpm_notch_filters;
mod rpm_notch_filters_state_machine;

pub use mixer_commands::{MotorMixerCommands, MotorMixerMessage};

pub use mixer_config::{
    MixerConfig, MixerType, MotorConfig, MotorDeviceConfig, MotorOutputRange, MotorProtocol, ProtocolFamily,
    ServoConfig, ServoDeviceConfig, YawCompensationStrategy,
};

pub use motor_driver::MotorDriver;

pub use drivers::{MotorDriverDshot, MotorDriverPwm};

pub use motor_mixer::{
    DshotCommands, MAX_SUPPORTED_MOTOR_COUNT, MotorFrequencies, MotorMixer, MotorOutputFilters, MotorOutputs,
    MotorSaturation,
};

pub use rpm_notch_filters::{RpmNotchFilterBank, RpmNotchFilterBankConfig, RpmNotchFilterFrequencies, RpmNotchFilters};

pub use dynamic_idle_controller::{DynamicIdleController, DynamicIdleControllerConfig, RpmHz};
