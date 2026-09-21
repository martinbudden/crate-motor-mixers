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
mod mixer_config;
mod motor_driver;
mod motor_mixer;
mod motor_output_filters;
mod rpm_notch_filters;
mod rpm_notch_filters_state_machine;

pub use drivers::{MAX_SUPPORTED_MOTOR_COUNT, MotorDriverDshot, MotorDriverPwm, MotorFrequencies, MotorOutputs};
pub use mixers::{MotorMixerCommands, MotorMixerMessage, MotorOutputRange, SaturationCompensation};

pub use motor_driver::MotorDriver;

pub use mixer_config::{
    MixerConfig, MixerType, MotorConfig, MotorDeviceConfig, MotorProtocol, ServoConfig, ServoDeviceConfig,
};
pub use motor_mixer::MotorMixer;
pub use motor_output_filters::MotorOutputFilters;

pub use rpm_notch_filters::{RpmNotchFilterBank, RpmNotchFilterBankConfig, RpmNotchFilterFrequencies, RpmNotchFilters};

pub use dynamic_idle_controller::{DynamicIdleController, DynamicIdleControllerConfig, RpmHz};
