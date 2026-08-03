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

mod dshot_codec;
mod dynamic_idle_controller;

mod commands;
mod mixer;
mod mixer_calculations;
mod mixer_config;
mod mixer_quad_x_dshot;
mod mixer_quad_x_pwm;
mod mixer_quad_x_pwm_drivers;

mod rpm_notch_filters;
mod rpm_notch_filters_state_machine;

pub use mixer::{MotorMixer, MotorMixerCommon, MotorMixerDriver, MotorMixerOutput};

pub use commands::{MotorMixerCommands, MotorMixerMessage};

pub use mixer_config::{
    MixerConfig, MixerType, MotorConfig, MotorDeviceConfig, MotorMixerParameters, MotorProtocol, ProtocolFamily,
    ServoConfig, ServoDeviceConfig,
};

#[cfg(feature = "eight_motors")]
pub use mixer_calculations::mix_hex_x;
pub use mixer_calculations::{mix_airplane, mix_bicopter, mix_quad_x, mix_tricopter, mix_wing};

pub use mixer_quad_x_pwm::MotorMixerQuadXPwm;

pub use mixer_quad_x_dshot::MotorMixerQuadXDshot;

pub use rpm_notch_filters::{
    MotorFrequencies, RpmNotchFilterBank, RpmNotchFilterBankConfig, RpmNotchFilterFrequencies, RpmNotchFilters,
};

pub use dynamic_idle_controller::{DynamicIdleController, DynamicIdleControllerConfig, RpmHz};

pub use dshot_codec::DshotCodec;
