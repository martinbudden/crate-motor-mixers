#![doc = include_str!("README.md")]
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

mod command;
mod dshot_codec;
mod dshot_error;
mod esc_dshot;
mod protocol;
mod telemetry_type;

pub use command::Command;
pub use telemetry_type::TelemetryType;
pub use dshot_codec::DshotCodec;
#[allow(unused)]
pub use dshot_error::DshotError;
#[allow(unused)]
pub use esc_dshot::EscDshot;
pub use protocol::Protocol;
