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
mod dshot_decoder;
mod dshot_encoder;
mod dshot_error;
mod esc_dshot;
mod dshot_bidirectional_frame;
mod protocol;
mod telemetry_type;

pub use command::Command;
#[allow(unused)]
pub use dshot_decoder::{DecodeError, DshotDecoder};
#[allow(unused)]
pub use dshot_error::DshotError;
#[allow(unused)]
pub use esc_dshot::EscDshot;
#[allow(unused)]
pub use dshot_bidirectional_frame::DshotBidirectionalFrame;
pub use protocol::Protocol;
pub use telemetry_type::TelemetryType;
