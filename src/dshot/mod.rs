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
mod dshot_bidirectional_frame;
mod dshot_error;
mod erpm_telemetry_frame;
mod esc_dshot;
mod gcr_frame;
mod protocol;
mod telemetry;

#[allow(unused)]
pub use esc_dshot::EscDshot;

pub use command::Command;
pub use dshot_bidirectional_frame::DshotBidirectionalFrame;
pub use dshot_error::{DecodeError, DshotError};
pub use erpm_telemetry_frame::ErpmTelemetryFrame;
pub use gcr_frame::GcrFrame;
pub use protocol::Protocol;
#[allow(unused)]
pub use telemetry::{Telemetry, TelemetryType};
