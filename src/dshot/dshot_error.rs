#![allow(unused)]

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    /// Throttle value out of range (must be 0-1999).
    InvalidThrottle,
    /// Telemetry CRC checksum mismatch.
    InvalidTelemetryCrc,
    /// ESC did not respond to telemetry request in time.
    TelemetryTimeout,
    /// Invalid GCR encoding in telemetry response.
    GcrDecodeError,
}
