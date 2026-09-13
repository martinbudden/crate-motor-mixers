#![allow(unused)]

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    /// Throttle value out of range (must be 0-1999).
    InvalidThrottle,
    /// Telemetry CRC checksum mismatch.
    InvalidTelemetryChecksum,
    InvalidTelemetryData,
    PioTxTimeout,
    /// ESC did not respond to telemetry request in time.
    PioRxTimeout,
    /// Invalid GCR encoding in telemetry response.
    GcrDecodeError,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    NoData,
    InvalidRunLength,
    GcrData,
    InvalidChecksum,
    Erpm,
    _TelemetryType,
}
