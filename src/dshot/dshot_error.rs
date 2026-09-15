#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    TxTimeout,
    /// ESC did not respond to telemetry request in time.
    RxTimeout,
    NoDecodeData,
    InvalidRunLength,
    InvalidGcr20Data,
    InvalidChecksum,
    InvalidErpm,
}
