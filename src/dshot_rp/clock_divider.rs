use fixed::{FixedU32, types::extra::U8};

use crate::dshot::Protocol;

#[allow(unused)]
pub fn tx_pio_clock_divider(protocol: Protocol, sys_clock_frequency: u32) -> FixedU32<U8> {
    let sys_clock = u64::from(sys_clock_frequency);
    #[allow(clippy::cast_possible_truncation)]
    FixedU32::<U8>::from_bits(((sys_clock << 8) / (8 * u64::from(protocol.baud_rate()))) as u32)
}

#[allow(unused)]
pub fn bidir_pio_clock_divider(protocol: Protocol, sys_clock_frequency: u32) -> FixedU32<U8> {
    // pio clock divider = system_clock / (40 × protocol_baud_rate) encoded as FixedU32<U8>

    let sys_clock = u64::from(sys_clock_frequency);
    let target = 12_000_000u64 * u64::from(protocol.baud_rate()) / 300_000;
    #[allow(clippy::cast_possible_truncation)]
    FixedU32::<U8>::from_bits(((sys_clock << 8) / target) as u32)
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dshot_bidir_divider_at_125mhz() {
        // Verify bidir divider: target PIO clock = 12MHz * dshot_speed/300kHz
        // DShot600: target = 12MHz * 600/300 = 24MHz
        // At 125MHz: divider = 125/24 = 5.2083...
        const SYS_CLOCK: u32 = 125_000_000;
        let divider = bidir_pio_clock_divider(Protocol::Dshot600, SYS_CLOCK);
        let bits = (125 << 8) / 24;
        assert_eq!(1333, bits);
        let expected: FixedU32<U8> = FixedU32::from_bits(bits);

        assert_eq!(expected, divider);
    }
}
