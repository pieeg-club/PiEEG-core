//! ADS1299 register-value helpers shared by the native acquisition loop.
//!
//! Kept free of hardware dependencies so the mapping logic compiles and is
//! unit-tested on any platform, independent of the `hardware` feature.

/// Map an output data rate (SPS) to the ADS1299 CONFIG1 register value.
///
/// CONFIG1 = 0x90 | DR[2:0]; DR table (datasheet): 010=4k, 011=2k, 100=1k,
/// 101=500, 110=250. Unknown rates fall back to 250 SPS (the safe default).
pub fn sample_rate_to_config1(sample_rate: u32) -> u8 {
    let dr = match sample_rate {
        4000 => 0b010,
        2000 => 0b011,
        1000 => 0b100,
        500 => 0b101,
        250 => 0b110,
        _ => 0b110, // default 250 SPS
    };
    0x90 | dr
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config1_matches_datasheet_rates() {
        assert_eq!(sample_rate_to_config1(250), 0x96);
        assert_eq!(sample_rate_to_config1(500), 0x95);
        assert_eq!(sample_rate_to_config1(1000), 0x94);
        assert_eq!(sample_rate_to_config1(2000), 0x93);
        assert_eq!(sample_rate_to_config1(4000), 0x92);
    }

    #[test]
    fn config1_unknown_rate_defaults_to_250() {
        assert_eq!(sample_rate_to_config1(1234), 0x96);
    }
}
