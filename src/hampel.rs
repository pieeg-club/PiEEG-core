/// Hampel spike filter — API-compatible drop-in for pieeg_server.spike_filter.
///
/// Per-channel median + MAD outlier rejection.
/// At window_size=5 and 16 channels, this does 16 tiny sorts per sample.
/// In Rust that's ~200ns total vs ~50µs in Python.

use crate::ring::RingBuf;

const MAD_SCALE: f64 = 1.4826;
const MIN_MAD: f64 = 2.0;

pub struct HampelFilter {
    num_channels: usize,
    window_size: usize,
    n_sigma: f64,
    enabled: bool,
    buffers: Vec<RingBuf>,
    replaced_count: u64,
    // Scratch buffer for median/MAD computation — avoids per-call allocation
    scratch: Vec<f64>,
}

impl HampelFilter {
    pub fn new(num_channels: usize, window_size: usize, n_sigma: f64, enabled: bool) -> Self {
        let ws = Self::normalize_window(window_size);
        Self {
            num_channels,
            window_size: ws,
            n_sigma: n_sigma.max(1.0),
            enabled,
            buffers: (0..num_channels).map(|_| RingBuf::new(ws)).collect(),
            replaced_count: 0,
            scratch: vec![0.0; ws],
        }
    }

    fn normalize_window(ws: usize) -> usize {
        let ws = ws.max(3);
        if ws % 2 == 0 { ws + 1 } else { ws }
    }

    #[inline]
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn set_enabled(&mut self, v: bool) {
        self.enabled = v;
    }

    #[inline]
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    pub fn set_window_size(&mut self, ws: usize) {
        let ws = Self::normalize_window(ws);
        if ws != self.window_size {
            self.window_size = ws;
            self.scratch.resize(ws, 0.0);
            self.reset();
        }
    }

    #[inline]
    pub fn n_sigma(&self) -> f64 {
        self.n_sigma
    }

    pub fn set_n_sigma(&mut self, v: f64) {
        self.n_sigma = v.max(1.0);
    }

    #[inline]
    pub fn replaced_count(&self) -> u64 {
        self.replaced_count
    }

    pub fn reset(&mut self) {
        for buf in &mut self.buffers {
            *buf = RingBuf::new(self.window_size);
        }
    }

    /// Filter one multi-channel sample. Returns filtered values.
    /// API matches Python: `apply(channels: list[float]) -> list[float]`
    #[inline]
    pub fn apply(&mut self, channels: &[f64]) -> Vec<f64> {
        let n = channels.len().min(self.num_channels);
        let mut result = channels.to_vec();

        if !self.enabled {
            for ch in 0..n {
                self.buffers[ch].push(channels[ch]);
            }
            return result;
        }

        for ch in 0..n {
            let buf = &self.buffers[ch];
            let val = channels[ch];

            if buf.is_full() {
                let count = buf.copy_to(&mut self.scratch);
                let median = Self::median_sorted(&mut self.scratch[..count]);
                let mad = Self::mad_sorted(&self.scratch[..count], median);
                let threshold = self.n_sigma * MAD_SCALE * mad.max(MIN_MAD);

                if (val - median).abs() > threshold {
                    result[ch] = median;
                    self.replaced_count += 1;
                }
            }

            // Push original value (preserves statistical window integrity)
            self.buffers[ch].push(val);
        }

        result
    }

    /// Median of a mutable slice (sorts in place).
    #[inline]
    fn median_sorted(data: &mut [f64]) -> f64 {
        data.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let n = data.len();
        if n % 2 == 1 {
            data[n / 2]
        } else {
            (data[n / 2 - 1] + data[n / 2]) / 2.0
        }
    }

    /// MAD (Median Absolute Deviation). Expects `data` already sorted
    /// from median_sorted call — recomputes deviations and sorts.
    #[inline]
    fn mad_sorted(data: &[f64], median: f64) -> f64 {
        // data.len() is small (5-9), stack-allocate via small vec
        let mut devs: Vec<f64> = data.iter().map(|&v| (v - median).abs()).collect();
        devs.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let n = devs.len();
        if n % 2 == 1 {
            devs[n / 2]
        } else {
            (devs[n / 2 - 1] + devs[n / 2]) / 2.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_spike_passes_through() {
        let mut f = HampelFilter::new(1, 5, 3.0, true);
        // Fill the window with steady values
        for _ in 0..5 {
            f.apply(&[100.0]);
        }
        let out = f.apply(&[101.0]);
        assert!((out[0] - 101.0).abs() < 1e-9);
    }

    #[test]
    fn spike_replaced_with_median() {
        let mut f = HampelFilter::new(1, 5, 3.0, true);
        for _ in 0..5 {
            f.apply(&[100.0]);
        }
        let out = f.apply(&[9999.0]);
        // Should be replaced with median ≈ 100.0
        assert!((out[0] - 100.0).abs() < 1.0);
        assert_eq!(f.replaced_count(), 1);
    }

    #[test]
    fn disabled_passes_through() {
        let mut f = HampelFilter::new(1, 5, 3.0, false);
        for _ in 0..5 {
            f.apply(&[100.0]);
        }
        let out = f.apply(&[9999.0]);
        assert!((out[0] - 9999.0).abs() < 1e-9);
        assert_eq!(f.replaced_count(), 0);
    }
}
