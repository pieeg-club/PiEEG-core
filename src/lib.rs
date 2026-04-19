mod bandpass;
mod decode;
mod hampel;
mod ring;

use pyo3::prelude::*;

const VERSION: &str = env!("CARGO_PKG_VERSION");

// ── HampelFilter binding ────────────────────────────────────────────

#[pyclass(name = "HampelFilter")]
struct PyHampelFilter {
    inner: hampel::HampelFilter,
}

#[pymethods]
impl PyHampelFilter {
    #[new]
    #[pyo3(signature = (num_channels=16, window_size=5, n_sigma=3.0, enabled=true))]
    fn new(num_channels: usize, window_size: usize, n_sigma: f64, enabled: bool) -> Self {
        Self {
            inner: hampel::HampelFilter::new(num_channels, window_size, n_sigma, enabled),
        }
    }

    /// Filter one multi-channel sample.
    fn apply(&mut self, channels: Vec<f64>) -> Vec<f64> {
        self.inner.apply(&channels)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }

    fn config(&self) -> PyResult<PyObject> {
        Python::with_gil(|py| {
            let dict = pyo3::types::PyDict::new_bound(py);
            dict.set_item("enabled", self.inner.enabled())?;
            dict.set_item("window_size", self.inner.window_size())?;
            dict.set_item("n_sigma", self.inner.n_sigma())?;
            dict.set_item("replaced_count", self.inner.replaced_count())?;
            dict.set_item("native", true)?;
            Ok(dict.into())
        })
    }

    // ── Properties (match Python API exactly) ──

    #[getter]
    fn enabled(&self) -> bool {
        self.inner.enabled()
    }

    #[setter]
    fn set_enabled(&mut self, v: bool) {
        self.inner.set_enabled(v);
    }

    #[getter]
    fn window_size(&self) -> usize {
        self.inner.window_size()
    }

    #[setter]
    fn set_window_size(&mut self, v: usize) {
        self.inner.set_window_size(v);
    }

    #[getter]
    fn n_sigma(&self) -> f64 {
        self.inner.n_sigma()
    }

    #[setter]
    fn set_n_sigma(&mut self, v: f64) {
        self.inner.set_n_sigma(v);
    }

    #[getter]
    fn replaced_count(&self) -> u64 {
        self.inner.replaced_count()
    }
}

// ── MultichannelFilter binding ──────────────────────────────────────

#[pyclass(name = "MultichannelFilter")]
struct PyMultichannelFilter {
    inner: bandpass::MultichannelFilter,
}

#[pymethods]
impl PyMultichannelFilter {
    #[new]
    #[pyo3(signature = (num_channels=16, lowcut=1.0, highcut=40.0, fs=250.0, order=5))]
    fn new(num_channels: usize, lowcut: f64, highcut: f64, fs: f64, order: usize) -> Self {
        Self {
            inner: bandpass::MultichannelFilter::new(num_channels, lowcut, highcut, fs, order),
        }
    }

    /// Filter a single multi-channel sample.
    fn apply_sample(&mut self, channels: Vec<f64>) -> Vec<f64> {
        self.inner.apply_sample(&channels)
    }

    /// Filter a block of samples.
    fn apply_block(&mut self, block: Vec<Vec<f64>>) -> Vec<Vec<f64>> {
        self.inner.apply_block(&block)
    }

    fn reset(&mut self) {
        self.inner.reset();
    }
}

// ── decode_channels binding ─────────────────────────────────────────

/// Decode 8 channels from a 27-byte SPI read buffer.
/// Returns a list of microvolts (float).
#[pyfunction]
fn decode_channels(raw: Vec<u8>) -> Vec<f64> {
    decode::decode_channels(&raw)
}

// ── Module ──────────────────────────────────────────────────────────

#[pymodule]
fn pieeg_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", VERSION)?;
    m.add_class::<PyHampelFilter>()?;
    m.add_class::<PyMultichannelFilter>()?;
    m.add_function(wrap_pyfunction!(decode_channels, m)?)?;
    Ok(())
}
