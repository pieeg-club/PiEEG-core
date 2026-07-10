//! Native SPI acquisition loop for PiEEG (ADS1299), feature-gated behind
//! `hardware`.
//!
//! This is the hard-real-time path: a dedicated OS thread owns the SPI buses
//! and GPIO lines, polls DRDY, reads both ADC chips, decodes to microvolts and
//! pushes frames into a lock-free [`SampleQueue`] — all with the GIL released.
//! Python drains the queue in batches, so acquisition timing is fully
//! decoupled from CPython's GIL and garbage collector.
//!
//! The register-configuration sequence is a faithful port of
//! `pieeg_server.hardware.PiEEGHardware._configure_adc`, with CONFIG1's data
//! rate parameterised so power users can select 250 – 4000 SPS.
//!
//! Testing note: the SPI/GPIO paths can only be exercised on a Raspberry Pi
//! with a PiEEG shield. The non-hardware logic (queue, decode, CONFIG1 mapping)
//! is unit-tested off-device.

use std::io::{Read, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use gpio_cdev::{Chip, LineHandle, LineRequestFlags};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use spidev::{SpiModeFlags, Spidev, SpidevOptions};

use crate::adc_config::sample_rate_to_config1;
use crate::decode::decode_channels;
use crate::spsc::{Frame, SampleQueue};

const BYTES_PER_READ: usize = 27; // 3 status + 8 channels × 3 bytes

// --- ADS1299 register addresses ---
const CONFIG1: u8 = 0x01;
const CONFIG2: u8 = 0x02;
const CONFIG3: u8 = 0x03;

// --- ADS1299 commands ---
const CMD_WAKEUP: u8 = 0x02;
const CMD_RESET: u8 = 0x06;
const CMD_START: u8 = 0x08;
const CMD_STOP: u8 = 0x0A;
const CMD_RDATAC: u8 = 0x10;
const CMD_SDATAC: u8 = 0x11;

// Expected status header from ADC chip 2 (0xC0, 0x00, 0x08).
const EXPECTED_STATUS: (u8, u8, u8) = (192, 0, 8);

/// Immutable configuration captured from Python at construction time.
#[derive(Clone)]
struct Config {
    num_channels: usize,
    sample_rate: u32,
    gpiochip: String,
    spi_speed_hz: u32,
    cs_pin: u32,
    drdy_pin: u32,
    drdy2_pin: u32,
}

/// Owns all hardware handles and runs the read loop on its own thread.
struct Reader {
    spi1: Spidev,
    spi2: Option<Spidev>,
    drdy1: LineHandle,
    drdy2: Option<LineHandle>,
    /// Chip-select for ADC chip 2 (GPIO19 on the PiEEG shield). `None` in
    /// 8-channel mode where the kernel drives CE0 for chip 1.
    cs: Option<LineHandle>,
    num_channels: usize,
    queue: Arc<SampleQueue>,
    stop: Arc<AtomicBool>,
    seq: u64,
}

impl Reader {
    #[inline]
    fn cs_set(&self, value: u8) {
        if let Some(cs) = &self.cs {
            let _ = cs.set_value(value);
        }
    }

    /// Wait for chip 2 DRDY to fall (HIGH→LOW = data ready), matching
    /// `_wait_drdy2` in the Python driver.
    #[inline]
    fn wait_drdy2(&self) {
        if let Some(d2) = &self.drdy2 {
            while d2.get_value().unwrap_or(1) == 0 {} // wait out previous low
            while d2.get_value().unwrap_or(0) == 1 {} // wait for HIGH→LOW
        }
    }

    /// Read one sample (8 or 16 channels). Returns `None` on a status-header
    /// mismatch from chip 2 (corrupted frame).
    fn read_sample(&mut self) -> Option<Vec<f64>> {
        let mut raw1 = [0u8; BYTES_PER_READ];
        self.spi1.read(&mut raw1).ok()?;
        let mut channels = decode_channels(&raw1);

        if self.num_channels == 16 {
            self.wait_drdy2();
            self.cs_set(0);
            let mut raw2 = [0u8; BYTES_PER_READ];
            let res = self.spi2.as_mut().unwrap().read(&mut raw2);
            self.cs_set(1);
            res.ok()?;

            if (raw2[0], raw2[1], raw2[2]) != EXPECTED_STATUS {
                return None;
            }
            channels.extend(decode_channels(&raw2));
        }
        Some(channels)
    }

    /// Busy-poll DRDY and read until stopped. Mirrors the Python
    /// `_run_hardware` state machine (arm on DRDY high, trigger on DRDY low)
    /// but with no GIL and no per-sample asyncio hop.
    fn run(mut self) {
        let mut armed = false;
        while !self.stop.load(Ordering::Relaxed) {
            let drdy = self.drdy1.get_value().unwrap_or(1);

            if !armed {
                if drdy == 1 {
                    armed = true;
                }
                continue;
            }
            if drdy != 0 {
                continue;
            }
            armed = false;

            if let Some(channels) = self.read_sample() {
                self.seq += 1;
                let t = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0);
                self.queue.push(Frame::new(self.seq, t, &channels));
            }
        }
    }
}

// --- one-shot SPI helpers (used during ADC configuration) ---

fn send_cmd(spi: &mut Spidev, cs: Option<&LineHandle>, cmd: u8) -> std::io::Result<()> {
    if let Some(c) = cs {
        let _ = c.set_value(0);
    }
    let res = spi.write(&[cmd]);
    if let Some(c) = cs {
        let _ = c.set_value(1);
    }
    res.map(|_| ())
}

fn write_reg(spi: &mut Spidev, cs: Option<&LineHandle>, reg: u8, val: u8) -> std::io::Result<()> {
    if let Some(c) = cs {
        let _ = c.set_value(0);
    }
    let res = spi.write(&[0x40 | reg, 0x00, val]);
    if let Some(c) = cs {
        let _ = c.set_value(1);
    }
    res.map(|_| ())
}

/// Full ADC initialisation for one chip — port of `_configure_adc`, with the
/// data rate driven by `sample_rate`.
fn configure_chip(
    spi: &mut Spidev,
    cs: Option<&LineHandle>,
    sample_rate: u32,
) -> std::io::Result<()> {
    send_cmd(spi, cs, CMD_WAKEUP)?;
    send_cmd(spi, cs, CMD_STOP)?;
    send_cmd(spi, cs, CMD_RESET)?;
    thread::sleep(Duration::from_millis(10)); // let the ADC settle after reset
    send_cmd(spi, cs, CMD_SDATAC)?;

    write_reg(spi, cs, 0x14, 0x80)?; // GPIO
    write_reg(spi, cs, CONFIG1, sample_rate_to_config1(sample_rate))?;
    write_reg(spi, cs, CONFIG2, 0xD4)?;
    write_reg(spi, cs, CONFIG3, 0xFF)?;
    write_reg(spi, cs, 0x04, 0x00)?;
    write_reg(spi, cs, 0x0D, 0x00)?;
    write_reg(spi, cs, 0x0E, 0x00)?;
    write_reg(spi, cs, 0x0F, 0x00)?;
    write_reg(spi, cs, 0x10, 0x00)?;
    write_reg(spi, cs, 0x11, 0x00)?;
    write_reg(spi, cs, 0x15, 0x20)?;
    write_reg(spi, cs, 0x17, 0x00)?;

    // Enable all 8 channels with default gain (CH1SET..CH8SET = 0x05..0x0C).
    for ch_reg in 0x05u8..=0x0C {
        write_reg(spi, cs, ch_reg, 0x00)?;
    }

    send_cmd(spi, cs, CMD_RDATAC)?;
    send_cmd(spi, cs, CMD_START)?;
    Ok(())
}

fn open_spi(path: &str, speed_hz: u32) -> std::io::Result<Spidev> {
    let mut spi = Spidev::open(path)?;
    let opts = SpidevOptions::new()
        .bits_per_word(8)
        .max_speed_hz(speed_hz)
        .mode(SpiModeFlags::SPI_MODE_1) // mode 0b01, MSB-first
        .build();
    spi.configure(&opts)?;
    Ok(spi)
}

/// Python-facing handle for the native acquisition loop.
///
/// Lifecycle: construct with config → `start()` (opens hardware, configures the
/// ADC(s), spawns the read thread) → `read_batch()` repeatedly → `stop()`.
#[pyclass(name = "NativeAcquisition")]
pub struct PyNativeAcquisition {
    cfg: Config,
    queue: Arc<SampleQueue>,
    stop: Arc<AtomicBool>,
    handle: Option<JoinHandle<()>>,
}

#[pymethods]
impl PyNativeAcquisition {
    #[new]
    #[pyo3(signature = (
        num_channels = 16,
        sample_rate = 250,
        gpiochip = "/dev/gpiochip4".to_string(),
        spi_speed_hz = 4_000_000,
        cs_pin = 19,
        drdy_pin = 26,
        drdy2_pin = 13,
        queue_capacity = 0,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        num_channels: usize,
        sample_rate: u32,
        gpiochip: String,
        spi_speed_hz: u32,
        cs_pin: u32,
        drdy_pin: u32,
        drdy2_pin: u32,
        queue_capacity: usize,
    ) -> PyResult<Self> {
        if num_channels != 8 && num_channels != 16 {
            return Err(PyRuntimeError::new_err(
                "num_channels must be 8 or 16",
            ));
        }
        // Default: ~4 seconds of headroom so Python draining every few ms
        // never overflows the ring.
        let cap = if queue_capacity == 0 {
            (sample_rate as usize * 4).max(4096)
        } else {
            queue_capacity
        };
        Ok(Self {
            cfg: Config {
                num_channels,
                sample_rate,
                gpiochip,
                spi_speed_hz,
                cs_pin,
                drdy_pin,
                drdy2_pin,
            },
            queue: Arc::new(SampleQueue::new(cap)),
            stop: Arc::new(AtomicBool::new(false)),
            handle: None,
        })
    }

    /// Open the hardware, configure the ADC(s) and spawn the read thread.
    fn start(&mut self) -> PyResult<()> {
        if self.handle.is_some() {
            return Err(PyRuntimeError::new_err("acquisition already started"));
        }
        let cfg = self.cfg.clone();

        let mut chip = Chip::new(&cfg.gpiochip)
            .map_err(|e| PyRuntimeError::new_err(format!("open {}: {e}", cfg.gpiochip)))?;

        let drdy1 = chip
            .get_line(cfg.drdy_pin)
            .and_then(|l| l.request(LineRequestFlags::INPUT, 0, "pieeg_drdy"))
            .map_err(|e| PyRuntimeError::new_err(format!("DRDY line {}: {e}", cfg.drdy_pin)))?;

        let mut spi1 = open_spi("/dev/spidev0.0", cfg.spi_speed_hz)
            .map_err(|e| PyRuntimeError::new_err(format!("open spidev0.0: {e}")))?;

        // Chip 2 (16-channel mode) — second SPI device plus GPIO19 chip-select
        // and its own DRDY line.
        let (mut spi2, drdy2, cs) = if cfg.num_channels == 16 {
            let spi2 = open_spi("/dev/spidev0.1", cfg.spi_speed_hz)
                .map_err(|e| PyRuntimeError::new_err(format!("open spidev0.1: {e}")))?;
            let drdy2 = chip
                .get_line(cfg.drdy2_pin)
                .and_then(|l| l.request(LineRequestFlags::INPUT, 0, "pieeg_drdy2"))
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("DRDY2 line {}: {e}", cfg.drdy2_pin))
                })?;
            let cs = chip
                .get_line(cfg.cs_pin)
                .and_then(|l| l.request(LineRequestFlags::OUTPUT, 1, "pieeg_cs"))
                .map_err(|e| PyRuntimeError::new_err(format!("CS line {}: {e}", cfg.cs_pin)))?;
            (Some(spi2), Some(drdy2), Some(cs))
        } else {
            (None, None, None)
        };

        // Configure ADC(s). Chip 1 uses the kernel-driven CE0 (no manual CS);
        // chip 2 is selected via GPIO19.
        configure_chip(&mut spi1, None, cfg.sample_rate)
            .map_err(|e| PyRuntimeError::new_err(format!("configure chip 1: {e}")))?;
        if let (Some(spi2), Some(cs)) = (spi2.as_mut(), cs.as_ref()) {
            configure_chip(spi2, Some(cs), cfg.sample_rate)
                .map_err(|e| PyRuntimeError::new_err(format!("configure chip 2: {e}")))?;
        }

        self.stop.store(false, Ordering::Relaxed);
        let reader = Reader {
            spi1,
            spi2,
            drdy1,
            drdy2,
            cs,
            num_channels: cfg.num_channels,
            queue: Arc::clone(&self.queue),
            stop: Arc::clone(&self.stop),
            seq: 0,
        };

        // Release the GIL for the lifetime of the spawn call; the thread runs
        // independently and never touches Python.
        let handle = thread::Builder::new()
            .name("pieeg-native-acq".to_string())
            .spawn(move || reader.run())
            .map_err(|e| PyRuntimeError::new_err(format!("spawn read thread: {e}")))?;
        self.handle = Some(handle);
        Ok(())
    }

    /// Drain up to `max_frames` buffered samples.
    ///
    /// Returns a list of `(seq, timestamp, [ch1..chN])` tuples in FIFO order.
    /// A jump in `seq` between consecutive `read_batch` calls means samples
    /// were dropped (ring overflow) — the gap is visible, never hidden.
    #[pyo3(signature = (max_frames = 256))]
    fn read_batch(&self, py: Python<'_>, max_frames: usize) -> Vec<(u64, f64, Vec<f64>)> {
        py.allow_threads(|| {
            let mut out = Vec::with_capacity(max_frames.min(self.queue.len()));
            for _ in 0..max_frames {
                match self.queue.pop() {
                    Some(f) => out.push((f.seq, f.t, f.data[..f.nch as usize].to_vec())),
                    None => break,
                }
            }
            out
        })
    }

    /// Frames dropped because the consumer fell behind (ring overflow).
    #[getter]
    fn dropped(&self) -> u64 {
        self.queue.dropped()
    }

    /// Frames currently buffered and waiting to be read.
    #[getter]
    fn pending(&self) -> usize {
        self.queue.len()
    }

    /// Signal the read thread to stop and join it.
    fn stop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for PyNativeAcquisition {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}
