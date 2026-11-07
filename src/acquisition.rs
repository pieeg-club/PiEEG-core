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
use std::os::unix::io::AsRawFd;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use gpio_cdev::{Chip, EventRequestFlags, LineEventHandle, LineHandle, LineRequestFlags};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use spidev::{SpiModeFlags, Spidev, SpidevOptions};

use crate::adc_config::sample_rate_to_config1;
use crate::decode::decode_channels;
use crate::spsc::{Frame, SampleQueue, MAX_CH};

const BYTES_PER_READ: usize = 27; // 3 status + 8 channels × 3 bytes

/// Poll timeout (ms) on the DRDY event fd so `stop` is honoured even when the
/// ADC stops producing edges (e.g. after a reset). Small enough for a snappy
/// shutdown, large enough to add no measurable overhead during acquisition.
const STOP_POLL_TIMEOUT_MS: i32 = 100;

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
    cs2_pin: u32,
    drdy_pin: u32,
    drdy2_pin: u32,
    drdy3_pin: u32,
    /// SCHED_FIFO real-time priority for the read thread (1–99). `0` disables
    /// the elevation and the thread stays at normal scheduling.
    rt_priority: i32,
    /// CPU core to pin the read thread to. `-1` leaves it unpinned.
    cpu_affinity: i32,
}

/// Owns all hardware handles and runs the read loop on its own thread.
struct Reader {
    spi1: Spidev,
    spi2: Option<Spidev>,
    /// DRDY for chip 1, subscribed to kernel falling-edge events. Blocking on
    /// the kernel event queue (rather than busy-polling the level) means a
    /// late thread never *misses* an edge — the kernel latches every one.
    drdy1: LineEventHandle,
    drdy2: Option<LineHandle>,
    _drdy3: Option<LineHandle>,
    /// Chip-select for ADC chip 2 (GPIO19 on the PiEEG shield). `None` in
    /// 8-channel mode where the kernel drives CE0 for chip 1.
    cs: Option<LineHandle>,
    /// Chip-select for ADC chip 3 (GPIO21 on the PiEEG-24 shield).
    cs2: Option<LineHandle>,
    num_channels: usize,
    rt_priority: i32,
    cpu_affinity: i32,
    queue: Arc<SampleQueue>,
    bad_frames: Arc<[AtomicU64; 3]>,
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

    #[inline]
    fn cs2_set(&self, value: u8) {
        if let Some(cs2) = &self.cs2 {
            let _ = cs2.set_value(value);
        }
    }

    /// Wait for chip 2 DRDY to fall (HIGH→LOW = data ready), matching
    /// `_wait_drdy2` in the Python driver.
    #[inline]
    fn wait_drdy2(&self) {
        if let Some(d2) = &self.drdy2 {
            // wait out previous low
            while d2.get_value().unwrap_or(1) == 0 {
                if self.stop.load(Ordering::Relaxed) {
                    return;
                }
                std::hint::spin_loop();
            }
            // wait for HIGH→LOW
            while d2.get_value().unwrap_or(0) == 1 {
                if self.stop.load(Ordering::Relaxed) {
                    return;
                }
                std::hint::spin_loop();
            }
        }
    }

    /// Read one sample (8, 16, or 24 channels) into `out`, returning the channel
    /// count. Returns `None` on a status-header mismatch (corrupted frame) and
    /// increments the per-chip `bad_frames` counter. Writing into a caller-owned
    /// stack buffer keeps the hot path allocation-free.
    fn read_sample(&mut self, out: &mut [f64; MAX_CH]) -> Option<usize> {
        let mut raw1 = [0u8; BYTES_PER_READ];
        self.spi1.read(&mut raw1).ok()?;
        let ok1 = (raw1[0] & 0xF0) == 0xC0;
        if !ok1 {
            self.bad_frames[0].fetch_add(1, Ordering::Relaxed);
        }
        out[..8].copy_from_slice(&decode_channels(&raw1));

        if self.num_channels == 16 {
            self.wait_drdy2();
            self.cs_set(0);
            let mut raw2 = [0u8; BYTES_PER_READ];
            let res = self.spi2.as_mut().unwrap().read(&mut raw2);
            self.cs_set(1);
            res.ok()?;

            if (raw2[0] & 0xF0) != 0xC0 {
                self.bad_frames[1].fetch_add(1, Ordering::Relaxed);
            }
            if (raw2[0], raw2[1], raw2[2]) != EXPECTED_STATUS {
                return None;
            }
            out[8..16].copy_from_slice(&decode_channels(&raw2));
            Some(16)
        } else if self.num_channels == 24 {
            // On PiEEG-24 all 3 chips share chip 1's clock and start in
            // 3 -> 2 -> 1 order, so when chip 1's DRDY falls, chips 2 and 3
            // already have their conversion ready.
            self.cs_set(0);
            let mut raw2 = [0u8; BYTES_PER_READ];
            let res2 = self.spi2.as_mut().unwrap().read(&mut raw2);
            self.cs_set(1);
            res2.ok()?;

            self.cs2_set(0);
            let mut raw3 = [0u8; BYTES_PER_READ];
            let res3 = self.spi2.as_mut().unwrap().read(&mut raw3);
            self.cs2_set(1);
            res3.ok()?;

            let ok2 = (raw2[0] & 0xF0) == 0xC0;
            let ok3 = (raw3[0] & 0xF0) == 0xC0;
            if !ok2 {
                self.bad_frames[1].fetch_add(1, Ordering::Relaxed);
            }
            if !ok3 {
                self.bad_frames[2].fetch_add(1, Ordering::Relaxed);
            }
            if !ok1 || !ok2 || !ok3 {
                return None;
            }

            out[8..16].copy_from_slice(&decode_channels(&raw2));
            out[16..24].copy_from_slice(&decode_channels(&raw3));
            Some(24)
        } else {
            if !ok1 {
                return None;
            }
            Some(8)
        }
    }

    /// Wait on kernel DRDY falling-edge events and read until stopped.
    ///
    /// The kernel timestamps and queues every falling edge, so — unlike the
    /// old level-polling arm/trigger state machine — a momentarily-late thread
    /// services the next queued edge instead of silently skipping a
    /// conversion. A `poll()` with a short timeout keeps `stop` responsive even
    /// if DRDY goes quiet (e.g. the ADC is reset mid-run).
    fn run(mut self) {
        // Best-effort real-time tuning; logged on failure, never fatal.
        apply_realtime(self.rt_priority, self.cpu_affinity);

        let drdy_fd = self.drdy1.as_raw_fd();
        // Reused across every sample so the read loop never allocates.
        let mut buf = [0.0f64; MAX_CH];
        while !self.stop.load(Ordering::Relaxed) {
            match poll_readable(drdy_fd, STOP_POLL_TIMEOUT_MS) {
                PollResult::Ready => {}
                PollResult::Timeout => continue,
                PollResult::Error => {
                    // Transient poll error — back off briefly, re-check stop.
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }
            }

            // Consume the queued edge record so the fd clears for the next one.
            if self.drdy1.get_event().is_err() {
                continue;
            }

            // Count every triggered sample so gaps in `seq` reliably signal
            // decode failures (corrupted frames), matching the documented
            // contract. Only successfully-decoded frames are pushed.
            self.seq += 1;
            if let Some(n) = self.read_sample(&mut buf) {
                let t = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0);
                self.queue.push(Frame::new(self.seq, t, &buf[..n]));
            }
        }
    }
}

/// Result of polling the DRDY event fd.
enum PollResult {
    Ready,
    Timeout,
    Error,
}

/// Block until `fd` is readable or `timeout_ms` elapses.
fn poll_readable(fd: std::os::unix::io::RawFd, timeout_ms: i32) -> PollResult {
    let mut pfd = libc::pollfd {
        fd,
        events: libc::POLLIN,
        revents: 0,
    };
    // SAFETY: a single, valid pollfd is passed with a matching count of 1.
    let rc = unsafe { libc::poll(&mut pfd as *mut libc::pollfd, 1, timeout_ms) };
    match rc {
        n if n > 0 => PollResult::Ready,
        0 => PollResult::Timeout,
        _ => PollResult::Error,
    }
}

/// Best-effort real-time tuning for the calling (read) thread.
///
/// Every step is optional: on failure (usually insufficient privileges) it logs
/// a warning to stderr and continues at normal scheduling, so acquisition still
/// runs — just with more jitter. Grant `CAP_SYS_NICE` (or run as root) for the
/// full effect. This is what turns the ~16% jittery loss at 2 kSPS into a clean
/// stream: the reader can no longer be preempted mid-sample by the Python drain,
/// the CSV writer or OS housekeeping.
fn apply_realtime(rt_priority: i32, cpu_affinity: i32) {
    // Lock the process's pages so the RT loop never stalls on a page fault.
    // SAFETY: plain libc call with constant flags.
    if unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) } != 0 {
        eprintln!(
            "[pieeg-native] mlockall failed ({}); continuing without memory locking",
            std::io::Error::last_os_error()
        );
    }

    // Pin to a specific core to avoid cross-core migration jitter.
    if cpu_affinity >= 0 {
        // SAFETY: the set is zero-initialised, then populated before the call;
        // sched_setaffinity reads exactly `size_of::<cpu_set_t>()` bytes.
        unsafe {
            let mut set: libc::cpu_set_t = std::mem::zeroed();
            libc::CPU_ZERO(&mut set);
            libc::CPU_SET(cpu_affinity as usize, &mut set);
            if libc::sched_setaffinity(0, std::mem::size_of::<libc::cpu_set_t>(), &set) != 0 {
                eprintln!(
                    "[pieeg-native] pin to core {cpu_affinity} failed ({}); continuing unpinned",
                    std::io::Error::last_os_error()
                );
            }
        }
    }

    // Elevate to SCHED_FIFO so the reader preempts normal work and isn't
    // preempted by it — the key to jitter-free DRDY servicing.
    if rt_priority > 0 {
        let param = libc::sched_param {
            sched_priority: rt_priority,
        };
        // SAFETY: pid 0 targets the calling thread; `param` outlives the call.
        if unsafe { libc::sched_setscheduler(0, libc::SCHED_FIFO, &param) } != 0 {
            eprintln!(
                "[pieeg-native] SCHED_FIFO(prio {rt_priority}) failed ({}); run as root or \
                 grant CAP_SYS_NICE for jitter-free acquisition — continuing at normal priority",
                std::io::Error::last_os_error()
            );
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
///
/// When `clk_en` is true (chip 1 on PiEEG-24), sets `CONFIG1.CLK_EN` (bit 5)
/// immediately after `SDATAC` so the internal oscillator drives the shared
/// `CLK` net for chips 2 and 3.
fn configure_chip(
    spi: &mut Spidev,
    cs: Option<&LineHandle>,
    sample_rate: u32,
    clk_en: bool,
    start_immediately: bool,
) -> std::io::Result<()> {
    let cfg1 = sample_rate_to_config1(sample_rate) | if clk_en { 0x20 } else { 0x00 };

    send_cmd(spi, cs, CMD_WAKEUP)?;
    send_cmd(spi, cs, CMD_STOP)?;
    send_cmd(spi, cs, CMD_RESET)?;
    thread::sleep(Duration::from_millis(10)); // let the ADC settle after reset
    send_cmd(spi, cs, CMD_SDATAC)?;
    if clk_en {
        thread::sleep(Duration::from_millis(1));
        write_reg(spi, cs, CONFIG1, cfg1)?;
    }

    write_reg(spi, cs, 0x14, 0x80)?; // GPIO
    write_reg(spi, cs, CONFIG1, cfg1)?;
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

    if start_immediately {
        send_cmd(spi, cs, CMD_RDATAC)?;
        send_cmd(spi, cs, CMD_START)?;
    }
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
    bad_frames: Arc<[AtomicU64; 3]>,
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
        rt_priority = 80,
        cpu_affinity = -1,
        cs2_pin = 21,
        drdy3_pin = 5,
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
        rt_priority: i32,
        cpu_affinity: i32,
        cs2_pin: u32,
        drdy3_pin: u32,
    ) -> PyResult<Self> {
        if num_channels != 8 && num_channels != 16 && num_channels != 24 {
            return Err(PyRuntimeError::new_err(
                "num_channels must be 8, 16, or 24",
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
                cs2_pin,
                drdy_pin,
                drdy2_pin,
                drdy3_pin,
                rt_priority,
                cpu_affinity,
            },
            queue: Arc::new(SampleQueue::new(cap)),
            bad_frames: Arc::new([AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0)]),
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

        // Subscribe to kernel falling-edge events on DRDY1: every conversion's
        // edge is latched and timestamped by the kernel, so the read thread can
        // block on it via poll() and never miss one to scheduling jitter.
        let drdy1 = chip
            .get_line(cfg.drdy_pin)
            .and_then(|l| {
                l.events(
                    LineRequestFlags::INPUT,
                    EventRequestFlags::FALLING_EDGE,
                    "pieeg_drdy",
                )
            })
            .map_err(|e| PyRuntimeError::new_err(format!("DRDY line {}: {e}", cfg.drdy_pin)))?;

        let mut spi1 = open_spi("/dev/spidev0.0", cfg.spi_speed_hz)
            .map_err(|e| PyRuntimeError::new_err(format!("open spidev0.0: {e}")))?;

        // Chip 2 (16/24-channel mode) and Chip 3 (24-channel mode).
        let (mut spi2, drdy2, drdy3, cs, cs2) = if cfg.num_channels == 16 || cfg.num_channels == 24 {
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
            let (drdy3, cs2) = if cfg.num_channels == 24 {
                let d3 = chip
                    .get_line(cfg.drdy3_pin)
                    .and_then(|l| l.request(LineRequestFlags::INPUT, 0, "pieeg_drdy3"))
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!("DRDY3 line {}: {e}", cfg.drdy3_pin))
                    })?;
                let c2 = chip
                    .get_line(cfg.cs2_pin)
                    .and_then(|l| l.request(LineRequestFlags::OUTPUT, 1, "pieeg_cs2"))
                    .map_err(|e| PyRuntimeError::new_err(format!("CS2 line {}: {e}", cfg.cs2_pin)))?;
                (Some(d3), Some(c2))
            } else {
                (None, None)
            };
            (Some(spi2), Some(drdy2), drdy3, Some(cs), cs2)
        } else {
            (None, None, None, None, None)
        };

        // Configure ADC(s).
        if cfg.num_channels == 24 {
            // Chip 1 outputs the shared CLK net (CONFIG1.CLK_EN=1). Wait 200 ms
            // for chips 2/3 power-on reset, configure them, then start 3 -> 2 -> 1.
            configure_chip(&mut spi1, None, cfg.sample_rate, true, false)
                .map_err(|e| PyRuntimeError::new_err(format!("configure chip 1: {e}")))?;
            thread::sleep(Duration::from_millis(200));
            let s2 = spi2.as_mut().unwrap();
            let c1 = cs.as_ref().unwrap();
            let c2 = cs2.as_ref().unwrap();
            configure_chip(s2, Some(c1), cfg.sample_rate, false, false)
                .map_err(|e| PyRuntimeError::new_err(format!("configure chip 2: {e}")))?;
            configure_chip(s2, Some(c2), cfg.sample_rate, false, false)
                .map_err(|e| PyRuntimeError::new_err(format!("configure chip 3: {e}")))?;
            // Start all in 3 -> 2 -> 1 order
            send_cmd(s2, Some(c2), CMD_RDATAC)
                .map_err(|e| PyRuntimeError::new_err(format!("RDATAC chip 3: {e}")))?;
            send_cmd(s2, Some(c1), CMD_RDATAC)
                .map_err(|e| PyRuntimeError::new_err(format!("RDATAC chip 2: {e}")))?;
            send_cmd(&mut spi1, None, CMD_RDATAC)
                .map_err(|e| PyRuntimeError::new_err(format!("RDATAC chip 1: {e}")))?;
            send_cmd(s2, Some(c2), CMD_START)
                .map_err(|e| PyRuntimeError::new_err(format!("START chip 3: {e}")))?;
            send_cmd(s2, Some(c1), CMD_START)
                .map_err(|e| PyRuntimeError::new_err(format!("START chip 2: {e}")))?;
            send_cmd(&mut spi1, None, CMD_START)
                .map_err(|e| PyRuntimeError::new_err(format!("START chip 1: {e}")))?;
        } else {
            configure_chip(&mut spi1, None, cfg.sample_rate, false, true)
                .map_err(|e| PyRuntimeError::new_err(format!("configure chip 1: {e}")))?;
            if let (Some(spi2), Some(cs)) = (spi2.as_mut(), cs.as_ref()) {
                configure_chip(spi2, Some(cs), cfg.sample_rate, false, true)
                    .map_err(|e| PyRuntimeError::new_err(format!("configure chip 2: {e}")))?;
            }
        }

        self.stop.store(false, Ordering::Relaxed);
        for b in self.bad_frames.iter() {
            b.store(0, Ordering::Relaxed);
        }
        let reader = Reader {
            spi1,
            spi2,
            drdy1,
            drdy2,
            _drdy3: drdy3,
            cs,
            cs2,
            num_channels: cfg.num_channels,
            rt_priority: cfg.rt_priority,
            cpu_affinity: cfg.cpu_affinity,
            queue: Arc::clone(&self.queue),
            bad_frames: Arc::clone(&self.bad_frames),
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

    /// Per-chip bad frame counts (frames where status byte was not 0xC_).
    #[getter]
    fn bad_frames(&self) -> [u64; 3] {
        [
            self.bad_frames[0].load(Ordering::Relaxed),
            self.bad_frames[1].load(Ordering::Relaxed),
            self.bad_frames[2].load(Ordering::Relaxed),
        ]
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
