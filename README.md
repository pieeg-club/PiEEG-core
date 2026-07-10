# PiEEG Core

Native accelerator for [PiEEG-server](https://github.com/pieeg-club/PiEEG-server). Compiled Rust replacements for hot paths.

> ⚠️ **Early development · experimental · no warranties.** This project is in early development — APIs, behavior, and packaging may change without notice. Research/hobbyist software provided **as-is**, without warranty of any kind, express or implied. Not a medical device. Not intended for clinical, diagnostic, or safety-critical use. Use at your own risk. See [LICENSE](LICENSE) for full terms.

## What it accelerates

| Component | What | Speedup |
|-----------|------|---------|
| `MultichannelFilter` | Butterworth IIR bandpass (SOS cascade) | ~1057x |
| `HampelFilter` | Per-channel spike rejection (median + MAD) | ~15x |
| `decode_channels` | 24-bit ADC → µV conversion | ~9x |

When installed, `pieeg-server` detects it and switches to the compiled implementations automatically. No config needed.

On the Raspberry Pi it additionally provides a **native acquisition loop** (`NativeAcquisition`) that owns the SPI/GPIO hardware directly — see [Native acquisition](#native-acquisition-raspberry-pi).

## Benchmark

| Hot path | Python | pieeg-core | Speedup |
|----------|--------|------------|---------|
| `MultichannelFilter` (bandpass 1–40 Hz) | 992 samples/s | 1,048,658 samples/s | ~1057× |
| `HampelFilter` (spike removal) | 24,619 samples/s | 358,361 samples/s | ~15× |
| `decode_channels` (24-bit SPI → µV) | 121,352 samples/s | 1,134,301 samples/s | ~9× |

## Native acquisition (Raspberry Pi)

Beyond the drop-in DSP accelerators, pieeg-core ships a hard-real-time acquisition loop that reads the ADS1299(s) directly. A dedicated OS thread owns the SPI buses and GPIO lines, polls DRDY, reads both ADC chips, decodes to microvolts and pushes frames into a lock-free single-producer/single-consumer queue — **all with the GIL released**. Acquisition timing is therefore fully decoupled from CPython's GIL and garbage collector.

This path is compiled only into the Linux/Raspberry Pi wheels (built with the `hardware` Cargo feature); it depends on `spidev` and `gpio-cdev`, so it is absent from the Windows/macOS wheels.

```python
from pieeg_core import NativeAcquisition

acq = NativeAcquisition(
    num_channels=16,      # 8 (one ADS1299) or 16 (two, PiEEG-16)
    sample_rate=250,      # 250, 500, 1000, 2000 or 4000 SPS
    gpiochip="/dev/gpiochip4",
    spi_speed_hz=4_000_000,
    cs_pin=19,            # chip-select for ADC 2 (16-ch mode)
    drdy_pin=26,          # DRDY for ADC 1
    drdy2_pin=13,         # DRDY for ADC 2
    queue_capacity=0,     # 0 → auto (~4 s of headroom)
)

acq.start()               # opens hardware, configures the ADC(s), spawns the read thread
try:
    while True:
        # Drain up to max_frames buffered samples in FIFO order.
        for timestamp, channels in acq.read_batch(max_frames=256):
            ...           # channels is a list of microvolts, len == num_channels

        if acq.dropped:   # samples lost to consumer back-pressure (ring overflow)
            ...           # counted, never silent
finally:
    acq.stop()            # signals the read thread and joins it
```

### API

| Member | Description |
|--------|-------------|
| `start()` | Open the hardware, configure the ADC(s) and spawn the read thread. |
| `read_batch(max_frames=256)` | Drain up to `max_frames` buffered samples as a list of `(timestamp, [ch1..chN])` tuples in FIFO order. Releases the GIL. |
| `stop()` | Signal the read thread to stop and join it. Also runs on drop. |
| `pending` | Frames currently buffered and waiting to be read. |
| `dropped` | Frames dropped because the consumer fell behind (ring overflow). |

The register-configuration sequence is a faithful port of `pieeg_server`'s `PiEEGHardware._configure_adc`, with the CONFIG1 data rate parameterised so you can select 250–4000 SPS. Each frame carries a monotonically increasing sequence index internally, so any upstream sample loss is recorded rather than hidden.

> The SPI/GPIO paths can only run on a Raspberry Pi with a PiEEG shield. The non-hardware logic (queue, decode, register mapping) is unit-tested off-device.

## Install

```
pip install pieeg-core
```

Pre-built wheels are published for Linux (x86_64, aarch64, armv7 — including Raspberry Pi), Windows (x86_64) and macOS (arm64) on CPython 3.10–3.13.

## Build from source

Requires [Rust](https://rustup.rs) and [maturin](https://www.maturin.rs):

```
pip install maturin
cd PiEEG-core
maturin develop --release
```

To include the native acquisition loop (Raspberry Pi only), enable the `hardware` feature:

```
maturin develop --release --features hardware
```

Build inside the same Python environment where `pieeg-server` is installed.
