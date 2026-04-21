# PiEEG Core

Native accelerator for [PiEEG-server](https://github.com/pieeg-club/PiEEG-server). Compiled Rust replacements for hot paths.

## What it accelerates

| Component | What | Speedup |
|-----------|------|---------|
| `HampelFilter` | Per-channel spike rejection (median + MAD) | ~20x |
| `MultichannelFilter` | Butterworth IIR bandpass (SOS cascade) | ~15x |
| `decode_channels` | 24-bit ADC → µV conversion | ~30x |

When installed, `pieeg-server` detects it and switches to the compiled implementations automatically. No config needed.

## Install

```
pip install pieeg-core
```

Pre-built wheels are published for Linux (x86_64, aarch64, armv7 — including Raspberry Pi), Windows (x86_64) and macOS (x86_64, arm64) on CPython 3.10–3.13.

## Build from source

Requires [Rust](https://rustup.rs) and [maturin](https://www.maturin.rs):

```
pip install maturin
cd PiEEG-core
maturin develop --release
```

Build inside the same Python environment where `pieeg-server` is installed.
