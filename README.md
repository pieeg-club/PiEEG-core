# PiEEG Core

Private native accelerator for [PiEEG-server](https://github.com/pieeg-club/PiEEG-server). Compiled Rust replacements for hot paths — not published on PyPI.

## What it accelerates

| Component | What | Speedup |
|-----------|------|---------|
| `HampelFilter` | Per-channel spike rejection (median + MAD) | ~20x |
| `MultichannelFilter` | Butterworth IIR bandpass (SOS cascade) | ~15x |
| `decode_channels` | 24-bit ADC → µV conversion | ~30x |

When installed, `pieeg-server` detects it and switches to the compiled implementations automatically. No config needed.

## Setup

Requires [Rust](https://rustup.rs) and [maturin](https://www.maturin.rs):

```
pip install maturin
cd PiEEG-core
maturin develop --release
```

Build inside the same Python environment where `pieeg-server` is installed.
