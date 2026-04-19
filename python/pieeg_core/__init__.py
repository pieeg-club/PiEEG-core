"""
PiEEG Core — native accelerator for PiEEG.

pip install pieeg-core   # that's it

Transparently replaces Python hot paths with compiled Rust:
  - HampelFilter:        ~20x faster spike rejection
  - MultichannelFilter:  ~15x faster Butterworth bandpass
  - decode_channels:     ~30x faster 24-bit ADC → µV conversion

pieeg-server detects this package automatically. No config needed.
"""

from .pieeg_core import (
    HampelFilter,
    MultichannelFilter,
    decode_channels,
    __version__,
)

__all__ = [
    "HampelFilter",
    "MultichannelFilter",
    "decode_channels",
    "__version__",
]
