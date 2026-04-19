"""Tests that verify pieeg-core is a drop-in replacement for pieeg-server's Python implementations."""

import math
import pieeg_core


# ── HampelFilter ─────────────────────────────────────────────────────

def test_hampel_passthrough():
    """Normal values pass through unchanged."""
    f = pieeg_core.HampelFilter(1, 5, 3.0, True)
    for _ in range(5):
        f.apply([100.0])
    out = f.apply([101.0])
    assert abs(out[0] - 101.0) < 1e-9


def test_hampel_spike_replaced():
    """A spike is replaced with the window median."""
    f = pieeg_core.HampelFilter(1, 5, 3.0, True)
    for _ in range(5):
        f.apply([100.0])
    out = f.apply([9999.0])
    assert abs(out[0] - 100.0) < 1.0
    assert f.replaced_count == 1


def test_hampel_disabled():
    """When disabled, spikes pass through."""
    f = pieeg_core.HampelFilter(1, 5, 3.0, False)
    for _ in range(5):
        f.apply([100.0])
    out = f.apply([9999.0])
    assert abs(out[0] - 9999.0) < 1e-9
    assert f.replaced_count == 0


def test_hampel_multichannel():
    """Works with 16 channels simultaneously."""
    f = pieeg_core.HampelFilter(16, 5, 3.0, True)
    sample = [50.0] * 16
    for _ in range(5):
        f.apply(sample)
    # Spike on channel 3 only
    spike = [50.0] * 16
    spike[3] = 9999.0
    out = f.apply(spike)
    assert abs(out[3] - 50.0) < 1.0  # replaced
    assert abs(out[0] - 50.0) < 1e-9  # untouched


def test_hampel_properties():
    """Properties match Python API."""
    f = pieeg_core.HampelFilter(16, 5, 3.0, True)
    assert f.enabled is True
    assert f.window_size == 5
    assert f.n_sigma == 3.0
    f.enabled = False
    assert f.enabled is False
    f.window_size = 7
    assert f.window_size == 7
    f.n_sigma = 4.0
    assert f.n_sigma == 4.0


def test_hampel_config():
    """config() returns a dict with native=True."""
    f = pieeg_core.HampelFilter(16, 5, 3.0, True)
    cfg = f.config()
    assert cfg["native"] is True
    assert cfg["enabled"] is True
    assert cfg["window_size"] == 5


def test_hampel_reset():
    """reset() clears internal state."""
    f = pieeg_core.HampelFilter(1, 5, 3.0, True)
    for _ in range(5):
        f.apply([100.0])
    f.reset()
    # After reset, window is empty — no filtering happens
    out = f.apply([9999.0])
    assert abs(out[0] - 9999.0) < 1e-9


# ── MultichannelFilter ───────────────────────────────────────────────

def test_filter_passes_sine():
    """A 10 Hz sine (in passband) survives the filter."""
    f = pieeg_core.MultichannelFilter(1, 1.0, 40.0, 250.0, 5)
    last = 0.0
    for i in range(500):
        t = i / 250.0
        x = math.sin(2 * math.pi * 10 * t) * 100.0
        out = f.apply_sample([x])
        last = out[0]
    assert abs(last) > 10.0, f"10 Hz should pass, got {last}"


def test_filter_rejects_dc():
    """DC is rejected by the bandpass."""
    f = pieeg_core.MultichannelFilter(1, 1.0, 40.0, 250.0, 5)
    last = 0.0
    for _ in range(500):
        out = f.apply_sample([100.0])
        last = out[0]
    assert abs(last) < 5.0, f"DC should be rejected, got {last}"


def test_filter_multichannel():
    """16-channel filtering works."""
    f = pieeg_core.MultichannelFilter(16, 1.0, 40.0, 250.0, 5)
    sample = [0.0] * 16
    out = f.apply_sample(sample)
    assert len(out) == 16


def test_filter_block():
    """apply_block processes multiple samples."""
    f = pieeg_core.MultichannelFilter(2, 1.0, 40.0, 250.0, 5)
    block = [[10.0, 20.0]] * 10
    out = f.apply_block(block)
    assert len(out) == 10
    assert len(out[0]) == 2


def test_filter_reset():
    """reset() clears filter state."""
    f = pieeg_core.MultichannelFilter(1, 1.0, 40.0, 250.0, 5)
    for _ in range(100):
        f.apply_sample([100.0])
    f.reset()
    out = f.apply_sample([0.0])
    assert abs(out[0]) < 1e-9


# ── decode_channels ──────────────────────────────────────────────────

def test_decode_zeros():
    """All-zero data bytes → 0 µV."""
    raw = bytes(27)
    ch = pieeg_core.decode_channels(raw)
    assert len(ch) == 8
    assert all(abs(v) < 0.01 for v in ch)


def test_decode_positive():
    """0x7FFFFF → large positive µV."""
    raw = bytearray(27)
    raw[3] = 0x7F
    raw[4] = 0xFF
    raw[5] = 0xFF
    ch = pieeg_core.decode_channels(raw)
    assert ch[0] > 2_000_000.0


def test_decode_negative():
    """0x800000 → negative µV."""
    raw = bytearray(27)
    raw[3] = 0x80
    raw[4] = 0x00
    raw[5] = 0x00
    ch = pieeg_core.decode_channels(raw)
    assert ch[0] < 0.0


def test_decode_length():
    """Always returns 8 channels from 27 bytes."""
    raw = bytes(27)
    assert len(pieeg_core.decode_channels(raw)) == 8
