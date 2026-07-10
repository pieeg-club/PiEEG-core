/// Lock-free sample queue for the native acquisition loop.
///
/// A single producer (the Rust SPI read thread) pushes decoded frames; a
/// single consumer (the Python side, via `read_batch`) drains them. Built on
/// `crossbeam_queue::ArrayQueue` so the producer never blocks and never takes
/// the GIL — that decoupling is the whole point of the native path.
///
/// Overflow is **counted, never silent**: if the consumer falls behind and the
/// ring fills, the newest frame is dropped and `dropped` is incremented so the
/// loss is visible in logs / CSV metadata (we never repeat the silent
/// data-loss trap of wall-clock timestamps).
///
/// This module has no hardware dependency and is unit-tested off-device.

use std::sync::atomic::{AtomicU64, Ordering};

use crossbeam_queue::ArrayQueue;

/// Maximum channel count a single frame can carry (PiEEG-16 = 2 × 8).
pub const MAX_CH: usize = 16;

/// One decoded multi-channel sample.
///
/// `Copy` + fixed-size array → zero heap allocation on the hot path.
#[derive(Clone, Copy)]
pub struct Frame {
    /// Monotonically increasing sample index (starts at 1). A gap here means
    /// samples were dropped upstream — the honest record of continuity.
    pub seq: u64,
    /// Wall-clock timestamp (seconds since UNIX epoch) taken at read time.
    pub t: f64,
    /// Number of valid channels in `data`.
    pub nch: u8,
    /// Channel values in microvolts; only the first `nch` entries are valid.
    pub data: [f64; MAX_CH],
}

impl Frame {
    #[inline]
    pub fn new(seq: u64, t: f64, channels: &[f64]) -> Self {
        let mut data = [0.0f64; MAX_CH];
        let n = channels.len().min(MAX_CH);
        data[..n].copy_from_slice(&channels[..n]);
        Self {
            seq,
            t,
            nch: n as u8,
            data,
        }
    }
}

/// Single-producer / single-consumer sample queue with an overflow counter.
pub struct SampleQueue {
    q: ArrayQueue<Frame>,
    dropped: AtomicU64,
}

impl SampleQueue {
    /// Create a queue holding up to `capacity` frames.
    pub fn new(capacity: usize) -> Self {
        Self {
            q: ArrayQueue::new(capacity.max(1)),
            dropped: AtomicU64::new(0),
        }
    }

    /// Push a frame. If the ring is full, drop the frame and count it.
    #[inline]
    pub fn push(&self, frame: Frame) {
        if self.q.push(frame).is_err() {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Pop the oldest frame, if any.
    #[inline]
    pub fn pop(&self) -> Option<Frame> {
        self.q.pop()
    }

    /// Number of frames dropped due to consumer back-pressure.
    #[inline]
    pub fn dropped(&self) -> u64 {
        self.dropped.load(Ordering::Relaxed)
    }

    /// Current number of buffered frames.
    #[inline]
    pub fn len(&self) -> usize {
        self.q.len()
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.q.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_copies_channels() {
        let f = Frame::new(1, 0.5, &[1.0, 2.0, 3.0]);
        assert_eq!(f.seq, 1);
        assert_eq!(f.nch, 3);
        assert_eq!(&f.data[..3], &[1.0, 2.0, 3.0]);
    }

    #[test]
    fn frame_truncates_beyond_max() {
        let big = vec![7.0f64; MAX_CH + 4];
        let f = Frame::new(9, 1.0, &big);
        assert_eq!(f.nch as usize, MAX_CH);
    }

    #[test]
    fn push_pop_fifo() {
        let q = SampleQueue::new(4);
        q.push(Frame::new(1, 0.0, &[10.0]));
        q.push(Frame::new(2, 0.0, &[20.0]));
        assert_eq!(q.pop().unwrap().seq, 1);
        assert_eq!(q.pop().unwrap().seq, 2);
        assert!(q.pop().is_none());
    }

    #[test]
    fn overflow_is_counted_not_silent() {
        let q = SampleQueue::new(2);
        q.push(Frame::new(1, 0.0, &[1.0]));
        q.push(Frame::new(2, 0.0, &[2.0]));
        q.push(Frame::new(3, 0.0, &[3.0])); // full → dropped
        assert_eq!(q.dropped(), 1);
        assert_eq!(q.len(), 2);
    }
}
