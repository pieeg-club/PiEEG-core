/// Fixed-size ring buffer with O(1) push and O(k log k) sorted snapshot.
///
/// Used by the Hampel filter — window sizes are tiny (5–9),
/// so sorting is cheaper than maintaining a heap.

#[derive(Clone)]
pub struct RingBuf {
    buf: Vec<f64>,
    head: usize,
    len: usize,
}

impl RingBuf {
    pub fn new(capacity: usize) -> Self {
        Self {
            buf: vec![0.0; capacity],
            head: 0,
            len: 0,
        }
    }

    #[inline]
    pub fn capacity(&self) -> usize {
        self.buf.len()
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.len == self.buf.len()
    }

    /// Push a value, overwriting the oldest if full.
    #[inline]
    pub fn push(&mut self, val: f64) {
        self.buf[self.head] = val;
        self.head = (self.head + 1) % self.buf.len();
        if self.len < self.buf.len() {
            self.len += 1;
        }
    }

    /// Copy current contents into `dst` (unordered) and return the count.
    /// `dst` must have at least `self.len()` elements.
    #[inline]
    pub fn copy_to(&self, dst: &mut [f64]) -> usize {
        let n = self.len;
        let cap = self.buf.len();
        // Start index of oldest element
        let start = if n < cap { 0 } else { self.head };
        for i in 0..n {
            dst[i] = self.buf[(start + i) % cap];
        }
        n
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_and_read() {
        let mut rb = RingBuf::new(3);
        rb.push(1.0);
        rb.push(2.0);
        rb.push(3.0);
        assert!(rb.is_full());

        let mut dst = [0.0; 3];
        let n = rb.copy_to(&mut dst);
        assert_eq!(n, 3);
        dst.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(dst, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn overwrite_oldest() {
        let mut rb = RingBuf::new(3);
        rb.push(1.0);
        rb.push(2.0);
        rb.push(3.0);
        rb.push(4.0); // overwrites 1.0

        let mut dst = [0.0; 3];
        rb.copy_to(&mut dst);
        dst.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(dst, [2.0, 3.0, 4.0]);
    }
}
