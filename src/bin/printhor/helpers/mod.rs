use crate::hwa;
use crate::math::Real;

#[allow(unused)]
#[inline]
pub(crate) fn to_fixed(val: (i32, u8)) -> Real {
    Real::new(val.0.into(), val.1 as u32)
}

pub struct RingBuffer<'d, T, const N: usize> {
    pub buf: &'d mut [T],
    start: usize,
    end: usize,
    empty: bool,
}

#[allow(unused)]
impl<'d, T, const N: usize> RingBuffer<'d, T, N> {
    pub fn new(buf: &'d mut [T]) -> Self {
        Self {
            buf,
            start: 0,
            end: 0,
            empty: true,
        }
    }

    pub fn push_buf(&mut self) -> &mut [T] {
        if self.start == self.end && !self.empty {
            hwa::trace!("  ringbuf: push_buf empty");
            return &mut self.buf[..0];
        }

        let n = if self.start <= self.end {
            self.buf.len() - self.end
        } else {
            self.start - self.end
        };

        hwa::trace!("  ringbuf: push_buf {:?}..{:?}", self.end, self.end + n);
        &mut self.buf[self.end..self.end + n]
    }

    pub fn push(&mut self, n: usize) {
        hwa::trace!("  ringbuf: push {:?}", n);
        if n == 0 {
            return;
        }

        self.end = self.wrap(self.end + n);
        self.empty = false;
    }

    pub fn pop_buf(&mut self) -> &mut [T] {
        if self.empty {
            hwa::trace!("  ringbuf: pop_buf empty");
            return &mut self.buf[..0];
        }

        let n = if self.end <= self.start {
            self.buf.len() - self.start
        } else {
            self.end - self.start
        };

        hwa::trace!("  ringbuf: pop_buf {:?}..{:?}", self.start, self.start + n);
        &mut self.buf[self.start..self.start + n]
    }

    pub fn pop(&mut self, n: usize) {
        hwa::trace!("  ringbuf: pop {:?}", n);
        if n == 0 {
            return;
        }

        self.start = self.wrap(self.start + n);
        self.empty = self.start == self.end;
    }

    pub fn is_full(&self) -> bool {
        self.start == self.end && !self.empty
    }

    pub fn is_empty(&self) -> bool {
        self.empty
    }

    #[allow(unused)]
    pub fn len(&self) -> usize {
        if self.empty {
            0
        } else if self.start < self.end {
            self.end - self.start
        } else {
            N + self.end - self.start
        }
    }

    pub fn clear(&mut self) {
        self.start = 0;
        self.end = 0;
        self.empty = true;
    }

    fn wrap(&self, n: usize) -> usize {
        assert!(n <= self.buf.len());
        if n == self.buf.len() {
            0
        } else {
            n
        }
    }
}