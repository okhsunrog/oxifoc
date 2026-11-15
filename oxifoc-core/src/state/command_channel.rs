//! Async producers with a lock-free empty check for the polling control ISR.

use core::future::{Future, poll_fn};
use core::pin::pin;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::{Channel, TryReceiveError, TrySendError};

/// A bounded FIFO whose receiver polls instead of registering a waker.
///
/// Every queue write and its pending flag are published under the same critical
/// section. A receiver cannot clear a concurrently published command, including
/// when a full async send resumes after space becomes available. Cancelling a
/// pending send neither publishes a command nor changes the flag.
pub struct CommandChannel<T, const N: usize> {
    queue: Channel<CriticalSectionRawMutex, T, N>,
    pending: AtomicBool,
}

impl<T, const N: usize> CommandChannel<T, N> {
    pub const fn new() -> Self {
        Self {
            queue: Channel::new(),
            pending: AtomicBool::new(false),
        }
    }

    pub fn try_send(&self, message: T) -> Result<(), TrySendError<T>> {
        critical_section::with(|_| {
            self.queue.try_send(message)?;
            self.pending.store(true, Ordering::Release);
            Ok(())
        })
    }

    pub async fn send(&self, message: T) {
        let mut send = pin!(self.queue.send(message));
        poll_fn(|cx| {
            critical_section::with(|_| {
                let result = send.as_mut().poll(cx);
                if result.is_ready() {
                    self.pending.store(true, Ordering::Release);
                }
                result
            })
        })
        .await;
    }

    #[inline]
    pub fn try_receive(&self) -> Result<T, TryReceiveError> {
        if !self.pending.load(Ordering::Acquire) {
            return Err(TryReceiveError::Empty);
        }
        critical_section::with(|_| {
            let result = self.queue.try_receive();
            if result.is_err() {
                self.pending.store(false, Ordering::Release);
            }
            result
        })
    }
}

impl<T, const N: usize> Default for CommandChannel<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::task::{Context, Poll, Waker};

    #[test]
    fn full_send_resumes_after_receive_and_cancellation_keeps_fifo() {
        let channel = CommandChannel::<u32, 2>::new();
        let mut cx = Context::from_waker(Waker::noop());
        assert!(channel.try_receive().is_err());
        channel.try_send(1).unwrap();
        channel.try_send(2).unwrap();
        assert!(matches!(channel.try_send(3), Err(TrySendError::Full(3))));
        {
            let mut cancelled = pin!(channel.send(99));
            assert!(cancelled.as_mut().poll(&mut cx).is_pending());
        }
        let mut send = pin!(channel.send(3));
        assert!(send.as_mut().poll(&mut cx).is_pending());
        assert_eq!(channel.try_receive(), Ok(1));
        assert_eq!(send.as_mut().poll(&mut cx), Poll::Ready(()));
        assert_eq!(channel.try_receive(), Ok(2));
        assert_eq!(channel.try_receive(), Ok(3));
        assert!(channel.try_receive().is_err());
        assert!(!channel.pending.load(Ordering::Relaxed));
        // Repeated empty polling must not hide the next producer publication.
        assert!(channel.try_receive().is_err());
        let mut send = pin!(channel.send(4));
        assert!(send.as_mut().poll(&mut cx).is_ready());
        assert_eq!(channel.try_receive(), Ok(4));
    }

    #[test]
    fn competing_producers_preserve_every_command_and_each_producers_order() {
        let channel = CommandChannel::<(usize, u32), 8>::new();
        std::thread::scope(|scope| {
            for producer in 0..2 {
                let queue = &channel;
                scope.spawn(move || {
                    for sequence in 0..10_000 {
                        while queue.try_send((producer, sequence)).is_err() {
                            std::thread::yield_now();
                        }
                    }
                });
            }
            let mut received = [0; 2];
            while received.iter().sum::<u32>() != 20_000 {
                if let Ok((producer, sequence)) = channel.try_receive() {
                    assert_eq!(sequence, received[producer]);
                    received[producer] += 1;
                } else {
                    std::thread::yield_now();
                }
            }
            assert_eq!(received, [10_000; 2]);
        });
        assert!(channel.try_receive().is_err());
    }
}
