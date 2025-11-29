use std::sync::Arc;

use esp_idf_hal::interrupt::asynch::HalIsrNotification;
use esp_idf_hal::task::queue::Queue;
use esp_idf_sys::EspError;

use crate::utils::yield_now;

pub struct AsyncQueue<T> {
    queue: Queue<T>,
    notif: Arc<HalIsrNotification>,
}

impl<T: Copy> AsyncQueue<T> {
    #[must_use]
    pub fn new(queue: Queue<T>) -> Self {
        Self {
            queue,
            notif: Arc::new(HalIsrNotification::new()),
        }
    }

    pub fn try_send_back(&self, item: T) -> Result<bool, EspError> {
        // The send does not block when sending:
        let has_woken = self.queue.send_back(item, 0)?;

        // Notify any waiting tasks that something is available:
        self.notif.notify_lsb();

        Ok(has_woken)
    }

    pub async fn recv_front(&self) -> T {
        loop {
            if let Some((item, has_woken)) = self.queue.recv_front(0) {
                if has_woken {
                    yield_now().await;
                }

                return item;
            }

            // Wait for a notification from an ISR that something is available:
            self.notif.wait().await;
        }
    }

    #[must_use]
    pub unsafe fn borrow(&self) -> Self {
        Self {
            queue: Queue::new_borrowed(self.queue.as_raw()),
            notif: self.notif.clone(),
        }
    }
}
