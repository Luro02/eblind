use embassy_time::{Duration, Timer};

/// Yields to the scheduler, allowing other tasks to run, and does a yield to the OS to allow other threads to run.
pub async fn yield_now() {
    embassy_futures::yield_now().await;
}

/// Yields for at least the given duration, allowing other tasks and threads to run.
pub async fn yield_for(duration: Duration) {
    Timer::after(duration).await
}
