use std::sync::Mutex;
use tokio_util::sync::CancellationToken;

/// Retains cancellation and completion; dropping a bare token does not cancel.
pub struct WorkerTask {
    cancel: CancellationToken,
    task: Mutex<Option<tokio::task::JoinHandle<()>>>,
}
impl WorkerTask {
    pub fn spawn(future: impl std::future::Future<Output = ()> + Send + 'static) -> Self {
        let cancel = CancellationToken::new();
        let stop = cancel.clone();
        let task = tokio::spawn(async move {
            tokio::select! { _ = future => {}, _ = stop.cancelled() => {} }
        });
        Self {
            cancel,
            task: Mutex::new(Some(task)),
        }
    }
    pub async fn stop(&self) {
        self.cancel.cancel();
        let handle = self.task.lock().unwrap().take();
        if let Some(handle) = handle {
            let _ = handle.await;
        }
    }
}
impl Drop for WorkerTask {
    fn drop(&mut self) {
        self.cancel.cancel();
    }
}
