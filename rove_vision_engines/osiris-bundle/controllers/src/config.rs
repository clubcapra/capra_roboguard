use std::sync::Arc;

use crate::engine::registry::EngineRegistry;
use crate::feed::manager::FeedManager;

pub struct AppState {
    pub engines: Arc<EngineRegistry>,
    pub feeds: Arc<FeedManager>,
}
