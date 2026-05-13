use thiserror::Error;

#[derive(Error, Debug)]
pub enum DriverError {
    #[error("sensor not found: {0}")]
    NotFound(String),

    #[error("command failed: {0}")]
    CommandFailed(String),

    #[error("initialization failed: {0}")]
    InitFailed(String),

    #[error("serialization error: {0}")]
    Serialization(#[from] serde_json::Error),

    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
}
