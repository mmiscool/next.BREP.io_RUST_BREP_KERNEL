//! Administrator control of one server-owned native bake worker.
//! Executable, credentials and server URL are operator configuration; HTTP
//! callers cannot supply commands or obtain token contents.

use crate::Error;
use serde::Serialize;
use std::fs::OpenOptions;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::sync::Mutex;

#[derive(Debug, Clone)]
pub struct Config {
    pub executable: PathBuf,
    pub token_file: PathBuf,
    pub server_url: String,
}

#[derive(Default)]
struct Process {
    child: Option<Child>,
    last_exit: Option<i32>,
    error: Option<String>,
}

#[derive(Default)]
pub struct Controller(Mutex<Process>);

#[derive(Serialize)]
pub struct Status {
    pub configured: bool,
    pub running: bool,
    pub pid: Option<u32>,
    pub last_exit: Option<i32>,
    pub error: Option<String>,
}

impl Process {
    fn refresh(&mut self) -> Result<(), Error> {
        if let Some(child) = self.child.as_mut() {
            if let Some(status) = child.try_wait().map_err(Error::internal)? {
                self.last_exit = status.code();
                self.error = (!status.success()).then(|| format!("Worker exited with {status}. See bake-worker.log in the server data directory."));
                self.child = None;
            }
        }
        Ok(())
    }

    fn status(&self, configured: bool) -> Status {
        Status {
            configured,
            running: self.child.is_some(),
            pid: self.child.as_ref().map(Child::id),
            last_exit: self.last_exit,
            error: self.error.clone(),
        }
    }
}

impl Controller {
    pub fn status(&self, config: Option<&Config>) -> Result<Status, Error> {
        let mut process = self
            .0
            .lock()
            .map_err(|_| Error::internal("worker controller unavailable"))?;
        process.refresh()?;
        Ok(process.status(config.is_some()))
    }

    pub fn start(&self, config: Option<&Config>, root: &Path) -> Result<Status, Error> {
        let config = config.ok_or_else(|| Error::bad_request("The server operator must configure --bake-worker-executable, --bake-worker-token-file and --bake-worker-url."))?;
        let mut process = self
            .0
            .lock()
            .map_err(|_| Error::internal("worker controller unavailable"))?;
        process.refresh()?;
        if process.child.is_some() {
            return Ok(process.status(true));
        }
        if !config.executable.is_file() || !config.token_file.is_file() {
            return Err(Error::bad_request(
                "The configured worker executable or token file is missing.",
            ));
        }
        let log = OpenOptions::new()
            .create(true)
            .append(true)
            .open(root.join("bake-worker.log"))
            .map_err(Error::internal)?;
        let stderr = log.try_clone().map_err(Error::internal)?;
        let child = Command::new(&config.executable)
            .arg("--bake-worker")
            .arg("--plm-url")
            .arg(&config.server_url)
            .arg("--plm-token-file")
            .arg(&config.token_file)
            .args(["--poll", "5"])
            .stdin(Stdio::null())
            .stdout(log)
            .stderr(stderr)
            .spawn()
            .map_err(|e| Error::bad_request(format!("Could not start bake worker: {e}")))?;
        process.child = Some(child);
        process.last_exit = None;
        process.error = None;
        process.refresh()?;
        Ok(process.status(true))
    }

    pub fn stop(&self, config: Option<&Config>) -> Result<Status, Error> {
        let mut process = self
            .0
            .lock()
            .map_err(|_| Error::internal("worker controller unavailable"))?;
        process.refresh()?;
        if let Some(child) = process.child.as_mut() {
            child.kill().map_err(Error::internal)?;
            child.wait().map_err(Error::internal)?;
            process.child = None;
            process.last_exit = None;
            process.error = None;
        }
        Ok(process.status(config.is_some()))
    }
}

impl Drop for Controller {
    fn drop(&mut self) {
        if let Ok(process) = self.0.get_mut() {
            if let Some(child) = process.child.as_mut() {
                let _ = child.kill();
                let _ = child.wait();
            }
        }
    }
}

