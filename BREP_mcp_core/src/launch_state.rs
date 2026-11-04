//! Native launch policy for execution journals. Resolving a policy never creates
//! a directory or opens a journal; the host applies it before admitting work.
use serde::{Deserialize, Serialize};
use std::path::{Path, PathBuf};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Host {
    Headless,
    Desktop,
}
impl Host {
    pub fn default_instance(self) -> &'static str {
        match self {
            Self::Headless => "headless",
            Self::Desktop => "desktop",
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Policy {
    pub instance: Option<String>,
    pub state_dir: Option<PathBuf>,
    pub ephemeral: bool,
}
impl Policy {
    fn supplied(&self) -> bool {
        self.instance.is_some() || self.state_dir.is_some() || self.ephemeral
    }
    /// Validate one selected mode without opening or creating its directory.
    pub fn validate(&self) -> Result<(), String> {
        if usize::from(self.instance.is_some())
            + usize::from(self.state_dir.is_some())
            + usize::from(self.ephemeral)
            > 1
        {
            return Err("choose only one MCP state mode per configuration layer: instance, state_dir, or ephemeral".into());
        }
        if let Some(name) = &self.instance {
            if name.is_empty()
                || name.len() > 64
                || !name
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
            {
                return Err(
                    "MCP instance must contain 1–64 ASCII letters, digits, hyphens or underscores"
                        .into(),
                );
            }
        }
        if self
            .state_dir
            .as_ref()
            .is_some_and(|p| p.as_os_str().is_empty())
        {
            return Err("MCP state_dir must not be empty".into());
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Settings {
    pub headless: Option<Policy>,
    pub desktop: Option<Policy>,
}
impl Settings {
    pub fn load(path: &Path) -> Result<Self, String> {
        match std::fs::read(path) {
            Ok(bytes) => serde_json::from_slice(&bytes)
                .map_err(|e| format!("MCP state settings {}: {e}", path.display())),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Self::default()),
            Err(e) => Err(format!("read MCP state settings {}: {e}", path.display())),
        }
    }
    /// Replace the settings as one file. The caller owns read/modify/write of
    /// host policies; this is not a concurrent preference-merging service.
    pub fn save(&self, path: &Path) -> Result<(), String> {
        for policy in [&self.headless, &self.desktop].into_iter().flatten() {
            policy.validate()?;
        }
        let parent = path
            .parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(Path::new("."));
        std::fs::create_dir_all(parent)
            .map_err(|e| format!("create MCP settings directory: {e}"))?;
        let temporary = parent.join(format!(".mcp-state-{}.tmp", uuid::Uuid::new_v4()));
        let result = (|| -> Result<(), String> {
            use std::io::Write;
            let mut options = std::fs::OpenOptions::new();
            options.write(true).create_new(true);
            #[cfg(unix)]
            {
                use std::os::unix::fs::OpenOptionsExt;
                options.mode(0o600);
            }
            let mut file = options.open(&temporary).map_err(|e| e.to_string())?;
            file.write_all(&serde_json::to_vec_pretty(self).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
            file.sync_all().map_err(|e| e.to_string())?;
            drop(file);
            std::fs::rename(&temporary, path).map_err(|e| e.to_string())
        })();
        if result.is_err() {
            let _ = std::fs::remove_file(&temporary);
        }
        result.map_err(|e| format!("save MCP state settings {}: {e}", path.display()))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Source {
    Cli,
    Environment,
    Saved,
    Default,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Resolved {
    /// Named profile, absent for explicit directories and ephemeral mode.
    pub instance: Option<String>,
    /// Exact override or resolved profile path; absent means ephemeral.
    pub state_dir: Option<PathBuf>,
    pub source: Source,
}

impl Resolved {
    /// Prepare only the generated profile's parent. Journal opening owns the
    /// leaf directory and its exclusive lock; explicit directories retain the
    /// existing host API's parent-directory requirements.
    pub fn prepare_named_parent(&self) -> Result<(), String> {
        if self.instance.is_some() {
            let parent = self
                .state_dir
                .as_deref()
                .and_then(Path::parent)
                .ok_or("named MCP profile has no parent directory")?;
            std::fs::create_dir_all(parent).map_err(|error| {
                format!("create MCP profile directory {}: {error}", parent.display())
            })?;
        }
        Ok(())
    }
}

/// Same native user-directory convention as the app and HTTP credential.
/// Unlike those older helpers, never silently fall back to the working directory.
pub fn config_dir() -> Option<PathBuf> {
    config_dir_from(&|name| std::env::var_os(name), cfg!(windows))
}
fn config_dir_from(
    env: &dyn Fn(&str) -> Option<std::ffi::OsString>,
    windows: bool,
) -> Option<PathBuf> {
    let var = |name| env(name).filter(|s| !s.is_empty()).map(PathBuf::from);
    var("XDG_CONFIG_HOME")
        .or_else(|| if windows { var("APPDATA") } else { None })
        .or_else(|| var("HOME").map(|h| h.join(".config")))
        .map(|p| p.join("brep-app"))
}
pub fn environment_policy() -> Result<Policy, String> {
    let text = |key: &str| -> Result<Option<String>, String> {
        match std::env::var(key) {
            Ok(s) if s.is_empty() => Ok(None),
            Ok(s) => Ok(Some(s)),
            Err(std::env::VarError::NotPresent) => Ok(None),
            Err(_) => Err(format!("{key} must be UTF-8")),
        }
    };
    let ephemeral = match text("BREP_MCP_EPHEMERAL")?.as_deref() {
        None | Some("0" | "false") => false,
        Some("1" | "true") => true,
        Some(_) => return Err("BREP_MCP_EPHEMERAL must be true, false, 1 or 0".into()),
    };
    Ok(Policy {
        instance: text("BREP_MCP_INSTANCE")?,
        state_dir: std::env::var_os("BREP_MCP_STATE_DIR")
            .filter(|s| !s.is_empty())
            .map(PathBuf::from),
        ephemeral,
    })
}
pub fn resolve(host: Host, cli: &Policy) -> Result<Resolved, String> {
    let directory = config_dir();
    resolve_loading(
        host,
        cli,
        directory.as_deref(),
        environment_policy,
        || match &directory {
            Some(dir) => Settings::load(&dir.join("mcp-state.json")),
            None => Ok(Settings::default()),
        },
    )
}

// Lazy loaders make an explicit override a recovery path even when a lower
// layer cannot be parsed. Reading directory environment variables has no IO.
fn resolve_loading(
    host: Host,
    cli: &Policy,
    directory: Option<&Path>,
    environment: impl FnOnce() -> Result<Policy, String>,
    saved: impl FnOnce() -> Result<Settings, String>,
) -> Result<Resolved, String> {
    if cli.supplied() {
        return resolve_with(
            host,
            cli,
            &Policy::default(),
            &Settings::default(),
            directory,
        );
    }
    let environment = environment()?;
    if environment.supplied() {
        return resolve_with(host, cli, &environment, &Settings::default(), directory);
    }
    resolve_with(host, cli, &environment, &saved()?, directory)
}

/// Highest supplied layer selects the entire mode; paths and names never mix
/// across layers. Only the selected layer is validated.
pub fn resolve_with(
    host: Host,
    cli: &Policy,
    environment: &Policy,
    settings: &Settings,
    directory: Option<&Path>,
) -> Result<Resolved, String> {
    let saved = match host {
        Host::Headless => &settings.headless,
        Host::Desktop => &settings.desktop,
    };
    let fallback = Policy {
        instance: Some(host.default_instance().into()),
        ..Policy::default()
    };
    let layers = [
        (cli, Source::Cli),
        (environment, Source::Environment),
        (
            saved.as_ref().unwrap_or(&fallback),
            if saved.is_some() {
                Source::Saved
            } else {
                Source::Default
            },
        ),
    ];
    let (policy, source) = layers
        .into_iter()
        .find(|(p, _)| p.supplied())
        .unwrap_or((&fallback, Source::Default));
    policy.validate()?;
    if policy.ephemeral {
        return Ok(Resolved {
            instance: None,
            state_dir: None,
            source,
        });
    }
    if let Some(path) = &policy.state_dir {
        return Ok(Resolved {
            instance: None,
            state_dir: Some(path.clone()),
            source,
        });
    }
    let name = policy.instance.as_ref().unwrap();
    let directory = directory.ok_or("cannot locate the MCP user directory; set XDG_CONFIG_HOME (or APPDATA/HOME), supply an explicit state directory, or choose ephemeral mode")?;
    Ok(Resolved {
        instance: Some(name.clone()),
        state_dir: Some(directory.join("mcp-state").join(name)),
        source,
    })
}

