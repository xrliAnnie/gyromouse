use crate::{config::settings::Settings, mapping::Buttons, opts::Run};

#[cfg(feature = "sdl2")]
pub mod sdl;

// LEARN-197: pure stall-recovery state machine (no SDL dependency; used by
// the SDL backend, but kept unconditional so its tests always run).
pub mod stall_watchdog;

#[cfg(feature = "hidapi")]
pub mod hidapi;

pub trait Backend {
    fn list_devices(&mut self) -> anyhow::Result<()>;
    fn run(&mut self, opts: Run, settings: Settings, bindings: Buttons) -> anyhow::Result<()>;
}
