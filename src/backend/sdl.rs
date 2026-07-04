use std::{
    collections::HashMap,
    thread::sleep,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use anyhow::{bail, Result};
use cgmath::{vec2, Vector3};
use hid_gamepad_types::{Acceleration, JoyKey, Motion, RotationSpeed};
use sdl2::{
    self,
    controller::{Axis, Button, GameController},
    event::Event,
    keyboard::Keycode,
    sensor::SensorType,
    GameControllerSubsystem, Sdl,
};

use crate::{
    calibration::{BetterCalibration, Calibration},
    config::settings::Settings,
    engine::Engine,
    mapping::Buttons,
    mouse::Mouse,
};

use super::stall_watchdog::{Action as WatchdogAction, RecoveryLadder, WatchdogCfg};
use super::Backend;

const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(30);
const ERR_DIAG_INTERVAL: Duration = Duration::from_secs(1);

/// LEARN-197: timestamped diagnostic line, persisted by run.sh's session
/// log. Epoch seconds with millisecond precision — the same time base as
/// the feel-lab ndjson `t` field, so [diag] events can be aligned with
/// recorded cursor trajectories. The prefix must never match run.sh's
/// error regex `^(Parsing error:|Error:|error:)`.
fn diag(msg: &str) {
    let ts = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0);
    println!("[diag] {:.3} {}", ts, msg);
}

/// Time-based rate limiter for error [diag] lines: a persistent per-tick
/// failure (1 kHz loop) must not flood the session log.
struct DiagRateLimit {
    last: Option<Instant>,
}

impl DiagRateLimit {
    fn new() -> Self {
        DiagRateLimit { last: None }
    }

    fn allow(&mut self, now: Instant) -> bool {
        if self
            .last
            .map_or(true, |l| now.duration_since(l) >= ERR_DIAG_INTERVAL)
        {
            self.last = Some(now);
            true
        } else {
            false
        }
    }
}

pub struct SDLBackend {
    sdl: Sdl,
    // Option so the L2 recovery path can take() + drop the OLD subsystem
    // BEFORE creating the replacement: rust-sdl2 refcounts subsystems and
    // only calls SDL_QuitSubSystem when the count reaches zero, so assigning
    // a freshly created subsystem over this field would keep the count >= 1
    // and silently turn L2 into a no-op (LEARN-197 Codex design review R1
    // §3). Do not "simplify" this back to a bare field.
    game_controller_system: Option<GameControllerSubsystem>,
}

impl SDLBackend {
    pub fn new() -> Result<Self> {
        sdl2::hint::set("SDL_JOYSTICK_HIDAPI_PS4_RUMBLE", "1");
        sdl2::hint::set("SDL_JOYSTICK_HIDAPI_PS5_RUMBLE", "1");
        sdl2::hint::set("SDL_JOYSTICK_HIDAPI_JOY_CONS", "1");
        // Expose single Joy-Cons in their vertical (upright) layout instead of
        // the sideways mini-gamepad layout, so sticks/buttons/gyro axes match
        // the natural one-handed grip.
        sdl2::hint::set("SDL_JOYSTICK_HIDAPI_VERTICAL_JOY_CONS", "1");
        sdl2::hint::set("SDL_JOYSTICK_HIDAPI_SWITCH_HOME_LED", "0");
        sdl2::hint::set("SDL_GAMECONTROLLER_USE_BUTTON_LABELS", "0");

        // Better Windows support. LEARN-197: hint keys fixed — SDL hint names
        // don't carry the "SDL_HINT_" prefix (that's the C macro identifier),
        // so the old strings were silent no-ops. run.sh additionally exports
        // SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1 as belt-and-braces.
        sdl2::hint::set("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1");
        sdl2::hint::set("SDL_JOYSTICK_THREAD", "1");

        let sdl = sdl2::init().expect("can't initialize SDL");
        let game_controller_system = sdl
            .game_controller()
            .expect("can't initialize SDL game controller subsystem");
        Ok(Self {
            sdl,
            game_controller_system: Some(game_controller_system),
        })
    }
}

impl Backend for SDLBackend {
    fn list_devices(&mut self) -> anyhow::Result<()> {
        let gcs = self
            .game_controller_system
            .as_ref()
            .expect("game controller subsystem present outside L2 reinit");
        let num_joysticks = match gcs.num_joysticks() {
            Ok(x) => x,
            Err(e) => bail!("{}", e),
        };
        if num_joysticks == 0 {
            println!("No controller detected");
        } else {
            println!("Detected controllers:");
            for i in 0..num_joysticks {
                let controller = gcs.open(i)?;
                println!(" - {}", controller.name());
            }
        }
        Ok(())
    }

    fn run(
        &mut self,
        _opts: crate::opts::Run,
        settings: Settings,
        bindings: Buttons,
    ) -> anyhow::Result<()> {
        if self
            .game_controller_system
            .as_ref()
            .expect("game controller subsystem present at startup")
            .num_joysticks()
            .expect("can't enumerate the joysticks")
            == 0
        {
            println!("Waiting for a game controller to connect...");
        }
        let mut event_pump = self
            .sdl
            .event_pump()
            .expect("can't create the SDL event pump");

        let mut controllers: HashMap<u32, ControllerState> = HashMap::new();
        // LEARN-197: engines parked across a recovery, keyed by controller
        // name — same identity the duplicate check below has always used.
        // Supported scope is a single active controller per name (Annie's
        // one-Joy-Con setup); two same-name Joy-Cons are out of scope.
        let mut parked: HashMap<String, Engine> = HashMap::new();
        let mut watches: HashMap<String, WatchEntry> = HashMap::new();
        let wd_cfg = WatchdogCfg::default();
        let mut last_heartbeat = Instant::now();
        let mut sensor_err_limit = DiagRateLimit::new();
        let mut action_err_limit = DiagRateLimit::new();

        let mut last_tick = Instant::now();

        'running: loop {
            let now = Instant::now();
            let dt = now.duration_since(last_tick);

            for event in event_pump.poll_iter() {
                match event {
                    Event::Quit { .. }
                    | Event::KeyDown {
                        keycode: Some(Keycode::Escape),
                        ..
                    } => break 'running,
                    Event::ControllerDeviceAdded { which, .. } => {
                        // LEARN-197 (C3): a failed open must not kill the
                        // whole run loop anymore — log and keep running.
                        if let Some(gcs) = self.game_controller_system.as_ref() {
                            match open_enable_controller(
                                gcs,
                                which,
                                &settings,
                                &bindings,
                                &controllers,
                                &mut parked,
                                &mut watches,
                                wd_cfg,
                                now,
                            ) {
                                Ok(Some(state)) => {
                                    controllers.insert(state.controller.instance_id(), state);
                                }
                                Ok(None) => {}
                                Err(e) => diag(&format!(
                                    "controller open failed index={}: {:#}",
                                    which, e
                                )),
                            }
                        } else {
                            diag("device added while subsystem down — picked up after L2 reinit");
                        }
                    }
                    Event::ControllerDeviceRemoved { which, .. } => {
                        if let Some(state) = controllers.remove(&which) {
                            println!("Controller disconnected: {}", state.controller.name());
                            diag(&format!(
                                "device removed by SDL (logical disconnect; BT link may still be up) name=\"{}\" instance={}",
                                state.name, which
                            ));
                            let name = state.name.clone();
                            park_engine(state, &mut parked, now);
                            if let Some(w) = watches.get_mut(&name) {
                                w.ladder.on_device_removed(now);
                            }
                        }
                    }
                    Event::ControllerButtonDown {
                        timestamp: _,
                        which,
                        button,
                    } => {
                        if let Some(controller) = controllers.get_mut(&which) {
                            if let Some(key) = sdl_to_sys(button) {
                                controller.engine.buttons().key_down(key, now);
                            }
                        }
                    }
                    Event::ControllerButtonUp {
                        timestamp: _,
                        which,
                        button,
                    } => {
                        if let Some(controller) = controllers.get_mut(&which) {
                            if let Some(key) = sdl_to_sys(button) {
                                controller.engine.buttons().key_up(key, now);
                            }
                        }
                    }
                    Event::ControllerSensorUpdated {
                        which,
                        sensor,
                        data,
                        ..
                    } => {
                        // LEARN-197 watchdog primary signal: SDL posts this
                        // event for every processed IMU sample, so its
                        // arrival time IS the last-packet time. Only feed
                        // the ladder for a currently-active instance — a
                        // stale queued event for a removed instance id must
                        // not reset an in-flight recovery.
                        if let Some(state) = controllers.get(&which) {
                            if let Some(w) = watches.get_mut(&state.name) {
                                w.ladder.on_sensor_event(now);
                                w.events_in_window += 1;
                                if sensor == SensorType::Gyroscope {
                                    // Noise-floor telemetry only (never a
                                    // trigger): how often are consecutive
                                    // gyro payloads bit-identical?
                                    if w.last_gyro == Some(data) {
                                        w.cur_streak += 1;
                                        w.window_max_streak =
                                            w.window_max_streak.max(w.cur_streak);
                                    } else {
                                        w.cur_streak = 0;
                                    }
                                    w.last_gyro = Some(data);
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }

            for controller in controllers.values_mut() {
                let c = &mut controller.controller;
                let engine = &mut controller.engine;
                let mut left = vec2(c.axis(Axis::LeftX), c.axis(Axis::LeftY))
                    .cast::<f64>()
                    .expect("can't cast i16 to f64")
                    / (i16::MAX as f64);
                let mut right = vec2(c.axis(Axis::RightX), c.axis(Axis::RightY))
                    .cast::<f64>()
                    .expect("can't cast i16 to f64")
                    / (i16::MAX as f64);

                // In SDL, -..+ y is top..bottom
                left.y = -left.y;
                right.y = -right.y;

                engine.handle_left_stick(left, now, dt);
                engine.handle_right_stick(right, now, dt);

                // SDL exposes ZL/ZR as trigger axes, not buttons (digital
                // 0/max on Joy-Cons); synthesize key edges so mappings like
                // `ZL = LMOUSE` work with the SDL backend too.
                let zl_now =
                    c.axis(Axis::TriggerLeft) as f64 / (i16::MAX as f64) >= 0.5;
                if zl_now != controller.zl_pressed {
                    controller.zl_pressed = zl_now;
                    if zl_now {
                        engine.buttons().key_down(JoyKey::ZL, now);
                    } else {
                        engine.buttons().key_up(JoyKey::ZL, now);
                    }
                }
                let zr_now =
                    c.axis(Axis::TriggerRight) as f64 / (i16::MAX as f64) >= 0.5;
                if zr_now != controller.zr_pressed {
                    controller.zr_pressed = zr_now;
                    if zr_now {
                        engine.buttons().key_down(JoyKey::ZR, now);
                    } else {
                        engine.buttons().key_up(JoyKey::ZR, now);
                    }
                }

                if c.sensor_enabled(SensorType::Accelerometer)
                    && c.sensor_enabled(SensorType::Gyroscope)
                {
                    let mut accel = [0.; 3];
                    let mut gyro = [0.; 3];
                    // LEARN-197 (C3): a transient sensor read error used to
                    // `?` out of run() entirely (and main exited 0) — the
                    // freeze looked like a silent quit. Log + skip instead.
                    let read = c
                        .sensor_get_data(SensorType::Accelerometer, &mut accel)
                        .and_then(|_| c.sensor_get_data(SensorType::Gyroscope, &mut gyro));
                    match read {
                        Ok(_) => {
                            let acceleration = Acceleration::from(
                                Vector3::from(accel)
                                    .cast::<f64>()
                                    .expect("can't cast f32 to f64")
                                    / 9.82,
                            );
                            let rotation_speed = RotationSpeed::from(
                                Vector3::from(gyro)
                                    .cast::<f64>()
                                    .expect("can't cast f32 to f64")
                                    / std::f64::consts::PI
                                    * 180.,
                            );

                            if let Some(ref mut calibrator) = controller.calibrator {
                                let finished = calibrator.push(
                                    Motion {
                                        rotation_speed,
                                        acceleration,
                                    },
                                    now,
                                    Duration::from_secs(2),
                                );
                                if finished {
                                    println!("Calibration finished for {}", c.name());
                                    let _ = c.set_rumble(220, 440, 100);
                                    engine.set_calibration(calibrator.finish());
                                    controller.calibrator = None;
                                }
                            } else {
                                // LEARN-197: right after a recovery the SDL
                                // sensor cache can hold a stale/burst value;
                                // the grace window swallows it instead of
                                // letting it fling the cursor.
                                let in_grace = watches
                                    .get(&controller.name)
                                    .map_or(false, |w| w.ladder.in_grace(now));
                                if !in_grace {
                                    engine.apply_motion(rotation_speed, acceleration, now, dt);
                                }
                            }
                        }
                        Err(e) => {
                            if sensor_err_limit.allow(now) {
                                diag(&format!(
                                    "sensor read error name=\"{}\": {} (motion skipped this tick)",
                                    controller.name, e
                                ));
                            }
                        }
                    }
                }

                // LEARN-69 F1 (UNVALIDATED): actions run AFTER motion — the
                // upstream order, REQUIRED for drag-safety. On the release tick
                // the gyro move must be posted as LeftMouseDragged BEFORE the
                // LeftMouseUp. If actions ran first (Up before the move),
                // enigo's move_mouse still reads the button as pressed
                // (NSEvent::pressedMouseButtons lags the just-posted Up) and
                // emits a LeftMouseDragged AFTER the Up — macOS then sticks the
                // left button after a drag (Annie real-device bug, LEARN-69).
                // F1 still arms on the press edge here; suppression covers the
                // click window from the next tick (~1-frame press leak,
                // negligible — far cheaper than a stuck button).
                // LEARN-197 (C3): enigo failures log instead of killing the
                // run loop.
                if let Err(e) = engine.apply_actions(now) {
                    if action_err_limit.allow(now) {
                        diag(&format!(
                            "apply_actions error name=\"{}\": {:#}",
                            controller.name, e
                        ));
                    }
                }
            }

            // LEARN-197: poll the recovery ladders. poll() transitions
            // internally, so collecting first and acting after is safe (the
            // same action is not returned twice for one deadline).
            let due: Vec<(String, WatchdogAction)> = watches
                .iter_mut()
                .filter_map(|(name, w)| w.ladder.poll(now).map(|a| (name.clone(), a)))
                .collect();
            let mut reinit_due = false;
            for (name, action) in due {
                match action {
                    WatchdogAction::ReopenController => {
                        let instance = controllers
                            .iter()
                            .find(|(_, s)| s.name == name)
                            .map(|(id, _)| *id)
                            .or_else(|| watches.get(&name).and_then(|w| w.last_instance));
                        diag(&format!(
                            "stall detected: sensor stream silent — L1 reopen name=\"{}\" instance={:?}",
                            name, instance
                        ));
                        if let Some(id) = instance {
                            if let Some(state) = controllers.remove(&id) {
                                park_engine(state, &mut parked, now);
                            }
                        }
                        let mut ok = false;
                        if let (Some(gcs), Some(id)) =
                            (self.game_controller_system.as_ref(), instance)
                        {
                            if let Some(index) = find_device_index(gcs, id) {
                                match open_enable_controller(
                                    gcs,
                                    index,
                                    &settings,
                                    &bindings,
                                    &controllers,
                                    &mut parked,
                                    &mut watches,
                                    wd_cfg,
                                    now,
                                ) {
                                    Ok(Some(state)) => {
                                        controllers
                                            .insert(state.controller.instance_id(), state);
                                        ok = true;
                                    }
                                    Ok(None) => diag(&format!(
                                        "L1 reopen: open was skipped (duplicate/filtered) name=\"{}\"",
                                        name
                                    )),
                                    Err(e) => diag(&format!(
                                        "L1 reopen failed name=\"{}\": {:#}",
                                        name, e
                                    )),
                                }
                            } else {
                                diag(&format!(
                                    "L1 reopen: instance {} no longer enumerated (SDL dropped the device)",
                                    id
                                ));
                            }
                        }
                        if let Some(w) = watches.get_mut(&name) {
                            w.ladder.on_reopen_result(ok, now);
                        }
                        diag(&format!("L1 reopen result name=\"{}\" ok={}", name, ok));
                    }
                    WatchdogAction::ReinitSubsystem => {
                        reinit_due = true;
                        if let Some(w) = watches.get_mut(&name) {
                            w.ladder.on_reinit_attempted(now);
                        }
                    }
                }
            }
            if reinit_due {
                diag("L2: reinit game controller subsystem (forces HID close/reopen)");
                for (_, state) in controllers.drain() {
                    park_engine(state, &mut parked, now);
                }
                // Codex design review R1 §3: drop the old subsystem BEFORE
                // creating the new one, otherwise rust-sdl2's refcount never
                // reaches zero, SDL_QuitSubSystem is never called, and the
                // HID device is never actually closed/reopened.
                drop(self.game_controller_system.take());
                match self.sdl.game_controller() {
                    Ok(gcs) => {
                        self.game_controller_system = Some(gcs);
                        diag("L2: subsystem reinitialized, waiting for device re-add");
                    }
                    Err(e) => {
                        // Stay down; the ladder's backoff retries the whole
                        // L2 on its next deadline (take() of None is fine).
                        diag(&format!(
                            "L2: subsystem reinit failed ({}); retrying on backoff",
                            e
                        ));
                    }
                }
            }

            if now.duration_since(last_heartbeat) >= HEARTBEAT_INTERVAL {
                for (name, w) in watches.iter_mut() {
                    let active = controllers.values().any(|s| &s.name == name);
                    diag(&format!(
                        "heartbeat name=\"{}\" active={} sensor_events={} max_identical_gyro_streak={}",
                        name, active, w.events_in_window, w.window_max_streak
                    ));
                    w.events_in_window = 0;
                    w.window_max_streak = 0;
                    w.cur_streak = 0;
                }
                last_heartbeat = now;
            }

            last_tick = now;
            sleep(Duration::from_millis(1));
        }

        Ok(())
    }
}

struct ControllerState {
    controller: GameController,
    engine: Engine,
    calibrator: Option<BetterCalibration>,
    // LEARN-197: identity key for parking/adoption and the watch ladder.
    name: String,
    // SDL reports ZL/ZR as trigger axes, not buttons; track the pressed
    // state so the poll loop can synthesize key up/down edges.
    zl_pressed: bool,
    zr_pressed: bool,
}

/// LEARN-197: per-controller-identity watchdog entry. The ladder outlives
/// the GameController handle (a recovery in flight has no live handle).
struct WatchEntry {
    ladder: RecoveryLadder,
    /// Last known joystick instance id, for L1 device-index lookup after
    /// the live handle has been dropped.
    last_instance: Option<u32>,
    // Heartbeat / noise-floor telemetry (never used as a trigger).
    events_in_window: u64,
    last_gyro: Option<[f32; 3]>,
    cur_streak: u32,
    window_max_streak: u32,
}

impl WatchEntry {
    fn new(cfg: WatchdogCfg) -> Self {
        WatchEntry {
            ladder: RecoveryLadder::new(cfg),
            last_instance: None,
            events_in_window: 0,
            last_gyro: None,
            cur_streak: 0,
            window_max_streak: 0,
        }
    }
}

/// LEARN-197: flush transient input state, then park the engine for later
/// adoption. A held mouse button or hold-to-move layer must not survive
/// the controller swap (stuck-button class of bug); the calibration stays
/// with the engine — that's the whole point of parking.
fn park_engine(state: ControllerState, parked: &mut HashMap<String, Engine>, now: Instant) {
    let ControllerState {
        controller,
        mut engine,
        name,
        ..
    } = state;
    engine.buttons().release_all(now);
    if let Err(e) = engine.apply_actions(now) {
        diag(&format!(
            "release-all flush failed name=\"{}\": {:#}",
            name, e
        ));
    }
    parked.insert(name, engine);
    // GameController dropped here => SDL CloseJoystick (sends the device
    // back to simple input mode; the next open re-runs the full handshake).
    drop(controller);
}

/// LEARN-197: the single open path for ALL of DeviceAdded / L1 reopen /
/// post-L2 re-add. SDL's OpenJoystick does NOT enable IMU reporting —
/// `sensor_set_enabled` (SDL_GameControllerSetSensorEnabled) does. Skipping
/// it on any path would leave a recovered controller silent forever (Codex
/// design review R1 §1). Returns Ok(None) for filtered devices (duplicate
/// name / Steam virtual pad), matching the historical behavior.
#[allow(clippy::too_many_arguments)]
fn open_enable_controller(
    gcs: &GameControllerSubsystem,
    device_index: u32,
    settings: &Settings,
    bindings: &Buttons,
    controllers: &HashMap<u32, ControllerState>,
    parked: &mut HashMap<String, Engine>,
    watches: &mut HashMap<String, WatchEntry>,
    wd_cfg: WatchdogCfg,
    now: Instant,
) -> Result<Option<ControllerState>> {
    let mut controller = gcs.open(device_index)?;
    let name = controller.name();

    if controllers.values().any(|c| c.name == name) {
        return Ok(None);
    }
    if name == "Steam Virtual Gamepad" {
        return Ok(None);
    }

    let sensors_ok = controller
        .sensor_set_enabled(SensorType::Accelerometer, true)
        .and(controller.sensor_set_enabled(SensorType::Gyroscope, true))
        .is_ok();

    let watch = watches
        .entry(name.clone())
        .or_insert_with(|| WatchEntry::new(wd_cfg));
    watch.last_instance = Some(controller.instance_id());

    let (engine, calibrator) = if let Some(engine) = parked.remove(&name) {
        // Recovery adoption: reuse the previous calibration. Recalibrating
        // for 2s while the user is mid-motion is exactly what produced the
        // 400-666px post-recovery flings (m4 evidence); the grace window
        // additionally swallows the first burst of resumed data.
        diag(&format!(
            "adopted parked engine name=\"{}\" instance={} sensors_ok={} (calibration preserved, grace started)",
            name,
            controller.instance_id(),
            sensors_ok
        ));
        watch.ladder.start_grace(now);
        (engine, None)
    } else {
        println!("New controller: {}", name);
        let calibrator = if sensors_ok {
            println!(
                "Starting calibration for {}, don't move the controller...",
                name
            );
            Some(BetterCalibration::default())
        } else {
            let _ = controller.set_rumble(220, 440, 100);
            None
        };
        let engine = Engine::new(
            settings.clone(),
            bindings.clone(),
            Calibration::empty(),
            Mouse::new()?,
        )?;
        (engine, calibrator)
    };

    Ok(Some(ControllerState {
        controller,
        engine,
        calibrator,
        name,
        zl_pressed: false,
        zr_pressed: false,
    }))
}

/// LEARN-197: map a joystick instance id back to its current device index
/// (`GameControllerSubsystem::open` takes an index). The sdl2 crate doesn't
/// wrap SDL_JoystickGetDeviceInstanceID, so use the re-exported sys FFI.
/// Returns None when the device is no longer enumerated (e.g. SDL already
/// logically disconnected it) — the caller counts that as a failed attempt.
fn find_device_index(gcs: &GameControllerSubsystem, instance_id: u32) -> Option<u32> {
    let n = gcs.num_joysticks().ok()?;
    (0..n).find(|&i| unsafe {
        sdl2::sys::SDL_JoystickGetDeviceInstanceID(i as i32) == instance_id as i32
    })
}

fn sdl_to_sys(button: Button) -> Option<JoyKey> {
    Some(match button {
        Button::A => JoyKey::S,
        Button::B => JoyKey::E,
        Button::X => JoyKey::W,
        Button::Y => JoyKey::N,
        Button::Back => JoyKey::Minus,
        Button::Guide => JoyKey::Home,
        Button::Start => JoyKey::Plus,
        Button::LeftStick => JoyKey::L3,
        Button::RightStick => JoyKey::R3,
        Button::LeftShoulder => JoyKey::L,
        Button::RightShoulder => JoyKey::R,
        Button::DPadUp => JoyKey::Up,
        Button::DPadDown => JoyKey::Down,
        Button::DPadLeft => JoyKey::Left,
        Button::DPadRight => JoyKey::Right,
        // Switch capture button arrives as Misc1.
        Button::Misc1 => JoyKey::Capture,
        // Joy-Con SL/SR rail buttons arrive as paddles; the exact paddle
        // number depends on which Joy-Con (L/R), so map upper/lower pairs to
        // SL/SR rather than panicking on todo!().
        Button::Paddle1 => JoyKey::SL,
        Button::Paddle2 => JoyKey::SL,
        Button::Paddle3 => JoyKey::SR,
        Button::Paddle4 => JoyKey::SR,
        // PS4/PS5 touchpad click has no JoyKey equivalent; ignore it.
        Button::Touchpad => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sdl_to_sys_covers_every_button_without_panicking() {
        let all = [
            Button::A,
            Button::B,
            Button::X,
            Button::Y,
            Button::Back,
            Button::Guide,
            Button::Start,
            Button::LeftStick,
            Button::RightStick,
            Button::LeftShoulder,
            Button::RightShoulder,
            Button::DPadUp,
            Button::DPadDown,
            Button::DPadLeft,
            Button::DPadRight,
            Button::Misc1,
            Button::Paddle1,
            Button::Paddle2,
            Button::Paddle3,
            Button::Paddle4,
            Button::Touchpad,
        ];
        for b in all {
            let _ = sdl_to_sys(b);
        }
    }

    #[test]
    fn sdl_to_sys_maps_switch_extras() {
        assert!(matches!(sdl_to_sys(Button::Misc1), Some(JoyKey::Capture)));
        assert!(matches!(sdl_to_sys(Button::Paddle1), Some(JoyKey::SL)));
        assert!(matches!(sdl_to_sys(Button::Paddle2), Some(JoyKey::SL)));
        assert!(matches!(sdl_to_sys(Button::Paddle3), Some(JoyKey::SR)));
        assert!(matches!(sdl_to_sys(Button::Paddle4), Some(JoyKey::SR)));
        assert!(sdl_to_sys(Button::Touchpad).is_none());
    }
}
