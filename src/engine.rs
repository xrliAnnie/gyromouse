use std::{
    collections::HashSet,
    ops::DerefMut,
    time::{Duration, Instant},
};

use cgmath::{InnerSpace, Vector2, Zero};
use enigo::{Direction, Mouse as _};
use hid_gamepad_types::{Acceleration, Motion, RotationSpeed};

use crate::{
    calibration::Calibration,
    config::{
        settings::{ClickStabSettings, GyroSettings, Settings},
        types::GyroSpace,
    },
    gyromouse::GyroMouse,
    joystick::{Stick, StickSide},
    mapping::{Buttons, ExtAction},
    motion_stick::MotionStick,
    mouse::{Mouse, MouseMovement},
    space_mapper::{
        self, LocalSpace, PlayerSpace, SensorFusion, SimpleFusion, SpaceMapper, WorldSpace,
    },
    ClickType,
};

pub struct Engine {
    settings: Settings,
    left_stick: Box<dyn Stick>,
    right_stick: Box<dyn Stick>,
    motion_stick: MotionStick,
    buttons: Buttons,
    mouse: Mouse,
    gyro: Gyro,
    /// LEARN-69 Feature 1 — Heisenberg click stabilization (UNVALIDATED).
    click_stab: ClickStabilizer,
    #[cfg(feature = "vgamepad")]
    gamepad: Option<Box<dyn virtual_gamepad::Backend>>,
}

/// LEARN-69 Feature 1 — Heisenberg click-stabilization state machine
/// (UNVALIDATED: written but not compiled / real-device tested in the headless
/// Runner; the Python `mapping.py::ClickStabilizer` is the validated oracle).
///
/// Holds only transient state; all parameters come from `ClickStabSettings`
/// each call. Tracks the set of held click buttons so it arms on the FIRST
/// button down, never re-arms on a second, and only fully resets once ALL are
/// up (drag/double-click safe). Gating happens in float pixel space BEFORE the
/// mouse error_accumulator/rounding (see `Engine::handle_motion_frame`).
#[derive(Debug)]
struct ClickStabilizer {
    pressed: HashSet<u8>,
    suppressing: bool,
    armed_at: Option<Instant>,
    accum: Vector2<f64>,
}

impl Default for ClickStabilizer {
    fn default() -> Self {
        // cgmath's Vector2 has no Default impl, so spell it out.
        Self {
            pressed: HashSet::new(),
            suppressing: false,
            armed_at: None,
            accum: Vector2::zero(),
        }
    }
}

fn click_button_key(button: enigo::Button) -> u8 {
    match button {
        enigo::Button::Left => 0,
        enigo::Button::Right => 1,
        enigo::Button::Middle => 2,
        _ => 3,
    }
}

impl ClickStabilizer {
    fn on_click_down(&mut self, settings: &ClickStabSettings, button: enigo::Button, now: Instant) {
        let was_empty = self.pressed.is_empty();
        self.pressed.insert(click_button_key(button));
        if !settings.enabled {
            return;
        }
        if was_empty {
            // arm on the FIRST button only; a second button must not re-arm.
            self.suppressing = true;
            self.armed_at = Some(now);
            self.accum = Vector2::zero();
        }
    }

    fn on_click_up(&mut self, button: enigo::Button) {
        self.pressed.remove(&click_button_key(button));
        if self.pressed.is_empty() {
            self.suppressing = false;
            self.accum = Vector2::zero();
        }
    }

    /// Instantaneous click (ClickType::Click, e.g. a `!LMOUSE` tap mapping) has
    /// no separate Release edge. Arm a time-bounded one-shot suppression that
    /// the window check in `filter()` releases; do NOT touch the pressed set
    /// (no Release will arrive). No-op if already suppressing or a button is
    /// held. Mirrors mapping.py::ClickStabilizer.on_click_tap.
    fn on_click_tap(&mut self, settings: &ClickStabSettings, now: Instant) {
        if !settings.enabled || self.suppressing || !self.pressed.is_empty() {
            return;
        }
        self.suppressing = true;
        self.armed_at = Some(now);
        self.accum = Vector2::zero();
    }

    /// Gate a would-be pixel delta (float, pre-quantization). Returns the delta
    /// to actually emit. Drag-safe via three release rules: cross drag distance
    /// (flush accumulated), window elapsed (drop jitter), or all buttons up.
    fn filter(
        &mut self,
        settings: &ClickStabSettings,
        delta: Vector2<f64>,
        now: Instant,
    ) -> Vector2<f64> {
        if !settings.enabled || !self.suppressing {
            return delta;
        }
        let cand = self.accum + delta;
        if cand.magnitude() >= settings.drag_release_dist {
            // drag detected: release + flush accumulated so the drag keeps its
            // start (prioritized over the window check below).
            self.suppressing = false;
            self.accum = Vector2::zero();
            return cand;
        }
        if let Some(armed_at) = self.armed_at {
            if now.saturating_duration_since(armed_at) >= settings.window {
                // window elapsed: click done — drop accumulated jitter, pass
                // through subsequent frames.
                self.suppressing = false;
                self.accum = Vector2::zero();
                return Vector2::zero();
            }
        }
        self.accum = cand;
        Vector2::zero()
    }
}

impl Engine {
    pub fn new(
        settings: Settings,
        buttons: Buttons,
        calibration: Calibration,
        mouse: Mouse,
    ) -> anyhow::Result<Self> {
        Ok(Engine {
            left_stick: settings.new_left_stick(),
            right_stick: settings.new_right_stick(),
            motion_stick: MotionStick::new(&settings),
            buttons,
            mouse,
            gyro: Gyro::new(&settings, calibration),
            click_stab: ClickStabilizer::default(),
            settings,
            #[cfg(feature = "vgamepad")]
            // TODO: Conditional virtual gamepad creation
            // Only create if option is enabled
            //gamepad: virtual_gamepad::new(virtual_gamepad::GamepadType::DS4)
            //    .map(|vg| -> Box<dyn virtual_gamepad::Backend> { Box::new(vg) })
            //    .map_err(|e| {
            //        eprintln!("Error initializing the virtual gamepad: {}", e);
            //        e
            //    })
            //    .ok(),
            gamepad: None,
        })
    }

    pub fn buttons(&mut self) -> &mut Buttons {
        &mut self.buttons
    }

    pub fn handle_left_stick(&mut self, stick: Vector2<f64>, now: Instant, dt: Duration) {
        self.left_stick.handle(
            stick,
            StickSide::Left,
            &self.settings,
            &mut self.buttons,
            &mut self.mouse,
            now,
            dt,
        );
    }

    pub fn handle_right_stick(&mut self, stick: Vector2<f64>, now: Instant, dt: Duration) {
        self.right_stick.handle(
            stick,
            StickSide::Right,
            &self.settings,
            &mut self.buttons,
            &mut self.mouse,
            now,
            dt,
        );
    }

    pub fn apply_actions(&mut self, now: Instant) -> anyhow::Result<()> {
        #[cfg(feature = "vgamepad")]
        let mut gamepad_pressed = false;
        for action in self.buttons.tick(now) {
            let verbose = false;
            if verbose {
                println!("Action: {}", action);
            }
            match action {
                ExtAction::GyroOn(ClickType::Press) | ExtAction::GyroOff(ClickType::Release) => {
                    self.gyro.enabled = true;
                }
                ExtAction::GyroOn(ClickType::Release) | ExtAction::GyroOff(ClickType::Press) => {
                    self.gyro.enabled = false;
                }
                ExtAction::GyroOn(ClickType::Toggle) | ExtAction::GyroOff(ClickType::Toggle) => {
                    self.gyro.enabled = !self.gyro.enabled;
                }
                ExtAction::GyroOn(ClickType::Click) | ExtAction::GyroOff(ClickType::Click) => {
                    eprintln!("Warning: event type Click has no effect on gyro on/off");
                }
                ExtAction::KeyPress(c, ClickType::Click) => {
                    self.mouse.key(c, Direction::Click)?
                }
                ExtAction::KeyPress(c, ClickType::Press) => {
                    self.mouse.key(c, Direction::Press)?
                }
                ExtAction::KeyPress(c, ClickType::Release) => {
                    self.mouse.key(c, Direction::Release)?
                }
                ExtAction::KeyPress(_, ClickType::Toggle) => {
                    // TODO: Implement key press toggle
                    eprintln!("Warning: key press toggle is not implemented");
                }
                ExtAction::MousePress(c, ClickType::Click) => {
                    // LEARN-69 F1 (UNVALIDATED): an instantaneous click has no
                    // Release edge — arm a time-bounded one-shot suppression so
                    // the same-tick click jerk is actually suppressed until the
                    // window expires (a bare down+up would clear it immediately).
                    self.click_stab.on_click_tap(&self.settings.click_stab, now);
                    self.mouse.enigo().button(c, Direction::Click)?;
                }
                ExtAction::MousePress(c, ClickType::Press) => {
                    // LEARN-69 F1 (UNVALIDATED): arm suppression on the click edge.
                    self.click_stab.on_click_down(&self.settings.click_stab, c, now);
                    self.mouse.enigo().button(c, Direction::Press)?
                }
                ExtAction::MousePress(c, ClickType::Release) => {
                    self.click_stab.on_click_up(c);
                    self.mouse.enigo().button(c, Direction::Release)?
                }
                ExtAction::MousePress(_, ClickType::Toggle) => {
                    // TODO: Implement mouse click toggle
                    eprintln!("Warning: mouse click toggle is not implemented");
                }
                #[cfg(feature = "vgamepad")]
                ExtAction::GamepadKeyPress(key, ClickType::Press) => {
                    if let Some(gamepad) = &mut self.gamepad {
                        gamepad.key(key, true)?;
                        gamepad_pressed = true;
                    }
                }
                #[cfg(feature = "vgamepad")]
                ExtAction::GamepadKeyPress(key, ClickType::Release) => {
                    if let Some(gamepad) = &mut self.gamepad {
                        gamepad.key(key, false)?;
                        gamepad_pressed = true;
                    }
                }
                #[cfg(feature = "vgamepad")]
                ExtAction::GamepadKeyPress(_, _) => todo!(),
                ExtAction::None => {}
            }
        }
        #[cfg(feature = "vgamepad")]
        if let Some(gamepad) = &mut self.gamepad {
            if gamepad_pressed {
                gamepad.push()?;
            }
        }
        Ok(())
    }

    pub fn apply_motion(
        &mut self,
        rotation_speed: RotationSpeed,
        acceleration: Acceleration,
        now: Instant,
        dt: Duration,
    ) {
        self.handle_motion_frame(
            &[Motion {
                rotation_speed,
                acceleration,
            }],
            now,
            dt,
        )
    }

    pub fn handle_motion_frame(&mut self, motions: &[Motion], now: Instant, dt: Duration) {
        // LEARN-69 F1 (UNVALIDATED): gyro produces the frame movement; we then
        // gate it through click-stabilization in FLOAT pixel space, BEFORE the
        // mouse error_accumulator/rounding. When the gate suppresses (zero) we
        // do NOT call the pixel emitter, leaving the sub-pixel remainder
        // untouched (no stale flush, no drift). Drag flush emits the
        // accumulated movement exactly once.
        let movement = self.gyro.handle_frame(&self.settings, motions, dt);
        let px = self.mouse.movement_to_pixels(&self.settings.mouse, movement);
        let emit = self
            .click_stab
            .filter(&self.settings.click_stab, px, now);
        if emit != Vector2::zero() {
            self.mouse.mouse_move_relative_pixel(emit);
        }
        self.handle_motion_stick(now, dt);
    }

    fn handle_motion_stick(&mut self, now: Instant, dt: Duration) {
        self.motion_stick.handle(
            self.gyro.sensor_fusion.up_vector(),
            &self.settings,
            &mut self.buttons,
            &mut self.mouse,
            now,
            dt,
        )
    }

    pub fn set_calibration(&mut self, calibration: Calibration) {
        self.gyro.calibration = calibration;
    }
}

/// LEARN-81 — motion-wake + dwell-auto-stop implicit clutch (LG Magic-Remote
/// style, UNVALIDATED: written but not real-device tested in the headless
/// Runner). Pure hysteresis state machine: wakes when raw angular speed exceeds
/// `motion_wake_speed`, sleeps after dwelling below `motion_sleep_speed` for
/// `motion_sleep_dwell`. Two-fold hysteresis (wake > sleep speed band + the
/// dwell timer) prevents oscillation/jitter.
///
/// It NEVER writes `Gyro::enabled`; it is AND-ed into the output gate as a
/// separate signal, so `- = GYRO_OFF` (which sets `enabled = false`) always
/// wins — motion can never re-wake a gyro the e-stop turned off (LEARN-62).
/// When disabled the gate is fully transparent (always awake), so the default
/// (OFF) behavior is bit-identical to the pre-feature engine.
#[derive(Debug, Default)]
struct MotionWake {
    awake: bool,
    /// Accumulated time spent below `motion_sleep_speed` while awake.
    still_for: Duration,
}

impl MotionWake {
    /// Update from the current raw angular speed (deg/s, pre-smoothing,
    /// post-calibration) and the frame `dt`. Returns whether the implicit
    /// clutch is awake. Starts asleep (`Default`) so the cursor never jumps on
    /// launch.
    fn update(&mut self, settings: &GyroSettings, speed: f64, dt: Duration) -> bool {
        if !settings.motion_wake_enabled {
            // Disabled = transparent gate: always awake, dwell cleared. This is
            // the root of "default-OFF bit-identical": the gate reduces to
            // `self.enabled`. (Does not preserve asleep state across a runtime
            // toggle, but config is not hot-reloaded so that is moot.)
            self.awake = true;
            self.still_for = Duration::ZERO;
            return true;
        }
        // Finite-guard the live, sensor-derived speed (LEARN-81 calls out
        // finite/panic guards): a non-finite reading is treated as parked (0),
        // never as wake motion.
        let speed = if speed.is_finite() { speed.max(0.) } else { 0. };
        let wake = settings.motion_wake_speed.max(0.);
        // sleep <= wake (speed hysteresis) enforced here, mirroring how
        // PrecisionMode clamps exit >= enter at use-time.
        let sleep = settings.motion_sleep_speed.max(0.).min(wake);
        if self.awake {
            if speed < sleep {
                self.still_for = self.still_for.saturating_add(dt);
                if self.still_for >= settings.motion_sleep_dwell {
                    self.awake = false;
                }
            } else {
                // Any motion at/above the sleep threshold (incl. slow aiming in
                // the sleep..wake band) resets the dwell — a brief pause shorter
                // than the dwell never sleeps mid-aim.
                self.still_for = Duration::ZERO;
            }
        } else if speed > wake {
            self.awake = true;
            self.still_for = Duration::ZERO;
        }
        self.awake
    }
}

pub struct Gyro {
    enabled: bool,
    calibration: Calibration,
    sensor_fusion: Box<dyn SensorFusion>,
    space_mapper: Box<dyn SpaceMapper>,
    gyromouse: GyroMouse,
    /// LEARN-81 — motion-wake implicit clutch gate (default OFF / transparent).
    motion_wake: MotionWake,
}

impl Gyro {
    pub fn new(settings: &Settings, calibration: Calibration) -> Gyro {
        Gyro {
            enabled: true,
            calibration,
            sensor_fusion: Box::new(SimpleFusion::new()),
            space_mapper: match settings.gyro.space {
                GyroSpace::Local => Box::new(LocalSpace::default()),
                GyroSpace::WorldTurn => Box::new(WorldSpace::default()),
                GyroSpace::WorldLean => todo!("World Lean is unimplemented for now"),
                GyroSpace::PlayerTurn => Box::new(PlayerSpace::default()),
                GyroSpace::PlayerLean => todo!("Player Lean is unimplemented for now"),
            },
            gyromouse: GyroMouse::default(),
            motion_wake: MotionWake::default(),
        }
    }

    /// Accumulate this frame's gyro movement and return it (origin bottom-left,
    /// degrees). LEARN-69 (UNVALIDATED): emission moved up to
    /// `Engine::handle_motion_frame` so click-stabilization can gate the
    /// movement before it reaches the OS. Returns zero when gyro is disabled
    /// (e.g. GYRO_OFF held) so that the e-stop pause channel still stops the
    /// cursor regardless of click state. LEARN-81 (UNVALIDATED): each sub-frame
    /// is also gated by the motion-wake implicit clutch (only awake sub-frames
    /// emit), and the whole result is gated by `enabled` so motion can never
    /// re-wake a gyro the e-stop disabled.
    pub fn handle_frame(
        &mut self,
        settings: &Settings,
        motions: &[Motion],
        dt: Duration,
    ) -> MouseMovement {
        // Guard before `dt / motions.len()` so an empty slice can't divide by
        // zero; nothing to emit anyway. (LEARN-81)
        if motions.is_empty() {
            return MouseMovement::zero();
        }
        let mut delta_position = MouseMovement::zero();
        let dt = dt / motions.len() as u32;
        for frame in motions.iter().cloned() {
            let frame = self.calibration.calibrate(frame);
            let delta = space_mapper::map_input(
                &frame,
                dt,
                self.sensor_fusion.deref_mut(),
                self.space_mapper.deref_mut(),
            );
            // Raw angular speed (deg/s, pre-smoothing, post-calibration) — same
            // quantity precision mode senses; a still, calibrated hand reads ~0.
            let speed = (delta.x * delta.x + delta.y * delta.y).sqrt();
            // Gate EACH sub-frame by its OWN motion-wake state: a multi-sample
            // batch (hidapi passes `report.motion` as several samples) must emit
            // only its awake sub-frames' movement, not apply the last frame's
            // state to the whole accumulation. `process` always runs (keeps the
            // smooth/precision state warm); only emission is gated. When the
            // feature is disabled `update` always returns true, so every frame
            // is added -> default-OFF bit-identical.
            let awake = self.motion_wake.update(&settings.gyro, speed, dt);
            let offset = self.gyromouse.process(&settings.gyro, delta, dt);
            if awake {
                delta_position += offset;
            }
        }
        // The GYRO_OFF e-stop (enabled = false) still wins over everything:
        // motion can never re-wake a gyro the e-stop disabled (MotionWake never
        // writes `enabled`).
        if self.enabled {
            delta_position
        } else {
            MouseMovement::zero()
        }
    }
}

// LEARN-69 Feature 1 — ClickStabilizer unit tests (UNVALIDATED: written to
// mirror the validated Python oracle tools/gyro-mouse-proto/test_mapping.py;
// NOT run in the headless Runner. Annie runs `cargo test` on a device build).
#[cfg(test)]
mod click_stab_test {
    use super::*;
    use enigo::Button;

    fn settings() -> ClickStabSettings {
        ClickStabSettings::default() // enabled, window 60ms, drag dist 40px
    }
    fn v(x: f64, y: f64) -> Vector2<f64> {
        Vector2::new(x, y)
    }
    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn idle_passes_through() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        assert_eq!(cs.filter(&s, v(5., 5.), Instant::now()), v(5., 5.));
        assert!(!cs.suppressing);
    }

    #[test]
    fn disabled_passes_through() {
        let mut cs = ClickStabilizer::default();
        let mut s = settings();
        s.enabled = false;
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert!(!cs.suppressing);
        assert_eq!(cs.filter(&s, v(5., 0.), t + ms(1)), v(5., 0.));
    }

    #[test]
    fn suppress_then_window_drops() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(3., 0.), t + ms(10)), v(0., 0.));
        assert_eq!(cs.filter(&s, v(1., 0.), t + s.window), v(0., 0.)); // window -> drop
        assert!(!cs.suppressing);
        assert_eq!(cs.filter(&s, v(5., 0.), t + ms(70)), v(5., 0.)); // passes through
    }

    #[test]
    fn click_up_releases_when_all_buttons_up() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(2., 0.), t + ms(5)), v(0., 0.));
        cs.on_click_up(Button::Left);
        assert!(!cs.suppressing);
        assert_eq!(cs.filter(&s, v(5., 0.), t + ms(6)), v(5., 0.));
    }

    #[test]
    fn fast_drag_crosses_distance_flushes() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(30., 0.), t + ms(5)), v(0., 0.));
        assert_eq!(cs.filter(&s, v(20., 0.), t + ms(10)), v(50., 0.)); // flush 50 >= 40
        assert!(!cs.suppressing);
    }

    #[test]
    fn double_button_overlap_no_premature_release() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        cs.on_click_down(&s, Button::Right, t + ms(1)); // second button: no re-arm
        assert_eq!(cs.filter(&s, v(3., 0.), t + ms(2)), v(0., 0.));
        cs.on_click_up(Button::Right);
        assert!(cs.suppressing); // L still held -> stay suppressing
    }

    #[test]
    fn second_press_during_suppress_does_not_rearm() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(35., 0.), t + ms(5)), v(0., 0.)); // accum (35,0)
        cs.on_click_down(&s, Button::Right, t + ms(6)); // must NOT reset accum
        // not re-armed: 35 + 10 = 45 >= 40 -> drag flush
        assert_eq!(cs.filter(&s, v(10., 0.), t + ms(7)), v(45., 0.));
    }

    #[test]
    fn exact_distance_boundary_is_drag() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(40., 0.), t + ms(5)), v(40., 0.)); // |.|==dist -> drag
        assert!(!cs.suppressing);
    }

    #[test]
    fn negative_vector_accumulation() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(-30., 0.), t + ms(5)), v(0., 0.));
        assert_eq!(cs.filter(&s, v(-20., 0.), t + ms(10)), v(-50., 0.)); // hypot 50 >= 40
    }

    #[test]
    fn rearm_after_all_up() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        cs.on_click_up(Button::Left);
        assert!(!cs.suppressing);
        cs.on_click_down(&s, Button::Left, t + ms(1000)); // fresh arm
        assert!(cs.suppressing);
        assert_eq!(cs.filter(&s, v(3., 0.), t + ms(1001)), v(0., 0.));
    }

    #[test]
    fn release_tick_motion_passes_through() {
        // v1 release strategy: only press-down jitter is suppressed; after all
        // buttons up, motion passes through (no release-jerk suppression).
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        assert_eq!(cs.filter(&s, v(2., 0.), t + ms(5)), v(0., 0.));
        cs.on_click_up(Button::Left);
        assert_eq!(cs.filter(&s, v(7., 3.), t + ms(6)), v(7., 3.));
    }

    #[test]
    fn tap_arms_one_shot_then_window_releases() {
        // ClickType::Click path: arm a time-bounded one-shot suppression.
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_tap(&s, t);
        assert!(cs.suppressing);
        assert_eq!(cs.filter(&s, v(2., 0.), t + ms(5)), v(0., 0.)); // tap jitter eaten
        assert_eq!(cs.filter(&s, v(1., 0.), t + s.window), v(0., 0.)); // window -> release
        assert!(!cs.suppressing);
        assert_eq!(cs.filter(&s, v(5., 0.), t + ms(70)), v(5., 0.)); // passes through
    }

    #[test]
    fn tap_ignored_while_held() {
        let mut cs = ClickStabilizer::default();
        let s = settings();
        let t = Instant::now();
        cs.on_click_down(&s, Button::Left, t);
        cs.on_click_tap(&s, t + ms(5)); // must be a no-op during a hold
        assert!(cs.suppressing);
        assert!(cs.pressed.contains(&0)); // Left still tracked
    }
}

// LEARN-81 — MotionWake state-machine unit tests (UNVALIDATED in the headless
// Runner; Annie runs `cargo test` on a real-device build). Pure logic, mirrors
// the precision-mode test style.
#[cfg(test)]
mod motion_wake_test {
    use super::*;

    fn settings(enabled: bool, wake: f64, sleep: f64, dwell_ms: u64) -> GyroSettings {
        let mut s = GyroSettings::default();
        s.motion_wake_enabled = enabled;
        s.motion_wake_speed = wake;
        s.motion_sleep_speed = sleep;
        s.motion_sleep_dwell = Duration::from_millis(dwell_ms);
        s
    }
    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    #[test]
    fn initial_state_asleep() {
        // Starts asleep so the cursor never jumps on launch.
        assert!(!MotionWake::default().awake);
    }

    #[test]
    fn disabled_transparent() {
        // Disabled = always awake + dwell cleared + transparent gate (NOT "no
        // state change"): this is the root of default-OFF bit-identical.
        let mut m = MotionWake::default();
        let s = settings(false, 8., 3., 500);
        assert!(m.update(&s, 0., ms(16))); // still input, yet awake
        assert!(m.awake);
        m.still_for = ms(999); // stale dwell from a prior enabled run
        assert!(m.update(&s, 0., ms(16)));
        assert_eq!(m.still_for, Duration::ZERO); // cleared
    }

    #[test]
    fn wakes_above_wake_speed() {
        let mut m = MotionWake::default(); // asleep
        assert!(m.update(&settings(true, 8., 3., 500), 9., ms(16)));
        assert!(m.awake);
    }

    #[test]
    fn stays_asleep_below_wake() {
        // In the (sleep 3, wake 8) band: not enough to wake from sleep.
        let mut m = MotionWake::default();
        assert!(!m.update(&settings(true, 8., 3., 500), 5., ms(16)));
        assert!(!m.awake);
    }

    #[test]
    fn dwell_required_to_sleep() {
        let s = settings(true, 8., 3., 100);
        let mut m = MotionWake::default();
        m.update(&s, 50., ms(16)); // wake
        assert!(m.awake);
        for _ in 0..5 {
            m.update(&s, 0., ms(16)); // 80ms < 100ms dwell -> still awake
        }
        assert!(m.awake);
        m.update(&s, 0., ms(40)); // total 120ms >= 100ms -> sleep
        assert!(!m.awake);
    }

    #[test]
    fn motion_resets_dwell() {
        let s = settings(true, 8., 3., 100);
        let mut m = MotionWake::default();
        m.update(&s, 50., ms(16)); // wake
        m.update(&s, 0., ms(60)); // 60ms still
        assert!(m.awake);
        m.update(&s, 50., ms(16)); // real motion resets dwell
        assert_eq!(m.still_for, Duration::ZERO);
        m.update(&s, 0., ms(60)); // only 60ms again -> still awake
        assert!(m.awake);
    }

    #[test]
    fn hysteresis_band_holds_awake() {
        // Awake + speed in (sleep, wake) -> stays awake, dwell not counting.
        let s = settings(true, 8., 3., 100);
        let mut m = MotionWake::default();
        m.update(&s, 50., ms(16)); // awake
        assert!(m.update(&s, 5., ms(16)));
        assert_eq!(m.still_for, Duration::ZERO);
        assert!(m.awake);
    }

    #[test]
    fn sleep_clamped_to_wake_when_inverted() {
        // Inverted config sleep(9) > wake(8): use-time clamp makes sleep = 8.
        let s = settings(true, 8., 9., 100);
        let mut m = MotionWake::default();
        m.update(&s, 50., ms(16)); // awake
        assert!(m.update(&s, 8.5, ms(16))); // >= clamped sleep -> dwell resets, awake
        assert_eq!(m.still_for, Duration::ZERO);
        m.update(&s, 1., ms(200)); // < sleep, past dwell -> sleep
        assert!(!m.awake);
    }

    #[test]
    fn non_finite_speed_parked() {
        // NaN/inf live readings are treated as parked (0), never as wake motion.
        let s = settings(true, 8., 3., 100);
        let mut m = MotionWake::default(); // asleep
        assert!(!m.update(&s, f64::NAN, ms(16)));
        assert!(!m.update(&s, f64::INFINITY, ms(16)));
        assert!(!m.awake);
    }

    #[test]
    fn negative_speed_clamped() {
        let s = settings(true, 8., 3., 100);
        let mut m = MotionWake::default();
        m.update(&s, 50., ms(16)); // awake
        m.update(&s, -5., ms(200)); // negative clamps to 0 -> past dwell -> sleep
        assert!(!m.awake);
    }
}

// LEARN-81 — MotionWake gate-integration tests at the Gyro::handle_frame
// boundary (UNVALIDATED in the headless Runner). `space = Local` makes the
// gyro->screen mapping deterministic (ignores the up vector).
#[cfg(test)]
mod motion_wake_gate_test {
    use super::*;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    /// A motion whose LocalSpace mapping is `(speed, 0)` (magnitude == `speed`):
    /// `LocalSpace::map = vec2(-rot.y, rot.x)`, so rot.y = -speed, rot.x = 0.
    fn motion(speed: f64) -> Motion {
        Motion {
            rotation_speed: RotationSpeed {
                x: 0.,
                y: -speed,
                z: 0.,
            },
            acceleration: Acceleration {
                x: 0.,
                y: 0.,
                z: 0.,
            },
        }
    }

    fn local_settings() -> Settings {
        let mut s = Settings::default();
        s.gyro.space = GyroSpace::Local;
        s
    }

    /// Hard requirement (a): the `- = GYRO_OFF` e-stop overrides motion-wake.
    /// Motion must never re-enable a gyro the e-stop turned off (LEARN-62).
    #[test]
    fn estop_overrides_motion_wake() {
        let mut settings = local_settings();
        settings.gyro.motion_wake_enabled = true;
        settings.gyro.motion_wake_speed = 0.; // any motion would wake it
        let mut gyro = Gyro::new(&settings, Calibration::empty());
        gyro.enabled = false; // e-stop engaged (GYRO_OFF held)
        assert_eq!(
            gyro.handle_frame(&settings, &[motion(50.)], ms(16)),
            MouseMovement::zero()
        );
        assert!(!gyro.enabled); // motion-wake never wrote enabled
        gyro.enabled = true; // release e-stop -> motion emits again
        assert_ne!(
            gyro.handle_frame(&settings, &[motion(50.)], ms(16)),
            MouseMovement::zero()
        );
    }

    /// Hard requirement (b): default OFF is an exact transparent gate. With the
    /// feature disabled, output equals the pre-feature path exactly, and the
    /// gate still reduces to `self.enabled` (hold/toggle/e-stop).
    #[test]
    fn default_off_preserves_movement_exactly() {
        let settings = local_settings();
        assert!(!settings.gyro.motion_wake_enabled); // default OFF
        let mut gyro = Gyro::new(&settings, Calibration::empty()); // enabled = true
        let dt = ms(10);
        // delta = (100, 0); speed 100 > precision exit (8) -> no precision;
        // default sens=(1,1), sign=(1,1), no smoothing/cutoff -> output =
        // delta * dt_secs, same op order as process (f64 bit-identical).
        let out = gyro.handle_frame(&settings, &[motion(100.)], dt);
        let expected = MouseMovement::from_vec_deg(Vector2::new(100. * dt.as_secs_f64(), 0.));
        assert_eq!(out, expected);
        gyro.enabled = false; // gate reduces to self.enabled -> zero
        assert_eq!(
            gyro.handle_frame(&settings, &[motion(100.)], dt),
            MouseMovement::zero()
        );
    }

    /// Auto-stop: after dwell the cursor freezes even for a real sub-wake
    /// motion (proving the sleep gate closed, not just that still frames emit
    /// zero), then a clear above-wake motion re-wakes.
    #[test]
    fn auto_stop_then_rewake() {
        let mut settings = local_settings();
        settings.gyro.motion_wake_enabled = true;
        settings.gyro.motion_wake_speed = 8.;
        settings.gyro.motion_sleep_speed = 3.;
        settings.gyro.motion_sleep_dwell = ms(100);
        let mut gyro = Gyro::new(&settings, Calibration::empty()); // starts asleep
        let dt = ms(16);
        assert_ne!(
            gyro.handle_frame(&settings, &[motion(50.)], dt), // wake
            MouseMovement::zero()
        );
        for _ in 0..8 {
            gyro.handle_frame(&settings, &[motion(0.)], dt); // 128ms still >= dwell
        }
        // asleep: a real in-band motion (sleep < 5 < wake) stays frozen
        assert_eq!(
            gyro.handle_frame(&settings, &[motion(5.)], dt),
            MouseMovement::zero()
        );
        // above-wake motion re-wakes -> moves again
        assert_ne!(
            gyro.handle_frame(&settings, &[motion(50.)], dt),
            MouseMovement::zero()
        );
    }

    #[test]
    fn empty_motions_returns_zero() {
        let settings = local_settings();
        let mut gyro = Gyro::new(&settings, Calibration::empty());
        assert_eq!(
            gyro.handle_frame(&settings, &[], ms(16)),
            MouseMovement::zero()
        );
    }

    // Codex R1 HIGH: a multi-sample batch (hidapi `report.motion`) must gate
    // each sub-frame by its OWN awake state, not apply the last frame's state
    // to the whole accumulation.
    fn motion_wake_settings() -> Settings {
        let mut s = local_settings();
        s.gyro.motion_wake_enabled = true;
        s.gyro.motion_wake_speed = 8.;
        s.gyro.motion_sleep_speed = 3.;
        s.gyro.motion_sleep_dwell = ms(1); // a single still sub-frame sleeps it
        s
    }

    #[test]
    fn multi_motion_keeps_awake_subframe_when_batch_ends_asleep() {
        // awake -> sleep within one batch: the earlier awake movement must
        // still be emitted (a last-frame gate would wrongly drop it to zero).
        let settings = motion_wake_settings();
        let mut gyro = Gyro::new(&settings, Calibration::empty());
        assert_ne!(
            gyro.handle_frame(&settings, &[motion(50.)], ms(16)), // wake first
            MouseMovement::zero()
        );
        let out = gyro.handle_frame(&settings, &[motion(50.), motion(0.)], ms(16));
        assert_ne!(out, MouseMovement::zero());
    }

    #[test]
    fn multi_motion_suppresses_asleep_subframe_when_batch_ends_awake() {
        // asleep -> wake within one batch: the leading asleep (in-band, nonzero)
        // sub-frame must contribute nothing; only the waking sub-frame emits.
        // Reference = the waking sub-frame alone at the same per-sub-frame dt.
        let settings = motion_wake_settings();
        let mut g1 = Gyro::new(&settings, Calibration::empty()); // asleep
        let batch_out = g1.handle_frame(&settings, &[motion(5.), motion(50.)], ms(16));
        let mut g2 = Gyro::new(&settings, Calibration::empty()); // asleep
        let ref_out = g2.handle_frame(&settings, &[motion(50.)], ms(8)); // dt 16/2
        assert_eq!(batch_out, ref_out);
        assert_ne!(batch_out, MouseMovement::zero());
    }
}
