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
        settings::{ClickStabSettings, Settings},
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
                    // LEARN-69 F1 (UNVALIDATED): an instantaneous click — bracket
                    // the synthesized press/release so any same-frame jitter is
                    // briefly suppressed, then released.
                    self.click_stab.on_click_down(&self.settings.click_stab, c, now);
                    self.mouse.enigo().button(c, Direction::Click)?;
                    self.click_stab.on_click_up(c);
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

pub struct Gyro {
    enabled: bool,
    calibration: Calibration,
    sensor_fusion: Box<dyn SensorFusion>,
    space_mapper: Box<dyn SpaceMapper>,
    gyromouse: GyroMouse,
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
        }
    }

    /// Accumulate this frame's gyro movement and return it (origin bottom-left,
    /// degrees). LEARN-69 (UNVALIDATED): emission moved up to
    /// `Engine::handle_motion_frame` so click-stabilization can gate the
    /// movement before it reaches the OS. Returns zero when gyro is disabled
    /// (e.g. GYRO_OFF held) so that the e-stop pause channel still stops the
    /// cursor regardless of click state.
    pub fn handle_frame(
        &mut self,
        settings: &Settings,
        motions: &[Motion],
        dt: Duration,
    ) -> MouseMovement {
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
            let offset = self.gyromouse.process(&settings.gyro, delta, dt);
            delta_position += offset;
        }
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
}
