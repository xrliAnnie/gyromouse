use std::{
    collections::HashMap,
    thread::sleep,
    time::{Duration, Instant},
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

use super::Backend;

pub struct SDLBackend {
    sdl: Sdl,
    game_controller_system: GameControllerSubsystem,
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

        // Better Windows support
        sdl2::hint::set("SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1");
        sdl2::hint::set("SDL_HINT_JOYSTICK_THREAD", "1");

        let sdl = sdl2::init().expect("can't initialize SDL");
        let game_controller_system = sdl
            .game_controller()
            .expect("can't initialize SDL game controller subsystem");
        Ok(Self {
            sdl,
            game_controller_system,
        })
    }
}

impl Backend for SDLBackend {
    fn list_devices(&mut self) -> anyhow::Result<()> {
        let num_joysticks = match self.game_controller_system.num_joysticks() {
            Ok(x) => x,
            Err(e) => bail!("{}", e),
        };
        if num_joysticks == 0 {
            println!("No controller detected");
        } else {
            println!("Detected controllers:");
            for i in 0..num_joysticks {
                let controller = self.game_controller_system.open(i)?;
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
                        let mut controller = self.game_controller_system.open(which)?;

                        if controllers
                            .values()
                            .any(|c| c.controller.name() == controller.name())
                        {
                            continue;
                        }

                        if controller.name() == "Steam Virtual Gamepad" {
                            continue;
                        }

                        println!("New controller: {}", controller.name());

                        // Ignore errors, handled later
                        let calibrator = if controller
                            .sensor_set_enabled(SensorType::Accelerometer, true)
                            .and(controller.sensor_set_enabled(SensorType::Gyroscope, true))
                            .is_ok()
                        {
                            println!(
                                "Starting calibration for {}, don't move the controller...",
                                controller.name()
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
                        controllers.insert(
                            controller.instance_id(),
                            ControllerState {
                                controller,
                                engine,
                                calibrator,
                                zl_pressed: false,
                                zr_pressed: false,
                            },
                        );
                    }
                    Event::ControllerDeviceRemoved { which, .. } => {
                        if let Some(controller) = controllers.remove(&which) {
                            println!("Controller disconnected: {}", controller.controller.name());
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

                // LEARN-69 F1 (UNVALIDATED): process queued actions (mouse
                // press/release, GYRO_OFF, ...) BEFORE this tick's motion so a
                // click edge arms click-stabilization in time to suppress the
                // same-tick click jerk, and a GYRO_OFF press stops this tick's
                // motion. Release is also processed here, so the release-tick
                // motion passes through (v1 does not suppress release jerk —
                // documented release strategy).
                engine.apply_actions(now)?;

                if c.sensor_enabled(SensorType::Accelerometer)
                    && c.sensor_enabled(SensorType::Gyroscope)
                {
                    let mut accel = [0.; 3];
                    c.sensor_get_data(SensorType::Accelerometer, &mut accel)?;
                    let acceleration = Acceleration::from(
                        Vector3::from(accel)
                            .cast::<f64>()
                            .expect("can't cast f32 to f64")
                            / 9.82,
                    );
                    let mut gyro = [0.; 3];
                    c.sensor_get_data(SensorType::Gyroscope, &mut gyro)?;
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
                        engine.apply_motion(rotation_speed, acceleration, now, dt);
                    }
                }
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
    // SDL reports ZL/ZR as trigger axes, not buttons; track the pressed
    // state so the poll loop can synthesize key up/down edges.
    zl_pressed: bool,
    zr_pressed: bool,
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
