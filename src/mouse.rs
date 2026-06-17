use std::ops::AddAssign;

use cgmath::{vec2, Deg, Vector2, Zero};
use enigo::{Coordinate, Enigo, Mouse as _};

use crate::config::settings::MouseSettings;

// PartialEq (LEARN-81): lets the engine gate tests assert exact zero/nonzero
// movement; Deg<f64> already implements PartialEq.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MouseMovement {
    /// Horizontal axis, + to the right
    x: Deg<f64>,
    /// Vertical axis, + to the top
    y: Deg<f64>,
}

impl MouseMovement {
    pub fn new(x: Deg<f64>, y: Deg<f64>) -> Self {
        Self { x, y }
    }
    pub fn zero() -> Self {
        Self::new(Deg(0.), Deg(0.))
    }
    /// Convert a Vector2 with degree movement values
    pub fn from_vec_deg(vec: Vector2<f64>) -> Self {
        Self {
            x: Deg(vec.x),
            y: Deg(vec.y),
        }
    }
}

impl AddAssign for MouseMovement {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

#[derive(Debug)]
pub struct Mouse {
    enigo: Enigo,
    error_accumulator: Vector2<f64>,
}

impl Mouse {
    pub fn new() -> anyhow::Result<Self> {
        Ok(Mouse {
            enigo: Enigo::new(&enigo::Settings::default())?,
            error_accumulator: Vector2::zero(),
        })
    }

    /// Convert a gyro `MouseMovement` (degrees, +y up) to a float pixel delta
    /// (+y down) using the calibration. LEARN-69 (UNVALIDATED): exposed so the
    /// engine can gate movement in float pixel space BEFORE the integer
    /// quantization / `error_accumulator` in `mouse_move_relative_pixel`.
    pub fn movement_to_pixels(
        &self,
        settings: &MouseSettings,
        offset: MouseMovement,
    ) -> Vector2<f64> {
        vec2(offset.x.0, -offset.y.0) * settings.real_world_calibration * settings.in_game_sens
    }

    // mouse movement is pixel perfect, so we keep track of the error.
    pub fn mouse_move_relative(&mut self, settings: &MouseSettings, offset: MouseMovement) {
        let offset_pixel = self.movement_to_pixels(settings, offset);
        self.mouse_move_relative_pixel(offset_pixel);
    }

    pub fn mouse_move_relative_pixel(&mut self, offset: Vector2<f64>) {
        let sum = offset + self.error_accumulator;
        let rounded = vec2(sum.x.round(), sum.y.round());
        self.error_accumulator = sum - rounded;
        if let Some(rounded) = rounded.cast::<i32>() {
            if rounded != Vector2::zero() {
                // In enigo, +y is toward the bottom
                self.enigo
                    .move_mouse(rounded.x, rounded.y, Coordinate::Rel)
                    .unwrap();
            }
        }
    }

    pub fn mouse_move_absolute_pixel(&mut self, offset: Vector2<i32>) {
        self.enigo
            .move_mouse(offset.x, offset.y, Coordinate::Abs)
            .unwrap();
    }

    pub fn enigo(&mut self) -> &mut Enigo {
        &mut self.enigo
    }

    /// Keyboard emission with platform fixups. On macOS the Fn/Globe key
    /// must carry NX_SECONDARYFNMASK on key-down (and clear it on key-up)
    /// or system listeners ignore it; enigo posts the bare keycode only,
    /// so mirror JoyKeyMapper's CGEvent behavior for that key.
    pub fn key(&mut self, key: enigo::Key, direction: enigo::Direction) -> anyhow::Result<()> {
        #[cfg(target_os = "macos")]
        if matches!(key, enigo::Key::Function) {
            return send_macos_fn_key(direction);
        }
        use enigo::Keyboard as _;
        self.enigo.key(key, direction)?;
        Ok(())
    }
}

#[cfg(target_os = "macos")]
fn send_macos_fn_key(direction: enigo::Direction) -> anyhow::Result<()> {
    use anyhow::anyhow;
    use core_graphics::event::{CGEvent, CGEventFlags, CGEventTapLocation};
    use core_graphics::event_source::{CGEventSource, CGEventSourceStateID};

    const FN_KEYCODE: u16 = 63; // kVK_Function

    let send = |key_down: bool| -> anyhow::Result<()> {
        let source = CGEventSource::new(CGEventSourceStateID::HIDSystemState)
            .map_err(|()| anyhow!("can't create CGEventSource"))?;
        let event = CGEvent::new_keyboard_event(source, FN_KEYCODE, key_down)
            .map_err(|()| anyhow!("can't create Fn key CGEvent"))?;
        event.set_flags(if key_down {
            CGEventFlags::CGEventFlagSecondaryFn
        } else {
            CGEventFlags::CGEventFlagNull
        });
        event.post(CGEventTapLocation::HID);
        Ok(())
    };

    match direction {
        enigo::Direction::Press => send(true),
        enigo::Direction::Release => send(false),
        enigo::Direction::Click => {
            send(true)?;
            send(false)
        }
    }
}
