use cgmath::{vec2, ElementWise, InnerSpace, Vector2, Zero};
use std::{collections::VecDeque, time::Duration};

use crate::{config::settings::GyroSettings, mouse::MouseMovement};

#[derive(Debug, Default)]
pub struct GyroMouse {
    smooth_buffer: VecDeque<Vector2<f64>>,
    /// LEARN-69 Feature 2 — slow-speed auto precision mode (UNVALIDATED).
    /// Holds only transient state; all params are read from GyroSettings each
    /// frame so they stay configurable. Mirrors mapping.py::PrecisionMode.
    precision: PrecisionMode,
}

/// LEARN-69 Feature 2 — precision-mode hysteresis state machine (UNVALIDATED:
/// written but not compiled / not real-device tested in the headless Runner;
/// the Python `mapping.py::PrecisionMode` is the validated correctness oracle).
#[derive(Debug, Default)]
struct PrecisionMode {
    active: bool,
}

impl PrecisionMode {
    /// Update the sticky precision state from the current angular speed (deg/s).
    /// enter_speed < exit_speed gives hysteresis; exit >= enter is enforced here
    /// (mirrors mapping.py::PrecisionMode.update).
    fn update(&mut self, settings: &GyroSettings, speed: f64) -> bool {
        if !settings.precision_enabled {
            self.active = false;
            return false;
        }
        let speed = speed.max(0.);
        let enter = settings.precision_enter_speed.max(0.);
        let exit = settings.precision_exit_speed.max(enter);
        if self.active {
            if speed > exit {
                self.active = false;
            }
        } else if speed < enter {
            self.active = true;
        }
        self.active
    }
}

impl GyroMouse {
    //    #/// Nothing applied.
    //    pub fn blank() -> GyroMouse {
    //        GyroMouse {
    //            apply_smoothing: false,
    //            smooth_threshold: 5.,
    //            smooth_buffer: VecDeque::new(),
    //
    //            apply_tightening: false,
    //            tightening_threshold: 5.,
    //
    //            apply_acceleration: false,
    //            acceleration_slow_sens: 8.,
    //            acceleration_slow_threshold: 5.,
    //            acceleration_fast_sens: 16.,
    //            acceleration_fast_threshold: 75.,
    //
    //            sensitivity: 1.,
    //        }
    //    }
    //    /// Good default values for a 2D mouse.
    //    pub fn d2() -> GyroMouse {
    //        GyroMouse {
    //            apply_smoothing: true,
    //            smooth_threshold: 5.,
    //            smooth_buffer: [Vector2::zero(); 25].iter().cloned().collect(),
    //
    //            apply_tightening: true,
    //            tightening_threshold: 5.,
    //
    //            apply_acceleration: true,
    //            acceleration_slow_sens: 16.,
    //            acceleration_slow_threshold: 5.,
    //            acceleration_fast_sens: 32.,
    //            acceleration_fast_threshold: 75.,
    //
    //            sensitivity: 32.,
    //        }
    //    }
    //
    //    /// Good default values for a 3D mouse.
    //    pub fn d3() -> GyroMouse {
    //        GyroMouse {
    //            apply_smoothing: false,
    //            smooth_threshold: 0.,
    //            smooth_buffer: VecDeque::new(),
    //
    //            apply_tightening: false,
    //            tightening_threshold: 0.,
    //
    //            apply_acceleration: true,
    //            acceleration_slow_sens: 1.,
    //            acceleration_slow_threshold: 0.,
    //            acceleration_fast_sens: 2.,
    //            acceleration_fast_threshold: 75.,
    //
    //            sensitivity: 1.,
    //        }
    //    }
    //

    /// Process a new gyro sample.
    ///
    /// Parameter is pitch + yaw.
    ///
    /// Updates `self.orientation` and returns the applied change.
    ///
    /// `orientation` and return value have origin in bottom left.
    pub fn process(
        &mut self,
        settings: &GyroSettings,
        mut rot: Vector2<f64>,
        dt: Duration,
    ) -> MouseMovement {
        // LEARN-69 Feature 2 (UNVALIDATED): decide precision state from the raw
        // (pre-filter) angular speed in deg/s, then strengthen anti-shake and
        // lower gain ONLY while parked. boost>=1 so a base threshold of 0 stays
        // 0 (precision never enables a filter that was off) — this also keeps
        // the existing cutoff_speed==0 assertion correct when recovery is 0.
        let speed = (rot.x.powf(2.) + rot.y.powf(2.)).sqrt();
        let precision_active = self.precision.update(settings, speed);
        let smooth_threshold = if precision_active {
            settings.smooth_threshold * settings.precision_boost
        } else {
            settings.smooth_threshold
        };
        let cutoff_recovery = if precision_active {
            settings.cutoff_recovery * settings.precision_boost
        } else {
            settings.cutoff_recovery
        };
        if smooth_threshold > 0. {
            rot = self.tiered_smooth(settings, smooth_threshold, rot, dt);
        }
        if cutoff_recovery > 0. {
            #[allow(clippy::float_cmp)]
            {
                assert_eq!(settings.cutoff_speed, 0.);
            }
            rot = self.tight(cutoff_recovery, rot);
        }
        let mut sens = self.get_sens(settings, rot);
        if precision_active {
            // precision mode temporarily lowers gain (clamped to (0,1] at apply)
            sens = sens * settings.precision_gain;
        }
        let sign = self.get_sign(settings);
        MouseMovement::from_vec_deg(
            rot.mul_element_wise(sens).mul_element_wise(sign) * dt.as_secs_f64(),
        )
    }

    fn tiered_smooth(
        &mut self,
        settings: &GyroSettings,
        thresh_high: f64,
        rot: Vector2<f64>,
        dt: Duration,
    ) -> Vector2<f64> {
        let thresh_low = thresh_high / 2.;
        let magnitude = (rot.x.powf(2.) + rot.y.powf(2.)).sqrt();
        let weight = ((magnitude - thresh_low) / (thresh_high - thresh_low))
            .max(0.)
            .min(1.);
        let smoothed = self.smooth(settings, rot * (1. - weight), dt);
        rot * weight + smoothed
    }

    fn smooth(&mut self, settings: &GyroSettings, rot: Vector2<f64>, dt: Duration) -> Vector2<f64> {
        self.smooth_buffer.push_front(rot);
        while dt * self.smooth_buffer.len() as u32 > settings.smooth_time {
            self.smooth_buffer.pop_back();
        }
        let sum = self
            .smooth_buffer
            .iter()
            .fold(Vector2::zero(), |acc, x| acc + x);
        sum / self.smooth_buffer.len() as f64
    }

    fn tight(&mut self, cutoff_recovery: f64, rot: Vector2<f64>) -> Vector2<f64> {
        let magnitude = (rot.x.powf(2.) + rot.y.powf(2.)).sqrt();
        if magnitude < cutoff_recovery {
            let scale = magnitude / cutoff_recovery;
            rot * scale
        } else {
            rot
        }
    }

    fn get_sens(&self, settings: &GyroSettings, rot: Vector2<f64>) -> Vector2<f64> {
        if settings.slow_sens.magnitude2() > 0. && settings.slow_sens.magnitude2() > 0. {
            let magnitude = (rot.x.powf(2.) + rot.y.powf(2.)).sqrt();
            let factor = ((magnitude - settings.slow_threshold)
                / (settings.fast_threshold - settings.slow_threshold))
                .max(0.)
                .min(1.);
            settings.slow_sens * (1. - factor) + settings.fast_sens * factor
        } else {
            settings.sens
        }
    }

    fn get_sign(&self, settings: &GyroSettings) -> Vector2<f64> {
        let x = if settings.invert.0 { -1. } else { 1. };
        let y = if settings.invert.1 { -1. } else { 1. };
        vec2(x, y)
    }
}

// LEARN-69 Feature 2 — PrecisionMode unit tests (UNVALIDATED: written to mirror
// the validated Python oracle tools/gyro-mouse-proto/test_mapping.py; NOT run in
// the headless Runner. Annie runs `cargo test` on a real-device build).
#[cfg(test)]
mod precision_test {
    use super::*;
    use crate::config::settings::GyroSettings;

    fn settings(enabled: bool, enter: f64, exit: f64) -> GyroSettings {
        let mut s = GyroSettings::default();
        s.precision_enabled = enabled;
        s.precision_enter_speed = enter;
        s.precision_exit_speed = exit;
        s
    }

    #[test]
    fn disabled_never_active() {
        let mut p = PrecisionMode::default();
        assert!(!p.update(&settings(false, 3., 8.), 0.));
        assert!(!p.active);
    }

    #[test]
    fn enter_below_enter_speed() {
        let mut p = PrecisionMode::default();
        assert!(p.update(&settings(true, 3., 8.), 2.));
    }

    #[test]
    fn exit_above_exit_speed() {
        let mut p = PrecisionMode::default();
        let s = settings(true, 3., 8.);
        p.update(&s, 2.); // active
        assert!(!p.update(&s, 9.));
    }

    #[test]
    fn hysteresis_band_holds_state() {
        let s = settings(true, 3., 8.);
        let mut inactive = PrecisionMode::default();
        assert!(!inactive.update(&s, 5.)); // stays inactive in the band
        let mut active = PrecisionMode::default();
        active.update(&s, 1.); // active
        assert!(active.update(&s, 5.)); // stays active in the band
    }

    #[test]
    fn exit_clamped_to_enter_when_inverted() {
        // exit < enter -> use-time clamp makes exit = enter; speed 2 < 8 -> active
        let mut p = PrecisionMode::default();
        assert!(p.update(&settings(true, 8., 3.), 2.));
    }

    #[test]
    fn negative_speed_clamped() {
        // negative speed clamps to 0 (treated as parked) — mirrors mapping.py.
        let mut p = PrecisionMode::default();
        assert!(p.update(&settings(true, 3., 8.), -5.));
    }
}
