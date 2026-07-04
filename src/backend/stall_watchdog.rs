//! LEARN-197: HID-stall recovery ladder (pure logic, no SDL dependency).
//!
//! The Joy-Con HID report stream can silently stop while the Bluetooth link
//! stays up (SDL's switch driver then declares a *logical* disconnect after
//! 3s of silence). Without recovery the cursor freezes until the process is
//! restarted. This module is the timing/escalation state machine; the SDL
//! backend performs the actual reopen/reinit actions and reports back.
//!
//! One `RecoveryLadder` exists per controller *identity* (name) and its
//! lifecycle is independent of the live `GameController` handle: while a
//! recovery is in flight the handle is already dropped, but the ladder keeps
//! being polled so a failed reopen can never leave a dead zone with nobody
//! in charge (Codex design review R1 §2).
//!
//! States:
//!   Unarmed  → never saw a sensor event; never fires (a controller whose
//!              sensors failed to enable must not trip the watchdog).
//!   Armed    → sensor events flowing; fires L1 after `stall_after` silence.
//!   Reopening → L1 ladder: reopen the controller every `reopen_interval`,
//!              failed opens count as attempts too; after `max_reopens`
//!              escalate to L2.
//!   ReinitPending → L2 ladder: reinit the whole game-controller subsystem
//!              with exponential backoff, capped, never gives up (the cursor
//!              is already dead — retrying costs nothing).
//!
//! Any sensor event fully resets the ladder to Armed (spontaneous recovery
//! stops all escalation). A reopen that *succeeds* is still not recovery:
//! only a real sensor event is (Codex R1 §1: reopen without fresh data must
//! keep climbing the ladder).

use std::time::{Duration, Instant};

/// Recovery action the SDL backend must perform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    /// L1: drop + reopen this controller (re-runs the driver handshake,
    /// SetInputMode + EnableIMU via sensor enable).
    ReopenController,
    /// L2: quit + reinit the game-controller subsystem (forces the HID
    /// device to be closed and reopened at the OS level).
    ReinitSubsystem,
}

/// All thresholds in one place; every value is a real-device-tunable guess
/// (标 [推测] in the plan) — see `Default`.
#[derive(Debug, Clone, Copy)]
pub struct WatchdogCfg {
    /// Sensor-event silence after which a stall is declared.
    pub stall_after: Duration,
    /// Interval between L1 reopen attempts.
    pub reopen_interval: Duration,
    /// L1 attempts (successful-but-silent or failed opens) before L2.
    pub max_reopens: u32,
    /// After an SDL-initiated logical disconnect, how long to wait for a
    /// spontaneous re-add before going straight to L2.
    pub orphan_after: Duration,
    /// First L2 retry interval; doubles each attempt.
    pub reinit_backoff_start: Duration,
    /// L2 retry interval cap.
    pub reinit_backoff_max: Duration,
    /// Post-recovery window during which motion must not be applied
    /// (swallows the resume burst that caused the 400-666px flings).
    pub grace: Duration,
}

impl Default for WatchdogCfg {
    fn default() -> Self {
        WatchdogCfg {
            stall_after: Duration::from_millis(2000),
            reopen_interval: Duration::from_millis(1000),
            max_reopens: 3,
            orphan_after: Duration::from_millis(5000),
            reinit_backoff_start: Duration::from_secs(10),
            reinit_backoff_max: Duration::from_secs(60),
            grace: Duration::from_millis(300),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum State {
    Unarmed,
    Armed {
        last_event: Instant,
    },
    Reopening {
        attempts: u32,
        next_at: Instant,
    },
    ReinitPending {
        next_at: Instant,
        /// Interval to schedule after the *next* attempt.
        backoff: Duration,
    },
}

/// Per-controller-identity stall watchdog + escalation ladder.
pub struct RecoveryLadder {
    cfg: WatchdogCfg,
    state: State,
    grace_until: Option<Instant>,
}

impl RecoveryLadder {
    pub fn new(cfg: WatchdogCfg) -> Self {
        todo!()
    }

    /// A sensor event arrived for this (active, adopted) controller.
    /// Arms the ladder / feeds the watchdog / resets all escalation.
    pub fn on_sensor_event(&mut self, now: Instant) {
        todo!()
    }

    /// SDL itself declared the device disconnected (logical disconnect —
    /// the device index is gone, so L1 is impossible). Wait `orphan_after`
    /// for a spontaneous re-add, then start the L2 ladder.
    pub fn on_device_removed(&mut self, now: Instant) {
        todo!()
    }

    /// Poll once per loop tick. Returns the action that is due, if any,
    /// and transitions so the same action is not returned again before the
    /// caller reports the result (`on_reopen_result` / `on_reinit_attempted`
    /// schedule the next deadline).
    pub fn poll(&mut self, now: Instant) -> Option<Action> {
        todo!()
    }

    /// Report the outcome of a `ReopenController` action. Failed opens
    /// count as attempts too (Codex R1 §2); success alone is not recovery —
    /// only a sensor event resets the ladder.
    pub fn on_reopen_result(&mut self, ok: bool, now: Instant) {
        todo!()
    }

    /// Report that a `ReinitSubsystem` action was performed; schedules the
    /// next retry with exponential backoff (×2, capped, never gives up).
    pub fn on_reinit_attempted(&mut self, now: Instant) {
        todo!()
    }

    /// Begin the post-recovery grace window (motion suppressed).
    pub fn start_grace(&mut self, now: Instant) {
        todo!()
    }

    /// True while inside the grace window: skip `apply_motion`.
    pub fn in_grace(&self, now: Instant) -> bool {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> WatchdogCfg {
        WatchdogCfg::default()
    }

    /// Fake timeline: t(ms) after an arbitrary epoch.
    fn timeline() -> impl Fn(u64) -> Instant {
        let t0 = Instant::now();
        move |ms: u64| t0 + Duration::from_millis(ms)
    }

    // 1. Never fires while unarmed, no matter how long the silence.
    #[test]
    fn unarmed_never_fires() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        assert_eq!(l.poll(t(0)), None);
        assert_eq!(l.poll(t(60_000)), None);
        assert_eq!(l.poll(t(3_600_000)), None);
    }

    // 2. Healthy event flow → poll always None.
    #[test]
    fn healthy_flow_never_fires() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        for i in 0..100 {
            l.on_sensor_event(t(i * 100)); // 10 Hz is plenty below stall_after
            assert_eq!(l.poll(t(i * 100 + 50)), None);
        }
    }

    // 3. Silence reaching stall_after → exactly one ReopenController; not
    //    repeated within reopen_interval.
    #[test]
    fn stall_fires_single_reopen() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(1999)), None);
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController));
        l.on_reopen_result(true, t(2000));
        // Within reopen_interval: nothing.
        assert_eq!(l.poll(t(2500)), None);
        assert_eq!(l.poll(t(2999)), None);
    }

    // 4. Still silent after reopen → retries at reopen_interval, up to
    //    max_reopens total attempts.
    #[test]
    fn silent_reopen_retries_at_interval() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController)); // attempt 1
        l.on_reopen_result(true, t(2000));
        assert_eq!(l.poll(t(3000)), Some(Action::ReopenController)); // attempt 2
        l.on_reopen_result(true, t(3000));
        assert_eq!(l.poll(t(4000)), Some(Action::ReopenController)); // attempt 3
        l.on_reopen_result(true, t(4000));
        // max_reopens = 3 reached → next due poll escalates instead.
        assert_eq!(l.poll(t(5000)), Some(Action::ReinitSubsystem));
    }

    // 5. Failed opens count as attempts too (Codex R1 §2): all-failing L1
    //    still escalates to L2, no dead zone.
    #[test]
    fn failed_opens_count_and_escalate() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(2000));
        assert_eq!(l.poll(t(3000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(3000));
        assert_eq!(l.poll(t(4000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(4000));
        assert_eq!(l.poll(t(5000)), Some(Action::ReinitSubsystem));
    }

    // 6. After max_reopens the next action is ReinitSubsystem (escalation
    //    happens exactly once at the due tick, not before).
    #[test]
    fn escalation_waits_for_due_tick() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(2000));
        assert_eq!(l.poll(t(3000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(3000));
        assert_eq!(l.poll(t(4000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(4000));
        // Not due yet (attempt scheduled next_at = 5000).
        assert_eq!(l.poll(t(4500)), None);
        assert_eq!(l.poll(t(5000)), Some(Action::ReinitSubsystem));
    }

    // 7. A sensor event at any point fully resets the ladder; a later stall
    //    starts again from L1.
    #[test]
    fn event_resets_everything() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController));
        l.on_reopen_result(false, t(2000));
        // Spontaneous recovery mid-ladder.
        l.on_sensor_event(t(2500));
        assert_eq!(l.poll(t(3000)), None);
        // New stall much later → starts from L1 again, attempts reset.
        assert_eq!(l.poll(t(4499)), None);
        assert_eq!(l.poll(t(4500)), Some(Action::ReopenController));
        l.on_reopen_result(true, t(4500));
        assert_eq!(l.poll(t(5000)), None); // interval respected, still L1
    }

    // 8. Grace window: true after start, false after expiry.
    #[test]
    fn grace_window() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        assert!(!l.in_grace(t(0)));
        l.start_grace(t(1000));
        assert!(l.in_grace(t(1000)));
        assert!(l.in_grace(t(1299)));
        assert!(!l.in_grace(t(1300)));
    }

    // 9. SDL-initiated removal → quiet for orphan_after (spontaneous re-add
    //    window), then ReinitSubsystem.
    #[test]
    fn device_removed_waits_orphan_then_reinit() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        l.on_device_removed(t(1000));
        assert_eq!(l.poll(t(1001)), None);
        assert_eq!(l.poll(t(5999)), None);
        assert_eq!(l.poll(t(6000)), Some(Action::ReinitSubsystem));
    }

    // 10. L2 backoff: 10s → 20s → 40s → 60s cap, never gives up.
    #[test]
    fn reinit_backoff_doubles_and_caps() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        l.on_device_removed(t(0));
        let mut now = 5_000; // orphan_after elapsed
        assert_eq!(l.poll(t(now)), Some(Action::ReinitSubsystem));
        l.on_reinit_attempted(t(now));
        for expected_gap in [10_000, 20_000, 40_000, 60_000, 60_000] {
            // Just before the deadline: nothing.
            assert_eq!(l.poll(t(now + expected_gap - 1)), None);
            now += expected_gap;
            assert_eq!(l.poll(t(now)), Some(Action::ReinitSubsystem));
            l.on_reinit_attempted(t(now));
        }
    }

    // 11. ReinitPending + sensor event (the re-added controller was adopted
    //     and produced data) → reset to Armed.
    #[test]
    fn reinit_pending_event_resets() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        l.on_device_removed(t(1000));
        l.on_sensor_event(t(3000)); // spontaneous re-add + data before orphan_after
        assert_eq!(l.poll(t(6000)), None);
        // Healthy again: next stall fires from L1.
        assert_eq!(l.poll(t(5000 - 1)), None);
        assert_eq!(l.poll(t(3000 + 2000)), Some(Action::ReopenController));
    }

    // 12. A reopen that succeeds but yields no events is NOT recovery: the
    //     ladder keeps counting and escalates on schedule.
    #[test]
    fn successful_but_silent_reopen_is_not_recovery() {
        let t = timeline();
        let mut l = RecoveryLadder::new(cfg());
        l.on_sensor_event(t(0));
        assert_eq!(l.poll(t(2000)), Some(Action::ReopenController));
        l.on_reopen_result(true, t(2000)); // open ok, but stream still dead
        assert_eq!(l.poll(t(3000)), Some(Action::ReopenController));
        l.on_reopen_result(true, t(3000));
        assert_eq!(l.poll(t(4000)), Some(Action::ReopenController));
        l.on_reopen_result(true, t(4000));
        assert_eq!(l.poll(t(5000)), Some(Action::ReinitSubsystem));
    }
}
