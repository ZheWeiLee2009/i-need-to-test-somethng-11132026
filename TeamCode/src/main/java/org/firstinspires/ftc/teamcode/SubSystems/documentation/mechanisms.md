> Do not modify this in any repos other than this one without knowing how to deal with git-merge.
# Mechanism Subsystems — Launcher & Intake Usage Guide

Covers the two non-drive, non-servo mechanisms in this project: `LauncherSubSystem` (flywheel
velocity control) and `IntakeSubSystem` (a single on/off roller motor). For servo-driven
mechanisms (turret, pitch, wrist, ...) see `servos.md`; for driving, see `drive.md`.

---

## 1. IntakeSubSystem

The simplest subsystem in the project — one motor, two states.

```java
bot.intake.on();   // full power
bot.intake.off();  // zero power
```

- `stop()` (called from `PracticeBot`/an OpMode's emergency-stop path) just calls `off()`.
- It has no `update()` override and is never marked active — there's nothing for it to do in
  the background. If you add sensing (e.g. a color sensor to detect a held game piece) or
  variable-speed control, that's when `update()` and `setActive(true)` start being useful —
  follow the pattern in `LauncherSubSystem` below.

**Pitfall:** `IntakeSubSystem` only wraps `Motors.get(0)` — if you ever wire a second intake
motor, either pass both motors in and drive them together inside `on()`/`off()`, or give the
second motor its own field. Silently ignoring `Motors.get(1)` is an easy bug to introduce later.

---

## 2. LauncherSubSystem

Closed-loop flywheel RPM control: a distance→RPM lookup table for automatic aiming, a manual RPM
override, and a PIDF velocity loop that runs in `update()`.

### 2.1 Two ways to set a target

```java
// Automatic: look up target RPM from a distance-to-goal lookup table, then PIDF to it
bot.launcher.autoPIDF(bot.odo.getPosition(), goalPose);

// Manual: skip the lookup table, PIDF straight to a specific RPM
bot.launcher.setRPM(3200);
```

Both funnel into the same internal `setTargetRPM()` — the only difference is whether the target
RPM comes from `flywheelLut` (distance → RPM, calibrated in `setup()`) or from you directly.
Calling either with `rpm <= 0` turns the flywheel off (see below), which also works as a
one-line "shut off the shooter" call if that's ever more convenient than `off()`.

### 2.2 The control loop

`update()` must be called every loop (via `bot.update()`, and only while
`isActive()` is true — see §2.3) for the flywheel to actually spin up: it reads the average
velocity of both flywheel motors, feeds it through the PIDF controller against `targetRPM`, and
writes the resulting power (clamped to ±1.0) to both motors. Skipping `update()` for a few loops
just means the flywheel doesn't correct — it won't damage anything, but it won't spin up either.

```java
if (bot.launcher.atTarget()) {
    // within 50 RPM of target — safe to fire
}
```

Useful telemetry/debugging getters: `getCurrentRPM()`, `getTargetRPM()`, `getOutput()`.

### 2.3 Activation

Unlike `IntakeSubSystem`, `LauncherSubSystem` **does** need `update()` to run every loop while
it's spinning — but only then. `setTargetRPM()` calls `setActive(true)` automatically when you
give it a positive RPM, and `off()` calls `setActive(false)`. You don't need to call
`setActive(...)` yourself in normal use; it's handled for you as part of turning the shooter on
and off. This is also why `LauncherSubSystem` must be in `PracticeBot`'s `subsystems` list —
without that, `bot.update()` never calls its `update()` at all, active or not, and the flywheel
will never reach target RPM no matter what you set it to.

### 2.4 Tuning the PIDF gains

`LauncherConstants.kP/kI/kD/kF` are documented inline with units in `LauncherConstants.java`.
Typical tuning order for a flywheel:

1. Start with `kI = kD = 0`.
2. Raise `kF` until commanding a target RPM gets you *close* to it almost immediately (feedforward
   does most of the work for a flywheel — a rough starting point is `1.0 / (max RPM your flywheel
   can actually reach)`).
3. Raise `kP` to close the remaining gap and recover RPM after a shot (which momentarily drops
   flywheel speed).
4. Only add `kI` if you see a *persistent* gap between target and current RPM that `kP` alone
   won't close. Add `kD` only if you see oscillation around the target.
5. `setPIDF(p, i, d, f)` exists for live-tuning from a test OpMode (see `LauncherTest`) — it's a
   no-op if you pass the same values already set, so it's safe to call every loop from a tuning
   dashboard without spamming `pidf.reset()`.

### 2.5 The distance→RPM lookup table

`flywheelLut` is built once in `setup()` from hardcoded distance/RPM pairs (24"→2800 RPM up to
72"→4000 RPM). These values are specific to one robot's flywheel — if you copy this subsystem to
a new robot, re-measure and replace them; don't assume they transfer.

---

## 3. Common pitfalls

| Pitfall | Fix |
|---|---|
| Flywheel never spins up | Confirm `LauncherSubSystem` is actually in `PracticeBot`'s `subsystems` list — if it isn't, `setup()` never runs (no motor mode/zero-power config, no LUT) and `update()` never ticks (no PIDF). |
| Flywheel spins but never reaches target | Tune `kF` first (see §2.4) — a pure-`kP` loop on a flywheel is usually too slow to spin up in the time available between shots. |
| `autoPIDF` throws / gives a nonsense RPM | `flywheelLut` only has entries from 24" to 72" — `InterpLUT` extrapolates outside that range, which can give wildly wrong RPMs at very short or very long distances. Clamp distance before calling, or extend the table. |
| Intake and launcher both "stop" from an emergency-stop button but the flywheel keeps coasting | That's expected — `stop()` sets power to 0 but a flywheel has physical inertia. If you need it to stop *instantly*, that requires a physical brake, not software. |
