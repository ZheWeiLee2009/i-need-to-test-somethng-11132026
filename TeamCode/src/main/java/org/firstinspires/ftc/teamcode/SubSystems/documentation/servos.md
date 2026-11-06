> >Do not modify this in any repos other than this one without knowing how to deal with git-merge.
# Servo Subsystem — Complete Usage Guide

A task-oriented servo control layer for FTC robots. Command angles in degrees relative to the robot's front; the system handles per-servo calibration, mirrored mounts, clamping, and hardware access.

---

## 1. Architecture at a glance

```
┌─────────────────────────────────────────────────────────────┐
│  OpMode                                                     │
│    bot.turret.setAngle(45.0);                               │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  ServoTaskSubsystem  (generic, extends Subsystem)            │
│    one instance per axis (turret, pitch, ...)                │
│    owns ──► ServoAxis                                        │
│    future: vision tracking, limit switches, PID             │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  ServoAxis  (utils — reusable helper, NOT a Subsystem)      │
│    owns Servo[]  +  calibration  +  lastAngle               │
│    uses ──► ServoUtility for all math                       │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  ServoUtility  (utils — pure static math, no hardware)      │
│    clamp, mapRange, degreesToPosition, slewLimit, ...       │
└─────────────────────────────────────────────────────────────┘

ServoConstants (Configs) ── read by PracticeBot & ServoTest
```

**Rule of thumb:**
- Extends `Subsystem` → `SubSystems/`
- Reusable helper → `utils/`
- Per-robot number → `Configs/roboConfigs/`

---

## 2. File map

| File | Package | Role |
|---|---|---|
| `ServoConstants.java` | `Configs.roboConfigs` | **Edit per robot.** Limits + calibration. |
| `ServoUtility.java` | `utils` | Pure math. Never edit. |
| `ServoAxis.java` | `utils` | Servos on one axis. Never edit. |
| `ServoTaskSubsystem.java` | `SubSystems` | Generic task subsystem. One instance per axis — not edited per axis. |
| `PracticeBot.java` | `teamcode` | Wiring. Edit once per new axis. |
| `ServoTest.java` | `utils.testClasses` | Calibration OpMode. |

---

## 3. First-time setup (new robot)

### Step 1 — Name your servos in the Driver Station

Configure the hardware map in the Robot Controller app. Note the exact names (case sensitive). Example: `turretLeft`, `turretRight`, `pitchServo`.

### Step 2 — Fill in `ServoConstants.java`

```java
public static final class Turret {
    public static final String[] NAMES   = { "turretLeft", "turretRight" };  // hardware map names
    public static final double   MIN_DEG = -80.0;   // leftmost safe angle, deg from robot front; the physical limits of the actual thing
    public static final double   MAX_DEG =  80.0;   // rightmost safe angle
    public static final double[] AT_MIN  = { 0.05, 0.95 };  // servo positions at MIN_DEG
    public static final double[] AT_MAX  = { 0.95, 0.05 };  // servo positions at MAX_DEG
}
```

Leave `AT_MIN` / `AT_MAX` at defaults for now. You'll measure them in Step 4.

**Order matters.** `NAMES[i]` corresponds to `AT_MIN[i]` and `AT_MAX[i]`. All three arrays must have the same length.

### Step 3 — Wire it into `PracticeBot`

```java
// raw servo fields (for debugging)
public Servo turretLeft, turretRight;
public Servo pitchServo;

// task subsystems — ServoTaskSubsystem is generic; one instance per axis
public ServoTaskSubsystem turret;
public ServoTaskSubsystem pitch;

// in the constructor:
turretLeft  = hwMap.get(Servo.class, ServoConstants.Turret.NAMES[0]);
turretRight = hwMap.get(Servo.class, ServoConstants.Turret.NAMES[1]);
pitchServo  = hwMap.get(Servo.class, ServoConstants.Pitch.NAMES[0]);

turret = new ServoTaskSubsystem(
        "turret",
        new Servo[]{ turretLeft, turretRight },
        ServoConstants.Turret.MIN_DEG,
        ServoConstants.Turret.MAX_DEG,
        ServoConstants.Turret.AT_MIN,
        ServoConstants.Turret.AT_MAX);

pitch = new ServoTaskSubsystem(
        "pitch",
        new Servo[]{ pitchServo },
        ServoConstants.Pitch.MIN_DEG,
        ServoConstants.Pitch.MAX_DEG,
        ServoConstants.Pitch.AT_MIN,
        ServoConstants.Pitch.AT_MAX);

subsystems.add(turret);
subsystems.add(pitch);
```

### Step 4 — Calibrate with `ServoTest`

Run the `ServoTest` OpMode from the Driver Station.

| Control | Action |
|---|---|
| **Share** | Cycle axis (turret ↔ pitch) |
| **A** | Cycle servo within the active axis |
| **B** | Toggle RAW (direct write) vs ANGLE (through subsystem) |
| **Dpad L/R** | Nudge (raw position or angle) |
| **Bumpers** | Change raw step size (fine `0.001` / coarse `0.020`) |
| **Dpad U/D** | Change angle step size (`0.5` / `10`) |
| **X / Y** | Jump to MIN / MAX |
| **Options** | Log the current raw position and its implied angle |

**Procedure for one servo:**

1. Press **B** until mode shows `RAW (direct)`.
2. Press **A** to select the servo.
3. Dpad to the **leftmost safe angle** (stop just before any bind).
4. Note the `Raw position` on telemetry → this is `AT_MIN[i]`.
5. Dpad to the **rightmost safe angle**.
6. Note the `Raw position` → this is `AT_MAX[i]`.
7. Repeat for each servo on the axis.
8. Paste all values into `ServoConstants.java`.
9. Press **B** to switch to `ANGLE` mode. Command 45°, 0°, -45°. Verify physically.
10. Repeat for the other axis.

**Mirrored mounts:** swap that servo's `AT_MIN` and `AT_MAX` values. No `setInverted` needed.

**Tip:** add a small safety margin (e.g. drive to `AT_MIN + 0.01`) so you never grind against a hard stop.

---

## 4. Daily usage in OpModes

### Minimal boilerplate

```java
@TeleOp(name = "MyOp", group = ".")
public class MyOp extends OpMode {

    private PracticeBot bot;

    @Override
    public void init() {
        bot = new PracticeBot(hardwareMap, new ElapsedTime());
    }

    @Override
    public void loop() {
        bot.update();   // clears bulk cache, ticks active subsystems

        bot.turret.setAngle(45.0);
        bot.pitch.setAngle(15.0);
    }
}
```

### Commanding angles

```java
// Absolute, clamped to [MIN_DEG, MAX_DEG]
bot.turret.setAngle(45.0);

// Relative to last commanded angle
bot.turret.nudge(-10.0);

// Jump to configured limits
bot.turret.atMin();
bot.turret.atMax();
```

### Reading state

```java
double commanded = bot.turret.getAngle();      // last commanded angle (deg)
double pos0       = bot.turret.getPosition(0); // current position of servo 0 (0..1)
```

### Raw hardware access (debugging / calibration only)

```java
bot.turretLeft.setPosition(0.42);   // bypasses all mapping
bot.turretRight.setPosition(0.58);
```

**Warning:** raw writes leave `lastAngle` stale. The next `setAngle(...)` resets it. That's fine for debug use — don't mix raw and high-level control in the same code path during a match.

### Joystick control (common pattern)

```java
// Scaled proportional control
double speed = 60.0;   // deg per second at full stick
double delta = -gamepad2.left_stick_x * speed * 0.02;  // dt ~ 20ms
bot.turret.nudge(delta);
```

Or rate-limited absolute:

```java
double target = -gamepad2.left_stick_x * 80.0;   // ±80°
double next = ServoUtility.slewLimit(bot.turret.getAngle(), target, 3.0);
bot.turret.setAngle(next);
```

---

## 5. Adding more servos or axes

### Adding a servo to an **existing** axis

Edit only `ServoConstants` and `PracticeBot`.

1. `ServoConstants.Turret` — add one entry to `NAMES`, `AT_MIN`, `AT_MAX`:
   ```java
   NAMES  = { "turretLeft", "turretRight", "turretMid" };
   AT_MIN = { 0.05, 0.95, 0.05 };
   AT_MAX = { 0.95, 0.05, 0.95 };
   ```
2. `PracticeBot` — add a raw field and pass it into the constructor array:
   ```java
   public Servo turretMid;
   // ...
   turretMid = hwMap.get(Servo.class, ServoConstants.Turret.NAMES[2]);
   turret = new ServoTaskSubsystem("turret",
           new Servo[]{ turretLeft, turretRight, turretMid }, ...);
   ```
3. Calibrate the new servo in `ServoTest`.

**No changes to `ServoAxis` or `ServoUtility`.** They already support any N.

### Adding a **new axis** (e.g. wrist)

1. `ServoConstants.java` — add a nested class:
   ```java
   public static final class Wrist {
       public static final String[] NAMES   = { "wristServo" };
       public static final double   MIN_DEG = -90.0;
       public static final double   MAX_DEG =  90.0;
       public static final double[] AT_MIN  = { 0.05 };
       public static final double[] AT_MAX  = { 0.95 };
   }
   ```
2. `PracticeBot` — raw field, `ServoTaskSubsystem` field, resolve, construct with the
   new axis's name (`new ServoTaskSubsystem("wrist", ...)`), `subsystems.add(wrist)`.
   No new Java file — `ServoTaskSubsystem` is generic. Only subclass it if this
   particular axis needs behavior the others don't (e.g. auto-aim).
3. (Optional) Add a case to `ServoTest`'s axis cycling if you want to calibrate it from the Driver Station.

---

## 6. Optimization

### Loop cost

`bot.update()` only ticks subsystems whose `isActive()` returns `true`. Servo subsystems default to inactive, so they cost **zero** per loop until you opt in.

```java
// Turn on per-loop behavior (auto-aim, tracking, etc.)
bot.turret.setActive(true);

// Turn it back off when idle
bot.turret.setActive(false);
```

If your servo subsystem's `update()` is still a no-op, leave `isActive` false and just call `setAngle(...)` on demand. That's the fastest configuration.

### Avoiding redundant writes

`ServoAxis.setAngle` writes to every servo every call. If you're calling it every loop with the same value, that's a wasted I²C/PWM write per servo. Two options:

**Option A — cache in your OpMode:**
```java
private double lastTurretTarget = Double.NaN;

double target = ...;
if (target != lastTurretTarget) {
    bot.turret.setAngle(target);
    lastTurretTarget = target;
}
```

**Option B — add caching to `ServoAxis`** (matches the `DriveSubSystem.applyOptimizedPower` pattern you already use):
```java
private static final double EPS = 1.0 / 1000.0;
private double lastWritten = Double.NaN;

public void setAngle(double deg) {
    double a = ServoUtility.clampAngle(deg, minDeg, maxDeg);
    if (Math.abs(a - lastWritten) < EPS) return;
    // ... existing write ...
    lastWritten = a;
}
```

### Allocation-free calls

`ServoAxis` reuses an internal `posBuffer`, so `setAngle` allocates nothing. Good for 100 Hz loops.

`ServoUtility.degreesToPositions(...)` is the bulk alloc-free path. Use it in custom code if you ever bypass `ServoAxis`.

### Slew limiting (smoother motion)

Prevents servos from slamming and reduces current spikes:

```java
double target = 45.0;
double next = ServoUtility.slewLimitPerSecond(
        bot.turret.getAngle(),
        target,
        /* degPerSec = */ 180.0,
        /* dt = */ getRuntime() - lastTime);
bot.turret.setAngle(next);
```

### Bulk cache interaction

`PracticeBot.update()` clears the Lynx bulk cache. Servo writes are **PWM**, not bulk-cached I²C reads, so they're unaffected. No action needed.

---

## 7. Debugging

### When the turret doesn't move

1. Check the hardware map name matches `ServoConstants.NAMES[i]` exactly (case-sensitive).
2. Confirm the servo is plugged into a **Servo Port** (not a Motor Port).
3. In the Driver Station, check the servo's "Servo Controller" tab — you should see the commanded position change when you call `setAngle`.
4. Run `ServoTest` in RAW mode and write `0.0`, `0.5`, `1.0` directly. If nothing moves, it's wiring or power.

### When the turret moves the wrong direction

- **Mirrored axis?** Swap the servo's `AT_MIN[i]` and `AT_MAX[i]` values.
- **Whole axis inverted?** Swap `MIN_DEG` and `MAX_DEG` and swap `AT_MIN` / `AT_MAX` per servo.

### When it binds at the extremes

- Trim `AT_MIN` inward by 0.01–0.02 (e.g. `0.05` → `0.06`) until binding stops.
- Or use `ServoUtility.marginMin/Max` if you want a software safe range:
  ```java
  double safeMin = ServoUtility.marginMin(-80, 80, 5);  // -75
  ```

### When the position drifts over time

Servos hold position as long as PWM is present. If the turret creeps, it's mechanical (gear slop, backdrive) — not software. Add a physical detent or a stronger servo.

### Logging in `ServoTest`

Press **Options** to log the current raw position and its implied angle. Use this when tuning — the Driver Station log preserves the readings after the run.

---

## 8. Common pitfalls

| Pitfall | Fix |
|---|---|
| Array length mismatch between `NAMES`, `AT_MIN`, `AT_MAX` | Constructor throws at `PracticeBot` init. Keep all three the same length. |
| Forgot to add to `subsystems` list | `bot.update()` won't tick it. Add `subsystems.add(myAxis)`. |
| Mixed raw and high-level writes | `lastAngle` becomes stale. Pick one style per code path. |
| Called `bot.turret.setAngle` before `PracticeBot` constructor finished | Impossible — fields are set in the constructor. Only an issue if you cache `bot.turret` in a field before `new PracticeBot(...)`. |
| Using `setInverted` on a servo | Don't. Mirror by swapping `AT_MIN` / `AT_MAX`. |
| Expecting position feedback | `Servo` has no `getPosition`. `getAngle()` returns the last **commanded** angle. |
| Editing `ServoUtility` or `ServoAxis` for robot-specific values | Those are robot-agnostic. Put per-robot numbers in `ServoConstants`. |

---

## 9. API reference

### `ServoAxis` (used via task subsystems)

| Method | Returns | Notes |
|---|---|---|
| `setAngle(double deg)` | `void` | Clamped, per-servo mapped, one write per servo |
| `nudge(double delta)` | `void` | Relative to last commanded |
| `atMin()` / `atMax()` | `void` | Jump to configured limits |
| `getAngle()` | `double` | Last commanded angle (deg) |
| `getPosition(int i)` | `double` | Position (0..1) for servo i at current angle |
| `getServos()` | `Servo[]` | Direct access (debug) |
| `getMinDeg()` / `getMaxDeg()` | `double` | Configured limits |
| `getServoCount()` | `int` | N |
| `getName()` | `String` | Label |
| `stop()` | `void` | No-op; override in subclass for stow |

### `ServoTaskSubsystem` (generic — one instance per axis)

Same public API as `ServoAxis`, forwarded. Plus:
- `getAxis()` → returns the internal `ServoAxis` (for `ServoTest` and advanced code).
- `setActive(boolean)` / `isActive()` → inherited from `Subsystem`, controls whether `update()` ticks.

### `ServoUtility` — commonly used

| Method | Use |
|---|---|
| `clamp(v, lo, hi)` | Generic clamp |
| `clampAngle(deg, min, max)` | Angle clamp |
| `mapRange(v, inMin, inMax, outMin, outMax)` | Linear remap |
| `degreesToPosition(deg, minDeg, maxDeg, atMin, atMax)` | Single conversion |
| `degreesToPositions(..., out[])` | Bulk, alloc-free |
| `positionToDegrees(pos, ...)` | Inverse (calibration) |
| `slewLimit(cur, target, maxStep)` | Per-call rate limit |
| `slewLimitPerSecond(cur, target, rate, dt)` | dt-based rate limit |
| `wrapDegrees180(deg)` / `wrapDegrees360(deg)` | Angle normalization |
| `shortestAngleDelta(from, to)` | Shortest signed rotation |
| `smoothStep(t)` / `easeInOut(t, exp)` | Easing |
| `invertPosition(pos)` | Flip 0↔1 |
| `deadbandClamp(pos, deadband)` | Snap to center |
| `marginMin/Max(minDeg, maxDeg, margin)` | Shrink safe range |
| `positionToPulseMs(pos, minMs, maxMs)` | For smart servos |

---

## 10. Quick copy-paste recipes

### Joystick turret with rate limiting

```java
double target = -gamepad2.left_stick_x * 80.0;
double next = ServoUtility.slewLimitPerSecond(
        bot.turret.getAngle(), target, 240.0, 0.02);
bot.turret.setAngle(next);
```

### Two-position pitch (high/low preset)

```java
bot.pitch.setAngle(gamepad2.a ? 45.0 : -10.0);
```

### Button-triggered nudge

```java
private boolean lastDpadL, lastDpadR;

boolean dpadL = gamepad2.dpad_left;
boolean dpadR = gamepad2.dpad_right;
if (dpadL && !lastDpadL) bot.turret.nudge(-5.0);
if (dpadR && !lastDpadR) bot.turret.nudge( 5.0);
lastDpadL = dpadL; lastDpadR = dpadR;
```

### Track a vision target angle

```java
if (vision.hasTarget()) {
    double targetDeg = ServoUtility.wrapDegrees180(
            ServoUtility.radToDeg(vision.getTargetYawRad()));
    bot.turret.setAngle(targetDeg);
} else {
    bot.turret.nudge(-gamepad2.left_stick_x * 2.0);
}
```

### Safe-stow on stop

Override `stop()` in the task subsystem:

```java
@Override
public void stop() {
    axis.atMin();   // or axis.setAngle(0.0)
}
```

---

## 11. Migration checklist (transferring to a new robot)

1. Copy `ServoUtility.java` and `ServoAxis.java` verbatim to `utils/`.
2. Copy `ServoTaskSubsystem.java` (and any axis-specific subclass, if you made one) into `SubSystems/`.
3. Copy `ServoConstants.java` into `Configs/roboConfigs/` and update all values.
4. Add raw servo fields + task subsystem fields to your new master class.
5. Resolve servos and construct subsystems in the constructor.
6. `subsystems.add(...)` each one.
7. Copy `ServoTest.java` to `utils/testClasses/`.
8. Run `ServoTest`, calibrate, paste values back.
9. Done.

The only file with robot-specific numbers is `ServoConstants.java`. Everything else is reusable as-is.