> Do not modify this in any repos other than this one without knowing how to deal with git-merge.
# Drive Subsystem — Complete Usage Guide

Mecanum drive control for FTC robots: robot-centric and field-oriented driving, an optional
instant-stop brake, and Pinpoint odometry updates. This file covers `DriveSubSystem` — for
Pedro Pathing's own follower-based driving (used in `samples/`), see the Pedro Pathing docs
linked from `Configs/pedroPathing/Constants.java` instead. Both drive the same physical robot;
most teams pick one and stick with it (see §5).

---

## 1. Architecture at a glance

```
┌─────────────────────────────────────────────────────────────┐
│  OpMode                                                     │
│    bot.drive.setFODDrive(y, x, rx, heading, 1.0);            │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  DriveSubSystem  (extends Subsystem)                         │
│    owns 4 DcMotorEx + the GoBildaPinpointDriver (odo)         │
│    mecanumMix() / rotateByHeading() — single source of truth  │
│    for the mixing math, shared by every public drive method   │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  GoBildaPinpointDriver (odo)                                 │
│    field position + velocity, used for heading and braking    │
└─────────────────────────────────────────────────────────────┘

DriveConstants (Configs) ── read by DriveSubSystem
```

`DriveSubSystem.update()` just calls `odo.update()` — it must run every loop (via
`bot.update()`) or the pose/velocity data used for field-oriented drive and braking goes stale.
`bot.drive.setActive(false)` pauses only this background odometry refresh; it does **not** disable
manual drive-motor commands. This is useful for robot-centric driving, which does not need fresh
odometry. The subsystem is activated by `setup()`; an OpMode can manually pause or resume updates
with `bot.drive.setActive(false/true)`.

---

## 2. Which drive method to use

All four public drive methods take the same core joystick inputs (`axial`, `lateral`, `yaw`)
and a `speedMultiplier`, and all four end up calling the same internal mecanum-mix math. The
difference is what they do with heading and with a released stick.

| Method | Field- or robot-centric | Needs `heading`? | Brakes on stick release? | Typical use |
|---|---|---|---|---|
| `setSTDDrive` | Robot-centric | No | No | Simplest driving; forward is always "robot forward". Good for a first working TeleOp or for Autonomous state machines. |
| `setFODDrive` | Field-centric | Yes | No | Forward is always "away from the driver", regardless of robot orientation. Most teams' default TeleOp. |
| `instantStopDrive` | Robot-centric | No (reads odo internally) | Yes | Like `setSTDDrive`, but snaps to a stop instead of coasting when the driver lets go and starts turning. |
| `instantStopFODDrive` | Field-centric | Yes | Yes | `setFODDrive` + the same instant-stop brake. |

**Picking one:** start with `setFODDrive` unless you have a specific reason to want
robot-centric driving. Only reach for the `instantStop*` variants if drivers are complaining
about the robot coasting/drifting when they let go of the stick to turn in place — the brake
adds real complexity (see §4) for a fairly narrow benefit.

`setIndividualPowers(fl, bl, fr, br)` bypasses mixing entirely — direct per-wheel power, useful
for Autonomous state machines that already know the wheel powers they want (see
`AutoDriveByRandomDir`).

`calculateMotorPowers` / `calculateFODMotorPowers` / `setDriveMotorPowers` are `@Deprecated`.
They predate the methods above and are kept only so old OpModes that reference them still
compile — don't call them in new code.

---

## 3. Coordinate conventions

- **axial**: forward (+) / backward (-)
- **lateral**: right (+) / left (-)
- **yaw**: clockwise (+) / counterclockwise (-)
- **heading**: radians, from `bot.odo.getHeading(AngleUnit.RADIANS)`. 0 = whatever direction the
  robot was facing at the last `resetPosAndIMU()` / `setPosition()` call. CCW-positive.

### Field frame vs. raw Pinpoint frame

The two coordinate systems in this project do **not** agree:

| | +X | +Y | Heading |
|---|---|---|---|
| **Field frame** (targets, ray-cast zones, turret/launcher math, start poses; same x/y orientation as PedroPathing) | right | forward | CCW, 0 = facing +Y |
| **goBILDA Pinpoint, raw** (`odo.getPosX/getPosY/getVelX/getVelY`) | forward | left | CCW, 0 = facing +X |

Same origin, axes rotated 90°: `field.x = -pinpoint.y`, `field.y = pinpoint.x`; heading is identical
(both mean "CCW from the way the robot faced at reset").

**Rule: never read raw `odo.getPosX()/getPosY()/getVelX()/getVelY()` in field-frame code.** Use
`DriveSubSystem`'s converters instead:

| Method | Returns |
|---|---|
| `getFieldX()` / `getFieldY()` | position, inches, field frame |
| `getFieldVelX()` / `getFieldVelY()` | velocity, in/s, field frame |
| `getHeadingRad()` | heading, radians |
| `getFieldPose()` | `Pose2D` in the field frame |
| `setFieldPose(Pose2D)` | seeds the Pinpoint from a field-frame pose (converts axes for you) |

The lower-level helpers live in `utils/customMath.java` (`pinpointToField`, `fieldToPinpoint`,
`convertPose2D`/`convertPose` for PedroPathing `Pose`s — Pedro heading is measured from +X, so it is
field heading + π/2).

Pinpoint *configuration* (`setOffsets`, `setEncoderDirections`, `PinpointConfig` pod offsets) stays in
the Pinpoint's own frame: x pod = forward-measuring pod, y pod = strafe-measuring pod, and offsets are
x-pod-sideways / y-pod-forward as goBILDA defines them. Do not convert those.

Gamepad sticks read **positive down and right**, so the standard mapping into `setFODDrive` is:

```java
double y  = -gamepad1.left_stick_y;   // up on the stick = positive axial
double x  =  gamepad1.left_stick_x;
double rx = -gamepad1.right_stick_x;
bot.drive.setFODDrive(y, x, rx, bot.odo.getHeading(AngleUnit.RADIANS), 1.0);
```

(This is exactly what `FOTeleOpDrive` does.)

---

## 4. How the instant-stop brake works

`instantStopDrive` / `instantStopFODDrive` only touch the odometry-based braking logic when the
stick is nearly centered (`Math.hypot(axial, lateral) < 0.1`) — otherwise they drive normally.
When the stick is centered:

1. Read the robot's current field-relative velocity (`getFieldVelX/Y`, already converted out of
   the Pinpoint's x-forward / y-left axes) and rotate it into robot-relative
   `(robotAxial, robotLateral)`.
2. If robot speed is **above** `VELOCITY_CUTOFF` **and** the driver is commanding yaw
   (`yaw > .01`): command a reverse power proportional to the current velocity
   (`-velocity * instantStopConstant`), clipped to `MAX_REVERSE_POWER`, to kill translational
   motion fast so the turn-in-place isn't fighting momentum.
3. Otherwise: just spin in place on `yaw` alone (or sit idle if `yaw` is also ~0).

All three `DriveConstants` involved (`instantStopConstant`, `VELOCITY_CUTOFF`,
`MAX_REVERSE_POWER`) are documented inline in `DriveConstants.java` — start there if the brake
feels too weak, too aggressive, or triggers when it shouldn't.

**Why it only brakes when turning:** braking on every stick release would fight a driver trying
to coast in a straight line. Gating on `yaw > .01` limits the brake to the "stop translating,
about to spin" case it was designed for.

---

## 5. The two drive stacks in this project — don't mix wiring

This project has **two independent ways to drive the robot**, and they name the same four
physical motors differently:

| | `DriveSubSystem` (this file) | Pedro Pathing follower (`samples/`) |
|---|---|---|
| Motor names | `FL`, `BL`, `FR`, `BR` (see `PracticeBot`) | `lf`, `lr`, `rf`, `rr` (see `Configs/pedroPathing/Constants.java`) |
| Odometry | `GoBildaPinpointDriver` wired directly (`bot.odo`) | `PinpointLocalizer` wrapping the same Pinpoint board |
| Used by | `FOTeleOpDrive`, `DriveTest`, `AutoDriveByRandomDir`, `MotorTest` | `PedroTeleOpSample`, `PedroAutoSample`, `PedroCommands` |

Both stacks drive the *same physical motors* — they just look them up under different
hardware-map names. **Your robot's hardware configuration file needs both sets of names
pointing at the correct physical ports.** If you rewire a drive motor or rename it in the
config, update both `PracticeBot.java` and `Configs/pedroPathing/Constants.java` — there's a
comment next to each motor-name block pointing at the other one as a reminder.

If your team only uses one of the two stacks, it's worth deleting the OpModes for the other one
(or at least leaving them `@Disabled`, as they already are) so nobody accidentally builds a
robot config that only wires up one naming scheme.

---

## 6. Common pitfalls

| Pitfall | Fix |
|---|---|
| Robot drives but odometry/heading never updates | Confirm `bot.drive.isActive()` is true — `setActive(false)` intentionally pauses only background odometry updates. |
| `instantStopDrive`'s brake never fires | Check `VELOCITY_CUTOFF` — if it's set higher than the robot's actual driving speed, `speed > VELOCITY_CUTOFF` is never true. Also confirm you're actually releasing the stick below the `0.1` deadzone. |
| Robot spins the wrong way from a joystick nudge | A drive motor's direction or `*_Offset` is flipped from what you expect — check `PracticeBot`'s `setDirection(...)` calls and the `DriveConstants` offsets together, not just one or the other. |
| Field-oriented drive "forward" drifts over a match | Odometry heading drift — re-run `bot.odo.resetPosAndIMU()` at a known heading before each match, and press the reset button mapped in your OpMode (`gamepad1.y` in `FOTeleOpDrive`) if it drifts mid-match. |
| Mixing `DriveSubSystem` and the Pedro follower in the same OpMode | Don't — they'll fight over the same motors. Pick one stack per OpMode. |
| Editing the mecanum math in one drive method but not another | Don't — `setSTDDrive`, `setFODDrive`, and both `instantStop*` methods all call the shared private `mecanumMix()` / `rotateByHeading()` helpers. Change the math there once, not per-method. |

---

## 7. API reference

| Method | Notes |
|---|---|
| `setSTDDrive(axial, lateral, yaw, speedMultiplier)` | Robot-centric. |
| `setFODDrive(axial, lateral, yaw, heading, speedMultiplier)` | Field-centric. |
| `instantStopDrive(axial, lateral, yaw, speedMultiplier)` | Robot-centric + brake; reads heading from `odo` internally. |
| `instantStopFODDrive(axial, lateral, yaw, heading, speedMultiplier)` | Field-centric + brake. |
| `setIndividualPowers(fl, bl, fr, br)` | No mixing, no offsets — raw per-wheel power. |
| `setActive(boolean)` | Enables/disables periodic odometry updates. Disabling does not block drive-motor commands. |
| `stop()` | Zeroes all four motors and resets the write-cache. |
| `update()` | Ticks `odo.update()`. Call once per loop via `bot.update()` while active. |
| `setup()` | Configures motor mode + zero-power behavior. Called automatically by `PracticeBot`. |

`calculateMotorPowers`, `calculateFODMotorPowers`, and `setDriveMotorPowers` are deprecated —
present for backward compatibility only.
