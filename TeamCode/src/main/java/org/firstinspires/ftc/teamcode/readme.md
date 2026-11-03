## TeamCode Module

This is a working robot codebase built around one master class, **`PracticeBot`**, plus a set
of reusable "subsystem" classes for drive, intake, the launcher, and servo-driven mechanisms.
If you're new here, read this file first, then the guide for whatever you're touching.

## Project structure

```
teamcode/
├── PracticeBot.java        Hardware map + subsystem wiring. The "master class" for the robot.
├── OpModes/                Competition-ready OpModes (currently: FOTeleOpDrive).
├── samples/                Pedro Pathing sample OpModes (Autonomous + TeleOp). @Disabled by
│                           default — copy and rename rather than editing in place.
├── SubSystems/             One class per robot mechanism. All extend Subsystem.java.
│   ├── Subsystem.java         Base class: setup()/update()/stop() lifecycle + shared hardware
│   │                          helpers (setMotorsMode, setZeroPowerBehavior).
│   ├── DriveSubSystem.java    Mecanum drive + odometry. See drive.md.
│   ├── LauncherSubSystem.java Flywheel PIDF velocity control. See mechanisms.md.
│   ├── IntakeSubSystem.java   Single on/off roller motor. See mechanisms.md.
│   ├── ServoTaskSubsystem.java Generic single-axis servo mechanism (turret, pitch, ...).
│   │                          See servos.md.
│   ├── drive.md, servos.md, mechanisms.md   Full usage guides for the subsystems above.
├── utils/                  Reusable, robot-agnostic helpers (no per-robot numbers).
│   ├── ServoAxis.java, ServoUtility.java   Servo math — used internally by ServoTaskSubsystem.
│   ├── customMath.java, RayCasting.java    Misc math + field-zone detection.
│   └── testClasses/        Calibration / diagnostic OpModes (DriveTest, ServoTest, MotorTest,
│                            LauncherTest, AutoDriveByRandomDir, panelsTest).
└── Configs/
    ├── roboConfigs/        EDIT PER ROBOT. DriveConstants, LauncherConstants, ServoConstants —
    │                       tuning numbers and calibration, each documented inline.
    └── pedroPathing/       Pedro Pathing 3.0 setup (Constants.java) + AutoTune procedures,
                            used by the OpModes under samples/.
```

**Rule of thumb for where new code goes:** extends `Subsystem` → `SubSystems/`; reusable,
robot-agnostic helper → `utils/`; per-robot number or calibration value → `Configs/roboConfigs/`.

> **Tip — make it yours:** `PracticeBot` is a placeholder name from this template. Once you've
> got things wired up, rename the class (and its file, `PracticeBot.java`) to something specific
> to your robot — e.g. `Team0417Bot`. Use your IDE's rename/refactor tool (not find-and-replace)
> so every reference — the field type in `FOTeleOpDrive`, the `new PracticeBot(...)` calls in
> the `samples/` and `utils/testClasses/` OpModes — updates automatically. Nothing else in this
> codebase depends on the name `PracticeBot` specifically.

## Two drive stacks — read this before wiring motors

This project can drive the robot two different ways: the hand-rolled `DriveSubSystem` (used by
`OpModes/FOTeleOpDrive.java` and the test OpModes) or Pedro Pathing's own follower (used by
`samples/`). They drive the **same four physical motors** under two different hardware-map
naming schemes (`FL/BL/FR/BR` vs `lf/lr/rf/rr`). See §5 of `SubSystems/drive.md` before you wire
up a new drive base or rename a motor — both names need to point at the correct ports.

## Robot configuration (`res/xml`)

This is the one piece of setup that lives **outside** this folder. This readme, and everything
else described above, is under:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
```

The Robot Controller's hardware config — the file that maps a physical port on a Control Hub or
Expansion Hub to a string name — lives in a completely different part of the module:

```
TeamCode/src/main/res/xml/
```

Every `hwMap.get(SomeClass, "someName")` call in this codebase (`PracticeBot.java`,
`Configs/pedroPathing/Constants.java`) only works if a device with that exact, case-sensitive
name exists in whichever config XML is active on the Driver Station. The **file name** (without
`.xml`) is what shows up as the config's name on the Driver Station's Configure Robot screen —
it has nothing to do with any Java class name.

**Names this codebase looks for**, so match these exactly when you build a config:

| Hardware name | Device | Used by |
|---|---|---|
| `FL`, `BL`, `FR`, `BR` | Drive motors | `PracticeBot` → `DriveSubSystem` (`OpModes/FOTeleOpDrive`, `utils/testClasses/*`) |
| `intake` | Motor | `PracticeBot` → `IntakeSubSystem` |
| `fwL`, `fwR` | Motors | `PracticeBot` → `LauncherSubSystem` |
| `pinpoint` | goBILDA Pinpoint odometry computer | `PracticeBot.odo`, and Pedro's `PinpointConstants` in `Configs/pedroPathing/Constants.java` |
| `turretLeft`, `turretRight` | Servos | `ServoConstants.Turret` — only needed if you uncomment the turret block in `PracticeBot` |
| `pitchServo` | Servo | `ServoConstants.Pitch` — only needed if you uncomment the pitch block in `PracticeBot` |
| `lf`, `lr`, `rf`, `rr` | Drive motors | Pedro Pathing's follower (`samples/`) — see the callout below, this is the *same* four motors as `FL/BL/FR/BR` above |

**Building the file:** rather than hand-writing the XML, use a config editor like
[xml.davisluxenberg.com](https://xml.davisluxenberg.com/) — add your Control Hub / Expansion
Hub(s), place each motor/servo/sensor on the port it's actually wired to, and name each one to
match a row in the table above. Export it, then drop the `.xml` file straight into
`TeamCode/src/main/res/xml/`.

Example covering the core robot (drive + intake + launcher, no turret/pitch yet), two hubs:

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Robot type="FirstInspires-FTC">
   <LynxUsbDevice name="Control Hub Portal" serialNumber="(embedded)" parentModuleAddress="173">
      <LynxModule name="Control Hub" port="173">
         <goBILDA5201SeriesMotor name="FL" port="0"/>
         <goBILDA5201SeriesMotor name="BL" port="1"/>
         <goBILDA5201SeriesMotor name="FR" port="2"/>
         <goBILDA5201SeriesMotor name="BR" port="3"/>
         <goBILDAPinpoint name="pinpoint" port="0" bus="1"/>
      </LynxModule>
      <LynxModule name="Expansion Hub 0" port="0">
         <goBILDA5201SeriesMotor name="intake" port="0"/>
         <goBILDA5201SeriesMotor name="fwL" port="1"/>
         <goBILDA5201SeriesMotor name="fwR" port="2"/>
      </LynxModule>
   </LynxUsbDevice>
</Robot>
```

Save this as, e.g., `PracticeBotConfig.xml` — "PracticeBotConfig" is then the name your team
selects as the active config on the Driver Station. Add `<Servo name="turretLeft" .../>` etc. to
the `Control Hub` module once you've uncommented the turret/pitch block in `PracticeBot.java`.

**A config file can't give one physical port two names**, so if you want to run *both*
`DriveSubSystem`-based OpModes and the Pedro Pathing `samples/` OpModes on the same robot, pick
one:
1. Keep two config files (this one, plus e.g. `PedroConfig.xml` with `lf/lr/rf/rr` in place of
   `FL/BL/FR/BR` on the same four ports) and switch the active config from the Driver Station
   depending which OpModes you're running that session.
2. Or edit one side of the code to match the other — e.g. change `frontLeftName.set("lf")` etc.
   in `Configs/pedroPathing/Constants.java` to `"FL"/"BL"/"FR"/"BR"` — so a single config file
   covers both stacks. This is the less error-prone option if your team genuinely uses both.

## Getting started on a new robot

1. Build your hardware config XML under `res/xml/` (see above) and select it as the active
   config on the Driver Station.
2. Read `SubSystems/drive.md`, `SubSystems/servos.md`, and `SubSystems/mechanisms.md`.
3. Run the relevant calibration OpMode from `utils/testClasses/` (`DriveTest`, `ServoTest`,
   `LauncherTest`, `MotorTest`) before trusting a competition OpMode.
4. Build your competition OpModes in `OpModes/`, following the pattern in `FOTeleOpDrive.java`.

## Creating a new OpMode

The easiest way is to copy an existing OpMode and adapt it — `OpModes/FOTeleOpDrive.java` for a
TeleOp built on `DriveSubSystem`, or one of the `samples/` OpModes for Pedro Pathing. Each
OpMode declares its name and visibility at the top:

```java
@TeleOp(name = "My New OpMode", group = ".")
// @Disabled   <- remove/comment this line to make the OpMode selectable on the Driver Station
public class MyNewOpMode extends OpMode {
```

`name` is what shows up in the Driver Station's OpMode list; `group` can be used to sort related
OpModes together. An OpMode marked `@Disabled` won't appear on the list at all — every OpMode
under `samples/` starts this way since they're templates, not robot-ready code.

## Advanced: cloning this module for multiple teams

If your club runs multiple teams that share this code as a starting point but each maintain
their own copy, you can clone the whole `TeamCode` module rather than starting from scratch.
This is an Android Studio / file-system operation, not something done from within this project,
and it's easy to get wrong — back up the whole project first.

1. Using your OS's file tools (Android Studio closed), copy the whole `TeamCode` folder to a
   sibling folder with a new name, e.g. `Team0417`.
2. In the new folder, delete the `TeamCode.iml` file.
3. Rename `src/main/java/org/firstinspires/ftc/teamcode` to a matching lowercase name, e.g.
   `team0417`.
4. In `src/main/AndroidManifest.xml`, change `package="org.firstinspires.ftc.teamcode"` to
   `package="org.firstinspires.ftc.team0417"`.
5. Add `include ':Team0417'` to the project's `settings.gradle`.
6. Reopen Android Studio and use Build → Clean Project to clear out stale references.

Each team's module will then appear alongside the others in Android Studio's module list; select
the module you want from the dropdown before pressing Run.