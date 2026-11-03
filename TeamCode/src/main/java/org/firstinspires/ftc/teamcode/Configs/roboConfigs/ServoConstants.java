/*
SERVO SUBSYSTEM
Angle-based servo control. Command degrees relative to robot front (0 = ahead,
+ = right, - = left). Maps to per-servo positions (0..1), handles mirrored
mounts, clamps to safe limits. Works for any N servos per axis.

FILES
    PracticeBot          wiring
    ServoConstants       EDIT PER ROBOT (limits + calibration)
    ServoUtility         pure static math
    ServoAxis            internal helper (servos on one axis)
    ServoTaskSubsystem   generic task subsystem (owns a ServoAxis) — one instance per axis
                         (turret, pitch, ...), not a separate class per axis
    ServoTest            calibration OpMode

SETUP
    1. In ServoConstants, fill per axis: NAMES[], MIN_DEG, MAX_DEG,
       AT_MIN[], AT_MAX[] (positions 0..1 at those angles).
    2. In PracticeBot, resolve servos + construct a ServoTaskSubsystem per axis.
    3. Run ServoTest, calibrate AT_MIN/AT_MAX, paste back.

CALIBRATION (ServoTest)
    Share axis | A servo | B RAW/ANGLE | Dpad nudge | X/Y limits
    RAW mode: dpad to each safe extreme, read "Raw position",
    that's AT_MIN[i] / AT_MAX[i]. Mirrored servo: swap the two values.

USAGE
    bot.turret.setAngle(45.0);
    bot.turret.nudge(-10.0);
    bot.turretLeft.setPosition(0.42);   // raw, debug only

NEW AXIS
    Add a nested constants class here + raw Servo field(s) in PracticeBot, then
    construct a new ServoTaskSubsystem("yourAxisName", ...) in PracticeBot and
    subsystems.add(...). No new Java class needed — ServoTaskSubsystem is generic.

NOTES
    Open-loop; lastAngle = last commanded. setAngle() clamps.
    stop()/update() are no-ops (subclass ServoTaskSubsystem for auto-aim, etc.).
*/


package org.firstinspires.ftc.teamcode.Configs.roboConfigs;

public final class ServoConstants {

    private ServoConstants() { }

    public static final class Turret {
        public static final String[] NAMES   = { "turretL", "turretR" };
        public static final double   MIN_DEG = -100.0;
        public static final double   MAX_DEG =  100.0;
        public static final double[] AT_MIN  = { 0, 0 };
        public static final double[] AT_MAX  = { 1, 1 };
    }

    public static final class Gate {
        public static final String[] NAMES   = { "gate" };
        public static final double   MIN_DEG = -20.0;
        public static final double   MAX_DEG =  100.0;
        public static final double[] AT_MIN  = { 0.68 };
        public static final double[] AT_MAX  = { 1.0 };
    }

    public static final class Flower {
        public static final String[] NAMES   = { "flower" };
        public static final double   MIN_DEG = -20.0;
        public static final double   MAX_DEG =  60.0;
        public static final double[] AT_MIN  = { 0.34 };
        public static final double[] AT_MAX  = { 0.92 };
    }

    public static final class Pitch {
        public static final String[] NAMES   = { "pitchServo" };
        public static final double   MIN_DEG = -20.0;
        public static final double   MAX_DEG =  60.0;
        public static final double[] AT_MIN  = { 0.10 };
        public static final double[] AT_MAX  = { 0.90 };
    }
}