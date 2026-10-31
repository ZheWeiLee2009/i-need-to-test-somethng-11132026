package org.firstinspires.ftc.teamcode.utils;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * Pure static math for servo control. No hardware, no state.
 * Callable from anywhere: OpModes, subsystems, calibration tools, tests.
 */
public final class ServoUtility {

    private ServoUtility() { }

    // ================================================================
    //  CONSTANTS
    // ================================================================
    public static final double SERVO_MIN    = 0.0;
    public static final double SERVO_MAX    = 1.0;
    public static final double SERVO_CENTER = 0.5;
    public static final double DEG_TO_RAD   = Math.PI / 180.0;
    public static final double RAD_TO_DEG   = 180.0 / Math.PI;

    // ================================================================
    //  BASIC MATH
    // ================================================================
    public static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double inverseLerp(double a, double b, double v) {
        return (a == b) ? 0.0 : (v - a) / (b - a);
    }

    public static double mapRange(double v, double inMin, double inMax,
                                  double outMin, double outMax) {
        return lerp(outMin, outMax, inverseLerp(inMin, inMax, v));
    }

    public static double sign(double v) {
        return v > 0 ? 1 : (v < 0 ? -1 : 0);
    }

    public static boolean approxEquals(double a, double b, double eps) {
        return Math.abs(a - b) <= eps;
    }

    // ================================================================
    //  ANGLE HELPERS
    // ================================================================
    public static double degToRad(double d) { return d * DEG_TO_RAD; }
    public static double radToDeg(double r) { return r * RAD_TO_DEG; }

    /** Wrap to [-180, 180). */
    public static double wrapDegrees180(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    /** Wrap to [0, 360). */
    public static double wrapDegrees360(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    /** Shortest signed delta from a to b, result in [-180, 180]. */
    public static double shortestAngleDelta(double from, double to) {
        return wrapDegrees180(to - from);
    }

    /** Circular mean of two angles. */
    public static double meanAngle(double a, double b) {
        double x = Math.cos(degToRad(a)) + Math.cos(degToRad(b));
        double y = Math.sin(degToRad(a)) + Math.sin(degToRad(b));
        return radToDeg(Math.atan2(y, x));
    }

    // ================================================================
    //  RANGE / LIMIT HELPERS
    // ================================================================
    public static double clampAngle(double deg, double minDeg, double maxDeg) {
        return clamp(deg, minDeg, maxDeg);
    }

    /** Shrink a range inward by margin on each side (safe operating range). */
    public static double marginMin(double minDeg, double maxDeg, double marginDeg) {
        return minDeg + Math.min(marginDeg, (maxDeg - minDeg) / 2.0);
    }

    public static double marginMax(double minDeg, double maxDeg, double marginDeg) {
        return maxDeg - Math.min(marginDeg, (maxDeg - minDeg) / 2.0);
    }

    // ================================================================
    //  POSITION TRANSFORMS
    // ================================================================
    /** Flip a position: 0.0 -> 1.0, 1.0 -> 0.0. */
    public static double invertPosition(double pos) {
        return SERVO_MAX - clamp(pos, SERVO_MIN, SERVO_MAX);
    }

    /** Snap to center if inside deadband, else return unchanged. */
    public static double deadbandClamp(double pos, double deadband) {
        return Math.abs(pos - SERVO_CENTER) < deadband ? SERVO_CENTER : pos;
    }

    /** S-curve easing on t in [0, 1]. */
    public static double smoothStep(double t) {
        t = clamp(t, 0.0, 1.0);
        return t * t * (3.0 - 2.0 * t);
    }

    /** Ease in/out with arbitrary exponent. */
    public static double easeInOut(double t, double exponent) {
        t = clamp(t, 0.0, 1.0);
        double a = Math.pow(t, exponent);
        double b = Math.pow(1.0 - t, exponent);
        return a / (a + b);
    }

    // ================================================================
    //  MOTION / SLEW RATE
    // ================================================================
    /** Cap step size between current and target. */
    public static double slewLimit(double current, double target, double maxStep) {
        double d = target - current;
        if (Math.abs(d) <= maxStep) return target;
        return current + sign(d) * maxStep;
    }

    /** Slew limit with units-per-second and a loop dt. */
    public static double slewLimitPerSecond(double current, double target,
                                            double maxRatePerSec, double dtSec) {
        return slewLimit(current, target, maxRatePerSec * dtSec);
    }

    // ================================================================
    //  ANGLE <-> SERVO POSITION
    // ================================================================
    public static double degreesToPosition(double deg,
                                           double minDeg, double maxDeg,
                                           double servoAtMin, double servoAtMax) {
        if (maxDeg == minDeg) return servoAtMin;
        return mapRange(clamp(deg, minDeg, maxDeg), minDeg, maxDeg, servoAtMin, servoAtMax);
    }

    /** Bulk: fill out[] with one position per servo. No allocation. */
    public static void degreesToPositions(double deg,
                                          double minDeg, double maxDeg,
                                          double[] atMin, double[] atMax,
                                          double[] out) {
        int n = out.length;
        if (atMin.length != n || atMax.length != n)
            throw new IllegalArgumentException("array length mismatch");
        double a = clamp(deg, minDeg, maxDeg);
        for (int i = 0; i < n; i++) {
            out[i] = mapRange(a, minDeg, maxDeg, atMin[i], atMax[i]);
        }
    }

    public static double positionToDegrees(double pos,
                                           double minDeg, double maxDeg,
                                           double servoAtMin, double servoAtMax) {
        if (servoAtMin == servoAtMax) return minDeg;
        return mapRange(pos, servoAtMin, servoAtMax, minDeg, maxDeg);
    }

    // ================================================================
    //  PULSE-WIDTH CONVERSIONS  (smart servos only)
    // ================================================================
    public static double positionToPulseMs(double pos, double minMs, double maxMs) {
        return mapRange(clamp(pos, SERVO_MIN, SERVO_MAX),
                SERVO_MIN, SERVO_MAX, minMs, maxMs);
    }

    public static double pulseMsToPosition(double pulseMs, double minMs, double maxMs) {
        return mapRange(pulseMs, minMs, maxMs, SERVO_MIN, SERVO_MAX);
    }

    // ================================================================
    //  VALIDATION
    // ================================================================
    public static boolean isWithinRange(double v, double lo, double hi) {
        return v >= lo && v <= hi;
    }

    public static boolean isValidPosition(double pos) {
        return isWithinRange(pos, SERVO_MIN, SERVO_MAX);
    }

    public static void requireValidRange(double minDeg, double maxDeg) {
        if (minDeg >= maxDeg)
            throw new IllegalArgumentException("minDeg must be < maxDeg");
    }

    public static void requireMatchingLengths(int... lengths) {
        if (lengths.length == 0) return;
        int first = lengths[0];
        for (int i = 1; i < lengths.length; i++) {
            if (lengths[i] != first)
                throw new IllegalArgumentException("array length mismatch");
        }
    }
}