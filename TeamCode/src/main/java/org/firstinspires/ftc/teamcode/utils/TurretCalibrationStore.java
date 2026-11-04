package org.firstinspires.ftc.teamcode.utils;

import java.util.Map;

/**
 * Hands the turret calibration from one OpMode to the next through the SDK blackboard
 * (the shared map every OpMode can see; pass it in as {@code blackboard}).
 *
 * <p>The blackboard lasts only while the Robot Controller app keeps running. It is cleared by a
 * reboot, a code download/redeploy, or a robot reset. {@code ServoConstants.Turret} is the
 * permanent record; this is the carry-over between OpModes in between.
 *
 * <p>Stored value: a {@code double[]} of { minDeg, maxDeg, atMin, atMax } for the reference
 * servo; both turret servos share it. Always read and written as double[] (the blackboard does
 * not convert types).
 */
public final class TurretCalibrationStore {

    private TurretCalibrationStore() { }

    public static final String KEY = "turretCalibration";

    /** Publish a calibration. */
    public static void put(Map<String, Object> blackboard, double minDeg, double maxDeg,
                           double atMin, double atMax) {
        if (blackboard == null) return;
        blackboard.put(KEY, new double[]{ minDeg, maxDeg, atMin, atMax });
    }

    /** Remove any published calibration (the robot then falls back to ServoConstants). */
    public static void clear(Map<String, Object> blackboard) {
        if (blackboard != null) blackboard.remove(KEY);
    }

    /**
     * @return a copy of { minDeg, maxDeg, atMin, atMax }, or null if nothing usable is published
     *         (missing, wrong type, not finite, min >= max, or positions outside 0..1).
     */
    public static double[] get(Map<String, Object> blackboard) {
        if (blackboard == null) return null;
        Object o = blackboard.get(KEY);
        if (!(o instanceof double[])) return null;
        double[] v = ((double[]) o).clone();
        if (v.length != 4) return null;
        for (double d : v) if (Double.isNaN(d) || Double.isInfinite(d)) return null;
        if (v[0] >= v[1]) return null;
        if (v[2] < 0 || v[2] > 1 || v[3] < 0 || v[3] > 1) return null;
        return v;
    }

    /** n copies of v, for the per-servo AT_MIN / AT_MAX arrays. */
    public static double[] fill(double v, int n) {
        double[] a = new double[n];
        java.util.Arrays.fill(a, v);
        return a;
    }
}
