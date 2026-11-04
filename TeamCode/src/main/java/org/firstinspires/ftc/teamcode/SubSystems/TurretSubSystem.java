/**
 * EXAMPLE, Turret Setup
 */

package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoAxis;
import org.firstinspires.ftc.teamcode.utils.ServoUtility;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * TURRET — one specific task: aim the shooter horizontally.
 * Owns its servos. Future home of vision tracking / limit sensors.
 *
 * <p>Clip: the turret is limited to {@code CLIP_TOTAL_DEG} of travel centered on zero
 * (270 deg total = +/-135 deg). Every angle command goes through {@link #clipAngle(double)}.
 * The clip is applied on top of the axis range from ServoConstants / the calibration, never
 * past it: if the calibrated range is smaller than the clip on a side, that smaller range wins.
 */
public class TurretSubSystem extends Subsystem {

    /** Total allowed travel, split evenly either side of zero. */
    public static final double CLIP_TOTAL_DEG = 270.0;
    private static final double CLIP_HALF_DEG = CLIP_TOTAL_DEG / 2.0;

    private final ServoAxis axis;

    public TurretSubSystem(Servo[] servos,
                           double minDeg, double maxDeg,
                           double[] atMin, double[] atMax) {
        this.axis = new ServoAxis("turret", servos, minDeg, maxDeg, atMin, atMax);
    }

    // ---- clip ----

    /** Smallest allowed angle: -135 deg, or the axis minimum if that is tighter. */
    public double getClipMin() { return Math.max(-CLIP_HALF_DEG, axis.getMinDeg()); }

    /** Largest allowed angle: +135 deg, or the axis maximum if that is tighter. */
    public double getClipMax() { return Math.min(180, axis.getMaxDeg()); }

    /** Clamp an angle (deg, 0 = zero direction, + = clockwise) into the clipped range. */
    public double clipAngle(double deg) {
        return ServoUtility.clamp(deg, getClipMin(), getClipMax());
    }

    // ---- control (all clipped) ----

    public void   setAngle(double deg) { axis.setAngle(clipAngle(deg)); }
    public void   nudge(double d)      { setAngle(axis.getAngle() + d); }
    public void   atMin()              { setAngle(getClipMin()); }
    public void   atMax()              { setAngle(getClipMax()); }
    public double getAngle()           { return axis.getAngle(); }
    public double getPosition(int i)   { return axis.getPosition(i); }
    public Servo[] getServos()         { return axis.getServos(); }

    /** Exposed for the calibration OpMode. Not needed by normal OpModes. Bypasses the clip. */
    public ServoAxis getAxis()         { return axis; }

    @Override public void setup() { setAngle(getClipMax()); }
    @Override public void stop()   { axis.stop(); }
    @Override public void update() { /* no-op; add auto-aim here later */ }
}
