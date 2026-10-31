package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoAxis;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * Generic single-axis servo task subsystem — one specific task: move a group of servos
 * that share one degree of freedom (turret, pitch, wrist, claw angle, etc.) between
 * calibrated angle limits.
 *
 * <p>{@code TurretSubSystem} and {@code PitchSubSystem} used to be two separate,
 * byte-for-byte identical classes that each just wrapped a {@link ServoAxis}. This class
 * replaces both: a new axis is a new instance with its own {@code name} and
 * {@code ServoConstants} block, not a new Java file.
 *
 * <p>Owns its servos via {@link ServoAxis}. Future home of axis-specific behavior
 * (vision tracking, limit sensors, presets) — subclass if one axis needs logic the
 * others don't.
 *
 * <p>See {@code SubSystems/servos.md} for the full setup and usage guide.
 */
public class ServoTaskSubsystem extends Subsystem {

    private final ServoAxis axis;

    /**
     * @param name    Label for this axis (e.g. "turret", "pitch"). Used in logging only.
     * @param servos  Servos that move together on this axis.
     * @param minDeg  Minimum safe angle, degrees.
     * @param maxDeg  Maximum safe angle, degrees.
     * @param atMin   Per-servo raw position (0..1) at minDeg. Same order as servos.
     * @param atMax   Per-servo raw position (0..1) at maxDeg. Same order as servos.
     */
    public ServoTaskSubsystem(String name, Servo[] servos,
                               double minDeg, double maxDeg,
                               double[] atMin, double[] atMax) {
        this.axis = new ServoAxis(name, servos, minDeg, maxDeg, atMin, atMax);
    }

    public void   setAngle(double deg) { axis.setAngle(deg); }
    public void   nudge(double d)      { axis.nudge(d); }
    public void   atMin()              { axis.atMin(); }
    public void   atMax()              { axis.atMax(); }
    public double getAngle()           { return axis.getAngle(); }
    public double getPosition(int i)   { return axis.getPosition(i); }
    public Servo[] getServos()         { return axis.getServos(); }

    /** Exposed for the calibration OpMode. Not needed by normal OpModes. */
    public ServoAxis getAxis()         { return axis; }

    @Override public void stop()   { axis.stop(); }
    @Override public void update() { /* no-op; override in a subclass for auto-aim, etc. */ }
}
