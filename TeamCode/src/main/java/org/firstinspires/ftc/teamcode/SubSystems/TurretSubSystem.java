/**
 * EXAMPLE, Turret Setup
 */

package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoAxis;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * TURRET — one specific task: aim the shooter horizontally.
 * Owns its servos. Future home of vision tracking / limit sensors.
 */
public class TurretSubSystem extends Subsystem {

    private final ServoAxis axis;

    public TurretSubSystem(Servo[] servos,
                           double minDeg, double maxDeg,
                           double[] atMin, double[] atMax) {
        this.axis = new ServoAxis("turret", servos, minDeg, maxDeg, atMin, atMax);
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
    @Override public void update() { /* no-op; add auto-aim here later */ }
}