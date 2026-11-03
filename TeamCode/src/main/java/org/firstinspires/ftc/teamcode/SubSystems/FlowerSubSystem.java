/**
 * EXAMPLE, PITCH ANGLE Setup
 */

package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.ServoAxis;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * PITCH — one specific task: raise/lower the shooter.
 * Owns its servo. Future home of pitch presets / encoder feedback.
 */
public class FlowerSubSystem extends Subsystem {

    private final ServoAxis axis;

    public FlowerSubSystem(Servo[] servos,
                           double minDeg, double maxDeg,
                           double[] atMin, double[] atMax) {
        this.axis = new ServoAxis("flower", servos, minDeg, maxDeg, atMin, atMax);
    }

    public void   setAngle(double deg) { axis.setAngle(deg); }
    public void   nudge(double d)      { axis.nudge(d); }
    public void   atMin()              { axis.atMin(); }
    public void   atMax()              { axis.atMax(); }
    public double getAngle()           { return axis.getAngle(); }
    public double getPosition(int i)   { return axis.getPosition(i); }
    public Servo[] getServos()         { return axis.getServos(); }
    public ServoAxis getAxis()         { return axis; }

    @Override public void setup() { axis.setAngle(axis.getMinDeg()); }
    @Override public void stop()   { axis.stop(); }
    @Override public void update() { /* no-op for now */ }
}