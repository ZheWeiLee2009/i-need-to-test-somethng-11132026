package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * Internal helper. Owns a group of servos that move together on one axis
 * and maps angles (deg) -> servo positions (0..1). NOT a Subsystem.
 * Task subsystems compose this; they do not extend it.
 */
public class ServoAxis {

    private final String   name;
    private final double   minDeg, maxDeg;
    private final double[] servoAtMin, servoAtMax;
    private final Servo[]  servos;
    private final double[] posBuffer;
    private double lastAngle;

    public ServoAxis(String name, Servo[] servos,
                     double minDeg, double maxDeg,
                     double[] servoAtMin, double[] servoAtMax) {
        if (minDeg >= maxDeg) throw new IllegalArgumentException(name + ": minDeg must be < maxDeg");
        if (servos == null || servoAtMin == null || servoAtMax == null)
            throw new IllegalArgumentException(name + ": null config");
        int n = servos.length;
        if (n == 0 || servoAtMin.length != n || servoAtMax.length != n)
            throw new IllegalArgumentException(name + ": length mismatch");
        for (int i = 0; i < n; i++)
            if (servos[i] == null) throw new IllegalArgumentException(name + ": null servo at " + i);

        this.name       = name;
        this.servos     = servos.clone();
        this.minDeg     = minDeg;
        this.maxDeg     = maxDeg;
        this.servoAtMin = servoAtMin.clone();
        this.servoAtMax = servoAtMax.clone();
        this.posBuffer  = new double[n];
        this.lastAngle  = minDeg;
    }

    public void setAngle(double deg) {
        double a = ServoUtility.clampAngle(deg, minDeg, maxDeg);
        ServoUtility.degreesToPositions(a, minDeg, maxDeg, servoAtMin, servoAtMax, posBuffer);
        for (int i = 0; i < servos.length; i++) servos[i].setPosition(posBuffer[i]);
        lastAngle = a;
    }

    public void nudge(double d)  { setAngle(lastAngle + d); }
    public void atMin()          { setAngle(minDeg); }
    public void atMax()          { setAngle(maxDeg); }
    public void stop()           { /* servos hold */ }

    public double   getAngle()         { return lastAngle; }
    public double   getPosition(int i) { return ServoUtility.degreesToPosition(
            lastAngle, minDeg, maxDeg, servoAtMin[i], servoAtMax[i]); }
    public double   getMinDeg()        { return minDeg; }
    public double   getMaxDeg()        { return maxDeg; }
    public String   getName()          { return name; }
    public int      getServoCount()    { return servos.length; }
    public Servo[] getServos() { return servos.clone(); }
}