package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.TurretConstants.AIM_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.TurretConstants.AIM_OFFSET_DEG;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.TurretConstants.PINPOINT_FORWARD_OF_TURRET_MM;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.ServoAxis;
import org.firstinspires.ftc.teamcode.utils.ServoUtility;

/**
 * Turret subsystem. When active and given a target field pose, points the turret at that pose.
 * The target is set explicitly with setTargetPose(); it is not coupled to launcher target selection.
 */
public class TurretSubSystem extends Subsystem {

    /** Physical/software travel limits are additionally restricted by the calibrated ServoAxis. */
    public static final double CLIP_TOTAL_DEG = 270.0;
    private static final double CLIP_HALF_DEG = CLIP_TOTAL_DEG / 2.0;
    private static final double MIN_TARGET_DISTANCE_IN = 0.25;
    private static final double MM_PER_INCH = 25.4;

    private final ServoAxis axis;
    private Pose2D robotPose;
    private Pose2D targetPose;

    private double rawAimAngleDeg = Double.NaN;
    private double aimTargetAngleDeg = Double.NaN;
    private boolean aimTargetClipped = false;

    public TurretSubSystem(Servo[] servos,
                           double minDeg, double maxDeg,
                           double[] atMin, double[] atMax) {
        axis = new ServoAxis("turret", servos, minDeg, maxDeg, atMin, atMax);
    }

    /** The configured axis is asymmetric: 0 degrees faces backward; 180 degrees faces forward. */
    public double getClipMin() { return Math.max(-140, axis.getMinDeg()); }
    public double getClipMax() { return Math.min(190.0, axis.getMaxDeg()); }

    public double clipAngle(double deg) {
        return ServoUtility.clamp(deg, getClipMin(), getClipMax());
    }

    public void setAngle(double deg) { axis.setAngle(clipAngle(deg)); }
    public void nudge(double deltaDeg) { setAngle(axis.getAngle() + deltaDeg); }
    public void atMin() { setAngle(getClipMin()); }
    public void atMax() { setAngle(getClipMax()); }
    public double getAngle() { return axis.getAngle(); }
    public double getPosition(int i) { return axis.getPosition(i); }
    public Servo[] getServos() { return axis.getServos(); }

    /** Set the field-space pose that the turret should point toward. */
    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public Pose2D getTargetPose() { return targetPose; }

    /**
     * Updated by LM1Robot after the drivetrain/Pinpoint update each loop. Must be a FIELD-frame
     * pose (x right, y forward, Pedro heading: CCW from +X) — use DriveSubSystem.getFieldPose(),
     * not raw Pinpoint x/y (which are x-forward / y-left).
     */
    public void setRobotPose(Pose2D robotPose) {
        this.robotPose = robotPose;
    }

    public double getRawAimAngleDeg() { return rawAimAngleDeg; }
    public double getAimTargetAngleDeg() { return aimTargetAngleDeg; }
    public boolean isAimTargetClipped() { return aimTargetClipped; }

    /**
     * Runs only while this subsystem is active. The Pinpoint is 20 mm forward of the turret
     * pivot, so the robot pose is translated backward along robot-forward to obtain the turret
     * pivot's field position before calculating the bearing.
     *
     * <p>Turret angle convention: 0 degrees is backward and 180 degrees is forward.
     */
    @Override
    public void update() {
        if (!isActive() || robotPose == null || targetPose == null) return;

        double headingRad = robotPose.getHeading(AngleUnit.RADIANS);
        double pinX = robotPose.getX(DistanceUnit.INCH);
        double pinY = robotPose.getY(DistanceUnit.INCH);
        if (!Double.isFinite(headingRad) || !Double.isFinite(pinX) || !Double.isFinite(pinY)) return;

        double offsetIn = PINPOINT_FORWARD_OF_TURRET_MM / MM_PER_INCH;
        // Field frame (x right, y forward), Pedro heading (0 = +X, CCW-positive):
        // robot-forward = (cos(heading), sin(heading)).
        // Turret pivot is 20 mm backward from Pinpoint.
        double turretX = pinX - offsetIn * Math.cos(headingRad);
        double turretY = pinY - offsetIn * Math.sin(headingRad);

        double dx = targetPose.getX(DistanceUnit.INCH) - turretX;
        double dy = targetPose.getY(DistanceUnit.INCH) - turretY;
        double distance = Math.hypot(dx, dy);
        if (!Double.isFinite(distance) || distance < MIN_TARGET_DISTANCE_IN) return;

        // Clockwise bearing relative to robot-forward = heading - bearing (both CCW from +X).
        // 180 degrees is turret-forward.
        double relativeBearingClockwiseDeg = Math.toDegrees(headingRad - Math.atan2(dy, dx));
        rawAimAngleDeg = 180.0 + relativeBearingClockwiseDeg + AIM_OFFSET_DEG;
        if (!Double.isFinite(rawAimAngleDeg)) return;

        aimTargetAngleDeg = resolveEquivalentAngle(rawAimAngleDeg);
        aimTargetClipped = Math.abs(ServoUtility.shortestAngleDelta(
                aimTargetAngleDeg, rawAimAngleDeg)) > 0.5;

        double deadband = Math.max(0.0, AIM_DEADBAND_DEG);
        if (Math.abs(ServoUtility.shortestAngleDelta(axis.getAngle(), aimTargetAngleDeg)) > deadband) {
            setAngle(aimTargetAngleDeg);
        }
    }

    private double resolveEquivalentAngle(double requestedDeg) {
        double normalized = ServoUtility.wrapDegrees180(requestedDeg);
        double min = getClipMin();
        double max = getClipMax();

        double best = ServoUtility.clamp(normalized, min, max);
        double bestError = Math.abs(normalized - best);

        double plusTurn = normalized + 360.0;
        double plusClipped = ServoUtility.clamp(plusTurn, min, max);
        double plusError = Math.abs(plusTurn - plusClipped);
        if (plusError < bestError) {
            best = plusClipped;
            bestError = plusError;
        }

        double minusTurn = normalized - 360.0;
        double minusClipped = ServoUtility.clamp(minusTurn, min, max);
        double minusError = Math.abs(minusTurn - minusClipped);
        if (minusError < bestError) best = minusClipped;
        return best;
    }

    /** Exposed for the calibration OpMode. */
    public ServoAxis getAxis() { return axis; }

    @Override
    public void setup() {
        setActive(false);
        setAngle(0.0); // turret's forward-facing position
    }

    @Override
    public void stop() {
        setActive(false);
        axis.stop(); // positional servos hold their last commanded position
    }
}
