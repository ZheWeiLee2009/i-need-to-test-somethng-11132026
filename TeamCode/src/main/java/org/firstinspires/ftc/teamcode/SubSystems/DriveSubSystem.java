package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.BL_Offset;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.BR_Offset;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.FL_Offset;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.FR_Offset;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.MAX_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.VELOCITY_CUTOFF;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.instantStopConstant;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * DRIVE — owns the four drive motors and the odometry computer, and turns joystick /
 * autonomous input into wheel powers.
 *
 * <p>See {@code SubSystems/drive.md} for the full guide, including which of the drive
 * methods below to use and when.
 */
@Configurable
public class DriveSubSystem extends Subsystem {

    private final List<DcMotorEx> driveMotors;
    private final DcMotorEx fl, bl, fr, br;

    private final GoBildaPinpointDriver odo;

    private final double[] lastPowers = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private static final double POWER_EPSILON = 1.0 / 1000.0;

    private final PIDFController headingPID = new PIDFController(0.7, 0.0, 0.004, 0.0);

    public DriveSubSystem(List<DcMotorEx> driveMotors, GoBildaPinpointDriver odo) {
        this.driveMotors = driveMotors;
        this.fl = driveMotors.get(0);
        this.bl = driveMotors.get(1);
        this.fr = driveMotors.get(2);
        this.br = driveMotors.get(3);
        this.odo = odo;

        // Hardware is configured in setup(), not here — PracticeBot calls setup() on every
        // subsystem right after construction, so doing it twice would just be dead code.
    }

    /**
     * Centralized helper method to apply power with precision checks and caching.
     * Prevents floating-point errors and redundant hardware writes across all drive types.
     * @param motorIndex Index of the motor in the tracking array (0-3)
     * @param motor      The physical DcMotorEx instance
     * @param targetPower The desired power level calculated for the motor
     */
    private void applyOptimizedPower(int motorIndex, DcMotorEx motor, double targetPower) {
        if (!(Math.abs(targetPower - lastPowers[motorIndex]) < POWER_EPSILON)) {
            motor.setPower(targetPower);
            lastPowers[motorIndex] = targetPower;
        }
    }

    // ==========================================================
    // SHARED MATH — single source of truth for the mecanum mix and the
    // field<->robot rotation, so every drive method below (and the deprecated
    // calculate*MotorPowers helpers) computes the same numbers the same way.
    // ==========================================================

    /**
     * The mecanum-drive mixing formula. Combines forward/strafe/turn into raw,
     * per-wheel power (before offsets or speed scaling).
     *
     * @return Array of raw motor powers in order:
     * [0] Front Left, [1] Back Left, [2] Front Right, [3] Back Right
     */
    private static double[] mecanumMix(double axial, double lateral, double yaw) {
        return new double[] {
                axial + lateral + yaw, // FL
                axial - lateral + yaw, // BL
                axial - lateral - yaw, // FR
                axial + lateral - yaw  // BR
        };
    }

    /**
     * Rotates the 2D vector (x, y) by -heading (radians). Used to convert between
     * field-relative and robot-relative values — both field-relative joystick input
     * (heading = robot heading) and a field-relative velocity vector (for the
     * instant-stop brake) go through this same rotation.
     */
    private static double[] rotateByHeading(double x, double y, double heading) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        return new double[] { x * cos - y * sin, x * sin + y * cos };
    }

    /** Mixes + scales + applies power to all four drive motors in one call. */
    private void driveMecanum(double axial, double lateral, double yaw, double speedMultiplier) {
        double[] p = mecanumMix(axial, lateral, yaw);
        applyOptimizedPower(0, fl, p[0] * speedMultiplier * FL_Offset);
        applyOptimizedPower(1, bl, p[1] * speedMultiplier * BL_Offset);
        applyOptimizedPower(2, fr, p[2] * speedMultiplier * FR_Offset);
        applyOptimizedPower(3, br, p[3] * speedMultiplier * BR_Offset);
    }

    /** Mixes + clips (instead of scales) + applies power — used by the instant-stop brake. */
    private void driveMecanumClipped(double axial, double lateral, double max) {
        double[] p = mecanumMix(axial, lateral, 0);
        applyOptimizedPower(0, fl, clip(p[0], max) * FL_Offset);
        applyOptimizedPower(1, bl, clip(p[1], max) * BL_Offset);
        applyOptimizedPower(2, fr, clip(p[2], max) * FR_Offset);
        applyOptimizedPower(3, br, clip(p[3], max) * BR_Offset);
    }

    /**
     * Sets each drive motor's power directly and independently (no mecanum mixing, no offsets).
     * Goes through the same write-caching as the other drive methods, so repeatedly calling this
     * with the same values (e.g. holding a power for a timed Autonomous state) costs no extra
     * hardware writes after the first call.
     *
     * @param fl Front Left power
     * @param bl Back Left power
     * @param fr Front Right power
     * @param br Back Right power
     */
    public void setIndividualPowers(double fl, double bl, double fr, double br) {
        applyOptimizedPower(0, this.fl, fl);
        applyOptimizedPower(1, this.bl, bl);
        applyOptimizedPower(2, this.fr, fr);
        applyOptimizedPower(3, this.br, br);
    }

    /**
     * Sets standard robot-centric drive power.
     * @param axial           Forward/backward input
     * @param lateral         Left/right strafe input
     * @param yaw             Rotation input
     * @param speedMultiplier Global scaling factor
     */
    public void setSTDDrive(double axial, double lateral, double yaw, double speedMultiplier) {
        driveMecanum(axial, lateral, yaw, speedMultiplier);
    }

    /**
     * Sets Field-Oriented Mecanum Drive power.
     * @param axial           Forward/backward field input (+forward, -backward)
     * @param lateral         Left/right field input (+right, -left)
     * @param yaw             Rotation input (+clockwise, -counterclockwise)
     * @param heading         Robot current heading in RADIANS (from IMU / Pinpoint)
     * @param speedMultiplier Global scaling factor
     */
    public void setFODDrive(double axial, double lateral, double yaw, double heading, double speedMultiplier) {
        double[] rot = rotateByHeading(lateral, axial, heading);
        double rotX = rot[0], rotY = rot[1];
        driveMecanum(rotY, rotX, yaw, speedMultiplier);
    }


    /**
     * Sets robot-centric drive power, with an instant-stop reverse-braking
     * routine applied when the driver releases the stick.
     * @param axial           Forward/backward input
     * @param lateral         Left/right strafe input
     * @param yaw             Rotation input
     * @param speedMultiplier Global scaling factor
     */
    public void instantStopDrive(double axial, double lateral, double yaw, double speedMultiplier){
        double stickMag = Math.hypot(axial, lateral);

        if (stickMag < 0.1) {
            double vx = odo.getVelX(DistanceUnit.INCH);
            double vy = odo.getVelY(DistanceUnit.INCH);
            double heading = odo.getHeading(AngleUnit.RADIANS);

            applyBrakeOrSpin(vx, vy, heading, yaw, speedMultiplier);
        } else {
            driveMecanum(axial, lateral, yaw, speedMultiplier);
        }
    }

    /**
     * Sets Field-Oriented Mecanum Drive power, with an instant-stop
     * reverse-braking routine applied when the driver releases the stick.
     * @param axial           Forward/backward field input (+forward, -backward)
     * @param lateral         Left/right field input (+right, -left)
     * @param yaw             Rotation input (+clockwise, -counterclockwise)
     * @param heading         Robot current heading in RADIANS (from IMU / Pinpoint)
     * @param speedMultiplier Global scaling factor
     */
    public void instantStopFODDrive(double axial, double lateral, double yaw, double heading, double speedMultiplier) {
        double stickMag = Math.hypot(axial, lateral);

        if (stickMag < 0.1) {
            double vx = odo.getVelX(DistanceUnit.INCH);
            double vy = odo.getVelY(DistanceUnit.INCH);

            applyBrakeOrSpin(vx, vy, heading, yaw, speedMultiplier);
        } else {
            double[] rot = rotateByHeading(lateral, axial, heading);
            double rotX = rot[0], rotY = rot[1];
            driveMecanum(rotY, rotX, yaw, speedMultiplier);
        }
    }

    /**
     * Shared "stick released" logic for both instant-stop drive methods: brakes hard against
     * the robot's current field-relative velocity if it's still moving fast and the driver is
     * asking to turn, otherwise just spins in place (or sits idle) on yaw alone.
     */
    private void applyBrakeOrSpin(double vx, double vy, double heading, double yaw, double speedMultiplier) {
        double[] robotVel = rotateByHeading(vx, vy, heading);
        double robotAxial = robotVel[0], robotLateral = robotVel[1];
        double speed = Math.hypot(robotAxial, robotLateral);

        if (speed > VELOCITY_CUTOFF && yaw > .01) {
            double brakeAxial   = -robotAxial   * instantStopConstant;
            double brakeLateral = -robotLateral * instantStopConstant;
            driveMecanumClipped(brakeAxial, brakeLateral, MAX_REVERSE_POWER);
        } else {
            driveMecanum(0, 0, yaw, speedMultiplier);
        }
    }

    private double clip(double val, double max) {
        return Math.max(-max, Math.min(max, val));
    }

    /**
     * Calculates robot-centric mecanum motor powers.
     *
     * @param axial   Forward/backward input (+forward, -backward)
     * @param lateral Left/right strafe input (+right, -left)
     * @param yaw     Rotation input (+clockwise, -counterclockwise)
     * @return Array of motor powers in order:
     * [0] Front Left, [1] Back Left, [2] Front Right, [3] Back Right
     */
    @Deprecated
    public double[] calculateMotorPowers(double axial, double lateral, double yaw) {
        return mecanumMix(axial, lateral, yaw);
    }

    /**
     * Calculates field-oriented mecanum motor powers.
     * Converts field-centric joystick inputs into robot-centric motion
     * using the robot's current heading.
     *
     * @param axial   Forward/backward field input (+forward, -backward)
     * @param lateral Left/right field input (+right, -left)
     * @param yaw     Rotation input (+clockwise, -counterclockwise)
     * @param heading Robot current heading in RADIANS (from IMU)
     * @return Array of motor powers in order:
     * [0] Front Left, [1] Back Left, [2] Front Right, [3] Back Right
     */
    @Deprecated
    public double[] calculateFODMotorPowers(double axial, double lateral, double yaw, double heading) {
        double[] rot = rotateByHeading(lateral, axial, heading);
        double rotX = rot[0], rotY = rot[1];
        return mecanumMix(rotY, rotX, yaw);
    }

    /**
     * Sets power to all drive motors using precomputed motor velocities.
     * Expected motor order in list:
     * [0] Front Left
     * [1] Back Left
     * [2] Front Right
     * [3] Back Right
     * @param power Array of motor powers (length must be 4)
     * @param speedMultiplier Global speed scaling factor
     */
    @Deprecated
    public void setDriveMotorPowers(@NonNull double[] power, double speedMultiplier) {
        applyOptimizedPower(0, fl, power[0] * speedMultiplier * FL_Offset);
        applyOptimizedPower(1, bl, power[1] * speedMultiplier * BL_Offset);
        applyOptimizedPower(2, fr, power[2] * speedMultiplier * FR_Offset);
        applyOptimizedPower(3, br, power[3] * speedMultiplier * BR_Offset);
    }

    /**
     * Getting theta of pose difference.
     *
     * @param target Target's Pose
     * @return Angle of difference
     */
    private double headingToTarget(Pose2d target) {
        double dx = target.getX() - odo.getPosX(DistanceUnit.INCH);
        double dy = target.getX() - odo.getPosY(DistanceUnit.INCH);

        Vector2d lineToTarget = new Vector2d(dx, dy);
        double error = lineToTarget.angle() - odo.getHeading(AngleUnit.RADIANS);

        if (error > Math.PI)  error -= 2 * Math.PI;
        if (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }

    /**
     * PIDF setting for drive autoaim.
     *
     * @param kP P
     * @param kI I
     * @param kD D
     * @param kF F
     */
    public void setHeadingPIDF(double kP, double kI,double kD,double kF){
        headingPID.setPIDF(kP,kI,kD,kF);
    }

    /**
     * auto aiming control speed.
     * @param MAX_TURN_VELOCITY max rx
     * @param target Target's Pose
     * @return rx replacement, now it will auto-aim.
     */
    public double headingControl(double MAX_TURN_VELOCITY, Pose2d target){
        double output = headingPID.calculate(headingToTarget(target),0);
        return Math.max(-MAX_TURN_VELOCITY, Math.min(MAX_TURN_VELOCITY, output));
    }

    /**
     * Stops all drive motors and resets the internal power optimization cache.
     */
    @Override
    public void stop() {
        lastPowers[0] = 0.0;
        lastPowers[1] = 0.0;
        lastPowers[2] = 0.0;
        lastPowers[3] = 0.0;

        for (DcMotorEx motor : driveMotors) {
            motor.setPower(0);
        }
    }

    @Override
    public void update(){
        odo.update();
    }

    @Override
    public void setup() {
        setZeroPowerBehavior(driveMotors, DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorsMode(driveMotors, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
}
