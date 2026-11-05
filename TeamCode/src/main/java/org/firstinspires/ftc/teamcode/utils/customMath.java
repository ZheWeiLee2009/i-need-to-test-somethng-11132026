package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */
/** Computing utility: unit conversions, hardware constants, pose conversions, joystick and projectile math. */
public class customMath {

    // ---------- Gravity ----------
    public static final double G_IN_PER_S2 = 386.09;
    public static final double G_M_PER_S2  = 9.81;
    public static final double G_FT_PER_S2 = 32.174;

    // ---------- Distance ----------
    public static final double IN_PER_FT = 12.0;
    public static final double M_PER_IN  = 0.0254;
    public static final double CM_PER_IN = 2.54;
    public static final double MM_PER_IN = 25.4;
    public static final double M_PER_FT  = 0.3048;
    public static final double M_PER_CM  = 0.01;
    public static final double M_PER_MM  = 0.001;
    public static final double CM_PER_M  = 100.0;
    public static final double MM_PER_M  = 1000.0;
    public static final double IN_PER_M  = 1.0 / M_PER_IN;
    public static final double FT_PER_M  = 1.0 / M_PER_FT;

    // ---------- Area / volume / mass ----------
    public static final double IN2_PER_FT2 = 144.0;
    public static final double M2_PER_FT2  = 0.09290304;
    public static final double IN3_PER_FT3 = 1728.0;
    public static final double M3_PER_FT3  = 0.028316846592;
    public static final double KG_PER_LB   = 0.45359237;

    // ---------- Time ----------
    public static final double MS_PER_S   = 1000.0;
    public static final double S_PER_MIN  = 60.0;
    public static final double MIN_PER_HR = 60.0;
    public static final double S_PER_HR   = 3600.0;

    // ---------- Angles ----------
    public static final double DEG_PER_RAD = 180.0 / Math.PI;
    public static final double RAD_PER_DEG = Math.PI / 180.0;
    public static final double DEG_PER_REV = 360.0;
    public static final double RAD_PER_REV = 2.0 * Math.PI;

    // ---------- Angular velocity ----------
    public static final double RAD_PER_S_PER_RPM = RAD_PER_REV / 60.0;
    public static final double RPM_PER_RAD_PER_S = 60.0 / RAD_PER_REV;
    public static final double DEG_PER_S_PER_RPM = 6.0;
    public static final double RPM_PER_DEG_PER_S = 1.0 / 6.0;

    // ---------- Encoder ticks per revolution ----------
    // REV
    public static final double REV_HD_HEX_CPR       = 28.0;    // bare motor
    public static final double REV_HD_HEX_20_CPR    = 560.0;   // 20:1
    public static final double REV_HD_HEX_40_CPR    = 1120.0;  // 40:1
    public static final double REV_CORE_HEX_CPR     = 288.0;
    public static final double REV_THROUGH_BORE_CPR = 8192.0;
    // goBILDA
    public static final double GOBILDA_5202_CPR      = 28.0;   // bare motor
    public static final double GOBILDA_5203_CPR      = 28.0;   // bare motor
    public static final double GOBILDA_5202_13_7_CPR = 383.6;
    public static final double GOBILDA_5202_19_2_CPR = 537.6;
    public static final double GOBILDA_5202_26_9_CPR = 753.2;
    public static final double GOBILDA_5203_50_CPR   = 1400.0;

    // ---------- Wheel diameters (inches) ----------
    public static final double REV_90MM_MECANUM_DIAMETER_IN      = 3.543;
    public static final double REV_4IN_MECANUM_DIAMETER_IN       = 4.0;
    public static final double REV_4IN_TRACTION_DIAMETER_IN      = 4.0;
    public static final double GOBILDA_96MM_MECANUM_DIAMETER_IN  = 3.7795;
    public static final double GOBILDA_104MM_MECANUM_DIAMETER_IN = 4.0945;
    public static final double GOBILDA_4IN_OMNI_DIAMETER_IN      = 4.0;

    // ---------- Field ----------
    public static final double FTC_FIELD_SIZE_FT   = 12.0;
    public static final double FTC_FIELD_SIZE_IN   = 141.5;  // usable field width (not the nominal 144)
    public static final double FTC_FIELD_CENTER_IN = 72.0;
    public static final double FTC_TILE_SIZE_IN    = 24.0;

    // ============================================================
    // Unit conversions
    // ============================================================

    /**
     * Converts angular velocity from degrees per second to RPM.
     *
     * @param degPerSec Angular velocity in degrees per second.
     * @return Angular velocity in revolutions per minute.
     */
    public static double degPerSecToRPM(double degPerSec) {
        return degPerSec * RPM_PER_DEG_PER_S;
    }

    // ============================================================
    // Pose conversions
    // ============================================================

    // ------------------------------------------------------------
    // COORDINATE FRAMES
    //
    //  goBILDA Pinpoint (raw):  +X = robot forward at start, +Y = LEFT,
    //                           heading CCW-positive, 0 = facing +X.
    //  FIELD frame (this code): +X = RIGHT, +Y = FORWARD,
    //                           heading CCW-positive, 0 = facing +Y.
    //                           Targets, ray-cast zones, start poses and the
    //                           turret/launcher math all live in this frame, and it is
    //                           the same x/y orientation PedroPathing uses.
    //
    //  Same origin, axes rotated 90 degrees:
    //      field.x = -pinpoint.y        pinpoint.x = field.y
    //      field.y =  pinpoint.x        pinpoint.y = -field.x
    //      field.heading = pinpoint.heading   (both are "CCW from the robot's start direction")
    //
    //  Never feed raw odo.getPosX()/getPosY()/getVelX()/getVelY() into field-frame code;
    //  go through these helpers (DriveSubSystem.getFieldPose() etc. wrap them).
    // ------------------------------------------------------------

    /** Pinpoint (x fwd, y left) -> field X (right). */
    public static double pinpointToFieldX(double pinpointX, double pinpointY) {
        return -pinpointY;
    }

    /** Pinpoint (x fwd, y left) -> field Y (forward). */
    public static double pinpointToFieldY(double pinpointX, double pinpointY) {
        return pinpointX;
    }

    /** Field (x right, y fwd) -> Pinpoint X (forward). */
    public static double fieldToPinpointX(double fieldX, double fieldY) {
        return fieldY;
    }

    /** Field (x right, y fwd) -> Pinpoint Y (left). */
    public static double fieldToPinpointY(double fieldX, double fieldY) {
        return -fieldX;
    }

    /**
     * Converts a raw Pinpoint pose (x forward, y left) to a field-frame pose (x right, y forward).
     * Works for positions in any distance unit; the unit is preserved. Heading is unchanged.
     */
    public static Pose2D pinpointToField(Pose2D pinpointPose) {
        DistanceUnit u = DistanceUnit.INCH;
        double px = pinpointPose.getX(u);
        double py = pinpointPose.getY(u);
        return new Pose2D(u,
                pinpointToFieldX(px, py),
                pinpointToFieldY(px, py),
                AngleUnit.RADIANS,
                pinpointPose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Converts a field-frame pose (x right, y forward) to the raw Pinpoint frame
     * (x forward, y left). Use this before calling {@code odo.setPosition(...)}.
     */
    public static Pose2D fieldToPinpoint(Pose2D fieldPose) {
        DistanceUnit u = DistanceUnit.INCH;
        double fx = fieldPose.getX(u);
        double fy = fieldPose.getY(u);
        return new Pose2D(u,
                fieldToPinpointX(fx, fy),
                fieldToPinpointY(fx, fy),
                AngleUnit.RADIANS,
                fieldPose.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Converts a field-frame FTC Pose2D (x right, y forward, heading 0 = +Y) to a PedroPathing
     * Pose. Pedro measures heading from +X, so facing field-forward (+Y) is PI/2.
     *
     * <p>For a raw Pinpoint pose, call {@link #pinpointToField(Pose2D)} first.
     *
     * @param pose field-frame Pose2D to convert.
     * @return Equivalent PedroPathing Pose.
     */
    public static Pose convertPose2D(Pose2D pose) {
        return new Pose(
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.normalizeRadians(pose.getHeading(AngleUnit.RADIANS) + Math.PI / 2.0));
    }

    /**
     * Converts a PedroPathing Pose to a field-frame FTC Pose2D (x right, y forward,
     * heading 0 = +Y). Inverse of {@link #convertPose2D(Pose2D)}.
     *
     * <p>To seed the Pinpoint with the result, pass it through
     * {@link #fieldToPinpoint(Pose2D)}.
     *
     * @param pose PedroPathing Pose to convert.
     * @return Equivalent field-frame Pose2D.
     */
    public static Pose2D convertPose(Pose pose) {
        return new Pose2D(
                DistanceUnit.INCH,
                pose.x(),
                pose.y(),
                AngleUnit.RADIANS,
                AngleUnit.normalizeRadians(pose.heading() - Math.PI / 2.0));
    }

    /**
     * Converts an FTC Pose2D to a SolversLib Vector2d. Heading is discarded.
     *
     * @param pose FTC Pose2D to convert.
     * @return Vector2d of the pose's X and Y in inches.
     */
    public static Vector2d poseToVector(Pose2D pose) {
        return new Vector2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
    }

    /**
     * Converts an FTC Pose2D to a SolversLib Translation2d. Heading is discarded.
     *
     * @param pose FTC Pose2D to convert.
     * @return Translation2d of the pose's X and Y in inches.
     */
    public static Translation2d poseToTranslation(Pose2D pose) {
        return new Translation2d(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH));
    }

    // ============================================================
    // Joystick
    // ============================================================

    /**
     * Applies a rescaled deadband so output ramps smoothly from 0 just above the threshold to 1.
     *
     * @param input     Raw joystick value in [-1.0, 1.0].
     * @param threshold Minimum motion that registers as input (e.g. 0.05).
     * @return Rescaled value in [-1.0, 1.0].
     */
    public static double applyDeadband(double input, double threshold) {
        if (Math.abs(input) <= threshold) return 0.0;
        return Math.copySign((Math.abs(input) - threshold) / (1.0 - threshold), input);
    }

    /**
     * Blends linear and cubic curves for fine control at low speed.
     *
     * @param input  Raw stick input in [-1.0, 1.0].
     * @param weight Blend factor: 0.0 = pure linear, 1.0 = pure cubic.
     * @return Curved power value.
     */
    public static double applyDriveCurve(double input, double weight) {
        return weight * input * input * input + (1.0 - weight) * input;
    }

    // ============================================================
    // 1D constant-acceleration kinematics
    // ============================================================

    /**
     * Final velocity after constant acceleration. v = v0 + a*t
     *
     * @param v0 Initial velocity.
     * @param a  Constant acceleration.
     * @param t  Time elapsed.
     * @return Final velocity.
     */
    public static double finalVelocity(double v0, double a, double t) {
        return v0 + a * t;
    }

    /**
     * Final position after constant acceleration. x = x0 + v0*t + 0.5*a*t^2
     *
     * @param x0 Initial position.
     * @param v0 Initial velocity.
     * @param a  Constant acceleration.
     * @param t  Time elapsed.
     * @return Final position.
     */
    public static double displacement(double x0, double v0, double a, double t) {
        return x0 + v0 * t + 0.5 * a * t * t;
    }

    /**
     * Final velocity from displacement and constant acceleration. v^2 = v0^2 + 2*a*dx
     *
     * @param v0 Initial velocity.
     * @param a  Constant acceleration.
     * @param dx Displacement.
     * @return Final velocity, or NaN if physically impossible.
     */
    public static double finalVelocityFromDisplacement(double v0, double a, double dx) {
        return safeSqrt(v0 * v0 + 2.0 * a * dx);
    }

    /**
     * Final position using average velocity. x = x0 + (v0 + v)/2 * t
     *
     * @param x0 Initial position.
     * @param v0 Initial velocity.
     * @param v  Final velocity.
     * @param t  Time elapsed.
     * @return Final position.
     */
    public static double displacementFromAverageVelocity(double x0, double v0, double v, double t) {
        return x0 + 0.5 * (v0 + v) * t;
    }

    // ============================================================
    // Projectile motion
    // ============================================================

    /**
     * Horizontal velocity component. v0x = v0*cos(theta)
     *
     * @param v0       Initial speed.
     * @param angleRad Launch angle in radians.
     * @return Horizontal velocity component.
     */
    public static double vx(double v0, double angleRad) {
        return v0 * Math.cos(angleRad);
    }

    /**
     * Vertical velocity component. v0y = v0*sin(theta)
     *
     * @param v0       Initial speed.
     * @param angleRad Launch angle in radians.
     * @return Vertical velocity component.
     */
    public static double vy(double v0, double angleRad) {
        return v0 * Math.sin(angleRad);
    }

    /**
     * Time to reach maximum height. t = vy0 / g
     *
     * @param vy0 Initial vertical velocity.
     * @param g   Gravitational acceleration.
     * @return Time to apex.
     */
    public static double timeToMaxHeight(double vy0, double g) {
        return vy0 / g;
    }

    /**
     * Maximum height above ground. H = y0 + vy0^2 / (2g)
     *
     * @param y0  Initial height.
     * @param vy0 Initial vertical velocity.
     * @param g   Gravitational acceleration.
     * @return Maximum height.
     */
    public static double maxHeight(double y0, double vy0, double g) {
        return y0 + vy0 * vy0 / (2.0 * g);
    }

    /**
     * Total flight time on level ground. T = 2*vy0 / g
     *
     * @param vy0 Initial vertical velocity.
     * @param g   Gravitational acceleration.
     * @return Total flight time.
     */
    public static double totalFlightTimeLevel(double vy0, double g) {
        return 2.0 * vy0 / g;
    }

    /**
     * Horizontal range on level ground. R = v0^2 * sin(2*theta) / g
     *
     * @param v0       Initial speed.
     * @param angleRad Launch angle in radians.
     * @param g        Gravitational acceleration.
     * @return Horizontal range.
     */
    public static double rangeLevelGround(double v0, double angleRad, double g) {
        return v0 * v0 * Math.sin(2.0 * angleRad) / g;
    }

    /**
     * Projectile height at a given horizontal distance.
     *
     * @param x        Horizontal distance traveled.
     * @param y0       Initial height.
     * @param v0       Initial speed.
     * @param angleRad Launch angle in radians.
     * @param g        Gravitational acceleration.
     * @return Height at distance x, or NaN if horizontal velocity is near zero.
     */
    public static double heightAtDistance(double x, double y0, double v0, double angleRad, double g) {
        double vx0 = vx(v0, angleRad);
        if (Math.abs(vx0) < 1e-9) return Double.NaN;
        double t = x / vx0;
        return y0 + vy(v0, angleRad) * t - 0.5 * g * t * t;
    }

    /**
     * Launch speed required to hit a target at (x, y).
     *
     * @param x        Horizontal distance to target.
     * @param y        Target height.
     * @param y0       Initial height.
     * @param angleRad Launch angle in radians.
     * @param g        Gravitational acceleration.
     * @return Required launch speed, or NaN if unreachable at this angle.
     */
    public static double requiredLaunchSpeedToHitTarget(
            double x, double y, double y0, double angleRad, double g) {
        double cos = Math.cos(angleRad);
        double denom = 2.0 * cos * cos * (x * Math.tan(angleRad) - (y - y0));
        return denom > 0 ? Math.sqrt(g * x * x / denom) : Double.NaN;
    }

    /**
     * Speed magnitude at a given height. v = sqrt(v0^2 - 2g(y - y0))
     *
     * @param v0 Initial speed.
     * @param y0 Initial height.
     * @param y  Current height.
     * @param g  Gravitational acceleration.
     * @return Speed at height y, or NaN if physically impossible.
     */
    public static double speedAtHeight(double v0, double y0, double y, double g) {
        return safeSqrt(v0 * v0 - 2.0 * g * (y - y0));
    }

    /**
     * Checks whether the projectile clears a minimum height at a given distance.
     *
     * @param x         Horizontal distance to the obstacle.
     * @param y0        Initial height.
     * @param v0        Initial speed.
     * @param angleRad  Launch angle in radians.
     * @param g         Gravitational acceleration.
     * @param minHeight Minimum height to clear.
     * @return True if the projectile is at or above minHeight at distance x.
     */
    public static boolean clearsHeight(
            double x, double y0, double v0, double angleRad, double g, double minHeight) {
        double h = heightAtDistance(x, y0, v0, angleRad, g);
        return !Double.isNaN(h) && h >= minHeight;
    }

    /**
     * Effective horizontal range considering an obstacle.
     *
     * @param xObstacle Horizontal distance to the obstacle.
     * @param hObstacle Height of the obstacle.
     * @param y0        Initial height.
     * @param v0        Initial speed.
     * @param angleRad  Launch angle in radians.
     * @param g         Gravitational acceleration.
     * @return Full landing range if the obstacle is cleared, otherwise xObstacle.
     */
    public static double effectiveHorizontalRange(
            double xObstacle, double hObstacle, double y0, double v0, double angleRad, double g) {
        double vx0 = vx(v0, angleRad);
        double vy0 = vy(v0, angleRad);
        if (Math.abs(vx0) < 1e-9) return 0.0;

        double disc = vy0 * vy0 + 2.0 * g * y0;
        if (disc < 0) return Double.NaN;
        double fullRange = vx0 * (vy0 + Math.sqrt(disc)) / g;

        boolean clear = xObstacle >= fullRange
                || clearsHeight(xObstacle, y0, v0, angleRad, g, hObstacle);
        return clear ? fullRange : xObstacle;
    }

    // ============================================================
    // Rotational / mechanical
    // ============================================================

    /**
     * Converts angular velocity to linear tangential speed. v = r * omega
     *
     * @param radius                   Radius of rotation.
     * @param angularVelocityRadPerSec Angular velocity in rad/s.
     * @return Linear tangential speed.
     */
    public static double linearSpeedFromAngular(double radius, double angularVelocityRadPerSec) {
        return radius * angularVelocityRadPerSec;
    }

    /**
     * Converts flywheel RPM to linear exit speed. v = RPM * PI * r / 30
     *
     * @param rpm    Rotations per minute.
     * @param radius Flywheel radius.
     * @return Exit speed in radius units per second.
     */
    public static double rpmToLinearSpeed(double rpm, double radius) {
        return rpm * RAD_PER_S_PER_RPM * radius;
    }

    /**
     * Calculates the gear ratio.
     *
     * @param drivingTeeth Teeth on the driving gear.
     * @param drivenTeeth  Teeth on the driven gear.
     * @return Gear ratio (driven / driving).
     */
    public static double gearRatio(double drivingTeeth, double drivenTeeth) {
        return drivenTeeth / drivingTeeth;
    }

    /**
     * Calculates wheel or flywheel circumference. C = 2 * PI * r
     *
     * @param radius Radius of the wheel.
     * @return Circumference.
     */
    public static double wheelCircumference(double radius) {
        return RAD_PER_REV * radius;
    }

    // ============================================================
    // Helpers
    // ============================================================

    /**
     * Square root that returns NaN instead of failing on negative input.
     *
     * @param v Value to take the square root of.
     * @return sqrt(v), or NaN if v is negative.
     */
    private static double safeSqrt(double v) {
        return v >= 0 ? Math.sqrt(v) : Double.NaN;
    }
}