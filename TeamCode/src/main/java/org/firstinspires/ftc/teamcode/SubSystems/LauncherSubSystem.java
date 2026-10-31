package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.BANG_BANG_ENTER_RPM;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.BANG_BANG_EXIT_RPM;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.kD;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.kF;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.kI;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.kP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.customMath;

import java.util.List;

public class LauncherSubSystem extends Subsystem {

    private final DcMotorEx fwL, fwR;
    private final InterpLUT flywheelLut = new InterpLUT();
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double output = 0.0;

    /** True while the bang-bang region is active, false while PIDF is in control. */
    private boolean bangBangActive = true;

    /** True while the motors are held in BRAKE by stop(); off()/resume restore FLOAT. */
    private boolean braked = false;

    /** Optional overrides (used by test OpModes). NaN = fall back to LauncherConstants. */
    private double enterOverrideRpm = Double.NaN;
    private double exitOverrideRpm = Double.NaN;

    public LauncherSubSystem(List<DcMotorEx> motors) {
        fwL = motors.get(0);
        fwR = motors.get(1);
    }

    // ==========================================================
    // LIFECYCLE (@Override)
    // ==========================================================

    @Override
    public void setup() {
        setMotorsMode(fwL, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorsMode(fwR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehavior(fwL, DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(fwR, DcMotor.ZeroPowerBehavior.FLOAT);
        braked = false;

        // Distance (inches) -> RPM
        flywheelLut.add(24.0, 2800.0);
        flywheelLut.add(30.0, 2950.0);
        flywheelLut.add(36.0, 3100.0);
        flywheelLut.add(42.0, 3250.0);
        flywheelLut.add(48.0, 3400.0);
        flywheelLut.add(54.0, 3550.0);
        flywheelLut.add(60.0, 3700.0);
        flywheelLut.add(66.0, 3850.0);
        flywheelLut.add(72.0, 4000.0);
        flywheelLut.createLUT();

        off();
    }

    /**
     * Piecewise control: bang-bang when |error| &gt; enter threshold, PIDF when |error| &lt; exit
     * threshold, otherwise keep the current mode (hysteresis).
     *
     * <p>Only called by the master loop while active, so it does not check {@code isActive()}.
     */
    @Override
    public void update() {
        currentRPM = customMath.degPerSecToRPM(
                (fwL.getVelocity(AngleUnit.DEGREES)
                        + fwR.getVelocity(AngleUnit.DEGREES)) / 2.0
        );

        double error = targetRPM - currentRPM;
        double absError = Math.abs(error);

        double enterRpm = Double.isNaN(enterOverrideRpm) ? BANG_BANG_ENTER_RPM : enterOverrideRpm;
        double exitRpm = Double.isNaN(exitOverrideRpm) ? BANG_BANG_EXIT_RPM : exitOverrideRpm;

        if (absError > enterRpm) {
            bangBangActive = true;
        } else if (absError < exitRpm) {
            bangBangActive = false;
        }

        if (bangBangActive) {
            // Full power to spin up; coast (no reverse power) when overspeed.
            output = error > 0.0 ? 1.0 : 0.0;

            // Keep the integrator clean so PIDF starts fresh at handoff.
            pidf.reset();
        } else {
            output = pidf.calculate(currentRPM, targetRPM);
            output = Math.max(-1.0, Math.min(1.0, output));
        }

        fwL.setPower(output);
        fwR.setPower(output);
    }

    /**
     * Full stop: cuts power AND actively brakes the flywheel (motor terminals shorted), then
     * deactivates the subsystem so no further encoder reads happen. Setting a new target later
     * releases the brake automatically.
     *
     * <p> ONLY USE AS ESTOP.
     */
    @Override
    public void stop() {
        fwL.setPower(0.0);
        fwR.setPower(0.0);

        setZeroPowerBehavior(fwL, DcMotor.ZeroPowerBehavior.BRAKE);
        setZeroPowerBehavior(fwR, DcMotor.ZeroPowerBehavior.BRAKE);
        braked = true;

        resetState();
    }

    // ==========================================================
    // COMMANDS
    // ==========================================================

    /**
     * Rest: cuts power and lets the flywheel coast down freely (FLOAT), then deactivates the
     * subsystem so no further encoder reads happen. Setting a new target resumes control.
     */
    public void off() {
        fwL.setPower(0.0);
        fwR.setPower(0.0);

        releaseBrake();
        resetState();
    }

    /**
     * Automatic mode:
     * distance -> LUT -> target RPM -> piecewise controller.
     */
    public void autoPIDF(Pose2D local, Pose2D target) {
        double distance = customMath.poseToTranslation(local)
                .getDistance(customMath.poseToTranslation(target));

        setTargetRPM(flywheelLut.get(distance));
    }

    /**
     * Manual mode:
     * manually supplied RPM -> piecewise controller.
     *
     * <p>The interpolation table is not used.
     */
    public void setRPM(double rpm) {
        setTargetRPM(rpm);
    }

    /**
     * Sets the target RPM used by both manual and automatic modes.
     *
     * <p>If the subsystem was off/stopped, this wakes it back up: the brake is released and the
     * controller restarts in bang-bang with a clean PIDF, so the next update() reads the real
     * flywheel speed and goes to the target from wherever it currently is.
     */
    private void setTargetRPM(double rpm) {
        if (rpm <= 0.0) {
            off();
            return;
        }

        if (!isActive()) {
            releaseBrake();
            bangBangActive = true;
            pidf.reset();
            setActive(true);
        } else if (Math.abs(targetRPM - rpm) > 1.0) {
            pidf.reset();
        }

        targetRPM = rpm;
    }

    /**
     * Just for testing
     *
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public void setPIDF(double p, double i, double d, double f) {
        if (pidf.getP() == p &&
                pidf.getI() == i &&
                pidf.getD() == d &&
                pidf.getF() == f) {
            return;
        }

        pidf.setPIDF(p, i, d, f);
        pidf.reset();
    }

    /**
     * Just for testing: overrides the bang-bang thresholds from LauncherConstants.
     * The exit threshold is clamped so it never exceeds the enter threshold.
     *
     * @param enterRpm Error range above which bang-bang takes over.
     * @param exitRpm  Error range below which PIDF takes over.
     */
    public void setBangBangThresholds(double enterRpm, double exitRpm) {
        enterOverrideRpm = enterRpm;
        exitOverrideRpm = Math.min(exitRpm, enterRpm);
    }

    // ==========================================================
    // GETTERS
    // ==========================================================

    /**
     * Returns the flywheel RPM from the last active update(). While the subsystem is off or
     * stopped the encoders are not read, so this is 0 (meaning "not measured").
     */
    public double getCurrentRPM() {
        return currentRPM;
    }

    /**
     * Returns the current target RPM (0 while off/stopped).
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Returns the current controller output (0 while off/stopped).
     */
    public double getOutput() {
        return output;
    }

    /**
     * Returns true while the bang-bang region is in control (useful for telemetry).
     */
    public boolean isBangBangActive() {
        return bangBangActive;
    }

    /**
     * Returns true when the subsystem is running and the flywheel is within 50 RPM of the
     * target. Always false while off/stopped.
     */
    public boolean atTarget() {
        return isActive() && Math.abs(targetRPM - currentRPM) <= 50.0;
    }

    // ==========================================================
    // INTERNAL HELPERS
    // ==========================================================

    /** Restores FLOAT on both motors if stop() had put them in BRAKE. */
    private void releaseBrake() {
        if (!braked) return;

        setZeroPowerBehavior(fwL, DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(fwR, DcMotor.ZeroPowerBehavior.FLOAT);
        braked = false;
    }

    /** Clears all controller state and deactivates the subsystem (no more encoder reads). */
    private void resetState() {
        targetRPM = 0.0;
        currentRPM = 0.0;
        output = 0.0;

        bangBangActive = true;
        pidf.reset();
        setActive(false);
    }
}