package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utils.customMath;

import java.util.List;

public class LauncherSubSystem extends Subsystem {

    private final DcMotorEx fwL, fwR;
    private final InterpLUT leftFlywheelLut = new InterpLUT();
    private final InterpLUT rightFlywheelLut = new InterpLUT();
    private final PIDFController fwLpidf = new PIDFController(fwLkP, fwLkI, fwLkD, fwLkF);
    private final PIDFController fwRpidf = new PIDFController(fwRkP, fwRkI, fwRkD, fwRkF);
    private double fwLTargetRPM = 0.0;
    private double fwRTargetRPM = 0.0;
    private double fwLCurrentRPM = 0.0;
    private double fwRCurrentRPM = 0.0;
    private double fwLOutput = 0.0;
    private double fwROutput = 0.0;

    /** True while the bang-bang region is active, false while PIDF is in control. */
    private boolean fwLBangBangActive = true;
    private boolean fwRBangBangActive = true;

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

    /**
     * setActive(true): the controller runs (bang-bang -> PIDF toward the current target) and
     * sends power on every update().
     *
     * <p>setActive(false): power is cut, the flywheel coasts (FLOAT), and the controller state
     * and target are cleared. Nothing is maintained or sent while inactive.
     */
    @Override
    public void setActive(boolean active) {
        if (active) {
            if (isActive()) return;
            releaseBrake();
            fwLBangBangActive = true;
            fwRBangBangActive = true;
            fwLpidf.reset();
            fwRpidf.reset();
            super.setActive(true);
        } else {
            fwL.setPower(0.0);
            fwR.setPower(0.0);
            releaseBrake();
            resetState();
        }
    }

    @Override
    public void setup() {
        setMotorsMode(fwL, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMotorsMode(fwR, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehavior(fwL, DcMotor.ZeroPowerBehavior.FLOAT);
        setZeroPowerBehavior(fwR, DcMotor.ZeroPowerBehavior.FLOAT);
        braked = false;

        // Distance (inches) -> RPM
        leftFlywheelLut.add(38.0, 2275.0); //right is 2200
        leftFlywheelLut.add(40.5, 2250.0); //right is 2250
        leftFlywheelLut.add(43.5, 2200.0); //right is 2300
        leftFlywheelLut.add(52.0, 2225.0); //right is 2450
        leftFlywheelLut.add(55.0, 2250.0); //right is 2450
        leftFlywheelLut.createLUT();

        rightFlywheelLut.add(38.0, 2200.0); //right is 2200
        rightFlywheelLut.add(40.5, 2250.0); //right is 2250
        rightFlywheelLut.add(43.5, 2300.0); //right is 2300
        rightFlywheelLut.add(52.0, 2450.0); //right is 2450
        rightFlywheelLut.add(55.0, 2450.0); //right is 2450
        rightFlywheelLut.createLUT();

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
        if (!isActive()) return;

//        currentRPM = customMath.degPerSecToRPM(
//                (fwL.getVelocity(AngleUnit.DEGREES)
//                        + fwR.getVelocity(AngleUnit.DEGREES)) / 2.0
//        );

        fwLCurrentRPM = fwL.getVelocity() * 60/28 /* Ticks per second -> RPM*/;
        fwRCurrentRPM = fwR.getVelocity() * 60/28 /* Ticks per second -> RPM*/;

        double fwLError = fwLTargetRPM - fwLCurrentRPM;
        double fwLAbsError = Math.abs(fwLError);
        double fwRError = fwRTargetRPM - fwRCurrentRPM;
        double fwRAbsError = Math.abs(fwRError);

        double enterRpm = Double.isNaN(enterOverrideRpm) ? BANG_BANG_ENTER_RPM : enterOverrideRpm;
        double exitRpm = Double.isNaN(exitOverrideRpm) ? BANG_BANG_EXIT_RPM : exitOverrideRpm;

        if (fwLAbsError > enterRpm) {
            fwLBangBangActive = true;
        } else if (fwLAbsError < exitRpm) {
            fwLBangBangActive = false;
        }

        if (fwRAbsError > enterRpm) {
            fwRBangBangActive = true;
        } else if (fwRAbsError < exitRpm) {
            fwRBangBangActive = false;
        }

        if (fwLBangBangActive) {
            // Full power to spin up; coast (no reverse power) when overspeed.
            fwLOutput = fwLError > 0.0 ? 1.0 : 0.0;

            // Keep the integrator clean so PIDF starts fresh at handoff.
            fwLpidf.reset();
        } else {
            fwLOutput = fwLpidf.calculate(fwLCurrentRPM, fwLTargetRPM);
            fwLOutput = Math.max(-1.0, Math.min(1.0, fwLOutput));
        }

        if (fwRBangBangActive) {
            // Full power to spin up; coast (no reverse power) when overspeed.
            fwROutput = fwRError > 0.0 ? 1.0 : 0.0;

            // Keep the integrator clean so PIDF starts fresh at handoff.
            fwRpidf.reset();
        } else {
            fwROutput = fwRpidf.calculate(fwRCurrentRPM, fwRTargetRPM);
            fwROutput = Math.max(-1.0, Math.min(1.0, fwROutput));
        }

        fwL.setPower(fwLOutput);
        fwR.setPower(fwROutput);
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
        setActive(false);
    }

    /**
     * Automatic mode:
     * distance -> LUT -> target RPM -> piecewise controller.
     */
    public void autoPIDF(Pose2D local, Pose2D target) {
        double distance = customMath.poseToTranslation(local)
                .getDistance(customMath.poseToTranslation(target));

        setTargetRPM(leftFlywheelLut.get(distance), rightFlywheelLut.get(distance));
    }

    /**
     * Manual mode:
     * manually supplied RPM -> piecewise controller.
     *
     * <p>The interpolation table is not used.
     */
    public void setRPM(double fwLRpm, double fwRRpm) {
        setTargetRPM(fwLRpm, fwRRpm);
    }

    /**
     * Sets the target RPM used by both manual and automatic modes.
     *
     * <p>This only sets the target. It does NOT activate the launcher: call setActive(true) for
     * the controller to run. A target of 0 or less turns the launcher off.
     */
    private void setTargetRPM(double fwLRpm, double fwRRpm) {
        if (fwLRpm <= 0.0) {
            off();
            return;
        }

        if (fwRRpm <= 0.0) {
            off();
            return;
        }

        if (Math.abs(fwLTargetRPM - fwLRpm) > 1.0) fwLpidf.reset();
        if (Math.abs(fwRTargetRPM - fwRRpm) > 1.0) fwRpidf.reset();

        fwLTargetRPM = fwLRpm;
        fwRTargetRPM = fwRRpm;
    }

    /**
     * Just for testing
     *
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public void setFwLPIDF(double p, double i, double d, double f) {
        if (fwLpidf.getP() == p &&
                fwLpidf.getI() == i &&
                fwLpidf.getD() == d &&
                fwLpidf.getF() == f) {
            return;
        }

        fwLpidf.setPIDF(p, i, d, f);
        fwLpidf.reset();
    }

    public void setFwRPIDF(double p, double i, double d, double f) {
        if (fwRpidf.getP() == p &&
                fwRpidf.getI() == i &&
                fwRpidf.getD() == d &&
                fwRpidf.getF() == f) {
            return;
        }

        fwRpidf.setPIDF(p, i, d, f);
        fwRpidf.reset();
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
    public double getFwLCurrentRPM() {
        return fwLCurrentRPM;
    }

    public double getFwRCurrentRPM() {
        return fwRCurrentRPM;
    }

    /**
     * Returns the current target RPM (0 while off/stopped).
     */
    public double getFwLTargetRPM() {
        return fwLTargetRPM;
    }

    public double getFwRTargetRPM() {
        return fwRTargetRPM;
    }

    /**
     * Returns the current controller output (0 while off/stopped).
     */
    public double getFwLOutput() {
        return fwLOutput;
    }

    public double getFwROutput() {
        return fwROutput;
    }

    /**
     * Returns true while the bang-bang region is in control (useful for telemetry).
     */
    public boolean isFwLBangBangActive() {
        return fwLBangBangActive;
    }

    public boolean isFwRBangBangActive() {
        return fwRBangBangActive;
    }

    /**
     * Returns true when the subsystem is running and the flywheel is within 50 RPM of the
     * target. Always false while off/stopped.
     */
    public boolean isFwLAtTarget() {
        return isActive() && Math.abs(fwLTargetRPM - fwLCurrentRPM) <= 50.0;
    }

    public boolean isFwRAtTarget() {
        return isActive() && Math.abs(fwRTargetRPM - fwRCurrentRPM) <= 50.0;
    }

    public boolean isAtTarget(){
        return isFwLAtTarget() && isFwRAtTarget();
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
        fwLTargetRPM = 0.0;
        fwLCurrentRPM = 0.0;
        fwLOutput = 0.0;

        fwRTargetRPM = 0.0;
        fwRCurrentRPM = 0.0;
        fwROutput = 0.0;

        fwLBangBangActive = true;
        fwRBangBangActive = true;
        fwLpidf.reset();
        fwRpidf.reset();
        super.setActive(false);
    }
}