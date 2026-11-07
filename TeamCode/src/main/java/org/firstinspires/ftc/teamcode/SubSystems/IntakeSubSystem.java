package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.roboConfigs.IntakeConstants;

import java.util.List;

/**
 * INTAKE — one specific task: run the intake roller at full power or stop it.
 * See {@code SubSystems/mechanisms.md} for usage notes and pitfalls.
 */
public class IntakeSubSystem extends Subsystem{
    private final DcMotorEx intake;
    private final ElapsedTime emptyTimer;
    private boolean intakeRequest = false, launchRequest = false, spitRequest = false;
    private boolean startedIntake = false, startedLaunch = false, startedSpit = false;
    private int ballCount;
    private ElapsedTime settleTimer;

    public IntakeSubSystem(List<DcMotorEx> Motors) {
        this.intake = Motors.get(0);

        settleTimer = new ElapsedTime();
        emptyTimer = new ElapsedTime();
    }

    /** Runs the intake motor at full power. */
    public void on(){ intake.setPower(IntakeConstants.ON); }

    /** Stops the intake motor. */
    public void off(){ intake.setPower(IntakeConstants.OFF); }

    /** Reverses the intake motor. */
    public void spit(){ intake.setPower(IntakeConstants.SPIT); }

    /** Runs intake motors when launching, not full power for consistency of shot */
    public void launch() { intake.setPower(IntakeConstants.LAUNCH); }

    /** Command requests. */
    public void intakeRequest(boolean set){
        if (!set || startedIntake) return;
        intakeRequest = true;
        setActive(true);
    }
    public void launchRequest(boolean set){
        if (!set || startedLaunch) return;
        launchRequest = true;
        setActive(true);
    }

    public void spitRequest(boolean set){
        if (!set || startedSpit) return;
        spitRequest = true;
        setActive(true);
    }

    /** Transfer states. */
    public void startedIntake(boolean set){ startedIntake = set; }
    public void startedLaunch(boolean set){ startedLaunch = set; }
    public void startedSpit(boolean set){ startedSpit = set; }

    /** Stops intake if needed. */
    public void interruptStop(){ resetState(); }

    /** Updates ball count. */
    public void setBallCount(int ballCount){ this.ballCount = ballCount; }


    /** Updates subsystem when active */
    @Override
    public void update(){
        //starts intake if requested
        if (intakeRequest && !startedIntake){
            on();
            startedIntake(true);
        }

        //stops intake if full
        if (startedIntake && !spitRequest && !launchRequest){
            if (ballCount != 4) {
                settleTimer.reset();
            } else if (settleTimer.milliseconds() > IntakeConstants.SETTLE_TIME) {
                intakeRequest = false;
                startedIntake = false;
                setActive(false);
                off();
            }
        }

        //spits if requested
        if (spitRequest && !startedSpit){
            spit();
            startedSpit(true);
            emptyTimer.reset();
        }

        //launch if requested
        if (launchRequest && !startedLaunch){
            startedLaunch(true);
            launch();
            emptyTimer.reset();
        }

        //stops once empty
        if (startedSpit || startedLaunch){
            if (ballCount != 0){
                emptyTimer.reset();
            } else if (emptyTimer.milliseconds() > IntakeConstants.EMPTY_TIME) {
                resetState();
            }
        }
    }

    /** Resets all booleans */
    public void resetState(){
        off();
        setActive(false);

        intakeRequest = false;
        spitRequest = false;
        launchRequest = false;

        startedIntake = false;
        startedLaunch = false;
        startedSpit = false;
    }

    /** Stops the intake motors.*/
    @Override
    public void stop() {
        off();
    }

    // ==========================================================
    // GETTERS
    // ==========================================================

    public boolean hasFour(){ return ballCount == 4 && settleTimer.milliseconds() > 200; }

}
