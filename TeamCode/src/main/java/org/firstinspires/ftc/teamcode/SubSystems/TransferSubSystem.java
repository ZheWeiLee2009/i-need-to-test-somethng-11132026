package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.roboConfigs.IntakeConstants;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.ServoConstants.Gate;
import org.firstinspires.ftc.teamcode.utils.ServoUtility;

import java.util.List;

/**
 * INTAKE — one specific task: run the intake roller at full power or stop it.
 * See {@code SubSystems/mechanisms.md} for usage notes and pitfalls.
 */
public class TransferSubSystem extends Subsystem{
    private final DcMotorEx transfer;
    public final DigitalChannel l1, l2, l3, l4, r1, r2, r3, r4;
    public ServoTaskSubsystem gate;
    private final ElapsedTime emptyTimer, settledTimer, fullTimer;
    private boolean b1 = false, b2 = false, b3 = false, b4 = false;
    private boolean read1 = false, read2 = false, read3 = false, readAll = false;
    private boolean intakeRequest = false, launchRequest = false, spitRequest = false;
    private boolean startedIntake = false, startedLaunch = false, startedSpit = false;
    private boolean transferFull = false;
    private double gateAngle;

    public TransferSubSystem(List<DcMotorEx> Motors, List<Servo> Servos, List<DigitalChannel> Sensors){
        this.transfer = Motors.get(0);

        l1 = Sensors.get(0);
        l2 = Sensors.get(1);
        l3 = Sensors.get(2);
        l4 = Sensors.get(3);
        r1 = Sensors.get(4);
        r2 = Sensors.get(5);
        r3 = Sensors.get(6);
        r4 = Sensors.get(7);

        gate = new ServoTaskSubsystem("gate",
                new Servo[] {Servos.get(0)},
                Gate.MIN_DEG,
                Gate.MAX_DEG,
                Gate.AT_MIN,
                Gate.AT_MAX);

            emptyTimer = new ElapsedTime();
            settledTimer = new ElapsedTime();
            fullTimer = new ElapsedTime();
    }

    /** Initializes gate. */
    @Override
    public void setup(){ gateDown(); }

    /** Runs the transfer motor at full power. */
    public void on(){ transfer.setPower(IntakeConstants.ON); }

    /** Stops the transfer motor. */
    public void off(){ transfer.setPower(IntakeConstants.OFF);}

    /** Reverses the transfer motor. */
    public void spit(){ transfer.setPower(IntakeConstants.SPIT); }

    /** Runs transfer motor when launching, not full power for consistency of shot */
    public void launch() {
        transfer.setPower(IntakeConstants.LAUNCH);
        gateUp();
    }

    /** Gate commands. */
    public void gateUp(){ gate.setAngle(gate.getAxis().getMinDeg()); }

    public void gateDown(){ gate.setAngle(gate.getAxis().getMaxDeg()); }

    /** Transfer command requests. */
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

    /** Stop intake if needed. */
    public void interruptStop(){ resetState(); }

    /** Updates subsystem when active */
    @Override
    public void update(){
        //updates sensor reads
        updateSensors();

        //starts transfer if requested
        if (intakeRequest && !startedIntake){
            on();
            startedIntake(true);
        }

        //stops transfer if ball count is 3 when intaking, sets subsystem inactive when full
        if (startedIntake && !spitRequest && !launchRequest){
            if (!(b1 && b2 && b3)){
                settledTimer.reset();
            } else if (settledTimer.milliseconds() > 300){
                transferFull = true;
                off();
            }

            if (getBallCount() != 4){
                fullTimer.reset();
            } else if (fullTimer.milliseconds() > 200 && transferFull){
                setActive(false);
                resetIntaking();
            }
        }

        //spits if requested
        if (spitRequest && !startedSpit){
            spit();
            startedSpit(true);
        }

        //launch if requested
        if (launchRequest && !startedLaunch){
            startedLaunch(true);
            launch();
        }

        //stops once empty
        if (startedSpit || startedLaunch){
            if (getBallCount() != 0){
                emptyTimer.reset();
            } else if (emptyTimer.milliseconds() > IntakeConstants.EMPTY_TIME) {
                resetState();
            }
        }

    }

    /** Reads intake sensors. */
    public void updateSensors(){
            b1 = l1.getState() || r1.getState();
            b2 = l2.getState() || r2.getState();
            b3 = l3.getState() || r3.getState();
            b4 = l4.getState() || r4.getState();
    }

    /** Reset sensor values. */
    public void resetSensors(){
        b1 = false;
        b2 = false;
        b3 = false;
        b4 = false;
    }

    /** Reset intake booleans. */
    public void resetIntaking(){
        intakeRequest = false;
        startedIntake = false;
        transferFull = false;
    }

    /** Reset launching booleans. */
    public void resetLaunching(){
        launchRequest = false;
        startedLaunch = false;
    }

    /** Reset spitting booleans. */
    public void resetSpitting(){
        spitRequest = false;
        startedSpit = false;
    }

    /** Resets all booleans */
    public void resetState(){
        setActive(false);
        off();
        gateDown();

        resetSensors();

        resetIntaking();

        resetLaunching();

        resetSpitting();
    }

    /** Stops the intake motors.*/
    @Override
    public void stop() {
        off();
    }

    // ==========================================================
    // GETTERS
    // ==========================================================

    /** Gets sensor states */
    public boolean getB1(){ return b1; }
    public boolean getB2(){ return b2; }
    public boolean getB3(){ return b3; }
    public boolean getB4(){ return b4; }

    /** Gets ball count in bot */
    public int getBallCount() {
        return (b1 ? 1 : 0) + (b2 ? 1 : 0) + (b3 ? 1 : 0) + (b4 ? 1 : 0);
    }

}
