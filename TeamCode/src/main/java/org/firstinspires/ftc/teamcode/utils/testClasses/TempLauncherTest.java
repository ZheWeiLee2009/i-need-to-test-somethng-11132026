package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants;
import org.firstinspires.ftc.teamcode.PracticeBot;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem;

import java.util.Arrays;
import java.util.List;

/**
 * gamepad1 A: start / stop the launcher
 *
 * <p>Runs through {@link PracticeBot}: the bot builds and sets up the launcher, and
 * {@code bot.update()} (once per loop) clears the bulk cache and ticks it while active.
 */
@Disabled
@Configurable
@TeleOp(name = "Temp Launcher Test", group = "9 - Utility")
public class TempLauncherTest extends OpMode {

    private enum State {
        IDLE,
        SPINNING_UP,
        READY,
        RECOVERING,
        SHOOTING
    }

    public static double DISTANCE_IN = 24.0;

    // Default to the shipped constants so the PIDF region isn't dead (0 output) until tuned.
    public static double fwLkP = LauncherConstants.fwLkP;
    public static double fwLkI = LauncherConstants.fwLkI;
    public static double fwLkD = LauncherConstants.fwLkD;
    public static double fwLkF = LauncherConstants.fwLkF;
    public static double fwRkP = LauncherConstants.fwRkP;
    public static double fwRkI = LauncherConstants.fwRkI;
    public static double fwRkD = LauncherConstants.fwRkD;
    public static double fwRkF = LauncherConstants.fwRkF;
    public static double fwLRpm = LauncherConstants.fwLRpm;
    public static double fwRRpm = LauncherConstants.fwRRpm;

    // Bang-bang region: error > ENTER -> full power, error < EXIT -> PIDF. Keep EXIT <= ENTER.
    public static double BANG_BANG_ENTER_RPM = LauncherConstants.BANG_BANG_ENTER_RPM;
    public static double BANG_BANG_EXIT_RPM = LauncherConstants.BANG_BANG_EXIT_RPM;
    DcMotorEx fwR, fwL;
    List<DcMotorEx> flywheelMotors;
    DcMotorEx intake1;
    List<DcMotorEx> intakeMotors;
    DcMotorEx transfer;
    ServoImplEx blocker;
    private LauncherSubSystem launcher;
    private IntakeSubSystem intake;
    private Telemetry joined;

    private State state = State.IDLE;
    private long stateEnterNs = 0;
    private boolean aPressedLast = false;

    // Reset once per loop; PracticeBot uses it.
    private final ElapsedTime runtime = new ElapsedTime();

    private double spinUpMs = 0;
    private double recoveryMs = 0;

    private double botUpdateMs = 0;
    private long lastLoopStartNs = 0;
    private double loopPeriodMs = 0;

    @Override
    public void init() {

        fwL = hardwareMap.get(DcMotorEx.class, "leftShooter");
        fwR = hardwareMap.get(DcMotorEx.class, "rightShooter");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        blocker = hardwareMap.get(ServoImplEx.class, "shooterGate");

        flywheelMotors = Arrays.asList(fwL, fwR);
        launcher = new LauncherSubSystem(flywheelMotors); // NOTE: change if PracticeBot names the field differently

        launcher.setup();

        launcher.setFwLPIDF(fwLkP, fwLkI, fwLkD, fwLkF);
        launcher.setFwRPIDF(fwRkP, fwRkI, fwRkD, fwRkF);
        launcher.setBangBangThresholds(BANG_BANG_ENTER_RPM, BANG_BANG_EXIT_RPM);

        intakeMotors = Arrays.asList(intake1);
//        intake = new IntakeSubSystem(intakeMotors);

        intake.setup();

        joined = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );
        joined.setMsTransmissionInterval(100);

        joined.addLine("Initialized");
        joined.update();
        runtime.reset();
    }

    @Override
    public void loop() {
        long loopStartNs = System.nanoTime();

        if (lastLoopStartNs != 0) {
            loopPeriodMs = (loopStartNs - lastLoopStartNs) / 1_000_000.0;
        }
        lastLoopStartNs = loopStartNs;

        if (gamepad1.b) {
            intake.on();
            transfer.setPower(1);
        } else if (!launcher.isActive()){
            intake.off();
            transfer.setPower(0);
        }

        if (gamepad1.a && !aPressedLast) {
            setState(state == State.IDLE ? State.SPINNING_UP : State.IDLE);
        }
        aPressedLast = gamepad1.a;

        // Commands first (they set target / wake the launcher), then one master update.
        switch (state) {
            case IDLE:
                break;

            case SPINNING_UP:
                commandLauncher();
                if (launcher.isFwRAtTarget() && launcher.isFwLAtTarget()) {
                    spinUpMs = stateElapsedMs();
                    setState(State.READY);
                }
                break;

            case READY:
                commandLauncher();
                if (!launcher.isFwRAtTarget() || !launcher.isFwLAtTarget()) {
                    setState(State.RECOVERING);
                } else {
                    intake.launch();
                    transfer.setPower(0.8);
                    blocker.setPosition(0.66);
                    setState(State.SHOOTING);
                }
                break;

            case RECOVERING:
                commandLauncher();
                if (launcher.isFwRAtTarget() && launcher.isFwLAtTarget()) {
                    recoveryMs = stateElapsedMs();
                    setState(State.READY);
                }
                break;

            case SHOOTING:
                commandLauncher();
                if (stateElapsedMs() > 2000){
                    setState(State.IDLE);
                }
        }

        long updateStart = System.nanoTime();
        launcher.update();
        botUpdateMs = (System.nanoTime() - updateStart) / 1_000_000.0;

        double fwLTargetRPM = launcher.getFwLTargetRPM();
        double fwLCurrentRPM = launcher.getFwLCurrentRPM();

        double fwRTargetRPM = launcher.getFwRTargetRPM();
        double fwRCurrentRPM = launcher.getFwRCurrentRPM();

        joined.addData("State", state.name());
        joined.addData("State Time (ms)", stateElapsedMs());
        joined.addData("Spin-Up Time (ms)", spinUpMs);
        joined.addData("Recovery Time (ms)", recoveryMs);

        joined.addData("Distance (in)", DISTANCE_IN);

        joined.addData("Left RPM", fwLRpm);
        joined.addData("Right  RPM", fwRRpm);

        joined.addData("Left Target RPM", fwLTargetRPM);
        joined.addData("Left Current RPM", fwLCurrentRPM);
        joined.addData("Right Target RPM", fwRTargetRPM);
        joined.addData("Right Current RPM", fwRCurrentRPM);
        joined.addData("Left RPM Error", fwLTargetRPM - fwLCurrentRPM);
        joined.addData("Right RPM Error", fwRTargetRPM - fwRCurrentRPM);
        joined.addData("Left Controller Output", launcher.getFwLOutput());
        joined.addData("Right Controller Output", launcher.getFwROutput());
        joined.addData("Left Mode", launcher.isFwLBangBangActive() ? "BANG-BANG" : "PIDF");
        joined.addData("Right Mode", launcher.isFwRBangBangActive() ? "BANG-BANG" : "PIDF");
        joined.addData("Active", launcher.isActive());
        joined.addData("At Target", launcher.isFwLAtTarget());

        joined.addData("fwLkP", fwLkP);
        joined.addData("fwLkI", fwLkI);
        joined.addData("fwLkD", fwLkD);
        joined.addData("fwLkF", fwLkF);
        joined.addData("fwRkP", fwRkP);
        joined.addData("fwRkI", fwRkI);
        joined.addData("fwRkD", fwRkD);
        joined.addData("fwRkF", fwRkF);
        joined.addData("Bang-Bang Enter (RPM)", BANG_BANG_ENTER_RPM);
        joined.addData("Bang-Bang Exit (RPM)", BANG_BANG_EXIT_RPM);
        joined.addData("Left RPM", LauncherConstants.fwLRpm);
        joined.addData("Right RPM", LauncherConstants.fwLRpm);

        joined.addData("Bot Update (ms)", botUpdateMs);
        joined.addData("Loop Time (ms)", loopPeriodMs);
        joined.addData("Loop Frequency (Hz)", 1000.0 / Math.max(loopPeriodMs, 0.001));

        joined.update();
        runtime.reset();
    }

    @Override
    public void stop() {
        if (launcher != null) {
            launcher.stop();
            intake.stop();
            transfer.setPower(0);
        }
    }

    private void setState(State next) {
        state = next;
        stateEnterNs = System.nanoTime();

        if (next == State.IDLE) {
            launcher.off(); // normal rest; stop() is reserved for e-stop
            intake.off();
            transfer.setPower(0);
            blocker.setPosition(1);
        }
    }

    private double stateElapsedMs() {
        return (System.nanoTime() - stateEnterNs) / 1_000_000.0;
    }

    private void commandLauncher() {
        // Two poses DISTANCE_IN apart so autoPIDF() uses its normal LUT logic.
//        Pose2D local = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0);
//        Pose2D target = new Pose2D(DistanceUnit.INCH, DISTANCE_IN, 0.0, AngleUnit.DEGREES, 0.0);

        launcher.setRPM(fwLRpm, fwRRpm);
    }
}