package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants;
import org.firstinspires.ftc.teamcode.Robot.PracticeBot;
import org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem;

/**
 * gamepad1 A: start / stop the launcher
 *
 * <p>Runs through {@link PracticeBot}: the bot builds and sets up the launcher, and
 * {@code bot.update()} (once per loop) clears the bulk cache and ticks it while active.
 */
@Disabled
@Configurable
@TeleOp(name = "Launcher Test", group = "9 - Utility")
public class LauncherTest extends OpMode {

    private enum State {
        IDLE,
        SPINNING_UP,
        READY,
        RECOVERING
    }

    public static double DISTANCE_IN = 24.0;

    // Default to the shipped constants so the PIDF region isn't dead (0 output) until tuned.
    public static double kP = LauncherConstants.fwRkP;
    public static double kI = LauncherConstants.fwRkI;
    public static double kD = LauncherConstants.fwRkD;
    public static double kF = LauncherConstants.fwRkF;

    // Bang-bang region: error > ENTER -> full power, error < EXIT -> PIDF. Keep EXIT <= ENTER.
    public static double BANG_BANG_ENTER_RPM = LauncherConstants.BANG_BANG_ENTER_RPM;
    public static double BANG_BANG_EXIT_RPM = LauncherConstants.BANG_BANG_EXIT_RPM;

    private PracticeBot bot;
    private LauncherSubSystem launcher;
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
        bot = new PracticeBot(hardwareMap, runtime);
        launcher = bot.launcher; // NOTE: change if PracticeBot names the field differently

        // Only the launcher may be active in this test; keep every other subsystem idle so
        // bot.update() ticks nothing else. Add any other PracticeBot subsystems here.
        bot.drive.setActive(false);

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

        launcher.setFwRPIDF(kP, kI, kD, kF);
        launcher.setBangBangThresholds(BANG_BANG_ENTER_RPM, BANG_BANG_EXIT_RPM);

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
                if (launcher.isFwLAtTarget()) {
                    spinUpMs = stateElapsedMs();
                    setState(State.READY);
                }
                break;

            case READY:
                commandLauncher();
                if (!launcher.isFwLAtTarget()) {
                    setState(State.RECOVERING);
                }
                break;

            case RECOVERING:
                commandLauncher();
                if (launcher.isFwLAtTarget()) {
                    recoveryMs = stateElapsedMs();
                    setState(State.READY);
                }
                break;
        }

        long updateStart = System.nanoTime();
        bot.update();
        botUpdateMs = (System.nanoTime() - updateStart) / 1_000_000.0;

        double targetRPM = launcher.getFwLTargetRPM();
        double currentRPM = launcher.getFwLCurrentRPM();

        joined.addData("State", state.name());
        joined.addData("State Time (ms)", stateElapsedMs());
        joined.addData("Spin-Up Time (ms)", spinUpMs);
        joined.addData("Recovery Time (ms)", recoveryMs);

        joined.addData("Distance (in)", DISTANCE_IN);

        joined.addData("Target RPM", targetRPM);
        joined.addData("Current RPM", currentRPM);
        joined.addData("RPM Error", targetRPM - currentRPM);
        joined.addData("Controller Output", launcher.getFwLOutput());
        joined.addData("Mode", launcher.isFwLBangBangActive() ? "BANG-BANG" : "PIDF");
        joined.addData("Active", launcher.isActive());
        joined.addData("At Target", launcher.isFwLAtTarget());

        joined.addData("kP", kP);
        joined.addData("kI", kI);
        joined.addData("kD", kD);
        joined.addData("kF", kF);
        joined.addData("Bang-Bang Enter (RPM)", BANG_BANG_ENTER_RPM);
        joined.addData("Bang-Bang Exit (RPM)", BANG_BANG_EXIT_RPM);

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
        }
    }

    private void setState(State next) {
        state = next;
        stateEnterNs = System.nanoTime();

        if (next == State.IDLE) {
            launcher.off(); // normal rest; stop() is reserved for e-stop
        }
    }

    private double stateElapsedMs() {
        return (System.nanoTime() - stateEnterNs) / 1_000_000.0;
    }

    private void commandLauncher() {
        // Two poses DISTANCE_IN apart so autoPIDF() uses its normal LUT logic.
        Pose2D local = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0);
        Pose2D target = new Pose2D(DistanceUnit.INCH, DISTANCE_IN, 0.0, AngleUnit.DEGREES, 0.0);

        launcher.setActive(true);
        launcher.autoPIDF(local, target);
    }
}