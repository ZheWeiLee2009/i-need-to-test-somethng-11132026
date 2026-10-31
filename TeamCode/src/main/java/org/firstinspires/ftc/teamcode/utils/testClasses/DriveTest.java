package org.firstinspires.ftc.teamcode.utils.testClasses;

import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.MAX_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.VELOCITY_CUTOFF;
import static org.firstinspires.ftc.teamcode.Configs.roboConfigs.DriveConstants.instantStopConstant;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Configs.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.PracticeBot;

/**
 * gamepad1 A: next drive state
 * gamepad1 Y: reset heading / pose
 */
@Utility(name = "Utility", description = "Gamepad-driven mecanum drive diagnostic")
public class DriveTest extends OpMode {

    private enum State {
        ROBOT_CENTRIC("Robot-Centric", false),
        ROBOT_CENTRIC_INSTANT_STOP("Robot-Centric + Instant Stop", true),
        FIELD_CENTRIC("Field-Centric", true),
        FIELD_CENTRIC_INSTANT_STOP("Field-Centric + Instant Stop", true),
//        PEDRO_FIELD("Pedro Field-Centric", false),
//        PEDRO_ROBO("Pedro Robo-Centric", false)
        ;

        final String label;
        final boolean driveActive;

        State(String label, boolean driveActive) {
            this.label = label;
            this.driveActive = driveActive;
        }

        State next() {
            State[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    // Current reads are slow hub commands, so poll at an interval. 0 = every loop.
    private static final long CURRENT_POLL_MS = 100;

    private PracticeBot bot;
    private Follower follower;
    private Telemetry joined;

    private State state = State.FIELD_CENTRIC;
    private boolean aPressedLast = false;

    // Reset once per loop; PracticeBot uses it.
    private final ElapsedTime runtime = new ElapsedTime();

    private double pedroDriveTimeMs = 0;

    private double totalCurrent = 0;
    private double maxTotalCurrent = 0;
    private long lastCurrentPollNs = 0;

    private long lastLoopStartNs = 0;
    private double loopPeriodMs = 0;

    @Override
    public void init() {
        bot = new PracticeBot(hardwareMap, runtime);
//        follower = Constants.createFollower(hardwareMap);

        // Only the drive may be active in this test (it is toggled per state in loop());
        // keep every other subsystem idle so bot.update() ticks nothing else. Add any other
        // PracticeBot subsystems here. NOTE: change if PracticeBot names the field differently.
        bot.launcher.setActive(false);

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

        if (gamepad1.a && !aPressedLast) {
            state = state.next();
            maxTotalCurrent = 0;
        }
        aPressedLast = gamepad1.a;

        if (gamepad1.y) {
            bot.odo.resetPosAndIMU();
            follower.setPose(new Pose(0, 0, 0));
        }

        bot.drive.setActive(state.driveActive);
        bot.update();

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double heading = bot.odo.getHeading(AngleUnit.RADIANS);

        long start;

        switch (state) {
            case ROBOT_CENTRIC:
                bot.drive.setSTDDrive(y, x, rx, 1.0);
                break;

            case ROBOT_CENTRIC_INSTANT_STOP:
                bot.drive.instantStopDrive(y, x, rx, 1.0);
                break;

            case FIELD_CENTRIC:
                bot.drive.setFODDrive(y, x, rx, heading, 1.0);
                break;

            case FIELD_CENTRIC_INSTANT_STOP:
                bot.drive.instantStopFODDrive(y, x, rx, heading, 1.0);
                break;

//            case PEDRO_FIELD:
//                start = System.nanoTime();
//
//                follower.manual(ManualDrive.fieldCentric(
//                        -gamepad1.left_stick_y,
//                        gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x,
//                        follower.pose().heading()
//                ));
//                follower.update();
//
//                pedroDriveTimeMs = (System.nanoTime() - start) / 1_000_000.0;
//                break;
//            case PEDRO_ROBO:
//                start = System.nanoTime();
//
//                follower.manual(
//                        -gamepad1.left_stick_y,
//                        gamepad1.left_stick_x,
//                        -gamepad1.right_stick_x
//                );
//                follower.update();
//
//                pedroDriveTimeMs = (System.nanoTime() - start) / 1_000_000.0;
//                break;
        }

        if (loopStartNs - lastCurrentPollNs >= CURRENT_POLL_MS * 1_000_000L) {
            totalCurrent =
                    bot.fl.getCurrent(CurrentUnit.AMPS)
                            + bot.fr.getCurrent(CurrentUnit.AMPS)
                            + bot.bl.getCurrent(CurrentUnit.AMPS)
                            + bot.br.getCurrent(CurrentUnit.AMPS);
            maxTotalCurrent = Math.max(maxTotalCurrent, totalCurrent);
            lastCurrentPollNs = loopStartNs;
        }

        joined.addData("State", state.label);

        joined.addData("Current (A)", totalCurrent);
        joined.addData("Max Current (A)", maxTotalCurrent);

        joined.addData("Instant Stop Constant", instantStopConstant);
        joined.addData("Velocity Cutoff", VELOCITY_CUTOFF);
        joined.addData("Max Reverse", MAX_REVERSE_POWER);

//        if (state == State.PEDRO_FIELD  || state == State.PEDRO_ROBO) {
//            Pose pose = follower.pose();
//            joined.addData("Pedro X", pose.x());
//            joined.addData("Pedro Y", pose.y());
//            joined.addData("Pedro Heading (deg)", Math.toDegrees(pose.heading()));
//            joined.addData("Pedro Manual + Update (ms)", pedroDriveTimeMs);
//        } else {
//            joined.addData("X (in)", bot.odo.getPosX(DistanceUnit.INCH));
//            joined.addData("Y (in)", bot.odo.getPosY(DistanceUnit.INCH));
//            joined.addData("Heading (deg)", bot.odo.getHeading(AngleUnit.DEGREES));
//        }

        joined.addData("Loop Time (ms)", loopPeriodMs);
        joined.addData("Loop Frequency (Hz)", 1000.0 / Math.max(loopPeriodMs, 0.001));
        joined.addData("Odo Loop (ms)", bot.odo.getLoopTime() * 0.001);
        joined.addData("Odo Frequency (Hz)", bot.odo.getFrequency());

        joined.update();
        runtime.reset();
    }

    @Override
    public void stop() {
        if (bot != null) {
            bot.drive.setSTDDrive(0, 0, 0, 1.0);
        }
    }
}