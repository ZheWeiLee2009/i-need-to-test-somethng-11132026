package org.firstinspires.ftc.teamcode.OpModes.Auto;

import org.firstinspires.ftc.teamcode.Configs.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Configs.pedroPathing.autoPaths.CheckSucceed;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SampleAuto extends OpMode {

    private Follower follower;
    private LM1Robot bot;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime actionTimer = new ElapsedTime();

    // Physical robot alignment position.
    private final Pose alignmentPose =
            PoseFactory.degrees().of(6, 6.5, 270);

    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 5.0;
    private static final double PATH4_PIECEWISE_SPLIT = 0.4680851064;
    private static final double VISION_SCAN_TIMEOUT_MS = 3500.0;
    private static final double SHOT_READY_TIMEOUT_MS = 8000.0;
    private static final double SHOT_FINISH_TIMEOUT_MS = 7000.0;
    private static final double FLOWER_SETTLE_TIME_MS = 350.0;
    private static final double TURRET_AIM_TOLERANCE_DEG = 5.0;
    private static final double TURRET_SETTLE_TIME_MS = 250.0;

    // Add new path/action states here. Timed actions should call beginAction() when they start.
    private enum AutoState {
        ALIGN_TO_PATH1_START, CHECK_TIPPED, PATH2, PATH3, PATH4, PATH5,
        WAIT_FOR_SHOT_READY, WAIT_FOR_SHOT_FINISH,
        DEPLOY_FLOWER, PATH6, RETRACT_FLOWER, PATH7, DONE, ABORTED
    }

    private enum ShootCheckpoint {
        START, END_PATH4, END_PATH5, END_PATH6
    }

    private AutoState state = AutoState.DONE;
    private ShootCheckpoint checkpoint = ShootCheckpoint.START;
    private boolean launchRequested = false;
    private boolean checkTippedBeforeShot = false;
    private String actionStatus = "None";

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);

        bot.odo.resetPosAndIMU();
        bot.setAutoSide(true); // turret/launcher target follows the ray-cast zone, like TeleOp's side switch

        // Same switchSide() as TeleOp. The robot starts with the (58, 84) target, so one switch
        // gives the (58, 56) target for the first shooting spot (end of PATH4). The target is
        // switched back after that shot (see continueAfterCheckpoint, END_PATH4).
        bot.switchSide();

        bot.drive.setActive(true);
        bot.intake.setActive(false);
        bot.transfer.setActive(false);
        bot.launcher.setActive(false); // Launcher stays off during INIT.
        bot.turret.setActive(true);    // Turret tracks the target during INIT.
        bot.flowerUp();

        follower = Constants.create(hardwareMap);

        // Initialize Pedro at the physical alignment position.
        follower.setPose(alignmentPose);
        follower.update();
        bot.update();

        telemetry.addLine("=== INITIALIZATION ===");
        addLocalizationTelemetry();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        bot.update();

        telemetry.addLine("=== READY ===");
        addLocalizationTelemetry();

        telemetry.addData("Target X", "%.2f", CheckSucceed.startPose().x());
        telemetry.addData("Target Y", "%.2f", CheckSucceed.startPose().y());
        telemetry.addData("Target Heading", "%.2f",
                Math.toDegrees(CheckSucceed.startPose().heading()));
        telemetry.addData("Turret Active", bot.turret.isActive());
        telemetry.addData("Launcher Active", bot.launcher.isActive());

        telemetry.update();
    }

    @Override
    public void start() {
        // Launcher remains off until a checkpoint requests a shot.
        bot.launcher.setActive(false);
        bot.turret.setActive(true);

        // Drive from the alignment pose to the PATH1 start point, then continue to PATH2.
        follower.follow(CheckSucceed.alignmenttoPATH1Start());
        beginAction("Following alignment to PATH1 start");
        state = AutoState.ALIGN_TO_PATH1_START;
    }

    @Override
    public void loop() {
        follower.update();
        bot.update();
        // Transfer only reads its sensors while active (and clears them when it shuts off),
        // so read them every loop to keep getBallCount() valid.
        bot.transfer.updateSensors();

        switch (state) {
            case ALIGN_TO_PATH1_START:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path2());
                    // Corner pickup: intake + transfer on (only starts when the robot is empty).
                    // They shut themselves off once 4 balls are loaded.
                    bot.startIntaking();
                    beginAction("Following PATH2; intaking");
                    state = AutoState.PATH2;
                }
                break;

            case CHECK_TIPPED:
                if (!bot.vision.isScanning()) {
                    boolean tipped = bot.vision.getTipped();
                    if (tipped) {
                        actionStatus = checkpoint + ": tipped; skipping shot";
                        continueAfterCheckpoint();
                    } else {
                        prepareShot();
                    }
                } else if (actionTimer.milliseconds() >= VISION_SCAN_TIMEOUT_MS) {
                    // Fail safe: stop an overlong scan, then evaluate the IDs collected so far.
                    bot.vision.setActive(false);
                    boolean tipped = bot.vision.getTipped();
                    if (tipped) {
                        actionStatus = checkpoint + ": vision timeout/tipped; skipping shot";
                        continueAfterCheckpoint();
                    } else {
                        prepareShot();
                    }
                }
                break;

            case PATH2:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path3());
                    beginAction("Following PATH3");
                    state = AutoState.PATH3;
                }
                break;

            case PATH3:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path4());
                    // Leave the corner: start spinning the flywheels now so they are at speed on
                    // arrival. Intake keeps running (until full) to finish loading on the way.
                    bot.launcher.setActive(true);
                    beginAction("Following PATH4; launcher spinning up");
                    state = AutoState.PATH4;
                }
                break;

            case PATH4:
                if (!follower.isBusy()) {
                    follower.hold(follower.pose());
                    // Second conditional shot checkpoint: end of PATH4.
                    beginShotCheckpoint(ShootCheckpoint.END_PATH4, true);
                }
                break;

            case PATH5:
                if (!follower.isBusy()) {
                    follower.hold(follower.pose());
                    // Unconditional shot checkpoint: end of PATH5.
                    beginShotCheckpoint(ShootCheckpoint.END_PATH5, false);
                }
                break;

            case WAIT_FOR_SHOT_READY:
                // Refresh sensors while transfer is inactive; bot.launch() checks this ball count.
                bot.transfer.updateSensors();

                if (bot.transfer.getBallCount() == 0) {
                    actionStatus = checkpoint + ": no ball detected; skipping shot";
                    continueAfterCheckpoint();
                } else if (isShooterReady()) {
                    bot.launch();
                    launchRequested = bot.transfer.isActive() && bot.intake.isActive();

                    if (launchRequested) {
                        beginAction("Shooting at " + checkpoint + "; waiting for transfer and intake");
                        state = AutoState.WAIT_FOR_SHOT_FINISH;
                    } else {
                        abortShot("Launch request was not accepted at " + checkpoint);
                    }
                } else if (actionTimer.milliseconds() >= SHOT_READY_TIMEOUT_MS) {
                    abortShot("Turret/launcher readiness timed out at " + checkpoint);
                }
                break;

            case WAIT_FOR_SHOT_FINISH:
                if (launchRequested && !bot.transfer.isActive() && !bot.intake.isActive()) {
                    launchRequested = false;
                    actionStatus = checkpoint + ": shot finished";
                    continueAfterCheckpoint();
                } else if (actionTimer.milliseconds() >= SHOT_FINISH_TIMEOUT_MS) {
                    abortShot("Shooting did not finish before timeout at " + checkpoint);
                }
                break;

            case DEPLOY_FLOWER:
                if (actionTimer.milliseconds() >= FLOWER_SETTLE_TIME_MS) {
                    follower.follow(CheckSucceed.path6());
                    bot.startIntaking();   // flower pickup (only starts when the robot is empty)
                    beginAction("Following PATH6; flower deployed, intaking");
                    state = AutoState.PATH6;
                }
                break;

            case PATH6:
                if (!follower.isBusy()) {
                    follower.hold(follower.pose());
                    bot.flowerUp();
                    beginAction("Retracting flower after PATH6");
                    state = AutoState.RETRACT_FLOWER;
                }
                break;

            case RETRACT_FLOWER:
                if (actionTimer.milliseconds() >= FLOWER_SETTLE_TIME_MS) {
                    // Unconditional shot checkpoint: end of PATH6, after the flower retracts.
                    beginShotCheckpoint(ShootCheckpoint.END_PATH6, false);
                }
                break;

            case PATH7:
                if (!follower.isBusy()) {
                    bot.launcher.setActive(false);
                    bot.turret.setActive(false);
                    actionStatus = "Complete";
                    state = AutoState.DONE;
                }
                break;

            case DONE:
            case ABORTED:
                break;
        }

        telemetry.addData("Auto State", state);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Action", actionStatus);
        telemetry.addData("Action Timer", "%.2f s", actionTimer.seconds());
        telemetry.addData("Shoot Checkpoint", checkpoint);
        telemetry.addData("Tipped", bot.vision.isScanning()
                ? "Waiting for scan" : String.valueOf(bot.vision.getTipped()));
        telemetry.addData("Vision Scanning", bot.vision.isScanning());
        telemetry.addData("AprilTag IDs", bot.vision.getTagIds());
        telemetry.addData("Tipped Check Required", checkTippedBeforeShot);
        telemetry.addData("Launch Requested", launchRequested);
        telemetry.addData("Ball Count", bot.transfer.getBallCount());
        telemetry.addData("Turret Active", bot.turret.isActive());
        telemetry.addData("Turret Angle", "%.1f deg", bot.turret.getAngle());
        telemetry.addData("Turret Aim Target", "%.1f deg", bot.turret.getAimTargetAngleDeg());
        telemetry.addData("Turret Target Clipped", bot.turret.isAimTargetClipped());
        telemetry.addData("Launcher Active", bot.launcher.isActive());
        telemetry.addData("Launcher At Target", bot.launcher.isAtTarget());
        telemetry.addData("Transfer Active", bot.transfer.isActive());
        telemetry.addData("Intake Active", bot.intake.isActive());

        if (state == AutoState.PATH4) {
            double path4T = follower.parametricCompletion();
            telemetry.addData("Path 4 Distance Progress", "%.1f%%", follower.completion() * 100);
            telemetry.addData("Path 4 Parameter (t)", "%.4f", path4T);
            telemetry.addData("Path 4 Heading Segment",
                    path4T < PATH4_PIECEWISE_SPLIT ? "1 (Constant)" : "2 (Linear)");
        }

        addLocalizationTelemetry();
        telemetry.update();
    }

    // Start a fresh tipped scan for a conditional shot, or prepare the unconditional shot directly.
    private void beginShotCheckpoint(ShootCheckpoint nextCheckpoint, boolean checkTipped) {
        checkpoint = nextCheckpoint;
        checkTippedBeforeShot = checkTipped;
        launchRequested = false;

        // Arrived: stop any intake still running. Sensors are re-read at the top of loop().
        if (bot.intake.isActive() || bot.transfer.isActive()) {
            bot.interruptStop();
        }

        if (checkTippedBeforeShot) {
            bot.vision.setActive(true);
            beginAction("Scanning AprilTags at " + checkpoint);
            state = AutoState.CHECK_TIPPED;
        } else {
            prepareShot();
        }
    }

    // Activate the launcher only when this checkpoint actually needs a shot.
    private void prepareShot() {
        bot.transfer.updateSensors();

        if (bot.transfer.getBallCount() == 0) {
            actionStatus = checkpoint + ": no ball detected; skipping shot";
            continueAfterCheckpoint();
            return;
        }

        bot.turret.setActive(true);
        bot.launcher.setActive(true);
        beginAction("Waiting for turret and launcher readiness at " + checkpoint);
        state = AutoState.WAIT_FOR_SHOT_READY;
    }

    // Continue to the next path/action after a checkpoint is skipped or the shot finishes.
    private void continueAfterCheckpoint() {
        launchRequested = false;

        switch (checkpoint) {
            case START:
                follower.follow(CheckSucceed.path2());
                beginAction("Following PATH2");
                state = AutoState.PATH2;
                break;

            case END_PATH4:
                // Leaving the first shooting spot: switch the turret/launcher target back to
                // (58, 84) for the shots at the end of PATH5 and PATH6 (same as TeleOp's switchSide).
                bot.switchSide();
                follower.follow(CheckSucceed.path5());
                beginAction("Following PATH5");
                state = AutoState.PATH5;
                break;

            case END_PATH5:
                bot.flowerDown();
                beginAction("Deploying flower before PATH6");
                state = AutoState.DEPLOY_FLOWER;
                break;

            case END_PATH6:
                bot.launcher.setActive(false);
                follower.follow(CheckSucceed.path7());
                beginAction("Following PATH7");
                state = AutoState.PATH7;
                break;
        }
    }

    // Require a valid turret aim command and both flywheels to be within their RPM tolerances.
    private boolean isShooterReady() {
        double turretTarget = bot.turret.getAimTargetAngleDeg();
        boolean turretReady = bot.turret.isActive()
                && Double.isFinite(turretTarget)
                && !bot.turret.isAimTargetClipped()
                && Math.abs(bot.turret.getAngle() - turretTarget) <= TURRET_AIM_TOLERANCE_DEG;

        return turretReady
                && actionTimer.milliseconds() >= TURRET_SETTLE_TIME_MS
                && bot.launcher.isAtTarget();
    }

    // Start/reset the timer whenever a timed action begins.
    private void beginAction(String status) {
        actionStatus = status;
        actionTimer.reset();
    }

    // Stop the shooting mechanisms and hold position if a launch check or timeout fails.
    private void abortShot(String reason) {
        bot.interruptStop();
        bot.launcher.setActive(false);
        bot.turret.setActive(false);
        bot.vision.setActive(false);
        bot.flowerUp();
        follower.hold(follower.pose());
        launchRequested = false;
        actionStatus = "ABORTED: " + reason;
        state = AutoState.ABORTED;
    }

    // Check whether Pedro has reached the hold target.
    private boolean atStartPose() {
        Pose current = follower.pose();
        Pose target = CheckSucceed.startPose();

        double distance = Math.hypot(
                current.x() - target.x(),
                current.y() - target.y()
        );

        double headingError = Math.atan2(
                Math.sin(current.heading() - target.heading()),
                Math.cos(current.heading() - target.heading())
        );

        double headingErrorDeg = Math.abs(Math.toDegrees(headingError));

        telemetry.addData("Hold Distance", "%.2f in", distance);
        telemetry.addData("Hold Heading Error", "%.2f deg", headingErrorDeg);

        return distance <= POSITION_TOLERANCE
                && headingErrorDeg <= HEADING_TOLERANCE_DEG;
    }

    // Display both localization systems.
    private void addLocalizationTelemetry() {
        Pose current = follower.pose();

        telemetry.addLine("=== PEDRO LOCALIZATION ===");
        telemetry.addData("Pedro X", "%.2f", current.x());
        telemetry.addData("Pedro Y", "%.2f", current.y());
        telemetry.addData("Pedro Heading", "%.2f",
                Math.toDegrees(current.heading()));

        telemetry.addData("Ray-cast zone", bot.rayCast.getZoneIds());
        telemetry.addData("Target goal", bot.isAudience() ? "CLOSER (58, 56)" : "FURTHER (58, 84)");

        telemetry.addLine("=== BOT.ODO LOCALIZATION (FIELD FRAME) ===");
        telemetry.addData("Odo X (right +)", "%.2f", bot.drive.getFieldX());
        telemetry.addData("Odo Y (forward +)", "%.2f", bot.drive.getFieldY());
        telemetry.addData("Odo Heading", "%.2f", Math.toDegrees(bot.drive.getHeadingRad()));
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
        }
        if (bot != null) {
            bot.stop();
        }
    }
}