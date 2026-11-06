
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    // Physical robot alignment position.
    private final Pose alignmentPose =
            PoseFactory.degrees().of(6, 6.5, 270);

    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE_DEG = 5.0;

    private enum AutoState {
        GO_TO_PATH_START, PATH2, PATH3, PATH4, PATH5, PATH6, PATH7, DONE
    }

    private AutoState state = AutoState.DONE;

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);

        bot.drive.setActive(true);
        bot.intake.setActive(false);
        bot.transfer.setActive(false);
        bot.launcher.setActive(false);
        bot.turret.setActive(true);

        follower = Constants.create(hardwareMap);

        // Initialize Pedro at the physical alignment position.
        follower.setPose(alignmentPose);
        follower.update();
        bot.odo.update();

        telemetry.addLine("=== INITIALIZATION ===");
        addLocalizationTelemetry();
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        bot.odo.update();

        telemetry.addLine("=== READY ===");
        addLocalizationTelemetry();

        telemetry.addData("Target X", "%.2f", CheckSucceed.startPose().x());
        telemetry.addData("Target Y", "%.2f", CheckSucceed.startPose().y());
        telemetry.addData("Target Heading", "%.2f",
                Math.toDegrees(CheckSucceed.startPose().heading()));

        telemetry.update();
    }

    @Override
    public void start() {
        // Move to Pedro's starting position before following the paths.
        follower.hold(CheckSucceed.startPose());
        state = AutoState.GO_TO_PATH_START;
    }

    @Override
    public void loop() {
        follower.update();
        bot.odo.update();

        switch (state) {
            case GO_TO_PATH_START:
                if (atStartPose()) {
                    follower.follow(CheckSucceed.path2());
                    state = AutoState.PATH2;
                }
                break;

            case PATH2:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path3());
                    state = AutoState.PATH3;
                }
                break;

            case PATH3:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path4());
                    state = AutoState.PATH4;
                }
                break;

            case PATH4:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path5());
                    state = AutoState.PATH5;
                }
                break;

            case PATH5:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path6());
                    state = AutoState.PATH6;
                }
                break;

            case PATH6:
                if (!follower.isBusy()) {
                    follower.follow(CheckSucceed.path7());
                    state = AutoState.PATH7;
                }
                break;

            case PATH7:
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("Auto State", state);
        telemetry.addData("Follower Busy", follower.isBusy());

        addLocalizationTelemetry();
        telemetry.update();
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

        telemetry.addLine("=== BOT.ODO LOCALIZATION ===");
        telemetry.addData("Odo X", "%.2f",
                bot.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Odo Y", "%.2f",
                bot.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Odo Heading", "%.2f",
                Math.toDegrees(bot.odo.getHeading(AngleUnit.RADIANS)));
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
