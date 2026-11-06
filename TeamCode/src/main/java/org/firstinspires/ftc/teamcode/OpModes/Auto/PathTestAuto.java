package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Configs.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Configs.pedroPathing.autoPaths.CheckSucceed;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;


@Autonomous(name = "Path Test (FSM)", group = "Test")
public class PathTestAuto extends OpMode {

    private LM1Robot bot;
    private Follower follower;

    private final ElapsedTime runtime = new ElapsedTime();    // handed to LM1Robot, same as TeleOp
    private final ElapsedTime stateTimer = new ElapsedTime(); // ours, for FSM waits
    private int state = 0;

    private final Pose startPose = new Pose(141.5/2 - 12.5/2, 141.5/2 - 8, 270);



    // ---- FSM ----

    private void setState(int s) {
        state = s;
        stateTimer.reset();
    }

    private void updateFsm() {
        switch (state) {
            case 0:
                follower.follow(CheckSucceed.path1());
                setState(1);
                break;
            default:
                break; // idle
        }
    }

    // ---- OpMode ----

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);
        bot.setDebug(false);

        follower = Constants.create(hardwareMap);

        bot.odo.resetPosAndIMU();
        follower.setPose(new Pose(0,0,0));

        // Set the start pose ONCE, here. Never again (not in init_loop, not in start()),
        // so you can push the robot around during init and the odometry keeps tracking it.
        setStartPose(startPose);
    }

    private void setStartPose(Pose pose) {
        bot.odo.setPosition(new Pose2D(
                DistanceUnit.INCH, pose.x(), pose.y(),
                AngleUnit.RADIANS, 19));

        follower.setPose(pose);
        follower.update();
    }

    @Override
    public void init_loop() {
        // Keep both localizers ticking while you move the robot around. No resets here.
        bot.update();
        follower.update();

        telemetry.addLine("Move robot freely. Pose is tracked, not reset at start.");
        telemetry.addData("X", "%.1f", bot.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", "%.1f", bot.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(bot.odo.getHeading(AngleUnit.RADIANS)));

        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Intentionally NO setPose / resetPosAndIMU here: keep whatever pose init tracked.
        runtime.reset();
        setState(0);
    }

    @Override
    public void loop() {
        bot.update();      // LM1Robot subsystems, same as TeleOp
        follower.update(); // Pedro: must run every loop
        updateFsm();

        telemetry.addData("State", state);
        telemetry.addData("State time (s)", "%.1f", stateTimer.seconds());
        telemetry.addData("X", "%.1f", follower.pose().x());
        telemetry.addData("Y", "%.1f", follower.pose().y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Mode", follower.mode());
        telemetry.addData("Completion", "%.2f", follower.completion());
        telemetry.addData("Remaining dist", "%.1f", follower.remainingDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        bot.stop();
    }
}