package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static com.pedropathing.api.Paths.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Configs.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SampleAuto extends OpMode {

    private Follower follower; // Add this
    private LM1Robot bot;
    private final ElapsedTime runtime = new ElapsedTime();    // handed to LM1Robot, same as TeleOp

    private final PoseFactory p = PoseFactory.degrees(); // add this
    private final Pose2D startPose2d = new Pose2D(DistanceUnit.INCH, 141.5/2, 141.5/2, AngleUnit.DEGREES,0);
    private Pose currPose;
    private final Pose toPose = p.of(24, 24, 0);


    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);

        bot.odo.resetPosAndIMU();
        bot.odo.setPosition(startPose2d);
        bot.odo.update();

        bot.drive.setActive(true);
        bot.intake.setActive(false);
        bot.transfer.setActive(false);
        bot.launcher.setActive(false);
        bot.turret.setActive(true);

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(bot.odo.getPosition().getX(DistanceUnit.INCH),bot.odo.getPosition().getX(DistanceUnit.INCH),bot.odo.getHeading(AngleUnit.DEGREES)));
        follower.update();
    }


    @Override
    public void init_loop() {
        bot.update();
        follower.update();

        telemetry.addLine("Move robot freely. Pose is tracked, not reset at start.");
        telemetry.addData("Follower X", "%.1f", bot.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Follower Y", "%.1f", bot.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Follower Heading (deg)", "%.1f", Math.toDegrees(bot.odo.getHeading(AngleUnit.RADIANS)));

        telemetry.addData("Follower X", "%.1f", follower.pose().x());
        telemetry.addData("Follower Y", "%.1f", follower.pose().y());
        telemetry.addData("Follower Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();

    }

    @Override
    public void start() {
        bot.update();
        follower.update();

        follower.hold(toPose);
    }

    @Override
    public void loop() {
        bot.update();
        follower.update();

        telemetry.addLine("Move robot freely. Pose is tracked, not reset at start.");
        telemetry.addData("Follower X", "%.1f", bot.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Follower Y", "%.1f", bot.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Follower Heading (deg)", "%.1f", Math.toDegrees(bot.odo.getHeading(AngleUnit.RADIANS)));

        telemetry.addData("Follower X", "%.1f", follower.pose().x());
        telemetry.addData("Follower Y", "%.1f", follower.pose().y());
        telemetry.addData("Follower Heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
        telemetry.update();

    }

    @Override
    public void stop(){
        bot.stop();
    }
}