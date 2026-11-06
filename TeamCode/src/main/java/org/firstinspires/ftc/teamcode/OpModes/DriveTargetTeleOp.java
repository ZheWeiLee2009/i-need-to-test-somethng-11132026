package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

/**
 * Drive-only TeleOp with two buttons that set the turret's aim target.
 *
 * gamepad1 left stick:  drive (field-oriented, forward/strafe)
 * gamepad1 right stick: turn (X axis)
 * gamepad1 X:           set target to TARGET_A (58.14, 59.5)
 * gamepad1 B:           set target to TARGET_B (58.14, 82.5)
 * gamepad1 back:        toggle launcher on/off (starts ON when Start is pressed)
 *
 * The turret tracks the target automatically once the OpMode starts.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Drive + Target TeleOp", group = ".")
public class DriveTargetTeleOp extends OpMode {
    private static final Pose2D TARGET_A =
            new Pose2D(DistanceUnit.INCH, 58.14, 59.5, AngleUnit.DEGREES, 0.0);
    private static final Pose2D TARGET_B =
            new Pose2D(DistanceUnit.INCH, 58.14, 82.5, AngleUnit.DEGREES, 0.0);

    private LM1Robot bot;
    private Pose2D currentTarget = TARGET_A; // default until a button is pressed
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);
        bot.setDebug(false);
    }

    @Override
    public void start() {
        bot.launcher.setActive(true); // launcher only runs after Start is pressed
        bot.turret.setActive(true);
    }

    @Override
    public void loop() {
        bot.update();

        bot.setDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1.0);

        if (gamepad1.yWasPressed()){ bot.drive.resetOdo();}
        if (gamepad1.xWasPressed()) currentTarget = TARGET_A;
        if (gamepad1.bWasPressed()) currentTarget = TARGET_B;
        if (gamepad1.backWasPressed()) { bot.launcher.setActive(!bot.launcher.isActive()); }

        // bot.update() hands the turret LM1Robot's own (unset) target every loop, so set ours
        // afterwards and run the turret again with it.
        bot.turret.setRobotPose(bot.drive.getFieldPose());
        bot.turret.setTargetPose(currentTarget);
        bot.turret.update();

        runtime.reset();

        telemetry.addData("raw X (fwd)", bot.odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("raw Y (left)", bot.odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading deg", Math.toDegrees(bot.odo.getHeading(AngleUnit.RADIANS)));
        telemetry.addData("field X", bot.drive.getFieldX());
        telemetry.addData("field Y", bot.drive.getFieldY());
        telemetry.update();

    }

    @Override
    public void stop() { bot.stop(); }
}