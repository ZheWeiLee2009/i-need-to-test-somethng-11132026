package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.TurretConstants;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

/**
 * Focused test OpMode for the turret's fixed field-pose aiming.
 *
 * <p>The target is read from TurretConstants every loop, so TARGET_X_IN and TARGET_Y_IN can
 * be adjusted from the Configurables dashboard while this OpMode is running (when available).
 * The robot should start from a known Pinpoint field pose that uses the same coordinate frame
 * as the target.
 *
 * <p>This OpMode intentionally does NOT call LM1Robot.update(), because that also runs launcher
 * logic. Instead it updates only the drivetrain's Pinpoint reader and the turret subsystem.
 *
 * <p>Gamepad 1 D-pad Down toggles turret aiming active/inactive. There are no manual turret
 * movement controls in this test.
 */
@TeleOp(name = "Turret Pose Aim Test", group = "9 - Utility")
public class TurretPoseAimTest extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private LM1Robot bot;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // Reuse the robot's normal hardware mapping, turret calibration, and subsystem setup.
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);

        // Start inactive. TurretSubSystem.setup() also places it at its forward-facing 180 deg.
        bot.turret.setActive(false);

        telemetry.addLine("Turret Pose Aim Test initialized");
        telemetry.addLine("D-pad Down: toggle turret aiming active/inactive");
        telemetry.addLine("Target: Configurables -> TurretConstants TARGET_X_IN / TARGET_Y_IN");
        telemetry.addLine("No manual turret movement controls are enabled.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Refresh Pinpoint without running the full robot loop / launcher logic.
        bot.drive.update();


        if (gamepad1.y) {bot.odo.resetPosAndIMU();} //reset

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        bot.drive.setSTDDrive(y, x, rx, 0.9);


        // Keep the target independent of the launcher's scoring/audience target.
        // Rebuild it each loop so dashboard edits to the constants are picked up immediately.
        bot.turret.setTargetPose(new Pose2D(
                DistanceUnit.INCH,
                TurretConstants.TARGET_X_IN,
                TurretConstants.TARGET_Y_IN,
                AngleUnit.RADIANS,
                0.0
        ));

        // The only driver control is enabling/disabling the pose-aim update.
        if (gamepad1.dpad_down && !lastDpadDown) {
            bot.turret.setActive(!bot.turret.isActive());
        }
        lastDpadDown = gamepad1.dpad_down;

        Pose2D robotPose = new Pose2D(
                DistanceUnit.INCH,
                bot.odo.getPosX(DistanceUnit.INCH),
                bot.odo.getPosY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                bot.odo.getHeading(AngleUnit.RADIANS)
        );
        bot.turret.setRobotPose(robotPose);

        // update() is called only while the turret is active; while inactive, the servos hold.
        if (bot.turret.isActive()) {
            bot.turret.update();
        }

        Pose2D target = bot.turret.getTargetPose();
        telemetry.addData("Turret active", bot.turret.isActive());
        telemetry.addData("Pinpoint X (in)", "%.2f", robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("Pinpoint Y (in)", "%.2f", robotPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f", robotPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Target X (in)", "%.2f", target.getX(DistanceUnit.INCH));
        telemetry.addData("Target Y (in)", "%.2f", target.getY(DistanceUnit.INCH));
        telemetry.addData("Turret commanded angle (deg)", "%.2f", bot.turret.getAngle());
        telemetry.addData("Calculated requested angle (deg)", "%.2f", bot.turret.getRawAimAngleDeg());
        telemetry.addData("Limited target angle (deg)", "%.2f", bot.turret.getAimTargetAngleDeg());
        telemetry.addData("Target clipped by travel limits", bot.turret.isAimTargetClipped());
        telemetry.addLine("D-pad Down: toggle active/inactive");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (bot != null) {
            bot.stop();
        }
    }
}
