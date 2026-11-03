package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PracticeBot;

/**
 * Standard field-oriented TeleOp drive, with brake-mode drive motors and on-field-zone
 * telemetry (via RayCasting).
 *
 * gamepad1 left stick:  drive (field-oriented, forward/strafe)
 * gamepad1 right stick: turn (X axis)
 * gamepad1 Y:           reset heading / pose (call when facing a known field direction)
 * gamepad1 dpad up:     re-center pose to the middle of the field
 * gamepad1 B:           intake on
 * gamepad1 A:           intake off
 */

@Disabled
@TeleOp(name = "FO TeleOp Drive", group = ".")
public class FOTeleOpDrive extends OpMode {
    private PracticeBot bot;

    private final ElapsedTime runtime = new ElapsedTime();

    double maxTotalCurrent = 0.0;

    @Override
    public void init() {
        bot = new PracticeBot(hardwareMap, runtime);
        // telemetry transmission limiter
        telemetry.setMsTransmissionInterval(250);
        telemetry.addLine("Initialized");
        telemetry.update();

        bot.drive.setZeroPowerBehavior(bot.driveMotors, DcMotor.ZeroPowerBehavior.BRAKE);
        bot.drive.setActive(true); // odo true
    }


    @Override
    public void loop() {
        bot.update();

        if (gamepad1.y) {bot.odo.resetPosAndIMU();} //reset

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        double botHeading = bot.odo.getHeading(AngleUnit.RADIANS);

        bot.drive.setFODDrive(y, x, rx, botHeading, 1.0);

        if(gamepad1.b){bot.intake.on();}
        if(gamepad1.a){bot.intake.off();}

        if(gamepad1.dpad_up){bot.odo.setPosition(new Pose2D(DistanceUnit.INCH,141.5/2,141.5/2,AngleUnit.RADIANS,0));}

        // ---- Ray casting: check the robot's current field position ----
        Pose2D pos = bot.odo.getPosition();
        double robotX = pos.getX(DistanceUnit.INCH);
        double robotY = pos.getY(DistanceUnit.INCH);

        bot.rayCast.update(robotX, robotY);

        telemetry.addData("Drive Mode", "Standard Field Oriented");
        telemetry.addData("Brake ","Standard Brake");

        telemetry.addData("Robot Pos (X, Y)", "%.1f, %.1f", robotX, robotY);
        telemetry.addData("In Zone?", bot.rayCast.isInside());
        telemetry.addData("Zone(s)", bot.rayCast.getZoneIds());

        telemetry.addData("Loop Time (ms)", runtime.milliseconds());
        telemetry.addData("Loop Time Odo (ms)", bot.odo.getLoopTime() * 0.001);
        telemetry.addData("Frequency (Hz)", bot.odo.getFrequency());

        telemetry.update();

        runtime.reset();
    }
}