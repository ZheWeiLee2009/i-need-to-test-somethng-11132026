package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

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
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = ".")
public class TeleOp extends OpMode {
    private LM1Robot bot;
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry);
        bot.toggleDebug(true);
    }


    @Override
    public void loop() {
        bot.update();

        if (gamepad1.y) {bot.odo.resetPosAndIMU();} //reset

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double botHeading = bot.odo.getHeading(AngleUnit.RADIANS);

        bot.setDrive(y, x, rx, botHeading, 1.0);

        if(gamepad1.bWasPressed()){ bot.startIntaking(); }
        if(gamepad1.aWasPressed()){ bot.startSpit(); }
        if(gamepad1.xWasPressed()){ bot.switchSide(); }
        if(gamepad1.yWasPressed()){ bot.odo.resetPosAndIMU(); }
        if(gamepad1.rightBumperWasPressed()){ bot.launch(); }
        if(gamepad1.leftBumperWasPressed()){ bot.interruptStop(); }

        if(gamepad1.right_trigger > 0.8){
            bot.flowerDown();
        } else {
            bot.flowerUp();
        }


//        if(gamepad1.dpad_up){bot.odo.setPosition(new Pose2D(DistanceUnit.INCH,141.5/2,141.5/2,AngleUnit.RADIANS,0));}

        // ---- Ray casting: check the robot's current field position ----

//        telemetry.addData("Drive Mode", "Standard Field Oriented");
//        telemetry.addData("Brake ","Standard Brake");
//
//        telemetry.addData("In Zone?", bot.rayCast.isInside());
//        telemetry.addData("Zone(s)", bot.rayCast.getZoneIds());
//
//        telemetry.addData("Loop Time (ms)", runtime.milliseconds());
//        telemetry.addData("Loop Time Odo (ms)", bot.odo.getLoopTime() * 0.001);
//        telemetry.addData("Frequency (Hz)", bot.odo.getFrequency());

//        telemetry.update();

        runtime.reset();
    }

    @Override
    public void stop() { bot.stop(); }
}