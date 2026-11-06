package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

/**
 * Standard field-oriented TeleOp drive, with brake-mode drive motors and on-field-zone
 * telemetry (via RayCasting).
 *
 * gamepad1 left stick:  drive (field-oriented, forward/strafe)
 * gamepad1 right stick: turn (X axis)
 * gamepad1 Y:           reset heading / pose (call when facing a known field direction)
 * gamepad1 dpad down:   toggle turret field-pose tracking active/inactive
 * gamepad1 dpad left/right: manually nudge turret when tracking is inactive
 * gamepad1 B:           intake on
 * gamepad1 A:           intake off
 * gamepad1 dpad up:     Limelight AprilTag scan (camera on for a short window only)
 * gamepad1 back:        toggle launcher on/off (starts ON when Start is pressed)
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = ".")
public class TeleOp extends OpMode {
    private LM1Robot bot;
    private final ElapsedTime runtime = new ElapsedTime();
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;


    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);
        bot.setDebug(true);
    }


    @Override
    public void start() {
        bot.launcher.setActive(true); // launcher only runs after Start is pressed
    }

    @Override
    public void loop() {
        bot.update();

//        if (gamepad1.y) {bot.odo.resetPosAndIMU();} //reset

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        bot.setDrive(y, x, rx, 1.0);

        if(gamepad1.bWasPressed()){ bot.startIntaking(); }
        if(gamepad1.aWasPressed()){ bot.startSpit(); }
        if(gamepad1.xWasPressed()){ bot.switchSide(); }
        if(gamepad1.yWasPressed()){ bot.drive.resetOdo(); }
        if (gamepad1.backWasPressed()) { bot.launcher.setActive(!bot.launcher.isActive()); }

        if (gamepad1.dpad_down && !lastDpadDown) {
            bot.turret.setActive(!bot.turret.isActive());
        }

        if (gamepad1.dpadUpWasPressed()) {
            bot.vision.setActive(!bot.vision.isActive());
        }

        if (!bot.turret.isActive()) {
            if (gamepad1.dpad_left && !lastDpadLeft) bot.turret.nudge(-5.0);
            if (gamepad1.dpad_right && !lastDpadRight) bot.turret.nudge(5.0);
        }
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        if(gamepad1.rightBumperWasPressed()){ bot.launch(); }
        if(gamepad1.leftBumperWasPressed()){ bot.interruptStop(); }

        if(gamepad1.right_trigger > 0.8){
            bot.flowerDown();
        } else {
            bot.flowerUp();
        }

        runtime.reset();
    }

    @Override
    public void stop() { bot.stop(); }
}