package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.LauncherConstants;
import org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem;
import org.firstinspires.ftc.teamcode.utils.customMath;

import java.util.Arrays;
import java.util.List;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */

@Disabled
@TeleOp(name = "PracticeDrive", group = "teleOp")
public class MaxPracticeDrive extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotorEx fwR, fwL;
    List<DcMotorEx> flywheelMotors;
    LauncherSubSystem launcher;
    Pose2D local;
    Pose2D target = new Pose2D(DistanceUnit.INCH, 58.5, 83.0, AngleUnit.DEGREES, 0.0);
    DigitalChannel rightBallSensor1, rightBallSensor2, rightBallSensor3, rightBallSensor4;
    DigitalChannel leftBallSensor1, leftBallSensor2, leftBallSensor3, leftBallSensor4;

    //SERVOS
    Servo blocker;
    Servo LT;
    Servo cameraServo;
    //    Servo RT;
    Servo ramp;
    GoBildaPinpointDriver pinpoint;

    DcMotorEx intakeDrive;
    DcMotorEx transfer;
    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    double shooterPower = 0.7;

    double backshooterPower = 0.7;
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    boolean lastDpadLeft = false;
    boolean lastDpadRight = false;
    boolean lastB = false;
    boolean lastX = false;
    double testvalue = .5;
    boolean shooting = false;
    ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime shootTimer = new ElapsedTime();

    double frontTargetVelocity = 2400;
    double backTargetVelocity = 3600;
    double backShootVelocity = frontTargetVelocity;
    double frontShootVelocity = backTargetVelocity;


    @Override
    public void init() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        intakeDrive = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        fwL= hardwareMap.get(DcMotorEx.class, "fwL"); //port 3
        fwR = hardwareMap.get(DcMotorEx.class, "fwR"); // prot 2

//        fwL.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotors = Arrays.asList(fwL, fwR);
        launcher = new LauncherSubSystem(flywheelMotors); // NOTE: change if PracticeBot names the field differently

        launcher.setup();
//        RT = hardwareMap.get(Servo.class, "rightTurret");
        LT = hardwareMap.get(Servo.class, "turretL");
        blocker = hardwareMap.get(Servo.class, "gate");
        ramp = hardwareMap.get(Servo.class, "flower");
        cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        rightBallSensor1 = hardwareMap.get(DigitalChannel.class, "ballSensor1R");
        rightBallSensor3 = hardwareMap.get(DigitalChannel.class, "ballSensor2R");
        rightBallSensor2 = hardwareMap.get(DigitalChannel.class, "ballSensor3R");
        rightBallSensor3 = hardwareMap.get(DigitalChannel.class, "ballSensor4R");
        leftBallSensor1 = hardwareMap.get(DigitalChannel.class, "ballSensor1L");
        leftBallSensor3 = hardwareMap.get(DigitalChannel.class, "ballSensor2L");
        leftBallSensor2 = hardwareMap.get(DigitalChannel.class, "ballSensor3L");
        leftBallSensor3 = hardwareMap.get(DigitalChannel.class, "ballSensor4L");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*
        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
*/
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Optional: Set offsets and units (check your specific robot config)
        pinpoint.setOffsets(2.4, -4.4, DistanceUnit.INCH) ; // distance in mm
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        // (6.5, 6) is a field-frame start (x right, y forward); convert to Pinpoint axes
        // (x forward, y left) before seeding.
        pinpoint.setPosition(customMath.fieldToPinpoint(
                new Pose2D(DistanceUnit.INCH, 6.5, 6, AngleUnit.DEGREES, 0)));
        local = customMath.pinpointToField(pinpoint.getPosition());

        telemetry.addData("local x (field, right +)", local.getX(DistanceUnit.INCH));
        telemetry.addData("local y (field, forward +)", local.getY(DistanceUnit.INCH));

        telemetry.addData("pinpoint raw x (forward +)", pinpoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("pinpoint raw y (left +)", pinpoint.getPosY(DistanceUnit.INCH));

        telemetry.update();

        launcher.setFwLPIDF(LauncherConstants.fwLkP, LauncherConstants.fwLkI, LauncherConstants.fwLkD, LauncherConstants.fwLkF);
        launcher.setFwRPIDF(LauncherConstants.fwRkP, LauncherConstants.fwRkI, LauncherConstants.fwRkD, LauncherConstants.fwRkF);


        //blocker.setPosition(0.79);
    }

    @Override
    public void loop() {
        loopTimer.reset();
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");
        telemetry.addData("test value", testvalue);

        pinpoint.update();
        // Field frame (x right, y forward), converted from the Pinpoint's x-forward / y-left.
        double x = customMath.pinpointToFieldX(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        double y = customMath.pinpointToFieldY(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        local = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, 0);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("local", local);
        telemetry.addData("Distance", customMath.poseToTranslation(local).getDistance(customMath.poseToTranslation(target)));
        telemetry.addData("Shooter Power", frontTargetVelocity);
        telemetry.addData("Back Shooter Power", backTargetVelocity);
//        telemetry.addData("encoder position right", shooterRight.getCurrentPosition());
//        telemetry.addData("encoder position left", shooterLeft.getCurrentPosition());
        telemetry.addData("Loop time", loopTimer.milliseconds());
//        telemetry.addData("RTposition", RT.getPosition());
        telemetry.addData("LTposition", LT.getPosition());
        telemetry.addData("ramp", ramp.getPosition());

        //telemetry.addData("servo pose", blocker.getPosition());
        telemetry.update();

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        //if (gamepad1.a) {
        //imu.resetYaw();
        //}
        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (gamepad1.left_trigger > 0.1) {
            ramp.setPosition(0.93);
        } else {
            ramp.setPosition(0.4);
//            RT.setPosition(1-testvalue);
            LT.setPosition(testvalue);
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            frontTargetVelocity += 50;
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            frontTargetVelocity -= 50;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            backTargetVelocity -= 50;
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            backTargetVelocity += 50;
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        frontTargetVelocity = Math.max(0, frontTargetVelocity);
        backTargetVelocity = Math.max(0, backTargetVelocity);

        if (gamepad1.b && !lastB) {
            testvalue += .02;
        }
        if (gamepad1.x && !lastX) {
            testvalue -= .02;
        }
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        testvalue = Math.max(0.0, Math.min(1.0, testvalue));

        // Automatic shooting state machine or manual controls
        if (shooting) {
            double elapsed = shootTimer.milliseconds();
            launcher.setActive(true);
            launcher.autoPIDF(local , target);

            if (elapsed < 400) {
                // Wait 200ms for shooter motors to spin up
                blocker.setPosition(1.0);
                intakeDrive.setPower(0);
                transfer.setPower(0);
            } else if (elapsed < 3000) { // 400ms + 4 * (100ms ON + 200ms OFF) = 1400ms
                blocker.setPosition(0.66);
//                double cycleTime = (elapsed - 400) % 300; //
//                if (cycleTime > 150) {
//                    intakeDrive.setPower(1);
//                    transfer.setPower(1);
//                } else {
//                    intakeDrive.setPower(0);
//                    transfer.setPower(0);
//                }
                double cycleTime = (elapsed - 400) % 300; //
                if (cycleTime < 10) {
                    backShootVelocity = (backShootVelocity + testvalue);
                    frontShootVelocity = (frontShootVelocity + testvalue);

                } else {
                    intakeDrive.setPower(0);
                    transfer.setPower(0);
                }
                if (elapsed > 900) {
                    intakeDrive.setPower(1);
                    transfer.setPower(1);
                }
            } else {
                // Sequence finished
                shooting = false;
                launcher.off();
                blocker.setPosition(1.0);
                intakeDrive.setPower(0);
                transfer.setPower(0);
                backShootVelocity = backTargetVelocity;
                frontShootVelocity = frontTargetVelocity;
            }
        } else {
            if (gamepad1.right_trigger > 0.1) {
                shooting = true;
                shootTimer.reset();
            } else {
                launcher.off();
                blocker.setPosition(1.0);
            }

            // Manual intake & transfer controls when not shooting
            if (gamepad1.a) {
                intakeDrive.setPower(1);
                transfer.setPower(1);
            } else if (gamepad1.y) {
                intakeDrive.setPower(-1);
                transfer.setPower(-1);
            } else if (gamepad1.right_bumper) {
                intakeDrive.setPower(1);
                transfer.setPower(0);
            } else if (gamepad1.left_bumper) {
                intakeDrive.setPower(-1);
                transfer.setPower(0);
            } else {
                intakeDrive.setPower(0);
                transfer.setPower(0);
            }
        }
        if (gamepad1.left_bumper) {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x * 0.7);
        }

        launcher.update();
    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}