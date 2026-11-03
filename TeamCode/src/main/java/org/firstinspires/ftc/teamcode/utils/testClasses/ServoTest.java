package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.roboConfigs.ServoConstants;
import org.firstinspires.ftc.teamcode.LM1Robot;
import org.firstinspires.ftc.teamcode.PracticeBot;
import org.firstinspires.ftc.teamcode.utils.ServoUtility;
import org.firstinspires.ftc.teamcode.utils.ServoAxis;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

@Disabled
@TeleOp(name = "ServoTest", group = "9 - Utility")
public class ServoTest extends OpMode {

    private LM1Robot bot;

    private final ElapsedTime runtime = new ElapsedTime();

    // ---- What we're controlling ----
    private int servoIndex = 0;               // index into the selected axis
    private int axisIndex  = 0;               // 0 = turret, 1 = pitch
    private boolean angleMode = false;        // false = raw position, true = angle via subsystem

    // ---- State ----
    private double rawPosition = 0.5;         // 0..1
    private double targetAngle = 0.0;         // degrees
    private double step = 0.005;              // per frame nudge (raw)
    private double angleStep = 1.0;           // per frame nudge (angle)

    // ---- Edge detection ----
    private boolean aLast = false;
    private boolean bLast = false;
    private boolean xLast = false;
    private boolean yLast = false;
    private boolean shareLast = false;
    private boolean optionsLast = false;

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry); //comment out prepareshooter()
        telemetry.setMsTransmissionInterval(100);
        telemetry.addLine("ServoTest Initialized");
        telemetry.addLine("A = next servo   B = toggle raw/angle");
        telemetry.addLine("X = atMin        Y = atMax");
        telemetry.addLine("Dpad = nudge     Bumpers = change step");
        telemetry.addLine("Share = cycle axis");
        telemetry.update();
    }

    @Override
    public void loop() {
        bot.update();

        // ---------- Axis / servo selection ----------
        if (gamepad1.share && !shareLast) {
            axisIndex = (axisIndex + 1) % 2;
            servoIndex = 0;
            // Sync current state from the active axis
            if (angleMode) {
                targetAngle = getActiveAxis().getAngle();
            } else {
                rawPosition = getActiveAxis().getPosition(servoIndex);
            }
        }
        shareLast = gamepad1.share;

        if (gamepad1.a && !aLast) {
            int n = getActiveAxis().getServoCount();
            servoIndex = (servoIndex + 1) % n;
        }
        aLast = gamepad1.a;

        // ---------- Mode toggle ----------
        if (gamepad1.b && !bLast) {
            angleMode = !angleMode;
            if (angleMode) {
                // Convert current raw position back into an angle as a starting point
                targetAngle = ServoUtility.positionToDegrees(
                        rawPosition,
                        getActiveAxis().getMinDeg(),
                        getActiveAxis().getMaxDeg(),
                        getActiveAxisAtMin(servoIndex),
                        getActiveAxisAtMax(servoIndex));
            } else {
                rawPosition = getActiveAxis().getPosition(servoIndex);
            }
        }
        bLast = gamepad1.b;

        // ---------- Step size ----------
        if (gamepad1.right_bumper) step = 0.02;
        else if (gamepad1.left_bumper) step = 0.001;
        else step = 0.005;

        if (gamepad1.dpad_up)    angleStep = 10.0;
        else if (gamepad1.dpad_down) angleStep = 0.5;
        else angleStep = 1.0;

        // ---------- Nudge ----------
        if (!angleMode) {
            if (gamepad1.dpad_left)  rawPosition -= step;
            if (gamepad1.dpad_right) rawPosition += step;
            rawPosition = ServoUtility.clamp(rawPosition, 0.0, 1.0);
        } else {
            if (gamepad1.dpad_left)  targetAngle -= angleStep;
            if (gamepad1.dpad_right) targetAngle += angleStep;
            targetAngle = ServoUtility.clampAngle(
                    targetAngle,
                    getActiveAxis().getMinDeg(),
                    getActiveAxis().getMaxDeg());
        }

        // ---------- Jump to limits ----------
        if (gamepad1.x && !xLast) {
            if (angleMode) {
                targetAngle = getActiveAxis().getMinDeg();
            } else {
                rawPosition = getActiveAxisAtMin(servoIndex);
            }
        }
        xLast = gamepad1.x;

        if (gamepad1.y && !yLast) {
            if (angleMode) {
                targetAngle = getActiveAxis().getMaxDeg();
            } else {
                rawPosition = getActiveAxisAtMax(servoIndex);
            }
        }
        yLast = gamepad1.y;

        // ---------- Options: print suggested constants ----------
        if (gamepad1.options && !optionsLast) {
            double implied = ServoUtility.positionToDegrees(
                    rawPosition,
                    getActiveAxis().getMinDeg(),
                    getActiveAxis().getMaxDeg(),
                    getActiveAxisAtMin(servoIndex),
                    getActiveAxisAtMax(servoIndex));
            telemetry.log().add(String.format(
                    "servo %d raw=%.3f implied=%.2f deg",
                    servoIndex, rawPosition, implied));
        }
        optionsLast = gamepad1.options;

        // ---------- Apply ----------
        Servo[] rawServos = getActiveAxis().getServos();

        if (!angleMode) {
            // Direct raw write — bypasses the subsystem entirely
            rawServos[servoIndex].setPosition(rawPosition);
        } else {
            // High-level angle write — moves all servos on the axis
            getActiveAxis().setAngle(targetAngle);
        }

        // ---------- Telemetry ----------
        String axisName = (axisIndex == 0) ? "turret" : "pitch";
        telemetry.addData("Axis", "%s (%d servos)", axisName, getActiveAxis().getServoCount());
        telemetry.addData("Servo index", servoIndex);
        telemetry.addData("Mode", angleMode ? "ANGLE (subsystem)" : "RAW (direct)");

        if (!angleMode) {
            telemetry.addData("Raw position", "%.4f", rawPosition);
            double implied = ServoUtility.positionToDegrees(
                    rawPosition,
                    getActiveAxis().getMinDeg(),
                    getActiveAxis().getMaxDeg(),
                    getActiveAxisAtMin(servoIndex),
                    getActiveAxisAtMax(servoIndex));
            telemetry.addData("Implied angle (deg)", "%.2f", implied);
            telemetry.addData("Step", "%.4f", step);
        } else {
            telemetry.addData("Target angle (deg)", "%.2f", targetAngle);
            telemetry.addData("Last commanded", "%.2f", getActiveAxis().getAngle());
            telemetry.addData("Clamped pos", "%.4f", getActiveAxis().getPosition(servoIndex));
            telemetry.addData("Step", "%.2f deg", angleStep);
        }

        telemetry.addData("Range", "[%.1f, %.1f] deg",
                getActiveAxis().getMinDeg(), getActiveAxis().getMaxDeg());
        telemetry.addData("Loop (ms)", runtime.milliseconds());
        telemetry.update();

        runtime.reset();
    }

    // ---------------- Helpers ----------------

    private ServoAxis getActiveAxis() {
//        return (axisIndex == 0) ? bot.turret.getAxis() : bot.pitch.getAxis();
        return bot.turret.getAxis();
    }

    private double getActiveAxisAtMin(int idx) {
        return (axisIndex == 0) ? ServoConstants.Turret.AT_MIN[idx]
                : ServoConstants.Pitch.AT_MIN[idx];
    }

    private double getActiveAxisAtMax(int idx) {
        return (axisIndex == 0) ? ServoConstants.Turret.AT_MAX[idx]
                : ServoConstants.Pitch.AT_MAX[idx];
    }
}