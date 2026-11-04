package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.LM1Robot;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

@Utility(name = "Turret Test", description = "Sweep the turret min -> 0 -> max to verify the calibrated range")
public class TurretTest extends LinearOpMode {
    private static final long HOLD_MS = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        // Passing blackboard means a calibration done earlier this session is used if present.
        LM1Robot bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);

        double min = bot.turret.getClipMin();   // includes the 270 deg clip
        double max = bot.turret.getClipMax();

        bot.turret.setAngle(0.0);   // go to zero during INIT; go() can't be used before Play

        telemetry.addData("Calibration source", bot.turretCalSource);
        telemetry.addData("Range", "%.1f to %.1f deg", min, max);
        telemetry.addLine("Ready. Keep the turret clear; Play sweeps min -> 0 -> max.");
        telemetry.update();

        waitForStart();

        // No bot.update(): it would spin up the flywheel via prepareShooter().
        while (opModeIsActive()) {
            go(bot, "MIN", min);
            go(bot, "ZERO", 0.0);
            go(bot, "MAX", max);
            go(bot, "ZERO", 0.0);
        }
    }

    private void go(LM1Robot bot, String label, double deg) {
        if (!opModeIsActive()) return;
        bot.turret.setAngle(deg);   // clamped to MIN_DEG..MAX_DEG by ServoAxis
        telemetry.addData("Calibration source", bot.turretCalSource);
        telemetry.addData("Going to", "%s (%.1f deg)", label, deg);
        telemetry.addData("Commanded", "%.1f deg", bot.turret.getAngle());
        telemetry.addData("Servo position", "%.4f", bot.turret.getPosition(0));
        telemetry.update();
        sleep(HOLD_MS);
    }
}
