package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.LM1Robot;
import org.firstinspires.ftc.teamcode.utils.TurretCalibrator;

/**
 * Standalone turret calibration. All of it happens in INIT; there is nothing to do after Play.
 * Re-run only if the turret gear, a servo, or the mounting changes (or the gear skips).
 * Controls and output are documented on {@link TurretCalibrator}.
 *
 * Keep the robot still during init: LM1Robot recalibrates the Pinpoint IMU, and its turret
 * subsystem moves the turret to its current MAX_DEG when it starts.
 */
@TeleOp(name = "TurretCalibration", group = "9 - Utility")
public class TurretCalibrationTeleOp extends OpMode {

    private LM1Robot bot;
    private TurretCalibrator calibrator;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);
        telemetry.setMsTransmissionInterval(100);
        calibrator = new TurretCalibrator(bot, gamepad1, telemetry, blackboard);
    }

    @Override
    public void init_loop() {
        calibrator.update();
    }

    @Override
    public void loop() {
        // No button handling here and no bot.update() (it would spin up the flywheel via
        // prepareShooter()). Just keep the numbers on screen.
        telemetry.addLine("Calibration happens in INIT. Press Stop; re-run to adjust.");
        calibrator.report();
    }
}
