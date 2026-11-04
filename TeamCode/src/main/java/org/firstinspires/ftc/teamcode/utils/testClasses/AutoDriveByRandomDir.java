package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.PracticeBot;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

@Utility(name = "Auto Drive Standard", description = "Timed autonomous drive diagnostic")
public class AutoDriveByRandomDir extends OpMode {

    private PracticeBot bot;

    private enum AutoState {
        FIRST_MOVEMENT,
        SECOND_MOVEMENT,
        DONE
    }

    private AutoState currentState = AutoState.FIRST_MOVEMENT;
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        bot = new PracticeBot(hardwareMap, runtime);
        // Bound how often telemetry.update() actually transmits, instead of forcing
        // a Driver Station send every single loop.
        telemetry.setMsTransmissionInterval(250);
        telemetry.addLine("Auto Initialized. Waiting for start...");
        telemetry.update();

        Pose2D startingPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0);
        bot.odo.resetPosAndIMU();
        bot.odo.setPosition(startingPose);

        // Without this, bot.drive.update() is skipped every loop (see Subsystem.isActive()),
        // which means odo.update() never runs and the pose telemetry below stays frozen
        // at the starting pose for the entire match.
        bot.drive.setActive(true);
    }


    @Override
    public void start() {
        stateTimer.reset();
        runtime.reset();
    }

    /**
     * Code to run REPEATEDLY while the autonomous period is active.
     */
    @Override
    public void loop() {
        bot.update();

        // State machine execution
        switch (currentState) {
            case FIRST_MOVEMENT:
                // Set motor powers for movement 1 (Spin in place)
                setMotorPowers(0.5, 0.5, -0.5, -0.5);

                // Check if 5 seconds (5000 ms) have elapsed
                if (stateTimer.milliseconds() >= 5000) {
                    // Stop motors briefly and transition to next state
                    setMotorPowers(0, 0, 0, 0);
                    currentState = AutoState.SECOND_MOVEMENT;
                    stateTimer.reset(); // Reset timer for the next state duration
                }
                break;

            case SECOND_MOVEMENT:
                // Set motor powers for movement 2 (Drive straight)
                setMotorPowers(0.5, 0.5, 0.5, 0.5);

                // Check if another 5 seconds have elapsed
                if (stateTimer.milliseconds() >= 5000) {
                    setMotorPowers(0, 0, 0, 0);
                    currentState = AutoState.DONE;
                }
                break;

            case DONE:
            default:
                setMotorPowers(0, 0, 0, 0);
                break;
        }

        // Telemetry reporting
        Pose2D pose = bot.odo.getPosition();
        telemetry.addData("Current State", currentState);
        telemetry.addData("Runtime (s)", bot.time.seconds());
        telemetry.addData("State Time (s)", stateTimer.seconds());
        telemetry.addData("X (in)", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    // Routed through DriveSubSystem's cached setter so holding the same power for
    // an entire timed state (thousands of loop cycles) only writes to the hardware once,
    // instead of re-sending an identical setPower() command every single loop.
    private void setMotorPowers(double fl, double bl, double fr, double br) {
        bot.drive.setIndividualPowers(fl, bl, fr, br);
    }
}