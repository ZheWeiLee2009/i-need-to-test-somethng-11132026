package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PracticeBot;


/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

@Utility(name = "Drive Motor Test", description = "Spin an individual drive motor to verify direction/wiring")
public class DriveMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        PracticeBot bot = new PracticeBot(hardwareMap, runtime);

        waitForStart();

        while (opModeIsActive()) {
            bot.fl.setPower(1);
            sleep(500);
            bot.fl.setPower(0);

            bot.fr.setPower(1);
            sleep(500);
            bot.fr.setPower(0);

            bot.bl.setPower(1);
            sleep(500);
            bot.bl.setPower(0);

            bot.br.setPower(1);
            sleep(500);
            bot.br.setPower(0);
        }
    }
}