package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;

/**
 * INTAKE — one specific task: run the intake roller at full power or stop it.
 * See {@code SubSystems/mechanisms.md} for usage notes and pitfalls.
 */
public class IntakeSubSystem extends Subsystem{
    private final DcMotorEx intake;

    public IntakeSubSystem(List<DcMotorEx> Motors){
        this.intake = Motors.get(0);
    }

    /** Runs the intake motor at full power. */
    public void on(){
        intake.setPower(1);
    }

    /** Stops the intake motor. */
    public void off(){
        intake.setPower(0);
    }

    /**
     * Stops the intake motor.
     */
    @Override
    public void stop() {
        off();
    }

}
