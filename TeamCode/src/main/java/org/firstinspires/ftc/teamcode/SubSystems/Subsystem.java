package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.List;

/*
    Do not modify this in any repos other than this one without knowing how to deal with git-merge.
 */

/**
 * The base abstract class for all robot mechanisms.
 * Provides standard lifecycle methods and hardware configuration utilities.
 */
public abstract class Subsystem {

    /** Tracks if this subsystem needs background updates (saves CPU when false) */
    private boolean active = false;

    /**
     * Gets the current active state of the subsystem.
     * @return true if active, false if idle
     */
    public boolean isActive() {
        return active;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    /**
     * Performs one-time subsystem initialization/setup.
     *
     * <p>Subclasses can override this method if they need to configure
     * hardware, initialize variables, or perform other startup tasks.
     *
     * <p>This method is intentionally not abstract, so subclasses that
     * do not require setup do not need to implement it.
     */
    public void setup() {
    }

    /**
     * Stops the subsystem.
     */
    public abstract void stop();

    /**
     * Performs the subsystem's periodic background update.
     *
     * <p>Subclasses can override this method if they require periodic
     * background processing.
     */
    public void update() {
    }

    // ==========================================================
    // SHARED HARDWARE UTILITIES
    // ==========================================================

    /**
     * Sets the run mode for a single motor.
     *
     * @param motor The motor to configure
     * @param mode  The desired RunMode
     */
    public void setMotorsMode(@NonNull DcMotorEx motor, @NonNull DcMotorEx.RunMode mode) {
        motor.setMode(mode);
    }

    /**
     * Sets the run mode for a list of motors.
     *
     * @param list  The list of motors to configure
     * @param mode  The desired RunMode
     */
    public void setMotorsMode(@NonNull List<DcMotorEx> list, @NonNull DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : list) {
            motor.setMode(mode);
        }
    }

    /**
     * Sets the ZeroPowerBehavior for a single motor.
     *
     * @param motor The motor to configure
     * @param zpb   The desired ZeroPowerBehavior (BRAKE or FLOAT)
     */
    public void setZeroPowerBehavior(@NonNull DcMotorEx motor, @NonNull DcMotor.ZeroPowerBehavior zpb) {
        motor.setZeroPowerBehavior(zpb);
    }

    /**
     * Sets the ZeroPowerBehavior for a list of motors.
     *
     * @param list  The list of motors to configure
     * @param zpb   The desired ZeroPowerBehavior (BRAKE or FLOAT)
     */
    public void setZeroPowerBehavior(@NonNull List<DcMotorEx> list, @NonNull DcMotor.ZeroPowerBehavior zpb) {
        for (DcMotorEx motor : list) {
            motor.setZeroPowerBehavior(zpb);
        }
    }
}