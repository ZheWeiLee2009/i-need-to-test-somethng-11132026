package org.firstinspires.ftc.teamcode.Configs.roboConfigs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * PIDF gains and bang-bang thresholds for {@link org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem}'s
 * flywheel velocity control loop. Units are motor power (-1..1) per unit of RPM error/derivative/target,
 * i.e. these are the standard {@code PIDFController} gains, tuned against flywheel RPM (not
 * encoder ticks or degrees). See {@code SubSystems/mechanisms.md} for a tuning walkthrough.
 */
@Configurable
public class LauncherConstants {

    /** Proportional gain: power added per RPM of current error (target - current). */
    @Sorter(sort = 0)
    public static double kP = 0.01;

    /** Integral gain: power added per RPM-second of accumulated error. Leave at 0 unless the
     *  flywheel settles below target and stays there (steady-state error) with kP/kF alone. */
    @Sorter(sort = 1)
    public static double kI = 0;

    /** Derivative gain: power added per RPM/second of error change. Damps overshoot; usually
     *  left at 0 for a flywheel unless you see oscillation around the target RPM. */
    @Sorter(sort = 2)
    public static double kD = 0;

    /** Feedforward gain: power commanded per RPM of target, applied regardless of current
     *  error. Should be close to (1.0 / max achievable RPM) as a starting point. */
    @Sorter(sort = 3)
    public static double kF = 0.00035;

    /** Bang-bang entry threshold (RPM): when |target - current| exceeds this, the launcher
     *  switches to bang-bang (full power if too slow, zero if too fast). Must be >= the exit
     *  threshold. */
    @Sorter(sort = 4)
    public static double BANG_BANG_ENTER_RPM = 500.0;

    /** Bang-bang exit threshold (RPM): once |target - current| falls below this, control hands
     *  off to PIDF. Keep it below the enter threshold; the gap is hysteresis that prevents
     *  chattering between modes near the boundary. */
    @Sorter(sort = 5)
    public static double BANG_BANG_EXIT_RPM = 400.0;
}