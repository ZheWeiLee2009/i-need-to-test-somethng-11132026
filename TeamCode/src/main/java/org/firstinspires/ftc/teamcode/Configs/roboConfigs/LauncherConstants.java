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
    public static double fwLkP = 0.0013;

    /** Integral gain: power added per RPM-second of accumulated error. Leave at 0 unless the
     *  flywheel settles below target and stays there (steady-state error) with kP/kF alone. */
    @Sorter(sort = 1)
    public static double fwLkI = 0;

    /** Derivative gain: power added per RPM/second of error change. Damps overshoot; usually
     *  left at 0 for a flywheel unless you see oscillation around the target RPM. */
    @Sorter(sort = 2)
    public static double fwLkD = 0;

    /** Feedforward gain: power commanded per RPM of target, applied regardless of current
     *  error. Should be close to (1.0 / max achievable RPM) as a starting point. */
    @Sorter(sort = 3)
    public static double fwLkF = 0.00017;

    @Sorter(sort = 4)
    public static double fwRkP = 0.0013;

    /** Integral gain: power added per RPM-second of accumulated error. Leave at 0 unless the
     *  flywheel settles below target and stays there (steady-state error) with kP/kF alone. */
    @Sorter(sort = 5)
    public static double fwRkI = 0;

    /** Derivative gain: power added per RPM/second of error change. Damps overshoot; usually
     *  left at 0 for a flywheel unless you see oscillation around the target RPM. */
    @Sorter(sort = 6)
    public static double fwRkD = 0;

    /** Feedforward gain: power commanded per RPM of target, applied regardless of current
     *  error. Should be close to (1.0 / max achievable RPM) as a starting point. */
    @Sorter(sort = 7)
    public static double fwRkF = 0.00018;


    /** Bang-bang entry threshold (RPM): when |target - current| exceeds this, the launcher
     *  switches to bang-bang (full power if too slow, zero if too fast). Must be >= the exit
     *  threshold. */
    @Sorter(sort = 8)
    public static double BANG_BANG_ENTER_RPM = 500.0;

    /** Bang-bang exit threshold (RPM): once |target - current| falls below this, control hands
     *  off to PIDF. Keep it below the enter threshold; the gap is hysteresis that prevents
     *  chattering between modes near the boundary. */
    @Sorter(sort = 9)
    public static double BANG_BANG_EXIT_RPM = 400.0;

    /** Both flywheels must stay inside the at-target tolerance continuously for this long (ms)
     *  before isAtTarget() reports ready. Stops a shot being fed while the wheels are only
     *  passing through the target on the way up (or ringing around it). */
    @Sorter(sort = 11)
    public static double AT_TARGET_STABLE_MS = 0.0;

    @Sorter(sort = 10)
    public static double fwLRpm = 1800;

    @Sorter(sort = 10)
    public static double fwRRpm = 1800;
}