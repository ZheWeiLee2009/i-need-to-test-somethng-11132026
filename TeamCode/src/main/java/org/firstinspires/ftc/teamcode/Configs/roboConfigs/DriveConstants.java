package org.firstinspires.ftc.teamcode.Configs.roboConfigs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/**
 * Per-robot tuning numbers for {@link org.firstinspires.ftc.teamcode.SubSystems.DriveSubSystem}.
 * See {@code SubSystems/drive.md} for what each of these does and how to tune it.
 */
@Configurable
public class DriveConstants {

    /** Global speed scaling factor (0..1). Not read by DriveSubSystem itself — apply it
     *  as the speedMultiplier argument on whichever drive method your OpMode calls. */
    @Sorter(sort = 0)
    public static double DriveSpeed = 1.0;

    /** Per-wheel power multiplier, unitless, normally +1 or -1. Use to correct a wheel that
     *  drives backwards relative to the others, or scale down one wheel if it's mechanically
     *  stronger than the rest. Leave at 1 unless you have a specific reason to change it. */
    @Sorter(sort = 1)
    public static double FL_Offset = 1;

    @Sorter(sort = 2)
    public static double BL_Offset = 1;

    @Sorter(sort = 3)
    public static double FR_Offset = 1;

    @Sorter(sort = 4)
    public static double BR_Offset = 1;

    /** Braking aggressiveness for instantStopDrive/instantStopFODDrive, unitless.
     *  Multiplies the robot's current velocity (inches/sec) to get the reverse brake power,
     *  so it's effectively "brake power per inch/sec of speed". Higher = harder, snappier stop;
     *  too high will make the robot jerk backwards or tip. Tune in small steps (~0.002). */
    @Sorter(sort = 5)
    public static double instantStopConstant = 0.016;

    /** Minimum robot speed, in inches/second, before the instant-stop brake kicks in when the
     *  stick is released. Below this the robot is treated as already stopped, so it just holds
     *  yaw instead of braking (avoids braking against sensor noise while sitting still). */
    @Sorter(sort = 6)
    public static double VELOCITY_CUTOFF = 3.6;

    /** Ceiling on the reverse brake power the instant-stop routines are allowed to command,
     *  as a fraction of full power (0..1). Caps how hard the brake can slam the motors,
     *  independent of how fast the robot was moving. */
    @Sorter(sort = 7)
    public static double MAX_REVERSE_POWER = .8;


}
