package org.firstinspires.ftc.teamcode.Configs.roboConfigs;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.Sorter;

/** Tuning values for field-pose-based turret aiming. */
@Configurable
public final class TurretConstants {

    private TurretConstants() { }

    /** Pinpoint is this far forward of the turret pivot, measured along robot forward. */
    @Sorter(sort = 0)
    public static double PINPOINT_FORWARD_OF_TURRET_MM = -20.0;

    /** Small mechanical correction added to the calculated turret angle. */
    @Sorter(sort = 1)
    public static double AIM_OFFSET_DEG = 0.0;

    /** Ignore smaller angle changes to avoid unnecessary servo writes. */
    @Sorter(sort = 2)
    public static double AIM_DEADBAND_DEG = 0.25;

    @Sorter(sort = 0)
    public static double TARGET_X_IN = 64.5;

    @Sorter(sort = 1)
    public static double TARGET_Y_IN = 63.5;
}
