package org.firstinspires.ftc.teamcode.Configs.roboConfigs;

/** Limelight 3A tuning. Edit per robot. */
public final class VisionConstants {

    private VisionConstants() { }

    /** Hardware-map name of the Limelight (must match the Driver Station config). */
    public static final String NAME = "limelight";

    /** Limelight pipeline index set up for AprilTags (tag36h11 for DECODE) in the Limelight web UI. */
    public static final int APRILTAG_PIPELINE = 0;

    /** How long (seconds) the camera stays on per scan before shutting itself off. */
    public static final double SCAN_SECONDS = 3.0;

    /** Poll rate (Hz) the Limelight runs its data stream at. */
    public static final int POLL_HZ = 100;

    /** If true, the camera shuts off as soon as a tag is seen (instead of waiting out the window). */
    public static final boolean STOP_ON_FIRST_TAG = true;
}
