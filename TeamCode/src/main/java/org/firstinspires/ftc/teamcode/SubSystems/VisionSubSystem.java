package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.roboConfigs.VisionConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * VISION — Limelight 3A AprilTag reader. One job: read which AprilTag IDs are visible.
 *
 * <p>The Limelight is only polled during a short "scan window". Call {@link #startScan()} (or
 * {@link #startScan(double)}) and the camera starts polling, the subsystem goes active, and it
 * shuts itself back off when the window expires (or on the first tag, see
 * {@code VisionConstants.STOP_ON_FIRST_TAG}). The last IDs seen are kept after the window closes.
 * See {@code SubSystems/vision.md}.
 */
public class VisionSubSystem extends Subsystem {
    private final Limelight3A limelight;
    private final ElapsedTime scanTimer = new ElapsedTime();

    private boolean scanning = false;
    private double windowSeconds = VisionConstants.SCAN_SECONDS;
    private List<Integer> tagIds = Collections.emptyList();

    public VisionSubSystem(Limelight3A limelight) {
        this.limelight = limelight;
    }

    @Override
    public void setup() {
        limelight.setPollRateHz(VisionConstants.POLL_HZ);
        limelight.pipelineSwitch(VisionConstants.APRILTAG_PIPELINE);
        // Deliberately NOT started here: the camera only runs during a scan window.
    }

    /** Opens a scan window using the default length from VisionConstants. */
    public void startScan() { startScan(VisionConstants.SCAN_SECONDS); }

    /** Opens a scan window of the given length (seconds). Restarts the window if already scanning. */
    public void startScan(double seconds) {
        windowSeconds = seconds;
        tagIds = Collections.emptyList();
        scanTimer.reset();
        if (!scanning) limelight.start();
        scanning = true;
        setActive(true);
    }

    /** Closes the scan window early. The last IDs seen are kept. */
    public void stopScan() {
        if (scanning) limelight.stop();
        scanning = false;
        setActive(false);
    }

    /** Polls the Limelight while the window is open; closes it when time is up. */
    @Override
    public void update() {
        if (!scanning) { setActive(false); return; }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<Integer> ids = new ArrayList<>();
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                ids.add(fr.getFiducialId());
            }
            if (!ids.isEmpty()) {
                tagIds = ids;
                if (VisionConstants.STOP_ON_FIRST_TAG) { stopScan(); return; }
            }
        }

        if (scanTimer.seconds() >= windowSeconds) stopScan();
    }

    @Override
    public void stop() { stopScan(); }

    // ==========================================================
    // GETTERS
    // ==========================================================

    /** True while the camera is on. */
    public boolean isScanning() { return scanning; }

    /** True if at least one tag was seen in the latest scan. */
    public boolean hasTag() { return !tagIds.isEmpty(); }

    /** IDs seen in the latest scan (empty if none). Kept after the window closes. */
    public List<Integer> getTagIds() { return tagIds; }

    /** First ID seen in the latest scan, or -1 if none. */
    public int getFirstTagId() { return tagIds.isEmpty() ? -1 : tagIds.get(0); }

    /** Seconds left in the scan window (0 when not scanning). */
    public double getTimeLeft() { return scanning ? Math.max(0, windowSeconds - scanTimer.seconds()) : 0; }
}
