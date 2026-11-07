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
 * <p>Use {@link #setActive(boolean)} as the sole scan control. Activating starts the camera
 * and opens a scan window; deactivating stops the camera immediately. The scan deactivates
 * when the window expires, or on the first tag only when STOP_ON_FIRST_TAG is enabled. Distinct
 * IDs seen during the scan are retained.
 */
public class VisionSubSystem extends Subsystem {
    private final Limelight3A limelight;
    private final ElapsedTime scanTimer = new ElapsedTime();

    private List<Integer> tagIds = Collections.emptyList();

    public VisionSubSystem(Limelight3A limelight) {
        this.limelight = limelight;
    }

    @Override
    public void setup() {
        limelight.setPollRateHz(VisionConstants.POLL_HZ);
        limelight.pipelineSwitch(VisionConstants.APRILTAG_PIPELINE);
        // Deliberately do not start the camera here. setActive(true) starts a scan.
    }

    /**
     * Starts a fresh scan when activated. Deactivation immediately stops the Limelight.
     */
    @Override
    public void setActive(boolean active) {
        if (active) {
            if (isActive()) return;

            tagIds = new ArrayList<>();
            scanTimer.reset();
            limelight.start();
            super.setActive(true);
        } else {
            if (isActive()) limelight.stop();
            super.setActive(false);
        }
    }

    /** Polls the Limelight while active, collects distinct IDs, and closes on configured stop or timeout. */
    @Override
    public void update() {
        if (!isActive()) return;

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                int id = fr.getFiducialId();
                if (!tagIds.contains(id)) {
                    tagIds.add(id);
                }
            }

            if (VisionConstants.STOP_ON_FIRST_TAG && !tagIds.isEmpty()) {
                setActive(false);
                return;
            }
        }

        if (scanTimer.seconds() >= VisionConstants.SCAN_SECONDS) {
            setActive(false);
        }
    }

    @Override
    public void stop() {
        setActive(false);
    }

    // ==========================================================
    // GETTERS
    // ==========================================================

    /** True while the camera is on and the scan is active. */
    public boolean isScanning() { return isActive(); }

    /** True if at least one tag was seen in the latest scan. */
    public boolean hasTag() { return !tagIds.isEmpty(); }

    /** Distinct IDs seen during the latest scan (empty immediately after activation). */
    public List<Integer> getTagIds() { return tagIds; }

    /** First ID seen in the latest scan, or -1 if none. */
    public int getFirstTagId() { return tagIds.isEmpty() ? -1 : tagIds.get(0); }

    /** Seconds left in the scan window (0 when not scanning). */
    public double getTimeLeft() {
        return isActive() ? Math.max(0, VisionConstants.SCAN_SECONDS - scanTimer.seconds()) : 0;
    }

    public boolean getTipped(){ return tagIds.size() < 3; }
}