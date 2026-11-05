> Do not modify this in any repos other than this one without knowing how to deal with git-merge.
# VisionSubSystem — Limelight 3A AprilTag Guide

Reads AprilTag IDs only (no pose/aiming). The camera runs **only during a scan window**.

```java
bot.startTagScan();            // open window (VisionConstants.SCAN_SECONDS)
bot.vision.getTagIds();        // List<Integer> from the latest scan
bot.vision.hasTag();           // any tag seen?
bot.vision.getFirstTagId();    // -1 if none
bot.vision.isScanning();       // camera currently on?
```

- The window closes itself on timeout, or on the first tag if `STOP_ON_FIRST_TAG` is true.
  Results persist after it closes until the next `startScan()`.
- The subsystem is only `active` while scanning, so `bot.update()` skips it otherwise.

## Setup checklist
1. Plug the Limelight into a Control Hub USB port; in the Robot Config add a Limelight 3A named
   `limelight` (or change `VisionConstants.NAME`).
2. In the Limelight web UI (limelight.local:5801), make pipeline `APRILTAG_PIPELINE` (default 0)
   an AprilTag pipeline using the 36h11 family.
3. Tune `SCAN_SECONDS` / `STOP_ON_FIRST_TAG` in `Configs/roboConfigs/VisionConstants.java`.

## Pitfalls
| Pitfall | Fix |
|---|---|
| Hardware map throws on init | Config has no device named `limelight` (name is case-sensitive). |
| Never sees tags | Wrong pipeline index/type, or the Limelight hasn't finished booting (~30s after power). |
| Scan seems to do nothing | `startScan()` was never called, or the window already closed — check `isScanning()`. |
