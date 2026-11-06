> Do not modify this in any repos other than this one without knowing how to deal with git-merge.
# VisionSubSystem — Limelight 3A AprilTag Guide

Reads AprilTag IDs only (no pose/aiming). Control the camera directly through `setActive()`:

```java
bot.vision.setActive(true);   // start a scan window (VisionConstants.SCAN_SECONDS)
bot.vision.setActive(false);  // stop the camera immediately
// Or toggle it:
bot.vision.setActive(!bot.vision.isActive());
bot.vision.getTagIds();       // List<Integer> from the latest scan
bot.vision.hasTag();          // any tag seen?
bot.vision.getFirstTagId();   // -1 if none
bot.vision.isScanning();      // camera currently on?
```

- The scan closes itself on timeout or when a tag is detected.
- Results persist after deactivation until the next `setActive(true)` starts a fresh scan.
- `setActive(false)` stops the Limelight immediately; the master `bot.update()` loop skips Vision
  while it is inactive.

## Setup checklist
1. Plug the Limelight into a Control Hub USB port; in the Robot Config add a Limelight 3A named
   `limelight` (or change `VisionConstants.NAME`).
2. In the Limelight web UI (limelight.local:5801), make pipeline `APRILTAG_PIPELINE` (default 0)
   an AprilTag pipeline using the 36h11 family.
3. Tune `SCAN_SECONDS` in `Configs/roboConfigs/VisionConstants.java`.

## Pitfalls
| Pitfall | Fix |
|---|---|
| Hardware map throws on init | Config has no device named `limelight` (name is case-sensitive). |
| Never sees tags | Wrong pipeline index/type, or the Limelight hasn't finished booting (~30s after power). |
| Scan seems to do nothing | Confirm `vision.setActive(true)` was called and check `isScanning()`. |
