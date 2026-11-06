package org.firstinspires.ftc.teamcode.Robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.ServoConstants;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.VisionConstants;
import org.firstinspires.ftc.teamcode.SubSystems.DriveSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.FlowerSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;
import org.firstinspires.ftc.teamcode.SubSystems.TransferSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.TurretSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.VisionSubSystem;
import org.firstinspires.ftc.teamcode.utils.RayCasting;
import org.firstinspires.ftc.teamcode.utils.customMath;
import org.firstinspires.ftc.teamcode.utils.TurretCalibrationStore;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * Master hardware mapping and subsystem manager class for PracticeBot.
 *
 * DO MODIFY THIS
 */
public class LM1Robot {

    private final List<LynxModule> allHubs;

    public GoBildaPinpointDriver odo;

    // ---- Raw motor hardware ----

    public DcMotorEx fl, bl, fr, br;
    public final List<DcMotorEx> driveMotors;

    public DcMotorEx intake1;
    public final List<DcMotorEx> intakeMotors;

    public DcMotorEx transfer1;
    public final List<DcMotorEx> transferMotors;

    public DcMotorEx fwL, fwR;
    public final List<DcMotorEx> flywheelMotors;

    // ---- Raw servo hardware (for debugging) ----
    public Servo gate;
    public Servo flower1;
    public final List<Servo> transferServos;
    public Servo turretLeft, turretRight;

    // ---- Digital sensor hardware ----
    public DigitalChannel ball1L, ball1R, ball2L, ball2R, ball3L, ball3R, ball4L, ball4R;
    public final List<DigitalChannel> transferSensors;

    // ---- subsystems ----

    public DriveSubSystem drive;
    public IntakeSubSystem intake;
    public TransferSubSystem transfer;
    public LauncherSubSystem launcher;
    public TurretSubSystem turret;
    /** Where the turret limits/calibration came from: "BLACKBOARD" or "ServoConstants". */
    public String turretCalSource = "ServoConstants";
    public FlowerSubSystem flower;
    public VisionSubSystem vision;
    public Limelight3A limelight;
    private final List<Subsystem> subsystems = new ArrayList<>();

    // ---- utils ----

    public final RayCasting rayCast;
    public HardwareMap hwMap;

    public ElapsedTime time;

    public final Telemetry joined;

    // ---- booleans ----

    //    private boolean autoShoot = true;
    private boolean isAudience = false;
    private boolean debug = false;

    // ---- poses ----

    private Pose2D lastLocal;
    private Pose2D local;
    private Pose2D target;

    /** Turret numbers come from ServoConstants only. */
    public LM1Robot(HardwareMap hwMap, ElapsedTime runtime, Telemetry telemetry) {
        this(hwMap, runtime, telemetry, null);
    }

    /**
     * @param blackboard the OpMode's {@code blackboard}. If a turret calibration was published to
     *                   it (by TurretCalibrator earlier this session), the turret uses that
     *                   instead of ServoConstants.Turret. Pass null to always use ServoConstants.
     */
    public LM1Robot(HardwareMap hwMap, ElapsedTime runtime, Telemetry telemetry,
                    Map<String, Object> blackboard) {
        this.time = runtime;
        this.hwMap = hwMap;

        // Manual bulk caching, remember to clear cache everytime you update that area (clear cache early, then update)
        allHubs = this.hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // NOTE: these hardware-map names ("FL"/"BL"/"FR"/"BR") drive the same four physical
        // motors as Configs/pedroPathing/Constants.java's drivetrainConfig, which resolves
        // them under its own names ("lf"/"lr"/"rf"/"rr") for Pedro's Mecanum follower.
        // Both naming schemes must point at the same physical ports in the robot's hardware
        // configuration file. If you rename or rewire a drive motor, update BOTH places.
        fl = this.hwMap.get(DcMotorEx.class, "FL");
        bl = this.hwMap.get(DcMotorEx.class, "BL");
        fr = this.hwMap.get(DcMotorEx.class, "FR");
        br = this.hwMap.get(DcMotorEx.class, "BR");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        driveMotors = Arrays.asList(fl, bl, fr, br);

        intake1 = this.hwMap.get(DcMotorEx.class, "intake");
        intakeMotors = Arrays.asList(intake1);

        transfer1 = this.hwMap.get(DcMotorEx.class, "transfer");
        transferMotors = Arrays.asList(transfer1);

        gate = this.hwMap.get(Servo.class, ServoConstants.Gate.NAMES[0]);
        transferServos = Arrays.asList(gate);

        ball1L = this.hwMap.get(DigitalChannel.class, "ballSensor1L");
        ball2L = this.hwMap.get(DigitalChannel.class, "ballSensor2L");
        ball3L = this.hwMap.get(DigitalChannel.class, "ballSensor3L");
        ball4L = this.hwMap.get(DigitalChannel.class, "ballSensor4L");
        ball1R = this.hwMap.get(DigitalChannel.class, "ballSensor1R");
        ball2R = this.hwMap.get(DigitalChannel.class, "ballSensor2R");
        ball3R = this.hwMap.get(DigitalChannel.class, "ballSensor3R");
        ball4R = this.hwMap.get(DigitalChannel.class, "ballSensor4R");
        transferSensors = Arrays.asList(ball1L, ball2L, ball3L, ball4L,
                ball1R, ball2R, ball3R, ball4R);

        flower1 = this.hwMap.get(Servo.class, ServoConstants.Flower.NAMES[0]);

        fwL = this.hwMap.get(DcMotorEx.class, "fwL");
        fwR = this.hwMap.get(DcMotorEx.class, "fwR");
        flywheelMotors = Arrays.asList(fwL, fwR);

        turretLeft = this.hwMap.get(Servo.class, ServoConstants.Turret.NAMES[0]);
        turretRight = this.hwMap.get(Servo.class, ServoConstants.Turret.NAMES[1]);

        limelight = this.hwMap.get(Limelight3A.class, VisionConstants.NAME);

        odo = this.hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(-2.46, -4.46, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        // Pinpoint is x-forward / y-left; field code is x-right / y-forward. Seed through the
        // converter (origin + heading 0 are identical in both frames, but keep it uniform).
        odo.setPosition(customMath.fieldToPinpoint(
                new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0)));
        odo.update();

        // ============================================================
        // Subsystem construction
        // ============================================================

        drive = new DriveSubSystem(driveMotors, odo);
        intake = new IntakeSubSystem(intakeMotors);
        transfer = new TransferSubSystem(transferMotors, transferServos, transferSensors);
        launcher = new LauncherSubSystem(flywheelMotors);

        // Turret numbers: blackboard (from a calibration earlier this session) if present,
        // otherwise the permanent values in ServoConstants.Turret.
        double[] cal = TurretCalibrationStore.get(blackboard);
        turretCalSource = cal != null ? "BLACKBOARD" : "ServoConstants";
        int turretServoCount = 2;

        turret = new TurretSubSystem(
                new Servo[]{turretLeft, turretRight},
                cal != null ? cal[0] : ServoConstants.Turret.MIN_DEG,
                cal != null ? cal[1] : ServoConstants.Turret.MAX_DEG,
                cal != null ? TurretCalibrationStore.fill(cal[2], turretServoCount) : ServoConstants.Turret.AT_MIN,
                cal != null ? TurretCalibrationStore.fill(cal[3], turretServoCount) : ServoConstants.Turret.AT_MAX
        );

        flower = new FlowerSubSystem(
                new Servo[] {flower1},
                ServoConstants.Flower.MIN_DEG,
                ServoConstants.Flower.MAX_DEG,
                ServoConstants.Flower.AT_MIN,
                ServoConstants.Flower.AT_MAX
                );

        vision = new VisionSubSystem(limelight);

        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(transfer);
        subsystems.add(launcher);
        subsystems.add(turret);
        subsystems.add(flower);
        subsystems.add(vision);

//
//        pitch = new ServoTaskSubsystem("pitch",
//                new Servo[]{pitchServo},
//                ServoConstants.Pitch.MIN_DEG,
//                ServoConstants.Pitch.MAX_DEG,
//                ServoConstants.Pitch.AT_MIN,
//                ServoConstants.Pitch.AT_MAX
//        );
//
//        subsystems.add(turret);
//        subsystems.add(pitch);

        // Setup for each subsystem
        for (Subsystem subsystem : subsystems) {
            subsystem.setup();
        }

        List<RayCasting.Polygon> zones = List.of(
                RayCasting.Polygon.builder("redGarden")
                        .addPoint(0, 0)
                        .addPoint(0, 70.75)
                        .addPoint(141.5, 70.75)
                        .addPoint(141.5, 0)
                        .build(),

                RayCasting.Polygon.builder("blueGarden")
                        .addPoint(0, 141.5)
                        .addPoint(0, 70.75)
                        .addPoint(141.5, 70.75)
                        .addPoint(141.5, 141.5)
                        .build()
        );

        this.rayCast = new RayCasting(zones, 141.5);

        updateTarget();

        joined = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                telemetry
        );

        joined.setMsTransmissionInterval(250);
        joined.addLine("Initialized");
        joined.update();
    }

    /**
     * Master update routine that clears hardware bulk caches and
     * ticks all currently active subsystems.
     *
     * <p>Must be called exactly once per loop cycle.
     */
    public void update() {

        // Clear hardware bulk cache
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // Run background logic only for ACTIVE subsystems. Drive is first in this list,
        // so its update refreshes Pinpoint before the turret receives the current pose.
        for (Subsystem subsystem : subsystems) {
            if (subsystem.isActive()) {

                if (subsystem == drive){
                    updatePinpointPose();
                }

                if (subsystem == intake){
                    intake.setBallCount(transfer.getBallCount());
                }

                if (subsystem == turret) {
                    turret.setRobotPose(getFieldPose());
                    turret.setTargetPose(target);
                    turret.update();
                }

                subsystem.update();
            }
        }

        rayCast.update(drive.getFieldX(), drive.getFieldY());

        prepareShooter();

        if (debug) debug();
    }

    public void stop(){
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }
    }


    // ==========================================================
    // DRIVETRAIN
    // ==========================================================

    public void setDrive(double y, double x, double rx, double multiplier){
        drive.setFODDrive(y, x, rx, local.getHeading(AngleUnit.RADIANS), multiplier);
    }

    /** Current Pinpoint pose, in inches and radians. */
    private void updatePinpointPose() {
        lastLocal = local;
        local =  new Pose2D(DistanceUnit.INCH,
                odo.getPosX(DistanceUnit.INCH),
                odo.getPosY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                odo.getHeading(AngleUnit.RADIANS));
    }


    // ==========================================================
    // INTAKE/TRANSFER
    // ==========================================================

    public void startIntaking(){
        boolean ready = transfer.getBallCount() == 0;
        intake.intakeRequest(ready);
        transfer.intakeRequest(ready);
    }

    public void startSpit(){
        boolean ready = transfer.getBallCount() > 0;
        intake.spitRequest(ready);
        transfer.spitRequest(ready);
    }

    public void interruptStop(){
        intake.interruptStop();
        transfer.interruptStop();
    }

    // ==========================================================
    // LAUNCHER
    // ==========================================================

    public void switchSide(){
        isAudience = !isAudience;
        updateTarget();
    }

    public void updateTarget(){
        //temporary
        if (isAudience) target = new Pose2D(DistanceUnit.INCH, 64.5, 88, AngleUnit.DEGREES, 0.0);
        else target = new Pose2D(DistanceUnit.INCH, 64.5, 63.5, AngleUnit.DEGREES, 0.0);
    }

    public void prepareShooter(){
        launcher.autoPIDF(getFieldPose(), target);
    }

    /**
     * Current robot pose in the FIELD frame (x right, y forward, inches/radians) — converted
     * from the Pinpoint's native x-forward / y-left axes. Targets and zones use this frame.
     */
    private Pose2D getFieldPose() {
        return drive.getFieldPose();
    }

    // ==========================================================
    // INTAKE/TRANSFER
    // ==========================================================

    public void flowerDown(){ flower.setAngle(flower.getAxis().getMaxDeg());}

    public void flowerUp(){ flower.setAngle(flower.getAxis().getMinDeg());}


    // ==========================================================
    // VISION
    // ==========================================================

    /** Turns the Limelight on for one scan window to read AprilTags. */
    public void startTagScan(){ vision.startScan(); }

    // ==========================================================
    // FULL BOT
    // ==========================================================

//    public void toggleAutoShoot(boolean set){ autoShoot = set; }

    public void launch(){
        boolean ready = /*Math.abs(local.getX(DistanceUnit.INCH) - lastLocal.getX(DistanceUnit.INCH)) < 0.5
                && Math.abs(local.getY(DistanceUnit.INCH) - lastLocal.getY(DistanceUnit.INCH)) < 0.5
                && rayCast.isInside() &&*/ launcher.isAtTarget() && transfer.getBallCount() > 0;
        intake.launchRequest(ready);
        transfer.launchRequest(ready);
    }

    // ==========================================================
    // DEBUG
    // ==========================================================

    /** Toggles debug */
    public void setDebug(boolean set){ debug = set; }

    /** Adds debug telemetry */
    public void debug(){
        joined.addLine("Time Control:");
        joined.addData("Loop Time (ms): ", time.milliseconds());
        joined.addLine();

        joined.addLine("Subsystem Active:");
        joined.addData("Drive Subsystem: ", drive.isActive());
        joined.addData("Intake Subsystem: ", intake.isActive());
        joined.addData("Transfer Subsystem: ", transfer.isActive());
        joined.addData("Launcher Subsystem: ", launcher.isActive());
        joined.addData("Turret Subsystem: ", turret.isActive());
        joined.addLine();

        joined.addLine("Turret:");
        joined.addData("Turret calibration source: ", turretCalSource);
        joined.addData("Turret angle (deg): ", turret.getAngle());
        joined.addData("Turret target angle (deg): ", turret.getAimTargetAngleDeg());
        joined.addData("Turret requested angle (deg): ", turret.getRawAimAngleDeg());
        joined.addData("Turret target clipped: ", turret.isAimTargetClipped());
        joined.addData("Flower Subsystem: ", flower.isActive());
        joined.addLine();

        joined.addLine("Vision (AprilTags):");
        joined.addData("Limelight on: ", vision.isScanning());
        joined.addData("Scan time left (s): ", vision.getTimeLeft());
        joined.addData("Tag IDs: ", vision.getTagIds().toString());
        joined.addData("Tipped: ", vision.getTipped() ? "TRUE" : "FALSE");
        joined.addLine();

        joined.addLine("Odo Position:");
        joined.addData("Target: ", isAudience ? "AUDIENCE" : "SCORING");
        joined.addData("Field X (right +)", drive.getFieldX());
        joined.addData("Field Y (forward +)", drive.getFieldY());
        joined.addData("Heading (rad, CCW +)", drive.getHeadingRad());
        joined.addData("Pinpoint raw X (fwd +)", odo.getPosX(DistanceUnit.INCH));
        joined.addData("Pinpoint raw Y (left +)", odo.getPosY(DistanceUnit.INCH));
        joined.addLine();

        joined.addLine("Transfer Sensors:");
        joined.addData("l1: ", transfer.l1.getState());
        joined.addData("l2: ", transfer.l2.getState());
        joined.addData("l3: ", transfer.l3.getState());
        joined.addData("l4: ", transfer.l4.getState());
        joined.addData("r1: ", transfer.r1.getState());
        joined.addData("r2: ", transfer.r2.getState());
        joined.addData("r3: ", transfer.r3.getState());
        joined.addData("r4: ", transfer.r4.getState());
        joined.addLine();

        joined.addLine("Launcher RPM");
        joined.addData("fwL: ", launcher.getFwLCurrentRPM());
        joined.addData("fwR: ", launcher.getFwRCurrentRPM());

        joined.update();
    }

}
