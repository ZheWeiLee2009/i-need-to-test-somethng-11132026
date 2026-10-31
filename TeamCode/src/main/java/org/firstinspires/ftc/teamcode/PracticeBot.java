package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.SubSystems.DriveSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.LauncherSubSystem;
import org.firstinspires.ftc.teamcode.SubSystems.ServoTaskSubsystem;
import org.firstinspires.ftc.teamcode.SubSystems.Subsystem;
import org.firstinspires.ftc.teamcode.utils.RayCasting;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Master hardware mapping and subsystem manager class for PracticeBot.
 *
 * DO MODIFY THIS
 */
public class PracticeBot {

    private final List<LynxModule> allHubs;

    public GoBildaPinpointDriver odo;

    public DcMotorEx fl, bl, fr, br;
    public final List<DcMotorEx> driveMotors;

    public DcMotorEx intake1;
    public final List<DcMotorEx> intakeMotors;

    public DcMotorEx fwL, fwR;
    public final List<DcMotorEx> flywheelMotors;

    public DriveSubSystem drive;
    public IntakeSubSystem intake;
    public LauncherSubSystem launcher;
    private final List<Subsystem> subsystems = new ArrayList<>();

    // ---- Raw servo hardware (for debugging) ----
    public Servo turretLeft, turretRight;
    public Servo pitchServo;

    public final RayCasting rayCast;

    // ---- Task subsystems ----
    public ServoTaskSubsystem turret;
    public ServoTaskSubsystem pitch;

    public HardwareMap hwMap;

    public ElapsedTime time;

    public PracticeBot(HardwareMap hwMap, ElapsedTime runtime) {
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

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        driveMotors = Arrays.asList(fl, bl, fr, br);

        intake1 = this.hwMap.get(DcMotorEx.class, "intake");
        intakeMotors = Arrays.asList(intake1);


        fwL = this.hwMap.get(DcMotorEx.class, "fwL");
        fwR = this.hwMap.get(DcMotorEx.class, "fwR");

        fwL.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelMotors = Arrays.asList(fwL, fwR);

        odo = this.hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(6.50, 3.25, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.resetPosAndIMU();

        // ============================================================
        // Subsystem construction
        // ============================================================

        drive = new DriveSubSystem(driveMotors, odo);
        intake = new IntakeSubSystem(intakeMotors);
        launcher = new LauncherSubSystem(flywheelMotors);

        subsystems.add(drive);
        subsystems.add(intake);
        subsystems.add(launcher);

        // ============================================================
        // Optional subsystem construction
        // ============================================================

//        turretLeft = hwMap.get(Servo.class, ServoConstants.Turret.NAMES[0]);
//        turretRight = hwMap.get(Servo.class, ServoConstants.Turret.NAMES[1]);
//        pitchServo = hwMap.get(Servo.class, ServoConstants.Pitch.NAMES[0]);
//
//        turret = new ServoTaskSubsystem("turret",
//                new Servo[]{turretLeft, turretRight},
//                ServoConstants.Turret.MIN_DEG,
//                ServoConstants.Turret.MAX_DEG,
//                ServoConstants.Turret.AT_MIN,
//                ServoConstants.Turret.AT_MAX
//        );
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

        // Run background logic only for ACTIVE subsystems
        for (Subsystem subsystem : subsystems) {
            if (subsystem.isActive()) {
                subsystem.update();
            }
        }
    }
}