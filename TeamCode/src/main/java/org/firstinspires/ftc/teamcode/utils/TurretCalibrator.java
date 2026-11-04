package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Configs.roboConfigs.ServoConstants;
import org.firstinspires.ftc.teamcode.Robot.LM1Robot;

import java.util.Map;

/**
 * Init-phase turret calibration for LM1Robot.
 *
 * <p>Mark two points (A = zero, B = 90 deg clockwise). This works out the
 * servo-position <-> turret-angle relationship and the turret's maximum angle each way, from the
 * servo's full 0..1 range. It prints those angles and a {@code ServoConstants.Turret} block to
 * paste in; that file is where the calibration is kept. Nothing else is written anywhere.
 *
 * <p>Both turret servos are geared together with matching limits, so they are always commanded
 * to the same position and share one pair of AT_MIN / AT_MAX values.
 *
 * <p>Whenever the numbers on screen are valid they are also placed on the blackboard (see
 * {@link TurretCalibrationStore}) so the next OpMode's LM1Robot uses them. Options removes them.
 * Pasting into ServoConstants is what makes them permanent.
 *
 * <p>Use in any OpMode / Auto that already has the robot:
 * <pre>
 *   private TurretCalibrator turretCal;
 *   init()      { bot = new LM1Robot(hardwareMap, runtime, telemetry, blackboard);
 *                 turretCal = new TurretCalibrator(bot, gamepad2, telemetry, blackboard); }
 *   init_loop() { turretCal.update(); }   // OpMode pushes telemetry for you in init_loop
 * </pre>
 *
 * <p>Controls:
 * <pre>
 *   Left stick X          move turret servos
 *   A                     mark ZERO  (turret facing the direction you want as 0)
 *   B                     mark 90    (turret turned 90 deg CLOCKWISE). Ignored until A is set.
 *   X / Y                 jump to the marked ZERO / 90 position
 *   Right bumper/trigger  jump to the absolute MAX (servo position 1.0)
 *   Left bumper/trigger   jump to the absolute MIN (servo position 0.0)
 *   Options               restart (clears both marks)
 *   Dpad                  unused
 * </pre>
 * Once both marks exist, A and B can be pressed again at any time to redo either one.
 */
public class TurretCalibrator {

    /** Known reference rotation, degrees clockwise from zero. */
    public static final double REFERENCE_DEG = 90.0;
    /** A and B must be at least this far apart (servo position) to give a usable ratio. */
    private static final double MIN_SPAN = 0.01;

    private static final double STICK_RATE_PER_SEC = 0.15;   // servo position per second at full stick
    private static final double STICK_DEADBAND = 0.05;
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final Servo[] servos;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final Map<String, Object> blackboard;
    private final ElapsedTime dt = new ElapsedTime();

    private double position;              // what we are commanding right now (both servos)
    private double lastWritten = Double.NaN;
    private boolean rtDown, ltDown;

    private double zeroPos = Double.NaN;  // A
    private double ninetyPos = Double.NaN;// B
    private String message = "";

    public TurretCalibrator(LM1Robot bot, Gamepad gamepad, Telemetry telemetry,
                            Map<String, Object> blackboard) {
        this.servos = bot.turret.getServos();
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.blackboard = blackboard;

        // Starting value only; the turret subsystem already commanded this, so nothing moves.
        this.position = bot.turret.getPosition(0);
        dt.reset();
    }

    /** Call every init_loop(). */
    public void update() {
        double elapsed = dt.seconds();
        dt.reset();

        // ---- move ----
        double stick = gamepad.left_stick_x;
        if (Math.abs(stick) > STICK_DEADBAND) position += stick * STICK_RATE_PER_SEC * elapsed;

        boolean rt = gamepad.right_trigger > TRIGGER_THRESHOLD;
        boolean lt = gamepad.left_trigger  > TRIGGER_THRESHOLD;
        if (gamepad.rightBumperWasPressed() || (rt && !rtDown)) position = ServoUtility.SERVO_MAX;
        if (gamepad.leftBumperWasPressed()  || (lt && !ltDown)) position = ServoUtility.SERVO_MIN;
        rtDown = rt;
        ltDown = lt;

        if (gamepad.xWasPressed() && hasZero())   position = zeroPos;
        if (gamepad.yWasPressed() && hasNinety()) position = ninetyPos;

        position = ServoUtility.clamp(position, ServoUtility.SERVO_MIN, ServoUtility.SERVO_MAX);
        write();

        // ---- mark ----
        if (gamepad.aWasPressed()) {
            zeroPos = position;
            message = "Zero marked.";
        }
        if (gamepad.bWasPressed()) {
            if (hasZero()) {
                ninetyPos = position;
                message = "90 deg marked.";
            } else {
                message = "Press A first (mark ZERO).";
            }
        }
        if (gamepad.optionsWasPressed()) {
            zeroPos = Double.NaN;
            ninetyPos = Double.NaN;
            message = "Restarted. Mark ZERO with A.";
        }

        publish();
        report();
    }

    // ================================================================
    //  THE MATH  (pure; uses ServoUtility)
    // ================================================================

    private boolean hasZero()   { return !Double.isNaN(zeroPos); }
    private boolean hasNinety() { return !Double.isNaN(ninetyPos); }

    private boolean isCalibrated() {
        return hasZero() && hasNinety() && Math.abs(ninetyPos - zeroPos) >= MIN_SPAN;
    }

    /** Turret angle (deg, + = clockwise from zero) at a servo position. Not clamped. */
    static double angleAt(double pos, double zero, double ninety) {
        return ServoUtility.positionToDegrees(pos, 0.0, REFERENCE_DEG, zero, ninety);
    }

    /** Servo position at a turret angle. Not clamped. */
    static double positionAt(double deg, double zero, double ninety) {
        return ServoUtility.mapRange(deg, 0.0, REFERENCE_DEG, zero, ninety);
    }

    /**
     * Turret limits from the servo's full range.
     * @return { minDeg, maxDeg, atMin, atMax } where atMin/atMax are the servo positions at
     *         minDeg/maxDeg (0 and 1, in whichever order matches the servo's direction).
     */
    static double[] limits(double zero, double ninety) {
        double a0 = angleAt(ServoUtility.SERVO_MIN, zero, ninety);
        double a1 = angleAt(ServoUtility.SERVO_MAX, zero, ninety);
        double minDeg = Math.min(a0, a1);
        double maxDeg = Math.max(a0, a1);
        double atMin = ServoUtility.clamp(positionAt(minDeg, zero, ninety), 0.0, 1.0);
        double atMax = ServoUtility.clamp(positionAt(maxDeg, zero, ninety), 0.0, 1.0);
        return new double[]{ minDeg, maxDeg, atMin, atMax };
    }

    /** Keep the blackboard equal to what the screen shows: calibrated -> published, else removed. */
    private void publish() {
        if (!isCalibrated()) {
            TurretCalibrationStore.clear(blackboard);
            return;
        }
        double[] l = limits(zeroPos, ninetyPos);
        TurretCalibrationStore.put(blackboard, l[0], l[1], l[2], l[3]);
    }

    // ================================================================
    //  HARDWARE + OUTPUT
    // ================================================================

    private void write() {
        if (position == lastWritten) return;
        for (Servo s : servos) s.setPosition(position);
        lastWritten = position;
    }

    /**
     * Prints every number. Called by update(); also call it on its own (e.g. in loop()) to keep
     * the numbers on screen without reading any buttons.
     */
    public void report() {
        telemetry.addLine("=== TURRET CALIBRATION ===");
        if (!hasZero())        telemetry.addLine("1) Move turret to the ZERO direction, press A");
        else if (!hasNinety()) telemetry.addLine("2) Turn turret 90 deg CLOCKWISE, press B");
        else                   telemetry.addLine("Done. A / B again to redo a point. Options = restart.");
        if (!message.isEmpty()) telemetry.addLine(message);

        telemetry.addData("Servo position", "%.4f", position);
        telemetry.addData("Zero (A)", hasZero() ? String.format("%.4f", zeroPos) : "not set");
        telemetry.addData("90 CW (B)", hasNinety() ? String.format("%.4f", ninetyPos) : "not set");

        boolean ok = isCalibrated();
        if (hasZero() && hasNinety() && !ok) {
            telemetry.addLine("A and B are too close together. Turn the turret further before B.");
        }
        double[] l = ok ? limits(zeroPos, ninetyPos) : null;

        telemetry.addLine("--- Calculated turret range (from the servo's full 0..1 travel) ---");
        telemetry.addData("Max clockwise", ok ? String.format("%+.1f deg", l[1]) : "n/a");
        telemetry.addData("Max counterclockwise", ok ? String.format("%+.1f deg", l[0]) : "n/a");
        telemetry.addData("Total sweep", ok ? String.format("%.1f deg", l[1] - l[0]) : "n/a");
        telemetry.addData("Turret now", ok
                ? String.format("%+.1f deg (+ = clockwise)", angleAt(position, zeroPos, ninetyPos)) : "n/a");
        telemetry.addData("Servo full range", ok
                ? String.format("%.0f deg (expect ~180/270/300 if 1:1)", Math.abs(REFERENCE_DEG / (ninetyPos - zeroPos))) : "n/a");
        telemetry.addData("Carried to next OpMode (blackboard)", ok ? "yes" : "no");

        telemetry.addLine("--- Paste into ServoConstants.Turret ---");
        telemetry.addData("MIN_DEG", ok ? String.format("%.1f", l[0]) : "n/a");
        telemetry.addData("MAX_DEG", ok ? String.format("%.1f", l[1]) : "n/a");
        telemetry.addData("AT_MIN", ok ? array(l[2], servos.length) : "n/a");
        telemetry.addData("AT_MAX", ok ? array(l[3], servos.length) : "n/a");
        telemetry.addLine("(NAMES stay " + java.util.Arrays.toString(ServoConstants.Turret.NAMES) + ")");
    }

    private static String array(double v, int n) {
        StringBuilder sb = new StringBuilder("{ ");
        for (int i = 0; i < n; i++) sb.append(i > 0 ? ", " : "").append(String.format("%.3f", v));
        return sb.append(" }").toString();
    }
}
