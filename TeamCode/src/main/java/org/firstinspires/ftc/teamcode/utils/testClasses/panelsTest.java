package org.firstinspires.ftc.teamcode.utils.testClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Panels Test", group = "9 - Utility")
public class panelsTest extends OpMode {

    public static double test = 123;
    public static int testInt = 1;
    public static boolean testBool = false;
    public static String testString = "hello";

    private TelemetryManager panelsTelemetry;
    private String phase = "NONE";
    private int initLoops = 0;
    private int runLoops = 0;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        phase = "INIT";
        sendLine("Panelstest initialized");
    }

    // Runs repeatedly during INIT (before you press Start)
    @Override
    public void init_loop() {
        initLoops++;
        phase = "INIT #4";
        sendTelemetry();
    }

    @Override
    public void start() {
        phase = "RUNNING";
        resetRuntime();
    }

    // Runs repeatedly after Start
    @Override
    public void loop() {
        runLoops++;
        phase = "RUNNING";
        sendTelemetry();
    }

    @Override
    public void stop() {
        phase = "STOPPED";
    }

    private void sendTelemetry() {
        addLine("Phase", phase);
        addLine("Init loops", String.valueOf(initLoops));
        addLine("Run loops", String.valueOf(runLoops));
        addLine("Runtime (s)", String.valueOf(getRuntime()));
        addLine("Test value", String.valueOf(test));
        addLine("Test int", String.valueOf(testInt));
        addLine("Test bool", String.valueOf(testBool));
        addLine("Test string", testString);

        panelsTelemetry.update();
        telemetry.update();
    }

    // Helper: pushes one key/value pair to both Panels and the Driver Station
    private void addLine(String key, String value) {
        panelsTelemetry.addData(key, value);
        telemetry.addData(key, value);
    }

    private void sendLine(String message) {
        panelsTelemetry.debug(message);
        telemetry.addLine(message);
        panelsTelemetry.update();
        telemetry.update();
    }
}