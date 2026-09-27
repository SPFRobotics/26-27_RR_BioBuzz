package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;

@Config
@Utility
public class FlywheelPIDTuner extends OpMode {

    public static String pollenMotorName = "OuttakeMotorP";
    public static String nectarMotorName = "OuttakeMotorN";
    public static boolean enablePollen = true;
    public static boolean enableNectar = true;
    public static double pollenTargetRPM = 3000.0;
    public static double nectarTargetRPM = 3000.0;

    private FlywheelPollen pollen;
    private FlywheelNectar nectar;
    private DcMotorEx pollenMotor;
    private DcMotorEx nectarMotor;
    private String selectedPollenMotorName;
    private String selectedNectarMotorName;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        selectedPollenMotorName = resolveMotorName(pollenMotorName, "OuttakeMotorP");
        selectedNectarMotorName = resolveMotorName(nectarMotorName, "OuttakeMotorN");
        pollenMotor = hardwareMap.get(DcMotorEx.class, selectedPollenMotorName);
        nectarMotor = hardwareMap.get(DcMotorEx.class, selectedNectarMotorName);
        if (pollenMotor == nectarMotor) {
            throw new IllegalArgumentException("Pollen and nectar must use different motors.");
        }
        pollen = new FlywheelPollen(hardwareMap, selectedPollenMotorName);
        nectar = new FlywheelNectar(hardwareMap, selectedNectarMotorName);
        stopWheel();
    }

    private String resolveMotorName(String configuredName, String defaultName) {
        return configuredName == null || configuredName.trim().isEmpty()
                ? defaultName : configuredName.trim();
    }

    @Override
    public void init_loop() {
        showInstructions();
        telemetry.update();
    }

    @Override
    public void loop() {
        showInstructions();
        enablePollen = updateWheel(false, enablePollen, pollenTargetRPM);
        enableNectar = updateWheel(true, enableNectar, nectarTargetRPM);
        telemetry.update();
    }

    private boolean updateWheel(boolean tuningNectar, boolean enabled, double requestedRPM) {
        double p = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kP : FlywheelPollen.FlywheelVars.kP;
        double i = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kI : FlywheelPollen.FlywheelVars.kI;
        double d = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kD : FlywheelPollen.FlywheelVars.kD;
        double v = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kV : FlywheelPollen.FlywheelVars.kV;
        double s = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kS : FlywheelPollen.FlywheelVars.kS;
        double tolerance = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.rpmTolerance
                : FlywheelPollen.FlywheelVars.rpmTolerance;
        boolean valid = Double.isFinite(requestedRPM) && requestedRPM >= 0.0
                && Double.isFinite(p) && Double.isFinite(i) && Double.isFinite(d)
                && Double.isFinite(v) && Double.isFinite(s)
                && Double.isFinite(tolerance) && tolerance >= 0.0;

        if (!valid) {
            enabled = false;
        } else if (tuningNectar) {
            FlywheelNectar.kP = p;
            FlywheelNectar.kI = i;
            FlywheelNectar.kD = d;
            FlywheelNectar.kV = v;
            FlywheelNectar.kS = s;
            FlywheelNectar.rpmTolerance = tolerance;
        } else {
            FlywheelPollen.kP = p;
            FlywheelPollen.kI = i;
            FlywheelPollen.kD = d;
            FlywheelPollen.kV = v;
            FlywheelPollen.kS = s;
            FlywheelPollen.rpmTolerance = tolerance;
        }

        double commandedRPM = enabled ? requestedRPM : 0.0;
        if (tuningNectar) {
            nectar.runAtRPM(commandedRPM);
            nectar.update();
        } else {
            pollen.runAtRPM(commandedRPM);
            pollen.update();
        }

        double actualRPM = tuningNectar ? nectar.getRPM() : pollen.getRPM();
        DcMotorEx motor = tuningNectar ? nectarMotor : pollenMotor;
        String label = tuningNectar ? "Nectar " : "Pollen ";
        telemetry.addData(label + "Enabled", enabled);
        telemetry.addData(label + "Valid settings", valid);
        telemetry.addData(label + "Target RPM", commandedRPM);
        telemetry.addData(label + "Actual RPM", actualRPM);
        telemetry.addData(label + "Error RPM", commandedRPM - actualRPM);
        telemetry.addData(label + "Motor power", motor.getPower());
        telemetry.addData(label + "At speed", commandedRPM > 0.0
                && Math.abs(commandedRPM - actualRPM) <= tolerance);
        return enabled;
    }

    private void showInstructions() {
        telemetry.addData("Pollen motor", selectedPollenMotorName);
        telemetry.addData("Nectar motor", selectedNectarMotorName);
        telemetry.addLine("Dashboard: FlywheelPIDTuner.enablePollen / enableNectar start or stop each wheel.");
        telemetry.addLine("Dashboard RPM: FlywheelPIDTuner.pollenTargetRPM / nectarTargetRPM.");
        telemetry.addLine("Gains: FlywheelVars / FlywheelVarsNectar.");
    }

    private void stopWheel() {
        if (pollen != null) {
            pollen.stop();
        }
        if (nectar != null) {
            nectar.stop();
        }
    }

    @Override
    public void stop() {
        enablePollen = false;
        enableNectar = false;
        stopWheel();
    }
}
