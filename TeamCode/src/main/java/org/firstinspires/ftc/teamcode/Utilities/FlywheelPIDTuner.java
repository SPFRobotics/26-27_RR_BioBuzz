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

    public static String motorName = "";
    public static boolean useNectar = false;
    public static double targetRPM = 3000.0;

    private FlywheelPollen pollen;
    private FlywheelNectar nectar;
    private DcMotorEx motor;
    private boolean tuningNectar;
    private String selectedMotorName;
    private boolean enabled;
    private boolean previousA;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tuningNectar = useNectar;
        if (motorName == null || motorName.trim().isEmpty()) {
            if (tuningNectar) {
                selectedMotorName = "OuttakeMotorN";
            } else {
                selectedMotorName = "OuttakeMotorP";
            }
        } else {
            selectedMotorName = motorName.trim();
        }
        motor = hardwareMap.get(DcMotorEx.class, selectedMotorName);
        if (tuningNectar) {
            nectar = new FlywheelNectar(hardwareMap, selectedMotorName);
        } else {
            pollen = new FlywheelPollen(hardwareMap, selectedMotorName);
        }
        enabled = false;
        stopWheel();
    }

    @Override
    public void init_loop() {
        showInstructions();
        telemetry.update();
    }

    @Override
    public void start() {
        enabled = false;
        previousA = gamepad1.dpad_up;
        stopWheel();
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !previousA) {
            enabled = !enabled;
        }
        previousA = gamepad1.dpad_up;
        if (gamepad1.dpad_down) {
            enabled = false;
        }

        double p = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kP : FlywheelPollen.FlywheelVars.kP;
        double i = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kI : FlywheelPollen.FlywheelVars.kI;
        double d = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kD : FlywheelPollen.FlywheelVars.kD;
        double v = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kV : FlywheelPollen.FlywheelVars.kV;
        double s = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.kS : FlywheelPollen.FlywheelVars.kS;
        double tolerance = tuningNectar ? FlywheelNectar.FlywheelVarsNectar.rpmTolerance
                : FlywheelPollen.FlywheelVars.rpmTolerance;
        double requestedRPM = targetRPM;
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
        showInstructions();
        telemetry.addData("Enabled", enabled);
        telemetry.addData("Valid settings", valid);
        telemetry.addData("Target RPM", commandedRPM);
        telemetry.addData("Actual RPM", actualRPM);
        telemetry.addData("Error RPM", commandedRPM - actualRPM);
        telemetry.addData("Motor power", motor.getPower());
        telemetry.addData("At speed", commandedRPM > 0.0
                && Math.abs(commandedRPM - actualRPM) <= tolerance);
        telemetry.update();
    }

    private void showInstructions() {
        telemetry.addData("Motor", selectedMotorName);
        telemetry.addData("Dashboard gains", tuningNectar ? "FlywheelVarsNectar" : "FlywheelVars");
        telemetry.addLine("Gamepad 1: A toggles flywheel; B stops. Dashboard: FlywheelPIDTuner.targetRPM.");
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
        enabled = false;
        stopWheel();
    }
}
