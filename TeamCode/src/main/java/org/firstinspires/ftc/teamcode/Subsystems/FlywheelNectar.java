package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class FlywheelNectar {
    @Config
    public static class FlywheelVarsNectar{

        public static double kP = 0.00983;
        public static double kI = 0.0;
        public static double kD = 0.000001;
        public static double kV = 0.000380;
        public static double kS = 0.135;
        public static double rpmTolerance = 100.0;

    }


    public static double kP = FlywheelVarsNectar.kP;
    public static double kI = FlywheelVarsNectar.kI;
    public static double kD = FlywheelVarsNectar.kD;
    public static double kV = FlywheelVarsNectar.kV;
    public static double kS = FlywheelVarsNectar.kS;
    public static double rpmTolerance = FlywheelVarsNectar.rpmTolerance;
    private static final double ticks = 28.0;
    private static final double gearRatio = 18.0 / 16.0;
    private static final double flywheelTicks =
            ticks * gearRatio;


    // Inches, RPM value


    private static final double[][] lookupTable = {
            {5, 6000} // placeholder
    };

    private final DcMotorEx motor;
    private final ElapsedTime controlTimer = new ElapsedTime();
    private final NavigableMap<Double, Double> rpmTable = new TreeMap<>();
    private double targetRPM = 0.0;
    private double integral = 0.0;
    private double previousError = 0.0;
    private boolean hasPreviousError = false;

    public FlywheelNectar(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (double[] point : lookupTable) {
            addCalibrationPoint(point[0], point[1]);
        }
    }

    public FlywheelNectar(HardwareMap hardwareMap) {
        this(hardwareMap, "OuttakeMotor");
    }

    public void addCalibrationPoint(double distance, double rpm) {

        rpmTable.put(distance, rpm);
    }


    public double distanceToRPM(double distance) {
        if (rpmTable.isEmpty()) {
            return 0.0;
        }

        Map.Entry<Double, Double> lower = rpmTable.floorEntry(distance);
        Map.Entry<Double, Double> upper = rpmTable.ceilingEntry(distance);
        if (lower == null) {
            return rpmTable.firstEntry().getValue();
        }
        if (upper == null || lower.getKey().equals(upper.getKey())) {
            return lower.getValue();
        }

        double fraction = (distance - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue() + fraction * (upper.getValue() - lower.getValue());
    }

//units in inches, same as PP
public void runAtDistance(double distance) {
        runAtRPM(distanceToRPM(distance));
    }

    public void runAtRPM(double rpm) {
        if (rpm == 0.0) {
            stop();
            return;
        }
        if (targetRPM == 0.0) {
            resetController();
        }
        targetRPM = rpm;
    }

    public double getRPM() {
        return motor.getVelocity() * 60.0 / flywheelTicks;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtSpeed() {
        return targetRPM > 0.0 && Math.abs(getRPM() - targetRPM) <= rpmTolerance;
    }

    // manual PID FTC PID sucks
    public void update() {
        if (targetRPM == 0.0) {
            stop();
            return;
        }

        double targetTicksPerSecond = targetRPM * flywheelTicks / 60.0;
        double error = targetTicksPerSecond - motor.getVelocity();
        double dt = controlTimer.seconds();
        controlTimer.reset();
        double derivative = hasPreviousError && dt > 0.0
                ? (error - previousError) / dt : 0.0;
        double nextIntegral = kI != 0.0 && hasPreviousError && dt > 0.0
                ? integral + error * dt : 0.0;
        double power = kP * error + kI * nextIntegral + kD * derivative
                + kV * targetTicksPerSecond + kS;

        if (Math.abs(power) <= 1.0 || Math.signum(kI * error) != Math.signum(power)) {
            integral = nextIntegral;
        }
        previousError = error;
        hasPreviousError = true;
        motor.setPower(Range.clip(power, -1.0, 1.0));
    }

    //run on stop duh
    public void stop() {
        targetRPM = 0.0;
        motor.setPower(0.0);
        resetController();
    }

    //resets kI
    private void resetController() {
        integral = 0.0;
        previousError = 0.0;
        hasPreviousError = false;
        controlTimer.reset();
    }


}
