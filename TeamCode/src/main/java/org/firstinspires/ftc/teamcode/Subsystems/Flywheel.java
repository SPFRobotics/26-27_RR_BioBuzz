package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.function.DoubleSupplier;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.units.Units;

@Config
public class Flywheel implements Mechanism {

    public static double kP = 0.00983;
    public static double kI = 0.0;
    public static double kD = 0.000001;
    public static double kV = 0.000380;
    public static double kS = 0.135;
    public static double rpmTolerance = 100.0;

    private static  double ticks = 28.0;
    private static final double gearRatio = 18.0 / 16.0;
    private static final double flywheelTicks =
            ticks * gearRatio;


    // Inches, RPM value


    private static final double[][] lookupTable = {
            {5, 6000} // placeholder
    };

    private final NextMotor motor;
    private final NavigableMap<Double, Double> rpmTable = new TreeMap<>();
    private double targetRPM = 0.0;

    public Flywheel(String motorName) {
        motor = new NextMotor(motorName,
                Units.getRadians(2.0 * Math.PI / flywheelTicks));
        motor.setDirection(NextMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.FLOAT);

        for (double[] point : lookupTable) {
            addCalibrationPoint(point[0], point[1]);
        }
    }

    public Flywheel() {
        this("OuttakeMotor");
    }

    public void addCalibrationPoint(double distance, double rpm) {
        requireNonNegativeFinite(distance, "Distance");
        requireNonNegativeFinite(rpm, "RPM");
        rpmTable.put(distance, rpm);
    }


    public double distanceToRPM(double distance) {
        requireNonNegativeFinite(distance, "Distance");
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

    public Command runAtDistance(double distance) {
        requireNonNegativeFinite(distance, "Distance");
        return runAtDistance(() -> distance);
    }

    public Command runAtDistance(DoubleSupplier distanceSupplier) {
        return runAtRPM(() -> {
            double distance = distanceSupplier.getAsDouble();
            return Double.isFinite(distance) && distance >= 0.0
                    ? distanceToRPM(distance) : 0.0;
        });
    }

    public Command runAtRPM(double rpm) {
        requireNonNegativeFinite(rpm, "RPM");
        return runAtRPM(() -> rpm);
    }

    private Command runAtRPM(DoubleSupplier rpmSupplier) {
        Runnable update = () -> applyRPM(rpmSupplier.getAsDouble());
        return infinite(update)
                .setStart(() -> {
                    motor.getVelocityPID().reset();
                    update.run();
                })
                .setEnd(condition -> stopMotor());
    }

    public Command stop() {
        return instant(this::stopMotor);
    }

    @Override
    public Command getDefaultCommand() {
        return runAtRPM(0.0);
    }

    public double getRPM() {
        return motor.getEncoderVelocity().getBaseUnitMagnitude() * 60.0 / (2.0 * Math.PI);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtSpeed() {
        return targetRPM > 0.0 && Math.abs(getRPM() - targetRPM) <= rpmTolerance;
    }

    private void applyRPM(double rpm) {
        if (!Double.isFinite(rpm) || rpm <= 0.0) {
            stopMotor();
            return;
        }


        double ticksPerSecondPerRPM = flywheelTicks / 60.0;
        motor.getVelocityConstants()
                .withP(kP * ticksPerSecondPerRPM)
                .withI(kI * ticksPerSecondPerRPM)
                .withD(kD * ticksPerSecondPerRPM)
                .withV(kV * ticksPerSecondPerRPM)
                .withS(kS);
        targetRPM = rpm;
        motor.setVelocitySetpoint(Units.getRpm(rpm));
    }

    private void stopMotor() {
        targetRPM = 0.0;
        motor.setThrottle(0.0);
        motor.getVelocityPID().reset();
    }

    private static void requireNonNegativeFinite(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and non-negative.");
        }
    }
}
