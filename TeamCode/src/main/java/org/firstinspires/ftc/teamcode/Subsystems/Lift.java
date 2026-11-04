package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;

public class Lift {
    public enum Status { IDLE, MOVING, COMPLETE, CANCELLED, TIMED_OUT, INVALID_ENCODER }

    private static final int MOVE_WRAPS = 2;
    // Observed on this lift: positive servo power (up) decreases encoder position.
    private static final int ENCODER_DIRECTION_FOR_UP = -1;
    private static final double BOTTOM_VOLTAGE = 0.806;
    private static final double FINAL_ZEROED_VOLTAGE = 0.1;
    private static final double UP_EXTRA_DEGREES = 90.0;
    private static final double ENCODER_MAX_VOLTAGE = 3.3;
    // Allow 50 mV of endpoint measurement error, then clamp to the nominal range.
    private static final double ENCODER_VOLTAGE_MARGIN = 0.05;
    private static final double MAX_POWER = 1.0;
    private static final double MIN_POWER = 0.15;
    private static final long MOVE_TIMEOUT_NANOS = 10_000_000_000L;
    private static final long INVALID_READING_GRACE_NANOS = 50_000_000L;

    private final DoubleConsumer setPower;
    private final DoubleSupplier readVoltage;
    private final LongSupplier clock;
    private double previousRotation;
    private double startingRotation;
    private int wrapCount;
    private int targetWrapCount;
    private double targetPosition;
    private boolean encoderInitialized;
    private double commandedPower;
    private double traveledRotations;
    private int direction;
    private long moveStarted;
    private Status status = Status.IDLE;
    private double lastEncoderVoltage = Double.NaN;
    private double lastRejectedVoltage = Double.NaN;
    private long invalidReadingSince = Long.MIN_VALUE;

    public Lift(HardwareMap hardwareMap) {
        this(hardwareMap, "lift", "liftEncoder");
    }

    public Lift(HardwareMap hardwareMap, String servoName, String encoderName) {
        this(hardwareMap.get(CRServo.class, servoName)::setPower,
                hardwareMap.get(AnalogInput.class, encoderName)::getVoltage,
                System::nanoTime);
    }

    Lift(DoubleConsumer setPower, DoubleSupplier readVoltage, LongSupplier clock) {
        this.setPower = setPower;
        this.readVoltage = readVoltage;
        this.clock = clock;
        applyPower(0);
        sampleEncoder();
    }

    public void up() {
        startMove(1);
    }

    public void down() {
        startMove(-1);
    }

    private void startMove(int requestedDirection) {
        // Repeated calls during a move must not restart its target or timer.
        if (direction == requestedDirection) return;
        finish(Status.CANCELLED);
        if (!sampleEncoder()) return;
        traveledRotations = 0;
        startingRotation = wrapCount + previousRotation;
        // Select the nearest endpoint turn, not the integer wrap bucket: tiny
        // changes across zero must not shift the next destination by a whole turn.
        int startingEndpoint = (int) Math.round(startingRotation
                - FINAL_ZEROED_VOLTAGE / ENCODER_MAX_VOLTAGE);
        targetPosition = startingEndpoint + requestedDirection * ENCODER_DIRECTION_FOR_UP * MOVE_WRAPS
                + FINAL_ZEROED_VOLTAGE / ENCODER_MAX_VOLTAGE;
        // Raise only the upper endpoint; down still targets the original lower endpoint.
        if (requestedDirection > 0) {
            targetPosition += ENCODER_DIRECTION_FOR_UP * UP_EXTRA_DEGREES / 360.0;
        }
        targetWrapCount = (int) Math.floor(targetPosition);
        moveStarted = clock.getAsLong();
        direction = requestedDirection;
        status = Status.MOVING;
        applyPower(direction * MAX_POWER);
    }


    public void update() {
        // Keep tracking between moves, including any motion after power is removed.
        if (!sampleEncoder()) return;
        if (!isBusy()) return;

        double position = wrapCount + previousRotation;
        traveledRotations = position - startingRotation;
        double remaining = direction * ENCODER_DIRECTION_FOR_UP * (targetPosition - position);
        if (remaining <= 1e-6) {
            finish(Status.COMPLETE);
        } else if (clock.getAsLong() - moveStarted >= MOVE_TIMEOUT_NANOS) {
            finish(Status.TIMED_OUT);
        } else {
            // Slow down over the final quarter-turn to reduce overshoot.
            double power = Math.max(MIN_POWER, Math.min(MAX_POWER, remaining * 4.0));
            applyPower(direction * power);
        }
    }

    private boolean sampleEncoder() {
        double voltage = readVoltage.getAsDouble();
        lastEncoderVoltage = voltage;
        if (!validVoltage(voltage)) {
            lastRejectedVoltage = voltage;
            // Skip a brief bad sample without counting it or requiring another
            // button press. Sustained invalid feedback still stops the servo.
            if (isBusy()) {
                long now = clock.getAsLong();
                if (invalidReadingSince == Long.MIN_VALUE) invalidReadingSince = now;
                if (now - invalidReadingSince < INVALID_READING_GRACE_NANOS
                        && now - moveStarted < MOVE_TIMEOUT_NANOS) return false;
            }
            finish(Status.INVALID_ENCODER);
            return false;
        }
        invalidReadingSince = Long.MIN_VALUE;

        double rotation = zeroedVoltage(voltage) / ENCODER_MAX_VOLTAGE;
        if (!encoderInitialized) {
            previousRotation = rotation;
            // Init occurs at the bottom. A reading just below 0.806 V is just
            // below zero, not almost one full turn above it.
            wrapCount = rotation > 0.5 ? -1 : 0;
            encoderInitialized = true;
            return true;
        }
        double delta = rotation - previousRotation;
        // Count signed wraps around the bottom offset, not around raw 0 V.
        // Reverse crossings undo a wrap so jitter cannot add completed turns.
        // update() must run faster than half a shaft turn.
        if (delta < -0.5) wrapCount++;
        else if (delta > 0.5) wrapCount--;
        previousRotation = rotation;
        return true;
    }

    public boolean isBusy() {
        return direction != 0;
    }

    public Status getStatus() {
        return status;
    }

    /** Net shaft travel for the most recent move. */
    public double getTraveledRotations() {
        return Math.abs(traveledRotations);
    }

    /** Latest reading used for continuous position tracking. */
    public double getLastEncoderVoltage() {
        return lastEncoderVoltage;
    }

    /** Retained even when subsequent readings are valid. */
    public double getLastRejectedVoltage() {
        return lastRejectedVoltage;
    }

    /** Signed crossings of the zeroed boundary since initialization. */
    public int getWrapCount() {
        return wrapCount;
    }

    public int getTargetWrapCount() {
        return targetWrapCount;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public void stop() {
        finish(Status.CANCELLED);
    }

    private void finish(Status reason) {
        direction = 0;
        applyPower(0);
        status = reason;
    }

    private void applyPower(double power) {
        commandedPower = power;
        setPower.accept(power);
    }

    private static boolean validVoltage(double voltage) {
        return Double.isFinite(voltage) && voltage >= -ENCODER_VOLTAGE_MARGIN
                && voltage <= ENCODER_MAX_VOLTAGE + ENCODER_VOLTAGE_MARGIN;
    }

    /** Subtract the bottom offset and normalize into [0, 3.3) V. */
    public static double zeroedVoltage(double voltage) {
        if (!validVoltage(voltage)) return Double.NaN;
        double shifted = Math.max(0, Math.min(ENCODER_MAX_VOLTAGE, voltage)) - BOTTOM_VOLTAGE;
        return shifted < 0 ? shifted + ENCODER_MAX_VOLTAGE : shifted;
    }
}
