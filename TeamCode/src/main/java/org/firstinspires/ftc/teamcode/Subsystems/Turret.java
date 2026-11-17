package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache.Alliance;

import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;

/** Odometry-based hive aiming. Angles are robot-relative, positive counter-clockwise. */
@Config
public final class Turret {
    public static final class Params {
        // Hidden from dashboard tuning.
        private final String name;
        // Turret position relative to robot center.
        public double forwardOffsetInches;
        public double leftOffsetInches;
        public double homeAngleDegrees = 0.0;
        public double aimTrimDegrees = 0.0;
        public double encoderRevsPerTurretRev = 1.0;
        public boolean encoderReversed = false;
        // Positive power should turn both servos counter-clockwise.
        public boolean servo1Reversed = false;
        public boolean servo2Reversed = false;
        public double kP = 0.015;
        public double kD = 0.0005;
        public double kS = 0.04;
        public double kV = 0.0;
        public double maxPower = 1.0;
        public double toleranceDegrees = 2.0;

        public Params(String name, double forwardOffsetInches, double leftOffsetInches) {
            this.name = name;
            this.forwardOffsetInches = forwardOffsetInches;
            this.leftOffsetInches = leftOffsetInches;
        }
    }

    public static Params NECTAR = new Params("Nectar", -4.51, 4.19);
    public static Params POLLEN = new Params("Pollen", -4.51, -4.25);

    // Pedro field inches; hives pivot at y = 72.
    public static double hiveOffsetInches = 12.75;
    public static double cellAimOffsetInches = 15.2;
    public static double sideHysteresisInches = 3.0;

    private static final double FIELD_CENTER = 72.0;
    private static final double ENCODER_TICKS_PER_REV = 8192.0;

    private enum Mode { IDLE, TRACK_HIVE, HOLD_ANGLE }

    private final Params params;
    private final Alliance alliance;
    private final IntSupplier readTicks;
    private final Runnable resetTicks;
    private final DoubleConsumer servo1;
    private final DoubleConsumer servo2;
    private final LongSupplier clock;

    private Mode mode = Mode.IDLE;
    private double holdDegrees;
    private int hiveSide;
    private int cellSide;
    private double targetX = Double.NaN;
    private double targetY = Double.NaN;
    private double distanceInches = Double.NaN;
    private double hiveAimDegrees = Double.NaN;

    private double angleDegrees;
    private double targetDegrees = Double.NaN;
    private double errorDegrees = Double.NaN;
    private double power;
    private boolean hasPrevious;
    private double previousTarget;
    private double previousError;
    private long previousNanos;

    public static Turret nectar(HardwareMap hardwareMap, String encoderPortName, Alliance alliance) {
        return new Turret(hardwareMap, NECTAR, "nectarTurret1", "nectarTurret2",
                encoderPortName, alliance);
    }

    public static Turret pollen(HardwareMap hardwareMap, String encoderPortName, Alliance alliance) {
        return new Turret(hardwareMap, POLLEN, "pollenTurret1", "pollenTurret2",
                encoderPortName, alliance);
    }

    public Turret(HardwareMap hardwareMap, Params params, String servo1Name, String servo2Name,
                  String encoderPortName, Alliance alliance) {
        this(params, alliance, hardwareMap.get(DcMotor.class, encoderPortName),
                hardwareMap.getAll(LynxModule.class),
                hardwareMap.get(CRServo.class, servo1Name),
                hardwareMap.get(CRServo.class, servo2Name));
    }

    private Turret(Params params, Alliance alliance, DcMotor encoderPort, List<LynxModule> hubs,
                   CRServo servo1, CRServo servo2) {
        this(params, alliance,
                // Raw ticks ignore the shared motor's direction.
                () -> encoderPort.getController().getMotorCurrentPosition(encoderPort.getPortNumber()),
                () -> {
                    // Reset briefly interrupts the motor sharing this port.
                    DcMotor.RunMode mode = encoderPort.getMode();
                    encoderPort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoderPort.setMode(mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER
                            ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode);
                    // Discard counts cached before reset.
                    for (LynxModule hub : hubs) hub.clearBulkCache();
                },
                servo1::setPower, servo2::setPower, System::nanoTime);
    }
    Turret(Params params, Alliance alliance, IntSupplier readTicks, Runnable resetTicks,
           DoubleConsumer servo1, DoubleConsumer servo2, LongSupplier clock) {
        this.params = params;
        this.alliance = alliance != null ? alliance : Alliance.UNKNOWN;
        this.readTicks = readTicks;
        this.resetTicks = resetTicks;
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.clock = clock;
        applyPower(0);
        angleDegrees = readAngle();
    }

    /** Zero once at physical home; the hub keeps this zero across OpModes. */
    public void zero() {
        resetTicks.run();
        angleDegrees = readAngle();
        clearTarget();
    }

    public void trackHive() {
        if (mode != Mode.TRACK_HIVE) clearTarget();
        mode = Mode.TRACK_HIVE;
    }

    public void holdAngle(double degrees) {
        if (mode != Mode.HOLD_ANGLE || degrees != holdDegrees) clearTarget();
        mode = Mode.HOLD_ANGLE;
        holdDegrees = degrees;
    }

    public void stop() {
        mode = Mode.IDLE;
        clearTarget();
        applyPower(0);
    }

    /** Call after odometry updates. Movement requires trackHive() or holdAngle(). */
    public void update(Pose robotPose) {
        long now = clock.getAsLong();
        angleDegrees = readAngle();
        aimAtHive(robotPose);

        double target = mode == Mode.TRACK_HIVE ? hiveAimDegrees
                : mode == Mode.HOLD_ANGLE ? holdDegrees : Double.NaN;
        double error = wrapDegrees(target - angleDegrees);
        if (!Double.isFinite(error)) {
            clearTarget();
            applyPower(0);
            return;
        }

        double seconds = (now - previousNanos) * 1e-9;
        double targetRate = 0.0;
        double errorRate = 0.0;
        if (hasPrevious && seconds > 0.0) {
            targetRate = wrapDegrees(target - previousTarget) / seconds;
            errorRate = wrapDegrees(error - previousError) / seconds;
        }
        previousTarget = target;
        previousError = error;
        previousNanos = now;
        hasPrevious = true;
        targetDegrees = target;
        errorDegrees = error;
        // Keep feedforward active inside the aiming tolerance.
        double command = params.kV * targetRate;
        if (Math.abs(error) > params.toleranceDegrees) {
            command += params.kP * error + params.kD * errorRate + params.kS * Math.signum(error);
        }
        applyPower(command);
    }

    private void aimAtHive(Pose robot) {
        if (robot == null || !Double.isFinite(robot.x()) || !Double.isFinite(robot.y())
                || !Double.isFinite(robot.heading())) {
            hiveAimDegrees = Double.NaN;
            return;
        }

        int previousHive = hiveSide;
        int previousCell = cellSide;
        // Unknown alliance uses the robot's half of the field.
        hiveSide = alliance == Alliance.RED ? -1
                : alliance == Alliance.BLUE ? 1 : sideOf(robot.x(), hiveSide);
        // Aim at the cell on the robot's side.
        cellSide = sideOf(robot.y(), cellSide);
        // Target switches must not count as motion.
        if (hiveSide != previousHive || cellSide != previousCell) hasPrevious = false;
        targetX = FIELD_CENTER + hiveSide * hiveOffsetInches;
        targetY = FIELD_CENTER + cellSide * cellAimOffsetInches;

        double heading = robot.heading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double turretX = robot.x() + params.forwardOffsetInches * cos - params.leftOffsetInches * sin;
        double turretY = robot.y() + params.forwardOffsetInches * sin + params.leftOffsetInches * cos;
        double dx = targetX - turretX;
        double dy = targetY - turretY;
        distanceInches = Math.hypot(dx, dy);
        hiveAimDegrees = wrapDegrees(
                Math.toDegrees(Math.atan2(dy, dx) - heading) + params.aimTrimDegrees);
    }

    private static int sideOf(double coordinate, int previous) {
        if (previous == 0) return coordinate < FIELD_CENTER ? -1 : 1;
        if (coordinate < FIELD_CENTER - sideHysteresisInches) return -1;
        if (coordinate > FIELD_CENTER + sideHysteresisInches) return 1;
        return previous;
    }

    private double readAngle() {
        double turns = readTicks.getAsInt()
                / (ENCODER_TICKS_PER_REV * params.encoderRevsPerTurretRev);
        return params.homeAngleDegrees + (params.encoderReversed ? -360.0 : 360.0) * turns;
    }

    private void clearTarget() {
        targetDegrees = Double.NaN;
        errorDegrees = Double.NaN;
        hasPrevious = false;
    }

    private void applyPower(double requested) {
        double limit = Math.min(1.0, Math.abs(params.maxPower));
        double clipped = Math.max(-limit, Math.min(limit, requested));
        power = Double.isFinite(clipped) ? clipped : 0.0;
        servo1.accept(params.servo1Reversed ? -power : power);
        servo2.accept(params.servo2Reversed ? -power : power);
    }

    public boolean isAimed() {
        return Math.abs(errorDegrees) <= params.toleranceDegrees;
    }

    /** Turret-to-cell distance in inches; NaN until a valid pose. */
    public double getDistanceToTarget() {
        return distanceInches;
    }

    public double getAngleDegrees() {
        return wrapDegrees(angleDegrees);
    }

    public double getTargetAngleDegrees() {
        return targetDegrees;
    }

    public double getErrorDegrees() {
        return errorDegrees;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    public double getCommandedPower() {
        return power;
    }

    public void addTelemetry(Telemetry telemetry) {
        String name = params.name + " turret";
        telemetry.addData(name, mode == Mode.IDLE ? "Idle" : isAimed() ? "Aimed" : "Turning");
        telemetry.addData(name + " angle / target (deg, +left)", "%.1f / %s", getAngleDegrees(),
                Double.isNaN(targetDegrees) ? "none" : String.format("%.1f", targetDegrees));
        if (cellSide == 0) {
            telemetry.addData(name + " aim", "Waiting for a valid pose");
        } else {
            telemetry.addData(name + " aim", "%s hive%s, %s cell, %.1f in at %.1f deg",
                    hiveSide < 0 ? "Red" : "Blue",
                    alliance == Alliance.UNKNOWN ? " (nearest, no cached alliance)" : "",
                    cellSide < 0 ? "audience" : "far", distanceInches, hiveAimDegrees);
        }
    }

    /** Wraps to (-180, 180]. */
    static double wrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) return wrapped - 360.0;
        if (wrapped <= -180.0) return wrapped + 360.0;
        return wrapped;
    }
}
