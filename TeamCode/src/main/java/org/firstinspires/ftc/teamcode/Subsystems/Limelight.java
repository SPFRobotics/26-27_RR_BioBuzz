package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache.Alliance;

import java.util.ArrayList;
import java.util.List;

/** Turns toward the largest visible alliance tag's center; the OpMode owns driving. */
public final class Limelight {
    public static final class Config {
        public String hardwareName = "limelight";
        public int pipeline = 2;
        public int pollRateHz = 100;
        public double maxStalenessMs = 100;
        public double turnGain = 0.02;
        public double maxTurnPower = 0.30;
        public double toleranceDegrees = 1.5;
    }

    public enum Status {
        STOPPED, CAMERA_MISSING, PIPELINE_ERROR, NO_TARGET,
        INVALID_FRAME, WRONG_PIPELINE, STALE_FRAME, TRACKING, ALIGNED
    }

    public static final class AimResult {
        public final Status status;
        public final int tagId;
        // Positive error/turn is counterclockwise, matching Pedro manual().
        public final double errorDegrees, turnPower;

        private AimResult(Status status, int tagId, double errorDegrees, double turnPower) {
            this.status = status;
            this.tagId = tagId;
            this.errorDegrees = errorDegrees;
            this.turnPower = turnPower;
        }

        public boolean hasTarget() {
            return status == Status.TRACKING || status == Status.ALIGNED;
        }
    }

    // Hardware-independent camera seam for deterministic selection and drive tests.
    interface Camera {
        boolean start();
        Frame read();
        void stop();
    }

    static final class Detection {
        final int id;
        final double area, txDegrees;
        Detection(int id, double area, double txDegrees) {
            this.id = id;
            this.area = area;
            this.txDegrees = txDegrees;
        }
    }

    static final class Frame {
        final boolean valid;
        final int pipeline;
        final double stalenessMs;
        final List<Detection> detections;
        Frame(boolean valid, int pipeline, double stalenessMs, List<Detection> detections) {
            this.valid = valid;
            this.pipeline = pipeline;
            this.stalenessMs = stalenessMs;
            this.detections = detections;
        }
    }

    private final Camera camera;
    private final Config config;
    private final Alliance alliance;
    private boolean started;
    private AimResult result = empty(Status.STOPPED);

    public Limelight(HardwareMap hardwareMap, Config config, Alliance alliance) {
        this(createCamera(hardwareMap, config), config, alliance);
    }

    Limelight(Camera camera, Config config, Alliance alliance) {
        validate(config);
        this.camera = camera;
        this.config = config;
        this.alliance = alliance != null ? alliance : Alliance.UNKNOWN;
    }

    private static Camera createCamera(HardwareMap hardwareMap, Config config) {
        validate(config);
        Limelight3A device = hardwareMap.tryGet(Limelight3A.class, config.hardwareName);
        if (device == null) return null;
        return new Camera() {
            @Override public boolean start() {
                device.setPollRateHz(config.pollRateHz);
                if (!device.pipelineSwitch(config.pipeline)) return false;
                device.start();
                return true;
            }

            @Override public Frame read() {
                LLResult frame = device.getLatestResult();
                if (frame == null) return null;
                List<Detection> detections = new ArrayList<>();
                if (frame.isValid() && frame.getPipelineIndex() == config.pipeline
                        && frame.getStaleness() >= 0 && frame.getStaleness() < config.maxStalenessMs) {
                    for (LLResultTypes.FiducialResult tag : frame.getFiducialResults()) {
                        detections.add(new Detection(tag.getFiducialId(), tag.getTargetArea(),
                                tag.getTargetXDegrees()));
                    }
                }
                return new Frame(frame.isValid(), frame.getPipelineIndex(),
                        frame.getStaleness(), detections);
            }

            @Override public void stop() { device.stop(); }
        };
    }

    public void start() {
        if (started) return;
        if (camera == null) {
            result = empty(Status.CAMERA_MISSING);
        } else if (!camera.start()) {
            result = empty(Status.PIPELINE_ERROR);
        } else {
            started = true;
            result = empty(Status.NO_TARGET);
        }
    }

    public AimResult update() {
        if (!started) return result;
        Frame frame = camera.read();
        if (frame == null || !frame.valid) return result = empty(Status.INVALID_FRAME);
        if (frame.pipeline != config.pipeline) return result = empty(Status.WRONG_PIPELINE);
        if (!Double.isFinite(frame.stalenessMs) || frame.stalenessMs < 0
                || frame.stalenessMs >= config.maxStalenessMs) {
            return result = empty(Status.STALE_FRAME);
        }

        Detection best = null;
        if (frame.detections != null) for (Detection tag : frame.detections) {
            if (tag == null || !Double.isFinite(tag.area) || tag.area <= 0
                    || !Double.isFinite(tag.txDegrees)) continue;
            if (alliance != Alliance.UNKNOWN && tagAlliance(tag.id) != alliance) continue;
            if (best == null || tag.area > best.area) best = tag;
        }
        if (best == null) return result = empty(Status.NO_TARGET);

        // Limelight tx is positive right; Pedro positive rotation is counterclockwise.
        double error = -best.txDegrees;
        boolean aligned = Math.abs(error) <= config.toleranceDegrees;
        double turn = aligned ? 0 : Math.max(-config.maxTurnPower,
                Math.min(config.maxTurnPower, error * config.turnGain));
        return result = new AimResult(aligned ? Status.ALIGNED : Status.TRACKING,
                best.id, error, turn);
    }

    /** Apply precision scaling to either manual turn or hold-to-aim. */
    public double driveTurn(boolean held, double manualTurn, double driveScale) {
        return (held ? result.turnPower : manualTurn) * driveScale;
    }

    public AimResult getResult() { return result; }

    private static Alliance tagAlliance(int id) {
        if (id >= 30 && id <= 37) return Alliance.RED;
        if (id >= 38 && id <= 45) return Alliance.BLUE;
        return Alliance.UNKNOWN;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Tag aim", result.status);
        telemetry.addData("Aim alliance", alliance == Alliance.UNKNOWN ? "Any tag (no cached alliance)" : alliance);
        telemetry.addData("Aim tag (largest visible)", result.tagId);
        telemetry.addData("Aim tag color", tagAlliance(result.tagId));
        telemetry.addData("Aim error (deg, +left)", result.errorDegrees);
        telemetry.addData("Aim turn", result.turnPower);
    }

    public void stop() {
        try {
            if (camera != null && started) camera.stop();
        } finally {
            started = false;
            result = empty(Status.STOPPED);
        }
    }

    private static AimResult empty(Status status) {
        return new AimResult(status, -1, 0, 0);
    }

    private static void validate(Config c) {
        if (c == null || c.hardwareName == null || c.hardwareName.isEmpty()
                || c.pipeline < 0 || c.pipeline > 9 || c.pollRateHz <= 0
                || !Double.isFinite(c.maxStalenessMs) || c.maxStalenessMs <= 0
                || !Double.isFinite(c.turnGain) || c.turnGain < 0
                || !Double.isFinite(c.maxTurnPower) || c.maxTurnPower < 0 || c.maxTurnPower > 1
                || !Double.isFinite(c.toleranceDegrees) || c.toleranceDegrees < 0) {
            throw new IllegalArgumentException("Invalid tag aiming configuration");
        }
    }
}
