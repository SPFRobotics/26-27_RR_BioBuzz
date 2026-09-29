package org.firstinspires.ftc.teamcode.Utilities.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;

/** Neural pollen selection adapted from FTC Teams 5193/10653. See UPSTREAM_LICENSE. */
public final class PollenVision {
    /** Set before start; areas are fractions of the image, not percentages. */
    public static final class Config {
        public String hardwareName = "limelight";
        public int pipeline = 0;
        public int pollRateHz = 100;
        public double minConfidence = 0.5;
        public double maxStalenessMs = 200;
        public double lostTimeoutSeconds = 0.5;
        public double groupGap = 3.3;
        public double degreesPerSqrtArea = 48.0;
        public double scorePerBall = 1.0;
        public double scorePerArea = 8.0;
        public double switchHysteresis = 1.25;
        public double matchAngleDegrees = 10.0;
    }

    /** Immutable snapshot; target values remain available during the loss grace period. */
    public static final class Target {
        public final boolean available;
        public final int pollenCount, groupCount, targetCount;
        public final double txDegrees, area, score, confidence;

        Target(boolean available, int pollenCount, int groupCount, int targetCount,
               double txDegrees, double area, double score, double confidence) {
            this.available = available;
            this.pollenCount = pollenCount;
            this.groupCount = groupCount;
            this.targetCount = targetCount;
            this.txDegrees = txDegrees;
            this.area = area;
            this.score = score;
            this.confidence = confidence;
        }
    }

    // Hardware-independent frame seam used by the deterministic tests.
    interface Camera {
        void start();
        Frame read();
        void stop();
    }

    static final class Detection {
        final String label;
        final double confidence, tx, ty, area;
        Detection(String label, double confidence, double tx, double ty, double area) {
            this.label = label;
            this.confidence = confidence;
            this.tx = tx;
            this.ty = ty;
            this.area = area;
        }
    }

    static final class Frame {
        final boolean valid;
        final double stalenessMs;
        final List<Detection> detections;
        Frame(boolean valid, double stalenessMs, List<Detection> detections) {
            this.valid = valid;
            this.stalenessMs = stalenessMs;
            this.detections = detections;
        }
    }

    private static final class Group {
        int count;
        double weightedTx, weight, maxArea, maxConfidence;
        double tx() { return weight > 0 ? weightedTx / weight : 0; }
        double score(Config c) { return c.scorePerBall * count + c.scorePerArea * maxArea; }
    }

    private final Camera camera;
    private final Config config;
    private final DoubleSupplier seconds;
    private boolean started;
    private double lastSeen = Double.NEGATIVE_INFINITY;
    private Target target = emptyTarget();

    public PollenVision(HardwareMap hardwareMap) { this(hardwareMap, new Config()); }

    public PollenVision(HardwareMap hardwareMap, Config config) {
        this(new Camera() {
            private final Limelight3A limelight = hardwareMap.get(Limelight3A.class, config.hardwareName);
            @Override public void start() {
                limelight.setPollRateHz(config.pollRateHz);
                limelight.pipelineSwitch(config.pipeline);
                limelight.start();
            }
            @Override public Frame read() {
                LLResult result = limelight.getLatestResult();
                if (result == null) return null;
                List<Detection> detections = new ArrayList<>();
                if (result.isValid() && result.getStaleness() < config.maxStalenessMs) {
                    for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
                        detections.add(new Detection(d.getClassName(), d.getConfidence(),
                                d.getTargetXDegrees(), d.getTargetYDegrees(), d.getTargetArea()));
                    }
                }
                return new Frame(result.isValid(), result.getStaleness(), detections);
            }
            @Override public void stop() { limelight.stop(); }
        }, config, () -> System.nanoTime() / 1e9);
    }

    PollenVision(Camera camera, Config config, DoubleSupplier seconds) {
        this.camera = camera;
        this.config = config;
        this.seconds = seconds;
    }

    public void start() {
        if (started) return;
        target = emptyTarget();
        lastSeen = Double.NEGATIVE_INFINITY;
        camera.start();
        started = true;
    }

    /** Call once per loop, or let PollenChaseController.update() call this. */
    public Target update() {
        if (!started) return target;
        double now = seconds.getAsDouble();
        Frame frame = camera.read();
        List<Detection> pollen = new ArrayList<>();
        if (frame != null && frame.valid && frame.stalenessMs < config.maxStalenessMs) {
            for (Detection d : frame.detections) {
                if ("pollen".equalsIgnoreCase(d.label) && d.confidence >= config.minConfidence
                        && Double.isFinite(d.confidence) && Double.isFinite(d.tx)
                        && Double.isFinite(d.ty) && Double.isFinite(d.area) && d.area >= 0) {
                    pollen.add(d);
                }
            }
        }
        List<Group> groups = group(pollen);
        Group best = null;
        for (Group g : groups) {
            if (best == null || g.score(config) > best.score(config)) best = g;
        }
        if (best != null) {
            if (now - lastSeen < config.lostTimeoutSeconds) {
                Group current = null;
                for (Group g : groups) {
                    double offset = Math.abs(g.tx() - target.txDegrees);
                    if (offset < config.matchAngleDegrees && (current == null
                            || offset < Math.abs(current.tx() - target.txDegrees))) current = g;
                }
                if (current != null && current != best
                        && best.score(config) < config.switchHysteresis * current.score(config)) {
                    best = current;
                }
            }
            lastSeen = now;
            target = new Target(true, pollen.size(), groups.size(), best.count,
                    best.tx(), best.maxArea, best.score(config), best.maxConfidence);
        } else {
            target = new Target(now - lastSeen < config.lostTimeoutSeconds,
                    pollen.size(), groups.size(), target.targetCount, target.txDegrees,
                    target.area, target.score, target.confidence);
        }
        return target;
    }

    private List<Group> group(List<Detection> detections) {
        int n = detections.size();
        int[] parent = new int[n];
        for (int i = 0; i < n; i++) parent[i] = i;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Detection a = detections.get(i), b = detections.get(j);
                double widthA = config.degreesPerSqrtArea * Math.sqrt(Math.max(a.area, 1e-6));
                double widthB = config.degreesPerSqrtArea * Math.sqrt(Math.max(b.area, 1e-6));
                if (Math.hypot(a.tx - b.tx, a.ty - b.ty)
                        < config.groupGap * 0.5 * (widthA + widthB)) {
                    int ra = find(parent, i), rb = find(parent, j);
                    if (ra != rb) parent[ra] = rb;
                }
            }
        }
        Map<Integer, Group> groups = new HashMap<>();
        for (int i = 0; i < n; i++) {
            int root = find(parent, i);
            Group g = groups.get(root);
            if (g == null) { g = new Group(); groups.put(root, g); }
            Detection d = detections.get(i);
            g.count++;
            g.weightedTx += d.tx * d.area;
            g.weight += d.area;
            g.maxArea = Math.max(g.maxArea, d.area);
            g.maxConfidence = Math.max(g.maxConfidence, d.confidence);
        }
        return new ArrayList<>(groups.values());
    }

    private static int find(int[] parent, int i) {
        while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
        return i;
    }

    public Target getTarget() { return target; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pollen seen", "%d in %d group(s)", target.pollenCount, target.groupCount);
        telemetry.addData("Pollen target available", target.available);
        if (target.available) {
            telemetry.addData("Target group", "%d ball(s), score %.1f", target.targetCount, target.score);
            telemetry.addData("tx (deg)", target.txDegrees);
            telemetry.addData("Nearest area (%)", target.area * 100);
            telemetry.addData("Confidence", target.confidence);
        }
    }

    public void stop() {
        try { if (started) camera.stop(); }
        finally {
            started = false;
            lastSeen = Double.NEGATIVE_INFINITY;
            target = emptyTarget();
        }
    }

    private static Target emptyTarget() { return new Target(false, 0, 0, 0, 0, 0, 0, 0); }
}
