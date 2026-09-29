// Adapted from FTC Teams 5193 and 10653; see UPSTREAM_LICENSE and limelight/README.md.
package org.firstinspires.ftc.teamcode.Utilities.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.Collections;
import java.util.List;

/**
 * Reads a Neural Detector pipeline (Limelight slot 0) trained on BIOBUZZ pollen
 * and reports the closest (largest) detection. Companion to YellowPollenTest,
 * which uses the color pipeline in slot 1.
 */
@TeleOp(name = "Pollen Detector Test", group = "Vision")
public class PollenDetectorTest extends LinearOpMode {

    private static final String TAG = "PollenDetector";

    private static final int DETECTOR_PIPELINE = 0;

    // Label as written in labels.txt when the model was trained
    private static final String POLLEN_CLASS = "pollen";

    // Ignore detections the network is not confident about
    private static final double MIN_CONFIDENCE = 0.5; // v2 model @ exposure 2000: real balls score 0.6-0.85

    private static final long LOG_INTERVAL_MS = 200;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(DETECTOR_PIPELINE);
        try {
            limelight.start();

            telemetry.addLine("Limelight ready - press START");
            telemetry.update();
            waitForStart();
            RobotLog.ii(TAG, "started, pipeline %d", DETECTOR_PIPELINE);

            long lastLogMs = 0;

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                boolean valid = result != null && result.isValid() && result.getStaleness() < 200;

                List<LLResultTypes.DetectorResult> detections =
                        valid ? result.getDetectorResults() : Collections.<LLResultTypes.DetectorResult>emptyList();

                // Closest pollen = largest detection that is confidently pollen
                LLResultTypes.DetectorResult best = null;
                for (LLResultTypes.DetectorResult d : detections) {
                    if (!POLLEN_CLASS.equalsIgnoreCase(d.getClassName())) continue;
                    if (d.getConfidence() < MIN_CONFIDENCE) continue;
                    if (best == null || d.getTargetArea() > best.getTargetArea()) best = d;
                }

                telemetry.addData("Detections", detections.size());
                if (best != null) {
                    telemetry.addData("Closest tx (deg)", "%.1f", best.getTargetXDegrees());
                    telemetry.addData("Closest ty (deg)", "%.1f", best.getTargetYDegrees());
                    telemetry.addData("Closest area (raw)", "%.4f", best.getTargetArea());
                    telemetry.addData("Confidence", "%.2f", best.getConfidence());
                } else {
                    telemetry.addLine("No pollen in view");
                }
                if (valid) {
                    telemetry.addData("Latency (ms)", "%.0f",
                            result.getCaptureLatency() + result.getTargetingLatency());
                }
                telemetry.update();

                long now = System.currentTimeMillis();
                if (now - lastLogMs >= LOG_INTERVAL_MS) {
                    lastLogMs = now;
                    StringBuilder all = new StringBuilder();
                    for (LLResultTypes.DetectorResult d : detections) {
                        if (all.length() > 0) all.append(" ");
                        all.append(String.format("%s:%.2f@%.4f", d.getClassName(), d.getConfidence(), d.getTargetArea()));
                    }
                    if (best != null) {
                        RobotLog.ii(TAG, "n=%d tx=%.1f ty=%.1f area=%.4f conf=%.2f all=[%s]",
                                detections.size(), best.getTargetXDegrees(), best.getTargetYDegrees(),
                                best.getTargetArea(), best.getConfidence(), all);
                    } else {
                        RobotLog.ii(TAG, "n=%d none (valid=%b) all=[%s]", detections.size(), valid, all);
                    }
                }
            }

            RobotLog.ii(TAG, "stopped");
        } finally {
            limelight.stop();
        }
    }
}
