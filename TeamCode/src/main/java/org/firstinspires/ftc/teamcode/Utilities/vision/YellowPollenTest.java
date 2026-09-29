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
 * Reads the BIOBUZZ_YellowPollen color pipeline (Limelight slot 1) and reports
 * the closest yellow Pollen. The Limelight must be named "limelight" in the
 * robot configuration.
 */
@TeleOp(name = "Yellow Pollen Test", group = "Vision")
public class YellowPollenTest extends LinearOpMode {

    private static final String TAG = "YellowPollen";

    private static final int POLLEN_PIPELINE = 1;

    // Ignore blobs smaller than this to reject noise. ColorResult.getTargetArea()
    // is a FRACTION of the image (0-1), unlike LLResult.getTa() which is percent.
    // 0.001 = 0.1% of the image.
    private static final double MIN_AREA = 0.001;

    // Also write results to the robot log (readable over adb), at most this often
    private static final long LOG_INTERVAL_MS = 200;

    // Distance from the lens to a 2.8" pollen ball, in inches, from its image area
    // (fraction). Calibrated 2026-09-25 on Bro-bot with balls at 18" and 20":
    // area ~1.7% -> 18", ~1.5% -> 20". Inverse-square, so distance = K / sqrt(area).
    // A 3.6" ball reads as if it were 1.29x closer.
    private static final double DISTANCE_K_INCHES = 2.39;

    static double estimateDistanceInches(double areaFraction) {
        return areaFraction > 0 ? DISTANCE_K_INCHES / Math.sqrt(areaFraction) : Double.NaN;
    }

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(POLLEN_PIPELINE);
        try {
            limelight.start();

            telemetry.addLine("Limelight ready - press START");
            telemetry.update();
            waitForStart();
            RobotLog.ii(TAG, "started, pipeline %d", POLLEN_PIPELINE);

            long lastLogMs = 0;

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                boolean valid = result != null && result.isValid() && result.getStaleness() < 100;

                List<LLResultTypes.ColorResult> pollen =
                        valid ? result.getColorResults() : Collections.<LLResultTypes.ColorResult>emptyList();

                // Closest pollen = largest area. Pick it explicitly rather than
                // trusting the camera's sort order.
                LLResultTypes.ColorResult best = null;
                for (LLResultTypes.ColorResult p : pollen) {
                    if (p.getTargetArea() < MIN_AREA) continue;
                    if (best == null || p.getTargetArea() > best.getTargetArea()) best = p;
                }

                telemetry.addData("Pollen seen", pollen.size());
                if (best != null) {
                    // tx: degrees right(+)/left(-) of crosshair. ty: degrees up(+)/down(-).
                    telemetry.addData("Closest tx (deg)", "%.1f", best.getTargetXDegrees());
                    telemetry.addData("Closest ty (deg)", "%.1f", best.getTargetYDegrees());
                    telemetry.addData("Closest area (%)", "%.2f", best.getTargetArea() * 100);
                    telemetry.addData("Est. distance (in)", "%.0f", estimateDistanceInches(best.getTargetArea()));

                    // Example: simple proportional turn toward the pollen
                    // double turn = 0.02 * best.getTargetXDegrees();
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
                    StringBuilder areas = new StringBuilder();
                    for (LLResultTypes.ColorResult p : pollen) {
                        if (areas.length() > 0) areas.append(",");
                        areas.append(String.format("%.4f", p.getTargetArea()));
                    }
                    if (best != null) {
                        RobotLog.ii(TAG, "seen=%d tx=%.1f ty=%.1f area=%.2f%% dist=%.0fin rawAreas=[%s]",
                                pollen.size(), best.getTargetXDegrees(),
                                best.getTargetYDegrees(), best.getTargetArea() * 100,
                                estimateDistanceInches(best.getTargetArea()), areas);
                    } else {
                        RobotLog.ii(TAG, "seen=%d none ball-sized (valid=%b) rawAreas=[%s]",
                                pollen.size(), valid, areas);
                    }
                }
            }

            RobotLog.ii(TAG, "stopped");
        } finally {
            limelight.stop();
        }
    }
}
