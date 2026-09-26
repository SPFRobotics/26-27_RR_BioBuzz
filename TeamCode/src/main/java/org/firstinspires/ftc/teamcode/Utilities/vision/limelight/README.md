# BioBuzz pollen detection on Limelight

`biobuzz_pollen.py` is a standalone Python SnapScript for yellow BioBuzz pollen.
Image processing runs on the Limelight. The Control Hub receives the results.
The file uses the documented `runPipeline(image, llrobot)` interface and only
OpenCV and NumPy, which Limelight supplies. No pip install is needed on the camera.

The yellow HSV values are initial tuning values, not calibration from a real ball.
The script detects round yellow candidates, selects the largest accepted blob,
and draws a green circle around it. Blue outlines show other accepted candidates.
This is color/shape detection: a similar-looking yellow object can be a false match.
It does not identify nectar, estimate distance, or control any motors.

## Install the script

1. Connect the Limelight to your laptop and open its web interface using Limelight
   Hardware Manager or [limelight.local:5801](http://limelight.local:5801).
2. Choose an unused pipeline slot and set its type to Python SnapScript/custom Python.
   Name it `BioBuzz Pollen`. Record the slot number for your Java code.
3. Paste the complete contents of `biobuzz_pollen.py` into the Python script editor.
   This is Python source, not an exported pipeline configuration; do not import it
   into a JSON/pipeline-settings import field. Use the editor's save/apply control
   if shown by your Limelight OS version.
4. Begin with a supported low resolution, such as 320x240, and aim at a real ball.
   Confirm that the preview appears and the script error panel is clear.
5. Set `SHOW_MASK = True` to tune: yellow-ball pixels should be white, background
   pixels black. Adjust `HSV_MIN` and `HSV_MAX`, then set `SHOW_MASK = False`.
   Hue is the color (0-179); saturation is color strength (0-255); value is brightness
   (0-255). Test different distances and lighting before tightening these limits.
6. Reconnect the camera to the Control Hub. Configure its hardware name as
   `limelight`, or use your existing name in Java.

Use consistent exposure and white balance where supported. Too much exposure can
wash yellow into white; too little can make it fall below the brightness threshold.

## Output contract: version 1

Read `result.getPythonOutput()` in FTC Java. The array always has eight numbers:

| Index | Meaning | Units / convention |
| --- | --- | --- |
| 0 | Ball found | 1=yes, 0=no |
| 1 | Accepted candidate count | Number of separate accepted blobs in this frame |
| 2 | Selected center X | -1 at image left, 0 center, +1 at right |
| 3 | Selected center Y | -1 at image bottom, 0 center, +1 at top |
| 4 | Selected enclosing-circle diameter | Percent of full image width |
| 5 | Selected outer contour area | Percent of full image area |
| 6 | Selected circularity | 0-1 shape measurement; **not detection confidence** |
| 7 | Output schema version | Always 1 |

No target returns `[0, 0, 0, 0, 0, 0, 0, 1]` and an empty contour; old detections
are not held. Position is relative to the full image even if ROI is changed.
Values are image coordinates, not inches, degrees, or robot/field coordinates.
The count is not guaranteed to equal the true number of balls: touching balls can
merge into one blob, and partial/hidden balls may be rejected.

Returning the selected contour also enables Limelight's normal `getTx()`, `getTy()`,
and `getTa()` calculation. These use Limelight's contour/crosshair settings, whereas
the Python center comes from an enclosing circle. Do not assume those centers or
area measurements are identical.

## Reading from FTC Java

The following is an integration example, not a standalone OpMode. Add the imports
and field to your OpMode, initialization to `init()`, and reading to `loop()`.
Use a slot matching the one selected in the Limelight interface.

```java
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

// Fields:
private Limelight3A limelight;
private static final int POLLEN_PIPELINE = 0; // Set to your actual slot.

// In init():
limelight = hardwareMap.get(Limelight3A.class, "limelight");
limelight.pipelineSwitch(POLLEN_PIPELINE);
limelight.start();

// In loop(): reset this decision each loop so old targets cannot persist.
boolean ballFound = false;
LLResult result = limelight.getLatestResult();
if (result != null
        && result.getPipelineIndex() == POLLEN_PIPELINE
        && result.getStaleness() < 100 // Starting freshness limit; tune on hardware.
        && result.isValid()) {
    double[] data = result.getPythonOutput();
    if (data != null && data.length == 8 && data[7] == 1.0) {
        boolean finite = true;
        for (double value : data) {
            finite &= !Double.isNaN(value) && !Double.isInfinite(value);
        }
        ballFound = finite && data[0] == 1.0;
        if (ballFound) {
            telemetry.addData("Pollen count", (int) data[1]);
            telemetry.addData("Pollen X (-left/+right)", data[2]);
            telemetry.addData("Pollen Y (-down/+up)", data[3]);
            telemetry.addData("Diameter (% width)", data[4]);
            telemetry.addData("Area (% image)", data[5]);
            telemetry.addData("Circularity", data[6]);
        }
    }
}
telemetry.addData("Pollen found", ballFound);
telemetry.update();

// In stop():
if (limelight != null) {
    limelight.stop();
}
```

Start by displaying these values with the robot stationary. For steering, subtract
the calibrated intake aim point from X and use a limited proportional correction:
a small error produces a small correction. Verify direction on your drivetrain.
Camera offset/rotation can change how image directions map to robot movement.
Only one controller should command the drive motors at a time when using Pedro.
On invalid/stale results, stop camera-guided motion or enter a deliberate search
state with a timeout. Confirm collection with an intake sensor if available.

## Tuning and limits

- `ROI` chooses where to search. It starts as the full image, because pollen can
  be on flowers as well as on the floor. Crop only after checking the camera view.
- Area limits are percentages of the entire image; retune after resolution changes
  because small shapes and morphology still behave differently at different sizes.
- Circularity, aspect ratio, and circle fill reject strips and many angular objects.
  Shadows, holes at the outer edge, and occlusion may also make real balls fail.
- `REJECT_EDGE_BLOBS` defaults to true. Detection can disappear when a nearby ball
  leaves the view. Disappearance alone does not mean successful collection.
- Selection is largest apparent area, with center proximity as a tie-breaker.
  It is not persistent tracking, and can switch between similarly sized balls.
- Adjacent balls are not explicitly separated. Test clusters before relying on counts.
- Distance in inches requires camera/target calibration. Pollen resting on flowers
  and pollen on the floor cannot share an unqualified floor-plane distance formula.

## Offline checks

With Python, `opencv-python-headless`, and `numpy` installed on a development PC:

```text
python -m unittest discover -s TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Utilities/vision/limelight -p "test_*.py" -v
```

These generated-image tests cover no target, color rejection, holes, shape filtering,
selection, count, coordinate signs, ROI, resolution changes, and target disappearance.
They do not establish real-camera accuracy, Limelight frame rate, or robot behavior.

Validation on September 22, 2026: all nine tests passed on desktop Python 3.12 with
OpenCV 5.0.0 and NumPy 2.5.3. The script uses standard OpenCV operations available
in Limelight's documented OpenCV 4.10 environment, but has not been executed on
the camera. The Java example has not been compiled as an OpMode.

## References

- [BioBuzz game manual](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20V1.htm): yellow pollen.
- [Limelight SnapScript interface](https://docs.limelightvision.io/docs/docs-limelight/pipeline-python/snapscript-pipelines).
- [Limelight FTC API](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).
- [Limelight 3A setup](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a).
