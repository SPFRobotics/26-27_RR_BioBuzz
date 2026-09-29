# BIOBUZZ pollen vision — Limelight 3A + Pedro 3

This replaces the previous Python SnapScript with the neural pollen detector from
[FTC Teams 5193 and 10653](https://github.com/SASApantheon5193/biobuzz-pollen-vision).
The model runs on the Limelight; Java selects a group and commands the existing
Pedro follower. Main TeleOp is not modified and no driving OpMode is registered.

Source revision: `842b880eebfd5b7bec93a88bdf996abbc3eed0d8`.
The targeting and chase algorithms are adapted from upstream `PollenChase.java`;
the two diagnostic OpModes are copied with package and guaranteed camera-cleanup
changes. The upstream MIT notice is in [UPSTREAM_LICENSE](../UPSTREAM_LICENSE).
The model's CC BY 4.0 dataset credits and author-reported results are preserved in
[MODEL.md](model/MODEL.md). Only the CPU model is included here, not the Coral model
mentioned in the upstream model document. The PNG training graph is copied unchanged.

## Camera setup

1. Connect the Limelight 3A to a computer over USB. Open
   [limelight.local:5801](http://limelight.local:5801) or its USB adapter's camera IP.
2. Select **pipeline 0**, type **Neural Detector**, runtime **CPU**.
3. Upload `model/limelight_neural_detector_8bit.tflite` and
   `model/limelight_neural_detector_labels.txt`.
4. Set confidence threshold **0.3** (Java filters at **0.5**), exposure **2000**
   in .01 ms units (**20 ms**), and sensor gain **15**. Save the pipeline.
5. Configure the robot's Limelight hardware name as **limelight**.
6. Run **Pollen Detector Test**, group **Vision**, to see the largest confident
   pollen detection, tx/ty, area, confidence, and latency. It does not initialize motors.

Upstream reports about 11 FPS on the 3A CPU. Polling at 100 Hz does not increase the
model's frame rate. Adjust exposure to venue lighting; bright but unwashed balls
worked better in upstream tests. These are starting settings, not measurements
from this robot. No Python pipeline or Python dependencies are needed.

## Reusable API and loop ownership

`PollenVision` owns camera start/update/stop, filters neural results, groups pollen,
and exposes an immutable `Target` snapshot and `addTelemetry()`.
`PollenChaseController` accepts that vision instance and an **existing** Pedro 3
`Follower`. It owns vision polling and the manual drive command while active.
The OpMode calls `follower.update()` exactly once afterward. Do not also call
`vision.update()`, `follower.manual()`, or `follower.follow()` in the same loop.

The controller accepts normalized, **unscaled Pedro robot-centric** manual inputs:
forward positive, strafe left positive, turn counterclockwise positive. Convert
sticks with `-left_stick_y`, `-left_stick_x`, `-right_stick_x`; the latter two
negate the upstream right/clockwise convention. Manual scale is applied inside
the controller. The upstream automatic clockwise turn command is also negated
inside the controller. Drivetrain names and directions stay in Pedro `Constants`.

Example fragments for an OpMode already owning `follower` (not a new OpMode):

```java
import org.firstinspires.ftc.teamcode.Utilities.vision.PollenVision;
import org.firstinspires.ftc.teamcode.Utilities.vision.PollenChaseController;

// Fields:
private PollenVision pollenVision;
private PollenChaseController pollenChase;

// In init(), after creating your existing follower:
PollenVision.Config visionConfig = new PollenVision.Config();
visionConfig.hardwareName = "limelight";
visionConfig.pipeline = 0;
pollenVision = new PollenVision(hardwareMap, visionConfig);
PollenChaseController.Config chaseConfig = new PollenChaseController.Config();
chaseConfig.stopArea = 0.14; // Calibrate for your camera and bumper.
pollenChase = new PollenChaseController(follower, pollenVision, chaseConfig);

// In start():
pollenChase.start();

// In loop(), replacing the existing manual command and follower update:
pollenChase.update(gamepad1.right_bumper,
        -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
follower.update();
pollenChase.addTelemetry(telemetry);
telemetry.update();

// In stop():
try {
    if (pollenChase != null) pollenChase.stop();
} finally {
    // Flush Pedro's stop mode even if camera shutdown throws.
    if (follower != null) follower.update();
}
```

For vision-only use, call `pollenVision.start()`, `pollenVision.update()` each loop,
read `getTarget()`, and call `stop()` on shutdown. No follower is required.
The controller can take over a follower previously following a path; after
controller shutdown, the caller must explicitly issue the next desired path.
It never automatically resumes an old path or resets localization.

## Preserved targeting behavior

Only class `pollen` (case-insensitive), confidence >=0.5, and valid results less
than 200 ms stale are accepted. Non-finite numeric detections and negative areas
are additionally rejected. Areas are **image fractions**, unlike `LLResult.getTa()`
which is a percentage.

Single-linkage clustering connects centers within 3.3 average estimated ball
widths; width is `48 * sqrt(area)` degrees. Aim is area-weighted horizontal center.
Group score is `count + 8 * largestArea`. Match the previous target within 10°;
a challenger needs at least 1.25 times its score to replace it. The biggest box
sets stop distance. This is image-space selection, not physical distance or a
field coordinate estimate.

The controller exposes `MANUAL`, `CHASE`, `ARRIVED`, and `NO_TARGET`:

- Hold right bumper to chase. Releasing it restores manual input immediately.
- Any manual axis above 0.05 in magnitude overrides chase while held. Chase resumes
  when sticks return within the deadband and the bumper is still held.
- Forward power is 0.35 at area <=0.02 and falls linearly to zero at area 0.14.
- Turn gain is 0.02 per degree, cap 0.30, with a 2° deadband. Above 15° error,
  forward power is multiplied by 0.3. Automatic strafe remains zero.
- A lost target retains the last command basis for less than 0.5 seconds, then
  commands zero. No target at startup also commands zero. An arrived target
  commands zero translation and rotation.
- Manual inputs are scaled by 0.5 before Pedro's wheel mixing; combined inputs can
  produce wheel powers above 0.5, just as with upstream mixing.

Set `PollenVision.Config` and `PollenChaseController.Config` before starting.
Defaults preserve upstream behavior; `stopArea` must be greater than `farArea`,
and timeout, gain limits, and grouping settings must remain sensible positive
values (turn gain may be negated for reversed camera orientation). Stop area is
camera-dependent: calibrate it before normal use. These commands use Pedro's
manual mode, not generated paths, and do not operate the intake or flywheels.

## Optional color diagnostic

**Yellow Pollen Test** (Vision group) reads pipeline 1 without drivetrain hardware.
Set pipeline type Color/Retroreflective, hue 21–33, saturation 170–255, value
165–255, exposure 600 (.01 ms units), gain 15, erosion/dilation 1/1, minimum area
0.1%, minimum fullness 60%, and leave W/H ratio wide open. Save the pipeline.

Its distance display uses the upstream calibration `2.39 / sqrt(area)` inches,
measured on another robot. Check against a tape measure before relying on it.
Color detection can confuse yellow/orange objects and merges touching balls.
The chase controller always uses the neural pipeline, not this color fallback.

## Verification and robot acceptance

Run the local tests and compile with:

```sh
./gradlew :TeamCode:testDebugUnitTest :TeamCode:compileDebugJavaWithJavac
```

Tests exercise class/confidence/freshness filtering, two-dimensional transitive
clustering, weighted aim, group scoring, hysteresis, target-loss expiration,
reacquisition, slowing/stopping, manual override, and Pedro command signs.
They do not establish camera accuracy or real drivetrain behavior.

Hardware acceptance, after a caller integrates the module:

1. Run the neural diagnostic first. Verify `pollen` boxes and confidence >=0.5.
2. Verify normal manual drive directions with the existing Pedro configuration.
3. Place one ball about 4 ft ahead, slightly off-center, in clear space. Hold the
   bumper and verify turning toward it, slowing, and stopping short of the bumper.
4. Check left/right targets and calibrate stop area. Check a pair versus a single
   and similarly scored separated groups for stable selection.
5. Release the bumper, override with each stick, cover the camera, and verify
   immediate manual takeover and a stop after target-loss grace expires.
6. Stop the OpMode and verify motors and camera stop. Hardware results remain
   unverified until these checks are performed on this robot.

Note: as upstream, the last accepted result can be reused until its 200 ms
freshness limit; the 0.5-second grace period starts from its last acceptance.
The total delay after a frozen camera frame may therefore approach 0.7 seconds.

### Copied asset verification

Verified Git blob SHA-1 values from the pinned upstream tree:

| Asset | Git blob SHA-1 |
|---|---|
| CPU model | `17eac987908ec2bf66ba16b938909a393db4c14a` |
| Labels | `19c1116045ef74179df8cf86162d278c9d4affed` |
| MODEL.md | `03f4424cb41eb2e199fc2565f561bbb28fd9dabf` |
| Training graph | `9e6c76ca552da0f4a641b280aa03d63588d9477d` |
| UPSTREAM_LICENSE | `6e8ffb0585f14a68ab572ddd97a486692676caf4` |

These are Git blob hashes (`git hash-object <file>`), not plain file SHA-1 hashes.
