package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;
import org.firstinspires.ftc.teamcode.Utilities.LedLights;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Tele extends OpMode {
    // Coordinates are in inches.
    private static final Pose TARGET_POSE = new Pose(0, 0);
    private static final double PRECISION_DRIVE_SCALE = 0.35;
    private static final double PRECISION_TRIGGER_THRESHOLD = 0.1;
    private static final double REVERSE_PULSE_SECONDS = 0.05;
    boolean fieldCentric = true;

    private Follower follower;
    private Intake intake;
    private LedLights leds;
    private Limelight limelight;

    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private boolean intakeEnabled;
    private boolean intakeReversed;
    private boolean intakeReverseHeld;
    private boolean flywheelEnabled;
    private boolean transferEnabled;
    private boolean blockerClosed = true;
    private boolean pushbackRampDown;
    private boolean previousIntakeButton;
    private boolean previousReversePulseButton;
    private double reversePulseEndTime;
    private boolean previousFlywheelButton;
    private boolean previousTransferButton;
    private boolean previousBlockerButton;
    private boolean previousPushbackRampButton;
    private boolean previousHeadingResetButton;
    private double fieldCentricHeadingOffset;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        pollenWheel = new FlywheelPollen(hardwareMap);
        nectarWheel = new FlywheelNectar(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pushbackRamp = new PushbackRamp(hardwareMap);
        leds = new LedLights(hardwareMap);
        stopAll();
        updateLeds();

        follower = Constants.create(hardwareMap);
        AutoCache.Snapshot handoff = AutoCache.consumeSnapshot();
        Pose startPose = handoff.pose;
        follower.setPose(startPose != null ? startPose : Pose.zero());
        limelight = new Limelight(hardwareMap, new Limelight.Config(), handoff.alliance);
        limelight.start();
    }

    @Override
    public void init_loop() {
        updateLeds();
        follower.update();
        limelight.update();
        addControlTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        intakeEnabled = false;
        intakeReversed = false;
        intakeReverseHeld = false;
        flywheelEnabled = false;
        transferEnabled = false;
        blockerClosed = true;
        pushbackRampDown = false;
        previousIntakeButton = false;
        previousReversePulseButton = false;
        reversePulseEndTime = 0.0;
        previousFlywheelButton = false;
        previousTransferButton = false;
        previousBlockerButton = false;
        previousPushbackRampButton = false;
        previousHeadingResetButton = false;
        fieldCentricHeadingOffset = 0.0;
        stopAll();
        updateLeds();
    }

    @Override
    public void loop() {
        boolean headingResetButton = gamepad1.options;
        if (headingResetButton && !previousHeadingResetButton) {
            fieldCentricHeadingOffset = Math.toRadians(180) - follower.pose().heading();
        }
        previousHeadingResetButton = headingResetButton;

        double driveScale = gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD
                ? PRECISION_DRIVE_SCALE : 1.0;
        double forward = -gamepad1.left_stick_y * driveScale;
        double strafe = -gamepad1.left_stick_x * driveScale;
        if (fieldCentric) {
            double heading = follower.pose().heading() + fieldCentricHeadingOffset;
            double robotForward = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = robotForward;
        }
        limelight.update();
        follower.manual(
                forward,
                strafe,
                limelight.driveTurn(gamepad1.dpad_up, -gamepad1.right_stick_x, driveScale));
        follower.update();

        boolean intakeButton = gamepad1.square;
        boolean reversePulseButton = gamepad1.circle;
        intakeReverseHeld = gamepad1.right_bumper;
        boolean flywheelButton = gamepad2.left_bumper;
        boolean transferButton = gamepad2.y;
        boolean blockerButton = gamepad2.b;
        boolean pushbackRampButton = gamepad1.a;
        intake.setReverseSpeed(-1);
        double now = getRuntime();
        if (reversePulseButton && !previousReversePulseButton) {
            reversePulseEndTime = now + REVERSE_PULSE_SECONDS;
            intake.setReverseSpeed(-0.5);
        }
        boolean reverseRequested = intakeReverseHeld || now < reversePulseEndTime;

        // Preserve the normal settings throughout reverse, including its release loop.
        if (!reverseRequested && !intakeReversed) {
            if (intakeButton && !previousIntakeButton) {
                intakeEnabled = !intakeEnabled;
            }
            if (transferButton && !previousTransferButton) {
                transferEnabled = !transferEnabled;
            }
            if (blockerButton && !previousBlockerButton) {
                blockerClosed = !blockerClosed;
            }
        }
        intakeReversed = reverseRequested;
        if (flywheelButton && !previousFlywheelButton) {
            flywheelEnabled = !flywheelEnabled;
        }
        if (pushbackRampButton && !previousPushbackRampButton) {
            pushbackRampDown = !pushbackRampDown;
        }

        previousIntakeButton = intakeButton;
        previousReversePulseButton = reversePulseButton;
        previousFlywheelButton = flywheelButton;
        previousTransferButton = transferButton;
        previousBlockerButton = blockerButton;
        previousPushbackRampButton = pushbackRampButton;

        if (intakeReversed) {
            transfer.setPower(-1);
        } else if (transferEnabled) {
            transfer.on();
        } else {
            transfer.off();
        }
        // Only held reverse lowers the blocker; a burst preserves its normal setting.
        if (intakeReverseHeld) {
            transfer.release();
        } else if (blockerClosed) {
            transfer.block();
        } else {
            transfer.release();
        }
        if (pushbackRampDown) {
            pushbackRamp.down();
        } else {
            pushbackRamp.up();
        }

        if (intakeReversed) {
            intake.outtake();
        } else if (intakeEnabled) {
            intake.intake();
        } else {
            intake.stop();
        }

        double distance = getDistanceToTarget();
        if (flywheelEnabled && Double.isFinite(distance) && distance >= 0.0) {
            pollenWheel.runAtRPM(2700);
            nectarWheel.runAtRPM(3525);
        } else {
            pollenWheel.stop();
            nectarWheel.stop();
        }
        pollenWheel.update();
        nectarWheel.update();
        updateLeds();

        addControlTelemetry();
        telemetry.addData("Target distance (in)", distance);
        telemetry.addData("pollenWheel target RPM", pollenWheel.getTargetRPM());
        telemetry.addData("pollenWheel actual RPM", pollenWheel.getRPM());
        telemetry.addData("nectarWheel target RPM", nectarWheel.getTargetRPM());
        telemetry.addData("nectarWheel actual RPM", nectarWheel.getRPM());
        telemetry.update();
    }

    private void updateLeds() {
        boolean blockerOpen = intakeReverseHeld || !blockerClosed;
        boolean flywheelsAtSpeed = flywheelEnabled
                && pollenWheel.isAtSpeed() && nectarWheel.isAtSpeed();
        leds.setColors(blockerOpen ? LedLights.GREEN : LedLights.RED,
                flywheelsAtSpeed ? LedLights.ORANGE : LedLights.BLUE);
    }

    private void addControlTelemetry() {
       // telemetry.addLine("Gamepad 1 - Driving and collection (RB/R1 hold; other buttons toggle)");
        //telemetry.addData("Drive mode", fieldCentric ? "Field-centric" : "Robot-centric");
       // telemetry.addData("GP1 Left stick", "Drive / strafe");
       // telemetry.addData("GP1 Right stick left/right", "Turn");
       // telemetry.addData("GP1 LT / L2 - Hold for precision driving",
             //   gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD ? "35% speed" : "Full speed");
        telemetry.addData("GP1 Y / Triangle - Intake", intakeEnabled ? "On" : "Off");
        telemetry.addLine("GP1 X / Square: reverse 300 ms; RB / R1: hold to reverse");
        telemetry.addData("Reverse intake + transfer", intakeReversed ? "Reversing" : "Inactive");
        telemetry.addData("Pushback ramp", pushbackRampDown ? "Down" : "Up");

       // telemetry.addLine("Gamepad 2 - Scoring (buttons toggle)");
        telemetry.addData("Flywheels: ", flywheelEnabled ? "On" : "Off");
        //telemetry.addData("GP2 Y / Triangle - Transfer", intakeReversed ? "Reverse" : (transferEnabled ? "On" : "Off"));
        telemetry.addData("Blocker: ", intakeReverseHeld ? "Down (reverse)" : (blockerClosed ? "Closed" : "Open"));
        telemetry.addData("Pos: ", follower.pose());
        telemetry.addLine("GP1 Options: reset field-centric heading to 90 degrees");
        telemetry.addData("GP1 D-pad Up - Hold to aim", gamepad1.dpad_up ? "Held" : "Manual turn");
        limelight.addTelemetry(telemetry);
    }

    @Override
    public void stop() {
        if (leds != null) {
            leds.disableLeds();
        }
        intakeEnabled = false;
        intakeReversed = false;
        intakeReverseHeld = false;
        reversePulseEndTime = 0.0;
        flywheelEnabled = false;
        transferEnabled = false;
        blockerClosed = true;
        pushbackRampDown = false;
        stopAll();

        try {
            if (follower != null) {
                follower.stop();
                follower.update();
            }
        } finally {
            if (limelight != null) {
                limelight.stop();
            }
        }
    }
    private double getDistanceToTarget() {
        Pose robotPose = follower.pose();
        return Math.hypot(TARGET_POSE.x() - robotPose.x(),
                TARGET_POSE.y() - robotPose.y());
    }

    private void stopAll() {
        if (pushbackRamp != null) {
            pushbackRamp.up();
        }
        if (transfer != null) {
            transfer.off();
            transfer.block();
        }
        if (intake != null) {
            intake.stop();
        }
        if (pollenWheel != null) {
            pollenWheel.stop();
            nectarWheel.stop();
        }
    }
}
