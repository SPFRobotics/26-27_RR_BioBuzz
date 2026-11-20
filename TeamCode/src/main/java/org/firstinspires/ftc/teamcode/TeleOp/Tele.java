package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
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
    private static final double REVERSE_PULSE_SECONDS = 0.1;
    boolean fieldCentric = true;

    private Follower follower;
    private Intake intake;
    private LedLights leds;
    private Limelight limelight;
    private Lift lift;

    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private boolean intakeEnabled;
    private boolean intakeReversed;
    private boolean flywheelEnabled;
    private boolean transferEnabled;
    private boolean nectarGateClosed = true;
    private boolean pollenGateClosed = true;
    private boolean pushbackRampDown;
    private boolean liftTargetRaised;
    private boolean previousLiftButton;
    private boolean previousIntakeButton;
    private boolean previousReversePulseButton;
    private double reversePulseEndTime;
    private boolean previousFlywheelButton;
    private boolean previousTransferButton;
    private boolean previousNectarGateButton;
    private boolean previousPollenGateButton;
    private boolean previousPushbackRampButton;
    private boolean previousHeadingResetButton;
    private AutoCache.Alliance driveAlliance = AutoCache.Alliance.UNKNOWN;
    private double fieldCentricHeadingOffset;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        pollenWheel = new FlywheelPollen(hardwareMap);
        nectarWheel = new FlywheelNectar(hardwareMap);
        transfer = new Transfer(hardwareMap);
        pushbackRamp = new PushbackRamp(hardwareMap);
        lift = new Lift(hardwareMap);
        leds = new LedLights(hardwareMap);
        stopAll();
        updateLeds();

        follower = Constants.create(hardwareMap);
        AutoCache.Snapshot handoff = AutoCache.consumeSnapshot();
        driveAlliance = handoff.alliance;
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
        lift.update();
        addControlTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        intakeEnabled = false;
        intakeReversed = false;
        transferEnabled = false;
        nectarGateClosed = true;
        pollenGateClosed = true;
        pushbackRampDown = false;
        liftTargetRaised = false;
        previousLiftButton = gamepad2.cross;
        previousIntakeButton = false;
        previousReversePulseButton = false;
        reversePulseEndTime = 0.0;
        previousFlywheelButton = gamepad2.dpad_up;
        previousTransferButton = false;
        previousNectarGateButton = gamepad2.left_bumper;
        previousPollenGateButton = gamepad2.right_bumper;
        previousPushbackRampButton = false;
        previousHeadingResetButton = false;
        // Blue's existing field directions are correct; red drivers face the opposite way.
        fieldCentricHeadingOffset = driveAlliance == AutoCache.Alliance.RED ? Math.PI : 0.0;
        stopAll();
        flywheelEnabled = true;
        updateFlywheels(getDistanceToTarget());
        updateLeds();
    }

    @Override
    public void loop() {
        boolean headingResetButton = gamepad1.options;
        if (headingResetButton && !previousHeadingResetButton) {
            // Recalibrate with the robot's front pointing toward the driver station.
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
        boolean flywheelButton = gamepad2.dpad_up;
        boolean transferButton = gamepad2.y;
        boolean nectarGateButton = gamepad2.left_bumper;
        boolean pollenGateButton = gamepad2.right_bumper;
        boolean liftButton = gamepad2.cross;
        boolean pushbackRampButton = gamepad1.a;
        intake.setReverseSpeed(-1);
        double now = getRuntime();
        if (reversePulseButton && !previousReversePulseButton) {
            reversePulseEndTime = now + REVERSE_PULSE_SECONDS;
            intake.setReverseSpeed(-0.5);
        }
        boolean reverseRequested = gamepad1.right_bumper || now < reversePulseEndTime;

        // Preserve the normal settings throughout reverse, including its release loop.
        if (!reverseRequested && !intakeReversed) {
            if (intakeButton && !previousIntakeButton) {
                intakeEnabled = !intakeEnabled;
            }
            if (transferButton && !previousTransferButton) {
                transferEnabled = !transferEnabled;
            }
        }
        intakeReversed = reverseRequested;
        if (nectarGateButton && !previousNectarGateButton) {
            nectarGateClosed = !nectarGateClosed;
        }
        if (pollenGateButton && !previousPollenGateButton) {
            pollenGateClosed = !pollenGateClosed;
        }
        if (flywheelButton && !previousFlywheelButton) {
            flywheelEnabled = !flywheelEnabled;
        }
        if (pushbackRampButton && !previousPushbackRampButton) {
            pushbackRampDown = !pushbackRampDown;
        }
        if (liftButton && !previousLiftButton) {
            // Toggle the requested endpoint, even if the last move stalled or timed out.
            liftTargetRaised = !liftTargetRaised;
            if (liftTargetRaised) {
                lift.up();
            } else {
                lift.down();
            }
        }

        previousIntakeButton = intakeButton;
        previousReversePulseButton = reversePulseButton;
        previousFlywheelButton = flywheelButton;
        previousTransferButton = transferButton;
        previousNectarGateButton = nectarGateButton;
        previousPollenGateButton = pollenGateButton;
        previousPushbackRampButton = pushbackRampButton;
        previousLiftButton = liftButton;

        if (intakeReversed) {
            transfer.setPower(-1);
        } else if (transferEnabled) {
            transfer.on();
        } else {
            transfer.off();
        }
        if (nectarGateClosed) {
            transfer.nectarBlock();
        } else {
            transfer.nectarRelease();
        }
        if (pollenGateClosed) {
            transfer.pollenBlock();
        } else {
            transfer.pollenRelease();
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
        updateFlywheels(distance);
        lift.update();
        updateLeds();

        addControlTelemetry();
        telemetry.addData("Target distance (in)", distance);
        telemetry.addData("pollenWheel target RPM", pollenWheel.getTargetRPM());
        telemetry.addData("pollenWheel actual RPM", pollenWheel.getRPM());
        telemetry.addData("nectarWheel target RPM", nectarWheel.getTargetRPM());
        telemetry.addData("nectarWheel actual RPM", nectarWheel.getRPM());
        telemetry.update();
    }

    private void updateFlywheels(double distance) {
        if (flywheelEnabled && Double.isFinite(distance) && distance >= 0.0) {
            pollenWheel.runAtRPM(2700);
            nectarWheel.runAtRPM(3525);
        } else {
            pollenWheel.stop();
            nectarWheel.stop();
        }
        pollenWheel.update();
        nectarWheel.update();
    }

    private void updateLeds() {
        leds.setColors(nectarGateClosed ? LedLights.RED : LedLights.BLUE,
                pollenGateClosed ? LedLights.RED : LedLights.YELLOW);
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
        telemetry.addData("GP2 D-pad Up - Toggle flywheels", flywheelEnabled ? "On" : "Off");
        //telemetry.addData("GP2 Y / Triangle - Transfer", intakeReversed ? "Reverse" : (transferEnabled ? "On" : "Off"));
        telemetry.addData("GP2 LB / L1 - Nectar gate", nectarGateClosed ? "Closed" : "Open");
        telemetry.addData("GP2 RB / R1 - Pollen gate", pollenGateClosed ? "Closed" : "Open");
        telemetry.addData("GP2 Cross (X) / A - Lift target", liftTargetRaised ? "Up" : "Down");
        telemetry.addData("Lift status", lift.getStatus());
        telemetry.addData("Lift encoder (V)", lift.getLastEncoderVoltage());
        telemetry.addData("Lift power", lift.getCommandedPower());
        telemetry.addData("Pos: ", follower.pose());
        telemetry.addLine("GP1 Options: reset drive reference with robot facing your driver station");
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
        reversePulseEndTime = 0.0;
        flywheelEnabled = false;
        transferEnabled = false;
        nectarGateClosed = true;
        pollenGateClosed = true;
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
        if (lift != null) {
            lift.stop();
        }
        if (pushbackRamp != null) {
            pushbackRamp.up();
        }
        if (transfer != null) {
            transfer.off();
            transfer.closeGates();
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
