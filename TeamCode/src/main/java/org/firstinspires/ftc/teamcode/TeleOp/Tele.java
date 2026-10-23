package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
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
    boolean fieldCentric = false;

    private Follower follower;
    private Intake intake;
    private LedLights leds;

    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private boolean intakeEnabled;
    private boolean intakeReversed;
    private boolean flywheelEnabled;
    private boolean transferEnabled;
    private boolean blockerClosed = true;
    private boolean pushbackRampDown;
    private boolean previousIntakeButton;
    private boolean previousReverseIntakeButton;
    private boolean previousFlywheelButton;
    private boolean previousTransferButton;
    private boolean previousBlockerButton;
    private boolean previousPushbackRampButton;

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
        Pose startPose = AutoCache.consume();
        follower.setPose(new Pose(127.2438, 21.2716, 180));
    }

    @Override
    public void init_loop() {
        updateLeds();
        follower.update();
        addControlTelemetry();
        telemetry.update();
    }

    @Override
    public void start() {
        intakeEnabled = false;
        intakeReversed = false;
        flywheelEnabled = false;
        transferEnabled = false;
        blockerClosed = true;
        pushbackRampDown = false;
        previousIntakeButton = false;
        previousReverseIntakeButton = false;
        previousFlywheelButton = false;
        previousTransferButton = false;
        previousBlockerButton = false;
        previousPushbackRampButton = false;
        stopAll();
        updateLeds();
    }

    @Override
    public void loop() {

        double driveScale = gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD
                ? PRECISION_DRIVE_SCALE : 1.0;
        double forward = -gamepad1.left_stick_y * driveScale;
        double strafe = -gamepad1.left_stick_x * driveScale;
        if (fieldCentric) {
            double heading = follower.pose().heading();
            double robotForward = forward * Math.cos(heading) + strafe * Math.sin(heading);
            strafe = -forward * Math.sin(heading) + strafe * Math.cos(heading);
            forward = robotForward;
        }
        follower.manual(
                forward,
                strafe,
                -gamepad1.right_stick_x * driveScale);
        follower.update();

        boolean intakeButton = gamepad1.x;
        boolean reverseIntakeButton = gamepad1.right_bumper;
        boolean flywheelButton = gamepad2.left_bumper;
        boolean transferButton = gamepad2.y;
        boolean blockerButton = gamepad2.b;
        boolean pushbackRampButton = gamepad1.a;

        // Preserve the normal settings throughout reverse, including its release loop.
        if (!reverseIntakeButton && !previousReverseIntakeButton) {
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
        intakeReversed = reverseIntakeButton;
        if (flywheelButton && !previousFlywheelButton) {
            flywheelEnabled = !flywheelEnabled;
        }
        if (pushbackRampButton && !previousPushbackRampButton) {
            pushbackRampDown = !pushbackRampDown;
        }

        previousIntakeButton = intakeButton;
        previousReverseIntakeButton = reverseIntakeButton;
        previousFlywheelButton = flywheelButton;
        previousTransferButton = transferButton;
        previousBlockerButton = blockerButton;
        previousPushbackRampButton = pushbackRampButton;

        if (intakeReversed) {
            transfer.setPower(-0.8);
        } else if (transferEnabled) {
            transfer.on();
        } else {
            transfer.off();
        }
        if (intakeReversed) {
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
            nectarWheel.runAtRPM(3300);
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
        boolean blockerOpen = intakeReversed || !blockerClosed;
        boolean flywheelsAtSpeed = flywheelEnabled
                && pollenWheel.isAtSpeed() && nectarWheel.isAtSpeed();
        leds.setColors(blockerOpen ? LedLights.GREEN : LedLights.RED,
                flywheelsAtSpeed ? LedLights.PINK : LedLights.BLUE);
    }

    private void addControlTelemetry() {
       // telemetry.addLine("Gamepad 1 - Driving and collection (RB/R1 hold; other buttons toggle)");
        //telemetry.addData("Drive mode", fieldCentric ? "Field-centric" : "Robot-centric");
       // telemetry.addData("GP1 Left stick", "Drive / strafe");
       // telemetry.addData("GP1 Right stick left/right", "Turn");
       // telemetry.addData("GP1 LT / L2 - Hold for precision driving",
             //   gamepad1.left_trigger > PRECISION_TRIGGER_THRESHOLD ? "35% speed" : "Full speed");
       // telemetry.addData("GP1 X / Square - Intake", intakeReversed ? "Reverse" : (intakeEnabled ? "On" : "Off"));
         telemetry.addData("reverse intake + transfer", intakeReversed ? "Held" : "Released");
        telemetry.addData("Pushback ramp", pushbackRampDown ? "Down" : "Up");

       // telemetry.addLine("Gamepad 2 - Scoring (buttons toggle)");
        telemetry.addData("Flywheels: ", flywheelEnabled ? "On" : "Off");
        //telemetry.addData("GP2 Y / Triangle - Transfer", intakeReversed ? "Reverse" : (transferEnabled ? "On" : "Off"));
        telemetry.addData("Blocker: ", intakeReversed ? "Down (reverse)" : (blockerClosed ? "Closed" : "Open"));
        telemetry.addData("Pos: ", follower.pose());
    }

    @Override
    public void stop() {
        if (leds != null) {
            leds.disableLeds();
        }
        intakeEnabled = false;
        intakeReversed = false;
        flywheelEnabled = false;
        transferEnabled = false;
        blockerClosed = true;
        pushbackRampDown = false;
        stopAll();

        if (follower != null) {
            follower.stop();
            follower.update();
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
