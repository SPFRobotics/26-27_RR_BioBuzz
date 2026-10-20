package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Utility(name = "Omypoo")
public class OmSanityTele extends OpMode {
//coords in inches imperial better than metric RAHHHH
private static final Pose TARGET_POSE = new Pose(0, 0);

    private Follower follower;
    private Intake intake;

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
        stopAll();

        follower = Constants.create(hardwareMap);
        Pose startPose = AutoCache.consume();
        follower.setPose(startPose != null ? startPose : Pose.zero());
    }

    @Override
    public void init_loop() {
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
    }

    @Override
    public void loop() {

        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);
        follower.update();

        boolean intakeButton = gamepad1.x;
        boolean reverseIntakeButton = gamepad1.right_bumper;
        boolean flywheelButton = gamepad1.left_bumper;
        boolean transferButton = gamepad1.y;
        boolean blockerButton = gamepad1.b;
        boolean pushbackRampButton = gamepad1.a;

        if (intakeButton && !previousIntakeButton) {
            intakeEnabled = !intakeEnabled;
            intakeReversed = false;
        }
        if (reverseIntakeButton && !previousReverseIntakeButton) {
            intakeReversed = !intakeReversed;
            intakeEnabled = false;
        }
        if (flywheelButton && !previousFlywheelButton) {
            flywheelEnabled = !flywheelEnabled;
        }
        if (transferButton && !previousTransferButton) {
            transferEnabled = !transferEnabled;
        }
        if (blockerButton && !previousBlockerButton) {
            blockerClosed = !blockerClosed;
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

        if (transferEnabled) {
            transfer.on();
        } else {
            transfer.off();
        }
        if (blockerClosed) {
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
            pollenWheel.runAtDistance(distance);
            nectarWheel.runAtDistance(distance);
        } else {
            pollenWheel.stop();
            nectarWheel.stop();
        }
        pollenWheel.update();
        nectarWheel.update();

        addControlTelemetry();
        telemetry.addData("Target distance (in)", distance);
        telemetry.addData("pollenWheel target RPM", pollenWheel.getTargetRPM());
        telemetry.addData("pollenWheel actual RPM", pollenWheel.getRPM());
        telemetry.addData("nectarWheel target RPM", nectarWheel.getTargetRPM());
        telemetry.addData("nectarWheel actual RPM", nectarWheel.getRPM());
        telemetry.update();
    }

    private void addControlTelemetry() {
        telemetry.addLine("Controller 1 controls (buttons toggle)");
        telemetry.addData("Left stick", "Drive / strafe");
        telemetry.addData("Right stick left/right", "Turn");
        telemetry.addData("X / Square - Intake", intakeEnabled ? "On" : "Off");
        telemetry.addData("RB / R1 - Reverse intake", intakeReversed ? "On" : "Off");
        telemetry.addData("LB / L1 - Flywheels", flywheelEnabled ? "On" : "Off");
        telemetry.addData("Y / Triangle - Transfer", transferEnabled ? "On" : "Off");
        telemetry.addData("B / Circle - Blocker", blockerClosed ? "Closed" : "Open");
        telemetry.addData("A / Cross - Pushback ramp", pushbackRampDown ? "Down" : "Up");
    }

    @Override
    public void stop() {
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
