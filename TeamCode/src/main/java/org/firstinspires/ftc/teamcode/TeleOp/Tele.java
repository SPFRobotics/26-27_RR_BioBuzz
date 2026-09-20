package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Tele extends OpMode {
//coords in inches imperial better than metric RAHHHH
private static final Pose TARGET_POSE = new Pose(0, 0); //placeholder

    private Follower follower;
    private Intake intake;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private boolean intakeEnabled;
    private boolean flywheelEnabled;
    private boolean previousIntakeButton;
    private boolean previousFlywheelButton;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        pollenWheel = new FlywheelPollen(hardwareMap);
        nectarWheel = new FlywheelNectar(hardwareMap);
        stopAll();

        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero()); //placeholder
    }

    @Override
    public void start() {
        intakeEnabled = false;
        flywheelEnabled = false;
        previousIntakeButton = false;
        previousFlywheelButton = false;
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
        boolean flywheelButton = gamepad2.square;

        if (intakeButton && !previousIntakeButton) {
            intakeEnabled = !intakeEnabled;
        }
        if (flywheelButton && !previousFlywheelButton) {
            flywheelEnabled = !flywheelEnabled;
        }

        previousIntakeButton = intakeButton;
        previousFlywheelButton = flywheelButton;

        if (intakeEnabled) {
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

        telemetry.addData("Target distance (in)", distance);
        telemetry.addData("pollenWheel target RPM", pollenWheel.getTargetRPM());
        telemetry.addData("pollenWheel actual RPM", pollenWheel.getRPM());
        telemetry.addData("nectarWheel target RPM", nectarWheel.getTargetRPM());
        telemetry.addData("nectarWheel actual RPM", nectarWheel.getRPM());
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeEnabled = false;
        flywheelEnabled = false;
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
        if (intake != null) {
            intake.stop();
        }
        if (pollenWheel != null) {
            pollenWheel.stop();
            nectarWheel.stop();
        }
    }
}
