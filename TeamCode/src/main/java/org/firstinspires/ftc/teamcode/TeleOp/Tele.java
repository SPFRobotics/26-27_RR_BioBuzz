package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.triggers.Trigger;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Tele extends OpMode {
//coords in inches imperial better than metric RAHHHH
private static final Pose TARGET_POSE = new Pose(0, 0); //placeholder

    private Follower follower;
    private Intake intake;
    private Flywheel flywheel;

    @Override
    public void init() {
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
        NextMotor.Companion.getMotorEventLoop().clear();

        intake = new Intake();
        flywheel = new Flywheel();
        stopMechanisms();

        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero()); //placeholder
    }

    @Override
    public void start() {
        new Trigger(() -> gamepad1.x).toggleOnTrue(intake.intake());
        new Trigger(() -> gamepad2.square)
                .toggleOnTrue(flywheel.runAtDistance(this::getDistanceToTarget));
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);
        follower.update();
        Trigger.Companion.getDefaultEventLoop().poll();
        updateCommandsAndMotors();

        telemetry.addData("Target distance (in)", getDistanceToTarget());
        telemetry.addData("Flywheel target RPM", flywheel.getTargetRPM());
        telemetry.addData("Flywheel actual RPM", flywheel.getRPM());
        telemetry.update();
    }

    @Override
    public void stop() {
        Trigger.Companion.getDefaultEventLoop().clear();
        Scheduler.reset();

        stopMechanisms();
        NextMotor.Companion.getMotorEventLoop().clear();

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

    private void stopMechanisms() {
        if (intake != null) {
            intake.stop().schedule();
        }
        if (flywheel != null) {
            flywheel.stop().schedule();
        }
        updateCommandsAndMotors();
    }

    private void updateCommandsAndMotors() {
        Scheduler.execute();
        NextMotor.Companion.getMotorEventLoop().poll();
    }
}
