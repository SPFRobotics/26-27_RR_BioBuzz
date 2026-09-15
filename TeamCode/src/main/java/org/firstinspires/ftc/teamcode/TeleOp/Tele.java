package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.triggers.Trigger;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Tele extends OpMode {
    private Follower follower;
    private Intake intake;

    @Override
    public void init() {
        Scheduler.reset();
        Trigger.Companion.getDefaultEventLoop().clear();
        NextMotor.Companion.getMotorEventLoop().clear();
        intake = new Intake();
        intake.stop().schedule();
        Scheduler.execute();
        NextMotor.Companion.getMotorEventLoop().poll();
        follower = Constants.create(hardwareMap);
        follower.setPose(Pose.zero());
    }

    @Override
    public void start() {
        new Trigger(() -> gamepad1.right_bumper).toggleOnTrue(intake.intake());
    }

    @Override
    public void loop() {
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x,
                -gamepad1.right_stick_x);
        follower.update();
        Trigger.Companion.getDefaultEventLoop().poll();
        Scheduler.execute();
        NextMotor.Companion.getMotorEventLoop().poll();
    }

    @Override
    public void stop() {
        Trigger.Companion.getDefaultEventLoop().clear();
        Scheduler.reset();
        if (intake != null) {
            intake.stop().schedule();
            Scheduler.execute();
            NextMotor.Companion.getMotorEventLoop().poll();
        }
        NextMotor.Companion.getMotorEventLoop().clear();
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }
}
