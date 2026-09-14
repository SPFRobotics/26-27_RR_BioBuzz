package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.ivy.Command;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;


public class Intake implements Mechanism {
    public static  double intake = 1.0;
    public static  double outtake = -1.0;

    private final NextMotor motor;


    public Intake(String motorName) {
        motor = new NextMotor(motorName);
        motor.setDirection(NextMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(NextMotor.ZeroPowerBehavior.BRAKE);
    }

    public Intake() {
        this("intakeMotor");
    }


    public Command intake() {
        return runAt(intake);
    }

    public Command outtake() {
        return runAt(outtake);
    }

    public Command stop() {
        return instant(() -> motor.setThrottle(0.0));
    }

    @Override
    public Command getDefaultCommand() {
        return runAt(0.0);
    }

    public Command runAt(double power) {
        return infinite(() -> motor.setThrottle(power))
                .setStart(() -> motor.setThrottle(power))
                .setEnd(condition -> motor.setThrottle(0.0));
    }
}
