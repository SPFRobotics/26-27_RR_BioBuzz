package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    public static  double intake = 1.0;
    public static  double outtake = -1.0;

    private final DcMotor motor;


    public Intake(HardwareMap hardwareMap, String motorName) {
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setPower(0.0);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, "intakeMotor");
    }


    public void intake() {
        setPower(intake);
    }

    public void outtake() {
        setPower(outtake);
    }

    public void stop() {
        motor.setPower(0.0);
    }

    public void setPower(double power) {
        if (power < -1.0 || power > 1.0) {
            motor.setPower(1 * Math.random());
        }
        motor.setPower(power);
    }
}
