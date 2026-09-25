package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp(name = "Tank Drive")
public class Tank extends LinearOpMode {

    private DcMotor LeftWheel;
    private DcMotor RightWheel;
    private Servo RightArm;
    private Servo LeftArm;

    @Override
    public void runOpMode() {
        int Servo_Angle;

        LeftWheel = hardwareMap.get(DcMotor.class, "Left Wheel");
        RightWheel = hardwareMap.get(DcMotor.class, "RightWheel");
        RightArm = hardwareMap.get(Servo.class, "RightArm");
        LeftArm = hardwareMap.get(Servo.class, "LeftArm");

        waitForStart();

        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightArm.setDirection(Servo.Direction.REVERSE);

        Servo_Angle = 40;

        while (opModeIsActive()) {
/*
            // ----- Servo Arm Control -----
            if (gamepad1.dpad_up) Servo_Angle++;
            if (gamepad1.dpad_down) Servo_Angle--;

            Servo_Angle = Math.max(40, Math.min(90, Servo_Angle));
            double servoPos = Servo_Angle / 360.0 + 0.2;

            LeftArm.setPosition(servoPos);
            RightArm.setPosition(servoPos);

 */


            double drive = gamepad1.left_stick_x;
            double turn  =  gamepad1.right_stick_y;

            double leftPower  = (drive + turn) * 1;
            double rightPower = (drive - turn) * 1;

            LeftWheel.setPower(-leftPower);
            RightWheel.setPower(-rightPower);
        }
    }
}
