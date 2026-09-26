package org.firstinspires.ftc.teamcode.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arcade Drive")
public class Arcade extends LinearOpMode {

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

        LeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightArm.setDirection(Servo.Direction.REVERSE);

        Servo_Angle = 40;

        waitForStart();

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


            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Normalize powers if either exceeds 1.0
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));

            if (max > 1.0) {
                leftPower /= max;
                rightPower /= max;
            }

            LeftWheel.setPower(-leftPower);
            RightWheel.setPower(-rightPower);
        }
    }
}