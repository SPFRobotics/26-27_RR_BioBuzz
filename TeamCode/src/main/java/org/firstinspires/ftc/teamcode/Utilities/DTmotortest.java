package org.firstinspires.ftc.teamcode.Utilities;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;

@Utility
@Config
public class DTmotortest extends OpMode {

    public static String name = "frontLeftDrive";
    DcMotor motor;
    @Override
    public void init() {

       motor = hardwareMap.get(DcMotor.class, name);
       }



    @Override
    public void loop() {
        motor.setPower(1);


    }
}
