package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;

@Utility
public class Test extends OpMode {

    DcMotor hi;


    @Override
    public void init() {
        hi = hardwareMap.get(DcMotor.class, "hi");
    }

    @Override
    public void loop() {

        hi.setPower(1);

    }
}
