package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;

@Utility
@Config
public class lift extends OpMode {

    public static double speed = 0;

    CRServo lift;
    @Override
    public void init() {
        lift = hardwareMap.get(CRServo.class, "lift");
    }

    @Override
    public void loop() {
        lift.setPower(speed);

    }
}
