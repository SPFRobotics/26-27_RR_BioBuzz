package org.firstinspires.ftc.teamcode.Utilities;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;

@TeleOp
@Config
public class FlowerTest extends OpMode {

    public static double flowerTestPos=0;

    PushbackRamp ramp;
    @Override
    public void init() {

        ramp = new PushbackRamp(hardwareMap);

    }

    @Override
    public void loop() {

        ramp.setPos(flowerTestPos);





    }
}
