package org.firstinspires.ftc.teamcode.Utilities;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;

@Utility
public class pollenOuttakev2Test extends OpMode {
    FlywheelPollen pollen;
    public void init() {

        pollen = new FlywheelPollen(hardwareMap);

    }

    public void loop() {

        pollen.runAtRPM(4000);

        pollen.update();

    }
}
