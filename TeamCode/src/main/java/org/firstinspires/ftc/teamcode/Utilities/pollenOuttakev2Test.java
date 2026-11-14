package org.firstinspires.ftc.teamcode.Utilities;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;

@TeleOp(name = "OuttakeTest")
public class pollenOuttakev2Test extends OpMode {
    FlywheelPollen pollen;
    public void init() {


        pollen = new FlywheelPollen(hardwareMap);

    }

    public void loop() {

        pollen.runAtRPM(4000);

        telemetry.addData("Pollen RPM", pollen.getRPM());

        pollen.update();

        telemetry.update();

    }
}
