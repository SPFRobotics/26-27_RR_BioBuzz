package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@Utility
public class IntakeTest extends OpMode {
    Intake intake;

    @Override
    public void init() {

        intake = new Intake(hardwareMap);

    }

    @Override
    public void loop() {

        intake.intake();

    }
}
