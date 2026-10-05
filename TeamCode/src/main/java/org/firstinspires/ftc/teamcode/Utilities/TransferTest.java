package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Utility
@Config
public class TransferTest extends OpMode {

    //Transfer transfer;

    public static double pos = 0.45;

    Servo blocker;

    @Override
    public void init() {
        //transfer = new Transfer(hardwareMap);
        blocker = hardwareMap.get(Servo.class, "blocker");

    }

    @Override
    public void loop() {
        //transfer.on();
        blocker.setPosition(pos);
    }
}
