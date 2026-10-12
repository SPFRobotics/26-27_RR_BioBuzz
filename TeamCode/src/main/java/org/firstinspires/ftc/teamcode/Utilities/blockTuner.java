package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Config
@Utility
public class blockTuner extends OpMode {

    public static double pos = 0;
    public Transfer transfer;

    @Override
    public void init() {
        transfer = new Transfer(hardwareMap);
    }

    @Override
    public void loop() {

        transfer.blockPos(pos);

    }
}
