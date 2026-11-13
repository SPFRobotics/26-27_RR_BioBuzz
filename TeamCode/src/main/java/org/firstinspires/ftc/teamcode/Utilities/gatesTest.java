package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Subsystems.Transfer.TransferConfig.pollenPose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Utility
public class gatesTest extends OpMode {

    Transfer transfer;
    @Override
    public void init() {

        transfer = new Transfer(hardwareMap);

    }

    @Override
    public void loop() {

        transfer.pollenControl(pollenPose);


    }
}
