package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Utility
public class TransferTest extends OpMode {

    Transfer transfer;


    @Override
    public void init() {
        transfer = new Transfer(hardwareMap);
    }

    @Override
    public void loop() {
        transfer.on();
    }
}
