package org.firstinspires.ftc.teamcode.Utilities;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;

@Config
@Utility
public class TestOne extends OpMode {

    Intake intake;
    Transfer transfer;
    FlywheelNectar outtakeN;
    FlywheelPollen outtakeP;

    public static boolean block;

    public static boolean activateTransfer = true;
    @Override
    public void init() {

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        outtakeN = new FlywheelNectar(hardwareMap);
        outtakeP = new FlywheelPollen(hardwareMap);

    }

    @Override
    public void loop() {

        block = true;

        intake.intake();
        if(activateTransfer){transfer.on();}

        if(block){transfer.block();}

        outtakeN.runAtRPM(2600);
        outtakeP.runAtRPM(2600);

        outtakeP.update();
        outtakeN.update();

    }
}
