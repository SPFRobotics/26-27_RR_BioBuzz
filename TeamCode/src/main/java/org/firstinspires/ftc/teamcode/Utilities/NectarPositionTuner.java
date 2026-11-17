package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Transfer.TransferConfig;

@Utility(name = "nectar pos tuner")
@Config
public class NectarPositionTuner extends OpMode {
    public static double position = TransferConfig.nectarDownPos;

    private Servo nectarGate;

    @Override
    public void init() {
        nectarGate = hardwareMap.get(Servo.class, "nectarGate");
    }

    @Override
    public void loop() {
        double appliedPosition = Range.clip(position, 0.0, 1.0);
        nectarGate.setPosition(appliedPosition);
        telemetry.addData("Nectar gate position", appliedPosition);
        telemetry.update();
    }
}
