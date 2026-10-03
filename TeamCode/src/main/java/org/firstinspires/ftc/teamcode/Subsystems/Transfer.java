package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    @Config
    public static class TransferConfig{

        static double upPos = 0.5;
        static double downPos = 0;

    }
    DcMotor transfer;
    Servo blocker;

    public Transfer(HardwareMap hardwareMap, String motorName, String servoName) {

        transfer = hardwareMap.get(DcMotor.class, motorName);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blocker = hardwareMap.get(Servo.class, servoName);

    }

    public Transfer(HardwareMap hardwareMap)
    {

        this(hardwareMap, "transfer", "blocker");
    }

    public void on(){

        transfer.setPower(1);

    }

    public void off(){
        transfer.setPower(0);
    }

    public void setPower(double power){
        transfer.setPower(power);
    }

    public void block(){
        blocker.setPosition(TransferConfig.upPos);
        off();
    }

    public void release(){

        blocker.setPosition(TransferConfig.downPos);
        on();

    }

    }


