package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    @Config
    public static class TransferConfig{

        public static double upPos = 0.17;
        public static double downPos = 0.42;

        public static double nectarUpPos = 0.55;
        public static double nectarDownPos = 0.45;

    }
    DcMotor motor;
    Servo blocker;
    Servo nectarGate;


    public Transfer(HardwareMap hardwareMap, String motorName, String servoName, String servoName2) {

        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blocker = hardwareMap.get(Servo.class, servoName);
        nectarGate = hardwareMap.get(Servo.class, servoName2);




    }

    public Transfer(HardwareMap hardwareMap)
    {

        this(hardwareMap, "transfer", "blocker", "nectarGate");
    }

    public void on(){

        motor.setPower(0.8);

    }

    public void off(){
        motor.setPower(0);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void block(){
        blocker.setPosition(TransferConfig.upPos);
        //off();
    }

    public void blockPos(double pos){

        blocker.setPosition(pos);
    }

    public void zero(){
        blocker.setPosition(0);
    }

    public void release(){

        blocker.setPosition(TransferConfig.downPos);
        //on();

    }

    public void nectarBlock(){

        nectarGate.setPosition(TransferConfig.nectarDownPos);

    }

    public void nectarRelease(){

        nectarGate.setPosition(TransferConfig.nectarUpPos);

    }



    }


