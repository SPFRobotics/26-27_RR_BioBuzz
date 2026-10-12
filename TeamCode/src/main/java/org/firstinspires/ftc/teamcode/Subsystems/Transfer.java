package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {

    @Config
    public static class TransferConfig{

        static double upPos = 0.17;
        static double downPos = 0.42;

    }
    DcMotor motor;
    Servo blocker;


    public Transfer(HardwareMap hardwareMap, String motorName, String servoName) {

        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blocker = hardwareMap.get(Servo.class, servoName);


    }

    public Transfer(HardwareMap hardwareMap)
    {

        this(hardwareMap, "transfer", "blocker");
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

    }


