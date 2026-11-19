package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private static final String NECTAR_GATE_NAME = "nectarGate";

    @Config
    public static class TransferConfig{

        public static double nectarUpPos = 0.18;
        public static double nectarDownPos = 0.05;

        public static double pollenUp = 0.15;
        public static double pollenDown = 0.07;

        public static double pollenPose;

    }
    private final DcMotor motor;
    private final Servo nectarGate;
    private final Servo pollenGate;

    public Transfer(HardwareMap hardwareMap, String motorName, String nectarGateName, String pollenGateName) {
        motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nectarGate = hardwareMap.get(Servo.class, nectarGateName);
        pollenGate = hardwareMap.get(Servo.class, pollenGateName);
    }

    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, "transfer", NECTAR_GATE_NAME, "pollenGate");
    }

    public static void closeNectarGateIfPresent(HardwareMap hardwareMap) {
        Servo nectarGate = hardwareMap.tryGet(Servo.class, NECTAR_GATE_NAME);
        if (nectarGate != null) {
            nectarGate.setPosition(TransferConfig.nectarDownPos);
        }
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

    public void closeGates() {
        nectarBlock();
        pollenBlock();
    }

    public void openGates() {
        nectarRelease();
        pollenRelease();
    }

    public void nectarBlock(){

        nectarGate.setPosition(TransferConfig.nectarDownPos);

    }

    public void nectarRelease(){

        nectarGate.setPosition(TransferConfig.nectarUpPos);

    }

    public void pollenBlock() {
        pollenGate.setPosition(TransferConfig.pollenDown);
    }

    public void pollenRelease() {
        pollenGate.setPosition(TransferConfig.pollenUp);
    }

    public void pollenControl(double pos) {
        pollenGate.setPosition(pos);
    }


    }


