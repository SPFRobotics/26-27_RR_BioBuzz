package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class PushbackRamp {
    public static double downPosition = 0.0;

    public static double upPosition = 0.5;

    public static double floatPos = 0.4;

    Servo left;
    Servo right;

    public PushbackRamp(HardwareMap hardwaremap, String servo1, String servo2) {

        left = hardwaremap.get(Servo.class, servo1);
        right = hardwaremap.get(Servo.class, servo2);

        right.setDirection(Servo.Direction.REVERSE);
        left.setDirection(Servo.Direction.FORWARD);

    }

    public PushbackRamp(HardwareMap hardwareMap){
        this(hardwareMap, "pushbackLeft", "pushbackRight");
    }


    public void down() {
        left.setPosition(downPosition);
        right.setPosition(downPosition);
    }

    public void up() {
        left.setPosition(upPosition);
        right.setPosition(upPosition);
    }

    public void floater(){
        left.setPosition(floatPos);
        right.setPosition(floatPos);
    }




}
