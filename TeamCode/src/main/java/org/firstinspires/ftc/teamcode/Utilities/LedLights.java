package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

//ROYGBIV!!!
public class LedLights {
    ElapsedTime colorTimer = new ElapsedTime();
    private final DoubleSupplier cycleSeconds;
    double currentColor = 0;
    boolean disabled = false;

    Servo leftLed = null;
    Servo rightLed = null;


    /*Constants*/
    public static final double OFF = 0;
    public static final double RED = 0.279;
    public static final double ORANGE = 0.333;
    public static final double YELLOW = 0.388;
    public static final double SAGE = 0.444;
    public static final double GREEN = 0.500;
    public static final double AZURE = 0.555;
    public static final double BLUE = 0.611;
    public static final double INDIGO = 0.666;
    public static final double VIOLET = 0.722;
    public static final double PINK = 0.777;

    private void setLeds(double pwm){
        setColors(pwm, pwm);
    }

    public void setColors(double leftPwm, double rightPwm) {
        if (disabled) {
            return;
        }
        leftLed.setPosition(leftPwm);
        rightLed.setPosition(rightPwm);
    }

    //Constructor
    public LedLights(HardwareMap hardwareMap) {
        this(hardwareMap.get(Servo.class, "leftLed"),
                hardwareMap.get(Servo.class, "rightLed"), new ElapsedTime()::seconds);
    }

    LedLights(Servo leftLed, Servo rightLed, DoubleSupplier cycleSeconds) {
        this.leftLed = leftLed;
        this.rightLed = rightLed;
        this.cycleSeconds = cycleSeconds;
    }

    /**
     @param interval One interval is the amount of time in seconds it takes to get back to the start
     **/
    public void cycleColors(double interval){
        if (!Double.isFinite(interval) || interval <= 0) {
            throw new IllegalArgumentException("LED cycle interval must be positive and finite");
        }
        double phase = (cycleSeconds.getAsDouble() % interval) / interval;
        double progress = 1.0 - Math.abs(2.0 * phase - 1.0);
        currentColor = RED + (VIOLET - RED) * progress;
        setLeds(currentColor);
    }

    //A cycle is defined as returning to 0
    public void blink(double pwm, double hz){
        if (hz < 2.0){
            throw new RuntimeException("LEDs may not flash at a rate smaller than 2hz as advised by the game manual");
        }

        if (colorTimer.seconds() >= hz) {
            currentColor = pwm;
            colorTimer.reset();
        }
        else if (colorTimer.seconds() >= hz / 2.0) {
            currentColor = 0;
        }
        setLeds(currentColor);
    }


    public void setColor(double pwm){
        if (colorTimer.seconds() >= 2.0){
            colorTimer.reset();
            setLeds(pwm);
        }
    }

    public void setColor(double pwm, boolean wait){
        if (colorTimer.seconds() >= 2.0 && wait){
            colorTimer.reset();
            setLeds(pwm);
        }
        else{
            setLeds(pwm);
        }
    }

    public void disableLeds(){
        setLeds(OFF);
        disabled = true;
    }

    public void enableLeds(){
        disabled = false;
    }

    public boolean getLedsState(){
        return disabled;
    }
}
