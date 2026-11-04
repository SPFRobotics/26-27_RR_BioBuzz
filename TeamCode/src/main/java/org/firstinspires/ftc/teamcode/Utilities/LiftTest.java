package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Utility(name = "Lift Test - Two Wraps")
public class LiftTest extends OpMode {
    private Lift lift;
    private AnalogInput encoder;
    private boolean previousUp;
    private boolean previousDown;
    private String lastCommand = "None";

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
        encoder = hardwareMap.get(AnalogInput.class, "liftEncoder");
        showTelemetry();
    }

    @Override
    public void init_loop() {
        lift.update();
        showTelemetry();
    }

    @Override
    public void start() {
        // Require a fresh press after START if a direction was already held.
        previousUp = gamepad1.dpad_up;
        previousDown = gamepad1.dpad_down;
    }

    @Override
    public void loop() {
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (gamepad1.b || (up && down)) {
            lift.stop();
            lastCommand = "Stopped";
        } else if (up && !previousUp) {
            lift.up();
            lastCommand = "Up: upper stop extended 90 degrees";
        } else if (down && !previousDown) {
            lift.down();
            lastCommand = "Down: original lower stop";
        }

        previousUp = up;
        previousDown = down;
        lift.update();
        showTelemetry();
    }

    private void showTelemetry() {
        telemetry.addLine("D-pad up/down: move lift | B: cancel");
        telemetry.addLine("Upper stop: +90 degrees | Lower stop unchanged");
        telemetry.addLine("Raw targets: up 0.081 V, down 0.906 V");
        telemetry.addLine("Hardware: lift (CR servo), liftEncoder (analog input)");
        telemetry.addData("Last command", lastCommand);
        telemetry.addData("Moving", lift.isBusy());
        telemetry.addData("Move result", lift.getStatus());
        telemetry.addData("Wraps since init", lift.getWrapCount());
        telemetry.addData("Target wraps", lift.getTargetWrapCount());
        telemetry.addData("Commanded power", "%.2f", lift.getCommandedPower());
        telemetry.addData("Measured shaft rotations", "%.3f", lift.getTraveledRotations());
        double rawVoltage = encoder.getVoltage();
        telemetry.addData("Raw encoder voltage", "%.3f V", rawVoltage);
        telemetry.addData("Zeroed encoder voltage", "%.3f V", Lift.zeroedVoltage(rawVoltage));
        telemetry.addData("Latest control reading", "%.4f V", lift.getLastEncoderVoltage());
        telemetry.addData("Last rejected voltage", "%.5f V", lift.getLastRejectedVoltage());
        telemetry.addData("Accepted voltage range", "-0.050 to 3.350 V");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (lift != null) lift.stop();
    }
}
