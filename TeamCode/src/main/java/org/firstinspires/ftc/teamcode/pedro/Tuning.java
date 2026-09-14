package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OTOSTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.OctoQuadTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelIMUTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ThreeWheelTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure otosTuner() {
        return new OTOSTuner();
    }

    @Tuner
    public static Procedure octoQuadTuner() {
        return new OctoQuadTuner();
    }

    @Tuner
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    @Tuner
    public static Procedure threeWheelTuner() {
        return new ThreeWheelTuner();
    }

    @Tuner
    public static Procedure threeWheelIMUTuner() {
        return new ThreeWheelIMUTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
