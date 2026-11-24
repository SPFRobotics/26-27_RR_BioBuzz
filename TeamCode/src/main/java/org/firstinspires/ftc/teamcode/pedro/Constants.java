package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDrive");
        c.frontRightName.set("frontRightDrive");
        c.backLeftName.set("backLeftDrive");
        c.backRightName.set("backRightDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.022386784159292388);
        c.yPodOffset.set(-1.199623243076595);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3741352813141641);
                Controller secondaryTranslationalForward = Controller.proportional(0.1382329893667481);
                Controller primaryTranslationalLateral = Controller.proportional(0.5376667660048415);
                Controller secondaryTranslationalLateral = Controller.proportional(0.19865350331820564);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014087899603111098));
                c.brake.set(Controller.proportionalFeedforward(0.011974714662644433));

                c.headingFeedback.set(Controller.proportional(6.874257607663437));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05831582921263698, 0.006832108605268763));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12568665416192235, 0.09656226103225529));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011595942370793107, 0.0011736032521284746));

                c.maxAchievableForwardVelocity.set(74.26027978021483);
                c.maxAchievableStrafeVelocity.set(57.65990452855266);
                c.naturalForwardDeceleration.set(29.0176345969504);
                c.naturalStrafeDeceleration.set(65.78508739554643);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }


}
