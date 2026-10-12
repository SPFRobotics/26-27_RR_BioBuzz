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
        c.frontRightName.set("backLeftDrive");
        c.backLeftName.set("frontRightDrive");
        c.backRightName.set("backRightDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Pinpoint");
        c.xPodOffset.set(-0.9094858545017994);
        c.yPodOffset.set(-0.1257803308682179);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23902056174832254);
                Controller secondaryTranslationalForward = Controller.proportional(0.08831171081896885);
                Controller primaryTranslationalLateral = Controller.proportional(0.3182544961614024);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11758653241488187);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.015499529508841179));
                c.brake.set(Controller.proportionalFeedforward(0.013174600082515002));

                c.headingFeedback.set(Controller.proportional(3.758975182375197));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0533883770992607, 0.0030849254718464366));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09264987760041885, 0.06816914287114871));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.806925604627019E-4, 0.0011384708207048118));

                c.maxAchievableForwardVelocity.set(69.09768796761995);
                c.maxAchievableStrafeVelocity.set(55.411480755482614);
                c.naturalForwardDeceleration.set(34.206726084652814);
                c.naturalStrafeDeceleration.set(65.97476016467914);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }


}
