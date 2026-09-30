package org.firstinspires.ftc.teamcode.Auton;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Test Auton", group = "Auton")

public class TestAuton extends OpMode {
    private final Paths paths = new Paths();
    private Follower follower;
    private Path path;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(paths.start);
        path = paths.path1();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(paths.start);
        follower.follow(path);
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }

    public static class Paths {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(68.8378, 105.6337, 90);
        private final Pose path1 = poseFactory.of(68.8378, 104.7838, 110.601);
        private final Pose path1Control1 = poseFactory.of(93.9084, 1.1021, 0);
        private final Pose path1Control2 = poseFactory.of(56.955, 126.2342, 0);
        private final Pose path1Control3 = poseFactory.of(103.1044, 99.1269, 0);
        private final Pose path1Control4 = poseFactory.of(125.8581, 116.6652, 0);
        private final Pose path1Control5 = poseFactory.of(28.2575, 22.3484, 0);
        private final Pose path1Control6 = poseFactory.of(76.7845, 58.5045, 0);
        private final Pose path1Control7 = poseFactory.of(8.488, 20.9324, 0);
        private final Pose path1Control8 = poseFactory.of(71.0961, 135.0953, 0);
        private final Pose path1Control9 = poseFactory.of(56.289, 87.0113, 0);
        private final Pose path1Control10 = poseFactory.of(93.997, 63.5608, 0);
        private final Pose path1Control11 = poseFactory.of(133.6314, 58.5383, 0);
        private final Pose path1Control12 = poseFactory.of(64.5098, 1.6209, 0);
        private final Pose path1Control13 = poseFactory.of(90.2913, 54.9039, 0);

        public Path path1() {
            return curve(start, path1Control1, path1Control2, path1Control3, path1Control4, path1Control5, path1Control6, path1Control7, path1Control8, path1Control9, path1Control10, path1Control11, path1Control12, path1Control13, path1).tangent();
        }



    }
}
