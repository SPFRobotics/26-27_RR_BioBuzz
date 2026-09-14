package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Pedro Auto", group = "Pedro")
public class PedroAuto extends OpMode {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose startPose = poses.of(24, 24, 0);
    private final Pose endPose = poses.of(48, 24, 0);

    private Follower follower;
    private Path path;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        path = line(startPose, endPose).linear(startPose, endPose);
    }

    @Override
    public void init_loop() {
        follower.update();
        reportPose();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        follower.follow(path);
    }

    @Override
    public void loop() {
        follower.update();
        reportPose();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.stop();
            follower.update();
        }
    }

    private void reportPose() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
