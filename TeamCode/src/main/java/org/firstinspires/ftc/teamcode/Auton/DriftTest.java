package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Utility(name = "DriftTest")
public class DriftTest extends LinearOpMode {

    // Match the current FlywheelPIDTuner defaults; adjustable in the dashboard.
    public static double pollenRPM = 2560;
    public static double nectarRPM = 2650;

    private Follower follower;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private PushbackRamp pushbackRamp;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(78.4572, 128.6327, 90);
    private final Pose path1 = poseFactory.of(115.2233, 128.611, 90);
    private final Pose point2 = poseFactory.of(114.731, 104.9494, 90);
    private final Pose point3 = poseFactory.of(78.5393, 105.3687, 90);
    private final Pose point4 = poseFactory.of(78.927, 128.3813, 90);

    // Repeat the four sides of the box until the op mode stops.
    public Command autoRoutine() {
        return Groups.loop(sequential(
                followToEnd(path1()),
                followToEnd(path2()),
                followToEnd(path3()),
                followToEnd(path4())
        ));
    }

    private Command followToEnd(Path path) {
        return follow(follower, path).setDone(() -> !follower.isBusy());
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        try {
            pushbackRamp = new PushbackRamp(hardwareMap);
            pushbackRamp.up();
            pollenWheel = new FlywheelPollen(hardwareMap);
            nectarWheel = new FlywheelNectar(hardwareMap);

            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                pollenWheel.runAtRPM(pollenRPM);
                nectarWheel.runAtRPM(nectarRPM);
                pollenWheel.update();
                nectarWheel.update();
                pushbackRamp.up();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());
                telemetry.addData("Pollen RPM", pollenWheel.getRPM());
                telemetry.addData("Nectar RPM", nectarWheel.getRPM());
                telemetry.addData("Pushback ramp", "Up");

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
                idle();
            }
        } finally {
            Scheduler.reset();
            if (pollenWheel != null) {
                pollenWheel.stop();
            }
            if (nectarWheel != null) {
                nectarWheel.stop();
            }
            if (pushbackRamp != null) {
                pushbackRamp.up();
            }
            if (follower != null) {
                follower.stop();
                follower.update();
            }
        }
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }
}
