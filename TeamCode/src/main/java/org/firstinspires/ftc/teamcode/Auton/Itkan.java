package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "Itkan", group = "Blueton", preselectTeleOp = "TeleOp")
public class Itkan extends LinearOpMode {
    public static double pollenRPM = 2700;
    public static double nectarRPM = -6000;
    public static double pickupMs = 500;
    public static double flowerPickupMs = 1000;
    public static double rampRaiseDelayMs = 500;
    public static double spinupTimeoutMs = 2000;
    public static double feedMs = 1500;

    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private PushbackRamp ramp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private String stage = "Ready";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(90.6068, 133.3487, 90);
    private final Pose intakeCorner = poseFactory.of(133.2899, 133.1771, 0);
    private final Pose shootCorner = poseFactory.of(104.7984, 31.605, -131);
    private final Pose shootCornerSegment1Heading = poseFactory.of(104.7984, 31.605, 0);
    private final Pose shootCornerSegment3Start = poseFactory.of(104.7984, 31.605, 74.3309);
    private final Pose shootCornerSegment3End = poseFactory.of(104.7984, 31.605, 229);
    private final Pose intakeFlowerStart = poseFactory.of(104.7984, 31.605, 229);
    private final Pose intakeFlower = poseFactory.of(94.4811, 11.3786, 270);
    private final Pose intakeFlowerControl1 = poseFactory.of(95.617, 27.2297, 0);
    private final Pose shootFlowerStart = poseFactory.of(94.4811, 11.3786, 270);
    private final Pose shootFlower = poseFactory.of(104.4123, 31.8851, 229);
    private final Pose shootFlowerControl1 = poseFactory.of(96.1285, 27.2992, 0);
    private final Pose intakeRobot = poseFactory.of(132.2743, 70.3163, -90);
    private final Pose intakeRobotControl1 = poseFactory.of(113.6857, 83.6364, 0);
    private final Pose intakeRobotSegment1Start = poseFactory.of(132.2743, 70.3163, 229);
    private final Pose intakeRobotSegment1End = poseFactory.of(132.2743, 70.3163, 270);
    private final Pose intakeRobotSegment2Heading = poseFactory.of(132.2743, 70.3163, 270);
    private final Pose shootRobot = poseFactory.of(103.4501, 110.3294, -34);
    private final Pose shootRobotSegment1Heading = poseFactory.of(103.4501, 110.3294, 270);
    private final Pose shootRobotSegment3Start = poseFactory.of(103.4501, 110.3294, -54.2321);
    private final Pose shootRobotSegment3End = poseFactory.of(103.4501, 110.3294, 326);
    private final Pose intakeFlower2 = poseFactory.of(127.6684, 94.6302, 360);
    private final Pose intakeFlower2Control1 = poseFactory.of(121.9746, 94.7202, 0);
    private final Pose intakeFlower2Segment1Start = poseFactory.of(127.6684, 94.6302, 326);
    private final Pose intakeFlower2Segment1End = poseFactory.of(127.6684, 94.6302, 0);
    private final Pose shootFlower2Start = poseFactory.of(127.6684, 94.6302, 0);
    private final Pose shootFlower2 = poseFactory.of(102.9999, 110.1397, 326);
    private final Pose park = poseFactory.of(132.8863, 37.7609, -90);
    private final Pose parkControl1 = poseFactory.of(132.2831, 86.7698, 0);
    private final Pose parkSegment1Start = poseFactory.of(132.8863, 37.7609, 0);
    private final Pose parkSegment1End = poseFactory.of(132.8863, 37.7609, 270);
    private final Pose parkSegment2Heading = poseFactory.of(132.8863, 37.7609, 270);

    public Command autoRoutine() {
        return sequential(
                collectAndScore("Corner", intakeCorner(), shootCorner(), false),
                collectAndScore("Audience flower", intakeFlower(), shootFlower(), true),
                collectAndScore("Robot pickup", intakeRobot(), shootRobot(), false),
                collectAndScore("Alliance flower", intakeFlower2(), shootFlower2(), true),
                instant(() -> {
                    stage = "Park";
                    stopMechanisms();
                }),
                followToEnd(park()),
                instant(() -> stage = "Complete")
        );
    }

    private Command followToEnd(Path path) {
        return follow(follower, path).setDone(() -> !follower.isBusy());
    }

    private Command collectAndScore(String location, Path pickup, Path scoring, boolean flower) {
        Command returnDrive = followToEnd(scoring);
        if (flower) {
            // Let the robot back away from the flower before lifting the ramp.
            returnDrive = parallel(returnDrive,
                    sequential(waitMs(rampRaiseDelayMs), instant(ramp::up)));
        }
        return sequential(
                instant(() -> {
                    stage = location + " - collect";
                    intake.intake();
                    if (flower) {
                        ramp.down();
                    }
                }),
                followToEnd(pickup),
                waitMs(flower ? flowerPickupMs : pickupMs),
                instant(() -> {
                    stage = location + " - return to score";
                    intake.stop();
                    pollenWheel.runAtRPM(pollenRPM);
                    nectarWheel.runAtRPM(nectarRPM);
                }),
                returnDrive,
                waitMs(spinupTimeoutMs).until(pollenWheel::isAtSpeed),
                instant(() -> {
                    stage = location + " - feed";
                    transfer.pollenRelease();
                    transfer.on();
                    intake.intake();
                }),
                waitMs(feedMs),
                instant(() -> {
                    transfer.pollenBlock();
                    transfer.off();
                    intake.stop();
                })
        );
    }

    @Override
    public void runOpMode() {
        AutoCache.clear();
        Scheduler.reset();
        boolean started = false;
        try {
            intake = new Intake(hardwareMap);
            transfer = new Transfer(hardwareMap);
            ramp = new PushbackRamp(hardwareMap);
            pollenWheel = new FlywheelPollen(hardwareMap);
            nectarWheel = new FlywheelNectar(hardwareMap);
            stopMechanisms();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();
            Command routine = autoRoutine();
            telemetry.addData("Stage", stage);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }
            started = true;
            schedule(routine);
            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                pollenWheel.update();
                nectarWheel.update();

                telemetry.addData("Stage", stage);
                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());
                telemetry.addData("Pollen RPM", pollenWheel.getRPM());
                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }
                telemetry.update();
                idle();
            }
        } finally {
            Scheduler.reset();
            stopMechanisms();
            if (follower != null) {
                follower.stop();
                follower.update();
                if (started) {
                    AutoCache.save(follower.pose(), AutoCache.Alliance.BLUE);
                }
            }
        }
    }

    private void stopMechanisms() {
        if (intake != null) {
            intake.stop();
        }
        if (transfer != null) {
            transfer.off();
            transfer.closeGates();
        }
        if (ramp != null) {
            ramp.up();
        }
        if (pollenWheel != null) {
            pollenWheel.stop();
        }
        if (nectarWheel != null) {
            nectarWheel.stop();
        }
    }

    public Path intakeCorner() {
        return Paths.line(start, intakeCorner).constant(intakeCorner);
    }

    public Path shootCorner() {
        return Paths.line(intakeCorner, shootCorner).heading(Interpolator.piecewise().until(0.0944, Interpolator.constant(shootCornerSegment1Heading)).until(0.5472, Interpolator.tangent.reverse()).until(1, Interpolator.linear(shootCornerSegment3Start, shootCornerSegment3End).reverse()));
    }

    public Path intakeFlower() {
        return Paths.curve(intakeFlowerStart, intakeFlowerControl1, intakeFlower).linear(intakeFlowerStart, intakeFlower);
    }

    public Path shootFlower() {
        return Paths.curve(shootFlowerStart, shootFlowerControl1, shootFlower).linear(shootFlowerStart, shootFlower);
    }

    public Path intakeRobot() {
        return Paths.curve(shootFlower, intakeRobotControl1, intakeRobot).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(intakeRobotSegment1Start, intakeRobotSegment1End)).until(1, Interpolator.constant(intakeRobotSegment2Heading)));
    }

    public Path shootRobot() {
        return Paths.line(intakeRobot, shootRobot).heading(Interpolator.piecewise().until(0.1168, Interpolator.constant(shootRobotSegment1Heading)).until(0.75, Interpolator.tangent.reverse()).until(1, Interpolator.linear(shootRobotSegment3Start, shootRobotSegment3End)));
    }

    public Path intakeFlower2() {
        return Paths.curve(shootRobot, intakeFlower2Control1, intakeFlower2).heading(Interpolator.piecewise().until(1, Interpolator.linear(intakeFlower2Segment1Start, intakeFlower2Segment1End)));
    }

    public Path shootFlower2() {
        return Paths.line(shootFlower2Start, shootFlower2).linear(shootFlower2Start, shootFlower2);
    }

    public Path park() {
        return Paths.curve(shootFlower2, parkControl1, park).heading(Interpolator.piecewise().until(0.5, Interpolator.linear(parkSegment1Start, parkSegment1End)).until(1, Interpolator.constant(parkSegment2Heading)));
    }
}
