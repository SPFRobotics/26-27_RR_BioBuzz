package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
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
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "B- TwoTip", group = "BlueAuto")
public class BlueTwoTip extends LinearOpMode {

    // Starting RPMs match FlywheelPIDTuner; tune RPMs and feed times on the robot.
    public static double pollenRPM = 2800;
    public static double nectarRPM = 3300;
    public static double flywheelSpinupTimeoutMs = 2000;
    public static double preloadShootMs = 2000;
    public static double cornerShootMs = 2000;
    public static double flowerShootMs = 2000;
    public static double flowerIntakeMs = 500;
    public static double rampSettleMs = 650;

    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private String stage = "Ready";

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85.2792, 133.401, 90);
    private final Pose intakeCorner = poseFactory.of(134.3658, 133.408, 0.0081);
    private final Pose travelThroughHive = poseFactory.of(81.0731, 21.2102, 180);
    private final Pose travelThroughHiveControl1 = poseFactory.of(72.0752, 131.433, 0);
    private final Pose travelThroughHiveSegment1Heading = poseFactory.of(81.0731, 21.2102, 0);
    private final Pose travelThroughHiveSegment3Start = poseFactory.of(81.0731, 21.2102, 86.5814);
    private final Pose travelThroughHiveSegment3End = poseFactory.of(81.0731, 21.2102, 180);
    private final Pose runToFlowerStart = poseFactory.of(81.0731, 21.2102, 180);
    private final Pose runToFlower = poseFactory.of(93.6955, 14.5892, 270);
    private final Pose runToFlowerControl1 = poseFactory.of(105.0053, 59.2547, 0);
    private final Pose returnFromFlowerStart = poseFactory.of(93.6955, 14.5892, 270);
    private final Pose returnFromFlower = poseFactory.of(81.0022, 21.5447, 180);
    private final Pose returnFromFlowerControl1 = poseFactory.of(105.9419, 59.8149, 0);
    private final Pose park = poseFactory.of(133.8792, 21.7693, 180);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> follower.hold(start)),
                shoot("Shoot 4 preload", preloadShootMs),
                step("Intake corner", () -> {
                    transfer.block();
                    intake.intake();
                }),
                followToEnd(intakeCorner()),
                step("Travel through hive", () -> {
                    intake.stop();
                    transfer.block();
                    startFlywheels();
                }),
                followToEnd(travelThroughHive()),
                shoot("Shoot corner collection", cornerShootMs),
                step("Run to flower", () -> {
                    pushbackRamp.down();
                    intake.intake();
                }),
                waitMs(rampSettleMs),
                followToEnd(runToFlower()),
                waitMs(flowerIntakeMs),
                step("Return from flower", () -> { }),
                followToEnd(returnFromFlower()),
                step("Raise ramp", () -> {
                    pushbackRamp.up();
                    intake.stop();
                }),
                waitMs(rampSettleMs),
                shoot("Shoot flower collection", flowerShootMs),
                step("Park", () -> { }),
                followToEnd(park()),
                step("Complete", this::stopMechanisms)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        try {
            intake = new Intake(hardwareMap);
            transfer = new Transfer(hardwareMap);
            pushbackRamp = new PushbackRamp(hardwareMap);
            pollenWheel = new FlywheelPollen(hardwareMap);
            nectarWheel = new FlywheelNectar(hardwareMap);
            stopMechanisms();

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
                pollenWheel.update();
                nectarWheel.update();

                telemetry.addData("Stage", stage);
                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());
                telemetry.addData("Pollen RPM", pollenWheel.getRPM());
                telemetry.addData("Nectar RPM", nectarWheel.getRPM());

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
            }
        }
    }

    private Command step(String name, Runnable action) {
        return instant(() -> {
            stage = name;
            action.run();
        });
    }

    private Command followToEnd(Path path) {
        // Ivy's default follow finishes at the parametric end, before pose settling.
        return follow(follower, path).setDone(() -> !follower.isBusy());
    }

    private Command shoot(String name, double feedMs) {
        return sequential(
                step(name + " - spin up", this::startFlywheels),
                waitMs(flywheelSpinupTimeoutMs)
                        .until(() -> pollenWheel.isAtSpeed() && nectarWheel.isAtSpeed()),
                step(name + " - feed", () -> {
                    transfer.release();
                    transfer.on();
                    intake.intake();
                }),
                // No piece sensor is configured, so shot completion is timed.
                waitMs(feedMs),
                instant(this::stopShooting)
        );
    }

    private void startFlywheels() {
        pollenWheel.runAtRPM(pollenRPM);
        nectarWheel.runAtRPM(nectarRPM);
    }

    private void stopShooting() {
        if (intake != null) {
            intake.stop();
        }
        if (transfer != null) {
            transfer.off();
            transfer.block();
        }
        if (pollenWheel != null) {
            pollenWheel.stop();
        }
        if (nectarWheel != null) {
            nectarWheel.stop();
        }
    }

    private void stopMechanisms() {
        stopShooting();
        if (pushbackRamp != null) {
            pushbackRamp.up();
        }
    }

    public Path intakeCorner() {
        return Paths.line(start, intakeCorner).tangent();
    }

    public Path travelThroughHive() {
        return Paths.curve(intakeCorner, travelThroughHiveControl1, travelThroughHive).heading(Interpolator.piecewise().until(0.0553, Interpolator.constant(travelThroughHiveSegment1Heading)).until(0.7105, Interpolator.tangent.reverse()).until(1, Interpolator.linear(travelThroughHiveSegment3Start, travelThroughHiveSegment3End)));
    }

    public Path runToFlower() {
        return Paths.curve(runToFlowerStart, runToFlowerControl1, runToFlower).linear(runToFlowerStart, runToFlower);
    }

    public Path returnFromFlower() {
        return Paths.curve(returnFromFlowerStart, returnFromFlowerControl1, returnFromFlower).linear(returnFromFlowerStart, returnFromFlower);
    }

    public Path park() {
        return Paths.line(returnFromFlower, park).constant(park);
    }
}
