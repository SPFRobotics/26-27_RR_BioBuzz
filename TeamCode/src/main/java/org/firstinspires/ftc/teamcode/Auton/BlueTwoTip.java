package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
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
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;
import org.firstinspires.ftc.teamcode.Utilities.LedLights;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "B- TwoTip", group = "BlueAuto")
public class BlueTwoTip extends LinearOpMode {

    public static double pollenRPM = 2800;
    public static double nectarRPM = 3300;
    public static double flywheelSpinupTimeoutMs = 2000;
    public static double preloadShootMs = 3000;
    public static double cornerShootMs = 3000;
    public static double flowerShootMs = 3000;
    public static double flowerIntakeMs = 2000;
    // Distance to either side of the flower intake pose, in robot-relative inches.
    public static double flowerStrafeInches = 1.0;
    public static double flowerStrafeToleranceInches = 0.15;
    public static double rampSettleMs = 650;
    public static double returnRampRaiseDelayMs = 500;

    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    //private LedLights leds;
    private String stage = "Ready";



    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(90.9964, 132.1164, 0);
    private final Pose intakeCorner = poseFactory.of(132.9365, 133.588, 0);
    private final Pose travelThroughHive = poseFactory.of(80.7555, 20.8926, 180);
    private final Pose travelThroughHiveControl1 = poseFactory.of(69.4789, 140.1208, 0);
    private final Pose travelThroughHiveControl2 = poseFactory.of(85.3224, 61.3505, 0);
    private final Pose travelThroughHiveSegment1Heading = poseFactory.of(80.7555, 20.8926, 0);
    private final Pose travelThroughHiveSegment3Start = poseFactory.of(80.7555, 20.8926, 89.5414);
    private final Pose travelThroughHiveSegment3End = poseFactory.of(80.7555, 20.8926, 180);
    private final Pose runToFlowerStart = poseFactory.of(80.7555, 20.8926, 180);
    private final Pose runToFlower = poseFactory.of(93.6955, 14.5892, 270);
    private final Pose runToFlowerControl1 = poseFactory.of(105.0053, 59.2547, 0);
    private final Pose returnFromFlowerStart = poseFactory.of(93.6955, 14.5892, 270);
    private final Pose returnFromFlower = poseFactory.of(81.0022, 21.5447, 180);
    private final Pose returnFromFlowerControl1 = poseFactory.of(104.4489, 60.1466, 0);
    private final Pose park = poseFactory.of(127.2438, 21.2716, 180);

    public Path intakeCorner() {
        return Paths.line(start, intakeCorner).constant(intakeCorner);
    }

    public Path travelThroughHive() {
        return Paths.curve(intakeCorner, travelThroughHiveControl1, travelThroughHiveControl2, travelThroughHive).heading(Interpolator.piecewise().until(0.1166, Interpolator.constant(travelThroughHiveSegment1Heading)).until(0.7804, Interpolator.tangent.reverse()).until(1, Interpolator.linear(travelThroughHiveSegment3Start, travelThroughHiveSegment3End)));
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

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> follower.hold(start)),
                shoot("Shoot 4 preload", preloadShootMs),
                step("intake Off", () -> {
                    intake.stop();
                    waitMs(1000);
                }),
                step("Intake corner", () -> {
                    transfer.block();
                    intake.intake();
                }),
                followToEnd(intakeCorner()),
                //followToEnd(strafe()),
                step("Travel through hive", () -> {
                    intake.stop();
                    transfer.block();
                    startFlywheels();
                }),
                followToEnd(travelThroughHive()),
                // Lower for flower intake while shooting the corner collection.
                parallel(
                        instant(() -> pushbackRamp.down()),
                        waitMs(rampSettleMs),
                        shoot("Shoot corner collection", cornerShootMs)
                ),
                step("Run to flower", () -> intake.intake()),
                followToEnd(runToFlower()),
                intakeAtFlower(),
                step("Return from flower", () -> { }),
                returnToShootingSpot(returnFromFlower()),
                instant(() -> intake.stop()),
                shoot("Shoot flower collection", flowerShootMs),
                step("Park", () -> { }),
                followToEnd(park()),
                step("Complete", this::stopMechanisms)
        );
    }

    @Override
    public void runOpMode() {
        AutoCache.clear();
        boolean autonomousStarted = false;
        Scheduler.reset();
        try {
            intake = new Intake(hardwareMap);
            transfer = new Transfer(hardwareMap);
            pushbackRamp = new PushbackRamp(hardwareMap);
            pollenWheel = new FlywheelPollen(hardwareMap);
            nectarWheel = new FlywheelNectar(hardwareMap);
           // leds = new LedLights(hardwareMap);
            stopMechanisms();

            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            waitForStart();
            if (isStopRequested()) {
                return;
            }
            autonomousStarted = true;
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                //leds.cycleColors(10);
                AutoCache.save(follower.pose());
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
                if (autonomousStarted) {
                    AutoCache.save(follower.pose());
                }
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

    private Command intakeAtFlower() {
        double dx = -Math.sin(runToFlower.heading()) * flowerStrafeInches;
        double dy = Math.cos(runToFlower.heading()) * flowerStrafeInches;
        Pose left = new Pose(runToFlower.x() + dx, runToFlower.y() + dy, runToFlower.heading());
        Pose right = new Pose(runToFlower.x() - dx, runToFlower.y() - dy, runToFlower.heading());

        return sequential(
                step("Intake flower - strafe", () -> intake.intake()),
                deadline(waitMs(flowerIntakeMs), Groups.loop(sequential(
                        strafeTo(left),
                        strafeTo(right)
                ))),
                instant(() -> follower.stop())
        );
    }

    private Command strafeTo(Pose target) {
        return Command.build()
                .setStart(() -> follower.hold(target))
                .setDone(() -> follower.pose().distance(target) <= flowerStrafeToleranceInches);
    }

    private Command returnToShootingSpot(Path path) {
        return parallel(
                followToEnd(path),
                sequential(
                        // Give the robot time to clear the flower before lifting the arm.
                        waitMs(returnRampRaiseDelayMs),
                        instant(() -> pushbackRamp.up())
                )
        );
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
            //pollenWheel.stop();
        }
        if (nectarWheel != null) {
            //nectarWheel.stop();
        }
    }

    private void stopMechanisms() {
        stopShooting();
        if (pushbackRamp != null) {
            pushbackRamp.up();
        }
    }

    /*
    public Path intakeCorner() {
        return Paths.line(start, intakeCorner).constant(intakeCorner);
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
    */
    }

