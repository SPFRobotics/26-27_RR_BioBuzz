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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelNectar;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelPollen;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.PushbackRamp;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;
import org.firstinspires.ftc.teamcode.Utilities.LedLights;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@Autonomous(name = "B- TwoTip", group = "Blueton", preselectTeleOp = "TeleOp")
public class BlueTwoTip extends LinearOpMode {
    private static final double LED_CYCLE_SECONDS = 10.0;

    public static double pollenRPM = 2700;
    public static double nectarRPM = -6000;
    public static double flywheelSpinupTimeoutMs = 2000;
    public static double preloadShootMs = 2000;
    public static double cornerShootMs = 2000;
    public static double flowerShootMs = 2000;
    public static double flowerIntakeMs = 2000;
    // Distance to either side of the flower intake pose, in robot-relative inches.
    public static double flowerStrafeInches = 1.5;
    public static double flowerStrafePower = 0.25;
    public static double flowerRampLowerDelayMs = 750;
    public static double rampSettleMs = 650;
    public static double returnRampRaiseDelayMs = 500;

    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private PushbackRamp pushbackRamp;
    private FlywheelPollen pollenWheel;
    private FlywheelNectar nectarWheel;
    private LedLights leds;
    private String stage = "Ready";



    private final Pose start;
    private final Pose intakeCorner;
    private final Pose travelThroughHive;
    private final Pose travelThroughHiveControl1;
    private final Pose travelThroughHiveControl2;
    private final Pose travelThroughHiveSegment1Heading;
    private final Pose travelThroughHiveSegment3Start;
    private final Pose travelThroughHiveSegment3End;
    private final Pose runToFlower;
    private final Pose runToFlowerControl1;
    private final Pose runToFlowerSegment1Start;
    private final Pose runToFlowerSegment1End;
    private final Pose runToFlowerSegment2Heading;
    private final Pose returnFromFlower;
    private final Pose returnFromFlowerControl1;
    private final Pose returnFromFlowerSegment1Heading;
    private final Pose returnFromFlowerSegment2Start;
    private final Pose returnFromFlowerSegment2End;
    private final Pose park;
    private final Pose parkControl1;

    public BlueTwoTip() {
        this(PoseFactory.degrees());
    }

    protected AutoCache.Alliance alliance() {
        return AutoCache.Alliance.BLUE;
    }

    protected BlueTwoTip(PoseFactory poseFactory) {
        start = poseFactory.of(89.8739, 132.3744, 0);
        intakeCorner = poseFactory.of(132, 132.5472, 0);
        travelThroughHive = poseFactory.of(82, 11.4285, 180);
        travelThroughHiveControl1 = poseFactory.of(69.4789, 140.1208, 0);
        travelThroughHiveControl2 = poseFactory.of(85.3224, 61.3505, 0);
        travelThroughHiveSegment1Heading = poseFactory.of(82, 11.4285, 0);
        travelThroughHiveSegment3Start = poseFactory.of(82, 11.4285, 89.7949);
        travelThroughHiveSegment3End = poseFactory.of(82, 11.4285, 180);
        runToFlower = poseFactory.of(94.8616, 16, -90);
        runToFlowerControl1 = poseFactory.of(83.1872, 60.645, 0);
        runToFlowerSegment1Start = poseFactory.of(94.8616, 16, 180);
        runToFlowerSegment1End = poseFactory.of(94.8616, 16, 270);
        runToFlowerSegment2Heading = poseFactory.of(94.8616, 16, 270);
        returnFromFlower = poseFactory.of(80.5405, 11.3881, 180);
        returnFromFlowerControl1 = poseFactory.of(83.051, 61.3093, 0);
        returnFromFlowerSegment1Heading = poseFactory.of(80.5405, 11.3881, 270);
        returnFromFlowerSegment2Start = poseFactory.of(80.5405, 11.3881, 270);
        returnFromFlowerSegment2End = poseFactory.of(80.5405, 11.3881, 180);
        park = poseFactory.of(127.2438, 21.2716, 180);
        parkControl1 = poseFactory.of(96.1465, 38.3028, 0);
    }

    public Path intakeCorner() {
        return Paths.line(start, intakeCorner).constant(intakeCorner);
    }

    public Path travelThroughHive() {
        return Paths.curve(intakeCorner, travelThroughHiveControl1, travelThroughHiveControl2, travelThroughHive).heading(Interpolator.piecewise().until(0.1166, Interpolator.constant(travelThroughHiveSegment1Heading)).until(0.7804, Interpolator.tangent.reverse()).until(1, Interpolator.linear(travelThroughHiveSegment3Start, travelThroughHiveSegment3End)));
    }

    public Path runToFlower() {
        return Paths.curve(travelThroughHive, runToFlowerControl1, runToFlower)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.linear(runToFlowerSegment1Start, runToFlowerSegment1End))
                        .until(1, Interpolator.constant(runToFlowerSegment2Heading)));
    }

    public Path returnFromFlower() {
        return Paths.curve(runToFlower, returnFromFlowerControl1, returnFromFlower)
                .heading(Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(returnFromFlowerSegment1Heading))
                        .until(1, Interpolator.linear(returnFromFlowerSegment2Start, returnFromFlowerSegment2End)));
    }

    public Path park() {
        return Paths.curve(returnFromFlower, parkControl1, park).constant(park);
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> follower.hold(start)),
                shoot("Shoot 4 preload", preloadShootMs),
                step("intake Off", () -> {
                    intake.stop();
                    waitMs(500);
                }),
                shoot("Shoot 4 preload", 1500),
                step("Intake corner", () -> {
                    transfer.block();
                    intake.intake();
                }),
                followToEnd(intakeCorner()),
                waitMs(500),
                //followToEnd(strafe()),
                step("Travel through hive", () -> {
                    //intake.stop();
                    transfer.block();
                    startFlywheels();
                }),
                followToEnd(travelThroughHive()),
                shoot("Shoot corner collection", cornerShootMs),
                step("intake Off", () -> {
                    intake.stop();
                    waitMs(500);
                }),
                shoot("Shoot 4 preload", 1500),
                step("Run to flower", () -> intake.intake()),
                parallel(
                        followToEnd(runToFlower()),
                        sequential(
                                waitMs(flowerRampLowerDelayMs),
                                instant(() -> pushbackRamp.down()),
                                waitMs(rampSettleMs)
                        )
                ),
                intakeAtFlower(),
                step("Return from flower", () -> { }),
                returnToShootingSpot(returnFromFlower()),
                instant(() -> intake.stop()),
                shoot("Shoot flower collection", flowerShootMs),
                step("intake Off", () -> {
                    intake.stop();
                    waitMs(500);
                }),
                shoot("Shoot 4 preload", 1500),
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
            leds = new LedLights(hardwareMap);
            leds.cycleColors(LED_CYCLE_SECONDS);
            stopMechanisms();

            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            while (opModeInInit() && !isStopRequested()) {
                leds.cycleColors(LED_CYCLE_SECONDS);
                idle();
            }
            waitForStart();
            if (isStopRequested()) {
                return;
            }
            autonomousStarted = true;
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                leds.cycleColors(LED_CYCLE_SECONDS);
                AutoCache.save(follower.pose(), alliance());
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
            if (leds != null) {
                leds.disableLeds();
            }
            Scheduler.reset();
            stopMechanisms();
            if (follower != null) {
                follower.stop();
                follower.update();
                if (autonomousStarted) {
                    AutoCache.save(follower.pose(), alliance());
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
        ElapsedTime timer = new ElapsedTime();
        Pose[] center = new Pose[1];
        int[] direction = {1};

        return Command.build()
                .setStart(() -> {
                    stage = "Intake flower - strafe";
                    intake.intake();
                    // Measure the offset from where we actually arrived at the flower.
                    center[0] = follower.pose();
                    direction[0] = 1;
                    timer.reset();
                    follower.manual(0, Range.clip(Math.abs(flowerStrafePower), 0, 1), 0);
                })
                .setExecute(() -> {
                    Pose pose = follower.pose();
                    double heading = center[0].heading();
                    double offset = -(pose.x() - center[0].x()) * Math.sin(heading)
                            + (pose.y() - center[0].y()) * Math.cos(heading);
                    double limit = Math.abs(flowerStrafeInches);
                    if (offset >= limit) {
                        direction[0] = -1;
                    } else if (offset <= -limit) {
                        direction[0] = 1;
                    }
                    double headingError = Math.atan2(Math.sin(heading - pose.heading()),
                            Math.cos(heading - pose.heading()));
                    double turn = Range.clip(headingError * 2.0, -0.2, 0.2);
                    double power = Range.clip(Math.abs(flowerStrafePower), 0, 1);
                    if (timer.milliseconds() >= flowerIntakeMs || limit == 0) {
                        follower.stop();
                    } else {
                        follower.manual(0, direction[0] * power, turn);
                    }
                    telemetry.addData("Flower strafe offset (in)", offset);
                    telemetry.addData("Flower strafe direction", direction[0]);
                    telemetry.addData("Flower intake elapsed (ms)", timer.milliseconds());
                })
                .setDone(() -> timer.milliseconds() >= flowerIntakeMs)
                .setEnd(condition -> follower.stop());
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
                        .until(() -> pollenWheel.isAtSpeed()),
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
            //intake.stop();
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

