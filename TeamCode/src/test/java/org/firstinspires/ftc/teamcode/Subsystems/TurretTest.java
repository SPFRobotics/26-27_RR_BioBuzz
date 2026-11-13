package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Utilities.AutoCache.Alliance;
import org.junit.Test;
import static org.junit.Assert.*;

public class TurretTest {
    private static final double TICKS_PER_DEGREE = 8192.0 / 360.0;
    private static final double LOOP_SECONDS = 0.02;

    private static class Fixture {
        final Turret.Params params;
        final Turret turret;
        int ticks;
        int resets;
        double power1 = Double.NaN;
        double power2 = Double.NaN;
        long nanos;

        Fixture(Turret.Params params, Alliance alliance) {
            this.params = params;
            turret = new Turret(params, alliance, () -> ticks, () -> { ticks = 0; resets++; },
                    p -> power1 = p, p -> power2 = p, () -> nanos);
        }

        // Fresh copies of the shipped offsets, so a test can retune without touching the statics.
        static Fixture nectar(Alliance alliance) {
            return new Fixture(new Turret.Params("Nectar",
                    Turret.NECTAR.forwardOffsetInches, Turret.NECTAR.leftOffsetInches), alliance);
        }

        static Fixture pollen(Alliance alliance) {
            return new Fixture(new Turret.Params("Pollen",
                    Turret.POLLEN.forwardOffsetInches, Turret.POLLEN.leftOffsetInches), alliance);
        }

        void turnTo(double degrees) {
            ticks = (int) Math.round(degrees * TICKS_PER_DEGREE);
        }

        void update(Pose pose) {
            nanos += (long) (LOOP_SECONDS * 1e9);
            turret.update(pose);
        }

        // Model the servos turning the turret counter-clockwise at 300 deg/s per unit power.
        void drive(Pose pose, int loops) {
            double degrees = ticks / TICKS_PER_DEGREE;
            for (int i = 0; i < loops; i++) {
                update(pose);
                degrees += power1 * 300.0 * LOOP_SECONDS;
                turnTo(degrees);
            }
        }
    }

    private static Pose pose(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    private static void assertTarget(Alliance alliance, Pose robot, double x, double y) {
        Fixture f = Fixture.nectar(alliance);
        f.update(robot);
        assertEquals(x, f.turret.getTargetX(), 1e-9);
        assertEquals(y, f.turret.getTargetY(), 1e-9);
    }

    @Test public void nectarSitsBackLeftAndPollenBackRightOfTheRobotCenter() {
        // Facing +x, back is -x and left is +y. From (89.25, 120) both turret centers land on
        // the blue hive's centerline (x = 84.75): nectar at y = 124.5, pollen at y = 115.5.
        // The far cell is aimed at y = 87.2, straight out the robot's right side.
        Pose robot = pose(89.25, 120, 0);
        Fixture nectar = Fixture.nectar(Alliance.BLUE);
        Fixture pollen = Fixture.pollen(Alliance.BLUE);
        nectar.turret.trackHive();
        pollen.turret.trackHive();
        nectar.update(robot);
        pollen.update(robot);
        assertEquals(-90, nectar.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(-90, pollen.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(37.3, nectar.turret.getDistanceToTarget(), 1e-9);
        assertEquals(28.3, pollen.turret.getDistanceToTarget(), 1e-9);
    }

    @Test public void turretOffsetsRotateWithTheRobot() {
        // Facing -x, back is +x and left is -y: nectar at (84.75, 124.5), pollen at (84.75, 133.5).
        // The far cell is now out the robot's left side.
        Pose robot = pose(80.25, 129, 180);
        Fixture nectar = Fixture.nectar(Alliance.BLUE);
        Fixture pollen = Fixture.pollen(Alliance.BLUE);
        nectar.turret.trackHive();
        pollen.turret.trackHive();
        nectar.update(robot);
        pollen.update(robot);
        assertEquals(90, nectar.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(90, pollen.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(37.3, nectar.turret.getDistanceToTarget(), 1e-9);
        assertEquals(46.3, pollen.turret.getDistanceToTarget(), 1e-9);

        // Facing +y, back is -y and left is -x: nectar at (95.5, 87.2), pollen at (104.5, 87.2),
        // level with the far cell, which is again out the robot's left side.
        robot = pose(100, 91.7, 90);
        nectar.update(robot);
        pollen.update(robot);
        assertEquals(90, nectar.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(90, pollen.turret.getTargetAngleDegrees(), 1e-9);
        assertEquals(10.75, nectar.turret.getDistanceToTarget(), 1e-9);
        assertEquals(19.75, pollen.turret.getDistanceToTarget(), 1e-9);
    }

    @Test public void aimsAtTheAllianceHiveCellOnTheRobotsSideOfTheField() {
        // The alliance picks the hive even from the other alliance's half of the field.
        assertTarget(Alliance.RED, pose(100, 30, 0), 59.25, 56.8);
        assertTarget(Alliance.RED, pose(40, 110, 0), 59.25, 87.2);
        assertTarget(Alliance.BLUE, pose(40, 30, 0), 84.75, 56.8);
        assertTarget(Alliance.BLUE, pose(100, 110, 0), 84.75, 87.2);
    }

    @Test public void launchingAlongTheAimAngleReachesTheTargetFromAnywhere() {
        for (Alliance alliance : new Alliance[]{Alliance.RED, Alliance.BLUE}) {
            for (double x : new double[]{9, 40, 72, 101, 135}) {
                for (double y : new double[]{9, 40, 101, 135}) {
                    for (double heading : new double[]{-170, -90, -30, 0, 45, 90, 135, 180, 400}) {
                        for (Fixture f : new Fixture[]{Fixture.nectar(alliance), Fixture.pollen(alliance)}) {
                            f.turret.trackHive();
                            f.update(pose(x, y, heading));
                            double h = Math.toRadians(heading);
                            double turretX = x + f.params.forwardOffsetInches * Math.cos(h)
                                    - f.params.leftOffsetInches * Math.sin(h);
                            double turretY = y + f.params.forwardOffsetInches * Math.sin(h)
                                    + f.params.leftOffsetInches * Math.cos(h);
                            double launch = h + Math.toRadians(f.turret.getTargetAngleDegrees());
                            double distance = f.turret.getDistanceToTarget();
                            assertEquals(f.turret.getTargetX(), turretX + distance * Math.cos(launch), 1e-6);
                            assertEquals(f.turret.getTargetY(), turretY + distance * Math.sin(launch), 1e-6);
                        }
                    }
                }
            }
        }
    }

    @Test public void theTwoTipAutoShootingPosesAimOutTheRobotsRightSide() {
        // The fixed launchers fire out the robot's right side, so each pose the TwoTip autos
        // shoot from already points that side at the hive cell the turret should pick.
        assertShootsRight(Alliance.BLUE, pose(89.8739, 132.3744, 0), 84.75, 87.2);
        assertShootsRight(Alliance.BLUE, pose(82, 11.4285, 180), 84.75, 56.8);
        assertShootsRight(Alliance.RED, pose(54.1261, 11.6256, 180), 59.25, 56.8);
        assertShootsRight(Alliance.RED, pose(62, 132.5715, 0), 59.25, 87.2);
    }

    private static void assertShootsRight(Alliance alliance, Pose robot, double x, double y) {
        for (Fixture f : new Fixture[]{Fixture.nectar(alliance), Fixture.pollen(alliance)}) {
            f.turret.trackHive();
            f.update(robot);
            assertEquals(x, f.turret.getTargetX(), 1e-9);
            assertEquals(y, f.turret.getTargetY(), 1e-9);
            assertEquals(-90, f.turret.getTargetAngleDegrees(), 3);
        }
    }

    @Test public void targetCellOnlyChangesOnceTheRobotIsClearlyPastThePivotLine() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.update(pose(100, 100, 0));
        assertEquals(87.2, f.turret.getTargetY(), 1e-9);
        f.update(pose(100, 70, 0));
        assertEquals(87.2, f.turret.getTargetY(), 1e-9);
        f.update(pose(100, 68.9, 0));
        assertEquals(56.8, f.turret.getTargetY(), 1e-9);
        f.update(pose(100, 74, 0));
        assertEquals(56.8, f.turret.getTargetY(), 1e-9);
        f.update(pose(100, 75.1, 0));
        assertEquals(87.2, f.turret.getTargetY(), 1e-9);
    }

    @Test public void withoutAnAllianceItAimsAtTheHiveOnTheRobotsHalf() {
        for (Alliance alliance : new Alliance[]{Alliance.UNKNOWN, null}) {
            Fixture f = Fixture.nectar(alliance);
            f.update(pose(30, 30, 0));
            assertEquals(59.25, f.turret.getTargetX(), 1e-9);
            f.update(pose(73, 30, 0));
            assertEquals(59.25, f.turret.getTargetX(), 1e-9);
            f.update(pose(110, 30, 0));
            assertEquals(84.75, f.turret.getTargetX(), 1e-9);
        }
    }

    @Test public void staysUnpoweredUntilToldToAim() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        assertEquals(0, f.power1, 0);
        assertEquals(0, f.power2, 0);
        for (int i = 0; i < 5; i++) {
            f.update(pose(100, 110, 0));
            assertEquals(0, f.power1, 0);
            assertEquals(0, f.power2, 0);
        }
        assertFalse(f.turret.isAimed());
        assertTrue(Double.isNaN(f.turret.getTargetAngleDegrees()));
        // The distance is still live, so the flywheels can be set up before the turret moves.
        assertEquals(Math.hypot(84.75 - 95.5, 87.2 - 114.5), f.turret.getDistanceToTarget(), 1e-9);
    }

    @Test public void turnsToTheHiveAndStopsOnceAimed() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        Pose robot = pose(89.25, 120, 0);
        f.turret.trackHive();
        assertFalse(f.turret.isAimed());
        f.update(robot);
        assertEquals(-90, f.turret.getErrorDegrees(), 1e-9);
        assertTrue(f.power1 < 0);
        assertEquals(f.power1, f.power2, 0);
        assertFalse(f.turret.isAimed());

        f.drive(robot, 100);
        assertTrue(f.turret.isAimed());
        assertEquals(-90, f.turret.getAngleDegrees(), f.params.toleranceDegrees);
        assertEquals(0, f.power1, 0);
        assertEquals(0, f.power2, 0);
    }

    @Test public void keepsTheHiveInItsSightsWhileTheRobotSpinsAndDrives() {
        Fixture f = Fixture.pollen(Alliance.RED);
        f.params.kV = 1.0 / 300.0;
        f.turret.trackHive();
        f.drive(pose(30, 110, 0), 100);
        assertTrue(f.turret.isAimed());
        // 45 deg/s of spin while strafing along the far wall.
        for (int i = 1; i <= 200; i++) {
            f.drive(pose(30 + 0.2 * i, 110, 0.9 * i), 1);
            assertTrue("error " + f.turret.getErrorDegrees() + " at loop " + i,
                    Math.abs(f.turret.getErrorDegrees()) < 5);
        }
    }

    @Test public void takesTheShorterWayAroundThroughTheBack() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.turnTo(170);
        f.turret.holdAngle(-170);
        f.update(pose(100, 110, 0));
        assertEquals(20, f.turret.getErrorDegrees(), 0.05);
        assertTrue(f.power1 > 0);

        f.drive(pose(100, 110, 0), 100);
        assertTrue(f.turret.isAimed());
        // It kept going counter-clockwise past 180 rather than unwinding through zero.
        assertEquals(190, f.ticks / TICKS_PER_DEGREE, f.params.toleranceDegrees);
        assertEquals(-170, f.turret.getAngleDegrees(), f.params.toleranceDegrees);
    }

    @Test public void anyNumberOfFullTurnsIsTheSameDirection() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.turnTo(170 + 3 * 360);
        f.turret.holdAngle(-170);
        f.update(pose(100, 110, 0));
        assertEquals(170, f.turret.getAngleDegrees(), 0.05);
        assertEquals(20, f.turret.getErrorDegrees(), 0.05);
        f.turnTo(170 - 5 * 360);
        f.update(pose(100, 110, 0));
        assertEquals(170, f.turret.getAngleDegrees(), 0.05);
        assertEquals(20, f.turret.getErrorDegrees(), 0.05);
    }

    @Test public void holdAngleIgnoresWhereTheRobotIs() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.turret.holdAngle(45);
        f.drive(pose(100, 110, 0), 100);
        assertTrue(f.turret.isAimed());
        f.update(pose(20, 20, 135));
        assertEquals(45, f.turret.getTargetAngleDegrees(), 0);
        assertTrue(f.turret.isAimed());
        assertEquals(0, f.power1, 0);

        // A held angle needs no odometry at all, so it still works if the pose is lost.
        f.turret.holdAngle(-90);
        f.drive(null, 100);
        assertTrue(f.turret.isAimed());
        assertEquals(-90, f.turret.getAngleDegrees(), f.params.toleranceDegrees);
    }

    @Test public void switchingFromAHeldAngleToTheHiveStartsAFreshAim() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        Pose robot = pose(89.25, 120, 0);
        f.turret.holdAngle(45);
        f.drive(robot, 100);
        assertTrue(f.turret.isAimed());

        // Being on the held angle says nothing about the hive, which is 135 degrees away.
        f.params.kP = 0;
        f.params.kD = 0;
        f.params.kS = 0;
        f.params.kV = 0.004;
        f.turret.trackHive();
        assertFalse(f.turret.isAimed());
        f.update(robot);
        assertEquals(-135, f.turret.getErrorDegrees(), f.params.toleranceDegrees);
        // Nor is the jump between the two targets motion for the feedforward to chase.
        assertEquals(0, f.power1, 0);
    }

    @Test public void reversalFlagsFlipEachServoIndependently() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.servo2Reversed = true;
        f.turret.holdAngle(90);
        f.update(pose(100, 110, 0));
        assertTrue(f.power1 > 0);
        assertEquals(-f.power1, f.power2, 0);

        f.params.servo1Reversed = true;
        f.params.servo2Reversed = false;
        f.update(pose(100, 110, 0));
        assertTrue(f.power1 < 0);
        assertEquals(-f.power1, f.power2, 0);
    }

    @Test public void encoderDirectionAndGearingScaleTheAngle() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.ticks = 2048;
        f.update(pose(100, 110, 0));
        assertEquals(90, f.turret.getAngleDegrees(), 1e-9);

        f.params.encoderRevsPerTurretRev = 4;
        f.update(pose(100, 110, 0));
        assertEquals(22.5, f.turret.getAngleDegrees(), 1e-9);

        f.params.encoderReversed = true;
        f.update(pose(100, 110, 0));
        assertEquals(-22.5, f.turret.getAngleDegrees(), 1e-9);
    }

    @Test public void zeroResetsTheHubCountToTheHomeAngle() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.homeAngleDegrees = -90;
        f.turnTo(37);
        f.update(pose(100, 110, 0));
        assertEquals(-53, f.turret.getAngleDegrees(), 0.05);
        assertEquals(0, f.resets);

        f.turret.zero();
        assertEquals(1, f.resets);
        assertEquals(0, f.ticks);
        assertEquals(-90, f.turret.getAngleDegrees(), 1e-9);
    }

    @Test public void aNewOpModeKeepsTheAngleBecauseConstructionNeverResetsTheEncoder() {
        Fixture auto = Fixture.nectar(Alliance.BLUE);
        auto.turret.holdAngle(-90);
        auto.drive(pose(100, 110, 0), 100);
        assertTrue(auto.turret.isAimed());

        Fixture teleOp = Fixture.nectar(Alliance.BLUE);
        teleOp.ticks = auto.ticks;
        teleOp.update(pose(100, 110, 0));
        assertEquals(0, teleOp.resets);
        assertEquals(auto.turret.getAngleDegrees(), teleOp.turret.getAngleDegrees(), 1e-9);
        assertEquals(0, teleOp.power1, 0);
    }

    @Test public void aBadPoseStopsTheServosInsteadOfSendingThemNaN() {
        Pose good = pose(100, 110, 0);
        Pose[] bad = {
                null,
                new Pose(Double.NaN, 110, 0),
                new Pose(100, Double.NaN, 0),
                new Pose(100, 110, Double.NaN),
                new Pose(Double.POSITIVE_INFINITY, 110, 0),
        };
        for (Pose pose : bad) {
            Fixture f = Fixture.nectar(Alliance.BLUE);
            f.turret.trackHive();
            f.update(good);
            assertTrue(f.power1 != 0);
            double distance = f.turret.getDistanceToTarget();

            f.update(pose);
            assertEquals(0, f.power1, 0);
            assertEquals(0, f.power2, 0);
            assertFalse(f.turret.isAimed());
            assertEquals(distance, f.turret.getDistanceToTarget(), 0);

            f.update(good);
            assertTrue(f.power1 != 0);
        }
    }

    @Test public void brokenTuningValuesNeverReachTheServos() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.turret.holdAngle(90);
        f.params.kP = Double.NaN;
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);

        f.params.kP = 0.015;
        f.params.maxPower = Double.NaN;
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);

        f.params.maxPower = 5;
        f.update(pose(100, 110, 0));
        assertEquals(1, f.power1, 0);

        f.params.encoderRevsPerTurretRev = 0;
        f.ticks = 100;
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);
        assertEquals(0, f.power2, 0);
    }

    @Test public void maxPowerLimitsTheCommand() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.maxPower = 0.2;
        f.turret.holdAngle(-120);
        f.update(pose(100, 110, 0));
        assertEquals(-0.2, f.power1, 0);
        assertEquals(-0.2, f.turret.getCommandedPower(), 0);
    }

    @Test public void stopCutsPowerAndUpdateDoesNotRestartIt() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.turret.trackHive();
        f.update(pose(100, 110, 0));
        assertTrue(f.power1 != 0);

        f.turret.stop();
        assertEquals(0, f.power1, 0);
        assertEquals(0, f.power2, 0);
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);
        assertFalse(f.turret.isAimed());
    }

    @Test public void feedforwardLeadsTheTurretAgainstRobotRotation() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.kV = 0.004;
        f.turret.trackHive();
        f.update(pose(100, 110, 0));
        double previousTarget = f.turret.getTargetAngleDegrees();
        // The robot turns counter-clockwise at 50 deg/s while the turret is held on target,
        // so feedback is silent and only the feedforward speaks.
        for (int i = 1; i <= 20; i++) {
            f.turnTo(previousTarget);
            // Asking again every loop must not restart the rate estimate.
            f.turret.trackHive();
            f.update(pose(100, 110, i));
            double target = f.turret.getTargetAngleDegrees();
            assertTrue(f.turret.isAimed());
            assertTrue(f.power1 < 0);
            assertEquals(0.004 * (target - previousTarget) / LOOP_SECONDS, f.power1, 1e-9);
            previousTarget = target;
        }
    }

    @Test public void derivativeDampsTheApproachButNeverKicksOnANewTarget() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.kP = 0;
        f.params.kS = 0;
        f.params.kD = 0.001;
        f.turret.holdAngle(30);
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);

        // Closing 10 degrees in one loop is -500 deg/s of error, so the damping pushes back.
        f.turnTo(10);
        f.turret.holdAngle(30);
        f.update(pose(100, 110, 0));
        assertEquals(-0.5, f.power1, 0.01);

        f.turret.holdAngle(-150);
        f.update(pose(100, 110, 0));
        assertEquals(0, f.power1, 0);
    }

    @Test public void changingCellsIsNotMistakenForTargetMotion() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.kV = 0.004;
        f.params.kP = 0;
        f.params.kD = 0;
        f.params.kS = 0;
        f.turret.trackHive();
        f.update(pose(100, 100, 0));
        f.update(pose(100, 69.4, 0));
        f.update(pose(100, 69.2, 0));
        assertEquals(87.2, f.turret.getTargetY(), 1e-9);
        assertTrue(f.power1 != 0);
        // Crossing the hysteresis line swings the aim by more than 100 degrees in one loop.
        f.update(pose(100, 68.9, 0));
        assertEquals(56.8, f.turret.getTargetY(), 1e-9);
        assertEquals(0, f.power1, 0);
    }

    @Test public void trimShiftsTheHiveAimCounterClockwise() {
        Fixture f = Fixture.nectar(Alliance.BLUE);
        f.params.aimTrimDegrees = 3;
        f.turret.trackHive();
        f.update(pose(89.25, 120, 0));
        assertEquals(-87, f.turret.getTargetAngleDegrees(), 1e-9);
    }

    @Test public void anglesWrapToTheHalfOpenTurn() {
        assertEquals(0, Turret.wrapDegrees(0), 0);
        assertEquals(180, Turret.wrapDegrees(180), 0);
        assertEquals(180, Turret.wrapDegrees(-180), 0);
        assertEquals(-170, Turret.wrapDegrees(190), 1e-9);
        assertEquals(170, Turret.wrapDegrees(-190), 1e-9);
        assertEquals(180, Turret.wrapDegrees(540), 0);
        assertEquals(10, Turret.wrapDegrees(-710), 1e-9);
    }
}
