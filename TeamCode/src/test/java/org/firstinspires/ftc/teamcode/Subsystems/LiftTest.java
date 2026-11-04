package org.firstinspires.ftc.teamcode.Subsystems;

import org.junit.Test;
import static org.junit.Assert.*;

public class LiftTest {
    private static class Fixture {
        double voltage = 0.806;
        double power;
        long nanos;
        final Lift lift;

        Fixture() { this(0.806); }

        Fixture(double startingVoltage) {
            voltage = startingVoltage;
            lift = new Lift(p -> power = p, () -> voltage, () -> nanos);
        }

        void raw(double value) {
            voltage = value;
            nanos += 20_000_000;
            lift.update();
        }

        void zeroed(double value) {
            raw((value + 0.806) % 3.3);
        }

        void forwardTurn() {
            for (double v : new double[]{0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 0.0}) zeroed(v);
        }

        // Model the reported robot: positive power makes encoder voltage decrease.
        void followPower() {
            double next = (voltage - power * 0.02 * 3.3) % 3.3;
            raw(next < 0 ? next + 3.3 : next);
        }
    }

    @Test public void bottomOffsetIsAppliedAndNegativeValuesWrap() {
        assertEquals(0, Lift.zeroedVoltage(0.806), 1e-9);
        assertEquals(0.1, Lift.zeroedVoltage(0.906), 1e-9);
        assertEquals(3.2, Lift.zeroedVoltage(0.706), 1e-9);
        assertEquals(Lift.zeroedVoltage(0), Lift.zeroedVoltage(3.3), 1e-9);
    }

    @Test public void tinyBottomNoiseBeforePressDoesNotAddAnEntireTurn() {
        Fixture f = new Fixture();
        f.raw(0.805);
        f.lift.up();
        assertEquals(-3, f.lift.getTargetWrapCount());
    }

    @Test public void initializingOnEitherSideOfBottomHasTheSameTravel() {
        for (double voltage : new double[]{0.805, 0.806, 0.807}) {
            Fixture f = new Fixture(voltage);
            f.lift.up();
            for (int i = 0; i < 450 && f.lift.isBusy(); i++) f.followPower();
            assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
            assertEquals(2.25 - 0.1 / 3.3, f.lift.getTraveledRotations(), 0.01);
        }
    }

    @Test public void oneUpCommandThenOneDownCommandCompletesWithoutPausingOrResetting() {
        Fixture f = new Fixture();
        f.lift.up();
        assertEquals(-3, f.lift.getTargetWrapCount());
        for (int i = 0; i < 450 && f.lift.isBusy(); i++) {
            assertTrue(f.power > 0);
            f.followPower();
        }
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
        assertEquals(-3, f.lift.getWrapCount());
        f.lift.down();
        assertEquals(-3, f.lift.getWrapCount());
        assertEquals(0, f.lift.getTargetWrapCount());
        assertTrue(f.lift.isBusy());
        for (int i = 0; i < 450 && f.lift.isBusy(); i++) {
            assertTrue(f.power < 0);
            f.followPower();
        }
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
        assertEquals(0, f.lift.getWrapCount());
        assertEquals(0, f.power, 0);
    }

    @Test public void repeatedUpCallsDoNotRestartTheActiveMove() {
        Fixture f = new Fixture();
        f.lift.up();
        for (int i = 0; i < 450 && f.lift.isBusy(); i++) {
            f.lift.up();
            assertEquals(-3, f.lift.getTargetWrapCount());
            f.followPower();
        }
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
    }

    @Test public void repeatedCyclesUseRaisedTopAndOriginalBottomWithoutDrifting() {
        Fixture f = new Fixture(0.906);
        for (int cycle = 0; cycle < 3; cycle++) {
            f.lift.up();
            for (int i = 0; i < 450 && f.lift.isBusy(); i++) f.followPower();
            assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
            assertEquals(0.081, f.voltage, 0.015);
            assertEquals(2.25, f.lift.getTraveledRotations(), 0.015);
            f.lift.down();
            for (int i = 0; i < 450 && f.lift.isBusy(); i++) f.followPower();
            assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
            assertEquals(0.906, f.voltage, 0.015);
            assertEquals(0, f.lift.getWrapCount());
        }
    }

    @Test public void movementWhileStoppedIsTrackedWithoutRestartingTheServo() {
        Fixture f = new Fixture();
        f.zeroed(3.2);
        assertEquals(-1, f.lift.getWrapCount());
        assertEquals(0, f.power, 0);
        f.lift.down();
        assertEquals(-1, f.lift.getWrapCount());
        assertEquals(2, f.lift.getTargetWrapCount());
    }

    @Test public void downCountsTwoPositiveWrapsThenContinuesToPointOne() {
        Fixture f = new Fixture();
        f.lift.down();
        f.forwardTurn();
        assertEquals(1, f.lift.getWrapCount());
        assertTrue(f.lift.isBusy());
        f.forwardTurn();
        assertEquals(2, f.lift.getWrapCount());
        assertEquals(2, f.lift.getTraveledRotations(), 1e-9);
        assertTrue(f.lift.isBusy());
        f.zeroed(0.05);
        assertTrue(f.lift.isBusy());
        f.zeroed(0.1);
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
        assertEquals(2 + 0.1 / 3.3, f.lift.getTraveledRotations(), 1e-9);
        assertEquals(0, f.power, 0);
    }

    @Test public void rawVoltageRolloverIsNotTheZeroedWrap() {
        Fixture f = new Fixture();
        f.lift.down();
        for (double v : new double[]{1.2, 2.0, 3.0, 3.31, -0.01, 0.5}) {
            f.raw(v);
            assertTrue(f.lift.isBusy());
            assertEquals(0, f.lift.getWrapCount());
        }
        f.raw(0.806);
        assertEquals(1, f.lift.getWrapCount());
        assertTrue(f.lift.isBusy());
    }

    @Test public void upContinuesNinetyDegreesPastThePreviousUpperStop() {
        Fixture f = new Fixture();
        f.raw(0.906);
        f.lift.up();
        for (int turn = 0; turn < 2; turn++) {
            for (double v : new double[]{0.05, 3.2, 2.7, 2.2, 1.7, 1.2, 0.7, 0.2}) {
                f.zeroed(v);
                assertTrue(f.lift.isBusy());
                assertTrue(f.power > 0);
            }
            f.zeroed(0.1);
            assertEquals(-(turn + 1), f.lift.getWrapCount());
            assertTrue(f.lift.isBusy());
        }
        f.zeroed(3.2);
        assertTrue(f.lift.isBusy());
        f.zeroed(2.7);
        assertTrue(f.lift.isBusy());
        f.zeroed(2.575);
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
        assertEquals(2.25, f.lift.getTraveledRotations(), 1e-9);
        assertEquals(0.081, f.voltage, 1e-9);
        assertEquals(0, f.power, 0);
    }

    @Test public void jitterAcrossZeroDoesNotAccumulateWraps() {
        Fixture f = new Fixture();
        f.lift.down();
        for (int i = 0; i < 100; i++) {
            f.zeroed(3.29);
            f.zeroed(0.01);
        }
        assertEquals(0, f.lift.getWrapCount());
        assertTrue(f.lift.isBusy());
    }

    @Test public void aNewCommandPreservesTheWrapCount() {
        Fixture f = new Fixture();
        f.lift.down();
        f.forwardTurn();
        assertEquals(1, f.lift.getWrapCount());
        f.lift.up();
        assertEquals(1, f.lift.getWrapCount());
        assertEquals(-2, f.lift.getTargetWrapCount());
        assertEquals(0, f.lift.getTraveledRotations(), 0);
        assertEquals(1, f.power, 0);
    }

    @Test public void stopsWhenSamplePassesFinalTarget() {
        Fixture f = new Fixture();
        f.lift.down();
        f.forwardTurn();
        f.forwardTurn();
        f.zeroed(0.15);
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
        f.zeroed(0.2);
        assertEquals(0, f.power, 0);
    }

    @Test public void stopCancelsMotionAndUpdateDoesNotRestartIt() {
        Fixture f = new Fixture();
        f.lift.down();
        f.zeroed(0.5);
        f.lift.stop();
        f.zeroed(1.0);
        assertEquals(Lift.Status.CANCELLED, f.lift.getStatus());
        assertFalse(f.lift.isBusy());
        assertEquals(0, f.power, 0);
    }

    @Test public void stalledEncoderTimesOut() {
        Fixture f = new Fixture();
        f.lift.down();
        f.nanos = 10_000_000_000L;
        f.lift.update();
        assertEquals(Lift.Status.TIMED_OUT, f.lift.getStatus());
        assertEquals(0, f.power, 0);
    }

    @Test public void invalidFeedbackStopsMotionAndPreventsStarting() {
        for (double voltage : new double[]{Double.NaN, Double.POSITIVE_INFINITY, -0.1, 3.4}) {
            Fixture f = new Fixture();
            f.lift.down();
            f.raw(voltage);
            assertTrue(f.lift.isBusy());
            f.nanos += 50_000_000;
            f.lift.update();
            assertEquals(Lift.Status.INVALID_ENCODER, f.lift.getStatus());
            assertEquals(voltage, f.lift.getLastEncoderVoltage(), 0);
            assertEquals(0, f.power, 0);
            f.lift.up();
            assertFalse(f.lift.isBusy());
            assertEquals(Lift.Status.INVALID_ENCODER, f.lift.getStatus());
            assertEquals(0, f.power, 0);
        }
    }

    @Test public void oneInvalidReadingDoesNotCancelOrCountAsMotion() {
        Fixture f = new Fixture();
        f.lift.up();
        f.zeroed(3.2);
        int wraps = f.lift.getWrapCount();
        f.raw(3.4);
        assertTrue(f.lift.isBusy());
        assertEquals(wraps, f.lift.getWrapCount());
        assertTrue(f.power > 0);
        f.zeroed(3.1);
        assertEquals(Lift.Status.MOVING, f.lift.getStatus());
        assertEquals(3.4, f.lift.getLastRejectedVoltage(), 0);
        for (int i = 0; i < 450 && f.lift.isBusy(); i++) f.followPower();
        assertEquals(Lift.Status.COMPLETE, f.lift.getStatus());
    }

    @Test public void repeatedShortBurstsOfBadFeedbackNeverExtendTheMoveTimeout() {
        Fixture f = new Fixture();
        f.lift.up();
        for (int i = 0; i < 300 && f.lift.isBusy(); i++) {
            f.raw(3.4);
            f.raw(0.806);
        }
        assertFalse(f.lift.isBusy());
        assertEquals(0, f.power, 0);
    }
}
