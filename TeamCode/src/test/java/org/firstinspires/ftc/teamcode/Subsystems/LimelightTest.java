package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.Utilities.AutoCache.Alliance;
import org.junit.Test;
import java.util.Arrays;
import static org.firstinspires.ftc.teamcode.Subsystems.Limelight.Status.*;
import static org.junit.Assert.*;

public class LimelightTest {
    static class Camera implements Limelight.Camera {
        Limelight.Frame frame;
        int starts, stops, reads;
        boolean pipelineSuccess = true;
        @Override public boolean start() { starts++; return pipelineSuccess; }
        @Override public Limelight.Frame read() { reads++; return frame; }
        @Override public void stop() { stops++; }
        void show(Limelight.Detection... tags) {
            frame = new Limelight.Frame(true, 2, 0, Arrays.asList(tags));
        }
    }

    static class Fixture {
        final Camera camera = new Camera();
        final Limelight aim;
        Fixture() { this(Alliance.UNKNOWN); }
        Fixture(Alliance alliance) {
            aim = new Limelight(camera, new Limelight.Config(), alliance);
            aim.start();
        }
        Limelight.AimResult show(Limelight.Detection... tags) {
            camera.show(tags);
            return aim.update();
        }
    }

    static Limelight.Detection tag(int id, double area, double tx) {
        return new Limelight.Detection(id, area, tx);
    }

    @Test public void largestMatchingTagWinsAcrossBothCellsAndReacquiresEveryFrame() {
        Fixture red = new Fixture(Alliance.RED);
        assertEquals(35, red.show(tag(30, 1, 10), tag(35, 3, -5), tag(38, 20, 0)).tagId);
        assertEquals(30, red.show(tag(30, 4, 10), tag(35, 3, -5), tag(45, 20, 0)).tagId);
        assertEquals(NO_TARGET, red.show(tag(38, 20, 0), tag(21, 100, 0)).status);
        Fixture blue = new Fixture(Alliance.BLUE);
        assertEquals(45, blue.show(tag(30, 50, 0), tag(38, 1, -10), tag(45, 2, 5)).tagId);
        assertEquals(38, blue.show(tag(38, 4, -10), tag(45, 2, 5)).tagId);
        assertEquals(NO_TARGET, blue.show(tag(37, 100, 0)).status);
    }

    @Test public void noCachedAllianceAimsAtLargestVisibleTagIncludingOtherIds() {
        Fixture f = new Fixture();
        assertEquals(21, f.show(tag(30, 2, -10), tag(45, 3, 10), tag(21, 5, 4)).tagId);
        assertEquals(-4, f.aim.getResult().errorDegrees, 0);
        assertEquals(30, f.show(tag(30, 2, 0), tag(45, 1, 0)).tagId);
    }

    @Test public void aimsAtTagCenterWithCorrectPedroSignsClampAndTolerance() {
        Fixture f = new Fixture();
        assertEquals(-0.30, f.show(tag(30, 1, 30)).turnPower, 1e-9);
        assertEquals(0.30, f.show(tag(38, 1, -30)).turnPower, 1e-9);
        assertEquals(-0.1, f.show(tag(35, 1, 5)).turnPower, 1e-9);
        assertEquals(0.1, f.show(tag(44, 1, -5)).turnPower, 1e-9);
        assertEquals(ALIGNED, f.show(tag(30, 1, 0)).status);
        assertEquals(ALIGNED, f.show(tag(45, 1, 1.5)).status);
        assertEquals(0, f.aim.getResult().turnPower, 0);
        assertEquals(ALIGNED, f.show(tag(21, 1, -1.5)).status);
        assertEquals(TRACKING, f.show(tag(30, 1, 1.51)).status);
    }

    @Test public void releaseImmediatelyRestoresManualTurnAndPrecisionScalesEitherMode() {
        Fixture f = new Fixture();
        f.show(tag(30, 1, 20));
        assertEquals(-0.30, f.aim.driveTurn(true, 0.8, 1), 1e-9);
        assertEquals(-0.105, f.aim.driveTurn(true, 0.8, 0.35), 1e-9);
        assertEquals(0.8, f.aim.driveTurn(false, 0.8, 1), 1e-9);
        assertEquals(0.28, f.aim.driveTurn(false, 0.8, 0.35), 1e-9);
        f.show();
        assertEquals(0, f.aim.driveTurn(true, 1, 1), 0);
        assertEquals(-0.4, f.aim.driveTurn(false, -0.4, 1), 0);
    }

    @Test public void staleWrongPipelineOrMissingResultsImmediatelyClearTurn() {
        Fixture f = new Fixture();
        assertTrue(f.show(tag(30, 1, 10)).hasTarget());
        f.camera.frame = new Limelight.Frame(true, 2, 99, Arrays.asList(tag(30, 1, 10)));
        assertTrue(f.aim.update().hasTarget());
        for (double age : new double[]{100, 101, -1, Double.NaN}) {
            f.camera.frame = new Limelight.Frame(true, 2, age, Arrays.asList(tag(30, 1, 10)));
            assertEquals(STALE_FRAME, f.aim.update().status);
            assertEquals(0, f.aim.driveTurn(true, 1, 1), 0);
        }
        f.camera.frame = new Limelight.Frame(true, 0, 0, Arrays.asList(tag(30, 1, 10)));
        assertEquals(WRONG_PIPELINE, f.aim.update().status);
        f.camera.frame = new Limelight.Frame(false, 2, 0, Arrays.asList(tag(30, 1, 10)));
        assertEquals(INVALID_FRAME, f.aim.update().status);
        f.camera.frame = null;
        assertEquals(INVALID_FRAME, f.aim.update().status);
        assertEquals(0, f.aim.getResult().turnPower, 0);
        assertEquals(NO_TARGET, f.show().status);
        assertTrue(f.show(tag(30, 1, 10)).hasTarget());
    }

    @Test public void ignoresInvalidMeasurementsWithoutRequiringAnyThreeDimensionalPose() {
        Fixture f = new Fixture();
        assertEquals(42, f.show(null, tag(30, Double.NaN, 10),
                tag(31, -1, 10), tag(32, 0, 10), tag(33, 100, Double.NaN),
                tag(34, Double.POSITIVE_INFINITY, 10), tag(35, 100, Double.NEGATIVE_INFINITY),
                tag(42, 1, -3)).tagId);
        assertEquals(NO_TARGET, f.show(tag(30, 1, Double.NaN)).status);
    }

    @Test public void lifecycleAndMissingCameraPreserveManualTurn() {
        Fixture f = new Fixture();
        f.aim.start();
        assertEquals(1, f.camera.starts);
        f.show(tag(30, 1, 10));
        f.aim.stop();
        f.aim.stop();
        assertEquals(1, f.camera.stops);
        assertEquals(STOPPED, f.aim.update().status);
        assertEquals(0, f.aim.driveTurn(true, 1, 1), 0);
        assertEquals(0.6, f.aim.driveTurn(false, 0.6, 1), 0);
        Limelight missing = new Limelight((Limelight.Camera) null, new Limelight.Config(), Alliance.BLUE);
        missing.start();
        assertEquals(CAMERA_MISSING, missing.update().status);
        assertEquals(0.7, missing.driveTurn(false, 0.7, 1), 0);
        Camera camera = new Camera();
        camera.pipelineSuccess = false;
        Limelight failed = new Limelight(camera, new Limelight.Config(), Alliance.RED);
        failed.start();
        assertEquals(PIPELINE_ERROR, failed.update().status);
        assertEquals(0, camera.reads);
    }
}
