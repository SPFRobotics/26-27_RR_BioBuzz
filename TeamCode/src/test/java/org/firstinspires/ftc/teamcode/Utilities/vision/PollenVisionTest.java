package org.firstinspires.ftc.teamcode.Utilities.vision;

import org.junit.Test;
import java.util.Arrays;
import static org.junit.Assert.*;

public class PollenVisionTest {
    static class Camera implements PollenVision.Camera {
        PollenVision.Frame frame;
        int starts, stops;
        @Override public void start() { starts++; }
        @Override public PollenVision.Frame read() { return frame; }
        @Override public void stop() { stops++; }
        void show(PollenVision.Detection... detections) {
            frame = new PollenVision.Frame(true, 0, Arrays.asList(detections));
        }
    }
    static class Fixture {
        final Camera camera = new Camera();
        double time;
        final PollenVision vision = new PollenVision(camera, new PollenVision.Config(), () -> time);
        Fixture() { vision.start(); }
    }
    static PollenVision.Detection ball(double tx, double ty, double area) {
        return new PollenVision.Detection("pollen", 0.8, tx, ty, area);
    }

    @Test public void filtersClassConfidenceAndMalformedDetections() {
        Fixture f = new Fixture();
        f.camera.show(new PollenVision.Detection("POLLEN", 0.5, 2, 0, 0.01),
                new PollenVision.Detection("nectar", 1, 0, 0, 0.2),
                new PollenVision.Detection("pollen", 0.499, 0, 0, 0.2),
                new PollenVision.Detection("pollen", Double.NaN, 0, 0, 0.2),
                ball(Double.NaN, 0, 0.2), ball(0, 0, -1));
        PollenVision.Target t = f.vision.update();
        assertTrue(t.available);
        assertEquals(1, t.pollenCount);
        assertEquals(2, t.txDegrees, 1e-9);
    }

    @Test public void rejectsNullInvalidAndStaleFramesAtBoundary() {
        Fixture f = new Fixture();
        assertFalse(f.vision.update().available);
        f.camera.frame = new PollenVision.Frame(false, 0, Arrays.asList(ball(0, 0, 0.1)));
        assertFalse(f.vision.update().available);
        f.camera.frame = new PollenVision.Frame(true, 200, Arrays.asList(ball(0, 0, 0.1)));
        assertFalse(f.vision.update().available);
        f.camera.frame = new PollenVision.Frame(true, 199.9, Arrays.asList(ball(0, 0, 0.1)));
        assertTrue(f.vision.update().available);
    }

    @Test public void clustersTransitivelyAndWeightsAimByArea() {
        Fixture f = new Fixture();
        // A-C are too far apart; B links the whole cluster.
        f.camera.show(ball(-12, 0, 0.01), ball(0, 0, 0.01), ball(12, 0, 0.02));
        PollenVision.Target t = f.vision.update();
        assertEquals(1, t.groupCount);
        assertEquals(3, t.targetCount);
        assertEquals(3, t.txDegrees, 1e-9);
        assertEquals(0.02, t.area, 1e-9);
        assertEquals(3.16, t.score, 1e-9);
    }

    @Test public void groupsUseBothImageAxes() {
        Fixture f = new Fixture();
        f.camera.show(ball(0, -20, 0.001), ball(0, 20, 0.001));
        assertEquals(2, f.vision.update().groupCount);
    }

    @Test public void pairBeatsFarSingleAndCloserEqualSizeGroupWins() {
        Fixture f = new Fixture();
        f.camera.show(ball(-25, 0, 0.01), ball(-23, 0, 0.01), ball(25, 0, 0.03));
        assertEquals(2, f.vision.update().targetCount);
        f.vision.stop(); f.vision.start();
        f.camera.show(ball(-25, 0, 0.01), ball(25, 0, 0.03));
        assertEquals(25, f.vision.update().txDegrees, 1e-9);
    }

    @Test public void hysteresisKeepsCurrentUntilChallengerScoresEnough() {
        Fixture f = new Fixture();
        f.camera.show(ball(-25, 0, 0.01));
        f.vision.update();
        f.camera.show(ball(-25, 0, 0.01), ball(25, 0, 0.02));
        assertEquals(-25, f.vision.update().txDegrees, 1e-9);
        f.camera.show(ball(-25, 0, 0.01), ball(25, 0, 0.05));
        assertEquals(25, f.vision.update().txDegrees, 1e-9);
    }

    @Test public void targetExpiresAtHalfSecondAndReacquires() {
        Fixture f = new Fixture();
        f.camera.show(ball(10, 0, 0.02)); f.vision.update();
        f.camera.frame = null;
        f.time = 0.499;
        PollenVision.Target grace = f.vision.update();
        assertTrue(grace.available);
        assertEquals(0, grace.pollenCount);
        assertEquals(10, grace.txDegrees, 1e-9);
        f.time = 0.5;
        assertFalse(f.vision.update().available);
        f.camera.show(ball(-10, 0, 0.02));
        assertEquals(-10, f.vision.update().txDegrees, 1e-9);
    }

    @Test public void stopAndRestartClearTracking() {
        Fixture f = new Fixture();
        f.camera.show(ball(0, 0, 0.1)); f.vision.update();
        f.vision.stop();
        assertFalse(f.vision.getTarget().available);
        assertEquals(1, f.camera.stops);
        f.camera.frame = null;
        f.vision.start();
        assertFalse(f.vision.update().available);
        f.vision.start();
        assertEquals(2, f.camera.starts);
    }
}
