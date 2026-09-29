package org.firstinspires.ftc.teamcode.Utilities.vision;

import org.junit.Test;
import static org.junit.Assert.*;
import static org.firstinspires.ftc.teamcode.Utilities.vision.PollenVisionTest.ball;
import static org.firstinspires.ftc.teamcode.Utilities.vision.PollenChaseController.Status.*;

public class PollenChaseControllerTest {
    static class Drive implements PollenChaseController.Drive {
        double forward, strafe, turn;
        int stops;
        @Override public void manual(double f, double s, double t) { forward=f; strafe=s; turn=t; }
        @Override public void stop() { stops++; forward=strafe=turn=0; }
    }
    static class Fixture extends PollenVisionTest.Fixture {
        final Drive drive = new Drive();
        final PollenChaseController controller = new PollenChaseController(
                drive, vision, new PollenChaseController.Config());
        Fixture() { controller.start(); }
        void show(double tx, double area) { camera.show(ball(tx, 0, area)); }
        PollenChaseController.Status chase() { return controller.update(true, 0, 0, 0); }
    }

    @Test public void convertsTurnToPedroAndCapsMisalignedMotion() {
        Fixture f = new Fixture();
        f.show(20, 0.01);
        assertEquals(CHASE, f.chase());
        assertEquals(-0.30, f.drive.turn, 1e-9);
        assertEquals(0.105, f.drive.forward, 1e-9);
        assertEquals(0, f.drive.strafe, 0);
        f.show(-20, 0.01); f.chase();
        assertEquals(0.30, f.drive.turn, 1e-9);
    }

    @Test public void slowsByAreaAndStopsAtThreshold() {
        Fixture f = new Fixture();
        f.show(0, 0.02); f.chase();
        assertEquals(0.35, f.drive.forward, 1e-9);
        f.show(0, 0.08); f.chase();
        assertEquals(0.175, f.drive.forward, 1e-9);
        f.show(10, 0.14);
        assertEquals(ARRIVED, f.chase());
        assertEquals(0, f.drive.forward, 0);
        assertEquals(0, f.drive.turn, 0);
    }

    @Test public void respectsTurnDeadbandAndProportionalGain() {
        Fixture f = new Fixture();
        f.show(1.99, 0.01); f.chase();
        assertEquals(0, f.drive.turn, 0);
        f.show(2, 0.01); f.chase();
        assertEquals(-0.04, f.drive.turn, 1e-9);
        f.show(10, 0.01); f.chase();
        assertEquals(-0.2, f.drive.turn, 1e-9);
    }

    @Test public void releaseImmediatelyRestoresScaledPedroManualInputs() {
        Fixture f = new Fixture();
        f.show(10, 0.01); f.chase();
        assertEquals(MANUAL, f.controller.update(false, 0.8, -0.4, -0.6));
        assertEquals(0.4, f.drive.forward, 1e-9);
        assertEquals(-0.2, f.drive.strafe, 1e-9);
        assertEquals(-0.3, f.drive.turn, 1e-9);
        f.controller.update(false, 0, 0, 0);
        assertEquals(0, f.drive.forward, 0);
        assertEquals(0, f.drive.turn, 0);
    }

    @Test public void eachStickOverridesAboveButNotAtDeadband() {
        Fixture f = new Fixture();
        f.show(0, 0.01);
        assertEquals(CHASE, f.controller.update(true, 0.05, -0.05, 0.05));
        assertEquals(MANUAL, f.controller.update(true, 0.051, 0, 0));
        assertEquals(MANUAL, f.controller.update(true, 0, -0.051, 0));
        assertEquals(MANUAL, f.controller.update(true, 0, 0, 0.051));
    }

    @Test public void missingTargetStopsAfterGracePeriod() {
        Fixture f = new Fixture();
        assertEquals(NO_TARGET, f.chase());
        assertEquals(0, f.drive.forward, 0);
        f.show(10, 0.01); f.chase();
        f.camera.frame = null;
        f.time = 0.499;
        assertEquals(CHASE, f.chase());
        f.time = 0.5;
        assertEquals(NO_TARGET, f.chase());
        assertEquals(0, f.drive.forward, 0);
        assertEquals(0, f.drive.turn, 0);
    }

    @Test public void stopStopsCameraAndFollower() {
        Fixture f = new Fixture();
        f.show(0, 0.01); f.chase();
        f.controller.stop();
        assertEquals(1, f.drive.stops);
        assertEquals(1, f.camera.stops);
        assertFalse(f.vision.getTarget().available);
        assertEquals(NO_TARGET, f.controller.getStatus());
    }
}
