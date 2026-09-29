package org.firstinspires.ftc.teamcode.Utilities.vision;

import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Upstream visual chase behavior through Pedro 3. See UPSTREAM_LICENSE. */
public final class PollenChaseController {
    public enum Status { MANUAL, CHASE, ARRIVED, NO_TARGET }

    public static final class Config {
        public double maxDrive = 0.35;
        public double maxTurn = 0.30;
        public double turnKp = 0.02;
        public double txDeadbandDegrees = 2.0;
        public double farArea = 0.02;
        public double stopArea = 0.14;
        public double misalignmentDegrees = 15.0;
        public double misalignmentScale = 0.3;
        public double manualScale = 0.5;
        public double stickDeadband = 0.05;
    }

    interface Drive {
        void manual(double forward, double strafe, double turn);
        void stop();
    }

    private final Drive drive;
    private final PollenVision vision;
    private final Config config;
    private Status status = Status.NO_TARGET;
    private double forward, strafe, turn;
    private boolean started;

    public PollenChaseController(Follower follower, PollenVision vision) {
        this(follower, vision, new Config());
    }

    public PollenChaseController(Follower follower, PollenVision vision, Config config) {
        this(new Drive() {
            @Override public void manual(double f, double s, double t) { follower.manual(f, s, t); }
            @Override public void stop() { follower.stop(); }
        }, vision, config);
    }

    PollenChaseController(Drive drive, PollenVision vision, Config config) {
        this.drive = drive;
        this.vision = vision;
        this.config = config;
    }

    public void start() {
        vision.start();
        started = true;
    }

    /**
     * Inputs are unscaled robot-centric Pedro powers: forward+, left+, counterclockwise+.
     * For a gamepad pass -left_stick_y, -left_stick_x, -right_stick_x.
     * This owns vision.update() and the drive command; caller must call follower.update()
     * exactly once afterward. Do not also follow a path or issue another manual command.
     */
    public Status update(boolean chaseEnabled, double manualForward, double manualStrafe,
                         double manualTurn) {
        if (!started) throw new IllegalStateException("Call start() before update()");
        PollenVision.Target target = vision.update();
        boolean stickActive = Math.abs(manualForward) > config.stickDeadband
                || Math.abs(manualStrafe) > config.stickDeadband
                || Math.abs(manualTurn) > config.stickDeadband;
        forward = strafe = turn = 0;
        if (!chaseEnabled || stickActive) {
            status = Status.MANUAL;
            forward = manualForward * config.manualScale;
            strafe = manualStrafe * config.manualScale;
            turn = manualTurn * config.manualScale;
        } else if (!target.available) {
            status = Status.NO_TARGET;
        } else if (target.area >= config.stopArea) {
            status = Status.ARRIVED;
        } else {
            status = Status.CHASE;
            double upstreamTurn = Math.abs(target.txDegrees) < config.txDeadbandDegrees
                    ? 0 : config.turnKp * target.txDegrees;
            // Upstream clockwise+ turns are Pedro counterclockwise- turns.
            turn = -Math.max(-config.maxTurn, Math.min(config.maxTurn, upstreamTurn));
            double t = (target.area - config.farArea) / (config.stopArea - config.farArea);
            forward = config.maxDrive * (1 - Math.max(0, Math.min(1, t)));
            if (Math.abs(target.txDegrees) > config.misalignmentDegrees) {
                forward *= config.misalignmentScale;
            }
        }
        drive.manual(forward, strafe, turn);
        return status;
    }

    public Status getStatus() { return status; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pollen mode", status);
        telemetry.addData("Pedro forward / strafe / turn", "%.2f / %.2f / %.2f", forward, strafe, turn);
        vision.addTelemetry(telemetry);
    }

    /** Caller should call follower.update() after this to flush Pedro's stop mode. */
    public void stop() {
        try { drive.stop(); }
        finally {
            started = false;
            forward = strafe = turn = 0;
            status = Status.NO_TARGET;
            vision.stop();
        }
    }
}
