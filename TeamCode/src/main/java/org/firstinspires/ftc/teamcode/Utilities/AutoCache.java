package org.firstinspires.ftc.teamcode.Utilities;

import com.pedropathing.math.Pose;


public final class AutoCache {
    public enum Alliance { UNKNOWN, RED, BLUE }

    public static final class Snapshot {
        public final Pose pose;
        public final Alliance alliance;

        private Snapshot(Pose pose, Alliance alliance) {
            this.pose = pose;
            this.alliance = alliance;
        }
    }

    private static Snapshot cached = new Snapshot(null, Alliance.UNKNOWN);

    private AutoCache() { }
    public static synchronized void clear() {
        cached = new Snapshot(null, Alliance.UNKNOWN);
    }

    public static synchronized void save(Pose pose) {
        save(pose, Alliance.UNKNOWN);
    }

    public static synchronized void save(Pose pose, Alliance alliance) {
        if (pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading())) {
            cached = new Snapshot(pose, alliance != null ? alliance : Alliance.UNKNOWN);
        }
    }

    public static synchronized Pose consume() {
        return consumeSnapshot().pose;
    }

    public static synchronized Snapshot consumeSnapshot() {
        Snapshot snapshot = cached;
        clear();
        return snapshot;
    }
}
