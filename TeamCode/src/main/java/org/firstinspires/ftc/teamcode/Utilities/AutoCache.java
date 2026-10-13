package org.firstinspires.ftc.teamcode.Utilities;

import com.pedropathing.math.Pose;


public final class AutoCache {
    private static Pose endPose;

    private AutoCache() { }
    public static synchronized void clear() {
        endPose = null;
    }

    public static synchronized void save(Pose pose) {
        if (pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading())) {
            endPose = pose;
        }
    }

    public static synchronized Pose consume() {
        Pose pose = endPose;
        endPose = null;
        return pose;
    }
}
