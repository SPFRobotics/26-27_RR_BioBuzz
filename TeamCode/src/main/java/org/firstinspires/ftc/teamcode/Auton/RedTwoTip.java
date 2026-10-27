package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Utilities.AutoCache;

@Autonomous(name = "R- TwoTip", group = "RedAuto")
public class RedTwoTip extends BlueTwoTip {
    public RedTwoTip() {
        super(PoseFactory.degrees().rotateAround(72, 72, 180));
    }

    @Override
    protected AutoCache.Alliance alliance() {
        return AutoCache.Alliance.RED;
    }
}
