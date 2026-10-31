package org.firstinspires.ftc.teamcode.Auton.PPs;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class extraStuff {
    

    public static class Paths {

        private static final PoseFactory p = PoseFactory.degrees();
        public Path IntakeCorner;
        public Path TravelThroughHive;
        public Path RunToFlower;
        public Path ReturnFromFlower;
        public Path Park;

        public Paths(Follower follower) {
            // Max speeds are set through Constants.foresightConfig (Pedro's "Modifying Constants").
            // If the build fails with "cannot find symbol ... foresightConfig", your Constants class needs a public static ForesightConfig foresightConfig that your Follower is built from.
            IntakeCorner = line(
                    p.of(89.870, 133.588, 0.0),
                    p.of(133.290, 133.700, 0.0)
            ).constant(Math.toRadians(0));

            TravelThroughHive = curve(
                    p.of(133.290, 133.700, 0.0),
                    p.of(70.482, 137.487, 0.0),
                    p.of(86.828, 59.845, 0.0),
                    p.of(83.244, 15.220, 0.0)
            ).heading(
                    Interpolator.piecewise()
                            .until(0.11660214206231453, Interpolator.constant(Math.toRadians(0)))
                            .until(0.71, Interpolator.tangent.reverse())
                            .until(1, Interpolator.linear(Math.toRadians(90), Math.toRadians(180)))
            );

            RunToFlower = curve(
                    p.of(83.244, 15.220, 0.0),
                    p.of(105.005, 59.255, 0.0),
                    p.of(96.710, 15.220, 0.0)
            )
                    .heading(
                            Interpolator.piecewise()
                                    .until(
                                            0.5,
                                            Interpolator.linear(Math.toRadians(180), Math.toRadians(270))
                                    )
                                    .until(1, Interpolator.constant(Math.toRadians(-90)))
                    )
                    .with(Constants.foresightConfig.maxVelocityConstraint.at(40.0));

            ReturnFromFlower = curve(
                    p.of(96.710, 15.220, 0.0),
                    p.of(104.574, 59.143, 0.0),
                    p.of(83.244, 15.220, 0.0)
            )
                    .linear(Math.toRadians(-90), Math.toRadians(180))
                    .with(Constants.foresightConfig.maxVelocityConstraint.at(40.0));

            Park = curve(
                    p.of(83.244, 15.220, 0.0),
                    p.of(93.627, 38.141, 0.0),
                    p.of(127.244, 21.272, 0.0)
            ).constant(Math.toRadians(180));
        }
    }

}
