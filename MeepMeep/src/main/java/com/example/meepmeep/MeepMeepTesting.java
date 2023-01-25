package com.example.meepmeep;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MeepMeepTesting {
    public static void main(String args[]) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(42, 45, 5.67341411415613, Math.toRadians(259.2284275862069), 11.19)
                .followTrajectorySequence(drive ->
                        // when x is 33 the robot is running auto left
                        // when x is -33
                        drive.trajectorySequenceBuilder(new Pose2d(33,-61,Math.toRadians(90)))
                                .strafeRight(19)
                                .lineToLinearHeading(new Pose2d(45, -10, Math.PI))
                .forward(3.12)


                .strafeLeft(12)
                .back(4)
                .turn(Math.toRadians(-90))
                .strafeRight(10)
                .back(4)
                .turn(Math.toRadians(-90))
                .strafeRight(32.1)
                .back(22)
                .build()
                );
        meepMeep
                        .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}