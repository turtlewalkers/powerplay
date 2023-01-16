package com.example.meepmeep;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MeepMeepTesting {
    public static void main(String args[]) {

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(45, 45, 5.579565990580749, Math.toRadians(259.2284275862069), 10.6)
                .followTrajectorySequence(drive ->
                        // when x is 33 the robot is running auto left
                        // when x is -33
                        drive.trajectorySequenceBuilder(new Pose2d(33,-61,Math.toRadians(90)))
                                .strafeLeft(19)
                                .forward(45)
                                .strafeRight(10.5)
                                .forward(3.12)
                                .lineToLinearHeading(new Pose2d(56,-11,0))
                                .lineToLinearHeading(new Pose2d(24, -13,Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(56,-11,0))
                                .lineToLinearHeading(new Pose2d(24, -13,Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(56,-11,0))
                                .lineToLinearHeading(new Pose2d(24, -13,Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(56,-11,0))
                                .lineToLinearHeading(new Pose2d(24, -13,Math.toRadians(90)))

                                .lineToLinearHeading(new Pose2d(56,-11,0))
                                .lineToLinearHeading(new Pose2d(24, -13,Math.toRadians(90)))

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