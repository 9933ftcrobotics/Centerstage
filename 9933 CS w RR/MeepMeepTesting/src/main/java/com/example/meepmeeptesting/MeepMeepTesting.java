package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -66, Math.toRadians(90)))
                                .forward(20)
                                .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(180)))
                                .forward(55)
                                .back(55)
                                .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                                .back(5)
                                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}