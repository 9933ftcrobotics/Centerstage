package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))
                        drive.trajectorySequenceBuilder(new Pose2d(-37, 61, Math.toRadians(-90)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(-36, -47, Math.toRadians(90)))
                                /*.lineTo(new Vector2d(-37, -38))
                                .strafeTo(new Vector2d(-49, -38))
                                .lineToSplineHeading(new Pose2d(-49, -12, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(-34, -12, Math.toRadians(90)))
                                .lineToSplineHeading(new Pose2d(30, -12, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(44, -36, Math.toRadians(0)))
                                .lineTo(new Vector2d(48, -36))
                                .lineTo(new Vector2d(42, -36))
                                .lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90)))*/
                                .waitSeconds(1)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}