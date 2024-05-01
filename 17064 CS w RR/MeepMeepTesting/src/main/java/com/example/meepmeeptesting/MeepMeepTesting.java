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

                                drive.trajectorySequenceBuilder(new Pose2d(13.5, 47, Math.toRadians(-90)))
//                        drive.setPoseEstimate(new Pose2d(13.5, 47, Math.toRadians(-90)));
//        TrajectorySequence RightSpike = drive.trajectorySequenceBuilder(new Pose2d(13.5, 47, Math.toRadians(-90)))

                .lineToSplineHeading(new Pose2d(12, 39, Math.toRadians(-135)))
                //.forward(3)



                .back(12)
                //.lineToSplineHeading(new Pose2d(40, 20, Math.toRadians(0)))


                .lineToLinearHeading(new Pose2d(50.5, 22, Math.toRadians(0)))




                .lineToSplineHeading(new Pose2d(42, 58, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(50, 58, Math.toRadians(-90)))//Park Corner
                //.lineToSplineHeading(new Pose2d(48, 15, Math.toRadians(-90)))//Park Middle

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}