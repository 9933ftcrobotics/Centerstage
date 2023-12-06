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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 16.34)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(14, -66, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(16.5,-40,Math.toRadians(90)))
                                .addTemporalMarker(() -> {
                                    //LeftClaw.setPosition(0);
                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(35,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //ArmUpDown.setTargetPosition(1600);
                                })
                                .waitSeconds(0.5)
                                //.lineToLinearHeading(new Pose2d(37.5,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //RightClaw.setPosition(0);
                                })
                                .lineToLinearHeading(new Pose2d(42.5,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //RightClaw.setPosition(0);
                                })
                                //.waitSeconds(3)
                                .addTemporalMarker(() -> {
                                    //ArmUpDown.setTargetPosition(200);
                                })

                                .lineToLinearHeading(new Pose2d(30,-37,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //LeftClaw.setPosition(0.5);
                                })
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(35,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //ArmUpDown.setTargetPosition(1600);
                                })
                                .waitSeconds(0.5)
                                //.lineToLinearHeading(new Pose2d(37.5,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //RightClaw.setPosition(0);
                                })
                                .lineToLinearHeading(new Pose2d(42.5,-37,Math.toRadians(180)))
                                .addTemporalMarker(() -> {
                                    //RightClaw.setPosition(0);
                                })
                                //.waitSeconds(3)
                                .addTemporalMarker(() -> {
                                    //ArmUpDown.setTargetPosition(100);
                                })
                                .lineToLinearHeading(new Pose2d(40,-25,Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(52,-12,Math.toRadians(90)))

                                .waitSeconds(1.5)
                                .lineToLinearHeading(new Pose2d(14,-66,Math.toRadians(90)))



                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}