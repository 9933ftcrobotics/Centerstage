package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(group = "ParkBotBlueRight")

public class ParkBotBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
        TrajectorySequence trajseq = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                .forward(50)
                .strafeLeft(95)
                .build();


        drive.followTrajectorySequence(trajseq);

    }
}
