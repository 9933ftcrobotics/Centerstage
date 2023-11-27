package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class DemoAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(12, -66, Math.toRadians(90)));
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(12, -66, Math.toRadians(90)))
                .forward(20)
                .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(8, -36, Math.toRadians(180)))
                .forward(55)
                .back(55)
                .lineToSplineHeading(new Pose2d(45, -38, Math.toRadians(0)))
                .back(5)
                .splineToConstantHeading(new Vector2d(60, -10), Math.toRadians(0))
                .build()

        drive.followTrajectorySequence(traj);


    }
}
