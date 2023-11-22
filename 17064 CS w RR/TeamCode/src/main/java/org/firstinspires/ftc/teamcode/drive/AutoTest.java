package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "AutoTest")

public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(8, -60, Math.toRadians(90)));
        TrajectorySequence trajseq = drive.trajectorySequenceBuilder(new Pose2d(8, -60, Math.toRadians(90)))
                .forward(27)
                .waitSeconds(1)
                .back(25)
                .turn(Math.toRadians(90))
                .forward(80)
                .turn(Math.toRadians(-90))
                .forward(23.75)
                .turn(Math.toRadians(90))
                .forward(8)
                .waitSeconds(2)
                .back(8)
                .turn(Math.toRadians(-90))
                .back(24)
                .strafeLeft(18)
                .waitSeconds(1)
                .build();


        drive.followTrajectorySequence(trajseq);

        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
}
