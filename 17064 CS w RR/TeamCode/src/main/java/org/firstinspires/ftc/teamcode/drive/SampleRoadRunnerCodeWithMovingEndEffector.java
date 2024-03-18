package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*
 * Import files above
 */
@Autonomous(group = "SampleRoadRunnerCodeWithMovingEndEffector")
@Disabled
public class SampleRoadRunnerCodeWithMovingEndEffector extends LinearOpMode {


    //Set up motors, servos, sensors, and variables

    @Override
    public void runOpMode() throws InterruptedException {

        //Config motors, servos, sensors

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //Change x, y, and rotation for your path. 
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0))); //Start at this position
        TrajectorySequence Path = drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0))) //Name Sequence, set starting path.
                .lineToLinearHeading(new Pose2d(1, 1, Math.toRadians(0)))//Move here
                .addTemporalMarker(() ->
                        //Move end effector
                        telemetry.addLine("add Temporal Marker test") //Testing marker
                )
                .build();
        drive.followTrajectorySequence(Path); //Run Path

    }
}