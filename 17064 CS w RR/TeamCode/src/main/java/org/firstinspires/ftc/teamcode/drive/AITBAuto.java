package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
//@Disabled
public class AITBAuto extends LinearOpMode {

    private DcMotor ArmUpDown;
    private DcMotor ArmInOut;

    private Servo LeftClaw;
    private Servo RightClaw;

    private DistanceSensor RightDistance;
    private DistanceSensor LeftDistance;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ArmUpDown = hardwareMap.get(DcMotor.class, "ArmUpDown");
        ArmInOut = hardwareMap.get(DcMotor.class, "ArmInOut");

        LeftClaw = hardwareMap.get(Servo.class, "LeftClaw");
        RightClaw = hardwareMap.get(Servo.class, "RightClaw");

        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");

        ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmUpDown.setTargetPosition(0);
        ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmUpDown.setPower(1);

        ArmInOut.setDirection(DcMotor.Direction.REVERSE);
        ArmInOut.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmInOut.setTargetPosition(0);
        ArmInOut.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmInOut.setPower(1);

        RightClaw.setDirection(Servo.Direction.REVERSE);

        drive.setPoseEstimate(new Pose2d(14, -66, Math.toRadians(90)));

        TrajectorySequence toDetect = drive.trajectorySequenceBuilder(new Pose2d(14, -66, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14,-50,Math.toRadians(90)))
                .build();

        /*TrajectorySequence toRightP = drive.trajectorySequenceBuilder(new Pose2d(14, -35, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(90)))
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0.21)
                )
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(90)))
                .build();*/

        TrajectorySequence toCenterP = drive.trajectorySequenceBuilder(new Pose2d(14, -50, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14,-37,Math.toRadians(90)))
                .addTemporalMarker(() ->
                        LeftClaw.setPosition(0)
                )
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(14,-55,Math.toRadians(90)))
                .build();

        /*TrajectorySequence toLeftP = drive.trajectorySequenceBuilder(new Pose2d(14, -35, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(90)))
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0.21)
                )
                .lineToLinearHeading(new Pose2d(14,-35,Math.toRadians(90)))
                .build();*/

        TrajectorySequence toBackdrop = drive.trajectorySequenceBuilder(new Pose2d(14, -55, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(32,-34,Math.toRadians(0)))
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(360)
                )
                .addTemporalMarker(() ->
                        ArmInOut.setTargetPosition(1500)
                )
                .waitSeconds(1)
                .build();

        double Offset = 0;
        TrajectorySequence toYellow = drive.trajectorySequenceBuilder(new Pose2d(32,-34,Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(37,(-34+Offset),Math.toRadians(0)))
                .addTemporalMarker(() ->
                        RightClaw.setPosition(0)
                )
                .waitSeconds(1.5)
                .addTemporalMarker(() ->
                                ArmInOut.setTargetPosition(0)

                )
                .waitSeconds(0.5)
                .addTemporalMarker(() ->
                        ArmUpDown.setTargetPosition(150)
                )
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(34,-35,Math.toRadians(0)))
                .build();

        TrajectorySequence toPark = drive.trajectorySequenceBuilder(new Pose2d(34, -35, Math.toRadians(0)))
                .back(5)
                .splineToConstantHeading(new Vector2d(55,-11),Math.toRadians(0))

                .build();

        RightClaw.setPosition(0.7);
        LeftClaw.setPosition(0.7);
        sleep(2000);
        telemetry.addLine("ALEX IS THE BEST!");
        ArmUpDown.setTargetPosition(110);

        waitForStart();

        if (isStopRequested()) return;






        drive.followTrajectorySequence(toDetect);
        drive.followTrajectorySequence(toCenterP);
        drive.followTrajectorySequence(toBackdrop);
        Offset = 50;
        drive.followTrajectorySequence(toYellow);
        drive.followTrajectorySequence(toPark);



        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }
}
