package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ArmAndClawPosition;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "RedRightAutoWithRoad")

public class RedRightAutoWithRoad extends LinearOpMode {

    private DcMotor ArmUpDown;
    private DcMotor ArmInOut;

    private Servo LeftClaw;
    private Servo RightClaw;

    private DistanceSensor RightDistance;
    private DistanceSensor LeftDistance;

    @Override
    public void runOpMode() throws InterruptedException {

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        LeftClaw.setPosition(ArmAndClawPosition.LeftClawClosed);
        RightClaw.setPosition(ArmAndClawPosition.RightClawClosed);
        sleep(500);
        ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);
        sleep(500);

        if (RightDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Left

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence LeftOne = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(8,-37,Math.toRadians(135)))
                    .build();

            drive.followTrajectorySequence(LeftOne);

            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround);
            sleep(250);
            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen);
            sleep(250);
            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);

            drive.setPoseEstimate(new Pose2d(8, -37, Math.toRadians(135)));
            TrajectorySequence LeftTwo = drive.trajectorySequenceBuilder(new Pose2d(8, -37, Math.toRadians(135)))
                    .back(10)
                    .lineToSplineHeading(new Pose2d(40, -26, Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(LeftTwo);

            ArmUpDown.setTargetPosition(360);
            ArmInOut.setTargetPosition(400);

            drive.setPoseEstimate(new Pose2d(40, -22, Math.toRadians(0)));
            TrajectorySequence LeftThree = drive.trajectorySequenceBuilder(new Pose2d(40, -22, Math.toRadians(0)))
                    .forward(7)
                    .build();
            drive.followTrajectorySequence(LeftThree);

            LeftClaw.setPosition(0);

            drive.setPoseEstimate(new Pose2d(42, -22, Math.toRadians(0)));
            TrajectorySequence LeftFour = drive.trajectorySequenceBuilder(new Pose2d(42, -22, Math.toRadians(0)))
                    .back(5)
                    .turn(Math.toRadians(90))
                    .build();
            drive.followTrajectorySequence(LeftFour);

            ArmInOut.setTargetPosition(0);
            ArmUpDown.setTargetPosition(120);

            drive.setPoseEstimate(new Pose2d(37, -22, Math.toRadians(90)));
            TrajectorySequence LeftFive = drive.trajectorySequenceBuilder(new Pose2d(37, -22, Math.toRadians(90)))
                    .back(24)
                    .strafeRight(14)
                    .build();
            drive.followTrajectorySequence(LeftFive);

        } else if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Right

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence LeftOne = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(15, -37, Math.toRadians(45)))
                    .build();

            drive.followTrajectorySequence(LeftOne);

            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround);
            sleep(250);
            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen);
            sleep(250);
            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest);

            drive.setPoseEstimate(new Pose2d(15, -37, Math.toRadians(45)));
            TrajectorySequence LeftTwo = drive.trajectorySequenceBuilder(new Pose2d(15, -37, Math.toRadians(45)))
                    .back(6)
                    .lineToSplineHeading(new Pose2d(40, -45, Math.toRadians(0)))
                    .build();
            drive.followTrajectorySequence(LeftTwo);

            ArmUpDown.setTargetPosition(360);
            ArmInOut.setTargetPosition(400);

            drive.setPoseEstimate(new Pose2d(40, -45, Math.toRadians(0)));
            TrajectorySequence LeftThree = drive.trajectorySequenceBuilder(new Pose2d(40, -45, Math.toRadians(0)))
                    .forward(4)
                    .build();
            drive.followTrajectorySequence(LeftThree);

            LeftClaw.setPosition(0);

            drive.setPoseEstimate(new Pose2d(44, -45, Math.toRadians(0)));
            TrajectorySequence LeftFour = drive.trajectorySequenceBuilder(new Pose2d(42, -45, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(39, -45, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(LeftFour);

            ArmInOut.setTargetPosition(0);
            ArmUpDown.setTargetPosition(120);

            drive.setPoseEstimate(new Pose2d(39, -45, Math.toRadians(90)));
            TrajectorySequence LeftFive = drive.trajectorySequenceBuilder(new Pose2d(39, -45, Math.toRadians(90)))
                    .back(24)
                    .strafeRight(14)
                    .build();
            drive.followTrajectorySequence(LeftFive);
        } else {

            //Marker Middle

            telemetry.addLine("Middle Spike Mark!");
            telemetry.update();


            drive.setPoseEstimate(new Pose2d(14.5, -61, Math.toRadians(90)));
            TrajectorySequence PartOne = drive.trajectorySequenceBuilder(new Pose2d(14.5, -61, Math.toRadians(90)))
                    .forward(27)
                    .build();


            drive.followTrajectorySequence(PartOne);

            ArmUpDown.setTargetPosition(0);
            sleep(500);
            RightClaw.setPosition(0);
            sleep(500);
            ArmUpDown.setTargetPosition(150);
            sleep(250);

            drive.setPoseEstimate(new Pose2d(14.5, -36, Math.toRadians(90)));
            TrajectorySequence PartTwo = drive.trajectorySequenceBuilder(new Pose2d(14.5, -36, Math.toRadians(90)))
                    .back(5)
                    .lineToSplineHeading(new Pose2d(40, -31, Math.toRadians(0)))
                    .build();

            drive.followTrajectorySequence(PartTwo);

            ArmUpDown.setTargetPosition(340);
            ArmInOut.setTargetPosition(400);
            sleep(200);

            drive.setPoseEstimate(new Pose2d(40, -31, Math.toRadians(0)));
            TrajectorySequence PartThree = drive.trajectorySequenceBuilder(new Pose2d(40, -31, Math.toRadians(0)))
                    .forward(4)
                    .build();

            drive.followTrajectorySequence(PartThree);

            LeftClaw.setPosition(0);
            sleep(200);

            drive.setPoseEstimate(new Pose2d(44, -31, Math.toRadians(0)));
            TrajectorySequence PartFour = drive.trajectorySequenceBuilder(new Pose2d(44, -31, Math.toRadians(0)))
                    .back(5)
                    .turn(Math.toRadians(90))
                    .build();

            drive.followTrajectorySequence(PartFour);

            ArmInOut.setTargetPosition(0);
            sleep(300);
            ArmUpDown.setTargetPosition(150);
            sleep(200);

            drive.setPoseEstimate(new Pose2d(39, -31, Math.toRadians(90)));
            TrajectorySequence PartFive = drive.trajectorySequenceBuilder(new Pose2d(39, -31, Math.toRadians(90)))
                    .back(23)
                    .strafeRight(14)
                    .build();
            drive.followTrajectorySequence(PartFive);
        }


        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }


}