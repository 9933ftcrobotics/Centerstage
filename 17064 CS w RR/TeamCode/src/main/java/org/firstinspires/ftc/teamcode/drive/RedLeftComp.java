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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "RedLeft")

public class RedLeftComp extends LinearOpMode {

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
        ArmUpDown.setTargetPosition(100);
        sleep(500);

        drive.setPoseEstimate(new Pose2d(-37, -61, Math.toRadians(90)));
        TrajectorySequence Start = drive.trajectorySequenceBuilder(new Pose2d(-37, -61, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, -47, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(Start);

        if (LeftDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Left

            telemetry.addLine("Left Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(-36, -47, Math.toRadians(90)));
            TrajectorySequence LeftSpike = drive.trajectorySequenceBuilder(new Pose2d(-36, -47, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-38, -40, Math.toRadians(135)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToSplineHeading(new Pose2d(-34, -45, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-34, -10, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(30, -10, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(44, -25, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(500)
                    )

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineToLinearHeading(new Pose2d(51, -30, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .lineToLinearHeading(new Pose2d(42, -30, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90))) //Park Middle
                    //.lineToLinearHeading(new Pose2d(47, -58, Math.toRadians(90))) //Park Corner

                    .build();
            drive.followTrajectorySequence(LeftSpike);

        } else if (RightDistance.getDistance(DistanceUnit.CM) < 100) {

            //Marker Right

            telemetry.addLine("Right Spike Mark!");
            telemetry.update();

            drive.setPoseEstimate(new Pose2d(-36, -47, Math.toRadians(90)));
            TrajectorySequence RightSpike = drive.trajectorySequenceBuilder(new Pose2d(-36, -47, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-32, -35, Math.toRadians(45)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToSplineHeading(new Pose2d(-40, -40, Math.toRadians(45)))
                    .lineToSplineHeading(new Pose2d(-40, -8, Math.toRadians(0)))
                    .lineToSplineHeading(new Pose2d(30, -8, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(44, -40, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(400)
                    )

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineToLinearHeading(new Pose2d(52, -40, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(330)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .lineToLinearHeading(new Pose2d(42, -42, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90)))// Park Middle
                    //.lineToLinearHeading(new Pose2d(47, -58, Math.toRadians(90))) //Park Corner

                    .build();
            drive.followTrajectorySequence(RightSpike);
        } else {

            //Marker Middle

            telemetry.addLine("Middle Spike Mark!");
            telemetry.update();


            drive.setPoseEstimate(new Pose2d(-36, -47, Math.toRadians(90)));
            TrajectorySequence MiddleSpike = drive.trajectorySequenceBuilder(new Pose2d(-36, -47, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-37, -33, Math.toRadians(90)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownGround)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            RightClaw.setPosition(ArmAndClawPosition.RightClawOpen)
                    )

                    .waitSeconds(0.5)
                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .lineToLinearHeading(new Pose2d(-37, -38, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-49, -38, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-49, -10, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(-34, -10, Math.toRadians(90)))
                    .lineToSplineHeading(new Pose2d(30, -10, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(44, -33, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(450)
                    )

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(400)
                    )

                    .lineToLinearHeading(new Pose2d(51, -33, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            LeftClaw.setPosition(ArmAndClawPosition.LeftClawOpen)
                    )

                    .lineToLinearHeading(new Pose2d(42, -36, Math.toRadians(0)))

                    .addTemporalMarker(() ->
                            ArmUpDown.setTargetPosition(ArmAndClawPosition.ArmUpDownRest)
                    )

                    .addTemporalMarker(() ->
                            ArmInOut.setTargetPosition(ArmAndClawPosition.ArmInOutRest)
                    )

                    .lineToLinearHeading(new Pose2d(47, -14, Math.toRadians(90)))//Park Middle
                    //.lineToLinearHeading(new Pose2d(47, -58, Math.toRadians(90))) //Park Corner

                    .build();
            drive.followTrajectorySequence(MiddleSpike);
        }


        /*sleep(2000);

        drive.followTrajectory(
                drive.trajectoryBuilder(traj.end(), true)
                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                        .build()
        );*/
    }


}